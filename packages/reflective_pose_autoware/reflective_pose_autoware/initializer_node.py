"""ROS node: hand a detected board pose to Autoware's pose initializer.

The decide half of the old ``golfcart_board_initializer`` node. It knows
nothing about point clouds, TF or geometry: ``reflective_pose_ros`` detects the
board and publishes the composed vehicle pose on ``~/board_pose``, latched, and
this node answers the three questions that are about the *vehicle stack* rather
than about the board:

- is the vehicle stopped enough to trust a cold-start pose
  (``autoware_vehicle_msgs/VelocityReport`` against ``max_speed_for_init``),
- how many times do we try (``max_attempts``),
- and what happens when we run out (``fallback_to_user_defined_pose``).

Then it calls ``autoware_localization_msgs/InitializeLocalization``.

Everything Autoware-specific in the repository is in this file and in this
package's ``package.xml``. See docs/design/reflective_pose_detector.md.

What the split changed about retry
----------------------------------
In the old single process, one *attempt* was one accumulated batch of scans put
through the detector: a batch that produced no candidate counted against
``max_attempts``, and the budget therefore bounded how long a failing detector
was allowed to keep failing. Detection now happens in another process, which
publishes only when it succeeds, so a failed detection is not visible here and
cannot be counted. An attempt here is one board pose acted on -- one
initialization attempt -- and the budget bounds how many of those may fail
before the fallback policy runs.

The consequence, stated plainly: if the detector never detects anything, this
node never counts an attempt and the fallback never fires. That case surfaces
on ``/diagnostics``, where the detector node reports its own failure, not here.
Bounding it would need a "waited this long for a pose" timeout, which is a new
configuration key and so not this node's to invent.

The other half of the old gate moved with it. The old node cleared its scan
accumulation whenever the vehicle moved, because stacking scans assumes they
can be stacked without deskewing. The accumulation now lives on the other side
of the topic, so the same protection is applied here by *stamp*: a board pose
stamped before the last time the vehicle was seen moving was accumulated across
that motion, and is dropped rather than initialized from.
"""

from dataclasses import dataclass
from dataclasses import fields as dataclass_fields
from enum import Enum
from typing import Optional

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from autoware_localization_msgs.srv import InitializeLocalization

#: Where ``reflective_pose_ros``'s ``~/board_pose`` lands when its node is named
#: ``board_detector`` in the ``/localization`` namespace, which is what this
#: package's launch file does. The topic is configurable by remapping
#: (``--ros-args -r`` or ``<remap>``), not by a parameter: a topic name is
#: what remapping is for.
DEFAULT_BOARD_POSE_TOPIC = "/localization/board_detector/board_pose"

VELOCITY_TOPIC = "/vehicle/status/velocity_status"


@dataclass
class Policy:
    """The handoff policy. Declared as ROS parameters, one per field.

    Shipped as ``config/board_pose_initializer.param.yaml``. This node reads no
    detector file: everything it decides is a property of the vehicle stack,
    not of the board.
    """

    initialize_service: str = "/localization/initialize"
    max_speed_for_init: float = 0.05
    max_attempts: int = 5
    # Off by default: a silent fallback to a fixed pose turns a detector
    # failure into a mislocalization report three weeks later.
    fallback_to_user_defined_pose: bool = False
    # How long to wait for a board pose before the attempt budget counts one
    # as spent. Zero disables it. Detection and the service call are two
    # processes, so an attempt is one pose acted on: a detector that never
    # detects spends no attempts and the fallback never runs. This bounds it.
    pose_wait_timeout: float = 0.0


def declare_policy(node: Node) -> Policy:
    """Declare every ``Policy`` field as a parameter and read it back."""
    values = {}
    for item in dataclass_fields(Policy):
        node.declare_parameter(item.name, item.default)
        values[item.name] = node.get_parameter(item.name).value
    return Policy(**values)


class State(Enum):
    WAIT_POSE = "wait_pose"
    CALL_SERVICE = "call_service"
    DONE = "done"
    FAILED = "failed"


class BoardPoseInitializer(Node):
    """Gate on speed, spend the attempt budget, seed the pose initializer."""

    def __init__(self, **node_kwargs):
        super().__init__("board_pose_initializer", **node_kwargs)

        self._params = declare_policy(self)

        self._state = State.WAIT_POSE
        self._attempts = 0

        # Detection and the service call are two processes now, so an attempt is
        # one pose acted on: a detector that never detects spends no attempts
        # and the fallback policy never runs. The all-in-one node counted those
        # failures because it owned the detector loop. This restores the bound.
        # Zero disables it, which is the safe default only because the detector
        # reports its own silence on /diagnostics.
        self._pose_wait_timeout = float(self._params.pose_wait_timeout)
        self._wait_timer = None
        if self._pose_wait_timeout > 0.0:
            self._wait_timer = self.create_timer(
                self._pose_wait_timeout, self._on_pose_wait_timeout
            )
        self._speed = 0.0
        self._last_moving_time: Optional[Time] = None
        self._last_reason = ""

        # Matches the detector's latched publisher (transient_local, depth 1):
        # this node is a cold-start path and may well start after a detection
        # has already been published.
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            DEFAULT_BOARD_POSE_TOPIC,
            self._on_board_pose,
            latched,
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._velocity_sub = self.create_subscription(
            VelocityReport,
            VELOCITY_TOPIC,
            self._on_velocity,
            sensor_qos,
        )

        self._client = self.create_client(
            InitializeLocalization, self._params.initialize_service
        )

        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f"board_pose_initializer waiting for {DEFAULT_BOARD_POSE_TOPIC}"
        )

    # -- callbacks ----------------------------------------------------------

    def _on_velocity(self, msg: VelocityReport):
        self._speed = abs(float(msg.longitudinal_velocity))
        if self._speed > self._params.max_speed_for_init:
            self._last_moving_time = self.get_clock().now()

    def _on_board_pose(self, msg: PoseWithCovarianceStamped):
        if self._state in (State.DONE, State.FAILED, State.CALL_SERVICE):
            return

        # No VelocityReport at all leaves _speed at 0.0 and the gate open, the
        # same as the old node on a bench with no vehicle interface running.
        if self._speed > self._params.max_speed_for_init:
            self._note(
                f"vehicle moving at {self._speed:.2f} m/s, "
                f"above {self._params.max_speed_for_init:.2f}: board pose ignored"
            )
            return

        if self._accumulated_while_moving(msg):
            self._note("board pose was accumulated while the vehicle moved: ignored")
            return

        self._attempts += 1
        self.get_logger().info(
            "attempt %d/%d: initializing from board pose at x=%.3f y=%.3f z=%.3f"
            % (
                self._attempts,
                self._params.max_attempts,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
        )
        self._call_initialize(msg)

    def _accumulated_while_moving(self, msg: PoseWithCovarianceStamped) -> bool:
        """True when this pose predates the last motion we saw.

        A zero stamp means the publisher did not stamp it; there is then
        nothing to compare and the speed gate above is the whole test.
        """
        if self._last_moving_time is None:
            return False
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            return False
        return stamp < self._last_moving_time

    # -- the service call ---------------------------------------------------

    def _call_initialize(self, source: PoseWithCovarianceStamped):
        self._state = State.CALL_SERVICE
        message = self._pose_message(source)

        if not self._client.wait_for_service(timeout_sec=5.0):
            self._attempt_failed("pose initializer service unavailable")
            return

        request = InitializeLocalization.Request()
        request.pose_with_covariance = [message]
        # AUTO, not DIRECT: the board supplies a guess and NDT align refines it.
        # DIRECT would convert every detection error into a localization error.
        request.method = InitializeLocalization.Request.AUTO

        future = self._client.call_async(request)
        future.add_done_callback(self._on_service_response)

    def _pose_message(
        self, source: PoseWithCovarianceStamped
    ) -> PoseWithCovarianceStamped:
        """The request payload: the detector's pose, stamped at call time.

        The pose and covariance are the detector's; it composed them from the
        detection and the board's pose in the map, and re-deriving either here
        would mean this package knowing geometry it has no business knowing.
        Only the header is this node's: ``map``, and now, exactly as the old
        node built it immediately before the call.
        """
        frame = source.header.frame_id
        if frame and frame != "map":
            self.get_logger().warn(
                f"board pose arrived in frame '{frame}', not 'map'; "
                "sending it as a map pose unchanged"
            )
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = source.pose
        return message

    def _on_service_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._attempt_failed(f"initialize service raised: {error}")
            return

        if response.status.success:
            self._state = State.DONE
            self._last_reason = "initialized from board"
            self.get_logger().info("localization initialized from the board")
        else:
            self._attempt_failed(
                f"initialize service rejected: {response.status.message}"
            )

    # -- the attempt budget -------------------------------------------------

    def _attempt_failed(self, reason: str):
        """One initialization attempt failed. Retry, or spend the budget.

        The old node treated a service error as terminal because its budget was
        spent on failed *detections*, which it could see. Here the only failure
        that reaches this node is a service failure, so making the first one
        terminal would leave ``max_attempts`` and the fallback policy dead: the
        detector would keep republishing into a node that had stopped listening.
        A failed attempt therefore returns to waiting for the detector's next
        publication until the budget is gone.
        """
        self.get_logger().warn(f"attempt {self._attempts}: {reason}")
        self._last_reason = reason
        if self._attempts >= self._params.max_attempts:
            self._maybe_fallback(reason)
            return
        self._state = State.WAIT_POSE

    def _maybe_fallback(self, reason: str):
        if not self._params.fallback_to_user_defined_pose:
            self._fail(reason)
            return
        # Off by default. A silent fallback to a fixed pose is how a detector
        # failure becomes a mislocalization report three weeks later.
        self.get_logger().warn("falling back to the user-defined initial pose")
        request = InitializeLocalization.Request()
        request.pose_with_covariance = []
        request.method = InitializeLocalization.Request.AUTO
        self._client.call_async(request)
        self._state = State.DONE
        self._last_reason = "fell back to the user-defined initial pose"

    def _fail(self, reason: str):
        self._state = State.FAILED
        self._last_reason = reason
        self.get_logger().error(reason)

    def _note(self, reason: str):
        self._last_reason = reason
        self.get_logger().info(reason)

    # -- diagnostics --------------------------------------------------------

    def _on_pose_wait_timeout(self):
        """No board pose arrived within the timeout: spend an attempt.

        Only while actually waiting. A run that is mid-service-call, finished or
        already failed is not waiting for anything, and charging it an attempt
        would end a healthy initialization on a slow service.
        """
        if self._state is not State.WAIT_POSE:
            return
        self._fail(
            f"no board pose within {self._pose_wait_timeout:.1f} s "
            "(is the detector running and seeing the board?)"
        )

    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "localization: board_pose_initializer"
        status.hardware_id = "board_pose_initializer"
        if self._state is State.DONE:
            status.level = DiagnosticStatus.OK
        elif self._state is State.FAILED:
            status.level = DiagnosticStatus.ERROR
        else:
            status.level = DiagnosticStatus.WARN
        status.message = self._last_reason or self._state.value
        status.values = [
            KeyValue(key="state", value=self._state.value),
            KeyValue(key="attempts", value=str(self._attempts)),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = BoardPoseInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
