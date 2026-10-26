"""ROS node: hand a detected board pose to Autoware's pose initializer.

The decide half of the old ``golfcart_board_initializer`` node. It knows
nothing about point clouds, TF or geometry: ``reflective_pose_ros`` detects the
board and publishes the composed vehicle pose on ``~/board_pose``, latched, and
this node answers the three questions that are about the *vehicle stack* rather
than about the board:

- is the vehicle stopped enough to trust a cold-start pose
  (``autoware_vehicle_msgs/VelocityReport`` against ``max_speed_for_init``),
- is the rest of the stack ready to take a pose (``require_velocity``,
  ``ready_pointcloud_topic``),
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

Where the board was seen from is not decided here
-------------------------------------------------
A pose reaches this node only if its detection lay inside the detector file's
``detector.seed_zone``: range, bearing and aspect, each detection judged on
its own (golf-cart Phase 8, L6). That gate is in ``board_detector_node``,
not here, because this node receives only the composed vehicle pose: judging
the view would need the board's map pose and the sensor's TF, which are the
detector's, and copying them here is how the two would drift apart. Nothing
here re-initializes on its own either: after one success every later pose is
ignored, and a re-initialization stays the operator's request.

Failures before the scan matcher has a map are free
---------------------------------------------------
A cart parked at the board while the stack boots is detected within a second
of the first scan, long before the map reaches the scan matcher (8.6 s after
playback in the 2026-10-02 replay). Every align until then fails with "No map
loaded", and five detections a second apart used to spend the whole budget
before the map arrived. Those failures say nothing about the board, so an
attempt counts only if the matcher had published its loaded map when the call
went out. The call itself still goes out: cuda_ndt's align is what asks the
map loader for the tiles around the pose. ``ignore_failures_before_map``
turns this off.

Readiness
---------
The detector finds the board within a second or two of launch, well before the
rest of a cold-started stack can take a pose: Autoware's pose initializer refuses
while it has no wheel speed to confirm the vehicle is stopped ("The vehicle is not
stopped") and while NDT has no scan to align ("align server failed"). Calling
then spends an attempt on a stack that was never going to accept, and a budget
of five ran out in seven seconds on a stack that took a minute to come up. So a
pose is acted on only when the data the call depends on is flowing: with
``require_velocity``, ``VelocityReport`` s arriving without a gap longer than
``ready_timeout`` and all stopped for the last ``stopped_for`` seconds (Autoware's stop
check wants a buffer of twists that long, ``stop_check_duration``, 3 s); with
``ready_pointcloud_topic`` (NDT's input cloud), a cloud within ``ready_timeout``.
A pose that arrives before then is ignored, not counted, and the detector's next
one is tried. Both are off by default, which is the old behaviour.
"""

from dataclasses import dataclass
from dataclasses import fields as dataclass_fields
from enum import Enum
from typing import Optional

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from autoware_localization_msgs.srv import InitializeLocalization

#: Where ``reflective_pose_ros``'s ``~/board_pose`` lands when its node is named
#: ``board_detector`` in the ``/localization`` namespace, which is what this
#: package's launch file does. The topic is configurable by remapping
#: (``--ros-args -r`` or ``<remap>``), not by a parameter: a topic name is
#: what remapping is for.
DEFAULT_BOARD_POSE_TOPIC = "/localization/board_detector/board_pose"

VELOCITY_TOPIC = "/vehicle/status/velocity_status"

#: The scan matcher publishes the map it holds here, once it holds one: both
#: Autoware's ndt_scan_matcher and cuda_ndt. Remap it for another namespace.
MATCHER_MAP_TOPIC = "/localization/pose_estimator/debug/loaded_pointcloud_map"


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
    # Failed attempts made before the scan matcher has published a map do not
    # count against max_attempts: the align could not have succeeded.
    ignore_failures_before_map: bool = True

    # Readiness (see the module docstring). Off by default: the old behaviour,
    # and a bench with no vehicle interface keeps working.
    require_velocity: bool = False
    # A PointCloud2 topic that must be publishing before a pose is sent, e.g.
    # NDT's input /localization/util/downsample/pointcloud. Empty: no check.
    ready_pointcloud_topic: str = ""
    # How recent the last VelocityReport / cloud must be, seconds.
    ready_timeout: float = 1.0
    # How long the wheel speed must have reported a stop, unbroken, seconds: Autoware's
    # pose initializer stop_check_duration, which it checks against a buffer of twists.
    stopped_for: float = 3.0


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
        self._matcher_has_map = False
        # Whether the attempt in flight counts against the budget, fixed when
        # the call goes out: the map can arrive between the call and its
        # failed response, and that align still ran without it.
        self._attempt_counts = True

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
        self._last_velocity_time: Optional[Time] = None
        # since when VelocityReports have arrived without a gap and stopped
        self._stopped_since: Optional[Time] = None
        self._last_points_time: Optional[Time] = None
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

        # Reliable and volatile, which matches both publishers: upstream's is
        # transient_local, cuda_ndt's is volatile, and a transient_local
        # subscription would not connect to the latter. Raw, because only the
        # arrival matters and the map can be millions of points; dropped after
        # the first message.
        self._matcher_map_sub = None
        if self._params.ignore_failures_before_map:
            self._matcher_map_sub = self.create_subscription(
                PointCloud2, MATCHER_MAP_TOPIC, self._on_matcher_map, 1, raw=True
            )

        if self._params.ready_pointcloud_topic:
            self._points_sub = self.create_subscription(
                PointCloud2,
                self._params.ready_pointcloud_topic,
                self._on_points,
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
        now = self.get_clock().now()
        gap = (self._last_velocity_time is None or
               (now - self._last_velocity_time).nanoseconds * 1e-9 > self._params.ready_timeout)
        self._last_velocity_time = now
        self._speed = abs(float(msg.longitudinal_velocity))
        if self._speed > self._params.max_speed_for_init or gap:
            self._stopped_since = None if self._speed > self._params.max_speed_for_init else now
        elif self._stopped_since is None:
            self._stopped_since = now
        if self._speed > self._params.max_speed_for_init:
            self._last_moving_time = self.get_clock().now()

    def _on_matcher_map(self, _serialized: bytes):
        if self._matcher_has_map:
            return
        self._matcher_has_map = True
        self.get_logger().info("the scan matcher has a map: failed attempts count from now")
        if self._matcher_map_sub is not None:
            self.destroy_subscription(self._matcher_map_sub)
            self._matcher_map_sub = None

    def _on_points(self, _msg):
        self._last_points_time = self.get_clock().now()

    def _not_ready(self) -> str:
        """Why the stack cannot take a pose yet; '' when it can."""
        now = self.get_clock().now()
        timeout = self._params.ready_timeout

        def stale(last: Optional[Time]) -> bool:
            return last is None or (now - last).nanoseconds * 1e-9 > timeout

        if self._params.require_velocity:
            if stale(self._last_velocity_time):
                return f"no wheel speed within {timeout:g} s: not ready to confirm a stop"
            held = (0.0 if self._stopped_since is None
                    else (now - self._stopped_since).nanoseconds * 1e-9)
            if held < self._params.stopped_for:
                return (f"wheel speed stopped for {held:.1f} s of the "
                        f"{self._params.stopped_for:g} s the stop check needs")
        if self._params.ready_pointcloud_topic and stale(self._last_points_time):
            return (f"no cloud on {self._params.ready_pointcloud_topic} within "
                    f"{timeout:g} s: NDT has nothing to align")
        return ""

    def _on_board_pose(self, msg: PoseWithCovarianceStamped):
        if self._state in (State.DONE, State.FAILED, State.CALL_SERVICE):
            return

        why = self._not_ready()
        if why:
            # Not an attempt: the call would be refused for a reason the board
            # pose has nothing to do with. Logged when the reason changes only:
            # the detector republishes every second or two.
            reason = f"waiting for the stack: {why}"
            if reason != self._last_reason:
                self._note(reason)
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

        self._attempt_counts = (
            self._matcher_has_map or not self._params.ignore_failures_before_map
        )
        if self._attempt_counts:
            self._attempts += 1
            attempt = "attempt %d/%d" % (self._attempts, self._params.max_attempts)
        else:
            attempt = "attempt before the scan matcher has a map, not counted"
        self.get_logger().info(
            "%s: initializing from board pose at x=%.3f y=%.3f z=%.3f"
            % (
                attempt,
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
        if not self._attempt_counts:
            reason = f"{reason} (the scan matcher has no map yet; not counted)"
            self.get_logger().warn(reason)
            self._last_reason = reason
            self._state = State.WAIT_POSE
            return
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
            KeyValue(key="matcher_has_map", value=str(self._matcher_has_map)),
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
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT from a launch file shuts the context down under spin.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
