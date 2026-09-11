"""ROS node: detect the retroreflective board and publish the vehicle pose.

This node detects and publishes. It does not decide. Subscribe to a cloud, look
up TF, accumulate scans, run ``reflective_pose_core.detect_board``, and publish

* ``~/board_pose``  -- ``geometry_msgs/PoseWithCovarianceStamped``, latched,
  the vehicle pose in ``map`` with the covariance the detection earns, for
  every batch whose lone survivor clears ``detector.min_confidence``
* ``~/debug/*``     -- the board points, the board pose in the sensor frame,
  and a per-cluster rejection marker for every cluster that was discarded
* ``/diagnostics``  -- the last batch's verdict (see ``decision.judge``) with
  its confidence terms, plus state and attempt counts, once a second

No outcome is terminal. An ambiguous batch and a low-confidence batch each
suppress their own pose, say so on /diagnostics, and the next batch is
processed like any other; ``decision.py`` carries the reasoning.

What happens to that pose afterwards is somebody else's problem: whether the
vehicle is stopped enough to trust it, how many attempts are worth making, and
which localization stack is told about it are all policy, and policy lives in
the package that owns the stack. That split is what lets this node run under a
stack that is not Autoware. See docs/design/reflective_pose_detector.md.

One consequence is worth stating rather than discovering. The old node reset its
accumulation whenever the vehicle was moving, because stacking scans without
deskewing them assumes a stationary sensor. The signal it gated on is a vehicle
message this package cannot depend on, so the guard moved out with the policy: a
consumer that cares must gate on its own speed source before acting on a pose
published here.

Two kinds of setting, taken two ways. What the detector *looks for* -- the
board, the gates, the covariance -- is the detector file, one parameter
(``config_file``), because the offline anchoring tool reads the same file and
the two must not be able to drift apart through a launch override. Where the
node is *plugged in* -- frames, accumulation, the motion guard -- is ordinary
ROS parameters (``NodeParams``), shipped as ``config/board_detector.param.yaml``.
The input cloud is not a parameter at all: it is ``~/input/pointcloud``, and a
launch file remaps it.
"""

from dataclasses import dataclass
from dataclasses import fields as dataclass_fields
from dataclasses import is_dataclass
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from reflective_pose_core.config import load_config
from reflective_pose_core.detector import detect_board
from reflective_pose_core.geometry import (
    CovarianceParams,
    covariance_from_detection,
    make_transform,
    map_pose_from_detection,
    matrix_from_euler_rpy,
    matrix_from_quaternion,
    quaternion_from_matrix,
)

from .debug_viz import board_pose_stamped, detection_points_cloud, rejection_marker_array
from .decision import Verdict, judge


@dataclass
class NodeParams:
    """Wiring. Declared as ROS parameters, one per field, defaults as written."""

    sensor_frame: str = "velodyne"
    base_frame: str = "base_link"
    # Feeds DetectorParams.scan_count. The expected return count scales with
    # it, so a node accumulating 10 scans against a detector assuming 1 rejects
    # every real board as ten times too dense. Set here once; load_config
    # derives the rest.
    accumulate_scans: int = 10
    # Stacking scans assumes a stationary sensor -- nothing deskews them, so a
    # batch taken while the vehicle rolls is smeared and the board's extents
    # measure wrong. Empty disables the guard: right on a bench, wrong on a
    # vehicle. nav_msgs/Odometry, geometry_msgs/TwistStamped and
    # geometry_msgs/TwistWithCovarianceStamped are accepted.
    twist_topic: str = ""
    # The motion source's message type, e.g.
    # `geometry_msgs/msg/TwistWithCovarianceStamped`. Empty picks it from the
    # topic's advertised type at startup, which is a race on a vehicle: the
    # detector and the velocity source start together, and a topic not yet
    # advertised is subscribed as Odometry for the life of the node. Name it
    # on a vehicle.
    twist_type: str = ""
    max_speed_for_accumulation: float = 0.05


def declare_node_params(node: Node) -> NodeParams:
    """Declare every ``NodeParams`` field as a parameter and read it back."""
    values = {}
    for item in dataclass_fields(NodeParams):
        node.declare_parameter(item.name, item.default)
        values[item.name] = node.get_parameter(item.name).value
    return NodeParams(**values)


class State(Enum):
    WAIT_TF = "wait_tf"
    ACCUMULATE = "accumulate"
    DETECTED = "detected"


def covariance_params_from_config(config) -> CovarianceParams:
    """The covariance sigmas, from the config's own ``covariance`` section.

    The covariance is a property of the detection -- it grows with range, with
    the plane-fit residual, and with any axis whose bounding edges were never
    seen -- so it is computed here, beside the detection, and shipped with the
    pose rather than recomputed by whoever consumes it.

    Its keys therefore live in a section of their own. They were briefly filed
    under the handoff section, left over from the single-node layout, which
    would have forced this package to name a consumer it is meant to know
    nothing about.
    """
    return config.covariance


class BoardDetectorNode(Node):
    """Detect the board, compose the vehicle pose, publish it."""

    def __init__(self, **node_kwargs):
        super().__init__("board_detector", **node_kwargs)

        # The detector file. Empty means "wherever core says the default
        # lives": $REFLECTIVE_POSE_CONFIG, then the installed package data,
        # then the checkout.
        self.declare_parameter("config_file", "")
        path = self.get_parameter("config_file").value or None
        self._params = declare_node_params(self)

        self._config = load_config(path, scan_count=self._params.accumulate_scans)
        self._detector_params = self._config.detector
        # The gate on what gets published. Part of the detector file, beside
        # the gates that decide what counts as a candidate at all.
        self._min_confidence = float(self._config.detector.min_confidence)
        self._covariance_params = covariance_params_from_config(self._config)
        self._board_pose_in_map = self._board_transform()
        self._accumulate_scans = int(self._params.accumulate_scans)

        # Stacking scans assumes a stationary sensor: nothing deskews them, so a
        # batch taken while the vehicle rolls is smeared and the board's extents
        # measure wrong. The all-in-one node got this from the Autoware velocity
        # gate; keeping it here, on a std message type, is what lets the gate
        # survive the split without the detector learning about Autoware.
        self._max_speed = float(self._params.max_speed_for_accumulation)
        self._speed = 0.0
        self._twist_sub = None

        self._state = State.WAIT_TF
        self._attempts = 0
        self._detections = 0
        self._scans = []
        self._transform_base_sensor: Optional[np.ndarray] = None
        self._last_verdict: Optional[Verdict] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._subscribe_twist(self._params.twist_topic)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # A fixed name under the node, remapped by whoever launches it. A
        # parameter would be a second way to say the same thing.
        self._cloud_sub = self.create_subscription(
            PointCloud2, "~/input/pointcloud", self._on_cloud, sensor_qos
        )

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # Latched, because the consumer of this pose is a cold-start path that
        # may well subscribe after the board was already found.
        self._board_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "~/board_pose", latched
        )
        self._points_pub = self.create_publisher(PointCloud2, "~/debug/board_points", latched)
        self._pose_pub = self.create_publisher(PoseStamped, "~/debug/board_pose", latched)
        self._rejected_pub = self.create_publisher(MarkerArray, "~/debug/rejected", latched)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f"board_detector reading {self._cloud_sub.topic_name}, "
            f"waiting for {self._params.base_frame} <- {self._params.sensor_frame}; "
            f"board {self._config.board.width:.2f} x {self._config.board.height:.2f} m "
            f"at {list(self._config.board.pose_in_map)}"
        )

    def _board_transform(self) -> np.ndarray:
        """The board's pose in the map, as a 4x4."""
        values = self._config.board.pose_in_map
        rotation = matrix_from_euler_rpy(values[3:6])
        return make_transform(rotation, values[0:3])

    def _subscribe_twist(self, topic: str):
        """Optional motion guard, on whichever std type the topic carries.

        Empty topic disables it, which is right on a bench and wrong on a
        vehicle -- so say which one is happening rather than leaving it to be
        discovered from smeared extents later.
        """
        if not topic:
            self.get_logger().warn(
                "twist_topic is empty: scans will be accumulated regardless "
                "of vehicle motion. Correct on a bench; on a vehicle this "
                "silently smears the board's extents."
            )
            return

        from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
        from nav_msgs.msg import Odometry

        # Every type _on_twist can read. TwistWithCovarianceStamped is what
        # Autoware's vehicle_velocity_converter publishes, and before it was
        # listed here such a topic fell through to Odometry and the
        # subscription could never match.
        supported = {
            "nav_msgs/msg/Odometry": Odometry,
            "geometry_msgs/msg/TwistStamped": TwistStamped,
            "geometry_msgs/msg/TwistWithCovarianceStamped": TwistWithCovarianceStamped,
        }
        wanted = self._params.twist_type
        if wanted:
            if wanted not in supported:
                raise ValueError(
                    f"twist_type {wanted!r} is not one the motion guard can read; "
                    f"expected one of {sorted(supported)}"
                )
            kind = supported[wanted]
        else:
            # Pick by what is actually advertised. Nothing published yet means
            # we cannot tell, and Odometry is the guess; twist_type exists so a
            # vehicle never has to rely on this.
            kind = Odometry
            for name, types in self.get_topic_names_and_types():
                if name == topic:
                    for advertised in types:
                        if advertised in supported:
                            kind = supported[advertised]
                            break
                    break

        self._twist_sub = self.create_subscription(kind, topic, self._on_twist, 10)
        self.get_logger().info(
            f"motion guard on {topic} ({kind.__name__}), "
            f"max {self._max_speed:.3f} m/s"
        )

    def _on_twist(self, msg):
        twist = msg.twist.twist if hasattr(msg.twist, "twist") else msg.twist
        self._speed = float(
            np.linalg.norm([twist.linear.x, twist.linear.y, twist.linear.z])
        )

    # -- callbacks ----------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2):
        if self._transform_base_sensor is None:
            self._transform_base_sensor = self._lookup_transform()
            if self._transform_base_sensor is None:
                return
            self._state = State.ACCUMULATE

        # Discard rather than pause: a batch straddling a stop is as smeared as
        # one taken entirely in motion, and the sensor gives us another in
        # 100 ms.
        if self._twist_sub is not None and self._speed > self._max_speed:
            if self._scans:
                self.get_logger().debug(
                    f"moving at {self._speed:.2f} m/s, discarding "
                    f"{len(self._scans)} accumulated scan(s)"
                )
            self._scans = []
            return

        self._scans.append(self._read_cloud(msg))
        if len(self._scans) < self._accumulate_scans:
            return

        points = np.vstack([scan[0] for scan in self._scans])
        intensity = np.concatenate([scan[1] for scan in self._scans])
        self._scans = []
        self._attempt_detection(points, intensity, msg.header.frame_id)

    def _lookup_transform(self) -> Optional[np.ndarray]:
        base = self._params.base_frame
        sensor = self._params.sensor_frame
        try:
            stamped = self._tf_buffer.lookup_transform(base, sensor, rclpy.time.Time())
        except Exception as error:  # tf2 raises several unrelated types
            self.get_logger().debug(f"waiting for {base} <- {sensor}: {error}")
            return None

        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        return make_transform(
            matrix_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w]),
            [translation.x, translation.y, translation.z],
        )

    @staticmethod
    def _read_cloud(msg: PointCloud2):
        array = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        array = np.asarray(array)
        if array.dtype.names is not None:
            points = np.column_stack(
                [array["x"], array["y"], array["z"]]
            ).astype(np.float64)
            intensity = np.asarray(array["intensity"], dtype=np.float64)
        else:
            array = array.astype(np.float64)
            points, intensity = array[:, :3], array[:, 3]
        return points, intensity

    # -- detection ----------------------------------------------------------

    def _attempt_detection(self, points, intensity, frame_id: str):
        self._attempts += 1
        result = detect_board(
            points, intensity, self._transform_base_sensor, self._detector_params
        )
        self._publish_clusters(result, frame_id)

        verdict = judge(result, self._min_confidence)
        self._last_verdict = verdict
        if not verdict.publish:
            # No attempt budget here: the node keeps looking, and a consumer
            # that wants to give up after N tries counts the poses it did not
            # receive. Giving up is policy.
            self.get_logger().warn(f"attempt {self._attempts}: {verdict.message}")
            return

        detection = result.detection
        self._publish_detection(detection, frame_id)

        pose = map_pose_from_detection(
            detection, self._transform_base_sensor, self._board_pose_in_map
        )
        covariance = covariance_from_detection(detection, self._covariance_params)
        horizontal_ok, vertical_ok = detection.centre_constrained
        px, py, pz = float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3])
        yaw = float(np.arctan2(pose[1, 0], pose[0, 0]))
        self.get_logger().info(
            "board detected at %.1f m, %d points, extents %.2f x %.2f, "
            "centre constrained h=%s v=%s, confidence %.2f (%s), "
            "pose: x=%.3f y=%.3f z=%.3f yaw=%.2f rad (%.1f deg)"
            % (
                detection.range_m,
                detection.n_points,
                detection.extents[0],
                detection.extents[1],
                horizontal_ok,
                vertical_ok,
                detection.confidence,
                " ".join(f"{k}={v:.2f}" for k, v in detection.confidence_terms.items()),
                px,
                py,
                pz,
                yaw,
                np.rad2deg(yaw),
            )
        )

        self._board_pose_pub.publish(self._pose_message(pose, covariance))
        self._detections += 1
        self._state = State.DETECTED

    def _pose_message(
        self, pose: np.ndarray, covariance: np.ndarray
    ) -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(pose[0, 3])
        message.pose.pose.position.y = float(pose[1, 3])
        message.pose.pose.position.z = float(pose[2, 3])
        x, y, z, w = quaternion_from_matrix(pose[:3, :3])
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.pose.covariance = covariance.reshape(-1).tolist()
        return message

    # -- debug output -------------------------------------------------------
    #
    # Marker and cloud construction lives in debug_viz.py, shared with the
    # offline anchoring viewer so a map-cloud debug run draws identically to a
    # live one.

    def _publish_detection(self, detection, frame_id: str):
        stamp = self.get_clock().now().to_msg()
        self._pose_pub.publish(board_pose_stamped(detection, frame_id, stamp))

    def _publish_clusters(self, result, frame_id: str):
        """Label every cluster this attempt looked at, kept or discarded.

        Two things this has to get right. When detection fails on site the
        question is always what it saw and why it was discarded, so each
        rejected cluster carries its reason. And an ambiguous result must show
        *both* candidates: without that the operator sees an empty scene and no
        indication of which second object broke the one-board assumption.

        Everything is cleared first. The debug topics are latched, so a stale
        detection from a previous attempt would otherwise sit on screen looking
        like a current one.
        """
        stamp = self.get_clock().now().to_msg()
        self._rejected_pub.publish(rejection_marker_array(result, frame_id, stamp))
        self._points_pub.publish(detection_points_cloud(result, frame_id, stamp))

    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "localization: board_detector"
        status.hardware_id = "board_detector"
        # The level follows the last batch, not the best one: a board found
        # ten batches ago and lost since is a WARN with a reason, not an OK.
        verdict = self._last_verdict
        if verdict is None:
            status.level = DiagnosticStatus.WARN
            status.message = self._state.value
        else:
            status.level = verdict.level
            status.message = verdict.message
        status.values = [
            KeyValue(key="state", value=self._state.value),
            KeyValue(key="attempts", value=str(self._attempts)),
            KeyValue(key="detections", value=str(self._detections)),
        ]
        if verdict is not None:
            status.values += [KeyValue(key=key, value=value) for key, value in verdict.values]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = BoardDetectorNode()
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
