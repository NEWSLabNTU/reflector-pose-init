"""ROS node: detect the retroreflective board and publish the vehicle pose.

This node detects and publishes. It does not decide. Subscribe to a cloud, look
up TF, accumulate scans, run ``reflective_pose_core.detect_board``, and publish

* ``~/board_pose``  -- ``geometry_msgs/PoseWithCovarianceStamped``, latched,
  the vehicle pose in ``map`` with the covariance the detection earns
* ``~/debug/*``     -- the board points, the board pose in the sensor frame,
  and a per-cluster rejection marker for every cluster that was discarded
* ``/diagnostics``  -- state and attempt count, once a second

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

Configuration is one file, not thirty parameters -- see the ``config_file``
parameter below.
"""

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
from reflective_pose_core.detector import Status, detect_board
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


class State(Enum):
    WAIT_TF = "wait_tf"
    ACCUMULATE = "accumulate"
    DETECTED = "detected"
    FAILED = "failed"


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

    def __init__(self):
        super().__init__("board_detector")

        # The only parameter. Empty means "wherever core says the canonical
        # file lives": $REFLECTIVE_POSE_CONFIG, then the installed package
        # data, then the checkout.
        self.declare_parameter("config_file", "")
        path = self.get_parameter("config_file").value or None

        self._config = load_config(path)
        # Resolve the runtime policy explicitly. The map CLI resolves a
        # separate policy from the same board contract; sharing a flat
        # DetectorParams here would make its base_link height and scan-density
        # gates leak into offline map anchoring.
        self._detector_params = self._config.runtime_detector
        self._covariance_params = covariance_params_from_config(self._config)
        self._board_pose_in_map = self._board_transform()
        self._accumulate_scans = int(self._config.ros.accumulate_scans)

        # Stacking scans assumes a stationary sensor: nothing deskews them, so a
        # batch taken while the vehicle rolls is smeared and the board's extents
        # measure wrong. The all-in-one node got this from the Autoware velocity
        # gate; keeping it here, on a std message type, is what lets the gate
        # survive the split without the detector learning about Autoware.
        self._max_speed = float(self._config.ros.max_speed_for_accumulation)
        self._speed = 0.0
        self._twist_sub = None

        self._state = State.WAIT_TF
        self._attempts = 0
        self._detections = 0
        self._scans = []
        self._transform_base_sensor: Optional[np.ndarray] = None
        self._last_reason = ""

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._subscribe_twist(self._config.ros.twist_topic)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, self._config.ros.input_topic, self._on_cloud, sensor_qos
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
            f"board_detector reading {self._config.ros.input_topic}, "
            f"waiting for {self._config.ros.base_frame} <- "
            f"{self._config.ros.sensor_frame}"
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
                "ros.twist_topic is empty: scans will be accumulated regardless "
                "of vehicle motion. Correct on a bench; on a vehicle this "
                "silently smears the board's extents."
            )
            return

        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped

        # Pick by what is actually advertised, so one config key serves either
        # source. Nothing published yet means we cannot tell; Odometry is the
        # commoner of the two in this stack.
        kind = Odometry
        for name, types in self.get_topic_names_and_types():
            if name == topic:
                if any(t.endswith("TwistStamped") for t in types):
                    kind = TwistStamped
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
        if self._state is State.FAILED:
            return

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
        base = self._config.ros.base_frame
        sensor = self._config.ros.sensor_frame
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

        if result.status is Status.AMBIGUOUS:
            # The map holds one board. A second survivor means that assumption
            # is broken, and choosing between them would produce a confident
            # wrong pose, so this failure is terminal rather than retried.
            details = "; ".join(
                f"{candidate.range_m:.1f} m, {candidate.n_points} pts, "
                f"{candidate.extents[0]:.2f}x{candidate.extents[1]:.2f}"
                for candidate in result.candidates
            )
            rejected = ", ".join(
                f"{rejection.reason} {rejection.detail}".strip()
                for rejection in result.rejections
            )
            self._fail(
                f"ambiguous: {len(result.candidates)} board candidates survived "
                f"gating [{details}] out of {result.n_clusters} clusters "
                f"(rejected: {rejected or 'none'})"
            )
            return

        if result.status is not Status.OK:
            # No attempt budget here: the node keeps looking, and a consumer
            # that wants to give up after N tries counts the poses it did not
            # receive. Giving up is policy.
            reasons = ", ".join(
                f"{rejection.reason} {rejection.detail}".strip()
                for rejection in result.rejections
            )
            reason = (
                f"no candidate (retro points {result.n_after_gates}, "
                f"clusters {result.n_clusters}): {reasons or 'nothing clustered'}"
            )
            self.get_logger().warn(f"attempt {self._attempts}: {reason}")
            self._last_reason = reason
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
            "centre constrained h=%s v=%s, pose: x=%.3f y=%.3f z=%.3f yaw=%.2f rad (%.1f deg)"
            % (
                detection.range_m,
                detection.n_points,
                detection.extents[0],
                detection.extents[1],
                horizontal_ok,
                vertical_ok,
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
        self._last_reason = f"board detected, pose published ({self._detections})"

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

    def _fail(self, reason: str):
        self._state = State.FAILED
        self._last_reason = reason
        self.get_logger().error(reason)

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
        if self._state is State.DETECTED:
            status.level = DiagnosticStatus.OK
        elif self._state is State.FAILED:
            status.level = DiagnosticStatus.ERROR
        else:
            status.level = DiagnosticStatus.WARN
        status.message = self._last_reason or self._state.value
        status.values = [
            KeyValue(key="state", value=self._state.value),
            KeyValue(key="attempts", value=str(self._attempts)),
            KeyValue(key="detections", value=str(self._detections)),
        ]

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
