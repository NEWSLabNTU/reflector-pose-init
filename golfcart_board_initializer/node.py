"""ROS node: initialize localization from the retroreflective board.

Runs once at startup and then idles. Everything geometric lives in detector.py
and geometry.py, which are ROS-free; this file is wiring, a state machine, and
diagnostics.

See docs/design/board_pose_initializer.md.
"""

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

from .debug_viz import board_pose_stamped, detection_points_cloud, rejection_marker_array
from .detector import DetectorParams, Status, detect_board
from .geometry import (
    CovarianceParams,
    covariance_from_detection,
    make_transform,
    map_pose_from_detection,
    matrix_from_euler_rpy,
    matrix_from_quaternion,
    quaternion_from_matrix,
)

try:
    from autoware_vehicle_msgs.msg import VelocityReport
except ImportError:  # pragma: no cover - only when the vehicle msgs are absent
    VelocityReport = None

from tier4_localization_msgs.srv import InitializeLocalization


class State(Enum):
    WAIT_TF = "wait_tf"
    ACCUMULATE = "accumulate"
    CALL_SERVICE = "call_service"
    DONE = "done"
    FAILED = "failed"


class BoardPoseInitializer(Node):
    """Detect the board, compose the vehicle pose, seed the pose initializer."""

    def __init__(self):
        super().__init__("board_pose_initializer")

        self._declare_parameters()
        self._detector_params = self._build_detector_params()
        self._covariance_params = CovarianceParams(
            sigma_xy_base=self.get_parameter("sigma_xy_base").value,
            sigma_xy_per_metre=self.get_parameter("sigma_xy_per_metre").value,
            sigma_z=self.get_parameter("sigma_z").value,
            sigma_yaw_base=self.get_parameter("sigma_yaw_base").value,
            safety_factor=self.get_parameter("covariance_safety_factor").value,
        )

        self._state = State.WAIT_TF
        self._attempts = 0
        self._scans = []
        self._speed = 0.0
        self._transform_base_sensor: Optional[np.ndarray] = None
        self._last_reason = ""

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._on_cloud,
            sensor_qos,
        )
        if VelocityReport is not None:
            self.create_subscription(
                VelocityReport,
                "/vehicle/status/velocity_status",
                self._on_velocity,
                sensor_qos,
            )

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._points_pub = self.create_publisher(PointCloud2, "~/debug/board_points", latched)
        self._pose_pub = self.create_publisher(PoseStamped, "~/debug/board_pose", latched)
        self._rejected_pub = self.create_publisher(MarkerArray, "~/debug/rejected", latched)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "~/debug/initial_pose", latched
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self._client = self.create_client(
            InitializeLocalization, self.get_parameter("initialize_service").value
        )

        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info("board_pose_initializer waiting for TF and point clouds")

    # -- parameters ---------------------------------------------------------

    def _declare_parameters(self):
        defaults = DetectorParams()
        self.declare_parameter("input_topic", "/sensing/lidar/top/pointcloud_raw_ex")
        self.declare_parameter("initialize_service", "/localization/initialize")
        self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("accumulate_scans", 10)
        self.declare_parameter("max_speed_for_init", 0.05)
        self.declare_parameter("max_attempts", 5)
        self.declare_parameter("fallback_to_user_defined_pose", False)
        self.declare_parameter("dry_run", False)

        self.declare_parameter("intensity_threshold", defaults.intensity_threshold)
        self.declare_parameter("range_min", defaults.range_min)
        self.declare_parameter("range_max", defaults.range_max)
        self.declare_parameter("height_min", defaults.height_min)
        self.declare_parameter("height_max", defaults.height_max)
        self.declare_parameter("cluster_tolerance", defaults.cluster_tolerance)
        self.declare_parameter("cluster_min_points", defaults.cluster_min_points)
        self.declare_parameter("board_width", defaults.board_width)
        self.declare_parameter("board_height", defaults.board_height)
        self.declare_parameter("board_centre_height", defaults.board_centre_height)
        self.declare_parameter("extent_tolerance", list(defaults.extent_tolerance))
        self.declare_parameter("planarity_max_thickness", defaults.planarity_max_thickness)
        self.declare_parameter("verticality_max_dot", defaults.verticality_max_dot)
        self.declare_parameter("centre_height_tolerance", defaults.centre_height_tolerance)
        self.declare_parameter("density_max_ratio", defaults.density_max_ratio)
        self.declare_parameter("density_check_enabled", defaults.density_check_enabled)
        self.declare_parameter("azimuth_step_rad", defaults.azimuth_step_rad)

        # [x, y, z, roll, pitch, yaw], with angles in radians.
        self.declare_parameter("board_pose_in_map", [0.0, 0.0, 1.075, 0.0, 0.0, 0.0])

        self.declare_parameter("sigma_xy_base", 0.15)
        self.declare_parameter("sigma_xy_per_metre", 0.03)
        self.declare_parameter("sigma_z", 0.10)
        self.declare_parameter("sigma_yaw_base", 0.05)
        self.declare_parameter("covariance_safety_factor", 2.0)

    def _build_detector_params(self) -> DetectorParams:
        extent = self.get_parameter("extent_tolerance").value
        return DetectorParams(
            intensity_threshold=self.get_parameter("intensity_threshold").value,
            range_min=self.get_parameter("range_min").value,
            range_max=self.get_parameter("range_max").value,
            height_min=self.get_parameter("height_min").value,
            height_max=self.get_parameter("height_max").value,
            cluster_tolerance=self.get_parameter("cluster_tolerance").value,
            cluster_min_points=self.get_parameter("cluster_min_points").value,
            board_width=self.get_parameter("board_width").value,
            board_height=self.get_parameter("board_height").value,
            board_centre_height=self.get_parameter("board_centre_height").value,
            extent_tolerance=(float(extent[0]), float(extent[1])),
            planarity_max_thickness=self.get_parameter("planarity_max_thickness").value,
            verticality_max_dot=self.get_parameter("verticality_max_dot").value,
            centre_height_tolerance=self.get_parameter("centre_height_tolerance").value,
            density_max_ratio=self.get_parameter("density_max_ratio").value,
            density_check_enabled=self.get_parameter("density_check_enabled").value,
            azimuth_step_rad=self.get_parameter("azimuth_step_rad").value,
            scan_count=self.get_parameter("accumulate_scans").value,
        )

    def _board_pose_in_map(self) -> np.ndarray:
        values = self.get_parameter("board_pose_in_map").value
        if len(values) != 6:
            raise ValueError(
                "board_pose_in_map must be [x, y, z, roll, pitch, yaw] "
                "with angles in radians"
            )
        rotation = matrix_from_euler_rpy(values[3:6])
        return make_transform(rotation, values[0:3])

    # -- callbacks ----------------------------------------------------------

    def _on_velocity(self, msg):
        self._speed = abs(float(msg.longitudinal_velocity))

    def _on_cloud(self, msg: PointCloud2):
        if self._state in (State.DONE, State.FAILED, State.CALL_SERVICE):
            return

        if self._transform_base_sensor is None:
            self._transform_base_sensor = self._lookup_transform()
            if self._transform_base_sensor is None:
                return
            self._state = State.ACCUMULATE

        if self._speed > self.get_parameter("max_speed_for_init").value:
            # A moving vehicle invalidates the accumulation, which assumes the
            # scans can be stacked without deskewing.
            if self._scans:
                self.get_logger().info("vehicle moving, restarting accumulation")
            self._scans = []
            return

        self._scans.append(self._read_cloud(msg))
        if len(self._scans) < self.get_parameter("accumulate_scans").value:
            return

        points = np.vstack([scan[0] for scan in self._scans])
        intensity = np.concatenate([scan[1] for scan in self._scans])
        self._scans = []
        self._attempt_detection(points, intensity, msg.header.frame_id)

    def _lookup_transform(self) -> Optional[np.ndarray]:
        base = self.get_parameter("base_frame").value
        sensor = self.get_parameter("sensor_frame").value
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
            if self._attempts >= self.get_parameter("max_attempts").value:
                self._maybe_fallback(reason)
            return

        detection = result.detection
        self._publish_detection(detection, frame_id)

        pose = map_pose_from_detection(
            detection, self._transform_base_sensor, self._board_pose_in_map()
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
        self._call_initialize(pose, covariance)

    def _call_initialize(self, pose: np.ndarray, covariance: np.ndarray):
        self._state = State.CALL_SERVICE
        message = self._pose_message(pose, covariance)

        if self.get_parameter("dry_run").value:
            # Exercises detection and pose composition without a localization
            # stack, which is what makes the synthetic scenes useful on a desk.
            self._initial_pose_pub.publish(message)
            self._state = State.DONE
            self._last_reason = "dry run: pose published, service not called"
            self.get_logger().info(self._last_reason)
            return

        if not self._client.wait_for_service(timeout_sec=5.0):
            self._fail("pose initializer service unavailable")
            return

        request = InitializeLocalization.Request()
        request.pose_with_covariance = [message]
        # AUTO, not DIRECT: the board supplies a guess and NDT align refines it.
        # DIRECT would convert every detection error into a localization error.
        request.method = InitializeLocalization.Request.AUTO

        future = self._client.call_async(request)
        future.add_done_callback(self._on_service_response)

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

    def _on_service_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._fail(f"initialize service raised: {error}")
            return

        if response.status.success:
            self._state = State.DONE
            self._last_reason = "initialized from board"
            self.get_logger().info("localization initialized from the board")
        else:
            self._fail(f"initialize service rejected: {response.status.message}")

    def _maybe_fallback(self, reason: str):
        if not self.get_parameter("fallback_to_user_defined_pose").value:
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

    def _fail(self, reason: str):
        self._state = State.FAILED
        self._last_reason = reason
        self.get_logger().error(reason)

    # -- debug output -------------------------------------------------------
    #
    # Marker and cloud construction lives in debug_viz.py, shared with the
    # offline anchoring CLI so a map-cloud debug run draws identically to a
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
