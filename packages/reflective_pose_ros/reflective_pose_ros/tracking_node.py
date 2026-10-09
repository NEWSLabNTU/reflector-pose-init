"""ROS node: find the board in every scan and publish where it is.

The tracking counterpart of ``board_detector_node``. Same detector, same
detector file, same verdict; a different question. The initializer asks
"where is the *vehicle*, given a board at a surveyed place?" once, from a
stationary car, and answers in ``map``. Tracking asks "where is the *board*,
relative to me, now?" on every scan, from a moving car, and answers in
``base_link``. For each cloud on ``~/input/pointcloud`` it

* runs ``reflective_pose_core.detect_board`` on that one scan -- no
  accumulation, ``scan_count`` is always 1 -- and judges it with
  ``decision.judge``, exactly as the initializer does;
* on an accepted detection only, publishes the board pose as
  ``geometry_msgs/PoseStamped`` on ``~/board``, in ``target_frame``
  (``base_link`` by default), stamped with the *scan's* stamp, not the time it
  was processed, so a downstream tracker can account for the latency;
* publishes ``/diagnostics`` once a second: the scan rate, the detection rate,
  the processing time per scan and the scan age at publication, over a
  sliding window, plus the last verdict.

The pose is the board's frame: origin at the board centre, x the board normal
pointing back at the sensor, y to the board's left as seen from the sensor,
z up. A follower that only wants range and bearing reads the position.

Why a separate node rather than a mode of ``board_detector_node``: every piece
of the initializer's wiring is wrong for tracking, not merely unused. Its pose
is the vehicle's, in ``map``, through a surveyed board pose; its output is
latched, which hands a late subscriber a board position from minutes ago;
it stacks scans and discards them while the vehicle moves, and moving is the
whole point here. A ``mode`` switch would put a branch in front of each of
those and make every parameter mean two things. What the two nodes do share --
reading a cloud, the detector, the verdict, the debug markers -- is already
shared code.

Nothing here assumes Autoware, and no message type outside common_interfaces
is used, for the same reason as the initializer: the consumer brings its own
policy.
"""

import collections
import time
from dataclasses import dataclass
from dataclasses import fields as dataclass_fields
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from reflective_pose_core.config import load_config
from reflective_pose_core.detector import Status, detect_board
from reflective_pose_core.geometry import (
    board_pose_in_sensor,
    make_transform,
    matrix_from_quaternion,
    quaternion_from_matrix,
)

from .debug_viz import (
    board_outline_marker_array,
    clear_all_marker,
    detection_points_cloud,
    rejection_marker_array,
)
from .decision import Verdict, judge
from .detector_node import BoardDetectorNode


@dataclass
class TrackingParams:
    """Wiring. Declared as ROS parameters, one per field, defaults as written."""

    # The cloud's frame. Empty takes it from each cloud's header, which is
    # what the TF lookup needs anyway; set it only to override a driver that
    # stamps the wrong frame.
    sensor_frame: str = ""
    # The frame the detector's height gates are measured in, plus
    # detector.runtime.base_link_height_above_ground. Static TF to the sensor
    # must exist; it is looked up once.
    base_frame: str = "base_link"
    # The frame the board pose is published in. Empty is base_frame, which
    # reuses the static lookup; anything else is looked up at each scan's
    # stamp, so a moving frame (odom, map) is placed where it was at the scan.
    target_frame: str = ""
    # ~/debug/* markers and clouds, every scan. Off saves the message
    # construction on a CPU-bound vehicle; the diagnostics are always on.
    publish_debug: bool = True
    # Sliding window over which the rates and timings on /diagnostics are
    # computed, seconds.
    diagnostics_window: float = 2.0
    # Below this detection rate (Hz, over the window) /diagnostics is WARN.
    min_detection_rate: float = 5.0


def declare_tracking_params(node: Node) -> TrackingParams:
    """Declare every ``TrackingParams`` field as a parameter and read it back."""
    values = {}
    for item in dataclass_fields(TrackingParams):
        node.declare_parameter(item.name, item.default)
        values[item.name] = node.get_parameter(item.name).value
    return TrackingParams(**values)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


class _Window:
    """Timestamped samples kept for ``span`` seconds of wall time."""

    def __init__(self, span: float):
        self.span = float(span)
        self.samples: Deque[Tuple[float, float]] = collections.deque()

    def add(self, now: float, value: float = 0.0):
        self.samples.append((now, value))
        self.trim(now)

    def trim(self, now: float):
        while self.samples and now - self.samples[0][0] > self.span:
            self.samples.popleft()

    def rate(self, now: float) -> float:
        self.trim(now)
        return len(self.samples) / self.span if self.span > 0.0 else 0.0

    def values(self, now: float) -> np.ndarray:
        self.trim(now)
        return np.array([value for _, value in self.samples], dtype=np.float64)


class BoardTrackingNode(Node):
    """Detect the board in every scan and publish its pose in base_link."""

    def __init__(self, **node_kwargs):
        super().__init__("board_tracking", **node_kwargs)

        self.declare_parameter("config_file", "")
        path = self.get_parameter("config_file").value or None
        self._params = declare_tracking_params(self)

        # One scan, always: the density model is told so, and nothing is
        # stacked, so a moving car measures an unsmeared board.
        self._config = load_config(path, scan_count=1)
        self._detector_params = self._config.runtime_detector
        self._height_offset = float(
            self._config.detector.runtime.base_link_height_above_ground
        )
        self._min_confidence = float(self._detector_params.min_confidence)
        self._target_frame = self._params.target_frame or self._params.base_frame

        self._transform_base_sensor: Optional[np.ndarray] = None
        self._sensor_frame: Optional[str] = None
        self._last_verdict: Optional[Verdict] = None
        self._last_status = "waiting for a cloud"
        self._scans = 0
        self._detections = 0

        window = float(self._params.diagnostics_window)
        self._scan_window = _Window(window)
        self._detection_window = _Window(window)
        self._processing_window = _Window(window)
        self._age_window = _Window(window)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Depth 1: if a scan arrives while one is still being processed, the
        # older one is the one to lose. Best effort matches any driver.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, "~/input/pointcloud", self._on_cloud, sensor_qos
        )

        # Volatile, unlike the initializer's latched pose: a follower that
        # subscribes late must not be handed a board position from the past.
        self._board_pub = self.create_publisher(PoseStamped, "~/board", 10)
        self._points_pub = self.create_publisher(PointCloud2, "~/debug/board_points", 1)
        self._rejected_pub = self.create_publisher(MarkerArray, "~/debug/rejected", 1)
        self._outline_pub = self.create_publisher(MarkerArray, "~/debug/board_outline", 1)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_timer(1.0, self._publish_diagnostics)
        board = self._config.board
        self.get_logger().info(
            f"board_tracking reading {self._cloud_sub.topic_name}, publishing "
            f"{self._board_pub.topic_name} in {self._target_frame}; sensor "
            f"{self._config.detector.sensor}, board {board.width:.2f} x "
            f"{board.height:.2f} m centred {self._detector_params.board_centre_height:.2f} "
            f"+/- {self._detector_params.centre_height_tolerance:.2f} m, range "
            f"{self._detector_params.range_min:.1f}-{self._detector_params.range_max:.1f} m"
        )

    # -- TF -------------------------------------------------------------------

    def _lookup(self, target: str, source: str, stamp) -> Optional[np.ndarray]:
        try:
            stamped = self._tf_buffer.lookup_transform(target, source, stamp)
        except Exception as error:  # tf2 raises several unrelated types
            self._last_status = f"waiting for {target} <- {source}: {error}"
            self.get_logger().debug(self._last_status)
            return None
        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        return make_transform(
            matrix_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w]),
            [translation.x, translation.y, translation.z],
        )

    def _transform_target_sensor(self, sensor_frame: str, stamp) -> Optional[np.ndarray]:
        if self._target_frame == self._params.base_frame:
            return self._transform_base_sensor
        return self._lookup(self._target_frame, sensor_frame, Time.from_msg(stamp))

    # -- the scan -------------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2):
        started = time.monotonic()
        sensor_frame = self._params.sensor_frame or msg.header.frame_id
        if self._transform_base_sensor is None or sensor_frame != self._sensor_frame:
            self._transform_base_sensor = self._lookup(
                self._params.base_frame, sensor_frame, Time()
            )
            if self._transform_base_sensor is None:
                return
            self._sensor_frame = sensor_frame

        self._scans += 1
        self._scan_window.add(started)

        points, intensity = BoardDetectorNode._read_cloud(msg)
        result = detect_board(
            points,
            intensity,
            self._transform_base_sensor,
            self._detector_params,
            height_offset=self._height_offset,
        )
        verdict = judge(result, self._min_confidence)
        self._last_verdict = verdict

        if self._params.publish_debug:
            self._publish_debug(result, sensor_frame, msg.header.stamp)

        if verdict.publish:
            transform = self._transform_target_sensor(sensor_frame, msg.header.stamp)
            if transform is not None:
                pose = transform @ board_pose_in_sensor(result.detection)
                self._board_pub.publish(self._pose_message(pose, msg.header.stamp))
                self._detections += 1
                finished = time.monotonic()
                self._detection_window.add(finished)
                age = self.get_clock().now().nanoseconds * 1e-9 - _stamp_seconds(
                    msg.header.stamp
                )
                self._age_window.add(finished, age)

        self._processing_window.add(started, time.monotonic() - started)

    def _pose_message(self, pose: np.ndarray, stamp) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._target_frame
        message.header.stamp = stamp
        message.pose.position.x = float(pose[0, 3])
        message.pose.position.y = float(pose[1, 3])
        message.pose.position.z = float(pose[2, 3])
        x, y, z, w = quaternion_from_matrix(pose[:3, :3])
        message.pose.orientation.x = x
        message.pose.orientation.y = y
        message.pose.orientation.z = z
        message.pose.orientation.w = w
        return message

    def _publish_debug(self, result, frame_id: str, stamp):
        """Same markers as the initializer, stamped with the scan."""
        self._rejected_pub.publish(rejection_marker_array(result, frame_id, stamp))
        self._points_pub.publish(detection_points_cloud(result, frame_id, stamp))
        if result.status is Status.OK and result.detection is not None:
            board = self._config.board
            outline = board_outline_marker_array(
                result.detection, (board.width, board.height), frame_id, stamp
            )
        else:
            outline = MarkerArray()
            outline.markers.append(clear_all_marker())
        self._outline_pub.publish(outline)

    # -- diagnostics ----------------------------------------------------------

    def diagnostic_status(self) -> DiagnosticStatus:
        """The tracking health over the last window, as one status."""
        now = time.monotonic()
        scan_rate = self._scan_window.rate(now)
        detection_rate = self._detection_window.rate(now)
        processing = self._processing_window.values(now)
        ages = self._age_window.values(now)

        status = DiagnosticStatus()
        status.name = "perception: board_tracking"
        status.hardware_id = "board_tracking"
        verdict = self._last_verdict
        if scan_rate == 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = f"no scans in {self._params.diagnostics_window:.1f} s: {self._last_status}"
        elif detection_rate < self._params.min_detection_rate:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f"detection rate {detection_rate:.1f} Hz < "
                f"{self._params.min_detection_rate:.1f} Hz"
                + (f"; last scan: {verdict.message}" if verdict is not None else "")
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"tracking at {detection_rate:.1f} Hz"

        def stat(values, scale=1e3):
            if len(values) == 0:
                return "-", "-"
            return f"{scale * values.mean():.1f}", f"{scale * values.max():.1f}"

        processing_mean, processing_max = stat(processing)
        age_mean, age_max = stat(ages)
        status.values = [
            KeyValue(key="scans", value=str(self._scans)),
            KeyValue(key="detections", value=str(self._detections)),
            KeyValue(key="scan_rate_hz", value=f"{scan_rate:.1f}"),
            KeyValue(key="detection_rate_hz", value=f"{detection_rate:.1f}"),
            KeyValue(key="processing_ms_mean", value=processing_mean),
            KeyValue(key="processing_ms_max", value=processing_max),
            KeyValue(key="scan_age_ms_mean", value=age_mean),
            KeyValue(key="scan_age_ms_max", value=age_max),
            KeyValue(key="target_frame", value=self._target_frame),
        ]
        if verdict is not None:
            status.values += [KeyValue(key=key, value=value) for key, value in verdict.values]
        return status

    def _publish_diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [self.diagnostic_status()]
        self._diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = BoardTrackingNode()
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
