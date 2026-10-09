"""Publish synthetic LiDAR scans so the detector nodes can run without hardware.

Emits the same scenes the detector tests use, plus the static
``base_link -> sensor`` transform the node needs, so the full wiring —
subscription, TF lookup, accumulation, detection, pose publication — is
exercisable on a desk. ``sensor`` picks the beam model (vlp32c, vlp16,
vlp16_hires, robin_w); the tracking scenes are AutoSDV's, and ``walking``
moves the board every scan for the tracking node.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from reflective_pose_core.geometry import quaternion_from_matrix
from reflective_pose_sim import scenes, vlp32_sim

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.FLOAT32, count=1),
]


class ScenePublisher(Node):
    """Render one scene repeatedly, with a fresh noise seed per scan."""

    def __init__(self):
        super().__init__("board_scene_publisher")

        self.declare_parameter("output_topic", "/sensing/lidar/top/pointcloud_raw_ex")
        self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sensor_height", scenes.SENSOR_HEIGHT)
        # The z of the published base_link -> sensor TF. Negative means
        # sensor_height. AutoSDV's calibration puts the LiDAR at z = 0 and its
        # detector profiles carry the real height as an offset; 0.0 here
        # reproduces that.
        self.declare_parameter("tf_sensor_height", -1.0)
        # A name from reflective_pose_core.sensors.
        self.declare_parameter("sensor", "vlp32c")
        # board | distractors | two_boards, or AutoSDV's tracking scenes:
        # tracking | tracking_distractors | walking
        self.declare_parameter("scene", "board")
        # tracking and walking: the handheld board's centre height, and for
        # walking the speed (m/s, positive away) and the walk's length (s),
        # after which it starts again.
        self.declare_parameter("centre_height", scenes.HANDHELD_BOARD_CENTRE_HEIGHT)
        self.declare_parameter("speed_mps", 1.0)
        self.declare_parameter("walk_duration_s", 4.0)
        self.declare_parameter("range_m", 6.0)
        self.declare_parameter("bearing_deg", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("tilt_deg", 0.0)
        self.declare_parameter("blooming", False)
        self.declare_parameter("rate_hz", 10.0)

        self._sensor_height = self.get_parameter("sensor_height").value
        self._sensor = self.get_parameter("sensor").value
        self._frames = self._build_frames()
        self._seed = 0

        self._publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, 5
        )
        self._broadcast_static_transform()
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._publish)
        self.get_logger().info(
            f"publishing synthetic '{self.get_parameter('scene').value}' scans"
        )

    def _build_frames(self):
        """The scenes to cycle through, one per scan."""
        name = self.get_parameter("scene").value
        if name == "walking":
            rate = self.get_parameter("rate_hz").value
            frames = scenes.walking_board_track(
                start_range_m=self.get_parameter("range_m").value,
                speed_mps=self.get_parameter("speed_mps").value,
                duration_s=self.get_parameter("walk_duration_s").value,
                rate_hz=rate,
                bearing_deg=self.get_parameter("bearing_deg").value,
                sway_deg=10.0,
                centre_height=self.get_parameter("centre_height").value,
                sensor_height=self._sensor_height,
            )
            return [scene for _, scene, _ in frames]
        return [self._build_scene(name)]

    def _build_scene(self, name):
        if name == "distractors":
            return scenes.distractor_only_scene(sensor_height=self._sensor_height)
        if name == "two_boards":
            return scenes.two_board_scene(sensor_height=self._sensor_height)
        if name == "tracking_distractors":
            return scenes.tracking_distractor_scene(sensor_height=self._sensor_height)
        if name == "tracking":
            scene, _ = scenes.tracking_board_scene(
                range_m=self.get_parameter("range_m").value,
                bearing_deg=self.get_parameter("bearing_deg").value,
                yaw_deg=self.get_parameter("yaw_deg").value,
                centre_height=self.get_parameter("centre_height").value,
                sensor_height=self._sensor_height,
            )
            return scene

        scene, _ = scenes.board_scene(
            range_m=self.get_parameter("range_m").value,
            bearing_deg=self.get_parameter("bearing_deg").value,
            yaw_deg=self.get_parameter("yaw_deg").value,
            tilt_deg=self.get_parameter("tilt_deg").value,
            sensor_height=self._sensor_height,
        )
        return scene

    def _broadcast_static_transform(self):
        self._static_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter("base_frame").value
        transform.child_frame_id = self.get_parameter("sensor_frame").value
        tf_height = self.get_parameter("tf_sensor_height").value
        if tf_height < 0.0:
            tf_height = self._sensor_height
        transform.transform.translation.z = float(tf_height)
        base_from_sensor = scenes.transform_base_sensor(tf_height, self._sensor)
        x, y, z, w = quaternion_from_matrix(base_from_sensor[:3, :3])
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = x, y, z, w
        self._static_broadcaster.sendTransform(transform)

    def _publish(self):
        scene = self._frames[self._seed % len(self._frames)]
        self._seed += 1
        scan = vlp32_sim.simulate(
            scene,
            vlp32_sim.SimParams(
                sensor=self._sensor,
                seed=self._seed,
                blooming=self.get_parameter("blooming").value,
            ),
        )
        data = np.column_stack(
            (scan.points, scan.intensity, scan.ring.astype(np.float64))
        ).astype(np.float32)

        message = point_cloud2.create_cloud(self._make_header(), FIELDS, data)
        self._publisher.publish(message)

    def _make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("sensor_frame").value
        return header


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
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
