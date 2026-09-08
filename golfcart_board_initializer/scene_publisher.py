"""Publish synthetic VLP-32C scans so the initializer can run without hardware.

Emits the same scenes the detector tests use, plus the static
``base_link -> velodyne`` transform the node needs, so the full wiring —
subscription, TF lookup, accumulation, detection, service call — is exercisable
on a desk.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from .simulation import scenes, vlp32_sim

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
        self.declare_parameter("scene", "board")  # board | distractors | two_boards
        self.declare_parameter("range_m", 6.0)
        self.declare_parameter("bearing_deg", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("tilt_deg", 0.0)
        self.declare_parameter("blooming", False)
        self.declare_parameter("rate_hz", 10.0)

        self._sensor_height = self.get_parameter("sensor_height").value
        self._scene = self._build_scene()
        self._seed = 0

        self._publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, 5
        )
        self._broadcast_static_transform()
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._publish)
        self.get_logger().info(
            f"publishing synthetic '{self.get_parameter('scene').value}' scans"
        )

    def _build_scene(self):
        name = self.get_parameter("scene").value
        if name == "distractors":
            return scenes.distractor_only_scene(sensor_height=self._sensor_height)
        if name == "two_boards":
            return scenes.two_board_scene(sensor_height=self._sensor_height)

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
        transform.transform.translation.z = float(self._sensor_height)
        transform.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(transform)

    def _publish(self):
        self._seed += 1
        scan = vlp32_sim.simulate(
            self._scene,
            vlp32_sim.SimParams(
                seed=self._seed, blooming=self.get_parameter("blooming").value
            ),
        )
        data = np.column_stack(
            (scan.points, scan.intensity, scan.ring.astype(np.float64))
        ).astype(np.float32)

        message = point_cloud2.create_cloud(
            self._make_header(), FIELDS, data.tolist()
        )
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
