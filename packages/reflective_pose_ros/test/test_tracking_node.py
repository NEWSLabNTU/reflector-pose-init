"""board_tracking_node: one scan in, the board in base_link out, stamped with the scan.

Clouds are fed straight into ``_on_cloud`` with the TF pre-resolved, so this
runs without a spin, a bag or a transform publisher, like the initializer's
tests. The detector file is the shipped AutoSDV Robin-W profile, and the
clouds are simulated Robin-W scans in Seyond's native axes.
"""

from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy  # noqa: E402

from reflective_pose_core.geometry import matrix_from_quaternion  # noqa: E402
from reflective_pose_ros.debug_viz import xyzi_cloud  # noqa: E402
from reflective_pose_ros.tracking_node import BoardTrackingNode, TrackingParams  # noqa: E402
from reflective_pose_sim import scenes, vlp32_sim  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAM_FILE = PACKAGE_ROOT / "config" / "board_tracking.param.yaml"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "board_tracking.launch.xml"
PROFILE = (
    PACKAGE_ROOT.parent / "reflective_pose_core" / "reflective_pose_core" / "data"
    / "autosdv_robin_w.yaml"
)
FRAME = "robin_w"
STAMP = Time(sec=1234, nanosec=500_000_000)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    overrides.setdefault("config_file", str(PROFILE))
    return BoardTrackingNode(
        parameter_overrides=[Parameter(name, value=value) for name, value in overrides.items()]
    )


@pytest.fixture
def node():
    node = make_node()
    # AutoSDV's TF: the LiDAR at base_link, Seyond axes rotated into ROS.
    node._transform_base_sensor = scenes.transform_base_sensor(0.0, "robin_w")
    node._sensor_frame = FRAME
    node._published = []
    node._board_pub.publish = node._published.append
    yield node
    node.destroy_node()


def cloud_of(scene, seed=1, stamp=STAMP):
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor="robin_w", seed=seed))
    return xyzi_cloud(scan.points, scan.intensity, FRAME, stamp)


def test_an_accepted_scan_publishes_the_board_in_base_link_with_the_scan_stamp(node):
    scene, truth = scenes.tracking_board_scene(range_m=2.0, bearing_deg=15.0)
    node._on_cloud(cloud_of(scene))

    assert len(node._published) == 1
    message = node._published[0]
    assert message.header.frame_id == "base_link"
    assert message.header.stamp == STAMP

    expected = scenes.transform_base_sensor(0.0) @ truth
    position = message.pose.position
    assert np.linalg.norm(
        [position.x - expected[0, 3], position.y - expected[1, 3], position.z - expected[2, 3]]
    ) < 0.05
    # Bearing to the left is positive, as the follower reads it.
    assert np.degrees(np.arctan2(position.y, position.x)) == pytest.approx(15.0, abs=1.0)
    q = message.pose.orientation
    normal = matrix_from_quaternion([q.x, q.y, q.z, q.w])[:, 0]
    assert np.dot(normal, expected[:3, 0]) > np.cos(np.radians(10.0))


def test_every_scan_is_judged_on_its_own(node):
    """No accumulation: each accepted scan publishes once, each rejected one never."""
    board, _ = scenes.tracking_board_scene(range_m=3.0)
    nothing = scenes.tracking_distractor_scene()
    for seed, scene in enumerate([board, board, nothing, board, nothing]):
        node._on_cloud(cloud_of(scene, seed=seed, stamp=Time(sec=seed)))

    assert [m.header.stamp.sec for m in node._published] == [0, 1, 3]
    assert node._scans == 5
    assert node._detections == 3


def test_a_distractor_scene_publishes_nothing_and_says_why(node):
    node._on_cloud(cloud_of(scenes.tracking_distractor_scene()))
    assert node._published == []
    status = node.diagnostic_status()
    assert status.level == DiagnosticStatus.WARN
    assert "detection rate" in status.message
    assert "no candidate" in status.message


def test_diagnostics_report_rates_and_timings(node):
    scene, _ = scenes.tracking_board_scene(range_m=4.0)
    for seed in range(3):
        node._on_cloud(cloud_of(scene, seed=seed))
    values = {kv.key: kv.value for kv in node.diagnostic_status().values}
    assert values["scans"] == "3"
    assert values["detections"] == "3"
    assert float(values["detection_rate_hz"]) == pytest.approx(3 / 2.0)
    assert float(values["processing_ms_mean"]) > 0.0
    assert values["target_frame"] == "base_link"


def test_no_scans_is_an_error():
    node = make_node()
    try:
        assert node.diagnostic_status().level == DiagnosticStatus.ERROR
    finally:
        node.destroy_node()


def test_another_target_frame_is_looked_up_at_the_scan_stamp(node):
    node._target_frame = "odom"
    seen = []
    odom_from_sensor = np.eye(4)
    odom_from_sensor[:3, :3] = scenes.transform_base_sensor(0.0, "robin_w")[:3, :3]
    odom_from_sensor[:3, 3] = [10.0, 0.0, 0.0]

    def lookup(target, source, stamp):
        seen.append((target, source, stamp.nanoseconds))
        return odom_from_sensor

    node._lookup = lookup
    scene, truth = scenes.tracking_board_scene(range_m=2.0)
    node._on_cloud(cloud_of(scene))

    assert seen == [("odom", FRAME, 1234_500_000_000)]
    message = node._published[0]
    assert message.header.frame_id == "odom"
    assert message.pose.position.x == pytest.approx(12.0, abs=0.05)


def test_the_output_is_volatile_and_the_input_keeps_only_the_latest_scan():
    node = make_node()
    try:
        assert node._board_pub.qos_profile.durability == DurabilityPolicy.VOLATILE
        assert node._cloud_sub.qos_profile.depth == 1
        assert node._board_pub.topic_name == "/board_tracking/board"
        assert node._cloud_sub.topic_name == "/board_tracking/input/pointcloud"
    finally:
        node.destroy_node()


def test_tracking_always_runs_one_scan():
    node = make_node()
    try:
        assert node._detector_params.scan_count == 1
        assert node._config.detector.sensor == "robin_w"
        assert not node.has_parameter("accumulate_scans")
    finally:
        node.destroy_node()


def test_shipped_param_file_declares_exactly_the_node_parameters():
    document = yaml.safe_load(PARAM_FILE.read_text(encoding="utf-8"))
    shipped = document["/**"]["ros__parameters"]
    assert set(shipped) == set(TrackingParams.__dataclass_fields__)
    node = make_node()
    try:
        for name, value in shipped.items():
            assert node.get_parameter(name).value == value, name
    finally:
        node.destroy_node()


def test_launch_file_remaps_the_output_topic():
    text = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '<remap from="~/board" to="$(var output_topic)"/>' in text
    assert 'exec="board_tracking_node"' in text
