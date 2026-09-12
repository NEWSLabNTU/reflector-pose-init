"""The detector node's wiring is ROS parameters; its judgement is the file.

Frames, accumulation and the motion guard describe where the node is plugged
in, so they arrive the way every other ROS node takes such things and can be
set from a launch file or a ``ros__parameters`` YAML. The detector file is
still one parameter, ``config_file``, because the anchoring tool reads the
same file and the two must not be able to drift apart by a launch override.

Needs rclpy. Nothing spins; constructing the node is the whole test.
"""

from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter  # noqa: E402

from reflective_pose_ros.detector_node import (  # noqa: E402
    BoardDetectorNode,
    NodeParams,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAM_FILE = PACKAGE_ROOT / "config" / "board_detector.param.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    node = BoardDetectorNode(
        parameter_overrides=[Parameter(name, value=value) for name, value in overrides.items()]
    )
    return node


def test_accumulate_scans_reaches_the_detector_as_scan_count():
    """The coupling the file used to carry: the node stacks N, the gate expects N."""
    node = make_node(accumulate_scans=3)
    try:
        assert node._accumulate_scans == 3
        assert node._detector_params.scan_count == 3
    finally:
        node.destroy_node()


def test_frames_come_from_parameters():
    node = make_node(sensor_frame="lidar_top", base_frame="vehicle")
    try:
        assert node._params.sensor_frame == "lidar_top"
        assert node._params.base_frame == "vehicle"
    finally:
        node.destroy_node()


def test_input_cloud_is_a_fixed_name_for_remapping():
    """No input_topic parameter: the subscription is remapped like any other."""
    node = make_node()
    try:
        assert not node.has_parameter("input_topic")
        assert node._cloud_sub.topic_name == "/board_detector/input/pointcloud"
    finally:
        node.destroy_node()


def test_config_file_from_the_old_layout_fails_loudly(tmp_path):
    old = tmp_path / "reflective_pose.yaml"
    old.write_text("ros:\n  accumulate_scans: 10\n", encoding="utf-8")
    with pytest.raises(ValueError, match="board_detector.param.yaml"):
        make_node(config_file=str(old))


def test_shipped_param_file_declares_exactly_the_node_parameters():
    """The YAML the launch file hands the node must not drift from the code."""
    document = yaml.safe_load(PARAM_FILE.read_text(encoding="utf-8"))
    shipped = document["/**"]["ros__parameters"]
    assert set(shipped) == set(NodeParams.__dataclass_fields__)
    node = make_node()
    try:
        for name, value in shipped.items():
            assert node.get_parameter(name).value == value, name
    finally:
        node.destroy_node()


# -- the motion guard's message type (golf-cart phase 7, A3) -----------------
#
# The vehicle's natural motion source is vehicle_velocity_converter's
# `twist_with_covariance`, a geometry_msgs/TwistWithCovarianceStamped. The node
# picked its subscription type by the topic's advertised type and knew only
# Odometry and TwistStamped, so that topic fell through to Odometry and the
# subscription could never match. And picking by advertisement is a race on a
# vehicle, where the detector and the velocity converter start together: a
# topic not yet advertised is subscribed as Odometry for the life of the node.
# `twist_type` names the type outright; empty keeps the detection.


def _wait_until_advertised(node, topic, timeout_s=3.0):
    import time

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if any(name == topic for name, _ in node.get_topic_names_and_types()):
            return
        time.sleep(0.05)
    raise AssertionError(f"{topic} never appeared in the graph")


def test_twist_with_covariance_stamped_is_detected_when_advertised():
    from geometry_msgs.msg import TwistWithCovarianceStamped

    advertiser = rclpy.create_node("twist_cov_advertiser")
    advertiser.create_publisher(TwistWithCovarianceStamped, "/test_a3/twist_cov", 10)
    try:
        _wait_until_advertised(advertiser, "/test_a3/twist_cov")
        node = make_node(twist_topic="/test_a3/twist_cov")
        try:
            assert node._twist_sub.msg_type is TwistWithCovarianceStamped
        finally:
            node.destroy_node()
    finally:
        advertiser.destroy_node()


def test_explicit_twist_type_needs_no_publisher_to_exist_yet():
    from geometry_msgs.msg import TwistWithCovarianceStamped

    node = make_node(
        twist_topic="/test_a3/not_advertised",
        twist_type="geometry_msgs/msg/TwistWithCovarianceStamped",
    )
    try:
        assert node._twist_sub.msg_type is TwistWithCovarianceStamped
    finally:
        node.destroy_node()


def test_an_unsupported_twist_type_is_refused_by_name():
    with pytest.raises(ValueError, match="twist_type"):
        make_node(twist_topic="/test_a3/x", twist_type="std_msgs/msg/Float64")


def test_speed_is_read_from_a_twist_with_covariance_stamped():
    from geometry_msgs.msg import TwistWithCovarianceStamped

    node = make_node(
        twist_topic="/test_a3/speed",
        twist_type="geometry_msgs/msg/TwistWithCovarianceStamped",
    )
    try:
        msg = TwistWithCovarianceStamped()
        msg.twist.twist.linear.x = 0.3
        msg.twist.twist.linear.y = 0.4
        node._on_twist(msg)
        assert node._speed == pytest.approx(0.5)
    finally:
        node.destroy_node()
