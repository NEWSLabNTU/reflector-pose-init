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
