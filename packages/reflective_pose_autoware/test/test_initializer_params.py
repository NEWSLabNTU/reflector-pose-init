"""The handoff policy is ROS parameters, not a section of the detector file.

This node reads no detector file at all: the pose it acts on was composed
upstream, and the only things it decides -- speed gate, attempt budget,
fallback -- are settings of the vehicle stack. They arrive as parameters and
ship as ``board_pose_initializer.param.yaml``.

Needs rclpy and the Autoware messages. Nothing spins.
"""

from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_vehicle_msgs")
pytest.importorskip("autoware_localization_msgs")

from rclpy.parameter import Parameter  # noqa: E402

from reflective_pose_autoware.initializer_node import (  # noqa: E402
    BoardPoseInitializer,
    Policy,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAM_FILE = PACKAGE_ROOT / "config" / "board_pose_initializer.param.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    return BoardPoseInitializer(
        parameter_overrides=[Parameter(name, value=value) for name, value in overrides.items()]
    )


def test_policy_comes_from_parameters():
    node = make_node(max_attempts=2, fallback_to_user_defined_pose=True)
    try:
        assert node._params.max_attempts == 2
        assert node._params.fallback_to_user_defined_pose is True
        assert not node.has_parameter("config_file")
    finally:
        node.destroy_node()


def test_shipped_param_file_declares_exactly_the_node_parameters():
    document = yaml.safe_load(PARAM_FILE.read_text(encoding="utf-8"))
    shipped = document["/**"]["ros__parameters"]
    assert set(shipped) == set(Policy.__dataclass_fields__)
    node = make_node()
    try:
        for name, value in shipped.items():
            assert node.get_parameter(name).value == value, name
    finally:
        node.destroy_node()


def test_client_speaks_the_service_type_autoware_serves():
    """Autoware 1.5.0 (the apt distribution) serves /localization/initialize as
    autoware_localization_msgs/srv/InitializeLocalization (see
    autoware/component_interface_specs/localization.hpp). A client built on
    the older tier4_localization_msgs type has an identical definition but a
    different type name, and rclpy's service_is_ready() then never becomes
    true: the node waited five seconds and reported the service unavailable
    while `ros2 service list` showed it, on the D1a replay of 2026-09-10.
    """
    pytest.importorskip("autoware_localization_msgs")
    node = make_node()
    try:
        assert node._client.srv_type.__module__.startswith("autoware_localization_msgs.")
    finally:
        node.destroy_node()
