"""The node keeps going: an ambiguous or low-confidence batch suppresses its
own pose and nothing else.

Clouds are fed straight into ``_on_cloud`` with the TF pre-resolved, so this
runs without a spin, a bag or a transform publisher. The detector gates are
the simulator's, because the scenes are.
"""

import pytest

rclpy = pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from reflective_pose_core.detector import DetectorParams  # noqa: E402
from reflective_pose_ros.debug_viz import xyzi_cloud  # noqa: E402
from reflective_pose_ros.detector_node import BoardDetectorNode  # noqa: E402
from reflective_pose_sim import scenes, vlp32_sim  # noqa: E402

FRAME = "velodyne"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    node = BoardDetectorNode(
        parameter_overrides=[Parameter("accumulate_scans", value=1)]
    )
    node._transform_base_sensor = scenes.transform_base_sensor()
    node._detector_params = DetectorParams()
    node._published = []
    node._board_pose_pub.publish = node._published.append
    node._diagnostics = []
    node._diagnostics_pub.publish = node._diagnostics.append
    yield node
    node.destroy_node()


def cloud_of(scene, seed=1):
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=seed))
    return xyzi_cloud(scan.points, scan.intensity, FRAME, Time())


def last_diagnostic(node):
    node._publish_diagnostics()
    status = node._diagnostics[-1].status[0]
    return status, dict((kv.key, kv.value) for kv in status.values)


def test_ambiguous_batch_suppresses_its_pose_and_the_next_batch_still_runs(node):
    node._on_cloud(cloud_of(scenes.two_board_scene()))

    assert node._published == []
    status, values = last_diagnostic(node)
    assert status.level == DiagnosticStatus.WARN
    assert "ambiguous" in status.message
    assert values["candidates"] == "2"
    assert values["attempts"] == "1"

    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))

    assert len(node._published) == 1
    assert node._attempts == 2
    assert node._detections == 1
    status, values = last_diagnostic(node)
    assert status.level == DiagnosticStatus.OK
    assert values["confidence"] == values["confidence"]  # present
    assert float(values["confidence"]) >= float(values["min_confidence"])


def test_low_confidence_batch_is_suppressed_with_the_reason(node):
    node._min_confidence = 1.0

    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))

    assert node._published == []
    status, values = last_diagnostic(node)
    assert status.level == DiagnosticStatus.WARN
    assert "low confidence" in status.message
    assert "< 1.00" in status.message
    assert "confidence.edges" in values
    assert values["min_confidence"] == "1.00"

    node._min_confidence = 0.0
    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))
    assert len(node._published) == 1


def test_min_confidence_comes_from_the_detector_file():
    node = BoardDetectorNode()
    try:
        assert node._min_confidence == node._config.detector.min_confidence
    finally:
        node.destroy_node()
