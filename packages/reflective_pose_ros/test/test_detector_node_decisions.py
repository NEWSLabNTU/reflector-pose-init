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
    # The hand-built simulator scene uses the historical ground-level base_link
    # convention; the packaged runtime policy supplies its offset separately.
    node._runtime_height_offset = 0.0
    node._published = []
    node._board_pose_pub.publish = node._published.append
    node._diagnostics = []
    node._diagnostics_pub.publish = node._diagnostics.append
    node._outlines = []
    node._outline_pub.publish = node._outlines.append
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
        assert node._min_confidence == node._config.runtime_detector.min_confidence
    finally:
        node.destroy_node()


# -- the outline follows the verdict (golf-cart phase 7, A5) ------------------
#
# ~/debug/board_outline is latched, so whatever a batch leaves on it stays on
# screen. Every batch therefore publishes one: the outline when it detected,
# the clear-all alone when it did not, or a board from an earlier batch would
# sit in RViz looking like a current one.


def _namespaces(array):
    return {m.ns for m in array.markers if m.action != m.DELETEALL}


def test_a_detection_publishes_both_outlines(node):
    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))
    assert len(node._published) == 1
    assert node._outlines[-1].markers[0].action == node._outlines[-1].markers[0].DELETEALL
    assert _namespaces(node._outlines[-1]) == {"nominal", "measured"}


def test_a_batch_without_a_detection_clears_the_outline(node):
    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))
    node._on_cloud(cloud_of(scenes.distractor_only_scene()))
    assert len(node._outlines) == 2
    assert _namespaces(node._outlines[-1]) == set()


def test_an_ambiguous_batch_clears_the_outline(node):
    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))
    node._on_cloud(cloud_of(scenes.two_board_scene()))
    assert _namespaces(node._outlines[-1]) == set()


def test_a_low_confidence_detection_still_draws_its_outline(node):
    """Suppressed, but still a detection: the outline is how you see why."""
    node._min_confidence = 1.0
    node._on_cloud(cloud_of(scenes.board_scene(range_m=5.0)[0]))
    assert node._published == []
    assert _namespaces(node._outlines[-1]) == {"nominal", "measured"}


# -- the seed zone (golf-cart Phase 8, L6) -------------------------------------
#
# A detection that passes every shape gate is published only when its own
# range, bearing and aspect lie inside the detector file's seed_zone; outside,
# the batch is a WARN that names the bound it broke, and the next batch runs.

from reflective_pose_core.seed_zone import SeedZone  # noqa: E402

ZONE = SeedZone(range_min=4.0, range_max=17.0, bearing_max_deg=60.0, aspect_max_deg=20.0)


def test_a_detection_inside_the_zone_is_published_with_its_geometry(node):
    node._seed_zone = ZONE
    node._on_cloud(cloud_of(scenes.board_scene(range_m=6.0)[0]))

    assert len(node._published) == 1
    status, values = last_diagnostic(node)
    assert status.level == DiagnosticStatus.OK
    assert abs(float(values["bearing_deg"])) < 5.0
    assert abs(float(values["aspect_deg"])) < 5.0


def test_a_detection_seen_too_obliquely_is_not_published(node):
    node._seed_zone = ZONE
    node._on_cloud(cloud_of(scenes.board_scene(range_m=6.0, yaw_deg=35.0)[0]))

    assert node._published == []
    status, values = last_diagnostic(node)
    assert status.level == DiagnosticStatus.WARN
    assert "outside the seed zone" in status.message
    assert "aspect" in status.message
    assert abs(float(values["aspect_deg"])) > 20.0
    # Still a detection: the outline stays, as for low confidence.
    assert _namespaces(node._outlines[-1]) == {"nominal", "measured"}


def test_a_detection_closer_than_the_zone_is_not_published(node):
    node._seed_zone = ZONE
    node._on_cloud(cloud_of(scenes.board_scene(range_m=3.5)[0]))

    assert node._published == []
    status, _ = last_diagnostic(node)
    assert "range" in status.message


def test_a_detection_off_to_the_side_is_not_published(node):
    node._seed_zone = ZONE
    node._on_cloud(cloud_of(scenes.board_scene(range_m=6.0, bearing_deg=75.0)[0]))

    assert node._published == []
    status, _ = last_diagnostic(node)
    assert "bearing" in status.message


def test_without_a_zone_the_same_oblique_detection_is_published(node):
    node._on_cloud(cloud_of(scenes.board_scene(range_m=6.0, yaw_deg=35.0)[0]))

    assert len(node._published) == 1


def test_the_seed_zone_comes_from_the_detector_file():
    node = BoardDetectorNode()
    try:
        assert node._seed_zone == node._config.detector.seed_zone
    finally:
        node.destroy_node()
