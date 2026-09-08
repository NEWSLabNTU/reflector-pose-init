"""Debug marker/cloud construction, checked against real detector outcomes.

Uses the same synthetic scenes as the detector test matrix so each fixture is a
real OK / AMBIGUOUS / NO_CANDIDATE result rather than a hand-built stand-in.
"""

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from golfcart_board_initializer.debug_viz import (
    board_pose_stamped,
    detection_points_cloud,
    rejection_marker_array,
    xyzi_cloud,
)
from golfcart_board_initializer.detector import DetectorParams, Status, detect_board
from golfcart_board_initializer.simulation import scenes, vlp32_sim

STAMP = Time()
FRAME = "map_debug"


def run(scene, seed=1, params=None):
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=seed))
    return detect_board(
        scan.points, scan.intensity, scenes.transform_base_sensor(), params or DetectorParams()
    )


def test_rejection_markers_label_every_rejected_cluster():
    scene = scenes.distractor_only_scene()
    result = run(scene)
    assert result.status is Status.NO_CANDIDATE
    assert result.rejections

    markers = rejection_marker_array(result, FRAME, STAMP)

    # First marker is always the clear-all, so a stale detection from a
    # previous attempt cannot linger under a latched topic.
    assert markers.markers[0].action == Marker.DELETEALL
    rejected = [m for m in markers.markers if m.ns == "rejected"]
    assert len(rejected) == len(result.rejections)
    for marker, rejection in zip(rejected, result.rejections):
        assert rejection.reason in marker.text
        assert marker.header.frame_id == FRAME
        assert marker.color.r == 1.0 and marker.color.g == 0.6


def test_ok_result_draws_one_green_arrow_and_no_rejection_left_unlabelled():
    scene, _ = scenes.board_scene()
    result = run(scene)
    assert result.status is Status.OK

    markers = rejection_marker_array(result, FRAME, STAMP)
    arrows = [m for m in markers.markers if m.ns == "detection"]
    assert len(arrows) == 1
    assert arrows[0].type == Marker.ARROW
    assert arrows[0].color.g == 1.0

    rejected = [m for m in markers.markers if m.ns == "rejected"]
    assert len(rejected) == len(result.rejections)
    assert not [m for m in markers.markers if m.ns == "candidate"]


def test_ambiguous_result_labels_every_surviving_candidate():
    scene = scenes.two_board_scene()
    result = run(scene)
    assert result.status is Status.AMBIGUOUS
    assert len(result.candidates) >= 2

    markers = rejection_marker_array(result, FRAME, STAMP)
    candidates = [m for m in markers.markers if m.ns == "candidate"]
    assert len(candidates) == len(result.candidates)
    for marker in candidates:
        assert "AMBIGUOUS" in marker.text
        assert marker.color.r == 1.0 and marker.color.g == 0.0


def test_detection_points_cloud_matches_status():
    ok_scene, _ = scenes.board_scene()
    ok_result = run(ok_scene)
    ok_cloud = detection_points_cloud(ok_result, FRAME, STAMP)
    assert ok_cloud.width == ok_result.detection.n_points

    empty_result = run(scenes.distractor_only_scene())
    empty_cloud = detection_points_cloud(empty_result, FRAME, STAMP)
    assert empty_cloud.width == 0


def test_board_pose_stamped_uses_detection_centre():
    scene, _ = scenes.board_scene()
    result = run(scene)
    pose = board_pose_stamped(result.detection, FRAME, STAMP)
    assert pose.header.frame_id == FRAME
    assert pose.pose.position.x == float(result.detection.centre[0])


def test_xyzi_cloud_round_trips_point_count():
    scene, _ = scenes.board_scene()
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=1))
    cloud = xyzi_cloud(scan.points, scan.intensity, FRAME, STAMP)
    assert cloud.width == len(scan.points)
    assert cloud.header.frame_id == FRAME
