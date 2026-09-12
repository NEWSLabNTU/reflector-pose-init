"""Debug marker/cloud construction, checked against real detector outcomes.

Uses the same synthetic scenes as the detector test matrix so each fixture is a
real OK / AMBIGUOUS / NO_CANDIDATE result rather than a hand-built stand-in.
"""

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

import numpy as np

from reflective_pose_ros.debug_viz import (
    board_outline_marker_array,
    board_pose_stamped,
    detection_points_cloud,
    rejection_marker_array,
    xyzi_cloud,
)
from reflective_pose_core.detector import DetectorParams, Status, detect_board
from reflective_pose_sim import scenes, vlp32_sim

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


# -- the detected board's outline ---------------------------------------------
#
# Two rectangles in the sensor frame, around the detected centre and in the
# board plane: `nominal` is the configured board, which is what the published
# pose is composed from; `measured` is what the sensor actually saw, drawn one
# edge per marker so an edge the detector did not observe can be told apart --
# a hidden edge is the one defect that biases the centre.

NOMINAL = (0.6, 0.6)


def _in_board_plane(detection, point):
    """(u, v) of a marker point: along `right` and `up` from the centre."""
    offset = np.array([point.x, point.y, point.z]) - detection.centre
    return float(offset @ detection.right), float(offset @ detection.up)


def _ok_detection(**scene_kwargs):
    scene, _ = scenes.board_scene(**scene_kwargs)
    result = run(scene)
    assert result.status is Status.OK
    return result.detection


def test_outline_starts_with_clear_all_and_uses_the_given_frame():
    detection = _ok_detection()
    markers = board_outline_marker_array(detection, NOMINAL, FRAME, STAMP).markers
    assert markers[0].action == Marker.DELETEALL
    assert all(m.header.frame_id == FRAME for m in markers[1:])


def test_nominal_outline_is_the_configured_board_around_the_detected_centre():
    detection = _ok_detection()
    markers = board_outline_marker_array(detection, NOMINAL, FRAME, STAMP).markers
    (nominal,) = [m for m in markers if m.ns == "nominal"]
    assert nominal.type == Marker.LINE_STRIP
    assert len(nominal.points) == 5, "a closed loop repeats its first corner"
    assert nominal.points[0] == nominal.points[-1]
    corners = sorted(
        (round(u, 6), round(v, 6))
        for u, v in (_in_board_plane(detection, p) for p in nominal.points[:4])
    )
    assert corners == [(-0.3, -0.3), (-0.3, 0.3), (0.3, -0.3), (0.3, 0.3)]


def test_measured_edges_sit_where_their_names_say():
    detection = _ok_detection()
    width, height = detection.extents
    markers = board_outline_marker_array(detection, NOMINAL, FRAME, STAMP).markers
    measured = {m.text: m for m in markers if m.ns == "measured"}
    assert set(measured) == {"left", "right", "bottom", "top"}
    for name, marker in measured.items():
        assert marker.type == Marker.LINE_LIST
        assert len(marker.points) == 2
        uv = [_in_board_plane(detection, p) for p in marker.points]
        if name == "left":
            assert all(abs(u + width / 2) < 1e-6 for u, _ in uv), uv
        elif name == "right":
            assert all(abs(u - width / 2) < 1e-6 for u, _ in uv), uv
        elif name == "bottom":
            assert all(abs(v + height / 2) < 1e-6 for _, v in uv), uv
        else:
            assert all(abs(v - height / 2) < 1e-6 for _, v in uv), uv


def test_a_clean_board_draws_every_measured_edge_green():
    detection = _ok_detection()
    assert all(detection.observed_edges.values())
    markers = board_outline_marker_array(detection, NOMINAL, FRAME, STAMP).markers
    for marker in (m for m in markers if m.ns == "measured"):
        assert (marker.color.r, marker.color.g) == (0.0, 1.0), marker.text


def test_an_edge_the_detector_did_not_observe_is_drawn_red():
    # The confidence tests' partial view: a third of the board hidden.
    detection = _ok_detection(range_m=6.0, occlusion=("horizontal", 0.3, "low"))
    hidden = {name for name, seen in detection.observed_edges.items() if not seen}
    assert hidden, "the scene must hide at least one edge for this to prove anything"
    markers = board_outline_marker_array(detection, NOMINAL, FRAME, STAMP).markers
    for marker in (m for m in markers if m.ns == "measured"):
        expected = (1.0, 0.0) if marker.text in hidden else (0.0, 1.0)
        assert (marker.color.r, marker.color.g) == expected, marker.text
