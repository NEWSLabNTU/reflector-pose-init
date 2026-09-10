"""What the node does with one detection result, checked against real results.

``judge`` is the whole per-result decision of ``board_detector_node``: publish
or not, what /diagnostics says, and at which level. It is a pure function so
that the three outcomes that matter -- ambiguous frames are suppressed but not
terminal, low-confidence frames are suppressed with the reason spelled out,
and a trusted frame is published -- can be pinned without a bag or a spin.

Fixtures come from the simulator scenes, so each is a real OK / AMBIGUOUS /
NO_CANDIDATE result rather than a hand-built stand-in.
"""

from diagnostic_msgs.msg import DiagnosticStatus

from reflective_pose_core.detector import DetectorParams, Status, detect_board
from reflective_pose_ros.decision import judge
from reflective_pose_sim import scenes, vlp32_sim


def run(scene, seed=1, params=None):
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=seed))
    return detect_board(
        scan.points, scan.intensity, scenes.transform_base_sensor(), params or DetectorParams()
    )


def values_of(verdict):
    return dict(verdict.values)


def test_ambiguous_frame_is_suppressed_but_not_terminal():
    """A second reflector in frame is expected in a basement, not exceptional."""
    result = run(scenes.two_board_scene())
    assert result.status is Status.AMBIGUOUS

    verdict = judge(result, min_confidence=0.0)

    assert not verdict.publish
    assert not verdict.terminal
    assert verdict.level == DiagnosticStatus.WARN
    assert "ambiguous" in verdict.message
    values = values_of(verdict)
    assert values["candidates"] == "2"
    assert "candidate_1" in values and "candidate_2" in values
    # The candidates' ranges and extents are what an operator needs to tell
    # the board from the thing that is not the board.
    for candidate in result.candidates:
        assert f"{candidate.range_m:.1f} m" in values["candidate_1"] + values["candidate_2"]
        assert f"{candidate.extents[0]:.2f}x{candidate.extents[1]:.2f}" in (
            values["candidate_1"] + values["candidate_2"]
        )


def test_no_candidate_frame_is_suppressed_with_its_reasons():
    result = run(scenes.distractor_only_scene())
    assert result.status is Status.NO_CANDIDATE

    verdict = judge(result, min_confidence=0.0)

    assert not verdict.publish
    assert not verdict.terminal
    assert verdict.level == DiagnosticStatus.WARN
    assert "no candidate" in verdict.message
    values = values_of(verdict)
    assert values["retro_points"] == str(result.n_after_gates)
    assert values["clusters"] == str(result.n_clusters)
    assert values["confidence"] == "0.00"
    for rejection in result.rejections:
        assert rejection.reason in verdict.message


def test_trusted_detection_is_published_at_ok():
    scene, _ = scenes.board_scene(range_m=5.0)
    result = run(scene)
    assert result.status is Status.OK

    verdict = judge(result, min_confidence=0.5)

    assert verdict.publish
    assert verdict.level == DiagnosticStatus.OK
    values = values_of(verdict)
    assert values["confidence"] == f"{result.detection.confidence:.2f}"
    assert values["range_m"] == f"{result.detection.range_m:.1f}"
    for name, value in result.detection.confidence_terms.items():
        assert values[f"confidence.{name}"] == f"{value:.2f}"


def test_low_confidence_detection_is_suppressed_and_says_why():
    scene, _ = scenes.board_scene(range_m=5.0)
    result = run(scene)
    assert result.status is Status.OK
    gate = result.detection.confidence + 0.05

    verdict = judge(result, min_confidence=gate)

    assert not verdict.publish
    assert not verdict.terminal
    assert verdict.level == DiagnosticStatus.WARN
    assert f"low confidence {result.detection.confidence:.2f} < {gate:.2f}" in verdict.message
    values = values_of(verdict)
    assert values["confidence"] == f"{result.detection.confidence:.2f}"
    assert values["min_confidence"] == f"{gate:.2f}"
    for name in result.detection.confidence_terms:
        assert f"confidence.{name}" in values


def test_ambiguous_frame_reports_each_candidates_confidence():
    result = run(scenes.two_board_scene())
    verdict = judge(result, min_confidence=0.0)
    values = values_of(verdict)
    for index, candidate in enumerate(result.candidates, start=1):
        assert f"conf {candidate.confidence:.2f}" in values[f"candidate_{index}"]
