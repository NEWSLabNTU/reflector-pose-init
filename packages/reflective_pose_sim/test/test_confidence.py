"""The confidence scalar, checked against what the simulator renders.

One number in [0, 1] from what the detection already measures: plane residual,
extent error, point density, how many bounding edges were seen, and range. A
clean board at moderate range must score high; a board with a hidden edge, or
a cluster denser or sparser than the sensor model predicts, must score lower.
The gate that acts on it lives in the node; this is the number itself.
"""

import numpy as np
import pytest

from reflective_pose_core.detector import (
    CONFIDENCE_TERMS,
    DetectorParams,
    Status,
    confidence_terms,
    detect_board,
)
from reflective_pose_sim import scenes, vlp32_sim


def detect(points, intensity, params=None):
    return detect_board(
        points, intensity, scenes.transform_base_sensor(), params or DetectorParams()
    )


def run(scene, seed=1, dropout=None):
    sim_params = vlp32_sim.SimParams(seed=seed)
    if dropout is not None:
        sim_params.dropout = dropout
    scan = vlp32_sim.simulate(scene, sim_params)
    return detect(scan.points, scan.intensity)


def clean_result(range_m=5.0):
    scene, _ = scenes.board_scene(range_m=range_m)
    result = run(scene)
    assert result.status is Status.OK
    return result


def test_confidence_is_carried_on_the_detection():
    result = clean_result()
    assert 0.0 <= result.detection.confidence <= 1.0
    assert set(result.detection.confidence_terms) <= set(CONFIDENCE_TERMS)


def test_every_term_is_in_the_unit_interval():
    result = clean_result()
    terms = confidence_terms(result.detection, DetectorParams())
    assert terms == result.detection.confidence_terms
    for name, value in terms.items():
        assert 0.0 <= value <= 1.0, name


def test_clean_board_at_moderate_range_scores_high():
    assert clean_result(range_m=5.0).detection.confidence > 0.8


def test_occluded_edge_scores_lower_than_a_clean_view():
    scene, _ = scenes.board_scene(range_m=6.0, occlusion=("horizontal", 0.3, "low"))
    occluded = run(scene)
    assert occluded.status is Status.OK
    assert not occluded.detection.centre_constrained[0]

    clean = clean_result(range_m=6.0)
    assert occluded.detection.confidence < clean.detection.confidence - 0.1
    assert occluded.detection.confidence_terms["edges"] < clean.detection.confidence_terms["edges"]


def test_over_sparse_cluster_scores_lower_than_a_clean_view():
    """Half the returns dropped: the same board, measured by half the points."""
    scene, _ = scenes.board_scene(range_m=6.0)
    sparse = run(scene, dropout=0.5)
    assert sparse.status is Status.OK

    clean = clean_result(range_m=6.0)
    assert sparse.detection.confidence_terms["density"] < clean.detection.confidence_terms["density"]
    assert sparse.detection.confidence < clean.detection.confidence


def test_over_dense_cluster_scores_lower_than_a_clean_view():
    """A cluster a third denser than one scan should produce, inside the gate."""
    scene, _ = scenes.board_scene(range_m=6.0)
    first = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=1))
    second = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=2))
    rng = np.random.default_rng(0)
    extra = rng.random(len(second.points)) < 0.3
    points = np.vstack([first.points, second.points[extra]])
    intensity = np.concatenate([first.intensity, second.intensity[extra]])

    dense = detect(points, intensity)
    assert dense.status is Status.OK
    clean = detect(first.points, first.intensity)
    assert clean.status is Status.OK

    assert dense.detection.confidence_terms["density"] < clean.detection.confidence_terms["density"]
    assert dense.detection.confidence < clean.detection.confidence


def test_farther_board_scores_lower_on_range():
    near = clean_result(range_m=5.0)
    far = clean_result(range_m=15.0)
    assert far.detection.confidence_terms["range"] < near.detection.confidence_terms["range"]


@pytest.mark.parametrize("range_m", [3.0, 5.0, 10.0, 15.0])
def test_every_clean_range_clears_the_default_gate(range_m):
    """The packaged min_confidence must not reject a clean board anywhere in range."""
    result = clean_result(range_m=range_m)
    assert result.detection.confidence >= DetectorParams().min_confidence


def test_one_sided_occlusion_falls_below_the_default_gate():
    """What the gate is for: a hidden edge biases the centre, and that frame
    must not be published as a pose."""
    scene, _ = scenes.board_scene(range_m=6.0, occlusion=("horizontal", 0.3, "low"))
    result = run(scene)
    assert result.status is Status.OK
    assert result.detection.confidence < DetectorParams().min_confidence
