"""Detector test matrix, driven entirely by synthetic VLP-32C scans.

No ROS, no hardware, no bag. Ground truth is the pose of the rectangle the
simulator actually rendered, so the assertions compare against the geometry
under test rather than a separately maintained constant.
"""

import numpy as np
import pytest

from golfcart_board_initializer.detector import DetectorParams, Status, detect_board
from golfcart_board_initializer.geometry import (
    covariance_from_detection,
    make_transform,
    map_pose_from_detection,
    pose_error,
    transform_inverse,
)
from golfcart_board_initializer.simulation import scenes, vlp32_sim

POSITION_TOLERANCE = 0.20  # m
YAW_TOLERANCE = np.radians(5.0)


def run(scene, seed=1, blooming=False, params=None):
    scan = vlp32_sim.simulate(
        scene, vlp32_sim.SimParams(seed=seed, blooming=blooming)
    )
    return detect_board(
        scan.points,
        scan.intensity,
        scenes.transform_base_sensor(),
        params or DetectorParams(),
    )


def board_pose_in_map():
    """Map origin on the floor below the board centre, map x along its normal."""
    return make_transform(np.eye(3), [0.0, 0.0, scenes.BOARD_CENTRE_HEIGHT])


def truth_vehicle_pose(truth_sensor_board):
    return (
        board_pose_in_map()
        @ transform_inverse(truth_sensor_board)
        @ transform_inverse(scenes.transform_base_sensor())
    )


def estimated_vehicle_pose(detection):
    return map_pose_from_detection(
        detection, scenes.transform_base_sensor(), board_pose_in_map()
    )


@pytest.mark.parametrize("range_m", [3.0, 5.0, 10.0, 15.0])
def test_range_sweep(range_m):
    scene, truth = scenes.board_scene(range_m=range_m)
    result = run(scene)
    assert result.status is Status.OK

    position, yaw = pose_error(
        estimated_vehicle_pose(result.detection), truth_vehicle_pose(truth)
    )
    assert position < POSITION_TOLERANCE
    assert yaw < YAW_TOLERANCE


@pytest.mark.parametrize("yaw_deg", [0.0, 30.0, -30.0, 60.0, -60.0])
def test_yaw_sweep(yaw_deg):
    scene, truth = scenes.board_scene(range_m=6.0, yaw_deg=yaw_deg)
    result = run(scene)
    assert result.status is Status.OK

    position, yaw = pose_error(
        estimated_vehicle_pose(result.detection), truth_vehicle_pose(truth)
    )
    assert position < POSITION_TOLERANCE
    assert yaw < YAW_TOLERANCE


def test_distractors_only_yields_no_detection():
    """Every object here passes the intensity gate. None may pass the rest."""
    result = run(scenes.distractor_only_scene())
    assert result.status is Status.NO_CANDIDATE
    assert result.detection is None


def test_two_boards_abort_as_ambiguous():
    """The map holds one board; a second survivor is a broken assumption."""
    result = run(scenes.two_board_scene())
    assert result.status is Status.AMBIGUOUS
    assert result.detection is None
    assert len(result.candidates) == 2


def test_tilted_board_is_not_rejected_by_the_verticality_gate():
    scene, truth = scenes.board_scene(range_m=6.0, tilt_deg=10.0)
    result = run(scene)
    assert result.status is Status.OK

    position, _ = pose_error(
        estimated_vehicle_pose(result.detection), truth_vehicle_pose(truth)
    )
    assert position < POSITION_TOLERANCE


def test_dirty_board_still_clears_the_intensity_threshold():
    """Reflectivity degraded until returns peak near 130, against a gate at 110."""
    scene, truth = scenes.board_scene(range_m=6.0, reflectivity=130.0 / 255.0)
    result = run(scene)
    assert result.status is Status.OK


def test_blooming_bias_stays_within_tolerance():
    """Retro halo plus inflated range: the failure mode most likely on hardware."""
    scene, truth = scenes.board_scene(range_m=6.0)
    result = run(scene, blooming=True)
    assert result.status is Status.OK

    position, yaw = pose_error(
        estimated_vehicle_pose(result.detection), truth_vehicle_pose(truth)
    )
    assert position < POSITION_TOLERANCE
    assert yaw < YAW_TOLERANCE


def test_horizontal_occlusion_flags_the_unconstrained_axis():
    """A one-sided view still detects, but must not claim a constrained centre."""
    scene, truth = scenes.board_scene(
        range_m=6.0, occlusion=("horizontal", 0.3, "low")
    )
    result = run(scene)
    assert result.status is Status.OK

    horizontal_ok, vertical_ok = result.detection.centre_constrained
    assert not horizontal_ok
    assert vertical_ok

    covariance = covariance_from_detection(result.detection)
    # The unconstrained axis must be inflated well past the nominal guess.
    assert covariance[0, 0] > covariance[2, 2]

    # A one-sided view cannot say which edge is missing — the bounding rectangle
    # is a lower bound on the board, so the centre is biased by up to half the
    # hidden width. The tight tolerance is therefore not claimed here; what is
    # claimed is that the bias is bounded and that the covariance advertises it.
    position, _ = pose_error(
        estimated_vehicle_pose(result.detection), truth_vehicle_pose(truth)
    )
    assert position < 0.40


def test_heavy_vertical_occlusion_rejects_cleanly():
    """Half the board hidden leaves an extent no gate should accept."""
    scene, _ = scenes.board_scene(range_m=6.0, occlusion=("vertical", 0.5, "low"))
    result = run(scene)
    assert result.status is Status.NO_CANDIDATE
    assert any(rejection.reason == "bad_height" for rejection in result.rejections)


def test_below_minimum_range_is_not_detected():
    """At 2 m the board falls into the 9.4 deg gap at the bottom of the fan."""
    scene, _ = scenes.board_scene(range_m=2.0)
    result = run(scene)
    assert result.status is Status.NO_CANDIDATE


def test_intensity_gate_alone_leaves_distractors():
    """Documents why geometric gating exists at all."""
    scene = scenes.distractor_only_scene()
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=1))
    retro = scan.intensity >= DetectorParams().intensity_threshold
    assert retro.sum() > 100  # plenty of retroreflective points, none a board


def test_detection_is_deterministic_across_repeat_runs():
    scene, _ = scenes.board_scene(range_m=6.0)
    first = run(scene, seed=7).detection
    second = run(scene, seed=7).detection
    assert np.allclose(first.centre, second.centre)


@pytest.mark.parametrize("seed", range(5))
def test_detection_rate_across_seeds(seed):
    scene, truth = scenes.board_scene(range_m=8.0, bearing_deg=15.0)
    result = run(scene, seed=seed)
    assert result.status is Status.OK

    position, yaw = pose_error(
        estimated_vehicle_pose(result.detection), truth_vehicle_pose(truth)
    )
    assert position < POSITION_TOLERANCE
    assert yaw < YAW_TOLERANCE
