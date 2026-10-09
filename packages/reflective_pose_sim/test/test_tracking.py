"""Tracking on AutoSDV: the shipped VLP-16 and Robin-W profiles, one scan each.

The coach-board lab's geometry, not the golf cart's: a 0.6 m handheld board,
a LiDAR 0.30 m off the ground, ranges 1.5 to 10 m, a board that walks. Every
cloud is a single simulated scan from the named sensor model, fed through the
profile exactly as ``board_tracking_node`` loads it -- the TF puts the LiDAR at
base_link (AutoSDV's calibration has z = 0) and the profile's
``base_link_height_above_ground`` carries the mounting height.
"""

from pathlib import Path

import numpy as np
import pytest

from reflective_pose_core.config import load_config
from reflective_pose_core.detector import Status, _expected_point_count, detect_board
from reflective_pose_core.geometry import board_pose_in_sensor
from reflective_pose_core.sensors import sensor_model
from reflective_pose_sim import scenes, vlp32_sim

DATA = (
    Path(__file__).resolve().parents[2]
    / "reflective_pose_core" / "reflective_pose_core" / "data"
)
PROFILES = {"vlp16": "autosdv_vlp16", "robin_w": "autosdv_robin_w"}
POSITION_TOLERANCE = 0.10  # m, board centre in base_link

CHEST = scenes.HANDHELD_BOARD_CENTRE_HEIGHT  # 1.0 m
KNEE = 0.55


def profile(sensor):
    return load_config(str(DATA / f"{PROFILES[sensor]}.yaml"), scan_count=1)


def transform_base_cloud(sensor):
    """What AutoSDV's TF says: the LiDAR at base_link, axes per the driver."""
    return scenes.transform_base_sensor(0.0, sensor)


def truth_in_base(truth_intrinsic):
    """Ground truth, from the scene's intrinsic sensor frame to base_link."""
    base_from_intrinsic = scenes.transform_base_sensor(0.0)
    return base_from_intrinsic @ truth_intrinsic


def track_one(sensor, scene, seed=1):
    """One scan through the profile. Returns the result and the published pose."""
    config = profile(sensor)
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor=sensor, seed=seed))
    transform = transform_base_cloud(sensor)
    result = detect_board(
        scan.points,
        scan.intensity,
        transform,
        config.runtime_detector,
        height_offset=config.detector.runtime.base_link_height_above_ground,
    )
    accepted = (
        result.status is Status.OK
        and result.detection.confidence >= config.runtime_detector.min_confidence
    )
    pose = transform @ board_pose_in_sensor(result.detection) if accepted else None
    return result, pose


def describe(result):
    return [(r.reason, r.detail) for r in result.rejections]


# -- Robin-W: the whole range, at chest height -----------------------------------


@pytest.mark.parametrize("range_m", [1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0])
def test_robin_w_tracks_the_chest_height_board_across_the_range(range_m):
    scene, truth = scenes.tracking_board_scene(range_m=range_m, bearing_deg=10.0)
    result, pose = track_one("robin_w", scene)
    assert pose is not None, (result.status, describe(result))

    expected = truth_in_base(truth)
    assert np.linalg.norm(pose[:3, 3] - expected[:3, 3]) < POSITION_TOLERANCE
    # x of the board frame is its normal, pointing back at the car.
    assert np.dot(pose[:3, 0], expected[:3, 0]) > np.cos(np.radians(10.0))


@pytest.mark.parametrize("centre_height", [0.75, 1.35])
def test_robin_w_accepts_the_ends_of_the_handheld_band(centre_height):
    scene, _ = scenes.tracking_board_scene(range_m=3.0, centre_height=centre_height)
    result, pose = track_one("robin_w", scene)
    assert pose is not None, describe(result)


# -- VLP-16: where the geometry allows -------------------------------------------


@pytest.mark.parametrize("range_m", [1.5, 2.0, 3.0, 4.0, 6.0, 8.0])
def test_vlp16_tracks_a_knee_height_board_from_the_standoff_out(range_m):
    scene, truth = scenes.tracking_board_scene(range_m=range_m, centre_height=KNEE)
    result, pose = track_one("vlp16", scene)
    assert pose is not None, (result.status, describe(result))
    assert np.linalg.norm(pose[:3, 3] - truth_in_base(truth)[:3, 3]) < POSITION_TOLERANCE


@pytest.mark.parametrize("range_m", [4.0, 6.0, 8.0])
def test_vlp16_tracks_a_chest_height_board_once_it_is_in_the_fan(range_m):
    scene, truth = scenes.tracking_board_scene(range_m=range_m, centre_height=CHEST)
    result, pose = track_one("vlp16", scene)
    assert pose is not None, (result.status, describe(result))
    assert np.linalg.norm(pose[:3, 3] - truth_in_base(truth)[:3, 3]) < POSITION_TOLERANCE


def test_vlp16_cannot_see_a_chest_height_board_at_the_standoff():
    """+/-15 deg from 0.30 m reaches 0.84 m at 2 m; the board starts at 0.7 m.

    Not a detector defect: two rings cross the bottom 0.14 m of the board, and
    no gate should call that a board. The lab holds the board lower on a
    VLP-16 car. See docs/guides/tracking-mode.md.
    """
    scene, _ = scenes.tracking_board_scene(range_m=2.0, centre_height=CHEST)
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor="vlp16", seed=1))
    board = [i for i, r in enumerate(scene.rectangles) if r.name == "board"][0]
    assert len(np.unique(scan.ring[scan.source == board])) <= 2

    result, pose = track_one("vlp16", scene)
    assert pose is None


def test_vlp16_hires_sees_nothing_of_a_chest_height_board_at_the_standoff():
    """The Hi-Res fan (+/-10 deg) tops out at 0.65 m at 2 m."""
    scene, _ = scenes.tracking_board_scene(range_m=2.0, centre_height=CHEST)
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor="vlp16_hires", seed=1))
    board = [i for i, r in enumerate(scene.rectangles) if r.name == "board"][0]
    assert not np.any(scan.source == board)


# -- rejection -------------------------------------------------------------------


@pytest.mark.parametrize("sensor", ["vlp16", "robin_w"])
@pytest.mark.parametrize("seed", range(3))
def test_reflectors_that_are_not_the_board_publish_nothing(sensor, seed):
    """A vest, a post strip, a big sign, a tail reflector, floor tape."""
    scene = scenes.tracking_distractor_scene()
    config = profile(sensor)
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor=sensor, seed=seed))
    retro = scan.intensity >= config.runtime_detector.intensity_threshold
    assert retro.sum() > 50  # they all pass the intensity gate

    result, pose = track_one(sensor, scene, seed=seed)
    assert result.status is Status.NO_CANDIDATE
    assert pose is None
    assert len(result.rejections) >= 3


@pytest.mark.parametrize("sensor", ["vlp16", "robin_w"])
def test_the_board_is_picked_out_from_among_the_distractors(sensor):
    scene, truth = scenes.tracking_board_scene(
        range_m=3.0, centre_height=KNEE if sensor == "vlp16" else CHEST
    )
    result, pose = track_one(sensor, scene)
    assert result.status is Status.OK
    assert result.rejections, "the distractors should be seen and rejected"
    assert np.linalg.norm(pose[:3, 3] - truth_in_base(truth)[:3, 3]) < POSITION_TOLERANCE


# -- a walking board ---------------------------------------------------------------


@pytest.mark.parametrize(
    "sensor, centre_height, min_rate",
    [("robin_w", CHEST, 0.95), ("vlp16", KNEE, 0.9)],
)
def test_a_board_walked_away_and_swayed_is_tracked_scan_by_scan(sensor, centre_height, min_rate):
    """2 m to 5 m at 1 m/s, swaying +/-10 deg every 2 s, 10 Hz."""
    frames = scenes.walking_board_track(
        start_range_m=2.0, speed_mps=1.0, duration_s=3.0, sway_deg=10.0,
        centre_height=centre_height,
    )
    hits, errors, ranges = 0, [], []
    for index, (t, scene, truth) in enumerate(frames):
        _, pose = track_one(sensor, scene, seed=index)
        if pose is None:
            continue
        hits += 1
        expected = truth_in_base(truth)
        errors.append(np.linalg.norm(pose[:3, 3] - expected[:3, 3]))
        ranges.append((t, np.hypot(pose[0, 3], pose[1, 3])))

    assert hits >= min_rate * len(frames)
    assert max(errors) < POSITION_TOLERANCE
    # The published range follows the walk: about 1 m/s.
    times, measured = np.array(ranges).T
    slope = np.polyfit(times, measured, 1)[0]
    assert slope == pytest.approx(1.0, abs=0.1)


def test_a_board_walking_towards_the_car_is_tracked_down_to_the_standoff():
    frames = scenes.walking_board_track(
        start_range_m=5.0, speed_mps=-1.0, duration_s=3.0, centre_height=CHEST,
    )
    published = [track_one("robin_w", scene, seed=i)[1] for i, (_, scene, _) in enumerate(frames)]
    assert all(pose is not None for pose in published)
    assert np.hypot(*published[-1][:2, 3]) == pytest.approx(2.1, abs=0.1)


# -- the density model reads the sensor's own up axis -------------------------------


def test_density_model_measures_elevation_about_the_sensor_up_axis():
    """Robin-W clouds are in Seyond's axes, where the cloud's z is forward.

    Counting rows from the cloud z would put a board straight ahead 4 m *up*.
    """
    config = profile("robin_w")
    params = config.runtime_detector
    scene, truth = scenes.tracking_board_scene(range_m=4.0, with_distractors=False)
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor="robin_w", seed=1))
    board = [i for i, r in enumerate(scene.rectangles) if r.name == "board"][0]
    actual = int(np.sum(scan.source == board))

    centre_in_cloud = sensor_model("robin_w").cloud_rotation @ truth[:3, 3]
    height = float(np.dot(centre_in_cloud, params.sensor_up))
    expected = _expected_point_count(params, 4.0, height)
    assert expected == pytest.approx(actual, rel=0.3)

    wrong = _expected_point_count(params, 4.0, float(centre_in_cloud[2]))
    assert wrong is None or not (0.7 * actual < wrong < 1.3 * actual)
