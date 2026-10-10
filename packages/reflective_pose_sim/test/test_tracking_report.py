"""reflective_pose_core.tracking_report on simulated AutoSDV scans.

The report is what sets the profiles' unmeasured numbers from a field bag, so
it is checked here against scenes whose answers are known: the detection rate
per range, the board's centre height, and an intensity threshold that falls in
the simulator's gap between diffuse (<= 100) and retroreflective returns.
"""

from pathlib import Path

import numpy as np
import pytest

from reflective_pose_core.config import load_config
from reflective_pose_core.tracking_report import (
    TrackingReport, default_bins, percentile_from_hist)
from reflective_pose_sim import scenes, vlp32_sim

DATA = (
    Path(__file__).resolve().parents[2]
    / "reflective_pose_core" / "reflective_pose_core" / "data"
)
PROFILES = {"vlp16": "autosdv_vlp16", "robin_w": "autosdv_robin_w"}
KNEE, CHEST = 0.55, 1.0


def run(sensor, ranges, centre_height, scans=4, truth=True):
    config = load_config(str(DATA / f"{PROFILES[sensor]}.yaml"), scan_count=1)
    report = TrackingReport(config, scenes.transform_base_sensor(0.0, sensor))
    t = 0.0
    for r in ranges:
        for k in range(scans):
            scene, _ = scenes.tracking_board_scene(
                range_m=r, bearing_deg=5.0, centre_height=centre_height)
            scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor=sensor, seed=k))
            report.add_scan(t, scan.points, scan.intensity, truth_range=r if truth else None)
            t += 0.1
        t += 3.0
    return report, config


def rates(report, config):
    bins, _ = report.bins(default_bins(config.runtime_detector.range_max))
    return {round(0.5 * (b.lo + b.hi)): b for b in bins if b.scans}


def test_vlp16_knee_height_board_is_seen_at_every_lab_range():
    report, config = run("vlp16", [2.0, 4.0, 6.0, 8.0], KNEE)
    by_bin = rates(report, config)
    for r in (2, 4, 6, 8):
        assert by_bin[r].rate == 1.0, (r, by_bin[r].outcomes)
        assert abs(np.mean(by_bin[r].heights) - KNEE) < 0.10
        assert abs(np.mean(by_bin[r].ranges) - r) < 0.15


def test_vlp16_chest_height_board_is_missed_at_the_standoff():
    """The plan's risk: a VLP-16 0.30 m up cannot see a chest-height board at 2 m."""
    report, config = run("vlp16", [2.0, 4.0], CHEST)
    by_bin = rates(report, config)
    assert by_bin[2].rate == 0.0
    assert by_bin[4].rate == 1.0
    assert abs(np.mean(by_bin[4].heights) - CHEST) < 0.10


def test_robin_w_chest_height_board():
    report, config = run("robin_w", [2.0, 6.0], CHEST, scans=2)
    by_bin = rates(report, config)
    assert by_bin[2].rate == 1.0 and by_bin[6].rate == 1.0
    assert abs(np.mean(by_bin[2].heights + by_bin[6].heights) - CHEST) < 0.05


def test_threshold_falls_in_the_gap_between_diffuse_and_retro():
    report, _ = run("vlp16", [2.0, 4.0], KNEE, scans=3)
    th = report.suggest_threshold()
    # The simulator clips diffuse returns at 100 and retro returns at >= 101,
    # and a board at near-normal incidence reads above 200.
    assert 100.0 < th["suggested"] < 200.0
    assert th["suggested_board_kept"] >= 0.99
    assert percentile_from_hist(report.background_hist, 50.0) <= 100.0
    # What still passes is the scene's retroreflective distractors.
    assert 0.0 < th["suggested_background_passed"] < 0.05


def test_misses_without_truth_take_the_nearest_detection_or_none():
    report, config = run("vlp16", [2.0, 4.0], CHEST, scans=3, truth=False)
    bins, unassigned = report.bins(default_bins(config.runtime_detector.range_max))
    # Nothing at 2 m was ever detected, so its misses have no range to borrow.
    assert unassigned.scans == 3 and unassigned.detected == 0
    assert sum(b.detected for b in bins) == 3


def test_latency_and_format():
    report, config = run("vlp16", [4.0], KNEE, scans=2)
    lat = report.latency()
    assert lat["scans"] == 2 and lat["mean"] > 0.0
    text = report.format(default_bins(config.runtime_detector.range_max))
    assert "3.5-4.5 m" in text and "suggested" in text


@pytest.mark.parametrize("values,q,expected",
                         [([5, 5, 5, 9], 50.0, 5.0), ([1, 2, 3, 4], 100.0, 4.0)])
def test_percentile_from_hist(values, q, expected):
    hist = np.bincount(values, minlength=256)
    assert percentile_from_hist(hist, q) == expected
