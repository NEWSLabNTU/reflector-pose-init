"""The ported config must produce the same numbers as the file it replaces.

The values in reflective_pose.yaml are measurements -- the 101-255
retroreflector band, the 3 m minimum that follows from the VLP-32C's 9.36 degree
beam gap, the density ratio that is an upper bound only. A port that quietly
moved one of them would not fail any other test in this repository: detection
would still run, and it would simply be wrong in the field.

So this compares the new loader's output against the old flat
``/**: ros__parameters`` file, key by key, and is expected to be deleted only
when that old file is deleted.

It reads the legacy YAML directly rather than through the legacy loader: that
loader imports the modules this split moved, so it cannot run any more. Reading
the file is also the stricter check, since it compares what was written down
rather than what one particular reader made of it.
"""

from pathlib import Path

import pytest
import yaml

from reflective_pose_core.config import load_config

REPO_ROOT = Path(__file__).resolve().parents[3]
LEGACY = REPO_ROOT / "config" / "board_initializer.param.yaml"
CANONICAL = (
    REPO_ROOT
    / "packages"
    / "reflective_pose_core"
    / "reflective_pose_core"
    / "data"
    / "reflective_pose.yaml"
)


def _legacy_values():
    with open(LEGACY, encoding="utf-8") as handle:
        return yaml.safe_load(handle)["/**"]["ros__parameters"]


@pytest.mark.skipif(not LEGACY.is_file(), reason="legacy config already removed")
def test_every_legacy_key_survives_the_port():
    """No key may be dropped silently; each is either ported or listed here."""
    legacy = _legacy_values()
    config = load_config(str(CANONICAL))

    # Where each legacy key now lives. Nothing is allowed to be absent from
    # this map: a key that is neither ported nor deliberately dropped is a
    # setting someone can no longer reach.
    located = {
        "board_pose_in_map": config.board.pose_in_map,
        "board_width": config.board.width,
        "board_height": config.board.height,
        "input_topic": config.ros.input_topic,
        "sensor_frame": config.ros.sensor_frame,
        "base_frame": config.ros.base_frame,
        "accumulate_scans": config.ros.accumulate_scans,
        "initialize_service": config.autoware.initialize_service,
        "max_speed_for_init": config.autoware.max_speed_for_init,
        "max_attempts": config.autoware.max_attempts,
        "fallback_to_user_defined_pose": config.autoware.fallback_to_user_defined_pose,
        # Moved out of the legacy file's flat block into their own section:
        # the covariance is computed in reflective_pose_ros, which should not
        # have to read an "autoware" section to find its own inputs.
        "sigma_xy_base": config.covariance.sigma_xy_base,
        "sigma_xy_per_metre": config.covariance.sigma_xy_per_metre,
        "sigma_z": config.covariance.sigma_z,
        "sigma_yaw_base": config.covariance.sigma_yaw_base,
        "covariance_safety_factor": config.covariance.safety_factor,
        "intensity_threshold": config.detector.intensity_threshold,
        "range_min": config.detector.runtime.range_min,
        "range_max": config.detector.runtime.range_max,
        "height_min": config.detector.runtime.height_min,
        "height_max": config.detector.runtime.height_max,
        "cluster_tolerance": config.detector.runtime.cluster_tolerance,
        "cluster_min_points": config.detector.runtime.cluster_min_points,
        "extent_tolerance": config.detector.runtime.extent_tolerance,
        "planarity_max_thickness": config.detector.runtime.planarity_max_thickness,
        "verticality_max_dot": config.detector.runtime.verticality_max_dot,
        "centre_height_tolerance": config.detector.runtime.centre_height_tolerance,
        "density_max_ratio": config.detector.runtime.density_max_ratio,
        "density_check_enabled": config.detector.runtime.density_check_enabled,
        "azimuth_step_rad": config.detector.azimuth_step_rad,
    }

    missing = set(legacy) - set(located)
    assert not missing, f"legacy keys with no home in the new schema: {sorted(missing)}"

    for key, new_value in located.items():
        old_value = legacy[key]
        if isinstance(old_value, list):
            assert tuple(old_value) == tuple(new_value), key
        else:
            assert old_value == new_value, key


def test_scan_count_tracks_the_node_accumulation():
    """The coupling the old flat file left to the caller to remember."""
    config = load_config(str(CANONICAL))
    assert config.runtime_detector.scan_count == config.ros.accumulate_scans


def test_unknown_key_is_an_error(tmp_path):
    """A misspelled threshold must not read as a threshold that did nothing."""
    bad = tmp_path / "bad.yaml"
    bad.write_text("detector:\n  intensity_thresold: 240.0\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(str(bad))
