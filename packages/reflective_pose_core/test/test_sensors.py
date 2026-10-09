"""Sensor models, how the detector file selects one, and the shared clustering.

The elevation table used to be the VLP-32C's, hard-coded. These pin the
tables that replaced it, the default that keeps the packaged initializer on
exactly the table it was measured with, and the vectorised clustering that
tracking runs on every scan.
"""

from pathlib import Path

import numpy as np
import pytest

from reflective_pose_core.config import load_config
from reflective_pose_core.detector import DetectorParams, cluster_voxel_grid
from reflective_pose_core.sensors import SENSOR_NAMES, sensor_model
from reflective_pose_core.vlp32 import elevation_table

DATA = Path(__file__).resolve().parents[1] / "reflective_pose_core" / "data"


def test_vlp16_is_the_nebula_table():
    model = sensor_model("vlp16")
    assert np.allclose(np.degrees(model.elevation_table_rad), np.arange(-15.0, 15.1, 2.0))
    assert model.mean_elevation_step_rad == pytest.approx(np.radians(2.0))
    assert model.azimuth_step_rad == pytest.approx(np.radians(0.2))
    assert model.exact


def test_vlp16_hires_is_the_puck_hi_res_table():
    table = np.degrees(sensor_model("vlp16_hires").elevation_table_rad)
    assert table[0] == pytest.approx(-10.0)
    assert table[-1] == pytest.approx(10.0)
    assert np.allclose(np.diff(table), 4.0 / 3.0, atol=0.01)


def test_vlp32c_is_the_table_the_detector_always_used():
    model = sensor_model("vlp32c")
    assert np.allclose(model.elevation_table_rad, elevation_table())
    assert np.allclose(DetectorParams().elevation_table_rad, elevation_table())
    # The packaged mean step was the VLP-32C fan's own.
    assert model.mean_elevation_step_rad == pytest.approx(0.0225, abs=2e-4)


def test_robin_w_is_a_marked_approximation_in_native_axes():
    model = sensor_model("robin_w")
    assert not model.exact
    assert len(model.elevation_rad) == 192
    assert np.degrees(model.elevation_table_rad[[0, -1]]) == pytest.approx([-35.0, 35.0])
    lo, hi = model.azimuth_fov_rad
    assert np.degrees(hi - lo) == pytest.approx(120.0)
    # Seyond native: x up, y right, z forward.
    assert np.allclose(model.up_axis, [1.0, 0.0, 0.0])
    assert np.allclose(model.cloud_rotation @ [1.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    assert np.allclose(model.cloud_rotation @ [0.0, 1.0, 0.0], [0.0, -1.0, 0.0])


def test_sensor_names_are_normalised_and_unknown_ones_refused():
    assert sensor_model("Robin-W") is sensor_model("robin_w")
    with pytest.raises(ValueError, match="vlp16"):
        sensor_model("hdl64")
    assert set(SENSOR_NAMES) == {"vlp32c", "vlp16", "vlp16_hires", "robin_w"}


def _write(path, detector_lines):
    path.write_text(
        "detector:\n"
        + detector_lines
        + "  map:\n    aabb:\n      min: [-.inf, -.inf, -.inf]\n"
        "      max: [.inf, .inf, .inf]\n",
        encoding="utf-8",
    )
    return str(path)


def test_the_detector_file_selects_the_sensor(tmp_path):
    config = load_config(_write(tmp_path / "d.yaml", "  sensor: vlp16\n"))
    params = config.runtime_detector
    assert config.detector.sensor == "vlp16"
    assert np.allclose(params.elevation_table_rad, sensor_model("vlp16").elevation_table_rad)
    assert params.mean_elevation_step_rad == pytest.approx(np.radians(2.0))
    assert params.azimuth_step_rad == pytest.approx(np.radians(0.2))
    assert params.sensor_up == (0.0, 0.0, 1.0)


def test_explicit_steps_override_the_model(tmp_path):
    path = _write(
        tmp_path / "d.yaml",
        "  sensor: robin_w\n  azimuth_step_rad: 0.004\n  mean_elevation_step_rad: 0.01\n",
    )
    params = load_config(path).runtime_detector
    assert params.azimuth_step_rad == 0.004
    assert params.mean_elevation_step_rad == 0.01
    assert params.sensor_up == (1.0, 0.0, 0.0)


def test_an_unknown_sensor_is_refused_with_the_file_named(tmp_path):
    path = _write(tmp_path / "d.yaml", "  sensor: hdl64\n")
    with pytest.raises(ValueError, match="detector.sensor"):
        load_config(path)


def test_a_file_without_a_sensor_keeps_the_vlp32c(tmp_path, monkeypatch):
    """The packaged initializer file names no sensor; nothing about it moves."""
    monkeypatch.delenv("REFLECTIVE_POSE_CONFIG", raising=False)
    params = load_config().runtime_detector
    assert np.allclose(params.elevation_table_rad, elevation_table())
    assert params.azimuth_step_rad == 0.0035
    assert params.mean_elevation_step_rad == pytest.approx(0.0225, abs=2e-4)
    assert params.sensor_up == (0.0, 0.0, 1.0)
    assert params.extent_sampling_slack is False


@pytest.mark.parametrize(
    "name, sensor", [("autosdv_vlp16", "vlp16"), ("autosdv_robin_w", "robin_w")]
)
def test_autosdv_profiles_load_as_single_scan_tracking_files(name, sensor):
    config = load_config(str(DATA / f"{name}.yaml"), scan_count=1)
    params = config.runtime_detector
    assert config.detector.sensor == sensor
    assert params.scan_count == 1
    assert params.board_width == params.board_height == 0.6
    assert params.range_min == 1.0
    assert params.range_max >= 10.0
    assert params.extent_sampling_slack is True
    # The handheld chest-height board sits inside the centre gate.
    low = params.board_centre_height - params.centre_height_tolerance
    high = params.board_centre_height + params.centre_height_tolerance
    assert low <= 0.7 + 1e-9 and high >= 1.4 - 1e-9


# -- clustering ----------------------------------------------------------------


def _flood_fill_reference(points, tolerance, min_points):
    """The pure-Python clustering this module shipped before tracking."""
    keys = np.floor(points / tolerance).astype(np.int64)
    voxels = {}
    for idx, key in enumerate(map(tuple, keys)):
        voxels.setdefault(key, []).append(idx)
    neighbours = [
        (dx, dy, dz)
        for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    unvisited = set(voxels)
    clusters = []
    while unvisited:
        seed = unvisited.pop()
        component, stack = [seed], [seed]
        while stack:
            cx, cy, cz = stack.pop()
            for dx, dy, dz in neighbours:
                nb = (cx + dx, cy + dy, cz + dz)
                if nb in unvisited:
                    unvisited.remove(nb)
                    component.append(nb)
                    stack.append(nb)
        indices = np.concatenate([np.asarray(voxels[v]) for v in component])
        if len(indices) >= min_points:
            clusters.append(indices)
    return clusters


def _as_sets(clusters):
    return sorted(tuple(sorted(int(i) for i in c)) for c in clusters)


@pytest.mark.parametrize("seed", range(6))
def test_vectorised_clustering_matches_the_flood_fill(seed):
    rng = np.random.default_rng(seed)
    blobs = [rng.normal(rng.uniform(-5, 5, 3), rng.uniform(0.05, 0.6), (rng.integers(5, 300), 3))
             for _ in range(rng.integers(1, 12))]
    # Long thin chains exercise many label-propagation rounds.
    blobs.append(np.column_stack((np.linspace(-8, 8, 400), np.zeros(400), np.full(400, 3.0))))
    points = np.vstack(blobs)
    for tolerance, min_points in ((0.15, 1), (0.3, 20), (0.05, 3)):
        got = cluster_voxel_grid(points, tolerance, min_points)
        want = _flood_fill_reference(points, tolerance, min_points)
        assert _as_sets(got) == _as_sets(want)
        sizes = [len(c) for c in got]
        assert sizes == sorted(sizes, reverse=True)
        assert all(np.all(np.diff(c) > 0) for c in got)


def test_clustering_edge_cases():
    assert cluster_voxel_grid(np.zeros((0, 3)), 0.2, 1) == []
    single = cluster_voxel_grid(np.array([[1.0, 2.0, 3.0]]), 0.2, 1)
    assert [c.tolist() for c in single] == [[0]]
    assert cluster_voxel_grid(np.array([[1.0, 2.0, 3.0]]), 0.2, 2) == []
    # Negative coordinates and voxels touching only at a corner.
    corner = np.array([[-0.01, -0.01, -0.01], [0.01, 0.01, 0.01], [0.5, 0.5, 0.5]])
    assert _as_sets(cluster_voxel_grid(corner, 0.1, 1)) == [(0, 1), (2,)]
