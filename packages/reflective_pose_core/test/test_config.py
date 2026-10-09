"""The detector file's shared board and split mode-policy contract."""

import pytest
import yaml

from reflective_pose_core.config import (
    CONFIG_ENV_VAR,
    BoardParams,
    anchor_params,
    default_config_path,
    load_config,
)
from reflective_pose_core.detector import Aabb, DetectorParams


def write_config(path, pose="[12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679]", aabb=None):
    if aabb is None:
        aabb = (("-.inf", "-.inf", 0.5), (".inf", ".inf", 1.65))
    minimum, maximum = aabb
    path.write_text(
        f"""board:
  pose_in_map: {pose}
  width: 0.6
  height: 0.6

detector:
  intensity_threshold: 110.0
  runtime:
    base_link_height_above_ground: 0.265
    board_centre_height: 1.6
    height_min: 0.5
    height_max: 1.65
  map:
    board_centre_height: 1.6
    aabb:
      min: [{', '.join(str(value) for value in minimum)}]
      max: [{', '.join(str(value) for value in maximum)}]
""",
        encoding="utf-8",
    )


def test_runtime_and_map_policies_are_resolved_independently(tmp_path):
    config_file = tmp_path / "detector.yaml"
    write_config(config_file)

    config = load_config(str(config_file), scan_count=3)
    runtime = config.runtime_detector
    mapped = config.map_detector

    assert config.board.pose_in_map[2] == 1.6
    assert runtime.board_width == mapped.board_width == config.board.width == 0.6
    assert runtime.board_height == mapped.board_height == config.board.height == 0.6
    assert runtime.board_centre_height == mapped.board_centre_height == 1.6
    assert runtime.height_min == 0.5
    assert runtime.height_max == 1.65
    assert mapped.height_min == float("-inf")
    assert mapped.height_max == float("inf")
    assert runtime.scan_count == 3
    assert mapped.scan_count == 1
    assert config.detector.runtime.base_link_height_above_ground == 0.265
    assert config.detector.map.aabb == Aabb(
        (None, None, 0.5), (None, None, 1.65)
    )


def test_runtime_and_map_gate_values_do_not_leak(tmp_path):
    config_file = tmp_path / "profiles.yaml"
    config_file.write_text(
        """board:
  pose_in_map: [0.0, 0.0, 1.3, 0.0, 0.0, 0.0]
  width: 0.6
  height: 0.97

detector:
  runtime:
    cluster_tolerance: 0.30
    cluster_min_points: 20
    planarity_max_thickness: 0.04
    board_centre_height: 1.035
  map:
    cluster_tolerance: 0.05
    cluster_min_points: 60
    planarity_max_thickness: 0.08
    board_centre_height: 1.0
    aabb:
      min: [-.inf, -.inf, 0.8]
      max: [.inf, .inf, 1.2]
""",
        encoding="utf-8",
    )

    config = load_config(str(config_file))
    runtime = config.runtime_detector
    mapped = config.map_detector

    assert runtime.cluster_tolerance == 0.30
    assert mapped.cluster_tolerance == 0.05
    assert runtime.cluster_min_points == 20
    assert mapped.cluster_min_points == 60
    assert runtime.planarity_max_thickness == 0.04
    assert mapped.planarity_max_thickness == 0.08
    assert runtime.board_centre_height == 1.035
    assert mapped.board_centre_height == 1.0
    assert config.detector.map_aabb == Aabb(
        (None, None, 0.8), (None, None, 1.2)
    )


@pytest.mark.parametrize("offset", ["-0.01", ".inf"])
def test_runtime_height_offset_must_be_finite_and_non_negative(tmp_path, offset):
    config_file = tmp_path / "invalid_offset.yaml"
    write_config(config_file)
    document = config_file.read_text(encoding="utf-8").replace(
        "base_link_height_above_ground: 0.265",
        f"base_link_height_above_ground: {offset}",
    )
    config_file.write_text(document, encoding="utf-8")

    with pytest.raises(ValueError, match="finite non-negative"):
        load_config(str(config_file))


def test_anchor_params_share_only_board_shape_and_pose(tmp_path):
    config_file = tmp_path / "detector.yaml"
    write_config(config_file)
    config = load_config(str(config_file))

    params = anchor_params(config, floor_band=0.5, max_floor_tilt_deg=2.0)

    assert params.board_pose_in_map == config.board.pose_in_map
    assert params.board_width == config.board.width
    assert params.board_height == config.board.height
    assert not hasattr(params, "board_centre_height")
    assert params.floor_band == 0.5
    assert params.max_floor_tilt_deg == 2.0
    with pytest.raises(TypeError):
        anchor_params(config, board_width=1.0)


def test_map_aabb_is_required(tmp_path):
    config_file = tmp_path / "missing_aabb.yaml"
    config_file.write_text("detector:\n  runtime:\n    height_min: 0.5\n", encoding="utf-8")

    with pytest.raises(ValueError, match="aabb is required"):
        load_config(str(config_file))


@pytest.mark.parametrize(
    "document, message",
    [
        ("detector:\n  map:\n    aabb: null\n", "cannot be null"),
        (
            "detector:\n  map:\n    aabb:\n      min: [null, -.inf, 0]\n      max: [.inf, .inf, 1]\n",
            "null AABB bounds",
        ),
        ("detector:\n  map:\n    height_min: 0.5\n", "runtime-only"),
        ("detector:\n  map_aabb: null\n", "unknown key"),
    ],
)
def test_load_config_rejects_ambiguous_or_legacy_map_settings(
    tmp_path, document, message
):
    config_file = tmp_path / "invalid.yaml"
    config_file.write_text(document, encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        load_config(str(config_file))


@pytest.mark.parametrize(
    "aabb, message",
    [
        ("{min: [0, 0, 0]}", "min.*max"),
        ("{min: [0, 0], max: [1, 1, 1]}", "three values"),
        ("{min: [.inf, 0, 0], max: [1, 1, 1]}", "lower AABB bounds"),
        ("{min: [-1, 0, 0], max: [-.inf, 1, 1]}", "upper AABB bounds"),
        ("{min: [1, 0, 0], max: [1, 1, 1]}", "min < max"),
    ],
)
def test_map_aabb_rejects_invalid_bounds(tmp_path, aabb, message):
    config_file = tmp_path / "invalid_aabb.yaml"
    config_file.write_text(
        f"detector:\n  map:\n    aabb: {aabb}\n", encoding="utf-8"
    )

    with pytest.raises(ValueError, match=message):
        load_config(str(config_file))


def test_flat_gate_keys_are_rejected_even_when_map_aabb_is_present(tmp_path):
    config_file = tmp_path / "sparse.yaml"
    config_file.write_text(
        """detector:
  range_max: 25.0
  map:
    aabb:
      min: [-.inf, -.inf, -.inf]
      max: [.inf, .inf, .inf]
""",
        encoding="utf-8",
    )

    # range_max is no longer a flat detector key: a typo/old layout fails.
    with pytest.raises(ValueError, match="unknown key"):
        load_config(str(config_file))


def test_scan_count_is_injected_by_the_runtime_caller(tmp_path):
    config_file = tmp_path / "scan_count.yaml"
    write_config(config_file)

    assert load_config(str(config_file), scan_count=3).runtime_detector.scan_count == 3
    assert load_config(str(config_file)).runtime_detector.scan_count == 1


@pytest.mark.parametrize(
    "section, moved_to",
    [
        ("ros", "board_detector.param.yaml"),
        ("autoware", "board_pose_initializer.param.yaml"),
        ("anchor", "anchor-map-to-board"),
    ],
)
def test_sections_that_moved_are_rejected_by_name(tmp_path, section, moved_to):
    config_file = tmp_path / "old_layout.yaml"
    config_file.write_text(f"{section}:\n  some_key: 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match=moved_to):
        load_config(str(config_file))


@pytest.mark.parametrize(
    "document, message",
    [
        ("- not a mapping\n", "expected a mapping"),
        ("bord:\n  width: 0.6\n", "unknown section"),
        ("board:\n  widht: 0.6\n", "unknown key"),
        ("board:\n  centre_height: 1.3\n", "unknown key"),
        ("detector:\n  height_reference: base_link\n", "unknown key"),
        (
            "detector:\n  runtime:\n    height_reference: map_floor\n",
            "unknown key",
        ),
        ("board:\n  pose_in_map: [0, 0, 1, 0, 0, 0, 1]\n", "pose_in_map must"),
        ("board:\n  pose_in_map: 1.0\n", "pose_in_map must"),
    ],
)
def test_load_config_rejects_a_malformed_document(tmp_path, document, message):
    config_file = tmp_path / "invalid.yaml"
    config_file.write_text(document, encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        load_config(str(config_file))


def test_default_config_path_prefers_the_environment_override(tmp_path, monkeypatch):
    override = tmp_path / "elsewhere.yaml"
    write_config(override, "[0.0, 0.0, 1.3, 0.0, 0.0, 0.0]")
    monkeypatch.setenv(CONFIG_ENV_VAR, str(override))

    assert default_config_path() == str(override)
    assert load_config().board.pose_in_map[2] == 1.3


def test_default_config_path_falls_back_to_the_packaged_file(monkeypatch):
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    resolved = default_config_path()

    assert resolved.endswith("reflective_pose_core/data/detector.yaml")
    assert load_config().detector.intensity_threshold == 100.0


def test_packaged_defaults_are_the_decided_values(monkeypatch):
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    config = load_config(scan_count=10)

    assert config.board.width == 0.6
    assert config.board.height == 0.6
    assert config.board.pose_in_map == (0.0, 0.0, 1.3, 0.0, 0.0, 0.0)
    assert config.detector.intensity_threshold == 100.0
    assert config.detector.runtime.base_link_height_above_ground == 0.265
    assert config.runtime_detector.board_centre_height == 1.3
    # The map policy was retuned for the sparse anchoring map in 8cb17c7.
    assert config.map_detector.board_centre_height == 1.1
    assert config.map_detector.height_min == float("-inf")
    assert config.map_detector.height_max == float("inf")


def test_board_dataclass_defaults_agree_with_the_packaged_file(monkeypatch):
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    assert BoardParams() == load_config().board


def test_min_confidence_is_a_shared_detector_key(tmp_path, monkeypatch):
    config_file = tmp_path / "confidence.yaml"
    config_file.write_text(
        """detector:
  min_confidence: 0.9
  map:
    aabb:
      min: [-.inf, -.inf, -.inf]
      max: [.inf, .inf, .inf]
""",
        encoding="utf-8",
    )

    assert load_config(str(config_file)).runtime_detector.min_confidence == 0.9

    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)
    document = yaml.safe_load(open(default_config_path(), encoding="utf-8"))
    assert "min_confidence" in document["detector"]
    assert load_config().runtime_detector.min_confidence == DetectorParams().min_confidence
