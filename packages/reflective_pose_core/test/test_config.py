"""Shared YAML contract for offline anchoring and runtime initialization.

``test_config_equivalence.py`` checks that the ported file holds the same
numbers as the flat one it replaces. This file checks the loader's behaviour
around those numbers: that ``board:`` really is read once and fanned out to
every dataclass that carries a copy of it, that a malformed document is
rejected rather than silently defaulted, and that ``default_config_path``
resolves in the documented order.

The fan-out is the reason ``board:`` exists as a section. ``DetectorParams``
and ``AnchorParams`` each keep their own ``board_width``/``board_height``, and
the runtime node and the anchoring tool must agree on ``pose_in_map``. Runtime
and map policies must not share their height or geometry gates.
"""

import pytest

from reflective_pose_core.config import (
    CONFIG_ENV_VAR,
    default_config_path,
    load_config,
)


def write_config(path, pose):
    path.write_text(
        f"""board:
  pose_in_map: {pose}
  width: 0.6
  height: 0.6

detector:
  intensity_threshold: 110.0
  runtime:
    board_centre_height: null
    floor_height_in_frame: -0.265
  map:
    board_centre_height: 1.6
""",
        encoding="utf-8",
    )


def test_board_section_fans_out_to_anchor_and_detector(tmp_path):
    config_file = tmp_path / "reflective_pose.yaml"
    write_config(config_file, "[12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679]")

    config = load_config(str(config_file))

    assert config.board.pose_in_map == (12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679)
    assert config.anchor.board_pose_in_map == config.board.pose_in_map
    assert config.board.width == config.anchor.board_width == 0.6
    assert config.board.height == config.anchor.board_height == 0.6
    assert config.runtime_detector.board_width == config.map_detector.board_width == 0.6
    assert config.runtime_detector.board_height == config.map_detector.board_height == 0.6
    assert config.runtime_detector.board_centre_height == pytest.approx(1.335)
    assert config.map_detector.board_centre_height == 1.6
    assert config.detector.intensity_threshold == 110.0


def test_runtime_and_map_policies_do_not_share_gate_values(tmp_path):
    config_file = tmp_path / "profiles.yaml"
    config_file.write_text(
        """board:
  pose_in_map: [0.0, 0.0, 1.3, 0.0, 0.0, 0.0]
  width: 0.6
  height: 0.97

detector:
  intensity_threshold: 240.0
  runtime:
    floor_height_in_frame: -0.265
    board_centre_height: null
    height_min: 0.5
    height_max: 1.5
    cluster_tolerance: 0.30
    cluster_min_points: 20
    planarity_max_thickness: 0.04
  map:
    height_reference: map_floor
    board_centre_height: 1.0
    height_min: 0.8
    height_max: 1.2
    cluster_tolerance: 0.05
    cluster_min_points: 60
    planarity_max_thickness: 0.08
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
    assert runtime.board_centre_height == pytest.approx(1.035)
    assert mapped.board_centre_height == 1.0
    assert runtime.height_reference == "base_link"
    assert mapped.height_reference == "map_floor"
    assert runtime.board_width == mapped.board_width == config.board.width
    assert runtime.board_height == mapped.board_height == config.board.height
    assert config.anchor.board_pose_in_map == config.board.pose_in_map


def test_omitted_sections_fall_back_to_the_dataclass_defaults(tmp_path):
    """A file may set only what it changes; the rest is the code's default."""
    config_file = tmp_path / "sparse.yaml"
    config_file.write_text(
        "detector:\n  runtime:\n    range_max: 25.0\n", encoding="utf-8"
    )

    config = load_config(str(config_file))

    assert config.detector.runtime.range_max == 25.0
    assert config.ros.base_frame == "base_link"
    assert config.autoware.max_attempts == 5
    # Still fanned out, from the board defaults rather than from the file.
    assert config.runtime_detector.board_width == config.board.width


def test_scan_count_follows_accumulate_scans(tmp_path):
    """The coupling the flat file left to each caller to remember."""
    config_file = tmp_path / "accumulate.yaml"
    config_file.write_text("ros:\n  accumulate_scans: 3\n", encoding="utf-8")

    config = load_config(str(config_file))

    assert config.runtime_detector.scan_count == 3


@pytest.mark.parametrize(
    "document, message",
    [
        ("- not a mapping\n", "expected a mapping"),
        ("bord:\n  width: 0.6\n", "unknown section"),
        ("board:\n  widht: 0.6\n", "unknown key"),
        ("board:\n  pose_in_map: [0, 0, 1, 0, 0, 0, 1]\n", "pose_in_map must"),
        ("board:\n  pose_in_map: 1.0\n", "pose_in_map must"),
        (
            "detector:\n  runtime:\n    height_reference: map_floor\n",
            "height_reference",
        ),
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
    assert load_config().map_detector.board_centre_height == 1.6


def test_default_config_path_falls_back_to_the_packaged_file(monkeypatch):
    """With no override, the canonical file ships inside the package."""
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    resolved = default_config_path()

    assert resolved.endswith("reflective_pose_core/data/reflective_pose.yaml")
    # It is a real file, and it loads: the packaged default must always work.
    assert load_config().detector.intensity_threshold == 240.0
