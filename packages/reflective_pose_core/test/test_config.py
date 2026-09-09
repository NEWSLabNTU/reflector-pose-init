"""Shared YAML contract for offline anchoring and runtime initialization.

``test_config_equivalence.py`` checks that the ported file holds the same
numbers as the flat one it replaces. This file checks the loader's behaviour
around those numbers: that ``board:`` really is read once and fanned out to
every dataclass that carries a copy of it, that a malformed document is
rejected rather than silently defaulted, and that ``default_config_path``
resolves in the documented order.

The fan-out is the reason ``board:`` exists as a section. ``DetectorParams``
and ``AnchorParams`` each keep their own ``board_width``/``board_height``/
``board_centre_height``, and the runtime node and the anchoring tool must agree
on ``pose_in_map``. If the loader ever stopped fanning out, the two paths would
drift apart exactly as the flat file's comment warned, and no detection test
would notice.
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
  centre_height: 1.6

detector:
  intensity_threshold: 110.0
""",
        encoding="utf-8",
    )


def test_board_section_fans_out_to_anchor_and_detector(tmp_path):
    config_file = tmp_path / "reflective_pose.yaml"
    write_config(config_file, "[12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679]")

    config = load_config(str(config_file))

    assert config.board.pose_in_map == (12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679)
    assert config.anchor.board_pose_in_map == config.board.pose_in_map
    assert config.board.width == config.anchor.board_width == config.detector.board_width == 0.6
    assert config.board.height == config.anchor.board_height == config.detector.board_height == 0.6
    assert (
        config.board.centre_height
        == config.anchor.board_centre_height
        == config.detector.board_centre_height
        == 1.6
    )
    assert config.detector.intensity_threshold == 110.0


def test_omitted_sections_fall_back_to_the_dataclass_defaults(tmp_path):
    """A file may set only what it changes; the rest is the code's default."""
    config_file = tmp_path / "sparse.yaml"
    config_file.write_text("detector:\n  range_max: 25.0\n", encoding="utf-8")

    config = load_config(str(config_file))

    assert config.detector.range_max == 25.0
    assert config.ros.base_frame == "base_link"
    assert config.autoware.max_attempts == 5
    # Still fanned out, from the board defaults rather than from the file.
    assert config.detector.board_width == config.board.width


def test_scan_count_follows_accumulate_scans(tmp_path):
    """The coupling the flat file left to each caller to remember."""
    config_file = tmp_path / "accumulate.yaml"
    config_file.write_text("ros:\n  accumulate_scans: 3\n", encoding="utf-8")

    config = load_config(str(config_file))

    assert config.detector.scan_count == 3


@pytest.mark.parametrize(
    "document, message",
    [
        ("- not a mapping\n", "expected a mapping"),
        ("bord:\n  width: 0.6\n", "unknown section"),
        ("board:\n  widht: 0.6\n", "unknown key"),
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
    assert load_config().board.centre_height == 1.6


def test_default_config_path_falls_back_to_the_packaged_file(monkeypatch):
    """With no override, the canonical file ships inside the package."""
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    resolved = default_config_path()

    assert resolved.endswith("reflective_pose_core/data/reflective_pose.yaml")
    # It is a real file, and it loads: the packaged default must always work.
    assert load_config().detector.intensity_threshold == 240.0
