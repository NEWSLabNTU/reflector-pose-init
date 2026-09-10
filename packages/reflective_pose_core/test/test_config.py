"""The detector file: one YAML, read by the detector node and the anchoring tool.

Three sections -- ``board``, ``detector``, ``covariance`` -- and nothing else.
The file used to carry the ROS wiring, the Autoware handoff policy and the
floor-fit knobs too, so that one document was read by three kinds of consumer;
those moved to where each consumer already keeps its settings (ROS parameters
and CLI flags). A file from that layout must fail loudly and say where each
section went, because a section that is silently ignored reads exactly like
one whose values took effect.

``board:`` is still fanned out: ``DetectorParams`` carries its own copy of the
board geometry, and ``anchor_params`` builds ``AnchorParams`` from the same
numbers, so the runtime guess and the map it is checked against cannot
disagree.
"""

import pytest
import yaml

from reflective_pose_core.config import (
    CONFIG_ENV_VAR,
    BoardParams,
    anchor_params,
    default_config_path,
    load_config,
)
from reflective_pose_core.detector import DetectorParams


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


def test_board_section_fans_out_to_the_detector(tmp_path):
    config_file = tmp_path / "detector.yaml"
    write_config(config_file, "[12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679]")

    config = load_config(str(config_file))

    assert config.board.pose_in_map == (12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679)
    assert config.board.width == config.detector.board_width == 0.6
    assert config.board.height == config.detector.board_height == 0.6
    assert config.board.centre_height == config.detector.board_centre_height == 1.6
    assert config.detector.intensity_threshold == 110.0


def test_anchor_params_share_the_board_and_take_floor_fit_overrides(tmp_path):
    """The anchoring tool's inputs: the file's board, the caller's floor fit."""
    config_file = tmp_path / "detector.yaml"
    write_config(config_file, "[12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679]")
    config = load_config(str(config_file))

    params = anchor_params(config, floor_band=0.5, max_floor_tilt_deg=2.0)

    assert params.board_pose_in_map == config.board.pose_in_map
    assert params.board_width == config.board.width
    assert params.board_height == config.board.height
    assert params.board_centre_height == config.board.centre_height
    assert params.floor_band == 0.5
    assert params.max_floor_tilt_deg == 2.0
    # An override that is not a floor-fit knob is a mistake, not a feature.
    with pytest.raises(TypeError):
        anchor_params(config, board_width=1.0)


def test_omitted_sections_fall_back_to_the_dataclass_defaults(tmp_path):
    """A file may set only what it changes; the rest is the code's default."""
    config_file = tmp_path / "sparse.yaml"
    config_file.write_text("detector:\n  range_max: 25.0\n", encoding="utf-8")

    config = load_config(str(config_file))

    assert config.detector.range_max == 25.0
    assert config.covariance.sigma_z == 0.10
    # Still fanned out, from the board defaults rather than from the file.
    assert config.detector.board_width == config.board.width


def test_scan_count_is_injected_by_the_caller(tmp_path):
    """The expected return count scales with how many scans the node stacks.

    The node is the one thing that knows that number, so it passes it in; the
    file no longer carries a ``ros:`` section to read it from.
    """
    config_file = tmp_path / "detector.yaml"
    config_file.write_text("detector:\n  range_max: 25.0\n", encoding="utf-8")

    assert load_config(str(config_file), scan_count=3).detector.scan_count == 3
    assert load_config(str(config_file)).detector.scan_count == 1


@pytest.mark.parametrize(
    "section, moved_to",
    [
        ("ros", "board_detector.param.yaml"),
        ("autoware", "board_pose_initializer.param.yaml"),
        ("anchor", "anchor-map-to-board"),
    ],
)
def test_sections_that_moved_are_rejected_by_name(tmp_path, section, moved_to):
    """A file from the six-section layout must say where each section went."""
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
        ("detector:\n  intensity_thresold: 240.0\n", "unknown key"),
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
    """With no override, the default file ships inside the package."""
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    resolved = default_config_path()

    assert resolved.endswith("reflective_pose_core/data/detector.yaml")
    # It is a real file, and it loads: the packaged default must always work.
    assert load_config().detector.intensity_threshold == 100.0


def test_packaged_defaults_are_the_decided_values(monkeypatch):
    """The 2026-09-10 decisions: 0.6 x 0.6 m board, centre 1.3 m, threshold 100.

    100 is the bottom of the VLP-32C's datasheet retroreflector band -- a
    sensor contract that is reachable, where the previous 240 was not: no
    scan in the replay bag cleared cluster_min_points at 240.
    """
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    config = load_config()

    assert config.board.width == 0.6
    assert config.board.height == 0.6
    assert config.board.centre_height == 1.3
    assert config.board.pose_in_map == (0.0, 0.0, 1.3, 0.0, 0.0, 0.0)
    assert config.detector.intensity_threshold == 100.0


def test_board_dataclass_defaults_agree_with_the_packaged_file(monkeypatch):
    """A file that omits ``board:`` must describe the same board as the packaged one."""
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)

    packaged = load_config().board

    assert BoardParams() == packaged


def test_min_confidence_is_a_detector_key(tmp_path, monkeypatch):
    """The gate on what gets published lives beside the gates on what counts."""
    config_file = tmp_path / "detector.yaml"
    config_file.write_text("detector:\n  min_confidence: 0.9\n", encoding="utf-8")
    assert load_config(str(config_file)).detector.min_confidence == 0.9

    # The packaged file names it explicitly, with its reasoning, rather than
    # inheriting the dataclass default silently.
    monkeypatch.delenv(CONFIG_ENV_VAR, raising=False)
    document = yaml.safe_load(open(default_config_path(), encoding="utf-8"))
    assert "min_confidence" in document["detector"]
    assert load_config().detector.min_confidence == DetectorParams().min_confidence
