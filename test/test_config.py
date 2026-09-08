"""Shared YAML contract for offline anchoring and runtime initialization."""

import pytest

from golfcart_board_initializer.config import load_anchor_config


def write_config(path, pose):
    path.write_text(
        f"""/**:
  ros__parameters:
    board_width: 0.6
    board_height: 0.6
    board_centre_height: 1.6
    intensity_threshold: 110.0
    board_pose_in_map: {pose}
"""
    )


def test_load_anchor_config_uses_same_board_pose_and_geometry(tmp_path):
    config = tmp_path / "board_initializer.param.yaml"
    write_config(config, "[12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679]")

    anchor, detector = load_anchor_config(str(config))

    assert anchor.board_pose_in_map == (12.0, -4.0, 1.6, 0.0, 0.0, 1.57079632679)
    assert anchor.board_width == detector.board_width == 0.6
    assert anchor.board_height == detector.board_height == 0.6
    assert anchor.board_centre_height == detector.board_centre_height == 1.6


@pytest.mark.parametrize(
    "document, message",
    [
        ("/**: {}\n", "ros__parameters"),
        (
            "/**:\n  ros__parameters:\n"
            "    board_pose_in_map: [0, 0, 1, 0, 0, 0, 1]\n",
            "board_pose_in_map must",
        ),
    ],
)
def test_load_anchor_config_rejects_invalid_pose_contract(tmp_path, document, message):
    config = tmp_path / "invalid.yaml"
    config.write_text(document)

    with pytest.raises(ValueError, match=message):
        load_anchor_config(str(config))
