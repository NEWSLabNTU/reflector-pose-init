"""Load the detector file, without importing ROS.

One YAML, three sections, read by the two things that run the detector: the
ROS node and the offline anchoring tool.

``board:``      the board itself -- its map pose and its face dimensions
``detector:``   the gates ``detect_board`` applies
``covariance:`` the covariance a published guess carries

That is deliberately all. An earlier layout put the ROS wiring, the Autoware
handoff policy and the floor-fit knobs in the same file, so that one document
was read by three kinds of consumer and ``board:`` was fanned out to each of
them. Those sections now live where each consumer already keeps its settings:
the two nodes take ROS parameters, and ``anchor-map-to-board`` takes flags. A
file from that layout is rejected by name (see ``MOVED_SECTIONS``) rather than
partially read, because a section that is silently ignored looks exactly like
one whose values took effect.

Two couplings are still enforced here rather than left to each caller:

``board:`` is shared truth. ``DetectorParams`` carries its own copy of the
board's width, height and centre height, and ``anchor_params`` builds
``AnchorParams`` from the same numbers plus the pose. They are written once
under ``board:`` and fanned out from there.

``DetectorParams.scan_count`` follows however many scans the caller stacks. The
expected return count scales with it, so a node accumulating ten against a
detector assuming one rejects every real board as ten times too dense. The
node is the one thing that knows that number, so it passes it to ``load_config``.
"""

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from .anchor import AnchorParams
from .detector import DetectorParams
from .geometry import CovarianceParams

CONFIG_ENV_VAR = "REFLECTIVE_POSE_CONFIG"
CONFIG_BASENAME = "detector.yaml"

SECTIONS = ("board", "detector", "covariance")

#: Sections the six-section layout had, and where each one went. Named in the
#: error so the fix is in the message rather than in a changelog.
MOVED_SECTIONS = {
    "ros": "ROS parameters of board_detector_node (board_detector.param.yaml)",
    "autoware": "ROS parameters of board_pose_initializer (board_pose_initializer.param.yaml)",
    "anchor": "flags of anchor-map-to-board (--floor-band, --max-floor-tilt-deg, ...)",
}

#: The floor-fit knobs of ``AnchorParams``: everything that is not the board.
FLOOR_FIT_KEYS = tuple(
    f.name for f in fields(AnchorParams) if not f.name.startswith("board_")
)


@dataclass
class BoardParams:
    """The board itself. Shared by the runtime and offline paths."""

    # The decided board (2026-09-10): 0.6 x 0.6 m, centre 1.3 m above the
    # floor, at the map origin facing +x. Same numbers as the packaged file; a
    # test holds the two together.
    pose_in_map: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 1.300, 0.0, 0.0, 0.0
    )
    width: float = 0.6
    height: float = 0.6
    centre_height: float = 1.3


@dataclass
class Config:
    """Everything the detector file holds."""

    board: BoardParams = field(default_factory=BoardParams)
    detector: DetectorParams = field(default_factory=DetectorParams)
    # Its own section rather than a corner of the detector gates: the
    # covariance is a property of the detection, computed in
    # reflective_pose_ros, and it is tuned separately from what counts as a
    # detection at all.
    covariance: CovarianceParams = field(default_factory=CovarianceParams)


def default_config_path() -> str:
    """Where the default file is, in the order a caller should prefer.

    An explicit override first, then the installed package data, then the
    checkout. The checkout fallback is what lets the tests and a `pip install
    -e` both work without a share directory existing.
    """
    override = _env_override()
    if override:
        return override

    packaged = Path(__file__).parent / "data" / CONFIG_BASENAME
    if packaged.is_file():
        return str(packaged)

    # Repo checkout: packages/reflective_pose_core/reflective_pose_core/config.py
    return str(Path(__file__).parents[3] / "config" / CONFIG_BASENAME)


def _env_override() -> Optional[str]:
    import os

    value = os.environ.get(CONFIG_ENV_VAR)
    return value or None


def _select(values: Dict[str, Any], target_type) -> Dict[str, Any]:
    """Keys of ``values`` that ``target_type`` actually declares."""
    names = {f.name for f in fields(target_type)}
    return {name: values[name] for name in names if name in values}


def load_config(path: Optional[str] = None, *, scan_count: Optional[int] = None) -> Config:
    """Read the detector file into typed parameters.

    ``scan_count`` is how many scans the caller stacks before detecting; it
    sets ``DetectorParams.scan_count``. Left unset, the dataclass default (one
    scan) applies, which is right for the anchoring tool and wrong for a node
    that accumulates.

    Unknown keys are an error, not a shrug: a misspelled threshold that is
    silently ignored reads at runtime exactly like a threshold that had no
    effect, and this file's values are measurements.
    """
    resolved = path or default_config_path()
    with open(resolved, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}

    if not isinstance(document, dict):
        raise ValueError(f"{resolved}: expected a mapping at the top level")

    moved = [name for name in document if name in MOVED_SECTIONS]
    if moved:
        where = "; ".join(f"'{name}' is now {MOVED_SECTIONS[name]}" for name in moved)
        raise ValueError(
            f"{resolved}: section(s) {moved} no longer belong in the detector "
            f"file: {where}"
        )

    unknown_sections = set(document) - set(SECTIONS)
    if unknown_sections:
        raise ValueError(
            f"{resolved}: unknown section(s) {sorted(unknown_sections)}; "
            f"expected any of {list(SECTIONS)}"
        )

    board_values = dict(document.get("board") or {})
    detector_values = dict(document.get("detector") or {})
    covariance_values = dict(document.get("covariance") or {})

    # Every section, checked against the file's own keys before anything is
    # injected below -- otherwise the keys this function adds would excuse the
    # ones the author misspelled.
    _reject_unknown(resolved, "board", board_values, BoardParams)
    _reject_unknown(resolved, "detector", detector_values, DetectorParams)
    _reject_unknown(resolved, "covariance", covariance_values, CovarianceParams)

    board = _build_board(resolved, board_values)

    if "extent_tolerance" in detector_values:
        detector_values["extent_tolerance"] = tuple(detector_values["extent_tolerance"])

    # board: fans out. The detector carries its own copy of the geometry;
    # this is the single place that decides what that copy contains.
    detector_values.update(
        board_width=board.width,
        board_height=board.height,
        board_centre_height=board.centre_height,
    )
    if scan_count is not None:
        detector_values["scan_count"] = int(scan_count)

    # viewpoint and elevation_table_rad are runtime objects, not config: the
    # anchoring tool supplies a viewpoint, and the beam table comes from the
    # sensor calibration. Neither is settable from the file.
    for name in ("viewpoint", "elevation_table_rad"):
        detector_values.pop(name, None)

    return Config(
        board=board,
        detector=DetectorParams(**_select(detector_values, DetectorParams)),
        covariance=CovarianceParams(**_select(covariance_values, CovarianceParams)),
    )


def anchor_params(config: Config, **floor_fit) -> AnchorParams:
    """``AnchorParams`` for the anchoring tool: the file's board, the caller's floor fit.

    The board block is the same one the detector reads, so the map is anchored
    to exactly the geometry the runtime will look for. The floor-fit knobs are
    a property of one run of one tool and arrive as keyword overrides; a
    keyword that is not one of them is a ``TypeError``, so the board cannot be
    quietly overridden from the command line.
    """
    unknown = set(floor_fit) - set(FLOOR_FIT_KEYS)
    if unknown:
        raise TypeError(
            f"anchor_params(): {sorted(unknown)} are not floor-fit settings; "
            f"expected any of {list(FLOOR_FIT_KEYS)}"
        )
    return AnchorParams(
        board_width=config.board.width,
        board_height=config.board.height,
        board_centre_height=config.board.centre_height,
        board_pose_in_map=config.board.pose_in_map,
        **floor_fit,
    )


def _reject_unknown(path: str, section: str, values: Dict[str, Any], target_type) -> None:
    names = {f.name for f in fields(target_type)}
    unknown = set(values) - names
    if unknown:
        raise ValueError(
            f"{path}: unknown key(s) in '{section}': {sorted(unknown)}"
        )


def _build_board(path: str, values: Dict[str, Any]) -> BoardParams:
    pose = values.get("pose_in_map", BoardParams.pose_in_map)
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise ValueError(
            f"{path}: board.pose_in_map must be [x, y, z, roll, pitch, yaw] "
            "with angles in radians"
        )
    selected = _select(values, BoardParams)
    selected["pose_in_map"] = tuple(float(v) for v in pose)
    return BoardParams(**selected)
