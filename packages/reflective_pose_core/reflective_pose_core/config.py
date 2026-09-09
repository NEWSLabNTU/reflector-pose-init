"""Load the canonical configuration, without importing ROS.

One YAML file, five sections, read by every package in this repository. The
previous layout was a flat ``/**: ros__parameters`` block that this module
parsed by hand precisely so it would not have to import ROS -- the shape was
ROS's, the reader was not. The sections here are the kinds of setting that were
already mixed in that block, separated by who consumes them.

Two couplings are enforced here rather than left to each caller, because both
have already been the subject of a comment warning about drift:

``board:`` is shared truth. ``DetectorParams`` and ``AnchorParams`` each carry
their own copy of the board's width, height and centre height, and the runtime
node and the offline anchoring tool must agree on the pose. They are written
once under ``board:`` and fanned out from there.

``ros.accumulate_scans`` feeds ``DetectorParams.scan_count``. The expected
return count scales with the number of stacked scans, so a node accumulating ten
against a detector assuming one rejects every real board as ten times too dense.
"""

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from .anchor import AnchorParams
from .detector import DetectorParams
from .geometry import CovarianceParams

CONFIG_ENV_VAR = "REFLECTIVE_POSE_CONFIG"
CONFIG_BASENAME = "reflective_pose.yaml"

SECTIONS = ("board", "detector", "anchor", "covariance", "ros", "autoware")


@dataclass
class BoardParams:
    """The board itself. Shared by the runtime and offline paths."""

    pose_in_map: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 1.300, 0.0, 0.0, 0.0
    )
    width: float = 0.6
    height: float = 0.97
    centre_height: float = 1.0


@dataclass
class RosParams:
    """Wiring for reflective_pose_ros. No algorithm lives here."""

    input_topic: str = "/sensing/lidar/top/pointcloud_raw_ex"
    sensor_frame: str = "velodyne"
    base_frame: str = "base_link"
    accumulate_scans: int = 10

    # Stacking scans assumes a stationary sensor: nothing deskews them, so a
    # batch taken while the vehicle rolls is smeared and the board's extents
    # measure wrong. The all-in-one node got this for free from the Autoware
    # velocity gate; with detection split from the Autoware handoff, the
    # detector needs its own source, in a message type that does not drag
    # Autoware into this package.
    #
    # Empty disables the guard, which is right on a bench and wrong on a
    # vehicle. `nav_msgs/Odometry` and `geometry_msgs/TwistStamped` are both
    # accepted; the node picks by the topic's advertised type.
    twist_topic: str = ""
    max_speed_for_accumulation: float = 0.05


@dataclass
class AutowareParams:
    """Handoff policy for reflective_pose_autoware."""

    initialize_service: str = "/localization/initialize"
    max_speed_for_init: float = 0.05
    max_attempts: int = 5
    fallback_to_user_defined_pose: bool = False

    # How long to wait for a board pose before the attempt budget is considered
    # spent. Zero disables it.
    #
    # Detection and the service call are two processes now. An attempt is one
    # pose acted on, so a detector that never detects spends no attempts and the
    # fallback policy never runs -- the all-in-one node counted those failures
    # because it owned the detector loop. This bounds that case.
    pose_wait_timeout: float = 0.0


@dataclass
class Config:
    """Everything the five packages read, from one file."""

    board: BoardParams = field(default_factory=BoardParams)
    detector: DetectorParams = field(default_factory=DetectorParams)
    anchor: AnchorParams = field(default_factory=AnchorParams)
    # Its own section rather than a corner of `autoware:`, because the
    # covariance is a property of the detection and is computed in
    # reflective_pose_ros -- which must not have to name an Autoware section to
    # find its own inputs.
    covariance: CovarianceParams = field(default_factory=CovarianceParams)
    ros: RosParams = field(default_factory=RosParams)
    autoware: AutowareParams = field(default_factory=AutowareParams)


def default_config_path() -> str:
    """Where the canonical file is, in the order a caller should prefer.

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


def load_config(path: Optional[str] = None) -> Config:
    """Read the canonical YAML into typed parameters.

    Unknown keys are an error, not a shrug: a misspelled threshold that is
    silently ignored reads at runtime exactly like a threshold that had no
    effect, and this file's values are measurements.
    """
    resolved = path or default_config_path()
    with open(resolved, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}

    if not isinstance(document, dict):
        raise ValueError(f"{resolved}: expected a mapping at the top level")

    unknown_sections = set(document) - set(SECTIONS)
    if unknown_sections:
        raise ValueError(
            f"{resolved}: unknown section(s) {sorted(unknown_sections)}; "
            f"expected any of {list(SECTIONS)}"
        )

    board_values = dict(document.get("board") or {})
    detector_values = dict(document.get("detector") or {})
    anchor_values = dict(document.get("anchor") or {})
    ros_values = dict(document.get("ros") or {})
    autoware_values = dict(document.get("autoware") or {})
    covariance_values = dict(document.get("covariance") or {})

    # Every section, checked against the file's own keys before anything is
    # injected below -- otherwise the keys this function adds would excuse the
    # ones the author misspelled.
    _reject_unknown(resolved, "board", board_values, BoardParams)
    _reject_unknown(resolved, "ros", ros_values, RosParams)
    _reject_unknown(resolved, "autoware", autoware_values, AutowareParams)
    _reject_unknown(resolved, "detector", detector_values, DetectorParams)
    _reject_unknown(resolved, "anchor", anchor_values, AnchorParams)
    _reject_unknown(resolved, "covariance", covariance_values, CovarianceParams)

    board = _build_board(resolved, board_values)
    ros = RosParams(**_select(ros_values, RosParams))
    autoware = AutowareParams(**_select(autoware_values, AutowareParams))

    if "extent_tolerance" in detector_values:
        detector_values["extent_tolerance"] = tuple(detector_values["extent_tolerance"])

    # board: fans out. Both dataclasses carry their own copy of the geometry;
    # this is the single place that decides what those copies contain.
    detector_values.update(
        board_width=board.width,
        board_height=board.height,
        board_centre_height=board.centre_height,
        # See the module docstring: this must track the node's accumulation.
        scan_count=ros.accumulate_scans,
    )
    anchor_values.update(
        board_width=board.width,
        board_height=board.height,
        board_centre_height=board.centre_height,
        board_pose_in_map=board.pose_in_map,
    )

    # viewpoint and elevation_table_rad are runtime objects, not config: the
    # anchoring tool supplies a viewpoint, and the beam table comes from the
    # sensor calibration. Neither is settable from the file.
    for name in ("viewpoint", "elevation_table_rad"):
        detector_values.pop(name, None)

    return Config(
        board=board,
        detector=DetectorParams(**_select(detector_values, DetectorParams)),
        anchor=AnchorParams(**_select(anchor_values, AnchorParams)),
        covariance=CovarianceParams(**_select(covariance_values, CovarianceParams)),
        ros=ros,
        autoware=autoware,
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
