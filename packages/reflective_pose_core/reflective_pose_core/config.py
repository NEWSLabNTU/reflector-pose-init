"""Load the canonical configuration, without importing ROS.

One YAML file, five sections, read by every package in this repository. The
previous layout was a flat ``/**: ros__parameters`` block that this module
parsed by hand precisely so it would not have to import ROS -- the shape was
ROS's, the reader was not. The sections here are the kinds of setting that were
already mixed in that block, separated by who consumes them.

Two couplings are enforced here rather than left to each caller, because both
have already been the subject of a comment warning about drift:

``board:`` is shared truth. ``DetectorParams`` and ``AnchorParams`` each carry
their own copy of the board's width and height, and the runtime node and the
offline anchoring tool must agree on the pose. They are written once under
``board:`` and fanned out from there. Height and gate thresholds live in
independent runtime and map policies because the two paths use different
height frames and point densities.

``ros.accumulate_scans`` feeds ``DetectorParams.scan_count``. The expected
return count scales with the number of stacked scans, so a node accumulating ten
against a detector assuming one rejects every real board as ten times too dense.
"""

from dataclasses import dataclass, field, fields, replace
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
    """The board contract shared by the runtime and offline paths.

    ``pose_in_map`` is the authoritative placement of the board centre. The
    detector's height gate is deliberately not part of this contract: runtime
    and map clouds use different height frames and therefore resolve their own
    expected centre height.
    """

    pose_in_map: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 1.300, 0.0, 0.0, 0.0
    )
    width: float = 0.6
    height: float = 0.97


@dataclass
class DetectionPolicy:
    """Mode-specific detector gates.

    The same board dimensions are injected into both resolved
    :class:`DetectorParams` objects. These values are intentionally separate:
    runtime points are filtered in ``base_link`` while map points are filtered
    after levelling against the fitted floor.
    """

    range_min: float = 3.0
    range_max: float = 18.0
    height_min: float = 0.4
    height_max: float = 1.8
    board_centre_height: Optional[float] = 1.075
    cluster_tolerance: float = 0.30
    cluster_min_points: int = 20
    extent_tolerance: Tuple[float, float] = (0.6, 1.2)
    planarity_max_thickness: float = 0.03
    verticality_max_dot: float = 0.25
    centre_height_tolerance: float = 0.30
    density_max_ratio: float = 1.4
    density_check_enabled: bool = True

    # Metadata for the frame contract. ``floor_height_in_frame`` is the z
    # coordinate of the physical floor in the height frame. It is used only
    # when board_centre_height is omitted and the shared map pose supplies the
    # board's height above that floor.
    height_reference: str = "base_link"
    floor_height_in_frame: float = 0.0


def _default_map_policy() -> DetectionPolicy:
    """Defaults for a merged map, whose origin is not a sensor."""
    return DetectionPolicy(
        range_min=0.0,
        range_max=float("inf"),
        height_reference="map_floor",
        density_check_enabled=False,
    )


@dataclass
class DetectorConfig:
    """Shared detector inputs plus independent runtime/map policies."""

    # Sensor contracts are common to both modes.
    intensity_threshold: float = 110.0
    azimuth_step_rad: float = 0.0035
    mean_elevation_step_rad: float = 0.0225
    runtime: DetectionPolicy = field(default_factory=DetectionPolicy)
    map_policy: DetectionPolicy = field(default_factory=_default_map_policy)

    @staticmethod
    def _resolve(
        policy: DetectionPolicy, board: BoardParams, scan_count: int,
        intensity_threshold: float, azimuth_step_rad: float,
        mean_elevation_step_rad: float, expected_height_reference: str,
    ) -> DetectorParams:
        centre_height = policy.board_centre_height
        if centre_height is None:
            # This derivation is valid because the map contract puts its
            # origin on the floor directly below the board. If a deployment
            # uses another vertical datum, it must set board_centre_height
            # explicitly instead of relying on this shortcut.
            # Add the floor's coordinate in the selected height frame
            # (negative wheel radius for base_link).
            centre_height = board.pose_in_map[2] + policy.floor_height_in_frame

        if policy.height_reference != expected_height_reference:
            raise ValueError(
                f"detector policy for {expected_height_reference} must use "
                f"height_reference={expected_height_reference!r}, got "
                f"{policy.height_reference!r}"
            )

        return DetectorParams(
            intensity_threshold=intensity_threshold,
            range_min=policy.range_min,
            range_max=policy.range_max,
            height_min=policy.height_min,
            height_max=policy.height_max,
            cluster_tolerance=policy.cluster_tolerance,
            cluster_min_points=policy.cluster_min_points,
            board_width=board.width,
            board_height=board.height,
            board_centre_height=float(centre_height),
            extent_tolerance=tuple(policy.extent_tolerance),
            planarity_max_thickness=policy.planarity_max_thickness,
            verticality_max_dot=policy.verticality_max_dot,
            centre_height_tolerance=policy.centre_height_tolerance,
            density_max_ratio=policy.density_max_ratio,
            density_check_enabled=policy.density_check_enabled,
            azimuth_step_rad=azimuth_step_rad,
            mean_elevation_step_rad=mean_elevation_step_rad,
            scan_count=scan_count,
            height_reference=policy.height_reference,
        )

    def params_for_runtime(self, board: BoardParams, scan_count: int) -> DetectorParams:
        """Resolve the policy whose heights are measured in ``base_link``."""
        return self._resolve(
            self.runtime,
            board,
            scan_count,
            self.intensity_threshold,
            self.azimuth_step_rad,
            self.mean_elevation_step_rad,
            "base_link",
        )

    def params_for_map(self, board: BoardParams) -> DetectorParams:
        """Resolve the policy whose heights are measured from the fitted floor."""
        return self._resolve(
            self.map_policy,
            board,
            1,
            self.intensity_threshold,
            self.azimuth_step_rad,
            self.mean_elevation_step_rad,
            "map_floor",
        )


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
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    anchor: AnchorParams = field(default_factory=AnchorParams)
    # Its own section rather than a corner of `autoware:`, because the
    # covariance is a property of the detection and is computed in
    # reflective_pose_ros -- which must not have to name an Autoware section to
    # find its own inputs.
    covariance: CovarianceParams = field(default_factory=CovarianceParams)
    ros: RosParams = field(default_factory=RosParams)
    autoware: AutowareParams = field(default_factory=AutowareParams)

    @property
    def runtime_detector(self) -> DetectorParams:
        """The fully resolved runtime detector parameters."""
        return self.detector.params_for_runtime(self.board, self.ros.accumulate_scans)

    @property
    def map_detector(self) -> DetectorParams:
        """The fully resolved map-anchoring detector parameters."""
        return self.detector.params_for_map(self.board)


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
    _reject_unknown(resolved, "anchor", anchor_values, AnchorParams)
    _reject_unknown(resolved, "covariance", covariance_values, CovarianceParams)

    runtime_values = dict(detector_values.pop("runtime", {}) or {})
    map_values = dict(detector_values.pop("map", {}) or {})
    common_names = {
        "intensity_threshold",
        "azimuth_step_rad",
        "mean_elevation_step_rad",
    }
    unknown_common = set(detector_values) - common_names
    if unknown_common:
        raise ValueError(
            f"{resolved}: unknown key(s) in 'detector': "
            f"{sorted(unknown_common)}"
        )
    _reject_unknown(resolved, "detector.runtime", runtime_values, DetectionPolicy)
    _reject_unknown(resolved, "detector.map", map_values, DetectionPolicy)

    board = _build_board(resolved, board_values)
    ros = RosParams(**_select(ros_values, RosParams))
    autoware = AutowareParams(**_select(autoware_values, AutowareParams))

    for values in (runtime_values, map_values):
        if "extent_tolerance" in values:
            values["extent_tolerance"] = tuple(values["extent_tolerance"])

    detector_kwargs = _select(detector_values, DetectorConfig)
    detector_kwargs["runtime"] = DetectionPolicy(
        **_select(runtime_values, DetectionPolicy)
    )
    detector_kwargs["map_policy"] = replace(
        _default_map_policy(), **_select(map_values, DetectionPolicy)
    )
    detector_config = DetectorConfig(**detector_kwargs)
    # Resolve both modes while loading so a frame mismatch fails at startup,
    # before either a ROS node or the CLI can run with a mislabeled height gate.
    detector_config.params_for_runtime(board, ros.accumulate_scans)
    detector_config.params_for_map(board)

    # board: fans out only the physical shape and authoritative placement.
    # Mode-specific height and gate settings stay in their own policies.
    anchor_values.update(
        board_width=board.width,
        board_height=board.height,
        board_pose_in_map=board.pose_in_map,
    )

    return Config(
        board=board,
        detector=detector_config,
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
