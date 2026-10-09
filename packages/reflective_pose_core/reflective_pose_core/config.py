"""Load the detector configuration without importing ROS.

The detector file has three sections, read by the runtime node and the offline
anchoring tool:

``board:``      the board's map pose and face dimensions
``detector:``   shared sensor settings plus independent runtime/map policies
``covariance:`` the covariance attached to a published guess

Runtime and map clouds deliberately do not share a flat set of gates. Runtime
points are one accumulated sensor view and their coordinates are relative to
``base_link``; map points are a merged cloud that is levelled and translated so
the fitted floor is z=0. Both policies describe absolute heights above the
physical ground. The runtime policy carries the base-link-to-ground offset
needed to convert its TF coordinates; the map policy owns its spatial AABB,
including the only map Z filter. The scalar offset assumes base-link z is
gravity-aligned; a tilted vehicle needs a gravity-aligned height frame.
"""

from dataclasses import dataclass, field, fields, replace
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from .anchor import AnchorParams
from .detector import Aabb, DetectorParams
from .geometry import CovarianceParams
from .sensors import DEFAULT_SENSOR, SensorModel, sensor_model

CONFIG_ENV_VAR = "REFLECTIVE_POSE_CONFIG"
CONFIG_BASENAME = "detector.yaml"
SECTIONS = ("board", "detector", "covariance")

# Sections from the old all-in-one layout. They now live with their consumer;
# naming the destination makes an accidental old file actionable.
MOVED_SECTIONS = {
    "ros": "ROS parameters of board_detector_node (board_detector.param.yaml)",
    "autoware": "ROS parameters of board_pose_initializer (board_pose_initializer.param.yaml)",
    "anchor": "flags of anchor-map-to-board (--floor-band, --max-floor-tilt-deg, ...)",
}

FLOOR_FIT_KEYS = tuple(
    name for name in (item.name for item in fields(AnchorParams))
    if not name.startswith("board_")
)
_MISSING = object()


@dataclass
class BoardParams:
    """The board geometry and authoritative placement shared by both modes."""

    pose_in_map: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 1.300, 0.0, 0.0, 0.0
    )
    width: float = 0.6
    height: float = 0.6


@dataclass
class DetectionPolicy:
    """Gates shared in shape by the runtime and map policies."""

    range_min: float = 3.0
    range_max: float = 18.0
    cluster_tolerance: float = 0.15
    cluster_min_points: int = 60
    board_centre_height: float = 1.3
    extent_tolerance: Tuple[float, float] = (0.8, 1.5)
    # See DetectorParams.extent_sampling_slack. Off keeps the measured gates.
    extent_sampling_slack: bool = False
    planarity_max_thickness: float = 0.08
    verticality_max_dot: float = 0.25
    centre_height_tolerance: float = 0.30
    density_max_ratio: float = 1.4
    density_check_enabled: bool = True


@dataclass
class RuntimeDetectionPolicy(DetectionPolicy):
    """Gates for a sensor cloud, with heights expressed above ground."""

    height_min: float = 0.5
    height_max: float = 1.65
    # ``base_link <- sensor`` remains the real TF transform. This scalar is
    # added only to height measurements; changing the transform would also
    # change range, viewpoint, density, and the published pose.
    base_link_height_above_ground: float = 0.265


def _default_map_policy() -> "MapDetectionPolicy":
    """Defaults for a levelled merged map; the YAML must still provide AABB."""
    return MapDetectionPolicy(
        range_min=0.0,
        range_max=float("inf"),
        density_check_enabled=False,
        aabb=None,
    )


@dataclass
class MapDetectionPolicy(DetectionPolicy):
    """Gates for a floor-zero map.

    ``height_min`` and ``height_max`` are intentionally absent. The map AABB
    is the sole point-height filter, so an accidental second Z band cannot
    silently disagree with it.
    """

    range_min: float = 0.0
    range_max: float = float("inf")
    density_check_enabled: bool = False
    aabb: Optional[Aabb] = None


@dataclass
class DetectorConfig:
    """Shared detector settings plus independent runtime and map policies."""

    intensity_threshold: float = 110.0
    # The beam model, by name (reflective_pose_core.sensors). It supplies the
    # elevation table, the sensor's up axis, and the two step sizes below.
    sensor: str = DEFAULT_SENSOR
    # Overrides of the sensor model's own steps; None takes the model's.
    azimuth_step_rad: Optional[float] = None
    mean_elevation_step_rad: Optional[float] = None
    min_confidence: float = 0.6
    runtime: RuntimeDetectionPolicy = field(default_factory=RuntimeDetectionPolicy)
    map_policy: MapDetectionPolicy = field(default_factory=_default_map_policy)

    @property
    def map(self) -> MapDetectionPolicy:
        """YAML-facing name for the offline policy."""
        return self.map_policy

    @property
    def map_aabb(self) -> Optional[Aabb]:
        """Compatibility view; the value is owned by ``detector.map.aabb``."""
        return self.map_policy.aabb

    @property
    def sensor_model(self) -> SensorModel:
        """The named beam model."""
        return sensor_model(self.sensor)

    def _resolve_common(
        self, policy: DetectionPolicy, board: BoardParams, scan_count: int
    ) -> Dict[str, Any]:
        model = self.sensor_model
        azimuth_step = (
            model.azimuth_step_rad if self.azimuth_step_rad is None
            else float(self.azimuth_step_rad)
        )
        elevation_step = (
            model.mean_elevation_step_rad if self.mean_elevation_step_rad is None
            else float(self.mean_elevation_step_rad)
        )
        return dict(
            intensity_threshold=self.intensity_threshold,
            range_min=policy.range_min,
            range_max=policy.range_max,
            cluster_tolerance=policy.cluster_tolerance,
            cluster_min_points=policy.cluster_min_points,
            board_width=board.width,
            board_height=board.height,
            board_centre_height=policy.board_centre_height,
            extent_tolerance=tuple(policy.extent_tolerance),
            extent_sampling_slack=bool(policy.extent_sampling_slack),
            planarity_max_thickness=policy.planarity_max_thickness,
            verticality_max_dot=policy.verticality_max_dot,
            centre_height_tolerance=policy.centre_height_tolerance,
            density_max_ratio=policy.density_max_ratio,
            density_check_enabled=policy.density_check_enabled,
            min_confidence=self.min_confidence,
            azimuth_step_rad=azimuth_step,
            mean_elevation_step_rad=elevation_step,
            elevation_table_rad=model.elevation_table_rad,
            sensor_up=tuple(float(value) for value in model.up_axis),
            scan_count=scan_count,
        )

    def params_for_runtime(self, board: BoardParams, scan_count: int) -> DetectorParams:
        """Resolve the runtime gates; all height values remain ground-relative."""
        values = self._resolve_common(self.runtime, board, int(scan_count))
        values.update(
            height_min=self.runtime.height_min,
            height_max=self.runtime.height_max,
        )
        return DetectorParams(**values)

    def params_for_map(self, board: BoardParams) -> DetectorParams:
        """Resolve the offline gates; AABB owns map point-height filtering."""
        values = self._resolve_common(self.map_policy, board, 1)
        values.update(height_min=-float("inf"), height_max=float("inf"))
        return DetectorParams(**values)


@dataclass
class Config:
    """Everything held in the detector file."""

    board: BoardParams = field(default_factory=BoardParams)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    covariance: CovarianceParams = field(default_factory=CovarianceParams)
    runtime_scan_count: int = 1

    @property
    def runtime_detector(self) -> DetectorParams:
        """Fully resolved runtime detector parameters."""
        return self.detector.params_for_runtime(self.board, self.runtime_scan_count)

    def runtime_detector_for_scan_count(self, scan_count: int) -> DetectorParams:
        """Resolve runtime parameters for a caller's accumulation count."""
        return self.detector.params_for_runtime(self.board, scan_count)

    @property
    def map_detector(self) -> DetectorParams:
        """Fully resolved map-anchoring detector parameters."""
        return self.detector.params_for_map(self.board)


def default_config_path() -> str:
    """Return the explicit override, packaged file, or checkout fallback."""
    override = _env_override()
    if override:
        return override

    packaged = Path(__file__).parent / "data" / CONFIG_BASENAME
    if packaged.is_file():
        return str(packaged)

    return str(Path(__file__).parents[3] / "config" / CONFIG_BASENAME)


def _env_override() -> Optional[str]:
    import os

    value = os.environ.get(CONFIG_ENV_VAR)
    return value or None


def load_config(path: Optional[str] = None, *, scan_count: Optional[int] = None) -> Config:
    """Read and validate the detector file.

    ``scan_count`` is supplied by the runtime node because only that caller
    knows how many scans it stacked. Map anchoring always resolves one scan.
    The map AABB is mandatory; use signed ``-.inf``/``.inf`` for open sides.
    YAML ``null`` is rejected so there is one spelling for an unbounded bound.
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

    board_values = _mapping(resolved, "board", document.get("board"))
    detector_values = _mapping(resolved, "detector", document.get("detector"))
    covariance_values = _mapping(resolved, "covariance", document.get("covariance"))

    _reject_unknown(resolved, "board", board_values, BoardParams)
    _reject_unknown(resolved, "covariance", covariance_values, CovarianceParams)
    board = _build_board(resolved, board_values)

    runtime_values = _mapping(
        resolved, "detector.runtime", detector_values.pop("runtime", None)
    )
    map_values = _mapping(resolved, "detector.map", detector_values.pop("map", None))
    common_names = {
        "intensity_threshold",
        "sensor",
        "azimuth_step_rad",
        "mean_elevation_step_rad",
        "min_confidence",
    }
    unknown_common = set(detector_values) - common_names
    if unknown_common:
        raise ValueError(
            f"{resolved}: unknown key(s) in 'detector': {sorted(unknown_common)}"
        )
    _reject_unknown(resolved, "detector.runtime", runtime_values, RuntimeDetectionPolicy)

    if "height_min" in map_values or "height_max" in map_values:
        duplicate = sorted(set(map_values) & {"height_min", "height_max"})
        raise ValueError(
            f"{resolved}: {duplicate} are runtime-only; use "
            "detector.map.aabb.min/max[z] for map Z bounds"
        )
    map_aabb_value = map_values.pop("aabb", _MISSING)
    _reject_unknown(resolved, "detector.map", map_values, MapDetectionPolicy)
    map_aabb = _parse_aabb(resolved, map_aabb_value)

    if "extent_tolerance" in runtime_values:
        runtime_values["extent_tolerance"] = tuple(runtime_values["extent_tolerance"])
    if "extent_tolerance" in map_values:
        map_values["extent_tolerance"] = tuple(map_values["extent_tolerance"])

    runtime = RuntimeDetectionPolicy(**_select(runtime_values, RuntimeDetectionPolicy))
    try:
        runtime_offset = float(runtime.base_link_height_above_ground)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{resolved}: detector.runtime.base_link_height_above_ground "
            f"must be a finite non-negative number ({error})"
        ) from error
    if not math.isfinite(runtime_offset) or runtime_offset < 0.0:
        raise ValueError(
            f"{resolved}: detector.runtime.base_link_height_above_ground "
            "must be a finite non-negative number"
        )
    runtime.base_link_height_above_ground = runtime_offset
    map_policy = replace(
        _default_map_policy(),
        **_select(map_values, MapDetectionPolicy),
        aabb=map_aabb,
    )
    detector = DetectorConfig(
        **_select(detector_values, DetectorConfig),
        runtime=runtime,
        map_policy=map_policy,
    )
    try:
        detector.sensor_model
    except ValueError as error:
        raise ValueError(f"{resolved}: detector.sensor: {error}") from error

    resolved_scan_count = 1 if scan_count is None else int(scan_count)
    if resolved_scan_count < 1:
        raise ValueError(f"{resolved}: scan_count must be at least one")
    # Resolve both objects at load time. Bad policy values should fail before a
    # node or CLI has started processing a cloud.
    detector.params_for_runtime(board, resolved_scan_count)
    detector.params_for_map(board)

    return Config(
        board=board,
        detector=detector,
        covariance=CovarianceParams(**_select(covariance_values, CovarianceParams)),
        runtime_scan_count=resolved_scan_count,
    )


def _mapping(path: str, section: str, value: Any) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{path}: '{section}' must be a mapping")
    return dict(value)


def _select(values: Dict[str, Any], target_type) -> Dict[str, Any]:
    names = {item.name for item in fields(target_type)}
    return {name: values[name] for name in names if name in values}


def _reject_unknown(path: str, section: str, values: Dict[str, Any], target_type) -> None:
    names = {item.name for item in fields(target_type)}
    unknown = set(values) - names
    if unknown:
        raise ValueError(f"{path}: unknown key(s) in '{section}': {sorted(unknown)}")


def _parse_aabb(path: str, value: Any) -> Aabb:
    """Parse the required map crop; YAML null is intentionally not accepted."""
    if value is _MISSING:
        raise ValueError(
            f"{path}: detector.map.aabb is required; use signed -.inf/.inf "
            "for unbounded axes"
        )
    if value is None:
        raise ValueError(
            f"{path}: detector.map.aabb cannot be null; use signed -.inf/.inf "
            "for unbounded axes"
        )
    if not isinstance(value, dict) or set(value) != {"min", "max"}:
        raise ValueError(
            f"{path}: detector.map.aabb must be a mapping with only 'min' and 'max'"
        )
    try:
        minimum_values = value["min"]
        maximum_values = value["max"]
        if any(item is None for item in list(minimum_values) + list(maximum_values)):
            raise ValueError("null AABB bounds are not allowed; use signed infinity")
        minimum = tuple(float(item) for item in minimum_values)
        maximum = tuple(float(item) for item in maximum_values)
        return Aabb(minimum, maximum)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"{path}: detector.map.aabb requires [x, y, z] bounds with finite "
            "values, -inf lower bounds, or inf upper bounds for unbounded axes, "
            f"and min < max on bounded axes ({error})"
        ) from error


def _build_board(path: str, values: Dict[str, Any]) -> BoardParams:
    pose = values.get("pose_in_map", BoardParams.pose_in_map)
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise ValueError(
            f"{path}: board.pose_in_map must be [x, y, z, roll, pitch, yaw] "
            "with angles in radians"
        )
    selected = _select(values, BoardParams)
    try:
        selected["pose_in_map"] = tuple(float(value) for value in pose)
        selected["width"] = float(selected.get("width", BoardParams.width))
        selected["height"] = float(selected.get("height", BoardParams.height))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path}: board values must be numeric ({error})") from error
    return BoardParams(**selected)


def anchor_params(config: Config, **floor_fit) -> AnchorParams:
    """Build floor-fit inputs while sharing board dimensions and map pose."""
    unknown = set(floor_fit) - set(FLOOR_FIT_KEYS)
    if unknown:
        raise TypeError(
            f"anchor_params(): {sorted(unknown)} are not floor-fit settings; "
            f"expected any of {list(FLOOR_FIT_KEYS)}"
        )
    return AnchorParams(
        board_width=config.board.width,
        board_height=config.board.height,
        board_pose_in_map=config.board.pose_in_map,
        **floor_fit,
    )
