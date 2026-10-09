"""Retroreflective board detection from a single accumulated LiDAR scan.

This module is deliberately free of ROS imports. It is shared between the
runtime initializer node and the offline map-anchoring step, and keeping it pure
numpy is what lets its tests run without ROS, without hardware, and without a
bag.

See docs/design/reflective_pose_detector.md for the package layout, and the
gates below for the algorithm.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

from .sensors import DEFAULT_SENSOR, sensor_model


def _default_elevation_table() -> np.ndarray:
    return sensor_model(DEFAULT_SENSOR).elevation_table_rad


class Status(Enum):
    """Outcome of a detection attempt."""

    OK = "ok"
    NO_CANDIDATE = "no_candidate"
    AMBIGUOUS = "ambiguous"


@dataclass(frozen=True)
class Aabb:
    """An axis-aligned box in a named point-cloud frame.

    ``None`` on one side of an axis means that side is unbounded. Signed
    infinities are accepted as input aliases (``-inf`` for an unbounded lower
    bound and ``inf`` for an unbounded upper bound) and normalized to ``None``.
    This lets a map use the same object for a full XY search with only a Z
    slab, without reintroducing separate map height bounds.
    """

    minimum: Tuple[Optional[float], Optional[float], Optional[float]]
    maximum: Tuple[Optional[float], Optional[float], Optional[float]]

    def __post_init__(self):
        def normalize(value, lower):
            if value is None:
                return None
            value = float(value)
            if np.isnan(value):
                raise ValueError("AABB bounds cannot contain NaN")
            if np.isneginf(value):
                if lower:
                    return None
                raise ValueError("upper AABB bounds cannot be -inf")
            if np.isposinf(value):
                if not lower:
                    return None
                raise ValueError("lower AABB bounds cannot be inf")
            return value

        try:
            minimum = tuple(
                normalize(value, lower=True) for value in self.minimum
            )
            maximum = tuple(
                normalize(value, lower=False) for value in self.maximum
            )
        except TypeError as error:
            raise ValueError("AABB bounds must each contain three values") from error
        if len(minimum) != 3 or len(maximum) != 3:
            raise ValueError("AABB bounds must each contain exactly three values")
        for lower, upper in zip(minimum, maximum):
            if lower is not None and upper is not None and lower >= upper:
                raise ValueError(
                    "AABB minimum must be less than maximum on every bounded axis"
                )
        object.__setattr__(self, "minimum", minimum)
        object.__setattr__(self, "maximum", maximum)

    def contains(self, points: np.ndarray) -> np.ndarray:
        """Return an inclusive mask for points inside this box."""
        points = np.asarray(points)
        keep = np.ones(len(points), dtype=bool)
        for axis, (lower, upper) in enumerate(zip(self.minimum, self.maximum)):
            if lower is not None:
                keep &= points[:, axis] >= lower
            if upper is not None:
                keep &= points[:, axis] <= upper
        return keep


@dataclass
class DetectorParams:
    """Detection thresholds.

    The defaults here are a starting point, not the deployed values: those live
    in detector.yaml and are loaded through reflective_pose_core.config.
    """

    # Stage 1 gates
    intensity_threshold: float = 110.0
    # 3 m is not arbitrary. Below it the board drops into the VLP-32C's sparse
    # lower elevation band, where the gap between the -25.0 deg and -15.6 deg
    # beams is 9.4 deg: the bottom of the board is sampled by a single ring that
    # then fails to connect to the rest, and the cluster loses its lower third.
    range_min: float = 3.0
    range_max: float = 18.0
    # Absolute heights above ground. ``detect_board`` adds its height_offset to
    # the transformed z value before applying these point and centre gates.
    height_min: float = 0.4
    height_max: float = 1.8

    # Stage 2 clustering
    cluster_tolerance: float = 0.30
    cluster_min_points: int = 20

    # Board geometry
    board_width: float = 0.8
    board_height: float = 1.0
    # Absolute expected board-centre height above ground.
    board_centre_height: float = 1.075

    # Stage 3 gates
    extent_tolerance: Tuple[float, float] = (0.6, 1.2)
    # Forgive the extent gate's lower bound the sampling loss: a measured
    # extent is the distance between the outermost samples, so it falls short
    # of the object by up to one sample spacing at *each* edge. Off by default,
    # which is the behaviour the packaged initializer was measured with; needed
    # on a sparse sensor, where a VLP-16's 2 deg rows are 0.23 m apart at
    # 6.5 m and a 0.6 m board can read 0.23 m tall.
    extent_sampling_slack: bool = False
    planarity_max_thickness: float = 0.03
    verticality_max_dot: float = 0.25
    centre_height_tolerance: float = 0.30
    # Density is checked as an upper bound only. Yaw, occlusion, and dropout all
    # legitimately reduce the return count; nothing legitimately inflates it, so
    # an excess means the cluster is not the object its extents claim.
    density_max_ratio: float = 1.4
    density_check_enabled: bool = True

    # The confidence gate. detect_board reports every survivor's confidence
    # and applies no threshold itself; the node refuses to publish a pose
    # below this. See confidence_terms for what goes into the number.
    min_confidence: float = 0.6

    # Sensor model. The elevation table drives both the density gate and the
    # edge-observation margins; without it the density gate is skipped, because
    # a mean-step approximation is wrong by 3x across the working range. The
    # defaults are the VLP-32C's; the detector file's ``detector.sensor``
    # selects another model from reflective_pose_core.sensors.
    azimuth_step_rad: float = 0.0035  # 0.2 deg at 600 rpm / 10 Hz
    mean_elevation_step_rad: float = 0.0225  # mean gap of the VLP-32C table
    elevation_table_rad: Optional[np.ndarray] = field(
        default_factory=_default_elevation_table
    )
    # The sensor's own up axis in the cloud frame. The elevation table is
    # measured about it, so a driver that publishes in the sensor's native axes
    # (Seyond: x up) needs it said; a Velodyne's is the cloud's z.
    sensor_up: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    # Scans stacked before detection. The expected return count scales with it,
    # so a node accumulating 10 scans and a detector assuming 1 would reject
    # every real board as ten times too dense.
    scan_count: int = 1
    # Stage 4
    edge_margin_scale: float = 1.5  # multiples of local point spacing

    # Where the board is observed from, used only to orient its normal. A live
    # scan leaves this at the sensor origin; a merged map has no sensor, so the
    # anchoring tool passes the room centre instead.
    viewpoint: Optional[np.ndarray] = None


@dataclass
class Rejection:
    """A cluster that failed a gate, and why. Published for field debugging."""

    reason: str
    centroid: np.ndarray
    n_points: int
    detail: str = ""


@dataclass
class BoardDetection:
    """A board pose in the sensor frame."""

    centre: np.ndarray  # (3,) rectangle centre, sensor frame
    # Board frame, ROS convention: x is the outward normal, y is to the board's
    # left as seen from the sensor, z is up. Columns are those axes in the
    # sensor frame.
    rotation: np.ndarray  # (3, 3)
    normal: np.ndarray  # (3,) unit, pointing back towards the sensor
    up: np.ndarray  # (3,) unit, gravity-up projected into the board plane
    right: np.ndarray  # (3,) unit
    extents: Tuple[float, float]  # observed (width, height)
    points: np.ndarray  # (N, 3) the cluster itself, sensor frame, for debug output
    n_points: int
    plane_residual: float  # RMS distance to the fitted plane
    range_m: float
    observed_edges: Dict[str, bool]  # left / right / bottom / top
    # One scalar in [0, 1] from the terms below, and the terms themselves so
    # a consumer can say which one dragged it down. Filled in by
    # _evaluate_cluster; the defaults exist so a detection can be built by
    # hand in a test.
    confidence: float = 0.0
    confidence_terms: Dict[str, float] = field(default_factory=dict)

    @property
    def centre_constrained(self) -> Tuple[bool, bool]:
        """Whether the horizontal and vertical centre estimates are constrained.

        A centre coordinate is only trustworthy when at least one of its two
        bounding edges was actually observed. With neither edge seen, the
        bounding rectangle is a lower bound on the board, not the board.
        """
        e = self.observed_edges
        return (e["left"] or e["right"], e["bottom"] or e["top"])


@dataclass
class DetectResult:
    """Everything one detection attempt produced."""

    status: Status
    detection: Optional[BoardDetection] = None
    rejections: List[Rejection] = field(default_factory=list)
    n_after_gates: int = 0
    n_clusters: int = 0
    candidates: List[BoardDetection] = field(default_factory=list)


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


#: The 13 voxel offsets that, with their negatives, make up 26-connectivity.
#: Each edge is found once from its lexicographically smaller end.
_HALF_NEIGHBOURS = np.array(
    [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) > (0, 0, 0)
    ],
    dtype=np.int64,
)


def cluster_voxel_grid(
    points: np.ndarray, tolerance: float, min_points: int
) -> List[np.ndarray]:
    """Connected-component clustering on a voxel grid.

    Points are binned at ``tolerance`` resolution and occupied voxels are joined
    under 26-connectivity. This approximates Euclidean clustering at the same
    tolerance in O(N log N), needs no KD-tree, and is deterministic.

    The tolerance is governed by *across-ring* spacing rather than within-ring
    spacing: the VLP-32C's elevation gaps run from 0.33 deg to 9.36 deg, so a
    tolerance tuned to the dense band splits a board into horizontal stripes.

    Vectorised, because tracking runs it on every scan: voxels are encoded as
    one integer each, neighbours are found with a sorted search, and component
    labels are propagated along the edges until they stop changing (as many
    rounds as the widest component is long, a handful for a board). A
    pure-Python flood fill cost 8 ms of a Robin-W frame with a board at 1.5 m.

    Returns sorted index arrays, largest cluster first; equal sizes keep the
    order of their first point.
    """
    if len(points) == 0:
        return []

    keys = np.floor(np.asarray(points, dtype=np.float64) / tolerance).astype(np.int64)
    # Shift so every key and every neighbour of one is a non-negative index,
    # then fold the three axes into one code.
    keys -= keys.min(axis=0) - 1
    dims = keys.max(axis=0) + 2
    codes = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]

    voxel_codes, point_voxel = np.unique(codes, return_inverse=True)
    point_voxel = point_voxel.reshape(-1)
    n_voxels = len(voxel_codes)

    # Edges between occupied voxels.
    first = np.zeros(n_voxels, dtype=np.int64)
    first[point_voxel[::-1]] = np.arange(len(points))[::-1]
    voxel_keys = keys[first]
    sources, targets = [], []
    for offset in _HALF_NEIGHBOURS:
        neighbour = voxel_keys + offset
        neighbour_codes = (neighbour[:, 0] * dims[1] + neighbour[:, 1]) * dims[2] + neighbour[:, 2]
        where = np.searchsorted(voxel_codes, neighbour_codes)
        where = np.minimum(where, n_voxels - 1)
        hit = voxel_codes[where] == neighbour_codes
        if hit.any():
            sources.append(np.flatnonzero(hit))
            targets.append(where[hit])

    labels = np.arange(n_voxels)
    if sources:
        a = np.concatenate(sources)
        b = np.concatenate(targets)
        while True:
            low = np.minimum(labels[a], labels[b])
            updated = labels.copy()
            np.minimum.at(updated, a, low)
            np.minimum.at(updated, b, low)
            # Pointer jumping: a label's own label, until it is a root.
            updated = updated[updated]
            if np.array_equal(updated, labels):
                break
            labels = updated

    point_labels = labels[point_voxel]
    order = np.argsort(point_labels, kind="stable")
    sorted_labels = point_labels[order]
    boundaries = np.flatnonzero(np.diff(sorted_labels)) + 1
    groups = np.split(order, boundaries)

    clusters = [group for group in groups if len(group) >= min_points]
    clusters.sort(key=lambda group: (-len(group), int(group[0])))
    return clusters


def _sensor_elevation_height(params: DetectorParams, point: np.ndarray) -> float:
    """``point``'s component along the sensor's own up axis."""
    up = np.asarray(params.sensor_up, dtype=np.float64)
    return float(np.dot(point, up) / max(np.linalg.norm(up), 1e-12))


def _expected_point_count(
    params: DetectorParams, range_m: float, centre_z: float
) -> Optional[float]:
    """Returns a fully visible board should produce at this range and height.

    Rows come from the sensor's actual elevation table rather than a mean step.
    The VLP-32C's gaps span 0.33 to 9.36 degrees, so which part of the fan the
    board falls into changes the count by a factor of three — a mean-step model
    is not merely imprecise, it is wrong in a range-dependent direction.

    ``centre_z`` is the board centre's height along the sensor's own up axis
    (``_sensor_elevation_height``), not the cloud frame's z.
    """
    if params.elevation_table_rad is None:
        return None

    horizontal = max(np.hypot(range_m, 0.0), 1e-3)
    half_height = 0.5 * params.board_height
    elevation_low = np.arctan2(centre_z - half_height, horizontal)
    elevation_high = np.arctan2(centre_z + half_height, horizontal)

    table = np.asarray(params.elevation_table_rad)
    rows = int(np.count_nonzero((table >= elevation_low) & (table <= elevation_high)))
    if rows == 0:
        return None

    width_angle = 2.0 * np.arctan(0.5 * params.board_width / max(range_m, 1e-3))
    columns = width_angle / params.azimuth_step_rad
    return max(rows * columns * max(params.scan_count, 1), 1.0)


def _fit_plane(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """PCA of a point set. Returns centroid, eigenvalues asc, eigenvectors."""
    centroid = points.mean(axis=0)
    centred = points - centroid
    cov = centred.T @ centred / max(len(points) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    return centroid, eigenvalues, eigenvectors


def _observed_edges(
    coords: np.ndarray, extents: Tuple[float, float], params: DetectorParams, range_m: float
) -> Dict[str, bool]:
    """Decide which board edges the scan actually reached.

    An edge counts as observed when the measured extent along that axis reaches
    the nominal board dimension to within a margin scaled by the local point
    spacing. Without this, a partially observed board yields a bounding
    rectangle whose centre is biased towards the visible side.
    """
    az_spacing = range_m * params.azimuth_step_rad
    el_spacing = range_m * params.mean_elevation_step_rad
    margin_w = params.edge_margin_scale * az_spacing
    margin_h = params.edge_margin_scale * el_spacing

    width, height = extents
    centre_u = 0.5 * (coords[:, 0].min() + coords[:, 0].max())
    centre_v = 0.5 * (coords[:, 1].min() + coords[:, 1].max())

    half_w = 0.5 * params.board_width
    half_h = 0.5 * params.board_height

    # If the observed extent already spans the nominal board, both edges are in.
    full_w = width >= params.board_width - margin_w
    full_h = height >= params.board_height - margin_h

    return {
        "left": bool(full_w or coords[:, 0].min() <= centre_u - half_w + margin_w),
        "right": bool(full_w or coords[:, 0].max() >= centre_u + half_w - margin_w),
        "bottom": bool(full_h or coords[:, 1].min() <= centre_v - half_h + margin_h),
        "top": bool(full_h or coords[:, 1].max() >= centre_v + half_h - margin_h),
    }


#: The terms of the confidence scalar, and their weights in it. Every term is
#: in [0, 1] with 1 meaning "as good as this measurement gets".
#:
#: edges carries twice the weight of the others because an unobserved bounding
#: edge is the one defect that biases the *centre* -- by up to half the hidden
#: width -- rather than merely widening the covariance. range carries half:
#: precision degrades with distance, but a clean board at 12 m is still a
#: clean board, and the covariance already grows with range.
CONFIDENCE_TERMS: Dict[str, float] = {
    "planarity": 1.0,
    "extent": 1.0,
    "density": 1.0,
    "edges": 2.0,
    "range": 0.5,
}


def _unit(value: float) -> float:
    return float(np.clip(value, 0.0, 1.0))


def confidence_terms(detection: BoardDetection, params: DetectorParams) -> Dict[str, float]:
    """Each confidence term of a detection, in [0, 1], keyed by name.

    All of them are things the gates already measured, re-expressed as "how
    far inside the gate did this land":

    - ``planarity``: 1 for a residual anywhere below half of
      ``planarity_max_thickness``, falling to 0 at the gate. The gate is set
      at a few times the sensor's range noise, and a residual at the noise
      floor is as flat as a board can measure; scoring it against zero would
      mark a clean board down for the sensor it was seen with.
    - ``extent``: 1 at the nominal size; 0 at the edge of ``extent_tolerance``
      on whichever side the error is, each axis; the worse axis counts. An
      extent is measured from samples, so it falls short of the object by up
      to one sample spacing per axis -- the same spacing ``_observed_edges``
      allows -- and that much under-read is forgiven. Over-read is not: a
      sampled object cannot measure larger than it is, so an excess is
      evidence (a frame, a halo, a neighbour) and counts in full.
    - ``density``: the return count against what the sensor model expects for
      this range and height. Under-dense scores the ratio itself (an oblique
      or partly hidden board returns fewer points); over-dense falls linearly
      to 0 at ``density_max_ratio``. Absent when the model has no rows for the
      board, exactly when the density gate is skipped.
    - ``edges``: the fraction of the four bounding edges observed.
    - ``range``: 1 at ``range_min``, 0 at ``range_max``.
    """
    terms: Dict[str, float] = {}

    terms["planarity"] = _unit(
        2.0 * (1.0 - detection.plane_residual / max(params.planarity_max_thickness, 1e-9))
    )

    lo, hi = params.extent_tolerance
    spacings = (
        detection.range_m * params.azimuth_step_rad,
        detection.range_m * params.mean_elevation_step_rad,
    )
    worst = 0.0
    for measured, nominal, spacing in zip(
        detection.extents, (params.board_width, params.board_height), spacings
    ):
        nominal = max(nominal, 1e-9)
        if measured < nominal:
            error = max(nominal - measured - spacing, 0.0) / (nominal * max(1.0 - lo, 1e-9))
        else:
            error = (measured - nominal) / (nominal * max(hi - 1.0, 1e-9))
        worst = max(worst, error)
    terms["extent"] = _unit(1.0 - worst)

    expected = _expected_point_count(
        params, detection.range_m, _sensor_elevation_height(params, detection.centre)
    )
    if expected is not None:
        ratio = detection.n_points / expected
        if ratio <= 1.0:
            terms["density"] = _unit(ratio)
        else:
            terms["density"] = _unit(
                1.0 - (ratio - 1.0) / max(params.density_max_ratio - 1.0, 1e-9)
            )

    terms["edges"] = sum(1.0 for seen in detection.observed_edges.values() if seen) / 4.0

    span = max(params.range_max - params.range_min, 1e-9)
    terms["range"] = _unit(1.0 - (detection.range_m - params.range_min) / span)

    return terms


def confidence_from_terms(terms: Dict[str, float]) -> float:
    """Weighted mean of the terms present, by ``CONFIDENCE_TERMS``."""
    total = sum(CONFIDENCE_TERMS[name] for name in terms)
    if total <= 0.0:
        return 0.0
    return float(sum(CONFIDENCE_TERMS[name] * value for name, value in terms.items()) / total)


def _evaluate_cluster(
    points: np.ndarray,
    params: DetectorParams,
    up_world: np.ndarray,
    sensor_origin: np.ndarray,
    height_of: np.ndarray,
) -> Tuple[Optional[BoardDetection], Optional[Rejection]]:
    """Apply the stage 3 gates to one cluster and, if it passes, extract a pose."""
    centroid, eigenvalues, eigenvectors = _fit_plane(points)
    normal = eigenvectors[:, 0]  # smallest eigenvalue
    thickness = float(np.sqrt(max(eigenvalues[0], 0.0)))

    if thickness > params.planarity_max_thickness:
        return None, Rejection(
            "not_planar", centroid, len(points), f"thickness {thickness:.3f} m"
        )

    # Point the normal back at the sensor so the board frame is unambiguous.
    if np.dot(normal, sensor_origin - centroid) < 0.0:
        normal = -normal

    vertical_dot = float(abs(np.dot(normal, up_world)))
    if vertical_dot > params.verticality_max_dot:
        return None, Rejection(
            "not_vertical", centroid, len(points), f"|n.up| {vertical_dot:.2f}"
        )

    up = up_world - np.dot(up_world, normal) * normal
    norm = np.linalg.norm(up)
    if norm < 1e-6:
        return None, Rejection("degenerate_up", centroid, len(points))
    up = up / norm
    right = np.cross(up, normal)
    right = right / np.linalg.norm(right)

    centred = points - centroid
    coords = np.column_stack((centred @ right, centred @ up))
    width = float(coords[:, 0].max() - coords[:, 0].min())
    height = float(coords[:, 1].max() - coords[:, 1].min())

    lo, hi = params.extent_tolerance
    slack_w = slack_h = 0.0
    if params.extent_sampling_slack:
        sample_range = float(np.linalg.norm(centroid - sensor_origin))
        slack_w = 2.0 * sample_range * params.azimuth_step_rad
        slack_h = 2.0 * sample_range * params.mean_elevation_step_rad
    if not (lo * params.board_width - slack_w <= width <= hi * params.board_width):
        return None, Rejection(
            "bad_width", centroid, len(points), f"{width:.2f} m"
        )
    if not (lo * params.board_height - slack_h <= height <= hi * params.board_height):
        return None, Rejection(
            "bad_height", centroid, len(points), f"{height:.2f} m"
        )

    mean_height = float(height_of(centroid))
    if abs(mean_height - params.board_centre_height) > params.centre_height_tolerance:
        return None, Rejection(
            "bad_mount_height", centroid, len(points), f"{mean_height:.2f} m"
        )

    range_m = float(np.linalg.norm(centroid - sensor_origin))
    if params.density_check_enabled:
        expected = _expected_point_count(
            params, range_m, _sensor_elevation_height(params, centroid)
        )
        if expected is not None:
            ratio = len(points) / expected
            if ratio > params.density_max_ratio:
                return None, Rejection(
                    "too_dense", centroid, len(points), f"ratio {ratio:.2f}"
                )

    # Bounding-rectangle centre, not the centroid: with a partially observed
    # board the centroid is pulled towards the visible side.
    centre_u = 0.5 * (coords[:, 0].min() + coords[:, 0].max())
    centre_v = 0.5 * (coords[:, 1].min() + coords[:, 1].max())
    centre = centroid + centre_u * right + centre_v * up

    residual = float(np.sqrt(np.mean((centred @ normal) ** 2)))
    edges = _observed_edges(coords, (width, height), params, range_m)

    detection = BoardDetection(
        points=points,
        centre=centre,
        rotation=np.column_stack((normal, right, up)),
        normal=normal,
        up=up,
        right=right,
        extents=(width, height),
        n_points=len(points),
        plane_residual=residual,
        range_m=range_m,
        observed_edges=edges,
    )
    detection.confidence_terms = confidence_terms(detection, params)
    detection.confidence = confidence_from_terms(detection.confidence_terms)
    return detection, None


def detect_board(
    points: np.ndarray,
    intensity: np.ndarray,
    transform_height_frame_sensor: np.ndarray,
    params: Optional[DetectorParams] = None,
    *,
    height_offset: float = 0.0,
) -> DetectResult:
    """Detect the retroreflective board in one accumulated scan.

    Args:
        points: (N, 3) points in the sensor frame.
        intensity: (N,) calibrated reflectivity. On a VLP-32C, 0-100 is diffuse
            and 101-255 is reserved for retroreflectors, which is why the
            intensity gate is a sensor contract rather than a tuned threshold.
        transform_height_frame_sensor: 4x4 transform from the sensor frame to
            the frame used by the height policy. Runtime callers pass the real
            ``base_link <- sensor`` TF. Map callers pass identity only after
            the cloud has been gravity-levelled and translated so its fitted
            floor is z=0.
        params: thresholds; defaults are the shipped configuration.
        height_offset: additive offset from the transform's z datum to ground,
            in metres. Runtime uses the measured base-link height above ground;
            map mode uses zero. It affects only point and candidate height
            gates, never range, viewpoint, or the returned pose.

    Returns:
        A DetectResult. ``status`` is AMBIGUOUS when more than one cluster
        survives every gate: the map holds exactly one board, so a second
        survivor means the assumption is violated and picking a winner would
        produce a confident wrong pose.
    """
    params = params or DetectorParams()
    points = np.asarray(points, dtype=np.float64)
    intensity = np.asarray(intensity, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be (N, 3)")
    if len(points) != len(intensity):
        raise ValueError("points and intensity must have the same length")

    height_offset = float(height_offset)
    if not np.isfinite(height_offset):
        raise ValueError("height_offset must be finite")

    rotation = transform_height_frame_sensor[:3, :3]
    # Gravity-up expressed in the sensor frame.
    up_world = rotation.T @ np.array([0.0, 0.0, 1.0])
    up_world = up_world / np.linalg.norm(up_world)

    def height_of(p: np.ndarray) -> np.ndarray:
        return (rotation @ p + transform_height_frame_sensor[:3, 3])[..., 2] + height_offset

    if len(points) == 0:
        return DetectResult(Status.NO_CANDIDATE)

    # Intensity first, and the geometric point gates on what survives it: the
    # gates are a conjunction, so the order changes nothing but the cost, and
    # on a dense sensor almost every point is diffuse (a Robin-W frame is
    # ~220k points, of which a board is a few thousand).
    candidates = points[intensity >= params.intensity_threshold]
    ranges = np.linalg.norm(candidates, axis=1)
    heights = _transform_points(candidates, transform_height_frame_sensor)[:, 2] + height_offset
    keep = (ranges >= params.range_min) & (ranges <= params.range_max)
    keep &= (heights >= params.height_min) & (heights <= params.height_max)

    kept = candidates[keep]
    result = DetectResult(Status.NO_CANDIDATE, n_after_gates=int(keep.sum()))
    if len(kept) < params.cluster_min_points:
        return result

    clusters = cluster_voxel_grid(kept, params.cluster_tolerance, params.cluster_min_points)
    result.n_clusters = len(clusters)

    sensor_origin = (
        np.zeros(3) if params.viewpoint is None
        else np.asarray(params.viewpoint, dtype=np.float64)
    )
    survivors: List[BoardDetection] = []
    for indices in clusters:
        detection, rejection = _evaluate_cluster(
            kept[indices], params, up_world, sensor_origin, height_of
        )
        if detection is not None:
            survivors.append(detection)
        elif rejection is not None:
            result.rejections.append(rejection)

    result.candidates = survivors
    if len(survivors) == 0:
        result.status = Status.NO_CANDIDATE
    elif len(survivors) > 1:
        result.status = Status.AMBIGUOUS
    else:
        result.status = Status.OK
        result.detection = survivors[0]
    return result
