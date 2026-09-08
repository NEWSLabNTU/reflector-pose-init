"""Retroreflective board detection from a single accumulated LiDAR scan.

This module is deliberately free of ROS imports. It is shared between the
runtime initializer node and the offline map-anchoring step, and keeping it pure
numpy is what lets its tests run without ROS, without hardware, and without a
bag.

See docs/design/board_pose_initializer.md for the algorithm rationale.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

from .vlp32 import elevation_table


class Status(Enum):
    """Outcome of a detection attempt."""

    OK = "ok"
    NO_CANDIDATE = "no_candidate"
    AMBIGUOUS = "ambiguous"


@dataclass
class DetectorParams:
    """Detection thresholds. Defaults match config/board_initializer.param.yaml."""

    # Stage 1 gates
    intensity_threshold: float = 110.0
    # 3 m is not arbitrary. Below it the board drops into the VLP-32C's sparse
    # lower elevation band, where the gap between the -25.0 deg and -15.6 deg
    # beams is 9.4 deg: the bottom of the board is sampled by a single ring that
    # then fails to connect to the rest, and the cluster loses its lower third.
    range_min: float = 3.0
    range_max: float = 18.0
    height_min: float = 0.4
    height_max: float = 1.8

    # Stage 2 clustering
    cluster_tolerance: float = 0.30
    cluster_min_points: int = 20

    # Board geometry
    board_width: float = 0.8
    board_height: float = 1.0
    board_centre_height: float = 1.075

    # Stage 3 gates
    extent_tolerance: Tuple[float, float] = (0.6, 1.2)
    planarity_max_thickness: float = 0.03
    verticality_max_dot: float = 0.25
    centre_height_tolerance: float = 0.30
    # Density is checked as an upper bound only. Yaw, occlusion, and dropout all
    # legitimately reduce the return count; nothing legitimately inflates it, so
    # an excess means the cluster is not the object its extents claim.
    density_max_ratio: float = 1.4
    density_check_enabled: bool = True

    # Sensor model. The elevation table drives both the density gate and the
    # edge-observation margins; without it the density gate is skipped, because
    # a mean-step approximation is wrong by 3x across the working range.
    azimuth_step_rad: float = 0.0035  # 0.2 deg at 600 rpm / 10 Hz
    mean_elevation_step_rad: float = 0.0225  # mean gap of the VLP-32C table
    elevation_table_rad: Optional[np.ndarray] = field(default_factory=elevation_table)
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


def cluster_voxel_grid(
    points: np.ndarray, tolerance: float, min_points: int
) -> List[np.ndarray]:
    """Connected-component clustering on a voxel grid.

    Points are binned at ``tolerance`` resolution and occupied voxels are joined
    under 26-connectivity. This approximates Euclidean clustering at the same
    tolerance in O(N), needs no KD-tree, and is deterministic.

    The tolerance is governed by *across-ring* spacing rather than within-ring
    spacing: the VLP-32C's elevation gaps run from 0.33 deg to 9.36 deg, so a
    tolerance tuned to the dense band splits a board into horizontal stripes.

    Returns index arrays, largest cluster first.
    """
    if len(points) == 0:
        return []

    keys = np.floor(points / tolerance).astype(np.int64)
    voxels: Dict[Tuple[int, int, int], List[int]] = {}
    for idx, key in enumerate(map(tuple, keys)):
        voxels.setdefault(key, []).append(idx)

    neighbours = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]

    unvisited = set(voxels)
    clusters: List[np.ndarray] = []
    while unvisited:
        seed = unvisited.pop()
        component = [seed]
        stack = [seed]
        while stack:
            cx, cy, cz = stack.pop()
            for dx, dy, dz in neighbours:
                nb = (cx + dx, cy + dy, cz + dz)
                if nb in unvisited:
                    unvisited.remove(nb)
                    component.append(nb)
                    stack.append(nb)

        indices = np.concatenate([np.asarray(voxels[v], dtype=np.int64) for v in component])
        if len(indices) >= min_points:
            clusters.append(indices)

    clusters.sort(key=len, reverse=True)
    return clusters


def _expected_point_count(
    params: DetectorParams, range_m: float, centre_z: float
) -> Optional[float]:
    """Returns a fully visible board should produce at this range and height.

    Rows come from the sensor's actual elevation table rather than a mean step.
    The VLP-32C's gaps span 0.33 to 9.36 degrees, so which part of the fan the
    board falls into changes the count by a factor of three — a mean-step model
    is not merely imprecise, it is wrong in a range-dependent direction.
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
    if not (lo * params.board_width <= width <= hi * params.board_width):
        return None, Rejection(
            "bad_width", centroid, len(points), f"{width:.2f} m"
        )
    if not (lo * params.board_height <= height <= hi * params.board_height):
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
        expected = _expected_point_count(params, range_m, float(centroid[2]))
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
    return detection, None


def detect_board(
    points: np.ndarray,
    intensity: np.ndarray,
    transform_base_sensor: np.ndarray,
    params: Optional[DetectorParams] = None,
) -> DetectResult:
    """Detect the retroreflective board in one accumulated scan.

    Args:
        points: (N, 3) points in the sensor frame.
        intensity: (N,) calibrated reflectivity. On a VLP-32C, 0-100 is diffuse
            and 101-255 is reserved for retroreflectors, which is why the
            intensity gate is a sensor contract rather than a tuned threshold.
        transform_base_sensor: 4x4 ``base_link <- sensor`` transform. Needed for
            the height gate and for the gravity-up direction.
        params: thresholds; defaults are the shipped configuration.

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

    rotation = transform_base_sensor[:3, :3]
    # Gravity-up expressed in the sensor frame.
    up_world = rotation.T @ np.array([0.0, 0.0, 1.0])
    up_world = up_world / np.linalg.norm(up_world)

    def height_of(p: np.ndarray) -> np.ndarray:
        return (rotation @ p + transform_base_sensor[:3, 3])[..., 2]

    if len(points) == 0:
        return DetectResult(Status.NO_CANDIDATE)

    ranges = np.linalg.norm(points, axis=1)
    heights = _transform_points(points, transform_base_sensor)[:, 2]

    keep = intensity >= params.intensity_threshold
    keep &= (ranges >= params.range_min) & (ranges <= params.range_max)
    keep &= (heights >= params.height_min) & (heights <= params.height_max)

    kept = points[keep]
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
