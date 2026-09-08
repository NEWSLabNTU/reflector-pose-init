"""Synthetic VLP-32C scan generator.

The beam model reads the per-laser elevation and azimuth corrections from the
Nebula calibration file the driver itself uses, so ring coverage on a target at
range is faithful rather than assumed uniform. The table is embedded as a
fallback so the tests run on machines without Autoware installed.

Intensity reproduces the sensor's calibrated-reflectivity semantics: 0-100 for
diffuse surfaces, 101-255 reserved for retroreflectors. The detector's intensity
gate is tested against that contract rather than against a guessed number.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from golfcart_board_initializer.vlp32 import load_beam_table

DIFFUSE = "diffuse"
RETRO = "retro"


@dataclass
class Rectangle:
    """A finite planar rectangle in the sensor frame."""

    centre: np.ndarray
    normal: np.ndarray  # unit, pointing towards the sensor side
    up: np.ndarray  # unit, in-plane
    half_width: float
    half_height: float
    material: str = DIFFUSE
    reflectivity: float = 1.0
    name: str = ""

    def __post_init__(self):
        self.centre = np.asarray(self.centre, dtype=np.float64)
        self.normal = np.asarray(self.normal, dtype=np.float64)
        self.normal /= np.linalg.norm(self.normal)
        up = np.asarray(self.up, dtype=np.float64)
        up = up - np.dot(up, self.normal) * self.normal
        self.up = up / np.linalg.norm(up)
        self.right = np.cross(self.up, self.normal)
        self.right /= np.linalg.norm(self.right)

    def pose(self) -> np.ndarray:
        """4x4 ``sensor <- rectangle``, columns normal / right / up.

        Same board-frame convention as the detector: x outward, y left, z up.
        """
        transform = np.eye(4)
        transform[:3, :3] = np.column_stack((self.normal, self.right, self.up))
        transform[:3, 3] = self.centre
        return transform


@dataclass
class Scene:
    """A collection of rectangles plus optional post-cast occlusion."""

    rectangles: List[Rectangle] = field(default_factory=list)
    # (name, axis, keep_fraction, side) — drops part of a named target's returns
    # after casting, which models occlusion without adding an occluder body.
    occlusions: List[Tuple[str, str, float, str]] = field(default_factory=list)

    def add(self, rectangle: Rectangle) -> "Rectangle":
        self.rectangles.append(rectangle)
        return rectangle


@dataclass
class SimParams:
    azimuth_step_rad: float = 0.0035  # 0.2 deg at 600 rpm / 10 Hz
    range_noise: float = 0.02
    dropout: float = 0.02
    blooming: bool = False
    bloom_range_bias: float = 0.03
    bloom_halo_fraction: float = 0.3
    max_range: float = 120.0
    seed: int = 0


@dataclass
class Scan:
    points: np.ndarray  # (N, 3) sensor frame
    intensity: np.ndarray  # (N,)
    ring: np.ndarray  # (N,)
    source: np.ndarray  # (N,) index into scene.rectangles, -1 for bloom halo


def beam_directions(params: SimParams) -> Tuple[np.ndarray, np.ndarray]:
    """Unit direction per (azimuth, laser) pair, with the ring index."""
    vertical, rotational = load_beam_table()
    azimuths = np.arange(0.0, 2.0 * np.pi, params.azimuth_step_rad)

    azimuth_grid = azimuths[:, None] + rotational[None, :]
    elevation_grid = np.broadcast_to(vertical[None, :], azimuth_grid.shape)

    cos_elevation = np.cos(elevation_grid)
    directions = np.stack(
        (
            cos_elevation * np.cos(azimuth_grid),
            cos_elevation * np.sin(azimuth_grid),
            np.sin(elevation_grid),
        ),
        axis=-1,
    ).reshape(-1, 3)

    rings = np.broadcast_to(
        np.arange(len(vertical))[None, :], azimuth_grid.shape
    ).reshape(-1)
    return directions, rings


def _intersect(rectangle: Rectangle, directions: np.ndarray) -> np.ndarray:
    """Ray parameter t per direction, np.inf where the ray misses."""
    denominator = directions @ rectangle.normal
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (rectangle.centre @ rectangle.normal) / denominator
    t = np.where(np.abs(denominator) < 1e-9, np.inf, t)
    t = np.where(t > 0.0, t, np.inf)

    finite = np.isfinite(t)
    if not finite.any():
        return t

    hits = directions[finite] * t[finite][:, None]
    local = hits - rectangle.centre
    inside = (np.abs(local @ rectangle.right) <= rectangle.half_width) & (
        np.abs(local @ rectangle.up) <= rectangle.half_height
    )
    result = t.copy()
    indices = np.flatnonzero(finite)
    result[indices[~inside]] = np.inf
    return result


def _intensity(
    rectangle: Rectangle,
    directions: np.ndarray,
    ranges: np.ndarray,
    rng: np.random.Generator,
) -> np.ndarray:
    cos_incidence = np.abs(directions @ rectangle.normal)
    if rectangle.material == RETRO:
        value = 255.0 * rectangle.reflectivity * np.sqrt(np.clip(cos_incidence, 0.0, 1.0))
        value = value + rng.normal(0.0, 10.0, size=value.shape)
        return np.clip(value, 101.0, 255.0)

    falloff = np.sqrt(5.0 / np.clip(ranges, 0.5, None))
    value = 80.0 * rectangle.reflectivity * cos_incidence * falloff
    value = value + rng.normal(0.0, 5.0, size=value.shape)
    return np.clip(value, 0.0, 100.0)


def _apply_occlusion(
    scene: Scene, scan_points: np.ndarray, source: np.ndarray, keep: np.ndarray
) -> np.ndarray:
    for name, axis, fraction, side in scene.occlusions:
        indices = [i for i, r in enumerate(scene.rectangles) if r.name == name]
        if not indices:
            continue
        rectangle = scene.rectangles[indices[0]]
        selected = np.flatnonzero((source == indices[0]) & keep)
        if len(selected) == 0:
            continue
        local = scan_points[selected] - rectangle.centre
        coordinate = local @ (rectangle.right if axis == "horizontal" else rectangle.up)
        lo, hi = coordinate.min(), coordinate.max()
        if side == "low":
            threshold = lo + fraction * (hi - lo)
            drop = coordinate < threshold
        else:
            threshold = hi - fraction * (hi - lo)
            drop = coordinate > threshold
        keep[selected[drop]] = False
    return keep


def simulate(scene: Scene, params: SimParams = SimParams()) -> Scan:
    """Cast the scene and return a noisy scan."""
    rng = np.random.default_rng(params.seed)
    directions, rings = beam_directions(params)

    best_t = np.full(len(directions), np.inf)
    best_source = np.full(len(directions), -1, dtype=np.int64)
    for index, rectangle in enumerate(scene.rectangles):
        t = _intersect(rectangle, directions)
        closer = t < best_t
        best_t[closer] = t[closer]
        best_source[closer] = index

    hit = np.isfinite(best_t) & (best_t <= params.max_range)
    directions = directions[hit]
    rings = rings[hit]
    ranges = best_t[hit]
    source = best_source[hit]

    intensity = np.zeros(len(ranges))
    for index, rectangle in enumerate(scene.rectangles):
        mask = source == index
        if not mask.any():
            continue
        intensity[mask] = _intensity(rectangle, directions[mask], ranges[mask], rng)
        if rectangle.material == RETRO and params.blooming:
            # Retroreflector returns read slightly long on Velodyne hardware.
            ranges[mask] = ranges[mask] + params.bloom_range_bias

    ranges = ranges + rng.normal(0.0, params.range_noise, size=ranges.shape)
    points = directions * ranges[:, None]

    keep = rng.random(len(points)) >= params.dropout
    keep = _apply_occlusion(scene, points, source, keep)

    points, intensity, rings, source = (
        points[keep],
        intensity[keep],
        rings[keep],
        source[keep],
    )

    if params.blooming:
        points, intensity, rings, source = _add_bloom_halo(
            scene, points, intensity, rings, source, params, rng
        )

    return Scan(points=points, intensity=intensity, ring=rings, source=source)


def _add_bloom_halo(
    scene: Scene,
    points: np.ndarray,
    intensity: np.ndarray,
    rings: np.ndarray,
    source: np.ndarray,
    params: SimParams,
    rng: np.random.Generator,
):
    """Spurious points around retroreflector edges, at inflated range.

    This is the failure mode most likely to bite on real hardware and the one a
    naive simulator omits, so the detector is exercised against it.
    """
    extra_points: List[np.ndarray] = []
    extra_intensity: List[np.ndarray] = []
    extra_rings: List[np.ndarray] = []

    for index, rectangle in enumerate(scene.rectangles):
        if rectangle.material != RETRO:
            continue
        mask = source == index
        selected = np.flatnonzero(mask)
        if len(selected) == 0:
            continue

        local = points[selected] - rectangle.centre
        u = local @ rectangle.right
        v = local @ rectangle.up
        near_edge = (np.abs(u) > rectangle.half_width - 0.10) | (
            np.abs(v) > rectangle.half_height - 0.10
        )
        candidates = selected[near_edge]
        if len(candidates) == 0:
            continue

        count = int(params.bloom_halo_fraction * len(candidates))
        if count == 0:
            continue
        chosen = rng.choice(candidates, size=count, replace=False)

        offsets = rng.uniform(0.03, 0.08, size=count)
        signs = rng.choice([-1.0, 1.0], size=count)
        axis = np.where(
            rng.random(count) < 0.5, 0.0, 1.0
        )  # 0 = along right, 1 = along up
        direction = (
            (1.0 - axis)[:, None] * rectangle.right + axis[:, None] * rectangle.up
        )
        halo = points[chosen] + (signs * offsets)[:, None] * direction
        halo = halo * (1.0 + rng.uniform(0.002, 0.005, size=count))[:, None]

        extra_points.append(halo)
        extra_intensity.append(rng.uniform(101.0, 200.0, size=count))
        extra_rings.append(rings[chosen])

    if not extra_points:
        return points, intensity, rings, source

    points = np.vstack([points] + extra_points)
    intensity = np.concatenate([intensity] + extra_intensity)
    rings = np.concatenate([rings] + extra_rings)
    source = np.concatenate(
        [source] + [np.full(len(p), -1, dtype=np.int64) for p in extra_points]
    )
    return points, intensity, rings, source


def to_pointcloud_fields(scan: Scan) -> dict:
    """Field arrays matching sensor_msgs/PointCloud2 x, y, z, intensity, ring."""
    return {
        "x": scan.points[:, 0].astype(np.float32),
        "y": scan.points[:, 1].astype(np.float32),
        "z": scan.points[:, 2].astype(np.float32),
        "intensity": scan.intensity.astype(np.float32),
        "ring": scan.ring.astype(np.uint16),
    }
