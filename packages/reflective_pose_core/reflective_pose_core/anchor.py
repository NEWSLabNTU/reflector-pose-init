"""Anchor a finished SLAM cloud to the retroreflective board.

A SLAM map is self-consistent but sits in an arbitrary frame — its origin is
wherever the vehicle happened to be for the first scan. Rebuild it and the frame
moves. This module fixes the frame to something physical instead:

    origin  the floor point directly below the board's face centre
    +x      the board's outward normal
    +z      gravity up
    +y      completes the right-handed frame

Three things fall out of that, and they are the reason the tool exists:

- ``board_pose_in_map`` from shared YAML becomes the board's exact map pose, so
  runtime initialization and offline anchoring use one map contract.
- The board's Lanelet2 polygon coordinates are derived from its dimensions
  rather than surveyed, which doubles as a check on the anchoring.
- A rebuild from the same bag can be compared against the stored transform
  instead of silently landing somewhere else.

Detection reuses ``detector.py`` unchanged, so the board pose that defines the
map and the board pose the vehicle computes at startup come from identical code:
a detector bias cancels rather than presenting as a localization error.

ROS-free. See docs/design/indoor_pcd_mapping_reflector_anchor.md §6.4.
"""

from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

from .detector import DetectorParams, DetectResult, Status, detect_board
from .geometry import make_transform, matrix_from_euler_rpy
from .pointcloud_io import PointCloud


@dataclass
class AnchorParams:
    """Anchoring inputs. Detection gates come from DetectorParams."""

    board_centre_height: float = 1.075
    board_width: float = 0.8
    board_height: float = 1.0
    # [x, y, z, roll, pitch, yaw], radians. This is shared with the runtime
    # node's board_pose_in_map parameter.
    board_pose_in_map: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 1.075, 0.0, 0.0, 0.0
    )

    # Floor fit
    floor_band: float = 0.3  # metres above the lowest points to fit within
    floor_percentile: float = 2.0  # percentile taken as "the lowest points"
    floor_inlier: float = 0.05  # refit tolerance, metres
    floor_refits: int = 3
    max_floor_tilt_deg: float = 10.0


@dataclass
class AnchorResult:
    transform_map_cloud: np.ndarray  # 4x4, apply to the cloud to anchor it
    board_centre_cloud: np.ndarray  # (3,) board centre in the input frame
    board_normal_cloud: np.ndarray  # (3,) outward normal in the input frame
    floor_normal_cloud: np.ndarray  # (3,) fitted up direction in the input frame
    floor_tilt_deg: float  # angle between the fitted floor and the input z
    detection: object  # BoardDetection, in the gravity-aligned frame
    n_points: int


def detector_params_for_map(base: Optional[DetectorParams] = None) -> DetectorParams:
    """Detection thresholds adjusted for a merged map rather than one scan.

    Two gates are meaningless here and are turned off rather than retuned:

    - **Range.** Distances in a map are measured from an arbitrary origin, not
      from a sensor, so a range window rejects the board for no reason.
    - **Density.** The expected return count assumes one scan from one
      viewpoint. A map merges many, so the count is unbounded from above and
      the gate would reject every real board.

    Everything geometric — planarity, verticality, extent, mounting height —
    still applies, and those are the gates that separate the board from the exit
    signage anyway.
    """
    params = base or DetectorParams()
    return DetectorParams(
        intensity_threshold=params.intensity_threshold,
        range_min=0.0,
        range_max=float("inf"),
        height_min=params.height_min,
        height_max=params.height_max,
        cluster_tolerance=params.cluster_tolerance,
        cluster_min_points=params.cluster_min_points,
        board_width=params.board_width,
        board_height=params.board_height,
        board_centre_height=params.board_centre_height,
        extent_tolerance=params.extent_tolerance,
        planarity_max_thickness=params.planarity_max_thickness,
        verticality_max_dot=params.verticality_max_dot,
        centre_height_tolerance=params.centre_height_tolerance,
        density_check_enabled=False,
    )


def fit_floor(points: np.ndarray, params: AnchorParams) -> Tuple[np.ndarray, float]:
    """Fit the floor plane, returning its upward normal and height along it.

    Assumes the SLAM output is roughly gravity-aligned, which holds for
    LiDAR-inertial SLAM: GLIM's IMU preintegration observes gravity, so its z
    axis is vertical to within about a degree. The fit refines that rather than
    discovering it, and a tilt beyond ``max_floor_tilt_deg`` is treated as a
    failure — at that point the assumption is wrong and a plane fit on the
    lowest points is fitting something that is not the floor.
    """
    if len(points) < 3:
        raise ValueError("too few points to fit a floor")

    threshold = np.percentile(points[:, 2], params.floor_percentile)
    band = points[points[:, 2] <= threshold + params.floor_band]
    if len(band) < 3:
        raise ValueError("no points in the floor band")

    def fit(sample):
        centre = sample.mean(axis=0)
        _, _, vectors = np.linalg.svd(sample - centre, full_matrices=False)
        direction = vectors[-1]
        if direction[2] < 0:
            direction = -direction
        return centre, direction

    centroid, normal = fit(band)

    # Refit on inliers. The initial band is everything within floor_band of the
    # lowest points, which also catches wall bases and floor markings; their
    # centroid sits above the floor and tilts the plane. Two or three refits
    # pull it onto the floor itself.
    for _ in range(params.floor_refits):
        distance = np.abs((band - centroid) @ normal)
        inliers = band[distance <= params.floor_inlier]
        if len(inliers) < 3:
            break
        centroid, normal = fit(inliers)

    tilt = np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))
    if tilt > params.max_floor_tilt_deg:
        raise ValueError(
            f"floor fit tilted {tilt:.1f} deg from vertical, beyond "
            f"{params.max_floor_tilt_deg} deg — the cloud is not gravity-aligned "
            "or the lowest points are not the floor"
        )

    return normal, float(np.dot(centroid, normal))


def _rotation_aligning(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Shortest rotation taking one unit vector onto another."""
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)
    axis = np.cross(source, target)
    sine = np.linalg.norm(axis)
    cosine = float(np.dot(source, target))
    if sine < 1e-9:
        return np.eye(3) if cosine > 0 else -np.eye(3)
    axis = axis / sine
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    angle = np.arctan2(sine, cosine)
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def anchor_cloud(
    cloud: PointCloud,
    params: Optional[AnchorParams] = None,
    detector_params: Optional[DetectorParams] = None,
    on_result: Optional[Callable[[np.ndarray, np.ndarray, DetectResult, np.ndarray], None]] = None,
) -> AnchorResult:
    """Find the board in a map cloud and compute the transform that anchors it.

    Raises when the board cannot be identified, including when a second
    board-shaped retroreflector survives the gates: the map is supposed to hold
    one board, and picking between two would define the map frame off the wrong
    object — an error with no later symptom except that everything is shifted.

    ``on_result``, if given, is called with ``(levelled_points, intensity,
    DetectResult, viewpoint)`` right after detection runs and before either
    outcome is decided — including on failure, when the raised ``ValueError``
    would otherwise discard every rejected cluster. This is the hook the CLI's
    ``--rviz`` debug output uses; this module stays free of the ROS/plotting
    concerns that would otherwise pull in.
    """
    params = params or AnchorParams()
    if cloud.intensity is None:
        raise ValueError(
            "cloud has no intensity channel; the board cannot be found without it "
            "(check the PLY-to-PCD conversion preserved the field)"
        )

    points = np.asarray(cloud.points, dtype=np.float64)
    intensity = np.asarray(cloud.intensity, dtype=np.float64)

    floor_normal, floor_offset = fit_floor(points, params)
    tilt = float(np.degrees(np.arccos(np.clip(floor_normal[2], -1.0, 1.0))))

    # Work in a gravity-aligned frame with the floor at z = 0, so the detector's
    # height gates mean what they say.
    rotation = _rotation_aligning(floor_normal, np.array([0.0, 0.0, 1.0]))
    levelled = points @ rotation.T
    levelled[:, 2] -= floor_offset

    # The board faces into the room, so the room's own centre is a sound
    # viewpoint for orienting the normal. A sensor origin is not available here
    # and the trajectory may not be either.
    viewpoint = np.median(levelled, axis=0)

    map_params = detector_params_for_map(detector_params)
    map_params.viewpoint = viewpoint
    result: DetectResult = detect_board(levelled, intensity, np.eye(4), map_params)

    if on_result is not None:
        on_result(levelled, intensity, result, viewpoint)

    if result.status is Status.AMBIGUOUS:
        raise ValueError(
            f"{len(result.candidates)} board candidates in the map; the map frame "
            "cannot be defined against an ambiguous anchor"
        )
    if result.status is not Status.OK:
        reasons = ", ".join(
            f"{r.reason} {r.detail}".strip() for r in result.rejections
        )
        raise ValueError(
            f"no board found in the map ({result.n_clusters} retroreflective "
            f"clusters: {reasons or 'none survived the gates'})"
        )

    detection = result.detection

    # Map frame: origin under the board on the floor, +x along its normal.
    forward = np.array([detection.normal[0], detection.normal[1], 0.0])
    if np.linalg.norm(forward) < 1e-6:
        raise ValueError("board normal is vertical; it is not mounted upright")
    forward = forward / np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, forward)

    origin = np.array([detection.centre[0], detection.centre[1], 0.0])
    rotation_map_levelled = np.column_stack((forward, left, up)).T
    transform_map_levelled = make_transform(
        rotation_map_levelled, -rotation_map_levelled @ origin
    )

    transform_levelled_cloud = np.eye(4)
    transform_levelled_cloud[:3, :3] = rotation
    transform_levelled_cloud[2, 3] = -floor_offset

    canonical_map_cloud = transform_map_levelled @ transform_levelled_cloud
    pose = np.asarray(params.board_pose_in_map, dtype=np.float64)
    if pose.shape != (6,):
        raise ValueError(
            "board_pose_in_map must be [x, y, z, roll, pitch, yaw] "
            "with angles in radians"
        )
    target_rotation = matrix_from_euler_rpy(pose[3:])
    canonical_board_centre = (
        transform_map_levelled[:3, :3] @ detection.centre
        + transform_map_levelled[:3, 3]
    )
    # ``canonical_map_cloud`` places the centre above the floor, not at its
    # origin. Offset that known intermediate centre so the detected board lands
    # exactly at the configured translation.
    transform_map_canonical = make_transform(
        target_rotation, pose[:3] - target_rotation @ canonical_board_centre
    )
    transform_map_cloud = transform_map_canonical @ canonical_map_cloud

    inverse_rotation = rotation.T
    return AnchorResult(
        transform_map_cloud=transform_map_cloud,
        board_centre_cloud=inverse_rotation
        @ (detection.centre + np.array([0.0, 0.0, floor_offset])),
        board_normal_cloud=inverse_rotation @ detection.normal,
        floor_normal_cloud=floor_normal,
        floor_tilt_deg=tilt,
        detection=detection,
        n_points=len(points),
    )


def anchored_board_centre(result: AnchorResult) -> np.ndarray:
    """Board centre after anchoring, in the map frame.

    Goes through ``board_centre_cloud`` rather than ``detection.centre``: the
    detection lives in the levelled intermediate frame, and feeding that to
    ``transform_map_cloud`` gives a wrong answer that only becomes obvious once
    the source frame is far from the origin.
    """
    centre = np.asarray(result.board_centre_cloud)
    return result.transform_map_cloud[:3, :3] @ centre + result.transform_map_cloud[:3, 3]


def apply_transform(cloud: PointCloud, transform: np.ndarray) -> PointCloud:
    """Rigidly transform a cloud, carrying intensity through untouched."""
    points = np.asarray(cloud.points, dtype=np.float64)
    moved = points @ transform[:3, :3].T + transform[:3, 3]
    return PointCloud(points=moved, intensity=cloud.intensity)


def board_polygon_osm(params: AnchorParams, marker_id: str = "board_001") -> str:
    """Lanelet2 landmark polygon for the board, in the anchored map frame.

    Coordinates derive from board dimensions and ``board_pose_in_map``. Vertices
    are counter-clockwise as ``autoware_landmark_manager`` requires.
    """
    half_width = 0.5 * params.board_width
    corners = np.array([
        (0.0, half_width, -0.5 * params.board_height),
        (0.0, -half_width, -0.5 * params.board_height),
        (0.0, -half_width, 0.5 * params.board_height),
        (0.0, half_width, 0.5 * params.board_height),
    ])
    pose = np.asarray(params.board_pose_in_map, dtype=np.float64)
    if pose.shape != (6,):
        raise ValueError("board_pose_in_map must contain six values")
    corners = corners @ matrix_from_euler_rpy(pose[3:]).T + pose[:3]

    lines = [
        "<?xml version='1.0' encoding='UTF-8'?>",
        "<osm version='0.6' generator='golfcart_board_initializer'>",
        "  <!-- Board polygon in the anchored map frame. Coordinates derive from",
        "       board dimensions and board_pose_in_map rather than survey. -->",
    ]
    for index, (x, y, z) in enumerate(corners, start=1):
        lines += [
            f"  <node id='{1000 + index}' visible='true'>",
            f"    <tag k='local_x' v='{x:.4f}'/>",
            f"    <tag k='local_y' v='{y:.4f}'/>",
            f"    <tag k='ele' v='{z:.4f}'/>",
            "  </node>",
        ]
    lines.append("  <way id='2001' visible='true'>")
    for index in range(1, 5):
        lines.append(f"    <nd ref='{1000 + index}'/>")
    lines += [
        "    <nd ref='1001'/>",
        "    <tag k='type' v='pose_marker'/>",
        "    <tag k='subtype' v='reflector'/>",
        f"    <tag k='marker_id' v='{marker_id}'/>",
        "    <tag k='area' v='yes'/>",
        "  </way>",
        "</osm>",
        "",
    ]
    return "\n".join(lines)


MAP_PROJECTOR_INFO = """\
# Indoor map: a local metric frame anchored to the reflective board, with no
# geodetic datum. Copying an outdoor map's TransverseMercator block here places
# the map in a coordinate system that does not exist indoors, and nothing warns.
projector_type: Local
vertical_datum: WGS84
"""


def transform_yaml(result: AnchorResult, source: str) -> str:
    """The anchoring transform, recorded so a rebuild can be checked against it."""
    rows = "\n".join(
        "    - [" + ", ".join(f"{v: .9f}" for v in row) + "]"
        for row in result.transform_map_cloud
    )
    return (
        "# Transform applied to the SLAM output to anchor it to the board.\n"
        "# The map frame is defined by this file: origin on the floor below the\n"
        "# board centre, +x along the board normal, +z up. A rebuild of the same\n"
        "# bag should reproduce it; if it does not, the maps are not comparable.\n"
        f"source_cloud: {source}\n"
        f"point_count: {result.n_points}\n"
        "board_in_source_frame:\n"
        f"  centre: [{', '.join(f'{v:.6f}' for v in result.board_centre_cloud)}]\n"
        f"  normal: [{', '.join(f'{v:.6f}' for v in result.board_normal_cloud)}]\n"
        f"floor_tilt_deg: {result.floor_tilt_deg:.3f}\n"
        "detection:\n"
        f"  points: {result.detection.n_points}\n"
        f"  extents: [{result.detection.extents[0]:.3f}, "
        f"{result.detection.extents[1]:.3f}]\n"
        f"  plane_residual_m: {result.detection.plane_residual:.4f}\n"
        "transform_map_cloud:\n"
        f"{rows}\n"
    )
