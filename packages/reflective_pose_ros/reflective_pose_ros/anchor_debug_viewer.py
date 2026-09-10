"""Replay an offline anchoring debug dump onto the live node's debug topics.

    $ anchor-map-to-board cloud.pcd -o map/ --dump-debug /tmp/anchor.npz
    $ ros2 run reflective_pose_ros anchor_debug_viewer /tmp/anchor.npz

``reflective_pose_cli`` is ROS-free, so it cannot draw its own failure. It
writes what it saw to an ``.npz`` instead, and this node draws it -- through the
same ``debug_viz`` builders the runtime node uses, so a map-cloud debug run and
a live one are the same picture with a different source. Two steps instead of
one, in exchange for a CLI that installs and runs with no ROS present.

Published, all latched (transient-local, depth 1), under this node's namespace:

    ~/debug/map_cloud     the levelled cloud with intensity, as context
    ~/debug/board_points  the accepted board's points, or every ambiguous one
    ~/debug/rejected      one text marker per rejected cluster, with its reason
    ~/debug/board_pose    the detection pose, when the dump has one

Open ``rviz/anchor_debug.rviz`` and set Fixed Frame to the dump's frame.

The file layout
---------------

``numpy.savez`` holds flat arrays, so nested records are spread across keys.
Everything except the cloud is optional: a dump from a failed detection has no
detection, which is the case this tool exists for.

    levelled            (N, 3) float, the levelled cloud
                        also accepted: levelled_cloud, cloud, points
    intensity           (N,)   float, per-point intensity
    frame_id            ()     str, default "map_debug"
    status              ()     str, a Status name or value ("ok",
                        "no_candidate", "ambiguous"); inferred from what else
                        is present when absent
    n_after_gates       ()     int, points that passed the map AABB (if enabled)
                               and stage-1 gates
    n_clusters          ()     int, clusters formed
    aabb_enabled        ()     bool, whether map-only cropping was enabled
    aabb_frame          ()     str, normally ``map_debug``
    aabb_min, aabb_max  (3,)   float, inclusive bounds; NaN when disabled or
                               unbounded on an axis
    n_inside_aabb       ()     int, points inside the crop (or all points)

    detection_points    (M, 3) float, the accepted cluster
    detection_centre    (3,)   float
    detection_rotation  (3, 3) float, board axes as columns in the cloud frame
    detection_normal    (3,)   float, unit, pointing back at the viewpoint
    detection_up        (3,)   float, unit
    detection_right     (3,)   float, unit
    detection_extents   (2,)   float, observed (width, height)
    detection_n_points  ()     int
    detection_range_m   ()     float
    detection_plane_residual  () float
    detection_observed_edges  (4,) bool, in the order left, right, bottom, top

    candidate_count     ()     int, K
    candidate_<i>_points, _centre, _rotation, _normal, _up, _right, _extents,
    _n_points, _range_m, _plane_residual, _observed_edges
                               the same fields, per surviving candidate, for
                               i in 0..K-1. Only ambiguous dumps carry these.

    rejection_reason    (K,)   str
    rejection_detail    (K,)   str
    rejection_centroid  (K, 3) float
    rejection_n_points  (K,)   int

If the writer's layout differs, this module is the only place that has to
change: everything below ``load_dump`` works on ``reflective_pose_core``
dataclasses, not on the file.
"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import PoseStamped
from reflective_pose_core.detector import BoardDetection, DetectResult, Rejection, Status

from .debug_viz import (
    board_pose_stamped,
    detection_points_cloud,
    rejection_marker_array,
    xyzi_cloud,
)

DEFAULT_FRAME = "map_debug"
CLOUD_KEYS = ("cloud_points", "levelled", "levelled_cloud", "cloud", "points")
INTENSITY_KEYS = ("cloud_intensity", "intensity")
DUMP_FORMAT = "reflective_pose_anchor_debug"
DUMP_VERSION = 1
EDGE_ORDER = ("left", "right", "bottom", "top")


def _scalar(data, key, default=None):
    if key not in data:
        return default
    value = data[key]
    return value.item() if getattr(value, "shape", ()) == () else value


def _text(data, key, default=""):
    value = _scalar(data, key, default)
    return value.decode() if isinstance(value, bytes) else str(value)


def _detection_from(data, prefix: str):
    """One ``BoardDetection`` out of ``<prefix>_*`` keys, or None if absent.

    Missing fields are filled with values that are honest about not being
    known: no observed edge claimed, zero residual, a centre-derived range.
    The debug drawing only reads centre, normal, rotation and points, so a
    partial dump still draws correctly.
    """
    points_key = f"{prefix}_points"
    centre_key = f"{prefix}_centre"
    if points_key not in data and centre_key not in data:
        return None

    points = np.asarray(data[points_key], dtype=float) if points_key in data else np.zeros((0, 3))
    centre = (
        np.asarray(data[centre_key], dtype=float)
        if centre_key in data
        else points.mean(axis=0) if len(points) else np.zeros(3)
    )
    rotation = (
        np.asarray(data[f"{prefix}_rotation"], dtype=float)
        if f"{prefix}_rotation" in data
        else np.eye(3)
    )
    normal = (
        np.asarray(data[f"{prefix}_normal"], dtype=float)
        if f"{prefix}_normal" in data
        else rotation[:, 0]
    )
    up = (
        np.asarray(data[f"{prefix}_up"], dtype=float)
        if f"{prefix}_up" in data
        else rotation[:, 2]
    )
    right = (
        np.asarray(data[f"{prefix}_right"], dtype=float)
        if f"{prefix}_right" in data
        else rotation[:, 1]
    )
    extents = (
        tuple(float(v) for v in np.asarray(data[f"{prefix}_extents"]).reshape(-1)[:2])
        if f"{prefix}_extents" in data
        else (0.0, 0.0)
    )
    if f"{prefix}_observed_edges" in data:
        flags = np.asarray(data[f"{prefix}_observed_edges"]).reshape(-1)
        observed = {name: bool(flags[i]) for i, name in enumerate(EDGE_ORDER)}
    else:
        observed = {name: False for name in EDGE_ORDER}

    return BoardDetection(
        centre=centre,
        rotation=rotation,
        normal=normal,
        up=up,
        right=right,
        extents=extents,
        points=points,
        n_points=int(_scalar(data, f"{prefix}_n_points", len(points))),
        plane_residual=float(_scalar(data, f"{prefix}_plane_residual", 0.0)),
        range_m=float(_scalar(data, f"{prefix}_range_m", float(np.linalg.norm(centre)))),
        observed_edges=observed,
    )


def _rejections_from(data):
    if "rejection_reason" not in data:
        return []
    reasons = np.asarray(data["rejection_reason"]).reshape(-1)
    details = (
        np.asarray(data["rejection_detail"]).reshape(-1)
        if "rejection_detail" in data
        else [""] * len(reasons)
    )
    centroids = (
        np.asarray(data["rejection_centroid"], dtype=float).reshape(-1, 3)
        if "rejection_centroid" in data
        else np.zeros((len(reasons), 3))
    )
    counts = (
        np.asarray(data["rejection_n_points"]).reshape(-1)
        if "rejection_n_points" in data
        else np.zeros(len(reasons), dtype=int)
    )
    out = []
    for index, reason in enumerate(reasons):
        out.append(
            Rejection(
                reason=str(reason),
                centroid=centroids[index],
                n_points=int(counts[index]),
                detail=str(details[index]),
            )
        )
    return out


def _status_from(data, detection, candidates) -> Status:
    if "status" in data:
        raw = _text(data, "status").lower()
        for member in Status:
            if raw in (member.value, member.name.lower()):
                return member
    if len(candidates) > 1:
        return Status.AMBIGUOUS
    return Status.OK if detection is not None else Status.NO_CANDIDATE


def _candidates_from(data):
    """Rebuild the candidate list from the concatenated-with-offsets layout.

    The writer stores all candidate points in one ``candidate_points`` array and
    slices them with ``candidate_offsets`` (K+1 entries), rather than emitting
    ``candidate_0_points``, ``candidate_1_points`` and so on. That keeps the key
    set fixed no matter how many candidates an ambiguous run produced, which is
    the case this dump exists for.

    Per-candidate scalars are stored as parallel arrays under plural names
    (``candidate_centres``, ``candidate_extents``, ...).
    """
    if "candidate_offsets" not in data:
        return []

    offsets = np.asarray(data["candidate_offsets"]).reshape(-1).astype(int)
    count = max(len(offsets) - 1, 0)
    if count == 0:
        return []

    points_all = np.asarray(data["candidate_points"], dtype=float).reshape(-1, 3)
    centres = np.asarray(data["candidate_centres"], dtype=float).reshape(count, 3)
    normals = np.asarray(data["candidate_normals"], dtype=float).reshape(count, 3)
    extents = np.asarray(data["candidate_extents"], dtype=float).reshape(count, 2)
    n_points = np.asarray(data["candidate_n_points"]).reshape(-1).astype(int)
    range_m = np.asarray(data["candidate_range_m"], dtype=float).reshape(-1)
    residual = np.asarray(data["candidate_plane_residual"], dtype=float).reshape(-1)

    out = []
    for index in range(count):
        points = points_all[offsets[index]:offsets[index + 1]]
        normal = normals[index]
        # The writer does not store a per-candidate basis: only the accepted
        # detection carries one. The drawing reads centre, normal and points,
        # so a basis completed from the normal is enough and is honest about
        # being derived rather than measured.
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(up, normal)
        norm = np.linalg.norm(right)
        right = right / norm if norm > 1e-9 else np.array([0.0, 1.0, 0.0])
        up = np.cross(normal, right)
        rotation = np.column_stack((normal, right, up))
        out.append(
            BoardDetection(
                centre=centres[index],
                rotation=rotation,
                normal=normal,
                up=up,
                right=right,
                extents=(float(extents[index][0]), float(extents[index][1])),
                points=points,
                n_points=int(n_points[index]),
                plane_residual=float(residual[index]),
                range_m=float(range_m[index]),
                observed_edges={name: False for name in EDGE_ORDER},
            )
        )
    return out


def load_dump(path: str):
    """Read the ``.npz`` into ``(levelled, intensity, DetectResult, frame_id)``.

    The layout is the one ``reflective_pose_cli``'s ``--dump-debug`` writes:
    format ``reflective_pose_anchor_debug`` version 1, in which every key is
    always present and an absent value is an empty array or NaN. The older
    aliases are still accepted so a dump written by hand still loads.
    """
    data = np.load(path, allow_pickle=False)

    fmt = _text(data, "format", "")
    if fmt and fmt != DUMP_FORMAT:
        raise ValueError(f"{path}: unknown dump format {fmt!r}, expected {DUMP_FORMAT!r}")
    version = int(_scalar(data, "format_version", 1))
    if version != DUMP_VERSION:
        raise ValueError(
            f"{path}: dump format version {version}, this viewer reads {DUMP_VERSION}"
        )

    key = next((name for name in CLOUD_KEYS if name in data), None)
    if key is None:
        raise ValueError(
            f"{path}: no cloud in the dump; expected one of {list(CLOUD_KEYS)}, "
            f"found {sorted(data.files)}"
        )
    levelled = np.asarray(data[key], dtype=float).reshape(-1, 3)

    intensity_key = next((n for n in INTENSITY_KEYS if n in data), None)
    intensity = (
        np.asarray(data[intensity_key], dtype=float).reshape(-1)
        if intensity_key
        else np.zeros(len(levelled))
    )
    if len(intensity) != len(levelled):
        raise ValueError(
            f"{path}: intensity has {len(intensity)} values for "
            f"{len(levelled)} points"
        )

    detection = _detection_from(data, "detection")
    candidates = _candidates_from(data)

    result = DetectResult(
        status=_status_from(data, detection, candidates),
        detection=detection,
        rejections=_rejections_from(data),
        n_after_gates=int(_scalar(data, "n_after_gates", 0)),
        n_clusters=int(_scalar(data, "n_clusters", 0)),
        candidates=candidates,
    )
    return levelled, intensity, result, _text(data, "frame_id", DEFAULT_FRAME)


class AnchorDebugViewer(Node):
    """Publish one dump, latched, and hold the process open for RViz."""

    def __init__(self, path: str, frame_id: str = None):
        super().__init__("anchor_debug_viewer")

        levelled, intensity, result, dump_frame = load_dump(path)
        frame = frame_id or dump_frame

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        map_pub = self.create_publisher(PointCloud2, "~/debug/map_cloud", latched)
        points_pub = self.create_publisher(PointCloud2, "~/debug/board_points", latched)
        rejected_pub = self.create_publisher(MarkerArray, "~/debug/rejected", latched)
        pose_pub = self.create_publisher(PoseStamped, "~/debug/board_pose", latched)

        stamp = self.get_clock().now().to_msg()
        map_pub.publish(xyzi_cloud(levelled, intensity, frame, stamp))
        points_pub.publish(detection_points_cloud(result, frame, stamp))
        rejected_pub.publish(rejection_marker_array(result, frame, stamp))
        if result.detection is not None:
            pose_pub.publish(board_pose_stamped(result.detection, frame, stamp))

        self.get_logger().info(
            f"{path}: {result.status.value}, {len(levelled)} points, "
            f"{len(result.rejections)} rejected cluster(s), "
            f"{len(result.candidates)} candidate(s); publishing on frame "
            f"'{frame}' under /anchor_debug_viewer/debug/*. Open "
            "rviz/anchor_debug.rviz and set Fixed Frame to match. Ctrl+C when done."
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="anchor_debug_viewer",
        description="Replay an anchor --dump-debug .npz onto the debug topics.",
    )
    parser.add_argument("dump", help="the .npz written by anchor-map-to-board --dump-debug")
    parser.add_argument(
        "--frame",
        default=None,
        help=(
            "override the frame_id in the dump (default: whatever it recorded, "
            f"else {DEFAULT_FRAME}); set RViz's Fixed Frame to match"
        ),
    )
    return parser


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    parsed = build_parser().parse_args(remove_ros_args(args=["_"] + list(argv))[1:])

    rclpy.init(args=args)
    try:
        node = AnchorDebugViewer(parsed.dump, parsed.frame)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        rclpy.shutdown()
        return 2

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
