"""ROS message builders for board-detection debug visualization.

Shared between the live node (``detector_node.py``) and the offline anchoring
dump viewer (``anchor_debug_viewer.py``), so the same per-cluster rejection
markers and detection overlays show up whether the source is a live accumulated
scan or a merged map cloud. See ``detector_node.py``'s ``_publish_clusters``
for why the debug topics are transient-local and cleared with DELETEALL on
every attempt: latched markers from a previous attempt would otherwise sit on
screen looking current.
"""

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from reflective_pose_core.detector import BoardDetection, DetectResult, Status
from reflective_pose_core.geometry import quaternion_from_matrix

REJECTED_COLOUR = (1.0, 0.6, 0.0)
AMBIGUOUS_COLOUR = (1.0, 0.0, 0.0)

XYZI_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def _header(frame_id: str, stamp) -> Header:
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    return header


def clear_all_marker() -> Marker:
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def text_marker(index, ns, position, text, colour, frame_id: str, stamp) -> Marker:
    marker = Marker()
    marker.header = _header(frame_id, stamp)
    marker.ns = ns
    marker.id = index
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.2
    marker.color.r, marker.color.g, marker.color.b = colour
    marker.color.a = 1.0
    marker.text = text
    return marker


def detection_arrow_marker(detection: BoardDetection, frame_id: str, stamp) -> Marker:
    marker = Marker()
    marker.header = _header(frame_id, stamp)
    marker.ns = "detection"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x, marker.scale.y, marker.scale.z = 0.05, 0.10, 0.0
    marker.color.g = 1.0
    marker.color.a = 1.0
    tip = detection.centre + 0.8 * detection.normal
    for point in (detection.centre, tip):
        target = Point()
        target.x, target.y, target.z = (float(v) for v in point)
        marker.points.append(target)
    return marker


def rejection_marker_array(result: DetectResult, frame_id: str, stamp) -> MarkerArray:
    """Label every cluster this attempt looked at: kept, ambiguous, or discarded.

    An ambiguous result draws every surviving candidate, not just the first: the
    map holds exactly one board, so a second survivor is the thing that needs
    explaining, not a runner-up to discard silently.
    """
    markers = MarkerArray()
    markers.markers.append(clear_all_marker())

    for index, rejection in enumerate(result.rejections):
        markers.markers.append(
            text_marker(
                index,
                "rejected",
                rejection.centroid,
                f"{rejection.reason} {rejection.detail} ({rejection.n_points} pts)",
                REJECTED_COLOUR,
                frame_id,
                stamp,
            )
        )

    if result.status is Status.OK:
        markers.markers.append(detection_arrow_marker(result.detection, frame_id, stamp))

    if result.status is Status.AMBIGUOUS:
        for index, candidate in enumerate(result.candidates):
            markers.markers.append(
                text_marker(
                    index,
                    "candidate",
                    candidate.centre,
                    f"AMBIGUOUS candidate {index + 1}: "
                    f"{candidate.range_m:.1f} m, {candidate.n_points} pts",
                    AMBIGUOUS_COLOUR,
                    frame_id,
                    stamp,
                )
            )

    return markers


def detection_points_cloud(result: DetectResult, frame_id: str, stamp) -> PointCloud2:
    """The accepted board's points, or every ambiguous candidate's, for overlay."""
    if result.status is Status.OK:
        points = result.detection.points
    elif result.status is Status.AMBIGUOUS:
        points = np.vstack([c.points for c in result.candidates])
    else:
        points = np.zeros((0, 3))
    return point_cloud2.create_cloud_xyz32(_header(frame_id, stamp), points.tolist())


NOMINAL_OUTLINE_COLOUR = (0.0, 1.0, 1.0)
SEEN_EDGE_COLOUR = (0.0, 1.0, 0.0)
UNSEEN_EDGE_COLOUR = (1.0, 0.0, 0.0)


def _point(vector) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(v) for v in vector)
    return point


def board_outline_marker_array(
    detection: BoardDetection, nominal_size, frame_id: str, stamp
) -> MarkerArray:
    """The detected board as two rectangles in its own plane, sensor frame.

    ``nominal``: ``nominal_size`` (the configured width and height) around the
    detected centre, one closed loop. It is the rectangle the published pose is
    composed from.

    ``measured``: the observed extents, one ``LINE_LIST`` per edge with the
    edge's name in ``text``, green where the detector saw that edge and red
    where it did not. A hidden edge is the one defect that biases the centre
    rather than merely widening the covariance, so it is the one worth seeing;
    a measured outline larger than the nominal one means a neighbour was
    clustered in.

    Edges follow the detector's own axes: ``left``/``right`` are the minimum
    and maximum along ``detection.right``, ``bottom``/``top`` along
    ``detection.up``, which is how ``_observed_edges`` names them.

    Starts with a clear-all: the topic is latched, and an outline from an
    earlier batch must not sit on screen looking current.
    """
    array = MarkerArray()
    array.markers.append(clear_all_marker())
    centre, right, up = detection.centre, detection.right, detection.up

    def corners(width, height):
        # bottom-left, bottom-right, top-right, top-left
        return [
            centre + su * width / 2 * right + sv * height / 2 * up
            for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]

    nominal = Marker()
    nominal.header = _header(frame_id, stamp)
    nominal.ns = "nominal"
    nominal.id = 0
    nominal.type = Marker.LINE_STRIP
    nominal.action = Marker.ADD
    nominal.pose.orientation.w = 1.0
    nominal.scale.x = 0.015
    nominal.color.r, nominal.color.g, nominal.color.b = NOMINAL_OUTLINE_COLOUR
    nominal.color.a = 1.0
    ring = corners(*nominal_size)
    nominal.points = [_point(c) for c in ring + ring[:1]]
    array.markers.append(nominal)

    bottom_left, bottom_right, top_right, top_left = corners(*detection.extents)
    edges = (
        ("bottom", bottom_left, bottom_right),
        ("right", bottom_right, top_right),
        ("top", top_right, top_left),
        ("left", top_left, bottom_left),
    )
    for index, (name, start, end) in enumerate(edges):
        marker = Marker()
        marker.header = _header(frame_id, stamp)
        marker.ns = "measured"
        marker.id = index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        seen = detection.observed_edges.get(name, False)
        colour = SEEN_EDGE_COLOUR if seen else UNSEEN_EDGE_COLOUR
        marker.color.r, marker.color.g, marker.color.b = colour
        marker.color.a = 1.0
        marker.text = name
        marker.points = [_point(start), _point(end)]
        array.markers.append(marker)
    return array


def board_pose_stamped(detection: BoardDetection, frame_id: str, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header = _header(frame_id, stamp)
    pose.pose.position.x = float(detection.centre[0])
    pose.pose.position.y = float(detection.centre[1])
    pose.pose.position.z = float(detection.centre[2])
    x, y, z, w = quaternion_from_matrix(detection.rotation)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def xyzi_cloud(points: np.ndarray, intensity: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Context cloud with intensity, for overlaying markers on the source scene.

    Used for the offline anchoring tool, which has no live sensor topic to
    provide this the way the runtime node's ``input_topic`` does.
    """
    data = np.column_stack((points, intensity)).astype(np.float32)
    return point_cloud2.create_cloud(_header(frame_id, stamp), XYZI_FIELDS, data.tolist())
