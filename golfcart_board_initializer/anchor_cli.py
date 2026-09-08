"""Command line front end for the map anchoring tool.

    anchor_map_to_board glim_export.ply -o data/huaxia-indoor

Writes the anchored cloud, the transform that produced it, the board's Lanelet2
polygon, and a map_projector_info.yaml declaring a local frame.
"""

import argparse
import os
import sys

from .anchor import (
    MAP_PROJECTOR_INFO,
    AnchorParams,
    anchor_cloud,
    anchored_board_centre,
    apply_transform,
    board_polygon_osm,
    transform_yaml,
)
from .config import default_config_path, load_anchor_config
from .pointcloud_io import read_cloud, write_pcd


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="anchor_map_to_board",
        description="Anchor a SLAM cloud to the retroreflective board.",
    )
    parser.add_argument("cloud", help="input .ply or .pcd from the SLAM run")
    parser.add_argument(
        "-o", "--output-dir", required=True, help="map directory to write"
    )
    parser.add_argument(
        "--name", default="pointcloud_map.pcd", help="anchored cloud filename"
    )
    parser.add_argument(
        "--config",
        default=default_config_path(),
        help="shared board_initializer.param.yaml (default: package config)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report the transform without writing anything",
    )
    parser.add_argument(
        "--rviz",
        action="store_true",
        help=(
            "publish the map cloud, board points, and per-cluster rejection "
            "markers on ROS topics and hold the process open for RViz "
            "inspection (rviz/anchor_debug.rviz); Ctrl+C to exit. Also works "
            "when detection fails, which is the case it's for."
        ),
    )
    parser.add_argument(
        "--rviz-frame",
        default="map_debug",
        help=(
            "frame_id for the topics --rviz publishes (default: map_debug); "
            "set RViz's Fixed Frame to match"
        ),
    )
    return parser


def _print_rejections(result, stream=None):
    """Per-cluster detail for a failed or ambiguous attempt.

    ``anchor_cloud``'s exception message is one flattened line so it stays
    ROS-free and testable; a real triage needs each rejected cluster against
    its own centroid, not a paragraph of concatenated reasons.

    ``stream`` defaults to ``sys.stderr`` resolved at call time, not import
    time: a default argument is evaluated once when the module loads, which
    would bind the real stderr object before a test's ``capsys`` fixture ever
    gets to swap it in.
    """
    stream = stream if stream is not None else sys.stderr
    print(
        f"  {result.n_after_gates} point(s) passed the intensity/range/height "
        f"gates, {result.n_clusters} cluster(s) formed, "
        f"{len(result.candidates)} survived every gate",
        file=stream,
    )
    if result.candidates:
        print("  surviving candidates:", file=stream)
        for index, candidate in enumerate(result.candidates, start=1):
            c = candidate.centre
            print(
                f"    {index}. range={candidate.range_m:.2f} m "
                f"n={candidate.n_points} "
                f"extents={candidate.extents[0]:.2f}x{candidate.extents[1]:.2f} "
                f"centre=({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})",
                file=stream,
            )
    if not result.rejections:
        return
    print("  rejected clusters:", file=stream)
    for index, rejection in enumerate(result.rejections, start=1):
        c = rejection.centroid
        print(
            f"    {index:3d}. {rejection.reason:16s} {rejection.detail:16s} "
            f"n={rejection.n_points:4d}  "
            f"centroid=({c[0]:7.2f}, {c[1]:7.2f}, {c[2]:7.2f})",
            file=stream,
        )


def _run_rviz_debug(levelled, intensity, result, frame_id: str):
    """Publish debug topics and hold the process open for RViz inspection.

    Reuses debug_viz.py so a map-cloud debug run draws identically to the live
    node's ~/debug/* topics: same colours, same DELETEALL-then-redraw markers,
    same per-cluster rejection text. Node name and topics are distinct from the
    runtime node's ("board_pose_initializer") so this can, in principle, run
    alongside it without clashing.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import MarkerArray

    from .debug_viz import detection_points_cloud, rejection_marker_array, xyzi_cloud

    rclpy.init(args=[])
    node = Node("anchor_map_to_board")
    try:
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        map_pub = node.create_publisher(PointCloud2, "~/debug/map_cloud", latched)
        points_pub = node.create_publisher(PointCloud2, "~/debug/board_points", latched)
        rejected_pub = node.create_publisher(MarkerArray, "~/debug/rejected", latched)

        stamp = node.get_clock().now().to_msg()
        map_pub.publish(xyzi_cloud(levelled, intensity, frame_id, stamp))
        points_pub.publish(detection_points_cloud(result, frame_id, stamp))
        rejected_pub.publish(rejection_marker_array(result, frame_id, stamp))

        print(
            f"publishing debug topics under /anchor_map_to_board on frame "
            f"'{frame_id}' — open RViz with rviz/anchor_debug.rviz (or point "
            "existing displays at the /anchor_map_to_board/debug/* topics and "
            f"set Fixed Frame to '{frame_id}'). Ctrl+C here when done.",
            file=sys.stderr,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    try:
        params, detector_params = load_anchor_config(args.config)
    except (OSError, ValueError) as error:
        print(f"error: cannot load config {args.config}: {error}", file=sys.stderr)
        return 2

    cloud = read_cloud(args.cloud)
    print(f"read {len(cloud)} points from {args.cloud}")
    if not cloud.has_intensity:
        print(
            "error: no intensity channel. The board cannot be found without it — "
            "check that the PLY-to-PCD conversion preserved the field.",
            file=sys.stderr,
        )
        return 2

    captured = {}

    def _capture(levelled, intensity, detect_result, viewpoint):
        captured["levelled"] = levelled
        captured["intensity"] = intensity
        captured["result"] = detect_result

    try:
        result = anchor_cloud(cloud, params, detector_params, on_result=_capture)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        if "result" in captured:
            _print_rejections(captured["result"])
        if args.rviz and captured:
            _run_rviz_debug(
                captured["levelled"], captured["intensity"], captured["result"],
                args.rviz_frame,
            )
        return 1

    detection = result.detection
    print(
        f"board found: {detection.n_points} points, "
        f"extents {detection.extents[0]:.2f} x {detection.extents[1]:.2f} m, "
        f"plane residual {detection.plane_residual * 100:.1f} cm"
    )
    print(f"floor tilt in the source frame: {result.floor_tilt_deg:.2f} deg")
    print("transform (map <- cloud):")
    for row in result.transform_map_cloud:
        print("  " + "  ".join(f"{v: 9.5f}" for v in row))

    anchored = apply_transform(cloud, result.transform_map_cloud)
    moved = anchored_board_centre(result)
    print(
        "board centre after anchoring: "
        f"[{moved[0]: .4f}, {moved[1]: .4f}, {moved[2]: .4f}] "
        f"(expected [{params.board_pose_in_map[0]}, {params.board_pose_in_map[1]}, "
        f"{params.board_pose_in_map[2]}])"
    )

    expected = params.board_pose_in_map[:3]
    if not all(abs(actual - wanted) <= 1e-3 for actual, wanted in zip(moved, expected)):
        print(
            f"error: anchoring did not place board at configured pose ({moved})",
            file=sys.stderr,
        )
        return 3

    if args.dry_run:
        print("dry run: nothing written")
        if args.rviz:
            _run_rviz_debug(
                captured["levelled"], captured["intensity"], captured["result"],
                args.rviz_frame,
            )
        return 0

    os.makedirs(args.output_dir, exist_ok=True)
    cloud_path = os.path.join(args.output_dir, args.name)
    write_pcd(cloud_path, anchored)

    with open(os.path.join(args.output_dir, "map_projector_info.yaml"), "w") as handle:
        handle.write(MAP_PROJECTOR_INFO)
    with open(os.path.join(args.output_dir, "board_anchor.yaml"), "w") as handle:
        handle.write(transform_yaml(result, os.path.abspath(args.cloud)))
    with open(os.path.join(args.output_dir, "board_polygon.osm"), "w") as handle:
        handle.write(board_polygon_osm(params))

    print(f"wrote {cloud_path}")
    print("wrote map_projector_info.yaml, board_anchor.yaml, board_polygon.osm")
    print(
        "next: merge board_polygon.osm into the route's lanelet2_map.osm, then "
        "tile the cloud with autoware_pointcloud_divider"
    )
    if args.rviz:
        _run_rviz_debug(
            captured["levelled"], captured["intensity"], captured["result"],
            args.rviz_frame,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
