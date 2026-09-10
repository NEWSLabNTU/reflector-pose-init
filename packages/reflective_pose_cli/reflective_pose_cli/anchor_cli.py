"""Command line front end for the map anchoring tool.

    anchor-map-to-board glim_export.ply -o data/huaxia-indoor

Writes the anchored cloud, the transform that produced it, the board's Lanelet2
polygon, and a map_projector_info.yaml declaring a local frame.

This module — and this whole package — imports no ROS. The previous ``--rviz``
flag imported ``rclpy`` inside a function to publish debug topics, which kept
the *module* importable without ROS while making the *package* need ROS for its
most useful failure-path flag. It is replaced by ``--dump-debug PATH``, which
writes a numpy ``.npz``; ``reflective_pose_ros``'s ``anchor_debug_viewer``
replays that file onto the same topics through the same ``debug_viz`` code, so
the picture is unchanged.

Debug dump format
=================

``--dump-debug PATH`` writes an uncompressed-key, compressed-payload ``.npz``.
**Every key below is always present**, so a reader never has to test for a key's
existence — absence of data is spelled as an empty array or NaN, never as a
missing key. Shapes use N for cloud points, K for surviving candidates, R for
rejections, M for the total points across all candidates.

Scalar entries are 0-d arrays; read them with ``value.item()``.

Identity
--------
``format``              () unicode — always ``"reflective_pose_anchor_debug"``
``format_version``      () int32 — this document is version 1
``source_cloud``        () unicode — absolute path of the input cloud
``frame_id``            () unicode — frame the points are expressed in. A hint
                        carrying the old ``--rviz-frame`` default,
                        ``"map_debug"``; a viewer may use its own.
``status``              () unicode — ``"ok"`` | ``"no_candidate"`` |
                        ``"ambiguous"``, the ``DetectResult.status`` value
``error``               () unicode — the ``anchor_cloud`` failure message, or
                        ``""`` when anchoring succeeded

The cloud the detector actually saw
-----------------------------------
Gravity-levelled, floor at z = 0 — the frame every other array here is in, and
what the old ``~/debug/map_cloud`` topic carried.

``cloud_points``        (N, 3) float32
``cloud_intensity``     (N,)   float32
``viewpoint``           (3,)   float64 — the room-centre viewpoint that oriented
                        the board normal; the detector's stand-in for a sensor
                        origin, and the origin ``candidate_range_m`` measures from

Counts, as the old stderr summary printed them
----------------------------------------------
``n_after_gates``       () int32 — points passing intensity/range/height
``n_clusters``          () int32 — clusters formed
``n_candidates``        () int32 — K, clusters surviving every gate
``n_rejections``        () int32 — R

The accepted board (all NaN / empty when ``status != "ok"``)
------------------------------------------------------------
``detection_points``    (P, 3) float32 — the cluster itself
``detection_centre``    (3,)   float64 — bounding-rectangle centre
``detection_normal``    (3,)   float64 — unit, outward. The old arrow marker ran
                        from ``detection_centre`` to ``centre + 0.8 * normal``.
``detection_up``        (3,)   float64
``detection_right``     (3,)   float64
``detection_rotation``  (3, 3) float64 — columns are (normal, right, up)
``detection_extents``   (2,)   float64 — observed (width, height), metres
``detection_n_points``  () int32
``detection_plane_residual`` () float64 — RMS distance to the fitted plane
``detection_range_m``   () float64
``detection_edge_names``   (4,) unicode — ``["left", "right", "bottom", "top"]``
``detection_observed_edges`` (4,) bool — same order as the names

Every surviving candidate
-------------------------
K == 1 on success (identical to the detection block); K > 1 is exactly the
ambiguous case the old viewer drew a red label for, at each candidate's centre.

``candidate_points``    (M, 3) float32 — all candidates' points, concatenated
``candidate_offsets``   (K + 1,) int64 — candidate k is
                        ``candidate_points[offsets[k]:offsets[k + 1]]``
``candidate_centres``   (K, 3) float64
``candidate_normals``   (K, 3) float64
``candidate_extents``   (K, 2) float64
``candidate_n_points``  (K,)   int32
``candidate_range_m``   (K,)   float64
``candidate_plane_residual`` (K,) float64

Rejected clusters
-----------------
``rejection_id``        (R,) int32 — 0..R-1. This is the marker id the live
                        viewer used, i.e. the index within the rejection list.
                        It is **not** the detector's cluster ordinal: surviving
                        clusters are absent from this list.
``rejection_reason``    (R,) unicode — ``not_planar``, ``not_vertical``,
                        ``degenerate_up``, ``bad_width``, ``bad_height``,
                        ``bad_mount_height``, ``too_dense``
``rejection_detail``    (R,) unicode — the detector's own formatted measurement,
                        verbatim, so a label reads as it did live
``rejection_centroid``  (R, 3) float64 — where the label was drawn
``rejection_n_points``  (R,) int32
``rejection_measured``  (R,) float64 — the number parsed out of ``detail``, NaN
                        when the reason carries none (``degenerate_up``)
``rejection_expected``  (R,) float64 — the ideal value for that gate: nominal
                        width/height/mount height, 0.0 for a plane's thickness
                        and for ``|n.up|``, 1.0 for the density ratio
``rejection_limit_lo``  (R,) float64 — lower bound of the accepted band, NaN
                        when the gate is one-sided or has no number
``rejection_limit_hi``  (R,) float64 — upper bound, same convention

The rejected clusters' *points* are not here because the detector does not keep
them: ``Rejection`` carries a centroid and a count, and the live viewer drew
only the centroid label. Nothing the old picture contained is lost.

The gates that produced those verdicts
--------------------------------------
``param_names``         (T,) unicode — every numeric scalar of the
                        ``DetectorParams`` this run used, **after**
                        ``detector_params_for_map`` relaxed range and density
``param_values``        (T,) float64 — parallel to ``param_names``. Booleans are
                        0.0 / 1.0; ``range_max`` is ``inf`` for a map cloud.

The anchoring result (NaN when anchoring failed)
------------------------------------------------
``transform_map_cloud`` (4, 4) float64
``floor_tilt_deg``      () float64
"""

import argparse
import os
import re
import sys

import numpy as np

from reflective_pose_core.anchor import (
    MAP_PROJECTOR_INFO,
    anchor_cloud,
    anchored_board_centre,
    apply_transform,
    board_polygon_osm,
    detector_params_for_map,
    transform_yaml,
)
from reflective_pose_core.anchor import AnchorParams
from reflective_pose_core.config import anchor_params, default_config_path, load_config
from reflective_pose_core.detector import Status
from reflective_pose_core.pointcloud_io import read_cloud, write_pcd

DUMP_FORMAT = "reflective_pose_anchor_debug"
DUMP_FORMAT_VERSION = 1

# The frame the old --rviz path published on. Recorded in the dump so a viewer
# has a default that matches rviz/anchor_debug.rviz rather than inventing one.
DUMP_FRAME_ID = "map_debug"

EDGE_NAMES = ("left", "right", "bottom", "top")

_NUMBER = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="anchor-map-to-board",
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
        help=(
            "the detector file (board, detector gates, covariance); the same "
            "file the runtime node loads (default: package config)"
        ),
    )
    # The floor fit is a property of one run of one tool, so its knobs are
    # flags rather than config. The board and the gates stay in the file on
    # purpose: there is no flag that could move the board.
    floor = parser.add_argument_group("floor fit")
    defaults = AnchorParams()
    floor.add_argument(
        "--floor-band", type=float, default=defaults.floor_band, metavar="M",
        help="metres above the lowest points to fit the floor within",
    )
    floor.add_argument(
        "--floor-percentile", type=float, default=defaults.floor_percentile,
        metavar="PCT", help="percentile of z taken as 'the lowest points'",
    )
    floor.add_argument(
        "--floor-inlier", type=float, default=defaults.floor_inlier, metavar="M",
        help="refit tolerance, metres",
    )
    floor.add_argument(
        "--floor-refits", type=int, default=defaults.floor_refits, metavar="N",
        help="how many times the plane is refitted on its inliers",
    )
    floor.add_argument(
        "--max-floor-tilt-deg", type=float, default=defaults.max_floor_tilt_deg,
        metavar="DEG",
        help="refuse a cloud whose fitted floor tilts more than this from level",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report the transform without writing anything",
    )
    parser.add_argument(
        "--dump-debug",
        metavar="PATH",
        default=None,
        help=(
            "write a .npz holding the levelled cloud, the per-cluster point "
            "sets, and every rejected cluster with its measured and expected "
            "values. Written whether detection succeeded or failed — the "
            "failure is the case it's for. Replay it with 'ros2 run "
            "reflective_pose_ros anchor_debug_viewer PATH'; see this module's "
            "docstring for the key layout."
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
                f"confidence={candidate.confidence:.2f} "
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


def _measured_value(detail: str) -> float:
    """The number the detector formatted into a rejection's detail string.

    The detail is kept verbatim as well, so a label still reads exactly as it
    did live; this is the same quantity as a float, for a viewer that wants to
    draw a bar against its limit rather than print a sentence.
    """
    match = _NUMBER.search(detail or "")
    return float(match.group()) if match else float("nan")


def _gate_bounds(reason: str, params):
    """(ideal, accepted low, accepted high) for the gate that rejected a cluster.

    The ideal is what a real board would measure, not the threshold: a plane's
    thickness should be 0, a density ratio should be 1, a width should be the
    nominal board width. The pair after it is the band the gate accepted.
    """
    nan = float("nan")
    lo, hi = params.extent_tolerance
    table = {
        "not_planar": (0.0, 0.0, params.planarity_max_thickness),
        "not_vertical": (0.0, 0.0, params.verticality_max_dot),
        "bad_width": (
            params.board_width, lo * params.board_width, hi * params.board_width
        ),
        "bad_height": (
            params.board_height, lo * params.board_height, hi * params.board_height
        ),
        "bad_mount_height": (
            params.board_centre_height,
            params.board_centre_height - params.centre_height_tolerance,
            params.board_centre_height + params.centre_height_tolerance,
        ),
        "too_dense": (1.0, nan, params.density_max_ratio),
        "degenerate_up": (nan, nan, nan),
    }
    return table.get(reason, (nan, nan, nan))


def _numeric_params(params):
    """DetectorParams as a name/value table, for the expected side of a verdict.

    Only the numeric scalars: the elevation table and the viewpoint are arrays
    that belong to the sensor model rather than to a gate, and the viewpoint is
    dumped on its own.
    """
    names, values = [], []
    for name in (
        "intensity_threshold",
        "range_min",
        "range_max",
        "height_min",
        "height_max",
        "cluster_tolerance",
        "cluster_min_points",
        "board_width",
        "board_height",
        "board_centre_height",
        "planarity_max_thickness",
        "verticality_max_dot",
        "centre_height_tolerance",
        "density_max_ratio",
        "density_check_enabled",
        "azimuth_step_rad",
        "mean_elevation_step_rad",
        "scan_count",
        "edge_margin_scale",
    ):
        names.append(name)
        values.append(float(getattr(params, name)))
    for index, bound in enumerate(params.extent_tolerance):
        names.append(f"extent_tolerance_{index}")
        values.append(float(bound))
    return names, values


def _empty_detection_block():
    nan = float("nan")
    return {
        "detection_points": np.zeros((0, 3), dtype=np.float32),
        "detection_centre": np.full(3, nan),
        "detection_normal": np.full(3, nan),
        "detection_up": np.full(3, nan),
        "detection_right": np.full(3, nan),
        "detection_rotation": np.full((3, 3), nan),
        "detection_extents": np.full(2, nan),
        "detection_n_points": np.int32(0),
        "detection_plane_residual": np.float64(nan),
        "detection_range_m": np.float64(nan),
        "detection_edge_names": np.array(EDGE_NAMES),
        "detection_observed_edges": np.zeros(4, dtype=bool),
    }


def _detection_block(detection):
    return {
        "detection_points": np.asarray(detection.points, dtype=np.float32),
        "detection_centre": np.asarray(detection.centre, dtype=np.float64),
        "detection_normal": np.asarray(detection.normal, dtype=np.float64),
        "detection_up": np.asarray(detection.up, dtype=np.float64),
        "detection_right": np.asarray(detection.right, dtype=np.float64),
        "detection_rotation": np.asarray(detection.rotation, dtype=np.float64),
        "detection_extents": np.asarray(detection.extents, dtype=np.float64),
        "detection_n_points": np.int32(detection.n_points),
        "detection_plane_residual": np.float64(detection.plane_residual),
        "detection_range_m": np.float64(detection.range_m),
        "detection_edge_names": np.array(EDGE_NAMES),
        "detection_observed_edges": np.array(
            [bool(detection.observed_edges[name]) for name in EDGE_NAMES], dtype=bool
        ),
    }


def _candidate_block(candidates):
    if candidates:
        points = np.vstack([np.asarray(c.points, dtype=np.float32) for c in candidates])
        counts = [len(c.points) for c in candidates]
    else:
        points = np.zeros((0, 3), dtype=np.float32)
        counts = []
    offsets = np.concatenate(([0], np.cumsum(counts))).astype(np.int64)
    return {
        "candidate_points": points,
        "candidate_offsets": offsets,
        "candidate_centres": np.asarray(
            [c.centre for c in candidates], dtype=np.float64
        ).reshape(-1, 3),
        "candidate_normals": np.asarray(
            [c.normal for c in candidates], dtype=np.float64
        ).reshape(-1, 3),
        "candidate_extents": np.asarray(
            [c.extents for c in candidates], dtype=np.float64
        ).reshape(-1, 2),
        "candidate_n_points": np.asarray(
            [c.n_points for c in candidates], dtype=np.int32
        ),
        "candidate_range_m": np.asarray(
            [c.range_m for c in candidates], dtype=np.float64
        ),
        "candidate_plane_residual": np.asarray(
            [c.plane_residual for c in candidates], dtype=np.float64
        ),
    }


def _rejection_block(rejections, params):
    bounds = [_gate_bounds(r.reason, params) for r in rejections]
    return {
        "rejection_id": np.arange(len(rejections), dtype=np.int32),
        "rejection_reason": np.array([r.reason for r in rejections], dtype=np.str_),
        "rejection_detail": np.array([r.detail for r in rejections], dtype=np.str_),
        "rejection_centroid": np.asarray(
            [r.centroid for r in rejections], dtype=np.float64
        ).reshape(-1, 3),
        "rejection_n_points": np.asarray(
            [r.n_points for r in rejections], dtype=np.int32
        ),
        "rejection_measured": np.asarray(
            [_measured_value(r.detail) for r in rejections], dtype=np.float64
        ),
        "rejection_expected": np.asarray([b[0] for b in bounds], dtype=np.float64),
        "rejection_limit_lo": np.asarray([b[1] for b in bounds], dtype=np.float64),
        "rejection_limit_hi": np.asarray([b[2] for b in bounds], dtype=np.float64),
    }


def write_debug_dump(
    path, captured, detector_params, source_cloud, anchor_result=None, error=""
):
    """Write everything the old ``--rviz`` path drew, as a numpy ``.npz``.

    ``captured`` is what ``anchor_cloud``'s ``on_result`` hook handed back:
    the levelled cloud, its intensity, the ``DetectResult`` and the viewpoint.
    The hook fires before either outcome is decided, so this works on the
    failure path — which is the path the picture is wanted on.
    """
    result = captured["result"]
    # The gates the detector actually ran with: anchoring relaxes range and
    # density for a merged cloud, so dumping the file's values would show a
    # viewer limits that were never applied.
    params = detector_params_for_map(detector_params)

    names, values = _numeric_params(params)
    nan = float("nan")

    arrays = {
        "format": np.array(DUMP_FORMAT),
        "format_version": np.int32(DUMP_FORMAT_VERSION),
        "source_cloud": np.array(os.path.abspath(source_cloud)),
        "frame_id": np.array(DUMP_FRAME_ID),
        "status": np.array(result.status.value),
        "error": np.array(error or ""),
        "cloud_points": np.asarray(captured["levelled"], dtype=np.float32),
        "cloud_intensity": np.asarray(captured["intensity"], dtype=np.float32),
        "viewpoint": np.asarray(captured["viewpoint"], dtype=np.float64),
        "n_after_gates": np.int32(result.n_after_gates),
        "n_clusters": np.int32(result.n_clusters),
        "n_candidates": np.int32(len(result.candidates)),
        "n_rejections": np.int32(len(result.rejections)),
        "param_names": np.array(names, dtype=np.str_),
        "param_values": np.asarray(values, dtype=np.float64),
        "transform_map_cloud": (
            np.asarray(anchor_result.transform_map_cloud, dtype=np.float64)
            if anchor_result is not None
            else np.full((4, 4), nan)
        ),
        "floor_tilt_deg": np.float64(
            anchor_result.floor_tilt_deg if anchor_result is not None else nan
        ),
    }
    if result.status is Status.OK and result.detection is not None:
        arrays.update(_detection_block(result.detection))
    else:
        arrays.update(_empty_detection_block())
    arrays.update(_candidate_block(result.candidates))
    arrays.update(_rejection_block(result.rejections, params))

    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    np.savez_compressed(path, **arrays)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    try:
        config = load_config(args.config)
    except (OSError, ValueError) as error:
        print(f"error: cannot load config {args.config}: {error}", file=sys.stderr)
        return 2
    params = anchor_params(
        config,
        floor_band=args.floor_band,
        floor_percentile=args.floor_percentile,
        floor_inlier=args.floor_inlier,
        floor_refits=args.floor_refits,
        max_floor_tilt_deg=args.max_floor_tilt_deg,
    )
    detector_params = config.detector

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
        captured["viewpoint"] = viewpoint

    def _dump(anchor_result=None, error=""):
        """Write the debug file, if one was asked for and there is one to write.

        Anchoring can fail before detection ever runs — an untiltable floor, a
        cloud with fewer than three points — and then there is nothing to draw.
        Say so rather than writing a file whose emptiness looks like a verdict.
        """
        if not args.dump_debug:
            return
        if "result" not in captured:
            print(
                "warning: no detection was reached, so --dump-debug wrote "
                "nothing (the run failed before the detector ran)",
                file=sys.stderr,
            )
            return
        write_debug_dump(
            args.dump_debug, captured, detector_params, args.cloud,
            anchor_result=anchor_result, error=error,
        )
        print(f"wrote debug dump {args.dump_debug}", file=sys.stderr)

    try:
        result = anchor_cloud(cloud, params, detector_params, on_result=_capture)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        if "result" in captured:
            _print_rejections(captured["result"])
        _dump(error=str(error))
        return 1

    # Written before the checks below rather than after each of them: every exit
    # from here on is one a person would want the picture for, and one call site
    # cannot go out of step with the others.
    _dump(anchor_result=result)

    detection = result.detection
    terms = " ".join(f"{k}={v:.2f}" for k, v in detection.confidence_terms.items())
    print(
        f"board found: {detection.n_points} points, "
        f"extents {detection.extents[0]:.2f} x {detection.extents[1]:.2f} m, "
        f"plane residual {detection.plane_residual * 100:.1f} cm, "
        f"confidence {detection.confidence:.2f} ({terms})"
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
