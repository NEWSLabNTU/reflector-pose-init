"""board_tracking_report: replay a bag's LiDAR scans through tracking mode, offline.

    ros2 run reflective_pose_ros board_tracking_report BAG --lidar vlp16
    ros2 run reflective_pose_ros board_tracking_report BAG --profile /path/to/profile.yaml

Reads the cloud topic and /tf_static from a rosbag2 directory, runs the
profile's detector on every scan as ``board_tracking_node`` would, and prints
``reflective_pose_core.tracking_report``: detection rate per range bin with the
gate that rejected each miss, detection time per scan, the board's intensity
against the background with a suggested ``intensity_threshold``, and the
detected board-centre height against the profile's centre-height gate.

Record the bag with the board held still at several distances (2, 4, 6, 8 m),
a few seconds at each, and the car parked. ``--synthesize OUT`` writes such a
bag from ``reflective_pose_sim`` instead, which is how this tool is checked.

``--lidar`` names AutoSDV's two lab LiDARs and picks both the profile
(``autosdv_vlp16`` / ``autosdv_robin_w``) and the default topic; ``--profile``
takes any detector file, by path or by name in reflective_pose_core's data.
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from reflective_pose_core.config import default_config_path, load_config
from reflective_pose_core.geometry import make_transform, matrix_from_quaternion
from reflective_pose_core.sensors import sensor_model
from reflective_pose_core.tracking_report import TrackingReport, default_bins

LIDARS = {
    # --lidar: (profile, default cloud topic, sensor model, cloud frame)
    "vlp16": ("autosdv_vlp16", "/sensing/lidar/velodyne_points", "vlp16", "velodyne"),
    "robin-w": ("autosdv_robin_w", "/sensing/lidar/iv_points", "robin_w", "robin_w"),
}


def resolve_profile(name: str) -> str:
    if os.path.isfile(name):
        return name
    data = Path(default_config_path()).parent
    candidate = data / (name if name.endswith(".yaml") else f"{name}.yaml")
    if candidate.is_file():
        return str(candidate)
    raise SystemExit(f"no detector profile {name!r} (looked in {data})")


# -- bags -----------------------------------------------------------------------


def _reader(path: str):
    import rosbag2_py

    storage = "sqlite3"
    meta = os.path.join(path, "metadata.yaml")
    if os.path.isfile(meta):
        with open(meta) as handle:
            for line in handle:
                if "storage_identifier" in line:
                    storage = line.split(":", 1)[1].strip()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions("", ""))
    return reader


def static_transforms(path: str) -> Dict[Tuple[str, str], np.ndarray]:
    """{(parent, child): parent <- child} from /tf_static."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = _reader(path)
    topics = {t.name for t in reader.get_all_topics_and_types()}
    out: Dict[Tuple[str, str], np.ndarray] = {}
    if "/tf_static" not in topics:
        return out
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static"]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            r, t = tf.transform.rotation, tf.transform.translation
            out[(tf.header.frame_id.lstrip("/"), tf.child_frame_id.lstrip("/"))] = make_transform(
                matrix_from_quaternion([r.x, r.y, r.z, r.w]), [t.x, t.y, t.z])
    return out


def chain(transforms: Dict[Tuple[str, str], np.ndarray], target: str,
          source: str) -> Optional[np.ndarray]:
    """target <- source through the static tree, in either direction per edge."""
    graph: Dict[str, List[Tuple[str, np.ndarray]]] = {}
    for (parent, child), m in transforms.items():
        graph.setdefault(child, []).append((parent, m))                 # child -> parent
        graph.setdefault(parent, []).append((child, np.linalg.inv(m)))  # parent -> child
    # BFS from source; carry target-less "frame <- source".
    seen = {source: np.eye(4)}
    queue = [source]
    while queue:
        frame = queue.pop(0)
        if frame == target:
            return seen[frame]
        for nxt, m in graph.get(frame, []):
            if nxt not in seen:
                seen[nxt] = m @ seen[frame]
                queue.append(nxt)
    return None


def iterate_clouds(path: str, topic: str, start: Optional[float], end: Optional[float],
                   max_scans: Optional[int]):
    """(t from the first scan, frame_id, points, intensity, read_ms) per scan."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    from .detector_node import BoardDetectorNode

    reader = _reader(path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t0 = None
    n = 0
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        started = time.perf_counter()
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec or t_ns * 1e-9
        if t0 is None:
            t0 = stamp
        t = stamp - t0
        if start is not None and t < start:
            continue
        if end is not None and t > end:
            break
        points, intensity = BoardDetectorNode._read_cloud(msg)
        read_ms = 1e3 * (time.perf_counter() - started)
        yield t, msg.header.frame_id.lstrip("/"), points, intensity, read_ms
        n += 1
        if max_scans and n >= max_scans:
            break


# -- synthetic bags ---------------------------------------------------------------


def synthesize(out: str, lidar: str, ranges: List[float], scans: int, centre_height: float,
               bearing: float = 5.0, rate: float = 10.0, gap: float = 3.0,
               distractors: bool = True, seed: int = 0) -> None:
    """A bag of a board held still at each range in turn: the cloud topic, as
    the driver frames it, plus /tf_static with AutoSDV's base_link <- LiDAR
    (the LiDAR at base_link, z = 0, as the sensor kit calibration has it)."""
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage

    from reflective_pose_core.geometry import quaternion_from_matrix
    from reflective_pose_sim import scenes, vlp32_sim

    _, topic, sensor, frame = LIDARS[lidar]
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=topic, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name="/tf_static", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))

    def stamp(t):
        s = Time()
        s.sec = int(t)
        s.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
        return s

    t0 = 1.7e9
    base_from_cloud = scenes.transform_base_sensor(0.0, sensor)
    tf = TransformStamped()
    tf.header.stamp = stamp(t0)
    tf.header.frame_id = "base_link"
    tf.child_frame_id = frame
    x, y, z, w = quaternion_from_matrix(base_from_cloud[:3, :3])
    tf.transform.rotation.x, tf.transform.rotation.y = x, y
    tf.transform.rotation.z, tf.transform.rotation.w = z, w
    writer.write("/tf_static", serialize_message(TFMessage(transforms=[tf])), int(t0 * 1e9))

    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(("x", "y", "z", "intensity"))]
    t = t0
    k = 0
    for range_m in ranges:
        for _ in range(scans):
            scene, _ = scenes.tracking_board_scene(
                range_m=range_m, bearing_deg=bearing, centre_height=centre_height,
                with_distractors=distractors)
            scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor=sensor, seed=seed + k))
            header = Header(stamp=stamp(t), frame_id=frame)
            cloud = point_cloud2.create_cloud(
                header, fields,
                np.column_stack((scan.points, scan.intensity)).astype(np.float32).tolist())
            writer.write(topic, serialize_message(cloud), int(t * 1e9))
            t += 1.0 / rate
            k += 1
        t += gap
    del writer


# -- main -------------------------------------------------------------------------


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="board_tracking_report",
        description="Run tracking mode offline on a bag's LiDAR scans: detection rate per "
                    "range, time per scan, board vs background intensity (and a suggested "
                    "intensity_threshold), board centre height.")
    ap.add_argument("bag", help="rosbag2 directory (or the output path with --synthesize)")
    ap.add_argument("--lidar", choices=sorted(LIDARS), help="AutoSDV lab LiDAR: profile + topic")
    ap.add_argument("--profile",
                    help="detector file, a path or a name in reflective_pose_core/data")
    ap.add_argument("--topic", help="PointCloud2 topic [per --lidar, else the bag's only cloud]")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--min-confidence", type=float, help="override the profile's")
    ap.add_argument("--intensity-threshold", type=float,
                    help="override the profile's, to see what another threshold does")
    ap.add_argument("--bin-width", type=float, default=1.0, help="range bin width [1 m]")
    ap.add_argument("--start", type=float, help="skip scans before START s")
    ap.add_argument("--end", type=float, help="stop after END s")
    ap.add_argument("--max-scans", type=int)
    ap.add_argument("--synthesize", action="store_true",
                    help="write a synthetic bag to BAG instead (reflective_pose_sim)")
    ap.add_argument("--ranges", default="2,4,6,8", help="--synthesize: board ranges, m")
    ap.add_argument("--scans", type=int, default=20, help="--synthesize: scans per range")
    ap.add_argument("--centre-height", type=float, default=0.55,
                    help="--synthesize: board centre above ground [0.55 m, knee]")
    ap.add_argument("--no-distractors", action="store_true")
    args = ap.parse_args(argv)

    if args.synthesize:
        if not args.lidar:
            ap.error("--synthesize needs --lidar")
        ranges = [float(r) for r in args.ranges.split(",") if r.strip()]
        synthesize(args.bag, args.lidar, ranges, args.scans, args.centre_height,
                   distractors=not args.no_distractors)
        print(f"wrote {args.bag}: {args.lidar}, board at {ranges} m, "
              f"{args.scans} scans each, centre {args.centre_height:.2f} m")
        return 0

    if not args.lidar and not args.profile:
        ap.error("give --lidar or --profile")
    profile = resolve_profile(args.profile or LIDARS[args.lidar][0])
    config = load_config(profile, scan_count=1)
    if args.intensity_threshold is not None:
        config.detector.intensity_threshold = args.intensity_threshold

    reader = _reader(args.bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    clouds = [n for n, t in types.items() if t == "sensor_msgs/msg/PointCloud2"]
    topic = args.topic or (LIDARS[args.lidar][1] if args.lidar else None)
    if topic not in types:
        if len(clouds) == 1:
            if topic:
                print(f"{topic} not in the bag; using {clouds[0]}", file=sys.stderr)
            topic = clouds[0]
        else:
            raise SystemExit(f"cloud topic {topic!r} not in the bag; PointCloud2 topics: {clouds}")

    transforms = static_transforms(args.bag)
    scans = iterate_clouds(args.bag, topic, args.start, args.end, args.max_scans)
    try:
        first = next(scans)
    except StopIteration:
        raise SystemExit(f"no scans on {topic}")
    frame = first[1]
    base_from_cloud = chain(transforms, args.base_frame, frame)
    tf_note = f"{args.base_frame} <- {frame} from /tf_static"
    if base_from_cloud is None:
        sensor = config.detector.sensor
        base_from_cloud = np.eye(4)
        base_from_cloud[:3, :3] = sensor_model(sensor).cloud_rotation.T
        tf_note = (f"NO {args.base_frame} <- {frame} in /tf_static: assuming the {sensor} "
                   f"axes at base_link, z = 0 (AutoSDV's calibration)")

    report = TrackingReport(config, base_from_cloud, min_confidence=args.min_confidence)
    read_ms = []
    for t, _, points, intensity, ms in [first] + list(scans):
        report.add_scan(t, points, intensity)
        read_ms.append(ms)

    p = config.runtime_detector
    label = (f"{args.bag}\nprofile {os.path.basename(profile)} (sensor {config.detector.sensor}, "
             f"intensity >= {p.intensity_threshold:.0f}, centre {p.board_centre_height:.2f} "
             f"+- {p.centre_height_tolerance:.2f} m, range {p.range_min:.1f}-{p.range_max:.1f} m)"
             f"\ntopic {topic}; {tf_note}")
    extra = {"decode time": f"{np.mean(read_ms):.1f} ms mean per scan (PointCloud2 -> numpy)"}
    print(report.format(default_bins(p.range_max, args.bin_width), extra, label))
    return 0


if __name__ == "__main__":
    sys.exit(main())
