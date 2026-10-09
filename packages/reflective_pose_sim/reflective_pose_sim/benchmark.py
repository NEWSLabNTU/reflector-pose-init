"""Per-scan cost of the detector on synthetic tracking scenes.

    python3 -m reflective_pose_sim.benchmark --sensor robin_w \\
        --config .../autosdv_robin_w.yaml

Times ``detect_board`` on single scans of AutoSDV's tracking scene (handheld
board, distractors, a hall) at a few ranges, and reports wall and process CPU
time per scan. Scans are simulated before timing starts, so only the detector
is measured; run it on the vehicle's computer for the number that matters, and
with ``OMP_NUM_THREADS=1`` for the single-core figure. ROS-free on purpose:
the node adds reading the PointCloud2 and, if ``publish_debug`` is on, the
debug messages.
"""

import argparse
import time
from typing import Dict, List, Sequence

import numpy as np

from reflective_pose_core.config import load_config
from reflective_pose_core.detector import Status, detect_board
from reflective_pose_sim import scenes, vlp32_sim


def run(
    sensor: str,
    config_path: str,
    ranges: Sequence[float],
    scans: int,
    centre_height: float,
) -> List[Dict[str, float]]:
    """One row per range: points per scan, detection rate, timings in ms."""
    config = load_config(config_path, scan_count=1)
    params = config.runtime_detector
    offset = config.detector.runtime.base_link_height_above_ground
    transform = scenes.transform_base_sensor(scenes.AUTOSDV_SENSOR_HEIGHT - offset, sensor)

    rows = []
    for range_m in ranges:
        scene, _ = scenes.tracking_board_scene(range_m=range_m, centre_height=centre_height)
        clouds = [
            vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor=sensor, seed=seed))
            for seed in range(scans)
        ]
        # Warm-up: first-call imports and allocations are not per-scan cost.
        detect_board(clouds[0].points, clouds[0].intensity, transform, params, height_offset=offset)

        wall, cpu, hits = [], [], 0
        for scan in clouds:
            w0, c0 = time.perf_counter(), time.process_time()
            result = detect_board(
                scan.points, scan.intensity, transform, params, height_offset=offset
            )
            cpu.append(time.process_time() - c0)
            wall.append(time.perf_counter() - w0)
            hits += int(
                result.status is Status.OK
                and result.detection.confidence >= params.min_confidence
            )
        wall_ms = 1e3 * np.array(wall)
        rows.append(
            dict(
                range_m=range_m,
                points=float(np.mean([len(s.points) for s in clouds])),
                detected=hits / len(clouds),
                wall_median_ms=float(np.median(wall_ms)),
                wall_p95_ms=float(np.percentile(wall_ms, 95)),
                wall_max_ms=float(wall_ms.max()),
                cpu_mean_ms=float(1e3 * np.mean(cpu)),
            )
        )
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sensor", default="robin_w")
    parser.add_argument("--config", required=True, help="detector file (profile)")
    parser.add_argument("--ranges", type=float, nargs="+", default=[1.5, 2.0, 4.0, 8.0])
    parser.add_argument("--scans", type=int, default=20)
    parser.add_argument("--centre-height", type=float, default=scenes.HANDHELD_BOARD_CENTRE_HEIGHT)
    args = parser.parse_args(argv)

    rows = run(args.sensor, args.config, args.ranges, args.scans, args.centre_height)
    print(
        f"{'range':>6} {'points':>8} {'detected':>8} {'median':>8} {'p95':>8} "
        f"{'max':>8} {'cpu':>8}   (ms per scan)"
    )
    for row in rows:
        print(
            f"{row['range_m']:6.1f} {row['points']:8.0f} {row['detected']:8.0%} "
            f"{row['wall_median_ms']:8.2f} {row['wall_p95_ms']:8.2f} "
            f"{row['wall_max_ms']:8.2f} {row['cpu_mean_ms']:8.2f}"
        )


if __name__ == "__main__":
    main()
