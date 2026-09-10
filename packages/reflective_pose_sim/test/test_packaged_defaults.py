"""The packaged detector file finds the board it describes.

The simulator's own scenes use the simulator's own board (0.8 x 1.0 m at
1.075 m, sensor at 1.6 m) against ``DetectorParams()``; this is the other
pairing, the packaged file against the decided board -- 0.6 x 0.6 m, centre
1.3 m, seen from the golf cart's 1.96 m sensor. Ten scans stacked, as the node
does. The packaged clustering tolerance used to be 0.05 m, which split this
board into ring stripes at every range: 0 detections in 235 batches of the
replay bag. This test would have caught that in the simulator.
"""

import numpy as np
import pytest

from reflective_pose_core.config import load_config
from reflective_pose_core.detector import Status, detect_board
from reflective_pose_sim import scenes, vlp32_sim

SENSOR_HEIGHT = 1.96
STACK = 10


def decided_board_scene(range_m: float):
    scene = scenes.make_room(sensor_height=SENSOR_HEIGHT)
    board = scenes.place_board(
        range_m=range_m,
        centre_height=1.3,
        sensor_height=SENSOR_HEIGHT,
        width=0.6,
        height=0.6,
    )
    scene.add(board)
    scenes.add_distractors(scene, sensor_height=SENSOR_HEIGHT)
    return scene, board.pose()


def stacked_scan(scene):
    points, intensity = [], []
    for seed in range(STACK):
        scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=seed))
        points.append(scan.points)
        intensity.append(scan.intensity)
    return np.vstack(points), np.concatenate(intensity)


@pytest.mark.parametrize("range_m", [6.0, 8.0, 10.0])
def test_packaged_file_detects_the_decided_board(range_m, monkeypatch):
    monkeypatch.delenv("REFLECTIVE_POSE_CONFIG", raising=False)
    config = load_config(scan_count=STACK)
    scene, truth = decided_board_scene(range_m)
    points, intensity = stacked_scan(scene)

    result = detect_board(
        points, intensity, scenes.transform_base_sensor(SENSOR_HEIGHT), config.detector
    )

    assert result.status is Status.OK, [
        (r.reason, r.detail) for r in result.rejections
    ]
    assert np.linalg.norm(result.detection.centre - truth[:3, 3]) < 0.15
    assert result.detection.confidence >= config.detector.min_confidence
