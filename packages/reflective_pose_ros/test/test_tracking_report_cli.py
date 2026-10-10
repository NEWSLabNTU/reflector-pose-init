"""board_tracking_report end to end: a synthetic bag in, the report out."""

import numpy as np
import pytest

from reflective_pose_core.detector import Status
from reflective_pose_core.tracking_report import outcome_of

rosbag2_py = pytest.importorskip("rosbag2_py")

from reflective_pose_ros import tracking_report_cli as cli  # noqa: E402
from reflective_pose_ros.decision import judge  # noqa: E402


def test_chain_composes_and_inverts_static_transforms():
    a = np.eye(4)
    a[:3, 3] = [1.0, 0.0, 0.0]
    b = np.eye(4)
    b[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    tf = {("base_link", "kit"): a, ("kit", "lidar"): b}
    assert np.allclose(cli.chain(tf, "base_link", "lidar"), a @ b)
    assert np.allclose(cli.chain(tf, "lidar", "base_link"), np.linalg.inv(a @ b))
    assert cli.chain(tf, "base_link", "camera") is None


def test_report_on_a_synthetic_bag(tmp_path, capsys):
    bag = str(tmp_path / "board")
    assert cli.main([bag, "--synthesize", "--lidar", "vlp16", "--ranges", "2,4",
                     "--scans", "3", "--centre-height", "0.55"]) == 0
    capsys.readouterr()
    assert cli.main([bag, "--lidar", "vlp16"]) == 0
    out = capsys.readouterr().out
    assert "base_link <- velodyne from /tf_static" in out
    assert "scans 6, detections 6 (100 %)" in out
    assert "1.5-2.5 m" in out and "3.5-4.5 m" in out
    assert "suggested" in out


def test_outcome_matches_the_nodes_verdict():
    """The report counts a detection exactly when the node would publish it."""
    from reflective_pose_core.config import load_config
    from reflective_pose_core.detector import detect_board
    from reflective_pose_sim import scenes, vlp32_sim

    config = load_config(cli.resolve_profile("autosdv_vlp16"), scan_count=1)
    params = config.runtime_detector
    for range_m, height in ((4.0, 0.55), (2.0, 1.0)):
        scene, _ = scenes.tracking_board_scene(range_m=range_m, centre_height=height)
        scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(sensor="vlp16", seed=1))
        result = detect_board(scan.points, scan.intensity,
                              scenes.transform_base_sensor(0.0, "vlp16"), params,
                              height_offset=config.detector.runtime.base_link_height_above_ground)
        published, _ = outcome_of(result, params.min_confidence)
        assert published == judge(result, params.min_confidence).publish
        assert published == (range_m == 4.0) or result.status is not Status.OK
