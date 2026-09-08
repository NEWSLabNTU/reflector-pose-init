"""The anchoring tool end to end, on a stand-in for a GLIM export."""

import numpy as np

from golfcart_board_initializer.anchor import AnchorParams, anchor_cloud, anchored_board_centre
from golfcart_board_initializer.anchor_cli import main
from golfcart_board_initializer.geometry import make_transform
from golfcart_board_initializer.pointcloud_io import read_cloud

from test_anchor import BOARD_CENTRE_HEIGHT, build_map_cloud, rotation_z


def write_config(path, pose="[0.0, 0.0, 1.075, 0.0, 0.0, 0.0]"):
    path.write_text(
        f"""/**:
  ros__parameters:
    board_width: 0.8
    board_height: 1.0
    board_centre_height: {BOARD_CENTRE_HEIGHT}
    board_pose_in_map: {pose}
"""
    )


def write_glim_style_ply(path, cloud):
    """GLIM writes binary little-endian with the field named scalar_intensity."""
    values = np.column_stack((cloud.points, cloud.intensity)).astype(np.float32)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(cloud.points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float scalar_intensity\nend_header\n"
    )
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(values.tobytes())


def test_cli_writes_a_map_directory(tmp_path, capsys):
    # An arbitrary source frame, well away from the origin, as SLAM would give.
    cloud = build_map_cloud(
        source_pose=make_transform(rotation_z(2.4), [110.0, -55.0, 3.2])
    )
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "board_initializer.param.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)
    output = tmp_path / "indoor-map"

    assert main([str(source), "-o", str(output), "--config", str(config)]) == 0

    for name in (
        "pointcloud_map.pcd",
        "map_projector_info.yaml",
        "board_anchor.yaml",
        "board_polygon.osm",
    ):
        assert (output / name).exists(), name

    projector = (output / "map_projector_info.yaml").read_text()
    assert "projector_type: Local" in projector

    anchor = (output / "board_anchor.yaml").read_text()
    assert "transform_map_cloud:" in anchor
    assert "source_cloud:" in anchor

    printed = capsys.readouterr().out
    assert "board found" in printed


def test_written_map_is_already_anchored(tmp_path):
    """Re-anchoring the output must be a no-op — the map is its own check."""
    cloud = build_map_cloud(
        source_pose=make_transform(rotation_z(0.7), [-20.0, 8.0, 1.4])
    )
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "board_initializer.param.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)
    output = tmp_path / "indoor-map"
    assert main([str(source), "-o", str(output), "--config", str(config)]) == 0

    written = read_cloud(str(output / "pointcloud_map.pcd"))
    assert written.has_intensity

    again = anchor_cloud(written)
    centre = anchored_board_centre(again)
    assert abs(centre[0]) < 0.02
    assert abs(centre[1]) < 0.02
    assert abs(centre[2] - BOARD_CENTRE_HEIGHT) < 0.15
    assert np.allclose(again.transform_map_cloud, np.eye(4), atol=0.05)


def test_dry_run_writes_nothing(tmp_path):
    cloud = build_map_cloud()
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "board_initializer.param.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)
    output = tmp_path / "indoor-map"

    assert main([str(source), "-o", str(output), "--config", str(config), "--dry-run"]) == 0
    assert not output.exists()


def test_cloud_without_intensity_exits_nonzero(tmp_path, capsys):
    cloud = build_map_cloud()
    source = tmp_path / "plain.ply"
    config = tmp_path / "board_initializer.param.yaml"
    values = cloud.points.astype(np.float32)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(values)}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    with open(source, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(values.tobytes())
    write_config(config)

    assert main([str(source), "-o", str(tmp_path / "out"), "--config", str(config)]) == 2
    assert "no intensity" in capsys.readouterr().err


def test_distractor_only_map_exits_nonzero(tmp_path, capsys):
    from golfcart_board_initializer.pointcloud_io import PointCloud
    from golfcart_board_initializer.simulation import scenes, vlp32_sim

    scan = vlp32_sim.simulate(
        scenes.distractor_only_scene(), vlp32_sim.SimParams(seed=1)
    )
    cloud = PointCloud(
        points=scan.points + np.array([0.0, 0.0, scenes.SENSOR_HEIGHT]),
        intensity=scan.intensity,
    )
    source = tmp_path / "no_board.ply"
    config = tmp_path / "board_initializer.param.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)

    assert main([str(source), "-o", str(tmp_path / "out"), "--config", str(config)]) == 1
    err = capsys.readouterr().err
    assert "no board found" in err
    # The flattened one-line summary in the exception is not enough to triage
    # which cluster is which; each rejection must be listed against its own
    # centroid and reason.
    assert "rejected clusters:" in err
    assert "cluster(s) formed" in err


def test_cli_places_board_at_configured_translation_and_yaw(tmp_path):
    cloud = build_map_cloud()
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "board_initializer.param.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config, "[12.0, -4.0, 1.075, 0.0, 0.0, 1.57079632679]")
    output = tmp_path / "indoor-map"

    assert main([str(source), "-o", str(output), "--config", str(config)]) == 0
    anchored = read_cloud(str(output / "pointcloud_map.pcd"))
    result = anchor_cloud(
        anchored,
        params=AnchorParams(
            board_pose_in_map=(12.0, -4.0, 1.075, 0.0, 0.0, 1.57079632679)
        ),
    )
    assert np.allclose(result.transform_map_cloud, np.eye(4), atol=0.05)
