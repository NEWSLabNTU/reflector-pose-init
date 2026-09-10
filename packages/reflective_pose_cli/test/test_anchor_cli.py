"""The anchoring tool end to end, on a stand-in for a GLIM export."""

import numpy as np
import pytest

from reflective_pose_core.anchor import (
    AnchorParams,
    anchor_cloud,
    anchored_board_centre,
)
from reflective_pose_core.geometry import make_transform
from reflective_pose_core.pointcloud_io import PointCloud, read_cloud
from reflective_pose_cli.anchor_cli import main
from reflective_pose_sim import scenes, vlp32_sim

BOARD_CENTRE_HEIGHT = scenes.BOARD_CENTRE_HEIGHT

# Sensor poses in the room frame: (x, y, heading). The board is fixed at the
# room origin facing +x, so each scan sees the same board from a different
# place — which is what makes the merged cloud a map rather than three boards.
SENSOR_POSES = [
    (-5.0, 0.0, 0.0),
    (-7.0, 2.5, np.radians(-18.0)),
    (-4.0, -1.5, np.radians(12.0)),
]


def rotation_z(angle):
    cos, sin = np.cos(angle), np.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def build_map_cloud(source_pose=None, with_distractors=True, seeds=(1, 2, 3)):
    """Merge scans taken from several sensor poses into one room frame.

    A copy of core's own map-cloud builder rather than an import of it: this
    package's tests depend on the ``reflective_pose_sim`` *package*, never on
    another package's test module. Reaching across for a fixture is what the
    old single-package ``conftest.py`` sys.path hack existed to allow, and it
    is what the split removes.

    Each scan is rendered in its own sensor frame — the scene builder places
    the board relative to the sensor — and then lifted into the room frame by
    the sensor's pose. Moving the board instead would put a separate board in
    the map for every viewpoint, which is a different scene, not a map.
    """
    points, intensity = [], []
    for seed, (sensor_x, sensor_y, heading) in zip(seeds, SENSOR_POSES):
        to_board = np.array([-sensor_x, -sensor_y])
        range_m = float(np.linalg.norm(to_board))
        bearing = np.arctan2(to_board[1], to_board[0]) - heading
        # The scene builder aims the board's normal at the sensor and then
        # rotates it by yaw_deg; the room-frame normal must come out as +x.
        yaw = -heading - bearing - np.pi

        scene, _ = scenes.board_scene(
            range_m=range_m,
            bearing_deg=float(np.degrees(bearing)),
            yaw_deg=float(np.degrees(yaw)),
            with_distractors=with_distractors,
        )
        scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=seed))

        rotation = rotation_z(heading)
        moved = scan.points @ rotation.T + np.array(
            [sensor_x, sensor_y, scenes.SENSOR_HEIGHT]
        )
        points.append(moved)
        intensity.append(scan.intensity)

    merged = np.vstack(points)
    merged_intensity = np.concatenate(intensity)

    if source_pose is not None:
        merged = merged @ source_pose[:3, :3].T + source_pose[:3, 3]

    return PointCloud(points=merged, intensity=merged_intensity)


def write_config(
    path, pose="[0.0, 0.0, 1.075, 0.0, 0.0, 0.0]", map_aabb=None
):
    """A minimal canonical config: the board section, and defaults elsewhere.

    Only ``board:`` is written because that is the section this tool's answer
    depends on. Omitting ``detector:`` leaves the map policy at its independent
    defaults, which is what a bare ``anchor_cloud(cloud)`` uses — so a test
    that goes through the CLI and one that does not are comparing like with
    like.
    """
    map_section = ""
    if map_aabb is not None:
        minimum, maximum = map_aabb
        map_section = f"""detector:
  map:
    aabb:
      min: [{', '.join(str(value) for value in minimum)}]
      max: [{', '.join(str(value) for value in maximum)}]
"""
    path.write_text(
        f"""board:
  pose_in_map: {pose}
  width: 0.8
  height: 1.0
{map_section}
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


def distractor_only_cloud():
    """A room with every retroreflective distractor and no board in it."""
    scan = vlp32_sim.simulate(
        scenes.distractor_only_scene(), vlp32_sim.SimParams(seed=1)
    )
    return PointCloud(
        points=scan.points + np.array([0.0, 0.0, scenes.SENSOR_HEIGHT]),
        intensity=scan.intensity,
    )


def test_cli_writes_a_map_directory(tmp_path, capsys):
    # An arbitrary source frame, well away from the origin, as SLAM would give.
    cloud = build_map_cloud(
        source_pose=make_transform(rotation_z(2.4), [110.0, -55.0, 3.2])
    )
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "reflective_pose.yaml"
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
    config = tmp_path / "reflective_pose.yaml"
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
    config = tmp_path / "reflective_pose.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)
    output = tmp_path / "indoor-map"

    assert main([str(source), "-o", str(output), "--config", str(config), "--dry-run"]) == 0
    assert not output.exists()


def test_cli_applies_map_aabb_and_records_the_full_debug_context(tmp_path, capsys):
    cloud = build_map_cloud(with_distractors=True)
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "reflective_pose.yaml"
    dump = tmp_path / "anchor.npz"
    write_glim_style_ply(source, cloud)
    write_config(config, map_aabb=((-1.0, -1.0, 0.4), (1.0, 1.0, 1.8)))

    assert main([
        str(source), "-o", str(tmp_path / "out"), "--config", str(config),
        "--dry-run", "--dump-debug", str(dump),
    ]) == 0

    with np.load(dump, allow_pickle=False) as data:
        assert data["aabb_enabled"].item() is True
        assert data["aabb_frame"].item() == "map_debug"
        assert np.allclose(data["aabb_min"], [-1.0, -1.0, 0.4])
        assert np.allclose(data["aabb_max"], [1.0, 1.0, 1.8])
        assert 0 < data["n_inside_aabb"].item() < len(cloud.points)
        assert data["cloud_points"].shape == (len(cloud.points), 3)

    assert "map AABB kept" in capsys.readouterr().out


def test_cloud_without_intensity_exits_nonzero(tmp_path, capsys):
    cloud = build_map_cloud()
    source = tmp_path / "plain.ply"
    config = tmp_path / "reflective_pose.yaml"
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
    source = tmp_path / "no_board.ply"
    config = tmp_path / "reflective_pose.yaml"
    write_glim_style_ply(source, distractor_only_cloud())
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
    config = tmp_path / "reflective_pose.yaml"
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


def test_rviz_flags_are_gone(tmp_path):
    """The interface change, asserted: --rviz took ROS into this package."""
    cloud = build_map_cloud()
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "reflective_pose.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)

    for flag in ("--rviz", "--rviz-frame"):
        with pytest.raises(SystemExit):
            main([
                str(source), "-o", str(tmp_path / "out"),
                "--config", str(config), flag, "map_debug",
            ])


def test_package_imports_no_ros():
    """The whole reason this package exists, checked against the source text.

    A lazy ``import rclpy`` inside a function is exactly what the old --rviz
    path did, and it passes any import-time check, so this reads the files.
    """
    import pathlib

    import reflective_pose_cli

    root = pathlib.Path(reflective_pose_cli.__file__).parent
    offenders = []
    for module in sorted(root.rglob("*.py")):
        text = module.read_text()
        for line in text.splitlines():
            stripped = line.strip()
            if not stripped.startswith(("import ", "from ")):
                continue
            if "rclpy" in stripped or "_msgs" in stripped:
                offenders.append(f"{module.name}: {stripped}")
    assert offenders == []


def _load_dump(path):
    with np.load(path, allow_pickle=False) as handle:
        return {key: handle[key] for key in handle.files}


def test_dump_debug_on_success_carries_the_whole_picture(tmp_path):
    cloud = build_map_cloud()
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "reflective_pose.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)
    dump = tmp_path / "nested" / "anchor.npz"

    assert main([
        str(source), "-o", str(tmp_path / "out"), "--config", str(config),
        "--dry-run", "--dump-debug", str(dump),
    ]) == 0
    assert dump.exists(), "the dump is written on the dry-run path too"

    data = _load_dump(dump)
    assert data["format"].item() == "reflective_pose_anchor_debug"
    assert data["format_version"].item() == 1
    assert data["status"].item() == "ok"
    assert data["error"].item() == ""
    assert data["source_cloud"].item() == str(source)

    # The context cloud the old ~/debug/map_cloud topic carried: levelled
    # points and their intensity, one intensity per point.
    assert data["cloud_points"].shape == (len(cloud.points), 3)
    assert data["cloud_intensity"].shape == (len(cloud.points),)
    assert data["viewpoint"].shape == (3,)

    # The board itself, and everything the arrow marker needed to be drawn.
    assert data["detection_n_points"].item() > 0
    assert data["detection_points"].shape == (data["detection_n_points"].item(), 3)
    assert np.isclose(np.linalg.norm(data["detection_normal"]), 1.0)
    assert data["detection_rotation"].shape == (3, 3)
    assert list(data["detection_edge_names"]) == ["left", "right", "bottom", "top"]
    assert data["detection_observed_edges"].shape == (4,)

    # One survivor, its points reachable through the offsets.
    assert data["n_candidates"].item() == 1
    offsets = data["candidate_offsets"]
    assert offsets.shape == (2,)
    assert offsets[-1] == len(data["candidate_points"])
    first = data["candidate_points"][offsets[0]:offsets[1]]
    assert len(first) == data["candidate_n_points"][0]

    # The gates that produced the verdict, so a viewer can show measured
    # against expected without re-reading the YAML.
    names = list(data["param_names"])
    values = data["param_values"]
    assert values[names.index("board_width")] == pytest.approx(0.8)
    assert values[names.index("board_height")] == pytest.approx(1.0)
    # Anchoring relaxes range and density for a merged cloud; the dump must
    # record what ran, not what the file said.
    assert values[names.index("range_max")] == float("inf")
    assert values[names.index("density_check_enabled")] == 0.0

    assert np.isfinite(data["transform_map_cloud"]).all()
    assert np.isfinite(data["floor_tilt_deg"].item())


def test_dump_debug_on_failure_carries_every_rejection(tmp_path):
    """The failure path is the one the picture is for."""
    source = tmp_path / "no_board.ply"
    config = tmp_path / "reflective_pose.yaml"
    write_glim_style_ply(source, distractor_only_cloud())
    write_config(config)
    dump = tmp_path / "anchor.npz"

    assert main([
        str(source), "-o", str(tmp_path / "out"), "--config", str(config),
        "--dump-debug", str(dump),
    ]) == 1

    data = _load_dump(dump)
    assert data["status"].item() == "no_candidate"
    assert "no board found" in data["error"].item()
    assert len(data["cloud_points"]) > 0

    # Nothing survived, so the detection block is empty rather than absent —
    # every key is present in every dump.
    assert data["detection_points"].shape == (0, 3)
    assert np.isnan(data["detection_centre"]).all()
    assert data["n_candidates"].item() == 0
    assert data["candidate_points"].shape == (0, 3)
    assert data["candidate_offsets"].tolist() == [0]
    assert data["candidate_centres"].shape == (0, 3)

    count = data["n_rejections"].item()
    assert count > 0
    assert data["rejection_id"].tolist() == list(range(count))
    assert data["rejection_reason"].shape == (count,)
    assert data["rejection_centroid"].shape == (count, 3)
    assert data["rejection_n_points"].shape == (count,)

    # Measured against expected, per cluster, which is what triage needs.
    reasons = list(data["rejection_reason"])
    measured = data["rejection_measured"]
    for index, reason in enumerate(reasons):
        if reason == "degenerate_up":
            continue
        assert np.isfinite(measured[index]), reason
        assert np.isfinite(data["rejection_expected"][index]), reason
        # The verdict must be consistent with the band it is reported against.
        lo, hi = data["rejection_limit_lo"][index], data["rejection_limit_hi"][index]
        outside = (np.isfinite(lo) and measured[index] < lo) or (
            np.isfinite(hi) and measured[index] > hi
        )
        assert outside, f"{reason} {measured[index]} inside [{lo}, {hi}]"

    # The detail string is kept verbatim so a replayed label reads as it did
    # live, and it is where the measured number came from.
    assert any(data["rejection_detail"][index] for index in range(count))

    # No anchor result to record on a failure.
    assert np.isnan(data["transform_map_cloud"]).all()
    assert np.isnan(data["floor_tilt_deg"].item())


def test_no_dump_flag_writes_no_file(tmp_path):
    cloud = build_map_cloud()
    source = tmp_path / "glim_export.ply"
    config = tmp_path / "reflective_pose.yaml"
    write_glim_style_ply(source, cloud)
    write_config(config)

    assert main([
        str(source), "-o", str(tmp_path / "out"), "--config", str(config),
        "--dry-run",
    ]) == 0
    assert list(tmp_path.glob("*.npz")) == []
