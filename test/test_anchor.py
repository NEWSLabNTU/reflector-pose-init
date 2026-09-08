"""Anchoring a synthetic map cloud to the board.

The map cloud is built the way a real one is: several scans from different
places in the room, merged into one frame, then rotated and translated into an
arbitrary pose to stand in for SLAM's arbitrary output frame. Anchoring has to
undo that and land the board at the origin regardless of where the source frame
happened to be.
"""

import numpy as np
import pytest

from golfcart_board_initializer.anchor import (
    AnchorParams,
    anchor_cloud,
    apply_transform,
    board_polygon_osm,
    fit_floor,
)
from golfcart_board_initializer.detector import Status
from golfcart_board_initializer.geometry import make_transform
from golfcart_board_initializer.pointcloud_io import PointCloud
from golfcart_board_initializer.simulation import scenes, vlp32_sim

BOARD_CENTRE_HEIGHT = scenes.BOARD_CENTRE_HEIGHT


def rotation_z(angle):
    cos, sin = np.cos(angle), np.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def rotation_y(angle):
    cos, sin = np.cos(angle), np.sin(angle)
    return np.array([[cos, 0.0, sin], [0.0, 1.0, 0.0], [-sin, 0.0, cos]])


# Sensor poses in the room frame: (x, y, heading). The board is fixed at the
# room origin facing +x, so each scan sees the same board from a different
# place — which is what makes the merged cloud a map rather than three boards.
SENSOR_POSES = [
    (-5.0, 0.0, 0.0),
    (-7.0, 2.5, np.radians(-18.0)),
    (-4.0, -1.5, np.radians(12.0)),
]


def build_map_cloud(source_pose=None, with_distractors=True, seeds=(1, 2, 3)):
    """Merge scans taken from several sensor poses into one room frame.

    Each scan is rendered in its own sensor frame — the scene builder places the
    board relative to the sensor — and then lifted into the room frame by the
    sensor's pose. Moving the board instead would put a separate board in the
    map for every viewpoint, which is a different scene, not a map.
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


def anchored_board_centre(result):
    """Board centre after anchoring, via the input frame.

    ``detection.centre`` lives in the levelled intermediate frame, not the input
    frame, so it must not be fed to transform_map_cloud directly — that mistake
    only shows up once the source frame is far from the origin.
    """
    centre = np.asarray(result.board_centre_cloud)
    return result.transform_map_cloud[:3, :3] @ centre + result.transform_map_cloud[:3, 3]


def test_board_lands_at_the_origin():
    cloud = build_map_cloud()
    result = anchor_cloud(cloud)

    centre = anchored_board_centre(result)
    assert abs(centre[0]) < 0.10
    assert abs(centre[1]) < 0.10
    assert centre[2] == pytest.approx(BOARD_CENTRE_HEIGHT, abs=0.15)


def test_anchoring_undoes_an_arbitrary_source_frame():
    """The whole point: the source frame is arbitrary, the map frame is not."""
    source_pose = make_transform(rotation_z(1.1), [37.0, -12.0, 4.5])
    cloud = build_map_cloud(source_pose=source_pose)

    result = anchor_cloud(cloud)
    centre = anchored_board_centre(result)

    assert abs(centre[0]) < 0.10
    assert abs(centre[1]) < 0.10
    assert centre[2] == pytest.approx(BOARD_CENTRE_HEIGHT, abs=0.15)


def test_anchoring_places_board_at_configured_pose():
    params = AnchorParams(
        board_pose_in_map=(12.0, -4.0, BOARD_CENTRE_HEIGHT, 0.0, 0.0, np.pi / 2)
    )
    result = anchor_cloud(build_map_cloud(), params)
    centre = anchored_board_centre(result)
    assert np.allclose(centre, params.board_pose_in_map[:3], atol=0.10)
    normal = result.transform_map_cloud[:3, :3] @ result.detection.normal
    assert np.allclose(normal, [0.0, 1.0, 0.0], atol=0.05)


def test_map_x_points_along_the_board_normal():
    cloud = build_map_cloud()
    result = anchor_cloud(cloud)

    normal = result.transform_map_cloud[:3, :3] @ np.asarray(result.detection.normal)
    assert normal[0] == pytest.approx(1.0, abs=0.05)
    assert abs(normal[2]) < 0.05


def test_floor_lands_at_zero_after_anchoring():
    cloud = build_map_cloud()
    result = anchor_cloud(cloud)
    anchored = apply_transform(cloud, result.transform_map_cloud)

    floor = anchored.points[anchored.points[:, 2] < 0.2]
    assert len(floor) > 1000
    assert abs(np.median(floor[:, 2])) < 0.05


def test_intensity_survives_anchoring():
    cloud = build_map_cloud()
    result = anchor_cloud(cloud)
    anchored = apply_transform(cloud, result.transform_map_cloud)

    assert anchored.has_intensity
    assert np.array_equal(anchored.intensity, cloud.intensity)


def test_slightly_tilted_source_frame_is_levelled():
    """LiDAR-inertial SLAM is gravity-aligned to about a degree, not exactly."""
    source_pose = make_transform(rotation_y(np.radians(1.5)), [0.0, 0.0, 0.0])
    cloud = build_map_cloud(source_pose=source_pose)

    result = anchor_cloud(cloud)
    assert result.floor_tilt_deg == pytest.approx(1.5, abs=0.5)

    anchored = apply_transform(cloud, result.transform_map_cloud)
    floor = anchored.points[anchored.points[:, 2] < 0.2]
    assert abs(np.median(floor[:, 2])) < 0.05


def test_badly_tilted_cloud_is_rejected_rather_than_levelled():
    source_pose = make_transform(rotation_y(np.radians(35.0)), [0.0, 0.0, 0.0])
    cloud = build_map_cloud(source_pose=source_pose)

    with pytest.raises(ValueError, match="not gravity-aligned"):
        anchor_cloud(cloud)


def test_cloud_without_intensity_is_refused():
    cloud = build_map_cloud()
    stripped = PointCloud(points=cloud.points, intensity=None)

    with pytest.raises(ValueError, match="no intensity"):
        anchor_cloud(stripped)


def test_two_boards_refuse_to_define_a_frame():
    """Anchoring to the wrong board shifts everything with no later symptom."""
    scene = scenes.two_board_scene()
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=1))
    cloud = PointCloud(
        points=scan.points + np.array([0.0, 0.0, scenes.SENSOR_HEIGHT]),
        intensity=scan.intensity,
    )

    with pytest.raises(ValueError, match="candidates"):
        anchor_cloud(cloud)


def test_distractors_alone_yield_no_anchor():
    scene = scenes.distractor_only_scene()
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=1))
    cloud = PointCloud(
        points=scan.points + np.array([0.0, 0.0, scenes.SENSOR_HEIGHT]),
        intensity=scan.intensity,
    )

    with pytest.raises(ValueError, match="no board found"):
        anchor_cloud(cloud)


def test_on_result_still_fires_when_detection_fails():
    """The CLI's --rviz debug hook needs the rejected clusters, not just the

    exception's flattened string — a raise must not skip the callback.
    """
    scene = scenes.distractor_only_scene()
    scan = vlp32_sim.simulate(scene, vlp32_sim.SimParams(seed=1))
    cloud = PointCloud(
        points=scan.points + np.array([0.0, 0.0, scenes.SENSOR_HEIGHT]),
        intensity=scan.intensity,
    )

    captured = {}

    def on_result(levelled, intensity, result, viewpoint):
        captured["levelled"] = levelled
        captured["intensity"] = intensity
        captured["result"] = result

    with pytest.raises(ValueError, match="no board found"):
        anchor_cloud(cloud, on_result=on_result)

    assert captured, "on_result must run before the failure is raised"
    assert captured["result"].status is Status.NO_CANDIDATE
    assert captured["result"].n_clusters > 0
    assert captured["result"].rejections
    assert len(captured["levelled"]) == len(captured["intensity"])


def test_on_result_fires_on_success_too():
    cloud = build_map_cloud()
    captured = {}

    result = anchor_cloud(
        cloud,
        on_result=lambda levelled, intensity, detect_result, viewpoint: captured.update(
            result=detect_result
        ),
    )

    assert captured["result"].status is Status.OK
    assert captured["result"].detection is result.detection


def test_floor_fit_recovers_a_known_plane():
    rng = np.random.default_rng(0)
    flat = np.column_stack(
        (rng.uniform(-5, 5, 4000), rng.uniform(-5, 5, 4000), rng.normal(0, 0.01, 4000))
    )
    ceiling = flat + np.array([0.0, 0.0, 3.0])
    normal, offset = fit_floor(np.vstack((flat, ceiling)), AnchorParams())

    assert normal[2] == pytest.approx(1.0, abs=1e-3)
    assert offset == pytest.approx(0.0, abs=0.02)


def test_board_polygon_is_derived_from_the_board_dimensions():
    osm = board_polygon_osm(AnchorParams())

    assert "pose_marker" in osm
    assert "reflector" in osm
    # Half width either side of the origin, centre height 1.075, x = 0 on the face.
    assert "v='0.4000'" in osm
    assert "v='-0.4000'" in osm
    assert "v='0.5750'" in osm  # 1.075 - 0.5
    assert "v='1.5750'" in osm  # 1.075 + 0.5
    assert osm.count("<nd ref=") == 5  # four corners, closed


def test_board_polygon_uses_configured_translation_and_yaw():
    params = AnchorParams(
        board_pose_in_map=(12.0, -4.0, 1.075, 0.0, 0.0, np.pi / 2)
    )
    osm = board_polygon_osm(params)

    # Board-local y becomes map x at +90 degrees yaw; all vertices retain y=-4.
    assert "local_x' v='11.6000'" in osm
    assert "local_x' v='12.4000'" in osm
    assert osm.count("local_y' v='-4.0000'") == 4
