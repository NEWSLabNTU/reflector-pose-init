"""Indoor scenes for the board detector tests.

Everything is expressed in the sensor's intrinsic frame (x forward, z up), with
``base_link`` directly below the sensor at ground level. The simulator rotates
its output into the driver's cloud frame; ``transform_base_sensor`` takes the
same sensor name so the TF a test hands the detector matches that cloud. Ground truth is the board rectangle's own pose, so
the tests compare against the geometry the simulator actually rendered rather
than against a separately maintained constant.
"""

from typing import List, Tuple

import numpy as np

from reflective_pose_core.sensors import DEFAULT_SENSOR, sensor_model

from .vlp32_sim import DIFFUSE, RETRO, Rectangle, Scene

SENSOR_HEIGHT = 1.6
BOARD_CENTRE_HEIGHT = 1.075
BOARD_WIDTH = 0.8
BOARD_HEIGHT = 1.0


def transform_base_sensor(
    sensor_height: float = SENSOR_HEIGHT, sensor: str = DEFAULT_SENSOR
) -> np.ndarray:
    """``base_link <- sensor cloud frame``: level, mounted above the origin.

    For a Velodyne that is a pure translation. For a sensor whose driver
    publishes its native axes it carries the axis rotation too, which is what
    the vehicle's own TF does (AutoSDV's Robin-W: roll pi, pitch -pi/2).
    """
    transform = np.eye(4)
    transform[:3, :3] = sensor_model(sensor).cloud_rotation.T
    transform[:3, 3] = np.array([0.0, 0.0, sensor_height])
    return transform


def make_room(
    size_x: float = 40.0,
    size_y: float = 30.0,
    ceiling: float = 3.0,
    sensor_height: float = SENSOR_HEIGHT,
) -> Scene:
    """A rectangular room: floor, ceiling, four walls, all diffuse."""
    scene = Scene()
    half_x, half_y = 0.5 * size_x, 0.5 * size_y
    floor_z = -sensor_height
    ceiling_z = ceiling - sensor_height

    scene.add(
        Rectangle([0, 0, floor_z], [0, 0, 1], [1, 0, 0], half_x, half_y, DIFFUSE, 0.5, "floor")
    )
    scene.add(
        Rectangle(
            [0, 0, ceiling_z], [0, 0, -1], [1, 0, 0], half_x, half_y, DIFFUSE, 0.7, "ceiling"
        )
    )
    wall_half_height = 0.5 * ceiling
    wall_centre_z = wall_half_height - sensor_height
    scene.add(
        Rectangle(
            [half_x, 0, wall_centre_z], [-1, 0, 0], [0, 0, 1], half_y, wall_half_height,
            DIFFUSE, 0.6, "wall_front",
        )
    )
    scene.add(
        Rectangle(
            [-half_x, 0, wall_centre_z], [1, 0, 0], [0, 0, 1], half_y, wall_half_height,
            DIFFUSE, 0.6, "wall_back",
        )
    )
    scene.add(
        Rectangle(
            [0, half_y, wall_centre_z], [0, -1, 0], [0, 0, 1], half_x, wall_half_height,
            DIFFUSE, 0.6, "wall_left",
        )
    )
    scene.add(
        Rectangle(
            [0, -half_y, wall_centre_z], [0, 1, 0], [0, 0, 1], half_x, wall_half_height,
            DIFFUSE, 0.6, "wall_right",
        )
    )
    return scene


def _rotate(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation."""
    axis = axis / np.linalg.norm(axis)
    return (
        vector * np.cos(angle)
        + np.cross(axis, vector) * np.sin(angle)
        + axis * np.dot(axis, vector) * (1.0 - np.cos(angle))
    )


def place_board(
    range_m: float = 5.0,
    bearing_deg: float = 0.0,
    yaw_deg: float = 0.0,
    tilt_deg: float = 0.0,
    centre_height: float = BOARD_CENTRE_HEIGHT,
    sensor_height: float = SENSOR_HEIGHT,
    width: float = BOARD_WIDTH,
    height: float = BOARD_HEIGHT,
    reflectivity: float = 1.0,
    name: str = "board",
) -> Rectangle:
    """A retroreflective board facing the sensor, optionally rotated and tilted.

    ``yaw_deg`` turns the board away from facing the sensor squarely, which is
    the viewing-angle axis of the test matrix. ``tilt_deg`` leans it about its
    own horizontal axis, exercising the verticality gate.
    """
    bearing = np.radians(bearing_deg)
    centre = np.array(
        [
            range_m * np.cos(bearing),
            range_m * np.sin(bearing),
            centre_height - sensor_height,
        ]
    )

    normal = -np.array([np.cos(bearing), np.sin(bearing), 0.0])
    normal = _rotate(normal, np.array([0.0, 0.0, 1.0]), np.radians(yaw_deg))
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, normal)
    right /= np.linalg.norm(right)

    if tilt_deg:
        normal = _rotate(normal, right, np.radians(tilt_deg))
        up = _rotate(up, right, np.radians(tilt_deg))

    return Rectangle(
        centre, normal, up, 0.5 * width, 0.5 * height, RETRO, reflectivity, name
    )


def add_distractors(scene: Scene, sensor_height: float = SENSOR_HEIGHT) -> Scene:
    """Retroreflective objects that are not the board.

    Every one of these passes the intensity gate — that is the point. They exist
    to exercise the geometric gates, which are the only thing separating the
    board from ordinary indoor safety hardware.
    """
    # Exit sign: right size band to be tempting, too small to pass the extent gate.
    scene.add(
        place_board(
            range_m=6.0,
            bearing_deg=35.0,
            width=0.30,
            height=0.20,
            centre_height=2.20,
            sensor_height=sensor_height,
            name="exit_sign",
        )
    )
    # Floor tape: correct size, wrong orientation and height.
    scene.add(
        Rectangle(
            [4.0, -3.0, 0.01 - sensor_height],
            [0, 0, 1],
            [1, 0, 0],
            0.60,
            0.08,
            RETRO,
            0.8,
            "floor_tape",
        )
    )
    # Safety vest: right height, too small.
    scene.add(
        place_board(
            range_m=4.5,
            bearing_deg=-40.0,
            width=0.35,
            height=0.45,
            centre_height=1.20,
            sensor_height=sensor_height,
            name="vest",
        )
    )
    return scene


def board_scene(
    range_m: float = 5.0,
    bearing_deg: float = 0.0,
    yaw_deg: float = 0.0,
    tilt_deg: float = 0.0,
    reflectivity: float = 1.0,
    with_distractors: bool = True,
    occlusion: Tuple[str, float, str] = None,
    sensor_height: float = SENSOR_HEIGHT,
) -> Tuple[Scene, np.ndarray]:
    """Room, board, and optional distractors. Returns the scene and ground truth.

    ``occlusion`` is ``(axis, fraction, side)`` with axis ``horizontal`` or
    ``vertical`` and side ``low`` or ``high``.
    """
    scene = make_room(sensor_height=sensor_height)
    board = place_board(
        range_m=range_m,
        bearing_deg=bearing_deg,
        yaw_deg=yaw_deg,
        tilt_deg=tilt_deg,
        reflectivity=reflectivity,
        sensor_height=sensor_height,
    )
    scene.add(board)
    if with_distractors:
        add_distractors(scene, sensor_height=sensor_height)
    if occlusion is not None:
        axis, fraction, side = occlusion
        scene.occlusions.append(("board", axis, fraction, side))
    return scene, board.pose()


def distractor_only_scene(sensor_height: float = SENSOR_HEIGHT) -> Scene:
    """Everything retroreflective except a board. Must yield no detection."""
    scene = make_room(sensor_height=sensor_height)
    return add_distractors(scene, sensor_height=sensor_height)


def two_board_scene(sensor_height: float = SENSOR_HEIGHT) -> Scene:
    """Two valid boards. Must abort as ambiguous rather than pick one."""
    scene = make_room(sensor_height=sensor_height)
    scene.add(place_board(range_m=5.0, bearing_deg=0.0, sensor_height=sensor_height))
    scene.add(
        place_board(range_m=6.0, bearing_deg=25.0, sensor_height=sensor_height, name="board_2")
    )
    return scene


# -- AutoSDV tracking scenes ----------------------------------------------------
#
# The coach-board pursuit lab: a 0.6 m square board held by a person in front of
# a 1/10-scale car, followed at about 2 m. The LiDAR sits on top of the car, far
# lower than the golf cart's 1.96 m, and the board centre is at chest height
# rather than on a fixed stand, so neither the defaults above nor the packaged
# detector file describe it.

#: LiDAR optical centre above the ground on AutoSDV. NOT MEASURED: the car's
#: vehicle_height is 0.262 m and the sensor sits on top of it; AutoSDV's
#: sensor_kit_calibration still has z = 0 for every LiDAR. Replace with a tape
#: measurement, here and in the AutoSDV detector profiles.
AUTOSDV_SENSOR_HEIGHT = 0.30
#: The handheld board: 0.6 m square, centre near 1.0 m.
HANDHELD_BOARD_SIZE = 0.6
HANDHELD_BOARD_CENTRE_HEIGHT = 1.0


def add_tracking_distractors(scene: Scene, sensor_height: float = AUTOSDV_SENSOR_HEIGHT) -> Scene:
    """Retroreflectors an outdoor or hall chase meets that are not the board.

    Every one passes the intensity gate, sits inside the tracking range, and
    each fails a different geometric gate: too small at the right height, too
    narrow, too large, too low, lying flat.
    """
    # Safety vest on a bystander: right height, too small.
    scene.add(
        place_board(
            range_m=4.0, bearing_deg=-35.0, width=0.35, height=0.45,
            centre_height=1.1, sensor_height=sensor_height, name="vest",
        )
    )
    # Reflective strip on a post: tall and narrow.
    scene.add(
        place_board(
            range_m=6.0, bearing_deg=30.0, width=0.08, height=0.9,
            centre_height=0.9, sensor_height=sensor_height, name="post_strip",
        )
    )
    # A large reflective sign: right shape, far too big. Kept clear of the
    # board's line of sight: partly hidden behind the board, the visible part
    # of a sign this size can be board-sized, and the scan is then rightly
    # AMBIGUOUS.
    scene.add(
        place_board(
            range_m=8.0, bearing_deg=-50.0, width=1.4, height=1.0,
            centre_height=1.2, sensor_height=sensor_height, name="sign",
        )
    )
    # Another car's tail reflector: small and low.
    scene.add(
        place_board(
            range_m=3.0, bearing_deg=40.0, width=0.20, height=0.06,
            centre_height=0.25, sensor_height=sensor_height, name="tail_reflector",
        )
    )
    # Floor tape: the right size, lying flat.
    scene.add(
        Rectangle(
            [3.0, -1.5, 0.01 - sensor_height], [0, 0, 1], [1, 0, 0], 0.30, 0.30,
            RETRO, 0.8, "floor_tape",
        )
    )
    return scene


def tracking_board_scene(
    range_m: float = 2.0,
    bearing_deg: float = 0.0,
    yaw_deg: float = 0.0,
    centre_height: float = HANDHELD_BOARD_CENTRE_HEIGHT,
    sensor_height: float = AUTOSDV_SENSOR_HEIGHT,
    size: float = HANDHELD_BOARD_SIZE,
    with_distractors: bool = True,
) -> Tuple[Scene, np.ndarray]:
    """A large hall, the handheld board, and optional distractors.

    Returns the scene and the board's ground-truth pose in the sensor's
    intrinsic frame.
    """
    scene = make_room(size_x=60.0, size_y=40.0, ceiling=4.0, sensor_height=sensor_height)
    board = place_board(
        range_m=range_m,
        bearing_deg=bearing_deg,
        yaw_deg=yaw_deg,
        centre_height=centre_height,
        sensor_height=sensor_height,
        width=size,
        height=size,
    )
    scene.add(board)
    if with_distractors:
        add_tracking_distractors(scene, sensor_height=sensor_height)
    return scene, board.pose()


def tracking_distractor_scene(sensor_height: float = AUTOSDV_SENSOR_HEIGHT) -> Scene:
    """The tracking distractors with no board. Must yield no detection."""
    scene = make_room(size_x=60.0, size_y=40.0, ceiling=4.0, sensor_height=sensor_height)
    return add_tracking_distractors(scene, sensor_height=sensor_height)


def walking_board_track(
    start_range_m: float = 2.0,
    speed_mps: float = 1.0,
    duration_s: float = 3.0,
    rate_hz: float = 10.0,
    bearing_deg: float = 0.0,
    sway_deg: float = 0.0,
    sway_period_s: float = 2.0,
    centre_height: float = HANDHELD_BOARD_CENTRE_HEIGHT,
    sensor_height: float = AUTOSDV_SENSOR_HEIGHT,
    with_distractors: bool = True,
) -> List[Tuple[float, Scene, np.ndarray]]:
    """A board walking away (``speed_mps`` > 0) or towards the car, one scene per scan.

    Returns ``(t, scene, truth)`` per frame, ``t`` in seconds from the start.
    Range changes linearly and the bearing sways sinusoidally by ``sway_deg``,
    which is how a person walking a board moves it. Each frame is one
    instantaneous scan: the motion inside a 100 ms revolution is ignored, which
    is fair for the board alone -- it spans a few degrees of azimuth, a few
    milliseconds of the sweep -- but means this does not test deskewing.
    """
    frames = []
    count = int(round(duration_s * rate_hz))
    for index in range(count):
        t = index / rate_hz
        range_m = start_range_m + speed_mps * t
        bearing = bearing_deg + sway_deg * np.sin(2.0 * np.pi * t / sway_period_s)
        scene, truth = tracking_board_scene(
            range_m=range_m,
            bearing_deg=bearing,
            centre_height=centre_height,
            sensor_height=sensor_height,
            with_distractors=with_distractors,
        )
        frames.append((t, scene, truth))
    return frames
