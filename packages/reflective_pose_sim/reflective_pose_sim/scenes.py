"""Indoor scenes for the board detector tests.

Everything is expressed in the sensor frame, with ``base_link`` directly below
the sensor at ground level. Ground truth is the board rectangle's own pose, so
the tests compare against the geometry the simulator actually rendered rather
than against a separately maintained constant.
"""

from typing import Tuple

import numpy as np

from .vlp32_sim import DIFFUSE, RETRO, Rectangle, Scene

SENSOR_HEIGHT = 1.6
BOARD_CENTRE_HEIGHT = 1.075
BOARD_WIDTH = 0.8
BOARD_HEIGHT = 1.0


def transform_base_sensor(sensor_height: float = SENSOR_HEIGHT) -> np.ndarray:
    """``base_link <- sensor``: no rotation, sensor mounted above the origin."""
    transform = np.eye(4)
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
