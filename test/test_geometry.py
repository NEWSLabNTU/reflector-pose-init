"""Pose composition and covariance model."""

import numpy as np
import pytest

from golfcart_board_initializer.detector import BoardDetection
from golfcart_board_initializer.geometry import (
    CovarianceParams,
    covariance_from_detection,
    make_transform,
    map_pose_from_detection,
    matrix_from_euler_rpy,
    matrix_from_quaternion,
    pose_error,
    quaternion_from_matrix,
    transform_inverse,
    yaw_from_matrix,
)


def make_detection(centre, rotation, **kwargs):
    defaults = dict(
        extents=(0.8, 1.0),
        points=np.zeros((400, 3)),
        n_points=400,
        plane_residual=0.005,
        range_m=float(np.linalg.norm(centre)),
        observed_edges={"left": True, "right": True, "bottom": True, "top": True},
    )
    defaults.update(kwargs)
    return BoardDetection(
        centre=np.asarray(centre, dtype=float),
        rotation=rotation,
        normal=rotation[:, 0],
        right=rotation[:, 1],
        up=rotation[:, 2],
        **defaults,
    )


def test_transform_inverse_round_trip():
    rotation = matrix_from_quaternion([0.1, 0.2, 0.3, 0.9])
    transform = make_transform(rotation, [1.0, -2.0, 0.5])
    assert np.allclose(transform @ transform_inverse(transform), np.eye(4), atol=1e-12)


def test_quaternion_matrix_round_trip():
    for quaternion in ([0, 0, 0, 1], [0.1, 0.2, 0.3, 0.9], [0, 0.7071, 0, 0.7071]):
        rotation = matrix_from_quaternion(quaternion)
        recovered = matrix_from_quaternion(quaternion_from_matrix(rotation))
        assert np.allclose(rotation, recovered, atol=1e-9)


def test_euler_rpy_yaw_rotates_x_to_y():
    rotation = matrix_from_euler_rpy([0.0, 0.0, np.pi / 2])
    assert np.allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])


def test_vehicle_pose_when_facing_the_board_head_on():
    """Sensor 5 m from the board, square on: the vehicle sits 5 m along map +x."""
    # Board frame in the sensor frame: normal points back at the sensor (-x),
    # left is +y in the board's own frame, up is +z.
    rotation = np.column_stack(
        ([-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0])
    )
    detection = make_detection([5.0, 0.0, -0.5], rotation)

    transform_base_sensor = make_transform(np.eye(3), [0.0, 0.0, 1.6])
    transform_map_board = make_transform(np.eye(3), [0.0, 0.0, 1.075])

    pose = map_pose_from_detection(
        detection, transform_base_sensor, transform_map_board
    )

    assert pose[0, 3] == pytest.approx(5.0, abs=1e-9)
    assert pose[1, 3] == pytest.approx(0.0, abs=1e-9)
    # Board centre sits 1.1 m above base_link (-0.5 below a sensor at 1.6 m) but
    # only 1.075 m above the map origin, so base_link lands 25 mm below it.
    assert pose[2, 3] == pytest.approx(1.075 - (-0.5 + 1.6), abs=1e-9)
    # Facing the board means facing map -x.
    assert abs(yaw_from_matrix(pose[:3, :3])) == pytest.approx(np.pi, abs=1e-9)


def test_vehicle_pose_respects_rotated_translated_board_pose():
    rotation = np.column_stack(
        ([-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0])
    )
    detection = make_detection([5.0, 0.0, -0.5], rotation)
    transform_base_sensor = make_transform(np.eye(3), [0.0, 0.0, 1.6])
    canonical_board = make_transform(np.eye(3), [0.0, 0.0, 1.075])
    configured_board = make_transform(
        matrix_from_euler_rpy([0.0, 0.0, np.pi / 2]), [12.0, -4.0, 1.075]
    )

    canonical_pose = map_pose_from_detection(
        detection, transform_base_sensor, canonical_board
    )
    pose = map_pose_from_detection(detection, transform_base_sensor, configured_board)

    expected = configured_board @ np.linalg.inv(canonical_board) @ canonical_pose
    assert np.allclose(pose, expected)


def test_covariance_grows_with_range():
    rotation = np.column_stack(
        ([-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0])
    )
    near = covariance_from_detection(make_detection([3.0, 0.0, -0.5], rotation))
    far = covariance_from_detection(make_detection([15.0, 0.0, -0.5], rotation))
    assert far[0, 0] > near[0, 0]


def test_covariance_inflates_unconstrained_axes():
    rotation = np.column_stack(
        ([-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0])
    )
    constrained = make_detection([5.0, 0.0, -0.5], rotation)
    unconstrained = make_detection(
        [5.0, 0.0, -0.5],
        rotation,
        observed_edges={"left": False, "right": False, "bottom": True, "top": True},
    )

    params = CovarianceParams()
    assert covariance_from_detection(unconstrained, params)[0, 0] > (
        covariance_from_detection(constrained, params)[0, 0]
    )


def test_pose_error_wraps_yaw():
    a = make_transform(matrix_from_quaternion([0, 0, 0, 1]), [0, 0, 0])
    rotation = np.array(
        [
            [np.cos(0.1), -np.sin(0.1), 0.0],
            [np.sin(0.1), np.cos(0.1), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    b = make_transform(rotation, [0.3, 0.4, 0.0])
    position, yaw = pose_error(b, a)
    assert position == pytest.approx(0.5)
    assert yaw == pytest.approx(0.1)
