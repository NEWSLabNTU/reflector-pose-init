"""Pose composition and the covariance model for the board initializer.

ROS-free by design, for the same reason as detector.py: the offline
map-anchoring step shares this code.
"""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from .detector import BoardDetection


@dataclass
class CovarianceParams:
    """Guess covariance sent to the pose initializer.

    Deliberately loose. The service is called with ``method=AUTO``, so NDT align
    refines the guess; an over-tight covariance makes it search too small a
    window, while an over-loose one only costs a few hundred milliseconds.
    """

    sigma_xy_base: float = 0.15
    sigma_xy_per_metre: float = 0.03
    sigma_z: float = 0.10
    sigma_yaw_base: float = 0.05
    sigma_roll_pitch: float = 0.02
    safety_factor: float = 2.0
    unconstrained_axis_sigma: float = 1.0


def transform_inverse(transform: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform."""
    result = np.eye(4)
    rotation = transform[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


def make_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    """Assemble a 4x4 transform from a rotation matrix and a translation."""
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, dtype=np.float64)
    return result


def matrix_from_euler_rpy(euler: Sequence[float]) -> np.ndarray:
    """Rotation matrix from intrinsic roll, pitch, yaw angles in radians.

    The returned matrix is ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``. This is the
    conventional ROS fixed-frame roll/pitch/yaw interpretation.
    """
    roll, pitch, yaw = (float(v) for v in euler)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def matrix_from_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    """Rotation matrix from a quaternion given in ROS order (x, y, z, w)."""
    x, y, z, w = (float(v) for v in quaternion)
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion_from_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Quaternion in ROS order (x, y, z, w) from a rotation matrix."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        w = (rotation[k, j] - rotation[j, k]) / s
        x, y, z = q
    return (float(x), float(y), float(z), float(w))


def board_pose_in_sensor(detection: BoardDetection) -> np.ndarray:
    """4x4 ``sensor <- board`` transform from a detection."""
    return make_transform(detection.rotation, detection.centre)


def map_pose_from_detection(
    detection: BoardDetection,
    transform_base_sensor: np.ndarray,
    transform_map_board: np.ndarray,
) -> np.ndarray:
    """Compose the vehicle pose in the map frame.

        T_map<-base = T_map<-board . (T_sensor<-board)^-1 . (T_base<-sensor)^-1

    ``T_map<-board`` is the identity rotation with the board's mounting height
    when the map is anchored to the board, which is the payoff of anchoring; it
    stays an argument so an un-anchored map is still usable.
    """
    transform_sensor_board = board_pose_in_sensor(detection)
    return (
        transform_map_board
        @ transform_inverse(transform_sensor_board)
        @ transform_inverse(transform_base_sensor)
    )


def covariance_from_detection(
    detection: BoardDetection, params: CovarianceParams = CovarianceParams()
) -> np.ndarray:
    """Row-major 6x6 covariance for the initial-pose guess.

    Grows with range, with the plane-fit residual, and — sharply — along any
    axis whose bounding edges were never observed, because there the rectangle
    centre is a lower bound on the board rather than the board.
    """
    sigma_xy = params.sigma_xy_base + params.sigma_xy_per_metre * detection.range_m
    extent = max(detection.extents[0], 1e-3)
    sigma_yaw = params.sigma_yaw_base + detection.plane_residual / extent

    horizontal_ok, vertical_ok = detection.centre_constrained
    sigma_x = sigma_xy if horizontal_ok else max(sigma_xy, params.unconstrained_axis_sigma)
    sigma_y = sigma_x
    sigma_z = params.sigma_z if vertical_ok else max(
        params.sigma_z, params.unconstrained_axis_sigma
    )

    k = params.safety_factor
    variances = [
        (k * sigma_x) ** 2,
        (k * sigma_y) ** 2,
        (k * sigma_z) ** 2,
        (k * params.sigma_roll_pitch) ** 2,
        (k * params.sigma_roll_pitch) ** 2,
        (k * sigma_yaw) ** 2,
    ]
    return np.diag(variances)


def yaw_from_matrix(rotation: np.ndarray) -> float:
    """Yaw about z, in radians."""
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def pose_error(estimated: np.ndarray, truth: np.ndarray) -> Tuple[float, float]:
    """Position error in metres and absolute yaw error in radians."""
    position = float(np.linalg.norm(estimated[:3, 3] - truth[:3, 3]))
    delta = yaw_from_matrix(estimated[:3, :3]) - yaw_from_matrix(truth[:3, :3])
    delta = (delta + np.pi) % (2 * np.pi) - np.pi
    return position, abs(float(delta))
