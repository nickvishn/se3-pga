import numpy as np

from src.math.constants import EPS, SMALL_ANGLE_EPS
from src.math.pga.quaternions import (
    quat_conj,
    quat_from_axis_angle,
    quat_mul,
    quat_norm,
    quat_normalize,
    quat_to_rotmat,
)
from src.math.se3.rotations import left_jacobian_so3


def motor_identity():
    qr = np.array([1.0, 0.0, 0.0, 0.0])
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    return qr, qd


def motor_normalize(qr: np.ndarray, qd: np.ndarray):
    qr = np.asarray(qr, dtype=float).reshape(4)
    qd = np.asarray(qd, dtype=float).reshape(4)

    n = quat_norm(qr)
    if n < EPS:
        raise ValueError("Motor real part has zero norm.")

    qr = qr / n
    qd = qd / n

    qd = qd - qr * np.dot(qr, qd)
    return qr, qd


def motor_mul(M1, M2):
    qr1, qd1 = M1
    qr2, qd2 = M2

    qr = quat_mul(qr1, qr2)
    qd = quat_mul(qr1, qd2) + quat_mul(qd1, qr2)

    return motor_normalize(qr, qd)


def motor_reverse(M):
    qr, qd = M
    return quat_conj(qr), quat_conj(qd)


def motor_from_rt(qr: np.ndarray, t: np.ndarray):
    qr = quat_normalize(qr)
    t = np.asarray(t, dtype=float).reshape(3)
    tq = np.array([0.0, t[0], t[1], t[2]])
    qd = 0.5 * quat_mul(tq, qr)
    return motor_normalize(qr, qd)


def motor_to_rt(M):
    qr, qd = M
    qr = quat_normalize(qr)

    tq = 2.0 * quat_mul(qd, quat_conj(qr))
    t = tq[1:]
    R = quat_to_rotmat(qr)

    return R, t


def motor_from_twist(v: np.ndarray, w: np.ndarray, dt: float):
    v = np.asarray(v, dtype=float).reshape(3)
    w = np.asarray(w, dtype=float).reshape(3)

    phi = w * dt
    angle = np.linalg.norm(phi)

    if angle < SMALL_ANGLE_EPS:
        qr = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        axis = phi / angle
        qr = quat_from_axis_angle(axis, angle)

    J = left_jacobian_so3(phi)
    t = J @ (v * dt)

    return motor_from_rt(qr, t)


def transform_points_motor(M, points: np.ndarray) -> np.ndarray:
    R, t = motor_to_rt(M)

    points = np.asarray(points, dtype=float)
    single = False
    if points.ndim == 1:
        points = points.reshape(1, 3)
        single = True

    transformed = (R @ points.T).T + t
    return transformed[0] if single else transformed


def generate_motor_trajectory(v, w, dt, steps):
    M = motor_identity()
    delta = motor_from_twist(v, w, dt)

    motors = [M]
    for _ in range(steps):
        M = motor_mul(M, delta)
        motors.append(M)

    return motors
