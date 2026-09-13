import numpy as np

from src.math.constants import SMALL_ANGLE_EPS


def skew(w: np.ndarray) -> np.ndarray:
    w = np.asarray(w, dtype=float).reshape(3)
    wx, wy, wz = w
    return np.array(
        [
            [0.0, -wz, wy],
            [wz, 0.0, -wx],
            [-wy, wx, 0.0],
        ]
    )


def exp_so3(phi: np.ndarray) -> np.ndarray:
    phi = np.asarray(phi, dtype=float).reshape(3)
    theta = np.linalg.norm(phi)
    Phi = skew(phi)

    if theta < SMALL_ANGLE_EPS:
        return np.eye(3) + Phi + 0.5 * (Phi @ Phi)

    A = np.sin(theta) / theta
    B = (1.0 - np.cos(theta)) / (theta**2)
    return np.eye(3) + A * Phi + B * (Phi @ Phi)


def left_jacobian_so3(phi: np.ndarray) -> np.ndarray:
    phi = np.asarray(phi, dtype=float).reshape(3)
    theta = np.linalg.norm(phi)
    Phi = skew(phi)

    if theta < SMALL_ANGLE_EPS:
        return np.eye(3) + 0.5 * Phi + (1.0 / 6.0) * (Phi @ Phi)

    A = (1.0 - np.cos(theta)) / (theta**2)
    B = (theta - np.sin(theta)) / (theta**3)
    return np.eye(3) + A * Phi + B * (Phi @ Phi)
