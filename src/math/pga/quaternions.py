import numpy as np

from src.math.constants import EPS


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    q1 = np.asarray(q1, dtype=float).reshape(4)
    q2 = np.asarray(q2, dtype=float).reshape(4)

    a1, b1, c1, d1 = q1
    a2, b2, c2, d2 = q2

    return np.array(
        [
            a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2,
            a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
            a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2,
            a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2,
        ]
    )


def quat_conj(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_norm(q: np.ndarray) -> float:
    q = np.asarray(q, dtype=float).reshape(4)
    return float(np.linalg.norm(q))


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = quat_norm(q)
    if n < EPS:
        raise ValueError("Zero quaternion cannot be normalized.")
    return q / n


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(3)
    n = np.linalg.norm(axis)

    if n < EPS or abs(angle) < EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])

    axis = axis / n
    half = 0.5 * angle
    s = np.sin(half)

    return np.array(
        [
            np.cos(half),
            axis[0] * s,
            axis[1] * s,
            axis[2] * s,
        ]
    )


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    vq = np.array([0.0, *np.asarray(v, dtype=float).reshape(3)])
    rotated = quat_mul(quat_mul(q, vq), quat_conj(q))
    return rotated[1:]


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    w, x, y, z = q

    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
