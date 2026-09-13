import numpy as np

from src.math.se3.rotations import exp_so3, left_jacobian_so3


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def split_transform(T: np.ndarray):
    return T[:3, :3], T[:3, 3]


def exp_se3(xi: np.ndarray) -> np.ndarray:
    xi = np.asarray(xi, dtype=float).reshape(6)
    v = xi[:3]
    w = xi[3:]

    R = exp_so3(w)
    J = left_jacobian_so3(w)
    t = J @ v

    return make_transform(R, t)


def integrate_pose(T: np.ndarray, xi: np.ndarray, dt: float) -> np.ndarray:
    return T @ exp_se3(np.asarray(xi, dtype=float) * dt)


def transform_points_se3(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])
    transformed = (T @ points_h.T).T
    return transformed[:, :3]


def generate_se3_trajectory(xi, dt, steps):
    T = np.eye(4)
    poses = [T.copy()]
    for _ in range(steps):
        T = integrate_pose(T, xi, dt)
        poses.append(T.copy())
    return poses
