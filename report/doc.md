## examples/pga_vs_se3.py

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.viewers import pga_rigid_body_viewer, se3_rigid_body_viewer


def main():
    se3_rigid_body_viewer()
    pga_rigid_body_viewer()


if __name__ == "__main__":
    main()

```

## src/__init__.py

```python
from src.math import (
    exp_se3,
    exp_so3,
    generate_motor_trajectory,
    generate_se3_trajectory,
    integrate_pose,
    left_jacobian_so3,
    make_transform,
    motor_from_rt,
    motor_from_twist,
    motor_identity,
    motor_mul,
    motor_normalize,
    motor_reverse,
    motor_to_rt,
    skew,
    split_transform,
    transform_points_motor,
    transform_points_se3,
)
from src.viewers import pga_rigid_body_viewer, se3_rigid_body_viewer


__all__ = [
    "exp_se3",
    "exp_so3",
    "generate_motor_trajectory",
    "generate_se3_trajectory",
    "integrate_pose",
    "left_jacobian_so3",
    "make_transform",
    "motor_from_rt",
    "motor_from_twist",
    "motor_identity",
    "motor_mul",
    "motor_normalize",
    "motor_reverse",
    "motor_to_rt",
    "pga_rigid_body_viewer",
    "se3_rigid_body_viewer",
    "skew",
    "split_transform",
    "transform_points_motor",
    "transform_points_se3",
]

```

## src/helpers/__init__.py

```python
from src.helpers.geometry import make_box
from src.helpers.plotting import (
    draw_box,
    draw_box_motor,
    draw_box_se3,
    draw_frame_from_rt,
    draw_frame_motor,
    draw_frame_se3,
    set_axes_equal,
)
from src.helpers.widgets import (
    bind_time_range,
    motion_controls,
    playback_controls,
    scene_controls,
)


__all__ = [
    "bind_time_range",
    "draw_box",
    "draw_box_motor",
    "draw_box_se3",
    "draw_frame_from_rt",
    "draw_frame_motor",
    "draw_frame_se3",
    "make_box",
    "motion_controls",
    "playback_controls",
    "scene_controls",
    "set_axes_equal",
]

```

## src/helpers/geometry.py

```python
import numpy as np


def make_box(lx=1.0, ly=0.6, lz=0.4):
    x = lx / 2
    y = ly / 2
    z = lz / 2

    vertices = np.array(
        [
            [-x, -y, -z],
            [x, -y, -z],
            [x, y, -z],
            [-x, y, -z],
            [-x, -y, z],
            [x, -y, z],
            [x, y, z],
            [-x, y, z],
        ]
    )

    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]

    return vertices, edges

```

## src/helpers/plotting.py

```python
import numpy as np

from src.math.pga import motor_to_rt, transform_points_motor
from src.math.se3 import split_transform, transform_points_se3


def draw_box(ax, vertices_world, edges, color="blue", alpha=1.0, lw=2.0):
    for i, j in edges:
        p1 = vertices_world[i]
        p2 = vertices_world[j]
        ax.plot(
            [p1[0], p2[0]],
            [p1[1], p2[1]],
            [p1[2], p2[2]],
            color=color,
            alpha=alpha,
            linewidth=lw,
        )

    return vertices_world


def draw_box_se3(ax, T, vertices_body, edges, color="blue", alpha=1.0, lw=2.0):
    vertices_world = transform_points_se3(T, vertices_body)
    return draw_box(ax, vertices_world, edges, color=color, alpha=alpha, lw=lw)


def draw_box_motor(ax, M, vertices_body, edges, color="blue", alpha=1.0, lw=2.0):
    vertices_world = transform_points_motor(M, vertices_body)
    return draw_box(ax, vertices_world, edges, color=color, alpha=alpha, lw=lw)


def draw_frame_from_rt(ax, R, t, scale=0.3):
    origin = t
    ex = origin + scale * R[:, 0]
    ey = origin + scale * R[:, 1]
    ez = origin + scale * R[:, 2]

    ax.plot([origin[0], ex[0]], [origin[1], ex[1]], [origin[2], ex[2]], linewidth=2)
    ax.plot([origin[0], ey[0]], [origin[1], ey[1]], [origin[2], ey[2]], linewidth=2)
    ax.plot([origin[0], ez[0]], [origin[1], ez[1]], [origin[2], ez[2]], linewidth=2)


def draw_frame_se3(ax, T, scale=0.3):
    R, t = split_transform(T)
    draw_frame_from_rt(ax, R, t, scale=scale)


def draw_frame_motor(ax, M, scale=0.3):
    R, t = motor_to_rt(M)
    draw_frame_from_rt(ax, R, t, scale=scale)


def set_axes_equal(ax, points):
    pts = np.asarray(points, dtype=float)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)

    centers = 0.5 * (mins + maxs)
    radius = 0.5 * np.max(maxs - mins)

    if radius < 1e-6:
        radius = 1.0

    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)

```

## src/helpers/widgets.py

```python
import ipywidgets as widgets


def motion_controls():
    vx = widgets.FloatSlider(value=1.0, min=-2.0, max=2.0, step=0.1, description="v_x")
    vy = widgets.FloatSlider(value=0.0, min=-2.0, max=2.0, step=0.1, description="v_y")
    vz = widgets.FloatSlider(value=0.2, min=-2.0, max=2.0, step=0.1, description="v_z")

    wx = widgets.FloatSlider(
        value=0.0, min=-3.0, max=3.0, step=0.1, description="omega_x"
    )
    wy = widgets.FloatSlider(
        value=0.0, min=-3.0, max=3.0, step=0.1, description="omega_y"
    )
    wz = widgets.FloatSlider(
        value=1.0, min=-3.0, max=3.0, step=0.1, description="omega_z"
    )

    return vx, vy, vz, wx, wy, wz


def scene_controls():
    lx = widgets.FloatSlider(value=1.0, min=0.2, max=3.0, step=0.1, description="L_x")
    ly = widgets.FloatSlider(value=0.6, min=0.2, max=3.0, step=0.1, description="L_y")
    lz = widgets.FloatSlider(value=0.4, min=0.2, max=3.0, step=0.1, description="L_z")

    dt = widgets.FloatSlider(value=0.05, min=0.01, max=0.2, step=0.01, description="dt")
    steps = widgets.IntSlider(value=150, min=20, max=400, step=10, description="steps")
    frame_scale = widgets.FloatSlider(
        value=0.4, min=0.1, max=1.5, step=0.1, description="frame"
    )

    return lx, ly, lz, dt, steps, frame_scale


def playback_controls(max_steps=150):
    time_slider = widgets.IntSlider(
        value=0, min=0, max=max_steps, step=1, description="t step"
    )
    play = widgets.Play(
        value=0, min=0, max=max_steps, step=1, interval=80, description="Play"
    )
    widgets.jslink((play, "value"), (time_slider, "value"))

    show_trace = widgets.Checkbox(value=True, description="show trajectory")
    show_frame = widgets.Checkbox(value=True, description="show body frame")
    show_start_body = widgets.Checkbox(value=True, description="show start pose")

    return time_slider, play, show_trace, show_frame, show_start_body


def bind_time_range(steps, time_slider, play):
    def refresh_time_range(*args):
        time_slider.max = steps.value
        play.max = steps.value
        if time_slider.value > steps.value:
            time_slider.value = steps.value

    steps.observe(refresh_time_range, names="value")
    refresh_time_range()
    return refresh_time_range

```

## src/math/__init__.py

```python
from src.math.constants import EPS, SMALL_ANGLE_EPS
from src.math.pga import (
    generate_motor_trajectory,
    motor_from_rt,
    motor_from_twist,
    motor_identity,
    motor_mul,
    motor_normalize,
    motor_reverse,
    motor_to_rt,
    transform_points_motor,
    quat_conj,
    quat_from_axis_angle,
    quat_mul,
    quat_norm,
    quat_normalize,
    quat_rotate,
    quat_to_rotmat,
)
from src.math.se3 import (
    exp_se3,
    exp_so3,
    generate_se3_trajectory,
    integrate_pose,
    left_jacobian_so3,
    make_transform,
    skew,
    split_transform,
    transform_points_se3,
)


__all__ = [
    "EPS",
    "SMALL_ANGLE_EPS",
    "exp_se3",
    "exp_so3",
    "generate_motor_trajectory",
    "generate_se3_trajectory",
    "integrate_pose",
    "left_jacobian_so3",
    "make_transform",
    "motor_from_rt",
    "motor_from_twist",
    "motor_identity",
    "motor_mul",
    "motor_normalize",
    "motor_reverse",
    "motor_to_rt",
    "quat_conj",
    "quat_from_axis_angle",
    "quat_mul",
    "quat_norm",
    "quat_normalize",
    "quat_rotate",
    "quat_to_rotmat",
    "skew",
    "split_transform",
    "transform_points_motor",
    "transform_points_se3",
]

```

## src/math/constants.py

```python
EPS = 1e-12
SMALL_ANGLE_EPS = 1e-8

```

## src/math/pga/__init__.py

```python
from src.math.pga.motors import (
    generate_motor_trajectory,
    motor_from_rt,
    motor_from_twist,
    motor_identity,
    motor_mul,
    motor_normalize,
    motor_reverse,
    motor_to_rt,
    transform_points_motor,
)
from src.math.pga.quaternions import (
    quat_conj,
    quat_from_axis_angle,
    quat_mul,
    quat_norm,
    quat_normalize,
    quat_rotate,
    quat_to_rotmat,
)


__all__ = [
    "generate_motor_trajectory",
    "motor_from_rt",
    "motor_from_twist",
    "motor_identity",
    "motor_mul",
    "motor_normalize",
    "motor_reverse",
    "motor_to_rt",
    "quat_conj",
    "quat_from_axis_angle",
    "quat_mul",
    "quat_norm",
    "quat_normalize",
    "quat_rotate",
    "quat_to_rotmat",
    "transform_points_motor",
]

```

## src/math/pga/motors.py

```python
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

```

## src/math/pga/quaternions.py

```python
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

```

## src/math/se3/__init__.py

```python
from src.math.se3.rotations import exp_so3, left_jacobian_so3, skew
from src.math.se3.transforms import (
    exp_se3,
    generate_se3_trajectory,
    integrate_pose,
    make_transform,
    split_transform,
    transform_points_se3,
)


__all__ = [
    "exp_se3",
    "exp_so3",
    "generate_se3_trajectory",
    "integrate_pose",
    "left_jacobian_so3",
    "make_transform",
    "skew",
    "split_transform",
    "transform_points_se3",
]

```

## src/math/se3/rotations.py

```python
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

```

## src/math/se3/transforms.py

```python
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

```

## src/viewers.py

```python
import matplotlib.pyplot as plt
import numpy as np
import ipywidgets as widgets
from IPython.display import display
from ipywidgets import HBox, VBox

from src.helpers.geometry import make_box
from src.helpers.plotting import (
    draw_box_motor,
    draw_box_se3,
    draw_frame_motor,
    draw_frame_se3,
    set_axes_equal,
)
from src.helpers.widgets import (
    bind_time_range,
    motion_controls,
    playback_controls,
    scene_controls,
)
from src.math.pga import generate_motor_trajectory, motor_to_rt
from src.math.se3 import generate_se3_trajectory


def _observe_update(widgets, update):
    for wdg in widgets:
        wdg.observe(update, names="value")


def _rigid_body_viewer(
    title,
    generate_trajectory,
    get_origin,
    draw_box,
    draw_frame,
):
    vx, vy, vz, wx, wy, wz = motion_controls()
    lx, ly, lz, dt, steps, frame_scale = scene_controls()
    time_slider, play, show_trace, show_frame, show_start_body = playback_controls()
    output = widgets.Output()

    bind_time_range(steps, time_slider, play)

    def update(*args):
        with output:
            output.clear_output(wait=True)

            v = np.array([vx.value, vy.value, vz.value], dtype=float)
            w = np.array([wx.value, wy.value, wz.value], dtype=float)
            vertices_body, edges = make_box(lx.value, ly.value, lz.value)
            states = generate_trajectory(v, w, dt.value, steps.value)

            k = time_slider.value
            current_state = states[k]
            origins = np.array([get_origin(state) for state in states])

            fig = plt.figure(figsize=(8, 7))
            ax = fig.add_subplot(111, projection="3d")

            if show_trace.value:
                ax.plot(
                    origins[:, 0],
                    origins[:, 1],
                    origins[:, 2],
                    linewidth=1.8,
                    label="trajectory",
                )

            if show_start_body.value:
                draw_box(
                    ax,
                    states[0],
                    vertices_body,
                    edges,
                    color="gray",
                    alpha=0.35,
                    lw=1.2,
                )

            current_vertices = draw_box(
                ax,
                current_state,
                vertices_body,
                edges,
                color="blue",
                alpha=1.0,
                lw=2.0,
            )

            if show_frame.value:
                draw_frame(ax, current_state, scale=frame_scale.value)

            ax.scatter(origins[0, 0], origins[0, 1], origins[0, 2], s=40, label="start")
            ax.scatter(
                origins[k, 0], origins[k, 1], origins[k, 2], s=50, label="current"
            )

            all_points = np.vstack([origins, current_vertices])
            set_axes_equal(ax, all_points)

            ax.set_title(title)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.legend(loc="upper left")

            plt.tight_layout()
            plt.show()

    controls_left = VBox([vx, vy, vz, wx, wy, wz])
    controls_mid = VBox([lx, ly, lz, dt, steps, frame_scale])
    controls_right = VBox([time_slider, play, show_trace, show_frame, show_start_body])
    ui = VBox([HBox([controls_left, controls_mid, controls_right]), output])

    _observe_update(
        [
            vx,
            vy,
            vz,
            wx,
            wy,
            wz,
            lx,
            ly,
            lz,
            dt,
            steps,
            frame_scale,
            time_slider,
            show_trace,
            show_frame,
            show_start_body,
        ],
        update,
    )

    display(ui)
    update()


def se3_rigid_body_viewer():
    def generate_trajectory(v, w, dt, steps):
        xi = np.concatenate([v, w])
        return generate_se3_trajectory(xi, dt, steps)

    _rigid_body_viewer(
        title="Rigid body motion in SE(3)",
        generate_trajectory=generate_trajectory,
        get_origin=lambda T: T[:3, 3],
        draw_box=draw_box_se3,
        draw_frame=draw_frame_se3,
    )


def pga_rigid_body_viewer():
    _rigid_body_viewer(
        title="Rigid body motion in PGA (motor form)",
        generate_trajectory=generate_motor_trajectory,
        get_origin=lambda M: motor_to_rt(M)[1],
        draw_box=draw_box_motor,
        draw_frame=draw_frame_motor,
    )

```

