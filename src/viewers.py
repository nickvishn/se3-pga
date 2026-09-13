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
