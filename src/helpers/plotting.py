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
