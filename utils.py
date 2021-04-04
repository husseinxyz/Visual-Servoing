# -*- coding: utf-8 -*-
"""
Visual Servoing
"""

import numpy as np


def generate_circuit(size=5, radius=2, n_pts=10):
    """ Generates a square circuit centered at (0,0) with round corners

    size: side half-length
    radius: radius of corners
    n_pts: number of points used for the each corner and side
    """

    x, y = [], []

    t = [
        np.linspace(0, np.pi / 2, n_pts),
        np.linspace(np.pi / 2, np.pi, n_pts),
        np.linspace(np.pi, 3 * np.pi / 2, n_pts),
        np.linspace(3 * np.pi / 2, 2 * np.pi, n_pts),
    ]
    side = size - radius
    anchor = [(side, side), (-side, side), (-side, -side), (side, -side)]

    sx = [
        np.linspace(side, -side, n_pts),
        -size * np.ones(n_pts),
        np.linspace(-side, side, n_pts),
        size * np.ones(n_pts),
    ]
    sy = [
        size * np.ones(n_pts),
        np.linspace(side, -side, n_pts),
        -size * np.ones(n_pts),
        np.linspace(-side, side, n_pts),
    ]

    for i in range(4):
        x.append(anchor[i][0] + radius * np.cos(t[i]))
        x.append(sx[i])
        y.append(anchor[i][1] + radius * np.sin(t[i]))
        y.append(sy[i])

    x = np.concatenate(x)
    y = np.concatenate(y)

    return x, y


def c2o(Pc, pose):
    """ Transformation from camera frame to origin frame """
    Xc, Yc = Pc
    x, y, theta = pose
    Xo = np.cos(theta) * Xc - np.sin(theta) * Yc + x
    Yo = np.sin(theta) * Xc + np.cos(theta) * Yc + y
    return Xo, Yo


def o2c(Po, pose):
    """ Transformation from origin frame to camera frame """
    Xo, Yo = Po
    x, y, theta = pose
    X_ = Xo - x
    Y_ = Yo - y
    Xc = np.cos(theta) * X_ + np.sin(theta) * Y_
    Yc = -np.sin(theta) * X_ + np.cos(theta) * Y_
    return Xc, Yc


def closest_point(Po, pose, f):
    """ Compute the closest point within the FoV of the camera """
    Xc, Yc = o2c(Po, pose)
    Xc_ = Xc[(Xc > f) & (-Xc < Yc) & (Yc < Xc)]
    Yc_ = Yc[(Xc > f) & (-Xc < Yc) & (Yc < Xc)]
    i = np.argmin(Xc_ ** 2 + Yc_ ** 2)
    return c2o((Xc_[i], Yc_[i]), pose)


def gen_image(Po, pose, f=1):
    """ One-dimensional image projection """
    Xc, Yc = o2c(Po, pose)
    return f * Yc / Xc


def draw_cam_ego(f, L):
    """ Draw the camera shape in its ego frame """
    return np.array([0, f, f, 0]), np.array([0, L, -L, 0])


def draw_cam_bev(pose, f=1.0, L=1.0):
    """ Draw the camera shape in the origin frame """
    Pc = draw_cam_ego(f, L)
    return c2o(Pc, pose)


def animate_navigation(ax1, ax2, t, pose, omega):
    """ Animation display """

    ax1.clear()
    ax1.set_title(f"Time: {t:2.2f} seconds")
    ax1.set_xlim(xmin=-10, xmax=10)
    ax1.set_ylim(ymin=-10, ymax=10)
    ax1.axis("equal")

    ax2.clear()
    ax2.set_title(f"Image. Yaw rate: {omega:2.2f} rad/s")
    ax2.axis("equal")
    ax2.set_xlim(xmin=1.2, xmax=-1.2)
    ax2.set_ylim(ymin=-1, ymax=1)

    # Draw borders
    ax1.plot([-8, 8, 8, -8, -8], [-8, -8, 8, 8, -8], "k")

    # Draw circuit
    circuit = generate_circuit()
    ax1.plot(circuit[0], circuit[1], "xk")

    # Draw trajectory
    traj_x = [p[0] for p in pose]
    traj_y = [p[1] for p in pose]
    ax1.plot(traj_x, traj_y, "r")

    # Draw camera
    cam = draw_cam_bev(pose[-1])
    ax1.plot(cam[0], cam[1], "g")

    Po = closest_point(circuit, pose[-1], 1.0)
    ax1.plot(Po[0], Po[1], "bo")

    # Draw image
    s = gen_image(Po, pose[-1])
    ax2.plot([-1.2, 1.2], [0, 0], "k")
    ax2.plot([-1, -1], [-0.2, 0.2], "k")
    ax2.plot([1, 1], [-0.2, 0.2], "k")
    ax2.plot(s, 0, "xk")
    ax2.plot(s, 0, "bo")

    return s


def on_press(event, ani):
    """ Start/stop animation pressing space """
    if event.key.isspace():
        if ani.running:
            ani.event_source.stop()
        else:
            ani.event_source.start()
        ani.running ^= True
