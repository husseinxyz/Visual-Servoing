# -*- coding: utf-8 -*-
"""
Visual Servoing
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from utils import animate_navigation, on_press

# Initial pose (m)
pose = [(0.0, -5.0, 0.0)]
# Initial yaw rate (rad/d)
omega = 0.0
# Longitudinal velocity (m/s)
v = 1.0
# Time discretization (s)
dt = 0.1
# Duration of animation (s)
Dt = 100

# Yaw rate control
def update_yaw_rate(s):
    """ Update the value of the yaw rate control given the input s """
    global omega
    # TODO
    omega = 0.0


# Update pose
def update_pose(dt, std=0.0):
    """ Update the robot pose with optional noise on the heading """
    global pose
    # Current pose
    x_, y_, theta_ = pose[-1]
    e_ = np.random.normal(scale=std)
    # New pose
    x = x_ + v * np.cos(theta_ + e_) * dt
    y = y_ + v * np.sin(theta_ + e_) * dt
    theta = theta_ + omega * dt + e_
    pose.append((x, y, theta))


def update_state(ax1, ax2, i, dt=0.1):
    """ Update state and display the animation """
    update_pose(dt)
    try:
        s = animate_navigation(ax1, ax2, i * dt, pose, omega)
    except ValueError:
        print("The circuit is out of the field of view of the camera.")
        global ani
        ani.event_source.stop()
        ani.running = False
    update_yaw_rate(s)


# Run simulation
fig = plt.figure("Simulation")
fig.clf()
ax1 = fig.add_subplot(1, 2, 1)
ax2 = fig.add_subplot(1, 2, 2)

ani = animation.FuncAnimation(
    fig,
    lambda i: update_state(ax1, ax2, i, dt),
    frames=range(int(Dt / dt)),
    blit=False,
    interval=int(dt * 1000),
    repeat=False,
)
ani.running = True

fig.canvas.mpl_connect("key_press_event", lambda event: on_press(event, ani))

plt.show()
