"""Yaw the tool 20 deg with a quaternion target, tilt it 10 deg with a rotation vector, return.
Usage: python rotate.py <hostname>   (FRANKA_REALTIME=ignore against franka-sim)"""

import sys
import time

import numpy as np

import franka


robot = franka.Robot(sys.argv[1])
R0 = robot.read_once().O_T_EE[:3, :3]
with robot.cartesian_targets(max_velocity=0.3) as arm:
    start = arm.target()  # position (m) and unit quaternion (x, y, z, w)
    yawed = np.r_[start[:3], franka.rotated(start[3:], [0.0, 0.0, np.radians(20.0)])]
    tilt = [0.0, 0.0, 0.0, np.radians(10.0), 0.0, 0.0]  # a rotation vector about the base x axis
    steps = ("yaw", arm.move_to, yawed), ("tilt", arm.move_by, tilt), ("back", arm.move_to, start)
    for step, move, target in steps:
        move(target)
        time.sleep(2.0)
        cos = (np.trace(R0.T @ arm.state().O_T_EE[:3, :3]) - 1) / 2
        print(f"{step}: {np.degrees(np.arccos(np.clip(cos, -1, 1))):.1f} deg from the start orientation")
