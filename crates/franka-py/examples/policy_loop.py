"""A low-rate policy loop on a real arm, as first run on a Panda: a jittery 6-10 Hz commander
draws a 4 cm circle with `move_to`, yaws and tilts the tool with `move_by`, then hands a
20-row chunk back to the start to `follow`.

Usage: python policy_loop.py <hostname> [--yes]    (FRANKA_REALTIME=ignore against franka-sim)
"""

import argparse
import math
import time

import numpy as np

import franka

# The collision thresholds of the libfranka examples.
TORQUE = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
FORCE = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
DURATION = 12.0


def rotation_from(R0, R):
    """Yaw about the base z axis and total angle, degrees, of `R` relative to `R0`."""
    relative = R @ R0.T  # the relative rotation in the base frame
    yaw = math.atan2(relative[1, 0], relative[0, 0])
    angle = math.acos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(yaw), math.degrees(angle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hostname")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = parser.parse_args()
    if not args.yes:
        input("This moves the robot: a 4 cm circle, a 10 deg yaw and a 15 deg tilt. Press Enter.")

    robot = franka.Robot(args.hostname)
    print("fci", robot.fci_version, "server", robot.server_version)
    robot.set_collision_behavior(TORQUE, TORQUE, TORQUE, TORQUE, FORCE, FORCE, FORCE, FORCE)
    first = robot.read_once()
    start, R0 = first.O_T_EE[:3, 3].copy(), first.O_T_EE[:3, :3].copy()
    print("start", np.round(start, 4))

    yaws, angles = [], []
    t0 = time.monotonic()
    with robot.cartesian_targets(max_velocity=0.3, max_acceleration=0.5, max_jerk=20.0) as arm:
        yaw_prev = tilt_prev = 0.0
        while (t := time.monotonic() - t0) < DURATION:
            phase = 2 * math.pi * t / 8.0
            arm.move_to(start + [0.04 * (math.cos(phase) - 1.0), 0.04 * math.sin(phase), 0.0])
            yaw = math.radians(10.0) * math.sin(2 * math.pi * t / 6.0)
            tilt = math.radians(15.0) * math.sin(2 * math.pi * t / 5.0)
            arm.move_by([0.0, 0.0, 0.0, tilt - tilt_prev, 0.0, yaw - yaw_prev])  # about base x, z
            yaw_prev, tilt_prev = yaw, tilt
            measured = rotation_from(R0, arm.state().O_T_EE[:3, :3])
            yaws.append(measured[0])
            angles.append(measured[1])
            time.sleep(0.1 + 0.05 * math.sin(7 * t))  # jittery 6-10 Hz
        time.sleep(0.5)  # a gap, then an action chunk of 20 rows back to the start
        arm.follow(np.linspace(arm.target()[:3], start, 20), dt=0.05)
        time.sleep(1.5)
        state = arm.state()
        print("errors", state.current_errors, "success rate", round(state.control_command_success_rate, 3))
        t_stop = time.monotonic()
        arm.stop()  # settles and finishes the motion; the block's __exit__ is then a no-op
        print(f"stop() took {time.monotonic() - t_stop:.3f} s")

    end = robot.read_once()
    print(f"measured yaw from start: {min(yaws):.1f} to {max(yaws):.1f} deg over {len(yaws)} samples")
    print(f"measured total rotation from start: max {max(angles):.1f} deg")
    print("end", np.round(end.O_T_EE[:3, 3], 4), "mode", end.robot_mode, "errors", end.current_errors)


if __name__ == "__main__":
    main()
