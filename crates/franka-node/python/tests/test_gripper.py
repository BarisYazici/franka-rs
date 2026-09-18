"""The gripper's waits against the `fake` node's scripted gripper state: a move seen moving, one
too quick to be seen, a stale reading at rest, one never taken, a grasp, the deadband, the lease."""

import time

import pytest

import franka_node
from franka_node import NodeError, _wire

CALIBRATED, MOVING, GRASPED = _wire.GRIPPER_CALIBRATED, _wire.GRIPPER_MOVING, _wire.GRIPPER_GRASPED


def test_a_move_judged_on_the_reading_after_rest(arm, fake):
    def driver(msg):
        width = float(msg["width"])
        fake.grip(flags=CALIBRATED | MOVING, commanded=width)
        time.sleep(0.15)
        fake.grip(flags=CALIBRATED)  # the command returned; the reader still has the old width
        time.sleep(0.08)
        fake.grip(reading=True, width=width)

    fake.gripper_script = driver
    assert arm.gripper.move(0.04)
    assert arm.gripper.state().width == 0.04


def test_a_move_quicker_than_a_state_period_is_taken_by_commanded(arm, fake):
    fake.gripper_script = lambda msg: fake.grip(
        reading=True, commanded=float(msg["width"]), width=float(msg["width"]))
    assert arm.gripper.move(0.05)


def test_a_command_never_taken_raises(arm, fake):
    began = time.monotonic()
    with pytest.raises(NodeError, match="not taken"):
        arm.gripper.move(0.03)
    assert time.monotonic() - began < 2.0


def test_a_grasp_reports_the_fresh_is_grasped(arm, fake):
    def driver(msg):
        fake.grip(flags=CALIBRATED | MOVING)
        time.sleep(0.1)
        fake.grip(flags=CALIBRATED)
        time.sleep(0.08)
        fake.grip(reading=True, flags=CALIBRATED | GRASPED, width=float(msg["width"]))

    fake.gripper_script = driver
    assert arm.gripper.grasp(0.02, 10.0)


def test_the_deadband_returns_at_once_and_widths_are_checked(arm, fake):
    began = time.monotonic()
    assert arm.gripper.move(0.08)
    assert time.monotonic() - began < 0.5
    with pytest.raises(ValueError):
        arm.gripper.move(0.09)
    with pytest.raises(ValueError):
        arm.gripper.grasp(0.02, -1.0)


def test_commands_need_the_lease(fake):
    with franka_node.Node(fake.endpoint) as node:
        node.arm(fake.ARM).state(timeout=5.0)
        with pytest.raises(NodeError, match="lease"):
            node.arm(fake.ARM).gripper.move(0.04)
