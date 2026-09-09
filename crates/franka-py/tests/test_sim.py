"""The Python bindings against franka-sim (--enforce-motion-limits); the `sim` fixture is in
conftest.py. Run under the simulator lock: `flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev
pytest crates/franka-py/tests`.
"""

import time

import numpy as np
import pytest

import franka

TORQUE = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
FORCE = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]


@pytest.fixture
def robot(sim):
    """A fresh connection per test, released before the next one takes the single FCI slot."""
    robot = franka.Robot(sim)
    if robot.read_once().robot_mode == "reflex":  # left behind on a persistent server
        robot.automatic_error_recovery()
    robot.set_collision_behavior(TORQUE, TORQUE, TORQUE, TORQUE, FORCE, FORCE, FORCE, FORCE)
    robot.set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])
    robot.set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])
    return robot



def quat_rotation(xyzw):
    """Rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def same_rotation(a, b, atol=1e-6):
    """Whether two unit quaternions are the same rotation, either sign."""
    return abs(abs(np.dot(a, b)) - 1.0) < atol


def rotation_angle(a, b):
    """Angle, rad, between two rotation matrices."""
    return np.arccos(np.clip((np.trace(a.T @ b) - 1.0) / 2.0, -1.0, 1.0))


def idle_state(robot, timeout=3.0):
    deadline = time.monotonic() + timeout
    while True:
        state = robot.read_once()
        if state.robot_mode == "idle" or time.monotonic() > deadline:
            return state


def test_connect_and_read_once(robot):
    assert robot.fci_version in (5, 10)
    assert robot.server_version > 0
    state = robot.read_once()
    assert state.q.shape == (7,) and state.O_T_EE.shape == (4, 4)
    assert state.O_T_EE[3, 3] == 1.0 and state.I_total.shape == (3, 3)
    flat = state.flat()
    assert flat.shape == (franka.FLAT_LEN,) and flat.dtype == np.float64
    assert franka.FLAT_LAYOUT[-1] == ("time", franka.FLAT_LEN - 1, franka.FLAT_LEN)
    for name, start, stop in franka.FLAT_LAYOUT[:4]:
        np.testing.assert_array_equal(flat[start:stop], getattr(state, name))
    np.testing.assert_array_equal(flat[28:31], state.O_T_EE[:3, 3])
    assert state.robot_mode == "idle" and state.current_errors == []


def test_cartesian_targets_move_by_then_follow(robot):
    with robot.cartesian_targets(max_velocity=0.3, max_acceleration=0.5, max_jerk=20.0) as arm:
        assert arm.running
        start = arm.target()
        assert start.shape == (7,)
        for _ in range(20):  # +5 cm in x at 10 Hz for 2 s
            arm.move_by([0.0025, 0.0, 0.0])
            obs = arm.state()
            assert obs.flat().shape == (franka.FLAT_LEN,)
            time.sleep(0.1)
        np.testing.assert_allclose(arm.target(), start + np.r_[0.05, 0.0, 0.0, np.zeros(4)])
        # an absolute orientation: the start orientation yawed by 10 deg about the base z axis
        yaw = np.radians(10.0)
        goal = franka.rotated(start[3:], [0.0, 0.0, yaw])
        arm.move_to(np.r_[arm.target()[:3], goal])
        time.sleep(1.5)
        assert same_rotation(arm.target()[3:], goal)
        assert rotation_angle(arm.state().O_T_EE[:3, :3], quat_rotation(goal)) < 2e-2
        arm.move_by([0.0, 0.0, 0.0, 0.0, 0.0, -yaw])  # a rotation vector: back to the start
        time.sleep(1.5)
        assert same_rotation(arm.target()[3:], start[3:])
        assert rotation_angle(arm.state().O_T_EE[:3, :3], arm.target_pose()[:3, :3]) < 0.01
        for bad in ([0.0, 0.0], np.r_[start[:3], 0.0, 0.0, 0.0], np.r_[start[:3], np.zeros(4)]):
            with pytest.raises(ValueError):
                arm.move_to(bad)
        with pytest.raises(ValueError):
            arm.move_by(np.r_[start[:3], 0.0, 0.0, 0.0, 1.0])
        # (N, 7) rows back to the start pose
        rows = np.linspace(start + np.r_[0.05, 0.0, 0.0, np.zeros(4)], start, 20)
        arm.follow(rows, dt=0.02)
        time.sleep(1.5)
        np.testing.assert_allclose(arm.target(), start, atol=1e-9)
        assert np.linalg.norm(arm.state().O_T_EE[:3, 3] - start[:3]) < 0.005
    assert not arm.running
    with pytest.raises(franka.FrankaError):
        arm.move_to(start)
    state = idle_state(robot)
    assert state.robot_mode == "idle"
    assert np.linalg.norm(state.O_T_EE[:3, 3] - start[:3]) < 0.005
    assert rotation_angle(state.O_T_EE[:3, :3], quat_rotation(start[3:])) < 0.01


def test_joint_targets_joint_7(robot):
    with robot.joint_targets(fraction=0.2) as arm:
        start = arm.target()
        step = np.zeros(7)
        step[6] = 0.015
        for _ in range(20):  # +0.3 rad on joint 7 at 10 Hz for 2 s
            arm.move_by(step)
            time.sleep(0.1)
        np.testing.assert_allclose(arm.target(), start + 20 * step)
        chunk = start + np.linspace(20 * step, np.zeros(7), 20)
        arm.follow(chunk, dt=0.02)
        time.sleep(1.5)
        np.testing.assert_allclose(arm.target(), chunk[-1])
        assert abs(arm.state().q[6] - chunk[-1][6]) < 0.005
    state = idle_state(robot)
    assert state.robot_mode == "idle"
    assert abs(state.q[6] - chunk[-1][6]) < 0.005


def test_exception_in_block_still_stops(robot):
    with pytest.raises(ZeroDivisionError):
        with robot.joint_targets() as arm:
            arm.move_by([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05])
            1 / 0
    assert not arm.running
    assert idle_state(robot).robot_mode == "idle"


def test_model(robot):
    model = robot.model()
    state = idle_state(robot)
    q = state.q
    ee = model.pose("ee", q, state.F_T_EE, state.EE_T_K)
    # franka-sim publishes the joint-7 origin as O_T_EE (tests/sim_commands.rs): the rotation
    # agrees, the origin up to the FR3's 0.107 m joint7 -> flange offset along the tool z.
    np.testing.assert_allclose(ee[:3, :3], state.O_T_EE[:3, :3], atol=1e-6)
    np.testing.assert_allclose(model.pose("joint7", q)[:3, 3], state.O_T_EE[:3, 3], atol=1e-6)
    np.testing.assert_allclose(ee[:3, 3] - state.O_T_EE[:3, 3], 0.107 * ee[:3, 2], atol=1e-6)
    poses = model.link_poses(q)
    assert poses.shape == (8, 4, 4)
    np.testing.assert_array_equal(poses[0], np.eye(4))
    np.testing.assert_allclose(poses[7], model.pose("joint7", q))
    hand = model.hand_pose(q)
    np.testing.assert_allclose(hand[:3, 3], model.pose("flange", q)[:3, 3])
    assert abs(np.linalg.det(hand[:3, :3]) - 1.0) < 1e-9
    for name in ("joint1", "flange", "ee", "stiffness"):
        assert model.body_jacobian(name, q).shape == (6, 7)
        assert model.zero_jacobian(name, q, state.F_T_EE).shape == (6, 7)
    mass = model.mass(q, state.I_total, state.m_total, state.F_x_Ctotal)
    np.testing.assert_allclose(mass, mass.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(mass) > 0.0)
    assert np.all(np.isfinite(model.gravity(q, state.m_total, state.F_x_Ctotal)))
    assert model.coriolis(q, np.zeros(7)).shape == (7,)
    with pytest.raises(ValueError):
        model.pose("link3", q)
    with pytest.raises(ValueError):
        model.pose("ee", q[:6])


def test_gripper(robot):
    gripper = robot.gripper()
    assert gripper.read_once().max_width > 0.0
    assert gripper.move(0.04, 0.1)
    assert abs(gripper.read_once().width - 0.04) < 0.005


def test_bad_address_raises_franka_error():
    with pytest.raises(franka.FrankaError) as info:
        franka.Robot("127.0.0.1:1")
    assert info.value.kind == "network"
    assert issubclass(franka.ControlException, franka.FrankaError)
