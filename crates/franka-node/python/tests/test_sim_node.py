"""The client against the `franka-node` binary and franka-sim (--enforce-motion-limits). Run under
the simulator lock after `cargo build --release -p franka-node`:
`flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev pytest crates/franka-node/python/tests`.
"""

import dataclasses
import time

import numpy as np
import pytest

import franka_node
from conftest import NODE_NAME, SIM_ARM
from franka_node import NodeError, Refused, SessionEnded, _wire

pytestmark = pytest.mark.sim

READY = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])
# franka-sim publishes O_T_EE at the joint-7 frame, this far short of the end effector along the
# tool z (docs/book/src/reference/simulator-gaps.md); on a robot the two coincide.
SIM_FLANGE = 0.107


@pytest.fixture(autouse=True)
def robot_pose(monkeypatch):
    """Decodes the simulator's `O_T_EE` as a robot publishes it, the end-effector pose."""
    decode = _wire.decode_state

    def at_the_end_effector(*args):
        state = decode(*args)
        pose = np.array(state.O_T_EE)
        pose[:3, 3] += SIM_FLANGE * pose[:3, 2]
        return dataclasses.replace(state, O_T_EE=pose, position=pose[:3, 3].copy())

    monkeypatch.setattr(_wire, "decode_state", at_the_end_effector)


@pytest.fixture
def node(node_endpoint):
    with franka_node.Node(node_endpoint) as node:
        node.arm(SIM_ARM).state(timeout=30.0)  # the node connects to the simulator first
        yield node


def until(arm, accept, timeout):
    deadline = time.monotonic() + timeout
    while not accept(state := arm.state()):
        assert time.monotonic() < deadline, state
        time.sleep(0.01)
    return state


def test_home_first(node):
    with node.arm(SIM_ARM) as arm:
        arm.home(speed=0.3)
        state = arm.state()
    assert np.max(np.abs(state.q - READY)) < 0.02


def test_the_book_sequence(node):
    with node.arm(SIM_ARM) as arm:
        with arm.cartesian_targets() as t:
            start = t.state()
            # The session's seed is the loop's pose: the decoded pose must be the same frame.
            np.testing.assert_allclose(start.position, t.target()[:3], atol=0.002)
            t.move_by([0.0, 0.0, 0.05])
            state = t.wait(timeout=10.0)
            print(f"+5 cm: z {start.position[2]:.4f} -> {state.position[2]:.4f}, "
                  f"refused {state.refused}, round trip {state.round_trip}")
            assert state.position[2] - start.position[2] == pytest.approx(0.05, abs=0.005)
            assert state.refused == 0
        assert arm.gripper.move(0.04)


def test_a_second_node_is_refused_while_held(node, node_endpoint):
    with node.arm(SIM_ARM):
        with franka_node.Node(node_endpoint) as other:
            with pytest.raises(Refused, match=f"held by {node.client_id}"):
                other.arm(SIM_ARM).acquire()


def test_move_to_arrives_inside_the_guard_with_the_lead_limits_on(node):
    with node.arm(SIM_ARM) as arm, arm.cartesian_targets() as t:
        goal = t.target()
        goal[0] += 0.10
        leads = []
        subscription = arm.on_state(
            lambda s: leads.append(np.linalg.norm(s.target[:3] - s.position)))
        t.move_to(goal[:3])
        state = t.wait(timeout=15.0)
        subscription.close()
        print(f"10 cm in x: tracking error peak {max(leads):.4f} m, accepted {state.accepted}")
        assert np.linalg.norm(state.position - goal[:3]) <= 0.005
        assert state.refused == 0
    # Back without a velocity bound: the lead clamp alone paces the stream, under the node's
    # lead and anchored-step checks.
    with node.arm(SIM_ARM) as arm, arm.cartesian_targets(max_velocity=None) as t:
        goal[0] -= 0.10
        t.move_to(goal)
        state = t.wait(timeout=15.0)
        assert np.linalg.norm(state.position - goal[:3]) <= 0.005
        assert state.refused == 0


def test_joints_move_by_joint_7(node):
    with node.arm(SIM_ARM) as arm, arm.joint_targets() as t:
        q0 = t.state().q
        t.move_by([0.0] * 6 + [0.1])
        state = t.wait(timeout=10.0)
        assert state.q[6] - q0[6] == pytest.approx(0.1, abs=0.01)
        assert state.refused == 0


def test_the_lease_goes_with_a_node(node, node_endpoint):
    watch = node.arm(SIM_ARM)
    holder = franka_node.Node(node_endpoint)
    holder.arm(SIM_ARM).acquire()
    until(watch, lambda s: s.holder == holder.client_id, 3.0)
    holder.close()
    until(watch, lambda s: s.holder == 0, 3.0)

    # As at interpreter exit: no verbs, the token goes with the session and the node stops.
    holder = franka_node.Node(node_endpoint)
    arm = holder.arm(SIM_ARM)
    arm.acquire()
    arm.cartesian_targets()
    until(watch, lambda s: s.holder == holder.client_id and s.phase == "active", 3.0)
    holder._abandon()
    until(watch, lambda s: s.holder == 0 and s.phase == "idle", 5.0)


def test_an_external_stop_ends_the_session(node, node_endpoint):
    with node.arm(SIM_ARM) as arm:
        t = arm.cartesian_targets()
        with franka_node.Node(node_endpoint) as other:
            other.arm(SIM_ARM).stop()
        with pytest.raises(SessionEnded) as ended:
            t.move_by([0.0, 0.0, 0.01])
        assert ended.value.phase in ("stopping", "idle")
        t.stop()


def test_the_gripper(node, node_endpoint):
    with franka_node.Node(node_endpoint) as other:
        with pytest.raises(NodeError, match="lease"):
            other.arm(SIM_ARM).gripper.move(0.03)
    with node.arm(SIM_ARM) as arm:
        gripper = arm.gripper
        assert gripper.homing()
        assert gripper.move(0.04)
        assert gripper.state().width == pytest.approx(0.04, abs=0.003)
        assert not gripper.grasp(0.02, 10.0)  # thin air: the sim reports no grasp
        gripper.stop()


def test_status_names_the_arm(node):
    status = node.status(name=NODE_NAME, timeout=3.0)
    assert status["arms"][0]["name"] == SIM_ARM
