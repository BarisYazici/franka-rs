"""`Node` and `Arm` against the loopback `fake` node: the reply mapping, the timeouts, the lease,
the exit rules, the status and a wrong wire version. No node binary, no simulator."""

import json
import threading
import time

import pytest
import zenoh

import franka_node
from franka_node import NodeError, NodeTimeout, ProtocolError, Refused


@pytest.fixture
def node(fake):
    with franka_node.Node(fake.endpoint) as node:
        node.arm(fake.ARM).state(timeout=5.0)  # the link is up
        yield node


def test_client_ids_are_random_non_zero_and_counters_start_at_one(fake):
    with franka_node.Node(fake.endpoint) as a, franka_node.Node(fake.endpoint) as b:
        assert a.client_id != b.client_id and 0 < a.client_id < 2**32
        assert [a._next_seq("target"), a._next_seq("target"), a._next_seq("gripper")] == [1, 2, 1]


def test_replies_map_to_none_or_refused(node, fake):
    arm = node.arm(fake.ARM)
    arm.recover()
    assert fake.requests[-1] == ("recover", {"client_id": node.client_id})
    fake.overrides["recover"] = {"ok": False, "error": "not allowed in active"}
    with pytest.raises(Refused) as refused:
        arm.recover()
    assert (refused.value.verb, refused.value.reason) == ("recover", "not allowed in active")


def test_no_reply_is_a_timeout(fake):
    with franka_node.Node(fake.endpoint) as node:
        began = time.monotonic()
        with pytest.raises(NodeTimeout):
            node._cmd("nobody", "acquire", timeout=0.3)
        assert time.monotonic() - began < 2.0


def test_acquire_retries_no_lease_then_holds_and_exit_releases(node, fake):
    fake.overrides["acquire"] = {"ok": False, "error": "no lease"}
    threading.Timer(0.2, fake.overrides.clear).start()
    arm = node.arm(fake.ARM)
    with arm:
        assert [verb for verb, _ in fake.requests].count("acquire") >= 2
        assert node.client_id in fake.leases
        arm._wait_state(lambda s: s.holder == node.client_id, 2.0)
    assert fake.requests[-1][0] == "release"
    time.sleep(0.2)
    assert node.client_id not in fake.leases


def test_acquire_held_by_another_raises_and_undeclares(node, fake):
    fake.state["client_id"] = 99
    with pytest.raises(Refused, match="held by 99"):
        node.arm(fake.ARM).acquire()
    assert node.arm(fake.ARM)._token is None


def test_exit_does_not_release_an_arm_held_by_another(node, fake):
    arm = node.arm(fake.ARM)
    arm.acquire()
    arm._wait_state(lambda s: s.holder == node.client_id, 2.0)
    fake.state["client_id"] = 99
    arm._wait_state(lambda s: s.holder == 99, 2.0)
    arm.__exit__(None, None, None)
    assert "release" not in [verb for verb, _ in fake.requests]
    assert arm._token is None


def test_stop_not_allowed_in_idle_is_success_and_in_faulted_is_not(node, fake):
    node.arm(fake.ARM).stop()
    assert fake.requests[-1][0] == "stop"
    with fake.lock:
        fake.state["phase"] = 4
    with pytest.raises(Refused, match="not allowed in faulted"):
        node.arm(fake.ARM).stop()


def test_the_fake_keeps_the_node_phase_rules(node, fake):
    arm = node.arm(fake.ARM)
    with pytest.raises(Refused, match="is not the holder"):
        arm.cartesian_targets()
    arm.acquire()
    with arm.cartesian_targets():
        with pytest.raises(Refused, match="not allowed in active"):
            arm.release()
    arm.release()


def test_home_acquires_first_and_sends_speed_and_episode(node, fake):
    node.arm(fake.ARM).home(0.3, episode="pick-0042")
    verbs = [verb for verb, _ in fake.requests]
    assert verbs[-1] == "home" and "acquire" in verbs
    request = {"client_id": node.client_id, "speed": 0.3, "episode": "pick-0042"}
    assert fake.requests[-1][1] == request


def test_status_is_the_parsed_json(node, fake):
    publisher = fake.session.declare_publisher("franka/node/fake-node/status")
    stop = threading.Event()

    def publish():
        while not stop.is_set():
            publisher.put(json.dumps({"node": "fake-node", "arms": [{"name": "fake"}]}))
            time.sleep(0.05)

    thread = threading.Thread(target=publish)
    thread.start()
    try:
        assert node.status()["arms"][0]["name"] == "fake"
        assert node.status(name="fake-node")["node"] == "fake-node"
        with pytest.raises(NodeTimeout):
            node.status(name="other", timeout=0.3)
    finally:
        stop.set()
        thread.join()


def test_a_wrong_wire_version_is_a_protocol_error(node, fake):
    arm = node.arm(fake.ARM)
    with fake.lock:
        fake.state["version"] = 2
    with pytest.raises(ProtocolError, match="wire version 2"):
        arm._wait_state(lambda s: False, 2.0)
    with fake.lock:
        fake.state["version"] = 1
    time.sleep(0.1)
    assert arm._wait_state(lambda s: True, 2.0).phase == "idle"


def test_on_state_callbacks_and_close(node, fake):
    seen = []
    subscription = node.arm(fake.ARM).on_state(seen.append)
    time.sleep(0.2)
    subscription.close()
    count = len(seen)
    time.sleep(0.1)
    assert count >= 5 and len(seen) == count
    assert isinstance(seen[0], franka_node.ArmState)


def test_a_borrowed_session_is_not_closed(fake):
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([fake.endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    session = zenoh.open(config)
    try:
        with franka_node.Node(session=session) as node:
            node.arm(fake.ARM).state(timeout=5.0)
        assert not session.is_closed()
        with pytest.raises(NodeError):
            node.arm(fake.ARM)
    finally:
        session.close()
