"""The streamer: its step and lead math as pure functions, then a session against the loopback
`fake` node (shapes, normalisation, seq, anchor, keepalive, `wait`, the end of a session)."""

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

import franka_node
from franka_node import SessionEnded, _targets, _wire

RNG = np.random.default_rng(7)


def unit(q):
    return np.asarray(q, dtype=float) / np.linalg.norm(q)


def random_pose():
    return np.r_[RNG.uniform(-0.5, 0.5, 3), unit(RNG.normal(size=4))]


def test_cartesian_step_bounds_the_step_and_the_lead():
    rate, v, w, lead, angular_lead = 50.0, 0.1, 0.3, 0.03, 0.15
    for _ in range(500):
        current, goal = random_pose(), random_pose()
        measured = np.r_[current[:3] + RNG.uniform(-0.02, 0.02, 3), current[3:]]
        nxt = _targets.cartesian_step(current, goal, measured, v, w, lead, angular_lead, rate)
        assert abs(np.linalg.norm(nxt[3:]) - 1.0) < 1e-12
        assert np.linalg.norm(nxt[:3] - measured[:3]) <= lead + 1e-12
        assert _targets.angle_between(nxt[3:], measured[3:]) <= angular_lead + 1e-9
        assert np.linalg.norm(nxt[:3] - current[:3]) <= v / rate + lead + 0.04
        # Without the lead binding, the step toward the goal is the bound.
        free = _targets.cartesian_step(current, goal, current, v, w, 10.0, 10.0, rate)
        assert np.linalg.norm(free[:3] - current[:3]) <= v / rate + 1e-12
        assert _targets.angle_between(free[3:], current[3:]) <= w / rate + 1e-9


def test_cartesian_step_is_sign_insensitive_and_lead_applies_without_a_velocity():
    q = unit([0.1, 0.2, 0.3, 0.9])
    pose = np.r_[0.4, 0.0, 0.4, q]
    same = _targets.cartesian_step(pose, np.r_[pose[:3], -q], pose, 0.1, 0.3, 0.03, 0.15, 50.0)
    assert _targets.angle_between(same[3:], q) < 1e-9
    far = np.r_[pose[:3] + [0.2, 0.0, 0.0], q]
    jump = _targets.cartesian_step(pose, far, pose, None, None, 0.03, 0.15, 50.0)
    assert np.linalg.norm(jump[:3] - pose[:3]) == pytest.approx(0.03)


def test_joint_step_caps_every_message_even_without_a_velocity():
    current = np.zeros(7)
    goal = np.full(7, 1.0)
    nxt = _targets.joint_step(current, goal, current, None, 10.0, 50.0)
    np.testing.assert_allclose(nxt, _targets.JOINT_STEP_CAP)
    nxt = _targets.joint_step(current, goal, current, 0.5, 10.0, 50.0)
    np.testing.assert_allclose(nxt, 0.01)
    measured = np.full(7, -0.5)
    nxt = _targets.joint_step(current, goal, measured, None, 0.15, 50.0)
    assert np.all(np.abs(nxt - current) <= _targets.JOINT_STEP_CAP + 1e-12)


def test_rotated_is_the_franka_composition():
    quarter = _targets.rotated([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, np.pi / 2])
    np.testing.assert_allclose(quarter, [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-12)
    assert franka_node.rotated is _targets.rotated


def sent(fake, since=0.0):
    with fake.lock:
        return [msg for t, msg in fake.targets if t >= since]


def test_targets_are_anchored_in_order_paced_and_kept_alive(arm, fake):
    with arm.cartesian_targets(max_velocity=0.1, rate=50.0) as t:
        t.move_by([0.0, 0.0, 0.01])
        time.sleep(0.6)
        messages = sent(fake)
        seqs = [int(m["seq"]) for m in messages]
        assert seqs == sorted(set(seqs)) and len(messages) >= 20
        assert all(int(m["flags"]) == _wire.TARGET_ANCHOR for m in messages)
        assert all(int(m["client_id"]) == arm._node.client_id for m in messages)
        z = [float(m["data"][2]) for m in messages]
        assert max(np.diff(z)) <= 0.1 / 50.0 + 1e-12
        # 1 cm at 2 mm per message is there after five; the rest is the keepalive.
        assert z[-1] == pytest.approx(0.41) and len(set(z[-10:])) == 1
        # 50 Hz, not woken by the 100 Hz states; loose below for a loaded runner.
        began = time.monotonic()
        time.sleep(0.5)
        assert 5 <= len(sent(fake, began)) <= 35


def test_the_lead_holds_the_target_near_a_still_arm(arm, fake):
    with arm.cartesian_targets(lead=0.03) as t:
        t.move_to([0.5, 0.0, 0.4])
        time.sleep(1.5)
        last = sent(fake)[-1]["data"]
        assert np.linalg.norm(last[:3] - np.array(fake.POSE[:3])) == pytest.approx(0.03)
        assert np.allclose(t.target(), [0.5, 0.0, 0.4, *fake.POSE[3:]])


def test_shapes_and_normalisation(arm, fake):
    with arm.cartesian_targets() as t:
        t.move_to([0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 2.0])
        np.testing.assert_allclose(t.target()[3:], [0.0, 0.0, 0.0, 1.0])
        t.move_by([0.0, 0.0, 0.0, 0.0, 0.0, 0.1])
        assert abs(np.linalg.norm(t.target()[3:]) - 1.0) < 1e-12
        for bad in ([0.1, 0.2], [0.0] * 7, [np.nan, 0.0, 0.0]):
            with pytest.raises(ValueError):
                t.move_by(bad)
        with pytest.raises(ValueError):
            t.move_to([0.1] * 5)
        with pytest.raises(ValueError):
            t.follow([[0.4, 0.0, 0.4]], dt=0.0)


def test_wait_needs_an_accepted_target_equal_to_the_goal(arm, fake):
    fake.track = True
    with arm.cartesian_targets(max_velocity=0.1) as t:
        t.move_by([0.0, 0.0, 0.02])
        state = t.wait(timeout=3.0)
        assert state.position[2] == pytest.approx(0.42, abs=0.005)
        assert state.seq_accepted >= t._goal_seq
        assert state.round_trip is not None and state.round_trip >= 0.0


def test_follow_walks_the_rows(arm, fake):
    fake.track = True
    with arm.cartesian_targets(max_velocity=None, lead=1.0) as t:
        rows = [[0.4, 0.0, 0.4 + 0.005 * i] for i in range(1, 6)]
        t.follow(rows, dt=0.05)
        state = t.wait(timeout=3.0)
        assert state.position[2] == pytest.approx(0.425, abs=1e-6)


def test_the_node_ending_the_session_ends_every_call(arm, fake):
    t = arm.cartesian_targets()
    t.move_by([0.1, 0.0, 0.0])
    errors = []

    def blocked():
        try:
            t.wait(timeout=5.0)
        except SessionEnded as e:
            errors.append(e)

    waiter = threading.Thread(target=blocked)
    waiter.start()
    time.sleep(0.2)
    with fake.lock:
        fake.state["phase"] = 4  # faulted
    waiter.join(2.0)
    assert len(errors) == 1 and errors[0].phase == "faulted"
    with pytest.raises(SessionEnded):
        t.move_by([0.0, 0.0, 0.01])
    count = len(sent(fake))
    time.sleep(0.2)
    assert len(sent(fake)) == count
    t.__exit__(None, None, None)
    assert "stop" not in [verb for verb, _ in fake.requests]


def test_joints_session_steps_per_joint_and_stops_on_exit(arm, fake):
    with arm.joint_targets(max_velocity=0.5, rate=50.0) as t:
        t.move_by([0.0] * 6 + [0.05])
        time.sleep(0.4)
        q7 = [float(m["data"][6]) for m in sent(fake) if int(m["kind"]) == _wire.KINDS["joints"]]
        assert max(np.diff(q7)) <= 0.5 / 50.0 + 1e-12 and q7[-1] == pytest.approx(0.05)
        with pytest.raises(ValueError):
            t.move_to([0.0] * 3)
    assert fake.requests[-1][0] == "stop"


def test_another_holder_ends_the_session_without_a_stop(arm, fake):
    t = arm.cartesian_targets()
    with fake.lock:
        fake.state["client_id"] = 99  # the lease was lost, or the node restarted
    time.sleep(0.1)
    with pytest.raises(SessionEnded) as ended:
        t.move_by([0.0, 0.0, 0.01])
    assert ended.value.phase == "active"
    t.stop()
    assert "stop" not in [verb for verb, _ in fake.requests]


def test_a_silent_node_ends_the_session(arm, fake):
    t = arm.cartesian_targets()
    t.move_by([0.1, 0.0, 0.0])
    began = time.monotonic()
    with fake.lock:
        fake.paused = True
    with pytest.raises(SessionEnded) as ended:
        t.wait(timeout=10.0)
    assert ended.value.phase == "unknown"
    assert 1.9 < time.monotonic() - began < 3.0  # the 2 s minimum at 100 Hz


def test_a_slow_state_rate_is_not_silence(arm, fake):
    t = arm.cartesian_targets()
    with fake.lock:
        fake.state_period = 1.25  # 0.8 Hz: gaps past 1 s, inside the 2 s minimum
    time.sleep(3.0)
    t.move_by([0.0, 0.0, 0.001])
    t.stop()


def test_a_stalled_streamer_waits_for_the_queued_states(arm, fake):
    t = arm.cartesian_targets()
    with t._cv:  # the streamer and the state callbacks stall, as behind a GIL held for 2.5 s
        time.sleep(2.5)
    time.sleep(0.3)
    t.move_by([0.0, 0.0, 0.001])
    t.stop()


@pytest.fixture
def silence_monitor(monkeypatch):
    """Drive the streamer's clock and callback ordering without threads or wall-clock sleeps."""
    clock = SimpleNamespace(now=1_000_000_000)
    monkeypatch.setattr(_targets, "time", SimpleNamespace(monotonic_ns=lambda: clock.now))
    monkeypatch.setattr(_targets._Targets, "_start", lambda self, episode: None)
    arm = SimpleNamespace(_node=SimpleNamespace(client_id=7), _claim=lambda session: None,
                          _silent_after_ns=lambda: 2_000_000_000)
    target = _targets._Targets(arm, None, 50.0)
    state = SimpleNamespace(received_ns=clock.now, has_errors=False, phase="active",
                            robot_mode="move", joints=False, holder=7)
    arm._latest = state
    return target, state, clock


def test_queued_state_can_arrive_several_ticks_after_a_stall(silence_monitor):
    target, state, clock = silence_monitor
    previous = clock.now
    clock.now += 2_500_000_000
    target._note_silence(state, previous)
    # The streamer wins scheduling repeatedly while the Zenoh callback is still queued.
    for _ in range(5):
        previous, clock.now = clock.now, clock.now + 20_000_000
        target._note_silence(state, previous)
        assert target._ended is None
    # The callback finally runs; normal operation then continues beyond the grace period.
    state.received_ns = clock.now
    target._note_silence(state, clock.now - 20_000_000)
    clock.now += 500_000_000
    target._note_silence(state, clock.now - 20_000_000)
    assert target._ended is None


@pytest.mark.parametrize("next_gap", [250_000_000, 2_500_000_000])
def test_stall_grace_expires_without_states_even_after_another_stall(silence_monitor, next_gap):
    target, state, clock = silence_monitor
    previous = clock.now
    clock.now += 2_500_000_000
    target._note_silence(state, previous)
    assert target._ended is None
    previous, clock.now = clock.now, clock.now + next_gap
    target._note_silence(state, previous)
    assert target._ended == ("unknown", "unknown", False)
    # Late state delivery must never resurrect an ended session.
    state.received_ns = clock.now
    target._observe_locked(state)
    target._note_silence(state, clock.now - 20_000_000)
    assert target._ended == ("unknown", "unknown", False)
    with pytest.raises(SessionEnded):
        target._check_locked()


@pytest.mark.parametrize("change", [{"phase": "faulted"}, {"holder": 99}])
def test_stall_grace_never_hides_a_session_ending(silence_monitor, change):
    target, state, clock = silence_monitor
    previous = clock.now
    clock.now += 2_500_000_000
    target._note_silence(state, previous)
    assert target._ended is None
    state.__dict__.update(change)
    target._observe_locked(state)
    with pytest.raises(SessionEnded):
        target._check_locked()
    state.phase, state.holder, state.received_ns = "active", 7, clock.now
    target._observe_locked(state)
    with pytest.raises(SessionEnded):
        target._check_locked()


def test_ordinary_node_silence_gets_no_stall_grace(silence_monitor):
    target, state, clock = silence_monitor
    clock.now += 1_999_000_000
    target._note_silence(state, clock.now - 20_000_000)
    assert target._ended is None
    clock.now += 2_000_000
    target._note_silence(state, clock.now - 20_000_000)
    assert target._ended == ("unknown", "unknown", False)


def test_a_failing_streamer_ends_the_session(arm, fake, monkeypatch, caplog):
    t = arm.cartesian_targets()

    def broken(*args):
        raise RuntimeError("broken step")

    monkeypatch.setattr(t, "_step", broken)
    with pytest.raises(SessionEnded):
        t.wait(timeout=2.0)
    assert any(r.levelname == "ERROR" and "streamer failed" in r.message for r in caplog.records)


def test_an_old_session_does_not_stop_a_newer_one(arm, fake):
    old = arm.cartesian_targets()
    arm.stop()
    new = arm.cartesian_targets()
    stops = [verb for verb, _ in fake.requests].count("stop")
    old.stop()
    assert [verb for verb, _ in fake.requests].count("stop") == stops
    new.move_by([0.0, 0.0, 0.001])
    new.stop()
    assert [verb for verb, _ in fake.requests].count("stop") == stops + 1


def test_a_refusal_reseeds_a_joints_session(arm, fake):
    with arm.joint_targets(max_velocity=0.5, lead=0.15) as t:
        time.sleep(0.1)
        with fake.lock:
            fake.state["target"] = [0.3, 0, 0, 0, 0, 0, 0]  # the node's last accepted target
            fake.state["refused"] = 1
        began = time.monotonic()
        time.sleep(0.2)
        first = [float(m["data"][0]) for m in sent(fake, began)]
        # From 0.3 toward the goal 0, held within the lead of the measured 0.
        assert first and max(first) == pytest.approx(0.15)
        assert t.target()[0] == 0.0
