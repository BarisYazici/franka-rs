import math

import pytest

import threading
import time

from franka_node.panel.metrics import (RING_MAX_ROWS, RING_S, STALE_S, STATE_HZ, MetricsEngine,
                                       Sample, SosFilter, WindowStats, butter_highpass_sos,
                                       compare, lag_samples)

FS = 100.0


def steady_gain(f_hz, fc=3.0, fs=FS):
    flt = SosFilter(butter_highpass_sos(4, fc, fs))
    ys = [flt.step(math.sin(2 * math.pi * f_hz * i / fs)) for i in range(2000)][1000:]
    return math.sqrt(2 * sum(y * y for y in ys) / len(ys))


def test_highpass_is_butterworth_3hz():
    assert steady_gain(3.0) == pytest.approx(1 / math.sqrt(2), abs=0.01)  # -3 dB at the corner
    assert steady_gain(0.3) < 1e-3  # 4th order: 40 dB/decade below
    assert steady_gain(1.0) == pytest.approx(1 / math.sqrt(1 + 3 ** 8), rel=0.05)
    assert steady_gain(20.0) == pytest.approx(1.0, abs=0.01)


def test_lag_recovers_a_shift_and_refuses_a_flat_signal():
    t = [math.sin(2 * math.pi * 0.5 * i / FS) for i in range(300)]
    assert lag_samples(t, [0.0] * 12 + t[:-12], 50) == 12
    assert lag_samples(t, t, 50) == 0
    assert lag_samples(t, [0.3] * 300, 50) is None


def drive(engine, t0, seconds, jitter, lag_s=0.1, valid=True):
    n = int(seconds * FS)
    for i in range(n):
        t = t0 + i / FS
        target = [0.4 + 0.1 * math.sin(2 * math.pi * 0.5 * t), 0.0, 0.3]
        ee = [0.4 + 0.1 * math.sin(2 * math.pi * 0.5 * (t - lag_s)), 0.0, 0.3]
        dq = [0.0] * 7
        dq[3] = jitter * math.sqrt(2) * math.sin(2 * math.pi * 8 * t)
        engine.push(Sample(t, dq, target, ee, valid))
    return t0 + n / FS


def test_before_after_windows_and_verdict():
    e = MetricsEngine()
    t = drive(e, 0.0, 6.0, jitter=0.06)
    e.mark("node v1", 1)
    t = drive(e, t, 4.0, jitter=0.02)
    s = e.snapshot()
    assert s["before"]["j4_rms"] == pytest.approx(0.06, rel=0.05)
    assert s["after"]["j4_rms"] == pytest.approx(0.02, rel=0.05)
    assert s["compare"]["j4_rms"] == "better"
    assert s["before"]["lag_ms"] == pytest.approx(100, abs=10)
    assert s["after"]["n"] == pytest.approx(3.5 * FS, abs=2)  # 4 s minus the 0.5 s slew skip
    assert s["markers"][-1] == {"t": pytest.approx(t - 4.0 - 1 / FS), "label": "node v1", "version": 1}
    assert s["j4_peak_since_marker"] == pytest.approx(0.02, rel=0.1)
    # worse in the other direction
    e.mark("node v2", 2)
    drive(e, t, 4.0, jitter=0.09)
    assert e.snapshot()["compare"]["j4_rms"] == "worse"


def test_compare_needs_both_windows():
    assert compare(WindowStats(n=0), WindowStats(j4_rms=1, n=5))["j4_rms"] is None
    assert compare(WindowStats(j4_rms=1.0, n=5), WindowStats(j4_rms=0.95, n=5))["j4_rms"] == "same"
    assert compare(WindowStats(j4_rms=1.0, n=5), WindowStats(j4_rms=0.8, n=5))["j4_rms"] == "better"


def test_reflex_discards_windows():
    e = MetricsEngine()
    t = drive(e, 0.0, 6.0, jitter=0.06)
    e.mark("v1", 1)
    t = drive(e, t, 2.0, jitter=0.02)
    t = drive(e, t, 0.5, jitter=0.02, valid=False)
    s = e.snapshot()
    assert s["valid"] is False and s["before"]["n"] == 0 and s["after"]["n"] == 0
    drive(e, t, 2.0, jitter=0.02)
    s = e.snapshot()
    assert s["before"]["n"] == 0 and s["after"]["n"] == pytest.approx(2.0 * FS, abs=2)


def test_a_reflex_is_a_timestamp_not_a_walk_over_the_ring():
    """The cost of a reflex must not grow with the ring: one scalar excludes the earlier rows,
    and no row is ever rewritten after it was pushed (which is what lets `snapshot` drop the lock)."""
    e = MetricsEngine()
    t = drive(e, 0.0, 6.0, jitter=0.04)
    rows = list(e.rows)
    t = drive(e, t, 0.01, jitter=0.04, valid=False)  # one invalid sample
    assert e.valid_from == pytest.approx(t - 1 / FS)
    assert [r.valid for r in e.rows][:len(rows)] == [True] * len(rows)  # untouched
    assert e._window(list(e.rows), 0.0, t, e.valid_from).n == 0
    with pytest.raises(Exception):  # frozen: the invariant, not a convention
        rows[0].valid = False
    end = drive(e, t, 1.0, jitter=0.04)
    assert e._window(list(e.rows), 0.0, end, e.valid_from).n == pytest.approx(1 * FS, abs=2)


def test_a_node_reboot_resets_the_ring_and_the_filter_state():
    """The node's clock restarts while the page is open: rows from the old clock are neither
    droppable by age nor comparable, and the filter state belongs to the old stream."""
    e = MetricsEngine()
    drive(e, 1e6, 5.0, jitter=0.05)
    assert len(e.rows) == pytest.approx(5 * FS, abs=2)
    t = drive(e, 0.0, 2.0, jitter=0.05)  # reboot: the clock goes back
    assert e.resets == 1 and len(e.rows) == pytest.approx(2 * FS, abs=2)
    assert all(r.t < 1e5 for r in e.rows) and e.markers == []
    s = e.snapshot()
    assert s["j4_rms"] == pytest.approx(0.05, rel=0.1)  # filters restarted: no step through them
    drive(e, t + 10 * RING_S, 2.0, jitter=0.05)  # and a jump forward past the ring
    assert e.resets == 2 and len(e.rows) == pytest.approx(2 * FS, abs=2)


def test_the_ring_is_bounded_whatever_the_stream_does():
    """A stream faster than the engine's rate is the bridge's business to decimate; the ring is
    still a hard memory ceiling, so a misconfigured node cannot grow it without limit."""
    for hz in (FS, 10 * FS):
        e = MetricsEngine()
        for i in range(int(70 * hz)):  # more than the ring's span
            e.push(Sample(i / hz, [0.0] * 7, [0, 0, 0], [0, 0, 0]))
        assert len(e.rows) <= RING_MAX_ROWS == RING_S * STATE_HZ * 2


def test_snapshot_does_not_hold_the_lock_across_the_maths():
    """The 1 kHz-capable push path must never wait on a snapshot: the lock covers the copy of the
    ring, not the correlation. `_lag` stands in for the slow part."""
    e = MetricsEngine()
    drive(e, 0.0, 4.0, jitter=0.03)
    inside, release, pushed = threading.Event(), threading.Event(), threading.Event()

    def blocking_lag(rows, rate):
        inside.set()
        release.wait(10)
        return None
    e._lag = blocking_lag
    snap = threading.Thread(target=e.snapshot)
    snap.start()
    assert inside.wait(10)
    threading.Thread(target=lambda: (e.push(Sample(4.0, [0.0] * 7, [0, 0, 0], [0, 0, 0])),
                                     pushed.set())).start()
    assert pushed.wait(10), "push waited on the engine lock while snapshot was computing"
    release.set()
    snap.join(10)
    assert not snap.is_alive()


def test_marker_uses_the_state_clock_not_the_bridge_clock():
    """Samples carry the node's clock; a marker stamped with time.time() would put both windows
    in the wrong place across two hosts. 30 s offset from wall time here."""
    e = MetricsEngine()
    assert e.mark("early") is None  # no state yet: refused, not guessed
    t0 = time.time() - 30.0
    t = drive(e, t0, 6.0, jitter=0.05)
    m = e.mark("node v1", 1)
    assert m.t == pytest.approx(t - 1 / FS) and abs(m.t - time.time()) > 20
    drive(e, t, 2.0, jitter=0.05)
    s = e.snapshot()
    assert s["before"]["n"] == pytest.approx(5 * FS, abs=2) and s["after"]["n"] == pytest.approx(1.5 * FS, abs=2)


def test_headroom_needs_both_owner_limits():
    e = MetricsEngine()
    e.push(Sample(0.0, [0.1, 0.9, 0.2, 0, 0, 0, 0], [0, 0, 0], [0, 0, 0]))
    assert e.snapshot()["dq_headroom"] is None
    e.set_limits(dq_limit=[1.0] * 7)
    assert e.snapshot()["dq_headroom"] is None  # still no release fraction
    e.set_limits(release_fraction=0.6)
    h = e.snapshot()["dq_headroom"]
    assert h["joint"] == 1 and h["fraction"] == pytest.approx(0.9) and h["release"] == 0.6
    e.set_limits(dq_limit=[2.0] * 7, release_fraction=0.85)
    h = e.snapshot()["dq_headroom"]
    assert h["fraction"] == pytest.approx(0.45) and h["release"] == 0.85


def test_snapshot_goes_stale_when_state_stops():
    e = MetricsEngine()
    drive(e, 0.0, 1.0, jitter=0.02)
    assert e.snapshot()["ok"] is True
    e.last_arrival -= STALE_S + 0.1  # as if no sample arrived for that long
    s = e.snapshot()
    assert s["ok"] is False and s["reason"] == "stale_state" and s["age_s"] > STALE_S


def test_engine_is_safe_across_threads():
    """push (subscriber thread) against snapshot/mark (ticker and HTTP threads) for two seconds."""
    e = MetricsEngine()
    e.set_limits([1.0] * 7, 0.85)
    errors = []
    stop = threading.Event()

    def writer():
        t = 0.0
        while not stop.is_set():
            t = drive(e, t, 0.05, jitter=0.03)

    def reader():
        while not stop.is_set():
            try:
                e.snapshot()
                e.mark("m")
            except Exception as ex:  # noqa: BLE001 - the point is to catch anything
                errors.append(repr(ex))
    threads = [threading.Thread(target=writer), threading.Thread(target=reader), threading.Thread(target=reader)]
    for th in threads:
        th.start()
    time.sleep(2.0)
    stop.set()
    for th in threads:
        th.join()
    assert errors == []
