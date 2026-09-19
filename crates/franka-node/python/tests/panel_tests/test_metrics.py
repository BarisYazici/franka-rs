import math

import pytest

import threading
import time

from franka_node.panel.metrics import (STALE_S, MetricsEngine, Sample, SosFilter, WindowStats,
                                       butter_highpass_sos, compare, lag_samples)

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
