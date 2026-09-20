"""Decimation: the bridge keeps 1 in N of the node's state so the metrics are computed at the
rate the filters, the lag scaling and the thresholds assume, whatever the node publishes at.

No Zenoh here: `ArmMonitor.on_state` takes the same bytes the subscriber hands it, and the sample
times are the stream's own, so nothing in these tests depends on the wall clock.
"""

import math

import pytest

from franka_node.panel.bridge import KEEP_BAND, ArmMonitor
from franka_node.panel.metrics import RING_MAX_ROWS, RING_S, STATE_HZ

from .mock_state import encode_state

TONE_HZ = 7.5  # the jitter the panel is there to show
FAST_HZ = 300.0  # a tone only a 1 kHz stream carries; the 100 Hz view cannot resolve it
LAG_S = 0.1
J4_RMS = 0.05


def feed(mon: ArmMonitor, hz: float, seconds: float, t0: float = 0.0) -> None:
    for i in range(int(seconds * hz)):
        t = t0 + i / hz
        target = 0.4 + 0.1 * math.sin(2 * math.pi * 0.5 * t)
        ee = 0.4 + 0.1 * math.sin(2 * math.pi * 0.5 * (t - LAG_S))
        dq = [0.0] * 7
        dq[3] = (J4_RMS * math.sqrt(2) * math.sin(2 * math.pi * TONE_HZ * t)
                 + 0.2 * math.sin(2 * math.pi * FAST_HZ * t))
        mon.on_state(encode_state(robot_mode=2, t_node_ns=int(t * 1e9), dq=dq,
                                  o_t_ee=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, ee, 0.0, 0.3, 1],
                                  target=[target, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]))


def fed(hz: float, seconds: float) -> ArmMonitor:
    mon = ArmMonitor("X", lambda owner, verb: {})
    feed(mon, hz, seconds)
    return mon


def test_a_faster_stream_gives_the_same_numbers():
    """The acceptance: the same arm published at 100 Hz and at 1 kHz must read the same. Without
    decimation the 1 kHz stream runs a filter designed for 100 Hz, so its '3 Hz' high-pass sits at
    30 Hz: the 7.5 Hz jitter is thrown away, the 300 Hz tone is kept, and `lag_ms` is ten times out."""
    slow, fast = fed(STATE_HZ, 7.0), fed(10 * STATE_HZ, 8.0)  # the fast one spends 1 s measuring
    assert (fast.rate.every, slow.rate.every) == (10, 1)
    a, b = slow.metrics.snapshot(), fast.metrics.snapshot()
    assert a["j4_rms"] == pytest.approx(J4_RMS, rel=0.05)
    assert b["j4_rms"] == pytest.approx(a["j4_rms"], rel=0.02)
    # the kept phase differs by up to N-1 source samples, so the lag agrees to one kept sample
    assert a["lag_ms"] == pytest.approx(LAG_S * 1e3, abs=20)
    assert b["lag_ms"] == pytest.approx(a["lag_ms"], abs=1e3 / STATE_HZ)


def test_the_work_per_snapshot_does_not_follow_the_publish_rate():
    """What pegged the Pi was the ticker's pass over the ring, and the ring was `60 s x stream`."""
    slow, fast = fed(STATE_HZ, 7.0), fed(10 * STATE_HZ, 8.0)
    for mon in (slow, fast):
        mon.metrics.scanned = 0
        mon.metrics.snapshot()
    assert len(fast.metrics.rows) == pytest.approx(len(slow.metrics.rows), abs=3)
    assert fast.metrics.scanned == pytest.approx(slow.metrics.scanned, rel=0.02)
    assert fast.metrics.scanned <= 2 * RING_MAX_ROWS


def test_the_page_is_told_what_it_is_looking_at():
    mon = fed(10 * STATE_HZ, 3.0)
    ev = mon.metrics_event()
    assert ev["decimation"] == 10 and ev["source_hz"] == pytest.approx(1000, rel=0.01)
    assert ev["effective_hz"] == pytest.approx(STATE_HZ, rel=0.01)
    assert ev["snapshot_age_s"] == 0.0
    assert mon.metrics_event()["snapshot_age_s"] >= 0.0  # the second read is the ticker's, aged


def test_a_rate_change_starts_the_metrics_over():
    """The rows and the filter state were built at the old rate; keeping them would mix two bands."""
    mon = fed(STATE_HZ, 3.0)
    assert mon.rate.every == 1 and mon.metrics.resets == 0
    feed(mon, 10 * STATE_HZ, 3.0, t0=3.0)  # the node was restarted at state_hz = 1000
    assert mon.rate.every == 10 and mon.metrics.resets == 1
    assert mon.metrics.snapshot()["j4_rms"] == pytest.approx(J4_RMS, rel=0.05)


@pytest.mark.parametrize("hz", [1, 99, 100, 101, 150, 250, 350, 999, 1000])
def test_every_rate_the_node_takes_settles_with_a_full_ring(hz):
    """`state_hz` is a node setting anywhere in 1..1000. Rounding the ratio alone flaps at every
    rate near an odd half-multiple of 100, and each flap costs the ring: at 350 Hz that left 0.8 s
    of history instead of 60 s. The deadband must leave one change, at startup, and never more."""
    mon = fed(float(hz), RING_S + 3.0)
    rows, effective = mon.metrics.rows, hz / mon.rate.every
    assert mon.metrics.resets <= 1
    assert KEEP_BAND[0] <= effective < KEEP_BAND[1] or hz < KEEP_BAND[0]
    assert rows[-1].t - rows[0].t >= RING_S - 2 / effective  # a full ring, not a restarted one
    assert len(rows) <= RING_MAX_ROWS


def test_a_rate_the_decimation_cannot_halve_still_times_correctly():
    """250 Hz is a legal `state_hz` and 1 in 2 of it is 125 Hz, not 100. Everything counted in
    samples converts through the measured rate, so `lag_ms` is the real lag; before, dividing by a
    nominal 100 Hz stretched every 8 ms sample into 10 and over-stated the lag by a quarter."""
    mon = fed(2.5 * STATE_HZ, 8.0)
    effective = mon.rate.source_hz / mon.rate.every
    assert mon.rate.every == 2 and effective == pytest.approx(125, rel=0.01)
    assert mon.metrics.rate_hz == pytest.approx(effective)
    lag = mon.metrics.snapshot()["lag_ms"]
    assert lag == pytest.approx(LAG_S * 1e3, abs=1e3 / effective)  # within one kept sample
    assert lag * STATE_HZ / effective != pytest.approx(LAG_S * 1e3, abs=1e3 / effective)  # the old scaling


def test_a_reset_invalidates_the_cached_snapshot():
    """`/api/<arm>/metrics` serves the ticker's last snapshot for up to a tick. After a reset that
    snapshot describes a stream that no longer exists, and it would be served under the new rate's
    labels: numbers from 1 kHz-assumed rows, stamped `decimation: 10`."""
    mon = fed(STATE_HZ, 3.0)
    cached = mon.metrics_event()
    assert cached["ok"] and cached["decimation"] == 1
    feed(mon, 10 * STATE_HZ, 1.02, t0=3.0)  # a node restarted at state_hz = 1000: measure, reset
    assert mon.metrics.resets == 1
    ev = mon.metrics_event()
    assert ev["decimation"] == 10 and ev["snapshot_age_s"] == 0.0
    assert ev["t"] > cached["t"] and ev["t"] >= 4.0  # post-reset rows, not the cached ones


def test_a_reset_during_the_snapshot_is_not_cached_as_fresh():
    """The same trap one tick narrower: read the reset count after the maths and a reset that
    landed while it ran looks like the count the snapshot was taken at."""
    mon = fed(STATE_HZ, 3.0)
    real = mon.metrics.snapshot

    def snapshot_then_reset():
        snap = real()  # computed on the pre-reset ring...
        mon.metrics.reset()  # ...and the node's clock restarts while it runs
        return snap
    mon.metrics.snapshot = snapshot_then_reset
    stale = mon.metrics_event()
    mon.metrics.snapshot = real
    feed(mon, STATE_HZ, 0.5, t0=3.0)
    ev = mon.metrics_event()  # within the tick: only the reset count can tell it to recompute
    assert ev["ok"] and ev["t"] > stale["t"] and ev["snapshot_age_s"] == 0.0


def test_nothing_is_reported_until_the_rate_is_known():
    """On a fast node the first second arrives un-decimated; a metric computed from it describes
    a stream the panel is not going to keep. The page says it is measuring instead."""
    mon = ArmMonitor("X", lambda owner, verb: {})
    feed(mon, 10 * STATE_HZ, 0.5)
    ev = mon.metrics_event()
    assert (ev["ok"], ev["reason"], ev["source_hz"]) == (False, "measuring_rate", None)
    assert "j4_rms" not in ev and ev["reach"] is not None  # liveness still reported
    feed(mon, 10 * STATE_HZ, 2.0, t0=0.5)
    assert mon.metrics_event()["ok"] is True


def test_the_hundred_hertz_path_decodes_every_message():
    """The default node rate must behave exactly as before: nothing skipped, no reset, no log."""
    mon = fed(STATE_HZ, 3.0)
    assert len(mon.metrics.rows) == 300 and mon.metrics.resets == 0
    assert mon.rate.every == 1 and mon.seen == 300  # every message counted, none skipped
