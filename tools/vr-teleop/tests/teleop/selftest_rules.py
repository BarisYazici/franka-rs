#!/usr/bin/env python3
"""The seven rules, the clutch algebra, the wire layouts, and where the node's limits come
from. Run through `selftest.py`, which owns the registry and the fakes."""
import contextlib
import io
import json
import math
import struct

from franka_node import _wire

from franka_vr.teleop import cli, geometry, limits, sinks, sources, teleop
from wire_tools import record_wire

from .selftest import (AWAKE, GRIPPER, GRIPPER_STATE, HELD, MS, NODE, PANDA_NODE, RECORDING,
                       STATE, TARGET, _app, _engaged, _Script, _Sink, _tick, _vr, _whys, check,
                       close)

# ---------------------------------------------------------------- rule 4


@check("rule 4: qcanon negates on a negative dot and leaves a positive one alone")
def _():
    q = (0.5, -0.5, -0.5, 0.5)
    assert geometry.qcanon(tuple(-c for c in q), q) == q
    assert geometry.qcanon(q, q) == q
    assert geometry.qcanon(q, None) == q


@check("rule 4: the measured +/-(0.5,-0.5,-0.5,0.5) pair is one rotation, not a 2 pi step")
def _():
    q = (0.5, -0.5, -0.5, 0.5)
    flipped = tuple(-c for c in q)
    # a consumer that differences the raw components sees a full turn's worth of change
    naive = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, flipped)))
    assert naive > 1.9, naive
    assert geometry.qangle(q, geometry.qcanon(flipped, q)) < 1e-6


# ---------------------------------------------------------------- rule 2

ARM_P, ARM_Q = (0.42, -0.08, 0.31), geometry.qunit((0.1, 0.97, -0.15, 0.12))
HAND_P, HAND_Q = (1.41, -0.73, 0.95), geometry.qunit((-0.2, 0.3, 0.5, 0.78))


@check("rule 2: compose() on the latch tick returns the arm's measured pose exactly")
def _():
    for scale in (0.2, 0.4, 1.0, 1.7):
        c = geometry.Clutch(scale, 1.0)
        c.latch(ARM_P, ARM_Q, HAND_P, HAND_Q)
        p, q = c.compose(HAND_P, HAND_Q)
        assert close(p, ARM_P), (scale, p, ARM_P)
        assert geometry.qangle(q, ARM_Q) < 1e-6, (scale, q, ARM_Q)


@check("rule 2: a re-latch from anywhere still returns the arm's pose (no re-engage jump)")
def _():
    c = geometry.Clutch(0.4, 1.0)
    c.latch(ARM_P, ARM_Q, HAND_P, HAND_Q)
    far_hand = (HAND_P[0] + 0.9, HAND_P[1] - 1.3, HAND_P[2] + 0.4)
    far_q = geometry.qunit((0.7, -0.1, 0.2, 0.6))
    moved_arm = (0.31, 0.22, 0.55)
    c.latch(moved_arm, ARM_Q, far_hand, far_q)
    p, q = c.compose(far_hand, far_q)
    assert close(p, moved_arm), (p, moved_arm)
    assert geometry.qangle(q, ARM_Q) < 1e-6


@check("rule 2: translation is scaled and the offsets are DECOUPLED (the direction bug)")
def _():
    c = geometry.Clutch(0.4, 1.0)
    c.latch(ARM_P, ARM_Q, HAND_P, HAND_Q)
    d = (0.10, -0.05, 0.02)
    p, q = c.compose(tuple(h + x for h, x in zip(HAND_P, d)), HAND_Q)
    assert close(p, tuple(a + 0.4 * x for a, x in zip(ARM_P, d))), p
    assert geometry.qangle(q, ARM_Q) < 1e-6             # moving the hand does not rotate
    turn = geometry.qmul(geometry.axis_angle((0.0, 0.0, 1.0), 0.7), HAND_Q)
    p2, q2 = c.compose(HAND_P, turn)
    assert close(p2, ARM_P), p2                       # rotating the hand does NOT move p_off
    assert abs(geometry.qangle(q2, ARM_Q) - 0.7) < 1e-9, geometry.qangle(q2, ARM_Q)


@check("qpow scales a rotation's angle and leaves its axis alone")
def _():
    axis = (0.3, -0.5, 0.8)
    for angle in (0.02, 0.5, 1.1, 2.9):
        for scale in (0.15, 0.25, 0.4, 1.0, 2.0):
            if angle * scale >= math.pi:
                continue          # past pi a rotation IS its shorter complement; no meaning
            q = geometry.qpow(geometry.axis_angle(axis, angle), scale)
            assert abs(geometry.qangle((0.0, 0.0, 0.0, 1.0), q) - angle * scale) < 1e-9
            back = geometry.qpow(q, 1.0 / scale)
            assert geometry.qangle(back, geometry.axis_angle(axis, angle)) < 1e-6
    assert geometry.qpow((0.0, 0.0, 0.0, 1.0), 0.25) == (0.0, 0.0, 0.0, 1.0)
    # the antipodal form is the same rotation and must scale the same short way
    q = geometry.axis_angle(axis, 0.7)
    assert geometry.qangle(geometry.qpow(tuple(-c for c in q), 0.25),
                           geometry.qpow(q, 0.25)) < 1e-9


@check("the rotation scale scales the delta and keeps the latch tick exact")
def _():
    for scale in (0.15, 0.25, 0.4, 1.0):
        c = geometry.Clutch(0.4, scale)
        c.latch(ARM_P, ARM_Q, HAND_P, HAND_Q)
        p, q = c.compose(HAND_P, HAND_Q)
        assert close(p, ARM_P), (scale, p)
        assert geometry.qangle(q, ARM_Q) < 1e-6, (scale, geometry.qangle(q, ARM_Q))
        for angle in (0.05, 0.4, 1.2):
            delta = geometry.axis_angle((0.0, 1.0, 0.0), angle)
            _, q = c.compose(HAND_P, geometry.qmul(delta, HAND_Q))
            assert abs(geometry.qangle(ARM_Q, q) - angle * scale) < 1e-9, (scale, angle)


@check("rule 2: the rotation delta is applied in the BASE frame")
def _():
    c = geometry.Clutch(1.0, 1.0)
    c.latch(ARM_P, ARM_Q, HAND_P, HAND_Q)
    delta = geometry.axis_angle((0.0, 1.0, 0.0), 0.3)
    _, q = c.compose(HAND_P, geometry.qmul(delta, HAND_Q))
    assert geometry.qangle(q, geometry.qmul(delta, ARM_Q)) < 1e-6


# ---------------------------------------------------------------- rule 1


@check("rule 1: the keepalive republishes the last target at --rate while a source is live")
def _():
    app, sink = _app()
    app.last_msg_ns = 1_000_000_000
    app.advance(app.last_msg_ns)
    app.keepalive(app.last_msg_ns)
    assert sink.puts == 1, sink.puts
    app.keepalive(app.last_msg_ns + 5_000_000)        # 5 ms: too soon
    assert sink.puts == 1
    app.keepalive(app.last_msg_ns + 20_000_000)       # 20 ms: due
    assert sink.puts == 2, sink.puts


@check("rule 1: and stops after --source-timeout-ms, leaving a dead bridge to the node")
def _():
    app, sink = _app("--source-timeout-ms", "200")
    app.last_msg_ns = 1_000_000_000
    app.advance(app.last_msg_ns)
    app.keepalive(app.last_msg_ns + 150_000_000)
    assert sink.puts == 1, sink.puts
    app.keepalive(app.last_msg_ns + 300_000_000)
    assert sink.puts == 1, "kept a session alive for a source that had gone away"


@check("rule 1: a not-fresh sample is never followed, whatever its engaged bit says")
def _():
    app, sink = _app()
    app.armed = True
    app.advance(0)
    payload = struct.Struct("<IIQ8dII").pack(
        sources.VR_MSG_MAGIC, 1, 99, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        sources.FLAG_ENGAGED, 0)                      # engaged set, fresh CLEAR
    app.on_wire(payload, 1_000_000)
    assert app.refusals["not_fresh"] == 1
    assert app.n_out - app.n_keepalive == 0, "followed a stale pose"


# ---------------------------------------------------------------- rule 5 and the layouts


@check("rule 5: TargetMsg is 80 bytes with the README's offsets, and seq starts at 1")
def _():
    assert TARGET.size == _wire.TARGET.itemsize == 80
    for prefix, offset in (("<B", 1), ("<BB", 2), ("<BBH", 4), ("<BBHI", 8),
                           ("<BBHIQ", 16), ("<BBHIQQ", 24)):
        assert struct.calcsize(prefix) == offset, prefix
    raw = _wire.encode_target("cartesian", 4242, 1, 7, tuple(range(7)), False)
    version, kind, flags, client, seq, stamp, *data = TARGET.unpack(raw)
    assert (version, kind, flags, client, seq, stamp) == (1, 1, 0, 4242, 1, 7)
    assert data == list(range(7))


@check("rule 5: the other three layouts are the sizes the node documents")
def _():
    assert STATE.size == _wire.STATE.itemsize == 474
    assert GRIPPER.size == _wire.GRIPPER.itemsize == 40
    assert GRIPPER_STATE.size == _wire.GRIPPER_STATE.itemsize == 40
    assert sources.VR_MSG_SIZE == 88


@check("the anchor bit is off by default and is the node's own TARGET_ANCHOR = 1")
def _():
    assert _wire.TARGET_ANCHOR == 1
    assert teleop.ANCHOR_ENABLED is False
    _, sink, _ = _engaged()
    assert sink.raws and all(TARGET.unpack(raw)[2] == 0 for raw in sink.raws)


# ---------------------------------------------------------------- the decoder


@check("the decoder agrees with record_wire.decode on a real recorded message")
def _():
    import gzip
    with gzip.open(RECORDING, "rt") as fh:
        for line in fh:
            rec = json.loads(line)
            if rec.get("type") or "raw" not in rec:
                continue
            raw = bytes.fromhex(rec["raw"])
            magic, version, seq, *rest = sources.VR_FMT.unpack(raw)
            reference = record_wire.decode(raw)
            assert magic == sources.VR_MSG_MAGIC == reference["magic"]
            assert version == reference["version"]
            assert list(rest[0:3]) == reference["pos"]
            assert list(rest[3:7]) == reference["quat"]
            assert rest[7] == reference["gripper"]
            assert bool(rest[8] & sources.FLAG_FRESH) == reference["fresh"]
            assert bool(rest[8] & sources.FLAG_ENGAGED) == reference["engaged"]
            assert sources.decode_vr(raw) == (tuple(rest[0:3]), tuple(rest[3:7]), rest[7],
                                              rest[8])
            return
    raise AssertionError(f"{RECORDING} holds no messages")


@check("quat_of() on the ready pose's tool-down O_T_EE is unit and 180 deg about x")
def _():
    q = geometry.quat_of([1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0.307, 0.0, 0.487, 1.0])
    assert abs(geometry.qlen(q) - 1.0) < 1e-12
    assert geometry.qangle(q, (0.0, 0.0, 0.0, 1.0)) > math.pi - 1e-9


@check("the rotation clamp lands exactly on its limit")
def _():
    a = (0.0, 0.0, 0.0, 1.0)
    b = geometry.axis_angle((0.0, 0.0, 1.0), 1.0)
    clamped = geometry.qslerp(a, b, 0.15 / 1.0)
    assert abs(geometry.qangle(a, clamped) - 0.15) < 1e-9, geometry.qangle(a, clamped)


# ---------------------------------------------------------------- rules 6 and 7


@check("rule 6: StateMsg's dq (offset 106) and O_T_EE decode into one stamped ArmState")
def _():
    import types
    assert struct.calcsize("<BBBBHIQQQQd7d") == 106             # msg.rs offset_of!(dq)
    dq = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7)
    raw = STATE.pack(1, 2, 2, 0, 0, 7200, 0, 0, 0, 0, 1.0, *(0.0,) * 7, *dq,
                     *(0.0,) * 7, *sinks.READY_O_T_EE, *(0.0,) * 6, 0.31, 0.0, 0.49,
                     1.0, 0.0, 0.0, 0.0, 3, 0, 0)
    arm, info = sinks.decode_state(raw, 123)
    assert (arm.t_ns, arm.dq, arm.measured[0], info["phase"], info["accepted"]) == \
        (123, dq, (0.307, 0.0, 0.487), 2, 3)
    assert arm.desired == ((0.31, 0.0, 0.49), (1.0, 0.0, 0.0, 0.0))
    sink = types.SimpleNamespace(latest=None, info={})
    sinks.ZenohSink._on_state(sink, types.SimpleNamespace(payload=raw))
    assert sinks.ZenohSink.state(sink).dq == dq and sink.info["holder"] == 7200
    assert abs(sinks.ZenohSink.clock_ns(sink) - sink.latest.t_ns) < 1e9    # monotonic, both


@check("rule 6: a trip publishes the arm's MEASURED pose once, keepalives hold it, --dq-latch")
def _():
    app, sink, t = _engaged("--dq-latch")
    for i in range(1, 11):                                    # the hand runs off, the arm stays
        t = _tick(app, HELD, t, x=0.01 * i)
    assert app.clamps["lead"] > 0 and geometry.dist(app.last_target[0], sink.measured[0]) > 0.02
    sink.dq = (0.1, 0.2, 0.3, 1.9, 0.1, 0.1, 0.1)             # j4: 1.9 > 0.85 x 2.175
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        t = _tick(app, HELD, t, x=0.11)
    why, pos, quat, _ = sink.targets[-1]
    assert why == "dq_hold" and close(pos, sink.measured[0]), (why, pos)
    assert geometry.qangle(quat, sink.measured[1]) < 1e-9
    assert app.dq_releases[0][1:] == (4, 1.9, 2.175), app.dq_releases
    assert "[dq-release] joint 4 |dq| 1.900 rad/s > 0.85 x 2.175" in out.getvalue()
    n = len(sink.targets)
    for k in range(3):
        app.keepalive((t + 20 * k) * MS)
    assert _whys(sink, n) == ["keepalive"] * 3
    assert all(close(p, pos) and q == quat for _, p, q, _ in sink.targets[n:])
    sink.dq = (0.0,) * 7                                      # joints calm again, and then
    for flags in [AWAKE] * 15 + [HELD] * 10:                  # a release and a new squeeze
        t = _tick(app, flags, t, x=0.11)
    assert "drive" not in _whys(sink, n) and app.n_engage == 1 and app.dq_tripped
    assert app.refusals["dq_release"] == 26 and len(app.dq_releases) == 1


@check("rule 6: the same dq under the node's OTHER arm does not trip (the limit is the node's)")
def _():
    app, sink, t = _engaged()                                 # a Panda-limited node
    fr3, fr3_sink, t2 = _engaged()
    fr3.node = NODE                                           # the fixture's FR3 limits
    for target, s in ((app, sink), (fr3, fr3_sink)):
        s.dq = (0.0, 0.0, 0.0, 0.0, 2.3, 0.0, 0.0)            # 88 % of a Panda's, 44 % of FR3's
    with contextlib.redirect_stdout(io.StringIO()):
        _tick(app, HELD, t)
        _tick(fr3, HELD, t2)
    assert app.dq_tripped and not fr3.dq_tripped, (app.dq_tripped, fr3.dq_tripped)
    assert app.dq_releases[0][3] == 2.61 and fr3.dq_releases == []


@check("rule 6: advance() alone trips it, and a hold past --max-step is shortened, not refused")
def _():
    app, sink, t = _engaged()
    before = app.last_target
    sink.measured = ((0.36, 0.0, 0.5),                        # 60 mm and 0.3 rad from it
                     geometry.qmul(geometry.axis_angle((0.0, 0.0, 1.0), 0.3), before[1]))
    sink.dq = (0.0, 0.0, 0.0, 0.0, 2.3, 0.0, 0.0)             # j5: 2.3 > 0.85 x 2.61
    with contextlib.redirect_stdout(io.StringIO()):
        app.advance(t * MS)                                   # no wire message at all
    why, pos, quat, _ = sink.targets[-1]
    assert why == "dq_hold" and app.dq_tripped
    assert abs(geometry.dist(pos, before[0]) - app.a.max_step) < 1e-9
    assert abs(geometry.qangle(quat, before[1]) - app.a.max_step_rot) < 1e-9
    assert abs(geometry.dist(pos, sink.measured[0]) - 0.02) < 1e-9     # moved toward it
    assert app.a.max_step <= app.node.max_step
    assert app.a.max_step_rot <= app.node.max_step_rotation
    assert geometry.dist(pos, sink.measured[0]) < app.node.max_lead
    assert geometry.qangle(quat, sink.measured[1]) < app.node.max_lead_rotation


@check("rule 6: on_wire() checks dq itself: a state newer than advance()'s drives nothing")
def _():
    app, sink, t = _engaged()
    app.advance(t * MS)
    n = len(sink.targets)
    sink.dq = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.3)             # arrives between the two
    with contextlib.redirect_stdout(io.StringIO()):
        app.on_wire(_vr(HELD), t * MS)
    assert app.dq_tripped and _whys(sink, n) == ["dq_hold"], _whys(sink, n)


@check("rule 7: no state for 50 ms stops the drive and the clamp; back, it needs a new squeeze")
def _():
    app, sink, t0 = _engaged()
    sink.gap = (t0 * MS, (t0 + 200) * MS)                     # the state stops at t0
    n, t = len(sink.targets), t0
    for _ in range(10):
        t = _tick(app, HELD, t)
    stamps = [s for why, _, _, s in sink.targets[n:] if why == "drive"]
    assert stamps and max(stamps) <= (t0 + limits.STATE_STALE_MS) * MS, stamps
    assert app.refusals["no_state"] >= 1 and not app.armed
    held = app.last_target
    sink.measured = ((0.36, 0.0, 0.5), sink.measured[1])      # a stale pose 60 mm away
    app.keepalive(t * MS)
    assert _whys(sink)[-1] == "keepalive" and sink.targets[-1][1] == held[0]
    sink.measured = ((0.3, 0.0, 0.5), sink.measured[1])
    assert t == t0 + 200
    n = len(sink.targets)
    for _ in range(5):                                        # state back, grip still held
        t = _tick(app, HELD, t)
    assert "drive" not in _whys(sink, n) and app.n_engage == 1
    for _ in range(15):                                       # released: armed, re-anchored
        t = _tick(app, AWAKE, t)
    t = _tick(app, HELD, t)                                   # and a new squeeze
    assert app.n_engage == 2 and _whys(sink)[-1] == "drive"


@check("rule 7: a stale state's dq over the limit trips nothing, and nothing is driven")
def _():
    app, sink, t0 = _engaged()
    sink.gap = (t0 * MS, (t0 + 200) * MS)                     # the state stops at t0
    t = t0
    for _ in range(4):                                        # t0 .. t0 + 60: stale by then
        t = _tick(app, HELD, t)
    assert app.refusals["no_state"] == 1
    n = len(sink.targets)
    sink.dq = (0.0, 0.0, 0.0, 0.0, 2.3, 0.0, 0.0)             # carried only by the stale state
    for _ in range(5):
        t = _tick(app, HELD, t)
    app.keepalive(t * MS)
    assert not app.dq_tripped and not app.dq_releases, app.dq_releases
    assert _whys(sink, n) == ["keepalive"], _whys(sink, n)


def _finite_targets(sink):
    return all(math.isfinite(c) for _, p, q, _ in sink.targets for c in (*p, *q))


@check("rule 7: a non-finite arm state is no state: the clutch releases, nothing is driven")
def _():
    app, sink, t = _engaged()
    good, n = sink.measured, len(sink.targets)
    sink.measured = ((math.nan, 0.0, 0.5), good[1])           # NaN passes every `>` leash
    with contextlib.redirect_stdout(io.StringIO()) as out:
        for k in range(1, 11):                               # 0.4 m of hand, 40 mm per sample
            t = _tick(app, HELD, t, x=0.04 * k)
        app.keepalive(t * MS)
    assert "drive" not in _whys(sink, n) and not app.armed, _whys(sink, n)
    assert app.refusals["no_state"] >= 1 and _finite_targets(sink)
    assert out.getvalue().count("not finite") == 1, out.getvalue()
    sink.measured = good                                      # back: a new squeeze drives
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(3):
            t = _tick(app, HELD, t)
        assert "drive" not in _whys(sink, n)
        for _ in range(15):
            t = _tick(app, AWAKE, t)
        t = _tick(app, HELD, t)
    assert app.n_engage == 2 and _whys(sink)[-1] == "drive"


@check("rule 7: a squeeze on a non-finite state latches nothing and publishes no NaN")
def _():
    app, sink = _app("--quiet")
    sink.measured = ((0.3, 0.0, 0.5), (math.nan,) * 4)        # decode_state's non-finite O_T_EE
    t = 0
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(20):
            t = _tick(app, AWAKE, t)
        for _ in range(5):
            t = _tick(app, HELD, t)
    assert app.n_engage == 0 and "drive" not in _whys(sink), _whys(sink)
    assert _finite_targets(sink)


@check("decode_state: any non-finite O_T_EE entry makes a state that is not finite()")
def _():
    def decode(m):
        return sinks.decode_state(STATE.pack(1, 2, 2, 0, 0, 7200, 0, 0, 0, 0, 1.0,
                                             *(0.0,) * 21, *m, *(0.0,) * 6, 0.31, 0.0, 0.49,
                                             1.0, 0.0, 0.0, 0.0, 3, 0, 0), 0)[0]
    assert decode(sinks.READY_O_T_EE).finite()
    for i in (*range(0, 3), *range(4, 7), *range(8, 11), *range(12, 15)):
        for bad in (math.nan, math.inf, -math.inf):
            m = list(sinks.READY_O_T_EE)
            m[i] = bad
            assert not decode(m).finite(), (i, bad, decode(m))


@check("rule 7: in run(), a state that stops mid-drive ends the session 500 ms later")
def _():
    rows = [(k * 20 * MS, _vr(AWAKE if k < 20 else HELD)) for k in range(150)]
    a = cli.parse(["--dry-run", "--quiet"])
    sink = _Sink()
    sink.gap = (1000 * MS, 10 ** 12)                          # never comes back
    source = _Script(rows)
    app = teleop.Teleop(a, source, sink, PANDA_NODE)
    abort = app.run(lambda: False)
    assert abort == "no arm state for over 500 ms", abort
    assert source.t == 1520 * MS, source.t / MS               # the first tick past 500
    stamps = [s for why, _, _, s in sink.targets if why == "drive"]
    assert stamps and min(stamps) < 1000 * MS and max(stamps) <= 1050 * MS, stamps
    facts = app.facts()
    assert facts["ended_because"] == abort
    assert facts["node_limits"]["source"] == PANDA_NODE.source
    assert facts["node_limits"]["max_lead"] == NODE.max_lead

