#!/usr/bin/env python3
"""The gripper path: the node's (client, seq) rule across a restart, the seed that never
sends, the deadband and the rate limit, and GripperStateMsg parsing. Run through
`selftest_recovery.py`."""
import types

from franka_vr.teleop import cli, sinks

from .selftest import GRIPPER, GRIPPER_STATE, MS, _app, _zenoh_sink, check


def _gripper_state(max_width=0.08, flags=1, size=40):
    raw = GRIPPER_STATE.pack(1, flags, 0, 7200, 5, 0.04, 0.04, max_width)
    return types.SimpleNamespace(payload=raw[:size])


class _NodeGripper:
    """franka-node arm/gripper.rs `GripperSide::target`: the holder, then (client, seq)
    strictly increasing, remembered for the node's lifetime, then the width."""

    def __init__(self, holder, max_width=0.08):
        self.holder, self.max_width, self.last = holder, max_width, None
        self.accepted, self.refused = [], []

    def put(self, raw):
        _v, _k, _p, client, seq, _t, width, _f = GRIPPER.unpack(raw)
        if client != self.holder:
            return self.refused.append("not the holder")
        if self.last and self.last[0] == client and seq <= self.last[1]:
            return self.refused.append(f"seq {seq} not after {self.last[1]}")
        if self.max_width > 0.0 and width > self.max_width:
            return self.refused.append("beyond max width")
        self.last = (client, seq)
        self.accepted.append(width)


def _gripper_app(node, *flags):
    app, sink = _app("--quiet", "--arm", "R", "--client-id", "7300", *flags)
    sink.gripper_ready = True
    sink.put_gripper = lambda raw, width, seq: node.put(raw)
    return app, sink


def _sweep(app, t_ms=0, fractions=(0.0, 0.5, 1.0, 0.2, 0.9, 0.0)):
    for f in fractions:
        for _ in range(12):                                   # 240 ms each, past 5 Hz
            app.gripper(f, t_ms * MS)
            t_ms += 20
    return t_ms


@check("gripper: a second session with the same --client-id is accepted (was refused by seq)")
def _():
    node = _NodeGripper(holder=7300)
    first, _ = _gripper_app(node)
    _sweep(first)
    assert node.accepted and not node.refused, node.refused
    second, _ = _gripper_app(node)                            # the same node, a restarted teleop
    _sweep(second)
    assert len(node.accepted) == 2 * first.n_gripper and not node.refused, node.refused
    old, _ = _gripper_app(node)                               # CONTROL: the old count from 1
    old.gseq = 0
    _sweep(old)
    assert old.n_gripper > 0 and len(node.refused) == old.n_gripper, node.refused
    assert node.refused[0].startswith("seq 1 not after"), node.refused


@check("gripper: no gripper state ever received -> not one command, whatever the trigger")
def _():
    node = _NodeGripper(holder=7300)
    app, sink = _gripper_app(node)
    sink.gripper_ready = False
    _sweep(app)
    assert app.n_gripper == 0 and node.accepted == []


@check("gripper: GripperStateMsg parsing: max_width, flags, a malformed one and uncalibrated")
def _():
    sink, _ = _zenoh_sink()
    assert not sink.gripper_ready and sink.gripper_info()["flags"] is None
    sink._on_gripper_state(_gripper_state(size=32))           # was a struct.error in the callback
    assert not sink.gripper_ready and sink.gripper_info()["bad_states"] == 1
    sink._on_gripper_state(_gripper_state(max_width=0.0, flags=0))
    assert sink.gripper_ready and sink.max_width == 0.08      # 0 = unknown: the default stays
    assert sink.gripper_info()["flags"] == []                 # and the summary says uncalibrated
    sink._on_gripper_state(_gripper_state(max_width=0.0789, flags=0b1001))
    info = sink.gripper_info()
    assert (info["states"], info["max_width"], info["flags"]) == (2, 0.0789, ["calibrated",
                                                                               "fault"])


@check("gripper: a positive max_width is applied before gripper_ready is set")
def _():
    seen = []

    class Watch(sinks.ZenohSink):
        def __setattr__(self, name, value):
            if name == "gripper_ready" and value:
                seen.append(self.max_width)
            super().__setattr__(name, value)

    a = cli.parse(["--arm", "R", "--skip-hand-check"])
    sink = Watch(a)
    sink._on_gripper_state(_gripper_state(max_width=0.0791))
    assert seen == [0.0791], seen


@check("gripper: seed never sends; the deadband, the 5 Hz rate and a held trigger at start")
def _():
    node = _NodeGripper(holder=7300)
    app, _ = _gripper_app(node)
    app.gripper(1.0, 0)                                       # held at start: seed closed, silent
    assert app.n_gripper == 0 and abs(app.last_width) < 1e-12
    app.gripper(0.0, 100 * MS)                                # released inside the 200 ms window
    assert app.n_gripper == 0
    app.gripper(0.0, 200 * MS)                                # due: it opens
    assert node.accepted == [0.08]
    app.gripper(0.04, 500 * MS)                               # 3.2 mm, under the 4 mm deadband
    assert app.n_gripper == 1
    app.gripper(0.06, 520 * MS)                               # 4.8 mm: sent
    assert app.n_gripper == 2 and abs(node.accepted[-1] - 0.0752) < 1e-12
    app.gripper(1.0, 560 * MS)                                # a 100 ms click right after a send
    app.gripper(0.06, 660 * MS)
    app.gripper(0.06, 740 * MS)                               # is lost: by design, 5 Hz latest
    assert app.n_gripper == 2
