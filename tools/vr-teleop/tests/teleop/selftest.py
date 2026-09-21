"""The teleop client's arithmetic, asserted in process.

    python -m tests.teleop.selftest     # from tools/vr-teleop; one line per check, exits
                                        # non-zero if any failed

This file is the harness: the check registry, the fake source and sink, and the node limits
the checks run against. The checks themselves are in `selftest_rules.py` (the seven rules,
the clutch algebra, the wire layouts and where the node's limits come from) and
`selftest_limits.py` (where the node's limits come from, and teleop/params) and
`selftest_hands.py` (the hand-to-port pairing and its pre-acquire check).
`selftest_recovery.py` and `selftest_gripper.py` are the other entry point.

No TCP socket: the hand check's listener is driven over inproc:// only, and the node's limits
come from `fixtures/node-limits.json`, never from a running node.
"""
import json
import os
import struct
import types

from franka_vr.teleop import cli, limits, node_limits, sinks, sources, teleop

from .facts import GRIPPER, TARGET  # noqa: F401  the README layouts, shared with the checks

CHECKS = []

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "fixtures")
FIXTURE = os.path.normpath(os.path.join(FIXTURES, "node-limits.json"))
RECORDING = os.path.normpath(os.path.join(FIXTURES, "desk-idle.jsonl.gz"))

# The node's layouts as the README documents them, written out here so that the checks read
# what the client puts on the wire with something other than the code that wrote it.
STATE = struct.Struct("<BBBBHIQQQQd7d7d7d16d6d7dQQQ")        # 474, StateMsg
GRIPPER_STATE = struct.Struct("<BBHIQddd")                   # 40, GripperStateMsg

# Exactly what `--node-limits fixtures/node-limits.json` gives a run: an FR3 node's derived
# block at the schema's defaults. Nothing in this tree types any of its numbers.
NODE = node_limits.from_file(FIXTURE)
# The same node with a Panda's joint velocity limits. The dq checks are written against
# those, and the difference between the two is itself a check
# (selftest_rules: a different dq_limit moves the release).
PANDA_DQ = (2.175,) * 4 + (2.61,) * 3
PANDA_NODE = NODE._replace(dq_limit=PANDA_DQ)

MS = 1_000_000
AWAKE = sources.FLAG_FRESH | sources.FLAG_CONTROLLER_ON
HELD = AWAKE | sources.FLAG_ENGAGED


def check(name):
    def wrap(fn):
        CHECKS.append((name, fn))
        return fn
    return wrap


def close(a, b, tol=1e-12):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _vr(flags, pos=(1.4, -0.7, 0.9)):
    return sources.VR_FMT.pack(sources.VR_MSG_MAGIC, 1, 1, *pos, 0.0, 0.0, 0.0, 1.0, 0.0,
                               flags, 0)


def _rejects(*flags, node=NODE):
    """Whether teleop refuses this command line: at parse (the shape checks) or at the
    node-limits check, which runs after the Zenoh session is open and before anything is
    acquired. Either way nothing has been touched."""
    import contextlib
    import io
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            limits.validate(cli.parse(["--dry-run", *flags]), node, cli.fail)
    except SystemExit:
        return True
    return False


class _Src:
    live = False
    done = True

    def now(self):
        return 0

    def poll(self, _ms):
        return []

    def close(self):
        pass


class _Script(_Src):
    live = True

    def __init__(self, rows):
        self.rows, self.i, self.t, self.done = rows, 0, 0, False

    def now(self):
        return self.t

    def poll(self, _ms):
        if self.i >= len(self.rows):
            self.done = True
            return []
        self.t, payload = self.rows[self.i]
        self.i += 1
        return [(self.t, payload)]


class _Sink:
    """A still arm whose state is stamped at the last tick's stream time, except inside
    `gap` = (start_ns, end_ns), where the stamp stays at start. It keeps every target as
    (why, pos, quat, t_send_ns), and its bytes in `raws`."""

    live = False
    max_width = 0.08
    gripper_ready = False

    def __init__(self):
        self.puts, self.targets, self.raws = 0, [], []
        self.dq, self.now, self.gap = None, 0, None
        self.measured = ((0.3, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
        self.params = None                # what take_params() hands the loop, once

    def start(self):
        pass

    def connect(self):
        pass

    def take_params(self):
        body, self.params = self.params, None
        return body

    def state(self):
        stamp = self.gap[0] if self.gap and self.gap[0] <= self.now < self.gap[1] else self.now
        return sinks.ArmState(stamp, self.measured, self.measured, self.dq)

    def clock_ns(self):
        return self.now

    def tick(self, t):
        self.now = t

    def put_target(self, raw, pos, quat, seq, why):
        self.puts += 1
        self.targets.append((why, pos, quat, TARGET.unpack(raw)[5]))
        self.raws.append(raw)

    def put_gripper(self, *_a):
        pass

    def fault(self):
        return None

    def recoverable(self):
        return False

    def gripper_info(self):
        return None

    def counters(self):
        return None

    def stop(self):
        pass


def _app(*extra, node=PANDA_NODE, params=None):
    a = cli.parse(["--dry-run", *extra])
    sink = _Sink()
    return teleop.Teleop(a, _Src(), sink, node, params), sink


def _tick(app, flags, t_ms, x=0.0):
    app.advance(t_ms * MS)
    app.on_wire(_vr(flags, (1.4 + x, -0.7, 0.9)), t_ms * MS)
    return t_ms + 20


def _engaged(*extra):
    """An app past arming and its re-anchor hold, engaged and driving for 5 samples; its
    source is live, so every target's t_send_ns is the stream time."""
    app, sink = _app("--quiet", *extra)
    app.source.live = True
    t = 0
    for _ in range(20):
        t = _tick(app, AWAKE, t)
    for _ in range(5):
        t = _tick(app, HELD, t)
    assert app.n_engage == 1 and app.n_out == 5, (app.n_engage, app.n_out, app.refusals)
    return app, sink, t


def _whys(sink, since=0):
    return [why for why, *_ in sink.targets[since:]]


def _payload(text):
    return types.SimpleNamespace(to_string=lambda: text)


def _reply(body=None, err=None):
    ok = None if body is None else types.SimpleNamespace(payload=_payload(json.dumps(body)))
    return types.SimpleNamespace(ok=ok, err=err and types.SimpleNamespace(payload=_payload(err)))


class _Node:
    """A node's cmd queryables on a fake clock. `script[verb]` is a list of (delay_s, info
    update) applied that long after the query; `answer[verb]` is the reply, or an error text
    delivered after `blocks[verb]` seconds as a query timeout would be."""

    def __init__(self, sink, info):
        self.sink, self.t, self.events, self.calls = sink, 0.0, [], []
        self.script, self.answer, self.blocks = {}, {}, {}
        sink.info = dict(info)
        sink.monotonic, sink.sleep = (lambda: self.t), self.advance
        sink.session = self

    def advance(self, dt):
        self.t += dt
        for event in [e for e in self.events if e[0] <= self.t]:
            self.events.remove(event)
            self.sink.info.update(event[1])

    def get(self, key, payload=None, timeout=None):
        verb = key.rsplit("/", 1)[-1]
        self.calls.append((verb, json.loads(payload) if payload else {}))
        for delay, update in self.script.get(verb, ()):
            self.events.append((self.t + delay, update))
        self.advance(self.blocks.get(verb, 0.0))
        answer = self.answer.get(verb, {"ok": True})
        return [_reply(err=answer)] if isinstance(answer, str) else [_reply(answer)]

    def close(self):
        self.closed = True

    def verbs(self):
        return [v for v, _ in self.calls]


FAULTED = dict(phase=4, robot_mode=4, has_errors=1, holder=7200)


def _zenoh_sink(*flags):
    a = cli.parse(["--arm", "R", "--episode", "pick", "--skip-hand-check", *flags])
    sink = sinks.ZenohSink(a)
    return sink, _Node(sink, FAULTED)


def run(checks, label):
    failed = 0
    for name, fn in checks:
        try:
            fn()
        except Exception as exc:            # a mutant can break a test's own asserts
            print(f"  FAIL  {name}\n          {type(exc).__name__}: {exc}")
            failed += 1
        else:
            print(f"  ok    {name}")
    print(f"  {label}: {len(checks) - failed}/{len(checks)} checks")
    return 1 if failed else 0


def main():
    from . import selftest                                   # the module, not this __main__
    from . import selftest_hands, selftest_limits, selftest_rules  # noqa: F401  register checks
    return run(selftest.CHECKS, "selftest")


if __name__ == "__main__":
    raise SystemExit(main())
