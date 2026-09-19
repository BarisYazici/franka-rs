"""Auto-recovery, the joint-velocity resume and the gripper path, asserted in process with
synthetic messages: a scripted sink for teleop.py's loop, and ZenohSink on a fake Zenoh
session and a fake clock, so nothing opens a socket or waits in real time.

    python -m tests.teleop.selftest_recovery   # one line per check, non-zero if any failed
"""
import contextlib
import io
import json
import types

from franka_vr.teleop import cli, limits, node_limits, teleop

from . import selftest as st
from .selftest import (AWAKE, FAULTED, HELD, MS, PANDA_NODE, _engaged, _reply, _Script,
                      _Sink, _tick, _vr, _whys, _zenoh_sink)

from .selftest import check                                   # one registry


def quiet(fn, *args):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        result = fn(*args)
    return result, out.getvalue()


# ---------------------------------------------------------------- recovery in teleop.py's loop


class _FaultSink(_Sink):
    """Faults at `fault_ms` of stream time and again `again_ms` after every recovery (None:
    never again). `recover()` answers `failure` and moves the arm, so a keepalive that held
    the old target would show."""

    def __init__(self, fault_ms, again_ms=None, failure=None):
        super().__init__()
        self.fault_at, self.again, self.failure = fault_ms * MS, again_ms, failure
        self.faulted, self.recovered_at = False, []

    def tick(self, t):
        super().tick(t)
        if self.fault_at is not None and t >= self.fault_at:
            self.faulted, self.fault_at = True, None

    def fault(self):
        return "arm Faulted (robot_mode Reflex)" if self.faulted else None

    def recoverable(self):
        return self.faulted

    def recover(self):
        self.recovered_at.append(len(self.targets))
        if self.failure:
            return self.failure
        self.faulted = False
        self.measured = ((0.32, 0.01, 0.48), self.measured[1])
        if self.again is not None:
            self.fault_at = self.now + self.again * MS
        return None


def _run(sink, rows, *flags):
    a = cli.parse(["--dry-run", "--quiet", *flags])
    app = teleop.Teleop(a, _Script(rows), sink, PANDA_NODE)
    abort, text = quiet(app.run, lambda: False)
    return app, abort, text


def _rows(*spans):
    """(flags, ticks) spans at 50 Hz, one after the other."""
    rows, k = [], 0
    for flags, n in spans:
        rows += [(j * 20 * MS, _vr(flags)) for j in range(k, k + n)]
        k += n
    return rows


@check("auto-recover: a mid-drive fault publishes nothing, recovers once, and needs a new squeeze")
def _():
    sink = _FaultSink(fault_ms=800)
    rows = _rows((AWAKE, 20), (HELD, 40), (AWAKE, 20), (HELD, 20))   # held through recovery
    app, abort, text = _run(sink, rows)
    assert abort is None and len(app.recoveries) == 1, (abort, app.recoveries)
    assert text.count("[recovered] 1/5 arm Faulted (robot_mode Reflex)") == 1, text
    at = sink.recovered_at[0]
    assert all(stamp < 800 * MS for *_, stamp in sink.targets[:at]), "published to a fault"
    after = sink.targets[at:]
    assert after and after[0][0] == "keepalive", _whys(sink, at)[:3]
    assert after[0][1] == sink.measured[0], "the keepalive held the old session's target"
    first_drive = _whys(sink, at).index("drive")
    assert "not_armed" in app.first and app.n_engage == 2, (app.refusals, app.n_engage)
    assert first_drive > 0 and all(w == "keepalive" for w in _whys(sink, at)[:first_drive])
    assert app.facts()["recoveries"][0]["failure"] is None


@check("auto-recover: the grip still held after recovery drives nothing until release+squeeze")
def _():
    sink = _FaultSink(fault_ms=800)
    app, abort, _ = _run(sink, _rows((AWAKE, 20), (HELD, 80)))
    at = sink.recovered_at[0]
    assert abort is None and "drive" not in _whys(sink, at), _whys(sink, at)
    assert app.refusals["not_armed"] > 0


@check("auto-recover: capped by --max-recoveries, then the fault ends the session")
def _():
    sink = _FaultSink(fault_ms=500, again_ms=200)
    app, abort, text = _run(sink, _rows((AWAKE, 20), (HELD, 80)), "--max-recoveries", "2")
    assert len(app.recoveries) == 2 and text.count("[recovered]") == 2, (app.recoveries, text)
    assert abort == "arm Faulted (robot_mode Reflex) (all 2 auto-recoveries used)", abort


@check("auto-recover: --no-auto-recover ends the session on the fault and never recovers")
def _():
    sink = _FaultSink(fault_ms=800)
    app, abort, _ = _run(sink, _rows((AWAKE, 20), (HELD, 80)), "--no-auto-recover")
    assert abort == "arm Faulted (robot_mode Reflex) (--no-auto-recover)", abort
    assert sink.recovered_at == [] and app.recoveries == []


@check("auto-recover: a failed recovery ends the session with its reason")
def _():
    sink = _FaultSink(fault_ms=800, failure="recover refused: command rejected")
    app, abort, _ = _run(sink, _rows((AWAKE, 20), (HELD, 80)))
    assert abort == ("arm Faulted (robot_mode Reflex); auto-recovery 1 failed: recover "
                     "refused: command rejected"), abort
    assert app.facts()["ended_because"] == abort


@check("auto-recover: a loss of Active that is not a fault still ends the session")
def _():
    sink = _FaultSink(fault_ms=800)
    sink.fault = lambda: "arm left Active (phase Idle)" if sink.faulted else None
    sink.recoverable = lambda: False
    _, abort, _ = _run(sink, _rows((AWAKE, 20), (HELD, 80)))
    assert abort == "arm left Active (phase Idle)" and sink.recovered_at == [], abort


# ---------------------------------------------------------------- the dq release resumes


@check("rule 6: the release resumes once every joint is under 0.5 and a new squeeze drives")
def _():
    app, sink, t = _engaged()
    sink.dq = (0.0, 0.0, 0.0, 0.0, 2.3, 0.0, 0.0)             # j5 over 0.85 x 2.61
    t, text = quiet(_tick, app, HELD, t)
    assert app.dq_tripped and "until every joint is under 0.5" in text, text
    n = len(sink.targets)
    sink.dq = (0.0, 0.0, 0.0, 0.0, 1.4, 0.0, 0.0)             # 0.54: not slow enough yet
    for flags in [AWAKE] * 15 + [HELD] * 5:
        t = _tick(app, flags, t)
    assert app.dq_tripped and "drive" not in _whys(sink, n)
    sink.dq = (0.0,) * 7
    sink.measured = ((0.31, 0.005, 0.5), sink.measured[1])    # the arm settled 11 mm away
    t, text = quiet(_tick, app, HELD, t)                      # slow, grip still held
    assert not app.dq_tripped and "[dq-resume]" in text, text
    for _ in range(20):                                       # past the 200 ms re-anchor hold
        t = _tick(app, HELD, t)
    assert "drive" not in _whys(sink, n) and app.n_engage == 1
    for flags in [AWAKE] * 15 + [HELD] * 3:                   # release, then a new squeeze
        t = _tick(app, flags, t)
    assert app.n_engage == 2 and _whys(sink)[-1] == "drive", (app.n_engage, _whys(sink, n))
    first = _whys(sink, n).index("drive") + n                 # the rising edge's latch: the arm
    assert st.close(sink.targets[first][1], sink.measured[0]), "re-engaged with a jump"


@check("rule 6: a recovery ends a dq release, except with --dq-latch")
def _():
    for flags, latched in (((), False), (("--dq-latch",), True)):
        app, sink, t = _engaged(*flags)
        sink.dq = (0.0, 0.0, 0.0, 0.0, 2.3, 0.0, 0.0)
        quiet(_tick, app, HELD, t)
        sink.recover = lambda: None
        quiet(app.recover, "arm Faulted", t * MS)
        assert app.dq_tripped == latched and not app.armed, (flags, app.dq_tripped)


# ---------------------------------------------------------------- ZenohSink on a fake session


def _recovers(node, after_s=4.0):
    """The node's recover: Idle after `after_s` with the old snapshot's errors, cleared at the
    next idle read half a second later; acquire and enable move the phase on."""
    node.script["recover"] = [(after_s, dict(phase=0, robot_mode=1)), (after_s + 0.5,
                                                                       dict(has_errors=0))]
    node.script["acquire"] = [(0.0, dict(phase=1))]
    node.script["enable"] = [(0.05, dict(phase=2))]


@check("zenoh: a Faulted arm is recovered before acquire, through a query that times out")
def _():
    sink, node = _zenoh_sink()
    _recovers(node)
    node.answer["recover"], node.blocks["recover"] = "Timeout", 3.0   # seen on hardware
    _, text = quiet(sink.bring_up)
    assert node.verbs() == ["recover", "acquire", "enable"], node.verbs()
    assert "recover: query error 'Timeout'; confirming from the state" in text, text
    assert "recovered before acquire" in text and sink.enabled, text
    assert node.t >= 4.5, node.t                              # waited for has_errors to clear
    assert node.calls[2][1] == {"client_id": 7200, "mode": "cartesian", "episode": "pick"}


@check("zenoh: a Reflex or errors-set arm in Idle is recovered too; a clean one is not")
def _():
    for info, want in ((dict(phase=0, robot_mode=4, has_errors=1), ["recover"]),
                       (dict(phase=0, robot_mode=1, has_errors=1), ["recover"]),
                       (dict(phase=0, robot_mode=1, has_errors=0), [])):
        sink, node = _zenoh_sink()
        sink.info = dict(info, holder=0)
        _recovers(node)
        quiet(sink.bring_up)
        assert node.verbs() == want + ["acquire", "enable"], (info, node.verbs())


@check("zenoh: --no-auto-recover refuses a Faulted arm before acquire and sends nothing")
def _():
    sink, node = _zenoh_sink("--no-auto-recover")
    try:
        quiet(sink.bring_up)
    except SystemExit as exc:
        assert "before acquire" in str(exc) and "Faulted" in str(exc), exc
    else:
        raise AssertionError("started on a Faulted arm")
    assert node.calls == []


@check("zenoh: a refused recover, or one that never clears in 20 s, is a failure, not a hang")
def _():
    sink, node = _zenoh_sink()
    node.answer["recover"] = {"ok": False, "error": "command rejected"}
    assert quiet(sink._recover)[0] == "recover refused: command rejected"
    sink, node = _zenoh_sink()
    node.answer["recover"], node.blocks["recover"] = "Timeout", 6.0
    failure, _ = quiet(sink._recover)
    assert failure.startswith("not Idle without errors 20 s after recover (phase Faulted"), \
        failure
    assert 20.0 <= node.t < 20.1, node.t
    sink, node = _zenoh_sink()
    sink.cancelled = lambda: node.t > 1.0                     # Ctrl-C during the wait
    assert quiet(sink._recover)[0] == "interrupted" and node.t < 1.1


@check("zenoh: another client's faulted arm is not recovered; Ctrl-C acquires nothing")
def _():
    sink, node = _zenoh_sink()
    sink.info["holder"] = 9000
    try:
        quiet(sink.bring_up)
    except SystemExit as exc:
        assert "held by client 9000" in str(exc), exc
    else:
        raise AssertionError("recovered another client's arm")
    assert node.calls == []
    sink.enabled = True
    assert not sink.recoverable() and sink.fault() == "lease taken by client 9000"
    for info in (dict(phase=0, robot_mode=1, has_errors=0, holder=0), FAULTED):
        sink, node = _zenoh_sink()
        sink.info = dict(info)
        _recovers(node)
        sink.cancelled = lambda: True
        try:
            quiet(sink.bring_up)
        except SystemExit as exc:
            assert "interrupted" in str(exc), exc
        assert "acquire" not in node.verbs() and not sink.enabled, (info, node.verbs())


@check("zenoh: mid-session recover() re-enables with the same episode, silently, once Idle")
def _():
    sink, node = _zenoh_sink()
    sink.enabled = True
    assert sink.recoverable() and sink.fault() == "arm Faulted (robot_mode Reflex)"
    _recovers(node)
    failure, text = quiet(sink.recover)
    assert failure is None and sink.enabled and not sink.recoverable(), failure
    assert node.verbs() == ["recover", "acquire", "enable"] and text == "", text
    assert node.calls[2][1]["episode"] == "pick"
    sink, node = _zenoh_sink()
    sink.enabled = True
    _recovers(node)
    node.answer["enable"] = {"ok": False, "error": "not allowed in faulted"}
    assert quiet(sink.recover)[0] == "R enable refused: not allowed in faulted"
    assert not sink.enabled


@check("zenoh: an error reply is a SystemExit with its text (was an AttributeError on .ok)")
def _():
    sink, node = _zenoh_sink()
    node.answer["acquire"] = "Timeout"
    try:
        sink._verb("acquire")
    except SystemExit as exc:
        assert str(exc) == "R acquire: query error 'Timeout'", exc
    else:
        raise AssertionError("an error reply passed")


@check("zenoh: shutdown on a Faulted arm skips stop and still releases, once")
def _():
    sink, node = _zenoh_sink()
    sink.enabled = True
    quiet(sink.stop)
    assert node.verbs() == ["release"] and node.closed and sink.session is None
    sink.stop()
    assert node.verbs() == ["release"]
    sink, node = _zenoh_sink()
    sink.enabled, sink.info["phase"] = True, 1                # enabled, Active not seen yet
    quiet(sink.stop)
    assert node.verbs() == ["stop", "release"]


# ---------------------------------------------------------------- the node's limits


@check("zenoh: a node that answers no params/schema stops the run BEFORE acquire")
def _():
    sink, node = _zenoh_sink()

    def silent(key, payload=None, timeout=None):
        node.calls.append((key.rsplit("/", 1)[-1], {}))
        return []
    node.get = silent
    try:
        sink.fetch_limits()
    except node_limits.LimitsError as exc:
        assert "params/schema" in str(exc) and "no reply" in str(exc), exc
    else:
        raise AssertionError("started without the node's limits")
    assert node.verbs() == ["schema"], node.verbs()   # and nothing was acquired or enabled
    code = []
    try:
        cli.load_limits(cli.parse(["--arm", "R", "--skip-hand-check"]), sink)
    except SystemExit as exc:
        code.append(exc.code)
    assert code == [2], code


@check("zenoh: fetch_limits reads params/schema and params/get, and validates against them")
def _():
    sink, node = _zenoh_sink()
    fixture = json.loads(open(st.FIXTURE).read())
    bodies = {"schema": fixture["schema"], "get": fixture["get"]}

    def answer(key, payload=None, timeout=None):
        name = key.rsplit("/", 1)[-1]
        node.calls.append((name, {}))
        return [_reply(bodies[name])]
    node.get = answer
    got = sink.fetch_limits()
    assert node.verbs() == ["schema", "get"], node.verbs()
    assert got.from_node and got.source == "the node"
    assert (got.max_lead, got.budget) == (st.NODE.max_lead, st.NODE.budget)
    # a node whose schema says 0.12 gives a client that allows a 0.11 clamp; the fixture's
    # 0.05 does not. Same code, same flags, different node.
    bodies["schema"] = json.loads(json.dumps(fixture["schema"]))
    bodies["schema"]["derived"]["max_lead"] = 0.12
    loose = sink.fetch_limits()
    a = cli.parse(["--arm", "R", "--skip-hand-check", "--clamp", "0.11"])
    limits.validate(a, loose, cli.fail)                          # accepted
    try:
        limits.validate(a, got, cli.fail)
    except SystemExit:
        pass
    else:
        raise AssertionError("--clamp 0.11 passed a node with max_lead 0.05")


@check("zenoh: params/current is stored by the Zenoh thread and read once by the loop")
def _():
    sink, _ = _zenoh_sink()
    body = {"owner": "node", "params": {"budget": [0.85, 5.0, 20.0],
                                        "rotation_budget": [1.2, 2.0, 20.0]}}
    sink._on_params(types.SimpleNamespace(payload=json.dumps(body).encode()))
    assert sink.take_params() == body and sink.take_params() is None
    sink._on_params(types.SimpleNamespace(payload=b"{not json"))
    assert sink.params_bad == 1 and sink.take_params() is None


def main():
    from . import selftest_gripper                       # noqa: F401  registers its checks
    return st.run(st.CHECKS, "selftest_recovery")


if __name__ == "__main__":
    raise SystemExit(main())
