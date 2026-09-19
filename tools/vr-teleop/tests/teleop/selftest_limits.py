#!/usr/bin/env python3
"""Where the node's limits come from, and teleop's own parameter surface.

Every check here asserts BOTH directions, because the whole point of the parcel is that a
limit is a value that arrived from the node and not a constant in this tree: what the node's
number does, and what a DIFFERENT number does to the same code and the same flags.
Run through `selftest.py`."""
import contextlib
import io
import json
import math
import os
import re

from franka_vr.teleop import cli, limits, node_limits, teleop_params
from franka_vr.teleop.cli import TUNABLES

from .selftest import (AWAKE, FIXTURE, HELD, NODE, PANDA_DQ, PANDA_NODE, _app, _rejects,
                      _tick, check)



def _schema(**derived):
    """The fixture's params/schema with `derived` overridden."""
    body = json.loads(open(FIXTURE).read())
    body["schema"]["derived"].update(derived)
    return body


def _node(**derived):
    body = _schema(**derived)
    return node_limits.from_node(body["schema"], body["get"], source="a synthetic node")


@check("no limit of the node's is a constant in this tree any more")
def _():
    mirrors = [n for n in dir(limits) if n.startswith("NODE_")] + \
              [n for n in dir(limits) if "PANDA" in n]
    assert mirrors == [], f"limits.py still mirrors {mirrors}"
    here = os.path.dirname(os.path.abspath(limits.__file__))
    pattern = re.compile(r"\bNODE_[A-Z]|\bPANDA_DQ_MAX\b")
    guilty = [name for name in sorted(os.listdir(here))
              if name.endswith(".py") and pattern.search(open(os.path.join(here, name)).read())]
    assert guilty == [], f"a node limit is mirrored in {guilty}"


@check("every flag a NODE limit bounds is refused before anything is acquired")
def _():
    # A refused target at the node is not followed (the arm holds its last accepted one and
    # measures the next step from it), so each of these has to fail before an arm is touched.
    lead, lead_rot = limits.lead_bounds(cli.parse(["--dry-run"]), NODE)
    cases = {
        "--clamp": (f"{NODE.max_lead}", "0.2", "0", "-0.01"),
        "--clamp-rot": (f"{NODE.max_lead_rotation}", "0.3", "0"),
        "--max-step": (f"{NODE.max_step * 1.02}", "0.5", "0"),
        "--max-step-rot": (f"{NODE.max_step_rotation * 1.02}", "1.0", "0"),
        "--rate": (f"{NODE.rate_hz * limits.KEEPALIVE_SLACK + 1}", "1000", "0"),
        "--source-timeout-ms": (f"{NODE.stop_after_ms + 1}", "9000", "0"),
        "--arm": ("cam", "node", "two words", ""),               # the node's arm-name shape
        "--episode": ("bad/name", "x" * 129, "a b"),             # [A-Za-z0-9_-]{1,128}
        "--client-id": ("0",),                                   # a non-zero u32
        "--gripper-max-width": ("0", "-0.01", "0.5"),
        "--spatial-scale": ("0", "2.1", "-1"),
        "--rotation-scale": ("0", "2.1", "-1"),
        "--max-hand-step": ("0", "-0.1"),
        "--max-recoveries": ("-1",),
        "--workspace": ("0.5,0,0,0.5,1,1", "1,2,3"),
    }
    for flag, values in cases.items():
        for value in values:
            assert _rejects(flag, value), f"{flag} {value!r} was accepted"
    assert lead > 0.0 and lead_rot > 0.0


@check("and the value just inside each node limit is accepted")
def _():
    a = _app()[0].a
    lead, lead_rot = limits.lead_bounds(a, NODE)
    for flag, value in (("--clamp", repr(lead)), ("--clamp-rot", repr(lead_rot)),
                        ("--max-step", repr(NODE.max_step)),
                        ("--max-step-rot", repr(NODE.max_step_rotation)),
                        ("--rate", repr(NODE.rate_hz * limits.KEEPALIVE_SLACK)),
                        ("--source-timeout-ms", repr(NODE.stop_after_ms)),
                        ("--arm", "L"), ("--episode", "pick-0042"),
                        ("--client-id", "4294967295")):
        assert not _rejects(flag, value), f"{flag} {value!r} was refused"


@check("the lead bound IS the node's number: 0.12 accepts a 0.11 clamp, 0.05 refuses it")
def _():
    loose = _node(max_lead=0.12, max_lead_rotation=0.60)
    assert not _rejects("--clamp", "0.11", node=loose), "0.11 refused under max_lead 0.12"
    assert _rejects("--clamp", "0.12", node=loose), "0.12 accepted under max_lead 0.12"
    assert not _rejects("--clamp-rot", "0.59", node=loose)
    assert _rejects("--clamp-rot", "0.60", node=loose)
    # the same command line, the fixture's node: the SAME clamp is now refused
    assert _rejects("--clamp", "0.11", node=NODE), "0.11 accepted under max_lead 0.05"
    assert NODE.max_lead == 0.05 and loose.max_lead == 0.12


@check("a missing derived key refuses to start and names the key; nothing defaults")
def _():
    body = _schema()
    for key in ("max_lead", "rate_hz", "dq_limit", "leash"):
        broken = json.loads(json.dumps(body))
        del broken["schema"]["derived"][key]
        try:
            node_limits.from_node(broken["schema"], broken["get"])
        except node_limits.LimitsError as exc:
            assert key in str(exc), (key, exc)
        else:
            raise AssertionError(f"a schema with no {key} was accepted")
    for owner in (None, "teleop"):
        broken = json.loads(json.dumps(body))
        broken["schema"]["owner"] = owner
        try:
            node_limits.from_node(broken["schema"], broken["get"])
        except node_limits.LimitsError as exc:
            assert "owner" in str(exc), exc
        else:
            raise AssertionError("a schema owned by someone else was accepted")
    broken = json.loads(json.dumps(body))
    broken["schema"]["schema_version"] = 2
    try:
        node_limits.from_node(broken["schema"], broken["get"])
    except node_limits.LimitsError as exc:
        assert "schema_version" in str(exc), exc
    else:
        raise AssertionError("an unknown schema_version was accepted")


@check("the staleness allowance is budget/state_hz, and the fixture's budget is the node's")
def _():
    a = _app()[0].a
    stale, stale_rot = limits.staleness(a, NODE)
    assert abs(stale - NODE.budget / NODE.state_hz) < 1e-15
    assert abs(stale - 0.003) < 1e-9, stale               # 0.3 m/s for one 100 Hz tick
    assert abs(stale_rot - 0.005) < 1e-9, stale_rot
    fast = _node()._replace(budget=0.85, budget_rotation=1.2)
    assert abs(limits.staleness(a, fast)[0] - 0.0085) < 1e-9
    assert NODE.max_lead / (a.clamp + stale) > 1.7
    assert NODE.max_lead_rotation / (a.clamp_rot + stale_rot) > 1.6
    assert "not followed" in limits.headroom(a, NODE)
    assert "0.85" not in limits.headroom(a, NODE)          # it quotes THIS node, not another


@check("a params/current with a bigger budget lowers --clamp onto the new bound, loudly")
def _():
    app, sink = _app("--quiet", "--clamp", "0.046")        # legal at budget 0.3 (bound 0.047)
    assert app.node.budget == 0.3
    body = {"owner": "node", "boot_id": app.node.boot_id, "version": 4,
            "params": {"budget": [0.85, 5.0, 20.0], "rotation_budget": [1.2, 2.0, 20.0]}}
    sink.params = body
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        app.advance(0)
    assert app.node.budget == 0.85 and app.node.version == 4
    assert abs(app.a.clamp - (0.05 - 0.0085)) < 1e-12, app.a.clamp
    assert "--clamp lowered to 0.0415" in out.getvalue(), out.getvalue()
    # and a budget that changes nothing prints nothing and leaves the clamp alone
    sink.params = dict(body, version=5)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        app.advance(0)
    assert out.getvalue() == "" and abs(app.a.clamp - 0.0415) < 1e-12
    # an unreadable body is ignored, not obeyed
    app2, sink2 = _app()
    sink2.params = {"owner": "node", "params": {"budget": "fast"}}
    app2.advance(0)
    assert app2.node.budget == 0.3


@check("--node-limits FILE says so in capitals; the node's own does not")
def _():
    line = limits.source_line(NODE)
    assert "WARNING" in line and "NOT from the node" in line and FIXTURE in line, line
    live = NODE._replace(source="the node", from_node=True, boot_id="a91c3f", arm="L")
    assert "WARNING" not in limits.source_line(live)
    assert "boot a91c3f" in limits.source_line(live)
    assert NODE.as_facts()["source"].startswith("FILE ")
    assert live.as_facts()["from_node"] is True


@check("rule 6: the release fraction is ours and the joint limits are the node's")
def _():
    assert limits.DQ_RELEASE_FRACTION == 0.85 and limits.DQ_RESUME_FRACTION == 0.5
    assert PANDA_NODE.dq_limit == PANDA_DQ
    assert NODE.dq_limit == (2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26)   # the fixture's FR3
    dq = (0.1, -0.2, 0.0, 0.0, -2.3, 0.0, 0.0)
    j, fraction = limits.worst_joint(dq, PANDA_DQ)
    assert j == 4 and abs(fraction - 2.3 / 2.61) < 1e-12
    # the SAME velocities against the node's other arm: past the release on a Panda, under
    # it on the FR3. The limit is the node's, not a table here.
    assert fraction > limits.DQ_RELEASE_FRACTION
    assert limits.worst_joint(dq, NODE.dq_limit)[1] < limits.DQ_RELEASE_FRACTION
    assert limits.worst_joint((0.0,) * 6 + (float("nan"),), PANDA_DQ) == (6, math.inf)
    assert (limits.STATE_STALE_MS, limits.STATE_LOST_MS) == (50.0, 500.0)


# ---------------------------------------------------------------- teleop/params


def _owner(*extra):
    app, sink = _app("--quiet", *extra)
    owner = teleop_params.TeleopParams(app.a, app.node, TUNABLES, now_ns=lambda: 7)
    app.params = owner
    return app, sink, owner


@check("teleop/params: the schema is the flag table, bounded by the node's derived values")
def _():
    _, _, owner = _owner()
    schema = owner.schema()
    assert schema["owner"] == "teleop" and schema["schema_version"] == 1
    assert {t["name"] for t in TUNABLES} == set(schema["params"])
    assert schema["params"]["clamp"]["max"] == NODE.max_lead - NODE.budget / NODE.state_hz
    assert schema["params"]["clamp"]["bound"] == "max_lead"
    assert schema["params"]["max_step"]["max"] == NODE.max_step
    assert schema["params"]["rate"]["max"] == NODE.rate_hz * limits.KEEPALIVE_SLACK
    assert schema["derived"]["node_bounds_seen"]["max_lead"] == NODE.max_lead
    body = owner.body()
    assert body["params"]["clamp"] == 0.025 and body["version"] == 0
    assert body["params"]["dq_release_fraction"] == limits.DQ_RELEASE_FRACTION
    # a node with a looser lead publishes a looser slider, from the same table
    loose = teleop_params.TeleopParams(owner.a, _node(max_lead=0.12), TUNABLES)
    assert abs(loose.schema()["params"]["clamp"]["max"] - 0.117) < 1e-9, \
        loose.schema()["params"]["clamp"]["max"]


@check("teleop/params: a set is accepted on the Zenoh thread and applied ONLY by the loop")
def _():
    app, _, owner = _owner()
    reply = owner.on_set(json.dumps({"client_id": 7, "params": {"clamp": 0.03}}))
    assert reply["ok"] and reply["version"] == 1 and reply["params"]["clamp"] == 0.03
    assert app.a.clamp == 0.025, "the callback wrote args from the Zenoh thread"
    assert owner.body()["params"]["clamp"] == 0.03      # accepted, not yet applied
    with contextlib.redirect_stdout(io.StringIO()):
        app.advance(0)
    assert app.a.clamp == 0.03
    with contextlib.redirect_stdout(io.StringIO()):     # and applying is idempotent
        app.advance(0)
    assert app.a.clamp == 0.03 and owner.pending is None


@check("teleop/params: a clamp above the node's max_lead is clamped to it, and said so")
def _():
    app, _, owner = _owner()
    reply = owner.on_set(json.dumps({"client_id": 7, "params": {"clamp": 0.2}}))
    bound = NODE.max_lead - NODE.budget / NODE.state_hz
    assert reply["ok"] and reply["params"]["clamp"] == bound, reply
    assert reply["clamped"] == [{"field": "clamp", "index": None, "requested": 0.2,
                                 "stored": bound}]
    with contextlib.redirect_stdout(io.StringIO()):
        app.advance(0)
    assert app.a.clamp == bound
    limits.validate(app.a, NODE, lambda why: (_ for _ in ()).throw(AssertionError(why)))
    # under a node with a looser lead the SAME request is stored whole
    loose = teleop_params.TeleopParams(app.a, _node(max_lead=0.12), TUNABLES)
    assert loose.on_set(json.dumps({"client_id": 7, "params": {"clamp": 0.1}}))["clamped"] == []


@check("teleop/params: the refusals -- unknown, stale, not settable, bad types")
def _():
    _, _, owner = _owner()
    cases = (({"params": {"clamp": 0.03}}, "invalid", "client_id"),
             ({"client_id": 7, "mode": "x"}, "unknown_field", "mode"),
             ({"client_id": 7, "params": {"nope": 1}}, "unknown_field", "nope"),
             ({"client_id": 7, "params": {"dq_release_fraction": 0.9}}, "not_ready",
              "dq_release_fraction"),
             ({"client_id": 7, "params": {"clamp": "big"}}, "type", "clamp"),
             ({"client_id": 7, "base_version": 9, "params": {}}, "stale", None))
    for request, reason, field in cases:
        reply = owner.on_set(json.dumps(request))
        assert reply["ok"] is False and reply["reason"] == reason, (request, reply)
        assert reply["field"] == field, (request, reply)
        assert reply["version"] == 0 and owner.pending is None, reply
    assert owner.on_set(b"not json")["reason"] == "type"


@check("teleop/params: the scales are adopted at the next latch, never under a held grip")
def _():
    app, sink, owner = _owner()
    app.source.live = True
    t = 0
    for _ in range(20):
        t = _tick(app, AWAKE, t)
    for _ in range(5):
        t = _tick(app, HELD, t)
    assert app.prev_engaged and app.clutch.scale == 0.4
    assert owner.on_set(json.dumps({"client_id": 7, "params": {"spatial_scale": 0.8}}))["ok"]
    with contextlib.redirect_stdout(io.StringIO()):
        t = _tick(app, HELD, t)
    assert app.a.spatial_scale == 0.8 and app.clutch.scale == 0.4, "moved an engaged arm"
    with contextlib.redirect_stdout(io.StringIO()):
        t = _tick(app, AWAKE, t)                       # the grip is released here
        t = _tick(app, AWAKE, t)                       # and the next cycle adopts the scale
    assert app.clutch.scale == 0.8 and not app.prev_engaged


@check("teleop/params: --rate moves the keepalive period on the loop thread")
def _():
    app, _, owner = _owner()
    assert app.period_ns == 20_000_000
    assert owner.on_set(json.dumps({"client_id": 7, "params": {"rate": 100}}))["ok"]
    assert app.period_ns == 20_000_000
    with contextlib.redirect_stdout(io.StringIO()):
        app.advance(0)
    assert app.period_ns == 10_000_000 and app.a.rate == 100.0
