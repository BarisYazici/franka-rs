"""The node's own limits, read FROM the node and never mirrored here.

franka-node publishes everything its guard enforces. `params/schema` carries the fixed
`derived` block (max_lead, max_step, rate_hz, state_hz, stop_after_ms, the backend leash and
the connected arm's dq_limit); `params/get` and `params/current` carry the live, tunable
Cartesian budgets. A client that keeps its own copy of those numbers drifts from them -- and
has, twice -- so this module is the only way they enter the process, and a value that is
missing or unreadable is a startup failure naming the key, never a default.

    from_node(schema, get, source=...)   the two bodies the node serves
    from_file(path)                      {"schema": ..., "get": ...} as JSON, off hardware
    budgets_from(body)                   the two live numbers, re-read on every params/current
"""
import json
import math
import os
from typing import NamedTuple, Tuple

SCHEMA_VERSION = 1
DERIVED = ("max_lead", "max_lead_rotation", "max_step", "max_step_rotation",
           "rate_hz", "state_hz", "stop_after_ms")
DQ_JOINTS = 7


class LimitsError(Exception):
    """Why the node's limits could not be read. The caller exits; it never guesses."""


class NodeLimits(NamedTuple):
    """One arm's limits as the node reports them, plus where they were read from."""

    max_step: float
    max_step_rotation: float
    max_lead: float
    max_lead_rotation: float
    rate_hz: float
    stop_after_ms: float
    state_hz: float
    budget: float               # m/s,   params.budget[0], live and tunable
    budget_rotation: float      # rad/s, params.rotation_budget[0]
    dq_limit: Tuple             # rad/s per joint, the connected arm's FCI-version limits
    leash: Tuple                # (translation m, rotation rad), the node's backend leash
    arm: str
    boot_id: str
    version: int
    source: str                 # "the node" or "FILE <path>"
    from_node: bool

    @property
    def derived(self):
        """The `derived` block as the node published it, for --probe-limits."""
        return {"max_lead": self.max_lead, "max_lead_rotation": self.max_lead_rotation,
                "max_step": self.max_step, "max_step_rotation": self.max_step_rotation,
                "rate_hz": self.rate_hz, "state_hz": self.state_hz,
                "stop_after_ms": self.stop_after_ms,
                "leash": {"translation": self.leash[0], "rotation": self.leash[1]},
                "dq_limit": list(self.dq_limit)}

    def as_facts(self):
        """The summary's `node_limits` block; `source` is what facts.py reads."""
        return {"source": self.source, "from_node": self.from_node, "arm": self.arm,
                "boot_id": self.boot_id, "version": self.version,
                "budget": self.budget, "budget_rotation": self.budget_rotation,
                **self.derived}


def _finite(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LimitsError(f"{where} is {value!r}, not a number")
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise LimitsError(f"{where} is {value}, not a finite positive number")
    return value


def _at(body, key, where):
    if not isinstance(body, dict) or key not in body:
        raise LimitsError(f"{where} has no {key!r}: this node does not publish the limits "
                          f"teleop validates against")
    return body[key]


def _envelope(body, what):
    if not isinstance(body, dict):
        raise LimitsError(f"{what} is not a JSON object")
    if body.get("owner") != "node":
        raise LimitsError(f"{what} owner is {body.get('owner')!r}, not 'node'")
    return body


def budgets_from(body):
    """(budget m/s, rotation budget rad/s) of a `params/get` or `params/current` body."""
    params = _at(_envelope(body, "params/get"), "params", "params/get")
    out = []
    for key in ("budget", "rotation_budget"):
        value = _at(params, key, "params/get params")
        if not isinstance(value, (list, tuple)) or not value:
            raise LimitsError(f"params/get params.{key} is {value!r}, not a [v, a, j] array")
        out.append(_finite(value[0], f"params/get params.{key}[0]"))
    return tuple(out)


def from_node(schema, get, *, source="the node", from_node=True):
    """A NodeLimits from the two bodies the node serves. Every key is required."""
    schema = _envelope(schema, "params/schema")
    version = _at(schema, "schema_version", "params/schema")
    if version != SCHEMA_VERSION:
        raise LimitsError(f"params/schema schema_version is {version!r}, not {SCHEMA_VERSION}; "
                          f"this teleop does not know that protocol")
    derived = _at(schema, "derived", "params/schema")
    values = {k: _finite(_at(derived, k, "params/schema derived"), f"derived.{k}")
              for k in DERIVED}
    dq = _at(derived, "dq_limit", "params/schema derived")
    if not isinstance(dq, (list, tuple)) or len(dq) != DQ_JOINTS:
        raise LimitsError(f"derived.dq_limit is {dq!r}, not {DQ_JOINTS} joint velocity limits")
    leash = _at(derived, "leash", "params/schema derived")
    budget, budget_rotation = budgets_from(get)
    return NodeLimits(
        max_step=values["max_step"], max_step_rotation=values["max_step_rotation"],
        max_lead=values["max_lead"], max_lead_rotation=values["max_lead_rotation"],
        rate_hz=values["rate_hz"], stop_after_ms=values["stop_after_ms"],
        state_hz=values["state_hz"], budget=budget, budget_rotation=budget_rotation,
        dq_limit=tuple(_finite(v, f"derived.dq_limit[{i}]") for i, v in enumerate(dq)),
        leash=(_finite(_at(leash, "translation", "derived.leash"), "derived.leash.translation"),
               _finite(_at(leash, "rotation", "derived.leash"), "derived.leash.rotation")),
        arm=str(schema.get("arm", "?")), boot_id=str(get.get("boot_id", "?")),
        version=int(get.get("version", 0)), source=source, from_node=from_node)


def from_file(path):
    """`{"schema": <params/schema body>, "get": <params/get body>}`, as the node serves them.

    The deliberate offline path: the dry run, the tests, and a node too old to answer. It is
    never a fallback -- the caller passes --node-limits for it and the startup line says so.
    """
    path = os.path.abspath(path)
    try:
        with open(path) as fh:
            body = json.load(fh)
    except OSError as exc:
        raise LimitsError(f"--node-limits {path}: {exc.strerror}") from exc
    except ValueError as exc:
        raise LimitsError(f"--node-limits {path}: not JSON ({exc})") from exc
    if not isinstance(body, dict) or "schema" not in body or "get" not in body:
        raise LimitsError(f"--node-limits {path}: wants "
                          f'{{"schema": <params/schema body>, "get": <params/get body>}}')
    return from_node(body["schema"], body["get"], source=f"FILE {path}", from_node=False)


def query(session, arm, timeout=2.0, where="the bus"):
    """`franka/<arm>/params/{schema,get}` off an open Zenoh session.

    Fail-closed by construction: every path out of here is either a NodeLimits built from
    what the node answered or a LimitsError naming what was missing. There is no default.
    """
    def one(name):
        key = f"franka/{arm}/params/{name}"
        try:
            for reply in session.get(key, payload=None, timeout=timeout):
                if reply.ok is None:
                    raise LimitsError(f"{key}: {reply.err.payload.to_string()!r}")
                return json.loads(reply.ok.payload.to_string())
        except LimitsError:
            raise
        except Exception as exc:              # zenoh's own errors have no public type
            raise LimitsError(f"{key}: query failed: {exc}") from exc
        raise LimitsError(f"{key}: no reply within {timeout:.0f} s. Is franka-node running "
                          f"on {where} with an arm named {arm!r}?")

    return from_node(one("schema"), one("get"))
