"""teleop.py as a parameter owner: `franka/<arm>/teleop/params/{schema,get,set,current}`.

Same envelope as franka-node's own (docs/book/src/reference/node-parameters.md) with
`owner: "teleop"`, so the tuning panel discovers this process the way it discovers the node.

Two things make this safe to run beside a 50 Hz control loop:

* The schema is GENERATED from `cli.TUNABLES` and the run's `NodeLimits`. There is one table
  behind the flags and the published bounds, and a knob the node bounds (`bound: max_lead`)
  publishes the node's live number as its maximum, so the panel cannot offer a value that
  `limits.validate` would have refused at startup.
* A `set` is validated on the Zenoh thread and APPLIED ONLY ON THE LOOP THREAD. The callback
  leaves the accepted set in a one-slot, latest-wins holder; `Teleop.advance()` moves it into
  `args` between samples. The loop is single-threaded and every clamp reads `args`, so
  nothing else may write it. `ok: true` therefore means accepted, not yet applied -- exactly
  what acceptance means on the node's own seqlock.
"""
import copy
import json
import math
import time
import uuid

from . import limits

PUBLISH_EVERY_NS = 1_000_000_000

# Which node `derived` value bounds each knob. `clamp` and `clamp_rot` are bounded by the
# lead less one state tick of staleness, which is what limits.validate enforces.
BOUND = {
    "clamp": lambda a, n: limits.lead_bounds(a, n)[0],
    "clamp_rot": lambda a, n: limits.lead_bounds(a, n)[1],
    "max_step": lambda a, n: n.max_step,
    "max_step_rot": lambda a, n: n.max_step_rotation,
    "rate": lambda a, n: n.rate_hz * limits.KEEPALIVE_SLACK,
}


class Refusal(Exception):
    def __init__(self, reason, field, error):
        super().__init__(error)
        self.reason, self.field, self.error = reason, field, error

    def reply(self, version):
        return {"ok": False, "reason": self.reason, "field": self.field,
                "error": self.error, "version": version}


class TeleopParams:
    def __init__(self, a, node, tunables, now_ns=time.monotonic_ns):
        self.a, self.node, self.tunables = a, node, list(tunables)
        self.now_ns = now_ns
        self.boot_id = uuid.uuid4().hex[:16]
        self.version = 0
        self.origin = None
        self.accepted = {t["name"]: getattr(a, t["name"], t["default"])
                         for t in self.tunables}
        self.pending = None                      # one slot, latest wins; read by the loop
        self.publish = lambda body: None         # set by the sink once its publisher exists
        self.last_pub_ns = 0
        self.published_version = -1
        self.applied = []                        # names the loop applied, for the log

    # -- the published bodies ---------------------------------------------------

    def spec(self, t):
        """One `params` entry. Where the node bounds a knob its live value IS the maximum --
        the table's own `max` is only the fallback for a knob the node says nothing about,
        so a node that loosens a bound loosens the slider with it."""
        d = {"type": "f64", "min": t["min"], "max": t["max"], "default": t["default"],
             "policy": t["policy"], "group": t["group"], "scale": "linear"}
        if t["name"] in BOUND:
            d["max"] = BOUND[t["name"]](self.a, self.node)
            d["bound"] = t["bound"]
        if t.get("unit"):
            d["unit"] = t["unit"]
        if t.get("note"):
            d["note"] = t["note"]
        if not t["settable"]:
            d["policy"] = "read_only"
        return d

    def schema(self):
        return {"owner": "teleop", "arm": self.a.arm, "boot_id": self.boot_id,
                "schema_version": 1,
                "params": {t["name"]: self.spec(t) for t in self.tunables},
                "relations": [{"rule": "dq_resume_fraction < dq_release_fraction"},
                              {"rule": "every bound knob stays under the node's derived value; "
                                       "the node tightening one lowers it here"}],
                "derived": {"node_bounds_seen": self.node.derived,
                            "node_source": self.node.source,
                            "node_boot_id": self.node.boot_id}}

    def body(self):
        return {"owner": "teleop", "arm": self.a.arm, "boot_id": self.boot_id,
                "version": self.version, "t_node_ns": self.now_ns(), "origin": self.origin,
                "params": copy.deepcopy(self.accepted), "slewing": {},
                "dirty": self.version > 0}

    # -- the Zenoh thread -------------------------------------------------------

    def on_set(self, payload):
        """Validate a `params/set` and queue it. NOTHING here touches `self.a`."""
        try:
            request = json.loads(payload) if payload else {}
            if not isinstance(request, dict):
                raise Refusal("type", None, "request is not a JSON object")
            extra = set(request) - {"client_id", "base_version", "confirm", "params"}
            if extra:
                raise Refusal("unknown_field", sorted(extra)[0],
                              f"unknown request field {sorted(extra)[0]!r}")
            client = request.get("client_id", 0)
            if not isinstance(client, int) or isinstance(client, bool) or \
                    not 1 <= client <= 0xFFFFFFFF:
                raise Refusal("invalid", "client_id", "client_id is a non-zero u32")
            if "base_version" in request and request["base_version"] != self.version:
                raise Refusal("stale", None, f"base_version {request['base_version']} != "
                                             f"current {self.version}")
            scratch = dict(self.accepted)
            clamped = []
            for name, value in (request.get("params") or {}).items():
                scratch[name] = self._checked(name, value, clamped)
            self._relations(scratch)
        except Refusal as r:
            return r.reply(self.version)
        except ValueError as exc:
            return Refusal("type", None, f"payload is not JSON ({exc})").reply(self.version)
        self.accepted = scratch
        self.version += 1
        self.origin = {"version": self.version, "by": client, "at_ns": self.now_ns()}
        self.pending = dict(scratch)             # latest wins; the loop applies it
        return {"ok": True, "version": self.version, "params": copy.deepcopy(scratch),
                "clamped": clamped, "slewing": {},
                "note": "accepted; applied by the control loop on its next cycle"}

    def _checked(self, name, value, clamped):
        t = next((t for t in self.tunables if t["name"] == name), None)
        if t is None:
            raise Refusal("unknown_field", name, f"unknown parameter {name!r}")
        if not t["settable"]:
            raise Refusal("not_ready", name,
                          f"{name} is published for reading only in this version")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise Refusal("type", name, f"{name} wants a number, got {value!r}")
        value = float(value)
        if not math.isfinite(value):
            raise Refusal("non_finite", name, f"{name} is not finite")
        low, high = t["min"], self.spec(t)["max"]
        if not low <= value <= high:
            stored = min(max(value, low), high)
            clamped.append({"field": name, "index": None, "requested": value,
                            "stored": stored})
            value = stored
        return value

    def _relations(self, scratch):
        if scratch["clamp"] <= 0.0 or scratch["max_step"] <= 0.0:
            raise Refusal("invalid", "clamp", "clamp and max_step must be positive")

    # -- the loop thread --------------------------------------------------------

    def apply_pending(self):
        """Move an accepted set into `args`. The ONLY writer of `args` mid-session, and it
        runs between samples on the loop thread. Returns the names that changed."""
        pending, self.pending = self.pending, None
        if pending is None:
            return []
        changed = []
        for name, value in pending.items():
            t = next(t for t in self.tunables if t["name"] == name)
            if not t["settable"]:
                continue
            if getattr(self.a, name, None) != value:
                setattr(self.a, name, value)
                changed.append(name)
        if changed:
            self.applied.append((self.now_ns(), tuple(changed)))
        return changed

    def retune(self, node):
        """The node's limits moved under us: republish so the panel's bounds follow."""
        self.node = node
        self.published_version = -1

    def tick(self):
        """Publish `current` on every accepted set and at least every second."""
        now = self.now_ns()
        if self.published_version == self.version and self.last_pub_ns and \
                now - self.last_pub_ns < PUBLISH_EVERY_NS:
            return
        self.last_pub_ns, self.published_version = now, self.version
        self.publish(self.body())


def serve(session, arm, owner):
    """Declare `franka/<arm>/teleop/params/{schema,get,set}` and the `current` publisher on an
    open Zenoh session. Returns the publisher. The queryables run on Zenoh threads and only
    read or validate; `owner.apply_pending()` on the loop thread is what writes anything."""
    prefix = f"franka/{arm}/teleop/params"
    pub = session.declare_publisher(f"{prefix}/current")
    owner.publish = lambda body: pub.put(json.dumps(body).encode())

    def reply(query, name, body):
        # The CONCRETE key, never `query.key_expr`: a panel discovers arms with
        # `franka/*/teleop/params/schema` and reads the arm out of the reply's key.
        query.reply(f"{prefix}/{name}", json.dumps(body).encode())

    session.declare_queryable(f"{prefix}/schema",
                              lambda q: reply(q, "schema", owner.schema()))
    session.declare_queryable(f"{prefix}/get", lambda q: reply(q, "get", owner.body()))
    session.declare_queryable(f"{prefix}/set", lambda q: reply(
        q, "set", owner.on_set(None if q.payload is None else q.payload.to_bytes())))
    return pub
