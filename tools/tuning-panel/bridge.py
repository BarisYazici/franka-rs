"""Zenoh side of the tuning bridge: mirrors the owners' `current`, computes metrics from
`state`, proxies schema/get/set, and runs the batched Apply (node first, teleop second). It holds
no authority: it checks types and forwards; the owner clamps, rejects and confirms.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import zenoh

import zbus
import statemsg
from metrics import MetricsEngine, Sample
from presets import Presets
from validation import Rejection, check_schema, check_set_request

log = logging.getLogger("tuning-bridge")
UNREACHABLE_S = 3.0
METRICS_HZ = 10.0
SEED_PERIOD_S = 2.0  # how often the seeder retries an owner that has not answered yet
CONDITION_KEYS = ("phase", "robot_mode", "has_errors", "holding", "joints", "client_id")


class SseClient:
    """One browser stream. `closed` is set when the client falls behind; `_sse` then returns and
    the socket closes, so the page sees the drop instead of a silent keepalive."""

    def __init__(self):
        self.queue: queue.Queue = queue.Queue(maxsize=256)
        self.closed = threading.Event()


class ArmMonitor:
    """Everything the bridge knows about one arm: last `current` per owner, status, metrics.
    `query(owner, verb)` reaches the owners so the headroom limits can be seeded from the node's
    `derived.dq_limit` and the teleop client's `dq_release_fraction` before their `current` arrives."""

    def __init__(self, arm: str, query: Callable[[str, str], Dict[str, Any]]):
        self.arm = arm
        self.query = query
        self.lock = threading.Lock()
        self.current: Dict[str, Dict[str, Any]] = {}
        self.current_at: Dict[str, float] = {}
        self.toml_set: Dict[str, Dict[str, Any]] = {}  # version-0 params per owner (built-in preset)
        self.status: Optional[Dict[str, Any]] = None  # the node's ArmStatus entry for this arm
        self.status_at: Optional[float] = None
        self.condition: Optional[Dict[str, Any]] = None  # phase, mode, errors: from the state stream
        self.state_at: Optional[float] = None
        self.decode_failures = 0
        self.metrics = MetricsEngine()
        self.clients: List[SseClient] = []

    def on_current(self, owner: str, body: Dict[str, Any]) -> None:
        with self.lock:
            self.current[owner] = body
            self.current_at[owner] = time.monotonic()
            if body.get("version") == 0:
                self.toml_set[owner] = {"boot_id": body.get("boot_id"), "params": body.get("params", {})}
        if owner == zbus.TELEOP and "dq_release_fraction" in body.get("params", {}):
            self.metrics.set_limits(release_fraction=body["params"]["dq_release_fraction"])
        self.broadcast("current", {"owner": owner, **body})

    def ensure_limits(self) -> None:
        """Seed what the headroom metric needs from the owners; a no-op once both are known.
        Each call may block for up to the query timeout, so it runs on the bridge's seeder thread
        only, never on the ticker or an HTTP handler: a slow owner would stall the liveness path
        (measured: 10 Hz metrics fell to 0.4 Hz with a 3 s owner on the ticker)."""
        if self.metrics.dq_limit is None:
            derived = self.query(zbus.NODE, "schema").get("derived") or {}
            if "dq_limit" in derived:
                self.metrics.set_limits(dq_limit=derived["dq_limit"])
        if self.metrics.release_fraction is None:
            params = self.query(zbus.TELEOP, "get").get("params") or {}
            if "dq_release_fraction" in params:
                self.metrics.set_limits(release_fraction=params["dq_release_fraction"])

    def on_status(self, entry: Dict[str, Any]) -> None:
        """One `arms[]` entry of the node's status (phase, holder, mode, counters); it carries no
        robot mode or errors, so the reflex flag rides on the state-derived condition."""
        with self.lock:
            self.status, self.status_at = entry, time.monotonic()
        self.broadcast("status", self.status_event())

    def on_state(self, payload: bytes) -> None:
        try:
            s = statemsg.decode(payload)
        except statemsg.DecodeError as e:
            self.decode_failures += 1
            if self.decode_failures == 1:
                log.error("state on arm %s cannot be decoded (%s); the page will say so", self.arm, e)
            return
        reflex = statemsg.is_reflex(s)
        self.state_at = time.monotonic()
        self.metrics.push(Sample(t=s["t_node_ns"] / 1e9, dq=s["dq"], target=s["target"][:3],
                                 ee=s["o_t_ee"][12:15], valid=not reflex))
        condition = {k: s[k] for k in CONDITION_KEYS}
        condition["reflex"] = reflex
        if condition != self.condition:  # phase, mode or errors changed: tell the page now, not at 1 Hz
            self.condition = condition
            self.broadcast("status", self.status_event())

    def status_event(self) -> Dict[str, Any]:
        """What the page shows in its header: the node's status entry merged with the condition
        the state stream reports (which is where `reflex` comes from)."""
        out: Dict[str, Any] = dict(self.status or {})
        if self.condition:
            out.update(self.condition)
            out["holder"] = self.condition["client_id"]
            out["robot_mode_name"] = statemsg.ROBOT_MODES[self.condition["robot_mode"]] \
                if self.condition["robot_mode"] < len(statemsg.ROBOT_MODES) else "?"
        out["reflex"] = bool(self.condition and self.condition["reflex"])
        out["channels"] = self.channels()
        return out

    def reach(self) -> Dict[str, bool]:
        now = time.monotonic()
        return {o: (now - self.current_at.get(o, -1e9)) < UNREACHABLE_S for o in zbus.OWNERS}

    def channels(self) -> Dict[str, Any]:
        """Health of the two node channels the page depends on, so it can say when either is absent."""
        now = time.monotonic()
        return {"state_age_s": None if self.state_at is None else now - self.state_at,
                "status_age_s": None if self.status_at is None else now - self.status_at,
                "state_decode_failures": self.decode_failures}

    def broadcast(self, event: str, data: Any) -> None:
        msg = f"event: {event}\ndata: {json.dumps(data, separators=(',', ':'))}\n\n".encode()
        with self.lock:
            for c in list(self.clients):
                try:
                    c.queue.put_nowait(msg)
                except queue.Full:
                    c.closed.set()
                    self.clients.remove(c)

    def tick(self) -> None:
        self.broadcast("metrics", self.metrics_event())

    def metrics_event(self) -> Dict[str, Any]:
        snap = self.metrics.snapshot()
        snap["reach"] = self.reach()
        snap["channels"] = self.channels()
        return snap

    def mark(self, label: str, version: Optional[int] = None) -> Optional[Dict[str, Any]]:
        m = self.metrics.mark(label, version)
        return None if m is None else {"t": m.t, "label": m.label, "version": m.version}


class Bridge:
    def __init__(self, session: zenoh.Session, presets_path: str, timeout: float = 1.0):
        self.session, self.timeout = session, timeout
        self.presets = Presets(presets_path)
        self.arms: Dict[str, ArmMonitor] = {}
        self.lock = threading.Lock()
        self.subs = [
            session.declare_subscriber("franka/*/params/current", self._on_current),
            session.declare_subscriber("franka/*/teleop/params/current", self._on_current),
            session.declare_subscriber("franka/*/state", self._on_state),
            session.declare_subscriber("franka/node/*/status", self._on_node_status),
        ]
        self._stop = threading.Event()
        threading.Thread(target=self._ticker, daemon=True, name="metrics-ticker").start()
        threading.Thread(target=self._seeder, daemon=True, name="limit-seeder").start()

    def arm(self, name: str) -> ArmMonitor:
        with self.lock:
            if name not in self.arms:
                self.arms[name] = ArmMonitor(name, lambda owner, verb: self.owner_get(name, owner, verb))
            return self.arms[name]

    def _on_current(self, s: zenoh.Sample) -> None:
        key = str(s.key_expr)
        self.arm(zbus.arm_of(key)).on_current(zbus.owner_of(key) or zbus.NODE, zbus.loads(s.payload))

    def _on_state(self, s: zenoh.Sample) -> None:
        self.arm(zbus.arm_of(str(s.key_expr))).on_state(s.payload.to_bytes())

    def _on_node_status(self, s: zenoh.Sample) -> None:
        """`franka/node/<name>/status` is node-scoped: one message, one entry per arm."""
        for entry in zbus.loads(s.payload).get("arms", []):
            if "name" in entry:
                self.arm(entry["name"]).on_status(entry)

    def _ticker(self) -> None:
        while not self._stop.wait(1.0 / METRICS_HZ):
            with self.lock:
                arms = list(self.arms.values())
            for a in arms:
                if a.clients:
                    try:
                        a.tick()
                    except Exception:  # a dead ticker would leave every page showing a frozen frame
                        log.exception("metrics tick failed for arm %s", a.arm)

    def _seeder(self) -> None:
        while not self._stop.wait(SEED_PERIOD_S):
            with self.lock:
                arms = list(self.arms.values())
            for a in arms:
                try:
                    a.ensure_limits()
                except Exception:
                    log.exception("seeding limits failed for arm %s", a.arm)

    # ---- owner access ------------------------------------------------------------------------
    def discover(self) -> Dict[str, Dict[str, bool]]:
        arms: Dict[str, Dict[str, bool]] = {}
        for owner in zbus.OWNERS:
            for key, body in zbus.query_json(self.session, zbus.params_prefix("*", owner) + "/schema",
                                             timeout=self.timeout):
                if body.get("owner"):
                    arms.setdefault(zbus.arm_of(key), {o: False for o in zbus.OWNERS})[owner] = True
        return arms

    def owner_get(self, arm: str, owner: str, verb: str, body: Optional[Dict] = None) -> Dict[str, Any]:
        r = zbus.query_one(self.session, f"{zbus.params_prefix(arm, owner)}/{verb}", body, self.timeout)
        if r is None:
            return {"ok": False, "reason": "unreachable", "field": None, "error": f"{owner} did not answer"}
        return r

    def schema(self, arm: str, owner: str) -> Dict[str, Any]:
        s = self.owner_get(arm, owner, "schema")
        if s.get("params"):
            problems = check_schema(s)
            if problems:  # a log slider with min 0 would put NaN on a control that sets a real value
                return {"ok": False, "reason": "bad_schema", "field": None, "error": "; ".join(problems)}
        return s

    def set_params(self, arm: str, owner: str, body: Dict[str, Any]) -> Dict[str, Any]:
        schema = self.schema(arm, owner)
        if not schema.get("params"):
            return {"ok": False, "reason": "unreachable", "field": None, "error": f"no schema from {owner}"}
        try:
            req = check_set_request(schema, body)
        except Rejection as r:
            return r.as_reply()
        reply = self.owner_get(arm, owner, "set", req)
        if reply.get("ok"):
            reply["marker"] = self.arm(arm).mark(f"{owner} v{reply.get('version')}", reply.get("version"))
        return reply

    def apply(self, arm: str, body: Dict[str, Any]) -> Dict[str, Any]:
        """Node first; a node refusal aborts the teleop send (its values may depend on node bounds)."""
        out: Dict[str, Any] = {"node": None, "teleop": None}
        for owner in zbus.OWNERS:
            req = body.get(owner)
            if not req:
                continue
            out[owner] = self.set_params(arm, owner, req)
            if not out[owner].get("ok"):
                if owner == zbus.NODE and body.get(zbus.TELEOP):
                    out[zbus.TELEOP] = {"ok": False, "reason": "aborted", "field": None,
                                        "error": "not sent: node refused"}
                break
        out["ok"] = all(v.get("ok") for v in (out["node"], out["teleop"]) if v)
        return out

    def presets_for(self, arm: str) -> Dict[str, Any]:
        doc = self.presets.load()
        mon = self.arm(arm)
        builtins = []
        if mon.toml_set:
            builtins.append({"name": "toml", "builtin": True, "note": "as loaded at boot",
                             **{o: v["params"] for o, v in mon.toml_set.items()}})
        defaults = {}
        for owner in zbus.OWNERS:
            s = self.schema(arm, owner)
            if s.get("params"):
                defaults[owner] = {k: v.get("default") for k, v in s["params"].items()}
        if defaults:
            builtins.append({"name": "library-defaults", "builtin": True, "note": "schema defaults", **defaults})
        return {"schema_version": 1, "presets": builtins + [p for p in doc["presets"] if p.get("arm") == arm]}

    def save_preset(self, arm: str, body: Dict[str, Any]) -> Dict[str, Any]:
        name = str(body.get("name", "")).strip()
        if not name or name in ("toml", "library-defaults"):
            return {"ok": False, "error": "preset needs a name that is not a built-in"}
        mon = self.arm(arm)
        preset = {"name": name, "arm": arm, "note": str(body.get("note", "")),
                  "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "by": body.get("client_id", 0)}
        for owner in zbus.OWNERS:
            cur = mon.current.get(owner) or self.owner_get(arm, owner, "get")
            if cur.get("params"):
                preset[owner] = cur["params"]
        snap = mon.metrics.snapshot()
        preset["snapshot"] = {k: snap.get(k) for k in ("j4_rms", "j4_peak_since_marker", "lag_ms",
                                                         "track_p99_mm", "after")} if snap.get("ok") else {}
        self.presets.add(preset)
        return {"ok": True, "preset": preset}
