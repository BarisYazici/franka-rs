"""Zenoh side of the tuning bridge: mirrors the owners' `current`, computes metrics from
`state`, proxies schema/get/set, and runs the batched Apply (node first, teleop second). It holds
no authority: it checks types and forwards; the owner clamps, rejects and confirms.

A state stream faster than the engine's rate is decimated on the raw payload, before the decode
and before any lock (`SourceRate`), so a 1 kHz node costs the panel what a 100 Hz one does and the
metrics keep the band they were tuned at.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import zenoh

from .._errors import ProtocolError
from .._wire import ROBOT_MODES, decode_state
from . import zbus
from .metrics import STATE_HZ, MetricsEngine, Sample
from .presets import Presets
from .validation import Rejection, check_schema, check_set_request

log = logging.getLogger("tuning-bridge")
UNREACHABLE_S = 3.0
METRICS_HZ = 10.0
SEED_PERIOD_S = 2.0  # how often the seeder retries an owner that has not answered yet
RATE_WINDOW_S = 1.0  # how often the source rate is re-measured
KEEP_BAND = (70.0, 160.0)  # effective rates worth keeping: a deadband, so the ratio cannot flap
MODE_CODES = {name: code for code, name in enumerate(ROBOT_MODES)}
REFLEX_MODES = ("reflex", "user_stopped", "automatic_error_recovery")


class SourceRate:
    """How fast one arm publishes state, and the decimation that brings it to the engine's
    `STATE_HZ`. Measured on the node's own clock of the samples that are kept, so it says the same
    thing whatever the bridge host is doing; `every` is what `on_state` reads per message.

    The ratio is only re-picked when the current one leaves `KEEP_BAND`. Rounding alone flaps at
    any rate near an odd half-multiple of 100 (350 Hz: 7 changes in 20 s), and a change costs the
    whole ring, so the deadband is what keeps 60 s of history at every `state_hz` the node takes."""

    def __init__(self):
        self.every = 1  # keep 1 in `every` of the source stream
        self.source_hz: Optional[float] = None
        self._t0: Optional[float] = None
        self._kept = 0

    def update(self, t: float) -> bool:
        """Feed a kept sample's node time; True once a window, when a fresh rate was measured."""
        if self._t0 is None or t < self._t0:
            self._t0, self._kept = t, 0
            return False
        self._kept += 1
        if t - self._t0 < RATE_WINDOW_S:
            return False
        self.source_hz = self.every * self._kept / (t - self._t0)
        self._t0, self._kept = t, 0
        if not KEEP_BAND[0] <= self.source_hz / self.every < KEEP_BAND[1]:
            self.every = max(1, round(self.source_hz / STATE_HZ))
        return True

    def as_dict(self) -> Dict[str, Any]:
        return {"source_hz": self.source_hz, "decimation": self.every,
                "effective_hz": None if self.source_hz is None else self.source_hz / self.every}


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
        self.rate = SourceRate()
        self.seen = 0  # messages since the last one kept
        self._every, self._announced = 1, False  # the decimation the ring was built at
        self._event: tuple = (-1e9, None, 0)  # the ticker's last snapshot: when, what, at which reset
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
        # Decimate on the raw payload: no decode, no lock. Zenoh delivers one subscriber's samples
        # on one thread (checked for 1.10.1), and a lock here would be 2000 acquisitions a second,
        # the cost this path exists to avoid. `seen` only ever grows and the test is modular, so
        # even a lost increment under some future concurrent dispatch costs one irregular sample:
        # the keep phase shifts by one, it cannot drift or stall.
        self.seen += 1
        if self.seen % self.rate.every:
            return
        try:
            s = decode_state(payload)
        except ProtocolError as e:
            self.decode_failures += 1
            if self.decode_failures == 1:
                log.error("state on arm %s cannot be decoded (%s); the page will say so", self.arm, e)
            return
        if self.rate.update(s.t_node_ns / 1e9):
            self._on_rate()
        reflex = s.has_errors or s.robot_mode in REFLEX_MODES
        self.state_at = time.monotonic()
        self.metrics.push(Sample(t=s.t_node_ns / 1e9, dq=s.dq.tolist(), target=s.target[:3].tolist(),
                                 ee=s.position.tolist(), valid=not reflex))
        condition = {"phase": s.phase, "robot_mode": MODE_CODES.get(s.robot_mode),
                     "robot_mode_name": s.robot_mode, "has_errors": s.has_errors,
                     "holding": s.holding, "joints": s.joints, "client_id": s.holder, "reflex": reflex}
        if condition != self.condition:  # phase, mode or errors changed: tell the page now, not at 1 Hz
            self.condition = condition
            self.broadcast("status", self.status_event())

    def _on_rate(self) -> None:
        """A fresh rate measurement: hand the engine the rate its samples really arrive at (which
        the decimation cannot always make exactly 100 Hz), say it once at startup and whenever the
        decimation moves, and start the metrics over when it does."""
        every = self.rate.every
        self.metrics.set_rate(self.rate.source_hz / every)
        if every != self._every or not self._announced:
            log.info("arm %s publishes state at %.0f Hz: metrics from 1 in %d of it (%.0f Hz)", self.arm,
                     self.rate.source_hz, every, self.rate.source_hz / every)
            self._announced = True
        if every != self._every:
            self.metrics.reset()  # the ring and the filter state were built at the old rate
            self._every = every

    def status_event(self) -> Dict[str, Any]:
        """What the page shows in its header: the node's status entry merged with the condition
        the state stream reports (which is where `reflex` comes from)."""
        out: Dict[str, Any] = dict(self.status or {})
        if self.condition:
            out.update(self.condition)
            out["holder"] = self.condition["client_id"]
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

    def hello(self) -> Dict[str, Any]:
        """The SSE opening frame, taken under the lock; the caller serialises it outside."""
        with self.lock:
            return {"arm": self.arm, "reach": self.reach(), "current": dict(self.current),
                    "status": self.status_event(), "channels": self.channels()}

    def tick(self) -> None:
        self.broadcast("metrics", self.metrics_event(fresh=True))

    def metrics_event(self, fresh: bool = False) -> Dict[str, Any]:
        """The ticker's last snapshot with its age, recomputed only when that is older than one
        tick: a poll of `/api/<arm>/metrics` then costs a dict, not a pass over the ring. Liveness
        (reach, channels, rate) is always read now, it is where the page looks for trouble.

        Nothing is reported until the rate has been measured once: on a fast node the first second
        arrives un-decimated, and a number computed from it is a number about the wrong stream."""
        at, snap, resets = self._event
        age = time.monotonic() - at
        # A reset (a rate change, or the node's clock restarting) makes the cached numbers describe
        # a stream that no longer exists; they must not be served under the new rate's labels. The
        # count is read before the snapshot, so a reset landing during it invalidates this one too.
        if self.rate.source_hz is None:
            snap, age = {"ok": False, "reason": "measuring_rate"}, 0.0
        elif fresh or snap is None or age >= 1.0 / METRICS_HZ or resets != self.metrics.resets:
            taken_at = self.metrics.resets
            snap, age = self.metrics.snapshot(), 0.0
            self._event = (time.monotonic(), snap, taken_at)
        return {**snap, **self.rate.as_dict(), "reach": self.reach(),
                "channels": self.channels(), "snapshot_age_s": age}

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
        """The monitor for an arm the bus has presented, created on first sight. Never call this
        with a name that came from a URL: a phantom arm is one the seeder then queries forever."""
        with self.lock:
            if name not in self.arms:
                self.arms[name] = ArmMonitor(name, lambda owner, verb: self.owner_get(name, owner, verb))
            return self.arms[name]

    def known(self, name: str) -> Optional[ArmMonitor]:
        return self.arms.get(name)

    def _on_current(self, s: zenoh.Sample) -> None:
        key = str(s.key_expr)
        self.arm(zbus.arm_of(key)).on_current(zbus.owner_of(key) or zbus.NODE, zbus.loads(s.payload))

    def _on_state(self, s: zenoh.Sample) -> None:
        name = zbus.arm_of(str(s.key_expr))
        mon = self.arms.get(name) or self.arm(name)  # a dict lookup per message, not the bridge lock
        mon.on_state(s.payload.to_bytes())

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
                    name = zbus.arm_of(key)
                    arms.setdefault(name, {o: False for o in zbus.OWNERS})[owner] = True
                    self.arm(name)  # an owner answered for it: the arm exists
        return arms

    def linked(self) -> bool:
        """Whether this session has any zenoh peer or router at all, so the page can tell
        'nothing on the bus' from 'on the bus, but no owner answered'."""
        info = self.session.info
        return bool(info.peers_zid() or info.routers_zid())

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
        if reply.get("ok"):  # an owner answered for this arm, so the monitor is not a phantom
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
        mon = self.known(arm)
        builtins = []
        if mon and mon.toml_set:
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
        mon = self.known(arm)
        preset = {"name": name, "arm": arm, "note": str(body.get("note", "")),
                  "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "by": body.get("client_id", 0)}
        for owner in zbus.OWNERS:
            cur = (mon.current.get(owner) if mon else None) or self.owner_get(arm, owner, "get")
            if cur.get("params"):
                preset[owner] = cur["params"]
        # `effective_hz` rides along: two snapshots are only comparable at the same metrics rate.
        snap = mon.metrics_event() if mon else {}
        keys = ("j4_rms", "j4_peak_since_marker", "lag_ms", "track_p99_mm", "after", "effective_hz")
        preset["snapshot"] = {k: snap.get(k) for k in keys} if snap.get("ok") else {}
        self.presets.add(preset)
        return {"ok": True, "preset": preset}
