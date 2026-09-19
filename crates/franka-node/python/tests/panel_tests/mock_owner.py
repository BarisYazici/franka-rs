"""Mock of the two parameter owners (franka-node and teleop.py) plus a synthetic state stream.

Serves `franka/<arm>/params/{schema,get,set}` and `franka/<arm>/teleop/params/{schema,get,set}`,
publishes `.../params/current` on change and every second, the packed `StateMsg` on
`franka/<arm>/state` at 100 Hz and the node-scoped `franka/node/mock-<arm>/status` at 1 Hz, both
in the node's own wire shapes. The `set` path runs the owner-side order:
types -> merge -> relations -> clamp -> confirm_above -> store.

`franka/<arm>/mock/ctl` accepts
{"robot_mode": n, "has_errors": b, "phase": s, "silent": b} so tests can drive the reflex path
and a node that has gone quiet. A developer and test tool, not in the wheel; run it from the
repository with the client installed: `python crates/franka-node/python/tests/panel_tests/mock_owner.py`.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import random
import threading
import time
import uuid
from typing import Any, Dict, List, Optional

import zenoh

from franka_node._wire import PHASES
from franka_node.panel import zbus
from franka_node.panel.validation import Rejection, check_params, crossings, parse_type
from mock_schemas import DQ_LIMIT, SLEW_TAU_S, node_schema, teleop_schema
from mock_state import encode_state

class Owner:
    """One parameter owner: schema, effective values, version, slew bookkeeping."""

    def __init__(self, session: zenoh.Session, prefix: str, schema: Dict[str, Any], relations):
        self.session, self.prefix, self.schema, self.relations = session, prefix, schema, relations
        self.params = {k: copy.deepcopy(v["default"]) for k, v in schema["params"].items()}
        self.version = 0
        self.origin: Optional[Dict[str, Any]] = None
        self.slew_started: Dict[str, float] = {}
        self.lock = threading.Lock()
        self.pub = session.declare_publisher(f"{prefix}/current")
        self._q = [session.declare_queryable(f"{prefix}/schema", self._on_schema),
                   session.declare_queryable(f"{prefix}/get", self._on_get),
                   session.declare_queryable(f"{prefix}/set", self._on_set)]

    # ---- state ------------------------------------------------------------------------------
    def slewing(self) -> Dict[str, float]:
        now = time.monotonic()
        out = {}
        for k, t0 in list(self.slew_started.items()):
            tau = self.schema["params"][k].get("slew_tau_s", SLEW_TAU_S)
            frac = math.exp(-(now - t0) / tau)
            if frac < 0.01:
                del self.slew_started[k]
            else:
                out[k] = round(frac, 3)
        return out

    def body(self) -> Dict[str, Any]:
        return {"owner": self.schema["owner"], "arm": self.schema["arm"], "version": self.version,
                "boot_id": self.schema["boot_id"], "t_node_ns": time.monotonic_ns(), "origin": self.origin,
                "params": copy.deepcopy(self.params), "slewing": self.slewing(),
                "dirty": self.version > 0}

    def publish_current(self) -> None:
        with self.lock:
            self.pub.put(zbus.dumps(self.body()))

    # ---- queryables ---------------------------------------------------------------------------
    def _on_schema(self, q: zenoh.Query) -> None:
        q.reply(f"{self.prefix}/schema", zbus.dumps(self.schema))

    def _on_get(self, q: zenoh.Query) -> None:
        with self.lock:
            q.reply(f"{self.prefix}/get", zbus.dumps(self.body()))

    def _on_set(self, q: zenoh.Query) -> None:
        try:
            req = json.loads(q.payload.to_bytes()) if q.payload is not None else {}
        except ValueError:
            q.reply(f"{self.prefix}/set", zbus.dumps({"ok": False, "reason": "type", "field": None,
                                                      "error": "payload is not JSON", "version": self.version}))
            return
        with self.lock:
            q.reply(f"{self.prefix}/set", zbus.dumps(self.apply(req)))
        self.publish_current()

    def apply(self, req: Dict[str, Any]) -> Dict[str, Any]:
        """The owner-side order. Nothing is stored unless every step passes."""
        try:
            extra = set(req) - {"client_id", "base_version", "confirm", "params"}
            if extra:
                raise Rejection("unknown_field", sorted(extra)[0], f"unknown request field {sorted(extra)[0]!r}")
            incoming = check_params(self.schema, req.get("params", {}))
            if "base_version" in req and req["base_version"] != self.version:
                raise Rejection("stale", None, f"base_version {req['base_version']} != current {self.version}")
            scratch = copy.deepcopy(self.params)
            scratch.update(incoming)
            self.relations(scratch)
            clamped = self._clamp(incoming, scratch)
            need = [f for f in crossings(self.schema, {k: scratch[k] for k in incoming}, self.params)
                    if f not in set(req.get("confirm", []))]
            if need:
                raise Rejection("needs_confirm", need[0], f"confirm required for {', '.join(need)}")
        except Rejection as r:
            return r.as_reply(self.version)
        now = time.monotonic()
        for k in incoming:
            if scratch[k] != self.params[k] and self.schema["params"][k].get("policy") == "slew":
                self.slew_started[k] = now
        self.params = scratch
        self.version += 1
        self.origin = {"version": self.version, "by": req.get("client_id", 0), "at_ns": time.monotonic_ns()}
        return {"ok": True, "version": self.version, "params": copy.deepcopy(self.params),
                "clamped": clamped, "slewing": self.slewing()}

    def _clamp(self, incoming: Dict[str, Any], scratch: Dict[str, Any]) -> List[Dict[str, Any]]:
        clamped = []
        for k in incoming:
            spec = self.schema["params"][k]
            kind, n = parse_type(spec["type"])
            if kind == "bool":
                continue
            if n is None:
                s = min(max(scratch[k], spec["min"]), spec["max"])
                if s != scratch[k]:
                    clamped.append({"field": k, "index": None, "requested": scratch[k], "stored": s})
                    scratch[k] = s
            else:
                for i in range(n):
                    s = min(max(scratch[k][i], spec["min"][i]), spec["max"][i])
                    if s != scratch[k][i]:
                        clamped.append({"field": k, "index": i, "requested": scratch[k][i], "stored": s})
                        scratch[k][i] = s
        return clamped


def node_domain_checks(p: Dict[str, Any]) -> None:
    """Runs in the relations step, before the clamp: the two hard-invalid domains are refused
    rather than snapped to the bound. v1 has no cross-field rule."""
    for k in ("ik_damping", "cartesian_stiffness"):
        if p[k] <= 0:
            raise Rejection("invalid", k, f"{k} must be > 0, got {p[k]}")


def teleop_relations(p: Dict[str, Any]) -> None:
    if p["dq_resume_fraction"] >= p["dq_release_fraction"]:
        raise Rejection("relation", "dq_resume_fraction",
                        f"dq_resume_fraction {p['dq_resume_fraction']} >= dq_release_fraction {p['dq_release_fraction']}")
    w = p["workspace"]
    if any(w[i] >= w[i + 3] for i in range(3)):
        raise Rejection("relation", "workspace", "workspace min must be below max on every axis")


class StateStream(threading.Thread):
    """Synthetic 100 Hz state whose lag and j4 jitter respond to the node's parameters, so the
    before/after windows show a change when a slider moves. Not a physics model."""

    def __init__(self, session: zenoh.Session, arm: str, node: Owner, hz: float = 100.0):
        super().__init__(daemon=True)
        self.arm = arm
        self.pub = session.declare_publisher(f"franka/{arm}/state")
        self.status_pub = session.declare_publisher(f"franka/node/mock-{arm}/status")
        self.node, self.hz = node, hz
        self.ctl = {"robot_mode": 2, "has_errors": False, "phase": "active", "holder": 7200, "silent": False}
        self.session_q = session.declare_queryable(f"franka/{arm}/mock/ctl", self._on_ctl)
        self.stop = threading.Event()

    def _on_ctl(self, q: zenoh.Query) -> None:
        if q.payload is not None:
            self.ctl.update(json.loads(q.payload.to_bytes()))
        q.reply(str(q.key_expr), zbus.dumps({"ok": True, **self.ctl}))

    def run(self) -> None:
        t0 = time.monotonic()
        i = 0
        hist: List[List[float]] = []
        rng = random.Random(1)
        last_status = 0.0
        while not self.stop.is_set():
            t = i / self.hz
            if self.ctl["silent"]:  # a dead node: nothing on the bus
                i += 1
                time.sleep(max(0.0, t0 + i / self.hz - time.monotonic()))
                continue
            p = self.node.params
            gain = p["velocity_feedforward_gain"]
            lag_s = 0.06 + 0.10 * (1 - gain)  # feedforward off -> slower
            jitter = 0.012 + 0.03 * gain * (p["velocity_feedforward_cutoff"] / 1000) \
                * (1 + p["joint_damping"][3] / 60) * (1 + p["ik_damping"] * 0.5)
            target = [0.45 + 0.10 * math.sin(2 * math.pi * 0.25 * t), 0.08 * math.sin(2 * math.pi * 0.17 * t),
                      0.35 + 0.05 * math.sin(2 * math.pi * 0.11 * t)]
            hist.append(target)
            k = min(len(hist) - 1, int(lag_s * self.hz))
            ee = [x + rng.gauss(0, 0.0005) for x in hist[-1 - k]]
            hist = hist[-200:]
            dq = [0.2 * math.sin(2 * math.pi * 0.25 * t + j) for j in range(7)]
            dq[3] += jitter * math.sqrt(2) * math.sin(2 * math.pi * 7.5 * t) + rng.gauss(0, 0.002)
            # headroom follows the budget: at the schema's confirm threshold j2 sits at ~60 % of its limit
            dq[1] = 0.6 * DQ_LIMIT[1] * p["budget"][0] / self.node.schema["params"]["budget"]["confirm_above"][0]
            o_t_ee = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, ee[0], ee[1], ee[2], 1]
            self.pub.put(encode_state(
                phase=PHASES.index(self.ctl["phase"]), robot_mode=self.ctl["robot_mode"],
                has_errors=self.ctl["has_errors"], client_id=self.ctl["holder"], t_node_ns=time.monotonic_ns(),
                robot_time_ms=int(t * 1e3), success_rate=1.0, q=[0.0, -0.78, 0.0, -2.36, 0.0, 1.57, 0.78],
                dq=dq, o_t_ee=o_t_ee, target=target + [0.0, 0.0, 0.0, 1.0], accepted=i))
            if t - last_status >= 1.0:
                last_status = t
                self.status_pub.put(zbus.dumps({
                    "node": f"mock-{self.arm}", "version": "mock", "uptime_s": int(t),
                    "arms": [{"name": self.arm, "phase": self.ctl["phase"], "holder": self.ctl["holder"],
                              "mode": "cartesian", "accepted": i, "refused": 0, "dropped": 0,
                              "decode_failures": 0, "success_rate": 1.0, "recording": None,
                              "episode": None, "gripper": None}]}))
            i += 1
            time.sleep(max(0.0, t0 + i / self.hz - time.monotonic()))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--arm", default="L")
    ap.add_argument("--no-state", action="store_true", help="serve params only, no state stream")
    ap.add_argument("--schema-override", default="{}", metavar="JSON",
                    help='patch node schema entries (or "derived"), e.g. '
                         '\'{"joint_damping": {"max": [45,45,45,45,45,45,45]}}\'')
    zbus.add_zenoh_args(ap)
    a = ap.parse_args()
    session = zenoh.open(zbus.config_from_args(a))
    boot = uuid.uuid4().hex[:8]
    ns = node_schema(a.arm, boot)
    for k, patch in json.loads(a.schema_override).items():
        (ns["derived"] if k == "derived" else ns["params"][k]).update(patch)
    node = Owner(session, zbus.params_prefix(a.arm, zbus.NODE), ns, node_domain_checks)
    teleop = Owner(session, zbus.params_prefix(a.arm, zbus.TELEOP),
                   teleop_schema(a.arm, uuid.uuid4().hex[:8], ns["derived"]), teleop_relations)
    stream = None if a.no_state else StateStream(session, a.arm, node)
    if stream:
        stream.start()
    print(f"mock owner: arm {a.arm}, node boot {boot}, params at {node.prefix} and {teleop.prefix}", flush=True)
    try:
        while True:
            time.sleep(1.0)
            if stream and stream.ctl["silent"]:
                continue
            node.publish_current()
            teleop.publish_current()
    except KeyboardInterrupt:
        pass
    finally:
        if stream:
            stream.stop.set()
        session.close()


if __name__ == "__main__":
    main()
