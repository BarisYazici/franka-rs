"""The node's packed `StateMsg` (`crates/franka-node/src/msg.rs`): little-endian, alignment one,
474 bytes, struct string `<BBBBHIQQQQd7d7d7d16d6d7dQQQ`. This is the one place the bridge and the
mock know the layout; both sides of the suite go through it, so a layout change that the node
bumps `VERSION` for stops here with a `DecodeError` rather than mis-reading a sample."""

from __future__ import annotations

import struct
from typing import Any, Dict

FORMAT = "<BBBBHIQQQQd7d7d7d16d6d7dQQQ"
SIZE = struct.calcsize(FORMAT)
VERSION = 1
PHASES = ("idle", "acquired", "active", "stopping", "faulted", "homing")
ROBOT_MODES = ("other", "idle", "move", "guiding", "reflex", "user_stopped", "automatic_error_recovery")
REFLEX_MODES = {4, 5, 6}  # reflex, user stopped, automatic error recovery
FLAG_HOLDING = 1
FLAG_JOINTS = 2
FIELDS = ("version", "phase", "robot_mode", "has_errors", "flags", "client_id", "seq_accepted",
          "t_send_ns_accepted", "t_node_ns", "robot_time_ms", "success_rate")
ARRAYS = (("q", 7), ("dq", 7), ("tau_ext", 7), ("o_t_ee", 16), ("o_f_ext_k", 6), ("target", 7))
TAIL = ("accepted", "refused", "dropped")
assert SIZE == 474


class DecodeError(ValueError):
    pass


def decode(payload: bytes) -> Dict[str, Any]:
    """Bytes -> dict with the message's field names (`t_node_ns`, `phase` as its name, ...)."""
    if len(payload) != SIZE:
        raise DecodeError(f"state is {len(payload)} bytes, StateMsg is {SIZE}")
    v = struct.unpack(FORMAT, payload)
    out: Dict[str, Any] = dict(zip(FIELDS, v[: len(FIELDS)]))
    if out["version"] != VERSION:
        raise DecodeError(f"StateMsg version {out['version']}, this decoder knows {VERSION}")
    i = len(FIELDS)
    for name, n in ARRAYS:
        out[name] = list(v[i:i + n])
        i += n
    out.update(zip(TAIL, v[i:]))
    out["phase"] = PHASES[out["phase"]] if out["phase"] < len(PHASES) else f"phase_{out['phase']}"
    out["has_errors"] = bool(out["has_errors"])
    out["holding"] = bool(out["flags"] & FLAG_HOLDING)
    out["joints"] = bool(out["flags"] & FLAG_JOINTS)
    return out


def is_reflex(state: Dict[str, Any]) -> bool:
    return state["has_errors"] or state["robot_mode"] in REFLEX_MODES


def encode(**f: Any) -> bytes:
    """Dict -> bytes, the mock's side; unset fields are zero, `phase` may be given by name."""
    phase = f.get("phase", 0)
    if isinstance(phase, str):
        phase = PHASES.index(phase)
    flags = (FLAG_HOLDING if f.get("holding") else 0) | (FLAG_JOINTS if f.get("joints") else 0)
    head = [VERSION, phase, f.get("robot_mode", 0), int(bool(f.get("has_errors"))), flags,
            f.get("client_id", 0), f.get("seq_accepted", 0), f.get("t_send_ns_accepted", 0),
            f.get("t_node_ns", 0), f.get("robot_time_ms", 0), f.get("success_rate", 0.0)]
    body = []
    for name, n in ARRAYS:
        arr = list(f.get(name, [0.0] * n))
        if len(arr) != n:
            raise ValueError(f"{name}: expected {n} elements, got {len(arr)}")
        body += arr
    return struct.pack(FORMAT, *head, *body, *(f.get(k, 0) for k in TAIL))
