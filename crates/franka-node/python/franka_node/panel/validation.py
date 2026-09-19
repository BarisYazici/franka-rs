"""Type-only checks on a `set` request against the owner's published schema.

The brief: the web layer checks types, the realtime owner is the authority on bounds. So this
module never looks at `min`/`max`; it refuses what the owner would refuse structurally (unknown
field, wrong type, wrong length, non-finite) so a typo never reaches the arm, and it parses the
schema so the panel and the tests share one reader.
"""

from __future__ import annotations

import math
import re
from typing import Any, Dict, List, Optional, Tuple

_ARRAY = re.compile(r"^(f64|u32)\[(\d+)\]$")
SCHEMA_VERSION = 1
# Owner reasons, including `invalid`: a finite value outside the field's domain, rejected rather
# than clamped (ik_damping <= 0, cartesian_stiffness <= 0).
REASONS = ("type", "non_finite", "length", "unknown_field", "relation", "needs_confirm",
           "stale", "not_ready", "busy", "invalid")
# Bridge reasons, same envelope, never produced by an owner.
BRIDGE_REASONS = ("unreachable", "aborted", "bad_schema", "no_state", "stale_state")


class Rejection(Exception):
    def __init__(self, reason: str, field: Optional[str], error: str):
        super().__init__(error)
        self.reason, self.field, self.error = reason, field, error

    def as_reply(self, version: Optional[int] = None) -> Dict[str, Any]:
        r = {"ok": False, "reason": self.reason, "field": self.field, "error": self.error}
        if version is not None:
            r["version"] = version
        return r


def parse_type(t: str) -> Tuple[str, Optional[int]]:
    """'f64' -> ('f64', None); 'f64[7]' -> ('f64', 7); 'bool' -> ('bool', None)."""
    m = _ARRAY.match(t)
    if m:
        return m.group(1), int(m.group(2))
    if t in ("f64", "u32", "bool"):
        return t, None
    raise ValueError(f"unsupported schema type {t!r}")


def _check_scalar(field: str, kind: str, v: Any, idx: Optional[int] = None) -> Any:
    where = field if idx is None else f"{field}[{idx}]"
    if kind == "bool":
        if not isinstance(v, bool):
            raise Rejection("type", field, f"{where}: expected bool, got {type(v).__name__}")
        return v
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise Rejection("type", field, f"{where}: expected number, got {type(v).__name__}")
    if kind == "u32":
        if isinstance(v, float) and not v.is_integer():
            raise Rejection("type", field, f"{where}: expected integer, got {v}")
        if v < 0:
            raise Rejection("type", field, f"{where}: expected unsigned integer, got {v}")
        return int(v)
    v = float(v)
    if not math.isfinite(v):
        raise Rejection("non_finite", field, f"{where}: {v} is not finite")
    return v


def check_params(schema: Dict[str, Any], params: Dict[str, Any]) -> Dict[str, Any]:
    """Return a normalised copy of `params` or raise `Rejection`. Partial updates are fine."""
    if not isinstance(params, dict):
        raise Rejection("type", None, "params must be an object")
    spec = schema.get("params", {})
    out: Dict[str, Any] = {}
    for name, v in params.items():
        if name not in spec:
            raise Rejection("unknown_field", name, f"unknown field {name!r}")
        kind, n = parse_type(spec[name]["type"])
        if n is None:
            out[name] = _check_scalar(name, kind, v)
            continue
        if not isinstance(v, list):
            raise Rejection("type", name, f"{name}: expected array of {n}, got {type(v).__name__}")
        if len(v) != n:
            raise Rejection("length", name, f"{name}: expected {n} elements, got {len(v)}")
        out[name] = [_check_scalar(name, kind, x, i) for i, x in enumerate(v)]
    return out


def check_set_request(schema: Dict[str, Any], body: Dict[str, Any]) -> Dict[str, Any]:
    """Validate the envelope the panel sends and return the request to forward to the owner."""
    if not isinstance(body, dict):
        raise Rejection("type", None, "request must be an object")
    allowed = {"client_id", "base_version", "confirm", "params"}
    extra = set(body) - allowed
    if extra:
        raise Rejection("unknown_field", sorted(extra)[0], f"unknown request field {sorted(extra)[0]!r}")
    if "params" not in body:
        raise Rejection("type", None, "request has no params")
    req: Dict[str, Any] = {"params": check_params(schema, body["params"])}
    cid = body.get("client_id", 0)
    if isinstance(cid, bool) or not isinstance(cid, int) or cid <= 0:
        raise Rejection("type", "client_id", "client_id must be a positive integer")
    req["client_id"] = cid
    if "base_version" in body:
        bv = body["base_version"]
        if isinstance(bv, bool) or not isinstance(bv, int) or bv < 0:
            raise Rejection("type", "base_version", "base_version must be a non-negative integer")
        req["base_version"] = bv
    confirm = body.get("confirm", [])
    if not isinstance(confirm, list) or not all(isinstance(c, str) for c in confirm):
        raise Rejection("type", "confirm", "confirm must be a list of field names")
    req["confirm"] = confirm
    return req


def check_schema(schema: Dict[str, Any]) -> List[str]:
    """Structural problems in a published schema (empty list = usable)."""
    problems = []
    if schema.get("schema_version") != SCHEMA_VERSION:
        problems.append(f"schema_version {schema.get('schema_version')!r} != {SCHEMA_VERSION}")
    for k in ("owner", "arm", "boot_id", "params"):
        if k not in schema:
            problems.append(f"missing {k}")
    for name, p in (schema.get("params") or {}).items():
        try:
            kind, n = parse_type(p.get("type", ""))
        except ValueError as e:
            problems.append(f"{name}: {e}")
            continue
        if kind == "bool":
            continue
        for b in ("min", "max", "default"):
            if b not in p:
                problems.append(f"{name}: missing {b}")
            elif n is not None and (not isinstance(p[b], list) or len(p[b]) != n):
                problems.append(f"{name}: {b} must be a list of {n}")
        if p.get("scale") == "log":
            mins = p.get("min") if n is not None else [p.get("min")]
            if isinstance(mins, list) and any(isinstance(m, (int, float)) and m <= 0 for m in mins):
                problems.append(f"{name}: log scale needs min > 0")
    return problems


def crossings(schema: Dict[str, Any], params: Dict[str, Any], current: Dict[str, Any]) -> List[str]:
    """Fields whose new value crosses `confirm_above` from below (the owner wants `confirm`)."""
    out = []
    for name, v in params.items():
        p = schema["params"].get(name, {})
        if p.get("danger") != "confirm_above":
            continue
        thr = p["confirm_above"]
        cur = current.get(name)
        if isinstance(thr, list):
            crossed = any(x > t and (cur is None or cur[i] <= t) for i, (x, t) in enumerate(zip(v, thr)))
        else:
            crossed = v > thr and (cur is None or cur <= thr)
        if crossed:
            out.append(name)
    return out
