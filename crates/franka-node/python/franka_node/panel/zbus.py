"""Zenoh glue shared by the bridge and the mock: config from CLI flags, JSON on the wire."""

from __future__ import annotations

import argparse
import json
from typing import Any, Dict, Iterator, List, Optional

import zenoh

NODE = "node"
TELEOP = "teleop"
OWNERS = (NODE, TELEOP)


def add_zenoh_args(p: argparse.ArgumentParser, mode: str = "client") -> None:
    """The node's own vocabulary (`[zenoh] mode`). A pure consumer defaults to `client`: it dials
    out and is never dialled, which is what works behind a tunnel, and it still scouts by
    multicast, so a plain LAN needs no flags either."""
    p.add_argument("--mode", choices=("peer", "client"), default=mode,
                   help=f"zenoh session mode (default {mode}); a client dials out and does not listen")
    p.add_argument("--connect", action="append", default=[], metavar="ENDPOINT",
                   help="zenoh endpoint to connect to, e.g. tcp/127.0.0.1:7447 (repeatable)")
    p.add_argument("--listen", action="append", default=[], metavar="ENDPOINT",
                   help="zenoh endpoint to listen on, peer mode only (repeatable)")
    p.add_argument("--zenoh-config", metavar="FILE", help="zenoh json5 config file (overrides the flags)")
    p.add_argument("--no-multicast", action="store_true", help="disable multicast scouting")


def _endpoints(c: zenoh.Config, key: str, mode: str) -> List[str]:
    """`connect/endpoints` and `listen/endpoints` are a list, or zenoh's per-mode defaults."""
    v = json.loads(c.get_json(key))
    return (v.get(mode, []) if isinstance(v, dict) else v) or []


def describe(c: zenoh.Config) -> str:
    """The startup line, read back off the resolved config: flags are easy to mean differently,
    and a `--zenoh-config` file that picks another mode or other endpoints must show as what it is,
    not as a filename."""
    mode = json.loads(c.get_json("mode")) or "peer"  # unset: zenoh's own default
    dial = ", ".join(_endpoints(c, "connect/endpoints", mode))
    if not dial:
        dial = "nothing" if json.loads(c.get_json("scouting/multicast/enabled")) is False \
            else "multicast scouting"
    listen = _endpoints(c, "listen/endpoints", mode) if mode != "client" else []
    return f"zenoh: {mode}, dialling {dial}" + (f", listening on {', '.join(listen)}" if listen else "")


def config_from_args(a: argparse.Namespace) -> zenoh.Config:
    if a.zenoh_config:
        return zenoh.Config.from_file(a.zenoh_config)
    if a.mode == "client" and a.listen:
        raise SystemExit("--listen needs --mode peer: a zenoh client dials out and does not listen")
    c = zenoh.Config()
    c.insert_json5("mode", json.dumps(a.mode))
    if a.connect:
        c.insert_json5("connect/endpoints", json.dumps(a.connect))
    if a.listen:
        c.insert_json5("listen/endpoints", json.dumps(a.listen))
    if a.no_multicast:
        c.insert_json5("scouting/multicast/enabled", "false")
    return c


def params_prefix(arm: str, owner: str) -> str:
    return f"franka/{arm}/params" if owner == NODE else f"franka/{arm}/teleop/params"


def owner_of(key: str) -> Optional[str]:
    """`franka/<arm>/params/x` -> node, `franka/<arm>/teleop/params/x` -> teleop."""
    parts = key.split("/")
    if len(parts) >= 4 and parts[2] == "params":
        return NODE
    if len(parts) >= 5 and parts[2] == TELEOP and parts[3] == "params":
        return TELEOP
    return None


def arm_of(key: str) -> str:
    return key.split("/")[1]


def dumps(obj: Any) -> bytes:
    return json.dumps(obj, separators=(",", ":"), allow_nan=False).encode()


def loads(payload) -> Any:
    return json.loads(payload.to_bytes())


def query_json(session: zenoh.Session, key: str, body: Optional[Dict] = None, timeout: float = 1.0
               ) -> Iterator[tuple[str, Any]]:
    """`z_get` yielding (key, decoded JSON) for every OK reply; errors become {'ok':False}."""
    replies = session.get(key, payload=None if body is None else dumps(body), timeout=timeout)
    for r in replies:
        if r.ok is not None:
            yield str(r.ok.key_expr), loads(r.ok.payload)
        elif r.err is not None:
            yield key, {"ok": False, "reason": "busy", "error": r.err.payload.to_string()}


def query_one(session: zenoh.Session, key: str, body: Optional[Dict] = None, timeout: float = 1.0
              ) -> Optional[Any]:
    for _, v in query_json(session, key, body, timeout):
        return v
    return None
