"""HTTP front of the tuning bridge: static files, the JSON routes, SSE. Owner refusals are
HTTP 200 with ok:false (the panel branches on `ok`); 422 only for a malformed request body, 413
for one over 64 KiB, 404 for an <arm> that is not a bounded [A-Za-z0-9_-] name, and 404
`unknown_arm` for a well-formed name no owner on the bus has published.

  GET  /  and  /static/<file>              the panel
  GET  /api/arms                           {arms: {L: {node: bool, teleop: bool}}}
  GET  /api/<arm>/<owner>/schema           owner's schema
  GET  /api/<arm>/<owner>/params           owner's get
  POST /api/<arm>/<owner>/params           owner's set (one owner)
  POST /api/<arm>/apply                    {node: setreq|null, teleop: setreq|null}
  GET  /api/<arm>/metrics                  metrics snapshot (what the SSE carries at 10 Hz)
  POST /api/<arm>/markers  {label}         drop a marker;  DELETE clears them
  GET  /api/<arm>/events                   SSE: hello, current, status, metrics
  GET  /api/<arm>/presets                  presets incl. built-ins toml and library-defaults
  POST /api/<arm>/presets  {name, note}    save the current effective sets + metrics snapshot
  DELETE /api/<arm>/presets/<name>
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import zenoh

from . import zbus
from .bridge import ArmMonitor, Bridge, SseClient

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
LOOPBACK_HOSTS = ("127.0.0.1", "localhost", "[::1]")
LOOPBACK_BINDS = ("127.0.0.1", "localhost", "::1")
EXPOSE_WARNING = ("binding %s: the bridge has no authentication, so anyone who can reach this port can "
                  "set every live parameter of every arm on the bus")


def resolve_bind(host: str, expose: bool) -> str:
    """The bridge is reached through an ssh tunnel; a bind beyond loopback hands the arm to the
    network, so it needs the explicit flag and is logged at startup."""
    if host in LOOPBACK_BINDS:
        return host
    if not expose:
        raise SystemExit(f"--host {host} is not loopback; the bridge has no authentication. "
                         "Use an ssh tunnel, or pass --expose-to-network if you mean it.")
    logging.getLogger("tuning-bridge").warning(EXPOSE_WARNING, host)
    return host
CTYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css"}
# An <arm> segment becomes part of a Zenoh key expression, where `*` and `**` reach every arm;
# its length is bounded because it is also a dictionary key the bridge would keep.
ARM_NAME = re.compile(r"[A-Za-z0-9_-]{1,64}")
MAX_BODY = 64 * 1024
NOT_FOUND = {"ok": False, "error": "not found"}
UNKNOWN_ARM = {"ok": False, "reason": "unknown_arm", "error": "no owner on the bus has this arm"}


class Handler(BaseHTTPRequestHandler):
    bridge: Bridge  # set by serve()
    protocol_version = "HTTP/1.1"
    # A socket read or write that stalls this long ends the connection, so a client that sends
    # its body slowly cannot hold a thread; above the SSE keepalive's 5 s.
    timeout = 30.0

    def log_message(self, fmt, *args):  # quiet
        pass

    def _json(self, code: int, body: Any, close: bool = False) -> None:
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        if close:
            self.send_header("Connection", "close")  # also ends the keep-alive loop
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> Optional[Dict[str, Any]]:
        """The request's JSON object, or None once a refusal is sent. A Content-Length that is
        missing, not a byte count or over MAX_BODY also closes the connection: nothing after the
        headers can be framed, or is worth reading."""
        n = self.headers.get("Content-Length", "")
        if not re.fullmatch(r"[0-9]+", n):
            self._json(422, {"ok": False, "reason": "type", "error": "Content-Length must be a byte count"},
                       close=True)
            return None
        if len(n) > 6 or int(n) > MAX_BODY:
            self._json(413, {"ok": False, "reason": "size", "error": f"body over {MAX_BODY} bytes"}, close=True)
            return None
        try:
            v = json.loads(self.rfile.read(int(n)) or b"{}")
        except (ValueError, RecursionError):
            v = None
        if not isinstance(v, dict):
            self._json(422, {"ok": False, "reason": "type", "error": "body must be a JSON object"})
            return None
        return v

    def _route(self) -> Optional[List[str]]:
        """The path's segments; None (a 404) for an /api/<arm>/... whose arm is not a name."""
        parts = [p for p in urlparse(self.path).path.split("/") if p]
        if len(parts) >= 3 and parts[0] == "api" and not ARM_NAME.fullmatch(parts[1]):
            return None
        return parts

    def do_GET(self) -> None:
        p = self._route()
        b = self.bridge
        if self._refused(write=False):
            return
        if p is None:
            return self._json(404, NOT_FOUND)
        if not p:
            return self._file("index.html")
        if p[0] == "static" and len(p) == 2:
            return self._file(p[1])
        if p[0] != "api":
            return self._json(404, NOT_FOUND)
        if p[1:] == ["arms"]:
            return self._json(200, {"ok": True, "arms": b.discover(), "linked": b.linked()})
        if len(p) == 4 and p[2] in zbus.OWNERS and p[3] == "schema":
            return self._json(200, b.schema(p[1], p[2]))
        if len(p) == 4 and p[2] in zbus.OWNERS and p[3] == "params":
            return self._json(200, b.owner_get(p[1], p[2], "get"))
        if len(p) == 3 and p[2] == "metrics":
            mon = self._mon(p[1])
            return self._json(200, mon.metrics_event()) if mon else None
        if len(p) == 3 and p[2] == "presets":
            return self._json(200, b.presets_for(p[1]))
        if len(p) == 3 and p[2] == "events":
            mon = self._mon(p[1])
            return self._sse(mon) if mon else None
        self._json(404, NOT_FOUND)

    def _mon(self, arm: str) -> Optional[ArmMonitor]:
        """The monitor of an arm the bus has presented, or a 404. A URL must not be able to make
        one: the bridge would then keep a ring and query its owners for an arm that never existed."""
        mon = self.bridge.known(arm)
        if mon is None:
            self._json(404, UNKNOWN_ARM)
        return mon

    def _foreign_host(self) -> bool:
        """DNS rebinding makes an attacker's page same-origin with whatever name it rebinds to
        127.0.0.1; only the loopback names are served at all. The port is not checked: an ssh
        tunnel may use any local port, and a rebound name is refused whatever port it carries."""
        host = self.headers.get("Host", "")
        name = host.rsplit(":", 1)[0] if not host.startswith("[") or "]:" in host else host
        return name not in LOOPBACK_HOSTS

    def _cross_origin(self) -> bool:
        """A form post from another origin is a 'simple' request with no preflight; the JSON
        content type forces one (which OPTIONS refuses) and Origin, when a browser sends it, must be
        this server exactly. No Origin (curl, tests) is allowed: browsers always send it cross-origin."""
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return True
        origin = self.headers.get("Origin")
        return origin is not None and origin != f"http://{self.headers.get('Host')}"

    def _refused(self, write: bool) -> bool:
        if self._foreign_host():
            self._json(403, {"ok": False, "reason": "host", "error": "served on loopback names only"})
            return True
        if write and self._cross_origin():
            self._json(403, {"ok": False, "reason": "origin", "error": "JSON from this origin only"})
            return True
        return False

    def do_POST(self) -> None:
        p = self._route()
        b = self.bridge
        if self._refused(write=True):
            return
        body = self._body()
        if body is None:
            return
        if p is None:
            return self._json(404, NOT_FOUND)
        if len(p) == 4 and p[0] == "api" and p[2] in zbus.OWNERS and p[3] == "params":
            return self._json(200, b.set_params(p[1], p[2], body))
        if len(p) == 3 and p[0] == "api" and p[2] == "apply":
            return self._json(200, b.apply(p[1], body))
        if len(p) == 3 and p[0] == "api" and p[2] == "markers":
            mon = self._mon(p[1])
            if mon is None:
                return
            m = mon.mark(str(body.get("label", "manual")))
            if m is None:
                return self._json(200, {"ok": False, "reason": "no_state", "field": None, "error": "no state yet"})
            return self._json(200, {"ok": True, "marker": m})
        if len(p) == 3 and p[0] == "api" and p[2] == "presets":
            return self._json(200, b.save_preset(p[1], body))
        self._json(404, NOT_FOUND)

    def do_DELETE(self) -> None:
        p = self._route()
        if self._refused(write=True):
            return
        if p is None:
            return self._json(404, NOT_FOUND)
        if len(p) == 3 and p[0] == "api" and p[2] == "markers":
            mon = self._mon(p[1])
            if mon is None:
                return
            mon.metrics.clear_markers()
            return self._json(200, {"ok": True})
        if len(p) == 4 and p[0] == "api" and p[2] == "presets":
            return self._json(200, {"ok": self.bridge.presets.remove(p[3], p[1])})
        self._json(404, NOT_FOUND)

    def _file(self, name: str) -> None:
        path = os.path.join(STATIC, os.path.basename(name))
        if not os.path.isfile(path):
            return self._json(404, NOT_FOUND)
        with open(path, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", CTYPES.get(os.path.splitext(name)[1], "application/octet-stream"))
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _sse(self, mon: ArmMonitor) -> None:
        client = SseClient()
        with mon.lock:
            mon.clients.append(client)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        hello = mon.hello()  # taken under the monitor's lock; serialised here, outside it
        try:
            self.wfile.write(f"event: hello\ndata: {json.dumps(hello)}\n\n".encode())
            self.wfile.flush()
            while not client.closed.is_set():
                try:
                    msg = client.queue.get(timeout=5.0)
                except queue.Empty:
                    msg = b": keepalive\n\n"
                self.wfile.write(msg)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with mon.lock:
                if client in mon.clients:
                    mon.clients.remove(client)


def open_session(a: argparse.Namespace) -> zenoh.Session:
    """A zenoh client refuses to open when nothing answers, and 'the node is not up yet' is the
    ordinary case someone opens the panel to look into. Say what to do, not a Rust file path."""
    config = zbus.config_from_args(a)
    logging.getLogger("tuning-bridge").info("%s", zbus.describe(a))
    try:
        return zenoh.open(config)
    except zenoh.ZError as e:
        reason = str(e).split(" at /")[0].strip()  # zenoh appends its own source path
        raise SystemExit(f"no zenoh session ({zbus.describe(a)}): {reason}\nStart the node first, point "
                         "--connect at it, or pass --mode peer to hold a session open until it appears.")


def serve(bridge: Bridge, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"bridge": bridge})
    srv = ThreadingHTTPServer((host, port), handler)
    srv.daemon_threads = True
    return srv


def main() -> None:
    ap = argparse.ArgumentParser(description="The tuning panel: a running franka-node's live parameters "
                                             "in the browser, served on loopback.")
    ap.add_argument("--host", default="127.0.0.1", help="bind address; loopback unless --expose-to-network")
    ap.add_argument("--expose-to-network", action="store_true",
                    help="allow a non-loopback --host: every live parameter of every arm becomes settable by "
                         "anyone who can reach the port, with no authentication")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--presets", default=os.path.expanduser("~/.local/state/franka-tuning/presets.json"))
    ap.add_argument("--timeout", type=float, default=1.0, help="zenoh query timeout, s")
    zbus.add_zenoh_args(ap)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    host = resolve_bind(a.host, a.expose_to_network)
    session = open_session(a)
    bridge = Bridge(session, a.presets, a.timeout)
    srv = serve(bridge, host, a.port)
    print(f"tuning bridge on http://{host}:{a.port}/  presets {a.presets}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        session.close()


if __name__ == "__main__":
    main()
