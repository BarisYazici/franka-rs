#!/usr/bin/env python
"""HTTP front of the tuning bridge: static files, the JSON routes, SSE. Owner refusals are
HTTP 200 with ok:false (the panel branches on `ok`); 422 only for a malformed request body.

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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional
from urllib.parse import urlparse

import zenoh

import zbus
from bridge import ArmMonitor, Bridge, SseClient

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


class Handler(BaseHTTPRequestHandler):
    bridge: Bridge  # set by serve()
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # quiet
        pass

    def _json(self, code: int, body: Any) -> None:
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> Optional[Dict[str, Any]]:
        n = int(self.headers.get("Content-Length") or 0)
        try:
            v = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return None
        return v if isinstance(v, dict) else None

    def _route(self):
        parts = [p for p in urlparse(self.path).path.split("/") if p]
        return parts

    def do_GET(self) -> None:
        p = self._route()
        b = self.bridge
        if self._refused(write=False):
            return
        if not p:
            return self._file("index.html")
        if p[0] == "static" and len(p) == 2:
            return self._file(p[1])
        if p[0] != "api":
            return self._json(404, {"ok": False, "error": "not found"})
        if p[1:] == ["arms"]:
            return self._json(200, {"ok": True, "arms": b.discover()})
        if len(p) == 4 and p[2] in zbus.OWNERS and p[3] == "schema":
            return self._json(200, b.schema(p[1], p[2]))
        if len(p) == 4 and p[2] in zbus.OWNERS and p[3] == "params":
            return self._json(200, b.owner_get(p[1], p[2], "get"))
        if len(p) == 3 and p[2] == "metrics":
            return self._json(200, b.arm(p[1]).metrics_event())
        if len(p) == 3 and p[2] == "presets":
            return self._json(200, b.presets_for(p[1]))
        if len(p) == 3 and p[2] == "events":
            return self._sse(b.arm(p[1]))
        self._json(404, {"ok": False, "error": "not found"})

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
            return self._json(422, {"ok": False, "reason": "type", "error": "body must be a JSON object"})
        if len(p) == 4 and p[0] == "api" and p[2] in zbus.OWNERS and p[3] == "params":
            return self._json(200, b.set_params(p[1], p[2], body))
        if len(p) == 3 and p[0] == "api" and p[2] == "apply":
            return self._json(200, b.apply(p[1], body))
        if len(p) == 3 and p[0] == "api" and p[2] == "markers":
            m = b.arm(p[1]).mark(str(body.get("label", "manual")))
            if m is None:
                return self._json(200, {"ok": False, "reason": "no_state", "field": None, "error": "no state yet"})
            return self._json(200, {"ok": True, "marker": m})
        if len(p) == 3 and p[0] == "api" and p[2] == "presets":
            return self._json(200, b.save_preset(p[1], body))
        self._json(404, {"ok": False, "error": "not found"})

    def do_DELETE(self) -> None:
        p = self._route()
        if self._refused(write=True):
            return
        if len(p) == 3 and p[0] == "api" and p[2] == "markers":
            self.bridge.arm(p[1]).metrics.clear_markers()
            return self._json(200, {"ok": True})
        if len(p) == 4 and p[0] == "api" and p[2] == "presets":
            return self._json(200, {"ok": self.bridge.presets.remove(p[3], p[1])})
        self._json(404, {"ok": False, "error": "not found"})

    def _file(self, name: str) -> None:
        path = os.path.join(STATIC, os.path.basename(name))
        if not os.path.isfile(path):
            return self._json(404, {"ok": False, "error": "not found"})
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
        hello = {"arm": mon.arm, "reach": mon.reach(), "current": mon.current, "status": mon.status_event(),
                 "channels": mon.channels()}
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


def serve(bridge: Bridge, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"bridge": bridge})
    srv = ThreadingHTTPServer((host, port), handler)
    srv.daemon_threads = True
    return srv


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
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
    session = zenoh.open(zbus.config_from_args(a))
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
