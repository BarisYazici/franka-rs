"""HTTP front of the tuning bridge: static files, the JSON routes, SSE. Owner refusals are
HTTP 200 with ok:false (the panel branches on `ok`); 422 only for a malformed request body, 413
for one over 64 KiB, 404 for an <arm> that is not a bounded [A-Za-z0-9_-] name, and 404
`unknown_arm` for a well-formed name no owner on the bus has published. 403 `host` for a Host the
bind does not serve (see `HostPolicy`), 403 `origin` for a write from another origin, and 400 for
a request this server will not frame (no Host or several, an absolute-form target, a chunked body,
a body on a GET or DELETE). Every refusal closes the connection: an unread body would frame the
next request.

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
import ipaddress
import json
import logging
import os
import queue
import re
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Iterable, List, Optional, Union
from urllib.parse import urlparse

import zenoh

from . import zbus
from .bridge import ArmMonitor, Bridge, SseClient

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
LOOPBACK_HOSTS = ("127.0.0.1", "localhost", "[::1]")
LOOPBACK_BINDS = ("127.0.0.1", "localhost", "::1")
ANY_BINDS = ("0.0.0.0", "::")
EXPOSE_WARNING = ("the panel is at %s, reachable by anyone on that network: it has no authentication, so "
                  "any of them can set every live parameter of every arm on the bus, while it moves")


def host_label(header: str) -> str:
    """A Host header without its port: '[::1]:8765' -> '[::1]', '192.0.2.5:8765' -> '192.0.2.5'. A
    bare IPv6 address is kept whole: RFC 7230 wants it bracketed, and a second colon says it
    cannot be carrying a port."""
    if header.startswith("["):
        return header.rsplit(":", 1)[0] if "]:" in header else header
    return header.rsplit(":", 1)[0] if header.count(":") == 1 else header


def ip_literal(label: str) -> Optional[Union[ipaddress.IPv4Address, ipaddress.IPv6Address]]:
    """The address a Host label *is*, or None when it is a name. A name that merely contains an
    address ('192.0.2.5.evil.example') is a name; the parse decides, never a pattern."""
    text = label[1:-1] if label.startswith("[") and label.endswith("]") else label
    try:
        return ipaddress.ip_address(text)
    except ValueError:
        return None


class HostPolicy:
    """Which Host headers are served.

    Loopback-bound that is the loopback names alone: a page at any other name reaches this port
    only by rebinding its own DNS name to 127.0.0.1, and the browser would then count it
    same-origin with the panel. Exposed, IP literals are served too, because a rebound name is
    never a literal -- that property, not which address it holds, is what keeps the defence. Any
    literal is accepted, not only this host's own addresses: the Host a browser sends is the
    address the operator typed, which behind NAT, a port forward or a container is not an address
    this process can see, while filtering to local addresses would stop no attacker (a page that
    aims at the panel's real address is refused on Origin for writes and cannot read the reply of
    a GET). Names beyond loopback are served only where the operator wrote them out, exactly,
    with --allowed-host; a suffix match would be a wildcard for anyone owning the parent domain,
    and no name at all is served unless the panel is exposed: a name is only as trustworthy as
    whoever can answer for it, and on a loopback bind a name anyone on the LAN can claim (mDNS
    answers for `.local` are unauthenticated) would be the rebinding attack through the door.
    """

    def __init__(self, expose: bool = False, allowed: Iterable[str] = ()) -> None:
        self.expose = expose
        self.allowed = frozenset(n.strip().lower() for n in allowed if n.strip())

    def accepts(self, header: str) -> bool:
        label = host_label(header).lower()
        if label in LOOPBACK_HOSTS:
            return True
        if not self.expose:
            return False
        return label in self.allowed or ip_literal(label) is not None

    def refusal(self) -> str:
        served = ["loopback names"]
        if self.expose:
            served.append("IP addresses")
            if self.allowed:
                served.append("the --allowed-host names")
        return "served on " + ", ".join(served) + " only"


def host_policy(bind: str, allowed: Iterable[str] = ()) -> HostPolicy:
    """The policy a resolved bind implies. Only a bind beyond loopback serves anything but the
    loopback names, and `resolve_bind` has already refused such a bind without
    --expose-to-network, so the bind is the flag: passing the flag while staying on loopback
    exposes nothing and loosens nothing."""
    expose = bind not in LOOPBACK_BINDS
    if allowed and not expose:
        raise SystemExit("--allowed-host needs --expose-to-network and a --host beyond loopback. On a "
                         "loopback bind the loopback names are the only ones served, and a name anyone "
                         "on the network can answer for (mDNS, .local) would be a way in.")
    return HostPolicy(expose, allowed)


def local_addresses() -> List[str]:
    """This host's own addresses, for the startup line only: the one a packet to the network
    would leave by, plus whatever the hostname resolves to. Best effort, stdlib only, and it
    gates nothing -- an address missing here still serves, it is just not printed."""
    found: List[str] = []
    for family, probe in ((socket.AF_INET, ("192.0.2.1", 9)), (socket.AF_INET6, ("2001:db8::1", 9))):
        try:
            with socket.socket(family, socket.SOCK_DGRAM) as s:  # connect(2) on UDP sends nothing
                s.connect(probe)
                found.append(s.getsockname()[0])
        except OSError:
            pass
    try:
        found += [info[4][0] for info in socket.getaddrinfo(socket.gethostname(), None,
                                                            type=socket.SOCK_STREAM)]
    except OSError:
        pass
    out: List[str] = []
    for addr in found:
        addr = addr.split("%")[0]  # a scope id names an interface, not something to print
        ip = ip_literal(addr)
        if ip is not None and not ip.is_loopback and addr not in out:
            out.append(addr)
    return out


def reachable_urls(host: str, port: int) -> List[str]:
    """The URL(s) this bind answers on, for the startup line."""
    addrs = local_addresses() if host in ANY_BINDS else [host]
    return [f"http://{'[%s]' % a if ':' in a else a}:{port}/" for a in addrs or [host]]


def resolve_bind(host: str, expose: bool, port: int = 8765) -> str:
    """The bridge is reached on loopback, through an ssh tunnel or by an operator who said
    --expose-to-network; a bind beyond loopback hands the arm to the network, so it needs that
    explicit flag and the startup log names the URLs it just opened."""
    if host in LOOPBACK_BINDS:
        return host
    if not expose:
        raise SystemExit(f"--host {host} is not loopback; the bridge has no authentication. "
                         "Use an ssh tunnel, or pass --expose-to-network if you mean it.")
    logging.getLogger("tuning-bridge").warning(EXPOSE_WARNING, " ".join(reachable_urls(host, port)))
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
    hosts = HostPolicy()  # loopback-only unless serve() is given another
    protocol_version = "HTTP/1.1"
    # A socket read or write that stalls this long ends the connection, so a client that sends
    # its body slowly cannot hold a thread; above the SSE keepalive's 5 s.
    timeout = 30.0

    def log_message(self, fmt, *args):  # quiet
        pass

    def end_headers(self) -> None:
        """On every response. The same-origin policy does not cover framing, and a framed panel
        drives a moving arm on a click the operator meant for someone else's page."""
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Content-Security-Policy", "frame-ancestors 'none'")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()

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
        if self._refused(write=False) or self._unread_body():
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
        127.0.0.1, so what is served is what `HostPolicy` allows. The port is not checked: an ssh
        tunnel may use any local port, and a rebound name is refused whatever port it carries."""
        return not self.hosts.accepts(self.headers.get("Host", ""))

    def _cross_origin(self) -> bool:
        """A form post from another origin is a 'simple' request with no preflight; the JSON
        content type forces one (which OPTIONS refuses) and Origin, when a browser sends it, must be
        this server exactly -- the Host the request already passed, so an exposed panel's own IP
        works and nothing else does. No Origin (curl, tests) is allowed: browsers always send it
        cross-origin."""
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return True
        origin = self.headers.get("Origin")
        return origin is not None and origin.lower() != f"http://{self.headers.get('Host', '')}".lower()

    def _malformed(self) -> bool:
        """A request this server will not try to read a meaning into: RFC 9112 wants exactly one
        Host header, an absolute-form target ('POST http://elsewhere/api/...') carries an authority
        of its own that outranks the Host the guard below would check, and a chunked body is one
        nothing here decodes -- its bytes would stay in the socket and frame the next request."""
        if self.headers.get("Transfer-Encoding"):
            self._json(400, {"ok": False, "reason": "body", "error": "no chunked request body"}, close=True)
            return True
        if len(self.headers.get_all("Host") or []) != 1:
            self._json(400, {"ok": False, "reason": "host", "error": "exactly one Host header"}, close=True)
            return True
        target = urlparse(self.path)
        if target.scheme or target.netloc:
            self._json(400, {"ok": False, "reason": "target", "error": "origin-form target only"}, close=True)
            return True
        return False

    def _unread_body(self) -> bool:
        """A GET or DELETE that declared a body by length. Nothing here reads one, so those bytes
        would be taken for the next request on this connection; refuse and close, as every other
        unread body does. A chunked one is already gone, refused by `_malformed`."""
        n = (self.headers.get("Content-Length") or "").strip()
        if not n or (n.isdigit() and int(n) == 0):
            return False
        self._json(400, {"ok": False, "reason": "body", "error": "no body on this method"}, close=True)
        return True

    def _refused(self, write: bool) -> bool:
        """Every refusal closes the connection. The request's body is still in the socket -- a
        refusal is sent before `_body()` reads it -- and on a kept-alive connection the handler
        would take those bytes for the next request, which is a cross-origin page's way to send
        one this server would then accept on its own terms."""
        if self._malformed():
            return True
        if self._foreign_host():
            self._json(403, {"ok": False, "reason": "host", "error": self.hosts.refusal()}, close=True)
            return True
        if write and self._cross_origin():
            self._json(403, {"ok": False, "reason": "origin", "error": "JSON from this origin only"},
                       close=True)
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
        if self._refused(write=True) or self._unread_body():
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
    resolved = zbus.describe(config) + (f", from {a.zenoh_config}" if a.zenoh_config else "")
    logging.getLogger("tuning-bridge").info("%s", resolved)
    try:
        return zenoh.open(config)
    except zenoh.ZError as e:
        reason = str(e).split(" at /")[0].strip()  # zenoh appends its own source path
        raise SystemExit(f"no zenoh session ({resolved}): {reason}\nStart the node first, point "
                         "--connect at it, or pass --mode peer to hold a session open until it appears.")


def serve(bridge: Bridge, host: str, port: int, hosts: Optional[HostPolicy] = None) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"bridge": bridge, "hosts": hosts or HostPolicy()})
    srv = ThreadingHTTPServer((host, port), handler)
    srv.daemon_threads = True
    return srv


def main() -> None:
    ap = argparse.ArgumentParser(description="The tuning panel: a running franka-node's live parameters "
                                             "in the browser, served on loopback unless "
                                             "--expose-to-network says otherwise.")
    ap.add_argument("--host", default="127.0.0.1", help="bind address; loopback unless --expose-to-network")
    ap.add_argument("--expose-to-network", action="store_true",
                    help="allow a non-loopback --host, and serve requests whose Host is an IP address: "
                         "every live parameter of every arm becomes settable by anyone who can reach the "
                         "port, with no authentication")
    ap.add_argument("--allowed-host", action="append", default=[], metavar="NAME",
                    help="with --expose-to-network, also serve requests whose Host is exactly NAME, e.g. "
                         "panel.example.com (repeatable); exact match, never a suffix, and only as "
                         "trustworthy as whoever can answer for that name")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--presets", default=os.path.expanduser("~/.local/state/franka-tuning/presets.json"))
    ap.add_argument("--timeout", type=float, default=1.0, help="zenoh query timeout, s")
    zbus.add_zenoh_args(ap)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    host = resolve_bind(a.host, a.expose_to_network, a.port)
    hosts = host_policy(host, a.allowed_host)
    session = open_session(a)
    bridge = Bridge(session, a.presets, a.timeout)
    srv = serve(bridge, host, a.port, hosts)
    urls = " ".join(reachable_urls(host, a.port))
    who = ("anyone on that network, with no authentication" if hosts.expose
           else "this machine, or an ssh tunnel to it")
    names = f"  also serving Host {' '.join(sorted(hosts.allowed))}" if hosts.allowed else ""
    print(f"tuning bridge on {urls} ({who}){names}  presets {a.presets}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        session.close()


if __name__ == "__main__":
    main()
