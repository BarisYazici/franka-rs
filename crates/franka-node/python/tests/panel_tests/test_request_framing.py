"""What the front does with a request it refuses, and with one it will not frame.

A refusal is sent before the body is read, so the body is still in the socket. On a kept-alive
connection the handler takes those bytes for the next request -- and a cross-origin page can put
a whole request in the body of a `text/plain` POST, which needs no preflight. That request
carries a Host this server serves and no Origin, which the origin check allows by design, so
both guards are passed by a page that was refused. Every refusal therefore closes the
connection, as the malformed-Content-Length paths already did.

Raw sockets here: the cases (a smuggled second request, two Host headers, an absolute-form
target) are ones no HTTP client library will send.
"""

import socket

import pytest

from franka_node.panel.web import HostPolicy

from .conftest import build_stack

CRLF = "\r\n"


@pytest.fixture(scope="module")
def exposed_stack(mock, tmp_path_factory):
    presets = str(tmp_path_factory.mktemp("framing") / "presets.json")
    http, bridge, stop = build_stack(mock, presets, HostPolicy(expose=True))
    yield http, bridge
    stop()


def addr_of(http):
    host, port = http.base.rsplit("/", 1)[1].rsplit(":", 1)
    return host, int(port)


def exchange(http, raw: bytes, wait: float = 1.0) -> bytes:
    """Everything the server says on one connection, until it closes or falls silent."""
    with socket.create_connection(addr_of(http), timeout=5) as s:
        s.sendall(raw)
        s.settimeout(wait)
        out = b""
        while True:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                return out  # still open and waiting for more: the connection was kept alive
            if not chunk:
                return out
            out += chunk


def responses(raw: bytes) -> int:
    return raw.count(b"HTTP/1.1 ")


def request(method: str, path: str, headers, body: str = "") -> str:
    head = f"{method} {path} HTTP/1.1{CRLF}" + "".join(f"{k}: {v}{CRLF}" for k, v in headers)
    return head + CRLF + body


SMUGGLED = '{"label":"smuggled"}'


def smuggle(http, outer_headers, inner_host: str) -> bytes:
    """A POST the front refuses, whose body is a second request the front would accept: the Host
    is one this bind serves and there is no Origin, which the origin check allows by design."""
    inner = request("POST", "/api/L/markers",
                    [("Host", inner_host), ("Content-Type", "application/json"),
                     ("Content-Length", str(len(SMUGGLED)))], SMUGGLED)
    outer = request("POST", "/api/L/markers", list(outer_headers) + [("Content-Length", str(len(inner)))],
                    inner)
    return exchange(http, outer.encode())


def test_a_refused_origin_does_not_carry_the_next_request(stack):
    """A page at evil.example posts text/plain (no preflight) to the documented loopback URL; the
    outer request is 403 origin and its body must not become a request of its own."""
    http, _ = stack
    port = addr_of(http)[1]
    out = smuggle(http, [("Host", f"127.0.0.1:{port}"), ("Origin", "http://evil.example"),
                         ("Content-Type", "text/plain")], f"127.0.0.1:{port}")
    assert b"403" in out and responses(out) == 1, out


def test_a_refused_origin_does_not_carry_the_next_request_when_exposed(exposed_stack):
    http, _ = exposed_stack
    out = smuggle(http, [("Host", "192.0.2.5:8765"), ("Origin", "http://evil.example"),
                         ("Content-Type", "text/plain")], "192.0.2.5:8765")
    assert b"403" in out and responses(out) == 1, out


def test_a_refused_host_does_not_carry_the_next_request(stack):
    """The rebound name is refused, and the request hidden in its body, aimed at the loopback
    name, must not be served either."""
    http, _ = stack
    port = addr_of(http)[1]
    out = smuggle(http, [("Host", "evil.example:8765"), ("Origin", "http://evil.example:8765"),
                         ("Content-Type", "application/json")], f"127.0.0.1:{port}")
    assert b"403" in out and responses(out) == 1, out


def test_the_body_that_was_refused_changed_nothing(stack):
    """The smuggled request asks for a marker, which the arm's monitor would keep. Self-contained:
    it clears the markers, smuggles, and looks -- so it says the same thing run alone."""
    http, _ = stack
    port = addr_of(http)[1]
    same_origin = {"Origin": f"http://127.0.0.1:{port}"}
    assert http.delete("/api/L/markers", headers=same_origin)["ok"]
    smuggle(http, [("Host", f"127.0.0.1:{port}"), ("Origin", "http://evil.example"),
                   ("Content-Type", "text/plain")], f"127.0.0.1:{port}")
    labels = [m["label"] for m in http.get("/api/L/metrics").get("markers", [])]
    assert labels == [], labels
    http.delete("/api/L/markers", headers=same_origin)


def test_more_than_one_host_header_is_refused(stack):
    """RFC 9112 wants exactly one. The guard reads the first, an intermediary may read the last;
    no browser sends two, so refusing costs nothing and removes the disagreement."""
    http, _ = stack
    port = addr_of(http)[1]
    for hosts in ((f"127.0.0.1:{port}", "evil.example"), ("evil.example", f"127.0.0.1:{port}")):
        out = exchange(http, request("GET", "/api/arms", [("Host", h) for h in hosts]).encode())
        assert out.startswith(b"HTTP/1.1 400") and responses(out) == 1, out


def test_no_host_header_is_refused(stack):
    http, _ = stack
    out = exchange(http, request("GET", "/api/arms", []).encode())
    assert out.startswith(b"HTTP/1.1 400"), out


def test_an_absolute_form_target_is_refused(stack):
    """`GET http://evil.example/api/arms` carries an authority that outranks the Host header the
    guard checks; the front will not decide which of the two the request meant."""
    http, _ = stack
    port = addr_of(http)[1]
    out = exchange(http, request("GET", "http://evil.example/api/arms",
                                 [("Host", f"127.0.0.1:{port}")]).encode())
    assert out.startswith(b"HTTP/1.1 400"), out
    # a target starting '//' never reaches the guard: http.server collapses it to one slash
    # (its open-redirect defence), so the authority is gone and the path is simply not a route
    out = exchange(http, request("GET", "//evil.example/api/arms",
                                 [("Host", f"127.0.0.1:{port}")]).encode())
    assert out.startswith(b"HTTP/1.1 404"), out


@pytest.mark.parametrize("method", ["GET", "DELETE"])
@pytest.mark.parametrize("declare", ["length", "chunked"])
def test_a_body_on_a_get_or_delete_is_refused(stack, method, declare):
    """Neither verb reads a body, so a declared one would frame the next request -- whether it is
    declared by length or by chunking, which this server decodes nowhere. Not reachable from a
    browser (fetch gives no GET a body, a cross-origin DELETE needs a preflight), but it is the
    same desync."""
    http, _ = stack
    port = addr_of(http)[1]
    inner = request("POST", "/api/L/markers",
                    [("Host", f"127.0.0.1:{port}"), ("Content-Type", "application/json"),
                     ("Content-Length", str(len(SMUGGLED)))], SMUGGLED)
    declared = [("Content-Length", str(len(inner)))] if declare == "length" else \
               [("Transfer-Encoding", "chunked")]
    out = exchange(http, request(method, "/api/arms",
                                 [("Host", f"127.0.0.1:{port}"),
                                  ("Content-Type", "application/json")] + declared, inner).encode())
    assert out.startswith(b"HTTP/1.1 400") and responses(out) == 1, out
    assert "smuggled" not in [m["label"] for m in http.get("/api/L/metrics").get("markers", [])]


def test_a_chunked_write_is_refused_too(stack):
    """A POST declaring chunking has no Content-Length, so `_body` refused it already; refusing it
    up front keeps one answer for 'this server frames no chunked request'."""
    http, _ = stack
    port = addr_of(http)[1]
    out = exchange(http, request("POST", "/api/L/markers",
                                 [("Host", f"127.0.0.1:{port}"), ("Content-Type", "application/json"),
                                  ("Origin", f"http://127.0.0.1:{port}"),
                                  ("Transfer-Encoding", "chunked")], SMUGGLED).encode())
    assert out.startswith(b"HTTP/1.1 400") and responses(out) == 1, out


def test_an_empty_body_length_is_still_served(stack):
    """`Content-Length: 0` is what a client that always sets the header sends; it frames nothing
    and must not be refused."""
    http, _ = stack
    port = addr_of(http)[1]
    out = exchange(http, request("GET", "/api/arms",
                                 [("Host", f"127.0.0.1:{port}"), ("Content-Length", "0")]).encode())
    assert out.startswith(b"HTTP/1.1 200"), out


@pytest.mark.parametrize("path", ["/", "/static/api.js", "/api/arms"])
def test_every_response_refuses_to_be_framed(stack, path):
    """The same-origin policy says nothing about framing, and a click on a framed panel retunes a
    moving arm."""
    http, _ = stack
    port = addr_of(http)[1]
    out = exchange(http, request("GET", path, [("Host", f"127.0.0.1:{port}")]).encode()).lower()
    assert out.startswith(b"http/1.1 200"), out
    assert b"x-frame-options: deny" in out
    assert b"content-security-policy: frame-ancestors \'none\'" in out
    assert b"x-content-type-options: nosniff" in out


def test_a_refusal_is_not_framed_either(stack):
    http, _ = stack
    out = exchange(http, request("GET", "/api/arms", [("Host", "evil.example")]).encode()).lower()
    assert out.startswith(b"http/1.1 403") and b"x-frame-options: deny" in out
