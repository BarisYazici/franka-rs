"""What the HTTP front refuses before anything reaches Zenoh: an `<arm>` segment that is not a
name (it becomes part of a key expression, where `*` and `**` reach every arm) and a
`Content-Length` it cannot frame."""

import socket
import time

import pytest

from franka_node.panel import web

BAD_ARMS = ("*", "**", "L*", "$*", "a%2Fb", "L.R")


def _spy(bridge, monkeypatch):
    """Every arm name that reaches the bridge's Zenoh side."""
    seen = []
    owner_get, arm = bridge.owner_get, bridge.arm
    monkeypatch.setattr(bridge, "owner_get", lambda a, *r, **k: seen.append(a) or owner_get(a, *r, **k))
    monkeypatch.setattr(bridge, "arm", lambda a: seen.append(a) or arm(a))
    return seen


@pytest.mark.parametrize("bad", BAD_ARMS)
def test_an_arm_that_is_not_a_name_is_404_and_reaches_nothing(stack, monkeypatch, bad):
    http, bridge = stack
    seen = _spy(bridge, monkeypatch)
    for path in ("node/schema", "node/params", "teleop/schema", "teleop/params", "metrics",
                 "presets", "events"):
        status, _ = http._call("GET", f"/api/{bad}/{path}")
        assert status == 404, (bad, path, status)
    for path, body in (("node/params", {"client_id": 1, "set": {}}),
                       ("teleop/params", {"client_id": 1, "set": {}}),
                       ("apply", {"node": {"client_id": 1, "set": {}}}),
                       ("markers", {"label": "x"}), ("presets", {"name": "p"})):
        status, _ = http.post(f"/api/{bad}/{path}", body)
        assert status == 404, (bad, path, status)
    for path in ("markers", "presets/p"):
        status, _ = http._call("DELETE", f"/api/{bad}/{path}")
        assert status == 404, (bad, path, status)
    assert bad not in seen, seen


def test_arm_names_with_dash_and_underscore_still_route(stack, monkeypatch):
    http, bridge = stack
    seen = _spy(bridge, monkeypatch)
    for arm in ("L", "left_arm-2"):
        status, _ = http._call("GET", f"/api/{arm}/metrics")
        assert status == 200
    assert "left_arm-2" in seen


def _post_head(base, length_header):
    """(host, port) and the head of a POST to /api/L/markers carrying `length_header`."""
    host, port = base.rsplit("/", 1)[1].rsplit(":", 1)
    head = (f"POST /api/L/markers HTTP/1.1\r\nHost: {host}:{port}\r\n"
            f"Content-Type: application/json\r\n{length_header}\r\n")
    return (host, int(port)), head.encode()


def _raw_post(base, length_header, body=b""):
    """One POST on a raw socket; (status, headers text, whether the server then closed)."""
    addr, head = _post_head(base, length_header)
    with socket.create_connection(addr, timeout=5) as s:
        s.sendall(head + body)
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = s.recv(4096)
            assert chunk, f"closed without a response: {data!r}"
            data += chunk
        head, rest = data.split(b"\r\n\r\n", 1)
        length = int([h for h in head.split(b"\r\n") if h.lower().startswith(b"content-length:")][0]
                     .split(b":")[1])
        while len(rest) < length:
            rest += s.recv(4096)
        s.settimeout(1.0)
        try:
            closed = s.recv(1) == b""
        except socket.timeout:
            closed = False
        return int(head.split()[1]), head.decode(), closed


@pytest.mark.parametrize("header, status", [
    ("Content-Length: -1\r\n", 422),
    ("Content-Length: abc\r\n", 422),
    ("Content-Length: 1e3\r\n", 422),
    ("", 422),
    ("Content-Length: 65537\r\n", 413),
    ("Content-Length: " + "9" * 5000 + "\r\n", 413),
], ids=["negative", "garbage", "float", "missing", "over-64KiB", "5000-digits"])
def test_a_bad_content_length_is_refused_and_the_connection_closed(stack, header, status):
    http, _ = stack
    got, head, closed = _raw_post(http.base, header)
    assert got == status, head
    assert "connection: close" in head.lower() and closed, head


def test_a_good_content_length_keeps_working(stack):
    http, _ = stack
    body = b'{"label": "raw"}'
    got, head, _ = _raw_post(http.base, f"Content-Length: {len(body)}\r\n", body)
    assert got == 200, head
    got, head, _ = _raw_post(http.base, "Content-Length: 0\r\n")
    assert got == 200, head


def test_a_stalled_body_times_out_and_frees_the_thread(stack, monkeypatch):
    http, _ = stack
    assert 5.0 < web.Handler.timeout <= 60.0             # above the SSE keepalive's 5 s
    monkeypatch.setattr(web.Handler, "timeout", 0.5)     # read when the connection opens
    addr, head = _post_head(http.base, "Content-Length: 10\r\n")
    with socket.create_connection(addr, timeout=5) as s:
        s.sendall(head + b'{"a"')                        # 4 of the 10 bytes, then nothing
        started = time.monotonic()
        assert s.recv(1) == b""                          # closed, no response
        assert time.monotonic() - started < 4.0
