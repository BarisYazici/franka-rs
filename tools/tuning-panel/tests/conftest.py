"""Fixtures: a mock owner subprocess on a private TCP endpoint and an in-process bridge + web
server connected to it. Both need `eclipse-zenoh`; tests that use them skip without it."""

import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request

import pytest

TOOL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOL)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Http:
    def __init__(self, base: str):
        self.base = base

    def _call(self, method: str, path: str, body=None, headers=None):
        """Mirrors static/api.js: GET bare, POST and DELETE with the JSON content type."""
        data = None if body is None else json.dumps(body).encode()
        headers = dict(headers or {})
        if method in ("POST", "DELETE"):
            headers.setdefault("Content-Type", "application/json")
        req = urllib.request.Request(self.base + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def get(self, path):
        return self._call("GET", path)[1]

    def post(self, path, body):
        return self._call("POST", path, body)

    def delete(self, path, headers=None):
        return self._call("DELETE", path, headers=headers)[1]


@pytest.fixture(scope="session")
def zenoh_mod():
    return pytest.importorskip("zenoh")


def start_mock(port: int, extra=()):
    proc = subprocess.Popen([sys.executable, os.path.join(TOOL, "mock_owner.py"), "--listen",
                             f"tcp/127.0.0.1:{port}", "--no-multicast", *extra],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    line = proc.stdout.readline()
    assert "mock owner" in line, line
    return proc


@pytest.fixture(scope="module")
def mock(zenoh_mod):
    port = free_port()
    proc = start_mock(port)
    yield port
    proc.terminate()
    proc.wait(5)


def build_stack(zenoh_mod, mock_port: int, presets_path: str):
    """In-process Bridge + web server connected to a mock; returns (Http, Bridge, stop)."""
    import argparse
    import zbus
    from bridge import Bridge
    from web import serve
    ap = argparse.ArgumentParser()
    zbus.add_zenoh_args(ap)
    args = ap.parse_args(["--connect", f"tcp/127.0.0.1:{mock_port}", "--no-multicast"])
    session = zenoh_mod.open(zbus.config_from_args(args))
    bridge = Bridge(session, presets_path, timeout=2.0)
    srv = serve(bridge, "127.0.0.1", free_port())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    http = Http("http://127.0.0.1:%d" % srv.server_address[1])
    for _ in range(50):  # until the owner is discovered and state flows
        if http.get("/api/arms").get("arms", {}).get("L") and http.get("/api/L/metrics").get("ok"):
            break
        time.sleep(0.2)

    def stop():
        srv.shutdown()
        session.close()
    return http, bridge, stop


@pytest.fixture(scope="module")
def stack(zenoh_mod, mock, tmp_path_factory):
    http, bridge, stop = build_stack(zenoh_mod, mock, str(tmp_path_factory.mktemp("presets") / "presets.json"))
    yield http, bridge
    stop()
