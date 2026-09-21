"""Transport: the panel is a pure consumer, so it dials out as a zenoh client and is never
dialled. The tunnel case is reproduced with a TCP relay, so the locator the panel dials is not
the one the owner advertises — which is what makes a peer session fail and a client one work.
"""

import argparse
import json
import socket
import threading
import time

import pytest
import zenoh

from franka_node.panel import web, zbus
from franka_node.panel.bridge import Bridge

from .conftest import free_port, start_mock


class Relay(threading.Thread):
    """What `ssh -L` gives you: a near port that forwards to a far one."""

    def __init__(self, far: int):
        super().__init__(daemon=True)
        self.far = far
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        self.opened = 0

    def run(self) -> None:
        while True:
            try:
                near, _ = self.sock.accept()
                self.opened += 1
                far = socket.create_connection(("127.0.0.1", self.far))
            except OSError:
                return
            for a, b in ((near, far), (far, near)):
                threading.Thread(target=self.pump, args=(a, b), daemon=True).start()

    @staticmethod
    def pump(a: socket.socket, b: socket.socket) -> None:
        try:
            while True:
                data = a.recv(65536)
                if not data:
                    return
                b.sendall(data)
        except OSError:
            return
        finally:
            a.close()
            b.close()


def parsed(*argv: str, mode: str = "client") -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    zbus.add_zenoh_args(ap, mode=mode)
    return ap.parse_args(list(argv))


def test_client_mode_reaches_an_owner_through_a_relay(tmp_path):
    owner_port = free_port()
    proc = start_mock(owner_port, ["--no-state"])
    relay = Relay(owner_port)
    relay.start()
    session = zenoh.open(zbus.config_from_args(
        parsed("--connect", f"tcp/127.0.0.1:{relay.port}", "--no-multicast")))
    bridge = Bridge(session, str(tmp_path / "p.json"), timeout=2.0)
    try:
        for _ in range(20):
            arms = bridge.discover()
            if arms:
                break
            time.sleep(0.2)
        assert arms == {"L": {"node": True, "teleop": True}}
        assert bridge.linked() is True
        assert relay.opened >= 1  # it really went through the relay, not around it
    finally:
        session.close()
        relay.sock.close()
        proc.terminate()
        proc.wait(5)


def test_the_panel_dials_out_and_never_listens():
    assert parsed().mode == "client"
    assert parsed(mode="peer").mode == "peer"  # an owner, e.g. the mock, is dialled instead
    with pytest.raises(SystemExit, match="--mode peer"):
        zbus.config_from_args(parsed("--listen", "tcp/0.0.0.0:7447"))
    for mode in ("client", "peer"):
        assert json.loads(zbus.config_from_args(parsed("--mode", mode)).get_json("mode")) == mode
    c = zbus.config_from_args(parsed("--mode", "peer", "--listen", "tcp/0.0.0.0:7447"))
    assert json.loads(c.get_json("listen/endpoints")) == ["tcp/0.0.0.0:7447"]


def test_a_client_with_nothing_to_dial_says_what_to_do():
    """A client session cannot open with nothing on the bus, which is exactly the case someone
    opens the panel to diagnose. It must not be a traceback, and it must name the way out."""
    with pytest.raises(SystemExit) as e:
        web.open_session(parsed("--no-multicast"))  # client, no endpoint, no scouting
    assert "--mode peer" in str(e.value) and "--connect" in str(e.value)
    assert "client, dialling" in str(e.value) and ".rs" not in str(e.value)  # what it tried, no source path
    session = web.open_session(parsed("--no-multicast", "--mode", "peer"))  # a peer holds it open
    try:
        assert session.info.peers_zid() == []
    finally:
        session.close()


def test_the_resolved_session_is_printable(tmp_path):
    """The startup line is read back off the config itself, so a `--zenoh-config` that picks
    another mode or other endpoints shows as the session it is, not as a filename."""
    def line(*argv):
        return zbus.describe(zbus.config_from_args(parsed(*argv)))
    assert line("--connect", "tcp/host:7447") == "zenoh: client, dialling tcp/host:7447"
    assert "multicast scouting" in line()
    assert line("--mode", "peer", "--listen", "tcp/0.0.0.0:7447", "--no-multicast") == \
        "zenoh: peer, dialling nothing, listening on tcp/0.0.0.0:7447"
    f = tmp_path / "z.json5"
    f.write_text('{ mode: "peer", connect: { endpoints: ["tcp/1.2.3.4:7447"] },'
                 '  listen: { endpoints: ["tcp/0.0.0.0:7447"] } }')
    # the file wins over every flag, including the one that would otherwise be refused
    assert line("--zenoh-config", str(f), "--connect", "tcp/ignored:1", "--listen", "tcp/0.0.0.0:1") == \
        "zenoh: peer, dialling tcp/1.2.3.4:7447, listening on tcp/0.0.0.0:7447"
