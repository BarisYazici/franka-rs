"""Which Host headers the panel serves, loopback-bound and after --expose-to-network.

The rule under test: loopback-bound only the loopback names are served, which is the DNS
rebinding defence; exposed, an IP literal is served as well, because a rebound DNS name is never
a literal. Any literal passes, not only this host's own addresses (see `HostPolicy`), and a name
that merely contains an address is still a name. Nothing here touches the network: the policy is
a pure function of the header, and the HTTP cases go to the in-process server on loopback with
the Host written by hand.
"""

import json
import urllib.error
import urllib.request

import pytest

from franka_node.panel.web import (HostPolicy, host_label, host_policy, ip_literal,
                                   local_addresses, reachable_urls, resolve_bind)

from .conftest import build_stack

LOOPBACK = HostPolicy()
EXPOSED = HostPolicy(expose=True)
NAMED = HostPolicy(expose=True, allowed=["bench-host.local"])


@pytest.fixture(scope="module")
def exposed_stack(mock, tmp_path_factory):
    """The same stack as `stack`, served with --expose-to-network's policy."""
    presets = str(tmp_path_factory.mktemp("exposed") / "presets.json")
    http, bridge, stop = build_stack(mock, presets, HostPolicy(expose=True))
    yield http, bridge
    stop()


def get(http, path, host, origin=None):
    headers = {"Host": host} if host else {}
    if origin:
        headers["Origin"] = origin
    req = urllib.request.Request(http.base + path, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def post(http, path, host, origin=None, body=b'{"label":"t"}'):
    headers = {"Host": host, "Content-Type": "application/json"}
    if origin:
        headers["Origin"] = origin
    req = urllib.request.Request(http.base + path, data=body, method="POST", headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


@pytest.mark.parametrize("host", ["localhost", "localhost:8765", "127.0.0.1", "127.0.0.1:9000",
                                  "[::1]", "[::1]:8765", "LocalHost:8765"])
def test_loopback_names_are_served_either_way(host):
    assert LOOPBACK.accepts(host) and EXPOSED.accepts(host)


@pytest.mark.parametrize("host", ["evil.example", "evil.example:8765", "bench-host.local:8765",
                                  "192.0.2.5:8765", "198.51.100.2", "[fe80::1]:8765", "",
                                  "localhost.", "127.0.0.1.", "192.0.2.5:notaport"])
def test_loopback_bound_serves_nothing_else(host):
    """Bound to loopback, a Host that is not a loopback name reached this port through someone's
    DNS; that includes IP literals, which a browser has no reason to send to a loopback bind."""
    assert not LOOPBACK.accepts(host)


@pytest.mark.parametrize("host", ["192.0.2.5:8765", "198.51.100.2", "[fe80::1]:8765", "[::1]",
                                  "[fe80::1%eth0]:8765", "172.16.0.2:8765",
                                  "[::ffff:192.0.2.5]:8765", "::ffff:192.0.2.5", "fe80::1",
                                  "192.0.2.5:notaport", "192.0.2.5:"])
def test_exposed_serves_ip_literals(host):
    """The port is not checked at all -- an ssh tunnel picks its own, and a literal is a literal
    whatever follows the colon."""
    assert EXPOSED.accepts(host)


def test_exposed_serves_any_ip_not_only_this_host_s_own():
    """The chosen side of local-vs-any: an address this process does not hold is served, because
    behind NAT, a port forward or a container the Host the browser sends is not one of ours."""
    foreign = "203.0.113.9:8765"  # TEST-NET-3, never a local address here
    assert ip_literal(host_label(foreign)) is not None
    assert host_label(foreign) not in local_addresses()
    assert EXPOSED.accepts(foreign)


@pytest.mark.parametrize("host", ["evil.example", "evil.example:8765", "bench-host.local:8765",
                                  "192.0.2.5.evil.example", "192.0.2.5.evil.example:8765",
                                  "10-42-0-2.evil.example", "1.2.3.4.nip.io", "192.0.2.5x",
                                  "0x7f000001", "2130706433", "192.0.2.5.", "localhost.",
                                  "192.0.2.5:8765:9", "[fe80::1"])
def test_exposed_still_refuses_names(host):
    """The other side of the rule: a name is a name however much of an address it contains, and
    a decimal or hex integer is not an address literal either."""
    assert not EXPOSED.accepts(host)


def test_allowed_names_are_exact():
    assert NAMED.accepts("bench-host.local") and NAMED.accepts("bench-host.local:8765")
    assert NAMED.accepts("PANDA-RT.local:8765")  # names are case insensitive
    for near in ("bench-host.local.evil.example", "evil.bench-host.local", "bench-host.localhost",
                 "bench-host.local.", "local", ".local", "rt.local"):
        assert not NAMED.accepts(near), near
    assert not LOOPBACK.accepts("bench-host.local")  # only where the operator wrote it out


def test_allowed_names_need_exposure():
    """A name is only as trustworthy as whoever can answer for it, and `.local` -- the example the
    flag and the book give -- is answered by mDNS, which anyone on the network can spoof. Serving
    such a name on a loopback bind would be the rebinding attack with no rebinding needed, so the
    policy ignores names unless exposed and the CLI refuses the combination outright."""
    assert not HostPolicy(expose=False, allowed=["bench-host.local"]).accepts("bench-host.local")
    for bind in ("127.0.0.1", "localhost", "::1"):
        with pytest.raises(SystemExit, match="--allowed-host needs --expose-to-network"):
            host_policy(bind, ["bench-host.local"])
    assert host_policy("0.0.0.0", ["bench-host.local"]).accepts("bench-host.local")


def test_the_bind_is_the_flag():
    """--expose-to-network only reaches the Host guard through the bind it unlocked: passing it
    while staying on loopback (where nothing outside can connect anyway) serves no literals."""
    for bind in ("127.0.0.1", "localhost", "::1"):
        assert not host_policy(bind).expose and not host_policy(bind).accepts("192.0.2.5:8765")
    for bind in ("0.0.0.0", "::", "192.0.2.5"):
        assert host_policy(bind).expose and host_policy(bind).accepts("192.0.2.5:8765")
    assert host_policy("0.0.0.0", ["bench-host.local"]).accepts("bench-host.local")


def test_refusal_says_what_is_served():
    assert HostPolicy().refusal() == "served on loopback names only"
    assert HostPolicy(expose=False, allowed=["bench-host.local"]).refusal() == "served on loopback names only"
    assert "IP addresses" in EXPOSED.refusal() and "--allowed-host" not in EXPOSED.refusal()
    assert "--allowed-host" in NAMED.refusal()


def test_host_label_and_ip_literal():
    assert host_label("[fe80::1]:8765") == "[fe80::1]" and host_label("[::1]") == "[::1]"
    assert host_label("192.0.2.5:8765") == "192.0.2.5" and host_label("localhost") == "localhost"
    assert str(ip_literal("[::1]")) == "::1" and ip_literal("evil.example") is None
    assert host_label("fe80::1") == "fe80::1"  # unbracketed: a second colon is not a port
    assert ip_literal("fe80::1") is not None and ip_literal("[fe80::1") is None


def test_loopback_bound_server_refuses_foreign_names(stack):
    http, _ = stack
    port = http.base.rsplit(":", 1)[1]
    for host in (f"localhost:{port}", "127.0.0.1:9000", "[::1]:9000"):
        assert get(http, "/api/arms", host)[0] == 200
    for host in ("evil.example:8765", "bench-host.local:8765", "192.0.2.5:8765"):
        status, body = get(http, "/api/arms", host)
        assert (status, body["reason"]) == (403, "host"), host
        assert body["error"] == "served on loopback names only"


def test_exposed_server_serves_ip_hosts(exposed_stack):
    http, _ = exposed_stack
    for host in ("192.0.2.5:8765", "198.51.100.2", "[fe80::1]:8765", "127.0.0.1:9000"):
        assert get(http, "/api/arms", host)[0] == 200, host
    for host in ("evil.example", "192.0.2.5.evil.example:8765"):
        status, body = get(http, "/api/arms", host)
        assert (status, body["reason"]) == (403, "host"), host


def test_exposed_write_from_the_exposed_origin(exposed_stack):
    """A write the panel itself makes when it is open at the exposed address: Origin == Host."""
    http, _ = exposed_stack
    status, body = post(http, "/api/L/markers", "192.0.2.5:8765", "http://192.0.2.5:8765")
    assert (status, body["ok"]) == (200, True)
    status, body = post(http, "/api/L/markers", "[fe80::1]:8765", "http://[fe80::1]:8765")
    assert (status, body["ok"]) == (200, True)
    # another page on the same network, aimed at the panel's address: the Host passes, the
    # Origin does not
    status, body = post(http, "/api/L/markers", "192.0.2.5:8765", "http://evil.example")
    assert (status, body["reason"]) == (403, "origin")
    status, body = post(http, "/api/L/markers", "192.0.2.5:8765", "http://198.51.100.2:8765")
    assert (status, body["reason"]) == (403, "origin")
    # and a rebound name is refused before the Origin is looked at
    status, body = post(http, "/api/L/markers", "evil.example:8765", "http://evil.example:8765")
    assert (status, body["reason"]) == (403, "host")


def test_origin_and_host_compare_case_insensitively(stack):
    """A browser lowercases both, but nothing in HTTP says it must: names and literals are
    case-insensitive, so the comparison is too. The value still has to be this server."""
    http, _ = stack
    port = http.base.rsplit(":", 1)[1]
    for host, origin in ((f"localhost:{port}", f"http://LOCALHOST:{port}"),
                         (f"LocalHost:{port}", f"http://localhost:{port}"),
                         (f"localhost:{port}", f"HTTP://localhost:{port}")):
        status, body = post(http, "/api/L/markers", host, origin)
        assert (status, body["ok"]) == (200, True), (host, origin)
    status, body = post(http, "/api/L/markers", f"localhost:{port}", f"http://127.0.0.1:{port}")
    assert (status, body["reason"]) == (403, "origin")  # another loopback name is another origin


def test_startup_urls_name_the_address(caplog):
    assert reachable_urls("192.0.2.10", 8765) == ["http://192.0.2.10:8765/"]
    assert reachable_urls("fe80::1", 8765) == ["http://[fe80::1]:8765/"]
    for url in reachable_urls("0.0.0.0", 8765):  # whatever this host holds, printable and bracketed
        assert url.startswith("http://") and url.endswith(":8765/")
    with caplog.at_level("WARNING", logger="tuning-bridge"):
        assert resolve_bind("192.0.2.10", True, 8765) == "192.0.2.10"
    assert "http://192.0.2.10:8765/" in caplog.text and "no authentication" in caplog.text


def test_local_addresses_are_addresses_and_never_loopback():
    for addr in local_addresses():
        assert ip_literal(addr) is not None and not ip_literal(addr).is_loopback
