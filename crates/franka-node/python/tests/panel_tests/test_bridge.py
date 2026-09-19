"""End to end against the mock owner: discovery, schema passthrough, the rejection paths, the
batched Apply, markers and metrics, presets, SSE."""

import json
import os
import time
import urllib.request

import pytest

from franka_node.panel import zbus
from franka_node.panel.bridge import ArmMonitor
from franka_node.panel.web import STATIC, resolve_bind

from .conftest import build_stack, free_port, start_mock
from .mock_state import encode_state


def test_discovery(stack):
    http, _ = stack
    assert http.get("/api/arms")["arms"] == {"L": {"node": True, "teleop": True}}


def test_schema_is_the_owners_and_carries_design_rt_bounds(stack):
    http, _ = stack
    s = http.get("/api/L/node/schema")
    p = s["params"]
    assert p["joint_damping"]["max"] == [60] * 7        # not the reference stack's 80
    assert p["ik_damping"]["min"] == 1e-3 and p["ik_damping"]["policy"] == "slew"
    assert p["budget"]["max"][0] == 1.2 and p["budget"]["policy"] == "step_up_gate_down"
    # The one limit no per-field row can express, published as the rule the node enforces.
    assert [r["rule"] for r in s["relations"]] == [
        "joint_damping[i] >= 0.25 * sqrt(joint_stiffness[i])"]
    assert "leash" in s["derived"] and "leash" not in p          # read-only, DESIGN-rt 7.3
    assert set(p) == {"joint_stiffness", "joint_damping", "cartesian_stiffness", "velocity_feedforward_gain",
                      "velocity_feedforward_cutoff", "ik_damping", "ik_nullspace_gain", "budget", "rotation_budget"}


def test_headroom_limits_follow_the_owners(tmp_path):
    """dq_limit from the node's derived block, the release line from the teleop client's own
    parameter; neither is a constant in the bridge."""
    port = free_port()
    proc = start_mock(port, ["--schema-override", json.dumps({"derived": {"dq_limit": [1.0] * 7}})])
    http, bridge, stop = build_stack(port, str(tmp_path / "p.json"))
    try:
        engine = bridge.arm("L").metrics
        for _ in range(30):  # the seeder thread retries every SEED_PERIOD_S
            if engine.dq_limit is not None and engine.release_fraction is not None:
                break
            time.sleep(0.2)
        with engine.lock:
            dq = list(engine.rows[-1].dq)
            h = engine._headroom(dq)
        assert h["release"] == http.get("/api/L/teleop/params")["params"]["dq_release_fraction"]
        assert h["fractions"] == pytest.approx([abs(v) / 1.0 for v in dq])
        assert http.get("/api/L/metrics")["dq_headroom"]["release"] == h["release"]
        _, r = http.post("/api/L/teleop/params", {"client_id": 1, "params": {"dq_release_fraction": 0.6}})
        assert r["ok"]
        time.sleep(0.3)
        assert http.get("/api/L/metrics")["dq_headroom"]["release"] == 0.6
    finally:
        stop()
        proc.terminate()


def test_malformed_schema_is_reported_not_rendered(tmp_path):
    port = free_port()
    proc = start_mock(port, ["--schema-override", json.dumps({"ik_damping": {"min": 0}})])
    http, _, stop = build_stack(port, str(tmp_path / "p.json"))
    try:
        s = http.get("/api/L/node/schema")
        assert s["ok"] is False and s["reason"] == "bad_schema" and "ik_damping: log scale" in s["error"]
        assert "params" not in s
    finally:
        stop()
        proc.terminate()


def test_bounds_come_from_the_published_schema_not_a_constant(tmp_path):
    """A different owner schema changes what the bridge serves and what the owner clamps to."""
    port = free_port()
    proc = start_mock(port, ["--schema-override", json.dumps({"joint_damping": {"max": [45] * 7}})])
    http, _, stop = build_stack(port, str(tmp_path / "p.json"))
    try:
        assert http.get("/api/L/node/schema")["params"]["joint_damping"]["max"] == [45] * 7
        _, r = http.post("/api/L/node/params", {"client_id": 1, "params": {"joint_damping": [50] * 7}})
        assert r["ok"] and r["params"]["joint_damping"] == [45] * 7
        assert r["clamped"][0] == {"field": "joint_damping", "index": 0, "requested": 50.0, "stored": 45}
    finally:
        stop()
        proc.terminate()


def test_get_and_version_zero(stack):
    http, _ = stack
    g = http.get("/api/L/teleop/params")
    assert g["owner"] == "teleop" and g["version"] == 0 and g["dirty"] is False
    assert g["params"]["clamp"] == 0.025 and g["params"]["rate"] == 50


def test_rejections(stack):
    http, _ = stack
    v0 = http.get("/api/L/node/params")["version"]
    code, r = http.post("/api/L/node/params", {"client_id": 1, "params": {"ik_damping": "x"}})
    assert (code, r["ok"], r["reason"], r["field"]) == (200, False, "type", "ik_damping")
    assert "version" not in r  # refused at the bridge, never reached the owner
    _, r = http.post("/api/L/node/params", {"client_id": 1, "params": {"nope": 1}})
    assert r["reason"] == "unknown_field"
    _, r = http.post("/api/L/node/params", {"client_id": 1, "params": {"budget": [1, 2]}})
    assert r["reason"] == "length"
    _, r = http.post("/api/L/node/params", {"client_id": 1, "base_version": v0 + 7, "params": {"ik_damping": 0.1}})
    assert r["reason"] == "stale" and r["version"] == v0
    _, r = http.post("/api/L/node/params", {"client_id": 1, "params": {"budget": [0.9, 10, 400]}})
    assert r["reason"] == "needs_confirm" and r["field"] == "budget"
    _, r = http.post("/api/L/teleop/params", {"client_id": 1, "params": {"dq_resume_fraction": 0.9}})
    assert r["reason"] == "relation" and r["field"] == "dq_resume_fraction"
    assert http.get("/api/L/node/params")["version"] == v0  # nothing stored by any of the above
    req = urllib.request.Request(http.base + "/api/L/node/params", data=b"not json", method="POST",
                                 headers={"Content-Type": "application/json"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 422


def test_confirmed_set_then_apply_batch_and_marker(stack):
    http, bridge = stack
    v0 = http.get("/api/L/node/params")["version"]
    n_markers = len(bridge.arm("L").metrics.markers)
    _, r = http.post("/api/L/node/params", {"client_id": 1, "base_version": v0, "confirm": ["budget"],
                                             "params": {"budget": [0.9, 10, 400], "ik_damping": 0.1}})
    assert r["ok"] and r["version"] == v0 + 1 and r["params"]["budget"] == [0.9, 10, 400]
    assert r["slewing"] == {"ik_damping": 1.0} and r["clamped"] == []
    m = bridge.arm("L").metrics.markers
    assert len(m) == n_markers + 1 and m[-1].label == f"node v{v0 + 1}" and m[-1].version == v0 + 1
    # batch: a node refusal aborts the teleop send
    tv = http.get("/api/L/teleop/params")["version"]
    _, r = http.post("/api/L/apply", {"node": {"client_id": 1, "params": {"ik_damping": -1}},
                                      "teleop": {"client_id": 1, "params": {"clamp": 0.04}}})
    assert r["ok"] is False and (r["node"]["reason"], r["node"]["field"]) == ("invalid", "ik_damping")
    assert r["teleop"] == {"ok": False, "reason": "aborted", "field": None, "error": "not sent: node refused"}
    assert http.get("/api/L/teleop/params")["version"] == tv
    # batch: both accepted
    _, r = http.post("/api/L/apply", {"node": {"client_id": 1, "params": {"ik_damping": 0.2}},
                                      "teleop": {"client_id": 1, "params": {"clamp": 0.04}}})
    assert r["ok"] and r["node"]["ok"] and r["teleop"]["ok"]
    assert http.get("/api/L/teleop/params")["params"]["clamp"] == 0.04
    assert len(bridge.arm("L").metrics.markers) == n_markers + 3


def test_metrics_from_state_and_reflex(stack):
    http, bridge = stack
    time.sleep(1.5)
    s = http.get("/api/L/metrics")
    assert s["ok"] and s["valid"] and 0 < s["j4_rms"] < 0.2 and s["lag_ms"] is not None
    assert len(s["dq_headroom"]["fractions"]) == 7
    assert s["reach"] == {"node": True, "teleop": True}
    rows = bridge.arm("L").metrics.rows
    assert rows[0].t <= s["markers"][-1]["t"] <= rows[-1].t  # markers live on the state stream's clock
    zbus.query_one(bridge.session, "franka/L/mock/ctl", {"robot_mode": 4})
    time.sleep(0.3)
    s = http.get("/api/L/metrics")
    assert s["valid"] is False and s.get("after", {}).get("n", 0) == 0
    zbus.query_one(bridge.session, "franka/L/mock/ctl", {"robot_mode": 2})
    time.sleep(0.3)
    assert http.get("/api/L/metrics")["valid"] is True


def test_undecodable_state_is_counted_logged_and_said(stack, caplog):
    """Bytes the bridge cannot parse (the old JSON shape, junk, a wrong length) must not raise
    on the subscriber thread and must reach the page as a number, not vanish."""
    http, bridge = stack
    mon = bridge.arm("L")
    before = mon.decode_failures
    with caplog.at_level("ERROR", logger="tuning-bridge"):
        mon.on_state(b'{"t_ns": 1, "dq": [0,0,0,0,0,0,0]}')
        mon.on_state(b"junk")
        mon.on_state(bytes(473))
    assert mon.decode_failures == before + 3
    assert http.get("/api/L/metrics")["channels"]["state_decode_failures"] == before + 3
    assert http.get("/api/L/metrics")["ok"] is True  # the real stream still decodes
    if before == 0:
        assert "cannot be decoded" in caplog.text and caplog.text.count("cannot be decoded") == 1
    # over the bus too: a publisher on the state key with the wrong bytes
    pub = bridge.session.declare_publisher("franka/L/state")
    pub.put(b"not a StateMsg")
    time.sleep(0.3)
    assert mon.decode_failures == before + 4


def test_reflex_from_state():
    """Reflex is robot mode 4, 5 or 6 (reflex, user stopped, error recovery) or `has_errors`."""
    mon = ArmMonitor("X", lambda owner, verb: {})

    def reflex(**fields):
        mon.on_state(encode_state(**fields))
        return mon.condition["reflex"]
    assert not reflex(robot_mode=2) and not reflex(robot_mode=3)
    assert all(reflex(robot_mode=mode) for mode in (4, 5, 6))
    assert reflex(robot_mode=2, has_errors=True)
    assert mon.status_event()["robot_mode_name"] == "move" and mon.decode_failures == 0


def test_status_comes_from_the_node_key_and_reflex_from_state(stack):
    """`franka/node/<node>/status` fans out per arm; the reflex flag is the state's, and a mode
    change reaches the page at once, not at the 1 Hz status."""
    http, bridge = stack
    zbus.query_one(bridge.session, "franka/L/mock/ctl", {"robot_mode": 2, "has_errors": False})
    time.sleep(1.2)
    mon = bridge.arm("L")
    assert mon.status["name"] == "L" and mon.status["mode"] == "cartesian" and mon.status["holder"] == 7200
    ev = mon.status_event()
    assert (ev["phase"], ev["robot_mode"], ev["robot_mode_name"], ev["holder"], ev["reflex"]) == ("active", 2, "move", 7200, False)
    assert ev["channels"]["state_age_s"] < 0.5 and ev["channels"]["status_age_s"] < 1.5
    with urllib.request.urlopen(http.base + "/api/L/events", timeout=5) as r:
        assert r.readline().decode() == "event: hello\n"
        hello = json.loads(r.readline().decode()[6:])
        assert hello["status"]["reflex"] is False and hello["channels"]["state_decode_failures"] >= 0
        t0 = time.monotonic()
        zbus.query_one(bridge.session, "franka/L/mock/ctl", {"robot_mode": 5})
        try:
            while time.monotonic() - t0 < 2.0:
                line = r.readline().decode()
                if line == "event: status\n":
                    st = json.loads(r.readline().decode()[6:])
                    if st["reflex"]:
                        break
            else:
                pytest.fail("no reflex status event")
            assert (st["robot_mode"], st["robot_mode_name"]) == (5, "user_stopped")
            assert time.monotonic() - t0 < 0.6  # from the state edge, not the 1 Hz status
        finally:
            zbus.query_one(bridge.session, "franka/L/mock/ctl", {"robot_mode": 2})
    time.sleep(0.3)
    assert mon.status_event()["reflex"] is False


def test_dead_node_is_reported_not_frozen(stack):
    http, bridge = stack
    zbus.query_one(bridge.session, "franka/L/mock/ctl", {"silent": True})
    try:
        time.sleep(3.5)
        s = http.get("/api/L/metrics")
        assert s["ok"] is False and s["reason"] == "stale_state" and s["age_s"] > 1.0
        assert s["reach"] == {"node": False, "teleop": False}
        assert s["channels"]["state_age_s"] > 1.0
    finally:
        zbus.query_one(bridge.session, "franka/L/mock/ctl", {"silent": False})
    time.sleep(1.5)
    s = http.get("/api/L/metrics")
    assert s["ok"] is True and s["reach"]["node"] is True


def test_ticker_never_waits_on_an_owner(stack):
    """Seeding limits may block for the query timeout; the 10 Hz tick must not."""
    _, bridge = stack
    mon = bridge.arm("L")
    calls = []

    def slow(owner, verb):
        calls.append((owner, verb))
        time.sleep(0.5)
        return {}
    mon.query, saved = slow, mon.query
    mon.metrics.dq_limit, saved_limits = None, (mon.metrics.dq_limit, mon.metrics.release_fraction)
    try:
        t0 = time.monotonic()
        for _ in range(10):
            mon.tick()
        assert time.monotonic() - t0 < 0.5 and calls == []
        mon.ensure_limits()  # the seeder's call is the one that pays
        assert calls and time.monotonic() - t0 >= 0.5
    finally:
        mon.query = saved
        mon.metrics.set_limits(*saved_limits)


def test_foreign_host_is_refused_and_loopback_allowed(stack):
    http, _ = stack
    port = http.base.rsplit(":", 1)[1]
    req = urllib.request.Request(http.base + "/api/arms", headers={"Host": f"evil.example:{port}"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403
    # rebinding: same-origin by the browser's reckoning, JSON, Origin == Host; still refused
    req = urllib.request.Request(http.base + "/api/L/markers", data=b"{}", method="POST",
                                 headers={"Content-Type": "application/json", "Host": f"evil.example:{port}",
                                          "Origin": f"http://evil.example:{port}"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403
    req = urllib.request.Request(http.base + "/api/L/markers", method="DELETE",
                                 headers={"Content-Type": "application/json", "Origin": "http://evil.example"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403
    for host in (f"127.0.0.1:{port}", f"localhost:{port}", "127.0.0.1:9000", "[::1]:9000"):  # any tunnel port
        req = urllib.request.Request(http.base + "/api/L/markers", data=b'{"label":"t"}', method="POST",
                                     headers={"Content-Type": "application/json", "Host": host,
                                              "Origin": f"http://{host}"})
        with urllib.request.urlopen(req) as r:
            assert r.status == 200 and json.loads(r.read())["ok"] is True
    with urllib.request.urlopen(urllib.request.Request(http.base + "/", headers={"Host": "localhost:9000"})) as r:
        assert r.status == 200  # the page itself through a tunnel on another local port


def test_delete_as_the_panel_sends_it(stack):
    """static/api.js sends DELETE with the JSON content type and no body; the gate must take
    exactly that and refuse a bare DELETE (a form cannot send the JSON type without a preflight)."""
    http, _ = stack
    port = http.base.rsplit(":", 1)[1]
    same = {"Origin": f"http://127.0.0.1:{port}"}
    api_js = open(os.path.join(STATIC, "api.js")).read()
    assert "method: 'DELETE', headers: { 'Content-Type': 'application/json' }" in api_js
    assert http.delete("/api/L/markers", headers=same) == {"ok": True}
    bare = urllib.request.Request(http.base + "/api/L/markers", method="DELETE", headers=same)
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(bare)
    assert e.value.code == 403


def test_cross_origin_post_is_refused(stack):
    http, _ = stack
    req = urllib.request.Request(http.base + "/api/L/markers", data=b"label=x", method="POST",
                                 headers={"Content-Type": "text/plain"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403
    req = urllib.request.Request(http.base + "/api/L/markers", data=b"{}", method="POST",
                                 headers={"Content-Type": "application/json", "Origin": "http://evil.example"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403


def test_presets(stack):
    http, _ = stack
    ps = {p["name"]: p for p in http.get("/api/L/presets")["presets"]}
    schema = http.get("/api/L/node/schema")["params"]
    assert ps["library-defaults"]["node"] == {k: v["default"] for k, v in schema.items()}
    assert ps["toml"]["node"]["ik_damping"] == 0.05  # the version-0 set the bridge saw
    _, r = http.post("/api/L/presets", {"name": "toml", "note": ""})
    assert r["ok"] is False
    _, r = http.post("/api/L/presets", {"name": "felt-right", "note": "lambda 0.2", "client_id": 1})
    assert r["ok"] and r["preset"]["node"]["ik_damping"] == 0.2 and r["preset"]["teleop"]["clamp"] == 0.04
    assert "j4_rms" in r["preset"]["snapshot"]
    assert "felt-right" in {p["name"] for p in http.get("/api/L/presets")["presets"]}
    assert "felt-right" not in {p["name"] for p in http.get("/api/R/presets")["presets"]}  # keyed by arm
    assert http.delete("/api/R/presets/felt-right")["ok"] is False
    assert http.delete("/api/L/presets/felt-right")["ok"] is True
    assert "felt-right" not in {p["name"] for p in http.get("/api/L/presets")["presets"]}


def test_sse_delivers_hello_current_and_metrics(stack):
    http, _ = stack
    with urllib.request.urlopen(http.base + "/api/L/events", timeout=5) as r:
        seen, deadline = {}, time.time() + 3
        while time.time() < deadline and not {"hello", "current", "metrics"} <= set(seen):
            line = r.readline().decode()
            if line.startswith("event: "):
                ev = line[7:].strip()
                seen[ev] = json.loads(r.readline().decode()[6:])
    assert seen["hello"]["arm"] == "L" and seen["hello"]["reach"]["node"] is True
    assert "reflex" in seen["hello"]["status"] and "channels" in seen["hello"]
    assert seen["current"]["owner"] in ("node", "teleop") and "params" in seen["current"]
    assert seen["metrics"]["ok"] and "reach" in seen["metrics"]


def test_bind_is_loopback_unless_exposed(caplog):
    assert resolve_bind("127.0.0.1", False) == "127.0.0.1"
    assert resolve_bind("::1", False) == "::1"
    with pytest.raises(SystemExit, match="not loopback"):
        resolve_bind("0.0.0.0", False)
    with pytest.raises(SystemExit):
        resolve_bind("192.0.2.10", False)
    with caplog.at_level("WARNING", logger="tuning-bridge"):
        assert resolve_bind("0.0.0.0", True) == "0.0.0.0"
    assert "no authentication" in caplog.text
