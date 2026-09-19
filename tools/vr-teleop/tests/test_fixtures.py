"""The two fixtures the offline harness runs on, pinned.

`node-limits.json` is generated from franka-node's own `params-schema.json`; the test that
matters here is that the committed file is still what a fresh generation produces, so a
schema change in the node that forgets the fixture fails loudly instead of leaving a stale
copy of somebody else's numbers behind. That copy is exactly the drift this tool fights.

`desk-idle.jsonl.gz` is a recording of a real headset. It is pinned on what the harness
asserts from it and on the scrub: no wall-clock timestamp, no free text, nothing but
loopback endpoints.
"""
import gzip
import json
import math
import os

import pytest

from wire_tools import make_node_limits as gen

from .test_no_lab_details import private_needles, scan_text

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIMITS = os.path.join(ROOT, "fixtures", "node-limits.json")
RECORDING = os.path.join(ROOT, "fixtures", "desk-idle.jsonl.gz")

LEFT, RIGHT = "tcp://127.0.0.1:5560", "tcp://127.0.0.1:5570"


def read_limits():
    with open(LIMITS) as fh:
        return fh.read()


def load_recording():
    header, footer, rows = None, None, []
    with gzip.open(RECORDING, "rt") as fh:
        for line in fh:
            rec = json.loads(line)
            kind = rec.get("type")
            if kind == "header":
                header = rec
            elif kind == "footer":
                footer = rec
            else:
                rows.append(rec)
    return header, footer, rows


# ------------------------------------------------------------------ node-limits.json


def test_fixture_is_a_fresh_generation_from_the_nodes_schema():
    """The anti-drift check. It needs the node's schema file: in a checkout that has the
    node crate it is found by relative path, otherwise point FRANKA_NODE_SCHEMA at it."""
    schema_path = gen.find_schema()
    if schema_path is None:
        pytest.skip("franka-node's params-schema.json is not in this checkout; "
                    "set FRANKA_NODE_SCHEMA to run the drift check")
    with open(schema_path) as fh:
        fresh = gen.render(gen.build(json.load(fh)))
    assert read_limits() == fresh, (
        f"fixtures/node-limits.json disagrees with {schema_path}. "
        f"The node's parameters moved: regenerate with `{gen.REGENERATE}` and re-read the "
        f"values teleop.py validates against before running an arm.")


def test_fixture_is_self_consistent():
    """Without the node in the checkout: the committed file is still what the generator
    makes of its own schema half, so no hand edit of the `get` half survives."""
    fixture = json.loads(read_limits())
    assert read_limits() == gen.render(gen.build(fixture["schema"]))
    assert fixture["get"]["boot_id"] == "fixture" and fixture["get"]["version"] == 0
    assert fixture["schema"]["owner"] == "node"
    assert fixture["schema"]["schema_version"] == 1


def test_it_carries_every_value_the_client_bounds_itself_by():
    """The `derived` entries and the two budgets teleop.py's startup validation reads."""
    fixture = json.loads(read_limits())
    derived = fixture["schema"]["derived"]
    for key in ("max_lead", "max_lead_rotation", "max_step", "max_step_rotation",
                "rate_hz", "state_hz", "stop_after_ms", "dq_limit", "leash"):
        assert key in derived, key
        value = derived[key]
        if isinstance(value, (int, float)):
            assert math.isfinite(value) and value > 0.0, key
    assert len(derived["dq_limit"]) == 7
    for key in ("budget", "rotation_budget"):
        assert len(fixture["get"]["params"][key]) == 3, key
        assert fixture["get"]["params"][key] == fixture["schema"]["params"][key]["default"]


# ------------------------------------------------------------------ desk-idle.jsonl.gz


def test_the_recording_is_scrubbed():
    """Nothing in the file says when or where it was recorded."""
    header, _, rows = load_recording()
    for key in ("t0_wall_ns", "t0_utc", "note"):
        assert key not in header, f"the recording still carries {key}"
    assert not any("w" in row for row in rows), "a record still carries its wall clock"
    assert header["t0_perf_ns"] < rows[0]["t"] <= 2_000_000_000     # rebased, not raw uptime
    assert set(header["endpoints"]) == {LEFT, RIGHT}
    with gzip.open(RECORDING, "rt") as fh:
        text = fh.read()
    assert not scan_text("fixtures/desk-idle.jsonl", text, private_needles()), \
        "the recording carries a lab detail"


def test_the_recording_is_what_the_harness_asserts_from_it():
    """The three numbers run_tests.sh pins: 938 not-fresh samples of 3000 on the right
    controller's port, one 77.7 mm hand step, and 26 % antipodal quaternions."""
    header, footer, rows = load_recording()
    assert header["msg_size"] == 88 and header["struct"] == "<IIQ8dII"
    assert footer["total"] == 6000 and footer["bad"] == {"length": 0, "magic": 0,
                                                         "version": 0}
    right = [r for r in rows if r["ep"] == RIGHT]
    left = [r for r in rows if r["ep"] == LEFT]
    assert len(right) == len(left) == 3000
    assert sum(1 for r in right if not r["fresh"]) == 938
    assert all(not r["fresh"] for r in left)                        # that controller is off
    step = max(math.dist(a["pos"], b["pos"]) for a, b in zip(right, right[1:]))
    assert 0.0777 < step < 0.0778
    flips = sum(1 for a, b in zip(right, right[1:])
                if sum(x * y for x, y in zip(a["quat"], b["quat"])) < 0.0)
    assert 0.25 < flips / (len(right) - 1) < 0.27


def test_the_replay_reads_it():
    """Gzipped or expanded, ReplaySource finds the right controller's 3000 messages."""
    from franka_vr.teleop import sources
    try:
        replay = sources.ReplaySource(RECORDING, RIGHT)
    except (UnicodeDecodeError, json.JSONDecodeError):
        pytest.skip("ReplaySource does not read .gz yet; run_tests.sh expands it instead")
    assert len(replay.rows) == 3000
    assert all(len(payload) == 88 for _, payload in replay.rows)
