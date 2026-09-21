# Fixtures

Everything the offline tests need, so `tests/teleop/run_tests.sh` and `pytest tools/vr-teleop`
run from a fresh checkout with no robot, no headset, no node and no network.
`tests/test_fixtures.py` pins both files.

## `node-limits.json`

The two bodies franka-node serves — `params/schema` and `params/get` — in one file:

```json
{"schema": <the node's params/schema body>, "get": <its params/get body>}
```

It is **generated, never edited**:

```sh
python wire_tools/make_node_limits.py          # rewrite it from the node's schema
python wire_tools/make_node_limits.py --check  # exit 1 when it is out of date
```

The source is the node's own generated `crates/franka-node/schema/params-schema.json`, which
a node unit test keeps equal to the implementation. In a checkout without the node crate,
point `FRANKA_NODE_SCHEMA` at that file. `tests/test_fixtures.py` fails, naming this command,
when the committed fixture is not what a fresh generation produces — a schema change in the
node that forgets the fixture is then a red test rather than a stale table of somebody
else's numbers, which is the drift this tool exists to prevent.

The `get` half is derived from the schema: every parameter at its `default`, `version` 0,
`boot_id` `"fixture"`. `franka-vr-teleop --node-limits fixtures/node-limits.json` reads it off
hardware; live, the same values come from the node itself.

## `desk-idle.jsonl.gz`

60 s of a real headset recorded by `wire_tools/record_wire.py`: awake on a desk, not worn,
no grip, the left controller absent and the right one resting. 6000 messages, 3000 per
endpoint at 50 Hz. It is the wire T1, T3, T4, T5 and T8 of `run_tests.sh` replay, and what
the tests pin from it:

- **938** of the right controller's 3000 samples are not fresh (31.3 %), all gated;
- one **77.7 mm** hand step in a single sample, on a controller lying still;
- **26 %** of consecutive quaternions are antipodal, and none of them reach a target.

Regenerate a scrubbed copy of a new recording with:

```sh
python -m wire_tools.scrub_wire raw.jsonl fixtures/desk-idle.jsonl.gz
```

The scrub drops the header's `t0_wall_ns`, `t0_utc` and free-text `note`, drops every
record's wall-clock `w`, and rebases the monotonic `t` so the first message sits at 1 s;
every interval is preserved exactly. What stays is the payload of every message, its decoded
fields, the loopback endpoints and the footer's counts.
