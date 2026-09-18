# Tuning panel

An optional browser interface for a running `franka-node` Cartesian impedance session.
The Python bridge connects to Zenoh and serves the static panel locally. Sliders and entries
come from the node's published schema; bounds are not maintained separately in the UI.
This tool runs from source and is not a separately published package.

Start with [Tune a running controller](../../docs/book/src/howto/live-tuning.md).
The authoritative node protocol is [Live parameter protocol](../../docs/book/src/reference/node-parameters.md).

## Install and run

From the repository root, create a dedicated environment (Python 3.13 is the tested version):

```sh
python3 -m venv .venv-tuning
source .venv-tuning/bin/activate
python -m pip install 'eclipse-zenoh>=1.10,<2'
cd tools/tuning-panel
python web.py --connect tcp/<node-address>:7447 --no-multicast \
  --presets ./presets.json
```

Open <http://127.0.0.1:8765/>. The bridge can run on your laptop, with the node on the Pi.
The node must already be running; the panel does not start a motion session. A commander
owns and enables the arm. Parameter writes are separate from that command lease, so run the
bridge on a trusted node network and coordinate changes with the operator.

The HTTP server binds loopback by default. If it runs on the control host, reach it through
an SSH tunnel from the laptop:

```sh
ssh -L 8765:127.0.0.1:8765 <user>@<control-host>
```

`--host` refuses non-loopback addresses unless `--expose-to-network` is given. That option
exposes a bridge with no authentication. The normal setup keeps the loopback bind.
Requests also check the Host header and, for browser writes, the Origin and JSON content type.

`--presets` selects a local JSON file; the default is
`~/.local/state/franka-tuning/presets.json`. Saving a preset does not write the node's TOML
or automatically restore settings after a restart or session end.

## Try the mock

In two terminals with the environment activated, from this directory:

```sh
# Terminal 1: synthetic parameter owner and state; no robot connection.
python mock_owner.py --listen tcp/127.0.0.1:17447 --no-multicast
```

```sh
# Terminal 2: bridge and panel.
python web.py --connect tcp/127.0.0.1:17447 --no-multicast --presets ./presets.json
```

The mock also implements an example teleoperation parameter owner. That is a test fixture:
this repository does not include a production teleoperation application serving that protocol.
`--arm R` selects arm R for that mock process; run another process for a second arm.
`--schema-override` exercises changed schema bounds.

The node mock reads `crates/franka-node/schema/params-schema.json`, generated from the Rust
bounds table. Regenerate it from the repository root after a schema change:

```sh
cargo run -p franka-node --example params_schema > crates/franka-node/schema/params-schema.json
```

A node unit test checks that the dump matches the implementation.

## Bridge behavior

- It discovers concrete arm keys through wildcard schema queries, subscribes to packed state
  and parameter updates, and reads the node-scoped status topic.
- Batched Apply sends the node update first. A node rejection aborts the optional teleop
  update. This is not an atomic transaction across owners or arms.
- Bridge errors include `unreachable`, `aborted`, `bad_schema`, `no_state` and `stale_state`;
  these are distinct from the node's [parameter refusals](../../docs/book/src/reference/node-parameters.md).
- Metrics use the node state's monotonic timestamps. Staleness uses the bridge's own clock.
  Markers use the latest state timestamp; markers and before/after windows are shared by
  every browser tab attached to this bridge for that arm.
- State decode failures and absent state/status are displayed instead of treating an
  unreadable stream as a healthy arm. The state wire layout lives in `statemsg.py`.
- Jitter, lag and tracking plots are estimates from published state, not measurements of
  every control cycle. The joint headroom display needs both node and teleop inputs and
  stays unavailable until they arrive.

## Current limits

The node has no `params/save` endpoint. Applied values reset to TOML on session end or
restart. `get/current.params` contains accepted targets, and `slewing` estimates the fraction
remaining; it is not sampled feedback of the instantaneous controller gains.

The node does not publish pose-independent joint inertia for a damping-ratio hint, or the
proposed per-cycle leash/cap metrics. Those displays cannot establish a controller's safety
or feasibility. A teleop slider's node bound is currently an advisory line, not a hard tick.

## Files and tests

| Files | Purpose |
|---|---|
| `web.py`, `bridge.py`, `zbus.py` | HTTP/SSE server, Zenoh bridge and connection settings |
| `validation.py`, `presets.py` | Request validation and saved presets |
| `metrics.py`, `statemsg.py` | Published-state metrics and wire decoding |
| `mock_owner.py`, `mock_schemas.py` | Synthetic node and teleop owners |
| `static/` | Browser interface; no frontend build step |
| `tests/` | Validation, metrics, browser logic and mock bridge tests |

In the dedicated environment:

```sh
python -m pip install pytest
python -m pytest tests
```

Without Zenoh, bridge integration tests skip. Some browser logic checks also need `node`.
The mock tests do not connect to physical robots.
