# Testing with franka-sim

Nothing in this repository requires a robot to develop against.
[`franka-sim`](https://github.com/BarisYazici/libfranka-sim) is a MuJoCo-backed simulator
that speaks the real FCI wire protocol, and the crate's integration tests drive it exactly
as they would drive an arm.

## How the harness works

`crates/franka-sim-test` is a small library whose `SimServer::start(config)` launches a
`franka-sim` container on Docker's host network, waits until it serves the FCI, and removes
the container when the returned `SimServer` is dropped.

```rust,no_run
# extern crate franka_sim_test;
use franka_sim_test::{SimConfig, SimServer};

let sim = SimServer::start(SimConfig::nominal());
// connect a client to sim.host(), port 1337 (robot) / 1338 (gripper) ...
drop(sim);   // container removed here
```

`SimConfig` selects the flavour: `nominal()`, `with_motion_limits()`,
`with_gripper_object(width)`, and `fer_v5()` for the FCI v5 image.

### The single FCI slot

The FCI's ports — **1337** robot, **1338** gripper — are fixed by the protocol, and only
one client may hold them at a time. So:

- a process-wide `Mutex` serialises the tests inside one test binary, and every test
  disconnects (drops its `Robot` / `Gripper`) before the next one connects;
- `SimServer::start` **refuses** to start a second container while a `franka-sim*`
  container is already running or those ports are already bound;
- running multiple `cargo test` invocations, or multiple test binaries, against the same
  host concurrently is not supported. Always pass `-- --test-threads=1`.

Locally, take the machine-wide lock so two shells cannot collide:

```sh
flock .sim.lock cargo test -p franka-rs --test sim_motions -- --test-threads=1
```

### Environment variables

| variable | default | meaning |
|---|---|---|
| `FRANKA_SIM_IMAGE` | `ghcr.io/barisyazici/franka-sim:latest` | The **FR3 / FCI v10** image. |
| `FRANKA_SIM_FER_IMAGE` | `franka-sim:panda-v5` | The **FER / FCI v5** image. Deliberately a separate variable: `FRANKA_SIM_IMAGE` is *not* consulted for v5, because inheriting the FR3 image would point every v5 test at a v10 server. |
| `FRANKA_SIM_ADDR` | unset | Attach to an already-running server at this address instead of starting a container. In this mode the harness never touches Docker, and any `SimConfig` a test passes is informational only — the caller is responsible for the running server matching it. |
| `FRANKA_SIM_KEEP` | unset | `=1` leaves the container up after the test for inspection. |

The two images are different builds. The FR3 one is published
(`docker pull ghcr.io/barisyazici/franka-sim:latest` is enough); the FER one is **not** —
its build recipe is kept privately, outside this repository. The v5 tests **fail rather
than skip** when it is missing, and the failure message names the variable and the build.

## Running locally

Docker-free, network-free — the crate unit tests plus the integration binaries that only
assert wire layout and check the model against a committed fixture:

```sh
cargo test -p franka-rs --lib \
  --test wire_sizes --test model_conformance \
  --test wire_sizes_v5 --test fer_native_conformance \
  --test fer_model_conformance
cargo test -p franka-sim-test --lib
cargo test --workspace --doc
```

That is exactly what CI's `check` job runs. Do **not** use `--tests` here: it selects every
integration binary in the workspace, including the `sim_*.rs` files, which start a Docker
container.

The FR3 simulator suite:

```sh
FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-rs \
  --test sim_handshake --test sim_commands --test sim_motions \
  --test sim_gripper --test sim_stop_and_reflex -- --test-threads=1
```

The FER suite:

```sh
FRANKA_SIM_FER_IMAGE=franka-sim:panda-v5 cargo test -p franka-rs \
  --test sim_v5_handshake --test sim_v5_commands --test sim_v5_motions \
  --test sim_v5_stop_and_reflex -- --test-threads=1
```

A full workspace run needs both images:

```sh
FRANKA_SIM_IMAGE=franka-sim:dev FRANKA_SIM_FER_IMAGE=franka-sim:panda-v5 \
  cargo test --workspace
```

One test file is excluded from every list above and from CI:
`crates/franka-sim-test/tests/harness.rs` tests the *container harness itself* — it starts
and tears down a real container and asserts on `docker ps` — so it needs Docker and cannot
run against an already-attached `FRANKA_SIM_ADDR` server. Run it by hand:

```sh
FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-sim-test --test harness
```

## How CI runs it

`.github/workflows/ci.yml` has three FR3 simulator jobs, each of which gets its server from
the [`BarisYazici/libfranka-sim`](https://github.com/BarisYazici/libfranka-sim) GitHub
Action and sets `FRANKA_SIM_ADDR: 127.0.0.1` so the harness attaches rather than shelling
out to Docker:

| job | server configuration | runs |
|---|---|---|
| `sim-nominal` | default | `sim_handshake`, `sim_commands`, `sim_motions`, `sim_gripper` (skipping the one test that needs an object), then the README example `readme_joint_move` against the simulator with `FRANKA_REALTIME=ignore` |
| `sim-motion-limits` | `--enforce-motion-limits` | `sim_stop_and_reflex` — reflex, recovery and the rate-limiting envelope |
| `sim-gripper-object` | `--gripper-object-width 0.04` | the one `sim_gripper` test that needs something between the fingers |

A fourth job, **`sim-fer-v5`**, runs the four `sim_v5_*` binaries — but it cannot use that
action, because there is no published FER image for it to pull and the FER image also
lacks the `franka-sim-check` binary the action's readiness probe needs. Instead the
`franka-sim-test` harness starts and stops the container itself, exactly as it does
locally, driven by `FRANKA_SIM_FER_IMAGE`. The job is gated on that name existing as a
**repository variable** (Settings → Secrets and variables → Actions → Variables): unset, the
job is skipped and the workflow stays green; set to a `docker pull`-able reference, the FCI
v5 simulator suite runs against it.

Until that variable is set, the v5 protocol is covered in CI by the `check` job only — wire
layout, model conformance and the mock-FCI unit tests, all offline — and the simulator half
is exercised locally.

## What the simulator is not

`franka-sim` does not run under `PREEMPT_RT`, and neither does CI, so every test and
example connects with `RealtimeConfig::Ignore`. The simulator also has documented
divergences from real hardware — most of them listed under [FER
specifics](./fer.md#known-simulator-gaps), and the FR3 image shares some (the `O_ddP_O`
and `O_T_EE`-origin ones). Every such divergence is pinned by a characterisation assertion
that names the stub in its failure message, so a rebuilt image tightens the test instead of
breaking it silently.

Timing numbers measured against the simulator are a same-box A/B, **not an FCI
qualification** — see [Benchmarks](./benchmarks.md).
