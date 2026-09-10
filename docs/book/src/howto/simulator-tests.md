# Test against the simulator

At the end of this page you can run the crate's offline checks, its FR3 and FER simulator
suites and the Python tests locally, and you know how CI does the same. Nothing here needs a
robot; the simulator itself is introduced in
[Without a robot: franka-sim](../getting-started/simulator.md).

## The harness: `crates/franka-sim-test`

`SimServer::start(config)` launches a franka-sim container on Docker's host network, waits
until it serves the FCI, and removes the container when the returned `SimServer` is dropped:

```rust,no_run
# extern crate franka_sim_test;
use franka_sim_test::{SimConfig, SimServer};

let sim = SimServer::start(SimConfig::nominal());
// connect a client to sim.host(), port 1337 (robot) / 1338 (gripper) ...
drop(sim);   // container removed here
```

`SimConfig::nominal()` is the permissive default (no motion-limit or communication-constraint
checks, gripper enabled); `.with_motion_limits()`, `.with_comm_constraints()` and
`.with_gripper_object(width)` add the corresponding server flags, and `SimConfig::fer_v5()`
selects the FER / FCI v5 image and `--protocol v5 --robot panda`.

| variable | default | meaning |
|---|---|---|
| `FRANKA_SIM_IMAGE` | `ghcr.io/barisyazici/franka-sim:latest` | The FR3 / FCI v10 image. |
| `FRANKA_SIM_FER_IMAGE` | `franka-sim:panda-v5` | The FER / FCI v5 image, a local build whose recipe is kept outside this repository; the v5 tests fail rather than skip without it. A separate variable on purpose: `FRANKA_SIM_IMAGE` is not consulted for v5, or every v5 test would talk to a v10 server. |
| `FRANKA_SIM_ADDR` | unset | Attach to a server already running at this address instead of starting a container. The harness then never touches Docker, and the `SimConfig` a test passes is informational only: the caller is responsible for the running server matching it. |
| `FRANKA_SIM_KEEP` | unset | `=1` leaves the container up after the test for inspection. |

## One FCI slot per host

The ports 1337 and 1338 are fixed by the protocol and one client holds them at a time:

- a process-wide mutex serialises the tests inside one test binary, and every test drops its
  `Robot` / `Gripper` before the next one connects;
- `SimServer::start` refuses to start a second container while a `franka-sim*` container is
  running or the ports are bound;
- two `cargo test` invocations, or two test binaries, against the same host are not
  supported. Always pass `-- --test-threads=1`, and take the repository-root lock so two
  shells cannot collide: `flock .sim.lock <command>`.

Never run `cargo test --tests`: it selects every integration binary in the workspace,
including the `sim_*.rs` files, which start a container. Before starting anything,
`ss -tlnp | grep -E '133[78]'` and `docker ps` must show nothing.

## The offline checks

What CI's `check` job runs, in order; none of it needs Docker, a network or a robot:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy --workspace --all-targets --no-default-features -- -D warnings
cargo clippy -p franka-rs --all-targets --no-default-features -- -D warnings
cargo clippy -p franka-rs --all-targets --features serde -- -D warnings
cargo test --workspace --lib
cargo test -p franka-rs --lib --features serde
cargo test -p franka-rs \
  --test wire_sizes --test model_conformance \
  --test wire_sizes_v5 --test fer_native_conformance \
  --test fer_model_conformance \
  --test example_motion_generator
cargo test --workspace --doc
cargo test -p franka-rerun
cargo build -p franka-rerun --examples
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
```

The `--no-default-features` invocations keep the crate building without `model-library`, as
it [cross-compiles to static musl](./cross-compile.md); the `-p franka-rs` one is needed
because `--workspace` unifies the other crates' default features back on. The `--lib` tests
include the README-sync test (the README's quick example must stay byte-identical to `main`
in `examples/readme_joint_move.rs`). `fer_model_conformance` drives an FER's shared object,
which is not committed; without `FRANKA_FER_MODEL_SO` it prints `SKIP:` lines and passes.

## The FR3 suite

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-rs \
  --test sim_handshake --test sim_commands --test sim_motions \
  --test sim_gripper --test sim_stop_and_reflex --test sim_target_control \
  -- --test-threads=1
```

`--release`, as for a robot: the loops under test answer the server's 1 kHz state stream,
and unoptimised the impedance loop of target control takes about a millisecond per cycle
(35 µs optimised), so on a loaded machine it falls behind until the server's continuity
checks abort the motion.

`sim_target_control` needs franka-sim 1.1.6 or later. Its tests, and some of those in
`sim_stop_and_reflex`, request `--enforce-motion-limits` through the `SimConfig` they build;
when attaching to a server through `FRANKA_SIM_ADDR` that server must have the flag. Since
1.1.5 the FR3 image's `--enforce-motion-limits` also runs the robot's joint-side continuity
check on every commanded Cartesian pose; its 10 rad/s² acceleration table was confirmed on a
real FR3 on 2026-09-09 ([Benchmarks and hardware validation](../reference/benchmarks.md)).

## The FER suite

```sh
flock .sim.lock env FRANKA_SIM_FER_IMAGE=franka-sim:panda-v5 cargo test --release -p franka-rs \
  --test sim_v5_handshake --test sim_v5_commands --test sim_v5_motions \
  --test sim_v5_stop_and_reflex -- --test-threads=1
```

The `panda-v5` image has no joint-side check ([FER / Panda specifics](../reference/fer.md)).
A full `cargo test --release --workspace` needs both images, both variables set, and the lock.

## The harness self-test

`crates/franka-sim-test/tests/harness.rs` starts and tears down a real container and asserts
on `docker ps`, so it needs Docker, cannot attach to `FRANKA_SIM_ADDR`, and is in no CI job:

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-sim-test --test harness
```

## The Python tests

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev pytest crates/franka-py/tests
```

`conftest.py` starts the container with `--enforce-motion-limits` (or attaches to
`FRANKA_SIM_ADDR`) and sets `FRANKA_REALTIME=ignore`. `test_notebook.py` executes the
notebook the same way when `nbclient`, `ipykernel`, `matplotlib` and `rerun-sdk[notebook]`
are installed, is skipped otherwise, and is not part of CI.

## How CI gets its server

CI never talks to a robot, and neither should any test: a test that would need an arm becomes
a simulator test plus a characterisation assertion for the gap
([Simulator gaps](../reference/simulator-gaps.md)). `.github/workflows/ci.yml` has four FR3
simulator jobs, each getting its server from the
[`BarisYazici/libfranka-sim@v1`](https://github.com/BarisYazici/libfranka-sim) GitHub Action
with `FRANKA_SIM_ADDR: 127.0.0.1`, so the harness attaches rather than shelling out to Docker:

| job | server configuration | runs |
|---|---|---|
| `sim-nominal` | default | `sim_handshake`, `sim_commands`, `sim_motions`, `sim_gripper` (skipping the one test that needs an object), then `readme_joint_move` against the simulator with `FRANKA_REALTIME=ignore` |
| `sim-motion-limits` | `--enforce-motion-limits` | `sim_stop_and_reflex` (reflex, recovery, the rate-limiting envelope) and `sim_target_control` (the Cartesian and joint target loops under a stepped, bursting, stalling commander) |
| `sim-gripper-object` | `--gripper-object-width 0.04` | the one `sim_gripper` test that needs something between the fingers |
| `python-bindings` | `--enforce-motion-limits` | `crates/franka-py/tests/test_sim.py`: the wheel built with maturin, driven by pytest |

A fifth job, `sim-fer-v5`, runs the four `sim_v5_*` binaries. It cannot use the action: there
is no published FER image for it to pull, and the FER image lacks the `franka-sim-check`
binary the action's readiness probe needs. Instead the harness starts and stops the container
itself, driven by `FRANKA_SIM_FER_IMAGE`. The job is gated on that name existing as a
repository variable (Settings, Secrets and variables, Actions, Variables): unset, the job is
skipped and the workflow stays green; set to a `docker pull`-able reference, the FCI v5 suite
runs against it. Until then the v5 protocol is covered in CI by the `check` job only (wire
layout, model conformance, the offline unit tests) and the simulator half runs locally.
