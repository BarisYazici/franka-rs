# franka-rs

[![CI](https://github.com/BarisYazici/franka-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/BarisYazici/franka-rs/actions/workflows/ci.yml)
[![Docs](https://github.com/BarisYazici/franka-rs/actions/workflows/docs.yml/badge.svg)](https://github.com/BarisYazici/franka-rs/actions/workflows/docs.yml)
[![crates.io](https://img.shields.io/crates/v/franka-rs.svg)](https://crates.io/crates/franka-rs)
[![docs.rs](https://img.shields.io/docsrs/franka-rs)](https://docs.rs/franka-rs)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

A pure-Rust client for the **Franka Control Interface** -- the network protocol `libfranka`
speaks to a Franka Emika / Franka Robotics arm and its gripper. It talks the wire protocol
directly (TCP command channel plus UDP state/command streaming at 1 kHz) and implements the
kinematics and dynamics natively, so there is no dependency on `libfranka`, Poco, Eigen,
Pinocchio or any C++ toolchain: `cargo build` is the whole story. `Robot::new` negotiates
the protocol version, so the same binary drives an FR3 and a Franka Emika Robot (FER).

**[Documentation](https://barisyazici.github.io/franka-rs/)** |
**[API reference](https://barisyazici.github.io/franka-rs/api/franka/index.html)** |
**[Python](https://barisyazici.github.io/franka-rs/getting-started/python.html)** |
**[Changelog](CHANGELOG.md)**

## Why a Rust client

`libfranka` is the reference and this crate is checked against it: wire layouts, rate
limiting, error text and the model agree, and loop timing measured side by side on real
FR3 and FER arms is the same. The reasons to pick `franka-rs` are elsewhere:

- **One client for both robot generations.** The FR3 (FCI v10) and the FER (FCI v5) need
  two incompatible `libfranka` versions; here the version is negotiated at connect and the
  same binary drives both.
- **No C++ stack.** `cargo add franka-rs` replaces `libfranka` plus Poco, Eigen and
  Pinocchio, with no system packages and no `find_package`; it cross-compiles to aarch64
  (glibc, or static musl).
- **The 1 kHz loop as a library.** Write it as a callback or with `read_once` /
  `write_once`, or let the crate run it on a realtime thread and set targets from any
  program at any rate -- including Python, where the GIL is never on the 1 kHz path.
- **Safe Rust on the realtime path.** Parsing, rate limiting, trajectory generation and
  the loops are safe Rust and allocate nothing once a motion runs; the `unsafe` in the
  crate is the libc socket and scheduler calls in `network/` and `realtime.rs`, plus the
  opt-in loader for the model an FER serves (`model-library` feature).
- **The model without a download.** FR3 from the robot's URDF, FER from shipped identified
  parameters, both evaluated natively, agreeing with `libfranka` to 1e-14.

What it does **not** give you: better timing than `libfranka` (they are equal), a
`ros2_control` hardware interface, or a `VacuumGripper`. The book's
[Compared with libfranka](https://barisyazici.github.io/franka-rs/reference/libfranka.html)
has the full list, in both directions.

## Supported robots

| FCI | libfranka semantics | Robot | System | `RobotState` | Model |
|---|---|---|---|---|---|
| **v10** | 0.20 - 0.21 | Franka Research 3 | 5.9.0 or later | 1377 B | URDF from `GetRobotModel`, evaluated natively |
| **v5** | 0.9.2 | Franka Emika Robot (FER) | 4.2.x | 2373 B | parameters identified from the robot's own `libfcimodels.so`, shipped and evaluated natively -- no download |

## Features

- **Control loops** for joint position/velocity, Cartesian pose/velocity, torques, and
  torques with any motion generator -- as callbacks or as `ActiveControl`
  (`read_once` / `write_once`), on **both** protocol versions.
- **Rate limiting and low-pass filtering** ported from `rate_limiting.cpp`,
  `joint_velocity_limits.cpp` and `lowpass_filter.cpp`, with the right constants picked
  from the negotiated version.
- **Target control** for low-rate commanders: `start_cartesian_target_control` /
  `start_joint_target_control` run the loop on a thread of their own and hand back a handle
  whose `set_position` / `set_joints` a planner, a socket or a keyboard can call at any
  rate; the loop turns the steps into a smooth, limit-respecting command and `stop()`
  settles and returns the loop's result.
- **Python bindings** (`crates/franka-py`, `import franka`): the target loops as context
  managers with `move_to` / `move_by` / `follow` for a policy at any rate, the 1 kHz loop
  on a Rust thread that never takes the GIL; see
  [Python](https://barisyazici.github.io/franka-rs/getting-started/python.html).
- **Online trajectory generation** (`otg`): the dependency-free, allocation-free
  jerk-limited generator underneath it, also usable on its own in a callback loop.
- **`Model`** -- pose, body and zero Jacobian for all ten frames, mass, Coriolis, gravity.
  Agrees with libfranka to ~1e-14.
- **`Gripper`** -- the Franka Hand on port 1338, byte-identical on both FCI versions.
- **Every setter**: collision behaviour, joint and Cartesian impedance, guiding mode,
  `EE`/`NE`/`K` transforms, load, plus the v5-only `set_filters` and `virtual_wall`.
- **`Robot` is `Send + Sync`** -- share it as an `Arc` and `stop()` from another thread.
- **libfranka's exact error text**, with a `FrankaError` variant per C++ exception type and
  a control log attached to every `ControlException` -- replayable as a Rerun
  [flight recording](https://barisyazici.github.io/franka-rs/howto/flight-recorder.html).
- **Cross-compiles to aarch64**, glibc or static musl.

## Quick example

```rust
    let realtime = match env::var("FRANKA_REALTIME").as_deref() {
        Ok("ignore") => RealtimeConfig::Ignore,
        _ => RealtimeConfig::Enforce,
    };
    let robot = Robot::new(&env::args().nth(1).expect("usage: <hostname>"), realtime)?;
    // `q_d`, the commanded position: FCI v5 rate-limits the first command against it.
    let initial = robot.read_once()?.q_d;
    let mut time = 0.0;
    robot.control_joint_positions(
        |_state, period| {
            time += period.as_secs_f64();
            let mut q = initial;
            q[3] += PI / 8.0 * (1.0 - (PI / 2.5 * time).cos());
            let mut output = JointPositions::new(q);
            output.motion_finished = time >= 5.0;
            output
        },
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;
```

Takes the robot's hostname from `argv[1]`, picks `RealtimeConfig` from `FRANKA_REALTIME`
(`ignore` or `enforce`, default `enforce`) like every example here, reads the commanded
joint configuration and moves joint 4 through a smooth 0-to-`pi/8`-and-back cosine ramp
over 5 seconds using the robot's internal joint impedance controller.

That block is the byte-identical body of `main` in
[`crates/franka-rs/examples/readme_joint_move.rs`](crates/franka-rs/examples/readme_joint_move.rs);
unit tests in [`crates/franka-rs/src/readme_sync.rs`](crates/franka-rs/src/readme_sync.rs)
assert that, and CI runs the example against the simulator, so neither can drift.

```toml
[dependencies]
franka-rs = "0.2"
```

For a commander that is not a 1 kHz program -- a planner, a vision loop, a script on a
socket -- the same move is `let control = robot.start_cartesian_target_control(
TargetControlOptions::default())?;`, then `control.set_position([x, y, z])?` whenever a
target comes and `control.stop()?` at the end; the loop runs on its own thread and turns
the steps into a smooth command. See
[Target control](https://barisyazici.github.io/franka-rs/howto/target-control.html).

The library is named `franka`, so `use franka::Robot;`. See
[The realtime machine](https://barisyazici.github.io/franka-rs/getting-started/realtime-machine.html)
for the realtime prerequisites (`PREEMPT_RT`, `ulimit -r`, `RealtimeConfig`,
`FRANKA_REALTIME`) and [Run the examples](https://barisyazici.github.io/franka-rs/howto/examples.html)
for the seventeen runnable examples.

### Feature flags

| Feature | Default | What it adds |
|---|---|---|
| `model-library` | yes | `Robot::load_model_from_robot()` on an FER: `LoadModelLibrary` plus `dlopen` of the shared object the robot serves. Off, the crate is pure Rust with no `dlopen`, which is what a static musl build needs -- the built-in FER model works either way. |
| `serde` | no | `Serialize` / `Deserialize` on `RobotState`, `RobotMode`, `Errors`, `Duration`, `Record`, `RobotCommandLog`, `MoveStatus` and `ControlException`, so a control log can be saved as JSON and replayed later (`crates/franka-rerun`). `Errors` serialises as the list of the set flags' names. |

## Running the tests locally

Everything CI's `check` job runs needs neither Docker, a network nor a robot:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --lib
cargo test --workspace --doc
```

The `sim_*.rs` integration tests drive a `franka-sim` container. Only one may run per host
-- the FCI ports (1337 robot, 1338 gripper) are fixed by the protocol -- so take the
repository-root lock and keep to a single test thread:

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-rs \
  --test sim_handshake --test sim_commands --test sim_motions \
  --test sim_gripper --test sim_stop_and_reflex --test sim_target_control -- --test-threads=1
```

Do **not** run `cargo test --tests`: it selects every integration binary in the workspace,
including those. Full detail in
[Contributing](https://barisyazici.github.io/franka-rs/contributing.html) and
[Testing with franka-sim](https://barisyazici.github.io/franka-rs/howto/simulator-tests.html).

## Validation

- **Hardware, against libfranka.** Model-in-the-loop torque control at 1 kHz on a real FR3
  (8 runs interleaved with libfranka 0.20.4) and on two real FERs (13 runs against libfranka
  0.9.2): no reflex, guard trip or control exception in any run; interval p99 and lost cycles
  the same for both clients within the run-to-run spread; franka-rs 2.6 percentage points
  cheaper on CPU on the FR3.
- **Simulator.** 36-run A/B matrix against `franka-sim` with alternating client order and a
  fresh container per cell: median cycle time 1000 us for both clients, no detectable p99
  difference, franka-rs cheaper on CPU in 18 of 18 paired cells.
- **The model.** Agrees with libfranka's Pinocchio backend to 4.97e-14 over 10 000 random
  FR3 states, and with a real FER's `libfcimodels_x64.so` to 4.4e-16 on kinematics and
  5e-14 on gravity.
- **Every interface on real arms.** `ActiveControl`, both Cartesian impedance examples,
  target control with translation and rotation targets, the Python policy loop and the live
  flight recorder ran on a real FER; target control and its joint-side acceleration bracket
  on a real FR3 (2026-09-04 to 2026-09-09).

The numbers and the per-run record are in the book's
[Benchmarks and hardware validation](https://barisyazici.github.io/franka-rs/reference/benchmarks.html);
the full measurement records are kept privately.

## Credit

`franka-rs` is an independent implementation, but its API shape was informed by
**[marcbone/libfranka-rs](https://github.com/marcbone/libfranka-rs)**, Marco Boneberger's
pure-Rust port of libfranka 0.9 (EUPL-1.2). No code from libfranka-rs is used, copied or derived
here; what it informed is the API shape: the `control_*` closure API, the
`Finishable` / `motion_finished` idea, the `MotionGenerator` helper and the naming of the
control types all trace back to that project. Thank you to Marco Boneberger and its
contributors.

The reference implementation is
**[`libfranka`](https://github.com/frankarobotics/libfranka)** itself (Apache-2.0), from
Franka Robotics. Wire formats, algorithms and error text here are ported and adapted from
its C++ sources -- 0.21.2 for FCI v10 and 0.9.2 for FCI v5 -- and every claim this crate
makes about the protocol is checked against it. Its copyright notice is reproduced in
[`NOTICE`](NOTICE).

`franka-rs` is an unofficial project. Franka, Franka Emika, Panda and FR3 are trademarks of
Franka Robotics GmbH; this project is not affiliated with, endorsed by or supported by Franka
Robotics GmbH, and the names are used only to identify the hardware it talks to.

## License

Licensed under the [Apache License, Version 2.0](LICENSE), the same license `libfranka`
uses.
