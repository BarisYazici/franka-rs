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
**[Changelog](CHANGELOG.md)**

## Why a Rust client

`libfranka` works well; this crate exists for the cases where it does not fit. Measured
against `libfranka` on the simulator and on real FR3 and FER arms, the control quality is
the same -- identical loop timing, identical lost-cycle behaviour, model outputs equal to
1e-14 -- so the reasons to pick `franka-rs` are elsewhere:

- **One dependency, no C++ stack.** `cargo add franka-rs` replaces `libfranka` plus Poco,
  Eigen and Pinocchio, with no system packages and no `find_package`. Static binaries and
  aarch64 cross-builds are one flag away.
- **One crate for both robot generations.** The FR3 (FCI v10) and the FER (FCI v5) need two
  incompatible `libfranka` versions; here the version is negotiated at connect and the same
  binary drives both.
- **Memory safety at the trust boundary.** The parser, the demux and the 1 kHz loop are safe
  Rust with no allocation per cycle; a malformed datagram becomes an error value. The only
  `unsafe` is two audited spots: the optional robot-served model loader and two socket calls.
- **No model download.** The FR3 model comes from the robot's URDF and the FER model from
  shipped parameters, both evaluated natively; nothing is fetched or `dlopen`ed at runtime.
- **`ActiveControl` on the FER.** `read_once`/`write_once` control is available on the older
  generation too, which `libfranka` 0.9.2 does not offer.
- **Rust ergonomics.** `Robot` is `Send + Sync` and stops from another thread through an
  `Arc`; errors are an enum, not exceptions; the whole thing compiles in seconds.

What it does **not** give you: better timing than `libfranka` (they are equal) or a
`ros2_control` hardware interface (those are C++ plugins).

## Supported robots

| FCI | libfranka semantics | Robot | System | `RobotState` | Model |
|---|---|---|---|---|---|
| **v10** | 0.20 - 0.21 | Franka Research 3 | 5.x | 1377 B | URDF from `GetRobotModel`, evaluated natively |
| **v5** | 0.9.2 | Franka Emika Robot (FER) | 4.2.x | 2373 B | parameters identified from the robot's own `libfcimodels.so`, shipped and evaluated natively -- no download |

## Features

- **Control loops** for joint position/velocity, Cartesian pose/velocity, torques, and
  torques with any motion generator -- as callbacks or as `ActiveControl`
  (`read_once` / `write_once`), on **both** protocol versions.
- **Rate limiting and low-pass filtering** ported from `rate_limiting.cpp`,
  `joint_velocity_limits.cpp` and `lowpass_filter.cpp`, with the right constants picked
  from the negotiated version.
- **Online trajectory generation** (`otg`): a dependency-free, allocation-free
  jerk-limited generator that turns stepped, bursty, stalled targets into a C2, limit-respecting
  1 kHz command; the bridge in the `nonrealtime_commander` example.
- **`Model`** -- pose, body and zero Jacobian for all ten frames, mass, Coriolis, gravity.
  Agrees with libfranka to ~1e-14.
- **`Gripper`** -- the Franka Hand on port 1338, byte-identical on both FCI versions.
- **Every setter**: collision behaviour, joint and Cartesian impedance, guiding mode,
  `EE`/`NE`/`K` transforms, load, plus the v5-only `set_filters` and `virtual_wall`.
- **`Robot` is `Send + Sync`** -- share it as an `Arc` and `stop()` from another thread.
- **libfranka's exact error text**, with a `FrankaError` variant per C++ exception type and
  a control log attached to every `ControlException` -- replayable as a Rerun
  [flight recording](https://barisyazici.github.io/franka-rs/flight-recorder.html).
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
franka-rs = "0.1"
```

The library is named `franka`, so `use franka::Robot;`. See
[Getting started](https://barisyazici.github.io/franka-rs/getting-started.html) for the
realtime prerequisites (`PREEMPT_RT`, `ulimit -r`, `RealtimeConfig`, `FRANKA_REALTIME`) and
for the fifteen runnable
[examples](https://barisyazici.github.io/franka-rs/getting-started.html#the-examples).

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
  --test sim_gripper --test sim_stop_and_reflex -- --test-threads=1
```

Do **not** run `cargo test --tests`: it selects every integration binary in the workspace,
including those. Full detail in
[Contributing](https://barisyazici.github.io/franka-rs/contributing.html) and
[Testing with franka-sim](https://barisyazici.github.io/franka-rs/testing-with-franka-sim.html).

## Validation

- **FR3 hardware.** Model-in-the-loop torque control at 1 kHz against a real FR3, 8 runs,
  interleaved with libfranka 0.20.4: **0 reflexes, 0 guard trips, 0 control exceptions**,
  interval p99 1079-1106 us for both clients, franka-rs 10.7 us cheaper per cycle in the
  model region and 2.6 percentage points cheaper on CPU (paired, 3/3 repetitions).
- **FER hardware.** Thirteen 30 s runs across two real FERs (system 4.2.1) against
  libfranka 0.9.2, with **no reflex, guard trip or control exception**. franka-rs holds the
  deadline as well as libfranka -- interval p99 1171.7 vs 1186.8 us, lost cycles 27.5 vs
  29.5 per 30 000 -- and the two agree on every model quantity to 1e-14 on the robot.
- **Dual-arm and `ActiveControl`.** Both FERs driven at once, as two processes and as one
  process with a `Robot` per thread, with no cross-interference; and `read_once`/`write_once`
  exercised on a real FER, matching the callback API within noise -- a control style
  libfranka 0.9.2 does not offer for that robot generation.
- **Cartesian impedance examples on hardware.** `cartesian_impedance_active_control` and
  `cartesian_impedance_figure_eight` ran on a real FER through `ActiveControl` (2026-09-07),
  holding and tracking the pose while being pushed, with no reflex.
- **Cartesian pose bridging and the flight recorder on hardware (2026-09-08).**
  `nonrealtime_commander` ran its full 19 s bridged sequence on a real FER with no reflex
  (peak commanded speed 0.25 m/s, measured pose within a few millimetres of the command),
  and its raw mode was refused at the first step and cleared with
  `automatic_error_recovery()`; the runs also showed that the robot checks the joint-space
  continuity of a Cartesian pose stream, which the client-side rate limiter does not bound
  (documented in the book). `reflex_replay`'s live `Recorder` pushed 23 941 records at 1 kHz
  with none dropped, and on every logged cycle the native FER model's end effector matched
  the measured `O_T_EE` to under 0.01 mm.
- **Simulator.** 36-run A/B matrix against `franka-sim` with alternating client order and a
  fresh container per cell: median cycle time 1000 us for both clients, no detectable p99
  difference, and franka-rs cheaper on CPU in 18 of 18 paired cells.
- **The model.** Agrees with libfranka's Pinocchio backend to **4.97e-14** over 10 000
  random FR3 states, and with a real FER's `libfcimodels_x64.so` to 4.4e-16 on kinematics
  and 5e-14 on gravity. Measuring it also found a per-call `pinocchio::Data` allocation in
  libfranka's kinematics path; the two-commit fix (see `patches/`) takes it from 143 heap
  allocations per call to zero.

Full measurement records for both campaigns are kept privately; the summary is in the
[book](https://barisyazici.github.io/franka-rs/benchmarks.html).

## Credit

`franka-rs` is an independent implementation, but its API shape was informed by
**[marcbone/libfranka-rs](https://github.com/marcbone/libfranka-rs)**, Marco Boneberger's
Rust binding for libfranka (EUPL-1.2). No code from libfranka-rs is used, copied or derived
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
