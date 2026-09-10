# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-09-09

### Added

- **Target control** (`robot::target_control`): `Robot::start_cartesian_target_control` and
  `Robot::start_joint_target_control` run the crate's control loop on a named thread of
  their own and return a `CartesianTargetControl` / `JointTargetControl` handle whose
  `set_position([f64; 3])` / `set_joints([f64; 7])` any low-rate commander can call from any
  thread at any rate; the loop bridges the steps with the online trajectory generator under
  the three rules of the `otg` module (per-axis limits from the norm budget, one nominal
  `DELTA_T` per command, re-anchoring on the robot's echo), the rate limiter under the same
  budget as the backstop, a measured-deviation guard and a settle-then-finish `stop()` that
  returns the loop's result. `TargetControlOptions` / `JointTargetControlOptions` carry the
  budget (joint default: 20 % of the negotiated version's limits), the controller mode, the
  guard, the settle criterion, an optional `SCHED_FIFO` priority for the loop thread and an
  observer called every cycle on the realtime thread with what was sent (the flight
  recorder's hook). `TargetSlot<N>` is the seqlock underneath, public. `MultiOtg::with_limits`
  builds a generator with per-axis limits and `realtime::set_current_thread_scheduler_priority`
  raises a thread to a chosen priority. Tested on franka-sim
  (`tests/sim_target_control.rs`) and run on a real FER and an FR3.
- **Cartesian target control carries an orientation.** `CartesianTargetControl::set_pose`
  (column-major, as `O_T_EE`; a rotation block within 1e-3 of orthonormal is repaired, one
  further off refused), `set_target(position, quaternion)` and `set_orientation(quaternion)`
  with unit quaternions in `[x, y, z, w]` order, `target_orientation()` and `target_pose()`;
  `set_position` keeps the target orientation. The orientation runs on three more axes of
  the same synchronised generator, on the base-frame rotation vector of the orientation
  error re-anchored on the echo every cycle, under `TargetControlOptions::rotation_limits`
  (0.5 rad/s, 1.0 rad/s², 20 rad/s³ by default) with `with_rotation_limits`, and an angular
  deviation guard `max_angular_deviation` (0.5 rad). `CartesianSent` gains the sent
  orientation, the angular velocity and acceleration and the rotational backstop alteration.
  The Cartesian backstop now references the twist and acceleration it sent rather than the
  echoed ones, whose float32 rounding had its jerk clamp firing at noise level (on the
  rotation that was an orbit of 5 mrad around the target), and `REST_VELOCITY` /
  `REST_ACCELERATION` drop to 1e-4 and 0.05 so the hold's freeze stays under the joint-side
  jerk limits. `OtgLimits::scaled` is new. The
  commander example's `--rotate` adds a slow yaw sweep of ±15° (bridged mode only).
- **`otg` module**: an online trajectory generator (`Otg`, `MultiOtg<N>`, `CartesianOtg`,
  `OtgLimits`) that re-plans a time-optimal, jerk-limited seven-segment profile every cycle
  from the commanded state to rest at the latest target, so a stream of stepped, bursty or
  stalled targets becomes a C2 command that never exceeds its velocity, acceleration and jerk
  limits, does not overshoot a reachable target, lands exactly and stays there; optional axis
  synchronisation, `set_position` to re-anchor on the robot's echo, `per_axis_for_norm` for
  a budget that is a norm. Dependency-free and allocation-free. Its first outing on a real FER
  ended in the rate limiter behind it orbiting at the velocity cap, which the module
  documentation explains and two replay tests pin down.
- **`nonrealtime_commander` example**: a scripted (or stdin) commander sets Cartesian
  targets through `start_cartesian_target_control` (`--bridged`, `--budget V,A,J`), or
  hands them to a bare `control_cartesian_pose` to provoke a reflex (`--raw`); `--log`
  writes one CSV row per cycle, from the loop's observer, with the joint angles, the
  external wrench and the generator's velocity and acceleration. The budget exists because
  the robot also checks the joint-space continuity of a Cartesian pose stream, which the
  rate limiter does not bound; see
  [Target control](docs/book/src/howto/target-control.md).
  `franka-rerun`'s `commander_live` example is on the same API, with the `Recorder` in the
  observer.
- **Python bindings** (`crates/franka-py`, `import franka`; `pip install franka-rs`, or
  `maturin develop` from the source tree): `Robot`, `RobotState` (numpy fields and a
  69-float `flat()` observation), `robot.cartesian_targets()` / `robot.joint_targets()` as
  context managers over the target control loops with `move_to`, `move_by`,
  `follow(chunk, dt)`, `target()`, `state()` and `stop()`, `Gripper`, `Model` over numpy
  (`robot.model()`), `franka.rotated`, and `FrankaError` / `ControlException`. A Cartesian
  target carries an optional unit quaternion, a delta an optional rotation vector. The
  1 kHz loop stays on its Rust thread and never takes the GIL. PyO3 0.29, abi3 for
  Python 3.9+; tested against franka-sim in CI's `python-bindings` job and run on a Panda
  (`crates/franka-py/examples/policy_loop.py`; `rotate.py` and the `quickstart.ipynb`
  notebook are the other two examples). See [Python](docs/book/src/getting-started/python.md).
  `.github/workflows/release.yml` builds the wheels and publishes them and the crate on a
  `v*` tag.
- **`automatic_error_recovery` example**: command-line recovery that prints the robot mode
  before and after. **`move_to_ready` example**: the examples' motion generator to
  libfranka's ready pose at a fraction of full speed.
- **`serde` feature** (off by default): `Serialize` / `Deserialize` for `RobotState`,
  `RobotMode`, `Errors`, `Duration`, `Record`, `RobotCommandLog`, `MoveStatus` and
  `ControlException`. `Errors` serialises as the list of the set flags' names.
- **`franka-rerun`**, a workspace crate (`publish = false`, Rust 1.96) that replays logs in
  Rerun: `csv` turns the commander example's log into target, commanded and measured
  positions, the command's derivatives against the rate limits and a 3D replay of the arm;
  `log` replays a saved `ControlException` control log as a flight recording (contact and
  collision flags, external wrench, commanded versus measured, errors, the arm), through
  `flight::{log_records, replay_exception, save_records, load_records}`; and `Recorder`
  streams the same live from inside a control loop with a non-blocking, non-allocating
  `push`. `examples/reflex_replay.rs` puts the last two together; `examples/commander_live.rs`
  streams the commander into a viewer. Both replays draw Franka's link meshes with
  `--meshes DIR`, `csv` has the screen-capture `--layout demo` and the commander's `--budget`
  lines, and `log` locates a contact on the arm from the external joint torques
  (`flight::contact`). See [the flight recorder page](docs/book/src/howto/flight-recorder.md).

## [0.1.0] - 2026-09-07

Initial release. `franka-rs` is a pure-Rust libfranka client speaking both
generations of the Franka Control Interface: FCI v10 (Franka Research 3,
ported from libfranka 0.21.2) and FCI v5 (Franka Emika Robot, FER, ported from
libfranka 0.9.2).

There is no earlier release to compare against, so "Added" describes the
release itself and "Changed" records the places where this crate deliberately
differs from libfranka.

### Added

- **Franka Emika Robot (FER) support (FCI v5)**, alongside the FR3 (FCI v10); see
  [the FER specifics page](docs/book/src/reference/fer.md). `Robot::new` negotiates the version
  automatically (connects announcing v10, retries as v5 if the robot reports
  `kIncompatibleLibraryVersion`); `RobotOptions::with_version(VersionPolicy::
  Exact(FciVersion::V5))` skips the extra round trip when the generation is
  already known. New public API: `FciVersion`, `VersionPolicy`,
  `RobotOptions`, `VirtualWallCuboid`, `Robot::{with_options, fci_version,
  set_filters, virtual_wall}`, `Model::{from_model_library_bytes,
  from_model_library_path}`, `model::{load_from_robot, SoModelBackend,
  model_library, so_backend}`, `RobotState::from_wire_v5`,
  `rate_limiting::fer` (selected automatically from the negotiated
  version), `wire::robot::{v5, codec}`. New example `fer_joint_impedance`.
- **The FER model no longer needs a download.** `Robot::load_model()` on an
  FER now returns the *native* model — `franka::model::FER_URDF`,
  evaluated by the same backend the FR3 uses — instead of downloading and
  `dlopen`ing the robot's shared object. Needs no network, no
  `model-library` feature and no x86-64 Linux host; also reachable without a
  robot as `Model::native_fer()`. Its link inertial parameters were
  identified from a real FER's own model library by the new
  `tools/fer-model-fit` (a regressor from the crate's own dynamics
  backend, solved with a truncated SVD), agreeing with the library to
  9e-16 on kinematics and 4e-14 on dynamics with no payload, and
  characterising the one known gap, a payload non-linearity in the library
  itself (see [the model page](docs/book/src/reference/model.md)).
  `Robot::load_model_from_robot()` keeps the previous behaviour (download +
  `dlopen` on v5, unchanged `GetRobotModel` URDF on v10). The native model
  is checked in CI via `tests/fer_native_conformance.rs` against a
  committed fixture, with no robot or shared object present.
- **aarch64 builds**, via `cargo-zigbuild` or a cross `gcc`, or as a fully static
  `aarch64-unknown-linux-musl` binary with `--no-default-features` (a static
  binary can't `dlopen`, and the FER's model path is x86-64-only anyway).
  CI cross-builds for `aarch64-unknown-linux-gnu` on every push; verified
  under `qemu-aarch64` against both simulators, identical to native.
- **New example `cartesian_impedance_active_control`** — libfranka's
  `cartesian_impedance_control.cpp` (a spring-damper system whose equilibrium is
  the initial end-effector pose) driven through the `ActiveControl` API's
  `read_once` / `write_once` instead of a control callback, allocation-free and
  running unchanged on an FR3 (FCI v10) and an FER (FCI v5).
- **New example `cartesian_impedance_figure_eight`** — the same `ActiveControl`
  impedance loop with a moving equilibrium: a Lissajous figure eight around the
  start pose, raised-cosine ramps on both the path amplitude and the stiffness at
  either end (and after Ctrl-C), a nullspace joint spring toward the initial
  configuration through a damped-inverse projector, a one-sided virtual floor,
  a ±25 Nm torque clamp and a deviation cut-out. Allocation-free, on both
  generations. The pieces it shares with `cartesian_impedance_active_control`
  moved to `examples/common/cartesian.rs`.
- **New example `dual_communication_test`** — the zero-torque
  communication-test loop against two robots at once from one process, with
  per-robot loop timing, lost-state and success-rate accounting.
- **`bench/fer-capture/`** — offline tooling that reassembles the FCI v5
  state datagrams from a pcap/pcapng capture and reports inter-arrival gaps,
  drift and a stall verdict, to tell whether an FER loop's lost cycles come
  from the robot/network or the host.
- **`bench/so-micro/`** — an offline per-call microbenchmark of the FCI v5
  model path, Rust vs C++, against the same captured shared object, needing
  no robot or simulator.

### Changed

- `CommandName::command()` is now `CommandName::command(self, version:
  FciVersion) -> Option<u32>`, since the TCP command numbering differs
  between the two protocol versions (`None` means the command does not
  exist in that version); `CommandName` gained two v5-only variants,
  `GetCartesianLimit` and `SetFilters`.
- `SoModelBackend::open`, `SoModelBackend::from_bytes`,
  `Model::from_model_library_bytes` and `Model::from_model_library_path` are
  now `unsafe fn`: they `dlopen` the file or bytes given to them, so the
  caller must assert it is a trusted model library. `Robot::load_model()`
  stays a safe `fn`, as in libfranka.
- A `Connect` reply with status `kSuccess` reporting a version other than
  the one the client announced is now `FrankaError::IncompatibleVersion`
  rather than being decoded against the wrong layout — what lets automatic
  version negotiation recognise an FER on the simulator; no effect against
  a real FR3 or FER.

[0.2.0]: https://github.com/BarisYazici/franka-rs/releases/tag/v0.2.0
[0.1.0]: https://github.com/BarisYazici/franka-rs/releases/tag/v0.1.0
