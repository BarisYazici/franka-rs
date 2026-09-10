# Compared with libfranka

[libfranka](https://github.com/frankarobotics/libfranka) is Franka Robotics' own C++
client and the reference this crate is checked against. This page is for someone choosing
between the two. Everything on it is either read from the two source trees or measured;
the measurements are on [Benchmarks and hardware validation](./benchmarks.md).

## Where the two are the same

- **Protocol and semantics.** Struct sizes and field offsets are asserted against
  libfranka's headers for both FCI versions; the rate limiter and the low-pass filter are
  ports of `rate_limiting.cpp`, `joint_velocity_limits.cpp` and `lowpass_filter.cpp`; the
  error strings are byte-identical, `"libfranka: "` prefix included; the control log holds
  the same 50 cycles by default. A motion refused by the robot fails the same way from
  both clients.
- **Loop timing.** Measured back to back on the same PC against franka-sim, a real FR3 and
  two real FERs: median cycle time 1000 µs for both, p99 within the run-to-run spread,
  lost cycles the same. Neither client is better at holding the 1 ms deadline. franka-rs
  used slightly less CPU (0.6 percentage points on the simulator, 2.6 on the FR3 in the
  model-in-the-loop variant); part of the FR3 gap is a per-call allocation in libfranka's
  kinematics path, for which `patches/` holds a fix that closes most of it.
- **The model.** Poses, Jacobians, mass, Coriolis and gravity agree with libfranka's
  Pinocchio backend to 5e-14 on the FR3 and with a real FER's `libfcimodels.so` to 5e-14
  on gravity and 4e-16 on kinematics. Details on [Model parameters and
  conformance](./model.md).

## What franka-rs has and libfranka does not

- **One client for both robot generations.** libfranka 0.18 and later speak FCI v10 (FR3
  on system 5.9.0 or later; this crate ports the 0.21.2 semantics); 0.9.2 speaks FCI v5
  (Panda); the two are incompatible and a program is built against one. Here `Robot::new`
  negotiates the version and the same binary drives both.
- **Target control.** libfranka gives you the 1 kHz callback and, on the FR3, the
  `ActiveControl` loop; since 0.18 it also has an asynchronous joint-position mode in which
  the robot itself moves point to point under maximum velocities (marked experimental,
  FR3 only, exposed in `pylibfranka`). Bridging a stream of Cartesian or joint targets
  onto the 1 kHz loop is otherwise left to the user or to software built on top of
  libfranka. Here `start_cartesian_target_control` / `start_joint_target_control` run the
  loop on a realtime thread, take Cartesian poses or joint targets at any rate on both
  robot generations, and the Python bindings are built on it.
- **`ActiveControl` on an FER.** libfranka 0.9.2 has only the callback API for that
  generation.
- **The model without a download.** libfranka evaluates the FR3 model with Pinocchio and,
  on an FER, downloads and `dlopen`s the robot's `libfcimodels.so`. Here both are
  evaluated natively from parameters; nothing is fetched at runtime unless you call the
  opt-in `load_model_from_robot()`.
- **Build footprint.** No C++ toolchain, no Poco, Eigen, Pinocchio or urdfdom, no system
  packages; `rustup target add` cross-compiles it.
- **Rust things.** `Robot` is `Send + Sync` and stops from another thread through an
  `Arc`; errors are one enum with a variant per libfranka exception; a `serde` feature
  makes the control log serialisable; `crates/franka-rerun` replays it in Rerun.

## What libfranka has and franka-rs does not

- **Vendor support.** libfranka is maintained by Franka Robotics, versioned against the
  robot system software, and is what Franka's documentation describes. This crate is an
  unofficial project.
- **`franka_ros2` and the `ros2_control` hardware interface.** Those are C++ plugins on
  libfranka; there is no ROS 2 integration here.
- **`VacuumGripper`.** Not implemented; only the Franka Hand is.
- **History.** libfranka's history starts in January 2017 and it has driven Franka arms
  since the Panda shipped. This crate's hardware record is the campaigns and runs listed
  in the README, all from September 2026.

## Deliberate differences

- A `kSuccess` handshake that reports a different version than the client announced is an
  `IncompatibleVersion` error here. libfranka only inspects the status byte; everything
  after the handshake is decoded against the announced version, so such a session would
  be unusable.
- `limit_rate` and `cutoff_frequency` are explicit arguments of every `control_*` method
  because Rust has no default arguments, and libfranka's defaults differ between the two
  versions (`limit_rate` is `false` in 0.21 and `true` in 0.9.2). See [Rate limiting and
  filtering](./rate-limiting.md).
- `control_*` never fails with `FrankaError::Realtime`; the priority is raised in
  `Robot::new`, as in libfranka's `Robot::Impl` constructor, so the question is settled
  before a loop starts.

## Other clients

Other Rust implementations exist. Marco Boneberger's
[libfranka-rs](https://github.com/marcbone/libfranka-rs) (2021, EUPL-1.2) is a pure-Rust
port of libfranka 0.9 for the Panda, FCI v5 only, and the project that informed this
crate's API shape; [franka_rust](https://crates.io/crates/franka_rust) is a separate Rust
FCI driver whose README lists Panda and FR3 types, not evaluated here. On the Python side,
Franka's own `pylibfranka` (libfranka 0.16 and later),
[franky](https://github.com/TimSchneider42/franky) and
[panda-py](https://github.com/JeanElsner/panda-py) all wrap libfranka: franky and panda-py
run a C++ control thread with online trajectory generation, so "send a target any time from
Python" is available there too. What they inherit from libfranka is one protocol version
per install and an x86-64 C++ build; that, not the interface, is the difference to this
crate. Surveyed 2026-09-10.
