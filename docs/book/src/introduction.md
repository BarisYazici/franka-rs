# franka-rs

`franka-rs` is a pure-Rust client for the **Franka Control Interface** — the network
protocol `libfranka` speaks to a Franka Emika / Franka Robotics arm and its gripper. It
talks the wire protocol directly: a TCP command channel plus a UDP state/command stream at
1 kHz, and a second TCP channel for the Franka Hand.

If you know `libfranka`, you already know this crate. The module layout follows it (`wire`,
`network`, `robot`, `rate_limiting`, `model`, `gripper`), the type and function names are
its names translated to Rust conventions, and wherever it matters for interoperability the
wire formats, the error text and the numeric behaviour are the same.

## Supported robots

| FCI version | libfranka semantics | Robot | System | `RobotState` on the wire | Model source |
|---|---|---|---|---|---|
| **v10** | 0.20 – 0.21 (ported from 0.21.2) | Franka Research 3 | 5.x | 1377 B, `RobotCommand` 371 B | URDF from `GetRobotModel`, evaluated natively |
| **v5** | 0.9.2 | Franka Emika Robot (FER) | 4.2.x | 2373 B, `RobotCommand` 370 B | shipped identified parameters, evaluated natively |

The two versions differ in far more than byte layout — command numbering, the `Move`
payload, the rate-limit envelope, whether a torque-only motion generator mode exists at all.
[FER specifics](./fer.md) is the full list.

### Version auto-detection

`Robot::new` negotiates the version for you. It connects announcing library version 10; an
FER rejects that and reports version 5, and the client then opens a fresh TCP+UDP session
and connects again as version 5. `robot.fci_version()` tells you which one you got.

```rust,no_run
# extern crate franka;
use franka::{FciVersion, RealtimeConfig, Robot};

# fn main() -> franka::FrankaResult<()> {
let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;   // FR3 or FER
match robot.fci_version() {
    FciVersion::V10 => println!("FR3"),
    FciVersion::V5 => println!("FER"),
}
# Ok(()) }
```

Auto-detection costs an FER one extra connect/reject/disconnect round trip. When you know
which arm you have, pin it with `VersionPolicy::Exact` — see
[FER specifics](./fer.md#version-negotiation).

## Why not just use libfranka?

Measured side by side on the simulator and on real FR3 and FER arms, `franka-rs` and
`libfranka` are equal at the thing that matters most: the 1 kHz loop timing, the lost-cycle
behaviour and the controller's behaviour inside the safety envelope are the same, and the
model outputs agree to 1e-14. See [Benchmarks](./benchmarks.md). Pick this crate for what
`libfranka` cannot give you:

| | `libfranka` | `franka-rs` |
|---|---|---|
| dependencies | Poco, Eigen, Pinocchio, urdfdom, Boost, a C++ toolchain | one crate, `cargo build` |
| FR3 and FER | two incompatible versions (0.20+ and 0.9.2) | one crate, version negotiated at connect |
| model | Pinocchio on the FR3; a library downloaded from the FER | native on both, nothing downloaded |
| `ActiveControl` on the FER | not available | available |
| memory safety | C++ | safe Rust outside two audited `unsafe` spots |
| cross-compiling | rebuild the whole C++ stack | `rustup target add aarch64-unknown-linux-gnu` |
| `ros2_control` hardware interface | `franka_ros2` | none (C++ plugins) |

## No C++ anywhere

There is no dependency on `libfranka`, and therefore none on **Poco**, **Eigen** or
**Pinocchio**. There is no C++ toolchain in the build, no `find_package`, no system
packages: `cargo build` is the whole story, and the crate cross-compiles to
aarch64 with a single `rustup target add`.

The kinematics and dynamics that `libfranka` delegates to Pinocchio are implemented
natively in the crate (`src/model/native_backend.rs`) — forward kinematics, geometric
Jacobians, CRBA and RNEA, against the same conventions Pinocchio uses. They agree with
libfranka's answers to a few floating-point units in the last place; see
[The model](./model.md).

The one optional exception is the FER's `LoadModelLibrary` path, which downloads and
`dlopen`s the robot's own `libfcimodels.so`. That is opt-out (`--no-default-features`
drops the `model-library` feature and the `libloading` dependency) and is **not needed** —
the crate ships the FER's model parameters and evaluates them with the same native
backend.

## Relationship to libfranka

`libfranka` is the reference implementation. Every claim this crate makes about the
protocol is checked against it: struct sizes and field offsets are asserted against
libfranka's, rate limiting and low-pass filtering are ports of `rate_limiting.cpp`,
`joint_velocity_limits.cpp` and `lowpass_filter.cpp`, error strings are byte-identical, and
the model is measured against libfranka's own output on tens of thousands of samples.
Where this crate deliberately diverges — for instance refusing a `kSuccess` handshake that
announces a version the client did not ask for — it is documented as a divergence.

`libfranka` is Apache-2.0, and so is this crate.

## Credit

`franka-rs` is an independent implementation (Apache-2.0); it contains no code from
**[marcbone/libfranka-rs](https://github.com/marcbone/libfranka-rs)** (EUPL-1.2), Marco
Boneberger's Rust binding for libfranka, but its API shape was informed by it: the way a
`Robot` exposes `control_*` methods taking a closure, the `Finishable` /
`motion_finished` idea, the `MotionGenerator` helper, the naming of the control-type
structs, and the general decision of what a Rust translation of `franka::Robot` should look
like all trace back to that project. If you have used `libfranka-rs`, this crate will feel
familiar on purpose. Thank you to Marco Boneberger and its contributors.

Thanks also to **Franka Robotics** for
[`libfranka`](https://github.com/frankarobotics/libfranka) itself, which is the reference
against which everything here is checked.

## Where to go next

- [Getting started](./getting-started.md) — dependency, realtime prerequisites, first program.
- [Controlling the robot](./controlling-the-robot.md) — callbacks, `ActiveControl` and target control.
- [The model](./model.md) — what it computes and where the parameters come from.
- [Benchmarks and hardware validation](./benchmarks.md) — the numbers, on real arms.
- [API reference](./api-reference.md) — generated rustdoc for the `franka` crate.
