# FCI v10 and FCI v5 on the wire

The crate speaks two versions of the Franka Control Interface. This page is the byte-level
comparison and the negotiation between them. What the difference means for a program on a
Franka Emika Robot is on [FER / Panda specifics](./fer.md); the rate-limit envelopes are on
[Rate limiting and filtering](./rate-limiting.md).

## Supported robots

| FCI version | libfranka semantics | Robot | System | `RobotState` | `RobotCommand` | Model source |
|---|---|---|---|---|---|---|
| **v10** | 0.20 – 0.21 (ported from 0.21.2) | Franka Research 3 | 5.9.0 or later (server version 10) | 1377 B | 371 B | URDF from `GetRobotModel`, evaluated natively |
| **v5** | 0.9.2 | Franka Emika Robot / Panda (FER) | 4.2.1 or later (server version 5) | 2373 B | 370 B | shipped identified parameters, evaluated natively |

Both versions share the framing: a TCP command channel on port 1337 with a 12-byte
`CommandHeader` (command, command id, total size) and `u8` status bytes, and a UDP channel on
which the robot sends one `RobotState` per millisecond and receives one `RobotCommand` per
state while a motion runs. The Franka Hand has its own protocol on port 1338 (10-byte header,
`u16` statuses), version 3 and byte-identical on both robots.

## Version negotiation

`Robot::new` connects announcing library version 10. An FR3 accepts. An FER answers
`kIncompatibleLibraryVersion` and reports version 5; the client then closes both sockets,
opens a fresh TCP+UDP session and connects again as version 5. `robot.fci_version()` says
which one you got:

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

Auto-detection costs an FER one full connect, a rejected handshake and a disconnect, i.e. two
FCI client slots in sequence. A supervisor that reconnects in a loop, or any program that
knows its arm, pins the version with `VersionPolicy::Exact`:

```rust,no_run
# extern crate franka;
use franka::{FciVersion, RealtimeConfig, Robot, RobotOptions, VersionPolicy};

# fn main() -> franka::FrankaResult<()> {
let robot = Robot::with_options(
    "172.16.0.2",
    RobotOptions::new(RealtimeConfig::Enforce).with_version(VersionPolicy::Exact(FciVersion::V5)),
)?;
# Ok(()) }
```

`VersionPolicy::Exact(FciVersion::V10)` against an FER never retries and fails with
`FrankaError::IncompatibleVersion { server_version: 5, library_version: 10 }`, which is what
a v10 libfranka reports when pointed at an FER. Any server version other than 5 or 10 is
returned as the same error.

One deliberate divergence from libfranka, which only inspects the handshake's status byte: a
`Connect` reply of `kSuccess` that reports a version other than the one the client announced
is refused with `IncompatibleVersion`. Everything after the handshake is decoded against the
announced version, so such a session would fail on the first state with
`Protocol("libfranka: incorrect object size")` instead of naming the problem. franka-sim's
FER build answers `kSuccess` with version 5 whatever the client announced, and this rule is
what lets auto-detection recognise it.

After the handshake, an FCI v10 session fetches the URDF (`GetRobotModel`) and derives the
position-dependent joint velocity limits from it, as libfranka 0.21.2 does; a v5 session
stops after the first state, as 0.9.2 does.

## What differs on the wire

| | FCI v5 (FER) | FCI v10 (FR3) |
|---|---|---|
| libfranka | 0.9.2 | 0.21.2 |
| `RobotState` datagram | **2373 bytes**, every field `f64`, no accelerometer arrays | 1377 bytes, mixed `f64`/`f32` |
| `RobotCommand` datagram | **370 bytes**: `MotionGeneratorCommand` 306 + `ControllerCommand` 56 (`tau_J_d` alone) | 371 bytes: `ControllerCommand` 57, also carries `torque_command_finished` |
| `tau_J_d` in the state | `f64` | `f32` |
| command numbering | `Connect` 0, `Move` 1, `StopMove` 2, `GetCartesianLimit` 3, `SetCollisionBehavior` 4, `SetJointImpedance` 5, `SetCartesianImpedance` 6, `SetGuidingMode` 7, `SetEEToK` 8, `SetNEToEE` 9, `SetLoad` 10, `SetFilters` 11, `AutomaticErrorRecovery` 12, `LoadModelLibrary` 13 | `Connect` 0, `Move` 1, `StopMove` 2, `SetCollisionBehavior` 3, `SetJointImpedance` 4, `SetCartesianImpedance` 5, `SetGuidingMode` 6, `SetEEToK` 7, `SetNEToEE` 8, `SetLoad` 9, `AutomaticErrorRecovery` 10, `GetRobotModel` 11 |
| `Move` request | 56 bytes | 113 bytes: adds `use_async_motion_generator` and seven `maximum_velocity` values, always transmitted |
| `Move` generator modes | four, **no `None`** | the same four plus `None` (4) |
| `Move::Status` | 10 values | 12: `PreemptedDueToActivatedSafetyFunctions` (3) and `CommandRejectedDueToActivatedSafetyFunctions` (4) inserted after `Preempted` (2), so `ReflexAborted` is 6 on v5 and 8 on v10 |
| torque-only control | joint-velocity generator commanding zeros | `MotionGeneratorMode::None` |
| `SetFilters` | yes (`Robot::set_filters`) | removed from the protocol |
| `GetCartesianLimit` | yes (`Robot::virtual_wall`) | removed from the protocol |
| `GetRobotModel` (URDF) | **no such command** | yes (`Robot::robot_model`) |
| `LoadModelLibrary` | yes (`Robot::load_model_from_robot`) | **no such command** |
| joint velocity limits | flat: 2.175 rad/s (J1–4), 2.610 rad/s (J5–7) | position-dependent envelope from the URDF |
| `kTolNumberPacketsLost` | 3.0 | 0.0 |
| joint position limits (`JOINT_POSITION_LIMITS`, from the robots' URDFs; the FR3 row is the robot URDF's hard limits, a few mrad wider than franka_description's datasheet values) | J1 ±2.8973, J2 ±1.7628, J3 ±2.8973, J4 [−3.0718, −0.0698], J5 ±2.8973, J6 [−0.0175, 3.7525], J7 ±2.8973 | J1 ±2.7501, J2 ±1.7918, J3 ±2.9065, J4 [−3.0481, −0.1458], J5 ±2.8101, J6 [0.5409, 4.5205], J7 ±3.0196 |
| gripper protocol | **identical** (version 3, port 1338) | identical |

The status enums other than `Move::Status` are shorter on v5 in the same way (no
safety-function values); the crate maps every v5 byte onto the v10 name set by name, and a
byte that is not valid for the negotiated version is
`FrankaError::Protocol("libfranka: Unexpected response while handling <name> command!")`.

## Commands a version does not have

Calling one fails client-side, before a byte is sent, with libfranka's own text:

```text
FrankaError::InvalidOperation("libfranka: Get Robot Model is not available on FCI version 5.")
FrankaError::InvalidOperation("libfranka: Set Filters is not available on FCI version 10.")
```

Two public items carry the version: `CommandName::command(version) -> Option<u32>` is the
wire id of a command under a version (`None` where it does not exist), and the FER's
rate-limit constants live only at `franka::rate_limiting::fer::*`, with the FR3's at the
crate root.

## Two IP fragments per state

The v5 state datagram, 2373 bytes, exceeds the 1500-byte Ethernet MTU, so every FER state
arrives as exactly two IP fragments: 2000 packets per second for the host to service instead
of 1000. The FER hardware campaign saw no reassembly failures or drops
(`ReasmFails`, `ReasmTimeout`, `Udp.InErrors` and NIC drops all zero); the doubled packet
rate is the most plausible reason the FER lost occasional cycles where the FR3 lost none,
as a load effect. See [Benchmarks](./benchmarks.md).

## How the layout is pinned

Every wire struct is `#[repr(C, packed)]`, and its size and every field offset are asserted
against the declarations in libfranka's `service_types.h`, `rbk_types.h` and the gripper's
`types.h`: `crates/franka-rs/tests/wire_sizes.rs` for FCI v10 (libfranka 0.21.2, including
the 23-byte gripper state) and `tests/wire_sizes_v5.rs` for FCI v5 (libfranka 0.9.2,
including the command numbering and the status tables). Both run offline, in CI's `check`
job. The mock-FCI unit tests under `robot/mock_tests` additionally pin the v5 `Move` payload,
the torque-only datagrams and the `LoadModelLibrary` request byte for byte.

Wire parsing, the state demultiplexer, the rate limiter, the trajectory generator and the
control loops are safe Rust; the crate's `unsafe` is confined to the libc scheduler-priority
calls in `realtime.rs`, the libc socket calls in `network/udp.rs` and `network/tcp.rs`, and
the opt-in `dlopen` model loader (`model/model_library.rs`, `model/so_backend/`).
