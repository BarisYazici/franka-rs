# FER specifics (FCI v5)

Everything in the rest of this book applies to a Franka Emika Robot (FER). This page is the list of places
where the robot, not the API, is different. A more exhaustive write-up — with the
wire-level detail and the simulator characterisation — is kept privately, outside this
repository.

An FER is **Franka system 4.2.x**, **FCI version 5**, and the reference client is
**libfranka 0.9.2**.

## Version negotiation

`Robot::new` connects announcing version 10, gets rejected with
`kIncompatibleLibraryVersion` and a reported version of 5, then opens a *fresh* TCP+UDP
session and connects again as version 5. That works, but it costs a full connect, a
rejected handshake and a disconnect — i.e. two FCI client slots in sequence. In a
supervisor that reconnects in a loop, skip it:

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
`FrankaError::IncompatibleVersion { server_version: 5, library_version: 10 }` — exactly
what a v10 libfranka reports when pointed at an FER.

## What differs on the wire

| | FCI v5 (FER) | FCI v10 (FR3) |
|---|---|---|
| libfranka | 0.9.2 | 0.21.2 |
| `RobotState` datagram | **2373 bytes**, every field `f64`, **no accelerometer arrays** | 1377 bytes, mixed `f64`/`f32` |
| `RobotCommand` datagram | **370 bytes**; `ControllerCommand` is `tau_J_d` alone | 371 bytes; also carries `torque_command_finished` |
| `tau_J_d` in the state | `f64` | `f32` |
| command numbering | `SetCollisionBehavior` is 4, … `LoadModelLibrary` 13 | different: `SetCollisionBehavior` is 5, no `GetCartesianLimit`/`SetFilters`, plus `GetRobotModel` |
| `Move` request | 56 bytes | 113 bytes (also `async` and maximum velocities) |
| `Move` generator modes | four — **no `None`** | the same four plus `None` |
| `Move::Status` | 10 values, `Preempted = 2` | 12 values, `Preempted = 4` |
| torque-only control | joint-velocity generator commanding zeros | `MotionGeneratorMode::None` |
| `SetFilters` | yes (`Robot::set_filters`) | removed from the protocol |
| `GetCartesianLimit` | yes (`Robot::virtual_wall`) | removed from the protocol |
| `GetRobotModel` (URDF) | **no such command** | yes (`Robot::robot_model`) |
| `LoadModelLibrary` | yes (`Robot::load_model_from_robot`) | **no such command** |
| joint velocity limits | flat: 2.175 rad/s (J1–4), 2.610 rad/s (J5–7) | position-dependent envelope from the URDF |
| `kTolNumberPacketsLost` | 3.0 | 0.0 |
| joint limits | J1 ±2.8973, J2 ±1.7628, J3 ±2.8973, J4 [−3.0718, −0.0698], J5 ±2.8973, J6 [−0.0175, 3.7525], J7 ±2.8973 | different (J1 ±2.7437, …) |
| gripper protocol | **identical** (version 3, port 1338) | identical |

Calling a command the negotiated version does not have fails client-side, before a byte is
sent, with libfranka's own text:

```text
FrankaError::InvalidOperation("libfranka: Get Robot Model is not available on FCI version 5.")
FrankaError::InvalidOperation("libfranka: Set Filters is not available on FCI version 10.")
```

Two public items changed shape when the second version arrived, because neither is
meaningful without one: `CommandName::command()` became
`CommandName::command(version) -> Option<u32>`, and the FER's rate-limit constants live
only at `franka::rate_limiting::fer::*` rather than at the crate root.

## Torque control on FCI v5

libfranka 0.9.2 has no `MotionGeneratorMode::kNone`, and its `finishMotion` always requires
a motion-generator command. So `franka::Robot::control(ControlCallback)` runs a
**joint-velocity motion generator commanding zero velocity** next to the external
controller. This crate reproduces that exactly, so on an FER:

- `control_torques` and `start_torque_control` send
  `Move { controller_mode: ExternalController, motion_generator_mode: JointVelocity }`;
- every 370-byte command datagram carries your `tau_J_d` **and** a zero `dq_c`, which goes
  through the same low-pass filter and rate limiter a user's velocity would;
- the motion ends with `motion_generation_finished` — a v5 `ControllerCommand` has no
  `torque_command_finished` field to set;
- the two callbacks are evaluated in 0.9.2's order (motion first, controller
  short-circuited) rather than 0.21.2's.

None of that reaches the public API.
`control_torques_and_joint_positions` and friends behave the same on both versions: there
you supply the motion generator, so nothing is substituted.

## The `q_d` seed rule

**Seed a motion from `state.q_d`, not `state.q`.**

On an FER the *first* setpoint of a motion is rate limited against the robot's own `q_d` /
`O_T_EE_c` like every other one, because libfranka 0.9.2's `convertMotion` has no
`initialized_filter_` special case for the first command. On an FR3 the first command is
treated differently. Seed from `q` and you start one tracking-error away from what the
robot is commanding, and the limiter has to absorb that step — which at best distorts the
start of your motion and at worst saturates.

This is why the README's joint-move example reads `robot.read_once()?.q_d` rather than `.q`.
The same defect, in the examples' shared `MotionGenerator`, was found and fixed during the
2026-09-05 hardware campaign.

## v5-only commands

```rust,no_run
# extern crate franka;
# use franka::Robot;
# fn f(robot: &Robot) -> franka::FrankaResult<()> {
// Robot-side first-order low-pass cut-off frequencies in Hz: joint position,
// joint velocity, Cartesian position, Cartesian velocity, controller.
robot.set_filters(100.0, 100.0, 100.0, 100.0, 100.0)?;

// The virtual wall (`GetCartesianLimit`) with the given id.
let wall = robot.virtual_wall(1)?;
println!("{:?} {:?} {}", wall.object_world_size, wall.p_frame, wall.active);
# Ok(()) }
```

`set_filters` is libfranka 0.9.2's `Robot::setFilters`, dropped in 0.10. Franka's own
advice is to leave the robot-side filters at their defaults and filter in the client
instead — which the crate's control loops already do — so this is here for parity, not
because you need it.

## The model

`Robot::load_model()` on an FER needs no download: the crate ships parameters identified
from a real FER's own `libfcimodels.so`. See [The model](./model.md#fer-fci-v5-shipped-identified-parameters),
including the payload caveat. `Robot::robot_model()` fails with `InvalidOperation` — an
FER has no URDF to serve.

## The joint-impedance example

`examples/fer_joint_impedance.rs` runs

```text
tau = K (q_d - q) - D dq + coriolis(state)
K = [600, 600, 600, 600, 250, 150,  50]   Nm/rad
D = [ 50,  50,  50,  50,  30,  25,  15]   Nm s/rad
```

rate-limited by hand against the robot's own `tau_J_d` with
`limit_rate_torques(&rate_limiting::fer::MAX_TORQUE_RATE, &tau, &state.tau_J_d)`, and run
through `control_torques` with `limit_rate = false` and `MAX_CUTOFF_FREQUENCY` — the
controller does its own limiting.

## Known simulator gaps

There is **no published FER simulator image**; `franka-sim:panda-v5` is a local build
(the simulator still calls this mode `panda`). Its known divergences from a real FER are all pinned by characterisation assertions in
`sim_v5_stop_and_reflex.rs` and friends, so a rebuilt image tightens the tests rather than
breaking them silently. The ones that will bite you:

| gap | consequence |
|---|---|
| `--enforce-motion-limits` uses the **FR3's** limit tables | The FER's jerk and acceleration limits are larger on most joints (`kMaxJointJerk = [7500, 3750, 5000, 6250, 7500, 10000, 10000]` vs a flat 5000), so whenever the client's FER rate limiter *saturates* it emits a command a real FER accepts and this simulator rejects. Do not home with `MotionGenerator` on this image with limits enforced, and read a reflex threshold as the FR3's. |
| joint 1 ignores velocity commands | The velocity servo uses FR3 gains; on the Menagerie `franka_emika_panda` model joint 1 limit-cycles at the 500 Hz Nyquist frequency with ±0.36 rad/s, so the clipped torque averages to zero. Commanding 0.1 rad/s for 1 s moves joint 1 by −0.00189 rad against ~0.099 on joints 2, 4 and 7. Position commands on joint 1 are fine. |
| `O_ddP_O` is `[0, 0, 0]` | `Model::gravity(&state)` is identically zero. Use `gravity_q` with an explicit vector. |
| `O_T_EE` is the joint-7 frame | 0.107 m short of the flange along the tool z axis (measured 0.10700011 m), with `F_T_EE` and `NE_T_EE` both identity. The v10 simulator has the same discrepancy. |
| `GetCartesianLimit` is a stub | `kSuccess` with an all-zero 154-byte body, so `virtual_wall(1)` returns all zeros and `active: false`. The framing is real, the content is not; only a real FER can confirm the field mapping. |
| `StopMove` answers `kSuccess` | Where a real robot sends `kPreempted`. `control_torques` then returns `Protocol("Unexpected reply to a Move command")` instead of the usual preemption `ControlException`. |
| no graspable object | This image predates `--gripper-object-width`, so a *successful* grasp is only covered on the FR3 image. The gripper protocol is byte-identical on the two versions anyway. |
| no `franka-sim-check` binary | The harness falls back to its own readiness probe: a version-5 Connect handshake plus one 2373-byte UDP datagram. |

Because there is no published image, CI's **`sim-fer-v5` job is gated on a
`FRANKA_SIM_FER_IMAGE` repository variable** and is skipped until someone sets it; the FCI
v5 simulator tests otherwise run locally. Everything that does not need a simulator does run
on every push, including `wire_sizes_v5.rs`, `fer_native_conformance.rs`, and the mock-FCI
unit tests that pin the v5 `Move` payload, the v5 command numbering, the torque-only
datagrams and the `LoadModelLibrary` request byte for byte. See
[Testing with franka-sim](./testing-with-franka-sim.md).
