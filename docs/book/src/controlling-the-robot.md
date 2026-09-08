# Controlling the robot

There are two ways to run a 1 kHz loop, and they are the same two libfranka offers on an
FR3: hand the crate a **callback**, or drive the loop yourself with **`ActiveControl`**.

## The callback API

Every `Robot::control_*` method takes a closure `FnMut(&RobotState, Duration) -> T`, where
`T` is the command type for that interface. The `Duration` is the time elapsed since the
previous callback (zero on the first call), so a controller integrates against it rather
than assuming exactly 1 ms.

| method | callback returns | commands |
|---|---|---|
| `control_joint_positions` | `JointPositions` | `q` |
| `control_joint_velocities` | `JointVelocities` | `dq` |
| `control_cartesian_pose` | `CartesianPose` | `O_T_EE` (+ optional `elbow`) |
| `control_cartesian_velocities` | `CartesianVelocities` | `O_dP_EE` (+ optional `elbow`) |
| `control_torques` | `Torques` | `tau_J` |
| `control_torques_and_joint_positions` | `Torques` + `JointPositions` | both |
| `control_torques_and_joint_velocities` | `Torques` + `JointVelocities` | both |
| `control_torques_and_cartesian_pose` | `Torques` + `CartesianPose` | both |
| `control_torques_and_cartesian_velocities` | `Torques` + `CartesianVelocities` | both |

The four motion-only methods also take a `ControllerMode` — `JointImpedance` or
`CartesianImpedance` — selecting the robot's *internal* controller. The `control_torques*`
methods do not: there the external controller is yours.

```rust,no_run
# extern crate franka;
use franka::{ControllerMode, JointVelocities, RealtimeConfig, Robot, DEFAULT_CUTOFF_FREQUENCY};

# fn main() -> franka::FrankaResult<()> {
let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
let mut time = 0.0;
robot.control_joint_velocities(
    |_state, period| {
        time += period.as_secs_f64();
        let omega = 0.5 * (std::f64::consts::PI * time / 2.0).sin();
        let mut out = JointVelocities::new([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, omega]);
        out.motion_finished = time >= 4.0;
        out
    },
    ControllerMode::JointImpedance,
    /* limit_rate  = */ false,
    DEFAULT_CUTOFF_FREQUENCY,
)?;
# Ok(()) }
```

### Ending a motion: `motion_finished`

A motion ends when the callback returns a command with `motion_finished` set. Set the field
directly, or wrap the value:

```rust,no_run
# extern crate franka;
# use franka::JointVelocities;
# fn f(out: JointVelocities, done: bool) -> JointVelocities {
if done {
    franka::motion_finished(out)   // sets the flag and returns the value
} else {
    out
}
# }
```

The flag is part of the `Finishable` trait that all five command types implement. The last
command still gets sent; the crate then closes the motion with the robot the way libfranka
does (`motion_generation_finished`, or `torque_command_finished` on an FR3 torque-only
session).

Never `return` early or panic out of the callback to stop a motion — set the flag, or use
[`stop()`](#stopping-from-another-thread).

## Rate limiting and the low-pass filter

Both `control_*` methods take `limit_rate: bool` and `cutoff_frequency: f64` explicitly,
because Rust has no default arguments. The important thing is that **libfranka's defaults
differ between the two versions**, and the crate reproduces that:

| | `limit_rate` default in libfranka | note |
|---|---|---|
| **FCI v10** (libfranka 0.21) | `false` | The FR3 does its own limiting; the client-side limiter can distort your motion. |
| **FCI v5** (libfranka 0.9.2) | `true` | The FER-era default. |

`cutoff_frequency` defaults to `DEFAULT_CUTOFF_FREQUENCY` in both; pass
`MAX_CUTOFF_FREQUENCY` to disable the first-order low-pass filter entirely. Since the
crate always requires both arguments, pick the value that matches the semantics you want
rather than inheriting a default by accident.

Which *constants* the limiter uses is chosen from the negotiated version, not by you: the
FR3 tables live at the crate root (`franka::MAX_JOINT_JERK` and friends, mirroring
libfranka's `franka::` namespace) and the FER's only at
`franka::rate_limiting::fer::*`, so the two envelopes cannot be confused at a glance.
One consequence worth knowing: on v10 the torque-rate budget is shrunk slightly to absorb
the `f32` quantisation of the `tau_J_d` the robot echoes; on v5 `tau_J_d` is an `f64` and
the limiting is exact.

The `limit_rate_*` functions are also public, if you would rather limit by hand — which is
what `examples/fer_joint_impedance.rs` does, running `control_torques` with
`limit_rate = false` and `MAX_CUTOFF_FREQUENCY` and calling
`limit_rate_torques(&rate_limiting::fer::MAX_TORQUE_RATE, &tau, &state.tau_J_d)` itself.

**Neither rate limiting nor filtering is applied on the `ActiveControl` path**, in this
crate or in libfranka. There, smooth setpoints are the caller's job.

### Bridging a non-realtime commander

`examples/nonrealtime_commander.rs` is what the filter and the limiter are *for*. Most
programs that want to move the arm are not 1 kHz programs — a planner, a vision loop, a script
on a socket, a person at a keyboard — and what they produce is a stream of **targets**:
irregular, sometimes bursty, sometimes silent for seconds, and every one of them a step. The
example puts such a commander on its own non-realtime thread. It walks a scripted sequence of
±5 cm steps in x, y and z inside a ±12 cm box around the start pose, with holds between 0.2 s
and 1.5 s, one 2 s stall in which nothing is sent, and one burst of 20 targets inside 100 ms
(about 20 s in all); with `--stdin` the commander is instead whoever writes `x y z` lines
(metres, relative to the start pose) to standard input. Either way it publishes only the
*latest* target, through a seqlock of four `AtomicU64` — three `f64`s as bits plus a sequence
number — that the control callback reads every cycle without blocking or allocating; if it
catches the writer mid-update it keeps the previous target for that one cycle rather than
spin.

The callback hands `start pose + target` to `control_cartesian_pose`, and the flag decides
what the robot sees:

| mode | `limit_rate` | `cutoff_frequency` | what happens to a 5 cm step |
|---|---|---|---|
| `--bridged` (default) | `true` | 1 Hz | The filter's gain per 1 ms cycle is `dt / (dt + 1 / (2π f_c))` = 0.0062, so the step becomes a demand of 0.31 m/s decaying with a 0.16 s time constant; the limiter turns that into a jerk-limited ramp to 9 m/s² and a peak of 0.29 m/s (0.37 m/s when a second step lands while the first is still under way). |
| `--raw` | `false` | `MAX_CUTOFF_FREQUENCY` | The step goes to the robot as a 50 m/s jump. The motion generator refuses it with `cartesian_motion_generator_velocity_discontinuity`; the example prints the robot's error text, calls `automatic_error_recovery()` and exits 0. |

Two details are worth carrying into your own code. The **first setpoint is always the start
pose**, whatever the slot holds: on FCI v10 the first command of a motion is its own filter
reference (libfranka's `initialized_filter_`), so a target that arrived before the first cycle
would go out unfiltered and unlimited — exactly the jump the bridge exists to prevent. And the
motion ends not when the commander is done but when the commanded `O_T_EE_c` has *settled* on
the last target (within 1 mm for 250 cycles), so the final `motion_finished` is sent from rest;
a measured deviation of more than 30 cm from the start freezes the target where the command is
and ends the same way. `--log PATH` records one row per cycle — the raw target, the echoed
`O_T_EE_c`, the measured `O_T_EE` — into a `Vec` sized before the loop, and
`bench/commander/plot.py` draws it with the stall and the burst marked.

## `ControlException` and the control log

A motion that ends abnormally returns `FrankaError::Control(ControlException)`. That is the
port of `franka::ControlException`, and it carries more than a message:

```rust,no_run
# extern crate franka;
# use franka::{FrankaError, Robot, RealtimeConfig};
# fn f(robot: &Robot) {
# let result: franka::FrankaResult<()> = Ok(());
match result {
    Err(FrankaError::Control(e)) => {
        println!("{}", e.message);                 // libfranka's exact text
        println!("{:?}", e.move_status);           // terminal Move status, if any
        println!("{:?}", e.last_motion_errors);    // the reflex reason
        for record in &e.log {                     // newest last
            println!("{:?} -> {:?}", record.state.q, record.command);
        }
    }
    _ => {}
}
# }
```

- **`message`** is libfranka's full string, including the error names and the success-rate
  lines when the motion was aborted by a reflex.
- **`log`** is a ring of the last states and the commands sent with them —
  `Vec<Record>`, newest last, with `Record { state, command: Option<RobotCommandLog> }`.
  The default size is **50** cycles (`franka::DEFAULT_LOG_SIZE`, libfranka's default);
  change it with `Robot::new_with_log_size` or
  `RobotOptions::new(..).with_log_size(n)`. This is the thing to dump when a reflex fires:
  it shows what was commanded in the 50 ms before the robot stopped.

After a reflex, `robot.automatic_error_recovery()` clears the errors so a new motion can
start. It fails with `FrankaError::Command` when manual recovery is required.

## Stopping from another thread

`Robot` is `Send + Sync` and every method takes `&self`, so share it as an `Arc<Robot>` and
call `stop()` from a second thread while the control loop runs. The loop then fails with
`FrankaError::Control` carrying `"libfranka: Move command preempted!"`.

```rust,no_run
# extern crate franka;
use std::sync::Arc;
use franka::{ControllerMode, JointVelocities, RealtimeConfig, Robot, DEFAULT_CUTOFF_FREQUENCY};

# fn main() -> franka::FrankaResult<()> {
let robot = Arc::new(Robot::new("172.16.0.2", RealtimeConfig::Enforce)?);

let stopper = Arc::clone(&robot);
std::thread::spawn(move || {
    std::thread::sleep(std::time::Duration::from_secs(2));
    let _ = stopper.stop();
});

let result = robot.control_joint_velocities(
    |_state, _period| JointVelocities::new([0.0; 7]),
    ControllerMode::JointImpedance,
    false,
    DEFAULT_CUTOFF_FREQUENCY,
);
// `result` is Err(FrankaError::Control(..)) with "Move command preempted!"
# let _ = result; Ok(()) }
```

Only one control or read operation may run at a time; a second one returns
`FrankaError::InvalidOperation`, with libfranka's own message.

## `ActiveControl`: `read_once` / `write_once`

The externally driven alternative. `Robot::start_*_control` returns a handle that holds the
robot's control lock for its lifetime and lets you drive the loop explicitly:

| starter | handle | `write_once` takes |
|---|---|---|
| `start_torque_control` | `ActiveTorqueControl` | `&Torques` |
| `start_joint_position_control` | `ActiveMotionGenerator<JointPositions>` | motion + `Option<&Torques>` |
| `start_joint_velocity_control` | `ActiveMotionGenerator<JointVelocities>` | motion + `Option<&Torques>` |
| `start_cartesian_pose_control` | `ActiveMotionGenerator<CartesianPose>` | motion + `Option<&Torques>` |
| `start_cartesian_velocity_control` | `ActiveMotionGenerator<CartesianVelocities>` | motion + `Option<&Torques>` |

The motion-generator starters take a `MoveControllerMode`. The `Option<&Torques>` must be
`Some` if and only if that mode is `ExternalController`.

```rust,no_run
# extern crate franka;
use franka::{JointVelocities, MoveControllerMode, RealtimeConfig, Robot};

# fn main() -> franka::FrankaResult<()> {
let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
let mut active = robot.start_joint_velocity_control(MoveControllerMode::JointImpedance)?;

let mut time = 0.0;
loop {
    let (_state, period) = active.read_once()?;
    time += period.as_secs_f64();
    let mut out = JointVelocities::new([0.0; 7]);
    out.motion_finished = time >= 4.0;
    let finished = out.motion_finished;
    active.write_once(&out, None)?;
    if finished {
        break;
    }
}
# Ok(()) }
```

`read_once` blocks for the next state and returns it with the elapsed period;
`write_once` sends the command. Setting `motion_finished` on either input ends the control
process, and a `write_once` after that fails with `FrankaError::Control`.

### Cartesian impedance with `ActiveControl`

`examples/cartesian_impedance_active_control.rs` is libfranka's
`cartesian_impedance_control.cpp` on this path: a spring-damper system, without inertia
shaping, whose equilibrium is the pose the end effector had when the loop started. The
callback becomes the loop body, and the Eigen calls become nalgebra ones (the crate already
depends on nalgebra):

```rust,no_run
# extern crate franka;
# extern crate nalgebra;
# use franka::{motion_finished, Frame, RealtimeConfig, Robot, Torques};
# use nalgebra::{Matrix4, Rotation3, SMatrix, SVector, UnitQuaternion, Vector3};
# fn main() -> franka::FrankaResult<()> {
# let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
# let model = robot.load_model()?;
# let (stiffness, damping) = (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
# let (position_d, orientation_d) = (Vector3::zeros(), UnitQuaternion::identity());
# let mut error = SVector::<f64, 6>::zeros();
let mut control = robot.start_torque_control()?;
loop {
    let (state, _period) = control.read_once()?;
    let coriolis = SVector::<f64, 7>::from_column_slice(&model.coriolis(&state));
    let jacobian =
        SMatrix::<f64, 6, 7>::from_column_slice(&model.zero_jacobian(Frame::EndEffector, &state));
    let dq = SVector::<f64, 7>::from_column_slice(&state.dq);

    let transform = Matrix4::from_column_slice(&state.O_T_EE);
    let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
    let mut orientation =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
    if orientation_d.coords.dot(&orientation.coords) < 0.0 {
        // A unit quaternion and its negation are the same rotation: take the shorter way.
        orientation = UnitQuaternion::new_unchecked(-orientation.into_inner());
    }
    let position_error = transform.fixed_view::<3, 1>(0, 3) - position_d;
    let orientation_error = -(rotation * (orientation.inverse() * orientation_d).vector());
    error.fixed_view_mut::<3, 1>(0, 0).copy_from(&position_error);
    error.fixed_view_mut::<3, 1>(3, 0).copy_from(&orientation_error);

    let tau = jacobian.transpose() * (-(stiffness * error) - damping * (jacobian * dq)) + coriolis;
    control.write_once(&Torques::new(tau.into()))?;
#   break;
}
# control.write_once(&motion_finished(Torques::new([0.0; 7])))?;
# Ok(()) }
```

Nothing in the body allocates: the nalgebra types are fixed-size and live on the stack, and
the model methods return fixed arrays. The example adds the argument parsing, the compliance
constants (150 N/m, 10 Nm/rad, `D = 2 sqrt(K)`), a `SIGINT` handler that lets the loop leave
through a final `motion_finished` write, and the `--duration` bound.

#### A demo you can push around

`examples/cartesian_impedance_figure_eight.rs` is the same control law with somewhere to go.
The equilibrium is no longer the start pose but a Lissajous figure eight around it — `y = A
sin(w t)`, `z = (A/2) sin(2 w t)` in the base frame, `A` = 0.08 m over a 10 s lap by default
(`--amplitude`, `--period`), the orientation setpoint held where it started — so the arm is
already moving when a visitor takes hold of it. Push the hand off the path and let go, and it
springs back onto a target that has meanwhile moved on. The stiffness is up at 200 N/m and
15 Nm/rad, still with `D = 2 sqrt(K)`.

A raised cosine takes the path amplitude *and* the stiffness from zero to nominal over the
first 3 s, and back down over the last 2 s of `--duration` — or over 1 s after Ctrl-C, or
after the end effector is dragged more than 30 cm off its setpoint, which the loop reads as
an operator having walked away with it. Nothing steps at either end. The damping is
deliberately left un-ramped: the loop is over-damped while the ramp is below one, never
under-damped, and what is left at the very end is a pure damper, so the final
`motion_finished` write of zero torques is not a step either.

Two additions to the law itself. A **nullspace joint spring** (5 Nm/rad, `2 sqrt(5)` damping)
pulls the arm back toward the configuration it started in, projected with
`N = I - J^T (J J^T + lambda^2 I)^-1 J` — the damped inverse, `lambda^2` = 1e-4, so the
projector stays defined next to a singularity — so it never fights the Cartesian task; without
it the elbow wanders while you push the hand around. And a **virtual floor**: a one-sided
1500 N/m spring along +z that exists only below `z_start - 0.10 m` (`--floor`). Press the end
effector down and there is a surface there. Both are scaled by the same ramp factor.

Safety on top of the base example's: each joint torque is clamped to ±25 Nm *before* the
Coriolis feedforward is added (that term is the robot's own dynamics, not something the
controller should be trimming), with a one-off note on stderr the first time the clamp bites.
The loop stays allocation-free — the tracking error goes into a fixed-size millimetre
histogram rather than a `Vec` of samples, and the 1 Hz stderr status line (time, `|e_pos|` in
mm, ramp factor, whether the floor is active, clamped-cycle count) warms `eprintln!`'s buffer
with a header printed before the loop starts.

### On the FER

`ActiveControl` works on FCI v5 too — and **libfranka 0.9.2 has no equivalent at all**: its
FER-era API is the callback `franka::Robot::control()` only, with no `startTorqueControl()`
/ `readOnce()` / `writeOnce()`. That surface was added later, for v10 arms. So on an FER
this crate offers a control style the C++ client cannot.

The public API is identical on both versions. Underneath, `start_torque_control()` on an
FER starts a joint-velocity motion generator commanding zeros alongside the external
controller, because v5 has no torque-only motion generator mode; see
[FER specifics](./fer.md#torque-control-on-fci-v5).

Measured on a real FER, `ActiveControl` is equivalent to the callback API within noise —
interval p50 999.2 vs 999.1 µs over 10 s runs, comparable p99, max and CPU. See
[Benchmarks](./benchmarks.md#activecontrol-on-the-fer).
