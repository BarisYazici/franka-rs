# Controlling the robot

There are two ways to run a 1 kHz loop, and they are the same two libfranka offers on an
FR3: hand the crate a **callback**, or drive the loop yourself with **`ActiveControl`**. For
a program that is not a 1 kHz program there is a third, built on the first:
[target control](#target-control-low-rate-commanders), where the crate runs the loop on a
thread of its own and you set targets whenever you like.

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
example's scripted commander walks ±5 cm steps in x, y and z inside a ±12 cm box around the
start pose, with holds between 0.2 s and 1.5 s, one 2 s stall in which nothing is sent, and one
burst of 20 targets inside 100 ms (about 20 s in all); with `--stdin` the commander is instead
whoever writes `x y z` lines (metres, relative to the start pose) to standard input.

In its default `--bridged` mode the example is three calls on the crate's
[target control](#target-control-low-rate-commanders) — `start_cartesian_target_control`,
`set_position` for every target, `stop()` — and the crate's own thread does the bridging with
the online trajectory generator below under a budget of **0.3 m/s, 0.5 m/s², 20 m/s³**
(`--budget V,A,J`), so that a 5 cm step becomes an S-curve that peaks at 0.15 m/s and lands,
exactly, after 0.66 s. `--log PATH` records one row per cycle from the loop's observer (the
raw target, the echoed `O_T_EE_c`, the measured `O_T_EE`, `q0..q6`, the external wrench and
the generator's velocity and acceleration), which `bench/commander/plot.py` draws. In `--raw`
mode the same targets go to a bare `control_cartesian_pose` with `limit_rate = false` and
`MAX_CUTOFF_FREQUENCY`: the step reaches the robot as a 50 m/s jump, the motion generator
refuses it with `cartesian_motion_generator_velocity_discontinuity` and its siblings, and the
example prints the robot's error text, calls `automatic_error_recovery()` and exits 0.

On a real FER (2026-09-08) an earlier version of the bridge — the public
`cartesian_low_pass_filter` at 1 Hz followed by the same rate limiter, which turned a 5 cm step
into a ramp peaking at about 0.25 m/s — ran the full 19 s sequence with no reflex, measured
pose within a few millimetres of the command, with the default budget and libfranka's example
collision thresholds (20 N nominal); raw mode held the start pose for 0.5 s and was refused at
the first 5 cm step, and `automatic_error_recovery()` cleared it. The first version of the
generator bridge, run on the same arm the same day, did **not** go cleanly, and the reason is
worth the paragraph below the next one: the generator stayed on its targets, but the rate
limiter behind it clamped it on the first two-axis move and from then on the command orbited
at the velocity cap, until the robot refused it. The rules that came out of that are now
implemented by the target control loop, which has since run clean on the same arm.

#### Online trajectory generation

An *online trajectory generator* (OTG) is the causal answer to a stream of stepped targets: a
small state machine that owns the commanded position, velocity and acceleration and, every
cycle, re-plans the time-optimal jerk-limited profile from that state to rest at whatever the
latest target is, then follows it for one cycle. The alternatives all fail on one of the
requirements. A spline needs future knots, and a commander at 10 Hz — or silent for two
seconds — cannot supply them. A quintic re-fitted from the current state to the target every
cycle is smooth but has no notion of the limits: its peak velocity and acceleration scale with
the step and shrink with the chosen duration, and no choice of duration is right for both a
1 mm and a 10 cm step. A first-order low-pass filter is causal and cheap, but its first-cycle
demand is the step times the gain — 0.31 m/s for 5 cm at 1 Hz — so the rate limiter behind it
does the real work, the peak speed still scales with the step, and the approach is exponential,
never quite arriving. The OTG is causal (it uses only the latest target), limit-respecting by
construction (velocity, acceleration and jerk never exceed the budget, on any cycle, including
a 2 ms one after a lost packet), C2 (the acceleration is continuous; only the jerk switches),
time-optimal for the profile family it plans in, and it lands exactly on the target and stays
there.

The crate's `otg` module implements it per axis in about 300 lines with no dependencies and
no allocation: the seven-segment profile (accelerate to a peak velocity with jerk-limited
acceleration, cruise, decelerate to rest) parameterised by that peak velocity, whose value is
found by bisection each cycle — the profile structure of Haschke, Weitnauer and Ritter
(*On-line planning of time-optimal, jerk-limited trajectories*, IROS 2008), evaluated one
cycle at a time the way Ruckig does. A target that moves mid-motion is re-planned from the
current velocity and acceleration; a target closer than the braking distance is passed,
braked for and returned to, without a jerk spike; a target that stays for seconds is reached
and held exactly. `CartesianOtg` runs three axes with the same limits and, optionally,
synchronised so that they arrive together. The property test in the module simulates two
thousand random target sequences — steps at random times, holds up to three seconds, bursts
of twenty targets 5 ms apart, gaps — and asserts every cycle that the velocity, acceleration
and finite-difference jerk stay within the limits, that a reachable target is never
overshot, and that every target is reached within 10 % of the time an admissible
brake-then-move profile would take.

**Putting it in a loop** has three rules, each learnt from that first run. The per-cycle log
of the run, replayed through the generator alone, ends exactly on the last target with every
per-axis limit respected; replayed through the generator *and* `limit_rate_cartesian_pose`,
the limiter clamps the command by nanometres at 1.601 s — `y` braking at −0.5 m/s² while `z`
starts at +0.5 m/s² is a norm of 0.71 m/s² and 28 m/s³ of jerk — and 300 ms later the command
is millimetres behind the generator's own state, which never hears of it. libfranka's limiter
has no braking logic; tracking a pose it has fallen behind it saturates at the budget, passes
the pose, and reverses, and the replay reproduces the ±10 cm orbit in the log. So: the limits
are **per axis** and a Cartesian budget is a norm, hence `OtgLimits::per_axis_for_norm(3)`
(budget / √3 per axis, which is also what keeps a synchronised diagonal move inside the
budget); step **one nominal millisecond per command** (`DELTA_T`) rather than the measured
period, because the limiter and the robot check every packet against 1 ms and the 52 cycles
of 2–4 ms in that log each doubled an increment; and **re-anchor the generator on the echo**
every cycle with `set_position(O_T_EE_c)` — the position only: the echoed twist is a mean over
the cycle, not the generator's end-of-cycle state, and re-anchoring on it throttles the plan
to a crawl — so that whatever runs behind it can only ever shape one command, never
accumulate a lag. With the first two rules the
replayed backstop never touches a command (worst alteration below 1e-9 m) and every target is
met exactly; with the third rule alone it binds by up to 50 µm and the run stays bounded
(millimetres from the targets, no orbit) but does not land exactly, because a vector-norm
clamp distorts one axis's corrections while another saturates. Both replays are regression
tests in the module.

#### Why the bridge has a budget of its own

The crate's rate limiter with `limit_rate = true` is a faithful port of libfranka's, and its
constants — 13 m/s² and 6500 m/s³ on an FER, 9 m/s² and 4500 m/s³ on an FR3 — are what the
robot accepts *in Cartesian space*. The robot also runs inverse kinematics on every commanded
pose and checks the continuity of the result in **joint space**, and that is the check a
stepped target stream trips. Measured on a real FER near the ready pose: a ramp at 2.5 m/s²
with 500 m/s³ of jerk was refused within six cycles as
`cartesian_motion_generator_joint_velocity_discontinuity`; a ramp at libfranka's own
Cartesian limits tripped both the joint velocity and the joint acceleration discontinuity;
1.5 m/s² with 200 m/s³ passed. The check is the ordinary per-joint acceleration limit
applied to the joint motion the poses imply: at that pose joint 2 moves about 3.2 rad per
metre of x-travel, so 2.5 m/s² is 8 rad/s² against its 7.5 rad/s² limit (see
[FER specifics](./fer.md#joint-space-continuity-of-cartesian-pose-commands)). libfranka
behaves identically — its Cartesian examples pass only because their trajectories start
with near-zero acceleration — and the simulator
currently accepts what the robot refuses here. So a program that steps its targets needs a
smaller budget than the limiter's, applied through the public `CartesianOtg` and
`limit_rate_cartesian_pose`, with the loop's limiter left on as the backstop; that is what
the example does.

A second limit appeared on the same arm above roughly 1 m/s² of commanded acceleration: the
robot's external-force estimate `O_F_ext_hat_K` crossed 20 N at about 0.25 m/s and raised
`cartesian_reflex`, so for fast target steps the collision thresholds, not the kinematic
limits, were the binding constraint. The examples' shared `set_default_behavior` (10 N
nominal) was crossed at 0.25 m/s, which is why this example sets libfranka's current example
thresholds explicitly. Both figures are an observation on one arm, not a specification.

Three details an earlier version of the example asked you to carry into your own code — the
first setpoint is the echo, the start is anchored in the first cycle, the motion ends only
once the command has settled — are the target control loop's job now; see
[Target control](#target-control-low-rate-commanders) for the full list.

For a closer look, `crates/franka-rerun` replays such a log in [Rerun](https://rerun.io):
`cargo run --release -p franka-rerun -- csv bridged.csv --robot fr3 -o bridged.rrd`, then
`rerun bridged.rrd` (viewer 0.37.1, matching the SDK the crate pins). The recording has the
target, commanded and measured positions per axis, the speed, acceleration and jerk of the
commanded position against the FR3's limits (or the FER's with `--robot fer`), the raw
target's implied speed for contrast, the commander's steps, stall and burst as a text log,
and a 3D replay of the arm computed from the logged joint angles with the model. To watch
it happen rather than replay it, the same crate has `examples/commander_live.rs`: the same
commander on the same target control, with the `Recorder` pushing from the loop's observer,
streaming into a viewer that is already open (`rerun --port 9876`, then `cargo run --release
-p franka-rerun --example commander_live -- <hostname> --live 127.0.0.1:9876 --meshes DIR`,
with `--bridged` or `--raw`, `--out FILE` for an `.rrd` as well); the crate's `README.md` has
the details.

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
  it shows what was commanded in the 50 ms before the robot stopped. The
  [flight recorder](./flight-recorder.md) replays it in Rerun, and with the `serde` feature
  it can be saved as JSON.

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

## Target control: low-rate commanders

The fourth way, for the programs that are not 1 kHz programs: `Robot::start_*_target_control`
spawns a named thread that runs the crate's own control loop and hands back a handle whose
`set_*` any thread can call at any rate. The loop does what the
[commander section](#bridging-a-non-realtime-commander) learnt the hard way, and `stop()`
brings the command to rest on the last target, finishes the motion, joins the thread and
returns the loop's result. Everything below has run on franka-sim and on a real FER.

```rust,no_run
# extern crate franka;
use std::sync::Arc;
use franka::{RealtimeConfig, Robot, TargetControlOptions};

# fn main() -> franka::FrankaResult<()> {
let robot = Arc::new(Robot::new("172.16.0.2", RealtimeConfig::Enforce)?);
let control = robot.start_cartesian_target_control(TargetControlOptions::default())?;
let start = control.target();            // the start position, base frame, metres
control.set_position([start[0] + 0.05, start[1], start[2]])?;   // any thread, any rate
std::thread::sleep(std::time::Duration::from_secs(1));
let state = control.state();             // the latest RobotState, copied out
assert!(control.is_running());
control.stop()?;                         // settle, finish the motion, join: the loop's result
# let _ = state; Ok(()) }
```

The API, in full:

```rust,ignore
impl Robot {
    pub fn start_cartesian_target_control(self: &Arc<Self>, options: TargetControlOptions)
        -> FrankaResult<CartesianTargetControl>;
    pub fn start_joint_target_control(self: &Arc<Self>, options: JointTargetControlOptions)
        -> FrankaResult<JointTargetControl>;
}
impl CartesianTargetControl {
    pub fn set_position(&self, position_in_base: [f64; 3]) -> FrankaResult<()>;
    pub fn set_orientation(&self, orientation_xyzw: [f64; 4]) -> FrankaResult<()>;
    pub fn set_target(&self, position_in_base: [f64; 3], orientation_xyzw: [f64; 4])
        -> FrankaResult<()>;
    pub fn set_pose(&self, pose: &[f64; 16]) -> FrankaResult<()>;   // column-major, as O_T_EE
    pub fn target(&self) -> [f64; 3];
    pub fn target_orientation(&self) -> [f64; 4];
    pub fn target_pose(&self) -> [f64; 16];
    pub fn state(&self) -> RobotState;
    pub fn is_running(&self) -> bool;
    pub fn stop(self) -> FrankaResult<()>;
}
// JointTargetControl is the same with set_joints([f64; 7]) and target() -> [f64; 7].
```

The Cartesian target is a pose, absolute in the base frame. `set_position` moves its position
and keeps its orientation (the start orientation until something sets it), `set_orientation`
the other way round, `set_target` and `set_pose` set both. Orientations are unit quaternions
in **`[x, y, z, w]` order** — the scalar part *last*, as in nalgebra's `coords` and Eigen's
`coeffs()` — or the rotation block of a column-major pose in the convention of `O_T_EE`. A
quaternion or rotation block within 1e-3 of unit / orthonormal is normalised on the way in;
one further off is refused. `set_joints` takes the seven joint positions. All return
`FrankaError::InvalidArgument` for a non-finite or malformed value and
`FrankaError::InvalidOperation` once the loop has ended for any reason — `is_running()` is
`false` then, and `stop()` has the reason.

### What the loop does

Every cycle, on its own thread:

1. **Anchor.** The first cycle takes the robot's echo of its commanded position (`O_T_EE_c`,
   `q_d`) as the start, the initial target and the first setpoint, so the first command of
   the motion is the echo itself — on FCI v10 the first command is its own filter reference
   and would otherwise go out as a jump. `start_*` returns only once that cycle has run, so
   `target()` and `state()` are valid from the first call.
2. **Read the slot.** The latest target comes through a single-writer seqlock
   (`robot::target_control::TargetSlot`) the loop polls without blocking; a torn read keeps
   the previous target for that one cycle. `set_*` serialises its callers with a mutex on the
   user side only.
3. **Generate under the three rules** of the [`otg` module](#online-trajectory-generation):
   per-axis limits — the Cartesian budgets are norms and get `OtgLimits::per_axis_for_norm(3)`,
   the joint limits are per joint already — one nominal `DELTA_T` per command whatever the
   measured period, and `set_position` on the echo before every re-plan. The axes are
   synchronised, so a diagonal target moves along a straight line. The orientation runs on
   three more axes of the same generator, on the rotation vector of the orientation error in
   the base frame (`log(R_target R_echo^T)`), re-anchored at zero every cycle and composed
   back as `exp(step) R_echo`: a constant-axis turn under `rotation_limits`, arriving
   together with the translation.
4. **Backstop.** `limit_rate_cartesian_pose` / `limit_rate_joint_positions` under the same
   budget (the joint one tightened to the robot's own velocity envelope at `q`), against the
   echo, and the loop's own libfranka limiter behind that. Neither is meant to bind; the
   observer is told when the backstop does. The Cartesian one references the twist and
   acceleration it sent, not the echoed ones: those are float32 on FCI v10, and the rounding
   of a rotation matrix is worth 200 rad/s³ of jerk against a 20 rad/s³ budget.
5. **Guard.** If the *measured* position strays more than `max_deviation` from the start
   (0.30 m, 1.0 rad by default), or the measured orientation turns more than
   `max_angular_deviation` (0.5 rad), the target freezes where the command is, the generator
   brings it to rest, and the loop ends with `FrankaError::Control` carrying
   `target_control::DEVIATION_MESSAGE`.
6. **Land, hold, finish.** After `stop()` the generator runs on until every axis has landed —
   within `Settle::tolerance` of the target (1 mm, 1 mrad), slower than 0.1 mm/s or mrad/s
   and accelerating less than 0.05 (`target_control::REST_VELOCITY`, `REST_ACCELERATION`;
   the float32 echo of an FR3 keeps a landed generator in micro-profiles well below that,
   and what the hold freezes is at most that velocity step, which the joint side of a
   Cartesian command amplifies) — then the loop stops stepping it and sends the robot's echo
   of the last command (continuous with what the robot has, whatever the backstop took off
   that command), bit for bit and past the backstop, for `Settle::cycles` cycles (250), and
   sets `motion_finished` on one more of it. A motion never finishes on a moving command: a real
   FER refused exactly that with `cartesian_motion_generator_velocity_discontinuity`. If the
   generator has not landed within five seconds the same hold starts from wherever the
   command is.

Dropping a handle without `stop()` requests the stop and detaches: the loop settles and
finishes on its own, holding its `Arc<Robot>` until it has. While the loop runs it holds the
robot's control lock, so `robot.read()` and the other loops fail with
`FrankaError::InvalidOperation` as with any callback loop on another thread; `robot.stop()`
preempts it, and the handle's `stop()` then returns the preemption as `FrankaError::Control`.

### Options

| `TargetControlOptions` (Cartesian) | default | `JointTargetControlOptions` | default |
|---|---|---|---|
| `limits: OtgLimits`, a norm budget | 0.3 m/s, 0.5 m/s², 20 m/s³ | `limits: Option<[OtgLimits; 7]>` | `None`: 20 % of the negotiated version's joint limits (`scaled_limits(version, fraction)`) |
| `rotation_limits: OtgLimits`, a norm budget | 0.5 rad/s, 1.0 rad/s², 20 rad/s³ | | |
| `controller_mode` | `CartesianImpedance` | `controller_mode` | `JointImpedance` |
| `max_deviation` | 0.30 m | `max_deviation` | 1.0 rad |
| `max_angular_deviation` | 0.5 rad | | |
| `settle: Settle` (landing tolerance, hold cycles) | 1 mm, 250 cycles | `settle` | 1 mrad, 250 cycles |
| `limit_rate` | `true` | `limit_rate` | `true` |
| `realtime_priority: Option<i32>` | `None` (highest) | same | same |
| `observer` | none | `observer` | none |

Every field is public and has a `with_*` builder; `validate()` checks them without starting
anything. The Cartesian default budget is the one measured on a real FER for the commander
example — the robot's joint-space continuity check refuses 2.5 m/s² near the ready pose and
its collision threshold trips above about 1 m/s², so the default sits well below both. The
joint default is deliberately slow; raise it with `JointTargetControlOptions::scaled_limits`
or explicit limits. The rotational default is a fifth of the FR3's rotational velocity limit
and a seventeenth of its acceleration limit; on the simulator with the joint side of every
commanded pose checked (`--joint-discontinuity-scale 1.0`) a 20° turn of the tool passes with
a wide margin, and a ±15° yaw sweep and a ±15° tilt have run on a real FER.

The **observer** is `FnMut(&RobotState, &CartesianSent)` (`&JointSent` for joints), called
every cycle *on the realtime thread* with the state and what was sent — the pose or `q` after
the backstop (and its quaternion), the target, the generator's velocity and acceleration
(angular too), and by how much the backstop bound. It must not allocate or block; copying into a preallocated ring is what it is for, and
it is how `franka_rerun::Recorder::push` and the commander example's CSV log hook in. Nothing
else allocates on the realtime thread after the start.

The loop thread is raised to `SCHED_FIFO` the way `Robot::new` raises its caller — to the
highest priority, or to `realtime_priority` when set, which is what a program with other
realtime threads wants; a failure is fatal under `RealtimeConfig::Enforce` and ignored under
`Ignore`, so the simulator runs it on an ordinary kernel.
