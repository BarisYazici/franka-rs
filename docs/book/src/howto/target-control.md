# Command from a low-rate program

At the end of this page you will have moved the end effector to targets set from an ordinary
thread, at whatever rate your program runs, with the crate's realtime thread doing the 1 kHz
work. This is the interface for planners, policies, teleoperation and scripts, and the one
the Python bindings use.

Prerequisites: [First program](../getting-started/first-program.md) works against your
robot or the simulator, the arm is at the ready pose (`move_to_ready`), and the space around
the end effector is free.

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

`start_cartesian_target_control` returns once the loop's first cycle has run, so `target()`
and `state()` are valid at once. A 5 cm step along one axis under the default budget becomes
an S-curve that peaks at about 0.12 m/s and lands after about 0.85 s (the budget is a norm
and each axis gets 1/√3 of it).

## The API

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

The Cartesian target is a pose, absolute in the base frame. `set_position` moves its
position and keeps its orientation (the start orientation until something sets it),
`set_orientation` the other way round, `set_target` and `set_pose` set both. Orientations
are unit quaternions in **`[x, y, z, w]` order**, the scalar part last, or the rotation
block of a column-major pose in the convention of `O_T_EE`. A quaternion or rotation block
within 1e-3 of unit or orthonormal is normalised on the way in; one further off is refused.
`set_joints` takes the seven joint positions in radians. Only the latest target counts.

Every setter returns `FrankaError::InvalidArgument` for a non-finite or malformed value and
`FrankaError::InvalidOperation` once the loop has ended for any reason; `is_running()` is
`false` then, and `stop()` has the reason. `target()` is the latest target, `state()` the
latest `RobotState` the loop received. While the loop runs it holds the robot's control
lock, so `robot.read()` and the other loops fail with `InvalidOperation`.

**`stop()`** lands the command on the last target, holds the robot's echo of it for 250
cycles, sets `motion_finished` on one more, joins the thread and returns the loop's result:
`Ok` for a regular end, `FrankaError::Control` if the robot aborted the motion or the
deviation guard fired. A target published just before `stop()` is not lost. Dropping the
handle without `stop()` requests the stop and detaches: the loop settles and finishes on its
own, holding its `Arc<Robot>` until it has. `robot.stop()` from elsewhere preempts the loop,
and the handle's `stop()` then returns the preemption as `FrankaError::Control`.

## Options

| `TargetControlOptions` (Cartesian) | default | `JointTargetControlOptions` | default |
|---|---|---|---|
| `limits: OtgLimits`, a norm budget | 0.3 m/s, 0.5 m/s², 20 m/s³ | `limits: Option<[OtgLimits; 7]>` | `None`: 20 % of the negotiated version's joint limits |
| `rotation_limits: OtgLimits`, a norm budget | 0.5 rad/s, 1.0 rad/s², 20 rad/s³ | | |
| `controller_mode` | `CartesianImpedance` | `controller_mode` | `JointImpedance` |
| `max_deviation` | 0.30 m | `max_deviation` | 1.0 rad |
| `max_angular_deviation` | 0.5 rad | | |
| `settle: Settle` (landing tolerance, hold cycles) | 1 mm, 250 cycles | `settle` | 1 mrad, 250 cycles |
| `limit_rate` | `true` | `limit_rate` | `true` |
| `realtime_priority: Option<i32>` | `None` (highest) | same | same |
| `observer` | none | `observer` | none |

Every field is public and has a `with_*` builder; `validate()` checks the options without
starting anything. A Cartesian budget is a norm and each axis gets 1/√3 of it. The default
was measured on a real FER: the robot's joint-space continuity check refuses 2.5 m/s² near
the ready pose and its collision threshold trips above about 1 m/s², so 0.5 m/s² sits below
both. The joint default is slow on purpose; raise it with
`JointTargetControlOptions::scaled_limits(version, fraction)` or explicit limits.

The **observer** is `FnMut(&RobotState, &CartesianSent)` (`&JointSent` for joints), called
every cycle *on the realtime thread* with the state and what was sent: the pose or `q`
after the backstop, the target, the generator's velocity and acceleration (angular too for a
pose), and by how much the backstop altered the command. It must not allocate or block;
copying into a preallocated ring is what it is for, and how `franka_rerun::Recorder::push`
and the commander example's CSV log hook in.

**`realtime_priority`** sets the loop thread's `SCHED_FIFO` priority; `None` is the highest,
as `Robot::new` uses for its caller. A program with other realtime threads gives the loop a
lower one. A failure to raise it is fatal under `RealtimeConfig::Enforce` and ignored under
`Ignore`, so the simulator runs the loop on an ordinary kernel.

## What the loop does every cycle

1. **Anchor.** The first cycle takes the robot's echo of its commanded pose or joints
   (`O_T_EE_c`, `q_d`) as start, first target and first setpoint.
2. **Read the slot.** The latest target comes through a single-writer seqlock the loop
   polls without blocking; a torn read keeps the previous target for one cycle.
3. **Generate.** One synchronised jerk-limited generator over all axes, per-axis limits,
   one nominal millisecond per command, re-anchored on the echo before every re-plan.
4. **Backstop.** The rate limiter under the same budget, then the loop's own libfranka
   limiter; neither is meant to bind, and the observer sees when one does.
5. **Guard.** If the measured pose strays past `max_deviation` or `max_angular_deviation`
   from the start, the target freezes and the loop ends with `FrankaError::Control`.
6. **Land, hold, finish.** After `stop()`: run until every axis is within `settle.tolerance`
   of the target and at rest, hold the echo of the last command for `settle.cycles`, then
   `motion_finished`; if the generator has not landed within five seconds, hold from where
   the command is.

Why each step is what it is, and what happened on the arm without it, is in
[Online trajectory generation](../reference/otg.md).

## Examples and Python

`examples/nonrealtime_commander.rs` drives this loop from a scripted commander that steps
the target by ±5 cm with irregular holds, a 2 s stall and a burst of 20 targets in 100 ms
(`--stdin` reads `x y z` lines instead; `--rotate` adds a ±15° yaw sweep through
`set_orientation`; `--log PATH` writes one CSV row per cycle from the observer). Its `--raw`
mode sends the same steps to a bare `control_cartesian_pose` so the robot refuses the first
one, for contrast. `crates/franka-rerun/examples/commander_live.rs` is the same commander
streamed into a Rerun viewer as it runs. Python's `move_to`, `move_by` and `follow` are this
loop; see [From Python](../getting-started/python.md).

Target control has run on franka-sim, on a real FER (2026-09-09: the commander's translation
and rotation sequences, and the Python policy loop) and on a real FR3 (2026-09-09: the same
sequences, with the robot's joint-side acceleration check bracketed at 10 rad/s²); see
[Benchmarks and hardware validation](../reference/benchmarks.md).
