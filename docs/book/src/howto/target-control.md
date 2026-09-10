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

**`stop()`** lands the command on the last target, holds it for 250 cycles (the robot's echo
of it in the robot-controller backend; the loop's own last setpoint in the impedance backend),
sets `motion_finished` on one more, joins the thread and returns the loop's result. In the
impedance backend the finish also waits for the arm: `motion_finished` goes out only once
every joint moves slower than `REST_JOINT_VELOCITY` (0.01 rad/s), or after the 5 s timeout,
the law kept on the held goal meanwhile, so that an arm still closing its lag is not handed
to the robot's controller short of the target. The result is
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
| `backend: Backend` | `Impedance(ImpedanceOptions::cartesian())` | `backend` | `Impedance(ImpedanceOptions::joint())` |
| `controller_mode` (robot-controller backend only) | `CartesianImpedance` | `controller_mode` | `JointImpedance` |
| `max_deviation` | 0.30 m | `max_deviation` | 1.0 rad |
| `max_angular_deviation` | 0.5 rad | | |
| `settle: Settle` (landing tolerance, hold cycles) | 1 mm, 250 cycles | `settle` | 1 mrad, 250 cycles |
| `limit_rate` | `true` | `limit_rate` | `true` |
| `realtime_priority: Option<i32>` | `None` (highest) | same | same |
| `observer` | none | `observer` | none |

Every field is public and has a `with_*` builder; `validate()` checks the options without
starting anything. A Cartesian budget is a norm and each axis gets 1/√3 of it. The default
was measured on a real FER with the robot's controller tracking: its joint-space continuity
check refuses 2.5 m/s² near the ready pose and its collision threshold trips above about
1 m/s², so 0.5 m/s² sits below both. The continuity check applies to a pose stream, so to the
robot-controller backend only; the collision thresholds apply to both. The joint default is
slow on purpose; raise it with `JointTargetControlOptions::scaled_limits(version, fraction)`
or explicit limits.

The **observer** is `FnMut(&RobotState, &CartesianSent)` (`&JointSent` for joints), called
every cycle *on the realtime thread* with the state and what was sent: the pose or `q`
after the backstop, the target, the generator's velocity and acceleration (angular too for a
pose), and by how much the backstop altered the command. In the impedance backend `pose` /
`q` is the loop's setpoint, the backstop alterations are 0, and the record also carries the
joint goal `q_goal`, the clamped torques `tau`, what the leash took off the desired state
this cycle (`leash_alteration`, m for a pose and rad for joints, plus
`leash_angular_alteration`, rad, for a pose; 0 while the arm follows) and, for a pose, the
inverse kinematics residual `ik_error` (m plus rad). It must not allocate or block; copying into a preallocated
ring is what it is for, and how `franka_rerun::Recorder::push` and the commander example's
CSV log hook in.

**`realtime_priority`** sets the loop thread's `SCHED_FIFO` priority; `None` is the highest,
as `Robot::new` uses for its caller. A program with other realtime threads gives the loop a
lower one. A failure to raise it is fatal under `RealtimeConfig::Enforce` and ignored under
`Ignore`, so the simulator runs the loop on an ordinary kernel.

## Backends

The generator produces a setpoint stream; `backend` decides who tracks it.

**`Backend::Impedance(ImpedanceOptions)`**, the default, runs the loop through
`control_torques` and sends, every cycle, the torques of the hybrid joint impedance law that
[DROID](https://arxiv.org/abs/2403.12945)'s controller
([polymetis](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/python/torchcontrol/policies/impedance.py)
`HybridJointImpedanceControl`) runs:

```text
Kp  = Jᵀ Kx J + diag(Kq)
Kd  = Jᵀ Kxd J + diag(Kqd)
tau = Kp (q_goal − q) + Kd (dq_goal − dq) + coriolis(q, dq)
      clamped to ±torque_limits, then low-pass filtered at cutoff_frequency
```

`J` is the zero Jacobian at the end-effector frame for the measured `q`, so the Cartesian
gains act at the frame `O_T_EE` targets are in; `q_goal` is the generator's output on the
joint interface and, on the Cartesian interface, the joint configuration a differential
inverse kinematics finds for the generator's pose each cycle; `dq_goal` is that goal's
velocity. Gravity is the robot's, as in every torque loop. Two things differ from DROID's law
by default: the damping acts on the velocity *error*, not the velocity (DROID's form is
`velocity_feedforward = false`, under which a goal moving at `v` is tracked `Kd v / Kp`
behind), and the generator is leashed to the arm (below).

| `ImpedanceOptions` | Cartesian interface (`::cartesian()`) | joint interface (`::joint()`) |
|---|---|---|
| `gains.cartesian_stiffness` Kx (N/m, Nm/rad) | 750, 750, 750, 15, 15, 15 | 0 |
| `gains.cartesian_damping` Kxd (Ns/m, Nms/rad) | 50, 50, 90, 2, 2, 2 | 0 |
| `gains.joint_stiffness` Kq (Nm/rad) | 40, 30, 50, 25, 35, 25, 10 | 600, 600, 600, 600, 250, 150, 50 |
| `gains.joint_damping` Kqd (Nms/rad) | 4, 6, 5, 5, 3, 2, 1 | 50, 50, 50, 50, 30, 25, 15 |
| `torque_limits` (Nm) | 86, 86, 86, 86, 11.5, 11.5, 11.5 | same |
| `cutoff_frequency` (Hz) | 100 | 100 |
| `velocity_feedforward` | `true` | `true` |
| `leash: Leash` | 0.025 m, 0.15 rad | 0.1 rad per joint (the torque clamp, not the leash, bounds the torque: 600 × 0.1 = 60 Nm on joints 1 to 4, under their 86 Nm clamp; on joints 5 and 6 the 11.5 Nm clamp binds first) |
| `project_joint_gains` | `false` | `false` |
| `posture` (IK nullspace reference) | `None`: the start configuration | not used |
| `ik: IkOptions` | λ 0.05, nullspace gain 1.0 /s, 3 iterations, tolerance 1e-6, limit margin 0.02 rad, max step 0.01 rad | not used |

The Cartesian column, `ImpedanceGains::CARTESIAN`, is DROID's preset with the translational
damping raised from 37 to 50, 50, 90 Ns/m: a damping ratio of about 0.8 from the arm's
apparent masses at the ready pose (0.94 kg along x and y, 3.9 kg along z, computed from the
model), where 37 leaves z at 0.34 and ringing. `ImpedanceGains::DROID` is the preset as DROID
ran it, and with `velocity_feedforward` off it is the polymetis law for replaying policies
trained on it. The joint column is the `fer_joint_impedance` example's gains, which have run
on a real FER. What each knob does:

- `cartesian_stiffness`: how hard the arm pulls back toward the target, per metre and per
  radian of end-effector error; the arm is a spring of this stiffness to anything that
  pushes it. `cartesian_damping`: the resistance to end-effector velocity error, per m/s and
  rad/s.
- `joint_stiffness`, `joint_damping`: the same in joint space, on every joint including the
  one direction the Cartesian term cannot see (the elbow's swing); on the Cartesian
  interface they regularise the nullspace and, unprojected, also stiffen the end effector
  (below); on the joint interface they are the whole law.
- `torque_limits`: the per-joint clamp on the command, before the filter.
- `cutoff_frequency`: the low-pass filter on the torques; `MAX_CUTOFF_FREQUENCY` turns it off.
- `velocity_feedforward`: `Kd (dq_goal − dq)` when on, `−Kd dq` when off. On the simulator
  (franka-sim 1.1.6) the peak in-motion lag of a 5 cm step is 3.7 mm with it on and 12.3 mm
  off.
- `leash`: how far the desired state may run ahead of the measured one. Every cycle the
  generator is anchored on the measured pose (the model's, for the measured `q`) pulled
  toward the previous desired by at most the leash, the torque-mode form of the third
  [generator rule](../reference/otg.md): while the arm follows, that is exactly the previous
  desired and nothing changes; held back by a hand, an obstacle or an unreachable target,
  the desired stays within the leash of the arm, so the spring force on whoever holds it is
  bounded by the felt stiffness times the leash (roughly 25 to 30 N at the default gains at
  the ready pose, 18.75 N with `project_joint_gains`; see the thresholds below), and on
  release the generator resumes from where the arm is, under its budget. The leash keeps
  acting during the stop's hold. On the joint interface each joint's goal stays within
  `leash.joint` of the measured joint; there the torque clamp bounds the torque (the
  `JOINT` preset's 600 Nm/rad × 0.1 rad is 60 Nm on joints 1 to 4, under their 86 Nm clamp;
  on joints 5 and 6 the 11.5 Nm clamp binds before the leash does).
- `project_joint_gains`: confines the joint gains to the Jacobian's nullspace (`N Kq N`,
  `N = I − J⁺ J`), so that the stiffness you set is the stiffness felt at the end effector,
  to within the damping of the projector and away from singularities. Unprojected, the joint
  springs are felt through `J` on top of `Kx`: at the ready pose the default 750 N/m is felt
  as about 990 to 1180 N/m in translation (computed on the FER model), and two to three times `Kx` in
  rotation. With the projection on, zero Cartesian gains would leave the end effector free.
- `posture`: the joint configuration the inverse kinematics drifts toward in the nullspace,
  at most 0.5 rad/s; `None` is the configuration the loop started in. A posture outside the
  joint limits (inset 0.02 rad) is refused with `InvalidArgument`, as is a joint target
  outside them.
- `ik.max_step`: the most any joint of the IK solution moves per cycle (0.01 rad, 10 rad/s);
  a larger step is scaled down whole, so an unreachable or singular pose is approached at a
  bounded rate rather than jumped at.

Every gain must be finite and non-negative, the leash finite and positive (`validate()`).
The law itself is public as `franka::impedance_torques`. The options are set through
builders:

```rust,ignore
use franka::{Backend, ImpedanceGains, ImpedanceOptions, TargetControlOptions};

let gains = ImpedanceGains {
    cartesian_stiffness: [400.0, 400.0, 400.0, 15.0, 15.0, 15.0],
    ..ImpedanceGains::CARTESIAN
};
let options = TargetControlOptions::default()
    .with_backend(Backend::Impedance(ImpedanceOptions::cartesian().with_gains(gains)));
// DROID's law as it ran, for replaying policies trained on it:
let parity = ImpedanceOptions::cartesian()
    .with_gains(ImpedanceGains::DROID)
    .with_velocity_feedforward(false);
```

**Collision thresholds.** Target control sets none; `set_collision_behavior` is yours, and
the robot's reflexes watch the external forces whatever commands the torques. A spring meets
them by deflection: at the default gains a push of 2.7 cm reaches the examples' 20 N
threshold (750 N/m; sooner at the felt stiffness), and an arm held at the leash pulls with
roughly 25 to 30 N at the ready pose. Measured on a real FER (2026-09-10): a slow push read
725 N/m along one direction and about 1090 N/m from the other side, the leash held the error
at exactly 2.5 cm, and a fast push at the leash (0.25 m/s) reached 45 to 50 N, because the
damping adds to the spring: with 40 N thresholds that push ended in a `cartesian_reflex`,
with 60 N it did not. So set the thresholds to at least 40 N and 40 Nm for a commander that
only sends targets, and to 60 N or more where someone will push the arm; or lower the
stiffness. On the joint interface the `JOINT` preset reaches the examples' 20 Nm joint
threshold at 0.033 rad of error. `nonrealtime_commander` takes `--thresholds N` for its
collision thresholds.

**Hand-guiding and the deviation guard.** The guard measures the arm against the *start*
pose (`max_deviation` 0.30 m, `max_angular_deviation` 0.5 rad). Compliance lets a person
move the arm, and moving it past either bound ends the loop: the target freezes where the
command is, the loop finishes at rest, and the robot's own controller holds the arm where
it was left (measured on the FER: a push that dragged the hand 12 cm and turned the wrist
past 0.5 rad ended the session that way, cleanly, without a reflex). That is the right
default against a runaway commander; a session where the arm is meant to be moved by hand
raises both bounds.

**`Backend::RobotController`** sends the setpoint stream as poses (`control_cartesian_pose`)
or joint positions (`control_joint_positions`) and the robot's own controller, selected by
`controller_mode`, tracks it: what target control did before the impedance backend existed.

```rust,ignore
let options = TargetControlOptions::default().with_backend(Backend::RobotController);
```

What changes between the two:

- **Compliance.** The impedance backend is a spring: push the arm and it gives way by
  about force / stiffness (10 N against the default 750 N/m is about 1 cm, the unprojected
  joint term making the arm somewhat stiffer than `Kx`) and returns when released; the
  robot's controller holds the pose stiffly. Hold the arm and the desired pose stops within
  the leash of it, so the spring never pulls harder than the felt stiffness times the leash
  (roughly 25 to 30 N at the defaults at the ready pose) however far the target has moved
  on; let go and it resumes from where the arm is.
- **No joint-side continuity refusals.** The robot checks the inverse kinematics of a pose
  stream against its joint limits and refuses a stream whose joints would accelerate too
  fast (2.5 m/s² near the ready pose on an FER, 10 rad/s² on a joint of an FR3; see
  [Online trajectory generation](../reference/otg.md)). Torques are not checked that way,
  so the budget can be raised further in the impedance backend, within the collision
  thresholds.
- **The deviation guard applies to both.** The measured pose is compared with the start in
  either backend; a compliant arm pushed past `max_deviation` ends the loop the same way.
- **Collision thresholds apply to both, and are yours to set.** The robot's contact and
  collision reflexes watch the external torques and forces whatever commands them; a
  compliant arm meeting an obstacle still trips them at the thresholds
  `set_collision_behavior` sets, and a spring reaches them by deflection (the paragraph
  above).
- **An unreachable or singular target** lags in the impedance backend, because the inverse
  kinematics never jumps (at most `ik.max_step` per cycle) and the leash holds the desired
  pose near the arm; the robot's controller refuses a pose stream it cannot follow.
- **The finish waits for the arm.** `stop()` in the impedance backend sets `motion_finished`
  only once every joint is slower than `REST_JOINT_VELOCITY`, or after the 5 s timeout.

On franka-sim 1.1.6 the arm did not move at the start of a session (measured change 0 to
within floating point over the first 500 cycles), a 5 cm step lands 0.5 to 0.8 mm from the
target and the peak lag during the motion is 3.7 mm with velocity feedforward and 12.3 mm
without. On two real FERs (2026-09-10, `PREEMPT_RT` host, thresholds 40 N) the first torque
of a session was that of rest (under 0.04 Nm), the commander's stepped sequence and the
±15° yaw sweep ran with no reflex and an IK residual under 1e-6, joint targets landed within
0.6 mrad (a 0.2 rad step) to 4 mrad, `stop()` at rest took 0.44 s and mid-motion 0.9 s, and
the tracking error at the holds was 4.6 mm (arm L) and 2.8 mm (arm R) at 750 N/m, 2.7 mm at
1500 N/m, 8 to 10 mm along a slow circle: a constant residual force of about 4 N on these
arms that the robot's own controller also shows (3.7 mm at the same holds) and that scales
with 1/K. The push tests are under [Collision thresholds](#backends) above. The full record
is in [Benchmarks and hardware validation](../reference/benchmarks.md).

## What the loop does every cycle

1. **Anchor.** The first cycle takes the start, first target and first setpoint from the
   robot's echo of its commanded pose or joints (`O_T_EE_c`, `q_d`) in the robot-controller
   backend, and from the measured state in the impedance backend, where no echo of the
   command exists: the measured `q`, and for a pose the model's pose of that configuration
   rather than `O_T_EE`, so the inverse kinematics starts with a zero residual; `q_goal`
   starts at `q`. Every later
   cycle of the impedance backend anchors on the measured state pulled toward the previous
   desired by at most the leash.
2. **Read the slot.** The latest target comes through a single-writer seqlock the loop
   polls without blocking; a torn read keeps the previous target for one cycle.
3. **Generate.** One synchronised jerk-limited generator over all axes, per-axis limits,
   one nominal millisecond per command, re-anchored before every re-plan on the echo
   (robot-controller backend) or on the leashed anchor (impedance backend).
4. **Track.** Impedance backend: the inverse kinematics step for a pose, the law, the
   clamp, `Torques`. Robot-controller backend: the rate limiter under the same budget,
   then the loop's own libfranka limiter; neither is meant to bind, and the observer sees
   when one does.
5. **Guard.** If the measured pose strays past `max_deviation` or `max_angular_deviation`
   from the start, the target freezes and the loop ends with `FrankaError::Control`.
6. **Land, hold, finish.** After `stop()`: run until every axis is within `settle.tolerance`
   of the target and at rest, hold the last command for `settle.cycles`, then
   `motion_finished`; if the generator has not landed within five seconds, hold from where
   the command is. The impedance backend finishes only once the arm itself is at rest
   (`REST_JOINT_VELOCITY`), or after five more seconds.

Why each step is what it is, and what happened on the arm without it, is in
[Online trajectory generation](../reference/otg.md); the impedance law, its inverse
kinematics and its provenance are in [The impedance backend](../reference/impedance.md).

## Examples and Python

`examples/nonrealtime_commander.rs` drives this loop from a scripted commander that steps
the target by ±5 cm with irregular holds, a 2 s stall and a burst of 20 targets in 100 ms
(`--stdin` reads `x y z` lines instead; `--rotate` adds a ±15° yaw sweep through
`set_orientation`; `--log PATH` writes one CSV row per cycle from the observer). Its `--raw`
mode sends the same steps to a bare `control_cartesian_pose` so the robot refuses the first
one, for contrast. `crates/franka-rerun/examples/commander_live.rs` is the same commander
streamed into a Rerun viewer as it runs. Python's `move_to`, `move_by` and `follow` are this
loop; see [From Python](../getting-started/python.md).

Target control with the robot's controller tracking has run on franka-sim, on a real FER
(2026-09-09: the commander's translation and rotation sequences, and the Python policy loop)
and on a real FR3 (2026-09-09: the same sequences, with the robot's joint-side acceleration
check bracketed at 10 rad/s²); see
[Benchmarks and hardware validation](../reference/benchmarks.md). The impedance backend ran
on both FERs on 2026-09-10 (the FR3 was not reachable that day); see [Backends](#backends).
