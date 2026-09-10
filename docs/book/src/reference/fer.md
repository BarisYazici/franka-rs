# FER / Panda specifics

Everything in this book applies to a Franka Emika Robot (FER, also sold as Panda). This page
lists the places where the robot, not the API, is different. An FER is Franka system 4.2.x,
FCI version 5, and the reference client is libfranka 0.9.2.

The byte-level differences and the version negotiation are on [FCI v10 and FCI v5 on the
wire](./wire-protocol.md); the FER's rate-limit constants on [Rate limiting and
filtering](./rate-limiting.md); the divergences of the FER simulator image on [Simulator
gaps](./simulator-gaps.md).

## Torque control on FCI v5

libfranka 0.9.2 has no `MotionGeneratorMode::kNone`, and its `finishMotion` always requires
a motion-generator command. So `franka::Robot::control(ControlCallback)` runs a
**joint-velocity motion generator commanding zero velocity** next to the external
controller. The crate reproduces that, so on an FER:

- `control_torques` and `start_torque_control` send
  `Move { controller_mode: ExternalController, motion_generator_mode: JointVelocity }`;
- every 370-byte command datagram carries your `tau_J_d` **and** a zero `dq_c`, which goes
  through the same low-pass filter and rate limiter a user's velocity would;
- the motion ends with `motion_generation_finished`: a v5 `ControllerCommand` has no
  `torque_command_finished` field to set;
- the two callbacks run in 0.9.2's order (motion first, controller short-circuited) rather
  than 0.21.2's, because the order decides which cycle's commands `finishMotion` sends.

None of that reaches the public API. `control_torques_and_joint_positions` and its siblings
behave the same on both versions: there you supply the motion generator, so nothing is
substituted.

## The `q_d` seed rule

**Seed a motion from `state.q_d`, not `state.q`.**

On an FER the *first* setpoint of a motion is rate limited against the robot's own `q_d` /
`O_T_EE_c` like every other one, because libfranka 0.9.2's `convertMotion` has no
`initialized_filter_` special case for the first command. On an FR3 the first command is its
own reference and passes the limiter unchanged. Seed from `q` and you start one tracking
error away from what the robot is commanding, and the limiter has to absorb that step, which
at best distorts the start of your motion and at worst saturates.

This is why the README's joint-move example reads `robot.read_once()?.q_d` rather than `.q`.
The same defect in the examples' shared `MotionGenerator` was found and fixed during the
2026-09-05 hardware campaign.

## Joint-space continuity of Cartesian pose commands

The robot runs inverse kinematics on every pose a Cartesian pose motion generator sends and
checks the continuity of the result in joint space, on top of the Cartesian velocity,
acceleration and jerk checks. The client-side rate limiter, the crate's with
`limit_rate = true` or libfranka's, only bounds the Cartesian side: its constants
(`rate_limiting::fer::MAX_TRANSLATIONAL_ACCELERATION` = 13 m/s², jerk 6500 m/s³) are what the
robot accepts there, not what the joint-side check accepts.

Measured on a real FER near the ready pose (2026-09-08): a translational ramp at 2.5 m/s²
with 500 m/s³ of jerk was refused within six cycles as
`cartesian_motion_generator_joint_velocity_discontinuity`; a ramp at libfranka's own
Cartesian limits tripped both that and
`cartesian_motion_generator_joint_acceleration_discontinuity`; 1.5 m/s² with 200 m/s³ passed.
The criterion is the per-joint acceleration limit
(`rate_limiting::fer::MAX_JOINT_ACCELERATION`, 7.5 rad/s² on joint 2) applied to the joint
motion the poses imply. At the ready pose a metre of end-effector travel in x costs about
3.2 rad on joint 2, so 2.5 m/s² is 8 rad/s² there, over its limit, while 1.5 m/s² is
4.8 rad/s². The same budget at a more extended pose, where that lever is larger, can still
trip. libfranka behaves identically, and its Cartesian examples pass because their
trajectories start with near-zero acceleration.

The check is not v5-specific. Both error names are in libfranka's error list for the FR3
too, and on an FR3 (2026-09-09) the bracket was repeated: a run whose IK peaked at
9.3 rad/s² passed, two runs were refused with
`cartesian_motion_generator_joint_velocity_discontinuity` in the cycle a joint crossed
10 rad/s² (`franka::MAX_JOINT_ACCELERATION`), with joint jerk under 1400 rad/s³, so the
published limit is applied as is (see
[Benchmarks and hardware validation](./benchmarks.md)).

A stream of stepped targets therefore needs its own, smaller budget with the loop's limiter
left on as the backstop. That is what [target control](../howto/target-control.md) does with
0.3 m/s, 0.5 m/s² and 20 m/s³ by default; the reasoning is on [Online trajectory
generation](./otg.md).

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

`set_filters` is libfranka 0.9.2's `Robot::setFilters`, dropped in 0.10. Franka's own advice
is to leave the robot-side filters at their defaults and filter in the client instead, which
the crate's control loops already do, so this is here for parity, not because you need it.
On an FR3 both calls fail with `InvalidOperation` before anything is sent. The simulator's
`GetCartesianLimit` is a stub; only a real FER can confirm the field mapping of
`VirtualWallCuboid`.

## The model on an FER

`Robot::load_model()` on an FER needs no download: it returns the crate's built-in model,
whose parameters were identified from a real FER's own `libfcimodels.so`, evaluated by the
same native backend as the FR3's URDF. `Robot::robot_model()` fails with `InvalidOperation`
because an FER has no URDF to serve; `Robot::load_model_from_robot()` is the opt-in
download-and-`dlopen` path. The fit, the agreement figures, the payload caveat and the check
against a real FER's measured `O_T_EE` are on [Model parameters and conformance](./model.md).

## The joint-impedance example

`examples/fer_joint_impedance.rs` runs

```text
tau = K (q_d - q) - D dq + coriolis(state)
K = [600, 600, 600, 600, 250, 150,  50]   Nm/rad
D = [ 50,  50,  50,  50,  30,  25,  15]   Nm s/rad
```

rate-limited by hand against the robot's own `tau_J_d` with
`limit_rate_torques(&rate_limiting::fer::MAX_TORQUE_RATE, &tau, &state.tau_J_d)`, and run
through `control_torques` with `limit_rate = false` and `MAX_CUTOFF_FREQUENCY`: the
controller does its own limiting.

## `ActiveControl` on the FER

`Robot::start_torque_control()` and the `start_*_control` motion-generator starters work on
FCI v5. libfranka 0.9.2 has no equivalent: its API for that generation is the callback
`franka::Robot::control()` only, with no `startTorqueControl()` / `readOnce()` / `writeOnce()`.
The public API is identical on both versions; underneath, `start_torque_control()` on an FER
starts the zero-velocity joint generator described above. The measurement against the
callback API is in [Benchmarks](./benchmarks.md) and the how-to is
[Drive the loop yourself](../howto/active-control.md).
