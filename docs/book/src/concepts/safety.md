# Reflexes, limits and recovery

The robot protects itself and its surroundings on its own: it checks every command against
its limits and every estimated external force against thresholds you set, and it stops with
a reflex when either is crossed. This page says what those checks are, what a reflex looks
like from the client, how to recover, and what the client adds on top, which is little.

## Collision thresholds

`Robot::set_collision_behavior` takes eight arrays: lower and upper torque thresholds per
joint (Nm) and lower and upper force thresholds per Cartesian axis (N, Nm), each for the
acceleration phase and for constant velocity. An estimated external torque or force between
the lower and the upper threshold is reported as a *contact* (`joint_contact`,
`cartesian_contact` in the state); above the upper threshold it is a *collision*, and the
robot stops the motion with a reflex. `set_collision_behavior_simple` takes one set of
values for both phases.

```rust,no_run
# extern crate franka;
# use franka::Robot;
# fn f(robot: &Robot) -> franka::FrankaResult<()> {
// The examples' `set_default_behavior`: 20 Nm / 20 N while accelerating, 10 Nm / 10 N at
// constant velocity, contact and collision thresholds equal.
robot.set_collision_behavior(
    [20.0; 7], [20.0; 7], [10.0; 7], [10.0; 7], [20.0; 6], [20.0; 6], [10.0; 6], [10.0; 6],
)?;
# Ok(()) }
```

Those are the thresholds of libfranka's `examples_common.cpp`, and the `generate_*` examples
use them. `nonrealtime_commander` sets libfranka's current example thresholds instead
(12 to 20 Nm per joint, 20 N and 25 Nm at the end effector), because on one FER the robot's
force estimate
`O_F_ext_hat_K` crossed 10 N at about 0.25 m/s of commanded speed, and above roughly 1 m/s²
of commanded acceleration it crossed 20 N: for fast target steps the collision thresholds,
not the kinematic limits, were the binding constraint. Measured on one arm, not a
specification. The two Cartesian impedance examples set 100 Nm and 100 N so that the arm can
be pushed around; keep the user stop in hand there.

## What a reflex is, from the client

The robot brakes, `robot_mode` becomes `Reflex`, and the `Move` ends with `ReflexAborted`.
The control call returns `FrankaError::Control(ControlException)`: `message` is libfranka's
text with the error names and the success-rate line, `move_status` the terminal status,
`last_motion_errors` the reason (`cartesian_reflex`, `joint_velocity_violation`,
`communication_constraints_violation`, ...), and `log` the last 50 states and the commands
sent with them. `robot.automatic_error_recovery()` then clears the errors so a new motion can
start; it fails with `FrankaError::Command` when the robot requires manual recovery.
`examples/automatic_error_recovery.rs` prints the mode and the error flags before and after.

A motion can also be ended by the client: `robot.stop()` from another thread preempts it,
and the control call returns `FrankaError::Control` carrying
`"libfranka: Move command preempted!"`.

## The user stop and Desk

The user stop and Desk are outside the FCI, and the client cannot override them. A pressed
user stop ends the motion (`MoveStatus::EmergencyAborted`, `RobotMode::UserStopped`); a
motion that a safety function stops or refuses comes back as
`PreemptedDueToActivatedSafetyFunctions` or `CommandRejectedDueToActivatedSafetyFunctions`
(FCI v10). Unlocking FCI mode and opening the brakes happen in Desk as well.

## The limits the robot enforces per command

Velocity, acceleration and jerk in joint space and in Cartesian space, the elbow, and the
torque rate. The client-side rate limiter uses the same constants (the FR3 tables at the
crate root, the FER tables under `franka::rate_limiting::fer`); see
[Rate limiting and filtering](../reference/rate-limiting.md). One check is easy to miss: the
robot runs inverse kinematics on every commanded Cartesian pose and checks the continuity of
the joint motion it implies, which the Cartesian limits do not bound. A pose stream inside
the Cartesian limits can still be refused with
`cartesian_motion_generator_joint_velocity_discontinuity`: near the ready pose joint 2 moves
about 3.2 rad per metre of x, so 2.5 m/s² was refused and 1.5 m/s² passed on an FER, and on
an FR3 the refusal came in the cycle a joint crossed 10 rad/s². Details in
[FER / Panda specifics](../reference/fer.md) and
[Online trajectory generation](../reference/otg.md).

## What the client adds

Little, by design. Target control has a deviation guard: if the measured position strays
more than `max_deviation` from the start (0.30 m; 1.0 rad on any joint for joint targets) or
the measured orientation turns more than `max_angular_deviation` (0.5 rad), the target is
frozen where the command is and the motion finished from rest with `FrankaError::Control`.
It is a coarse client-side check against a runaway target, not a safety function; the
robot's reflexes are. The simulator's FR3 image runs the robot's joint-side continuity check
under `--enforce-motion-limits`; the FER image does not (see
[Simulator gaps](../reference/simulator-gaps.md)).

## Before you move a real arm

- Start from the ready pose: `cargo run --release --example move_to_ready -- <hostname>`.
- The space the motion needs is free; `nonrealtime_commander`, for one, moves inside a
  24 cm cube around the start pose.
- The user stop is in someone's hand.
- The collision thresholds are set by the program, for the task.
- The program has run against the simulator first
  ([Without a robot: franka-sim](../getting-started/simulator.md)).
- `--release`, and `FRANKA_REALTIME=enforce` on the realtime machine.
