# How the FCI works

The Franka Control Interface (FCI) is the protocol a Franka control box speaks over
Ethernet. This page is the part of it to keep in mind whenever you write a program against
it. The byte-level detail is in [FCI v10 and FCI v5 on the wire](../reference/wire-protocol.md).

## Two channels

A session has two sockets. A **TCP channel** carries commands and their replies: `Move`,
which starts a motion, `StopMove`, the setters (`set_collision_behavior`,
`set_joint_impedance`, `set_load`, ...) and `automatic_error_recovery`. A **UDP channel**
carries the 1 kHz exchange: the robot sends a `RobotState` every millisecond, and while a
motion runs it expects one command back for every state.

That expectation is the deadline. The robot keeps score in
`control_command_success_rate`, the fraction of the last 100 commands it accepted. A late or
missing command lowers it, and a sustained fall ends the motion with a
`communication_constraints_violation` reflex. Everything in
[The realtime rules](./realtime-rules.md) follows from this one number.

## A motion is a `Move` session

Nothing moves until the client sends a `Move`. The request names a motion generator (joint
positions, joint velocities, Cartesian pose or Cartesian velocity) and a controller (the
robot's joint impedance or Cartesian impedance controller, or an external one, meaning the
client sends torques). The robot answers that the motion started, the 1 kHz exchange begins,
and the motion runs until the client sets `motion_finished` on a command or something else
ends it: a reflex, a `StopMove`, the user stop, or missed deadlines.

A motion never finishes on a moving command. The command that carries `motion_finished`
has to be one the robot can hold; a real FER refused a finish on a moving Cartesian pose
with `cartesian_motion_generator_velocity_discontinuity`. All three control interfaces of
this crate end a motion the same way, and target control's `stop()` brings the command to
rest before it finishes.

## Who does the tracking

When you send positions, velocities or poses, the robot's own controller does the tracking:
the joint impedance or Cartesian impedance controller named in the `Move`, with the
stiffness set by `set_joint_impedance` and `set_cartesian_impedance`. When you send torques,
your controller does the tracking. `Torques` are joint torques without gravity and friction;
the robot adds those, and echoes the last commanded torque as `tau_J_d`.

## The robot checks every command

Positions, velocities and poses are checked against the joint and Cartesian velocity,
acceleration and jerk limits, which are the same constants the client-side rate limiter
uses; a Cartesian pose stream is also checked for the continuity of the joint motion it
implies; torque commands are checked for continuity. All the while the robot compares the
external forces and torques it estimates with the collision thresholds you set. A violation
stops the motion with a **reflex**: the arm brakes, `robot_mode` becomes `Reflex`, the
reason is in `last_motion_errors`, and the client's control call returns
`FrankaError::Control`. `automatic_error_recovery()` clears it. See
[Reflexes, limits and recovery](./safety.md).

## One client, two ports

The FCI is reached at the robot's address (`172.16.0.2` in Franka's default setup) once FCI
mode is unlocked in Desk and the brakes are open. Port 1337 is the robot, port 1338 the
Franka Hand, and only one FCI client may be connected to the robot at a time.

## Two protocol versions

An FR3 speaks FCI v10 and a Franka Emika Robot / Panda FCI v5; `Robot::new` announces 10,
takes the rejection an FER answers with, and reconnects as 5, so one binary drives either
arm. The differences are listed in
[FCI v10 and FCI v5 on the wire](../reference/wire-protocol.md) and
[FER / Panda specifics](../reference/fer.md).

Next: [Three ways to control the arm](./control-interfaces.md).
