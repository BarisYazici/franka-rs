# Three ways to control the arm

The crate offers three ways to run a motion. Two of them, the callback and `ActiveControl`,
are the two libfranka offers on an FR3, and both put your code on the 1 kHz path. The third,
target control, keeps your code off it: the crate runs the loop and you set targets. This
page is for choosing; each interface has a how-to of its own.

## The callback

`Robot::control_joint_positions`, `control_joint_velocities`, `control_cartesian_pose`,
`control_cartesian_velocities`, `control_torques` and the four `control_torques_and_*`
methods take a closure `FnMut(&RobotState, Duration) -> T`. The crate runs the loop: it
receives a state, calls your closure, low-pass filters and rate limits what it returns if
you asked for that, sends it, and repeats until the closure returns a command with
`motion_finished` set. The closure must return every millisecond and must not allocate,
block or print. How-to: [Write a 1 kHz callback](../howto/callback-control.md).

## `ActiveControl`

`Robot::start_torque_control`, `start_joint_position_control`, `start_joint_velocity_control`,
`start_cartesian_pose_control` and `start_cartesian_velocity_control` return a handle. You
own the loop: `read_once` blocks for the next state, `write_once` sends the command, and the
motion ends when you write one with `motion_finished`. Nothing is filtered or rate limited on
this path; smooth setpoints are your job. The handle stays on the thread that started it.
How-to: [Drive the loop yourself](../howto/active-control.md).

## Target control

`Robot::start_cartesian_target_control` and `start_joint_target_control` spawn the loop on a
realtime thread of the crate's own and hand back a handle whose `set_position`,
`set_orientation`, `set_pose` or `set_joints` any thread can call at any rate. The loop turns
every new target into a jerk-limited profile within a budget, and the robot's impedance
controller does the tracking. `stop()` lands on the last target, then finishes the motion.
There is no torque backend. The Python bindings move the arm through this interface only.
How-to:
[Command from a low-rate program](../howto/target-control.md).

## Which one

| your program | use |
|---|---|
| runs at 1 kHz on a realtime machine and computes the next setpoint from the state | callback or `ActiveControl` |
| is a planner, policy, teleoperation or script that produces targets at its own rate | target control |
| needs to command torques | callback or `ActiveControl`; target control has no torque backend, the robot's impedance controller does the tracking |
| wants the crate's low-pass filter and rate limiter behind it | callback, with `limit_rate` and `cutoff_frequency`; target control has the rate limiter as a backstop under its own budget, with the filter off |
| is written in Python | target control: `move_to`, `move_by`, `follow` |

Rate limiting and low-pass filtering exist on the callback path; target control keeps the
rate limiter as a backstop and runs with the filter off. `ActiveControl` has neither, in
this crate as in libfranka.

## What they share

- Only one control or read operation may run on a `Robot` at a time; a second one returns
  `FrankaError::InvalidOperation` with libfranka's message.
- `Robot` is `Send + Sync`, so `robot.stop()` can be called from another thread while any of
  the three runs; the motion then ends with `FrankaError::Control` carrying
  `"libfranka: Move command preempted!"`.
- A motion that ends abnormally returns `FrankaError::Control(ControlException)` with the
  reflex reason and a log of the last cycles; see [State and errors](./state-and-errors.md).
- The realtime rules apply to the first two in full, and to the observer target control
  lets you install; see [The realtime rules](./realtime-rules.md).

Reading without controlling is `Robot::read_once` for one state and `Robot::read` for a
stream; neither commands anything.
