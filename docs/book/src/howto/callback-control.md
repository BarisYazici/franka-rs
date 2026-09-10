# Write a 1 kHz callback

At the end of this page you will have run a motion from a closure the crate calls once per
robot state, ended it with `motion_finished`, and read the control log a reflex leaves
behind. Prerequisites: [First program](../getting-started/first-program.md) works, the arm
is at the ready pose, and you have read [The realtime rules](../concepts/realtime-rules.md):
the closure runs on the realtime thread.

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

The closure is `FnMut(&RobotState, Duration) -> T`, `T` the command type of the method. The
`Duration` is the robot time since the previous call (zero on the first); integrate against
it rather than assuming 1 ms.

## The methods

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

The four motion-only methods take a `ControllerMode`, `JointImpedance` or
`CartesianImpedance`, which selects the robot's internal controller; the `control_torques*`
methods do not, there the controller is yours. Seed a position or pose motion from
`state.q_d` / `state.O_T_EE_c`, not from the measured `q` / `O_T_EE` (realtime rule 5).

## Ending a motion

A motion ends when the callback returns a command with `motion_finished` set. Set the field,
or wrap the value:

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

The flag belongs to the `Finishable` trait all five command types implement. The last command
is still sent, and the crate then closes the motion with the robot. Never `return` early or
panic out of the callback to stop a motion: set the flag, or call `stop()` from another
thread. A panic that unwinds out of the callback cancels the motion on the robot, as a thrown
exception does in libfranka, but that is an abort, not a finished motion.

## Rate limiting and the low-pass filter

Every method takes `limit_rate: bool` and `cutoff_frequency: f64` explicitly, because Rust
has no default arguments and libfranka's `limit_rate` default differs by version: `false` in
libfranka 0.21 for the FR3 (the robot does its own limiting; the client-side limiter can
distort a motion), `true` in libfranka 0.9.2 for the FER. `cutoff_frequency` defaults to
`DEFAULT_CUTOFF_FREQUENCY` in both; `MAX_CUTOFF_FREQUENCY` disables the first-order low-pass
filter. Which constants the limiter uses follows from the negotiated version, not from you;
they and the `limit_rate_*` functions are in
[Rate limiting and filtering](../reference/rate-limiting.md).

## `ControlException` and the control log

A motion that ends abnormally returns `FrankaError::Control(ControlException)`, the port of
`franka::ControlException`:

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

`message` is libfranka's full string, with the error names and the success-rate line when a
reflex aborted the motion. `log` is `Vec<Record>`, `Record { state, command:
Option<RobotCommandLog> }`, newest last: what was commanded in the 50 ms before the robot
stopped. The default size is 50 cycles (`franka::DEFAULT_LOG_SIZE`); `Robot::new_with_log_size`
or `RobotOptions::new(..).with_log_size(n)` changes it. The flight recorder replays the log
([Record and replay a run](./flight-recorder.md)); with the `serde` feature it saves as JSON.
After a reflex, `robot.automatic_error_recovery()` clears the errors for the next motion.

## Stopping from another thread

`Robot` is `Send + Sync` and every method takes `&self`: share it as an `Arc<Robot>` and call
`stop()` from a second thread. The loop then fails with `FrankaError::Control` carrying
`"libfranka: Move command preempted!"`.

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
`FrankaError::InvalidOperation` with libfranka's own message. The `generate_*` examples are
ports of libfranka's on this interface; see [Run the examples](./examples.md).
