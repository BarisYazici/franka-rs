# First program

Three steps on one page: connect and read a state, stream states, make the first motion. At
the end the arm has moved 5 cm up under your program and come to rest there.

Prerequisites: [Install](./install.md) and [The realtime machine](./realtime-machine.md), or
the [simulator](./simulator.md) with `RealtimeConfig::Ignore` in place of `Enforce` below.

## 1. Connect and read the state

```rust,no_run
# extern crate franka;
use franka::{RealtimeConfig, Robot};

fn main() -> franka::FrankaResult<()> {
    let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
    println!("FCI version {:?}, server version {}", robot.fci_version(), robot.server_version());

    let state = robot.read_once()?;
    println!("q      = {:?}", state.q);
    println!("O_T_EE = {:?}", state.O_T_EE);
    println!("mode   = {}", state.robot_mode);
    Ok(())
}
```

`Robot::new` opens the command channel, negotiates the protocol version (an FR3 speaks
FCI v10, a Franka Emika Robot FCI v5; the same binary drives both) and, with `Enforce`,
checks the realtime prerequisites. `read_once` waits for one state of the 1 kHz stream: `q`
are the seven joint angles in radians, `O_T_EE` the end-effector pose as a column-major 4x4
matrix in the base frame, and `robot_mode` prints `Idle` on a robot that is ready to move.

## 2. Stream states

`read` takes a callback and keeps going until it returns `false`:

```rust,no_run
# extern crate franka;
# use franka::{RealtimeConfig, Robot};
# fn main() -> franka::FrankaResult<()> {
# let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
let mut count = 0;
robot.read(|state| {
    println!("{:?}", state.q);
    count += 1;
    count < 100
})?;
# Ok(()) }
```

One state per millisecond, on the calling thread. `examples/echo_robot_state.rs` is this
program, a port of libfranka's example of the same name. The arm has not moved yet.

## 3. The first motion

Have the user stop button at hand and make sure there is free space above the end effector.
The robot must be in FCI mode with the brakes open.

```rust,no_run
# extern crate franka;
use std::sync::Arc;
use franka::{RealtimeConfig, Robot, TargetControlOptions};

# fn main() -> franka::FrankaResult<()> {
let robot = Arc::new(Robot::new("172.16.0.2", RealtimeConfig::Enforce)?);
let control = robot.start_cartesian_target_control(TargetControlOptions::default())?;
let start = control.target();            // the start position, base frame, metres
control.set_position([start[0], start[1], start[2] + 0.05])?;   // 5 cm up
std::thread::sleep(std::time::Duration::from_secs(2));
control.stop()?;                         // settle, finish the motion, join the thread
# Ok(()) }
```

What to expect on the first run: the arm rises 5 cm along the base z axis in about 0.9 s and
stays there; `stop()` returns about a quarter of a second later, and the program exits.

The target was a step; the motion is not. `start_cartesian_target_control` spawns the 1 kHz
loop on a realtime thread of its own and returns a handle. `set_position` writes the target
into a slot that the loop reads every cycle, and an online trajectory generator in the loop
re-plans a jerk-limited profile from the current command to the target on every cycle, under
a budget of 0.3 m/s, 0.5 m/s² and 20 m/s³ by default (a norm; each axis gets 1/√3 of it).
The robot's own Cartesian impedance controller tracks the resulting pose stream. `stop()`
waits until the command has landed on the target, holds it for 250 cycles and then finishes
the motion, because the robot refuses to finish on a moving command. `set_position` can be
called from any thread at any rate, which is the point: a planner at 10 Hz, a script, a
socket. [Command from a low-rate program](../howto/target-control.md) has the options and
the guards; [Three ways to control the arm](../concepts/control-interfaces.md) places this
next to the 1 kHz callback API and `ActiveControl`.

If the robot refuses the motion, `stop()` returns `FrankaError::Control` with the robot's
error text and the reflex reason, and `robot.automatic_error_recovery()` clears it; see
[State and errors](../concepts/state-and-errors.md).

The README's quick example (`examples/readme_joint_move.rs`) is the callback-style
equivalent of a first motion: a joint-space cosine on joint 4 through
`control_joint_positions`. CI runs it against the simulator.

## Where next

- [Command from a low-rate program](../howto/target-control.md): orientation and joint
  targets, the deviation guards, the observer.
- [Write a 1 kHz callback](../howto/callback-control.md) and
  [Drive the loop yourself](../howto/active-control.md): the two 1 kHz interfaces.
- [Use the model](../howto/model.md), [Use the gripper](../howto/gripper.md),
  [Record and replay a run](../howto/flight-recorder.md).
- [Run the examples](../howto/examples.md): the runnable programs in the repository.
- [From Python](./python.md): the same target control as `move_to`, `move_by` and `follow`.
