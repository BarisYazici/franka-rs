<p align="center">
  <img src="https://raw.githubusercontent.com/BarisYazici/franka-rs/main/docs/book/src/assets/logo.png" width="160" alt="">
</p>

<h1 align="center">franka-rs</h1>

<p align="center">
  A Rust client for the Franka Control Interface: one binary for the FR3 and the Panda, no C++,
  the 1 kHz loop as a library.
</p>

<p align="center">
  <a href="https://github.com/BarisYazici/franka-rs/actions/workflows/ci.yml"><img src="https://github.com/BarisYazici/franka-rs/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <a href="https://github.com/BarisYazici/franka-rs/actions/workflows/docs.yml"><img src="https://github.com/BarisYazici/franka-rs/actions/workflows/docs.yml/badge.svg" alt="Docs"></a>
  <a href="https://crates.io/crates/franka-rs"><img src="https://img.shields.io/crates/v/franka-rs.svg" alt="crates.io"></a>
  <a href="https://pypi.org/project/franka-rs/"><img src="https://img.shields.io/pypi/v/franka-rs.svg" alt="PyPI"></a>
  <a href="https://docs.rs/franka-rs"><img src="https://img.shields.io/docsrs/franka-rs" alt="docs.rs"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-blue.svg" alt="License: Apache-2.0"></a>
</p>

`franka-rs` speaks the protocol a Franka Research 3 or a Franka Emika Robot (Panda) is
controlled through: a TCP channel for commands and the 1 kHz UDP loop for state and
commands, with the robot's kinematics and dynamics evaluated in the crate. There is no
libfranka and no C++ underneath, so `cargo add franka-rs` or `pip install franka-rs` is the
whole installation, and one program drives both robot generations because the protocol
version is negotiated when it connects.

Your program does not have to be a 1 kHz program. A policy at 10 Hz, a teleoperation
device, a script or a notebook cell sends targets at its own rate; the crate's realtime
thread turns them into a continuous command within a velocity, acceleration and jerk
budget, and the robot follows. It tracks that command with torques from the crate's
impedance law by default, so the arm stays compliant, with stiffness and damping as
options, or with the robot's own controller on request (`Backend::RobotController`).

<p align="center">
  <img src="https://raw.githubusercontent.com/BarisYazici/franka-rs/main/docs/book/src/assets/bridge.png" width="800" alt="Stepped targets from a commander at 5 to 30 Hz enter franka-rs; a continuous, jerk-limited command leaves for the robot.">
</p>
<p align="center"><sub>Stepped, bursty, stalling targets in; one continuous command out. Illustration of the generator's profile for a scripted target sequence under a 0.25 m/s, 0.5 m/s², 20 m/s³ budget.</sub></p>

## Install

```sh
cargo add franka-rs        # Rust: `use franka::Robot;`, Rust 1.85 or newer
pip install franka-rs      # Python: `import franka`
```

The realtime prerequisites (a `PREEMPT_RT` kernel, `ulimit -r`, a wired link to the robot)
are on [The realtime machine](https://barisyazici.github.io/franka-rs/getting-started/realtime-machine.html).
Without a robot, a simulated FR3 is
[one `docker run` away](https://barisyazici.github.io/franka-rs/getting-started/simulator.html).

## Quick example

```rust
    let realtime = match env::var("FRANKA_REALTIME").as_deref() {
        Ok("ignore") => RealtimeConfig::Ignore,
        _ => RealtimeConfig::Enforce,
    };
    let robot = Robot::new(&env::args().nth(1).expect("usage: <hostname>"), realtime)?;
    // `q_d`, the commanded position: FCI v5 rate-limits the first command against it.
    let initial = robot.read_once()?.q_d;
    let mut control = robot.start_joint_position_control(MoveControllerMode::JointImpedance)?;
    let mut time = 0.0;
    loop {
        let (_state, period) = control.read_once()?;
        time += period.as_secs_f64();
        let mut q = initial;
        q[3] += PI / 8.0 * (1.0 - (PI / 2.5 * time).cos());
        let mut output = JointPositions::new(q);
        output.motion_finished = time >= 5.0;
        control.write_once(&output, None)?;
        if output.motion_finished {
            break;
        }
    }
```

Takes the robot's hostname from `argv[1]`, picks `RealtimeConfig` from `FRANKA_REALTIME`
(`ignore` or `enforce`, default `enforce`) like every example here, reads the commanded joint
configuration and drives the 1 kHz loop itself: `read_once` waits for the next robot state,
`write_once` answers it, and joint 4 goes through a smooth 0-to-`pi/8`-and-back cosine ramp
over 5 seconds with the robot's internal joint impedance controller tracking. That block is
the body of `main` in
[`examples/readme_joint_move.rs`](crates/franka-rs/examples/readme_joint_move.rs), byte for
byte; CI runs it against the simulator.

The same move for a program that is not a 1 kHz program:

```rust
let control = robot.start_cartesian_target_control(TargetControlOptions::default())?;
control.set_position([x, y, z])?;   // any thread, any rate
control.stop()?;                    // settle on the last target, finish, join
```

and from Python:

```python
with robot.cartesian_targets() as arm:
    arm.move_by([0.0, 0.0, 0.05])   # 5 cm up, relative to the current target
```

## Start here

| You want to… | Where to look |
|---|---|
| install it and move the arm for the first time | [Install](https://barisyazici.github.io/franka-rs/getting-started/install.html), [The realtime machine](https://barisyazici.github.io/franka-rs/getting-started/realtime-machine.html), [First program](https://barisyazici.github.io/franka-rs/getting-started/first-program.html) |
| drive it from Python or a notebook | [From Python](https://barisyazici.github.io/franka-rs/getting-started/python.html) |
| know what can stop you before it does | [Things to keep in mind](https://barisyazici.github.io/franka-rs/concepts/fci.html) |
| do one specific thing | [Command from a low-rate program](https://barisyazici.github.io/franka-rs/howto/target-control.html), [Write a 1 kHz callback](https://barisyazici.github.io/franka-rs/howto/callback-control.html), [Use the model](https://barisyazici.github.io/franka-rs/howto/model.html), [Record and replay a run](https://barisyazici.github.io/franka-rs/howto/flight-recorder.html), [Run the examples](https://barisyazici.github.io/franka-rs/howto/examples.html) |
| see the protocol, the constants and the measurements | [Compared with libfranka](https://barisyazici.github.io/franka-rs/reference/libfranka.html), [Benchmarks and hardware validation](https://barisyazici.github.io/franka-rs/reference/benchmarks.html) |
| read the rustdoc | [API reference](https://barisyazici.github.io/franka-rs/api/franka/index.html) |
| contribute or run the tests | [Contributing](https://barisyazici.github.io/franka-rs/contributing.html) |

## Status

Version 0.2. Both protocol versions, every control interface, target control from Rust and
Python, the gripper and the flight recorder have run on real FR3 and FER arms; measured side
by side with libfranka, loop timing is the same and the model agrees to 1e-14. Target
control's torque backend, added after 0.2.0 and the default now, has run on the simulator
only, so far. Not there yet:
a `ros2_control` hardware interface, the vacuum gripper, a published simulator image for the
FER. The dates and figures are in
[Benchmarks and hardware validation](https://barisyazici.github.io/franka-rs/reference/benchmarks.html),
what changed in [`CHANGELOG.md`](CHANGELOG.md).

## Credit and license

The reference implementation is Franka Robotics'
[`libfranka`](https://github.com/frankarobotics/libfranka) (Apache-2.0), against which every
wire layout, constant, error text and model output here is checked; its copyright notice is
reproduced in [`NOTICE`](NOTICE). The API shape was informed by Marco Boneberger's
[libfranka-rs](https://github.com/marcbone/libfranka-rs), a pure-Rust port of libfranka 0.9
(EUPL-1.2); no code from it is used.

`franka-rs` is an unofficial project, not affiliated with, endorsed by or supported by
Franka Robotics GmbH. Franka, Franka Emika, Panda and FR3 are trademarks of Franka Robotics
GmbH, used only to identify the hardware this crate talks to. Licensed under the
[Apache License, Version 2.0](LICENSE).
