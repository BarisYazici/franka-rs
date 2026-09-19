<p align="center">
  <img src="https://raw.githubusercontent.com/BarisYazici/franka-rs/main/docs/book/src/assets/logo.png" width="120" alt="">
</p>

<h1 align="center">franka-rs</h1>
<p align="center">Send targets. Let Rust handle the realtime control.</p>

<p align="center">
  <a href="https://github.com/BarisYazici/franka-rs/actions/workflows/ci.yml"><img src="https://github.com/BarisYazici/franka-rs/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <a href="https://github.com/BarisYazici/franka-rs/actions/workflows/docs.yml"><img src="https://github.com/BarisYazici/franka-rs/actions/workflows/docs.yml/badge.svg" alt="Docs"></a>
  <a href="https://crates.io/crates/franka-rs"><img src="https://img.shields.io/crates/v/franka-rs.svg" alt="crates.io"></a>
  <a href="https://pypi.org/project/franka-rs/"><img src="https://img.shields.io/pypi/v/franka-rs.svg" alt="PyPI"></a>
  <a href="https://docs.rs/franka-rs"><img src="https://img.shields.io/docsrs/franka-rs" alt="docs.rs"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-blue.svg" alt="License: Apache-2.0"></a>
</p>

Control a Franka FR3 or Panda from Rust or Python. `franka-rs` connects directly to the
robot, generates smooth motion from your targets, and runs the 1 kHz control loop.
It implements the Franka Control Interface in Rust, without libfranka or C++.

**[Website](https://barisyazici.github.io/franka-rs/)** ·
[Choose your setup](docs/book/src/getting-started/choose-your-setup.md) ·
[Documentation](https://barisyazici.github.io/franka-rs/introduction.html) ·
[API](https://docs.rs/franka-rs) · [Changelog](CHANGELOG.md)

## Two ways to use it

| Your setup | What to install | Where control runs |
|---|---|---|
| A Rust or Python program connected directly to the arm | `franka-rs` | Your Linux realtime machine |
| A laptop or GPU workstation commanding an arm through a Raspberry Pi 5 | `franka-node` on the Pi, `franka-node-client` on the laptop | The Pi, running `franka-rs` |

For direct control, install the published library in your project:

```sh
cargo add franka-rs                  # Rust: use franka::Robot
python -m pip install franka-rs      # Python: import franka
```

The machine wired to a real arm needs a `PREEMPT_RT` kernel, realtime scheduling
permissions, and FCI enabled on the robot. Follow
[the realtime setup](docs/book/src/getting-started/realtime-machine.md) before motion.
No robot yet? [Start with the simulator](docs/book/src/getting-started/simulator.md),
or [open its desktop viewer](docs/book/src/getting-started/simulator.md#start-with-visualization)
to watch the arm move.

## Keep the control loop on the Pi

```text
Your laptop / GPU workstation    Raspberry Pi 5              Franka arm
Python targets ── Zenoh ───────▶ franka-node ── wired FCI ──▶ FR3 or Panda
                                franka-rs at 1 kHz
```

Your script, policy, or teleoperation application sends goals at its own rate. On the
Pi, `franka-node` uses the library to generate a continuous trajectory and calculate
impedance-control torques every millisecond. Zenoh carries targets and state between
machines; the realtime loop stays on the Pi. A direct client connection needs no
separate Zenoh router.

**[Set up a Raspberry Pi 5](docs/book/src/getting-started/raspberry-pi.md)** has the
installation, configuration, first state read, and a short Python motion example.

## Add only what you need

| Optional part | What it adds |
|---|---|
| Franka Hand | Gripper commands through the library or node |
| `franka-cam` | USB camera streams over Zenoh, in a separate process |
| `franka-rerun` / the node's `record` feature | Record arm state and commands, then inspect them in Rerun |
| `franka-description` | Robot meshes used by the visualizer; usually pulled in for you |

[Peripherals and recording](docs/book/src/getting-started/peripherals.md) explains how
these fit together, including current hardware-validation limits. Cameras and Zenoh
are optional when using the core library directly.

Use the pieces to build a policy loop, a teleoperation application, or recorded experiments.
You supply the policy or input-device integration; this repository supplies robot control,
transport, and recording. Detailed configuration lives in the
[node guide](docs/book/src/howto/franka-node.md).

## Quick example

<details>
<summary>Rust: drive the 1 kHz loop yourself, or send targets from a slower program</summary>

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

</details>

## Project status

See [the changelog](CHANGELOG.md) for the distinction between released and development
features. The core supports FR3 / FCI v10 and Panda / FCI v5. The node documentation
reports Raspberry Pi 5 runs with one or two arms. Validation depends on the controller
and peripheral: the node's gripper path and camera recording still have simulator-only
or synthetic-test coverage, and the core docs retain an FR3 impedance-validation caveat.
There is no `ros2_control` hardware interface, vacuum-gripper support, or published Panda
simulator image yet.

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
