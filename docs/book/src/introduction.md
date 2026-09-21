# franka-rs

[Project home](https://barisyazici.github.io/franka-rs/) · Documentation

Send targets from your program. Let Rust handle the realtime control.

`franka-rs` controls a Franka FR3 or Panda directly from Rust or Python, without
libfranka or C++. Your application can send goals at its own rate while the library
runs the 1 kHz loop, or you can write your own controller.

[See the architecture](./getting-started/architecture.md): what runs where, how a Python goal
becomes motion, and which components you actually need.

## Start with your setup

| I want to… | Start here |
|---|---|
| Use Rust or Python on the machine connected to the arm | [Install the library](./getting-started/install.md), then [prepare the realtime machine](./getting-started/realtime-machine.md) |
| Keep control on a Raspberry Pi 5 and use my laptop for the application | [Set up the Pi and remote Python client](./getting-started/raspberry-pi.md) |
| Try it without hardware | [Run the simulator](./getting-started/simulator.md) |
| Add cameras, a gripper, or recordings | [Peripherals and recording](./getting-started/peripherals.md) |
| Drive a node-served arm from a Meta Quest controller | [Teleoperate with a Quest](./howto/vr-teleop.md) |

Not sure? [Choose your setup](./getting-started/choose-your-setup.md) explains what goes
on each machine and which pieces you need.

## Where the pieces fit

**The library is the core.** `franka-rs` communicates with the robot, evaluates the
model, and turns targets into continuous commands within velocity, acceleration, and
jerk limits. Its default target controller computes impedance torques; an alternative
uses the robot's own controller. [Target control](./howto/target-control.md) explains both.

**The node is optional.** `franka-node` wraps the library so another machine can send
targets over Zenoh. A Pi 5 can run the trajectory generation and impedance controller
beside the arm; your laptop runs the application. Zenoh carries messages between them.
A direct connection needs no separate router.

**Peripherals are optional too.** Add the Franka Hand, camera streams with `franka-cam`,
or recordings with Rerun when your application needs them. They are not prerequisites
for the library.

![Targets at a slower rate become a continuous command for the robot.](./assets/bridge.png)

The plot illustrates the trajectory generator under a 0.25 m/s, 0.5 m/s², 20 m/s³
budget. A policy, script, or teleoperation application supplies the targets; this
repository does not supply a trained policy.

## Go deeper when you need to

- [First program](./getting-started/first-program.md) and [Python](./getting-started/python.md).
- [How the FCI works](./concepts/fci.md) and [realtime requirements](./concepts/realtime-rules.md).
- [Node configuration](./howto/franka-node.md), [recording](./howto/flight-recorder.md), and
  [tuning a running controller](./howto/live-tuning.md) from the browser panel.
- [Benchmarks](./reference/benchmarks.md), [API reference](./api-reference.md), and [contributing](./contributing.md).

## Status and scope

The [changelog](./changelog.md) separates released features from development work.
Published packages may trail this checkout.

The node docs report Pi 5 runs with one or two arms. Hardware validation is specific to
each path: the core impedance reference still carries an FR3 validation caveat; the
node's gripper path is simulator-tested, and camera recording is tested with synthetic
input. Live tuning is tested against the simulator and against a mock owner, and is not yet
exercised on an arm; beyond that, no velocity-feedforward setting reachable through live
tuning has been validated on hardware. The Quest stack is verified on a Quest 3, with motion
scales and an axis mapping measured for one operator and one workspace; other headsets and
room layouts are not verified. See [peripheral status](./getting-started/peripherals.md), the
[impedance reference](./reference/impedance.md), and the status sections of
[live tuning](./howto/live-tuning.md#status) and [Quest teleoperation](./howto/vr-teleop.md#status). A `ros2_control` hardware interface,
vacuum gripper, and published Panda simulator image are not available.

This is an unofficial Apache-2.0 project, not affiliated with or endorsed by Franka
Robotics GmbH. Franka, Franka Emika, Panda, and FR3 identify the supported hardware.
