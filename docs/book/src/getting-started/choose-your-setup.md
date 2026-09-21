# Choose your setup

At the end of this page you know which of the two ways of running franka-rs fits your lab,
and which pages to read next. Nothing on this page touches a robot.

## What every setup has

Whatever you choose, one program somewhere runs the 1 kHz loop and talks to the robot's
control unit over a wired Ethernet link. That program is the crate `franka-rs` (in Rust or
through the `franka` Python package) or the `franka-node` binary that wraps it. The machine it
runs on is *the realtime machine*: it needs a `PREEMPT_RT` kernel, the right to schedule a
thread as `SCHED_FIFO` and a cable to the robot. [The realtime machine](./realtime-machine.md)
lists these requirements; they do not change with the setup.

Everything else, the policy, the notebook, the joystick, the cameras and the recording, is a
choice.

## Setup A: one machine runs everything

```text
your program (Rust or Python) ── franka-rs ── wired Ethernet ── robot
```

The commander and the 1 kHz loop are one process on one machine, usually a PC on the robot's
network with a `PREEMPT_RT` kernel. Your program sets targets at its own rate, and the crate
turns them into a continuous command on a realtime thread it owns.

Choose this when:

- the machine next to the robot can run your program as well (a realtime PC with a GPU, say);
- you want the `RobotState` at 1 kHz, an observer on the loop or your own 1 kHz callback;
- you are starting out and want the shortest path from install to the first motion.

Read next: [Install](./install.md), [The realtime machine](./realtime-machine.md),
[First program](./first-program.md) or [From Python](./python.md).

## Setup B: a small box next to the arm, your program anywhere

```text
laptop or workstation ── Zenoh (your network) ── Raspberry Pi 5 running franka-node ── wired Ethernet ── robot
```

A Raspberry Pi 5 with a `PREEMPT_RT` kernel sits on the robot's network and runs
`franka-node`. It owns the robot connection and the 1 kHz loop. Your program runs on any other
machine, in any language with a [Zenoh](https://zenoh.io) binding, and needs no realtime kernel,
no `franka-rs` dependency and no cable to the robot. It publishes targets over Zenoh and reads
the arm's state back.

Choose this when:

- your program lives on a laptop over Wi-Fi, a workstation without a realtime kernel, or in a
  notebook;
- several programs take turns on one arm, or one small host should serve two arms to the lab;
- you want the robot side to be a service that starts at boot and keeps running.

This adds one network hop per target and a service to run. The node has local watchdogs
and lease handling for a disconnected commander; link delay still affects how responsive
your application feels. Use a trusted network and read the
[node's stop behavior](../howto/franka-node.md) before streaming targets.

Install the node and its Python client at the same version: both from one release, or both
from one source revision.

Read next: [Run the control loop on a Raspberry Pi 5](./raspberry-pi.md), then
[Serve arms over Zenoh: franka-node](../howto/franka-node.md) for the full picture.

### What Zenoh is here, and what it is not

Zenoh is the messaging layer between your program and the node: a publish/subscribe and
request/reply protocol with client libraries for Rust, Python, C, C++ and others. It carries
targets one way and state the other. It is **not** a second controller: nothing in Zenoh
interprets a target, checks a limit or holds the arm. All of that happens in `franka-node` on
the Pi, and the 1 kHz loop never sees Zenoh at all.

For a direct connection between your program and the Pi, on the same network or reachable by
address, **no separate Zenoh router is required**. The node listens on a TCP endpoint and the
client connects to it. A router only enters the picture when the node must dial out, for
example from behind NAT, which the how-to page covers under `mode = "client"`.

## Optional parts, in either setup

| part | what it adds | where it is described |
|---|---|---|
| Franka Hand gripper | open, close and grasp over the hand's own connection on port 1338; in setup B, served by the node under the arm's lease | [Peripherals](./peripherals.md), [Use the gripper](../howto/gripper.md) |
| USB cameras | frames with capture timestamps, published over Zenoh by `franka-cam` next to the node, so a frame can be lined up with the arm state of the same instant | [Peripherals](./peripherals.md) |
| Recording | every 1 kHz cycle, the commands and the accepted targets as a [Rerun](https://rerun.io) file per session; camera frames can join the same recording | [Peripherals](./peripherals.md), [Record and replay a run](../howto/flight-recorder.md) |
| The tuning panel | gains, inverse-kinematics damping and the motion budgets of a running Cartesian session, changed from a browser within the node's own bounds; in setup B it is the `franka-tuning-panel` command of `franka-node-client`, in setup A your program changes the same parameters through the library | [Tune a running controller](../howto/live-tuning.md) |
| Quest teleoperation | a Meta Quest controller drives one arm, clutched on the grip, with every target clamped against the arm's measured pose; setup B only, since it commands an arm served by the node | [Teleoperate with a Quest](../howto/vr-teleop.md) |
| The simulator | a MuJoCo robot that speaks the same wire protocol, for trying either setup without an arm | [Without a robot: franka-sim](./simulator.md) |

## A decision in one table

| question | setup A | setup B |
|---|---|---|
| where does the 1 kHz loop run? | in your program's process | in `franka-node` on the Pi |
| what needs `PREEMPT_RT`? | the machine running your program | the Pi only |
| what does your program link? | `franka-rs` or `pip install franka-rs` | a Zenoh client; in Python `franka-node-client` |
| can it run on macOS or Windows? | no | yes, the client side |
| can a second program take over the arm? | not while the first is connected | once the first releases it or its lease lapses |
| 1 kHz state, observer hook, own callback | yes | no; state at 100 Hz |

If in doubt and you have one PC on the robot's network, start with setup A and the
[first program](./first-program.md). Moving to setup B later changes the transport, not the
control: the node runs the same target control the first program does.
