# Serve arms over Zenoh: franka-node

At the end of this page a small machine next to the robots runs `franka-node`, and a program
on another machine, in any language with a [Zenoh](https://zenoh.io) binding, takes an arm,
streams pose or joint targets to it and reads its state, without linking `franka-rs` or
running a realtime kernel itself. The crate is `franka-node` on crates.io and `crates/franka-node`
in the repository; its [README](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node)
has the byte-exact wire tables, the verb table and every configuration key. This page is the
guided tour.

Prerequisites: [Command from a low-rate program](./target-control.md), whose loop the node
runs; a host on the robot network that meets [The realtime machine](../getting-started/realtime-machine.md)
(a Raspberry Pi 5 with a `PREEMPT_RT` kernel does); and the simulator or a robot.

## What it is

`franka-node` is one process that owns one or more `franka::Robot`s and, per arm, runs
`start_cartesian_target_control` or `start_joint_target_control` with the impedance backend.
Clients publish 80-byte targets at any rate on a Zenoh key; the node gates each one (lease
holder, order, kind, finite values, step from the previous accepted target, workspace box,
joint limits, rate) and hands it to the library's generator, which does the 1 kHz work on the
realtime thread the library owns. The arm's state goes back out at 100 Hz as a fixed-layout
message, and `acquire`, `enable`, `stop`, `release`, `recover` and `home` are request/reply
queryables with JSON payloads.

A client holds an arm through a *lease*: a Zenoh liveliness token it declares while it
lives. The token going away, because the client exited, was killed or lost the link, stops
the arm; so does silence: no accepted target for `hold_after_ms` (200 ms) flags the state as
holding, no target at all for `stop_after_ms` (2 s) stops the loop, and the robot's own
controller holds the arm where it is. A second client is refused while the first holds the lease.

## The data path

```text
Zenoh RX ──send──▶ mpsc channel, at most 64 targets in flight ──▶ arm thread (one per arm:
                                                                   robot, control handle,
                                                                   guard, state machine,
                                                                   watchdog)
                                                   │ set_target
                                                   ▼
                                       1 kHz SCHED_FIFO loop (the library's)
                                                   │ state()
arm thread, every 10 ms: StateMsg ──▶ Zenoh publisher
arm thread, at a session's start and end: JSON ──▶ franka/<arm>/episode
status thread, every 1 s: JSON ──▶ franka/node/<name>/status
```

The Zenoh callbacks decode a fixed layout with `zerocopy` and send without blocking; the arm
thread drains its channel, runs commands and lease events in arrival order and judges only
the newest target of a drain (the rest count as `dropped`). The realtime loop never sees
Zenoh, the channel or an allocation from the node; the node never sees the robot's UDP
stream. This is the same design as the library's own target control, one process boundary
further out.

## When to use it, and when the library

Use the node when the commander is not on the realtime machine (a laptop over Wi-Fi, a
workstation running a policy, a notebook), when it is not written in Rust or Python, when
several programs take turns on one arm, or when one small host should serve two arms to the
rest of the lab. Its costs are a network hop per target (the state echoes the accepted
target's `t_send_ns`, so a client measures its own round trip; the `client` example prints
it) and one more process to run.

Use the library directly, [target control](./target-control.md) or the
[Python bindings](../getting-started/python.md), when the commander runs on the realtime
machine anyway, needs the `RobotState` at 1 kHz or an observer on the loop, or wants the
robot's controller (`Backend::RobotController`), which the node does not offer.

## Configuration

One TOML file; every key has a default except an arm's `name` and `host`:

```toml
name = "node1"

[zenoh]
listen = ["tcp/0.0.0.0:7447#iface=eth0"]
scouting_interface = "eth0"          # optional; "auto" when absent

[[arm]]
name = "fr3"
host = "172.16.0.2"
realtime = "enforce"                 # or "ignore" without a PREEMPT_RT kernel
collision_force = 40.0               # N; set_collision_behavior_simple(torque, force)
collision_torque = 40.0              # Nm
budget = [0.3, 0.5, 20.0]            # translation limits, a norm: m/s, m/s², m/s³
max_step = 0.05                      # m per target from the previous accepted one
max_lead = 0.05                      # m a target may lead the measured pose; 0 disables
workspace = { min = [0.2, -0.5, 0.0], max = [0.8, 0.5, 0.8] }
# cpu = 2                            # pin the loop thread to this core (isolcpus)
```

`name` is the node's, in its status key; the arm's `name` is the `<arm>` of every key
(`franka/fr3/target`, `franka/fr3/state`, ...). A second `[[arm]]` table is a second arm in
the same process, each with its own thread, loop and, with `cpu`, core. The full key list
with the stiffness, leash, deviation and joint keys is in the README and in
`crates/franka-node/config.example.toml`, which the config tests parse.

## Running it

```sh
cargo run --release -p franka-node -- node.toml
RUST_LOG=debug cargo run --release -p franka-node -- node.toml    # refusals, with their reasons
```

Transitions (`idle -> acquired (acquired by 7)`) are logged at `info`, refusals at `debug`.
SIGINT or SIGTERM stops every arm through the library's settle and exits. Under
`realtime = "enforce"` the node constructs each robot on a scratch thread, so `Robot::new`'s
`SCHED_FIFO` promotion lands nowhere that outlives it: the node's own threads stay at normal
priority and only the library's loop thread is realtime.

## The client

`examples/client` in the crate is a commander for the node: it declares its lease, acquires
and enables the arm, moves the end effector along a sine in z (or joint 7, with
`--mode joints`), prints the node's report once a second and stops and releases at the end;
`client <arm> home` drives the arm to the ready pose, `client <arm> recover|stop|release`
sends one verb, and `client <arm> gripper ...` commands the arm's gripper.

```sh
cargo run --release -p franka-node --example client -- fr3 --connect tcp/<node-host>:7447 \
    [--hz 20] [--dz 0.05] [--seconds 4] [--mode joints]
cargo run --release -p franka-node --example client -- fr3 home --connect tcp/<node-host>:7447
```

Its `common.rs` is the client side of the keys in about 200 lines; the simulator test
`tests/sim_node.rs` shares it and is the reference sequence: token, `acquire` (retried while
the token is still in flight), `enable`, targets, `stop`.

## From Python

`pip install franka-node-client` installs the Python client, imported as `franka_node`: pure
Python over `eclipse-zenoh` and numpy, so it runs wherever those do, macOS and Windows included.
This takes an arm, raises the end effector by 5 cm, waits until it is there and opens the gripper:

```python
import franka_node

node = franka_node.Node("tcp/<node-host>:7447")         # or Node(session=my_zenoh_session)
with node.arm("fr3") as arm:                              # lease token + acquire, release on exit
    with arm.cartesian_targets() as t:                    # enable; stop on exit
        t.move_by([0.0, 0.0, 0.05])                       # +5 cm z, relative to the goal
        state = t.wait(timeout=5.0)                       # until the arm is there
        print(state.position, state.quaternion, state.refused)
    arm.gripper.move(0.04)
```

The simulator test `crates/franka-node/python/tests/test_sim_node.py` runs the same sequence in
CI. `with node.arm(...)` declares the lease token under a random client id of the `Node`, acquires
(retrying while the token is still in flight) and on exit stops this client's session, releases
the arm if this client still holds it and undeclares the token; a second client's `acquire`
raises `Refused` with the node's `"held by <id>"`. The verbs and conventions are those of the
[Python bindings](../getting-started/python.md): quaternions `x, y, z, w`, rotation vectors in the
base frame, `move_to`, `move_by`, `follow(chunk, dt)`, `target()`, `state()` and `stop()`.

A session runs one streamer thread. Every `1 / rate` s (50 Hz) it publishes one target, moved
toward the goal by at most `max_velocity / rate` (0.1 m/s, 2 mm per message) and kept within
`lead` (0.03 m) and `angular_lead` (0.15 rad) of the measured pose; at the goal it repeats the
target, which is the keepalive `hold_after_ms` and `stop_after_ms` need. `move_to`, `move_by` and
`follow` only set the goal and never block. `wait()` returns the state once the node has accepted
a target equal to the goal and the arm is within 5 mm and 0.05 rad of it (0.01 rad per joint in a
joints session). Every Cartesian target carries the anchor flag. When the node ends the session
(phase no longer active, another holder after a lost lease or a node restart, or someone's `stop`),
every later call raises `SessionEnded`, a `wait()` already blocked included; the client never
re-acquires on its own. A node that publishes no state for three state periods, and at least 2 s, while
the streamer itself keeps running ends the session the same way, with phase `"unknown"`. A rising `refused` count, targets and gripper commands together, is logged
at `WARNING` on the `franka_node` logger at most once a second; the reasons are in the node's debug
log.

A streamer starved of the GIL for longer than `stop_after_ms` ends the session. Lower `lead` on a
high-latency link: the node checks the lead against a newer pose than the client has. A Faulted
arm needs `arm.recover()` before `acquire` or a new session.

```python
with node.arm("fr3") as arm:
    arm.home(speed=0.2)                                   # blocks until the ready pose
    with arm.joint_targets(max_velocity=0.5) as t:        # rad/s per joint
        t.move_by([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1])    # joint 7 by 0.1 rad
        t.wait(timeout=5.0)
    arm.gripper.homing()
    held = arm.gripper.grasp(0.02, 20.0)                  # True when an object is held
```

A target is refused, counted and logged rather than executed when it steps more than `max_step`
from the previous accepted one, leads the measured pose by more than `max_lead`, leaves the
workspace box, or comes from a client that does not hold the lease. The client's defaults stay
inside the node's: 2 mm per message against `max_step` 0.05 and under the 0.173 m/s per-axis
budget, `lead` 0.03 m under `max_lead` 0.05, and in a joints session at most 0.15 rad per message
against `max_step_joint` 0.2; raise `max_velocity` or `lead` only with the node's limits.

`max_step` is a distance per message, not a speed, so a commander faster than the arm would
otherwise walk the target ahead of it without limit and the arm would coast the whole lead when
the commander stopped; `max_lead` is what bounds that, and the client's `lead` keeps its stream
under it. The anchor flag is the way back in when a stream has jumped far enough that every later
target is refused: that one target's step is measured from the arm's measured pose instead of from
the previous accepted target. It lifts no limit -- an anchored target must still be within
`max_lead` of the arm -- which is why the client sets it on every message, and with
`max_lead = 0` the flag is ignored rather than left unbounded. A joints session has no anchor, so
the client re-seeds its target from the node's last accepted one when a refusal comes back.

A commander whose targets are all being refused is held but not stopped: `hold_after_ms`
measures from the last target the guard *accepted*, `stop_after_ms` from the last one that
*arrived* from the lease holder. A commander outrunning the arm, parked at the workspace wall
or pushing against a hand is alive, and stopping its session would cost it the loop for the
length of the settle.

A client in another language speaks the byte layouts of the README's
[Wire messages](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node#wire-messages);
`crates/franka-node/tests/wire.json` is the same layout, machine-readable.

## Installing

```sh
cargo binstall franka-node             # prebuilt, from the GitHub release; no compiler needed
cargo install franka-node --locked     # or compile it where it runs, a Raspberry Pi included
```

[`cargo binstall`](https://github.com/cargo-bins/cargo-binstall) fetches
`franka-node-<version>-<target>.tar.gz` from the release: `aarch64-unknown-linux-gnu` (glibc
2.31 or newer, so Raspberry Pi OS Bookworm), the static `aarch64-unknown-linux-musl` and
`x86_64-unknown-linux-gnu`, each built with `record`. A compiled `cargo install` needs Rust 1.89
and records only with `--features record` (Rust 1.96). Without cargo, unpack the tarball itself,
which also holds `config.example.toml` and the systemd unit below:

```sh
tag=$(curl -fsSLo /dev/null -w '%{url_effective}' https://github.com/BarisYazici/franka-rs/releases/latest)
tag=${tag##*/}
curl -fsSL https://github.com/BarisYazici/franka-rs/releases/download/$tag/franka-node-${tag#v}-aarch64-unknown-linux-gnu.tar.gz | tar -xz
```

`franka-cam` is released the same way.

## Building for a Raspberry Pi

From a checkout, the node cross-compiles like the rest of the workspace ([Build for another machine](./cross-compile.md)):

```sh
cargo zigbuild --release --target aarch64-unknown-linux-gnu -p franka-node \
    --bin franka-node --example client
```

The result is `target/aarch64-unknown-linux-gnu/release/franka-node` and
`.../examples/client`, dynamically linked against glibc; that page's system cross compiler
gives the same. On the Pi the same realtime prerequisites apply as for any program using
the crate: a `PREEMPT_RT` kernel and the limits Franka's realtime setup recommends (`rtprio`,
`memlock`), which the unit below sets for the service.

## The systemd unit

`crates/franka-node/deploy/franka-node.service` runs the node as a service: after
`network-online.target`, restarted two seconds after a failure, with `LimitRTPRIO=99` and
`LimitMEMLOCK=infinity` (the limits Franka's realtime setup recommends), stopped with SIGINT
so every arm goes through its settle before the process exits (`TimeoutStopSec=20`: the
arms stop one after the other, each within the library's 5 s stop timeout). Set `User=` to
the account that runs the node (`Group=` is `realtime`; an unedited copy refuses to start),
install the binary at `/usr/local/bin/franka-node` and the config at
`/etc/franka-node/node.toml` (from a release tarball, or `deploy/README.md`'s steps), then:

```sh
sudo systemctl enable --now franka-node
journalctl -u franka-node -f
```

`deploy/README.md` next to the unit has the copy steps.

## Status

A commander that is not on the robot's network needs no inbound rule: set `mode = "client"` in
the `[zenoh]` table and name a router in `connect`, and the node dials out instead of listening.
Use a `tls/` or `quic/` endpoint once the link leaves the local segment. Distance costs nothing
in safety: the generator turns a stalled target stream into a slower motion, and the hold and
stop watchdogs and the lease behave as they do on the bench.

The node publishes its own health on `franka/node/<name>/status` once a second as JSON:

```json
{"node": "node1", "version": "0.1.0", "uptime_s": 61,
 "arms": [{"name": "fr3", "phase": "active", "holder": 7, "mode": "cartesian",
           "accepted": 1210, "refused": 0, "dropped": 0, "decode_failures": 0,
           "success_rate": 0.99}]}
```

The fields are in the crate README; a stalled arm thread shows as a frozen entry, a missing
sample means the node is gone.

## Grippers

An arm's `[[arm]]` table may name a gripper, `gripper = "hand"` for the Franka Hand at the
arm's host. The node then takes 40-byte width or grasp commands on
`franka/<arm>/gripper/target` from the arm's lease holder, in any phase (the arm need not be
enabled), publishes the gripper's width, commanded width, stroke and flags on
`franka/<arm>/gripper/state` at 20 Hz, and answers `cmd/gripper_home` (the holder's, when the
homing is over) and `cmd/gripper_stop` (anyone's); the status gains a `gripper` entry. The
arm's own stop, watchdog and lease loss leave the gripper where it is. Widths are metres
throughout; a 0..1 convention is the client's. Inside, the driver is a small `Gripper` trait
the arm thread owns, and `franka_node::run` takes a factory that maps the config's name to a
driver, so a binary of your own can add one for another gripper and keep the rest of the
node. The [README's Grippers section](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node#grippers)
has the wire tables, the verb rules and the Hand driver's
behaviour; `client <arm> gripper 0.04`, `gripper grasp 0.02 20` and `gripper home` in the
client example exercise them.

## Recording

Built with `--features record` and given `record_dir`, the node writes every session (an
`enable`, a `home`) as one Rerun `.rrd`, `<arm>-<UTC stamp>.rrd`: the
[flight recording](./flight-recorder.md) of every 1 kHz cycle and the command sent, plus the targets
the node accepted and refused, opened and closed with the loop and named in the status's
`recording`. Every entity is under the arm's name, and every row carries `robot_time` and
`host_time`, the host's `CLOCK_MONOTONIC` where the record was taken.

`enable` and `home` take `"episode": "<name>"` (`[A-Za-z0-9_-]{1,128}`). With one, the session's
recording id is that name and the arm's file is `<name>-<arm>.rrd`, so two arms enabled with the
same name write two files a viewer loads as one recording, synchronised on `host_time` -- the only
clock the two controllers share. The client example passes it with `--episode NAME`. The
[README's Recording section](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node#recording)
has the details and how to open a file.

Either way, recording or not, every session's start and end go out on `franka/<arm>/episode`
as one JSON line carrying the session's `recording_id` and, when there is one, its file name.
With a recording that id is the Rerun `RecordingId` of the arm's `.rrd` -- the `episode` name,
or the file's stem without one; without a recording it is generated in the same shape. A process that records something else at the same time, a camera
for instance, writes its own file under that id and the application id `franka_rs`, and a
viewer loads the two as one recording. Both samples are best effort, as every publisher here
drops rather than blocks, so read whether a session is running from the state topic's `phase`.

What runs on hardware and what is tested on franka-sim only is in the
[README's Status section](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node#status).
