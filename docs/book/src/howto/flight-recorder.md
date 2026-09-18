# Record and replay a run

At the end of this page you can answer the question every reflex raises: what happened in
the seconds before? Which joint reported contact, how large was the external wrench, was the
command running away from the measurement, which error fired. The control log of every
`ControlException` holds the raw material; the workspace crate `franka-rerun` turns it into
a [Rerun](https://rerun.io) recording, and records the same picture live while a loop runs.

Prerequisites: a `rerun` viewer of exactly the version the crate pins, **0.37.1**
(`cargo install rerun-cli --locked --version 0.37.1` or `pip install rerun-sdk==0.37.1`);
`RUST_LOG=warn rerun` keeps its notification toasts to warnings. `franka-rerun` needs
Rust 1.96: `cargo install franka-rerun --locked` for the binary, `cargo add franka-rerun` for the
library.

## Keep a longer control log

`Robot::new` keeps the last **50** cycles (`franka::DEFAULT_LOG_SIZE`, libfranka's default)
of state and command in a ring and hands them over as `ControlException::log` (`Vec<Record>`,
newest last, `Record { state, command: Option<RobotCommandLog> }`) when a motion ends
abnormally. Fifty milliseconds shows the step behind a rate-limit reflex, not the approach
that ended in a collision. Make the ring longer:

```rust,no_run
# extern crate franka;
# use franka::{Robot, RealtimeConfig, RobotOptions};
# fn main() -> franka::FrankaResult<()> {
// Three seconds at 1 kHz; each record is a RobotState plus a command, about 2.5 KiB.
let robot = Robot::new_with_log_size("172.16.0.2", RealtimeConfig::Enforce, 3000)?;
// Or, through the options builder:
let robot = Robot::with_options(
    "172.16.0.2",
    RobotOptions::new(RealtimeConfig::Enforce).with_log_size(3000),
)?;
# Ok(()) }
```

The ring is sized once, so a large log costs memory, not cycle time.

## Save it: the `serde` feature

The optional `serde` feature of `franka-rs` (off by default;
`franka-rs = { version = "0.4", features = ["serde"] }`) derives `Serialize` and
`Deserialize` for `RobotState`, `RobotMode`, `Errors`, `Duration`, `Record`,
`RobotCommandLog`, `MoveStatus` and `ControlException`. Two representation choices: `Errors`
serialises as the list of the set flags' names in libfranka's order
(`["joint_reflex", "cartesian_reflex"]`, `[]` when none is set), not as 41 booleans, and a
name outside `franka::ERROR_NAMES` fails to deserialise; `Duration` (`RobotState::time`)
serialises as the bare millisecond count. Everything else is field by field under the
libfranka names. A `Vec<Record>` written with `serde_json` is what `franka-rerun` calls a
saved control log.

## Replay a reflex

```rust,ignore
use franka::FrankaError;
use franka_rerun::{flight, FlightOptions, RobotKind};

match robot.control_joint_positions(callback, mode, true, cutoff) {
    Err(FrankaError::Control(e)) => {
        let kind = RobotKind::from(robot.fci_version());
        let summary = flight::replay_exception(
            "reflex.rrd".as_ref(), &e, &model, kind, &FlightOptions::default())?;
        println!("{summary}");
        flight::save_records("reflex.json".as_ref(), &e.log)?;
        robot.automatic_error_recovery()?;
    }
    other => other?,
}
```

`replay_exception` writes `e.log` with `e.last_motion_errors` as the closing
`motion aborted: ...` line and returns a `flight::Summary` (rising edges per flag family,
error and mode changes, peak `|F_ext|` and `|tau_ext|`). `save_records` / `load_records`
move the log through JSON, and the binary replays a saved one later:

```sh
cargo run --release -p franka-rerun -- log reflex.json --robot fer -o reflex.rrd
rerun reflex.rrd
```

(`franka-rerun csv bridged.csv --robot fr3 -o bridged.rrd` replays the CSV of
`nonrealtime_commander --log`.) The recording opens with the 3D scene on the left, the plots
on the right and the event log along the bottom, all on the `robot_time` timeline, the
robot's own clock in seconds. `--prefix NAME` puts every entity below under `NAME`, as a node
does with an arm's name, so that two replays can be routed into one recording:

| entity | content |
|---|---|
| `joints/q`, `joints/q_d` | measured joint positions and the commanded ones (the sent `q_c` for a joint-position motion, else the robot's `q_d`) |
| `joints/dq`, `joints/tau_J`, `joints/tau_J_d`, `joints/tau_ext` | velocities, measured and desired torques, `tau_ext_hat_filtered` |
| `joints/q_goal`, `joints/dq_goal`, `joints/cap_scale`, `joints/tau_envelope` | a live recording of target control's torque backend (`Recorder::push_torque_at`, which franka-node uses): the joint goal, its velocity, the scale the joint velocity cap cut the goal's step by (1 when it did not), the velocity envelope's torque: the barrier's, less the law's torque faded out along a joint's motion above the fade's start |
| `ee/F_ext` | `O_F_ext_hat_K`, force in N and torque in Nm |
| `ee/position` | measured `O_T_EE` against commanded `O_T_EE_c`, translation only; per-axis plots and the derivatives of the sent position when the command was a Cartesian pose |
| `ee/orientation` | the rotations of those two poses as quaternions `xyzw`, every cycle (the 3D scene's pose is decimated by `--every`) |
| `flags/*` | `joint_contact`, `joint_collision` (7 series each), `cartesian_contact`, `cartesian_collision` (6 each) as 0/1; contact amber, collision red |
| `world/*` | the arm from `Model::pose_q`, a sphere per joint that turns amber on contact and red on collision and grows with `\|tau_ext\|`, the external force as an arrow from the end effector (1 cm per N by default), the end effector axes; Franka's link meshes (built in; `--meshes DIR` for other files, `--no-meshes` for none) |
| `world/contact/*`, `contact/link` | where the seven external joint torques say the arm was touched, and the force there |
| `events` | every change of `current_errors` and `robot_mode`, the first rising edge of every flag (`joint 4 contact`, `cartesian collision on Fz`), and `motion aborted: <names>` at the end |

## Record live

`franka_rerun::Recorder` streams the same picture from inside a running loop:

```rust,ignore
use franka_rerun::{Recorder, RecorderOptions, RobotKind};

let recorder = Recorder::to_file("run.rrd".as_ref(), model, kind, RecorderOptions::default())?;
// or Recorder::to_viewer("127.0.0.1:9876", ...) with `rerun` already running,
// or Recorder::to_viewer_and_file(...), or Recorder::spawn(...) to start one from PATH.
robot.control_joint_positions(
    |state, period| {
        let output = /* ... */;
        recorder.push(state, Some(RobotCommandLog { q_c: output.q, ..Default::default() }));
        output
    },
    mode, true, cutoff,
)?;
let stats = recorder.finish()?;   // Stats { pushed, dropped, summary }
```

`push` runs on the realtime thread, so it must be cheap: it reads the host's `CLOCK_MONOTONIC`
and copies the record into a bounded `std::sync::mpsc::sync_channel` of 4096 records (four
seconds at 1 kHz) with `try_send`, which neither blocks nor allocates (the ring is allocated
once, with the recorder), and drops the record, counted in `Stats::dropped`, when the ring is
full. Every row it writes carries that host stamp as a second timeline, `host_time`: two robots
are two controllers, so their `robot_time`s are unrelated, and `host_time` is the axis a
recording of both is synchronised on. An offline replay has no host clock for a record and
writes `robot_time` alone. A background thread
drains the channel every 100 ms and does all the Rerun work, the 3D scene decimated to every
10th record. `crates/franka-rerun/tests/flight.rs` checks the no-allocation claim with a
counting allocator around a 1 kHz producer. For a loop the crate runs for you, the observer
of [target control](./target-control.md) is the hook.

## The examples

- `crates/franka-rerun/examples/reflex_replay.rs`: a 3000-cycle log, collision thresholds
  from `--force` / `--torque` (default 10 N / 10 Nm, contact at half), a `Recorder` to
  `<out>/run.rrd` or `--live ADDR`, then joints 4 and 6 swing by `0.2 (1 - cos(2π t / 12))`
  for `--seconds` (60) while you push. On the reflex it writes `reflex.rrd` and
  `reflex.json`, recovers, and finishes the recorder.
- `crates/franka-rerun/examples/commander_live.rs`: the commander of `nonrealtime_commander`
  on target control, streamed into an open viewer (`--live ADDR`, `--out FILE`,
  `--meshes DIR`, `--bridged` or `--raw`).

## Status

Everything above is exercised against synthetic logs in `crates/franka-rerun/tests/flight.rs`.

**Known limitation.** `Recorder::finish()` joins the background thread, which ends with the
stream's `flush_blocking()`; neither has a timeout (`crates/franka-rerun/src/recorder.rs`,
`flight/logger.rs`). With `to_viewer` and a viewer that is not reachable, `finish()` can
therefore hang; Rerun's Python SDK gives up after a few seconds instead. Dropping the
recorder without `finish()` closes the channel and does not wait. Record to a file when the
viewer may not be up.
