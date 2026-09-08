# Flight recorder

When a reflex stops a motion, the question is always the same: what happened in the
seconds before? Which joint reported contact, how large was the external wrench, was the
command running away from the measurement, and which error actually fired? The crate keeps
the raw material for that answer in the control log of every `ControlException`; the
`franka-rerun` crate turns it into a [Rerun](https://rerun.io) recording that can be scrubbed
through, and can record the same picture live while a loop runs.

> **Status.** Everything on this page is exercised against synthetic logs and the simulator's
> data layout in `crates/franka-rerun/tests/flight.rs`, and the live `Recorder` has run on a
> real FER (2026-09-08): `reflex_replay` recorded 24 s at 1 kHz without a push, 23 941 records
> pushed and 0 dropped, peak |F_ext| 4.5 N, no flags raised.
> The pushed run on the same arm, same day: a hand push on the hand during the swing raised
> `cartesian_reflex` 3.9 s in. The replay written from the `ControlException` holds the last
> 3000 records; in it the Cartesian contact flag on `Fy` rises at 5 N, the joint 3 contact
> flag 13 ms later, and the Cartesian collision flag at 10.6 N in the cycle before the robot
> stopped -- the joint, the axis and the direction of the push are all readable from the
> recording. 3928 records were pushed by the live `Recorder` with 0 dropped.

## The control log

`Robot::new` keeps the last **50** cycles (`franka::DEFAULT_LOG_SIZE`, libfranka's
default) of state and command in a ring buffer, and hands them over as
`ControlException::log` -- `Vec<Record>`, newest last, `Record { state, command:
Option<RobotCommandLog> }` -- when a motion ends abnormally. Fifty milliseconds is enough
to see the discontinuity that caused a rate-limit reflex, but not the approach that ended in
a collision. Make the ring longer:

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

The logger never allocates in the control loop (the ring is sized once), so a large log
costs memory, not cycle time. The details of the exception itself are in
[`ControlException` and the control log](./controlling-the-robot.md#controlexception-and-the-control-log).

## The `serde` feature

`franka-rs` has an optional `serde` feature (off by default) that derives `Serialize` and
`Deserialize` for `RobotState`, `RobotMode`, `Errors`, `Duration`, `Record`,
`RobotCommandLog`, `MoveStatus` and `ControlException`:

```toml
[dependencies]
franka-rs = { version = "0.1", features = ["serde"] }
```

Two representation choices are worth knowing:

- `Errors` serialises as the **list of the set flags' names** in libfranka's order --
  `["joint_reflex", "cartesian_reflex"]`, `[]` when nothing is set -- not as 41 booleans, so
  a JSON log is readable and does not need `serde-big-array`. Deserialising a name that is
  not one of `franka::ERROR_NAMES` is an error.
- `Duration` (`RobotState::time`) serialises as the bare millisecond count.

Everything else is field by field under the libfranka names. A `Vec<Record>` written with
`serde_json` is what `franka-rerun` calls a saved control log.

## Replaying a reflex

`franka-rerun` (a workspace crate, `publish = false`, needs Rust 1.96 for the Rerun SDK)
writes a control log to an `.rrd`:

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

`replay_exception` writes `e.log` with `e.last_motion_errors` as the closing `motion
aborted: ...` line; `save_records` / `load_records` move the log through JSON, and
`franka-rerun log reflex.json --robot fer` replays a saved one later. What ends up in the
recording, on the `robot_time` timeline (the robot's own clock, in seconds):

| entity | content |
|---|---|
| `joints/q`, `joints/q_d` | measured joint positions and the commanded ones -- the sent command's `q_c` for a joint-position motion, else the robot's `q_d` |
| `joints/dq`, `joints/tau_J`, `joints/tau_J_d`, `joints/tau_ext` | velocities, measured and desired torques, `tau_ext_hat_filtered` |
| `ee/F_ext` | `O_F_ext_hat_K`, force in N and torque in Nm |
| `ee/position` | measured `O_T_EE` against commanded `O_T_EE_c`, translation only |
| `flags/*` | `joint_contact`, `joint_collision` (7 series each), `cartesian_contact`, `cartesian_collision` (6 each) as 0/1; contact amber, collision red |
| `world/*` | the arm from `Model::pose_q`, a sphere per joint that turns amber on contact and red on collision and grows with `\|tau_ext\|`, the external force as an arrow from the end effector (1 cm per N by default), the end effector axes |
| `events` | every change of `current_errors` and `robot_mode`, the first rising edge of every flag (`joint 4 contact`, `cartesian collision on Fz`), and `motion aborted: <names>` at the end |

The recording carries a layout: the 3D scene on the left, the plots on the right (commanded
and measured `q` share one), the event log along the bottom. `flight::Summary` -- what
`replay_exception` returns and `Display`s -- has the counts (rising edges per flag family,
error and mode changes) and the peak `|F_ext|` and `|tau_ext|`.

## Recording live

`franka_rerun::Recorder` streams the same picture from inside a running loop:

```rust,ignore
use franka_rerun::{Recorder, RecorderOptions, RobotKind};

let recorder = Recorder::to_file("run.rrd".as_ref(), model, kind, RecorderOptions::default())?;
// or Recorder::to_viewer("127.0.0.1:9876", ...) with `rerun` already running,
// or Recorder::spawn(...) to start one from PATH.
robot.control_joint_positions(
    |state, period| {
        let output = /* ... */;
        recorder.push(state, Some(RobotCommandLog { q_c: output.q, ..Default::default() }));
        output
    },
    mode, true, cutoff,
)?;
let stats = recorder.finish()?;   // pushed, dropped, and the Summary
```

`push` runs on the realtime thread, so it must be cheap: it copies the record into a
bounded `std::sync::mpsc::sync_channel` (4096 records, four seconds at 1 kHz) with
`try_send`, which neither blocks nor allocates -- the channel's ring is allocated once, when
the recorder is created -- and drops the record, counted in `Stats::dropped`, when the ring
is full. A background thread drains the channel every 100 ms and does all the Rerun work,
with the 3D scene decimated to every 10th record so the viewer keeps up. The test suite
checks the no-allocation claim with a counting allocator around a 1 kHz producer.

## The example

`crates/franka-rerun/examples/reflex_replay.rs` puts it together: a 3000-cycle control log,
collision thresholds from `--force` / `--torque` (default 10 N / 10 Nm, applied to all
eight thresholds), a live recorder to `run.rrd` or `--live ADDR`, and a slow swing of
joints 4 and 6 that you are meant to push against. On the reflex it writes `reflex.rrd`
and `reflex.json`, recovers, and finishes the recorder. On a real FER it ran for 24 s
without a push with no records dropped (see the status note at the top of this page); the
[franka-rerun README](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-rerun)
has the flags.
