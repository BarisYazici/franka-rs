# franka-rerun

Replays `franka-rs` logs as [Rerun](https://rerun.io) recordings, and records a robot live.
Three modes:

| mode | input | what you get |
|---|---|---|
| `csv` | the CSV that `examples/nonrealtime_commander.rs --log` writes | target, commanded and measured positions, the derivatives of the commanded position against the rate limits, the commander's events, a 3D replay of the arm |
| `log` | a control log -- the `Vec<franka::Record>` a `ControlException` carries, saved as JSON with `franka_rerun::save_records` | a **flight recording**: contact and collision flags per joint and axis, the external wrench, commanded versus measured, every error and mode change, the arm with the joints coloured by their flags |
| live | a running control loop, through `franka_rerun::Recorder::push` | the same flight recording, streamed to a file or a viewer while the loop runs |

It is a library (`series`, `scene`, `commander`, `flight`, `recorder`) plus the
`franka-rerun` binary for the two replays. It needs Rust 1.96 for `rerun` 0.37 (the rest of
the workspace builds on 1.85) and is `publish = false`.

## Install the viewer

The viewer must match the SDK version this crate pins (`rerun = "=0.37.1"`); either:

```sh
cargo install rerun-cli --locked --version 0.37.1
# or
pip install rerun-sdk==0.37.1
```

Both put a `rerun` binary on the path. Check with `rerun --version`.

## `csv`: a commander log

```sh
# A log from the simulator or a robot (see the book, "Bridging a non-realtime commander"):
FRANKA_REALTIME=ignore cargo run --release -p franka-rs --example nonrealtime_commander -- \
    127.0.0.1 --bridged --yes --log bridged.csv

cargo run --release -p franka-rerun -- csv bridged.csv --robot fr3 -o bridged.rrd
rerun bridged.rrd
```

```
franka-rerun csv <log.csv> --robot fr3|fer [--urdf PATH] [-o out.rrd] [--every N]
```

- `--robot fr3` plots against the FR3 limits (`franka::MAX_TRANSLATIONAL_*`) and draws the
  arm from a URDF: `--urdf`, or by default the repository's
  `crates/franka-rs/tests/data/fr3.urdf`.
- `--robot fer` uses the FER limits (`franka::rate_limiting::fer`) and the crate's built-in
  FER model (`Model::native_fer()`); no URDF.
- `-o` defaults to the input path with `.rrd`; `--every N` logs every N-th row to the 3D
  scene (the time series always carry every row).

The tool prints the peak speed, acceleration and jerk norms of the commanded position next
to the limits, the raw target's implied speed (50 m/s at a 5 cm step) and, for the 3D scene,
the tool offset it identified (below).

Everything is on the `robot_time` timeline, the CSV's `t` in seconds since the motion
started.

| entity | content |
|---|---|
| `position/x`, `position/y`, `position/z` | three series each: the raw target (orange staircase), the commanded `O_T_EE_c` the robot echoed (blue), the measured `O_T_EE` (green) |
| `derivatives/speed`, `derivatives/acceleration`, `derivatives/jerk` | norms of the first, second and third finite differences of the commanded position, each with the robot's limit as a grey line at `.../limit` |
| `derivatives/target_speed` | the implied speed of the raw target, for the contrast with the commanded one |
| `events` | a text log: every target step, a `stall` when no target changes for more than 1.5 s, a `burst` when 5 or more changes fall within 100 ms, and `motion aborted by the robot` if the log ends before the command settled on the target |
| `world/*` | the 3D scene, Z up: a base box, the arm as a polyline through the origins of `Frame::ALL` (from the logged joint angles), the end effector frame with axes, the target (large orange point), the commanded (blue) and measured (green) points, and the whole measured path as a faint static trail |

The recording carries a blueprint, so the viewer opens with the 3D scene on the left, the
positions and derivatives in a 3x2 grid on the right (the raw target speed is a tab behind
the speed plot) and the event log along the bottom. Two things to know when reading it:

- **Tool offset.** The CSV carries no `F_T_EE`, so the arm's end effector is placed with a
  tool offset identified from the first row: the measured `O_T_EE` expressed in the model's
  flange frame (translation only). The tool prints the offset and the largest residual between
  the model's end effector and the measured position over the whole log; on franka-sim the
  residual is under half a millimetre.
- **Quantisation of the echo.** The derivatives are finite differences of the *echoed*
  `O_T_EE_c`. franka-sim echoes it with float32 granularity (about 6e-8 m at 0.5 m), which the
  second and third differences at 1 kHz amplify to about 0.06 m/s^2 and 200 m/s^3 of noise;
  where the limiter is saturated, the commanded acceleration and jerk therefore peek a percent
  or so above the grey lines. That is the echo's precision, not the limiter.

## `log`: a control log, the flight recording

```sh
cargo run --release -p franka-rerun -- log reflex.json --robot fer -o reflex.rrd
rerun reflex.rrd
```

```
franka-rerun log <records.json> --robot fr3|fer [--urdf PATH] [-o out.rrd] [--every N] [--force-scale M]
```

`--robot`, `--urdf`, `-o` and `--every` as above; `--force-scale` is the length of the
external force arrow in metres per newton (default 0.01). The JSON is what
`franka_rerun::save_records` writes: a `Vec<franka::Record>` through `serde_json` with the
`serde` feature of `franka-rs` (`Errors` as lists of names, `time` in milliseconds, floats
round-tripped exactly). The last record's `last_motion_errors` is taken as the reflex
reason for the closing `motion aborted` line.

Everything is on `robot_time`: `state.time` in seconds (the robot's millisecond counter), or
the record index in milliseconds when `time` never changes over the log.

| entity | content |
|---|---|
| `joints/q`, `joints/q_d` | measured joint positions; commanded ones -- the sent command's `q_c` when the cycle sent a joint-position command, else the robot's `q_d` |
| `joints/dq`, `joints/tau_J`, `joints/tau_J_d`, `joints/tau_ext` | joint velocities, measured torques, desired torques, `tau_ext_hat_filtered` |
| `ee/F_ext` | `O_F_ext_hat_K`: force (N) and torque (Nm) on the stiffness frame, in the base frame |
| `ee/position` | measured `O_T_EE` translation (`x`, `y`, `z`) against the commanded `O_T_EE_c` (`x_c`, `y_c`, `z_c`) |
| `flags/joint_contact`, `flags/joint_collision` | seven 0/1 series each, amber and red |
| `flags/cartesian_contact`, `flags/cartesian_collision` | six 0/1 series each, `Fx` .. `Tz` |
| `world/*` | the arm from the logged `q` and `F_T_EE`; a sphere per joint, grey, amber on contact, red on collision, its radius growing with `\|tau_ext\|`; the external force as an arrow from the end effector, coloured by the Cartesian flags; the end effector axes |
| `events` | every change of `current_errors` (which names were set, which cleared) and of `robot_mode`, the first rising edge of every flag (`joint 4 contact`, `cartesian collision on Fz`), and `motion aborted: <names>` at the last record |

The layout: the 3D scene on the left; on the right `q` and `q_d` in one plot, the torques
as tabs, `F_ext`, the position, `dq`, and the four flag plots as tabs; the events along the
bottom. The tool prints a `Summary`: records and span, rising edges per flag family, error
and mode changes, the peak `|F_ext|` and where, the peak `|tau_ext|` and on which joint.

## Live: `Recorder`

```rust,ignore
let recorder = Recorder::to_file("run.rrd".as_ref(), model, kind, RecorderOptions::default())?;
robot.control_joint_positions(|state, period| {
    let output = /* ... */;
    recorder.push(state, Some(RobotCommandLog { q_c: output.q, ..Default::default() }));
    output
}, mode, true, cutoff)?;
let stats = recorder.finish()?;   // Stats { pushed, dropped, summary }
```

`Recorder::to_file`, `Recorder::to_viewer("host:9876")` (a `rerun` viewer must already be
listening; `rerun` alone opens one on 9876) and `Recorder::spawn()` (starts one from
`PATH`), or `Recorder::with_stream` for any `RecordingStream`. `push` is meant for the
realtime thread: it copies the record into a bounded `sync_channel` with `try_send`, which
does not block and does not allocate (the ring is allocated once, with the recorder), and
counts the record as dropped when the ring is full. A background thread drains the channel
every 100 ms and does the Rerun work with the same `FlightLogger` the `log` replay uses,
the 3D scene decimated to every 10th record. `tests/flight.rs` runs a 1 kHz producer for
two seconds under a counting allocator: no drops, no allocations in `push`.

## The example: `reflex_replay`

```sh
FRANKA_REALTIME=enforce cargo run --release -p franka-rerun --example reflex_replay -- \
    <robot-hostname> [--yes] [--force N] [--torque NM] [--seconds S] [--live ADDR] [--out DIR]
```

Connects with a 3000-cycle control log, sets `--torque` (10 Nm) on every joint and
`--force` (10 N) on every Cartesian axis as the collision thresholds, with the contact
thresholds at half of that, refuses to run if joints 4 or 6 have no
room for the swing, starts a `Recorder` (`<out>/run.rrd`, or `--live ADDR`), prints all of
that and waits for Enter. Then joints 4 and 6 swing `0.2 (1 - cos(2 pi t / 12))` -- up to
+0.4 rad and back, 12 s period -- until the first multiple of 12 s at or after `--seconds`
(60), pushing every cycle to the recorder. Pushing the arm across a threshold trips a
reflex: the example prints the exception and its reflex names, writes `<out>/reflex.rrd`
(`replay_exception`) and `<out>/reflex.json` (`save_records`), runs
`automatic_error_recovery()`, and finishes the recorder. On a real FER it has recorded
24 s at 1 kHz without a push, 23 941 records pushed and none dropped.

## Library

`franka_rerun::series` logs time series from slices (`log_positions`, `log_derivatives`,
`log_scalar`, `differences`); `franka_rerun::scene` the 3D arm from a `franka::Model` and
joint angles (`skeleton`, `tool_offset`, `log_arm`, `log_skeleton`, `log_point`,
`log_static`); `franka_rerun::commander::CommanderLog` reads the CSV and records it
(`CommanderLog::record`, `commander::send_blueprint`); `franka_rerun::flight` has
`log_records`, `replay_exception`, `save_records`, `load_records`, `send_blueprint`, the
streaming `FlightLogger`, `FlightOptions` and `Summary`; `franka_rerun::recorder` has
`Recorder`, `RecorderOptions` and `Stats`. `RobotKind::limits()` selects the FR3 or FER
limits, and `RobotKind::from(robot.fci_version())` picks the kind from a connection.

## Provenance

The commander recordings this was developed against came from
[franka-sim](https://github.com/BarisYazici/libfranka-sim) with `--enforce-motion-limits`,
and the flight recorder against synthetic logs. Both have since been run on a real FER:
`csv` converted the commander's hardware logs, where the model's end effector matched the
measured `O_T_EE` to under 0.01 mm on every row, and the live `Recorder` ran through
`reflex_replay` without dropping a record.
