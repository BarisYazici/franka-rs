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
franka-rerun csv <log.csv> --robot fr3|fer [--urdf PATH] [-o out.rrd] [--every N] [--meshes DIR] [--layout default|demo] [--budget V,A,J]
```

- `--robot fr3` plots against the FR3 limits (`franka::MAX_TRANSLATIONAL_*`) and draws the
  arm from a URDF: `--urdf`, or by default the repository's
  `crates/franka-rs/tests/data/fr3.urdf`.
- `--robot fer` uses the FER limits (`franka::rate_limiting::fer`) and the crate's built-in
  FER model (`Model::native_fer()`); no URDF.
- `-o` defaults to the input path with `.rrd`; `--every N` logs every N-th row to the 3D
  scene (the time series always carry every row); `--meshes DIR` draws the arm with Franka's
  link meshes (below); `--layout demo` swaps the viewer layout for the screen-capture one
  (below); `--budget V,A,J` is the commander's own velocity, acceleration and jerk budget
  (m/s, m/s^2, m/s^3; the `--budget` of `nonrealtime_commander`), the limit lines of the
  `processed/*` plots -- by default the robot's limits.

The tool prints the peak speed, acceleration and jerk norms of the commanded position next
to the limits, the raw target's implied speed (50 m/s at a 5 cm step) and, for the 3D scene,
the tool offset it identified (below).

Everything is on the `robot_time` timeline, the CSV's `t` in seconds since the motion
started.

| entity | content |
|---|---|
| `position/x`, `position/y`, `position/z` | three series each: the raw target (orange staircase), the commanded `O_T_EE_c` the robot echoed (blue), the measured `O_T_EE` (green) |
| `raw/x`, `raw/y`, `raw/z` | the raw target per axis as the user's staircase (`raw`, thick orange, drawn unaggregated so its corners stay corners at any zoom) |
| `raw/speed`, `raw/acceleration`, `raw/jerk` | norms of the finite differences of that staircase per cycle (`raw`, thin orange, unaggregated): 50 m/s, 50 000 m/s^2 and 5e7 m/s^3 at a 5 cm step, a line off the top of any plot of the processed ones |
| `processed/x`, `processed/y`, `processed/z` | the commanded position per axis (`processed`, blue) |
| `processed/speed`, `processed/acceleration`, `processed/jerk` | norms of the sent command's velocity, acceleration and jerk -- the generator's own `cmd_vx..cmd_az` columns when the CSV has them (the example logs them; jerk is the finite difference of that acceleration), else the finite differences of the commanded position -- each with the `--budget` (or the robot's limit) as a thin red line at `.../limit` |
| `derivatives/speed`, `derivatives/acceleration`, `derivatives/jerk` | norms of the first, second and third finite differences of the commanded position, each with the robot's limit as a grey line at `.../limit` |
| `derivatives/target_speed` | the implied speed of the raw target, for the contrast with the commanded one |
| `events` | a text log: every target step, a `stall` when no target changes for more than 1.5 s, a `burst` when 5 or more changes fall within 100 ms, and `motion aborted by the robot` if the log ends before the command settled on the target |
| `world/*` | the 3D scene, Z up: a base box, the arm as a polyline through the origins of `Frame::ALL` (from the logged joint angles), the end effector frame with axes, the target (large orange point), the commanded (blue) and measured (green) points, the whole measured path as a faint static trail (`world/measured_path`), and the last two seconds of it as `world/trail`, five strips fading with age, re-logged every five rows so the old path falls off as time advances |

The recording carries a blueprint, so the viewer opens with the 3D scene on the left, the
positions and derivatives in a 3x2 grid on the right (the raw target speed is a tab behind
the speed plot) and the event log along the bottom. `--layout demo` carries the layout made
for a screen capture instead: the arm at full height on the left (55 % of the width, the
meshes, the target and sent points and the fading `world/trail`, no skeleton and no static
path, a fixed three-quarter front camera on a solid dark background), and on the right six stacked
plots: `x`, `y`, `z` with the raw staircase over the processed position (y range
automatic), then the speed of both and the acceleration and jerk of the processed one
under the red `--budget` lines, each with a fixed y range of 1.25 times its line so the
processed command visibly sits under it (in the speed plot the raw one leaves the plot at
every step); legends top right, no event log,
the blueprint and selection panels hidden, the time panel collapsed to its controls and
playing on a loop. The SDK's blueprint builders have
no setter for a view's own properties (`ScalarAxis`, `EyeControls3D`, `PlotLegend`), so
`franka_rerun::demo` assembles that blueprint from the archetypes directly. Two things to
know when reading a replay:

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
franka-rerun log <records.json> --robot fr3|fer [--urdf PATH] [-o out.rrd] [--every N] [--meshes DIR] [--force-scale M] [--noise-floor NM]
```

`--robot`, `--urdf`, `-o`, `--every` and `--meshes` as above; `--force-scale` is the length
of the external force arrows in metres per newton (default 0.01), `--noise-floor` the
external joint torque below which the contact estimate stays quiet (default 1 Nm). The JSON
is what
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
| `ee/position/x`, `ee/position/y`, `ee/position/z` | for records whose command carries an `O_T_EE_c` (a Cartesian pose loop): the *sent* position against the measured one, per axis |
| `ee/derivatives/speed`, `ee/derivatives/acceleration`, `ee/derivatives/jerk` | for the same records: norms of the first, second and third finite differences of the sent position across consecutive records (carried across batches when streaming), each with the robot's limit as a grey line at `.../limit` |
| `flags/joint_contact`, `flags/joint_collision` | seven 0/1 series each, amber and red |
| `flags/cartesian_contact`, `flags/cartesian_collision` | six 0/1 series each, `Fx` .. `Tz` |
| `world/*` | the arm from the logged `q` and `F_T_EE`; a sphere per joint, grey, amber on contact, red on collision, its radius growing with `\|tau_ext\|`; the external force as an arrow from the end effector, coloured by the Cartesian flags; the end effector axes; a faint wireframe box around the workspace (so that a live viewer frames the arm's reach from the first frame, before the poses arrive); with `--meshes`, the link meshes under `world/links`; `world/commanded`, the sent position, for Cartesian commands |
| `world/contact/estimate`, `world/contact/force` | where the external joint torques say the arm is being touched, and the force there (below); present while a flag is set or a torque exceeds the noise floor, coloured like the flags, the marker growing with the force |
| `contact/link` | the estimated link over time |
| `events` | every change of `current_errors` (which names were set, which cleared) and of `robot_mode`, the first rising edge of every flag (`joint 4 contact`, `cartesian collision on Fz`), the first contact estimate and the one at the collision, and `motion aborted: <names>` at the last record |

The layout: the 3D scene on the left; on the right `q` and `q_d` in one plot, the torques
and the contact link as tabs, `F_ext`, the position (with the per-axis plots and the
derivatives of a Cartesian command as tabs behind it), `dq`, and the four flag plots as
tabs; the events along the bottom. The tool prints a `Summary`: records and span, rising edges per
flag family, error and mode changes, the peak `|F_ext|` and where, the peak `|tau_ext|` and
on which joint, and the last contact estimate.

### Where the contact was: `flight::contact`

`O_F_ext_hat_K` is the external wrench the robot transports to the stiffness frame, which
says nothing about *where* on the arm it was applied. The seven `tau_ext_hat_filtered` do:
a force `F` at a point `c` on link `i` loads joints `1..=i` with `tau_k = z_k . ((c - o_k) x
F)` (`o_k`, `z_k` the origin and axis of joint `k` from `Model::pose_q`) and leaves the
joints beyond alone. `contact::estimate` samples candidate points every centimetre along the
arm -- for each link the skeleton segment to the next joint origin (link 7 on to the flange
and the end effector), and for links 1, 3 and 5, whose castings run along their own joint
axis, that axis over the casting's extent -- solves the damped 7x3 least squares for `F` at
each, and reports the smallest residual: link, distance from the joint, point, force,
residual, the best residual of every other link (`next_best`) and the `span` of distances
along the winning line that fit within `tolerance` (0.3 Nm) of the best. The flight
recorder subtracts a running baseline of the torques taken while nothing touches the arm
(`flight::Tare`: an FER reads a phantom 4 N at the end effector at rest, from the model's
own error) before estimating.

What the torques can and cannot say: three force components and a position along a line
are four unknowns, so a contact on links 1 to 3 is found but not located along the link (the
span covers it); telling two lines apart needs a fifth loaded joint, so a push on the
forearm reads as link 4 (the chord from the elbow) and link 5 (the forearm axis) fitting
alike a few centimetres apart; a force is a line vector, so a push whose line of action
passes through a joint axis ties the links on either side of it; and a force along a link's
own direction cannot be located along that link. `tests/contact.rs` checks all of this on
torques synthesised from the crate's own frames with 0.3 Nm of noise: links 4, 6 and the
hand come back within 2 cm and 10 %, link 3 with a span covering the link, a force through
joint 4's axis with links 3 and 4 tied. On the real FER push described under Provenance,
the estimate put the contact on the forearm, 14 cm from the elbow, with a force in the
direction of the robot's own `O_F_ext_hat_K` and 2.3 times its magnitude -- the robot's
wrench is the end-effector equivalent of a force that acted 30 cm closer to the base.

### Meshes

`--meshes DIR` (or `FlightOptions::meshes`, `RecorderOptions::flight.meshes`, and the
`meshes` argument of `CommanderLog::record`) draws the arm with Franka's link meshes on top
of the skeleton: `DIR/link0 .. link7`, `hand` and `finger`, as `.glb` (or `.gltf`, `.obj`,
`.stl`, `.dae` -- Rerun 0.37 reads Collada too). `tools/franka-meshes/convert.py` makes
the `.glb`s from franka_description (Apache-2.0, Franka Robotics' work; see that
directory's README -- the converted files are not committed). Each mesh is logged once as an
`Asset3D` at `world/links/<name>/mesh`, and every record puts a `Transform3D` on
`world/links/<name>`.

The frames: franka_description's `link_k` is the child frame of `joint_k` -- the frame after
the joint's rotation -- with the joint origins of `robots/<robot>/kinematics.yaml`, which are
exactly the ones in `crates/franka-rs/tests/data/{fer,fr3}.urdf` (`tests/meshes.rs` checks
all eight against the description's values); `NativeBackend` composes `origin * Rot(axis,
q_k)` per joint like Pinocchio, so `Model::pose_q(Frame::JointK)` *is* the `link_k` frame
and `link0` is the base. Every arm visual has an identity `<origin>` and no `<scale>`, so the
meshes hang directly on those frames -- `link7`'s ends at `z = 0.1068` in its frame, the
flange being at 0.107, and `link1` reaches down to where `link0` ends, which the same test
checks on the converted files when `FRANKA_MESHES=DIR` is set. The hand is mounted as the
description mounts it: on the flange, yawed by -45 degrees (`rpy_ee`), the fingers 0.0584 m
along its `z` (closed; a control log carries no gripper width) and the right one turned by
180 degrees, so that with the Franka Hand's `F_T_EE` the end effector sits 0.1034 m along
the hand's `z`. A tool that is not the Franka Hand still gets the hand mesh if `hand.glb`
is in the directory; leave it out in that case.

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
listening; `rerun` alone opens one on 9876), `Recorder::to_viewer_and_file` (both at once)
and `Recorder::spawn()` (starts one from `PATH`), or `Recorder::with_stream` for any
`RecordingStream`. `push` is meant for the
realtime thread: it copies the record into a bounded `sync_channel` with `try_send`, which
does not block and does not allocate (the ring is allocated once, with the recorder), and
counts the record as dropped when the ring is full. A background thread drains the channel
every 100 ms and does the Rerun work with the same `FlightLogger` the `log` replay uses,
the 3D scene decimated to every 10th record. `tests/flight.rs` runs a 1 kHz producer for
two seconds under a counting allocator: no drops, no allocations in `push`. The blueprint
and the static setup (styles, base, end effector axes, meshes) go out the same way for
every sink, so a live viewer shows the arm as soon as the first batch arrives.
`recorder.stream()` hands out a clone of the `RecordingStream` for a *non-realtime* thread
to log its own entities into the same recording (`set_duration_secs(TIMELINE, t)`, then
`log`).

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

## The example: `commander_live`

The non-realtime commander of `crates/franka-rs/examples/nonrealtime_commander.rs` (see the
book, "Bridging a non-realtime commander"), streamed live into a viewer: the raw staircase
target arriving, the robot refusing it (`--raw`) or the filtered and rate-limited command
following it (`--bridged`, the default), the arm moving, the derivatives of the sent
position against the limits, the events as they happen.

```sh
rerun --port 9876                         # the viewer, first
FRANKA_REALTIME=enforce cargo run --release -p franka-rerun --example commander_live -- \
    <robot-hostname> --live 127.0.0.1:9876 --meshes /path/to/fr3-meshes [--bridged | --raw] \
    [--stdin] [--budget V,A,J] [--controller joint|cartesian] [--out run.rrd] [--yes]
```

`--live ADDR` streams to a viewer already listening there, `--out FILE` writes an `.rrd`,
either or both; `--controller` picks the robot's internal controller. The control side is
franka-rs's target control (`Robot::start_cartesian_target_control`, the book's "Target
control: low-rate commanders"):
the same scripted sequence, budget, collision thresholds and `automatic_error_recovery()`,
with the anchoring, the deviation guard and the settle logic now inside the crate's loop
thread; the CSV `--log` is gone, the recording carries all of it. The loop's observer does
one thing for the recording: `recorder.push(state, Some(RobotCommandLog { O_T_EE_c:
sent.pose, .. }))`. Everything else is logged off the realtime thread: the recorder's
background thread draws the flight recording above (`ee/position/{x,y,z}`,
`ee/derivatives/*`, the arm), and the commander, on the main thread, logs its own side the
moment it calls `set_position`, stamped with the robot time the observer keeps in an
`AtomicU64`:
`commander/target/{x,y,z}` (the staircase, absolute), `commander/target_speed` (the step over
one 1 ms cycle -- 50 m/s -- and over the time since the previous target), `world/target`, and
an `events` line per step, stall and burst. Those series are styled with the viewer's
per-pixel aggregation off (`flight::log_target_styles`), or the corner points of the
staircase and the spikes, a millisecond apart, would be merged and joined with diagonals.
The layout (`flight::send_commander_blueprint`): the arm over the events on the left; `x`,
`y`, `z` (raw target, sent, measured) and the raw target speed down one column, speed,
acceleration and jerk against the limits and `F_ext` down the other.

On franka-sim (`--enforce-motion-limits`, FR3 image, `FRANKA_REALTIME=ignore`, measured
before the port onto the target control API): bridged ran the sequence to the end, 19 107
records pushed and none dropped, peak sent speed 0.25 m/s against the 0.3 m/s budget; raw
was refused at the first step with
`cartesian_motion_generator_velocity_discontinuity` (the sent position's first difference
is 50 m/s) and recovered. The loop's rate-limiter backstop references the twist it sent,
not the robot's float32 echo of the previous command, so it does not fire on the echo's
rounding noise (see the `csv` replay for what that noise looks like in finite differences);
the sent derivatives stay far under the robot's limits, the grey lines. A viewer that is
not being presented (occluded, or on a busy display) may render nothing while the data
streams in and catch up afterwards; the `--out` file has everything regardless. Streamed
live from a real FER on 2026-09-08.

## Library

`franka_rerun::series` logs time series from slices (`log_positions`, `log_derivatives`,
`log_scalar`, `differences`); `franka_rerun::scene` the 3D arm from a `franka::Model` and
joint angles (`skeleton`, `tool_offset`, `log_arm`, `log_skeleton`, `log_point`,
`log_static`); `franka_rerun::meshes::Meshes` finds and logs the link meshes;
`franka_rerun::commander::CommanderLog` reads the CSV and records it
(`CommanderLog::record`, `commander::send_blueprint`); `franka_rerun::flight` has
`log_records`, `replay_exception`, `save_records`, `load_records`, `send_blueprint`,
`send_commander_blueprint`, `log_target_styles`, the streaming `FlightLogger`,
`FlightOptions`, `Summary` (with the peak derivatives of a sent position), the Cartesian
entity paths (`flight::cartesian`), the contact estimator (`flight::contact::estimate`,
`ContactOptions`, `ContactEstimate`, `Tare`);
`franka_rerun::recorder` has `Recorder`, `RecorderOptions` and `Stats`. `RobotKind::limits()` selects the FR3 or FER
limits, and `RobotKind::from(robot.fci_version())` picks the kind from a connection.

## Provenance

The commander recordings this was developed against came from
[franka-sim](https://github.com/BarisYazici/libfranka-sim) with `--enforce-motion-limits`,
and the flight recorder against synthetic logs. Both have since been run on a real FER:
`csv` converted the commander's hardware logs, where the model's end effector matched the
measured `O_T_EE` to under 0.01 mm on every row, and the live `Recorder` ran through
`reflex_replay` without dropping a record. The `reflex.json` that run left behind -- a hand
pushing the moving arm sideways until a Cartesian contact on `Fy`, a joint 3 contact and a
Cartesian collision at 10.6 N tripped the reflex -- is what the contact estimator and the
meshes were developed against.
