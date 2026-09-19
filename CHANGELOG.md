# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.4.0] - 2026-09-19

### Added

- **`franka-vr-teleop`: drive a node-served arm from a Meta Quest controller**
  (`tools/vr-teleop`, on PyPI with the other packages). `franka-vr-bridge` reads the headset
  over adb and publishes each controller on its own ZMQ channel; `franka-vr-teleop` follows
  one channel with a clutch on the grip, clamps every target against the arm's measured pose
  and reads the node's limits from the node before it acquires anything. The node's bytes are
  `franka-node-client`'s, which it pins to its own version. The headset APK is fetched and
  hash-checked by `franka-vr-fetch-apk` into the user's data directory, never shipped.

- A live-tuning guide and node parameter protocol reference, including accepted-versus-applied
  values, feedforward configuration, confirmation gates and session lifetime. A portable
  two-arm node example retains the shipped defaults and shows optional CPU pinning and Hands.

- **The tuning panel ships in `franka-node-client`** as the `franka-tuning-panel` command
  (`pip install franka-node-client`, then `franka-tuning-panel --connect tcp/<host>:7447`); it
  decodes state with the client's own wire module instead of a copy.

- **Live tuning: a running Cartesian session takes a change to the law and the plan, and
  `franka-node` serves it over Zenoh.** `LiveTuning` is the set of parameters an operator may
  move while the arm is engaged -- the joint springs and dampers, one Cartesian stiffness the
  twelve-word preset is derived from, the IK's damping and nullspace gain, the velocity
  feedforward's weight and cutoff, and the two budgets -- and `LiveTuning::BOUNDS` is how far
  each of its twenty-five scalars may be moved. `CartesianTargetControl::tune` applies a partial
  `TuningUpdate` all or nothing, clamps it to that table and returns the bounds that bit;
  `tuning` reads the targets back. A change is never a step in the torque or in the command: the
  fields that multiply a generally-nonzero state cross over `SLEW_TAU` (0.3 s), the budget's
  velocity and acceleration are raised whole and lowered as a ramp at the next order's limit
  (`TuningPolicy::StepUpGateDown`, whose `rate_word` names which word that limit is), and the
  rest step. `TuningPolicy::remaining` and `descent` say what is left of a crossing from the
  clock alone, so nothing is published back out of the realtime thread; the cycle itself holds
  no `exp`, no allocation and no match on a policy. A session whose gains one Cartesian
  stiffness cannot rebuild, or whose tracking is the robot's own, has no live tuning rather than
  a partial one. `Otg::set_limits` and `MultiOtg::set_limits` are what carry a budget to a
  running generator, with the caller's obligation -- never lower a limit below the state -- in
  the OTG's rules as a fourth.
- **`franka/<arm>/params/{schema,get,set}` and `params/current` on the node.** The schema is
  *serialised from* `LiveTuning::BOUNDS`: every bound, default, unit, policy, danger flag,
  slider range and log scale a panel renders is the row the arm is actually clamped to, so a
  client cannot draw a slider the arm would refuse, and the damping floor no per-field row can
  express is published beside them as a `relations` rule. `derived` adds what the library cannot
  know and a client must not guess -- the leash, the gate's lead and step limits, the rates, the
  connected arm's joint velocity limits and the Cartesian preset the one stiffness slider
  scales. A `set` is partial and all or nothing, checked for shape by the node and for domain by
  the library, refused with `stale` against a `base_version` that has moved on and with
  `needs_confirm` where a value crosses a threshold that widens what the arm may do; the reply
  echoes the clamped values so an operator learns a bound exists. The values are the session's
  own: with no session there is nothing to tune and `get` answers with what the next one will
  start at, and a session ending puts them back to the TOML's, deliberately, because an
  experiment nobody saved is not one a new session should inherit.
  `cargo run -p franka-node --example params_schema` dumps that schema without a robot, which
  is what the tuning panel's mock owner serves instead of a copy of the numbers.

- **A continuous weight and a bandwidth bound on the impedance law's velocity feedforward**
  (`ImpedanceOptions::velocity_feedforward_gain`, `velocity_feedforward_cutoff`,
  `MIN_FEEDFORWARD_CUTOFF`): the damping term is now `Kd (g dq_goal - dq)`, and `dq_goal` may
  pass a first-order low-pass before it is fed forward. `velocity_feedforward` off is exactly
  `g = 0`, and the default cutoff leaves the path bit-identical to before, so neither changes
  an existing configuration. The feedforward is what carries the joint reference's own ripple into
  the torque; bounding its bandwidth keeps the lead without the ripple, where the boolean could
  only drop both. With the feedforward off the damping acts on the absolute velocity, so holding
  speed `v` costs a standing error of `(Kqd / Kq) v` -- which a leash on the command then turns
  into a speed limit, the reason the weight is continuous rather than a switch.
- **`franka-node` config keys for the impedance law and the IK**: `ik_damping`,
  `ik_nullspace_gain`, `velocity_feedforward`, `velocity_feedforward_gain`,
  `velocity_feedforward_cutoff`, `cutoff_frequency`, `joint_stiffness` and `joint_damping`, all
  defaulting to the library's values and applied to both session kinds. A Cartesian session's
  joint gains are soft by design -- the Cartesian spring does the work -- which is wrong for an
  arm driven mostly by its joint law, and there was no way to say so without a rebuild.

- **`franka-description`** (`crates/franka-description`, no dependencies, `no_std`): the FR3
  and FER link meshes, the Franka Hand and its finger from franka_description (Apache-2.0,
  commit `7aeeddc`) as built-in glTF binaries, decimated to about 15 % of their triangles
  (30 % for the hand, the finger whole) by `tools/franka-meshes/convert.py --decimate`
  (meshoptimizer's quadric collapse; p99 surface deviation 0.21 to 1.00 mm across the
  decimated meshes, the worst the FER's link3). FR3 734 kB, FER 464 kB. `Robot`, `MeshSet` (`links`,
  `hand`, `finger`), `FR3`, `FER`, `MeshSet::for_robot`, `SOURCE_COMMIT`, `TRIANGLE_RATIO`;
  `NOTICE` carries franka_description's notice and `LICENSE-franka_description` its LICENSE;
  the root `NOTICE` carries the notice too and points at that LICENSE.
- **Built-in meshes in `franka-rerun`**, feature `builtin-meshes` (default): replays and
  recordings draw the arm with the robot's `franka-description` meshes out of the box, about
  0.7 MB of static mesh per arm in an `.rrd`. `franka-rerun csv|log --meshes DIR` still draws
  other files and `--no-meshes` none; `FlightOptions::meshes` is a `MeshChoice` (`Builtin`,
  `Dir`, `Off`), and `Meshes::builtin`. franka-node's `record_meshes` is an optional
  override, `"none"` turns the meshes off.
- **`franka-node-client`, the Python client of franka-node** (`crates/franka-node/python`,
  import `franka_node`): pure Python over `eclipse-zenoh` and numpy, one wheel released with the
  crates under the workspace version. `Node`, `Arm` (lease on `with`, `home`, `stop`, `recover`,
  `on_state`), `cartesian_targets`/`joint_targets` sessions with the `franka` verbs (`move_to`,
  `move_by`, `follow`, `target`, `state`, `wait`, `stop`) whose streamer thread paces goals
  under the node's `max_step` and `max_lead` and keeps the session alive, `Gripper` (`move`,
  `grasp`, `homing`, `stop`, `state`), and `NodeError`, `Refused`, `NodeTimeout`,
  `SessionEnded`, `ProtocolError`. `crates/franka-node/tests/wire.json` is the wire layout
  machine-readable, checked against the Rust types and the client's; the client runs against
  the node binary and franka-sim in CI.
- **`franka-description`, `franka-rerun`, `franka-node` and `franka-cam` on crates.io**,
  released with `franka-rs` under one workspace version. A `v*` tag also attaches prebuilt
  `franka-node` and `franka-cam` tarballs (aarch64 gnu and static musl, x86_64 gnu; built
  with `record`) to the GitHub release, which `cargo binstall franka-node` installs.
- **Joint velocity cap, fade and barrier of target control's torque backend.** No joint of
  the goal moves faster than `ImpedanceOptions::joint_velocity_fraction` (0.7) of the arm's
  velocity limit (`target_control::max_joint_velocity`, by the negotiated FCI version): a
  faster step is scaled as a whole, keeping its direction, and the generator is re-anchored
  on its own end-of-cycle velocity scaled the same way (keeping only the acceleration that
  brakes) so it does not wind up. A joint measured faster than `velocity_barrier_fraction`
  (0.85) of its limit meets `VELOCITY_BARRIER_GAIN` (20 Nm per rad/s) of damping on the
  excess, at most its torque limit, before the clamp; between the cap and the barrier's
  onset (`FADE_BAND`, 0.15 of the limit) the law's own torque along a joint's motion fades
  out, so a joint catching up on its lag cannot cancel the barrier. `CartesianSent` and
  `JointSent` carry `dq_goal`, `cap_scale` and `tau_envelope` (the velocity envelope's share
  of `tau`). The goal is capped before a fast turn of the hand near a wrist singularity
  reaches `joint_velocity_violation`; an ignored test replays a recorded teleoperation
  session with the cap on and off (`target_control/tests/replay.rs`), and a simulator test
  turns a nearly aligned wrist fast (`tests/sim_target_control/velocity_cap.rs`).
  `franka_rerun::TorqueLog` and `Recorder::push_torque_at` record `joints/q_goal`,
  `joints/dq_goal`, `joints/cap_scale`
  and `joints/tau_envelope`; franka-node takes `joint_velocity_fraction` and
  `velocity_barrier_fraction` per arm and records them every cycle.
- **Joint position limit guard of target control's torque backend.** The Cartesian IK solves
  each step inside a per-joint box instead of clamping it: the joint velocity cap, and toward
  each limit a braking envelope, per arm, that brings the goal to rest a margin inside it
  (`ImpedanceOptions::joint_position_margin`, 0.05 rad, in [0.035, 0.5]). A primal active set,
  warm-started, at most 12 passes, fixed-size and allocation-free, gives the exact
  box-constrained damped least-squares step, so the free joints take over what a pinned one
  cannot. The posture is clamped 0.3 rad beyond the margin and its pull fades out as the nearest
  joint closes on the margin. The orientation rows are weighted by the new
  `IkOptions::rotation_weight` (0.1 m/rad): with several joints pinned the tool's position is
  held, the rotation stops at the wall and the elbow does not swing, where the clamp stopped the
  rotation and walked the elbow. While the goal is stalled, position comes first, µm off (the
  translation solved alone, then the orientation with it held, the translation rows weighed 30
  times against it), eased in and out over 0.5 s. A stall flag on the pins' pressure (on above
  2e-5 weighted m per cycle, off on 20 of the last 40 cycles in which no position pin pushes
  at all)
  restarts the generator with the velocity into the wall removed per block and the rest cut to
  what the goal's step carried, so it
  neither winds up behind a wall nor lags a reversal. What went out is the goal's own step, and
  the generator is re-anchored as far ahead of the goal as it was, never further than its own
  step: the IK's lag behind it stays, so the goal velocity does not saw at the cap, and a goal
  closing a lag does not drag the desired along. A `stop()` held at a wall the target lies
  beyond counts as landed once the goal is within the settle tolerance of the target on every
  other axis and has come no tolerance closer to it for the time a generator needs from rest to
  cover two (0.1 s at the default budget). The
  joint interface scales its step into the same box. At the measured arm the law's torque toward
  a limit fades out over
  `POSITION_FADE_BAND` (0.02 rad) inside the margin, and beyond it a spring of
  `POSITION_BARRIER_STIFFNESS` (12.5 /rad × the torque clamp, at most the clamp, ramped in over
  500 cycles) pushes the joint out; the velocity barrier's onset and fade start drop to the
  braking envelope there. On the FR3 the box, the backstop, the restart, the fade and the
  barrier use the robot specifications' position-dependent joint velocity envelope instead of
  the flat limits, the step's limits taken where it ends and the loop's at the arm and a cycle
  on (the robot checks the specifications' parameters, not those of the deprecated
  `rate_limiting::compute_{lower,upper}_limits_joint_velocity`, which are unchanged); Franka
  publishes no such envelope for the FER, so there the robot's limit stays the published flat
  one and only the guard's braking narrows toward a position limit. That braking profile is
  `sqrt(k x + c²) − c` with `k = 2a`, under `sqrt(2 a x)`: `a` is the FR3's published `ddq_dec`
  on that arm and half the FER's published joint acceleration limit on the FER, which unlike the
  FR3 constants stays inside the FER's own rating on every joint. The velocity barrier's gain is
  lowered in proportion under a torque clamp below the preset. `CartesianSent` and `JointSent`
  carry `pinned` (0 free, ∓1 position, ∓2 velocity bound) and `tau_position` (the position
  envelope's share of `tau`), `CartesianSent` also `stall_pressure`, `stalled` and `ik_passes`;
  `franka_rerun::TorqueLog` records them as `joints/pinned`, `joints/tau_position`, `ik/stall`
  and `ik/passes`, and franka-node takes `joint_position_margin` per arm, its joint gate
  refusing targets inside it. Self-collision is not modelled. The spring is sized for a 0.02
  rad overshoot of the margin; the guard is tested on the model and a simulated plant, not yet
  on a robot.
- **`franka-node`** (`crates/franka-node`): a Zenoh node in front of
  Cartesian target control. One process owns one or more robots and, per arm, runs the
  impedance backend on the library's realtime thread; clients publish 80-byte pose or joint
  targets at any rate on `franka/<arm>/target`, read the arm's state at 100 Hz on
  `franka/<arm>/state` and drive `acquire`/`enable`/`stop`/`release`/`recover`/`home` through
  queryables. A per-arm
  thread is the single writer of the target seqlock; targets are gated (lease holder, order,
  finite, unit quaternion, step, lead, rate, and a workspace box when a config names one --
  there is none by default) before they reach it; a client's liveliness
  token and a target-staleness watchdog stop the arm when the commander goes away. Config in
  TOML, a `client` example, unit tests and a simulator test (`tests/sim_node.rs`).
  - Joint targets and `home`: `enable` takes `"mode": "joints"` for a session on
    `start_joint_target_control` (targets are `q`, gated per joint by `max_step_joint` and the
    arm's joint limits; the state's `target` carries `q` with the joints flag), and `home`
    drives the arm to the ready pose at a `speed` of the limits, answering when it has
    arrived; an early end (stop, lost lease, timeout, shutdown) decelerates in place. Config
    keys `max_step_joint`, `joint_budget_fraction`, `joint_max_deviation`; the library's
    `target_control::joint_position_limits` is public.
  - A status topic, `franka/node/<name>/status`: JSON once a second with the node's name,
    version and uptime and, per arm, the phase, holder, mode and the accepted, refused,
    dropped and decode-failure counters, read from atomics the arm thread stores into with
    every state tick (`ArmStats`, `transport::status_publisher`). A systemd unit and install
    notes in `crates/franka-node/deploy/`, and a book page, *Serve arms over Zenoh*, with a
    Python commander and the Raspberry Pi build.
  - Episode recording behind the cargo feature `record` (pulls `franka-rerun`, Rust 1.96):
    with `record_dir` set, every session (`enable` in either mode, `home`) is one
    `<arm>-<UTC stamp>.rrd` written by a `franka_rerun::Recorder` in the loop's observer,
    with the accepted and refused targets logged from the arm thread under
    `commander/target/*` and `events`; the status gains `recording`, the open file's name.
    `Recorder`'s constructors take `impl Into<Arc<Model>>` so one model serves many
    recordings, and `franka_rerun` re-exports `rerun`.
  - The gripper is recorded with the arm: width, commanded width, grasped, moving and fault go
    into the session's `.rrd` under `gripper/*` at the 20 Hz the gripper state is published, so a
    replay has the hand beside the arm that carried it. The flight recorder's default layout
    gained a tab for them and one for `<arm>/cam/*`, the entities a camera node records into the
    same episode; an explicit blueprint turns the viewer's automatic layout off, so an entity no
    view names would otherwise be in the file and not on screen.
  - An `episode` topic, `franka/<arm>/episode`: one JSON line at a session's start and end
    with the arm, the session's `recording_id`, the `.rrd` file name and the node's clock,
    and the id again in the status as `episode`. With the `record` feature the id is the
    Rerun `RecordingId` of the arm's own file (its stem), so a recorder in another process
    writes a file under the same id and a viewer loads them as one recording.
  - The target watchdog is fed by arrival, not acceptance: `stop_after_ms` now measures from
    the last target that reached the guard's content checks from the lease holder in order
    (`Reason::from_the_commander`), and `hold_after_ms` still measures from the last *accepted*
    one, so `holding` keeps its meaning. A commander whose targets are all being refused --
    parked at the workspace wall, rate limited, outrunning the arm, or pushing against a hand
    on it -- was treated exactly like one that had died: the session stopped, which blocks the
    arm thread for the library's settle and lands in Idle needing a fresh `enable`. Targets
    that are not the commander's (wrong kind, not the holder, out of order) are still no sign
    of life, so a silent commander is stopped whatever else is on the topic.
  - A lead limit and an anchor flag, for a commander a human drives. `max_step` bounds a
    distance per message and never looks at the arm, so a commander faster than the generator
    walks the target ahead without limit and the arm coasts the whole accumulated lead when the
    commander stops. `max_lead` (0.05 m) and `max_lead_rotation` (0.26 rad) bound how far an
    accepted Cartesian target may be from the measured pose, `0` disabling either, with
    `Reason::Lead` and `Reason::LeadRotation` counted and logged like every other refusal; a
    refusal still spends no token and advances nothing. `TargetMsg.flags` bit 0
    (`TARGET_ANCHOR`) has that target's step measured from the arm instead of from the previous
    accepted target, which is how a stream that jumped — a re-latched clutch, a moved tracking
    frame, an arm a hand has pushed off its target — gets back in; it lifts no limit, so an
    anchored target is bounded by the smaller of `max_step` and `max_lead` of where the arm is
    and anchoring every message is the most conservative way to command the node, not a way
    around it; with either lead limit at 0 the flag is ignored rather than left unbounded. A
    lead limit must be 0 or clear of `leash`, below which it would refuse the tracking error a
    healthy commander already has. The message's size and layout are unchanged and an unknown
    flag bit is ignored.
  - `node` and `cam` are refused as arm names (`RESERVED_ARM_NAMES`): `franka/node/*` is the
    status topic and `franka/cam/*` belongs to a camera node.
  - `[zenoh] mode = "client"` dials the routers in `connect` and listens on nothing, which is
    how a node behind NAT serves a commander that is not on its network; `peer`, the default,
    is unchanged. A client with no router named is a config error. The TLS transport is compiled
    in, and `[zenoh] zenoh_config` names a Zenoh file to start from, so the certificates of a
    `tls/` endpoint, authentication and access control are configuration rather than code; the
    table's own keys are applied on top of that file.
  - Grippers: a `Gripper` trait (`command`, `grasp`, `home`, `stop`, `state`; metres) and
    `FrankaHand`, the Franka Hand over `franka::Gripper` on a worker and a reader thread. Per
    arm, `franka/<arm>/gripper/target` takes a 40-byte `GripperMsg` (width or grasp) from
    the lease holder in any phase, `franka/<arm>/gripper/state` carries a 40-byte
    `GripperStateMsg` at 20 Hz, and `cmd/gripper_home` (the holder's, answered when homed)
    and `cmd/gripper_stop` (anyone's) join the verbs; the status gains `gripper`. Config
    keys `gripper = "hand"` and `gripper_speed`. The node is now `franka_node::run(config,
    factory)` with a `GripperFactory`, so a binary of its own can add drivers; the client
    example gains `gripper <width>`, `gripper grasp <width> <force>` and `gripper home`.
- **`franka-cam`** (`crates/franka-cam`): the wire and the configuration
  of a Zenoh camera node that publishes V4L2 frames with their capture timestamps next to the
  arm node. `CameraMsg` is a 36-byte header (`"<BBHHHIQQQ"`) carrying the format, the frame
  size, the driver's sequence number and the capture, dequeue and wall clocks, followed by the
  frame; a frame of a format whose length the header fixes is refused when it does not match.
  `CamConfig` reads `[zenoh]` and one `[[camera]]` table per camera. The capture path is there
  too: raw V4L2 ioctls with the struct layouts and ioctl numbers asserted against
  `videodev2.h`, an mmap stream that refuses a queue stamped with anything but
  `CLOCK_MONOTONIC`, one thread per camera at normal priority pinned to the config's `cpu`, one
  allocation per frame handed to Zenoh, sequence gaps counted as dropped by the driver, and a
  reopen with backoff when the device disappears. Per camera, `franka/cam/<name>/frame` at the
  camera's rate and, with `preview_fps`, every nth frame on `franka/cam/<name>/preview` for a
  consumer that must not pull the full rate; `franka/cam/<name>/state` and
  `franka/node/<name>/status` once a second. `[zenoh]` takes the arm node's keys, including
  `mode = "client"` and `zenoh_config`, so a camera behind NAT can feed a consumer that is not on
  its network over TLS. A camera's `cpu` may not name an isolated core, because those belong to
  the realtime loops.

  Frames go into the arm's episode behind the `record` feature: a camera with a `record_with`
  follows that arm's `franka/<arm>/episode`, and on a session's start opens
  `<record_dir>/<recording_id>-<arm>-cam.rrd` under the same Rerun `RecordingId` and application id as
  the arm's own file, so a viewer loads the two as one recording. Frames are logged as encoded
  images under `<arm>/cam/<name>`, where that arm's camera tab is rooted, at the robot time the
  arm's `state` topic maps them to, through a bounded channel and a writer thread, so the capture
  thread neither blocks on Rerun nor copies a frame between episodes. All the cameras of one arm
  share its file. A camera on another host does not record, because two monotonic clocks have no
  common origin; a format other than MJPEG is
  refused; and what an open episode could not record is counted in the state topic's
  `record_dropped`.
- **One recording for two arms.** A collector names an episode and every arm of it writes into
  one Rerun recording, synchronised on the host's clock.
  - `enable` and `home` take `"episode": "<name>"`, `[A-Za-z0-9_-]{1,128}`: the session's
    `recording_id` is that name, the arm's file is `<name>-<arm>.rrd`, and the episode topic
    publishes it, so two arms enabled with one name write two files a viewer loads as one
    recording -- no coordination and no timing heuristic in the node. Without a name nothing
    changes. A name of another shape is refused with the reason and starts nothing. The client
    example takes `--episode NAME`.
  - **`franka_rerun::HOST_TIMELINE`**, `host_time`: every row of a live recording carries the
    host's `CLOCK_MONOTONIC` beside `robot_time`, stamped in `Recorder::push` where the record is
    taken (still no allocation there, which `tests/flight.rs` proves). Two arms are two
    controllers, so their `robot_time`s are unrelated; `host_time` is the axis they agree on, and
    the layout of a named episode opens on it. An offline replay carries `robot_time` alone.
  - **`franka_rerun::Prefix`**, `FlightOptions::prefix`: every entity of a recording goes under a
    name (`L/joints/q`), the node's being the arm's. `scene`, `meshes`, the styles and the
    blueprints take it; `franka-rerun log --prefix NAME` does the same for a replay.
  - `flight::send_blueprint` takes a `Layout`, the prefixes of every robot in the recording and
    the timeline the time panel opens on, and lays out those views per robot; every arm of a node
    sends the same one, so a shared recording is laid out whichever arm's blueprint the viewer
    reads first. `RecorderOptions::layout` carries it.
  - `franka-cam`'s file is `<recording_id>-<arm>-cam.rrd`, so two cameras following different arms
    of one episode do not collide, and a frame is logged on `host_time` (its own exposure stamp)
    as well as on the arm's `robot_time`. A frame the arm's clock pairs cannot map is now written
    on `host_time` alone rather than dropped.
  - `ee/orientation`: the measured and the commanded end effector rotation as quaternions `xyzw`
    at every cycle, each continued in sign from the last, for an exporter that needs the
    orientation at the rate the rest of the state is at (the 3D scene's pose stays decimated).
  - `Recorder::push_at` takes a host stamp the caller has already read, so the node's observer
    reads the clock once per cycle and stamps the recorder's row and its own `commander/target/*`
    and `gripper/*` rows with the same instant.
- **`franka::realtime::monotonic_ns`**, the host's `CLOCK_MONOTONIC` in nanoseconds. The node's
  `monotonic_ns`, and with it every `t_node_ns` on the wire, now reads that clock instead of an
  `Instant` measured from the process's start, so a second process on the same host puts its
  own samples on the same timeline. `franka_rerun::APPLICATION_ID` names the Rerun application
  id every stream of that crate opens, and `Recorder::to_file_with_id` opens a file under a
  given `RecordingId`, which is what lets two processes write one recording.
- **CPU pinning of the target-control loop.** `TargetControlOptions` /
  `JointTargetControlOptions` gain `cpu: Option<usize>` (`with_cpu`): the loop thread pins
  itself to that core with `sched_setaffinity` right after raising its priority, through the
  new `franka::realtime::pin_current_thread_to_cpu`; a failure is `FrankaError::Realtime`
  under `RealtimeConfig::Enforce` and ignored under `Ignore`. `franka-node` passes the arm's
  `cpu` key through.
- **Seven diagnostic channels of the torque backend.** `CartesianSent` gains `ik_step` and
  `ik_step_clipped` (the cycle's largest raw IK step norm and what the per-joint box clipped
  off it, rad), `ik_blend` (the priority blend, 0 the weighted solve, 1 position first), `held`
  (the goal held at a wall) and `wall_age` (cycles since each block's wall was last pushed on,
  -1 for no wall). `franka_rerun::TorqueLog` carries them beside `ee_velocity`, `ik_error` and
  `leash`, and the recorder writes `ik/{step, blend, held, error}` and `ee/{velocity, leash}`,
  plus `ee/orientation/commanded` (`flight::COMMANDED_ORIENTATION`), the rotation of the pose
  the command carries, which is what the torque backend fills where `state.O_T_EE_c` is zero.
  The default layout names every one of them. The addition is additive: an older recording opens
  unchanged, and a record pushed without a `TorqueLog` writes none of the new series.

### Changed

- **Live tuning caps the wrist's `joint_damping` at 40 Nm s/rad** (joints 5 to 7; 60 on 1 to
  4): with the velocity barrier's 20 added, `K × 1 ms / I` on the wrist's 0.074 kg m² is 0.81
  there and would be 1.08 at 60, past the stability rule in the impedance reference.

- **Raising `velocity_feedforward_gain` above 0 needs the operator's confirmation**
  (`confirm_above` 0), as crossing a budget's threshold does; the tuning panel puts the
  feedforward group behind its lock.

- Lowering a budget's acceleration or jerk while the arm moves never steps the command but
  lengthens the stop (toward `v² / 2a`, plus the jerk's ramp): at the default 0.3 m/s (0.17 m/s
  per axis), an acceleration dragged from 0.5 to 0.1 m/s² with the goal 3 cm ahead overshoots
  it by about 20 cm, and faster motion or a lower acceleration overshoots further.
  `CartesianTargetControl::tune`, the live-tuning guide and the tuning panel's envelope
  advisory now say to lower the velocity first.

- **Velocity feedforward now defaults to off**: `ImpedanceOptions::velocity_feedforward` is
  `false` on both interfaces, and with it the node's `velocity_feedforward` key and the live
  weight it seeds (0). Real arms vibrated with it on: `dq_goal` is a finite difference (of the
  IK solution on the Cartesian interface) and `Kd` carries its noise into the torque. Off, a
  goal moving at `v` is tracked `(Kd / Kp) v` behind, which the leash turns into a speed cap.
  `velocity_feedforward_gain` stays 1, so switching it on means all of it;
  `velocity_feedforward_cutoff` is the knob to try against the ripple. A node config that sets
  only `velocity_feedforward_gain` now runs without feedforward and needs
  `velocity_feedforward = true` to keep it. Python's `velocity_feedforward` keyword now defaults
  to `None` (the Rust default), and `backend='robot'` refuses any explicit value, `True`
  included. `nonrealtime_commander`'s `--no-feedforward` became `--feedforward`.

- The `model-library` feature is now off by default. Both robots use the native Rust
  model through `Robot::load_model()`. Applications that explicitly download the Panda's
  shared library must enable `model-library`; the optional comparison path remains available.

- **The guard's workspace box is off unless a config asks for one.** `GuardOptions::workspace`
  is now an `Option<Workspace>` defaulting to `None`, and `ArmConfig::workspace` is optional in
  the TOML. The retired default -- x 0.2..0.8, y -0.5..0.5, z 0..0.8 in the base frame --
  assumed a forward-facing bench and refused good poses anywhere else, and the box had no off
  switch: `max_lead` and `max_lead_rotation` are disabled by a zero, but a zero is a legitimate
  coordinate, so "no box" could only be written as an absurdly large one. A config that names a
  box still gets exactly that box, and an inverted or non-finite one is still refused. With no
  box a target is still bounded by the step and lead limits, the rate bucket, the leash and the
  session's own deviation guard. `Workspace` moved from `config` to `guard`, beside the check
  that enforces it.

- The minimum Rust version is 1.89, what `nalgebra` 0.35 already required; 1.85 was stale.
- The torque backend caps the goal at 0.7 of each joint's velocity limit by default.
  `IkOptions::max_step` (10 rad/s) is removed; the cap bounds the IK's step instead.
- New public fields break struct literals: `ImpedanceOptions` gains
  `joint_velocity_fraction` and `velocity_barrier_fraction`; `CartesianSent` and `JointSent`
  gain `dq_goal`, `cap_scale` and `tau_envelope`.
- A `joint_velocity_fraction` above `velocity_barrier_fraction` (0.85) fails `validate`
  unless the barrier fraction is raised too.
- `IkOptions::limit_margin` is removed; `ImpedanceOptions::joint_position_margin` (0.05 rad)
  replaces the 0.02 rad clamp. `ImpedanceOptions` gains that field, `IkOptions` gains
  `rotation_weight`, `CartesianSent` gains `pinned`, `tau_position`, `ik_passes`,
  `stall_pressure` and `stalled`, `JointSent` gains `pinned` and `tau_position`, and
  `franka_rerun::TorqueLog` the recorded ones: struct literals break.
- The Cartesian IK weighs orientation by `rotation_weight` 0.1 m/rad instead of 1, also in free
  space; set it to 1 for the previous trade. `ik_error` and `IkOptions::tolerance` are the
  weighted norm (1e-6 is 1 µm or 10 µrad).
- `cap_scale` is the fraction of the generator's step the goal carried on a cycle a joint limit,
  position or velocity, cut it, and exactly 1 on every other cycle.
- With the impedance backend a joint target or posture is refused inside `joint_position_margin`
  of a joint limit (`JOINT_LIMIT_INSET`, 0.02 rad, with the robot's controller).
- `CartesianSent` gains `ik_step`, `ik_step_clipped`, `ik_blend`, `held` and `wall_age`, and
  `franka_rerun::TorqueLog` gains `ee_velocity`, `ik_step`, `ik_blend`, `held`, `wall_age`,
  `ik_error` and `leash`: adding a field is semver-visible because neither is
  `#[non_exhaustive]`, and both stay exhaustive on purpose so that a downstream test can still
  build one field by field. `TorqueLog::default()` leaves `wall_age` at 0, which reads as a
  wall; a producer with no walls sets `[-1; 2]` itself.

### Fixed

- franka-node: `enable`, `release` and `home` refuse `client_id` 0 (`"client 0 is not the
  holder"`), as `acquire` and `gripper_home` already did; a free arm's holder is 0, so an
  `enable` from client 0 used to start a session nobody held.
- franka-node: every stop of a joints session (the `stop` verb, the watchdog, a lost lease,
  shutdown) first re-targets the loop to the measured configuration, clamped inside the joint
  limits less the guard's margin, as an early end of `home` already did, so the arm decelerates
  where it is instead of travelling on to the last accepted target.
- Tuning panel: an `<arm>` path segment that is not `[A-Za-z0-9_-]+` is a 404 before it reaches
  a Zenoh key expression, so `*` or `**` can no longer fan one request out to every arm; a
  `Content-Length` that is missing, not a byte count or over 64 KiB is refused (422, or 413 for
  the size) and the connection closed, and a stalled request times out after 30 s.
- VR teleop: a fresh arm state with a non-finite pose or joint velocity is treated as no state:
  the clutch releases, nothing is driven, and the client says so once. Before, a NaN passed both
  leash checks and a latch on it made every target NaN.
- The Python node client gives queued state callbacks up to 250 ms to catch up after a
  long local streamer stall, preventing a false disconnect on the next tick. Repeated
  stalls cannot extend that deadline; normal node-silence detection and explicit
  session-end handling remain unchanged.
- The `franka-rs` wheel carries `LICENSE` and `NOTICE`, libfranka's notice included, under
  its `dist-info/licenses/`; only the sdist had them.

## [0.3.0] - 2026-09-10

### Added

- **An impedance backend for target control**, the default. `TargetControlOptions` /
  `JointTargetControlOptions` gain `backend: Backend` (`with_backend`):
  `Backend::Impedance(ImpedanceOptions)` runs the loop through `control_torques` and sends,
  every cycle, the torques of the hybrid joint impedance law of DROID's controller (polymetis
  `HybridJointImpedanceControl`), `tau = (Jᵀ Kx J + Kq)(q_goal − q) + (Jᵀ Kxd J +
  Kqd)(dq_goal − dq) + coriolis`, clamped to `torque_limits` and low-pass filtered at
  `cutoff_frequency` (100 Hz); the arm is compliant around the target. The damping acts on
  the velocity error (`velocity_feedforward`, default on; off is DROID's form). Without an
  echo of a torque command the generator is anchored every cycle on the measured state
  pulled toward the previous desired by at most the `Leash` (0.025 m, 0.15 rad; 0.1 rad per
  joint), also during the stop's hold, so an arm that is held back never meets more than the
  felt stiffness times the leash (roughly 25 to 30 N at the default gains at the ready pose,
  18.75 N with `project_joint_gains`; on the joint interface the torque clamp bounds the
  torque) and the generator resumes from the arm on release. Target control sets no
  collision thresholds: with the default gains set at least 40 N / 40 Nm, or lower the
  stiffness. On the Cartesian interface `q_goal` comes from a differential inverse kinematics
  (damped least squares, nullspace drift toward `posture` capped at 0.5 rad/s, a step cap
  `max_step` of 0.01 rad per cycle, clamp to the joint position limits; `IkOptions`) that
  follows the generator one cycle at a time, so an unreachable target lags rather than
  jumps; on the joint interface it is the generator's output. A `posture` or joint target
  outside the joint limits (inset 0.02 rad) is refused with `InvalidArgument`.
  `project_joint_gains` (default off) confines the joint gains to the Jacobian's nullspace so
  the end effector feels `Kx` alone (unprojected, the joint springs make the default 750 N/m
  about 990 to 1180 N/m at the ready pose). The finish waits for the arm to rest
  (`REST_JOINT_VELOCITY`, 0.01 rad/s, or the 5 s timeout). `ImpedanceGains` (`CARTESIAN`:
  750 N/m and 15 Nm/rad with damping 50, 50, 90 Ns/m, about ζ 0.8 at the ready pose, and a
  small joint term; `DROID`: DROID's gains as they were, damping 37; `JOINT`: the
  `fer_joint_impedance` example's), `ImpedanceOptions::cartesian()` / `::joint()` with
  `with_*` builders, `franka::impedance_torques` (the law, public at the crate root) and
  `rate_limiting::JOINT_POSITION_LIMITS` (FR3) / `rate_limiting::fer::JOINT_POSITION_LIMITS`
  (FER), from the URDFs in the repository. `nonrealtime_commander` gains `--no-feedforward`,
  `--project-joint-gains`, `--leash M` and `--thresholds N`, and its CSV `leash_alteration`.
  `CartesianSent` / `JointSent` gain `q_goal`, the
  clamped `tau`, `leash_alteration` (and `leash_angular_alteration` for a pose) and, for a
  pose, the IK residual `ik_error`. The Python `cartesian_targets` and `joint_targets` take
  `backend` (`'impedance'` | `'robot'`), `cartesian_stiffness`, `cartesian_damping` (6
  values, or one float for the translational three), `joint_stiffness`, `joint_damping`,
  `torque_limits`, `posture`, `torque_cutoff`, `velocity_feedforward`, `leash` and
  `project_joint_gains`. See
  [The impedance backend](docs/book/src/reference/impedance.md).

### Changed

- **Target control is compliant by default.** Both interfaces now send the impedance
  backend's torques; the robot's own controller tracking a pose or joint-position stream,
  the only behaviour before, is `Backend::RobotController`
  (`TargetControlOptions::default().with_backend(Backend::RobotController)`, Python
  `backend="robot"`), and `controller_mode` applies to that backend only. The robot's
  joint-side continuity check no longer refuses a fast Cartesian budget in the default
  backend; the deviation guard and the collision thresholds apply to both. A change of
  default behaviour, hence 0.3.0 rather than 0.2.1.
- **The book is reorganised** into Getting started, Things to keep in mind, How-to and
  Reference; the 0.2.0 page names redirect. The README is the front door only, with the
  quick example on `ActiveControl`.

## [0.2.0] - 2026-09-09

### Added

- **Target control** (`robot::target_control`): `Robot::start_cartesian_target_control` and
  `Robot::start_joint_target_control` run the crate's control loop on a named thread of
  their own and return a `CartesianTargetControl` / `JointTargetControl` handle whose
  `set_position([f64; 3])` / `set_joints([f64; 7])` any low-rate commander can call from any
  thread at any rate; the loop bridges the steps with the online trajectory generator under
  the three rules of the `otg` module (per-axis limits from the norm budget, one nominal
  `DELTA_T` per command, re-anchoring on the robot's echo), the rate limiter under the same
  budget as the backstop, a measured-deviation guard and a settle-then-finish `stop()` that
  returns the loop's result. `TargetControlOptions` / `JointTargetControlOptions` carry the
  budget (joint default: 20 % of the negotiated version's limits), the controller mode, the
  guard, the settle criterion, an optional `SCHED_FIFO` priority for the loop thread and an
  observer called every cycle on the realtime thread with what was sent (the flight
  recorder's hook). `TargetSlot<N>` is the seqlock underneath, public. `MultiOtg::with_limits`
  builds a generator with per-axis limits and `realtime::set_current_thread_scheduler_priority`
  raises a thread to a chosen priority. Tested on franka-sim
  (`tests/sim_target_control.rs`).
- **Cartesian target control carries an orientation.** `CartesianTargetControl::set_pose`
  (column-major, as `O_T_EE`; a rotation block within 1e-3 of orthonormal is repaired, one
  further off refused), `set_target(position, quaternion)` and `set_orientation(quaternion)`
  with unit quaternions in `[x, y, z, w]` order, `target_orientation()` and `target_pose()`;
  `set_position` keeps the target orientation. The orientation runs on three more axes of
  the same synchronised generator, on the base-frame rotation vector of the orientation
  error re-anchored on the echo every cycle, under `TargetControlOptions::rotation_limits`
  (0.5 rad/s, 1.0 rad/s², 20 rad/s³ by default) with `with_rotation_limits`, and an angular
  deviation guard `max_angular_deviation` (0.5 rad). `CartesianSent` gains the sent
  orientation, the angular velocity and acceleration and the rotational backstop alteration.
  The Cartesian backstop now references the twist and acceleration it sent rather than the
  echoed ones, whose float32 rounding had its jerk clamp firing at noise level (on the
  rotation that was an orbit of 5 mrad around the target), and `REST_VELOCITY` /
  `REST_ACCELERATION` drop to 1e-4 and 0.05 so the hold's freeze stays under the joint-side
  jerk limits. `OtgLimits::scaled` is new. The
  commander example's `--rotate` adds a slow yaw sweep of ±15° (bridged mode only).
- **`otg` module**: an online trajectory generator (`Otg`, `MultiOtg<N>`, `CartesianOtg`,
  `OtgLimits`) that re-plans a time-optimal, jerk-limited seven-segment profile every cycle
  from the commanded state to rest at the latest target, so a stream of stepped, bursty or
  stalled targets becomes a C2 command that never exceeds its velocity, acceleration and jerk
  limits, does not overshoot a reachable target, lands exactly and stays there; optional axis
  synchronisation, `set_position` to re-anchor on the robot's echo, `per_axis_for_norm` for
  a budget that is a norm. Dependency-free and allocation-free. The module documentation
  gives the rules that keep a rate limiter behind it from orbiting at the velocity cap, and
  two replay tests pin them down.
- **`nonrealtime_commander` example**: a scripted (or stdin) commander sets Cartesian
  targets through `start_cartesian_target_control` (`--bridged`, `--budget V,A,J`), or
  hands them to a bare `control_cartesian_pose` to provoke a reflex (`--raw`); `--log`
  writes one CSV row per cycle, from the loop's observer, with the joint angles, the
  external wrench and the generator's velocity and acceleration. The budget exists because
  the robot also checks the joint-space continuity of a Cartesian pose stream, which the
  rate limiter does not bound; see
  [Target control](docs/book/src/howto/target-control.md).
  `franka-rerun`'s `commander_live` example is on the same API, with the `Recorder` in the
  observer.
- **Python bindings** (`crates/franka-py`, `import franka`; `pip install franka-rs`, or
  `maturin develop` from the source tree): `Robot`, `RobotState` (numpy fields and a
  69-float `flat()` observation), `robot.cartesian_targets()` / `robot.joint_targets()` as
  context managers over the target control loops with `move_to`, `move_by`,
  `follow(chunk, dt)`, `target()`, `state()` and `stop()`, `Gripper`, `Model` over numpy
  (`robot.model()`), `franka.rotated`, and `FrankaError` / `ControlException`. A Cartesian
  target carries an optional unit quaternion, a delta an optional rotation vector. The
  1 kHz loop stays on its Rust thread and never takes the GIL. PyO3 0.29, abi3 for
  Python 3.9+; tested against franka-sim in CI's `python-bindings` job. Examples:
  `crates/franka-py/examples/policy_loop.py`, `rotate.py` and the `quickstart.ipynb`
  notebook. See [Python](docs/book/src/getting-started/python.md).
  `.github/workflows/release.yml` builds the wheels and publishes them and the crate on a
  `v*` tag.
- **`automatic_error_recovery` example**: command-line recovery that prints the robot mode
  before and after. **`move_to_ready` example**: the examples' motion generator to
  libfranka's ready pose at a fraction of full speed.
- **`serde` feature** (off by default): `Serialize` / `Deserialize` for `RobotState`,
  `RobotMode`, `Errors`, `Duration`, `Record`, `RobotCommandLog`, `MoveStatus` and
  `ControlException`. `Errors` serialises as the list of the set flags' names.
- **`franka-rerun`**, a workspace crate (`publish = false`, Rust 1.96) that replays logs in
  Rerun: `csv` turns the commander example's log into target, commanded and measured
  positions, the command's derivatives against the rate limits and a 3D replay of the arm;
  `log` replays a saved `ControlException` control log as a flight recording (contact and
  collision flags, external wrench, commanded versus measured, errors, the arm), through
  `flight::{log_records, replay_exception, save_records, load_records}`; and `Recorder`
  streams the same live from inside a control loop with a non-blocking, non-allocating
  `push`. `examples/reflex_replay.rs` puts the last two together; `examples/commander_live.rs`
  streams the commander into a viewer. Both replays draw Franka's link meshes with
  `--meshes DIR`, `csv` has the screen-capture `--layout demo` and the commander's `--budget`
  lines, and `log` locates a contact on the arm from the external joint torques
  (`flight::contact`). See [the flight recorder page](docs/book/src/howto/flight-recorder.md).

## [0.1.0] - 2026-09-07

Initial release. `franka-rs` is a pure-Rust libfranka client speaking both
generations of the Franka Control Interface: FCI v10 (Franka Research 3,
ported from libfranka 0.21.2) and FCI v5 (Franka Emika Robot, FER, ported from
libfranka 0.9.2).

There is no earlier release to compare against, so "Added" describes the
release itself and "Changed" records the places where this crate deliberately
differs from libfranka.

### Added

- **Franka Emika Robot (FER) support (FCI v5)**, alongside the FR3 (FCI v10); see
  [the FER specifics page](docs/book/src/reference/fer.md). `Robot::new` negotiates the version
  automatically (connects announcing v10, retries as v5 if the robot reports
  `kIncompatibleLibraryVersion`); `RobotOptions::with_version(VersionPolicy::
  Exact(FciVersion::V5))` skips the extra round trip when the generation is
  already known. New public API: `FciVersion`, `VersionPolicy`,
  `RobotOptions`, `VirtualWallCuboid`, `Robot::{with_options, fci_version,
  set_filters, virtual_wall}`, `Model::{from_model_library_bytes,
  from_model_library_path}`, `model::{load_from_robot, SoModelBackend,
  model_library, so_backend}`, `RobotState::from_wire_v5`,
  `rate_limiting::fer` (selected automatically from the negotiated
  version), `wire::robot::{v5, codec}`. New example `fer_joint_impedance`.
- **The FER model no longer needs a download.** `Robot::load_model()` on an
  FER now returns the *native* model — `franka::model::FER_URDF`,
  evaluated by the same backend the FR3 uses — instead of downloading and
  `dlopen`ing the robot's shared object. Needs no network, no
  `model-library` feature and no x86-64 Linux host; also reachable without a
  robot as `Model::native_fer()`. Its link inertial parameters were
  identified from a real FER's own model library by the new
  `tools/fer-model-fit` (a regressor from the crate's own dynamics
  backend, solved with a truncated SVD), agreeing with the library to
  9e-16 on kinematics and 4e-14 on dynamics with no payload, and
  characterising the one known gap, a payload non-linearity in the library
  itself (see [the model page](docs/book/src/reference/model.md)).
  `Robot::load_model_from_robot()` keeps the previous behaviour (download +
  `dlopen` on v5, unchanged `GetRobotModel` URDF on v10). The native model
  is checked in CI via `tests/fer_native_conformance.rs` against a
  committed fixture, with no robot or shared object present.
- **aarch64 builds**, via `cargo-zigbuild` or a cross `gcc`, or as a fully static
  `aarch64-unknown-linux-musl` binary with `--no-default-features` (a static
  binary can't `dlopen`, and the FER's model path is x86-64-only anyway).
  CI cross-builds for `aarch64-unknown-linux-gnu` on every push; verified
  under `qemu-aarch64` against both simulators, identical to native.
- **New example `cartesian_impedance_active_control`** — libfranka's
  `cartesian_impedance_control.cpp` (a spring-damper system whose equilibrium is
  the initial end-effector pose) driven through the `ActiveControl` API's
  `read_once` / `write_once` instead of a control callback, allocation-free and
  running unchanged on an FR3 (FCI v10) and an FER (FCI v5).
- **New example `cartesian_impedance_figure_eight`** — the same `ActiveControl`
  impedance loop with a moving equilibrium: a Lissajous figure eight around the
  start pose, raised-cosine ramps on both the path amplitude and the stiffness at
  either end (and after Ctrl-C), a nullspace joint spring toward the initial
  configuration through a damped-inverse projector, a one-sided virtual floor,
  a ±25 Nm torque clamp and a deviation cut-out. Allocation-free, on both
  generations. The pieces it shares with `cartesian_impedance_active_control`
  moved to `examples/common/cartesian.rs`.
- **New example `dual_communication_test`** — the zero-torque
  communication-test loop against two robots at once from one process, with
  per-robot loop timing, lost-state and success-rate accounting.
- **`bench/fer-capture/`** — offline tooling that reassembles the FCI v5
  state datagrams from a pcap/pcapng capture and reports inter-arrival gaps,
  drift and a stall verdict, to tell whether an FER loop's lost cycles come
  from the robot/network or the host.
- **`bench/so-micro/`** — an offline per-call microbenchmark of the FCI v5
  model path, Rust vs C++, against the same captured shared object, needing
  no robot or simulator.

### Changed

- `CommandName::command()` is now `CommandName::command(self, version:
  FciVersion) -> Option<u32>`, since the TCP command numbering differs
  between the two protocol versions (`None` means the command does not
  exist in that version); `CommandName` gained two v5-only variants,
  `GetCartesianLimit` and `SetFilters`.
- `SoModelBackend::open`, `SoModelBackend::from_bytes`,
  `Model::from_model_library_bytes` and `Model::from_model_library_path` are
  now `unsafe fn`: they `dlopen` the file or bytes given to them, so the
  caller must assert it is a trusted model library. `Robot::load_model()`
  stays a safe `fn`, as in libfranka.
- A `Connect` reply with status `kSuccess` reporting a version other than
  the one the client announced is now `FrankaError::IncompatibleVersion`
  rather than being decoded against the wrong layout — what lets automatic
  version negotiation recognise an FER on the simulator; no effect against
  a real FR3 or FER.

[0.4.0]: https://github.com/BarisYazici/franka-rs/releases/tag/v0.4.0
[0.3.0]: https://github.com/BarisYazici/franka-rs/releases/tag/v0.3.0
[0.2.0]: https://github.com/BarisYazici/franka-rs/releases/tag/v0.2.0
[0.1.0]: https://github.com/BarisYazici/franka-rs/releases/tag/v0.1.0
