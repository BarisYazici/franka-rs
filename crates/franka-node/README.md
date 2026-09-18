# franka-node

A [Zenoh](https://zenoh.io) node in front of `franka-rs`'s target control. One process owns
one or more `franka::Robot`s and, per arm, runs `Robot::start_cartesian_target_control` or
`start_joint_target_control` with the impedance backend. Clients publish pose or joint
targets at any rate over Zenoh; the node gates them (lease, order, kind, finite, step,
bounds, rate), hands them to the library's generator, publishes the arm's state at 100 Hz
and answers `acquire`/`enable`/`stop`/`release`/`recover`/`home` on queryables; an arm's
gripper (the Franka Hand, or another driver through the `Gripper` trait) has a target key,
a state key and two verbs of its own. It is a library plus the `franka-node` binary. The book's
[Serve arms over Zenoh](https://barisyazici.github.io/franka-rs/howto/franka-node.html) page
is the guided version of this README, with a Python commander and the Raspberry Pi build.

## Data path

```
Zenoh RX ──send──▶ mpsc channel, at most 64 targets in flight ──▶ arm thread (one per arm:
                                                                   robot, control handle,
                                                                   guard, state machine,
                                                                   watchdog)
                                                   │ set_target
                                                   ▼
                                       1 kHz SCHED_FIFO loop (the library's)
                                                   │ state()
arm thread, every 10 ms: StateMsg ──▶ Zenoh publisher
```

The Zenoh callbacks decode a fixed layout with `zerocopy` and send without blocking: a
target beyond 64 in flight is dropped and counted, commands and lease events are never
dropped. The arm thread drains the channel, runs the commands and lease events in arrival
order and judges only the newest target of a drain (the rest count as `dropped`). The
realtime loop never sees Zenoh, the channel or an allocation from the node.

## Keys

| key | kind | content |
|---|---|---|
| `franka/<arm>/target` | subscriber | `TargetMsg`, latest wins |
| `franka/<arm>/state` | publisher | `StateMsg` at `state_hz` |
| `franka/<arm>/episode` | publisher | JSON at a session's start and end: the recording's id and file |
| `franka/<arm>/cmd/{acquire,release,enable,stop,recover,home}` | queryable | JSON request and reply |
| `franka/<arm>/gripper/target` | subscriber | `GripperMsg`: a width or a grasp, from the lease holder |
| `franka/<arm>/gripper/state` | publisher | `GripperStateMsg` at 20 Hz, while the arm has a gripper |
| `franka/<arm>/cmd/{gripper_home,gripper_stop}` | queryable | JSON; `gripper_home` is the holder's, `gripper_stop` anyone's |
| `franka/<arm>/lease/<client_id>` | liveliness token | declared by the client while it lives |
| `franka/node/<name>/status` | publisher | JSON at 1 Hz: the node and every arm's phase, holder, mode, counters and gripper |

`<arm>` is the `name` of the `[[arm]]` table (`[A-Za-z0-9_-]+`, and neither `node` nor `cam`:
`franka/node/*` is the status and `franka/cam/*` belongs to a camera node); `<client_id>` is a
non-zero `u32` the client picks; `<name>` is the node's `name`.

The status is `{"node": "node1", "version": "0.1.0", "uptime_s": 61, "arms": [{"name": "fr3",
"phase": "active", "holder": 7, "mode": "cartesian", "accepted": 1210, "refused": 0,
"dropped": 0, "decode_failures": 0, "success_rate": 0.99, "recording": null, "episode":
null, "gripper": {"width": 0.04, "calibrated": true, "grasped": false, "fault": false}}]}`: `phase`
and `mode` are the lowercase names (`mode` is `null` without a session), `accepted`,
`refused` and `dropped` count since enable as in `StateMsg` (`refused` includes refused
gripper commands), `decode_failures` counts targets and gripper commands whose bytes were
not a message since the node started, `success_rate` is the robot's, `recording` is the
episode file being written (see Recording), `episode` is the `recording_id` of the session
that started last and outlives it, `gripper` is the arm's gripper as of its last
published state (`null` without one). A status thread reads per-arm atomics the arm thread
stores into with every state tick, so a stalled arm thread shows as a frozen entry rather
than a missing one; a missing sample means the node is gone.

## Wire messages

Little-endian, `#[repr(C)]`, every field of alignment one (no padding). The first byte is
the protocol version, 1; a target of another version or length is refused. `tests/wire.json` is
the same layout machine-readable (sizes, fields, one sample per message, codes, JSON payloads),
checked against these types by a unit test and against the Python client by its tests.

`TargetMsg`, 80 bytes, Python `struct.Struct("<BBHIQQ7d")`:

| field | type | offset | content |
|---|---|---|---|
| `version` | u8 | 0 | 1 |
| `kind` | u8 | 1 | 1 Cartesian, 2 Joints; must be the session's mode |
| `flags` | u16 | 2 | bit 0: anchor (this target's step is measured from the arm's measured pose, not from the previous accepted target). Every other bit is reserved, 0, and ignored, so a flag the node does not know never refuses a target |
| `client_id` | u32 | 4 | the lease holder's id |
| `seq` | u64 | 8 | per client, strictly increasing |
| `t_send_ns` | u64 | 16 | the sender's clock, echoed, never interpreted |
| `data` | 7 × f64 | 24 | Cartesian: position xyz, m, and quaternion xyzw, base frame; Joints: `q`, rad |

`StateMsg`, 474 bytes, Python `struct.Struct("<BBBBHIQQQQd7d7d7d16d6d7dQQQ")`:

| field | type | offset | content |
|---|---|---|---|
| `version` | u8 | 0 | 1 |
| `phase` | u8 | 1 | 0 Idle, 1 Acquired, 2 Active, 3 Stopping, 4 Faulted, 5 Homing |
| `robot_mode` | u8 | 2 | `franka::RobotMode` in declaration order: 0 Other, 1 Idle, 2 Move, 3 Guiding, 4 Reflex, 5 UserStopped, 6 AutomaticErrorRecovery |
| `has_errors` | u8 | 3 | 1 if `current_errors` is non-empty |
| `flags` | u16 | 4 | bit 0: holding (Active, no target accepted for `hold_after_ms`); bit 1: joints (a joints session or `home` runs and `target` is `q`) |
| `client_id` | u32 | 6 | the lease holder, 0 if none |
| `seq_accepted` | u64 | 10 | the last accepted target's `seq`, 0 if none |
| `t_send_ns_accepted` | u64 | 18 | its `t_send_ns`, for the client's own round trip |
| `t_node_ns` | u64 | 26 | the host's `CLOCK_MONOTONIC` at encode, so a second process on the same host can compare its own stamps with it |
| `robot_time_ms` | u64 | 34 | `RobotState.time` |
| `success_rate` | f64 | 42 | the robot's `control_command_success_rate`; meaningful while Active |
| `q`, `dq`, `tau_ext` | 3 × 7 f64 | 50, 106, 162 | joint positions, velocities, `tau_ext_hat_filtered` |
| `o_t_ee` | 16 × f64 | 218 | `O_T_EE`, column-major |
| `o_f_ext_k` | 6 × f64 | 346 | `O_F_ext_hat_K` |
| `target` | 7 × f64 | 394 | the current target: position and quaternion xyzw, or `q` with the joints flag; the measured pose outside a session |
| `accepted`, `refused`, `dropped` | 3 × u64 | 450, 458, 466 | targets since enable |

`GripperMsg`, 40 bytes, Python `struct.Struct("<BBHIQQdd")`:

| field | type | offset | content |
|---|---|---|---|
| `version` | u8 | 0 | 1 |
| `kind` | u8 | 1 | 1 width (move to `width`), 2 grasp (close on an object at `width` until `force` holds) |
| `_pad` | u16 | 2 | 0 |
| `client_id` | u32 | 4 | the lease holder's id |
| `seq` | u64 | 8 | per client, strictly increasing; its own sequence, not the targets' |
| `t_send_ns` | u64 | 16 | the sender's clock, never interpreted |
| `width` | f64 | 24 | m, `0 <= width <= max_width` |
| `force` | f64 | 32 | N, finite and non-negative; ignored for a width |

`GripperStateMsg`, 40 bytes, Python `struct.Struct("<BBHIQddd")`:

| field | type | offset | content |
|---|---|---|---|
| `version` | u8 | 0 | 1 |
| `flags` | u8 | 1 | bit 0 calibrated, bit 1 grasped, bit 2 moving, bit 3 fault |
| `_pad` | u16 | 2 | 0 |
| `client_id` | u32 | 4 | the arm's lease holder, 0 if none |
| `t_node_ns` | u64 | 8 | the host's `CLOCK_MONOTONIC` at the driver's reading of `width` |
| `width` | f64 | 16 | measured, m |
| `commanded` | f64 | 24 | the last commanded width, m; the measured one until a command |
| `max_width` | f64 | 32 | m; 0 until the gripper knows it |

Commands are request/reply JSON: request `{"client_id": 7}`, reply `{"ok": true}` or
`{"ok": false, "error": "<reason>"}`. `enable` also takes `"mode": "cartesian" | "joints"`
(default cartesian); `home` also takes `"speed": 0.2`, the fraction of the arm's joint
limits it moves at (default 0.2, clamped to 0.05..=0.5). Both take `"episode": "<name>"`,
`[A-Za-z0-9_-]{1,128}`, the collector's name for the session (see Recording); a name of
another shape is refused with the reason and nothing is started. Unknown keys are refused.

## State machine

```
Idle ──acquire──▶ Acquired ──enable──▶ Active ──stop | watchdog | lease lost──▶ Stopping ──▶ Idle
 ▲                  │ release  ▲                 │ loop ended with an error
 │                  │          └─ Homing ◀─home──┤
 │                  │   (stop | timeout | arrived: Stopping ──▶ Acquired)
 └──── recover ◀── Faulted ◀─────────────────────┘
```

| verb | allowed in | effect |
|---|---|---|
| `acquire` | Idle, Acquired | needs the client's liveliness token and a free arm (`"no lease"`, `"held by <id>"`) |
| `enable` | Idle with the lease, Acquired | reads the state, sets the collision thresholds, starts the Cartesian or the joint loop (`mode`); the guard steps from the loop's start target |
| `stop` | Active, Homing | blocks for the library's settle (about 0.3 s, at most 5 s; state publishing pauses), then Idle (Acquired after a `home`) with the holder kept; a failing stop is Faulted. A stop during `home` decelerates in place: the loop is re-targeted to the measured configuration before it stops |
| `release` | Idle, Acquired, Faulted | clears the holder; Acquired goes to Idle, Faulted stays Faulted |
| `recover` | all but Active, Homing | `automatic_error_recovery` over TCP, no motion, then Idle; the holder is kept |
| `home` | Acquired | a joints session at `speed` of the limits (0.05..=0.5, default 0.2; first runs on an arm at 0.2 or below) towards the ready pose `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`; the reply comes when every joint is within 0.02 rad of it, at rest or for 10 consecutive state ticks (then Acquired), or `"home timed out"` after 60 s. Every early end (stop, lost lease, timeout, shutdown) decelerates in place |
| `gripper_home` | any, the holder | the gripper's homing; the reply comes when it is over (the Hand driver gives up after 30 s); one at a time (`"gripper homing"`); `"no gripper"` without one |
| `gripper_stop` | any, anyone | stops the gripper's move or grasp now; `"no gripper"` without one |

In Active, a target passes the guard and reaches `set_target`, or is counted as `refused`
with the reason in the debug log. `hold_after_ms` without an accepted target sets the holding
flag, `stop_after_ms` without any target from the commander stops the loop; the holder keeps
the lease and may `enable` again. A lost lease stops the loop and clears the holder. A loop that
ended on its own is Faulted.

The two watchdogs measure different things, and the difference matters for anything that
streams a human's hand. `hold_after_ms` measures from the last **accepted** target, so the
holding flag means what it says: nothing has been accepted, and the arm is sitting on the last
target that was. `stop_after_ms` measures from the last target that **arrived** from the holder
in order — accepted, or refused on its content or its rate — so a commander whose targets are
all being refused is held but not stopped. That is deliberate: a commander parked at the
workspace wall, outrunning the arm, or pushing against a hand on the arm is alive and talking,
stopping its session tells it nothing, and the stop blocks the arm thread for the library's
settle (up to 5 s) and lands in Idle needing a fresh `enable`. Targets that are not the
commander's — the wrong kind byte, not the holder, out of order — are counted but are no sign
of life, so a silent commander is still stopped whatever else is on the topic. Watch `refused`
in the status: a `holding` arm with `refused` climbing is a commander being refused, not one
that has gone away, and one `anchor` target is usually what it needs to be accepted again.

`max_step` bounds the distance between one accepted target and the next, not a speed: at
`rate_hz` a commander may legally walk the target far faster than the arm can follow, and the
desired pose then runs ahead of the measured one until the backend's leash caps the spring. Two
consequences: the force settles at the plateau the leash produces, 40 to 50 N with the default
gains, and when the commander stops sending, the arm finishes travelling to the last target it
accepted rather than stopping where it is. `max_lead` is the bound on that: an accepted
Cartesian target is never further than 0.05 m, as a norm, from `o_t_ee`, nor its orientation
more than `max_lead_rotation` from the measured one, so whatever the commander does the arm
coasts at most that plus its own brake distance. Either limit set to 0 turns that check off.
A commander that follows a human hand should still keep its own leash, tighter than this one,
so that the node's limit never binds in ordinary use; a lead limit at or below `leash` is a
config error, because the backend's own leash is the tracking error a healthy commander already
has. For the same reason a commander whose own step exceeds the per-axis budget (0.173 m/s of
the default `budget`, so 3.5 mm per message at 50 Hz or 1.7 mm at 100 Hz) will sit in continuous
lead refusal: `examples/client --dz 0.2` and a 0.05 m sine of period 1 s are past that, while
the defaults of both have 2x and 5x of margin.

Because the step is measured against the previous accepted target and the lead against the
arm, a stream that jumps — a re-latched clutch, a tracking frame that moved, an arm a hand has
pushed away from the target it holds — can end up refused for ever: a refusal leaves the
previous target where it was, so every later target is too far from it. The `anchor` flag is
the way back in. A target with bit 0 of `flags` set has its step measured from the arm's
measured pose instead of from the previous accepted target; nothing else changes, and in
particular `max_lead` still applies, so an anchored target is bounded by the smaller of
`max_step` and `max_lead` from where the arm actually is. That is tighter than the ordinary
path, which allows a step from a previous target that may itself lead the arm, so anchoring
every message is not a way around the gate — it is the most conservative way to command it.
In a joints session the bit does nothing, there being no Cartesian reference, and setting
either lead limit to 0 turns the anchor off with it: without the lead bound the anchor would be
the one way to command a jump of any size, `max_step` from a measured pose that may be anywhere.

The guard's Cartesian rules are the translation and rotation step from the previous accepted
target (or from the measured pose, with the anchor flag), the lead limits above, a unit
quaternion and, when a config asks for one, the workspace box; its joint rules are a per-joint
step of at most `max_step_joint` and the arm's joint limits (the FER's or the FR3's, by the
negotiated FCI version) inset by `joint_position_margin`. Both check the lease holder, the
sequence, finite values and the rate; a target of the other kind is refused. A refusal spends
no token and leaves the previous target and the sequence where they were. During `home` every
target is refused (`"homing"`), `stop` ends it early (its reply is `"home stopped"`), a lost
lease ends it and clears the holder. A joints session budgets `joint_budget_fraction` of the
arm's joint limits and runs the library's joint impedance with a deviation guard of
`joint_max_deviation` from the start; `home` runs at its `speed` and widens that guard to
`max(joint_max_deviation, travel + 0.5 rad)`.

## Configuration

```toml
name = "node1"

[zenoh]
mode = "peer"                        # or "client": dial the routers in connect, listen on nothing
listen = ["tcp/0.0.0.0:7447#iface=eth0"]
connect = []
scouting_interface = "eth0"          # optional; "auto" when absent
lease_ms = 1000                      # transport.link.tx.lease, keepalive is lease/4
# zenoh_config = "/etc/franka/zenoh.json5"   # a Zenoh file to start from: TLS material, auth, ACLs

[[arm]]
name = "fr3"
host = "172.16.0.2"
realtime = "enforce"                 # or "ignore"
state_hz = 100
hold_after_ms = 200
stop_after_ms = 2000
collision_force = 40.0               # set_collision_behavior_simple(torque, force)
collision_torque = 40.0
budget = [0.3, 0.5, 20.0]            # OtgLimits for translation (norm)
rotation_budget = [0.5, 1.0, 20.0]
cartesian_stiffness = 750.0          # ImpedanceGains, N/m; rotational = stiffness/50
max_deviation = 0.3                  # m the measured pose may stray from its start before the loop ends
max_angular_deviation = 0.5          # rad, the same guard on the orientation
leash = { translation = 0.025, rotation = 0.15 }   # m, rad the desired pose may run ahead of the measured
max_step = 0.05                      # m per target from the previous accepted one
max_step_rotation = 0.26             # rad per target
max_step_joint = 0.2                 # rad per joint per target in a joints session
max_lead = 0.05                      # m a target may lead the measured pose; 0 disables,
                                     # otherwise must exceed leash.translation
max_lead_rotation = 0.26             # rad, the same on the orientation; 0 disables
joint_budget_fraction = 0.2          # of the arm's joint limits, (0, 1]; a joints session's budget
joint_max_deviation = 1.0            # rad a joint may stray from its start in a joints session
joint_velocity_fraction = 0.7        # of the arm's joint velocity limits the goal may move at, (0, 1]
velocity_barrier_fraction = 0.85     # a joint measured faster meets a damping barrier, [the above, 1];
                                     # the law's push along a joint's motion fades out from the lower
                                     # of the above and this less 0.15 to this
joint_position_margin = 0.05         # rad the joint goal keeps from the joint position limits, [0.035, 0.5];
                                     # joint targets inside it are refused
# workspace = { min = [0.2, -0.5, 0.0], max = [0.8, 0.5, 0.8] }
                                     # the box a target must lie in, m, base frame; off unless set
rate_hz = 250.0                      # targets per second per client, bucket of twice that
realtime_priority = 80               # optional
# gripper = "hand"                   # optional: the Franka Hand at host, FCI port 1338
gripper_speed = 0.1                  # m/s of every gripper move and grasp
# cpu = 2                            # optional: pin the loop thread to this core (isolcpus)
# record_dir = "/var/lib/franka-node"   # optional, needs --features record: one .rrd per session
# record_meshes = "/opt/franka-meshes"  # optional: meshes other than the built-in, or "none"
```

`name` and `host` of an arm are required, the rest has defaults; the example
(`config.example.toml` is this file) shows them except for `name` (`franka-node`), `mode`
(`peer`), `listen` (`tcp/0.0.0.0:7447`, no interface), `scouting_interface` (auto),
`realtime_priority` (the highest), `gripper` (none), `cpu` (not pinned) and `record_dir` (no
recording).

`joint_velocity_fraction` and `velocity_barrier_fraction` are the library's `ImpedanceOptions`
fields of the same names, for both session kinds: the loop scales the joint goal's step so no
joint is commanded faster than that fraction of the arm's velocity limit (the FER's or the
FR3's, by the negotiated FCI version), the law's torque along a joint's measured motion fades
out as the joint speeds from the cap fraction, or 0.15 under the barrier fraction if that is
lower, to the barrier fraction, and a joint measured faster than the barrier fraction meets a
damping of 20 Nm per rad/s of the excess. At the defaults, 0.7 and 0.85, the fade spans 0.7 to
0.85 of the limit. `joint_position_margin`, the library's field of the same name, is the
distance the joint goal keeps from each joint position limit: the IK (or, in a joints session,
the step's scale) brakes the goal to stop there, and a joint measured inside it has the law's
push toward the limit faded out over 0.02 rad, past which a spring pushes it back out. With the
`record` feature every cycle's `joints/q_goal`, `joints/dq_goal`, `joints/cap_scale`, `joints/pinned`,
`joints/tau_envelope`, `joints/tau_position`, `ik/stall` and `ik/passes` are in the recording.

`budget` is the library's default, 0.3 m/s at 0.5 m/s² and 20 m/s³ as a norm, which exists
because the robot's own controller refuses a pose stream whose IK crosses a joint's
acceleration limit. This node runs the impedance backend, where torques are not checked that
way, so a commander a human drives wants the acceleration raised — `budget = [0.3, 1.5, 20.0]`
— because the acceleration, not the velocity, is what decides how long the arm takes to stop:
at the default it needs 0.625 s and 5.4 cm on one axis to brake from its own top speed, at
1.5 m/s² it needs 0.275 s and 2.4 cm, and `max_lead` plus that distance is the whole coast.
The derived cost is 8 mm and 6 N of extra tracking error from the spring that produces the
acceleration. It is a per-arm setting and not the default because that arithmetic is not
validated on a robot.

`mode = "client"` is for a commander that is not on the robot's network: the node dials the
routers named in `connect` and listens on nothing, so no inbound rule and no port forwarding is
needed, and `listen` is ignored. It must then name at least one router, and an endpoint that
leaves the local segment should be `tls/` rather than `tcp/`. The TLS transport is compiled in;
the certificates, any authentication and any access control go in a Zenoh configuration file
named by `zenoh_config`, which is the base this table's keys are applied on top of. A client's own
liveliness, watchdog and guard behaviour is unchanged: a commander that stalls is held after
`hold_after_ms` and stopped after `stop_after_ms` however far away it sits. Unknown
keys are errors. `multicast_scouting` may be set explicitly; by default it is on unless the
node only connects. `gripper` takes any non-empty name; which names exist is the binary's
decision (see Grippers), so the public `franka-node` refuses everything but `"hand"` at start.

## Grippers

An arm's gripper is a `franka_node::Gripper`, a small `Send + Sync` trait: `command(width)`
(latest wins, the driver moves at its own limits), `grasp(width, force)`, `home()` (blocks
until the homing is over), `stop()` and `state()`; metres everywhere, the 0..1 convention of
teleoperation stacks is the client's business. The arm thread owns the driver: a `GripperMsg`
is accepted from the arm's lease holder only (same lease, own `seq`, finite width within
`max_width`, finite non-negative force) and forwarded, or counted as `refused`;
`gripper_home` runs on a thread of its own so the arm's ticks and watchdog go on;
`gripper_stop` is anyone's. The gripper works in every arm phase, Idle included, and the
arm's `stop`, watchdog and lease loss leave it alone (lease loss clears the holder, so the
next gripper command needs a new lease). Its state goes out every `state_hz / 20`th tick and
into the status.

The driver in the crate is `FrankaHand`, the Franka Hand over the library's
`franka::Gripper` (FCI port 1338 at the arm's `host`): a worker thread runs the commands,
which block on the hand for up to seconds (`move_gripper` at `gripper_speed` when the target
differs by more than 1 mm from the last commanded width; `grasp` with libfranka's 5 mm
epsilons; `homing`), a reader thread keeps the state at 20 Hz, and `stop` drops what is
pending and goes out from a thread of its own past a running move, as libfranka's does, so
the arm thread never waits on the hand. `fault` is a lost command or state channel; a
command the hand refuses or reports unsuccessful (out of stroke, blocked, aborted by a
stop) is a `warn` line and leaves no commanded width. Other drivers plug in
through `franka_node::run(config, factory)`: the binary is that call with a factory mapping
`gripper = "hand"` to `FrankaHand::connect`; a binary of your own may map other names to
its own `Gripper` and reuse everything else.

## Live tuning

A running Cartesian impedance session accepts parameter updates over
`franka/<arm>/params/{schema,get,set}` and publishes `params/current`. The node derives its
schema from the library's bounds table; changes use the controller's slew and budget gates.
The optional [browser panel](../../tools/tuning-panel/README.md) provides schema-driven controls.

Read [Tune a running controller](../../docs/book/src/howto/live-tuning.md) for the workflow,
feedforward settings and session lifetime; [Live parameter protocol](../../docs/book/src/reference/node-parameters.md)
defines requests, replies, confirmations and versioning. Values reset to the TOML on session
end or node restart; saving a panel preset does not persist the node configuration.

[config.two-arms.toml](config.two-arms.toml) shows two arms with the shipped defaults and
optional per-arm CPU pinning and Franka Hands. Use it as a deployment example, not a tuned
controller profile.

## Recording

With `cargo build --release -p franka-node --features record` (pulls `franka-rerun` and
Rerun 0.37.1, Rust 1.96) and `record_dir` set, every session of the arm, an `enable` in
either mode or a `home`, is one Rerun recording `<record_dir>/<arm>-<UTC
yyyymmddThhmmssZ>.rrd` (a second session within the same second gets `-2`), opened when
the loop starts and closed when it stops. It holds the library's flight recording, every
1 kHz cycle's state and the pose or joint positions the loop sent (`franka_rerun::Recorder`;
the observer copies one record into a ring, no allocation on the realtime thread), plus what
the arm thread logs under `commander/target/*`: each accepted target's
position and quaternion or `q`, its `seq` and `client_id`, and each refusal as a line in
`events` with its reason. The kinematic model is loaded from the robot when the node starts
and kept; if that fails, a warning is logged and the arm records nothing. The status names
the open file in `recording`; the log has the path, the pushed and dropped counts and the
closing time at `info`. The arm is drawn with the FR3's or the FER's meshes built into
`franka-rerun`; `record_meshes` names a directory of other ones (`link0..7`, `hand`, `finger`),
or `"none"` for the skeleton alone. Open a file with the 0.37.1 viewer, `rerun <file>.rrd` (`pip install
rerun-sdk==0.37.1`). A `record_dir` without the feature is a config error; a recording that
cannot be opened (directory not writable, `record_meshes` without link meshes) is a warning
and the session runs unrecorded.

Every entity is under the arm's name (`L/joints/q`, `L/commander/target/x`), and every row
carries two timelines: `robot_time`, the controller's own millisecond counter, and `host_time`,
the host's `CLOCK_MONOTONIC` read where the record was taken. The two arms of a node are two
controllers, so their `robot_time`s are unrelated and `host_time` is the only axis they share.
The layout the node sends names *every* arm of the config, so a recording two of them share
lays out both whichever arm's blueprint the viewer reads first.

### One recording for two arms

`enable` (and `home`) takes `"episode": "<name>"`. With one, the session's recording id is that
name, the arm's file is `<name>-<arm>.rrd`, and the episode topic publishes the name; without
one nothing changes. Two arms enabled with the same name write two files that a viewer loads as
one recording, with no coordination in the node -- the collector names the episode, which is
what a dataset wants anyway -- and the time panel of such a recording opens on `host_time`:

```sh
# the client example, one process per arm (see The client example)
cargo run --release -p franka-node --example client -- L --episode pick-0042 --seconds 20 &
cargo run --release -p franka-node --example client -- R --episode pick-0042 --seconds 20
rerun pick-0042-L.rrd pick-0042-R.rrd     # or route them into one file first:
rerun rrd route --recording-id pick-0042 pick-0042-*.rrd -o pick-0042.rrd
```

Each arm keeps its own ring buffer, writer thread and file, so the realtime properties and the
failure isolation are what they were for one arm. The layout of a named episode is the one that
names every arm of the config; an unnamed session, whose file is nobody else's, keeps the layout
of its own arm and opens on `robot_time`.

Keeping the names unique is the collector's job. A name used twice in one `record_dir` gets
`-2` on the second file, but both files carry the name as their recording id, so a viewer handed
the directory loads the two sessions as one recording -- the node will not rename an episode
behind the collector's back, because the name is what the other arm and any camera node were
told.

## Episodes

Every session's start and end go out on `franka/<arm>/episode` as one JSON line, whether the
`record` feature is built or not: `{"arm": "fr3", "recording_id": "fr3-20260101T101500Z",
"file": "fr3-20260101T101500Z.rrd", "t_node_ns": 8123456789, "phase": "start"}`, and the same
`recording_id` with `"phase": "end"` once the loop has stopped and the file is closed.

With the feature and a `record_dir`, `recording_id` is the Rerun `RecordingId` of the arm's
own `.rrd`: the `episode` name the session was enabled with, or the file's stem without one.
Without a recording it is generated in the same shape and `file` is `null`. A recorder in
another process, a camera node for instance, writes a file of its own under that id and the
application id `franka_rerun::APPLICATION_ID`, the string `franka_rs`, and a viewer then loads
the two as one recording. A viewer keys a store by both ids, so the application id is as
necessary as the recording id.

Both samples are best effort: the publisher drops rather than blocks, as every other one here,
and a node that is killed sends no end at all. Treat them as markers, not as the truth about
what is running, and read `phase` from `franka/<arm>/state` or the status for that. A
subscriber that missed a start finds the id in the status topic's `episode`, which outlives
the session. The start goes out once the loop is already running, so a frame captured in the
few milliseconds before it arrives belongs to the episode but cannot be attributed to it.

## Install

As of 18 September 2026, the node is not yet published on crates.io. From the repository
root, install this checkout with:

```sh
cargo install --path crates/franka-node --locked
```

The following commands require a matching crates.io and GitHub release:

```sh
cargo binstall franka-node             # prebuilt: aarch64 (gnu, static musl) and x86_64, with record
cargo install franka-node --locked     # or compiled, on a Pi too: Rust 1.89, 1.96 with --features record
```

Without cargo, a release's `franka-node-<version>-<target>.tar.gz` holds the binary,
`config.example.toml`, the systemd unit, LICENSE and NOTICE; the latest for Raspberry Pi OS 64-bit:

```sh
tag=$(curl -fsSLo /dev/null -w '%{url_effective}' https://github.com/BarisYazici/franka-rs/releases/latest)
tag=${tag##*/}
curl -fsSL https://github.com/BarisYazici/franka-rs/releases/download/$tag/franka-node-${tag#v}-aarch64-unknown-linux-gnu.tar.gz | tar -xz
```

`aarch64-unknown-linux-gnu` needs glibc 2.31 or newer, `aarch64-unknown-linux-musl` is static.
`deploy/README.md` installs the unpacked files as a service. A binary of your own around the
library, with another gripper driver, is `cargo add franka-node`.

## Running

```sh
franka-node config.toml                                              # installed
cargo run --release -p franka-node -- config.toml
RUST_LOG=debug cargo run --release -p franka-node -- config.toml    # refusals with reasons
```

Transitions are logged at `info`, refusals at `debug`. SIGINT or SIGTERM stops every arm
and exits. `realtime = "enforce"` needs a `PREEMPT_RT` kernel, as `franka::Robot::new`,
which puts its calling thread on `SCHED_FIFO`; the node constructs each robot on a scratch
thread, so its own threads run at normal priority and only the library's loop thread is
realtime. `deploy/franka-node.service` is a systemd unit for a host next to the robots (the
limits Franka's realtime setup recommends, restart on failure, SIGINT on stop);
`deploy/README.md` has the install steps.

## The client example

```sh
cargo run --release -p franka-node --example client -- fr3 --connect tcp/127.0.0.1:7447 \
    [--hz 20] [--dz 0.05] [--seconds 4] [--mode joints] [--episode NAME]
cargo run --release -p franka-node --example client -- fr3 home --connect tcp/127.0.0.1:7447 \
    [--speed 0.2] [--episode NAME]
cargo run --release -p franka-node --example client -- fr3 gripper 0.04 --connect tcp/127.0.0.1:7447
cargo run --release -p franka-node --example client -- fr3 gripper grasp 0.02 20 --connect ...
cargo run --release -p franka-node --example client -- fr3 gripper home --connect ...
```

Declares its lease token, acquires and enables the arm, publishes a sine in z of amplitude
`--dz` around the pose at enable (with `--mode joints`, a sine of 0.1 rad on joint 7 around
the start configuration), prints the node's report once a second (phase, holder, robot
mode, target z or joint 7, measured x y z, the robot's success rate, the accepted, refused
and dropped counters, the round trip from `t_send_ns_accepted`), then stops and releases.
`--episode NAME` names the session, so two clients driving two arms with one name record into
one recording (see Recording). `--help` prints the usage. `home` takes the lease, sends `home`
and waits for the arm to arrive (up to 90 s), prints the outcome and `q`, and releases. With another verb,
`client fr3 recover|stop|release [--connect ...]`, it sends that one command and no motion,
prints the reply and the state it settled in, and exits 0 on success; `release` takes the
holder's id as `--client-id N` (`stop` and `recover` need no lease). `gripper <width_m>`
takes the lease, publishes one width and waits until the gripper state is within 3 mm of it
(up to 10 s); `gripper grasp <width_m> <force_n>` publishes one grasp and waits for the move
to end; `gripper home` sends `gripper_home`; each prints the gripper's state and releases.
`examples/client/common.rs` is the client side of the keys, shared with the simulator test.

## Python client

`pip install franka-node-client` (import `franka_node`, source in `python/`) is the client for
Python: pure Python over `eclipse-zenoh` and numpy, released with the crates under the same
version. It holds the lease, paces targets under `max_step` and `max_lead` from a streamer thread
that is also the keepalive, and exposes the `franka` bindings' verbs:

```python
import franka_node

with franka_node.Node("tcp/<node-host>:7447") as node, node.arm("fr3") as arm:
    with arm.cartesian_targets() as t:
        t.move_by([0.0, 0.0, 0.05])
        t.wait(timeout=5.0)
```

The book's [Serve arms over Zenoh](https://barisyazici.github.io/franka-rs/howto/franka-node.html#from-python)
page describes it; `python/tests/test_sim_node.py` runs it against the node binary and franka-sim.

## Tests

Unit tests cover the wire layout and offsets, every guard refusal in both modes, the config
and the state machine against a fake robot (both modes, `home` arriving, timing out, stopped,
losing its lease) and a fake gripper (the holder rule, every refusal, both verbs, the 20 Hz
state and the status entry). The integration tests run the node in process against
[franka-sim](https://github.com/BarisYazici/libfranka-sim) (>= 1.1.6, with
`--enforce-motion-limits`): the Cartesian session, the watchdog, refusals and the lost
lease; then a joints session, a Cartesian target refused in it and `home` from 1.1 rad away;
then the gripper through `FrankaHand` against the sim's gripper server (the 20 Hz state, a
non-holder refused, `gripper_home`, a width, a grasp in thin air, `gripper_stop` from a
second client, the status entry; the sim's grasp in thin air closes fully and its stop
re-opens the fingers, which the test takes as given):

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-node \
    --test sim_node -- --test-threads=1
```

With `--features record` the same tests record every session into a temporary directory
and check the files (one per session, the Cartesian one above 10 kB).

## Status

Runs against franka-sim in CI. The node runs on a Raspberry Pi 5 (`PREEMPT_RT`) with one or
two arms per process and a client on another host: Cartesian and joint sessions, `home` and a
`stop` during it, `recover`, the deadman (a frozen client stops the arm in about 1 s, without a
reflex), lease loss when a client is killed, the episode topic, and recording on both arms.
Pushed past the leash, a held arm settles at a force plateau of 40 to 50 N whatever the
distance; a hard, fast push trips a `cartesian_reflex`.

Not yet validated on hardware: pushing the arm in a joints session, and the gripper (the
trait, the Franka Hand driver, the keys and verbs), which is tested against franka-sim's
gripper server only.
