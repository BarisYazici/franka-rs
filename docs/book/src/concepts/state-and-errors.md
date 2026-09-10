# State and errors

Every millisecond the robot sends a `RobotState`; every failure the crate reports is a
`FrankaError`. This page is the map of both.

## `RobotState`

`franka::RobotState` is a plain `Copy` struct with the field names and semantics of
libfranka's `franka::RobotState`, whose `robot_state.h` documents every field. Every value is
`f64` on both protocol versions; where the wire carries `f32`, the wire layer converts.

| group | fields |
|---|---|
| Frames | `O_T_EE`, `O_T_EE_d`, `O_T_EE_c`, `F_T_EE`, `F_T_NE`, `NE_T_EE`, `EE_T_K`: 4x4 homogeneous transforms, **column major** |
| Load | `m_ee`, `I_ee`, `F_x_Cee`, `m_load`, `I_load`, `F_x_Cload`, `m_total`, `I_total`, `F_x_Ctotal` |
| Elbow | `elbow`, `elbow_d`, `elbow_c`, `delbow_c`, `ddelbow_c` |
| Joint torques | `tau_J` (measured), `tau_J_d` (last commanded; what the rate limiter compares against), `dtau_J` |
| Joint motion | `q`, `q_d`, `dq`, `dq_d`, `ddq_d`, `theta`, `dtheta` |
| Contact and collision | `joint_contact`, `cartesian_contact`, `joint_collision`, `cartesian_collision` |
| External wrench | `tau_ext_hat_filtered`, `O_F_ext_hat_K`, `K_F_ext_hat_K` |
| Cartesian commands | `O_dP_EE_d`, `O_dP_EE_c`, `O_ddP_EE_c` |
| Base acceleration | `O_ddP_O`: the gravity vector as the robot measures it, about `[0, 0, -9.81]` on hardware |
| Accelerometers | `accelerometer_top`, `accelerometer_bottom` (FCI v10 only; zero on an FER) |
| Health | `current_errors`, `last_motion_errors`, `control_command_success_rate`, `robot_mode`, `time` |

**`q` versus `q_d`.** `q` is measured; `q_d` is the position the robot's controller is
commanding right now, and `O_T_EE_c` the commanded pose. Seed a motion from the commanded
values, not the measured ones: the two differ by the tracking error, and on an FER the first
setpoint of a motion is rate limited against the robot's own `q_d`. This is rule 5 of
[The realtime rules](./realtime-rules.md).

**`time`** is a `franka::Duration`, a millisecond counter from the robot rather than a host
clock; `as_secs_f64()` converts it. It advancing by more than 1 ms between two states is how
lost states are counted.

**`control_command_success_rate`** is the fraction of the last 100 commands the robot
accepted, and the health signal of a control loop; cycle 0 always reads 0. What a falling
rate means is in [The realtime rules](./realtime-rules.md).

## `RobotMode`

The seven values of `franka::RobotMode`: `Other`, `Idle`, `Move`, `Guiding`, `Reflex`,
`UserStopped`, `AutomaticErrorRecovery`. `Display` prints them with libfranka's spelling
(`"User stopped"`, `"Automatic error recovery"`), because that text appears inside the
robot's error strings. A read-only probe before a motion should require `Idle` with no error
flags.

## `Errors`

`franka::Errors` is a set of **41** boolean flags, `Errors(pub [bool; 41])`, in libfranka's
order; `franka::ERROR_NAMES` holds the names.

```rust,no_run
# extern crate franka;
# use franka::Robot;
# fn f(robot: &Robot) -> franka::FrankaResult<()> {
let state = robot.read_once()?;
if state.current_errors.any() {
    println!("errors: {:?}", state.current_errors.names());
}
if state.last_motion_errors.get("joint_velocity_violation") {
    println!("the previous motion tripped the joint velocity limit");
}
# Ok(()) }
```

`any()` says whether anything is set, `names()` lists the set flags as `&'static str`,
`get(name)` reads one flag by libfranka's name (unknown names are `false`), and
`Errors::index_of(name)` gives the bit index.

`current_errors` is what is wrong *now*. `last_motion_errors` is the reflex reason of the
motion that last ended, the same set a `ControlException` carries; on a freshly connected
robot it is the history, the reason the previous session's motion ended, which is worth
printing in a pre-flight probe. Two names are easy to misread:
`cartesian_motion_generator_joint_velocity_discontinuity` and its `joint_acceleration`
sibling are raised by a *Cartesian pose* motion, from the robot's check of the joint motion
the poses imply; see [Reflexes, limits and recovery](./safety.md).

## `FrankaError`

One enum with a variant per libfranka exception type, so a `match` covers what a `catch`
chain would. `FrankaResult<T>` is `Result<T, FrankaError>`.

| `FrankaError` variant | libfranka exception | raised when |
|---|---|---|
| `Network(String)` | `NetworkException` | connection or socket failure, including a UDP timeout |
| `Protocol(String)` | `ProtocolException` | malformed or unexpected protocol data |
| `IncompatibleVersion { server_version, library_version }` | `IncompatibleVersionException` | the server speaks another FCI version |
| `Command(String)` | `CommandException` | a TCP command was rejected by the robot |
| `Control(ControlException)` | `ControlException` | a motion ended abnormally: reflex, preemption, discontinuity |
| `Realtime(String)` | `RealtimeException` | `SCHED_FIFO` or `/sys/kernel/realtime` unavailable under `Enforce` |
| `InvalidOperation(String)` | `InvalidOperationException` | not allowed now: a second control loop, or a command the negotiated version does not have |
| `InvalidArgument(String)` | `std::invalid_argument` | non-finite values, an invalid transform, an invalid elbow configuration |
| `Model(String)` | `ModelException` | model loading or evaluation failure |

The message strings are libfranka's, including the `"libfranka: "` prefix, so log lines are
comparable with a C++ client's. Match on the variant, not on the text. `Control` is the only
variant with structured data: `message`, `move_status`, `last_motion_errors` and `log`, the
last states and commands before the end (50 by default). How to read it is in
[Write a 1 kHz callback](../howto/callback-control.md); the flight recorder replays it
([Record and replay a run](../howto/flight-recorder.md)).

Two behaviours differ from libfranka on purpose, a stricter version handshake and
`control_*` never failing with `Realtime`; both are listed under
[Compared with libfranka](../reference/libfranka.md).
