# Robot state and errors

## `RobotState`

`franka::RobotState` is a plain `Copy` struct with the same field names and semantics as
libfranka's `franka::RobotState` — see libfranka's `robot_state.h` for the per-field
documentation, which applies verbatim. Every value is `f64` here, even where the FCI v10
wire format uses `f32` (`tau_J_d`, the accelerometers): the conversion happens in the wire
layer, so user code sees one type on both protocol versions.

Grouped by what they are for:

| group | fields |
|---|---|
| Frames | `O_T_EE`, `O_T_EE_d`, `O_T_EE_c`, `F_T_EE`, `F_T_NE`, `NE_T_EE`, `EE_T_K` — 4×4 homogeneous transforms, **column major** |
| Load | `m_ee`, `I_ee`, `F_x_Cee`, `m_load`, `I_load`, `F_x_Cload`, `m_total`, `I_total`, `F_x_Ctotal` |
| Elbow | `elbow`, `elbow_d`, `elbow_c`, `delbow_c`, `ddelbow_c` |
| Joint torques | `tau_J` (measured), `tau_J_d` (last commanded, what the rate limiter compares against), `dtau_J` |
| Joint motion | `q`, `q_d`, `dq`, `dq_d`, `ddq_d`, `theta`, `dtheta` |
| Contact and collision | `joint_contact`, `cartesian_contact`, `joint_collision`, `cartesian_collision` |
| External wrench | `tau_ext_hat_filtered`, `O_F_ext_hat_K`, `K_F_ext_hat_K` |
| Cartesian commands | `O_dP_EE_d`, `O_dP_EE_c`, `O_ddP_EE_c` |
| Base acceleration | `O_ddP_O` — the gravity vector as the robot measures it; about `[0, 0, -9.81]` on real hardware |
| Accelerometers | `accelerometer_top`, `accelerometer_bottom` (**FCI v10 only**; zero on an FER, whose state datagram has no such arrays) |
| Health | `current_errors`, `last_motion_errors`, `control_command_success_rate`, `robot_mode`, `time` |

Two of these deserve a note.

**`q` vs `q_d`.** `q` is measured, `q_d` is the position the robot's controller is
currently commanding. Seed a joint-position motion from **`q_d`**, not `q`: on FCI v5 the
*first* setpoint of a motion is rate limited against the robot's own `q_d`, because
libfranka 0.9.2's `convertMotion` has no "first command" special case. Starting from `q`
means starting from a value that differs from `q_d` by the tracking error, and the limiter
absorbs the step. This is the [q_d seed rule](./fer.md#the-q_d-seed-rule).

**`time`** is a `franka::Duration` — a millisecond counter from the robot, not a host clock.
It advancing by more than 1 ms between two callbacks is how you count lost states.
`Duration::as_secs_f64()` converts it.

## `RobotMode`

```text
Other | Idle | Move | Guiding | Reflex | UserStopped | AutomaticErrorRecovery
```

The same seven values as `franka::RobotMode`, and `Display` prints them with libfranka's
spelling (`"User stopped"`, `"Automatic error recovery"`) because that text appears inside
the `commandNotPossibleMsg` error strings. A read-only probe before starting a motion
should require `Idle` with no error flags.

## `Errors`

`franka::Errors` is a set of **41** boolean flags — `Errors(pub [bool; 41])` — in
libfranka's own order, with `franka::ERROR_NAMES` giving the names. Note that this is a
plain array, not libfranka's 41 `const bool&` members bound to a struct; the observable
behaviour is the same.

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

- `any()` — is anything set.
- `names()` — the names of the set flags, as `&'static str`, for logging.
- `get(name)` — one flag by libfranka's name; unknown names are `false`.
- `Errors::index_of(name)` — the bit index, if you want to index the array directly.

`current_errors` is what is wrong *now*; `last_motion_errors` is the reflex reason of the
motion that just ended, and is also what a `ControlException` carries. Reading
`last_motion_errors` on a freshly connected robot gives you the *history* — the reason the
previous session ended — which is useful in a pre-flight probe.

## `control_command_success_rate`

The fraction of the last 100 commands the robot accepted, published in every state. It is
the single best health signal for a control loop: a falling rate means the client is
missing its deadline, and a sustained fall ends in a `communication_constraints_violation`
reflex. libfranka's own `communication_test` example asserts an average above 0.90, and
`examples/communication_test.rs` here reports the same min/avg/max.

Note that cycle 0 always reads 0 — nothing has been acknowledged at that point — so skip it
when averaging.

## The error taxonomy

`FrankaError` is one enum with a variant per libfranka exception type, so a `match` covers
the same cases a `catch` chain would:

| `FrankaError` variant | libfranka exception | raised when |
|---|---|---|
| `Network(String)` | `NetworkException` | connection or socket failure, including a UDP timeout |
| `Protocol(String)` | `ProtocolException` | malformed or unexpected protocol data |
| `IncompatibleVersion { server_version, library_version }` | `IncompatibleVersionException` | the server speaks another FCI version |
| `Command(String)` | `CommandException` | a TCP command was rejected by the robot |
| `Control(ControlException)` | `ControlException` | a motion ended abnormally — reflex, preemption, discontinuity |
| `Realtime(String)` | `RealtimeException` | `SCHED_FIFO` or `/sys/kernel/realtime` unavailable under `Enforce` |
| `InvalidOperation(String)` | `InvalidOperationException` | not allowed in the current state — a second control loop, or a command the negotiated version does not have |
| `InvalidArgument(String)` | `std::invalid_argument` | non-finite values, an invalid transform, an invalid elbow configuration |
| `Model(String)` | `ModelException` | model loading or evaluation failure |

`FrankaResult<T>` is `Result<T, FrankaError>`. The message strings are libfranka's, byte
for byte, including the `"libfranka: "` prefix — a unit test pins the
`IncompatibleVersion` text, for instance. Two consequences:

- Log lines are comparable with a C++ client's, which matters when you are debugging the
  same robot from both.
- Matching on message text is as (un)reliable as it is in C++; match on the variant.

`Control` is the only variant that carries structured data — see
[`ControlException` and the control log](./controlling-the-robot.md#controlexception-and-the-control-log).

Two divergences from libfranka are deliberate and documented:

- A `kSuccess` handshake that reports a *different* version than the client announced is
  also `IncompatibleVersion` here. libfranka only inspects the status byte, but everything
  after the handshake is decoded against the announced version, so such a session is
  unusable by construction.
- `control_*` never fails with `Realtime`. Realtime priority is raised in `Robot::new`,
  matching libfranka's `Robot::Impl` constructor, so by the time a control loop starts the
  question is settled.
