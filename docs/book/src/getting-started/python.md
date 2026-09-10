# From Python

At the end of this page a Python program running at its own rate drives the arm through the
same target control as the [first program](./first-program.md), and you know where the
notebook and the examples are.

```sh
pip install franka-rs
```

The wheel is `franka-rs`, the package `franka` (Python 3.9 or newer, numpy).

```python
import franka

robot = franka.Robot("172.16.0.2")   # FRANKA_REALTIME=ignore for franka-sim

with robot.cartesian_targets(max_velocity=0.3, max_acceleration=0.5, max_jerk=20.0) as arm:
    while not done:
        obs = arm.state().flat()      # 1-D float64 in the order of franka.FLAT_LAYOUT
        action = policy(obs)
        arm.move_by(action[:3])       # metres, relative to the current target
        # or: arm.move_to([x, y, z]); arm.follow(chunk, dt=0.02)  # chunk: (N, 3) absolute
```

`cartesian_targets` starts the 1 kHz loop on a Rust thread, and the GIL is never on it: a
`move_*` is one write into the loop's slot, `state()` a copy out of a mutex. Leaving the
`with` block stops the loop; an exception inside it still stops the motion and is re-raised.
The keyword arguments are the Rust [`TargetControlOptions`](../howto/target-control.md)
(`max_deviation`, `max_angular_velocity`, `max_angular_acceleration`, `max_angular_jerk`,
`max_angular_deviation`, `backend`, `cartesian_stiffness`, `cartesian_damping`,
`joint_stiffness`, `joint_damping`, `torque_limits`, `posture`, `torque_cutoff`,
`velocity_feedforward`, `leash`, `project_joint_gains`, `controller_mode`, `limit_rate`,
`realtime_priority`), and
`franka.Robot(address, realtime='ignore')` overrides `FRANKA_REALTIME`.

## Compliance

The loop tracks the targets with the crate's own impedance torques by default
(`backend='impedance'`), so the arm is a spring around the target: push it and it gives way,
let go and it returns. `cartesian_stiffness` is that spring, 6 values (x, y, z in N/m, then
three rotational in Nm/rad) or one float for the three translational entries with the
rotational defaults kept; the defaults are 750 N/m and 15 Nm/rad.

```python
with robot.cartesian_targets(cartesian_stiffness=400) as arm:   # softer than the default 750 N/m
    arm.move_by([0.03, 0.0, 0.0])
```

`cartesian_damping` (same shape), `joint_stiffness` and `joint_damping` (7 each, the joint
term), `torque_limits` (7, Nm), `posture` (7 rad, the configuration the inverse kinematics
prefers; default the start) and `torque_cutoff` (Hz, default 100) are the rest of the Rust
`ImpedanceOptions`; `None` keeps each default, a wrong length is a `ValueError`. Three more:

- `velocity_feedforward=True`: the damping acts on the velocity error, not the velocity;
  `False` is DROID's form, and with `cartesian_damping=[37, 37, 37, 2, 2, 2]` its law.
- `leash=(metres, radians)`, default `(0.025, 0.15)`: how far the target may run ahead of an
  arm that is held back, so the spring never pulls harder than the felt stiffness times the
  leash (roughly 25 to 30 N at the defaults at the ready pose; target control sets no
  collision thresholds, so set at least 40 N / 40 Nm with the default gains, see
  [Collision thresholds](../howto/target-control.md#backends)).
- `project_joint_gains=False`: `True` confines the joint term to the nullspace, so the end
  effector feels `cartesian_stiffness` alone.

`joint_targets` takes `backend`, `joint_stiffness`, `joint_damping`, `torque_limits`,
`torque_cutoff`, `velocity_feedforward`, `leash` (one float, rad per joint, default 0.1) and
`project_joint_gains`. `backend='robot'` has the robot's own controller (`controller_mode`) track
the targets instead, as the bindings did before the impedance backend existed; it takes none
of the gains. The law, the defaults and what differs between the backends are in
[Command from a low-rate program](../howto/target-control.md#backends), with what two real
FERs measured on 2026-09-10.

| call | what it does |
|---|---|
| `arm.move_to(t)` | Absolute target in the base frame: 3 elements `(x, y, z)` in m keep the target orientation, 7 add a unit quaternion `(x, y, z, w)`. Cancels a `follow`; never blocks. |
| `arm.move_by(d)` | Moves the *target* (not the measured pose) by `(dx, dy, dz)` m; 6 elements append a rotation vector (axis times angle, rad) composed onto the target orientation in the base frame. Cancels a `follow`. |
| `arm.follow(chunk, dt=0.02)` | Hands an `(N, 3)` or `(N, 7)` array of absolute targets to a Rust timer thread: row 0 now, row `i` at `i * dt` s. Returns at once; a new `follow` or `move_*` replaces it. |
| `arm.target()` | The current target, shape `(7,)`: position (m) and unit quaternion `(x, y, z, w)`. |
| `arm.target_pose()` | The same as a `(4, 4)` matrix, the convention of `RobotState.O_T_EE`. |
| `arm.state()` | The latest `franka.RobotState` the loop received; never blocks on the network. |
| `arm.running` | `False` once the loop ended for any reason or `stop()` was called. |
| `robot.joint_targets(fraction=0.2)` | The same over the seven joints (rad): `move_to`, `move_by` and the rows of `follow` take 7 values, `target()` is `(7,)`, and the budget is `fraction` of the robot's joint velocity, acceleration and jerk limits. |

`franka.rotated(q, r)` composes a rotation vector onto a quaternion the way `move_by` does,
for building `move_to` targets. `robot.read_once()`, `robot.set_collision_behavior(...)`
(the Rust arguments), `robot.automatic_error_recovery()`, `robot.stop()`,
`robot.fci_version`, `robot.server_version` and `robot.gripper()` (`homing`, `move`,
`grasp`, `stop`, `read_once`, GIL released while they wait) complete the surface.

## `RobotState`

Arrays are float64 numpy arrays, poses `(4, 4)` matrices with the translation in
`M[:3, 3]`; `robot_mode` is a string such as `'idle'`, `current_errors` and
`last_motion_errors` are lists of flag names. `state.flat()` is one array of length
`franka.FLAT_LEN` (69) for a policy's observation; the slices are `franka.FLAT_LAYOUT`:

| slice | field |
|---|---|
| `0:7` | `q` (rad) |
| `7:14` | `dq` (rad/s) |
| `14:21` | `tau_J` (Nm) |
| `21:28` | `tau_ext_hat_filtered` (Nm) |
| `28:31` | `O_T_EE` translation (m) |
| `31:35` | `O_T_EE` unit quaternion `(x, y, z, w)`, `w >= 0` |
| `35:41` | `O_F_ext_hat_K` (N, Nm) |
| `41:48` | `q_d` (rad) |
| `48:51` | `O_T_EE_c` translation (m) |
| `51:55` | `O_T_EE_c` quaternion `(x, y, z, w)` |
| `55:62` | `joint_contact` |
| `62:68` | `cartesian_contact` |
| `68` | `time` (s) |

## `Robot.model()`

`franka.Model` is the crate's model over numpy, native on both robots (an FR3 sends its URDF,
an FER's parameters are built in): `pose(frame, q)` is `(4, 4)`, `body_jacobian` and
`zero_jacobian` are `(6, 7)` with the linear rows first, `mass(q)` `(7, 7)`,
`coriolis(q, dq)` and `gravity(q)` `(7,)`, `link_poses(q)` `(8, 4, 4)` and `hand_pose(q)`
`(4, 4)`. `frame` is `'joint1'..'joint7'`, `'flange'`, `'ee'` or `'stiffness'`; the
optional `F_T_EE`, `EE_T_K`, `I_total`, `m_total` and `F_x_Ctotal` arguments default to the
identity and no payload, so pass the state's for the mounted tool. See
[Use the model](../howto/model.md).

## Errors

Every Rust error is a `franka.FrankaError`; `str(e)` is the Rust message and `e.kind` names
the variant (`'network'`, `'command'`, `'realtime'`, ...). A motion the robot aborted is a
`franka.ControlException`, a subclass, with `e.reason` (the reflex reason), `e.errors` (its
flag names) and `e.move_status`. See [State and errors](../concepts/state-and-errors.md).

## The notebook and the examples

`crates/franka-py/examples/quickstart.ipynb` connects, reads a state and the model, drives a
5 cm square with `move_to` at 10 Hz, yaws the tool 15° with a quaternion target and tilts it
10° with a `move_by` rotation vector, returns to the start with a `follow` chunk, then
replays the motion inline in Rerun (Franka's meshes if `FRANKA_MESHES` points at them,
otherwise a skeleton) and plots target against measured position. It runs top to bottom
against [franka-sim](./simulator.md): `FRANKA_ADDRESS` (default `127.0.0.1`) picks the
robot, `FRANKA_REALTIME` defaults to `ignore`, and the last cells need
`"rerun-sdk[notebook]==0.37.*"` and `matplotlib`.

`examples/policy_loop.py <hostname> [--yes]` is a jittery 6-10 Hz policy loop: a 4 cm circle
through `move_to`, yaw and tilt through `move_by`, a 20-row `follow` chunk back to the start.
It drove a real FER on 2026-09-09, with the robot's controller tracking. `examples/rotate.py
<hostname>` is the rotation alone: a
20° yaw as a quaternion target, a 10° tilt as a rotation vector, back to the start, printing
the measured angle after each.

## Details

`franka.Robot(...)` raises the thread that calls it to the highest `SCHED_FIFO` priority
when `RLIMIT_RTPRIO` allows, in both realtime modes, as libfranka's constructor does. In a
notebook that thread is the kernel's main thread, so a long-running cell then competes with
the control thread at realtime priority. The control thread of `cartesian_targets` /
`joint_targets` is raised separately (`realtime_priority`).

To build from the source tree into a virtualenv, `pip install maturin` and
`maturin develop --release -m crates/franka-py/Cargo.toml` (prefix with `env -u CONDA_PREFIX`
under an active conda environment). `crates/franka-py` is `publish = false`; the wheel on
PyPI is the artefact. Its tests run against the simulator; see
[Test against the simulator](../howto/simulator-tests.md).
