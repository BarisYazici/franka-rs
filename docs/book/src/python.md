# Python

`crates/franka-py` is `import franka`: the target control loops driven from Python at any
rate, with the 1 kHz loop on a Rust thread that never takes the GIL. Until the 0.2.0
release there is no wheel on PyPI (it will be `franka-rs`); build from the source tree
into a virtualenv with [maturin](https://www.maturin.rs), prefixed with
`env -u CONDA_PREFIX` under an active conda env:

```sh
pip install maturin
maturin develop --release -m crates/franka-py/Cargo.toml
```

```python
import franka

robot = franka.Robot("127.0.0.1")   # RealtimeConfig from FRANKA_REALTIME, default enforce

with robot.cartesian_targets(max_velocity=0.3, max_acceleration=0.5, max_jerk=20.0) as arm:
    while not done:
        obs = arm.state()          # franka.RobotState: numpy q, dq, tau_J, O_T_EE (4x4), ...
        action = policy(obs)
        arm.move_by([dx, dy, dz])  # relative to the current *target*, metres
        arm.move_to([x, y, z])     # absolute, base frame; 7 elements set the orientation too
        arm.follow(chunk, dt=0.02) # (N, 3) or (N, 7) absolute targets, a row every dt s from Rust
        arm.target()               # the current target: position + quaternion (x, y, z, w)
        arm.running                # bool
# __exit__ stops the loop; an exception inside the block still stops it and is re-raised
```

`robot.joint_targets(fraction=0.2)` has the same shape over the seven joints (rad).
`robot.read_once()`, `robot.set_collision_behavior(...)` (the Rust arguments),
`robot.automatic_error_recovery()`, `robot.fci_version`, `robot.server_version` and
`robot.gripper()` (`homing`, `move`, `grasp`, `stop`, `read_once`, GIL released while they
wait) complete the surface. Every Rust `FrankaError` is a `franka.FrankaError`; a
`ControlException` is `franka.ControlException(FrankaError)` with the reflex `reason`.
A Cartesian target is `(x, y, z)` in m; with 7 elements, `move_to` and the rows of
`follow` also set the orientation, a unit quaternion `(x, y, z, w)` (normalised, the zero
quaternion is a `ValueError`). A `move_by` of 6 elements appends a rotation vector (axis
times angle, rad) composed onto the target orientation in the base frame, `exp(r) * q`.
`arm.target()` is the 7-vector and `arm.target_pose()` the 4x4 matrix;
`franka.rotated(q, r)` applies the same composition in numpy, for building `move_to` targets.
`crates/franka-py/examples/policy_loop.py` is a jittery 6-10 Hz policy loop -- a circle
through `move_to`, yaw and tilt increments through `move_by`, a `follow` chunk back to the
start -- as run on a Panda: `python crates/franka-py/examples/policy_loop.py <hostname>`.
`examples/rotate.py` is the rotation alone: a yaw as a quaternion target, a tilt as a
rotation vector, and the measured angle. `examples/quickstart.ipynb` is the notebook
walk-through: connect, a square, a rotation, a `follow` chunk, then the motion replayed
inline with Franka's meshes (or a skeleton) through `rerun-sdk[notebook]` and a
target-versus-measured plot; it runs top to bottom against franka-sim.

## `Robot.model()`

`franka.Model` is the crate's `Model` (native on both an FR3, whose URDF the robot sends,
and an FER, whose model is built in) over numpy: `pose(frame, q, F_T_EE=None, EE_T_K=None)`
is `(4, 4)`, `body_jacobian` and `zero_jacobian` (same arguments) `(6, 7)` with the linear
rows first, `mass(q, I_total=None, m_total=0.0, F_x_Ctotal=None)` `(7, 7)`,
`coriolis(q, dq, ...)` and `gravity(q, m_total=0.0, F_x_Ctotal=None, gravity_earth=None)`
`(7,)`. `frame` is `'joint1'..'joint7'`, `'flange'`, `'ee'` or `'stiffness'`; the
transforms default to the identity and the payload to nothing attached, so pass the
state's `F_T_EE`, `EE_T_K`, `I_total`, `m_total`, `F_x_Ctotal` for the mounted tool.
`link_poses(q)` is `(8, 4, 4)`, the frames franka_description hangs `link0..link7` on
(the base and `joint1..joint7`), and `hand_pose(q)` the hand's, the flange yawed by
-45 deg; `franka-rerun` places the meshes the same way.

## `RobotState.flat()`

One float64 array of length `franka.FLAT_LEN` (69); the slices are `franka.FLAT_LAYOUT`:

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

The tests (`crates/franka-py/tests/test_sim.py`) run against franka-sim with
`FRANKA_REALTIME=ignore`, locally under `flock .sim.lock` and in CI's `python-bindings` job;
`test_notebook.py` executes the quickstart notebook the same way when `nbclient`,
`ipykernel`, `matplotlib` and `rerun-sdk[notebook]` are installed, and is not part of CI.
