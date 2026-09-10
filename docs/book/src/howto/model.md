# Use the model

At the end of this page you have a `Model` for the connected arm and can evaluate poses,
Jacobians, the mass matrix, Coriolis and gravity torques from a `RobotState` or from a
joint configuration of your own, with no download and no C++ library.

Prerequisites: a connected `Robot` ([First program](../getting-started/first-program.md)).

```rust,no_run
# extern crate franka;
use franka::{Frame, RealtimeConfig, Robot};

# fn main() -> franka::FrankaResult<()> {
let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
let model = robot.load_model()?;
let state = robot.read_once()?;

let pose: [f64; 16] = model.pose(Frame::EndEffector, &state);       // column major
let jacobian: [f64; 42] = model.zero_jacobian(Frame::EndEffector, &state);
let body_j: [f64; 42] = model.body_jacobian(Frame::EndEffector, &state);
let mass: [f64; 49] = model.mass(&state);
let coriolis: [f64; 7] = model.coriolis(&state);
let gravity: [f64; 7] = model.gravity(&state);
# Ok(()) }
```

## The six calls

| call | returns | layout |
|---|---|---|
| `pose(frame, &state)` | `[f64; 16]` | 4x4 pose of `frame` in the base frame, column-major |
| `zero_jacobian(frame, &state)` | `[f64; 42]` | 6x7, column-major; rows `vx vy vz wx wy wz`, base-frame aligned |
| `body_jacobian(frame, &state)` | `[f64; 42]` | 6x7, column-major, expressed in `frame` |
| `mass(&state)` | `[f64; 49]` | 7x7 mass matrix, column-major, symmetric, kg m² |
| `coriolis(&state)` | `[f64; 7]` | `C(q, dq) dq`, Nm |
| `gravity(&state)` | `[f64; 7]` | gravity torques, Nm |

`Frame` has the ten libfranka values: `Joint1` … `Joint7`, `Flange` (the URDF's `link8`),
`EndEffector` (flange post-multiplied by `F_T_EE`) and `Stiffness` (end effector
post-multiplied by `EE_T_K`). `Frame::ALL` lists them in that order.

## Supplying your own inputs: the `*_q` variants

Every call has a variant that takes the joint configuration and load parameters directly
instead of reading them out of a `RobotState`: `pose_q`, `zero_jacobian_q`,
`body_jacobian_q`, `mass_q`, `coriolis_q`, `gravity_q`. They are what you use offline, and
`gravity_q` is the only way to supply your own gravity vector:

```rust,no_run
# extern crate franka;
# use franka::Model;
# fn f(model: &Model, q: &[f64; 7]) {
let g = model.gravity_q(q, 0.73, &[0.0, 0.0, 0.1034], &[0.0, 0.0, -9.81]);
# let _ = g; }
```

`gravity(&state)` uses `state.O_ddP_O` as Earth's gravity, as libfranka does; `coriolis(&state)`
uses the fixed `[0, 0, -9.81]` (`franka::model::DEFAULT_GRAVITY_EARTH`), also as libfranka
does. franka-sim publishes `O_ddP_O` as `[0, 0, 0]`, so on the simulator `gravity(&state)`
is identically zero: use `gravity_q` with an explicit vector there (see [Simulator
gaps](../reference/simulator-gaps.md)).

A `Model` can also be built without a robot: `Model::from_urdf(&urdf)` for an FR3's URDF
and `Model::native_fer()` for the Franka Emika Robot.

## In a torque controller

`Model` is `Send + Sync`, so a control thread and a planner can share one. The robot adds
gravity and friction to the torques you send, so a model-based controller typically adds
the Coriolis term and uses a Jacobian for a Cartesian law:

```rust,no_run
# extern crate franka;
# use franka::{Model, RobotState, Torques};
# fn f(model: &Model, state: &RobotState, tau_task: [f64; 7]) -> Torques {
let coriolis = model.coriolis(state);
let mut tau = tau_task;
for i in 0..7 {
    tau[i] += coriolis[i];
}
Torques::new(tau)
# }
```

The full Cartesian impedance law (`J^T (-K e - D J dq) + coriolis`) is on [Drive the loop
yourself](./active-control.md); the joint impedance variant for an FER is
`examples/fer_joint_impedance.rs`.

Cost: evaluating all five dynamic and kinematic calls a model-based controller makes takes
about 3 µs offline and 11–15 µs inside a 1 kHz loop on a laptop-class CPU. The in-loop
figure is higher because a duty-cycled loop starts each cycle on a core that has just
idled; see [Benchmarks](../reference/benchmarks.md) for the numbers and that caveat.

## Where the parameters come from

On an FR3 (FCI v10), `load_model()` fetches the arm's URDF from the robot with
`GetRobotModel` and evaluates it natively; `robot.robot_model()` returns the URDF text. On
a Franka Emika Robot (FCI v5) there is no such command, and `load_model()` returns the
crate's built-in FER model (`Model::native_fer()`, the parameters in
`franka::model::FER_URDF`, identified from a real FER's own model library). On neither
robot does `load_model()` download or `dlopen` anything, and on an FER it cannot fail.
How the FER parameters were fitted, and how closely both models agree with libfranka's, is
on [Model parameters and conformance](../reference/model.md).

## `load_model_from_robot()`: the robot's own library

libfranka 0.9.2 gets an FER's model by downloading the robot's closed-source
`libfcimodels.so` over the command channel and `dlopen`ing it. That path is still
available, opt-in:

```rust,no_run
# extern crate franka;
# use franka::Robot;
# fn f(robot: &Robot) -> franka::FrankaResult<()> {
let model = robot.load_model_from_robot()?;   // LoadModelLibrary + dlopen, as libfranka does
# let _ = model; Ok(()) }
```

It needs the default `model-library` cargo feature (which links `libloading`), an x86-64
Linux host, and it executes code the robot served. On an FR3 it is `load_model()` exactly.
Its caveats are listed on the [reference page](../reference/model.md).

## From Python

`robot.model()` returns the same model with the same six calls; see
[From Python](../getting-started/python.md).
