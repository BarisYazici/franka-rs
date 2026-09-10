# Drive the loop yourself

At the end of this page you will have run a motion from a loop you wrote, reading a state
with `read_once` and answering it with `write_once`, and seen a Cartesian impedance
controller written that way. This is libfranka's `ActiveControl`: `startTorqueControl()`,
`readOnce()`, `writeOnce()`.

Prerequisites: [First program](../getting-started/first-program.md) works, the arm is at the
ready pose, and you have read [The realtime rules](../concepts/realtime-rules.md): the loop
body is on the realtime thread, and there is no filter or rate limiter behind it.

```rust,no_run
# extern crate franka;
use franka::{JointVelocities, MoveControllerMode, RealtimeConfig, Robot};

# fn main() -> franka::FrankaResult<()> {
let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
let mut active = robot.start_joint_velocity_control(MoveControllerMode::JointImpedance)?;

let mut time = 0.0;
loop {
    let (_state, period) = active.read_once()?;
    time += period.as_secs_f64();
    let mut out = JointVelocities::new([0.0; 7]);
    out.motion_finished = time >= 4.0;
    let finished = out.motion_finished;
    active.write_once(&out, None)?;
    if finished {
        break;
    }
}
# Ok(()) }
```

## The starters

`Robot::start_*_control` sends the `Move` and returns a handle that holds the robot's control
lock for its lifetime:

| starter | handle | `write_once` takes |
|---|---|---|
| `start_torque_control` | `ActiveTorqueControl` | `&Torques` |
| `start_joint_position_control` | `ActiveMotionGenerator<JointPositions>` | motion + `Option<&Torques>` |
| `start_joint_velocity_control` | `ActiveMotionGenerator<JointVelocities>` | motion + `Option<&Torques>` |
| `start_cartesian_pose_control` | `ActiveMotionGenerator<CartesianPose>` | motion + `Option<&Torques>` |
| `start_cartesian_velocity_control` | `ActiveMotionGenerator<CartesianVelocities>` | motion + `Option<&Torques>` |

The motion-generator starters take a `MoveControllerMode`: `JointImpedance`,
`CartesianImpedance` or `ExternalController`. The `Option<&Torques>` of `write_once` must be
`Some` if and only if the mode is `ExternalController`; otherwise `write_once` fails with
`FrankaError::Control`.

`read_once` blocks for the next state and returns it with the robot time elapsed since the
previous `read_once` (zero on the first call); it fails with `FrankaError::Control` if the
motion was aborted. `write_once` validates the command (finite values, a homogeneous
transform, a valid elbow) and sends it. Setting `motion_finished` on either input ends the
control process; a `write_once` after that fails with `FrankaError::Control`. Dropping a
handle before the motion has finished cancels the motion.

Two things this path does not do. **No rate limiting and no low-pass filtering** are applied,
in this crate as in libfranka: smooth setpoints are your job, and the robot refuses a
discontinuous one with a reflex. And the handle is **`!Send`**: it holds the control lock's
guard, so read and write it from the thread that started it; `robot.stop()` from another
thread still works through the `Arc<Robot>`.

## Cartesian impedance with `ActiveControl`

`examples/cartesian_impedance_active_control.rs` is libfranka's
`cartesian_impedance_control.cpp` on this path: a spring-damper system without inertia
shaping whose equilibrium is the pose the end effector had when the loop started. The
callback becomes the loop body and the Eigen calls become nalgebra ones (the crate depends on
nalgebra already):

```rust,no_run
# extern crate franka;
# extern crate nalgebra;
# use franka::{motion_finished, Frame, RealtimeConfig, Robot, Torques};
# use nalgebra::{Matrix4, Rotation3, SMatrix, SVector, UnitQuaternion, Vector3};
# fn main() -> franka::FrankaResult<()> {
# let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
# let model = robot.load_model()?;
# let (stiffness, damping) = (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
# let (position_d, orientation_d) = (Vector3::zeros(), UnitQuaternion::identity());
# let mut error = SVector::<f64, 6>::zeros();
let mut control = robot.start_torque_control()?;
loop {
    let (state, _period) = control.read_once()?;
    let coriolis = SVector::<f64, 7>::from_column_slice(&model.coriolis(&state));
    let jacobian =
        SMatrix::<f64, 6, 7>::from_column_slice(&model.zero_jacobian(Frame::EndEffector, &state));
    let dq = SVector::<f64, 7>::from_column_slice(&state.dq);

    let transform = Matrix4::from_column_slice(&state.O_T_EE);
    let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
    let mut orientation =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
    if orientation_d.coords.dot(&orientation.coords) < 0.0 {
        // A unit quaternion and its negation are the same rotation: take the shorter way.
        orientation = UnitQuaternion::new_unchecked(-orientation.into_inner());
    }
    let position_error = transform.fixed_view::<3, 1>(0, 3) - position_d;
    let orientation_error = -(rotation * (orientation.inverse() * orientation_d).vector());
    error.fixed_view_mut::<3, 1>(0, 0).copy_from(&position_error);
    error.fixed_view_mut::<3, 1>(3, 0).copy_from(&orientation_error);

    let tau = jacobian.transpose() * (-(stiffness * error) - damping * (jacobian * dq)) + coriolis;
    control.write_once(&Torques::new(tau.into()))?;
#   break;
}
# control.write_once(&motion_finished(Torques::new([0.0; 7])))?;
# Ok(()) }
```

Nothing in the body allocates: the nalgebra types are fixed-size and live on the stack, and
the model methods return fixed arrays. The example adds the argument parsing, the compliance
constants (150 N/m, 10 Nm/rad, `D = 2 sqrt(K)`), a `SIGINT` handler that lets the loop leave
through a final `motion_finished` write, and the `--duration` bound. It sets the C++
example's collision thresholds, 100 Nm and 100 N, so that the arm can be pushed; keep the
user stop in hand.

`examples/cartesian_impedance_figure_eight.rs` is the same law with a moving equilibrium: a
Lissajous figure eight in the base frame's y-z plane (0.08 m amplitude, a 10 s lap by
default), stiffness 200 N/m and 15 Nm/rad. A raised cosine ramps the amplitude and the
stiffness in over 3 s and out over 2 s (1 s after Ctrl-C, or after the end effector is
dragged more than 30 cm off its setpoint), so nothing steps at either end. It adds a
nullspace joint spring (5 Nm/rad, damped-inverse projector) that keeps the elbow near its
start configuration, a one-sided virtual floor 10 cm below the start (a 1500 N/m spring
along +z, `--floor`), and a ±25 Nm clamp on each joint torque before the Coriolis
feedforward is added. The loop stays allocation-free; the tracking error goes into a
fixed-size histogram. It is the example to run with someone standing next to the robot.

## On the FER

`ActiveControl` works on FCI v5 as well; libfranka 0.9.2 has no equivalent, its API for that
robot generation is the callback only. The public API is identical on both versions;
underneath, `start_torque_control()` on an FER runs a joint-velocity generator commanding
zeros alongside the external controller, because FCI v5 has no torque-only mode (see
[FER / Panda specifics](../reference/fer.md)). Measured on a real FER, `ActiveControl` is
equivalent to the callback API within noise: interval p50 999.2 against 999.1 µs over 10 s
runs, comparable p99, maximum and CPU; see
[Benchmarks and hardware validation](../reference/benchmarks.md). Both impedance examples ran
on a real FER through this path on 2026-09-07 with no reflex.
