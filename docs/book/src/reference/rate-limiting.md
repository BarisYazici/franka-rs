# Rate limiting and filtering

Every `Robot::control_*` method takes `limit_rate: bool` and `cutoff_frequency: f64`. This
page is what those two arguments do, which constants they use on each robot, and where the
same functions are available for a controller that limits by hand. The implementation is a
port of libfranka's `rate_limiting.cpp`, `joint_velocity_limits.cpp` and `lowpass_filter.cpp`
(0.21.2 for the FR3, 0.9.2 for the FER's constants), with libfranka's unit tests ported
alongside.

## On the callback path

Each cycle, the command your callback returned is converted before it is sent:

1. **Low-pass filter.** If `cutoff_frequency < MAX_CUTOFF_FREQUENCY` (1000 Hz), every
   commanded value is filtered first order against the robot's echo of the last command
   (`q_d`, `dq_d`, `O_T_EE_c` with the rotation interpolated by slerp, `O_dP_EE_c`,
   `elbow_c`; `tau_J_d` for torques) with gain `dt / (dt + 1 / (2π f_c))`.
   `DEFAULT_CUTOFF_FREQUENCY` is 100 Hz; pass `MAX_CUTOFF_FREQUENCY` to skip the filter.
2. **Rate limiter.** If `limit_rate`, the filtered command is clamped so that its velocity,
   acceleration and jerk relative to that same echo (`q_d`, `dq_d`, `ddq_d`; `O_T_EE_c`,
   `O_dP_EE_c`, `O_ddP_EE_c`; the elbow's) stay within the version's constants, and a torque
   so that its rate against `tau_J_d` stays within `MAX_TORQUE_RATE`.
3. **Validation.** A non-finite value is `FrankaError::InvalidArgument` with libfranka's text.

The first command of a motion is its own reference on an FR3 (libfranka 0.21.2's
`initialized_filter_`), and is limited against the echo like every other on an FER; that is
the [`q_d` seed rule](./fer.md).

libfranka's defaults differ between the versions, and since Rust has no default arguments you
choose explicitly:

| | `limit_rate` default in libfranka | note |
|---|---|---|
| FCI v10 (libfranka 0.21) | `false` | The FR3 does its own limiting; the client-side limiter can distort a motion. |
| FCI v5 (libfranka 0.9.2) | `true` | The FER-era default. |

`cutoff_frequency` defaults to `DEFAULT_CUTOFF_FREQUENCY` in both.

Neither the filter nor the limiter runs on the `ActiveControl` path (`read_once` /
`write_once`), here or in libfranka: there smooth setpoints are the caller's job.
[Target control](../howto/target-control.md) runs its loop with `limit_rate` on (default) and
the filter off, and additionally calls `limit_rate_cartesian_pose` /
`limit_rate_joint_positions` under its own, smaller budget as a backstop that is not meant to
bind; see [Online trajectory generation](./otg.md).

## The constants

Which table the loop uses is decided by the negotiated version, not by you. The FR3's live at
the crate root (`franka::MAX_JOINT_JERK` and neighbours, mirroring libfranka's `franka::`
namespace); the FER's only at `franka::rate_limiting::fer::*`, so the two envelopes cannot be
confused at a glance. Nominal values, from `crates/franka-rs/src/rate_limiting/`:

| constant | FR3, `franka::*` | FER, `rate_limiting::fer::*` |
|---|---|---|
| `TOL_NUMBER_PACKETS_LOST` | 0.0 | 3.0 |
| `MAX_TORQUE_RATE` (Nm/s, per joint) | 1000 | 1000 |
| `MAX_JOINT_JERK` (rad/s³) | 5000, all joints | 7500, 3750, 5000, 6250, 7500, 10000, 10000 |
| `MAX_JOINT_ACCELERATION` (rad/s²) | 10, all joints | 15, 7.5, 10, 12.5, 15, 20, 20 |
| joint velocity (rad/s) | position-dependent envelope (below), saturating at 2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26 | `MAX_JOINT_VELOCITY`, flat: 2.175 (J1–4), 2.61 (J5–7); `MIN_JOINT_VELOCITY` is its negation |
| `MAX_TRANSLATIONAL_VELOCITY` (m/s) | 3.0 | 2.0 |
| `MAX_TRANSLATIONAL_ACCELERATION` (m/s²) | 9.0 | 13.0 |
| `MAX_TRANSLATIONAL_JERK` (m/s³) | 4500 | 6500 |
| `MAX_ROTATIONAL_VELOCITY` (rad/s) | 2.5 | 2.5 |
| `MAX_ROTATIONAL_ACCELERATION` (rad/s²) | 17 | 25 |
| `MAX_ROTATIONAL_JERK` (rad/s³) | 8500 | 12500 |
| `MAX_ELBOW_VELOCITY` (rad/s) | 1.5 | 2.175 |
| `MAX_ELBOW_ACCELERATION` (rad/s²) | 10 | 10 |
| `MAX_ELBOW_JERK` (rad/s³) | 5000 | 5000 |

Every constant is the nominal value less `LIMIT_EPS` (1e-3), and every velocity limit is
further reduced by `TOL_NUMBER_PACKETS_LOST · DELTA_T · a_max`, the velocity the axis could
pick up over that many lost 1 ms cycles at its maximum acceleration: nothing on the FR3, and
on the FER 0.039 m/s off the translational velocity and 0.045 rad/s off joint 1's, for
instance. `DELTA_T` is 1e-3, `NORM_EPS` is `f64::EPSILON`, and
`FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE` (0.99) multiplies the three rotational limits
inside `limit_rate_cartesian_pose`, on both versions.

### The FR3's joint velocity envelope

On an FR3 the joint velocity limit depends on the joint position: it shrinks towards a joint
limit so that the joint can still decelerate before reaching it,

```text
upper(q) = min(v_max, max(0, -v_offset + sqrt(max(0, 2 a_dec (q_upper - q))))) - tolerance
lower(q) = max(-v_max, min(0,  v_offset - sqrt(max(0, 2 a_dec (q - q_lower))))) + tolerance
```

with the parameters read from the `<position_based_velocity_limits>` element of the URDF the
robot serves (`joint_velocity_limits` module: `JointVelocityLimitsConfig::from_urdf`,
`upper_limits(&q)`, `lower_limits(&q)`). `Robot::upper_joint_velocity_limits(&q)` and
`lower_joint_velocity_limits(&q)` return the envelope of the connected robot; on an FER they
return the flat `fer::MAX_JOINT_VELOCITY` / `MIN_JOINT_VELOCITY` and ignore `q`.
`franka::compute_upper_limits_joint_velocity` / `compute_lower_limits_joint_velocity` are the
deprecated libfranka versions with the FR3 parameters hardcoded.

### The torque-rate margin on FCI v10

The robot judges the rate of a torque command against the `f64` it last received, but
publishes `tau_J_d` as an `f32` on FCI v10. At |τ| ≥ 64 Nm one `f32` ULP is 2⁻¹⁷ Nm, and
the half-ULP the echo can be off by is worth up to about 3.8e-3 Nm/s over a 1 ms cycle, more
than the 1e-3 Nm/s `MAX_TORQUE_RATE` keeps below the nominal 1000. A saturated command
limited against the quantised echo could therefore read as up to about 1000.0028 Nm/s at
the robot. The control loop shrinks each joint's maximum torque rate per cycle by
`f32::EPSILON · |tau_J_d| / 2 / DELTA_T` on v10 only; `limit_rate_torques` itself stays an
exact port of `franka::limitRate`, and on v5, where `tau_J_d` is an `f64`, the limiting is
exact and the margin is off. The margin is zero at `tau_J_d == 0`.

## Limiting by hand

The functions are public, at the crate root and under `franka::rate_limiting`, and take
their bounds as arguments, so they serve either robot's constants:

| Rust | libfranka overload |
|---|---|
| `limit_rate_torques(max_derivatives, commanded, last)` | `limitRate(max_derivatives, ...)` |
| `limit_rate_joint_velocity(...)`, `limit_rate_joint_position(...)` | the scalar overloads |
| `limit_rate_joint_velocities(...)`, `limit_rate_joint_positions(...)` | the `array<double, 7>` overloads |
| `limit_rate_cartesian_velocity(...)` | the `O_dP_EE_c` overload |
| `limit_rate_cartesian_pose(...)` | the `O_T_EE_c` overload |
| `low_pass_filter(dt, y, y_last, f_c)`, `cartesian_low_pass_filter(dt, y, y_last, f_c)` | `lowpassFilter`, `cartesianLowpassFilter` |

Where the C++ throws `std::invalid_argument`, these return `FrankaError::InvalidArgument`
with the same message. `examples/fer_joint_impedance.rs` is the pattern: `control_torques`
with `limit_rate = false` and `MAX_CUTOFF_FREQUENCY`, and
`limit_rate_torques(&rate_limiting::fer::MAX_TORQUE_RATE, &tau, &state.tau_J_d)` inside the
callback.
