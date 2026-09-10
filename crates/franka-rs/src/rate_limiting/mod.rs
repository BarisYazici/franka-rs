//! Rate limiting for torques, joint positions/velocities and Cartesian poses/velocities.
//!
//! Port of libfranka 0.21.2 `include/franka/rate_limiting.h` and `src/rate_limiting.cpp`.
//! The C++ overload set `franka::limitRate` is spelled out here as separate functions, since
//! Rust has no overloading:
//!
//! | C++ | Rust |
//! |---|---|
//! | `limitRate(max_derivatives, commanded, last)` | [`limit_rate_torques`] |
//! | `limitRate(upper, lower, acc, jerk, v, v_last, a_last)` | [`limit_rate_joint_velocity`] |
//! | `limitRate(upper, lower, acc, jerk, q, q_last, v_last, a_last)` | [`limit_rate_joint_position`] |
//! | the `array<double, 7>` variants of the two above | [`limit_rate_joint_velocities`], [`limit_rate_joint_positions`] |
//! | `limitRate(..., O_dP_EE_c, ...)` | [`limit_rate_cartesian_velocity`] |
//! | `limitRate(..., O_T_EE_c, ...)` | [`limit_rate_cartesian_pose`] |
//!
//! Where the C++ throws `std::invalid_argument`, these functions return
//! [`crate::error::FrankaError::InvalidArgument`] with the same message text.
//!
//! The implementation is split by the quantity being limited (`torque`, `joint`, `cartesian`),
//! but those modules are private: every function is re-exported here, so the public paths are
//! `franka::rate_limiting::limit_rate_*` and nothing else.

mod cartesian;
pub mod fer;
mod joint;
mod torque;

pub(crate) use cartesian::scaled_axis;
pub use cartesian::{limit_rate_cartesian_pose, limit_rate_cartesian_velocity};
pub use joint::{
    compute_lower_limits_joint_velocity, compute_upper_limits_joint_velocity,
    limit_rate_joint_position, limit_rate_joint_positions, limit_rate_joint_velocities,
    limit_rate_joint_velocity,
};
pub use torque::limit_rate_torques;

/// Sample time constant. Port of `franka::kDeltaT`.
pub const DELTA_T: f64 = 1e-3;
/// Epsilon value for checking limits. Port of `franka::kLimitEps`.
pub const LIMIT_EPS: f64 = 1e-3;
/// Epsilon value for limiting Cartesian accelerations/jerks or not. Port of `franka::kNormEps`.
pub const NORM_EPS: f64 = f64::EPSILON;
/// Number of packets lost considered for the definition of velocity limits.
///
/// When a packet is lost, FCI assumes a constant acceleration model. For FR3 there are no
/// expected packet losses, therefore this number is 0. Port of `franka::kTolNumberPacketsLost`.
pub const TOL_NUMBER_PACKETS_LOST: f64 = 0.0;
/// Factor for the definition of rotational limits using the Cartesian pose interface.
///
/// Port of `franka::kFactorCartesianRotationPoseInterface`.
pub const FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE: f64 = 0.99;

/// Maximum torque rate. Port of `franka::kMaxTorqueRate`.
///
/// The `LIMIT_EPS` margin below the nominal 1000 Nm/s is *not* enough to absorb the `float32`
/// quantisation of `tau_J_d` at |τ| ≥ 64 Nm; see [`limit_rate_torques`].
pub const MAX_TORQUE_RATE: [f64; 7] = [1000.0 - LIMIT_EPS; 7];
/// Maximum joint jerk. Port of `franka::kMaxJointJerk`.
pub const MAX_JOINT_JERK: [f64; 7] = [5000.0 - LIMIT_EPS; 7];
/// Maximum joint acceleration. Port of `franka::kMaxJointAcceleration`.
pub const MAX_JOINT_ACCELERATION: [f64; 7] = [10.0 - LIMIT_EPS; 7];
/// Tolerance value for joint velocity limits to deal with numerical errors and data losses.
///
/// Port of `franka::kJointVelocityLimitsTolerance`.
pub const JOINT_VELOCITY_LIMITS_TOLERANCE: [f64; 7] = [
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[0],
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[1],
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[2],
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[3],
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[4],
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[5],
    LIMIT_EPS + TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[6],
];
/// Maximum translational jerk. Port of `franka::kMaxTranslationalJerk`.
pub const MAX_TRANSLATIONAL_JERK: f64 = 4500.0 - LIMIT_EPS;
/// Maximum translational acceleration. Port of `franka::kMaxTranslationalAcceleration`.
pub const MAX_TRANSLATIONAL_ACCELERATION: f64 = 9.0 - LIMIT_EPS;
/// Maximum translational velocity. Port of `franka::kMaxTranslationalVelocity`.
pub const MAX_TRANSLATIONAL_VELOCITY: f64 =
    3.0 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_TRANSLATIONAL_ACCELERATION;
/// Maximum rotational jerk. Port of `franka::kMaxRotationalJerk`.
pub const MAX_ROTATIONAL_JERK: f64 = 8500.0 - LIMIT_EPS;
/// Maximum rotational acceleration. Port of `franka::kMaxRotationalAcceleration`.
pub const MAX_ROTATIONAL_ACCELERATION: f64 = 17.0 - LIMIT_EPS;
/// Maximum rotational velocity. Port of `franka::kMaxRotationalVelocity`.
pub const MAX_ROTATIONAL_VELOCITY: f64 =
    2.5 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_ROTATIONAL_ACCELERATION;
/// Maximum elbow jerk. Port of `franka::kMaxElbowJerk`.
pub const MAX_ELBOW_JERK: f64 = 5000.0 - LIMIT_EPS;
/// Maximum elbow acceleration. Port of `franka::kMaxElbowAcceleration`.
pub const MAX_ELBOW_ACCELERATION: f64 = 10.0 - LIMIT_EPS;
/// Maximum elbow velocity. Port of `franka::kMaxElbowVelocity`.
pub const MAX_ELBOW_VELOCITY: f64 =
    1.5 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_ELBOW_ACCELERATION;
/// Minimum elbow velocity. Port of `franka::kMinElbowVelocity`.
pub const MIN_ELBOW_VELOCITY: f64 = -MAX_ELBOW_VELOCITY;
/// Joint position limits (lower, upper), rad: the `<limit>` of `joint1..7` in the FR3 URDF of
/// libfranka's test suite (`test/fr3.urdf`, the crate's `tests/data/fr3.urdf`).
pub const JOINT_POSITION_LIMITS: ([f64; 7], [f64; 7]) = (
    [
        -2.7501, -1.7918, -2.9065, -3.0481, -2.8101, 0.54092, -3.0196,
    ],
    [2.7501, 1.7918, 2.9065, -0.1458, 2.8101, 4.5205, 3.0196],
);

#[cfg(test)]
mod tests;
