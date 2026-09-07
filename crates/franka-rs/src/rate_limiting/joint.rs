//! Joint-space rate limiting: the position/velocity `limitRate` overloads and the deprecated
//! FR3 position-dependent joint-velocity envelope.
//!
//! Split out of libfranka's `src/rate_limiting.cpp`; the constants these functions are normally
//! called with live in [the parent module](super).

use super::{DELTA_T, JOINT_VELOCITY_LIMITS_TOLERANCE};
use crate::error::{FrankaError, FrankaResult};
use crate::math_utils::{cmax, cmin};

/// Computes the maximum joint velocity based on joint position.
///
/// Port of the (deprecated) `franka::computeUpperLimitsJointVelocity`, which hardcodes the FR3
/// parameters. Prefer [`crate::joint_velocity_limits::JointVelocityLimitsConfig::upper_limits`],
/// which reads the same parameters from the robot's URDF.
pub fn compute_upper_limits_joint_velocity(q: &[f64; 7]) -> [f64; 7] {
    [
        cmin(
            2.62,
            cmax(0.0, -0.30 + cmax(0.0, 12.0 * (2.75010 - q[0])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[0],
        cmin(
            2.62,
            cmax(0.0, -0.20 + cmax(0.0, 5.17 * (1.79180 - q[1])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[1],
        cmin(
            2.62,
            cmax(0.0, -0.20 + cmax(0.0, 7.00 * (2.90650 - q[2])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[2],
        cmin(
            2.62,
            cmax(0.0, -0.30 + cmax(0.0, 8.00 * (-0.1458 - q[3])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[3],
        cmin(
            5.26,
            cmax(0.0, -0.35 + cmax(0.0, 34.0 * (2.81010 - q[4])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[4],
        cmin(
            4.18,
            cmax(0.0, -0.35 + cmax(0.0, 11.0 * (4.52050 - q[5])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[5],
        cmin(
            5.26,
            cmax(0.0, -0.35 + cmax(0.0, 34.0 * (3.01960 - q[6])).sqrt()),
        ) - JOINT_VELOCITY_LIMITS_TOLERANCE[6],
    ]
}

/// Computes the minimum joint velocity based on joint position.
///
/// Port of the (deprecated) `franka::computeLowerLimitsJointVelocity`, which hardcodes the FR3
/// parameters. Prefer [`crate::joint_velocity_limits::JointVelocityLimitsConfig::lower_limits`],
/// which reads the same parameters from the robot's URDF.
pub fn compute_lower_limits_joint_velocity(q: &[f64; 7]) -> [f64; 7] {
    [
        cmax(
            -2.62,
            cmin(0.0, 0.30 - cmax(0.0, 12.0 * (2.750100 + q[0])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[0],
        cmax(
            -2.62,
            cmin(0.0, 0.20 - cmax(0.0, 5.17 * (1.791800 + q[1])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[1],
        cmax(
            -2.62,
            cmin(0.0, 0.20 - cmax(0.0, 7.00 * (2.906500 + q[2])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[2],
        cmax(
            -2.62,
            cmin(0.0, 0.30 - cmax(0.0, 8.00 * (3.048100 + q[3])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[3],
        cmax(
            -5.26,
            cmin(0.0, 0.35 - cmax(0.0, 34.0 * (2.810100 + q[4])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[4],
        cmax(
            -4.18,
            cmin(0.0, 0.35 - cmax(0.0, 11.0 * (-0.54092 + q[5])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[5],
        cmax(
            -5.26,
            cmin(0.0, 0.35 - cmax(0.0, 34.0 * (3.019600 + q[6])).sqrt()),
        ) + JOINT_VELOCITY_LIMITS_TOLERANCE[6],
    ]
}

/// Limits the rate of a desired joint velocity considering the limits provided.
///
/// Port of the scalar `franka::limitRate(double upper_limits_velocity, ..., double
/// commanded_velocity, ...)`.
///
/// Unlike the C++ overload, this function does not check `commanded_velocity` for finiteness
/// (the fixed Rust interface makes it infallible); [`limit_rate_joint_velocities`] performs the
/// check for the whole vector, exactly like the C++ vector overload does before delegating here.
///
/// # Note
/// FCI filters must be deactivated to work properly.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_joint_velocity(
    upper_limits_velocity: f64,
    lower_limits_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    // Differentiate to get jerk
    let commanded_jerk = (((commanded_velocity - last_commanded_velocity) / DELTA_T)
        - last_commanded_acceleration)
        / DELTA_T;

    // Limit jerk and integrate to get acceleration
    let commanded_acceleration =
        last_commanded_acceleration + cmax(cmin(commanded_jerk, max_jerk), -max_jerk) * DELTA_T;

    // Compute acceleration limits
    let safe_max_acceleration = cmin(
        (max_jerk / max_acceleration) * (upper_limits_velocity - last_commanded_velocity),
        max_acceleration,
    );
    let safe_min_acceleration = cmax(
        (max_jerk / max_acceleration) * (lower_limits_velocity - last_commanded_velocity),
        -max_acceleration,
    );

    // Limit acceleration and integrate to get desired velocities
    last_commanded_velocity
        + cmax(
            cmin(commanded_acceleration, safe_max_acceleration),
            safe_min_acceleration,
        ) * DELTA_T
}

/// Limits the rate of a desired joint position considering the limits provided.
///
/// Port of the scalar `franka::limitRate(double upper_limits_velocity, ..., double
/// commanded_position, ...)`.
///
/// Unlike the C++ overload, this function does not check `commanded_position` for finiteness
/// (the fixed Rust interface makes it infallible); [`limit_rate_joint_positions`] performs the
/// check for the whole vector, exactly like the C++ vector overload does before delegating here.
///
/// # Note
/// FCI filters must be deactivated to work properly.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_joint_position(
    upper_limits_velocity: f64,
    lower_limits_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_position: f64,
    last_commanded_position: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    last_commanded_position
        + limit_rate_joint_velocity(
            upper_limits_velocity,
            lower_limits_velocity,
            max_acceleration,
            max_jerk,
            (commanded_position - last_commanded_position) / DELTA_T,
            last_commanded_velocity,
            last_commanded_acceleration,
        ) * DELTA_T
}

/// Limits the rate of desired joint velocities considering the limits provided.
///
/// Port of the `std::array<double, 7>` velocity overload of `franka::limitRate`.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `commanded_velocities` are infinite or NaN.
///
/// # Note
/// FCI filters must be deactivated to work properly.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_joint_velocities(
    upper_limits_velocity: &[f64; 7],
    lower_limits_velocity: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded_velocities: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> FrankaResult<[f64; 7]> {
    if !commanded_velocities.iter().all(|v| v.is_finite()) {
        return Err(FrankaError::InvalidArgument(
            "commanded_velocities is infinite or NaN.".to_string(),
        ));
    }
    let mut limited = [0.0; 7];
    for i in 0..7 {
        limited[i] = limit_rate_joint_velocity(
            upper_limits_velocity[i],
            lower_limits_velocity[i],
            max_acceleration[i],
            max_jerk[i],
            commanded_velocities[i],
            last_commanded_velocities[i],
            last_commanded_accelerations[i],
        );
    }
    Ok(limited)
}

/// Limits the rate of desired joint positions considering the limits provided.
///
/// Port of the `std::array<double, 7>` position overload of `franka::limitRate`.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `commanded_positions` are infinite or NaN.
///
/// # Note
/// FCI filters must be deactivated to work properly.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_joint_positions(
    upper_limits_velocity: &[f64; 7],
    lower_limits_velocity: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded_positions: &[f64; 7],
    last_commanded_positions: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> FrankaResult<[f64; 7]> {
    if !commanded_positions.iter().all(|v| v.is_finite()) {
        return Err(FrankaError::InvalidArgument(
            "commanded_positions is infinite or NaN.".to_string(),
        ));
    }
    let mut limited = [0.0; 7];
    for i in 0..7 {
        limited[i] = limit_rate_joint_position(
            upper_limits_velocity[i],
            lower_limits_velocity[i],
            max_acceleration[i],
            max_jerk[i],
            commanded_positions[i],
            last_commanded_positions[i],
            last_commanded_velocities[i],
            last_commanded_accelerations[i],
        );
    }
    Ok(limited)
}
