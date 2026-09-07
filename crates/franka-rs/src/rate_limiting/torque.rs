//! Torque rate limiting: the `max_derivatives` overload of `franka::limitRate`.
//!
//! Split out of libfranka's `src/rate_limiting.cpp`; the constants this function is normally
//! called with live in [the parent module](super).

use super::DELTA_T;
use crate::error::{FrankaError, FrankaResult};
use crate::math_utils::{cmax, cmin};

/// Limits the rate of an input vector of per-joint commands considering the maximum allowed
/// time derivatives.
///
/// Port of `franka::limitRate(const std::array<double, 7>& max_derivatives, ...)`, used for
/// torque commands.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `commanded_values` are infinite or NaN.
///
/// # Note
/// FCI filters must be deactivated to work properly.
///
/// # float32 quantisation
/// `last_commanded_values` is normally `RobotState::tau_J_d`, which the robot publishes as a
/// `float32` while judging our command's rate against the `double` it last received. Saturating
/// this function against the quantised value can therefore read as up to ≈ 1000.0028 Nm/s at
/// the robot for |τ| ≥ 64 Nm — libfranka has the identical exposure, and this function stays an
/// exact port of `franka::limitRate`. The control loop compensates by shrinking
/// `max_derivatives` per cycle; see `crate::robot::control_loop::torque::torque_rate_margin`.
pub fn limit_rate_torques(
    max_derivatives: &[f64; 7],
    commanded_values: &[f64; 7],
    last_commanded_values: &[f64; 7],
) -> FrankaResult<[f64; 7]> {
    if !commanded_values.iter().all(|v| v.is_finite()) {
        return Err(FrankaError::InvalidArgument(
            "Commanding value is infinite or NaN.".to_string(),
        ));
    }
    let mut limited_values = [0.0; 7];
    for i in 0..7 {
        let commanded_derivative = (commanded_values[i] - last_commanded_values[i]) / DELTA_T;
        limited_values[i] = last_commanded_values[i]
            + cmax(
                cmin(commanded_derivative, max_derivatives[i]),
                -max_derivatives[i],
            ) * DELTA_T;
    }
    Ok(limited_values)
}
