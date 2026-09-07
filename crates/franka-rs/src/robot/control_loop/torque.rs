//! `ControlLoop::createControlCommand` — the torque half of a cycle — and the `float32`
//! `tau_J_d` margin the FR3 needs on top of libfranka's rate limiting.

use super::*;
use crate::lowpass_filter::low_pass_filter;
use crate::rate_limiting::{limit_rate_torques, DELTA_T};
use crate::wire::f64s_to_wire;
use crate::wire::robot::ControllerCommand;

/// Rate budget, in Nm/s, that a torque command must give up to absorb the `float32`
/// quantisation of the `tau_J_d` it is limited against.
///
/// A torque client must rate-limit against the published `tau_J_d`, and must not exceed the
/// nominal limits by float32 rounding at |τ| ≥ 64 Nm. The robot publishes
/// `tau_J_d` as a `float32` (`rbk_types.h:104`, widened in [`RobotState::from_wire`]) but
/// judges the rate of our command against the `double` it last received. At |τ| ≥ 64 Nm one
/// `f32` ULP is 2⁻¹⁷ ≈ 7.6e-6 Nm, so the half-ULP the robot's own value can differ by is worth
/// up to ≈ 3.8e-3 Nm/s over a 1 ms cycle — more than the 1e-3 Nm/s that
/// [`MAX_TORQUE_RATE`] keeps below the nominal 1000 Nm/s. A fully saturated command computed
/// against the quantised value therefore reads as up to ≈ 1000.0028 Nm/s at the robot.
///
/// [`limit_rate_torques`] stays an exact port of `franka::limitRate` (libfranka has the
/// identical exposure); the margin is applied here, in the control loop, by shrinking the
/// per-joint maximum derivative by half an `f32` ULP of `|tau_J_d|` per cycle.
/// `f32::EPSILON * |x|` is an upper bound on `ulp(x)` for every finite `x`, so the result is
/// conservative, and it is zero at `tau_J_d == 0` — the ordinary case, which therefore keeps
/// libfranka's exact behaviour.
pub(crate) fn torque_rate_margin(tau_J_d: f64) -> f64 {
    f64::from(f32::EPSILON) * tau_J_d.abs() / 2.0 / DELTA_T
}

/// Filters, rate limits and validates a `Torques` command
/// (`ControlLoop::createControlCommand`).
pub(crate) fn convert_torques(
    torques: &Torques,
    state: &RobotState,
    rate: &RateLimits,
    limit_rate: bool,
    cutoff_frequency: f64,
    command: &mut ControllerCommand,
) -> FrankaResult<()> {
    let mut tau_J = torques.tau_J;
    if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
        for (i, value) in tau_J.iter_mut().enumerate() {
            *value = low_pass_filter(DELTA_T, *value, state.tau_J_d[i], cutoff_frequency)?;
        }
    }
    if limit_rate {
        // `kMaxTorqueRate` minus the float32 quantisation margin (see `torque_rate_margin`),
        // which only applies where the robot publishes `tau_J_d` as a `float32` (FCI v10).
        let mut max_derivatives = rate.max_torque_rate;
        if rate.torque_f32_margin {
            for (max_derivative, tau_J_d) in max_derivatives.iter_mut().zip(state.tau_J_d) {
                *max_derivative -= torque_rate_margin(tau_J_d);
            }
        }
        tau_J = limit_rate_torques(&max_derivatives, &tau_J, &state.tau_J_d)?;
    }
    Torques::new(tau_J).validate()?;
    command.tau_J_d = f64s_to_wire(&tau_J);
    Ok(())
}
