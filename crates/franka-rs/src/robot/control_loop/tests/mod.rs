//! Port of libfranka's `test/control_loop_tests.cpp` conversion cases.
//!
//! The helpers live here; the cases are in [`torque`] and [`motion`].

mod motion;
mod torque;

use super::*;
use crate::error::FrankaError;
use crate::wire::f64s_to_f64;

/// The FR3 envelope, which every pre-existing case in this module is written against.
const V10_RATE: RateLimits = rate_limits(FciVersion::V10);
/// The FER envelope.
const V5_RATE: RateLimits = rate_limits(FciVersion::V5);

/// The fixed limits libfranka's `setupJointVelocityLimitsMock` injects.
struct MockLimits;

impl JointVelocityLimitsSource for MockLimits {
    fn upper_joint_velocity_limits(&self, _q: &[f64; 7]) -> [f64; 7] {
        [2.0; 7]
    }
    fn lower_joint_velocity_limits(&self, _q: &[f64; 7]) -> [f64; 7] {
        [-2.0; 7]
    }
}

const IDENTITY: [f64; 16] = [
    1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
];

fn valid_state() -> RobotState {
    // Port of the test helper `generateValidRobotState()`.
    RobotState {
        O_T_EE: IDENTITY,
        O_T_EE_d: IDENTITY,
        F_T_EE: IDENTITY,
        EE_T_K: IDENTITY,
        O_T_EE_c: IDENTITY,
        ..RobotState::default()
    }
}

fn context(limit_rate: bool, filter: bool) -> ConvertContext<'static> {
    context_for(FciVersion::V10, limit_rate, filter)
}

/// [`context`] for an explicit FCI version, so the first-cycle reference rule can be
/// exercised on both.
fn context_for(version: FciVersion, limit_rate: bool, filter: bool) -> ConvertContext<'static> {
    ConvertContext {
        limits: &MockLimits,
        rate: rate_limits(version),
        first_command_is_its_own_reference: version == FciVersion::V10,
        limit_rate,
        cutoff_frequency: if filter {
            crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY
        } else {
            MAX_CUTOFF_FREQUENCY
        },
        initialized_filter: false,
    }
}

fn invalid_argument(result: FrankaResult<()>) -> String {
    match result {
        Err(FrankaError::InvalidArgument(message)) => message,
        other => panic!("expected InvalidArgument, got {other:?}"),
    }
}
