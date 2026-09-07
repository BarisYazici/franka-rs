//! The 1 kHz control loop.
//!
//! Port of `franka::ControlLoop` (libfranka 0.21.2 `src/control_loop.{h,cpp}`): start a motion,
//! call the user callbacks once per cycle, low-pass filter and rate limit what they return,
//! send it, and finish or cancel the motion at the end.
//!
//! This module holds the pieces every control loop shares — the per-version rate-limiting
//! envelope, the joint-velocity-limit source and the `ConvertContext` the conversions run
//! in. The conversions themselves are in the `motion` submodule (the four motion generator
//! command types) and `torque`; the loop that drives them is in `runner`.

mod motion;
mod runner;
mod torque;

pub(crate) use runner::{control_torques, ControlLoop};
pub(crate) use torque::convert_torques;
#[cfg(test)]
pub(crate) use torque::torque_rate_margin;

use crate::control_types::{MotionGenerator, MotionGeneratorKind, Torques};
use crate::duration::Duration;
use crate::error::FrankaResult;
use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
use crate::rate_limiting;
use crate::robot::robot_impl::RobotImpl;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;
use crate::wire::robot::{MotionGeneratorCommand, MoveMotionGeneratorMode};

/// The rate-limiting envelope of one FCI version.
///
/// FCI v10 uses `franka::` constants of libfranka 0.21.2 (`include/franka/rate_limiting.h`),
/// FCI v5 the FER constants of libfranka 0.9.2 ([`crate::rate_limiting::fer`]). The joint
/// *velocity* limits are not here: they come from the robot through
/// [`JointVelocityLimitsSource`], which is what makes the FR3's position-dependent envelope and
/// the FER's flat one interchangeable.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RateLimits {
    pub max_joint_acceleration: [f64; 7],
    pub max_joint_jerk: [f64; 7],
    pub max_translational_velocity: f64,
    pub max_translational_acceleration: f64,
    pub max_translational_jerk: f64,
    pub max_rotational_velocity: f64,
    pub max_rotational_acceleration: f64,
    pub max_rotational_jerk: f64,
    pub max_elbow_velocity: f64,
    pub min_elbow_velocity: f64,
    pub max_elbow_acceleration: f64,
    pub max_elbow_jerk: f64,
    pub max_torque_rate: [f64; 7],
    /// Whether the `float32` `tau_J_d` margin of [`torque_rate_margin`] applies.
    ///
    /// FCI v10 publishes `tau_J_d` as a `float32` (`rbk_types.h:104`), so the margin is needed
    /// there. The FCI v5 `RobotState` carries `tau_J_d` as a `double` (`rbk_types.h` of
    /// libfranka 0.9.2), so an FER's rate limiting is exact and the margin — which would
    /// silently shrink the user's torque budget — is switched off.
    pub torque_f32_margin: bool,
}

/// The constants `version` rate limits against.
pub(crate) const fn rate_limits(version: FciVersion) -> RateLimits {
    match version {
        FciVersion::V5 => RateLimits {
            max_joint_acceleration: rate_limiting::fer::MAX_JOINT_ACCELERATION,
            max_joint_jerk: rate_limiting::fer::MAX_JOINT_JERK,
            max_translational_velocity: rate_limiting::fer::MAX_TRANSLATIONAL_VELOCITY,
            max_translational_acceleration: rate_limiting::fer::MAX_TRANSLATIONAL_ACCELERATION,
            max_translational_jerk: rate_limiting::fer::MAX_TRANSLATIONAL_JERK,
            max_rotational_velocity: rate_limiting::fer::MAX_ROTATIONAL_VELOCITY,
            max_rotational_acceleration: rate_limiting::fer::MAX_ROTATIONAL_ACCELERATION,
            max_rotational_jerk: rate_limiting::fer::MAX_ROTATIONAL_JERK,
            max_elbow_velocity: rate_limiting::fer::MAX_ELBOW_VELOCITY,
            min_elbow_velocity: rate_limiting::fer::MIN_ELBOW_VELOCITY,
            max_elbow_acceleration: rate_limiting::fer::MAX_ELBOW_ACCELERATION,
            max_elbow_jerk: rate_limiting::fer::MAX_ELBOW_JERK,
            max_torque_rate: rate_limiting::fer::MAX_TORQUE_RATE,
            torque_f32_margin: false,
        },
        FciVersion::V10 => RateLimits {
            max_joint_acceleration: rate_limiting::MAX_JOINT_ACCELERATION,
            max_joint_jerk: rate_limiting::MAX_JOINT_JERK,
            max_translational_velocity: rate_limiting::MAX_TRANSLATIONAL_VELOCITY,
            max_translational_acceleration: rate_limiting::MAX_TRANSLATIONAL_ACCELERATION,
            max_translational_jerk: rate_limiting::MAX_TRANSLATIONAL_JERK,
            max_rotational_velocity: rate_limiting::MAX_ROTATIONAL_VELOCITY,
            max_rotational_acceleration: rate_limiting::MAX_ROTATIONAL_ACCELERATION,
            max_rotational_jerk: rate_limiting::MAX_ROTATIONAL_JERK,
            max_elbow_velocity: rate_limiting::MAX_ELBOW_VELOCITY,
            min_elbow_velocity: rate_limiting::MIN_ELBOW_VELOCITY,
            max_elbow_acceleration: rate_limiting::MAX_ELBOW_ACCELERATION,
            max_elbow_jerk: rate_limiting::MAX_ELBOW_JERK,
            max_torque_rate: rate_limiting::MAX_TORQUE_RATE,
            torque_f32_margin: true,
        },
    }
}

/// Source of the position-dependent joint velocity limits used by the rate limiter.
///
/// libfranka 0.21 reads them from the robot through `RobotControl::getUpper/LowerJointVelocity
/// Limits`; the trait exists so the conversion functions can be unit tested with the fixed
/// limits the C++ tests inject through `MockRobotControl`.
pub(crate) trait JointVelocityLimitsSource {
    /// Upper joint velocity limits at `q`.
    fn upper_joint_velocity_limits(&self, q: &[f64; 7]) -> [f64; 7];
    /// Lower joint velocity limits at `q`.
    fn lower_joint_velocity_limits(&self, q: &[f64; 7]) -> [f64; 7];
}

impl JointVelocityLimitsSource for RobotImpl {
    fn upper_joint_velocity_limits(&self, q: &[f64; 7]) -> [f64; 7] {
        RobotImpl::upper_joint_velocity_limits(self, q)
    }

    fn lower_joint_velocity_limits(&self, q: &[f64; 7]) -> [f64; 7] {
        RobotImpl::lower_joint_velocity_limits(self, q)
    }
}

/// Everything `ControlLoop::convertMotion` needs besides the state and the command.
pub(crate) struct ConvertContext<'a> {
    /// Where the joint velocity limits come from.
    pub limits: &'a dyn JointVelocityLimitsSource,
    /// The version's acceleration/jerk/Cartesian/elbow envelope.
    pub rate: RateLimits,
    /// Whether the rate limiter is active.
    pub limit_rate: bool,
    /// Low-pass cutoff; filtering is skipped at [`MAX_CUTOFF_FREQUENCY`] and above.
    pub cutoff_frequency: f64,
    /// False until the first command of the motion has been converted
    /// (`ControlLoop::initialized_filter_`).
    pub initialized_filter: bool,
    /// Whether the *first* converted command of a motion is its own filter and rate-limiter
    /// reference.
    ///
    /// libfranka 0.21.2 does that through `ControlLoop::initialized_filter_`
    /// (`src/control_loop.h:120`, used at `src/control_loop.cpp:194-200` and `:245-252`), so on
    /// an FR3 the first joint-position / Cartesian-pose setpoint of a motion passes the rate
    /// limiter unchanged. libfranka 0.9.2 has no such field: its `convertMotion`
    /// (`src/control_loop.cpp:188-205`, `:225-262`) always references `robot_state.q_d` /
    /// `O_T_EE_c`, so on an FER the very first setpoint is clamped to one integration step
    /// from the robot's own state like every other one. `true` on FCI v10, `false` on FCI v5.
    pub first_command_is_its_own_reference: bool,
}

impl ConvertContext<'_> {
    fn filtering(&self) -> bool {
        self.cutoff_frequency < MAX_CUTOFF_FREQUENCY
    }
}

/// A command type that a [`ControlLoop`] can turn into a `MotionGeneratorCommand`.
///
/// Port of the `ControlLoop<T>::convertMotion` specialisations.
pub(crate) trait ControlLoopMotion: MotionGenerator {
    /// The `Move` motion generator mode this command type requests.
    const MOVE_MODE: MoveMotionGeneratorMode;

    /// Filters, rate limits and validates the callback's output, then writes it into `command`.
    fn convert(
        &self,
        context: &mut ConvertContext<'_>,
        state: &RobotState,
        command: &mut MotionGeneratorCommand,
    ) -> FrankaResult<()>;
}

/// Maps a [`MotionGeneratorKind`] onto the wire enum (libfranka's `MotionGeneratorTraits`).
pub(crate) const fn move_mode(kind: MotionGeneratorKind) -> MoveMotionGeneratorMode {
    match kind {
        MotionGeneratorKind::JointPosition => MoveMotionGeneratorMode::JointPosition,
        MotionGeneratorKind::JointVelocity => MoveMotionGeneratorMode::JointVelocity,
        MotionGeneratorKind::CartesianPosition => MoveMotionGeneratorMode::CartesianPosition,
        MotionGeneratorKind::CartesianVelocity => MoveMotionGeneratorMode::CartesianVelocity,
    }
}

/// The user's motion generator callback (`ControlLoop::MotionGeneratorCallback`).
pub(crate) type MotionCallback<'a, M> = &'a mut dyn FnMut(&RobotState, Duration) -> M;

/// The user's controller callback (`ControlLoop::ControlCallback`).
pub(crate) type ControlCallback<'a> = &'a mut dyn FnMut(&RobotState, Duration) -> Torques;

#[cfg(test)]
mod tests;
