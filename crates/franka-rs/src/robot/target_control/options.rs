//! The options of the two [target control](super) interfaces and their builders.

use super::cartesian::{CartesianObserver, CartesianSent};
use super::joint::{JointObserver, JointSent};
use super::{validate_common, Backend, ImpedanceOptions, Settle};
use crate::control_types::ControllerMode;
use crate::error::{FrankaError, FrankaResult};
use crate::otg::{MultiOtg, OtgLimits};
use crate::rate_limiting;
use crate::robot::control_loop::rate_limits;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;

/// The FR3's flat joint velocity caps, rad/s: the `<limit velocity>` of its URDF, which is
/// what the position-dependent envelope of `compute_upper_limits_joint_velocity` saturates
/// at away from the joint limits.
const FR3_MAX_JOINT_VELOCITY: [f64; 7] = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];

/// The fraction of the robot's joint limits [`JointTargetControlOptions::default`] budgets.
pub const DEFAULT_LIMIT_FRACTION: f64 = 0.2;

/// Options of [`Robot::start_cartesian_target_control`](crate::Robot::start_cartesian_target_control);
/// [`Default`] is the documented starting point and the `with_*` methods change one field each.
pub struct TargetControlOptions {
    /// The translational budget as a *norm*: m/s, m/s^2, m/s^3. The generator gets
    /// [`OtgLimits::per_axis_for_norm`]`(3)` of it and the backstop the norm itself.
    /// Default 0.3, 0.5, 20: measured on a real FER, the robot's joint-space continuity check
    /// refuses 2.5 m/s^2 near the ready pose and its collision threshold trips above about
    /// 1 m/s^2, so the default sits well below both.
    pub limits: OtgLimits,
    /// The rotational budget as a *norm*: rad/s, rad/s^2, rad/s^3; the generator gets
    /// libfranka's pose-interface factor (0.99) of its `per_axis_for_norm(3)`, the backstop
    /// applies that factor itself. Default 0.5, 1.0, 20: a fifth of the FR3's velocity
    /// limit, validated on the simulator's joint-side check only.
    pub rotation_limits: OtgLimits,
    /// What turns the generator's pose into a command: the crate's impedance law sending
    /// torques, or a pose stream to the robot's own controller. Default
    /// [`Backend::Impedance`] of [`ImpedanceOptions::cartesian`].
    pub backend: Backend,
    /// The robot's internal controller, with [`Backend::RobotController`] only. Default
    /// [`ControllerMode::CartesianImpedance`].
    pub controller_mode: ControllerMode,
    /// How far, m, the measured `O_T_EE` position may stray from the start before the target
    /// is frozen and the loop ends with [`super::DEVIATION_MESSAGE`]. Default 0.30.
    pub max_deviation: f64,
    /// How far, rad, the measured `O_T_EE` orientation may turn from the start before the
    /// same guard fires. Default 0.5.
    pub max_angular_deviation: f64,
    /// When a stop counts as settled; the tolerance applies to the position in m and to the
    /// orientation error in rad alike. Default 1 mm and 1 mrad for 250 cycles.
    pub settle: Settle,
    /// Whether `limit_rate_cartesian_pose` runs under the two budgets as the backstop, with
    /// the loop's own libfranka limiter behind it; with [`Backend::Impedance`], whether the
    /// torque rate limiter runs. Default `true`.
    pub limit_rate: bool,
    /// `SCHED_FIFO` priority for the loop thread; `None` is the highest, as in `Robot::new`.
    pub realtime_priority: Option<i32>,
    /// Called every cycle on the realtime thread; see [`CartesianObserver`].
    pub observer: Option<CartesianObserver>,
}

impl Default for TargetControlOptions {
    fn default() -> Self {
        TargetControlOptions {
            limits: OtgLimits {
                max_velocity: 0.3,
                max_acceleration: 0.5,
                max_jerk: 20.0,
            },
            rotation_limits: OtgLimits {
                max_velocity: 0.5,
                max_acceleration: 1.0,
                max_jerk: 20.0,
            },
            backend: Backend::Impedance(ImpedanceOptions::cartesian()),
            controller_mode: ControllerMode::CartesianImpedance,
            max_deviation: 0.30,
            max_angular_deviation: 0.5,
            settle: Settle::default(),
            limit_rate: true,
            realtime_priority: None,
            observer: None,
        }
    }
}

impl std::fmt::Debug for TargetControlOptions {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TargetControlOptions")
            .field("limits", &self.limits)
            .field("rotation_limits", &self.rotation_limits)
            .field("backend", &self.backend)
            .field("controller_mode", &self.controller_mode)
            .field("max_deviation", &self.max_deviation)
            .field("max_angular_deviation", &self.max_angular_deviation)
            .field("settle", &self.settle)
            .field("limit_rate", &self.limit_rate)
            .field("realtime_priority", &self.realtime_priority)
            .field("observer", &self.observer.is_some())
            .finish()
    }
}

impl TargetControlOptions {
    /// Sets the translational norm budget.
    pub fn with_limits(mut self, limits: OtgLimits) -> Self {
        self.limits = limits;
        self
    }

    /// Sets the rotational norm budget (rad/s, rad/s^2, rad/s^3).
    pub fn with_rotation_limits(mut self, limits: OtgLimits) -> Self {
        self.rotation_limits = limits;
        self
    }

    /// Sets the backend.
    pub fn with_backend(mut self, backend: Backend) -> Self {
        self.backend = backend;
        self
    }

    /// Sets the robot's internal controller ([`Backend::RobotController`] only).
    pub fn with_controller_mode(mut self, mode: ControllerMode) -> Self {
        self.controller_mode = mode;
        self
    }

    /// Sets the deviation guard on the position, m.
    pub fn with_max_deviation(mut self, metres: f64) -> Self {
        self.max_deviation = metres;
        self
    }

    /// Sets the deviation guard on the orientation, rad.
    pub fn with_max_angular_deviation(mut self, radians: f64) -> Self {
        self.max_angular_deviation = radians;
        self
    }

    /// Sets the settle criterion.
    pub fn with_settle(mut self, settle: Settle) -> Self {
        self.settle = settle;
        self
    }

    /// Switches the backstop and the loop's limiter.
    pub fn with_limit_rate(mut self, limit_rate: bool) -> Self {
        self.limit_rate = limit_rate;
        self
    }

    /// Sets the loop thread's `SCHED_FIFO` priority (`None`: the highest).
    pub fn with_realtime_priority(mut self, priority: Option<i32>) -> Self {
        self.realtime_priority = priority;
        self
    }

    /// Installs the observer.
    pub fn with_observer(
        mut self,
        observer: impl FnMut(&RobotState, &CartesianSent) + Send + 'static,
    ) -> Self {
        self.observer = Some(Box::new(observer));
        self
    }

    /// Checks the options without starting anything.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] naming the field.
    pub fn validate(&self) -> FrankaResult<()> {
        crate::otg::Otg::new(0.0, self.limits)?;
        crate::otg::Otg::new(0.0, self.rotation_limits).map_err(|_| {
            FrankaError::InvalidArgument(format!(
                "target control: rotation_limits must be finite and positive, got {:?}",
                self.rotation_limits
            ))
        })?;
        if !(self.max_angular_deviation.is_finite() && self.max_angular_deviation > 0.0) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: max_angular_deviation must be finite and positive, got {}",
                self.max_angular_deviation
            )));
        }
        validate_common(
            self.max_deviation,
            self.settle,
            self.realtime_priority,
            &self.backend,
        )
    }
}

/// Options of [`Robot::start_joint_target_control`](crate::Robot::start_joint_target_control);
/// [`Default`] is the documented starting point and the `with_*` methods change one field each.
pub struct JointTargetControlOptions {
    /// Per-joint limits, rad/s, rad/s^2, rad/s^3, for the generator and, with
    /// [`Backend::RobotController`], the backstop alike. `None`, the default, is
    /// [`JointTargetControlOptions::scaled_limits`] of the negotiated FCI version at
    /// [`DEFAULT_LIMIT_FRACTION`], resolved when the loop starts.
    pub limits: Option<[OtgLimits; 7]>,
    /// What turns the generator's joint positions into a command: the crate's impedance law
    /// sending torques, or a position stream to the robot's own controller. Default
    /// [`Backend::Impedance`] of [`ImpedanceOptions::joint`].
    pub backend: Backend,
    /// The robot's internal controller, with [`Backend::RobotController`] only. Default
    /// [`ControllerMode::JointImpedance`].
    pub controller_mode: ControllerMode,
    /// How far, rad, any measured joint may stray from its start before the target is frozen
    /// and the loop ends with [`super::DEVIATION_MESSAGE`]. Default 1.0.
    pub max_deviation: f64,
    /// When a stop counts as settled. Default 1 mrad for 250 cycles.
    pub settle: Settle,
    /// Whether `limit_rate_joint_positions` runs under the limits as the backstop, with the
    /// loop's own libfranka limiter behind it; with [`Backend::Impedance`], whether the torque
    /// rate limiter runs. Default `true`.
    pub limit_rate: bool,
    /// `SCHED_FIFO` priority for the loop thread; `None` is the highest, as in `Robot::new`.
    pub realtime_priority: Option<i32>,
    /// Called every cycle on the realtime thread; see [`JointObserver`].
    pub observer: Option<JointObserver>,
}

impl Default for JointTargetControlOptions {
    fn default() -> Self {
        JointTargetControlOptions {
            limits: None,
            backend: Backend::Impedance(ImpedanceOptions::joint()),
            controller_mode: ControllerMode::JointImpedance,
            max_deviation: 1.0,
            settle: Settle::default(),
            limit_rate: true,
            realtime_priority: None,
            observer: None,
        }
    }
}

impl std::fmt::Debug for JointTargetControlOptions {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JointTargetControlOptions")
            .field("limits", &self.limits)
            .field("backend", &self.backend)
            .field("controller_mode", &self.controller_mode)
            .field("max_deviation", &self.max_deviation)
            .field("settle", &self.settle)
            .field("limit_rate", &self.limit_rate)
            .field("realtime_priority", &self.realtime_priority)
            .field("observer", &self.observer.is_some())
            .finish()
    }
}

impl JointTargetControlOptions {
    /// `fraction` of `version`'s joint limits: the FR3's flat velocity caps or the FER's
    /// `MAX_JOINT_VELOCITY`, and the version's `MAX_JOINT_ACCELERATION` and
    /// `MAX_JOINT_JERK`.
    pub fn scaled_limits(version: FciVersion, fraction: f64) -> [OtgLimits; 7] {
        let rate = rate_limits(version);
        let velocity = match version {
            FciVersion::V5 => rate_limiting::fer::MAX_JOINT_VELOCITY,
            FciVersion::V10 => FR3_MAX_JOINT_VELOCITY,
        };
        std::array::from_fn(|i| OtgLimits {
            max_velocity: velocity[i] * fraction,
            max_acceleration: rate.max_joint_acceleration[i] * fraction,
            max_jerk: rate.max_joint_jerk[i] * fraction,
        })
    }

    /// Sets the per-joint limits.
    pub fn with_limits(mut self, limits: [OtgLimits; 7]) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Sets the backend.
    pub fn with_backend(mut self, backend: Backend) -> Self {
        self.backend = backend;
        self
    }

    /// Sets the robot's internal controller ([`Backend::RobotController`] only).
    pub fn with_controller_mode(mut self, mode: ControllerMode) -> Self {
        self.controller_mode = mode;
        self
    }

    /// Sets the deviation guard, rad.
    pub fn with_max_deviation(mut self, radians: f64) -> Self {
        self.max_deviation = radians;
        self
    }

    /// Sets the settle criterion.
    pub fn with_settle(mut self, settle: Settle) -> Self {
        self.settle = settle;
        self
    }

    /// Switches the backstop and the loop's limiter.
    pub fn with_limit_rate(mut self, limit_rate: bool) -> Self {
        self.limit_rate = limit_rate;
        self
    }

    /// Sets the loop thread's `SCHED_FIFO` priority (`None`: the highest).
    pub fn with_realtime_priority(mut self, priority: Option<i32>) -> Self {
        self.realtime_priority = priority;
        self
    }

    /// Installs the observer.
    pub fn with_observer(
        mut self,
        observer: impl FnMut(&RobotState, &JointSent) + Send + 'static,
    ) -> Self {
        self.observer = Some(Box::new(observer));
        self
    }

    /// Checks the options without starting anything.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] naming the field.
    pub fn validate(&self) -> FrankaResult<()> {
        if let Some(limits) = self.limits {
            MultiOtg::with_limits([0.0; 7], limits, true)?;
        }
        validate_common(
            self.max_deviation,
            self.settle,
            self.realtime_priority,
            &self.backend,
        )
    }
}
