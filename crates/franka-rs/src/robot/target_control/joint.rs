//! The joint interface of [target control](super): seven joint position targets, driven
//! through [`Robot::control_joint_positions`].

use std::sync::Arc;

use super::runner::identity;
use super::{spawn, validate_common, Handle, Runner, Settle, Shared};
use crate::control_types::{ControllerMode, JointPositions};
use crate::error::FrankaResult;
use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
use crate::otg::{MultiOtg, OtgLimits};
use crate::rate_limiting::{self, limit_rate_joint_positions};
use crate::robot::control_loop::rate_limits;
use crate::robot::Robot;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;

/// The FR3's flat joint velocity caps, rad/s: the `<limit velocity>` of its URDF, which is
/// what the position-dependent envelope of `compute_upper_limits_joint_velocity` saturates
/// at away from the joint limits.
const FR3_MAX_JOINT_VELOCITY: [f64; 7] = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];

/// The fraction of the robot's joint limits [`JointTargetControlOptions::default`] budgets.
pub const DEFAULT_LIMIT_FRACTION: f64 = 0.2;

/// What one cycle sent, for the observer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointSent {
    /// The joint positions that went to the robot (after the backstop), rad.
    pub q: [f64; 7],
    /// The target the generator was planning towards, rad.
    pub target: [f64; 7],
    /// The generator's velocity at the end of the cycle, rad/s.
    pub velocity: [f64; 7],
    /// The generator's acceleration at the end of the cycle, rad/s^2.
    pub acceleration: [f64; 7],
    /// The most the backstop moved any joint of the generator's position, rad; zero without
    /// `limit_rate`. Microradians are the noise of the limiter's velocity and acceleration
    /// reference, the robot's float32 echo (FCI v10); milliradians mean the generator is
    /// outrunning the limits.
    pub backstop_alteration: f64,
}

/// The observer's type: called every cycle on the realtime thread, so it must not allocate
/// or block.
pub type JointObserver = Box<dyn FnMut(&RobotState, &JointSent) + Send>;

/// Options of [`Robot::start_joint_target_control`]; [`Default`] is the documented starting
/// point and the `with_*` methods change one field each.
pub struct JointTargetControlOptions {
    /// Per-joint limits, rad/s, rad/s^2, rad/s^3, for the generator and the backstop alike.
    /// `None`, the default, is [`JointTargetControlOptions::scaled_limits`] of the negotiated
    /// FCI version at [`DEFAULT_LIMIT_FRACTION`], resolved when the loop starts.
    pub limits: Option<[OtgLimits; 7]>,
    /// The robot's internal controller. Default [`ControllerMode::JointImpedance`].
    pub controller_mode: ControllerMode,
    /// How far, rad, any measured joint may stray from its start before the target is frozen
    /// and the loop ends with [`super::DEVIATION_MESSAGE`]. Default 1.0.
    pub max_deviation: f64,
    /// When a stop counts as settled. Default 1 mrad for 250 cycles.
    pub settle: Settle,
    /// Whether `limit_rate_joint_positions` runs under the limits as the backstop, with the
    /// loop's own libfranka limiter behind it. Default `true`.
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

    /// Sets the robot's internal controller.
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
        validate_common(self.max_deviation, self.settle, self.realtime_priority)
    }
}

/// The handle of a running joint target control; see the [module documentation](super).
pub struct JointTargetControl {
    inner: Handle<7>,
}

impl JointTargetControl {
    /// Sets the target joint positions, rad. Callable from any thread at any rate; only the
    /// latest target counts.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] if a value is not finite,
    /// [`crate::error::FrankaError::InvalidOperation`] with [`super::ENDED_MESSAGE`] once the
    /// loop has ended for any reason.
    pub fn set_joints(&self, q: [f64; 7]) -> FrankaResult<()> {
        self.inner.set_target(q)
    }

    /// The latest target set (the start configuration until the first `set_joints`).
    pub fn target(&self) -> [f64; 7] {
        self.inner.target()
    }

    /// The latest robot state the loop received, copied out.
    pub fn state(&self) -> RobotState {
        self.inner.state()
    }

    /// Whether the loop is still running; `false` after it ended for any reason.
    pub fn is_running(&self) -> bool {
        self.inner.is_running()
    }

    /// Settles at the current target, finishes the motion, joins the thread and returns the
    /// loop's result: `Ok` for a regular end, [`crate::error::FrankaError::Control`] if the
    /// robot aborted the motion or the deviation guard fired.
    pub fn stop(self) -> FrankaResult<()> {
        self.inner.stop()
    }
}

fn max_abs_difference(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

pub(super) fn start(
    robot: &Arc<Robot>,
    options: JointTargetControlOptions,
) -> FrankaResult<JointTargetControl> {
    options.validate()?;
    let JointTargetControlOptions {
        limits,
        controller_mode,
        max_deviation,
        settle,
        limit_rate,
        realtime_priority,
        mut observer,
    } = options;
    let limits = limits.unwrap_or_else(|| {
        JointTargetControlOptions::scaled_limits(robot.fci_version(), DEFAULT_LIMIT_FRACTION)
    });
    let shared = Arc::new(Shared::<7>::default());
    let loop_shared = Arc::clone(&shared);
    let body = move |robot: &Robot, started| {
        let mut runner = Runner::new(loop_shared, started, limits, settle, identity)?;
        let velocity = limits.map(|l| l.max_velocity);
        let acceleration = limits.map(|l| l.max_acceleration);
        let jerk = limits.map(|l| l.max_jerk);
        let mut start = [0.0; 7];
        let mut anchored = false;
        let result = robot.control_joint_positions(
            |state: &RobotState, _period| {
                if !anchored {
                    anchored = true;
                    start = state.q_d;
                }
                let strayed = max_abs_difference(&state.q, &start) > max_deviation;
                let step = runner.cycle(state, state.q_d, strayed);

                let mut q = step.position;
                let mut backstop_alteration = 0.0;
                if limit_rate && !step.hold {
                    // The budget, tightened to the robot's own velocity envelope at `q`
                    // (position-dependent on an FR3), against the robot's echo; it only fails
                    // on non-finite input, and holding the echo is then the safe command.
                    // Skipped while the stop holds: that command must go out bit-identical.
                    let upper = robot.upper_joint_velocity_limits(&state.q);
                    let lower = robot.lower_joint_velocity_limits(&state.q);
                    let upper: [f64; 7] = std::array::from_fn(|i| upper[i].min(velocity[i]));
                    let lower: [f64; 7] = std::array::from_fn(|i| lower[i].max(-velocity[i]));
                    #[rustfmt::skip]
                    let limited = limit_rate_joint_positions(
                        &upper, &lower, &acceleration, &jerk,
                        &q, &state.q_d, &state.dq_d, &state.ddq_d,
                    ).unwrap_or(state.q_d);
                    backstop_alteration = max_abs_difference(&limited, &q);
                    q = limited;
                }
                if let Some(observe) = observer.as_mut() {
                    observe(
                        state,
                        &JointSent {
                            q,
                            target: step.target,
                            velocity: step.velocity,
                            acceleration: step.acceleration,
                            backstop_alteration,
                        },
                    );
                }
                let mut output = JointPositions::new(q);
                output.motion_finished = step.finished;
                output
            },
            controller_mode,
            limit_rate,
            MAX_CUTOFF_FREQUENCY,
        );
        runner.finish(result)
    };
    let inner = spawn(
        "franka-joint-target",
        robot,
        shared,
        realtime_priority,
        body,
    )?;
    Ok(JointTargetControl { inner })
}
