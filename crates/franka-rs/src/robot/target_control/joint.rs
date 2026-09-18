//! The joint interface of [target control](super): seven joint position targets.
//! [`Backend::Impedance`] sends the impedance law's torques towards the generator's output;
//! [`Backend::RobotController`] streams it through [`Robot::control_joint_positions`].

use std::sync::mpsc::SyncSender;
use std::sync::Arc;

use super::position::{JointLimits, VelocityLimit};
use super::runner::{identity, Step};
use super::torque::{JointTracker, TorqueLoop};
use super::{
    check_joint_limits, check_posture, joint_limit_inset, joint_position_limits, spawn, Backend,
    Handle, ImpedanceOptions, JointTargetControlOptions, Runner, Shared, DEFAULT_LIMIT_FRACTION,
};
use crate::control_types::JointPositions;
use crate::error::FrankaResult;
use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
use crate::model::Model;
use crate::otg::OtgLimits;
use crate::rate_limiting::limit_rate_joint_positions;
use crate::robot::Robot;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;

/// What one cycle sent, for the observer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointSent {
    /// The joint positions that went to the robot (after the backstop), or with
    /// [`Backend::Impedance`] the generator's output the law tracks, rad.
    pub q: [f64; 7],
    /// The target the generator was planning towards, rad.
    pub target: [f64; 7],
    /// The generator's velocity at the end of the cycle, rad/s.
    pub velocity: [f64; 7],
    /// The generator's acceleration at the end of the cycle, rad/s^2.
    pub acceleration: [f64; 7],
    /// The most the backstop moved any joint of the generator's position, rad; zero without
    /// `limit_rate` and with [`Backend::Impedance`]. Microradians are the noise of the
    /// limiter's velocity and acceleration reference, the robot's float32 echo (FCI v10);
    /// milliradians mean the generator is outrunning the limits.
    pub backstop_alteration: f64,
    /// The joint target of the impedance law, rad; zeros with [`Backend::RobotController`].
    pub q_goal: [f64; 7],
    /// The joint goal's velocity, rad/s: the finite difference of `q_goal` the law feeds
    /// forward, under the joint velocity cap; zeros while holding and with
    /// [`Backend::RobotController`].
    pub dq_goal: [f64; 7],
    /// The scale this cycle's goal step was cut by to stay in the joint limits' box: under
    /// [`ImpedanceOptions::joint_velocity_fraction`](super::ImpedanceOptions::joint_velocity_fraction)
    /// of the joint velocity limits, and braking to stop at
    /// [`ImpedanceOptions::joint_position_margin`](super::ImpedanceOptions::joint_position_margin).
    /// Exactly 1 when it was not, and with [`Backend::RobotController`].
    pub cap_scale: f64,
    /// Per joint, the bound that cut the step: 0 none, -1 / +1 the lower / upper position bound,
    /// -2 / +2 the velocity bound; zeros with [`Backend::RobotController`].
    pub pinned: [i8; 7],
    /// The torques sent, Nm, clamped to the torque limits; zeros with
    /// [`Backend::RobotController`].
    pub tau: [f64; 7],
    /// The velocity envelope's share of `tau`, Nm, before the clamp: the barrier opposing every
    /// joint measured faster than
    /// [`ImpedanceOptions::velocity_barrier_fraction`](super::ImpedanceOptions::velocity_barrier_fraction)
    /// of its limit, less the law's torque along a joint's motion faded out above the fade's
    /// start ([`FADE_BAND`](super::FADE_BAND)); zeros below it and with
    /// [`Backend::RobotController`].
    pub tau_envelope: [f64; 7],
    /// The position envelope's share of `tau`, Nm, before the clamp; see
    /// [`CartesianSent::tau_position`](super::CartesianSent::tau_position). Zeros with
    /// [`Backend::RobotController`].
    pub tau_position: [f64; 7],
    /// The most, rad, the leash pulled any joint of the generator's anchor back from the
    /// previous goal toward the measured position: zero while the arm follows, positive while
    /// it is held back ([`Leash`](super::Leash)); 0 with [`Backend::RobotController`].
    pub leash_alteration: f64,
}

/// The observer's type: called every cycle on the realtime thread, so it must not allocate
/// or block.
pub type JointObserver = Box<dyn FnMut(&RobotState, &JointSent) + Send>;

/// The handle of a running joint target control; see the [module documentation](super).
///
/// It has no live tuning, and no `tune`: [`LiveTuning`](super::LiveTuning) describes the
/// Cartesian interface's law and plan -- a Cartesian stiffness this interface has none of, and a
/// budget as a norm, where this one plans per joint. A joint session therefore runs the options
/// it was started with for its whole life.
pub struct JointTargetControl {
    inner: Handle<7>,
    limits: ([f64; 7], [f64; 7]),
    /// How far inside `limits` a target must lie.
    inset: f64,
}

impl JointTargetControl {
    /// Sets the target joint positions, rad. Callable from any thread at any rate; only the
    /// latest target counts.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] if a value is not finite or outside the
    /// arm's joint position limits inset by [`JOINT_LIMIT_INSET`](super::JOINT_LIMIT_INSET) (with
    /// [`Backend::Impedance`], by its
    /// [`joint_position_margin`](ImpedanceOptions::joint_position_margin) where that is larger),
    /// [`crate::error::FrankaError::InvalidOperation`] with [`super::ENDED_MESSAGE`] once the
    /// loop has ended for any reason.
    pub fn set_joints(&self, q: [f64; 7]) -> FrankaResult<()> {
        if q.iter().all(|v| v.is_finite()) {
            check_joint_limits(&q, &self.limits, self.inset, "target")?;
        }
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

pub(super) fn max_abs_difference(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// The observer's record of a step with `q` as sent; the backend's own fields stay zero.
pub(super) fn sent(step: &Step<7, 7>, q: [f64; 7]) -> JointSent {
    JointSent {
        q,
        target: step.target,
        velocity: step.velocity,
        acceleration: step.acceleration,
        backstop_alteration: 0.0,
        q_goal: [0.0; 7],
        dq_goal: [0.0; 7],
        cap_scale: 1.0,
        pinned: [0; 7],
        tau: [0.0; 7],
        tau_envelope: [0.0; 7],
        tau_position: [0.0; 7],
        leash_alteration: 0.0,
    }
}

const THREAD: &str = "franka-joint-target";

pub(super) fn start(
    robot: &Arc<Robot>,
    options: JointTargetControlOptions,
) -> FrankaResult<JointTargetControl> {
    options.validate()?;
    let joint_limits = joint_position_limits(robot.fci_version());
    check_posture(&options.backend, &joint_limits)?;
    let inset = joint_limit_inset(&options.backend);
    let limits = options.limits.unwrap_or_else(|| {
        JointTargetControlOptions::scaled_limits(robot.fci_version(), DEFAULT_LIMIT_FRACTION)
    });
    let shared = Arc::new(Shared::<7>::default());
    let loop_shared = Arc::clone(&shared);
    let scheduling = options.scheduling();
    let inner = match options.backend {
        Backend::RobotController => spawn(
            THREAD,
            robot,
            shared,
            scheduling,
            move |robot: &Robot, started| {
                position_loop(robot, started, options, limits, loop_shared)
            },
        )?,
        Backend::Impedance(impedance) => {
            let model = Arc::new(robot.load_model()?);
            spawn(
                THREAD,
                robot,
                shared,
                scheduling,
                move |robot: &Robot, started| {
                    let limit_rate = options.limit_rate;
                    let version = robot.fci_version();
                    torque_loop(
                        options,
                        limits,
                        impedance,
                        model,
                        version,
                        loop_shared,
                        started,
                    )?
                    .run(robot, limit_rate)
                },
            )?
        }
    };
    Ok(JointTargetControl {
        inner,
        limits: joint_limits,
        inset,
    })
}

/// The [`Backend::Impedance`] loop of this interface: a [`JointTracker`] on the seven-joint
/// runner under `limits`, boxed in the version's joint limits.
pub(super) fn torque_loop(
    options: JointTargetControlOptions,
    limits: [OtgLimits; 7],
    impedance: ImpedanceOptions,
    model: Arc<Model>,
    version: FciVersion,
    shared: Arc<Shared<7>>,
    started: SyncSender<()>,
) -> FrankaResult<TorqueLoop<7, 7, JointTracker>> {
    let runner = Runner::new(shared, started, limits, options.settle, identity)?;
    let velocity = VelocityLimit::of(version);
    let limits = JointLimits {
        position: joint_position_limits(version),
        margin: impedance.joint_position_margin,
        fraction: impedance.joint_velocity_fraction,
        velocity,
    };
    let tracker = JointTracker::new(&options, limits);
    // No live tuning: `LiveTuning`'s budget is a Cartesian norm, which this interface plans
    // nothing with, and the fields of it this tracker would keep a copy of -- the IK's, the
    // feedforward's filter -- it has none of. Its slot stays unseeded and its handle offers no
    // setter, so the loop reads nothing per cycle.
    Ok(TorqueLoop::new(
        runner,
        model,
        impedance,
        tracker,
        options.observer,
        None,
    ))
}

/// [`Backend::RobotController`]: the position stream, re-anchored on the robot's echo every
/// cycle with the rate limiter as the backstop; a stop holds the echo bit for bit.
fn position_loop(
    robot: &Robot,
    started: SyncSender<()>,
    options: JointTargetControlOptions,
    limits: [OtgLimits; 7],
    shared: Arc<Shared<7>>,
) -> FrankaResult<()> {
    let JointTargetControlOptions {
        controller_mode,
        max_deviation,
        settle,
        limit_rate,
        mut observer,
        ..
    } = options;
    let mut runner = Runner::new(shared, started, limits, settle, identity)?;
    let velocity = limits.map(|l| l.max_velocity);
    let acceleration = limits.map(|l| l.max_acceleration);
    let jerk = limits.map(|l| l.max_jerk);
    let mut start: Option<[f64; 7]> = None;
    let result = robot.control_joint_positions(
        |state: &RobotState, _period| {
            let start = *start.get_or_insert(state.q_d);
            let strayed = max_abs_difference(&state.q, &start) > max_deviation;
            let step = runner.cycle(state, state.q_d, strayed, false);

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
                        backstop_alteration,
                        ..sent(&step, q)
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
}
