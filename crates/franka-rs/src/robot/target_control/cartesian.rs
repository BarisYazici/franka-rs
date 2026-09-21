//! The Cartesian interface of [target control](super): a pose target in the base frame. The
//! slot holds the position and a unit quaternion `[x, y, z, w]`; one synchronised six-axis
//! generator runs on the position and on the base-frame rotation vector of the orientation
//! error (see [`chart`]). [`Backend::Impedance`] follows the generator's pose with the
//! differential IK and sends the impedance law's torques; [`Backend::RobotController`]
//! streams the pose through [`Robot::control_cartesian_pose`].

use std::sync::mpsc::SyncSender;
use std::sync::Arc;

use nalgebra::Matrix3;

use super::position::VelocityLimit;
use super::rotation::{
    angle_between, checked_pose, distance, exp, from_quaternion, log, pose_from, rotation_of,
    to_quaternion, translation_of, unit_quaternion,
};
use super::runner::Step;
use super::torque::{PoseTracker, TorqueLoop};
use super::{
    check_posture, joint_position_limits, spawn, Backend, FieldBound, Handle, ImpedanceOptions,
    LiveTuning, Runner, Shared, TargetControlOptions, TuningUpdate, DERIVED_GAINS_MESSAGE,
    NO_TUNING_MESSAGE,
};
use crate::control_types::CartesianPose;
use crate::error::{FrankaError, FrankaResult};
use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
use crate::math_utils::orthonormalized_rotation;
use crate::model::Model;
use crate::otg::OtgLimits;
use crate::rate_limiting::{
    limit_rate_cartesian_pose, DELTA_T, FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE,
};
use crate::robot::Robot;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;

/// What one cycle sent, for the observer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianSent {
    /// The pose that went to the robot (after the backstop), or with [`Backend::Impedance`]
    /// the desired pose the IK follows; column-major. On a cycle the joint velocity cap cut,
    /// the desired pose before the cut: `q_goal` and `dq_goal` are what was commanded.
    pub pose: [f64; 16],
    /// The orientation of `pose` as a unit quaternion, `[x, y, z, w]`.
    pub orientation: [f64; 4],
    /// The target position the generator was planning towards, base frame, m.
    pub target: [f64; 3],
    /// The target orientation the generator was planning towards, `[x, y, z, w]`.
    pub target_orientation: [f64; 4],
    /// The generator's velocity at the end of the cycle, m/s.
    pub velocity: [f64; 3],
    /// The generator's acceleration at the end of the cycle, m/s^2.
    pub acceleration: [f64; 3],
    /// The generator's angular velocity at the end of the cycle, base frame, rad/s.
    pub angular_velocity: [f64; 3],
    /// The generator's angular acceleration at the end of the cycle, base frame, rad/s^2.
    pub angular_acceleration: [f64; 3],
    /// How far the backstop moved the generator's position, m: zero unless the generator
    /// outran the budget (or without `limit_rate`, or with [`Backend::Impedance`]).
    pub backstop_alteration: f64,
    /// How far the backstop turned the generator's orientation, rad; as above.
    pub backstop_angular_alteration: f64,
    /// The joint target of the impedance law, rad; zeros with [`Backend::RobotController`].
    pub q_goal: [f64; 7],
    /// The joint goal's velocity, rad/s: the finite difference of `q_goal` the law feeds
    /// forward with [`ImpedanceOptions::velocity_feedforward`] on, under the joint velocity cap
    /// and after [`ImpedanceOptions::velocity_feedforward_cutoff`]'s low-pass if one is set;
    /// zeros on the first cycle, while holding and with [`Backend::RobotController`]. This is
    /// the value the law and the recorder both see, not the raw difference.
    pub dq_goal: [f64; 7],
    /// The fraction of the generator's step the goal carried on a cycle it fell short and the
    /// generator was restarted from the goal: held at a joint position limit, cut to
    /// [`ImpedanceOptions::joint_velocity_fraction`](super::ImpedanceOptions::joint_velocity_fraction)
    /// of the joint velocity limits, or both. Exactly 1 on every other cycle, and with
    /// [`Backend::RobotController`].
    pub cap_scale: f64,
    /// Per joint, the bound of the IK's box the goal was held on: 0 none, -1 / +1 the lower /
    /// upper position bound (the margin, or the braking toward it), -2 / +2 the velocity bound;
    /// zeros with [`Backend::RobotController`].
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
    /// The position envelope's share of `tau`, Nm, before the clamp: the spring pushing a joint
    /// measured inside the position barrier's onset out, less the law's torque toward a limit
    /// faded out inside
    /// [`ImpedanceOptions::joint_position_margin`](super::ImpedanceOptions::joint_position_margin);
    /// `tau` is the clamped sum of the law, `tau_envelope` and this. Zeros away from the limits
    /// and with [`Backend::RobotController`].
    pub tau_position: [f64; 7],
    /// The IK's residual toward `pose` after this cycle's iterations, the norm of the position
    /// error, m, and the orientation error weighted by
    /// [`IkOptions::rotation_weight`](super::IkOptions::rotation_weight); 0 with
    /// [`Backend::RobotController`].
    pub ik_error: f64,
    /// The IK's active-set passes this cycle, over its iterations; 0 with
    /// [`Backend::RobotController`].
    pub ik_passes: u32,
    /// How hard the residual pushes on the goal's position pins, in the weighted task units of
    /// `ik_error` (m) per cycle: the largest residual along a pinned joint's unit Jacobian
    /// column, counted only into its bound; 0 with [`Backend::RobotController`].
    pub stall_pressure: f64,
    /// Whether the IK's goal is stalled at a joint position limit, the flag on
    /// `stall_pressure` with its hysteresis; while it or a position pin after it holds, the
    /// generator is restarted from the goal every cycle without its velocity into the limit.
    /// Always false with [`Backend::RobotController`].
    pub stalled: bool,
    /// The largest raw step the cycle's IK solves asked for, rad: its norm with the nullspace
    /// bias in and before the per-joint box clip. While `ik_blend` is above 0 each iteration
    /// solves twice, and a position stage's norm is its own, before the blend scales it down.
    /// 0 with [`Backend::RobotController`].
    pub ik_step: f64,
    /// How much of that same solve's `ik_step` the box took off, rad; 0 with
    /// [`Backend::RobotController`].
    pub ik_step_clipped: f64,
    /// The IK's priority blend `β` this cycle: 0 the weighted solve, 1 position first (see
    /// `stalled`); 0 with [`Backend::RobotController`].
    pub ik_blend: f64,
    /// Whether the generator was held at a wall this cycle: the IK stalled, or a wall still
    /// within its life. Always false with [`Backend::RobotController`].
    pub held: bool,
    /// Per block (translation, rotation), cycles since that block was last pushed on a wall,
    /// and -1 when it claims no wall; `[-1, -1]` with [`Backend::RobotController`].
    ///
    /// A block takes a wall only when the free joints' remainder in it exceeds the stall's wall
    /// share of what the pins push, and keeps it for a while after. On this path, where
    /// position first drives the translation residual down to micrometres, the translation age
    /// normally stays -1 and only the rotation's varies.
    pub wall_age: [i8; 2],
    /// How far, m, the leash pulled the generator's anchor back from the previous desired
    /// position toward the measured one: zero while the arm follows, positive while it is
    /// held back ([`Leash`](super::Leash)); 0 with [`Backend::RobotController`].
    ///
    /// What the leash took off, not the lead itself: 0 whenever the pose is inside the leash --
    /// the common case -- so a recording of nothing but zeros says only that the arm kept up.
    /// The leash scales both blocks together, so an orientation beyond its own leash pulls the
    /// position back with it and this reads positive inside the translation leash.
    pub leash_alteration: f64,
    /// The same for the orientation, rad, and zero on the same terms.
    pub leash_angular_alteration: f64,
}

/// The observer's type: called every cycle on the realtime thread, so it must not allocate
/// or block.
pub type CartesianObserver = Box<dyn FnMut(&RobotState, &CartesianSent) + Send>;

/// The handle of a running Cartesian target control; see the [module documentation](super).
/// The target is a pose; orientations are unit quaternions in **`[x, y, z, w]` order** (the
/// scalar part last) or the rotation block of a column-major pose as in `O_T_EE`.
pub struct CartesianTargetControl {
    inner: Handle<7>,
    /// Why this session has no live tuning, `None` when it has: the reason is known at
    /// [`start`] and nowhere else, since an unseeded slot cannot say which of the two it was.
    no_tuning: Option<&'static str>,
}

impl CartesianTargetControl {
    /// Sets the target position, absolute, in the base frame, m; the target orientation
    /// stays what it is. Callable from any thread at any rate; only the latest target counts.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] if a coordinate is not finite,
    /// [`crate::error::FrankaError::InvalidOperation`] with [`super::ENDED_MESSAGE`] once the
    /// loop has ended for any reason.
    pub fn set_position(&self, position_in_base: [f64; 3]) -> FrankaResult<()> {
        self.inner
            .modify_target(|target| target[..3].copy_from_slice(&position_in_base))
    }

    /// Sets the target orientation, absolute, in the base frame, as a unit quaternion in
    /// **`[x, y, z, w]` order**; the target position stays what it is.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] if a component is not finite or the norm
    /// is further than [`super::UNIT_QUATERNION_TOLERANCE`] from one (within it, the
    /// quaternion is normalised), [`crate::error::FrankaError::InvalidOperation`] with
    /// [`super::ENDED_MESSAGE`] once the loop has ended.
    pub fn set_orientation(&self, orientation_xyzw: [f64; 4]) -> FrankaResult<()> {
        let orientation = unit_quaternion(orientation_xyzw)?;
        self.inner
            .modify_target(|target| target[3..].copy_from_slice(&orientation))
    }

    /// Sets the target position, m, and orientation, a unit quaternion in **`[x, y, z, w]`
    /// order** (the scalar part `w` last), both absolute in the base frame.
    ///
    /// # Errors
    /// As [`set_position`](Self::set_position) and [`set_orientation`](Self::set_orientation).
    pub fn set_target(
        &self,
        position_in_base: [f64; 3],
        orientation_xyzw: [f64; 4],
    ) -> FrankaResult<()> {
        let orientation = unit_quaternion(orientation_xyzw)?;
        self.inner.set_target(join(&position_in_base, &orientation))
    }

    /// Sets the target pose, a column-major 4x4 transform in the base frame as `O_T_EE`.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] if an entry is not finite, the last row
    /// is not `[0, 0, 0, 1]` or the rotation block is further than
    /// [`super::ORTHONORMAL_TOLERANCE`] from orthonormal (within it, it is
    /// re-orthonormalised), [`crate::error::FrankaError::InvalidOperation`] with
    /// [`super::ENDED_MESSAGE`] once the loop has ended.
    pub fn set_pose(&self, pose: &[f64; 16]) -> FrankaResult<()> {
        let (position, rotation) = checked_pose(pose)?;
        self.set_target(position, to_quaternion(&rotation))
    }

    /// The latest target position (the start until something sets it).
    pub fn target(&self) -> [f64; 3] {
        split(&self.inner.target()).0
    }

    /// The latest target orientation, `[x, y, z, w]`.
    pub fn target_orientation(&self) -> [f64; 4] {
        split(&self.inner.target()).1
    }

    /// The latest target pose, column-major as `O_T_EE`.
    pub fn target_pose(&self) -> [f64; 16] {
        let (position, orientation) = split(&self.inner.target());
        pose_from(&from_quaternion(&orientation), &position)
    }

    /// The latest robot state the loop received, copied out.
    pub fn state(&self) -> RobotState {
        self.inner.state()
    }

    /// Moves the law's parameters while the loop runs: applies every field `update` carries and
    /// returns the [`bounds`](FieldBound) that had to clamp one, so a caller can show a slider
    /// snapping. Callable from any thread at any rate; the loop reads the result on its next
    /// cycle and crosses to it under each field's [`policy`](FieldBound::policy).
    ///
    /// The update is all or nothing and then clamped into [`LiveTuning::BOUNDS`]; what it does
    /// not carry, it does not touch. A change is never a step in the torque: the fields that
    /// multiply a nonzero state cross over [`SLEW_TAU`](super::SLEW_TAU), and
    /// [`TuningPolicy::remaining`](super::TuningPolicy::remaining) says how much of a crossing
    /// is still to come.
    ///
    /// Nor is it ever a step in the command. [`budget`](LiveTuning::budget) and
    /// [`rotation_budget`](LiveTuning::rotation_budget) reach the generator, whose velocity and
    /// acceleration are raised on the cycle the update arrives and lowered as a ramp at the next
    /// order's limit, because a narrower budget clamps the generator's stored state rather than
    /// re-planning it ([`TuningPolicy::StepUpGateDown`](super::TuningPolicy::StepUpGateDown)).
    /// So a lowered budget takes `(current - target) / rate` seconds to be wholly in force; the
    /// jerks step, either way.
    ///
    /// The command never steps, but lowering the acceleration or jerk while the arm moves
    /// lengthens the stop (toward `v² / 2a`, plus the jerk's ramp), so a near goal is overshot and
    /// returned to. For example, at the default 0.3 m/s (0.17 m/s per axis), an acceleration
    /// dragged from 0.5 to 0.1 m/s² with the goal 3 cm ahead overshoots it by about 20 cm; faster
    /// motion or a lower acceleration overshoots further. Lower the velocity first, apply it, then
    /// lower the acceleration or jerk.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidArgument`] naming the field if a value is not finite
    /// or is a zero that means something other than "softer", with nothing written;
    /// [`crate::error::FrankaError::InvalidOperation`] with [`super::ENDED_MESSAGE`] once the
    /// loop has ended, with [`super::NO_TUNING_MESSAGE`] on a [`Backend::RobotController`]
    /// session, whose tracking is the robot's own, or with [`super::DERIVED_GAINS_MESSAGE`] on
    /// one whose gains a single Cartesian stiffness cannot rebuild.
    pub fn tune(&self, update: &TuningUpdate) -> FrankaResult<Vec<&'static FieldBound>> {
        self.tunable()?;
        self.inner.tune(update)
    }

    /// The tuning targets in force, the session's own options until something tunes them.
    ///
    /// # Errors
    /// [`crate::error::FrankaError::InvalidOperation`] with [`super::NO_TUNING_MESSAGE`] or
    /// [`super::DERIVED_GAINS_MESSAGE`] on a session that has no live tuning, as
    /// [`tune`](Self::tune) describes.
    pub fn tuning(&self) -> FrankaResult<LiveTuning> {
        self.tunable()?;
        self.inner.tuning()
    }

    /// Whether this session has live tuning at all, with why not if it has not.
    fn tunable(&self) -> FrankaResult<()> {
        match self.no_tuning {
            Some(why) => Err(FrankaError::InvalidOperation(why.to_string())),
            None => Ok(()),
        }
    }

    /// Whether the loop is still running; `false` after it ended for any reason.
    pub fn is_running(&self) -> bool {
        self.inner.is_running()
    }

    /// Settles at the current target, finishes the motion, joins the thread and returns the
    /// loop's result: `Ok` for a regular end, [`crate::error::FrankaError::Control`] if the
    /// robot aborted the motion or a deviation guard fired.
    pub fn stop(self) -> FrankaResult<()> {
        self.inner.stop()
    }
}

#[cfg(test)]
impl CartesianTargetControl {
    /// The handle `start` builds on `shared` for a session whose seed came back as `seed`.
    pub(super) fn on(shared: Arc<Shared<7>>, seed: Result<LiveTuning, &'static str>) -> Self {
        CartesianTargetControl {
            inner: Handle {
                shared,
                thread: None,
            },
            no_tuning: seed.err(),
        }
    }
}

/// A position and a rotation: an echo, a measured pose or a desired one.
pub(super) type Placement = ([f64; 3], Matrix3<f64>);

/// The Cartesian [`Chart`](super::runner::Chart): the position, re-anchored on the echo's,
/// and the base-frame rotation vector `log(R_target * R_echo^T)` from zero; the runner's
/// rotational step is composed back as `exp(step) * R_echo`.
pub(super) fn chart(target: &[f64; 7], commanded: &[f64; 7]) -> ([f64; 6], [f64; 6]) {
    let ([tx, ty, tz], target_orientation) = split(target);
    let ([x, y, z], orientation) = split(commanded);
    let r_target = from_quaternion(&target_orientation);
    let r_commanded = from_quaternion(&orientation);
    let [a, b, c] = log(&(r_target * r_commanded.transpose()));
    ([x, y, z, 0.0, 0.0, 0.0], [tx, ty, tz, a, b, c])
}

/// The translation and the orthonormalised rotation of a pose (float32 on FCI v10:
/// orthonormal to 1e-7 only, and composed into the next command).
pub(super) fn placement(pose: &[f64; 16]) -> Placement {
    (
        translation_of(pose),
        orthonormalized_rotation(&rotation_of(pose)),
    )
}

/// The column-major pose of a placement.
pub(super) fn pose_of(placement: &Placement) -> [f64; 16] {
    pose_from(&placement.1, &placement.0)
}

/// The pose a step lands on: its position, and its rotational step composed onto `rotation`,
/// the one the runner was anchored on.
pub(super) fn compose(step: &Step<6, 7>, rotation: &Matrix3<f64>) -> Placement {
    let [x, y, z, a, b, c] = step.position;
    ([x, y, z], exp(&[a, b, c]) * rotation)
}

/// Whether the measured pose is further from `start` than either guard allows.
pub(super) fn strayed(
    state: &RobotState,
    start: &Placement,
    max_deviation: f64,
    max_angular: f64,
) -> bool {
    distance(&translation_of(&state.O_T_EE), &start.0) > max_deviation
        || angle_between(&start.1, &rotation_of(&state.O_T_EE)) > max_angular
}

/// The observer's record of a step with `pose` as sent; the backend's own fields stay zero.
pub(super) fn sent(step: &Step<6, 7>, pose: [f64; 16]) -> CartesianSent {
    let [vx, vy, vz, wx, wy, wz] = step.velocity;
    let [ax, ay, az, bx, by, bz] = step.acceleration;
    let (target, target_orientation) = split(&step.target);
    CartesianSent {
        pose,
        orientation: to_quaternion(&rotation_of(&pose)),
        target,
        target_orientation,
        velocity: [vx, vy, vz],
        acceleration: [ax, ay, az],
        angular_velocity: [wx, wy, wz],
        angular_acceleration: [bx, by, bz],
        backstop_alteration: 0.0,
        backstop_angular_alteration: 0.0,
        q_goal: [0.0; 7],
        dq_goal: [0.0; 7],
        cap_scale: 1.0,
        pinned: [0; 7],
        tau: [0.0; 7],
        tau_envelope: [0.0; 7],
        tau_position: [0.0; 7],
        ik_error: 0.0,
        ik_passes: 0,
        stall_pressure: 0.0,
        stalled: false,
        ik_step: 0.0,
        ik_step_clipped: 0.0,
        ik_blend: 0.0,
        held: false,
        wall_age: [-1, -1],
        leash_alteration: 0.0,
        leash_angular_alteration: 0.0,
    }
}

/// The translation and the base-frame rotation vector from the echo to `pose`.
fn increment(pose: &[f64; 16], echo: &Placement) -> [f64; 6] {
    let [x, y, z] = translation_of(pose);
    let [a, b, c] = log(&(rotation_of(pose) * echo.1.transpose()));
    [x - echo.0[0], y - echo.0[1], z - echo.0[2], a, b, c]
}

/// The seven slot values of a pose: its translation and the quaternion of its rotation.
pub(super) fn slot_values(translation: &[f64; 3], rotation: &Matrix3<f64>) -> [f64; 7] {
    join(translation, &to_quaternion(rotation))
}

fn join(position: &[f64; 3], orientation: &[f64; 4]) -> [f64; 7] {
    let mut slot = [0.0; 7];
    slot[..3].copy_from_slice(position);
    slot[3..].copy_from_slice(orientation);
    slot
}

fn split(slot: &[f64; 7]) -> ([f64; 3], [f64; 4]) {
    let [x, y, z, a, b, c, d] = *slot;
    ([x, y, z], [a, b, c, d])
}

/// The per-axis limits of the six generator axes; the rotational ones carry the backstop's
/// 0.99 factor, or it binds on every synchronised jerk and the two orbit instead of landing.
pub(super) fn axis_limits(limits: OtgLimits, rotation_limits: OtgLimits) -> [OtgLimits; 6] {
    let translational = limits.per_axis_for_norm(3);
    let rotational = rotation_limits
        .scaled(FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE)
        .per_axis_for_norm(3);
    [
        translational,
        translational,
        translational,
        rotational,
        rotational,
        rotational,
    ]
}

const THREAD: &str = "franka-cartesian-target";

pub(super) fn start(
    robot: &Arc<Robot>,
    options: TargetControlOptions,
) -> FrankaResult<CartesianTargetControl> {
    options.validate()?;
    check_posture(
        &options.backend,
        &joint_position_limits(robot.fci_version()),
    )?;
    let shared = Arc::new(Shared::<7>::default());
    let loop_shared = Arc::clone(&shared);
    let scheduling = options.scheduling();
    let tuning = match &options.backend {
        Backend::Impedance(impedance) => seed(&options, impedance),
        Backend::RobotController => Err(NO_TUNING_MESSAGE),
    };
    let inner = match options.backend {
        Backend::RobotController => spawn(
            THREAD,
            robot,
            shared,
            scheduling,
            move |robot: &Robot, started| pose_loop(robot, started, options, loop_shared),
        )?,
        Backend::Impedance(impedance) => {
            let model = Arc::new(robot.load_model()?);
            // Before the thread, so the loop's first read of the slot is the seed the loop was
            // built with and no cycle runs with anything else. The user thread is the slot's
            // only writer for the session's whole life; the loop never writes it. A seed that
            // is not faithful is not published at all, and the session then has no live tuning.
            if let Ok(seed) = tuning {
                shared.tuning.publish(seed.to_words());
            }
            spawn(
                THREAD,
                robot,
                shared,
                scheduling,
                move |robot: &Robot, started| {
                    let limit_rate = options.limit_rate;
                    let version = robot.fci_version();
                    torque_loop(options, impedance, model, version, loop_shared, started)?
                        .run(robot, limit_rate)
                },
            )?
        }
    };
    Ok(CartesianTargetControl {
        inner,
        no_tuning: tuning.err(),
    })
}

/// What the session's live tuning starts at -- the law as `impedance` has it and the plan's two
/// budgets -- or why it has none. The loop and the slot are seeded from this one place, so the
/// loop's first read of the slot finds exactly what it already holds and nothing is applied
/// until something is tuned.
///
/// A seed is refused when [`LiveTuning::gains`] does not rebuild `impedance`'s own gains. The
/// operator's Cartesian stiffness is one number and the law's are twelve, derived from it by
/// [`ImpedanceGains::scaled_cartesian`](super::ImpedanceGains::scaled_cartesian), and the apply
/// step rebuilds all twelve whenever *any* field moves. On a gains set that derivation cannot
/// reproduce -- [`DROID`](super::ImpedanceGains::DROID), whose translational damping is 37
/// against the scaled preset's 50 and 90 -- the first retune of an unrelated field would rewrite
/// them in a single cycle, and the slew could not soften it, because the stiffness it watches
/// never moved. A session `LiveTuning` cannot represent is left with no live tuning rather than
/// quietly retuned into one it can.
pub(super) fn seed(
    options: &TargetControlOptions,
    impedance: &ImpedanceOptions,
) -> Result<LiveTuning, &'static str> {
    let seed = LiveTuning::from_options(impedance, options.limits, options.rotation_limits);
    (seed.gains() == impedance.gains)
        .then_some(seed)
        .ok_or(DERIVED_GAINS_MESSAGE)
}

/// The [`Backend::Impedance`] loop of this interface: a [`PoseTracker`] on the six-axis
/// runner, over the negotiated version's joint limits.
pub(super) fn torque_loop(
    options: TargetControlOptions,
    impedance: ImpedanceOptions,
    model: Arc<Model>,
    version: FciVersion,
    shared: Arc<Shared<7>>,
    started: SyncSender<()>,
) -> FrankaResult<TorqueLoop<6, 7, PoseTracker>> {
    let runner = Runner::new(
        shared,
        started,
        axis_limits(options.limits, options.rotation_limits),
        options.settle,
        chart,
    )?;
    let velocity = VelocityLimit::of(version);
    let tracker = PoseTracker::new(
        &options,
        &impedance,
        Arc::clone(&model),
        joint_position_limits(version),
        velocity,
    );
    // The loop reads the slot only where `start` seeded it, and the two agree because both ask
    // the same question of the same options.
    let tuning = seed(&options, &impedance).ok();
    Ok(TorqueLoop::new(
        runner,
        model,
        impedance,
        tracker,
        options.observer,
        tuning,
    ))
}

/// [`Backend::RobotController`]: the pose stream, re-anchored on the robot's echo every cycle
/// with the rate limiter as the backstop; a stop holds the echo bit for bit.
fn pose_loop(
    robot: &Robot,
    started: SyncSender<()>,
    options: TargetControlOptions,
    shared: Arc<Shared<7>>,
) -> FrankaResult<()> {
    let TargetControlOptions {
        limits,
        rotation_limits,
        controller_mode,
        max_deviation,
        max_angular_deviation,
        settle,
        limit_rate,
        mut observer,
        ..
    } = options;
    let mut runner = Runner::new(
        shared,
        started,
        axis_limits(limits, rotation_limits),
        settle,
        chart,
    )?;
    let mut start: Option<Placement> = None;
    let mut held: Option<[f64; 16]> = None;
    // The backstop's reference: the twist and acceleration of what it sent, not the echoed
    // ones, whose float32 rounding is worth 200 rad/s^3 of jerk on FCI v10.
    let (mut last_twist, mut last_acceleration) = ([0.0; 6], [0.0; 6]);
    let result = robot.control_cartesian_pose(
        |state: &RobotState, _period| {
            let echo = placement(&state.O_T_EE_c);
            let start = *start.get_or_insert(echo);
            let strayed = strayed(state, &start, max_deviation, max_angular_deviation);
            let step = runner.cycle(state, slot_values(&echo.0, &echo.1), strayed, false);

            let (mut backstop_alteration, mut backstop_angular_alteration) = (0.0, 0.0);
            let pose = if step.hold {
                // The robot's echo of the last command, bit for bit, for the whole hold.
                *held.get_or_insert(state.O_T_EE_c)
            } else {
                let (position, rotation) = compose(&step, &echo.1);
                let mut pose = pose_from(&rotation, &position);
                if limit_rate {
                    // It only fails on non-finite input; the echo is then the safe command.
                    #[rustfmt::skip]
                    let limited = limit_rate_cartesian_pose(
                        limits.max_velocity, limits.max_acceleration, limits.max_jerk,
                        rotation_limits.max_velocity, rotation_limits.max_acceleration,
                        rotation_limits.max_jerk,
                        &pose, &state.O_T_EE_c, &last_twist, &last_acceleration,
                    ).unwrap_or(state.O_T_EE_c);
                    backstop_alteration = distance(&translation_of(&limited), &position);
                    backstop_angular_alteration = angle_between(&rotation, &rotation_of(&limited));
                    pose = limited;
                }
                pose
            };
            let twist = increment(&pose, &echo).map(|d| d / DELTA_T);
            last_acceleration = std::array::from_fn(|i| (twist[i] - last_twist[i]) / DELTA_T);
            last_twist = twist;
            if let Some(observe) = observer.as_mut() {
                observe(
                    state,
                    &CartesianSent {
                        backstop_alteration,
                        backstop_angular_alteration,
                        ..sent(&step, pose)
                    },
                );
            }
            let mut output = CartesianPose::new(pose);
            output.motion_finished = step.finished;
            output
        },
        controller_mode,
        limit_rate,
        MAX_CUTOFF_FREQUENCY,
    );
    runner.finish(result)
}
