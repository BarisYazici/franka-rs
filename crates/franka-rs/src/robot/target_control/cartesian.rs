//! The Cartesian interface of [target control](super): a pose target in the base frame,
//! driven through [`Robot::control_cartesian_pose`]. The slot holds the position and a unit
//! quaternion `[x, y, z, w]`; one synchronised six-axis generator runs on the position and
//! on the base-frame rotation vector of the orientation error (see [`chart`]).

use std::sync::Arc;

use nalgebra::Matrix3;

use super::rotation::{
    angle_between, checked_pose, distance, exp, from_quaternion, log, pose_from, rotation_of,
    to_quaternion, translation_of, unit_quaternion,
};
use super::{spawn, validate_common, Handle, Runner, Settle, Shared};
use crate::control_types::{CartesianPose, ControllerMode};
use crate::error::{FrankaError, FrankaResult};
use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
use crate::math_utils::orthonormalized_rotation;
use crate::otg::OtgLimits;
use crate::rate_limiting::{
    limit_rate_cartesian_pose, DELTA_T, FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE,
};
use crate::robot::Robot;
use crate::robot_state::RobotState;

/// What one cycle sent, for the observer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianSent {
    /// The pose that went to the robot (after the backstop), column-major.
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
    /// outran the budget (or without `limit_rate`).
    pub backstop_alteration: f64,
    /// How far the backstop turned the generator's orientation, rad; as above.
    pub backstop_angular_alteration: f64,
}

/// The observer's type: called every cycle on the realtime thread, so it must not allocate
/// or block.
pub type CartesianObserver = Box<dyn FnMut(&RobotState, &CartesianSent) + Send>;

/// Options of [`Robot::start_cartesian_target_control`]; [`Default`] is the documented
/// starting point and the `with_*` methods change one field each.
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
    /// The robot's internal controller. Default [`ControllerMode::CartesianImpedance`].
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
    /// the loop's own libfranka limiter behind it. Default `true`.
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

    /// Sets the robot's internal controller.
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
        validate_common(self.max_deviation, self.settle, self.realtime_priority)
    }
}

/// The handle of a running Cartesian target control; see the [module documentation](super).
/// The target is a pose; orientations are unit quaternions in **`[x, y, z, w]` order** (the
/// scalar part last) or the rotation block of a column-major pose as in `O_T_EE`.
pub struct CartesianTargetControl {
    inner: Handle<7>,
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

/// The Cartesian [`Chart`](super::runner::Chart): the position, re-anchored on the echo's,
/// and the base-frame rotation vector `log(R_target * R_echo^T)` from zero; the runner's
/// rotational step is composed back as `exp(step) * R_echo`.
pub(super) fn chart(target: &[f64; 7], commanded: &[f64; 7]) -> ([f64; 6], [f64; 6]) {
    let (target_position, target_orientation) = split(target);
    let (position, orientation) = split(commanded);
    let r_target = from_quaternion(&target_orientation);
    let r_commanded = from_quaternion(&orientation);
    let error = log(&(r_target * r_commanded.transpose()));
    (
        [position[0], position[1], position[2], 0.0, 0.0, 0.0],
        [
            target_position[0],
            target_position[1],
            target_position[2],
            error[0],
            error[1],
            error[2],
        ],
    )
}

/// The translation and the base-frame rotation vector from the echo to `pose`.
fn increment(pose: &[f64; 16], echo_position: &[f64; 3], echo_rotation: &Matrix3<f64>) -> [f64; 6] {
    let p = translation_of(pose);
    let turn = log(&(rotation_of(pose) * echo_rotation.transpose()));
    [
        p[0] - echo_position[0],
        p[1] - echo_position[1],
        p[2] - echo_position[2],
        turn[0],
        turn[1],
        turn[2],
    ]
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
    (
        [slot[0], slot[1], slot[2]],
        [slot[3], slot[4], slot[5], slot[6]],
    )
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

pub(super) fn start(
    robot: &Arc<Robot>,
    options: TargetControlOptions,
) -> FrankaResult<CartesianTargetControl> {
    options.validate()?;
    let TargetControlOptions {
        limits,
        rotation_limits,
        controller_mode,
        max_deviation,
        max_angular_deviation,
        settle,
        limit_rate,
        realtime_priority,
        mut observer,
    } = options;
    let shared = Arc::new(Shared::<7>::default());
    let loop_shared = Arc::clone(&shared);
    let body = move |robot: &Robot, started| {
        let mut runner = Runner::new(
            loop_shared,
            started,
            axis_limits(limits, rotation_limits),
            settle,
            chart,
        )?;
        let mut start: Option<([f64; 3], Matrix3<f64>)> = None;
        let mut held: Option<[f64; 16]> = None;
        // The backstop's reference: the twist and acceleration of what it sent, not the
        // echoed ones, whose float32 rounding is worth 200 rad/s^3 of jerk on FCI v10.
        let (mut last_twist, mut last_acceleration) = ([0.0; 6], [0.0; 6]);
        let result = robot.control_cartesian_pose(
            |state: &RobotState, _period| {
                // Float32 on FCI v10: orthonormal to 1e-7 only, and composed into the command.
                let echo_rotation = orthonormalized_rotation(&rotation_of(&state.O_T_EE_c));
                let echo_position = translation_of(&state.O_T_EE_c);
                let (start_position, start_rotation) =
                    *start.get_or_insert((echo_position, echo_rotation));
                let strayed = distance(&translation_of(&state.O_T_EE), &start_position)
                    > max_deviation
                    || angle_between(&start_rotation, &rotation_of(&state.O_T_EE))
                        > max_angular_deviation;
                let commanded = slot_values(&echo_position, &echo_rotation);
                let step = runner.cycle(state, commanded, strayed);

                let position = [step.position[0], step.position[1], step.position[2]];
                let turn = [step.position[3], step.position[4], step.position[5]];
                let (mut backstop_alteration, mut backstop_angular_alteration) = (0.0, 0.0);
                let pose = if step.hold {
                    // The robot's echo of the last command, bit for bit, for the whole hold.
                    *held.get_or_insert(state.O_T_EE_c)
                } else {
                    let rotation = exp(&turn) * echo_rotation;
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
                        backstop_angular_alteration =
                            angle_between(&rotation, &rotation_of(&limited));
                        pose = limited;
                    }
                    pose
                };
                let twist = increment(&pose, &echo_position, &echo_rotation).map(|d| d / DELTA_T);
                last_acceleration = std::array::from_fn(|i| (twist[i] - last_twist[i]) / DELTA_T);
                last_twist = twist;
                if let Some(observe) = observer.as_mut() {
                    let (target, target_orientation) = split(&step.target);
                    observe(
                        state,
                        &CartesianSent {
                            pose,
                            orientation: to_quaternion(&rotation_of(&pose)),
                            target,
                            target_orientation,
                            velocity: [step.velocity[0], step.velocity[1], step.velocity[2]],
                            acceleration: [
                                step.acceleration[0],
                                step.acceleration[1],
                                step.acceleration[2],
                            ],
                            angular_velocity: [
                                step.velocity[3],
                                step.velocity[4],
                                step.velocity[5],
                            ],
                            angular_acceleration: [
                                step.acceleration[3],
                                step.acceleration[4],
                                step.acceleration[5],
                            ],
                            backstop_alteration,
                            backstop_angular_alteration,
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
    };
    let inner = spawn(
        "franka-cartesian-target",
        robot,
        shared,
        realtime_priority,
        body,
    )?;
    Ok(CartesianTargetControl { inner })
}
