//! The loop itself: `ControlLoop::loop`, its three libfranka constructors, the
//! cancel-on-drop guard and the torque-only entry point.

use super::*;
use crate::control_types::{ControllerMode, Finishable, JointPositions, JointVelocities};
use crate::robot::robot_impl::DEFAULT_DEVIATION;
use crate::wire::robot::{ControllerCommand, Deviation, MoveControllerMode};

/// A running control loop.
///
/// The callbacks are borrowed as trait objects so that the three libfranka constructors
/// (motion + controller, motion + controller mode, controller only) collapse into one type
/// without a generic explosion; the indirection costs one virtual call per cycle and never
/// allocates.
pub(crate) struct ControlLoop<'a, M: ControlLoopMotion> {
    robot: &'a RobotImpl,
    motion_callback: Option<MotionCallback<'a, M>>,
    control_callback: Option<ControlCallback<'a>>,
    limit_rate: bool,
    cutoff_frequency: f64,
    /// The negotiated version's rate-limiting envelope, resolved once at construction.
    rate: RateLimits,
    /// The negotiated FCI version, which also decides the order of the two callbacks and the
    /// first-cycle reference rule.
    version: FciVersion,
    motion_id: u32,
    initialized_filter: bool,
    /// Set once the loop has ended regularly; while it is false [`Drop`] cancels the motion.
    finished: bool,
}

impl<'a, M: ControlLoopMotion> ControlLoop<'a, M> {
    /// External controller plus motion generator (libfranka's first `ControlLoop` constructor).
    pub(crate) fn new_with_control_and_motion(
        robot: &'a RobotImpl,
        control_callback: ControlCallback<'a>,
        motion_callback: MotionCallback<'a, M>,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<Self> {
        let motion_id = start_motion(robot, MoveControllerMode::ExternalController, M::MOVE_MODE)?;
        Ok(ControlLoop {
            robot,
            motion_callback: Some(motion_callback),
            control_callback: Some(control_callback),
            limit_rate,
            cutoff_frequency,
            rate: rate_limits(robot.version()),
            version: robot.version(),
            motion_id,
            initialized_filter: false,
            finished: false,
        })
    }

    /// Motion generator with one of the built-in controllers (third C++ constructor).
    pub(crate) fn new_with_motion(
        robot: &'a RobotImpl,
        controller_mode: ControllerMode,
        motion_callback: MotionCallback<'a, M>,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<Self> {
        let mode = match controller_mode {
            ControllerMode::JointImpedance => MoveControllerMode::JointImpedance,
            ControllerMode::CartesianImpedance => MoveControllerMode::CartesianImpedance,
        };
        let motion_id = start_motion(robot, mode, M::MOVE_MODE)?;
        Ok(ControlLoop {
            robot,
            motion_callback: Some(motion_callback),
            control_callback: None,
            limit_rate,
            cutoff_frequency,
            rate: rate_limits(robot.version()),
            version: robot.version(),
            motion_id,
            initialized_filter: false,
            finished: false,
        })
    }

    /// External controller only (second C++ constructor), FCI v10 only.
    ///
    /// The type parameter is unused in this mode; [`control_torques`] instantiates it with
    /// [`JointPositions`] so that no motion command is ever built, exactly like the C++
    /// `ControlLoop<Torques>` whose `convertMotion` throws if it is ever reached.
    ///
    /// FCI v5 has no `kNone` motion generator mode, so [`control_torques`] does not use this
    /// constructor there.
    pub(crate) fn new_with_control(
        robot: &'a RobotImpl,
        control_callback: ControlCallback<'a>,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<Self> {
        let motion_id = start_motion(
            robot,
            MoveControllerMode::ExternalController,
            MoveMotionGeneratorMode::None,
        )?;
        Ok(ControlLoop {
            robot,
            motion_callback: None,
            control_callback: Some(control_callback),
            limit_rate,
            cutoff_frequency,
            rate: rate_limits(robot.version()),
            version: robot.version(),
            motion_id,
            initialized_filter: false,
            finished: false,
        })
    }

    /// Runs the loop to completion (`ControlLoop::loop`).
    ///
    /// Anything but a regular end cancels the motion — the C++ function-try-block around
    /// `loop()` — which [`Drop`] takes care of, so a panic unwinding out of a user callback
    /// cancels the motion just like a thrown exception does in C++.
    pub(crate) fn run(mut self) -> FrankaResult<()> {
        let result = self.run_inner();
        if result.is_ok() {
            self.finished = true;
        }
        result
    }

    fn run_inner(&mut self) -> FrankaResult<()> {
        let mut robot_state = self.robot.update_motion(None, None)?;
        self.robot
            .throw_on_motion_error(&robot_state, self.motion_id)?;

        let mut control_command = self
            .control_callback
            .is_some()
            .then(ControllerCommand::default);
        let mut motion_command = self
            .motion_callback
            .is_some()
            .then(MotionGeneratorCommand::default);

        // libfranka 0.9.2 runs `while (spinMotion(...) && spinControl(...))`
        // (`src/control_loop.cpp:133-138`), i.e. the *motion* callback first and the controller
        // short-circuited; 0.21.2 swapped them (`src/control_loop.cpp:119-129`). The order
        // decides which cycle's commands `finishMotion` sends, so each version keeps its own.
        let motion_first = self.version == FciVersion::V5;

        let mut previous_time = robot_state.time;
        loop {
            let time_step = robot_state.time - previous_time;

            // `&&` short-circuits exactly like the C++ `while` condition: whichever callback
            // runs second is skipped in the cycle where the first one reports finished.
            let is_not_finished = if motion_first {
                let motion_continues =
                    self.spin_motion(&robot_state, time_step, &mut motion_command)?;
                motion_continues
                    && self.spin_control(&robot_state, time_step, &mut control_command)?
            } else {
                let control_continues =
                    self.spin_control(&robot_state, time_step, &mut control_command)?;
                control_continues
                    && self.spin_motion(&robot_state, time_step, &mut motion_command)?
            };

            if !is_not_finished {
                break;
            }

            previous_time = robot_state.time;
            robot_state = self
                .robot
                .update_motion(motion_command.as_ref(), control_command.as_ref())?;
            self.robot
                .throw_on_motion_error(&robot_state, self.motion_id)?;
        }

        self.robot.finish_motion(
            self.motion_id,
            motion_command.as_ref(),
            control_command.as_ref(),
        )
    }

    /// `ControlLoop::spinControl`: converts the controller callback's output into `command` and
    /// returns `false` once the controller reports `motion_finished`.
    ///
    /// Returns `true` unchanged when this loop has no controller callback, so the caller can
    /// chain the two spins with `&&` exactly as the C++ `while` condition does.
    fn spin_control(
        &mut self,
        state: &RobotState,
        time_step: Duration,
        command: &mut Option<ControllerCommand>,
    ) -> FrankaResult<bool> {
        let (rate, limit_rate, cutoff_frequency) =
            (self.rate, self.limit_rate, self.cutoff_frequency);
        let (Some(callback), Some(command)) = (self.control_callback.as_mut(), command.as_mut())
        else {
            return Ok(true);
        };
        let output = callback(state, time_step);
        convert_torques(&output, state, &rate, limit_rate, cutoff_frequency, command)?;
        Ok(!output.is_finished())
    }

    /// `ControlLoop::spinMotion`: converts the motion callback's output into `command` and
    /// returns `false` once the motion generator reports `motion_finished`.
    ///
    /// Returns `true` unchanged when this loop has no motion callback (FCI v10 torque-only
    /// control), for the same reason as [`ControlLoop::spin_control`].
    fn spin_motion(
        &mut self,
        state: &RobotState,
        time_step: Duration,
        command: &mut Option<MotionGeneratorCommand>,
    ) -> FrankaResult<bool> {
        let mut context = ConvertContext {
            limits: self.robot,
            rate: self.rate,
            limit_rate: self.limit_rate,
            cutoff_frequency: self.cutoff_frequency,
            initialized_filter: self.initialized_filter,
            first_command_is_its_own_reference: self.version == FciVersion::V10,
        };
        let (Some(callback), Some(command)) = (self.motion_callback.as_mut(), command.as_mut())
        else {
            return Ok(true);
        };
        let output = callback(state, time_step);
        let converted = output.convert(&mut context, state, command);
        self.initialized_filter = context.initialized_filter;
        converted?;
        Ok(!output.is_finished())
    }
}

impl<M: ControlLoopMotion> Drop for ControlLoop<'_, M> {
    /// Cancels the motion unless the loop ended regularly.
    ///
    /// This is the `catch (...) { robot_.cancelMotion(motion_id_); throw; }` of
    /// `control_loop.cpp:143-149`, and unlike a plain error path it also fires when a user
    /// callback panics. The cancellation's own failure is swallowed, as in the C++.
    fn drop(&mut self) {
        if !self.finished {
            let _ = self.robot.cancel_motion(self.motion_id);
        }
    }
}

/// `RobotControl::startMotion` with libfranka's default deviations.
fn start_motion(
    robot: &RobotImpl,
    controller_mode: MoveControllerMode,
    motion_generator_mode: MoveMotionGeneratorMode,
) -> FrankaResult<u32> {
    let deviation = Deviation::new(
        DEFAULT_DEVIATION.0,
        DEFAULT_DEVIATION.1,
        DEFAULT_DEVIATION.2,
    );
    robot.start_motion(controller_mode, motion_generator_mode, deviation, deviation)
}

/// Runs a torque-only control loop (`Robot::control(control_callback, ...)`).
///
/// On FCI v10 this is libfranka 0.21.2's `ControlLoop<Torques>`: `Move` with
/// `kExternalController` and `kNone`, and no motion command on the wire.
///
/// FCI v5 has no `kNone`. libfranka 0.9.2 (`src/robot.cpp:41-57`) instead builds a
/// `ControlLoop<JointVelocities>` whose motion callback returns all-zero velocities, so an FER
/// runs a joint-velocity motion generator alongside the external controller and every cycle
/// carries a zero `dq_c` next to `tau_J_d`. This is that port; the zero velocities go through
/// the same filter and rate limiter as a user's, exactly as in the C++.
pub(crate) fn control_torques(
    robot: &RobotImpl,
    control_callback: ControlCallback<'_>,
    limit_rate: bool,
    cutoff_frequency: f64,
) -> FrankaResult<()> {
    match robot.version() {
        FciVersion::V5 => {
            let mut zero_velocities = |_: &RobotState, _: Duration| JointVelocities::new([0.0; 7]);
            let loop_ = ControlLoop::<JointVelocities>::new_with_control_and_motion(
                robot,
                control_callback,
                &mut zero_velocities,
                limit_rate,
                cutoff_frequency,
            )?;
            loop_.run()
        }
        FciVersion::V10 => ControlLoop::<JointPositions>::new_with_control(
            robot,
            control_callback,
            limit_rate,
            cutoff_frequency,
        )?
        .run(),
    }
}
