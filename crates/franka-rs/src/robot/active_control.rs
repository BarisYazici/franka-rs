//! Externally driven control: `read_once` / `write_once` instead of a callback loop.
//!
//! Port of `franka::ActiveControl`, `franka::ActiveTorqueControl` and
//! `franka::ActiveMotionGenerator` (libfranka 0.21.2 `src/active_control.cpp`,
//! `src/active_torque_control.cpp`, `src/active_motion_generator.cpp`).
//!
//! Neither rate limiting nor low-pass filtering is applied here — exactly like libfranka, the
//! caller is responsible for sending smooth setpoints.
//!
//! Both types hold the robot's control lock for their whole lifetime, so no other control or
//! read operation can run in parallel, and both cancel an unfinished motion when dropped.

use std::sync::MutexGuard;

use crate::control_types::{
    CartesianPose, CartesianVelocities, Finishable, JointPositions, JointVelocities,
    MotionGenerator, Torques,
};
use crate::duration::Duration;
#[allow(unused_imports)] // `FrankaError` is referenced by the `# Errors` doc sections below.
use crate::error::{FrankaError, FrankaResult};
use crate::robot::robot_impl::{control_error, time_since, RobotImpl};
use crate::robot_state::RobotState;
use crate::wire::f64s_to_wire;
use crate::wire::robot::{ControllerCommand, MotionGeneratorCommand, MoveControllerMode};

/// Motion command types that can drive an [`ActiveMotionGenerator`].
///
/// Implemented for exactly the four `franka::MotionGenerator` command types; the method is the
/// Rust equivalent of the `Robot::Impl::createMotionCommand` overload set, which validates the
/// user's values before they are put on the wire.
pub trait ActiveMotionInput: MotionGenerator {
    /// Validates the command and converts it into the wire motion command.
    fn create_motion_command(&self) -> FrankaResult<MotionGeneratorCommand>;
}

impl ActiveMotionInput for JointPositions {
    fn create_motion_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        self.validate()?;
        Ok(MotionGeneratorCommand {
            q_c: f64s_to_wire(&self.q),
            ..MotionGeneratorCommand::default()
        })
    }
}

impl ActiveMotionInput for JointVelocities {
    fn create_motion_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        self.validate()?;
        Ok(MotionGeneratorCommand {
            dq_c: f64s_to_wire(&self.dq),
            ..MotionGeneratorCommand::default()
        })
    }
}

impl ActiveMotionInput for CartesianPose {
    fn create_motion_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        self.validate()?;
        Ok(MotionGeneratorCommand {
            O_T_EE_c: f64s_to_wire(&self.O_T_EE),
            valid_elbow: u8::from(self.has_elbow),
            elbow_c: f64s_to_wire(if self.has_elbow {
                &self.elbow
            } else {
                &[0.0; 2]
            }),
            ..MotionGeneratorCommand::default()
        })
    }
}

impl ActiveMotionInput for CartesianVelocities {
    fn create_motion_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        self.validate()?;
        Ok(MotionGeneratorCommand {
            O_dP_EE_c: f64s_to_wire(&self.O_dP_EE),
            valid_elbow: u8::from(self.has_elbow),
            elbow_c: f64s_to_wire(if self.has_elbow {
                &self.elbow
            } else {
                &[0.0; 2]
            }),
            ..MotionGeneratorCommand::default()
        })
    }
}

/// Validates a torque command and converts it (`Robot::Impl::createControllerCommand`).
pub(crate) fn create_controller_command(
    control_input: &Torques,
) -> FrankaResult<ControllerCommand> {
    control_input.validate()?;
    Ok(ControllerCommand {
        tau_J_d: f64s_to_wire(&control_input.tau_J),
        torque_command_finished: 0,
    })
}

/// State shared by both active-control types (the C++ `ActiveControl` base class).
struct ActiveControl<'a> {
    robot: &'a RobotImpl,
    motion_id: u32,
    lock: Option<MutexGuard<'a, ()>>,
    control_finished: bool,
    last_read_access: Option<Duration>,
}

impl<'a> ActiveControl<'a> {
    fn new(robot: &'a RobotImpl, motion_id: u32, lock: MutexGuard<'a, ()>) -> ActiveControl<'a> {
        ActiveControl {
            robot,
            motion_id,
            lock: Some(lock),
            control_finished: false,
            last_read_access: None,
        }
    }

    /// `ActiveControl::readOnce`.
    fn read_once(&mut self) -> FrankaResult<(RobotState, Duration)> {
        let robot_state = self.robot.read_once()?;
        self.robot
            .throw_on_motion_error(&robot_state, self.motion_id)?;

        let time_since_last_read = time_since(self.last_read_access, robot_state.time);
        self.last_read_access = Some(robot_state.time);

        Ok((robot_state, time_since_last_read))
    }

    /// Marks the control as finished and releases the robot's control lock.
    ///
    /// Only called after a *successful* `finish_motion`: `active_torque_control.cpp:61-64` and
    /// `active_motion_generator.cpp:112-123` set `control_finished` after `finishMotion`
    /// returns, so a failing finish leaves the flag false and the destructor sends the
    /// `StopMove`. See [`ActiveControl::release_lock`] for the failing path.
    fn finish(&mut self) {
        self.control_finished = true;
        self.lock = None;
    }

    /// Releases the robot's control lock without marking the control as finished, so that
    /// [`Drop`] still cancels the motion.
    fn release_lock(&mut self) {
        self.lock = None;
    }

    /// Applies the outcome of a `finish_motion`: on success the control is over, on failure the
    /// motion is still considered running and the drop guard cancels it.
    fn apply_finish(&mut self, result: FrankaResult<()>) -> FrankaResult<()> {
        match result {
            Ok(()) => {
                self.finish();
                Ok(())
            }
            Err(error) => {
                self.release_lock();
                Err(error)
            }
        }
    }

    fn check_not_finished(&self) -> FrankaResult<()> {
        if self.control_finished {
            return Err(control_error(
                "writeOnce must not be called after the motion has finished.",
            ));
        }
        Ok(())
    }
}

impl Drop for ActiveControl<'_> {
    /// `ActiveControl::~ActiveControl`: an unfinished motion is cancelled.
    fn drop(&mut self) {
        if !self.control_finished {
            let _ = self.robot.cancel_motion(self.motion_id);
        }
    }
}

/// An external torque controller driven by the caller
/// (`Robot::start_torque_control`).
///
/// # FCI v5
/// An FER has no torque-only motion generator mode, so the motion is started with a joint
/// velocity generator and every [`ActiveTorqueControl::write_once`] sends an all-zero
/// `dq_c` alongside the torques, ending with `motion_generation_finished` instead of
/// `torque_command_finished`. That is exactly what libfranka 0.9.2's
/// `Robot::control(control_callback, ...)` does (`src/robot.cpp:41-57`); the API is
/// unchanged.
///
/// # Example
/// ```no_run
/// # use franka::{Robot, RealtimeConfig, Torques};
/// # fn main() -> franka::FrankaResult<()> {
/// let robot = Robot::new("192.168.0.1", RealtimeConfig::Ignore)?;
/// let mut control = robot.start_torque_control()?;
/// for _ in 0..1000 {
///     let (_state, _period) = control.read_once()?;
///     control.write_once(&Torques::new([0.0; 7]))?;
/// }
/// control.write_once(&franka::motion_finished(Torques::new([0.0; 7])))?;
/// # Ok(())
/// # }
/// ```
///
/// # Threading
/// Unlike [`crate::Robot`], which is `Send + Sync`, this type is **`!Send`**: it holds the
/// [`std::sync::MutexGuard`] for the robot's control lock (`franka::assertOwningLock`) for as
/// long as the motion runs, and a `MutexGuard` may not cross threads. Read and write the motion
/// from the thread that started it; a [`crate::Robot::stop`] from another thread still works,
/// because it goes through the `Arc<Robot>` and not through this handle.
pub struct ActiveTorqueControl<'a> {
    inner: ActiveControl<'a>,
}

impl<'a> ActiveTorqueControl<'a> {
    pub(crate) fn new(
        robot: &'a RobotImpl,
        motion_id: u32,
        lock: MutexGuard<'a, ()>,
    ) -> ActiveTorqueControl<'a> {
        ActiveTorqueControl {
            inner: ActiveControl::new(robot, motion_id, lock),
        }
    }

    /// Waits for the next robot state and returns it together with the time elapsed since the
    /// previous `read_once` (zero on the first call).
    ///
    /// # Errors
    /// [`FrankaError::Control`] if the motion was aborted, [`FrankaError::Network`] if the
    /// connection was lost.
    pub fn read_once(&mut self) -> FrankaResult<(RobotState, Duration)> {
        self.inner.read_once()
    }

    /// Sends the given torques.
    ///
    /// Setting `motion_finished` on `control_input` ends the control process; the control lock
    /// is released and any further call fails.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if the motion has already finished or was aborted,
    /// [`FrankaError::InvalidArgument`] if a torque is NaN or infinite,
    /// [`FrankaError::Network`] if the connection was lost.
    pub fn write_once(&mut self, control_input: &Torques) -> FrankaResult<()> {
        self.inner.check_not_finished()?;

        let control_command = create_controller_command(control_input)?;
        if control_input.is_finished() {
            let result = self
                .inner
                .robot
                .finish_torque_only(self.inner.motion_id, &control_command);
            return self.inner.apply_finish(result);
        }

        self.inner
            .robot
            .network()
            .tcp
            .throw_if_connection_closed()?;
        self.inner.robot.write_once_torque_only(&control_command)
    }
}

/// An externally driven motion generator (`Robot::start_*_control`).
///
/// `T` is one of [`JointPositions`], [`JointVelocities`], [`CartesianPose`] or
/// [`CartesianVelocities`].
///
/// # Threading
/// Unlike [`crate::Robot`], which is `Send + Sync`, this type is **`!Send`**: it holds the
/// [`std::sync::MutexGuard`] for the robot's control lock (`franka::assertOwningLock`) for as
/// long as the motion runs, and a `MutexGuard` may not cross threads. Read and write the motion
/// from the thread that started it; a [`crate::Robot::stop`] from another thread still works,
/// because it goes through the `Arc<Robot>` and not through this handle.
pub struct ActiveMotionGenerator<'a, T: ActiveMotionInput> {
    inner: ActiveControl<'a>,
    controller_type: MoveControllerMode,
    marker: std::marker::PhantomData<T>,
}

impl<'a, T: ActiveMotionInput> ActiveMotionGenerator<'a, T> {
    pub(crate) fn new(
        robot: &'a RobotImpl,
        motion_id: u32,
        lock: MutexGuard<'a, ()>,
        controller_type: MoveControllerMode,
    ) -> ActiveMotionGenerator<'a, T> {
        ActiveMotionGenerator {
            inner: ActiveControl::new(robot, motion_id, lock),
            controller_type,
            marker: std::marker::PhantomData,
        }
    }

    /// Waits for the next robot state and returns it together with the time elapsed since the
    /// previous `read_once` (zero on the first call).
    pub fn read_once(&mut self) -> FrankaResult<(RobotState, Duration)> {
        self.inner.read_once()
    }

    /// Sends the next motion setpoint, and the torques when the motion runs with an external
    /// controller.
    ///
    /// `control_input` must be `Some` if and only if the motion was started with
    /// `MoveControllerMode::ExternalController`. Setting `motion_finished` on either input ends
    /// the control process.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if the motion has already finished, if torques are given
    /// without an external controller (or missing with one), or if the motion was aborted;
    /// [`FrankaError::InvalidArgument`] for non-finite values, an invalid transformation matrix
    /// or an invalid elbow configuration.
    pub fn write_once(
        &mut self,
        motion_input: &T,
        control_input: Option<&Torques>,
    ) -> FrankaResult<()> {
        self.inner.check_not_finished()?;

        let external = self.controller_type == MoveControllerMode::ExternalController;
        if control_input.is_some() && !external {
            return Err(control_error(
                "Torques can only be commanded in kExternalController mode.",
            ));
        }
        if control_input.is_none() && external {
            return Err(control_error(
                "Torque command missing, please use writeOnce(const MotionGeneratorType& \
                 motion_generator_input, const Torques& control_input) for external controllers.",
            ));
        }

        let torque_finished = control_input.map(|t| t.is_finished()).unwrap_or(false);
        if motion_input.is_finished() || torque_finished {
            let motion_command = motion_input.create_motion_command()?;
            let result = match control_input {
                None => self.inner.robot.finish_motion(
                    self.inner.motion_id,
                    Some(&motion_command),
                    None,
                ),
                Some(control_input) => {
                    let control_command = create_controller_command(control_input)?;
                    self.inner.robot.finish_motion(
                        self.inner.motion_id,
                        Some(&motion_command),
                        Some(&control_command),
                    )
                }
            };
            return self.inner.apply_finish(result);
        }

        let motion_command = motion_input.create_motion_command()?;
        match control_input {
            None => {
                self.inner
                    .robot
                    .network()
                    .tcp
                    .throw_if_connection_closed()?;
                self.inner.robot.write_once_motion(&motion_command)
            }
            Some(control_input) => {
                let control_command = create_controller_command(control_input)?;
                self.inner
                    .robot
                    .network()
                    .tcp
                    .throw_if_connection_closed()?;
                self.inner
                    .robot
                    .write_once_motion_and_control(&motion_command, &control_command)
            }
        }
    }
}
