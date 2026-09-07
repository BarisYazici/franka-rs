//! The motion lifecycle of [`RobotImpl`]: `Move`, one control cycle, `StopMove`, the
//! cancel path and the `ActiveControl` single-command writes.

use super::*;

impl RobotImpl {
    /// Sends one robot command (`Robot::Impl::sendRobotCommand`).
    ///
    /// Returns the command that went out, which is the all-zero command when both arguments are
    /// `None` — libfranka sends nothing in that case but still logs the empty command. The
    /// bytes are produced by [`codec::robot_command`], so exactly
    /// [`codec::command_size`] bytes leave the socket: 370 on FCI v5, 371 on FCI v10.
    fn send_robot_command(
        &self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<RobotCommandData> {
        let mut data = RobotCommandData::default();
        if motion_command.is_none() && control_command.is_none() {
            return Ok(data);
        }

        let (current_motion_mode, current_controller_mode) = {
            let guard = self.lock();
            data.message_id = guard.message_id;
            (
                guard.current_move_motion_generator_mode,
                guard.current_move_controller_mode,
            )
        };

        if let Some(motion) = motion_command {
            if current_motion_mode == StateMotionGeneratorMode::Idle
                || current_motion_mode == StateMotionGeneratorMode::None
            {
                return Err(control_error(
                    "libfranka robot: Trying to send motion command, but no motion generator \
                     running!",
                ));
            }
            data.q_c = f64s_to_f64(&motion.q_c);
            data.dq_c = f64s_to_f64(&motion.dq_c);
            data.O_T_EE_c = f64s_to_f64(&motion.O_T_EE_c);
            data.O_dP_EE_c = f64s_to_f64(&motion.O_dP_EE_c);
            data.elbow_c = f64s_to_f64(&motion.elbow_c);
            data.valid_elbow = motion.valid_elbow != 0;
            data.motion_generation_finished = motion.motion_generation_finished != 0;
        }

        if let Some(control) = control_command {
            if current_controller_mode != StateControllerMode::ExternalController {
                return Err(control_error(
                    "libfranka robot: Trying to send control command, but no controller running!",
                ));
            }
            data.tau_J_d = f64s_to_f64(&control.tau_J_d);
            data.torque_command_finished = control.torque_command_finished != 0;
        }

        if current_motion_mode != StateMotionGeneratorMode::Idle
            && current_motion_mode != StateMotionGeneratorMode::None
            && current_controller_mode == StateControllerMode::ExternalController
            && (motion_command.is_none() || control_command.is_none())
        {
            return Err(control_error(
                "libfranka robot: Trying to send partial robot command!",
            ));
        }

        let mut bytes = [0u8; codec::ROBOT_COMMAND_MAX_LEN];
        let length = codec::robot_command(self.version, &data, &mut bytes);
        debug_assert_eq!(length, codec::command_size(self.version));
        self.network.send_udp(&bytes[..length])?;
        Ok(data)
    }

    /// One control cycle: send the command, wait for the next state, log both
    /// (`Robot::Impl::updateMotion`).
    pub fn update_motion(
        &self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<RobotState> {
        self.network.tcp.throw_if_connection_closed()?;

        let command = self.send_robot_command(motion_command, control_command)?;
        let state = self.receive_robot_state()?;
        self.logger().log(&state, &command_log(&command));

        Ok(state)
    }

    /// Waits for the next state without sending a command (`Robot::Impl::readOnce`).
    pub fn read_once(&self) -> FrankaResult<RobotState> {
        self.receive_robot_state()
    }

    /// Detects a motion error and turns the `Move` reply into a [`ControlException`]
    /// (`Robot::Impl::throwOnMotionError`).
    pub fn throw_on_motion_error(
        &self,
        robot_state: &RobotState,
        motion_id: u32,
    ) -> FrankaResult<()> {
        let errored = {
            let guard = self.lock();
            robot_state.robot_mode != RobotMode::Move || !guard.move_active()
        };
        if !errored {
            return Ok(());
        }

        let message = self.network.tcp.blocking_receive_response(motion_id)?;
        let status = self.move_status(&message)?;
        match handle_terminal_move_response(self, status) {
            Err(FrankaError::Command(text)) => {
                Err(self.create_control_exception(&text, status, robot_state.last_motion_errors))
            }
            Err(other) => Err(other),
            Ok(()) => Err(FrankaError::Protocol(
                "Unexpected reply to a Move command".to_string(),
            )),
        }
    }

    /// Starts a motion (`Robot::Impl::startMotion`).
    ///
    /// Sends `Move` — 113 bytes on FCI v10 (with `use_async_motion_generator = false` and zero
    /// maximum velocities), 56 on FCI v5, which has neither field — waits for `MotionStarted`,
    /// then spins the control cycle until the state reports the requested modes, polling for an
    /// early terminal `Move` reply on the way.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] when `motion_generator_mode` is
    /// [`MoveMotionGeneratorMode::None`] on FCI v5, which has no torque-only mode. Torque
    /// control on an FER runs a joint-velocity generator commanding zero velocities instead;
    /// `RobotImpl::torque_only_motion_mode` picks the right mode for the caller.
    pub fn start_motion(
        &self,
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
        maximum_path_deviation: Deviation,
        maximum_goal_pose_deviation: Deviation,
    ) -> FrankaResult<u32> {
        let mut request = [0u8; codec::MOVE_REQUEST_MAX_LEN];
        let length = codec::move_request(
            self.version,
            controller_mode,
            motion_generator_mode,
            maximum_path_deviation,
            maximum_goal_pose_deviation,
            &mut request,
        )?;
        let command = self.command_id(CommandName::Move)?;

        {
            let mut guard = self.lock();
            if guard.motion_generator_running() || guard.controller_running() {
                return Err(control_error(
                    "libfranka robot: Attempted to start multiple motions!",
                ));
            }
            guard.current_move_motion_generator_mode = match motion_generator_mode {
                MoveMotionGeneratorMode::JointPosition => StateMotionGeneratorMode::JointPosition,
                MoveMotionGeneratorMode::JointVelocity => StateMotionGeneratorMode::JointVelocity,
                MoveMotionGeneratorMode::CartesianPosition => {
                    StateMotionGeneratorMode::CartesianPosition
                }
                MoveMotionGeneratorMode::CartesianVelocity => {
                    StateMotionGeneratorMode::CartesianVelocity
                }
                MoveMotionGeneratorMode::None => StateMotionGeneratorMode::None,
            };
            guard.current_move_controller_mode = match controller_mode {
                MoveControllerMode::JointImpedance => StateControllerMode::JointImpedance,
                MoveControllerMode::CartesianImpedance => StateControllerMode::CartesianImpedance,
                MoveControllerMode::ExternalController => StateControllerMode::ExternalController,
            };
        }

        let move_command_id = self.network.tcp.send_request(command, &request[..length])?;
        let response = self
            .network
            .tcp
            .blocking_receive_response(move_command_id)?;
        handle_move_response(self, self.move_status(&response)?)?;

        while !self.lock().move_active() {
            if let Some(message) = self.network.tcp.try_receive_response(move_command_id)? {
                match handle_move_response(self, self.move_status(&message)?) {
                    Ok(()) => break,
                    Err(FrankaError::Command(text)) => {
                        return Err(FrankaError::Control(ControlException::new(text)))
                    }
                    Err(other) => return Err(other),
                }
            }
            self.update_motion(None, None)?;
        }

        self.logger().flush();
        Ok(move_command_id)
    }

    /// The `Move` motion generator mode a torque-only control process runs with.
    ///
    /// FCI v10 has `kNone` (`Robot::control(control_callback)` in libfranka 0.21.2 starts the
    /// motion with it). FCI v5 does not: libfranka 0.9.2's `Robot::control(control_callback,
    /// ...)` (`src/robot.cpp:41-57`) instantiates `ControlLoop<JointVelocities>` with a motion
    /// callback returning all-zero velocities, i.e. it runs a *joint velocity* generator
    /// alongside the external controller.
    pub(crate) fn torque_only_motion_mode(&self) -> MoveMotionGeneratorMode {
        match self.version {
            FciVersion::V5 => MoveMotionGeneratorMode::JointVelocity,
            FciVersion::V10 => MoveMotionGeneratorMode::None,
        }
    }

    /// Sends one torque command of a torque-only control process, adding the zero-velocity
    /// motion command FCI v5 needs alongside it.
    pub(crate) fn write_once_torque_only(&self, control: &ControllerCommand) -> FrankaResult<()> {
        match self.version {
            FciVersion::V5 => {
                let motion = MotionGeneratorCommand::default();
                self.write_once_motion_and_control(&motion, control)
            }
            FciVersion::V10 => self.write_once_control(control),
        }
    }

    /// Ends a torque-only control process, following the same version rule as
    /// [`RobotImpl::write_once_torque_only`].
    pub(crate) fn finish_torque_only(
        &self,
        motion_id: u32,
        control: &ControllerCommand,
    ) -> FrankaResult<()> {
        match self.version {
            FciVersion::V5 => {
                let motion = MotionGeneratorCommand::default();
                self.finish_motion(motion_id, Some(&motion), Some(control))
            }
            FciVersion::V10 => self.finish_motion(motion_id, None, Some(control)),
        }
    }

    /// Ends a motion regularly (`Robot::Impl::finishMotion`).
    ///
    /// The last command is repeated with the "finished" flag set until the robot leaves the
    /// motion, then the terminal `Move` reply is claimed.
    pub fn finish_motion(
        &self,
        motion_id: u32,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<()> {
        {
            let mut guard = self.lock();
            if !guard.motion_generator_running() && !guard.controller_running() {
                guard.current_move_motion_generator_mode = StateMotionGeneratorMode::Idle;
                guard.current_move_controller_mode = StateControllerMode::Other;
                return Ok(());
            }
        }

        let mut motion_finished_command = motion_command.copied();
        let mut controller_finished_command = control_command.copied();
        if let Some(motion) = motion_finished_command.as_mut() {
            motion.motion_generation_finished = 1;
        } else if self.version == FciVersion::V5 {
            // FCI v5's `ControllerCommand` has no `torque_command_finished`, so 0.9.2's
            // `finishMotion` (`src/robot_impl.cpp:150-155`) insists on a motion command.
            const MESSAGE: &str = "libfranka robot: No motion generator command given!";
            log_error(MESSAGE);
            return Err(control_error(MESSAGE));
        } else if let Some(control) = controller_finished_command.as_mut() {
            control.torque_command_finished = 1;
        } else {
            const MESSAGE: &str = "libfranka robot: No motion generator or control command given!";
            log_error(MESSAGE);
            return Err(control_error(MESSAGE));
        }

        // The TCP response for the finished Move might arrive while the robot state still shows
        // that the motion is running, or afterwards. To handle both situations, we do not
        // process TCP packages in this loop and explicitly wait for the Move response over TCP
        // afterwards.
        let mut robot_state = RobotState::default();
        loop {
            {
                let guard = self.lock();
                if !guard.motion_generator_running() && !guard.controller_running() {
                    break;
                }
            }
            robot_state = self.update_motion(
                motion_finished_command.as_ref(),
                controller_finished_command.as_ref(),
            )?;
        }

        let response = self.network.tcp.blocking_receive_response(motion_id)?;
        let status = self.move_status(&response)?;
        if status == MoveStatus::ReflexAborted {
            return Err(self.create_control_exception(
                "Motion finished commanded, but the robot is still moving!",
                status,
                robot_state.last_motion_errors,
            ));
        }
        // The terminal handler, so that `Ok(())` out of a control loop means the motion really
        // ended with `MoveStatus::Success` (see `handle_terminal_move_response`).
        match handle_terminal_move_response(self, status) {
            Ok(()) => {}
            Err(FrankaError::Command(text)) => {
                return Err(self.create_control_exception(
                    &text,
                    status,
                    robot_state.last_motion_errors,
                ))
            }
            Err(other) => return Err(other),
        }

        let mut guard = self.lock();
        guard.current_move_motion_generator_mode = StateMotionGeneratorMode::Idle;
        guard.current_move_controller_mode = StateControllerMode::Other;
        Ok(())
    }

    /// Aborts a motion (`Robot::Impl::cancelMotion`).
    ///
    /// Sends exactly one `StopMove`, drains the state stream until the robot is idle again and
    /// discards the `Move` reply if it has already arrived.
    pub fn cancel_motion(&self, motion_id: u32) -> FrankaResult<()> {
        if !self.network.tcp.is_alive() {
            log_warn("libfranka robot: TCP connection is closed. Cannot cancel motion.");
            return Ok(());
        }

        let stop_command_id = self
            .network
            .tcp
            .send_request(self.command_id(CommandName::StopMove)?, &[])?;
        let response = self
            .network
            .tcp
            .blocking_receive_response(stop_command_id)?;
        match handle_stop_move_response(self, self.stop_move_status(&response)?) {
            Ok(()) => {}
            Err(FrankaError::Command(text)) => {
                return Err(FrankaError::Control(ControlException::new(text)))
            }
            Err(other) => return Err(other),
        }

        loop {
            self.receive_robot_state()?;
            let guard = self.lock();
            if !guard.motion_generator_running() && !guard.controller_running() {
                break;
            }
        }

        // Ignore the Move response; it is not guaranteed to have arrived yet.
        let _ = self.network.tcp.try_receive_response(motion_id)?;

        let mut guard = self.lock();
        guard.current_move_motion_generator_mode = StateMotionGeneratorMode::Idle;
        guard.current_move_controller_mode = StateControllerMode::Other;
        Ok(())
    }

    /// Sends one torque command outside a control loop (`Robot::Impl::writeOnce(const
    /// Torques&)`).
    pub(crate) fn write_once_control(&self, control: &ControllerCommand) -> FrankaResult<()> {
        self.send_robot_command(None, Some(control))?;
        Ok(())
    }

    /// Sends one motion command outside a control loop
    /// (`Robot::Impl::writeOnce(const MotionGeneratorType&)`).
    pub(crate) fn write_once_motion(&self, motion: &MotionGeneratorCommand) -> FrankaResult<()> {
        self.send_robot_command(Some(motion), None)?;
        Ok(())
    }

    /// Sends one motion and one torque command outside a control loop
    /// (`Robot::Impl::writeOnce(const MotionGeneratorType&, const Torques&)`).
    pub(crate) fn write_once_motion_and_control(
        &self,
        motion: &MotionGeneratorCommand,
        control: &ControllerCommand,
    ) -> FrankaResult<()> {
        self.send_robot_command(Some(motion), Some(control))?;
        Ok(())
    }

    /// Whether a motion generator is active according to the last state
    /// (`Robot::Impl::motionGeneratorRunning`).
    pub fn motion_generator_running(&self) -> bool {
        self.lock().motion_generator_running()
    }

    /// Whether an external controller is active according to the last state
    /// (`Robot::Impl::controllerRunning`).
    pub fn controller_running(&self) -> bool {
        self.lock().controller_running()
    }

    /// Reads the status byte of a `Move` response and maps it through the codec.
    pub(crate) fn move_status(&self, message: &[u8]) -> FrankaResult<MoveStatus> {
        codec::parse_move_status(
            self.version,
            status_byte(message)?,
            CommandKind::Move.name(),
        )
    }

    /// Reads the status byte of a `StopMove` response and maps it through the codec.
    pub(crate) fn stop_move_status(
        &self,
        message: &[u8],
    ) -> FrankaResult<crate::wire::robot::StopMoveStatus> {
        codec::parse_stop_move_status(
            self.version,
            status_byte(message)?,
            CommandKind::StopMove.name(),
        )
    }

    /// Builds the [`ControlException`] libfranka's `createControlException` produces, including
    /// the reflex error names and the estimated command success rate.
    fn create_control_exception(
        &self,
        message: &str,
        move_status: MoveStatus,
        reflex_errors: Errors,
    ) -> FrankaError {
        let log = self.logger().flush();
        let exception = create_control_exception(message, move_status, reflex_errors, log);
        log_error(&exception.message);
        FrankaError::Control(exception)
    }
}
