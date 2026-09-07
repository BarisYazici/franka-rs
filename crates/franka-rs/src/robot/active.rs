//! The [`ActiveControl`](crate::robot::active_control) entry points of [`Robot`]
//! (`franka::Robot::startTorqueControl` and friends).

use super::*;

impl Robot {
    // -- ActiveControl -------------------------------------------------------------------

    /// Starts an external torque control process driven by
    /// [`ActiveTorqueControl::read_once`] / [`ActiveTorqueControl::write_once`].
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Control`] if a motion is already running on the robot.
    pub fn start_torque_control(&self) -> FrankaResult<ActiveTorqueControl<'_>> {
        let lock = self.acquire_control_lock()?;
        // FCI v5 has no torque-only motion generator mode; `torque_only_motion_mode` picks the
        // zero-velocity joint velocity generator libfranka 0.9.2 uses instead.
        let motion_id = self.start_motion(
            MoveControllerMode::ExternalController,
            self.robot.torque_only_motion_mode(),
        )?;
        Ok(ActiveTorqueControl::new(&self.robot, motion_id, lock))
    }

    /// Starts an externally driven joint position motion.
    pub fn start_joint_position_control(
        &self,
        controller_mode: MoveControllerMode,
    ) -> FrankaResult<ActiveMotionGenerator<'_, JointPositions>> {
        self.start_active_motion(controller_mode, MoveMotionGeneratorMode::JointPosition)
    }

    /// Starts an externally driven joint velocity motion.
    pub fn start_joint_velocity_control(
        &self,
        controller_mode: MoveControllerMode,
    ) -> FrankaResult<ActiveMotionGenerator<'_, JointVelocities>> {
        self.start_active_motion(controller_mode, MoveMotionGeneratorMode::JointVelocity)
    }

    /// Starts an externally driven Cartesian pose motion.
    pub fn start_cartesian_pose_control(
        &self,
        controller_mode: MoveControllerMode,
    ) -> FrankaResult<ActiveMotionGenerator<'_, CartesianPose>> {
        self.start_active_motion(controller_mode, MoveMotionGeneratorMode::CartesianPosition)
    }

    /// Starts an externally driven Cartesian velocity motion.
    pub fn start_cartesian_velocity_control(
        &self,
        controller_mode: MoveControllerMode,
    ) -> FrankaResult<ActiveMotionGenerator<'_, CartesianVelocities>> {
        self.start_active_motion(controller_mode, MoveMotionGeneratorMode::CartesianVelocity)
    }

    fn start_active_motion<T: ActiveMotionInput>(
        &self,
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
    ) -> FrankaResult<ActiveMotionGenerator<'_, T>> {
        let lock = self.acquire_control_lock()?;
        let motion_id = self.start_motion(controller_mode, motion_generator_mode)?;
        Ok(ActiveMotionGenerator::new(
            &self.robot,
            motion_id,
            lock,
            controller_mode,
        ))
    }

    fn start_motion(
        &self,
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
    ) -> FrankaResult<u32> {
        let deviation = Deviation::new(
            DEFAULT_DEVIATION.0,
            DEFAULT_DEVIATION.1,
            DEFAULT_DEVIATION.2,
        );
        self.robot
            .start_motion(controller_mode, motion_generator_mode, deviation, deviation)
    }
}
