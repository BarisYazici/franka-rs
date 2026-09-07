//! The callback control loops of [`Robot`] (`franka::Robot::control`).

use super::*;

impl Robot {
    // -- Control loops -------------------------------------------------------------------

    /// Starts a control loop for sending joint-level torque commands.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to torque control or motion generation
    /// occurred,
    /// [`FrankaError::InvalidArgument`] if joint-level torque commands are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    ///
    /// # FCI v5
    /// An FER has no torque-only motion generator mode, so this runs libfranka 0.9.2's scheme
    /// (`src/robot.cpp:41-57`): a joint-velocity motion generator commanding all-zero
    /// velocities alongside the external controller, ended with `motion_generation_finished`.
    /// The two callbacks are also evaluated in 0.9.2's order (motion first, controller
    /// short-circuited) rather than 0.21.2's.
    pub fn control_torques<C>(
        &self,
        mut control_callback: C,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        C: FnMut(&RobotState, Duration) -> Torques,
    {
        let _lock = self.acquire_control_lock()?;
        control_loop::control_torques(
            &self.robot,
            &mut control_callback,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a joint position motion generator with an external controller.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to torque control or motion generation
    /// occurred,
    /// [`FrankaError::InvalidArgument`] if joint-level torque or joint position commands are
    /// NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    ///
    /// # FCI v5
    /// On an FER the *first* setpoint of the motion is rate limited against the robot's own
    /// `q_d` / `O_T_EE_c` like every other one, because libfranka 0.9.2's `convertMotion`
    /// (`src/control_loop.cpp:188-205`) has no `initialized_filter_`. On an FR3 the first
    /// setpoint is its own reference and therefore passes the limiter unchanged, which is
    /// libfranka 0.21.2's behaviour (`src/control_loop.cpp:194-200`). Start an FER motion from
    /// (close to) the current pose either way.
    pub fn control_torques_and_joint_positions<C, M>(
        &self,
        control_callback: C,
        motion_callback: M,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        C: FnMut(&RobotState, Duration) -> Torques,
        M: FnMut(&RobotState, Duration) -> JointPositions,
    {
        self.control_with_torques(
            control_callback,
            motion_callback,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a joint velocity motion generator with an external controller.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to torque control or motion generation
    /// occurred,
    /// [`FrankaError::InvalidArgument`] if joint-level torque or joint velocity commands are
    /// NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    pub fn control_torques_and_joint_velocities<C, M>(
        &self,
        control_callback: C,
        motion_callback: M,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        C: FnMut(&RobotState, Duration) -> Torques,
        M: FnMut(&RobotState, Duration) -> JointVelocities,
    {
        self.control_with_torques(
            control_callback,
            motion_callback,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a Cartesian pose motion generator with an external controller.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to torque control or motion generation
    /// occurred,
    /// [`FrankaError::InvalidArgument`] if joint-level torque or Cartesian pose command elements
    /// are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    ///
    /// # FCI v5
    /// On an FER the *first* setpoint of the motion is rate limited against the robot's own
    /// `q_d` / `O_T_EE_c` like every other one, because libfranka 0.9.2's `convertMotion`
    /// (`src/control_loop.cpp:188-205`) has no `initialized_filter_`. On an FR3 the first
    /// setpoint is its own reference and therefore passes the limiter unchanged, which is
    /// libfranka 0.21.2's behaviour (`src/control_loop.cpp:194-200`). Start an FER motion from
    /// (close to) the current pose either way.
    pub fn control_torques_and_cartesian_pose<C, M>(
        &self,
        control_callback: C,
        motion_callback: M,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        C: FnMut(&RobotState, Duration) -> Torques,
        M: FnMut(&RobotState, Duration) -> CartesianPose,
    {
        self.control_with_torques(
            control_callback,
            motion_callback,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a Cartesian velocity motion generator with an external
    /// controller.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to torque control or motion generation
    /// occurred,
    /// [`FrankaError::InvalidArgument`] if joint-level torque or Cartesian velocity command
    /// elements are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    pub fn control_torques_and_cartesian_velocities<C, M>(
        &self,
        control_callback: C,
        motion_callback: M,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        C: FnMut(&RobotState, Duration) -> Torques,
        M: FnMut(&RobotState, Duration) -> CartesianVelocities,
    {
        self.control_with_torques(
            control_callback,
            motion_callback,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a joint position motion generator with one of the robot's
    /// internal controllers.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to motion generation occurred,
    /// [`FrankaError::InvalidArgument`] if joint position commands are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    ///
    /// # FCI v5
    /// On an FER the *first* setpoint of the motion is rate limited against the robot's own
    /// `q_d` / `O_T_EE_c` like every other one, because libfranka 0.9.2's `convertMotion`
    /// (`src/control_loop.cpp:188-205`) has no `initialized_filter_`. On an FR3 the first
    /// setpoint is its own reference and therefore passes the limiter unchanged, which is
    /// libfranka 0.21.2's behaviour (`src/control_loop.cpp:194-200`). Start an FER motion from
    /// (close to) the current pose either way.
    pub fn control_joint_positions<M>(
        &self,
        motion_callback: M,
        controller_mode: ControllerMode,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        M: FnMut(&RobotState, Duration) -> JointPositions,
    {
        self.control_with_mode(
            motion_callback,
            controller_mode,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a joint velocity motion generator with one of the robot's
    /// internal controllers.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to motion generation occurred,
    /// [`FrankaError::InvalidArgument`] if joint velocity commands are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    pub fn control_joint_velocities<M>(
        &self,
        motion_callback: M,
        controller_mode: ControllerMode,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        M: FnMut(&RobotState, Duration) -> JointVelocities,
    {
        self.control_with_mode(
            motion_callback,
            controller_mode,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a Cartesian pose motion generator with one of the robot's
    /// internal controllers.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to motion generation occurred,
    /// [`FrankaError::InvalidArgument`] if Cartesian pose command elements are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    ///
    /// # FCI v5
    /// On an FER the *first* setpoint of the motion is rate limited against the robot's own
    /// `q_d` / `O_T_EE_c` like every other one, because libfranka 0.9.2's `convertMotion`
    /// (`src/control_loop.cpp:188-205`) has no `initialized_filter_`. On an FR3 the first
    /// setpoint is its own reference and therefore passes the limiter unchanged, which is
    /// libfranka 0.21.2's behaviour (`src/control_loop.cpp:194-200`). Start an FER motion from
    /// (close to) the current pose either way.
    pub fn control_cartesian_pose<M>(
        &self,
        motion_callback: M,
        controller_mode: ControllerMode,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        M: FnMut(&RobotState, Duration) -> CartesianPose,
    {
        self.control_with_mode(
            motion_callback,
            controller_mode,
            limit_rate,
            cutoff_frequency,
        )
    }

    /// Starts a control loop for a Cartesian velocity motion generator with one of the robot's
    /// internal controllers.
    ///
    /// `limit_rate` enables the client-side rate limiter and `cutoff_frequency` the first-order
    /// low-pass filter applied to the commanded signal (pass
    /// [`crate::lowpass_filter::MAX_CUTOFF_FREQUENCY`] to disable the filter).
    ///
    /// libfranka 0.21 default: `false` (0.9: `true`); pass `true` to enable the client-side
    /// limiter -- it could distort your motion. libfranka's `cutoff_frequency` default is
    /// [`crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY`]. Rust has no default arguments, so
    /// both are always passed explicitly here.
    ///
    /// # Errors
    /// [`FrankaError::Control`] if an error related to motion generation occurred,
    /// [`FrankaError::InvalidArgument`] if Cartesian velocity command elements are NaN or infinity,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running,
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout. Unlike
    /// libfranka's `control` this never fails with [`FrankaError::Realtime`]: realtime priority
    /// is raised in [`Robot::new`], matching libfranka's `Robot::Impl` constructor.
    pub fn control_cartesian_velocities<M>(
        &self,
        motion_callback: M,
        controller_mode: ControllerMode,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        M: FnMut(&RobotState, Duration) -> CartesianVelocities,
    {
        self.control_with_mode(
            motion_callback,
            controller_mode,
            limit_rate,
            cutoff_frequency,
        )
    }

    fn control_with_torques<C, M, T>(
        &self,
        mut control_callback: C,
        mut motion_callback: M,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        C: FnMut(&RobotState, Duration) -> Torques,
        M: FnMut(&RobotState, Duration) -> T,
        T: ControlLoopMotion,
    {
        let _lock = self.acquire_control_lock()?;
        ControlLoop::new_with_control_and_motion(
            &self.robot,
            &mut control_callback,
            &mut motion_callback,
            limit_rate,
            cutoff_frequency,
        )?
        .run()
    }

    fn control_with_mode<M, T>(
        &self,
        mut motion_callback: M,
        controller_mode: ControllerMode,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<()>
    where
        M: FnMut(&RobotState, Duration) -> T,
        T: ControlLoopMotion,
    {
        let _lock = self.acquire_control_lock()?;
        ControlLoop::new_with_motion(
            &self.robot,
            controller_mode,
            &mut motion_callback,
            limit_rate,
            cutoff_frequency,
        )?
        .run()
    }
}
