//! The public `Robot` API.
//!
//! Port of `franka::Robot` (libfranka 0.21.2 `include/franka/robot.h`, `src/robot.cpp`). The
//! doc comments are libfranka's, adapted to Rust's error type.
//!
//! # Threading
//! [`Robot`] is `Send + Sync` and every method takes `&self`, so a control loop and a
//! [`Robot::stop`] call can run on two threads at the same time; share the robot as an
//! `Arc<Robot>`. Only one control or read operation may run at a time: a second one returns
//! [`FrankaError::InvalidOperation`].
//!
//! # Example
//! ```no_run
//! use franka::{JointPositions, RealtimeConfig, Robot};
//!
//! # fn main() -> franka::FrankaResult<()> {
//! let robot = Robot::new("192.168.0.1", RealtimeConfig::Enforce)?;
//! let initial = robot.read_once()?.q;
//! let mut time = 0.0;
//! robot.control_joint_positions(
//!     |_state, period| {
//!         time += period.as_secs_f64();
//!         let delta = std::f64::consts::PI / 8.0 * (1.0 - (std::f64::consts::PI / 2.5 * time).cos());
//!         let mut q = initial;
//!         q[3] += delta;
//!         let output = JointPositions::new(q);
//!         if time >= 5.0 {
//!             franka::motion_finished(output)
//!         } else {
//!             output
//!         }
//!     },
//!     franka::ControllerMode::JointImpedance,
//!     true,
//!     franka::DEFAULT_CUTOFF_FREQUENCY,
//! )?;
//! # Ok(())
//! # }
//! ```

pub mod active_control;
pub mod commands;
pub mod control_loop;
pub mod logger;
pub mod robot_impl;
pub mod target_control;

mod active;
mod control;
pub mod options;

#[cfg(test)]
mod mock_tests;

use std::sync::{Mutex, MutexGuard, TryLockError};

use crate::control_types::{
    CartesianPose, CartesianVelocities, ControllerMode, JointPositions, JointVelocities, Torques,
};
use crate::duration::Duration;
use crate::error::{FrankaError, FrankaResult};
use crate::model::Model;
use crate::realtime::RealtimeConfig;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;
use crate::wire::robot::{Deviation, MoveControllerMode, MoveMotionGeneratorMode};

pub use active_control::{ActiveMotionGenerator, ActiveMotionInput, ActiveTorqueControl};
use control_loop::{ControlLoop, ControlLoopMotion};
pub use options::{RobotOptions, VersionPolicy, VirtualWallCuboid};
use robot_impl::{RobotImpl, DEFAULT_DEVIATION, NUM_JOINTS};
pub use target_control::{
    CartesianSent, CartesianTargetControl, JointSent, JointTargetControl,
    JointTargetControlOptions, Settle, TargetControlOptions,
};

/// Default size of the control log attached to a [`crate::error::ControlException`]
/// (`franka::Robot`'s `log_size` default).
pub const DEFAULT_LOG_SIZE: usize = 50;

/// Text of the exception libfranka raises when a second control or read operation is started
/// (`franka::assertOwningLock`).
pub const CONTROL_LOCK_MESSAGE: &str =
    "libfranka robot: Cannot perform this operation while another control or read operation is \
     running.";

/// Compile-time proof of the `Send + Sync` promise this module's docs make (the contrast the
/// `!Send` [`ActiveTorqueControl`] / [`ActiveMotionGenerator`] handles are documented against).
const _: fn() = || {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Robot>();
};

/// Maintains a network connection to the robot, provides the current robot state, gives access
/// to the robot model and allows to control the robot.
///
/// Port of `franka::Robot`.
#[derive(Debug)]
pub struct Robot {
    robot: RobotImpl,
    control_lock: Mutex<()>,
}

impl Robot {
    /// Establishes a connection with the robot.
    ///
    /// `franka_address` is the IP/hostname of the robot, optionally with a `":port"` suffix;
    /// without one the FCI command port 1337 is used.
    ///
    /// # Errors
    /// [`FrankaError::Network`] if the connection could not be established,
    /// [`FrankaError::IncompatibleVersion`] if the robot speaks another FCI version,
    /// [`FrankaError::Realtime`] with [`RealtimeConfig::Enforce`] if realtime priority or a
    /// realtime kernel is unavailable.
    pub fn new(franka_address: &str, realtime_config: RealtimeConfig) -> FrankaResult<Robot> {
        Robot::new_with_log_size(franka_address, realtime_config, DEFAULT_LOG_SIZE)
    }

    /// [`Robot::new`] with an explicit size for the control log that is attached to a
    /// [`crate::error::ControlException`]; `0` disables logging.
    pub fn new_with_log_size(
        franka_address: &str,
        realtime_config: RealtimeConfig,
        log_size: usize,
    ) -> FrankaResult<Robot> {
        Robot::with_options(
            franka_address,
            RobotOptions::new(realtime_config).with_log_size(log_size),
        )
    }

    /// [`Robot::new`] with an explicit [`RobotOptions`], which is how a Franka Emika Robot (FER)
    /// (FCI v5) is connected.
    ///
    /// # Example
    /// ```no_run
    /// use franka::{FciVersion, RealtimeConfig, Robot, RobotOptions, VersionPolicy};
    ///
    /// # fn main() -> franka::FrankaResult<()> {
    /// let options = RobotOptions::new(RealtimeConfig::Ignore)
    ///     .with_version(VersionPolicy::Exact(FciVersion::V5));
    /// let robot = Robot::with_options("192.168.0.1", options)?;
    /// assert_eq!(robot.fci_version(), FciVersion::V5);
    /// # Ok(())
    /// # }
    /// ```
    pub fn with_options(franka_address: &str, options: RobotOptions) -> FrankaResult<Robot> {
        Ok(Robot {
            robot: RobotImpl::new_with_policy(
                franka_address,
                options.realtime_config,
                options.log_size,
                options.version,
            )?,
            control_lock: Mutex::new(()),
        })
    }

    /// Returns the software version reported by the connected server.
    pub fn server_version(&self) -> u16 {
        self.robot.server_version()
    }

    /// The FCI protocol version this connection negotiated.
    pub fn fci_version(&self) -> FciVersion {
        self.robot.version()
    }

    /// Waits for a robot state arriving over the UDP stream and returns it.
    ///
    /// Unlike [`Robot::read`] this does not take the control lock, so it can be called while
    /// nothing else is running.
    pub fn read_once(&self) -> FrankaResult<RobotState> {
        self.robot.read_once()
    }

    /// Starts a loop for reading the current robot state.
    ///
    /// The callback is invoked for every received state; returning `false` ends the loop.
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running.
    pub fn read<F: FnMut(&RobotState) -> bool>(&self, mut read_callback: F) -> FrankaResult<()> {
        let _lock = self.acquire_control_lock()?;
        loop {
            let robot_state = self.robot.update_motion(None, None)?;
            if !read_callback(&robot_state) {
                return Ok(());
            }
        }
    }

    /// Returns the robot's URDF model as a string (`GetRobotModel`).
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] on FCI v5, which has no `GetRobotModel`; an FER
    /// serves its model as a shared object, so use [`Robot::load_model`] there.
    pub fn robot_model(&self) -> FrankaResult<String> {
        self.robot.get_robot_model()
    }

    /// Loads the kinematic and dynamic model of the connected robot.
    ///
    /// Neither version downloads executable code:
    ///
    /// * **FCI v10 (FR3)** asks the robot for its URDF (`GetRobotModel`) and
    ///   evaluates it with the native backend.
    /// * **FCI v5 (FER)** has no `GetRobotModel`, and this returns
    ///   [`Model::native_fer`] — the same native backend over the built-in
    ///   [`crate::model::FER_URDF`], whose inertial parameters were identified
    ///   from an FER's own `libfcimodels_x64.so`. Nothing is fetched, so this
    ///   cannot fail, needs no `model-library` feature and works on any host.
    ///
    /// Use [`Robot::load_model_from_robot`] to download and `dlopen` the FER's
    /// own shared object instead, which is what libfranka 0.9.2 does and what
    /// franka-rs did before. The two agree on kinematics to 9e-16 and on gravity
    /// to 5e-14; with a payload attached their mass matrices differ by up to
    /// 3e-3 kg m^2, because the shared object's `M_NE` is not a rigid-body model
    /// of the payload. See `docs/book/src/reference/model.md`.
    ///
    /// # Errors
    ///
    /// [`FrankaError::Model`] if the model cannot be obtained or parsed;
    /// [`FrankaError::Network`] / [`FrankaError::Protocol`] for the usual
    /// command-socket failures. On FCI v5 none of these can happen.
    pub fn load_model(&self) -> FrankaResult<Model> {
        match self.fci_version() {
            FciVersion::V5 => Ok(Model::native_fer()),
            FciVersion::V10 => Model::from_urdf(&self.robot_model()?),
        }
    }

    /// Loads the model the robot itself serves.
    ///
    /// On FCI v10 this is [`Robot::load_model`] exactly: an FR3 serves a URDF.
    /// On FCI v5 it is libfranka 0.9.2's behaviour — `LoadModelLibrary`
    /// downloads `libfcimodels.so` and the client `dlopen`s it.
    ///
    /// # Security
    ///
    /// **On FCI v5 (FER) this downloads a shared object served by the robot,
    /// writes it to a temporary file and `dlopen`s it** — that is, it executes
    /// native code chosen by the FCI peer, in this process, with this process's
    /// privileges. Nothing validates the blob: the command socket is plaintext
    /// TCP with no authentication, and there is no signature to check.
    ///
    /// This is deliberate, and it is exactly what libfranka 0.9.2 does
    /// (`franka::Model::Model(Network&)` -> `LibraryDownloader` ->
    /// `LibraryLoader`): the FCI peer is already fully trusted, because it is
    /// the thing that commands the arm. Anyone who can impersonate the robot on
    /// this socket can already move it. So the method stays *safe*, like
    /// libfranka's, and the trust boundary is the network you put the robot on:
    /// give the FCI its own isolated link, as Franka's own setup guide requires.
    ///
    /// To opt out, use [`Robot::load_model`], which needs no download at all, or
    /// build without the default `model-library` feature — then this method
    /// returns [`FrankaError::Model`] on an FER instead of loading anything,
    /// and `libloading` is not linked at all. FCI v10 is unaffected either way:
    /// an FR3 serves a URDF, which is parsed, not executed.
    ///
    /// `Model::from_model_library_bytes` and `Model::from_model_library_path`
    /// (both `model-library` only) expose the same load as `unsafe fn`s, for
    /// callers who supply the bytes themselves.
    ///
    /// # Errors
    ///
    /// [`FrankaError::Model`] if the model cannot be obtained, parsed, saved or
    /// loaded; [`FrankaError::Network`] / [`FrankaError::Protocol`] for the
    /// usual command-socket failures.
    pub fn load_model_from_robot(&self) -> FrankaResult<Model> {
        match self.fci_version() {
            FciVersion::V5 => self.robot.load_model_v5(),
            FciVersion::V10 => Model::from_urdf(&self.robot_model()?),
        }
    }

    /// Sets the cut-off frequencies of the robot-side filters, in hertz (`Robot::setFilters`).
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] on FCI v10, which dropped the command;
    /// [`FrankaError::Command`] if the robot rejected it.
    pub fn set_filters(
        &self,
        joint_position_filter_frequency: f64,
        joint_velocity_filter_frequency: f64,
        cartesian_position_filter_frequency: f64,
        cartesian_velocity_filter_frequency: f64,
        controller_filter_frequency: f64,
    ) -> FrankaResult<()> {
        self.robot.set_filters(
            joint_position_filter_frequency,
            joint_velocity_filter_frequency,
            cartesian_position_filter_frequency,
            cartesian_velocity_filter_frequency,
            controller_filter_frequency,
        )
    }

    /// Returns the parameters of the virtual wall with the given `id`
    /// (`Robot::getVirtualWall`).
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] on FCI v10, which dropped the command;
    /// [`FrankaError::Command`] if the robot rejected it.
    pub fn virtual_wall(&self, id: i32) -> FrankaResult<VirtualWallCuboid> {
        self.robot.virtual_wall(id)
    }

    /// Stops all currently running motions.
    ///
    /// Can be called from a second thread while a control loop is running; the control loop
    /// then fails with a [`FrankaError::Control`] carrying
    /// `"libfranka: Move command preempted!"`.
    ///
    /// # Errors
    /// [`FrankaError::Command`] if the robot rejected the `StopMove`.
    pub fn stop(&self) -> FrankaResult<()> {
        self.robot.stop()
    }

    /// Runs automatic error recovery on the robot, clearing the errors of a reflex or a
    /// collision so that a new motion can be started.
    ///
    /// # Errors
    /// [`FrankaError::Command`] if the robot rejected the command, e.g. because manual error
    /// recovery is required.
    pub fn automatic_error_recovery(&self) -> FrankaResult<()> {
        self.robot.automatic_error_recovery()
    }

    /// Changes the collision behavior.
    ///
    /// Set separate torque and force boundaries for acceleration/deceleration and constant
    /// velocity movement phases. Forces or torques between lower and upper threshold are shown
    /// as contacts in the robot state; above the upper threshold the robot stops and enters an
    /// error state.
    ///
    /// # Errors
    /// [`FrankaError::Command`] if the robot rejected the command.
    #[allow(clippy::too_many_arguments)]
    pub fn set_collision_behavior(
        &self,
        lower_torque_thresholds_acceleration: [f64; 7],
        upper_torque_thresholds_acceleration: [f64; 7],
        lower_torque_thresholds_nominal: [f64; 7],
        upper_torque_thresholds_nominal: [f64; 7],
        lower_force_thresholds_acceleration: [f64; 6],
        upper_force_thresholds_acceleration: [f64; 6],
        lower_force_thresholds_nominal: [f64; 6],
        upper_force_thresholds_nominal: [f64; 6],
    ) -> FrankaResult<()> {
        self.robot.set_collision_behavior(
            &lower_torque_thresholds_acceleration,
            &upper_torque_thresholds_acceleration,
            &lower_torque_thresholds_nominal,
            &upper_torque_thresholds_nominal,
            &lower_force_thresholds_acceleration,
            &upper_force_thresholds_acceleration,
            &lower_force_thresholds_nominal,
            &upper_force_thresholds_nominal,
        )
    }

    /// [`Robot::set_collision_behavior`] with the same thresholds for the acceleration and the
    /// constant velocity phase.
    pub fn set_collision_behavior_simple(
        &self,
        lower_torque_thresholds: [f64; 7],
        upper_torque_thresholds: [f64; 7],
        lower_force_thresholds: [f64; 6],
        upper_force_thresholds: [f64; 6],
    ) -> FrankaResult<()> {
        self.robot.set_collision_behavior(
            &lower_torque_thresholds,
            &upper_torque_thresholds,
            &lower_torque_thresholds,
            &upper_torque_thresholds,
            &lower_force_thresholds,
            &upper_force_thresholds,
            &lower_force_thresholds,
            &upper_force_thresholds,
        )
    }

    /// Sets the impedance for each joint in the internal controller \[Nm/rad\].
    ///
    /// User-provided torques are not affected by this setting.
    pub fn set_joint_impedance(&self, K_theta: [f64; 7]) -> FrankaResult<()> {
        self.robot.set_joint_impedance(&K_theta)
    }

    /// Sets the Cartesian impedance for (x, y, z, roll, pitch, yaw) in the internal controller.
    ///
    /// User-provided torques are not affected by this setting.
    pub fn set_cartesian_impedance(&self, K_x: [f64; 6]) -> FrankaResult<()> {
        self.robot.set_cartesian_impedance(&K_x)
    }

    /// Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).
    ///
    /// If `elbow` is `true` the elbow is locked and the flag for the 3rd and 5th joint is
    /// ignored.
    pub fn set_guiding_mode(&self, guiding_mode: [bool; 6], elbow: bool) -> FrankaResult<()> {
        self.robot.set_guiding_mode(&guiding_mode, elbow)
    }

    /// Sets the transformation from the end effector frame `EE` to the stiffness frame `K`,
    /// column-major.
    pub fn set_k(&self, EE_T_K: [f64; 16]) -> FrankaResult<()> {
        self.robot.set_k(&EE_T_K)
    }

    /// Sets the transformation from the nominal end effector frame `NE` to the end effector
    /// frame `EE`, column-major.
    ///
    /// The transformation from flange to `NE` is set in Desk.
    pub fn set_ee(&self, NE_T_EE: [f64; 16]) -> FrankaResult<()> {
        self.robot.set_ee(&NE_T_EE)
    }

    /// Sets dynamic parameters of a payload: mass in \[kg\], centre of mass in the flange
    /// frame and the inertia matrix with respect to the centre of mass, column-major.
    ///
    /// This is not for setting end effector parameters, which have to be set in the
    /// administrator's interface.
    pub fn set_load(
        &self,
        load_mass: f64,
        F_x_Cload: [f64; 3],
        load_inertia: [f64; 9],
    ) -> FrankaResult<()> {
        self.robot.set_load(load_mass, &F_x_Cload, &load_inertia)
    }

    /// Position-dependent upper joint velocity limits at `q`, as used by the rate limiter.
    pub fn upper_joint_velocity_limits(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        self.robot.upper_joint_velocity_limits(q)
    }

    /// Position-dependent lower joint velocity limits at `q`, as used by the rate limiter.
    pub fn lower_joint_velocity_limits(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        self.robot.lower_joint_velocity_limits(q)
    }

    /// Takes the control lock, or reports that another operation owns it
    /// (`franka::assertOwningLock`).
    fn acquire_control_lock(&self) -> FrankaResult<MutexGuard<'_, ()>> {
        match self.control_lock.try_lock() {
            Ok(guard) => Ok(guard),
            // A panic in a previous control loop leaves the mutex poisoned; the data it guards
            // is `()`, so recovering is always safe.
            Err(TryLockError::Poisoned(poisoned)) => Ok(poisoned.into_inner()),
            Err(TryLockError::WouldBlock) => Err(FrankaError::InvalidOperation(
                CONTROL_LOCK_MESSAGE.to_string(),
            )),
        }
    }
}
