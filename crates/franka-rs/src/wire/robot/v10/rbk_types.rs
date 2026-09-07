//! Robot UDP state and command messages, ported from
//! `common/include/research_interface/robot/rbk_types.h` (libfranka 0.21.2).
//!
//! The robot streams [`RobotState`] (1377 bytes) at 1 kHz and expects a [`RobotCommand`]
//! (371 bytes) back in the same cycle. All state values are `float` on the wire and are widened
//! to `f64` by [`crate::robot_state::RobotState::from_wire`]; all command values are `double`.

use zerocopy::little_endian::{F32, F64, U64};
use zerocopy::{FromBytes, FromZeros, Immutable, IntoBytes, KnownLayout, Unaligned};

/// `research_interface::robot::MotionGeneratorMode` (`uint8_t`), as reported in the state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MotionGeneratorMode {
    /// No motion generator is running (`kIdle`).
    Idle = 0,
    /// Joint position motion generator (`kJointPosition`).
    JointPosition = 1,
    /// Joint velocity motion generator (`kJointVelocity`).
    JointVelocity = 2,
    /// Cartesian pose motion generator (`kCartesianPosition`).
    CartesianPosition = 3,
    /// Cartesian velocity motion generator (`kCartesianVelocity`).
    CartesianVelocity = 4,
    /// Torque-only control with no motion generator (`kNone`). FCI v10 only.
    None = 5,
}

impl MotionGeneratorMode {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<MotionGeneratorMode> {
        Some(match v {
            0 => MotionGeneratorMode::Idle,
            1 => MotionGeneratorMode::JointPosition,
            2 => MotionGeneratorMode::JointVelocity,
            3 => MotionGeneratorMode::CartesianPosition,
            4 => MotionGeneratorMode::CartesianVelocity,
            5 => MotionGeneratorMode::None,
            _ => return None,
        })
    }
}

/// `research_interface::robot::ControllerMode` (`uint8_t`), as reported in the state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControllerMode {
    /// Built-in joint impedance controller (`kJointImpedance`).
    JointImpedance = 0,
    /// Built-in Cartesian impedance controller (`kCartesianImpedance`).
    CartesianImpedance = 1,
    /// The client's own torque controller (`kExternalController`).
    ExternalController = 2,
    /// A mode this client does not model (`kOther`).
    Other = 3,
}

impl ControllerMode {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<ControllerMode> {
        Some(match v {
            0 => ControllerMode::JointImpedance,
            1 => ControllerMode::CartesianImpedance,
            2 => ControllerMode::ExternalController,
            3 => ControllerMode::Other,
            _ => return None,
        })
    }
}

/// `research_interface::robot::RobotMode` (`uint8_t`), as reported in the state.
///
/// The public [`crate::robot_state::RobotMode`] mirrors this enum one-to-one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RobotMode {
    /// A mode this client does not model (`kOther`).
    Other = 0,
    /// Ready, with no motion and no external controller running (`kIdle`).
    Idle = 1,
    /// A motion and/or an external controller is running (`kMove`).
    Move = 2,
    /// Hand-guiding mode (`kGuiding`).
    Guiding = 3,
    /// A reflex fired and the robot stopped (`kReflex`).
    Reflex = 4,
    /// The user stop is pressed (`kUserStopped`).
    UserStopped = 5,
    /// An automatic error recovery is in progress (`kAutomaticErrorRecovery`).
    AutomaticErrorRecovery = 6,
}

impl RobotMode {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<RobotMode> {
        Some(match v {
            0 => RobotMode::Other,
            1 => RobotMode::Idle,
            2 => RobotMode::Move,
            3 => RobotMode::Guiding,
            4 => RobotMode::Reflex,
            5 => RobotMode::UserStopped,
            6 => RobotMode::AutomaticErrorRecovery,
            _ => return None,
        })
    }
}

/// Number of error flags in a state (`std::array<bool, 41>`).
pub const ERROR_COUNT: usize = crate::errors::ERROR_COUNT;

/// The robot state as it appears on the UDP wire — 1377 bytes.
///
/// Field order is that of `research_interface::robot::RobotState`; note that `EE_T_K` precedes
/// `F_T_NE`/`NE_T_EE` here, unlike in the public [`crate::robot_state::RobotState`].
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct RobotState {
    /// Monotonically increasing 1 kHz cycle counter; also the robot's time in milliseconds.
    pub message_id: U64,
    /// Measured end-effector pose in base frame, column-major 4x4 homogeneous transform (translation in m).
    pub O_T_EE: [F32; 16],
    /// Last desired end-effector pose in base frame, column-major 4x4 (m).
    pub O_T_EE_d: [F32; 16],
    /// Flange-to-end-effector transform, column-major 4x4 (m).
    pub F_T_EE: [F32; 16],
    /// End-effector-to-stiffness-frame transform, column-major 4x4 (m).
    pub EE_T_K: [F32; 16],
    /// Flange-to-nominal-end-effector transform as configured in Desk, column-major 4x4 (m).
    pub F_T_NE: [F32; 16],
    /// Nominal-end-effector-to-end-effector transform, column-major 4x4 (m).
    pub NE_T_EE: [F32; 16],
    /// Mass of the end effector, in kg.
    pub m_ee: F32,
    /// Inertia matrix of the end effector about its centre of mass, column-major 3x3, in kg*m^2.
    pub I_ee: [F32; 9],
    /// Centre of mass of the end effector in flange frame, in m.
    pub F_x_Cee: [F32; 3],
    /// Mass of the external load, in kg.
    pub m_load: F32,
    /// Inertia matrix of the external load about its centre of mass, column-major 3x3, in kg*m^2.
    pub I_load: [F32; 9],
    /// Centre of mass of the external load in flange frame, in m.
    pub F_x_Cload: [F32; 3],
    /// Measured elbow configuration: joint-3 position in rad and the sign of joint 4.
    pub elbow: [F32; 2],
    /// Desired elbow configuration, same layout as `elbow`.
    pub elbow_d: [F32; 2],
    /// Measured link-side joint torques, in Nm.
    pub tau_J: [F32; 7],
    /// Desired link-side joint torques without gravity, in Nm.
    pub tau_J_d: [F32; 7],
    /// Derivative of the measured joint torques, in Nm/s.
    pub dtau_J: [F32; 7],
    /// Measured joint positions, in rad.
    pub q: [F32; 7],
    /// Desired joint positions, in rad.
    pub q_d: [F32; 7],
    /// Measured joint velocities, in rad/s.
    pub dq: [F32; 7],
    /// Desired joint velocities, in rad/s.
    pub dq_d: [F32; 7],
    /// Desired joint accelerations, in rad/s^2.
    pub ddq_d: [F32; 7],
    /// Per-joint contact indicator: 1 where a contact was detected, else 0.
    pub joint_contact: [F32; 7],
    /// Cartesian contact indicator in the stiffness frame (x, y, z, roll, pitch, yaw).
    pub cartesian_contact: [F32; 6],
    /// Per-joint collision indicator: 1 where a collision was detected, else 0.
    pub joint_collision: [F32; 7],
    /// Cartesian collision indicator in the stiffness frame (x, y, z, roll, pitch, yaw).
    pub cartesian_collision: [F32; 6],
    /// Filtered external torque estimate, in Nm.
    pub tau_ext_hat_filtered: [F32; 7],
    /// Estimated external wrench on the stiffness frame, in base frame: N then Nm.
    pub O_F_ext_hat_K: [F32; 6],
    /// Estimated external wrench on the stiffness frame, in the stiffness frame: N then Nm.
    pub K_F_ext_hat_K: [F32; 6],
    /// Desired end-effector twist in base frame: linear m/s then angular rad/s.
    pub O_dP_EE_d: [F32; 6],
    /// Gravity vector in base frame, in m/s^2.
    pub O_ddP_O: [F32; 3],
    /// Commanded elbow configuration: joint-3 position in rad and the sign of joint 4.
    pub elbow_c: [F32; 2],
    /// Commanded elbow velocity, in rad/s for joint 3.
    pub delbow_c: [F32; 2],
    /// Commanded elbow acceleration, in rad/s^2 for joint 3.
    pub ddelbow_c: [F32; 2],
    /// Last commanded end-effector pose in base frame, column-major 4x4 (m).
    pub O_T_EE_c: [F32; 16],
    /// Last commanded end-effector twist in base frame: linear m/s then angular rad/s.
    pub O_dP_EE_c: [F32; 6],
    /// Last commanded end-effector acceleration in base frame: m/s^2 then rad/s^2.
    pub O_ddP_EE_c: [F32; 6],
    /// Motor-side joint positions, in rad.
    pub theta: [F32; 7],
    /// Motor-side joint velocities, in rad/s.
    pub dtheta: [F32; 7],
    /// Raw readings of the six top accelerometers, in m/s^2.
    pub accelerometer_top: [[F32; 3]; 6],
    /// Raw readings of the six bottom accelerometers, in m/s^2.
    pub accelerometer_bottom: [[F32; 3]; 6],
    /// [`MotionGeneratorMode`] as a `u8`.
    pub motion_generator_mode: u8,
    /// [`ControllerMode`] as a `u8`.
    pub controller_mode: u8,
    /// Currently active errors; C++ `std::array<bool, 41>`.
    pub errors: [u8; ERROR_COUNT],
    /// Errors that caused the last reflex; C++ `std::array<bool, 41>`.
    pub reflex_reason: [u8; ERROR_COUNT],
    /// [`RobotMode`] as a `u8`.
    pub robot_mode: u8,
    /// Ratio of successfully received commands over the last second.
    pub control_command_success_rate: F32,
}

impl Default for RobotState {
    /// All-zero state, i.e. `message_id = 0`, `MotionGeneratorMode::Idle`,
    /// `ControllerMode::JointImpedance`, `RobotMode::Other`, no errors.
    fn default() -> Self {
        RobotState::new_zeroed()
    }
}

/// `research_interface::robot::MotionGeneratorCommand` — 306 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct MotionGeneratorCommand {
    /// Commanded joint positions, in rad.
    pub q_c: [F64; 7],
    /// Commanded joint velocities, in rad/s.
    pub dq_c: [F64; 7],
    /// Last commanded end-effector pose in base frame, column-major 4x4 (m).
    pub O_T_EE_c: [F64; 16],
    /// Last commanded end-effector twist in base frame: linear m/s then angular rad/s.
    pub O_dP_EE_c: [F64; 6],
    /// Commanded elbow configuration: joint-3 position in rad and the sign of joint 4.
    pub elbow_c: [F64; 2],
    /// C++ `bool`: whether `elbow_c` carries a valid value.
    pub valid_elbow: u8,
    /// C++ `bool`: set on the last command of a motion.
    pub motion_generation_finished: u8,
}

impl Default for MotionGeneratorCommand {
    fn default() -> Self {
        MotionGeneratorCommand::new_zeroed()
    }
}

/// `research_interface::robot::ControllerCommand` — 57 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct ControllerCommand {
    /// Desired link-side joint torques without gravity, in Nm.
    pub tau_J_d: [F64; 7],
    /// C++ `bool`: set on the last command of a torque control loop.
    pub torque_command_finished: u8,
}

impl Default for ControllerCommand {
    fn default() -> Self {
        ControllerCommand::new_zeroed()
    }
}

/// `research_interface::robot::RobotCommand` — 371 bytes, sent over UDP once per cycle.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct RobotCommand {
    /// `message_id` of the state this command answers.
    pub message_id: U64,
    /// The motion generator half of the command.
    pub motion: MotionGeneratorCommand,
    /// The external controller half of the command.
    pub control: ControllerCommand,
}

impl Default for RobotCommand {
    fn default() -> Self {
        RobotCommand::new_zeroed()
    }
}
