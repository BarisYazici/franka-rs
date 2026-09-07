//! Robot TCP command messages, ported from
//! `common/include/research_interface/robot/service_types.h` (libfranka 0.21.2).
//!
//! Every request/response struct here is the payload that follows a
//! [`crate::wire::CommandHeader`]; libfranka's `CommandMessage<T>` is header + payload, and the
//! parser in [`crate::network`] treats the two separately.
//!
//! This module holds the command identifier and the response status enums; the request and
//! response payload structs are in [`messages`] and are re-exported here, so every
//! `service_types::*` path is unchanged.

pub mod messages;

pub use messages::*;

pub use crate::error::MoveStatus;

/// Robot command identifiers (`research_interface::robot::Command`, `uint32_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Command {
    /// Opens the FCI session (`kConnect`).
    Connect = 0,
    /// Starts a motion (`kMove`).
    Move = 1,
    /// Stops the running motion (`kStopMove`).
    StopMove = 2,
    /// Port of `franka::Robot::setCollisionBehavior` (`kSetCollisionBehavior`).
    SetCollisionBehavior = 3,
    /// Port of `franka::Robot::setJointImpedance` (`kSetJointImpedance`).
    SetJointImpedance = 4,
    /// Port of `franka::Robot::setCartesianImpedance` (`kSetCartesianImpedance`).
    SetCartesianImpedance = 5,
    /// Port of `franka::Robot::setGuidingMode` (`kSetGuidingMode`).
    SetGuidingMode = 6,
    /// Port of `franka::Robot::setK` (`kSetEEToK`).
    SetEEToK = 7,
    /// Port of `franka::Robot::setEE` (`kSetNEToEE`).
    SetNEToEE = 8,
    /// Port of `franka::Robot::setLoad` (`kSetLoad`).
    SetLoad = 9,
    /// Clears a reflex and re-enables motion (`kAutomaticErrorRecovery`).
    AutomaticErrorRecovery = 10,
    /// Fetches the robot's URDF (`kGetRobotModel`). FCI v10 only.
    GetRobotModel = 11,
}

impl Command {
    /// Wire value.
    pub const fn to_u32(self) -> u32 {
        self as u32
    }

    /// Parses a wire value.
    pub const fn from_u32(v: u32) -> Option<Command> {
        Some(match v {
            0 => Command::Connect,
            1 => Command::Move,
            2 => Command::StopMove,
            3 => Command::SetCollisionBehavior,
            4 => Command::SetJointImpedance,
            5 => Command::SetCartesianImpedance,
            6 => Command::SetGuidingMode,
            7 => Command::SetEEToK,
            8 => Command::SetNEToEE,
            9 => Command::SetLoad,
            10 => Command::AutomaticErrorRecovery,
            11 => Command::GetRobotModel,
            _ => return None,
        })
    }
}

/// `Connect::Status` (`uint8_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ConnectStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The server speaks another FCI version (`kIncompatibleLibraryVersion`).
    IncompatibleLibraryVersion = 1,
}

impl ConnectStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<ConnectStatus> {
        Some(match v {
            0 => ConnectStatus::Success,
            1 => ConnectStatus::IncompatibleLibraryVersion,
            _ => return None,
        })
    }
}

/// `StopMove::Status` (`uint8_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StopMoveStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// A safety function is active (`kCommandRejectedDueToActivatedSafetyFunctions`). FCI v10 only.
    CommandRejectedDueToActivatedSafetyFunctions = 2,
    /// The emergency stop or user stop aborted the command (`kEmergencyAborted`).
    EmergencyAborted = 3,
    /// A reflex aborted the command (`kReflexAborted`).
    ReflexAborted = 4,
    /// The command was aborted for another reason (`kAborted`).
    Aborted = 5,
}

impl StopMoveStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<StopMoveStatus> {
        Some(match v {
            0 => StopMoveStatus::Success,
            1 => StopMoveStatus::CommandNotPossibleRejected,
            2 => StopMoveStatus::CommandRejectedDueToActivatedSafetyFunctions,
            3 => StopMoveStatus::EmergencyAborted,
            4 => StopMoveStatus::ReflexAborted,
            5 => StopMoveStatus::Aborted,
            _ => return None,
        })
    }
}

/// `AutomaticErrorRecovery::Status` (`uint8_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutomaticErrorRecoveryStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// A safety function is active (`kCommandRejectedDueToActivatedSafetyFunctions`). FCI v10 only.
    CommandRejectedDueToActivatedSafetyFunctions = 2,
    /// The error needs manual recovery in Desk (`kManualErrorRecoveryRequiredRejected`).
    ManualErrorRecoveryRequiredRejected = 3,
    /// A reflex aborted the command (`kReflexAborted`).
    ReflexAborted = 4,
    /// The emergency stop or user stop aborted the command (`kEmergencyAborted`).
    EmergencyAborted = 5,
    /// The command was aborted for another reason (`kAborted`).
    Aborted = 6,
}

impl AutomaticErrorRecoveryStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<AutomaticErrorRecoveryStatus> {
        Some(match v {
            0 => AutomaticErrorRecoveryStatus::Success,
            1 => AutomaticErrorRecoveryStatus::CommandNotPossibleRejected,
            2 => AutomaticErrorRecoveryStatus::CommandRejectedDueToActivatedSafetyFunctions,
            3 => AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected,
            4 => AutomaticErrorRecoveryStatus::ReflexAborted,
            5 => AutomaticErrorRecoveryStatus::EmergencyAborted,
            6 => AutomaticErrorRecoveryStatus::Aborted,
            _ => return None,
        })
    }
}

/// `CommandBase::Status` (`uint8_t`, `service_types.h:158-162`) — the status of every command
/// that does **not** override `Status`, i.e. `GetRobotModel` in FCI v10.
///
/// It shares only its first *two* values with [`GetterSetterStatus`]: value `2` is
/// `kCommandRejectedDueToActivatedSafetyFunctions` here but `kInvalidArgumentRejected` in the
/// getter/setter enum (`service_types.h:173-178`), so the two must not be used
/// interchangeably.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// A safety function is active (`kCommandRejectedDueToActivatedSafetyFunctions`). FCI v10 only.
    CommandRejectedDueToActivatedSafetyFunctions = 2,
}

impl CommandStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<CommandStatus> {
        Some(match v {
            0 => CommandStatus::Success,
            1 => CommandStatus::CommandNotPossibleRejected,
            2 => CommandStatus::CommandRejectedDueToActivatedSafetyFunctions,
            _ => return None,
        })
    }
}

/// `GetterSetterCommandBase::Status` (`uint8_t`, `service_types.h:173-178`), used by every
/// setter command (`SetCollisionBehavior`, `SetJointImpedance`, `SetCartesianImpedance`,
/// `SetGuidingMode`, `SetEEToK`, `SetNEToEE`, `SetLoad`).
///
/// Not to be confused with [`CommandStatus`]: they agree on `0` and `1` only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GetterSetterStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// An argument was out of range (`kInvalidArgumentRejected`).
    InvalidArgumentRejected = 2,
    /// A safety function is active (`kCommandRejectedDueToActivatedSafetyFunctions`). FCI v10 only.
    CommandRejectedDueToActivatedSafetyFunctions = 3,
}

impl GetterSetterStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<GetterSetterStatus> {
        Some(match v {
            0 => GetterSetterStatus::Success,
            1 => GetterSetterStatus::CommandNotPossibleRejected,
            2 => GetterSetterStatus::InvalidArgumentRejected,
            3 => GetterSetterStatus::CommandRejectedDueToActivatedSafetyFunctions,
            _ => return None,
        })
    }
}

/// `Move::ControllerMode` (`uint32_t`) — the controller requested by a `Move`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum MoveControllerMode {
    /// Built-in joint impedance controller (`kJointImpedance`).
    JointImpedance = 0,
    /// Built-in Cartesian impedance controller (`kCartesianImpedance`).
    CartesianImpedance = 1,
    /// The client's own torque controller (`kExternalController`).
    ExternalController = 2,
}

impl MoveControllerMode {
    /// Wire value.
    pub const fn to_u32(self) -> u32 {
        self as u32
    }

    /// Parses a wire value.
    pub const fn from_u32(v: u32) -> Option<MoveControllerMode> {
        Some(match v {
            0 => MoveControllerMode::JointImpedance,
            1 => MoveControllerMode::CartesianImpedance,
            2 => MoveControllerMode::ExternalController,
            _ => return None,
        })
    }
}

/// `Move::MotionGeneratorMode` (`uint32_t`) — the motion generator requested by a `Move`.
///
/// Note that this enum is **not** the same as the state's
/// [`crate::wire::robot::MotionGeneratorMode`], which has an extra leading `Idle`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum MoveMotionGeneratorMode {
    /// Joint position motion generator (`kJointPosition`).
    JointPosition = 0,
    /// Joint velocity motion generator (`kJointVelocity`).
    JointVelocity = 1,
    /// Cartesian pose motion generator (`kCartesianPosition`).
    CartesianPosition = 2,
    /// Cartesian velocity motion generator (`kCartesianVelocity`).
    CartesianVelocity = 3,
    /// Torque-only control with no motion generator (`kNone`). FCI v10 only.
    None = 4,
}

impl MoveMotionGeneratorMode {
    /// Wire value.
    pub const fn to_u32(self) -> u32 {
        self as u32
    }

    /// Parses a wire value.
    pub const fn from_u32(v: u32) -> Option<MoveMotionGeneratorMode> {
        Some(match v {
            0 => MoveMotionGeneratorMode::JointPosition,
            1 => MoveMotionGeneratorMode::JointVelocity,
            2 => MoveMotionGeneratorMode::CartesianPosition,
            3 => MoveMotionGeneratorMode::CartesianVelocity,
            4 => MoveMotionGeneratorMode::None,
            _ => return None,
        })
    }
}
