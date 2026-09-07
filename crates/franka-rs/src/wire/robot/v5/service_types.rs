//! Robot TCP command messages of FCI v5, ported from
//! `common/include/research_interface/robot/service_types.h` (libfranka 0.9.2).
//!
//! Every request/response struct here is the payload that follows a
//! [`crate::wire::CommandHeader`] — the 12-byte robot header is unchanged between v5 and v10
//! (`service_types.h:37-45`).
//!
//! Several payloads are byte-for-byte identical to their FCI v10 counterparts. Those are
//! re-exported from [`crate::wire::robot::v10`] rather than re-declared, so that the codec and
//! the command layer only need one type per command:
//!
//! | re-exported | reason |
//! |---|---|
//! | [`ConnectRequest`], [`ConnectResponse`], [`ConnectStatus`] | `service_types.h:106-121`, same shape and values |
//! | [`Deviation`] | `service_types.h:150-156` |
//! | [`MoveControllerMode`] | `service_types.h:124-128`, same three values |
//! | [`SetCollisionBehaviorRequest`], [`SetJointImpedanceRequest`], [`SetCartesianImpedanceRequest`], [`SetGuidingModeRequest`], [`SetEEToKRequest`], [`SetNEToEERequest`], [`SetLoadRequest`] | `service_types.h:210-299`, identical layouts |
//! | [`MoveResponse`], [`StopMoveResponse`], [`AutomaticErrorRecoveryResponse`], [`GetterSetterResponse`], [`CommandResponse`] | single status byte; only the *meaning* of the byte differs, which is what the v5 status enums below capture |
//!
//! Everything whose numbering or layout changed is declared here.

use zerocopy::little_endian::{F64, I32, U32};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

pub use crate::wire::robot::v10::service_types::{
    AutomaticErrorRecoveryResponse, CommandResponse, ConnectRequest, ConnectResponse,
    ConnectStatus, Deviation, GetterSetterResponse, MoveControllerMode, MoveResponse,
    SetCartesianImpedanceRequest, SetCollisionBehaviorRequest, SetEEToKRequest,
    SetGuidingModeRequest, SetJointImpedanceRequest, SetLoadRequest, SetNEToEERequest,
    StopMoveResponse,
};

/// FCI v5 robot command identifiers (`research_interface::robot::Command`, `uint32_t`,
/// `service_types.h:20-35`).
///
/// The numbering differs from FCI v10: v5 has `GetCartesianLimit` at 3 and `SetFilters` at 11,
/// which shifts every setter by one, and it has `LoadModelLibrary` instead of `GetRobotModel`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Command {
    /// Opens the FCI session (`kConnect`).
    Connect = 0,
    /// Starts a motion (`kMove`).
    Move = 1,
    /// Stops the running motion (`kStopMove`).
    StopMove = 2,
    /// Reads a virtual wall cuboid (`kGetCartesianLimit`). FCI v5 only.
    GetCartesianLimit = 3,
    /// Port of `franka::Robot::setCollisionBehavior` (`kSetCollisionBehavior`).
    SetCollisionBehavior = 4,
    /// Port of `franka::Robot::setJointImpedance` (`kSetJointImpedance`).
    SetJointImpedance = 5,
    /// Port of `franka::Robot::setCartesianImpedance` (`kSetCartesianImpedance`).
    SetCartesianImpedance = 6,
    /// Port of `franka::Robot::setGuidingMode` (`kSetGuidingMode`).
    SetGuidingMode = 7,
    /// Port of `franka::Robot::setK` (`kSetEEToK`).
    SetEEToK = 8,
    /// Port of `franka::Robot::setEE` (`kSetNEToEE`).
    SetNEToEE = 9,
    /// Port of `franka::Robot::setLoad` (`kSetLoad`).
    SetLoad = 10,
    /// Port of `franka::Robot::setFilters` (`kSetFilters`). FCI v5 only.
    SetFilters = 11,
    /// Clears a reflex and re-enables motion (`kAutomaticErrorRecovery`).
    AutomaticErrorRecovery = 12,
    /// Downloads the robot's `libfcimodels` shared object (`kLoadModelLibrary`). FCI v5 only.
    LoadModelLibrary = 13,
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
            3 => Command::GetCartesianLimit,
            4 => Command::SetCollisionBehavior,
            5 => Command::SetJointImpedance,
            6 => Command::SetCartesianImpedance,
            7 => Command::SetGuidingMode,
            8 => Command::SetEEToK,
            9 => Command::SetNEToEE,
            10 => Command::SetLoad,
            11 => Command::SetFilters,
            12 => Command::AutomaticErrorRecovery,
            13 => Command::LoadModelLibrary,
            _ => return None,
        })
    }
}

/// `Move::Status` (`uint8_t`, `service_types.h:137-148`) — the FCI v5 numbering.
///
/// This is **not** [`crate::error::MoveStatus`]: the shared Rust enum keeps the v10 name set
/// and numbering. `crate::wire::robot::codec::parse_move_status` maps these bytes to it by
/// name. FCI v5 has no `PreemptedDueToActivatedSafetyFunctions` and no
/// `CommandRejectedDueToActivatedSafetyFunctions`, so every value from
/// `kCommandNotPossibleRejected` on is shifted down by two.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MoveStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The motion is now running (`kMotionStarted`).
    MotionStarted = 1,
    /// The motion was pre-empted by another command (`kPreempted`).
    Preempted = 2,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 3,
    /// The robot starts at a singular pose (`kStartAtSingularPoseRejected`).
    StartAtSingularPoseRejected = 4,
    /// An argument was out of range (`kInvalidArgumentRejected`).
    InvalidArgumentRejected = 5,
    /// A reflex aborted the command (`kReflexAborted`).
    ReflexAborted = 6,
    /// The emergency stop or user stop aborted the command (`kEmergencyAborted`).
    EmergencyAborted = 7,
    /// The robot rejected a command sent during the motion (`kInputErrorAborted`).
    InputErrorAborted = 8,
    /// The command was aborted for another reason (`kAborted`).
    Aborted = 9,
}

impl MoveStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<MoveStatus> {
        Some(match v {
            0 => MoveStatus::Success,
            1 => MoveStatus::MotionStarted,
            2 => MoveStatus::Preempted,
            3 => MoveStatus::CommandNotPossibleRejected,
            4 => MoveStatus::StartAtSingularPoseRejected,
            5 => MoveStatus::InvalidArgumentRejected,
            6 => MoveStatus::ReflexAborted,
            7 => MoveStatus::EmergencyAborted,
            8 => MoveStatus::InputErrorAborted,
            9 => MoveStatus::Aborted,
            _ => return None,
        })
    }
}

/// `StopMove::Status` (`uint8_t`, `service_types.h:175-183`) — the FCI v5 numbering, without
/// the v10 `kCommandRejectedDueToActivatedSafetyFunctions` at 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StopMoveStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// The emergency stop or user stop aborted the command (`kEmergencyAborted`).
    EmergencyAborted = 2,
    /// A reflex aborted the command (`kReflexAborted`).
    ReflexAborted = 3,
    /// The command was aborted for another reason (`kAborted`).
    Aborted = 4,
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
            2 => StopMoveStatus::EmergencyAborted,
            3 => StopMoveStatus::ReflexAborted,
            4 => StopMoveStatus::Aborted,
            _ => return None,
        })
    }
}

/// `AutomaticErrorRecovery::Status` (`uint8_t`, `service_types.h:322-332`) — the FCI v5
/// numbering. The relative order is the same as in v10; only the
/// `kCommandRejectedDueToActivatedSafetyFunctions` value at 2 is missing, which shifts
/// everything from `kManualErrorRecoveryRequiredRejected` on down by one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutomaticErrorRecoveryStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// The error needs manual recovery in Desk (`kManualErrorRecoveryRequiredRejected`).
    ManualErrorRecoveryRequiredRejected = 2,
    /// A reflex aborted the command (`kReflexAborted`).
    ReflexAborted = 3,
    /// The emergency stop or user stop aborted the command (`kEmergencyAborted`).
    EmergencyAborted = 4,
    /// The command was aborted for another reason (`kAborted`).
    Aborted = 5,
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
            2 => AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected,
            3 => AutomaticErrorRecoveryStatus::ReflexAborted,
            4 => AutomaticErrorRecoveryStatus::EmergencyAborted,
            5 => AutomaticErrorRecoveryStatus::Aborted,
            _ => return None,
        })
    }
}

/// `GetterSetterCommandBase::Status` (`uint8_t`, `service_types.h:101-104`), used by
/// `GetCartesianLimit` and every setter command.
///
/// FCI v5 stops at `kInvalidArgumentRejected`; byte 3
/// (`kCommandRejectedDueToActivatedSafetyFunctions` in v10) is not a valid v5 status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GetterSetterStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
    /// An argument was out of range (`kInvalidArgumentRejected`).
    InvalidArgumentRejected = 2,
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
            _ => return None,
        })
    }
}

/// `CommandBase::Status` (`uint8_t`, `service_types.h:92`) — the status of every command that
/// does not override `Status`. In FCI v5 no command keeps it: `Connect`, `Move`, `StopMove`,
/// `AutomaticErrorRecovery` and `LoadModelLibrary` all override it and the rest are
/// getter/setters. It is exposed for completeness, for mock servers, and because the codec's
/// shared `CommandBase` parser has to reject its v10-only value `2` on a v5 connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The robot is not in a mode that allows this command (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected = 1,
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
            _ => return None,
        })
    }
}

/// `Move::MotionGeneratorMode` (`uint32_t`, `service_types.h:130-135`) — the motion generator
/// requested by a `Move`.
///
/// FCI v5 has **no** `kNone`: torque-only control is run with a joint-velocity motion generator
/// commanding zero velocities (libfranka 0.9.2 `src/control_loop.cpp`).
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
            _ => return None,
        })
    }
}

/// `Move::Request` — 56 bytes (`service_types.h:158-172`).
///
/// FCI v5 has no asynchronous-motion-generator tail: the v10 request is 113 bytes because of
/// the trailing `use_async_motion_generator` flag and `maximum_velocity` array.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct MoveRequest {
    /// [`MoveControllerMode`] as a `u32`.
    pub controller_mode: U32,
    /// [`MoveMotionGeneratorMode`] as a `u32`.
    pub motion_generator_mode: U32,
    /// Allowed deviation from the commanded path while the motion runs.
    pub maximum_path_deviation: Deviation,
    /// Allowed deviation from the commanded goal pose.
    pub maximum_goal_pose_deviation: Deviation,
}

impl MoveRequest {
    /// Builds the request libfranka 0.9.2's `Robot::Impl::startMotion` sends.
    pub fn new(
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
        maximum_path_deviation: Deviation,
        maximum_goal_pose_deviation: Deviation,
    ) -> Self {
        MoveRequest {
            controller_mode: U32::new(controller_mode.to_u32()),
            motion_generator_mode: U32::new(motion_generator_mode.to_u32()),
            maximum_path_deviation,
            maximum_goal_pose_deviation,
        }
    }
}

/// `SetFilters::Request` — 40 bytes (`service_types.h:301-320`), FCI v5 only.
///
/// The five cut-off frequencies of the robot-side filters, in hertz. libfranka 0.9.2 exposes
/// this as `Robot::setFilters`; FCI v10 dropped the command.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetFiltersRequest {
    /// Cutoff frequency of the joint position filter, in Hz.
    pub joint_position_filter_frequency: F64,
    /// Cutoff frequency of the joint velocity filter, in Hz.
    pub joint_velocity_filter_frequency: F64,
    /// Cutoff frequency of the Cartesian pose filter, in Hz.
    pub cartesian_position_filter_frequency: F64,
    /// Cutoff frequency of the Cartesian velocity filter, in Hz.
    pub cartesian_velocity_filter_frequency: F64,
    /// Cutoff frequency of the controller (torque) filter, in Hz.
    pub controller_filter_frequency: F64,
}

impl SetFiltersRequest {
    /// Builds the request, in the argument order of `Robot::setFilters`.
    pub fn new(
        joint_position_filter_frequency: f64,
        joint_velocity_filter_frequency: f64,
        cartesian_position_filter_frequency: f64,
        cartesian_velocity_filter_frequency: f64,
        controller_filter_frequency: f64,
    ) -> Self {
        SetFiltersRequest {
            joint_position_filter_frequency: F64::new(joint_position_filter_frequency),
            joint_velocity_filter_frequency: F64::new(joint_velocity_filter_frequency),
            cartesian_position_filter_frequency: F64::new(cartesian_position_filter_frequency),
            cartesian_velocity_filter_frequency: F64::new(cartesian_velocity_filter_frequency),
            controller_filter_frequency: F64::new(controller_filter_frequency),
        }
    }
}

/// `GetCartesianLimit::Request` — 4 bytes (`service_types.h:187-191`), FCI v5 only.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GetCartesianLimitRequest {
    /// Index of the virtual wall to query.
    pub id: I32,
}

impl GetCartesianLimitRequest {
    /// Builds the request.
    pub fn new(id: i32) -> Self {
        GetCartesianLimitRequest { id: I32::new(id) }
    }
}

/// `GetCartesianLimit::Response` — 154 bytes (`service_types.h:193-207`), FCI v5 only.
///
/// `1 + 3*8 + 16*8 + 1`. libfranka 0.9.2 turns this into a `franka::VirtualWallCuboid`.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GetCartesianLimitResponse {
    /// [`GetterSetterStatus`] as a `u8`.
    pub status: u8,
    /// Size of the cuboid along its own x/y/z axes.
    pub object_world_size: [F64; 3],
    /// Column-major 4x4 pose of the cuboid in the world frame.
    pub object_frame: [F64; 16],
    /// C++ `bool`: whether the wall is active.
    pub object_activation: u8,
}

/// `LoadModelLibrary::Architecture` (`uint8_t`, `service_types.h:337`), FCI v5 only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LoadModelLibraryArchitecture {
    /// x86-64 (`kX64`).
    X64 = 0,
    /// 32-bit x86 (`kX86`).
    X86 = 1,
    /// 32-bit ARM (`kARM`).
    ARM = 2,
    /// 64-bit ARM (`kARM64`).
    ARM64 = 3,
}

impl LoadModelLibraryArchitecture {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<LoadModelLibraryArchitecture> {
        Some(match v {
            0 => LoadModelLibraryArchitecture::X64,
            1 => LoadModelLibraryArchitecture::X86,
            2 => LoadModelLibraryArchitecture::ARM,
            3 => LoadModelLibraryArchitecture::ARM64,
            _ => return None,
        })
    }
}

/// `LoadModelLibrary::System` (`uint8_t`, `service_types.h:339`), FCI v5 only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LoadModelLibrarySystem {
    /// Linux (`kLinux`).
    Linux = 0,
    /// Windows (`kWindows`).
    Windows = 1,
}

impl LoadModelLibrarySystem {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<LoadModelLibrarySystem> {
        Some(match v {
            0 => LoadModelLibrarySystem::Linux,
            1 => LoadModelLibrarySystem::Windows,
            _ => return None,
        })
    }
}

/// `LoadModelLibrary::Status` (`uint8_t`, `service_types.h:335`), FCI v5 only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LoadModelLibraryStatus {
    /// The command succeeded (`kSuccess`).
    Success = 0,
    /// The library could not be served (`kError`).
    Error = 1,
}

impl LoadModelLibraryStatus {
    /// Wire value.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire value.
    pub const fn from_u8(v: u8) -> Option<LoadModelLibraryStatus> {
        Some(match v {
            0 => LoadModelLibraryStatus::Success,
            1 => LoadModelLibraryStatus::Error,
            _ => return None,
        })
    }
}

/// `LoadModelLibrary::Request` — 2 bytes (`service_types.h:341-347`), FCI v5 only.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct LoadModelLibraryRequest {
    /// [`LoadModelLibraryArchitecture`] as a `u8`.
    pub architecture: u8,
    /// [`LoadModelLibrarySystem`] as a `u8`.
    pub system: u8,
}

impl LoadModelLibraryRequest {
    /// Builds the request. libfranka picks the pair at compile time in `src/platform.h`; on
    /// x86-64 Linux that is `(X64, Linux)`.
    pub fn new(architecture: LoadModelLibraryArchitecture, system: LoadModelLibrarySystem) -> Self {
        LoadModelLibraryRequest {
            architecture: architecture.to_u8(),
            system: system.to_u8(),
        }
    }
}

/// `LoadModelLibrary::Response` — a single status byte, followed by the shared object as the
/// message tail.
///
/// libfranka reads the tail in `Network::tcpBlockingReceiveResponse` (0.9.2 `src/network.h:250`)
/// as `header.size - sizeof(CommandMessage<Response>)` bytes, i.e. everything after the
/// 12-byte header and this status byte: `header.size - 13`.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct LoadModelLibraryResponse {
    /// [`LoadModelLibraryStatus`] as a `u8`.
    pub status: u8,
}
