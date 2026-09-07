//! Gripper wire structs, ported from `common/include/research_interface/gripper/types.h`
//! (libfranka 0.21.2). The gripper speaks the same framing as the robot but with a 10-byte
//! header (`uint16_t` command) and `uint16_t` statuses.

use zerocopy::little_endian::{F64, U16, U32};
use zerocopy::{FromBytes, FromZeros, Immutable, IntoBytes, KnownLayout, Unaligned};

/// Gripper protocol version implemented by this crate (`research_interface::gripper::kVersion`).
pub const GRIPPER_VERSION: u16 = 3;

/// Gripper command TCP port (`research_interface::gripper::kCommandPort`).
pub const GRIPPER_COMMAND_PORT: u16 = 1338;

/// Gripper command identifiers (`research_interface::gripper::Command`, `uint16_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum GripperCommand {
    /// Handshake that opens the session (`kConnect`).
    Connect = 0,
    /// Homes the gripper and calibrates its maximum width (`kHoming`).
    Homing = 1,
    /// Grasps an object (`kGrasp`).
    Grasp = 2,
    /// Moves the fingers to a width at a speed (`kMove`).
    Move = 3,
    /// Aborts the running gripper command (`kStop`).
    Stop = 4,
}

impl GripperCommand {
    /// Wire value.
    pub const fn to_u16(self) -> u16 {
        self as u16
    }

    /// Parses a wire value.
    pub const fn from_u16(v: u16) -> Option<GripperCommand> {
        Some(match v {
            0 => GripperCommand::Connect,
            1 => GripperCommand::Homing,
            2 => GripperCommand::Grasp,
            3 => GripperCommand::Move,
            4 => GripperCommand::Stop,
            _ => return None,
        })
    }
}

/// `gripper::CommandBase::Status` (`uint16_t`), shared by every command but `Connect`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum GripperStatus {
    /// The command completed (`kSuccess`).
    Success = 0,
    /// The command could not be started (`kFail`).
    Fail = 1,
    /// The command ran but did not achieve its goal, e.g. a grasp that found nothing
    /// (`kUnsuccessful`).
    Unsuccessful = 2,
    /// The command was aborted, typically by [`GripperCommand::Stop`] (`kAborted`).
    Aborted = 3,
}

impl GripperStatus {
    /// Wire value.
    pub const fn to_u16(self) -> u16 {
        self as u16
    }

    /// Parses a wire value.
    pub const fn from_u16(v: u16) -> Option<GripperStatus> {
        Some(match v {
            0 => GripperStatus::Success,
            1 => GripperStatus::Fail,
            2 => GripperStatus::Unsuccessful,
            3 => GripperStatus::Aborted,
            _ => return None,
        })
    }
}

/// `gripper::Connect::Status` (`uint16_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum GripperConnectStatus {
    /// The gripper accepted the announced protocol version (`kSuccess`).
    Success = 0,
    /// The gripper speaks another protocol version (`kIncompatibleLibraryVersion`).
    IncompatibleLibraryVersion = 1,
}

impl GripperConnectStatus {
    /// Wire value.
    pub const fn to_u16(self) -> u16 {
        self as u16
    }

    /// Parses a wire value.
    pub const fn from_u16(v: u16) -> Option<GripperConnectStatus> {
        Some(match v {
            0 => GripperConnectStatus::Success,
            1 => GripperConnectStatus::IncompatibleLibraryVersion,
            _ => return None,
        })
    }
}

/// `gripper::CommandHeader` — 10 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GripperCommandHeader {
    /// [`GripperCommand`] as a `u16`.
    pub command: U16,
    /// Identifier echoed by the server; used to demultiplex responses.
    pub command_id: U32,
    /// Total message size in bytes, header included.
    pub size: U32,
}

impl GripperCommandHeader {
    /// Builds a header. `size` counts the header.
    pub fn new(command: GripperCommand, command_id: u32, size: u32) -> Self {
        GripperCommandHeader {
            command: U16::new(command.to_u16()),
            command_id: U32::new(command_id),
            size: U32::new(size),
        }
    }
}

/// `gripper::Connect::Request` — 4 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GripperConnectRequest {
    /// Gripper protocol version this client speaks ([`GRIPPER_VERSION`]).
    pub version: U16,
    /// UDP port the client wants gripper states sent to.
    pub udp_port: U16,
}

impl GripperConnectRequest {
    /// Builds a request announcing `version` and the client's `udp_port`.
    pub fn new(version: u16, udp_port: u16) -> Self {
        GripperConnectRequest {
            version: U16::new(version),
            udp_port: U16::new(udp_port),
        }
    }
}

/// `gripper::Connect::Response` — 4 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GripperConnectResponse {
    /// [`GripperConnectStatus`] as a `u16`.
    pub status: U16,
    /// Gripper protocol version implemented by the server.
    pub version: U16,
}

/// `gripper::Move::Request` — 16 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GripperMoveRequest {
    /// Target width in metres.
    pub width: F64,
    /// Closing speed in m/s.
    pub speed: F64,
}

impl GripperMoveRequest {
    /// Builds the request.
    pub fn new(width: f64, speed: f64) -> Self {
        GripperMoveRequest {
            width: F64::new(width),
            speed: F64::new(speed),
        }
    }
}

/// `gripper::Grasp::Request` — 40 bytes (`epsilon` is the nested `GraspEpsilon` struct).
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GraspRequest {
    /// Target width in metres.
    pub width: F64,
    /// `GraspEpsilon::inner`, in metres.
    pub epsilon_inner: F64,
    /// `GraspEpsilon::outer`, in metres.
    pub epsilon_outer: F64,
    /// Closing speed in m/s.
    pub speed: F64,
    /// Grasping force in N.
    pub force: F64,
}

impl GraspRequest {
    /// Builds the request. Argument order matches `franka::Gripper::grasp`.
    pub fn new(width: f64, epsilon_inner: f64, epsilon_outer: f64, speed: f64, force: f64) -> Self {
        GraspRequest {
            width: F64::new(width),
            epsilon_inner: F64::new(epsilon_inner),
            epsilon_outer: F64::new(epsilon_outer),
            speed: F64::new(speed),
            force: F64::new(force),
        }
    }
}

/// The response of `Homing`, `Grasp`, `Move` and `Stop` — 2 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GripperCommandResponse {
    /// [`GripperStatus`] as a `u16`.
    pub status: U16,
}

/// `gripper::GripperState` — 23 bytes, streamed over UDP at roughly 60 Hz.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GripperState {
    /// Monotonically increasing message counter; also the gripper's time in milliseconds.
    pub message_id: U32,
    /// Current opening width in metres.
    pub width: F64,
    /// Maximum opening width in metres.
    pub max_width: F64,
    /// C++ `bool`: whether an object is currently grasped.
    pub is_grasped: u8,
    /// Gripper temperature in degrees Celsius.
    pub temperature: U16,
}

impl Default for GripperState {
    fn default() -> Self {
        GripperState::new_zeroed()
    }
}
