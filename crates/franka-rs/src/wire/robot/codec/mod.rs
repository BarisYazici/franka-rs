//! The version seam of the robot protocol.
//!
//! Everything that differs between FCI v5 (Franka Emika Robot, FER, libfranka 0.9.2) and FCI v10
//! (Franka Research 3, libfranka 0.21.2) at the byte level is funnelled through the functions
//! in this module, so that [`crate::robot`] holds a single [`FciVersion`] and never names a
//! version-specific wire type.
//!
//! The functions take a `&mut [u8; N]` output buffer sized for the **larger** of the two
//! versions and return how many bytes they wrote; there is no allocation and no `dyn`
//! dispatch, only a `match` on [`FciVersion`], so they are safe to call from a control cycle.
//!
//! Status bytes are mapped to the *shared* Rust enums, which keep the FCI v10 name set
//! ([`crate::error::MoveStatus`], [`v10::StopMoveStatus`], [`v10::AutomaticErrorRecoveryStatus`],
//! [`v10::GetterSetterStatus`], [`v10::CommandStatus`]). Because the v5 enums are proper
//! subsets by *name*, mapping is lossless; a byte that is not a valid status for the negotiated
//! version is a [`crate::error::FrankaError::Protocol`].
//!
//! The module is split by direction: `encode` builds the `Move::Request` and the UDP
//! `RobotCommand`, `decode` reads the UDP `RobotState`, and `status` maps response status
//! bytes onto the shared enums. Everything is re-exported here, so `codec::*` paths are
//! unchanged.

mod decode;
mod encode;
mod status;

pub(crate) use decode::{parse_robot_state, parse_state_modes, StateModes, ROBOT_STATE_MAX_LEN};
pub(crate) use encode::{move_request, robot_command, RobotCommandData};
pub(crate) use status::{
    parse_automatic_error_recovery_status, parse_command_status, parse_getter_setter_status,
    parse_move_status, parse_stop_move_status,
};

use crate::wire::robot::{v10, v5};

/// The FCI protocol version a connection speaks.
///
/// `V5` is the Franka Emika Robot (FER) as served by libfranka 0.9.2
/// (`research_interface::robot::kVersion == 5`), `V10` the Franka Research 3 as served by
/// libfranka 0.21.2 (`kVersion == 10`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FciVersion {
    /// Franka Emika Robot (FER), FCI version 5.
    V5,
    /// Franka Research 3, FCI version 10.
    V10,
}

/// A command, named independently of the version that carries it.
///
/// The wire numbering is [`command_id`]; some commands exist in only one version, which is what
/// the `Option` there expresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommandKind {
    Connect,
    Move,
    StopMove,
    /// FCI v5 only (`Robot::getVirtualWall`).
    GetCartesianLimit,
    SetCollisionBehavior,
    SetJointImpedance,
    SetCartesianImpedance,
    SetGuidingMode,
    SetEEToK,
    SetNEToEE,
    SetLoad,
    /// FCI v5 only (`Robot::setFilters`).
    SetFilters,
    AutomaticErrorRecovery,
    /// FCI v5 only (downloads `libfcimodels.so`).
    LoadModelLibrary,
    /// FCI v10 only (downloads the URDF).
    GetRobotModel,
}

impl CommandKind {
    /// The command name libfranka prints in its `CommandException` and `ProtocolException`
    /// texts (`research_interface::robot::CommandTraits<T>::kName`,
    /// `common/include/research_interface/robot/service_traits.h`).
    ///
    /// The two files agree on every name they share; 0.9.2 adds `Get Cartesian Limit` and
    /// `Set Filters`, 0.21.2 adds `Get Robot Model`. `Connect` and `LoadModelLibrary` have no
    /// `CommandTraits` specialisation in either version — neither goes through
    /// `handleCommandResponse` — so the spellings below are this crate's own and are never
    /// compared against a libfranka string.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            CommandKind::Connect => "Connect",
            CommandKind::Move => "Move",
            CommandKind::StopMove => "Stop Move",
            CommandKind::GetCartesianLimit => "Get Cartesian Limit",
            CommandKind::SetCollisionBehavior => "Set Collision Behavior",
            CommandKind::SetJointImpedance => "Set Joint Impedance",
            CommandKind::SetCartesianImpedance => "Set Cartesian Impedance",
            CommandKind::SetGuidingMode => "Set Guiding Mode",
            CommandKind::SetEEToK => "Set EE to K",
            CommandKind::SetNEToEE => "Set NE to EE",
            CommandKind::SetLoad => "Set Load",
            CommandKind::SetFilters => "Set Filters",
            CommandKind::AutomaticErrorRecovery => "Automatic Error Recovery",
            CommandKind::LoadModelLibrary => "Load Model Library",
            CommandKind::GetRobotModel => "Get Robot Model",
        }
    }
}

impl FciVersion {
    /// The version number announced in `Connect::Request` and expected back in
    /// `Connect::Response`.
    pub(crate) const fn number(self) -> u16 {
        match self {
            FciVersion::V5 => v5::ROBOT_VERSION,
            FciVersion::V10 => crate::wire::ROBOT_VERSION,
        }
    }
}

/// The `Connect::Request::version` this crate sends for `version` (5 or 10).
pub(crate) const fn connect_version(version: FciVersion) -> u16 {
    version.number()
}

/// Size of one UDP `RobotState` datagram: 2373 bytes on FCI v5, 1377 on FCI v10.
pub(crate) const fn state_size(version: FciVersion) -> usize {
    match version {
        FciVersion::V5 => std::mem::size_of::<v5::RobotState>(),
        FciVersion::V10 => std::mem::size_of::<v10::RobotState>(),
    }
}

/// Size of one UDP `RobotCommand` datagram: 370 bytes on FCI v5, 371 on FCI v10.
pub(crate) const fn command_size(version: FciVersion) -> usize {
    match version {
        FciVersion::V5 => std::mem::size_of::<v5::RobotCommand>(),
        FciVersion::V10 => std::mem::size_of::<v10::RobotCommand>(),
    }
}

/// Largest `Move::Request` of any supported version (FCI v10's 113 bytes).
pub(crate) const MOVE_REQUEST_MAX_LEN: usize = std::mem::size_of::<v10::MoveRequest>();
const _: () = assert!(std::mem::size_of::<v5::MoveRequest>() <= MOVE_REQUEST_MAX_LEN);

/// Largest `RobotCommand` of any supported version (FCI v10's 371 bytes).
pub(crate) const ROBOT_COMMAND_MAX_LEN: usize = std::mem::size_of::<v10::RobotCommand>();
const _: () = assert!(std::mem::size_of::<v5::RobotCommand>() <= ROBOT_COMMAND_MAX_LEN);

/// The wire `Command` value of `kind` under `version`, or `None` when the command does not
/// exist there.
///
/// FCI v5 numbering is `service_types.h:20-35` of libfranka 0.9.2, FCI v10's is
/// `service_types.h` of 0.21.2. They agree only on `Connect`, `Move` and `StopMove`.
pub(crate) const fn command_id(version: FciVersion, kind: CommandKind) -> Option<u32> {
    Some(match version {
        FciVersion::V5 => match kind {
            CommandKind::Connect => v5::Command::Connect.to_u32(),
            CommandKind::Move => v5::Command::Move.to_u32(),
            CommandKind::StopMove => v5::Command::StopMove.to_u32(),
            CommandKind::GetCartesianLimit => v5::Command::GetCartesianLimit.to_u32(),
            CommandKind::SetCollisionBehavior => v5::Command::SetCollisionBehavior.to_u32(),
            CommandKind::SetJointImpedance => v5::Command::SetJointImpedance.to_u32(),
            CommandKind::SetCartesianImpedance => v5::Command::SetCartesianImpedance.to_u32(),
            CommandKind::SetGuidingMode => v5::Command::SetGuidingMode.to_u32(),
            CommandKind::SetEEToK => v5::Command::SetEEToK.to_u32(),
            CommandKind::SetNEToEE => v5::Command::SetNEToEE.to_u32(),
            CommandKind::SetLoad => v5::Command::SetLoad.to_u32(),
            CommandKind::SetFilters => v5::Command::SetFilters.to_u32(),
            CommandKind::AutomaticErrorRecovery => v5::Command::AutomaticErrorRecovery.to_u32(),
            CommandKind::LoadModelLibrary => v5::Command::LoadModelLibrary.to_u32(),
            CommandKind::GetRobotModel => return None,
        },
        FciVersion::V10 => match kind {
            CommandKind::Connect => v10::Command::Connect.to_u32(),
            CommandKind::Move => v10::Command::Move.to_u32(),
            CommandKind::StopMove => v10::Command::StopMove.to_u32(),
            CommandKind::SetCollisionBehavior => v10::Command::SetCollisionBehavior.to_u32(),
            CommandKind::SetJointImpedance => v10::Command::SetJointImpedance.to_u32(),
            CommandKind::SetCartesianImpedance => v10::Command::SetCartesianImpedance.to_u32(),
            CommandKind::SetGuidingMode => v10::Command::SetGuidingMode.to_u32(),
            CommandKind::SetEEToK => v10::Command::SetEEToK.to_u32(),
            CommandKind::SetNEToEE => v10::Command::SetNEToEE.to_u32(),
            CommandKind::SetLoad => v10::Command::SetLoad.to_u32(),
            CommandKind::AutomaticErrorRecovery => v10::Command::AutomaticErrorRecovery.to_u32(),
            CommandKind::GetRobotModel => v10::Command::GetRobotModel.to_u32(),
            CommandKind::GetCartesianLimit
            | CommandKind::SetFilters
            | CommandKind::LoadModelLibrary => return None,
        },
    })
}

#[cfg(test)]
mod tests;
