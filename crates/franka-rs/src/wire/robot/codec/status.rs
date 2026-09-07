//! Status bytes: the five response status families, mapped onto the FCI v10 name set.

use crate::error::{FrankaError, FrankaResult, MoveStatus};
use crate::wire::robot::{v10, v5};

use super::FciVersion;

/// A `ProtocolException` for a status byte that is not defined for the negotiated version.
///
/// The text is exactly libfranka's `"libfranka: Unexpected response while handling " + kName + "
/// command!"` (`Robot::Impl::handleCommandResponse`, `src/robot_impl.h`), so the message a user
/// sees is the same whether the byte was rejected here or by the C++ library. `command_name` is
/// the `research_interface::robot::CommandTraits<T>::kName` of the command whose response is
/// being parsed — [`CommandKind::name`] returns it.
fn unknown_status(command_name: &str) -> FrankaError {
    FrankaError::Protocol(format!(
        "libfranka: Unexpected response while handling {command_name} command!"
    ))
}

/// Maps a `Move::Response` status byte to the shared [`MoveStatus`].
///
/// `command_name` is the libfranka command name used in the error text; it is a parameter
/// rather than a constant because the same status family is shared by more than one command in
/// FCI v10, and because the caller already holds a [`CommandKind`] — pass
/// [`CommandKind::name`].
///
/// # Errors
/// [`FrankaError::Protocol`] for a byte that is not a valid `Move::Status` in `version`, with
/// libfranka's exact `"Unexpected response while handling <name> command!"` text.
pub(crate) fn parse_move_status(
    version: FciVersion,
    byte: u8,
    command_name: &str,
) -> FrankaResult<MoveStatus> {
    match version {
        FciVersion::V5 => match v5::MoveStatus::from_u8(byte) {
            Some(v5::MoveStatus::Success) => Ok(MoveStatus::Success),
            Some(v5::MoveStatus::MotionStarted) => Ok(MoveStatus::MotionStarted),
            Some(v5::MoveStatus::Preempted) => Ok(MoveStatus::Preempted),
            Some(v5::MoveStatus::CommandNotPossibleRejected) => {
                Ok(MoveStatus::CommandNotPossibleRejected)
            }
            Some(v5::MoveStatus::StartAtSingularPoseRejected) => {
                Ok(MoveStatus::StartAtSingularPoseRejected)
            }
            Some(v5::MoveStatus::InvalidArgumentRejected) => {
                Ok(MoveStatus::InvalidArgumentRejected)
            }
            Some(v5::MoveStatus::ReflexAborted) => Ok(MoveStatus::ReflexAborted),
            Some(v5::MoveStatus::EmergencyAborted) => Ok(MoveStatus::EmergencyAborted),
            Some(v5::MoveStatus::InputErrorAborted) => Ok(MoveStatus::InputErrorAborted),
            Some(v5::MoveStatus::Aborted) => Ok(MoveStatus::Aborted),
            None => Err(unknown_status(command_name)),
        },
        FciVersion::V10 => MoveStatus::from_u8(byte).ok_or_else(|| unknown_status(command_name)),
    }
}

/// Maps a `StopMove::Response` status byte to the shared [`v10::StopMoveStatus`].
///
/// `command_name` is the libfranka command name used in the error text; pass
/// [`CommandKind::name`].
///
/// # Errors
/// [`FrankaError::Protocol`] for a byte that is not a valid `StopMove::Status` in `version`,
/// with libfranka's exact `"Unexpected response while handling <name> command!"` text.
pub(crate) fn parse_stop_move_status(
    version: FciVersion,
    byte: u8,
    command_name: &str,
) -> FrankaResult<v10::StopMoveStatus> {
    match version {
        FciVersion::V5 => match v5::StopMoveStatus::from_u8(byte) {
            Some(v5::StopMoveStatus::Success) => Ok(v10::StopMoveStatus::Success),
            Some(v5::StopMoveStatus::CommandNotPossibleRejected) => {
                Ok(v10::StopMoveStatus::CommandNotPossibleRejected)
            }
            Some(v5::StopMoveStatus::EmergencyAborted) => Ok(v10::StopMoveStatus::EmergencyAborted),
            Some(v5::StopMoveStatus::ReflexAborted) => Ok(v10::StopMoveStatus::ReflexAborted),
            Some(v5::StopMoveStatus::Aborted) => Ok(v10::StopMoveStatus::Aborted),
            None => Err(unknown_status(command_name)),
        },
        FciVersion::V10 => {
            v10::StopMoveStatus::from_u8(byte).ok_or_else(|| unknown_status(command_name))
        }
    }
}

/// Maps an `AutomaticErrorRecovery::Response` status byte to the shared
/// [`v10::AutomaticErrorRecoveryStatus`].
///
/// `command_name` is the libfranka command name used in the error text; pass
/// [`CommandKind::name`].
///
/// # Errors
/// [`FrankaError::Protocol`] for a byte that is not a valid `AutomaticErrorRecovery::Status` in
/// `version`, with libfranka's exact `"Unexpected response while handling <name> command!"`
/// text.
pub(crate) fn parse_automatic_error_recovery_status(
    version: FciVersion,
    byte: u8,
    command_name: &str,
) -> FrankaResult<v10::AutomaticErrorRecoveryStatus> {
    match version {
        FciVersion::V5 => match v5::AutomaticErrorRecoveryStatus::from_u8(byte) {
            Some(v5::AutomaticErrorRecoveryStatus::Success) => {
                Ok(v10::AutomaticErrorRecoveryStatus::Success)
            }
            Some(v5::AutomaticErrorRecoveryStatus::CommandNotPossibleRejected) => {
                Ok(v10::AutomaticErrorRecoveryStatus::CommandNotPossibleRejected)
            }
            Some(v5::AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected) => {
                Ok(v10::AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected)
            }
            Some(v5::AutomaticErrorRecoveryStatus::ReflexAborted) => {
                Ok(v10::AutomaticErrorRecoveryStatus::ReflexAborted)
            }
            Some(v5::AutomaticErrorRecoveryStatus::EmergencyAborted) => {
                Ok(v10::AutomaticErrorRecoveryStatus::EmergencyAborted)
            }
            Some(v5::AutomaticErrorRecoveryStatus::Aborted) => {
                Ok(v10::AutomaticErrorRecoveryStatus::Aborted)
            }
            None => Err(unknown_status(command_name)),
        },
        FciVersion::V10 => v10::AutomaticErrorRecoveryStatus::from_u8(byte)
            .ok_or_else(|| unknown_status(command_name)),
    }
}

/// Maps a getter/setter status byte to the shared [`v10::GetterSetterStatus`].
///
/// `command_name` is the libfranka command name of the *specific* setter whose response this
/// is — `"Set Load"`, `"Set Filters"`, … — so the error text names that command and not the
/// status family; pass [`CommandKind::name`].
///
/// # Errors
/// [`FrankaError::Protocol`] for a byte that is not a valid `GetterSetterCommandBase::Status`
/// in `version`, with libfranka's exact `"Unexpected response while handling <name> command!"`
/// text. Note that byte `3` is valid on FCI v10 only.
pub(crate) fn parse_getter_setter_status(
    version: FciVersion,
    byte: u8,
    command_name: &str,
) -> FrankaResult<v10::GetterSetterStatus> {
    match version {
        FciVersion::V5 => match v5::GetterSetterStatus::from_u8(byte) {
            Some(v5::GetterSetterStatus::Success) => Ok(v10::GetterSetterStatus::Success),
            Some(v5::GetterSetterStatus::CommandNotPossibleRejected) => {
                Ok(v10::GetterSetterStatus::CommandNotPossibleRejected)
            }
            Some(v5::GetterSetterStatus::InvalidArgumentRejected) => {
                Ok(v10::GetterSetterStatus::InvalidArgumentRejected)
            }
            None => Err(unknown_status(command_name)),
        },
        FciVersion::V10 => {
            v10::GetterSetterStatus::from_u8(byte).ok_or_else(|| unknown_status(command_name))
        }
    }
}

/// Maps a `CommandBase::Status` byte to the shared [`v10::CommandStatus`].
///
/// `command_name` is the libfranka command name of the specific command whose response this is
/// — on FCI v10 that is `"Get Robot Model"`; pass [`CommandKind::name`].
///
/// # Errors
/// [`FrankaError::Protocol`] for a byte that is not a valid `CommandBase::Status` in `version`,
/// with libfranka's exact `"Unexpected response while handling <name> command!"` text. Note
/// that byte `2` is valid on FCI v10 only.
pub(crate) fn parse_command_status(
    version: FciVersion,
    byte: u8,
    command_name: &str,
) -> FrankaResult<v10::CommandStatus> {
    match version {
        FciVersion::V5 => match v5::CommandStatus::from_u8(byte) {
            Some(v5::CommandStatus::Success) => Ok(v10::CommandStatus::Success),
            Some(v5::CommandStatus::CommandNotPossibleRejected) => {
                Ok(v10::CommandStatus::CommandNotPossibleRejected)
            }
            None => Err(unknown_status(command_name)),
        },
        FciVersion::V10 => {
            v10::CommandStatus::from_u8(byte).ok_or_else(|| unknown_status(command_name))
        }
    }
}
