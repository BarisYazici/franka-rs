//! Decoding: the UDP `RobotState` datagram and the four fields the drain loop needs from it.

use zerocopy::FromBytes;

use crate::error::FrankaResult;
use crate::robot_state::RobotState;
use crate::wire::incorrect_object_size;
use crate::wire::robot::{v10, v5};

use super::FciVersion;

/// Parses one UDP state datagram into the public [`RobotState`].
///
/// # Errors
/// [`FrankaError::Protocol`] `"libfranka: incorrect object size"` when `bytes` is not exactly
/// [`state_size`] long — the same text libfranka's `Network::udpBlockingReceiveUnsafe` throws
/// (`src/network.h:140-142`).
pub(crate) fn parse_robot_state(version: FciVersion, bytes: &[u8]) -> FrankaResult<RobotState> {
    match version {
        FciVersion::V5 => v5::RobotState::read_from_bytes(bytes)
            .map(|w| RobotState::from_wire_v5(&w))
            .map_err(|_| incorrect_object_size()),
        FciVersion::V10 => v10::RobotState::read_from_bytes(bytes)
            .map(|w| RobotState::from_wire(&w))
            .map_err(|_| incorrect_object_size()),
    }
}

/// Largest `RobotState` datagram of any supported version (FCI v5's 2373 bytes).
///
/// A receive buffer of this size lets one code path read either version's datagram and still
/// notice a wrong length: reading into a buffer sized for the *negotiated* version would
/// silently truncate a longer datagram, which is how a v5 robot answering a v10 session would
/// go unnoticed.
pub(crate) const ROBOT_STATE_MAX_LEN: usize = std::mem::size_of::<v5::RobotState>();
const _: () = assert!(std::mem::size_of::<v10::RobotState>() <= ROBOT_STATE_MAX_LEN);

/// The four fields `Robot::Impl::updateState` reads out of a state datagram.
///
/// Parsing them does not build a [`RobotState`]: the drain loop of
/// `Robot::Impl::receiveRobotState` only needs the `message_id` of every datagram it discards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct StateModes {
    pub message_id: u64,
    /// `MotionGeneratorMode` as a `u8`; the numbering is shared by both versions except for
    /// v10's `None`, which FCI v5 never sends.
    pub motion_generator_mode: u8,
    /// `ControllerMode` as a `u8`.
    pub controller_mode: u8,
    /// `RobotMode` as a `u8`.
    pub robot_mode: u8,
}

/// Reads [`StateModes`] out of one state datagram without copying the whole state.
///
/// # Errors
/// [`FrankaError::Protocol`] `"libfranka: incorrect object size"` when `bytes` is not exactly
/// [`state_size`] long, like [`parse_robot_state`].
pub(crate) fn parse_state_modes(version: FciVersion, bytes: &[u8]) -> FrankaResult<StateModes> {
    match version {
        FciVersion::V5 => v5::RobotState::ref_from_bytes(bytes)
            .map(|s| StateModes {
                message_id: s.message_id.get(),
                motion_generator_mode: s.motion_generator_mode,
                controller_mode: s.controller_mode,
                robot_mode: s.robot_mode,
            })
            .map_err(|_| incorrect_object_size()),
        FciVersion::V10 => v10::RobotState::ref_from_bytes(bytes)
            .map(|s| StateModes {
                message_id: s.message_id.get(),
                motion_generator_mode: s.motion_generator_mode,
                controller_mode: s.controller_mode,
                robot_mode: s.robot_mode,
            })
            .map_err(|_| incorrect_object_size()),
    }
}
