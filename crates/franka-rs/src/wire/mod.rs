//! Packed, little-endian wire structs of the Franka Control Interface.
//!
//! Every struct in this module is a byte-for-byte port of the corresponding C++ struct in
//! libfranka 0.21.2 (`common/include/research_interface/**`), which are declared inside
//! `#pragma pack(push, 1)`. Rust equivalents are therefore `#[repr(C, packed)]` and use the
//! explicitly little-endian scalar types of [`zerocopy::little_endian`] so that the layout is
//! identical on any host. C++ `bool` members become `u8` (`0` = false, non-zero = true).
//!
//! Robot messages are versioned; FCI v10 lives in [`robot::v10`] and is re-exported as
//! [`robot`]. The gripper protocol (version 3) lives in [`gripper`].

pub mod gripper;
pub mod robot;

use zerocopy::little_endian::{F32, F64, U32};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

/// libfranka's `ProtocolException("libfranka: incorrect object size")` (`src/network.h:140`),
/// raised whenever a received message does not have the length its type requires.
pub(crate) fn incorrect_object_size() -> crate::error::FrankaError {
    crate::error::FrankaError::Protocol("libfranka: incorrect object size".to_string())
}

/// FCI library version implemented by this crate (`research_interface::robot::kVersion`).
pub const ROBOT_VERSION: u16 = 10;

/// Robot command TCP port (`research_interface::robot::kCommandPort`).
pub const ROBOT_COMMAND_PORT: u16 = 1337;

/// Which of the two protocol header layouts a TCP session frames with.
///
/// libfranka has one `CommandHeader` per protocol: the robot's has a `uint32_t` command
/// (`research_interface/robot/service_types.h`), the gripper's a `uint16_t` command
/// (`research_interface/gripper/types.h`). The framing code is otherwise identical, so the
/// Rust [`crate::network::TcpSession`] is generic over this enum instead of over a type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderLayout {
    /// 12-byte header: `u32 command, u32 command_id, u32 size`.
    Robot,
    /// 10-byte header: `u16 command, u32 command_id, u32 size`.
    Gripper,
}

impl HeaderLayout {
    /// Size of the header in bytes (12 for the robot, 10 for the gripper).
    pub const fn header_len(self) -> usize {
        match self {
            HeaderLayout::Robot => ROBOT_HEADER_LEN,
            HeaderLayout::Gripper => GRIPPER_HEADER_LEN,
        }
    }

    /// Width of the leading `command` field in bytes (4 for the robot, 2 for the gripper).
    pub const fn command_width(self) -> usize {
        match self {
            HeaderLayout::Robot => 4,
            HeaderLayout::Gripper => 2,
        }
    }

    /// Writes a header for `command` into a fresh byte vector, reserving room for `payload_len`.
    ///
    /// `size` counts the header itself, exactly like libfranka's `CommandHeader::size`.
    pub fn encode_header(self, command: u32, command_id: u32, payload_len: usize) -> Vec<u8> {
        let size = (self.header_len() + payload_len) as u32;
        let mut out = Vec::with_capacity(self.header_len() + payload_len);
        match self {
            HeaderLayout::Robot => out.extend_from_slice(&command.to_le_bytes()),
            HeaderLayout::Gripper => out.extend_from_slice(&(command as u16).to_le_bytes()),
        }
        out.extend_from_slice(&command_id.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out
    }

    /// Decodes `command`, `command_id` and `size` out of a header-sized prefix of `buf`.
    ///
    /// Returns `None` when `buf` is shorter than [`HeaderLayout::header_len`].
    pub fn decode_header(self, buf: &[u8]) -> Option<(u32, u32, u32)> {
        if buf.len() < self.header_len() {
            return None;
        }
        let (command, rest) = match self {
            HeaderLayout::Robot => (
                u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
                &buf[4..],
            ),
            HeaderLayout::Gripper => (u16::from_le_bytes([buf[0], buf[1]]) as u32, &buf[2..]),
        };
        let command_id = u32::from_le_bytes([rest[0], rest[1], rest[2], rest[3]]);
        let size = u32::from_le_bytes([rest[4], rest[5], rest[6], rest[7]]);
        Some((command, command_id, size))
    }
}

/// Size of the robot `CommandHeader` in bytes.
pub const ROBOT_HEADER_LEN: usize = 12;

/// Size of the gripper `CommandHeader` in bytes.
pub const GRIPPER_HEADER_LEN: usize = 10;

/// Robot command header (`research_interface::robot::CommandHeader`).
///
/// `size` is the total message length **including** this header.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct CommandHeader {
    /// `research_interface::robot::Command` as a `u32`.
    pub command: U32,
    /// Identifier echoed by the server; used to demultiplex responses.
    pub command_id: U32,
    /// Total message size in bytes, header included.
    pub size: U32,
}

impl CommandHeader {
    /// Builds a header. `size` counts the header.
    pub fn new(command: u32, command_id: u32, size: u32) -> Self {
        CommandHeader {
            command: U32::new(command),
            command_id: U32::new(command_id),
            size: U32::new(size),
        }
    }
}

/// Converts a wire array of little-endian `f32` to `[f64; N]`.
///
/// This is the Rust equivalent of libfranka's `RobotState::floatarray<N>::operator
/// std::array<double, N>()` (`rbk_types.h`): the robot streams `float`, the public API is
/// `double`.
#[inline]
pub fn f32s_to_f64<const N: usize>(src: &[F32; N]) -> [f64; N] {
    let mut out = [0.0f64; N];
    for (o, s) in out.iter_mut().zip(src.iter()) {
        *o = s.get() as f64;
    }
    out
}

/// Converts a wire array of little-endian `f64` to `[f64; N]`.
#[inline]
pub fn f64s_to_f64<const N: usize>(src: &[F64; N]) -> [f64; N] {
    let mut out = [0.0f64; N];
    for (o, s) in out.iter_mut().zip(src.iter()) {
        *o = s.get();
    }
    out
}

/// Converts `[f64; N]` to the little-endian wire representation.
#[inline]
pub fn f64s_to_wire<const N: usize>(src: &[f64; N]) -> [F64; N] {
    let mut out = [F64::new(0.0); N];
    for (o, &s) in out.iter_mut().zip(src.iter()) {
        *o = F64::new(s);
    }
    out
}

/// Converts a wire `[[f32; 3]; 6]` accelerometer block to `[[f64; 3]; 6]`.
#[inline]
pub fn accel_to_f64(src: &[[F32; 3]; 6]) -> [[f64; 3]; 6] {
    let mut out = [[0.0f64; 3]; 6];
    for (o, s) in out.iter_mut().zip(src.iter()) {
        *o = f32s_to_f64(s);
    }
    out
}

/// Returns the payload of a complete TCP message (everything after the header).
///
/// `message` must be a whole message as stored by [`crate::network::TcpSession`], i.e. the
/// header followed by exactly `header.size - header_len` payload bytes.
pub fn message_payload(layout: HeaderLayout, message: &[u8]) -> &[u8] {
    &message[layout.header_len().min(message.len())..]
}

/// Parses a response payload out of a complete TCP message.
///
/// Mirrors libfranka's `Network::tcpBlockingReceiveResponse`, which only rejects a message when
/// `header.size < sizeof(message)` — a longer message is accepted and the surplus is either
/// handed out as variable-length data (`GetRobotModel`) or ignored. franka-sim pads its
/// one-byte status responses to four bytes, so accepting trailing bytes is required in
/// practice, not just for `GetRobotModel`.
pub fn parse_response<T>(layout: HeaderLayout, message: &[u8]) -> crate::error::FrankaResult<T>
where
    T: FromBytes + KnownLayout + Immutable + Unaligned,
{
    let payload = message_payload(layout, message);
    match T::read_from_prefix(payload) {
        Ok((value, _rest)) => Ok(value),
        Err(_) => Err(crate::error::FrankaError::Protocol(
            "libfranka: Incorrect TCP message size.".to_string(),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_roundtrip() {
        for layout in [HeaderLayout::Robot, HeaderLayout::Gripper] {
            let bytes = layout.encode_header(3, 7, 5);
            assert_eq!(bytes.len(), layout.header_len());
            assert_eq!(
                layout.decode_header(&bytes),
                Some((3, 7, (layout.header_len() + 5) as u32))
            );
            assert_eq!(
                layout.decode_header(&bytes[..layout.header_len() - 1]),
                None
            );
        }
    }

    #[test]
    fn robot_header_struct_matches_encode_header() {
        let header = CommandHeader::new(1, 2, 12);
        assert_eq!(
            header.as_bytes(),
            &HeaderLayout::Robot.encode_header(1, 2, 0)[..]
        );
    }
}
