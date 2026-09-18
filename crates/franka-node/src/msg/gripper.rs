//! The gripper's wire messages: [`GripperMsg`] (`"<BBHIQQdd"`) and [`GripperStateMsg`]
//! (`"<BBHIQddd"`), both [`GRIPPER_SIZE`] bytes, laid out like the arm's.

use zerocopy::little_endian::{F64, U16, U32, U64};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

use super::{DecodeError, VERSION};

/// `size_of::<GripperMsg>()` and `size_of::<GripperStateMsg>()`.
pub const GRIPPER_SIZE: usize = 40;
/// [`GripperStateMsg::flags`] bits.
pub const GRIPPER_CALIBRATED: u8 = 1;
pub const GRIPPER_GRASPED: u8 = 2;
pub const GRIPPER_MOVING: u8 = 4;
pub const GRIPPER_FAULT: u8 = 8;

/// What a [`GripperMsg`] asks for.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperKind {
    /// Move to `width`; `force` is ignored.
    Width = 1,
    /// Close on an object at `width` until `force` holds.
    Grasp = 2,
}

impl GripperKind {
    /// Parses the wire value.
    pub const fn from_u8(v: u8) -> Option<GripperKind> {
        match v {
            1 => Some(GripperKind::Width),
            2 => Some(GripperKind::Grasp),
            _ => None,
        }
    }
}

/// A gripper command from a client, [`GRIPPER_SIZE`] bytes.
#[repr(C)]
#[derive(FromBytes, IntoBytes, KnownLayout, Immutable, Debug, Clone, Copy)]
pub struct GripperMsg {
    pub version: u8,
    /// A [`GripperKind`].
    pub kind: u8,
    /// Reserved, 0.
    pub _pad: U16,
    pub client_id: U32,
    /// Per client, strictly increasing; its own sequence, not the targets'.
    pub seq: U64,
    /// The sender's clock, never interpreted.
    pub t_send_ns: U64,
    /// m.
    pub width: F64,
    /// N, for [`GripperKind::Grasp`].
    pub force: F64,
}

impl GripperMsg {
    /// A message of the current [`VERSION`].
    pub fn new(
        kind: GripperKind,
        client_id: u32,
        seq: u64,
        t_send_ns: u64,
        width: f64,
        force: f64,
    ) -> Self {
        GripperMsg {
            version: VERSION,
            kind: kind as u8,
            _pad: U16::ZERO,
            client_id: U32::new(client_id),
            seq: U64::new(seq),
            t_send_ns: U64::new(t_send_ns),
            width: F64::new(width),
            force: F64::new(force),
        }
    }

    /// The kind, if the byte is one.
    pub fn kind(&self) -> Option<GripperKind> {
        GripperKind::from_u8(self.kind)
    }
}

impl TryFrom<&[u8]> for GripperMsg {
    type Error = DecodeError;

    fn try_from(bytes: &[u8]) -> Result<Self, DecodeError> {
        let msg = GripperMsg::read_from_bytes(bytes)
            .map_err(|_| DecodeError::Length(bytes.len(), GRIPPER_SIZE))?;
        if msg.version != VERSION {
            return Err(DecodeError::Version(msg.version));
        }
        GripperKind::from_u8(msg.kind).ok_or(DecodeError::Kind(msg.kind))?;
        Ok(msg)
    }
}

/// The gripper's state as published at 20 Hz, [`GRIPPER_SIZE`] bytes.
#[repr(C)]
#[derive(FromBytes, IntoBytes, KnownLayout, Immutable, Debug, Clone, Copy)]
pub struct GripperStateMsg {
    pub version: u8,
    /// [`GRIPPER_CALIBRATED`], [`GRIPPER_GRASPED`], [`GRIPPER_MOVING`], [`GRIPPER_FAULT`].
    pub flags: u8,
    /// Reserved, 0.
    pub _pad: U16,
    /// The arm's lease holder, 0 if none.
    pub client_id: U32,
    /// The host's `CLOCK_MONOTONIC` at the reading `width` comes from (`GripperState::t_ns`).
    pub t_node_ns: U64,
    /// Measured, m.
    pub width: F64,
    /// The last commanded width, m; the measured one until a command.
    pub commanded: F64,
    /// m; 0 until the gripper knows it.
    pub max_width: F64,
}

impl TryFrom<&[u8]> for GripperStateMsg {
    type Error = DecodeError;

    fn try_from(bytes: &[u8]) -> Result<Self, DecodeError> {
        let msg = GripperStateMsg::read_from_bytes(bytes)
            .map_err(|_| DecodeError::Length(bytes.len(), GRIPPER_SIZE))?;
        if msg.version != VERSION {
            return Err(DecodeError::Version(msg.version));
        }
        Ok(msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{offset_of, size_of};
    use zerocopy::FromZeros;

    #[test]
    fn sizes_match_the_constant() {
        assert_eq!(size_of::<GripperMsg>(), GRIPPER_SIZE);
        assert_eq!(size_of::<GripperStateMsg>(), GRIPPER_SIZE);
        assert_eq!(GRIPPER_SIZE, 40);
    }

    /// The offsets of `<BBHIQQdd` and `<BBHIQddd`.
    #[test]
    fn gripper_offsets_match_the_python_struct_strings() {
        assert_eq!(offset_of!(GripperMsg, version), 0);
        assert_eq!(offset_of!(GripperMsg, kind), 1);
        assert_eq!(offset_of!(GripperMsg, _pad), 2);
        assert_eq!(offset_of!(GripperMsg, client_id), 4);
        assert_eq!(offset_of!(GripperMsg, seq), 8);
        assert_eq!(offset_of!(GripperMsg, t_send_ns), 16);
        assert_eq!(offset_of!(GripperMsg, width), 24);
        assert_eq!(offset_of!(GripperMsg, force), 32);

        assert_eq!(offset_of!(GripperStateMsg, version), 0);
        assert_eq!(offset_of!(GripperStateMsg, flags), 1);
        assert_eq!(offset_of!(GripperStateMsg, _pad), 2);
        assert_eq!(offset_of!(GripperStateMsg, client_id), 4);
        assert_eq!(offset_of!(GripperStateMsg, t_node_ns), 8);
        assert_eq!(offset_of!(GripperStateMsg, width), 16);
        assert_eq!(offset_of!(GripperStateMsg, commanded), 24);
        assert_eq!(offset_of!(GripperStateMsg, max_width), 32);
    }

    #[test]
    fn gripper_msg_round_trips_and_refuses_length_version_and_kind() {
        let msg = GripperMsg::new(GripperKind::Grasp, 7, 3, 99, 0.04, 20.0);
        let bytes = msg.as_bytes().to_vec();
        assert_eq!(bytes.len(), GRIPPER_SIZE);
        assert_eq!(&bytes[32..40], 20.0f64.to_le_bytes());
        let back = GripperMsg::try_from(bytes.as_slice()).unwrap();
        assert_eq!(back.kind(), Some(GripperKind::Grasp));
        assert_eq!(back.client_id.get(), 7);
        assert_eq!(back.seq.get(), 3);
        assert_eq!(back.width.get(), 0.04);
        let decode = |bytes: &[u8]| GripperMsg::try_from(bytes).map(|m| m.kind);
        assert_eq!(decode(&bytes[..39]), Err(DecodeError::Length(39, 40)));
        let mut wrong_version = bytes.clone();
        wrong_version[0] = 2;
        assert_eq!(decode(&wrong_version), Err(DecodeError::Version(2)));
        let mut wrong_kind = bytes.clone();
        wrong_kind[1] = 3;
        assert_eq!(decode(&wrong_kind), Err(DecodeError::Kind(3)));
        assert_eq!(GripperKind::from_u8(1), Some(GripperKind::Width));
        assert_eq!(GripperKind::from_u8(0), None);
    }

    #[test]
    fn gripper_state_decode_checks_length_and_version() {
        let mut msg = GripperStateMsg::new_zeroed();
        msg.version = VERSION;
        msg.flags = GRIPPER_MOVING;
        msg.width = F64::new(0.03);
        let bytes = msg.as_bytes().to_vec();
        let back = GripperStateMsg::try_from(bytes.as_slice()).unwrap();
        assert_eq!((back.flags, back.width.get()), (GRIPPER_MOVING, 0.03));
        assert_eq!(
            GripperStateMsg::try_from(&bytes[..39]).map(|m| m.flags),
            Err(DecodeError::Length(39, 40))
        );
        let mut wrong_version = bytes.clone();
        wrong_version[0] = 0;
        assert_eq!(
            GripperStateMsg::try_from(wrong_version.as_slice()).map(|m| m.flags),
            Err(DecodeError::Version(0))
        );
    }
}
