//! Wire layouts: little-endian, `#[repr(C)]`, decoded with `zerocopy` without allocation.
//! Every field has alignment one, so there is no padding and the Python `struct` strings are
//! `"<BBHIQQ7d"` for [`TargetMsg`], `"<BBBBHIQQQQd7d7d7d16d6d7dQQQ"` for [`StateMsg`],
//! `"<BBHIQQdd"` for [`GripperMsg`] and `"<BBHIQddd"` for [`GripperStateMsg`].
//! The bytes of a message are `zerocopy::IntoBytes::as_bytes`. A target's [`Kind`] must be
//! the session's mode; [`StateMsg::flags`] says with [`FLAG_JOINTS`] when the session is a
//! joints one and `target` holds `q`. [`TargetMsg::flags`] carries [`TARGET_ANCHOR`] and
//! nothing else yet; an unknown bit is ignored rather than refused. `tests/wire.json` is the
//! same layout, machine-readable, which a test here keeps equal to these types.

use std::fmt;

use franka::RobotMode;
use serde::{Deserialize, Serialize};
use zerocopy::little_endian::{F64, U16, U32, U64};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

mod episode;
mod gripper;
pub mod params;
#[cfg(test)]
mod wire_json;

pub use episode::{check_episode_token, EpisodeMsg, EpisodePhase, EPISODE_TOKEN_MAX};
pub use gripper::{
    GripperKind, GripperMsg, GripperStateMsg, GRIPPER_CALIBRATED, GRIPPER_FAULT, GRIPPER_GRASPED,
    GRIPPER_MOVING, GRIPPER_SIZE,
};

/// The protocol version in the first byte of every message.
pub const VERSION: u8 = 1;
/// `size_of::<TargetMsg>()`.
pub const TARGET_SIZE: usize = 80;
/// `size_of::<StateMsg>()`.
pub const STATE_SIZE: usize = 474;
/// [`TargetMsg::flags`] bit: this target re-anchors the stream, so its step is measured from
/// where the arm is rather than from the previous accepted target; see
/// [`crate::guard::GuardOptions::max_lead`]. Every other bit is reserved and ignored.
pub const TARGET_ANCHOR: u16 = 1;
/// [`StateMsg::flags`] bit: Active, but no target accepted for `hold_after`.
pub const FLAG_HOLDING: u16 = 1;
/// [`StateMsg::flags`] bit: a joints session (`enable` in joints mode, or `home`) is running
/// and [`StateMsg::target`] holds joint positions.
pub const FLAG_JOINTS: u16 = 2;

/// What `TargetMsg::data` holds, and the mode of a session.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    /// Position xyz, m, and quaternion xyzw in the base frame.
    #[default]
    Cartesian = 1,
    /// Joint positions, rad.
    Joints = 2,
}

impl Kind {
    /// Parses the wire value.
    pub const fn from_u8(v: u8) -> Option<Kind> {
        match v {
            1 => Some(Kind::Cartesian),
            2 => Some(Kind::Joints),
            _ => None,
        }
    }
}

impl fmt::Display for Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Kind::Cartesian => "cartesian",
            Kind::Joints => "joints",
        })
    }
}

/// The arm's state machine phase as published in `StateMsg::phase`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Phase {
    Idle = 0,
    Acquired = 1,
    Active = 2,
    Stopping = 3,
    Faulted = 4,
    /// A `home` runs: a joints session towards the ready pose, targets refused.
    Homing = 5,
}

impl Phase {
    /// Parses the wire value.
    pub const fn from_u8(v: u8) -> Option<Phase> {
        match v {
            0 => Some(Phase::Idle),
            1 => Some(Phase::Acquired),
            2 => Some(Phase::Active),
            3 => Some(Phase::Stopping),
            4 => Some(Phase::Faulted),
            5 => Some(Phase::Homing),
            _ => None,
        }
    }
}

impl fmt::Display for Phase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Phase::Idle => "idle",
            Phase::Acquired => "acquired",
            Phase::Active => "active",
            Phase::Stopping => "stopping",
            Phase::Faulted => "faulted",
            Phase::Homing => "homing",
        })
    }
}

/// `franka::RobotMode` as a byte, in declaration order.
pub const fn robot_mode_code(mode: RobotMode) -> u8 {
    match mode {
        RobotMode::Other => 0,
        RobotMode::Idle => 1,
        RobotMode::Move => 2,
        RobotMode::Guiding => 3,
        RobotMode::Reflex => 4,
        RobotMode::UserStopped => 5,
        RobotMode::AutomaticErrorRecovery => 6,
    }
}

/// A target from a client, [`TARGET_SIZE`] bytes.
#[repr(C)]
#[derive(FromBytes, IntoBytes, KnownLayout, Immutable, Debug, Clone, Copy)]
pub struct TargetMsg {
    pub version: u8,
    /// A [`Kind`].
    pub kind: u8,
    /// [`TARGET_ANCHOR`]; every other bit is reserved, 0, and ignored, so a flag this node
    /// does not know never refuses a target.
    pub flags: U16,
    pub client_id: U32,
    /// Per client, strictly increasing.
    pub seq: U64,
    /// The sender's clock, echoed in [`StateMsg::t_send_ns_accepted`], never interpreted.
    pub t_send_ns: U64,
    /// Cartesian: position xyz, quaternion xyzw; Joints: q.
    pub data: [F64; 7],
}

impl TargetMsg {
    /// A message of the current [`VERSION`] with `flags` 0.
    pub fn new(kind: Kind, client_id: u32, seq: u64, t_send_ns: u64, data: [f64; 7]) -> Self {
        TargetMsg {
            version: VERSION,
            kind: kind as u8,
            flags: U16::ZERO,
            client_id: U32::new(client_id),
            seq: U64::new(seq),
            t_send_ns: U64::new(t_send_ns),
            data: data.map(F64::new),
        }
    }

    /// The same message with [`TARGET_ANCHOR`] set.
    pub fn with_anchor(mut self) -> Self {
        self.flags = U16::new(self.flags.get() | TARGET_ANCHOR);
        self
    }

    /// The kind, if the byte is one.
    pub fn kind(&self) -> Option<Kind> {
        Kind::from_u8(self.kind)
    }

    /// Whether [`TARGET_ANCHOR`] is set; unknown bits do not make one.
    pub fn anchor(&self) -> bool {
        self.flags.get() & TARGET_ANCHOR != 0
    }

    /// `data` as native floats.
    pub fn data(&self) -> [f64; 7] {
        self.data.map(F64::get)
    }
}

/// Why a byte slice is not a [`TargetMsg`], a [`GripperMsg`] or a [`GripperStateMsg`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    /// `(got, expected)` bytes.
    #[error("message: {0} bytes, expected {1}")]
    Length(usize, usize),
    /// Not [`VERSION`].
    #[error("message: version {0}, expected {VERSION}")]
    Version(u8),
    /// Not a [`Kind`] or a [`GripperKind`].
    #[error("message: unknown kind {0}")]
    Kind(u8),
}

impl TryFrom<&[u8]> for TargetMsg {
    type Error = DecodeError;

    fn try_from(bytes: &[u8]) -> Result<Self, DecodeError> {
        let msg = TargetMsg::read_from_bytes(bytes)
            .map_err(|_| DecodeError::Length(bytes.len(), TARGET_SIZE))?;
        if msg.version != VERSION {
            return Err(DecodeError::Version(msg.version));
        }
        Kind::from_u8(msg.kind).ok_or(DecodeError::Kind(msg.kind))?;
        Ok(msg)
    }
}

/// The arm's state as published, [`STATE_SIZE`] bytes.
///
/// `franka-cam` decodes the first 42 bytes of this on its own, to read `t_node_ns` and
/// `robot_time_ms` without depending on this crate, and asserts those two offsets against the
/// numbers the tests below pin. Moving a field before them, or changing their types, must bump
/// [`VERSION`] so that decoder stops rather than mis-stamping a recording.
#[repr(C)]
#[derive(FromBytes, IntoBytes, KnownLayout, Immutable, Debug, Clone, Copy)]
pub struct StateMsg {
    pub version: u8,
    /// A [`Phase`].
    pub phase: u8,
    /// [`robot_mode_code`] of the robot's mode.
    pub robot_mode: u8,
    /// 1 if `current_errors` is non-empty.
    pub has_errors: u8,
    /// [`FLAG_HOLDING`] and [`FLAG_JOINTS`]; the other bits are 0.
    pub flags: U16,
    /// The lease holder, 0 if none.
    pub client_id: U32,
    /// The last accepted target's `seq`, 0 if none.
    pub seq_accepted: U64,
    /// Its `t_send_ns`, for the client's own round trip.
    pub t_send_ns_accepted: U64,
    /// The host's `CLOCK_MONOTONIC` at encode; see [`crate::monotonic_ns`].
    pub t_node_ns: U64,
    /// `RobotState.time`, ms.
    pub robot_time_ms: U64,
    /// The robot's `control_command_success_rate`: the share of the last 100 cycles whose
    /// command reached the robot in time. Meaningful while Active; outside, the snapshot
    /// comes from `read_once` and it is 0.
    pub success_rate: F64,
    pub q: [F64; 7],
    pub dq: [F64; 7],
    pub tau_ext: [F64; 7],
    pub o_t_ee: [F64; 16],
    pub o_f_ext_k: [F64; 6],
    /// The current target: position xyz, quaternion xyzw, or joint positions with
    /// [`FLAG_JOINTS`]; the measured pose outside a session.
    pub target: [F64; 7],
    /// Targets accepted since enable.
    pub accepted: U64,
    /// Targets refused since enable.
    pub refused: U64,
    /// Targets dropped since enable: superseded in a drain or lost to a full channel.
    pub dropped: U64,
}

/// The JSON payload of every `cmd/*` query: `{"client_id": 7}`, `enable` also takes
/// `"mode": "cartesian" | "joints"` (default cartesian), `home` also takes `"speed"` (the
/// fraction of the joint limits, default 0.2, clamped to 0.05..=0.5), and a session-starting
/// verb takes `"episode"`, the collector's name for the episode
/// ([`check_episode_token`]): with one, the session's recording id is that name and the
/// arm's file is `<episode>-<arm>.rrd`, so every arm enabled with the same name writes into
/// one recording.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CmdRequest {
    pub client_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode: Option<Kind>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub episode: Option<String>,
}

impl CmdRequest {
    /// `{"client_id": N}`.
    pub fn new(client_id: u32) -> Self {
        CmdRequest {
            client_id,
            mode: None,
            speed: None,
            episode: None,
        }
    }

    /// The request's episode token, checked; `None` when it names none.
    pub fn episode(&self) -> Result<Option<&str>, String> {
        match self.episode.as_deref() {
            Some(token) => check_episode_token(token).map(|()| Some(token)),
            None => Ok(None),
        }
    }
}

/// The JSON reply of every `cmd/*` query.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CmdReply {
    pub ok: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

impl CmdReply {
    /// `{"ok": true}`.
    pub fn ok() -> Self {
        CmdReply {
            ok: true,
            error: None,
        }
    }

    /// `{"ok": false, "error": "<reason>"}`.
    pub fn err(reason: impl fmt::Display) -> Self {
        CmdReply {
            ok: false,
            error: Some(reason.to_string()),
        }
    }

    /// The reply as JSON text.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("a bool and an optional string serialise")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{offset_of, size_of};
    use zerocopy::FromZeros;

    #[test]
    fn sizes_match_the_constants() {
        assert_eq!(size_of::<TargetMsg>(), TARGET_SIZE);
        assert_eq!(size_of::<StateMsg>(), STATE_SIZE);
        assert_eq!(TARGET_SIZE, 80);
        assert_eq!(STATE_SIZE, 474);
    }

    /// The offsets of the module doc's `struct` strings, `<BBHIQQ7d` and
    /// `<BBBBHIQQQQd7d7d7d16d6d7dQQQ`.
    #[test]
    fn offsets_match_the_python_struct_strings() {
        assert_eq!(offset_of!(TargetMsg, version), 0);
        assert_eq!(offset_of!(TargetMsg, kind), 1);
        assert_eq!(offset_of!(TargetMsg, flags), 2);
        assert_eq!(offset_of!(TargetMsg, client_id), 4);
        assert_eq!(offset_of!(TargetMsg, seq), 8);
        assert_eq!(offset_of!(TargetMsg, t_send_ns), 16);
        assert_eq!(offset_of!(TargetMsg, data), 24);

        assert_eq!(offset_of!(StateMsg, version), 0);
        assert_eq!(offset_of!(StateMsg, phase), 1);
        assert_eq!(offset_of!(StateMsg, robot_mode), 2);
        assert_eq!(offset_of!(StateMsg, has_errors), 3);
        assert_eq!(offset_of!(StateMsg, flags), 4);
        assert_eq!(offset_of!(StateMsg, client_id), 6);
        assert_eq!(offset_of!(StateMsg, seq_accepted), 10);
        assert_eq!(offset_of!(StateMsg, t_send_ns_accepted), 18);
        assert_eq!(offset_of!(StateMsg, t_node_ns), 26);
        assert_eq!(offset_of!(StateMsg, robot_time_ms), 34);
        assert_eq!(offset_of!(StateMsg, success_rate), 42);
        assert_eq!(offset_of!(StateMsg, q), 50);
        assert_eq!(offset_of!(StateMsg, dq), 106);
        assert_eq!(offset_of!(StateMsg, tau_ext), 162);
        assert_eq!(offset_of!(StateMsg, o_t_ee), 218);
        assert_eq!(offset_of!(StateMsg, o_f_ext_k), 346);
        assert_eq!(offset_of!(StateMsg, target), 394);
        assert_eq!(offset_of!(StateMsg, accepted), 450);
        assert_eq!(offset_of!(StateMsg, refused), 458);
        assert_eq!(offset_of!(StateMsg, dropped), 466);
    }

    #[test]
    fn target_round_trips_through_bytes() {
        let data = [0.3, -0.1, 0.5, 0.0, 0.0, 0.0, 1.0];
        let msg = TargetMsg::new(Kind::Cartesian, 7, 42, 1_234_567, data);
        let bytes = msg.as_bytes();
        assert_eq!(bytes.len(), TARGET_SIZE);
        assert_eq!(bytes[0], VERSION);
        assert_eq!(&bytes[4..8], 7u32.to_le_bytes());
        let back = TargetMsg::try_from(bytes).unwrap();
        assert_eq!(back.kind(), Some(Kind::Cartesian));
        assert_eq!(back.client_id.get(), 7);
        assert_eq!(back.seq.get(), 42);
        assert_eq!(back.t_send_ns.get(), 1_234_567);
        assert_eq!(back.data(), data);
    }

    #[test]
    fn the_anchor_is_bit_zero_and_every_other_flag_is_ignored() {
        let plain = TargetMsg::new(Kind::Cartesian, 7, 1, 0, [0.0; 7]);
        assert_eq!(plain.flags.get(), 0);
        assert!(!plain.anchor());
        let anchored = plain.with_anchor();
        assert_eq!(anchored.flags.get(), 1);
        assert!(anchored.anchor());
        assert_eq!(anchored.as_bytes()[2..4], [1, 0]);
        // An unknown bit decodes, is not an anchor, and does not disturb one.
        let mut unknown = plain;
        unknown.flags = U16::new(0x8000);
        let back = TargetMsg::try_from(unknown.as_bytes()).expect("unknown flags decode");
        assert!(!back.anchor());
        assert_eq!(back.flags.get(), 0x8000);
        assert!(back.with_anchor().anchor());
    }

    #[test]
    fn target_decode_refuses_length_version_and_kind() {
        let msg = TargetMsg::new(Kind::Joints, 1, 1, 0, [0.0; 7]);
        let bytes = msg.as_bytes().to_vec();
        let decode = |bytes: &[u8]| TargetMsg::try_from(bytes).map(|m| m.kind);
        assert_eq!(decode(&bytes[..79]), Err(DecodeError::Length(79, 80)));
        let mut longer = bytes.clone();
        longer.push(0);
        assert_eq!(decode(&longer), Err(DecodeError::Length(81, 80)));
        let mut wrong_version = bytes.clone();
        wrong_version[0] = 2;
        assert_eq!(decode(&wrong_version), Err(DecodeError::Version(2)));
        let mut wrong_kind = bytes.clone();
        wrong_kind[1] = 9;
        assert_eq!(decode(&wrong_kind), Err(DecodeError::Kind(9)));
        assert_eq!(
            TargetMsg::try_from(bytes.as_slice()).unwrap().kind(),
            Some(Kind::Joints)
        );
    }

    #[test]
    fn state_encodes_little_endian_at_fixed_offsets() {
        let mut state = StateMsg::new_zeroed();
        state.version = VERSION;
        state.phase = Phase::Active as u8;
        state.flags = U16::new(FLAG_HOLDING);
        state.client_id = U32::new(0x0102_0304);
        state.o_t_ee[14] = F64::new(0.5);
        state.dropped = U64::new(3);
        let bytes = state.as_bytes();
        assert_eq!(bytes.len(), STATE_SIZE);
        assert_eq!(bytes[1], 2);
        assert_eq!(&bytes[4..6], [1, 0]);
        assert_eq!(&bytes[6..10], [4, 3, 2, 1]);
        let z_offset = 218 + 14 * 8;
        assert_eq!(&bytes[z_offset..z_offset + 8], 0.5f64.to_le_bytes());
        assert_eq!(&bytes[466..474], 3u64.to_le_bytes());
        let back = StateMsg::read_from_bytes(bytes).unwrap();
        assert_eq!(back.o_t_ee[14].get(), 0.5);
        assert_eq!(back.flags.get(), FLAG_HOLDING);
    }

    #[test]
    fn robot_mode_codes_follow_declaration_order() {
        for code in 0..7u8 {
            let mode = RobotMode::from_u8(code).unwrap();
            assert_eq!(robot_mode_code(mode), code);
        }
        assert_eq!(RobotMode::from_u8(7), None);
    }

    #[test]
    fn phase_and_kind_round_trip() {
        for code in 0..6u8 {
            assert_eq!(Phase::from_u8(code).unwrap() as u8, code);
        }
        assert_eq!(Phase::from_u8(6), None);
        assert_eq!(Phase::Homing.to_string(), "homing");
        assert_eq!(Kind::from_u8(0), None);
        assert_eq!(Kind::from_u8(1), Some(Kind::Cartesian));
        assert_eq!(Kind::default(), Kind::Cartesian);
        assert_eq!(Kind::Joints.to_string(), "joints");
    }

    #[test]
    fn cmd_json_shapes() {
        let request: CmdRequest = serde_json::from_str(r#"{"client_id": 7}"#).unwrap();
        assert_eq!(request, CmdRequest::new(7));
        assert_eq!(
            serde_json::to_string(&request).unwrap(),
            r#"{"client_id":7}"#
        );
        let joints: CmdRequest =
            serde_json::from_str(r#"{"client_id": 7, "mode": "joints", "speed": 0.3}"#).unwrap();
        assert_eq!(joints.mode, Some(Kind::Joints));
        assert_eq!(joints.speed, Some(0.3));
        let named: CmdRequest =
            serde_json::from_str(r#"{"client_id": 7, "episode": "pick-0042"}"#).unwrap();
        assert_eq!(named.episode().unwrap(), Some("pick-0042"));
        assert_eq!(
            serde_json::to_string(&named).unwrap(),
            r#"{"client_id":7,"episode":"pick-0042"}"#
        );
        let bad: CmdRequest =
            serde_json::from_str(r#"{"client_id": 7, "episode": "../etc"}"#).unwrap();
        assert!(bad.episode().unwrap_err().contains("not [A-Za-z0-9_-]"));
        assert_eq!(CmdRequest::new(7).episode(), Ok(None));
        assert!(
            serde_json::from_str::<CmdRequest>(r#"{"client_id": 7, "mode": "elbow"}"#).is_err()
        );
        assert!(serde_json::from_str::<CmdRequest>(r#"{"client_id": 7, "x": 1}"#).is_err());
        assert_eq!(CmdReply::ok().to_json(), r#"{"ok":true}"#);
        assert_eq!(
            CmdReply::err("no lease").to_json(),
            r#"{"ok":false,"error":"no lease"}"#
        );
        let back: CmdReply = serde_json::from_str(r#"{"ok": true}"#).unwrap();
        assert_eq!(back, CmdReply::ok());
    }
}
