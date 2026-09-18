//! `tests/wire.json`, the layout clients in other languages pin themselves to: each message's
//! size and fields from `offset_of!`, one sample with distinct values per message, the codes
//! and flag bits, and the JSON payloads. Compared as parsed JSON; `FRANKA_NODE_BLESS=1`
//! rewrites the file.

use std::mem::{offset_of, size_of};
use std::path::Path;

use franka::RobotMode;
use serde_json::{json, Value};
use zerocopy::little_endian::{F64, U16, U32, U64};
use zerocopy::{Immutable, IntoBytes};

use super::*;

fn size_of_field<M, F>(_: fn(&M) -> &F) -> usize {
    size_of::<F>()
}

fn width(ty: &str) -> usize {
    match ty {
        "u8" => 1,
        "u16" => 2,
        "u32" => 4,
        "u64" | "f64" => 8,
        other => panic!("type {other}"),
    }
}

/// `[name, type, offset, count]`, checked against the field's size.
macro_rules! field {
    ($msg:ty, $name:ident, $ty:literal, $count:expr) => {{
        let size = size_of_field(|m: &$msg| &m.$name);
        assert_eq!(size, $count * width($ty), stringify!($name));
        json!([stringify!($name), $ty, offset_of!($msg, $name), $count])
    }};
}

/// The fields must tile the message: contiguous from 0 to its size.
fn message<M: IntoBytes + Immutable>(sample: &M, fields: Vec<Value>, values: Value) -> Value {
    let mut end = 0;
    for f in &fields {
        let (ty, offset, count) = (f[1].as_str().unwrap(), f[2].as_u64(), f[3].as_u64());
        assert_eq!(offset, Some(end as u64), "{f}");
        end += width(ty) * count.unwrap() as usize;
    }
    assert_eq!(end, size_of::<M>());
    let hex: String = sample
        .as_bytes()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    json!({
        "size": size_of::<M>(),
        "fields": fields,
        "sample": {"values": values, "hex": hex},
    })
}

/// `N` distinct floats, exact in binary, from `start`.
fn floats<const N: usize>(start: f64) -> [F64; N] {
    std::array::from_fn(|i| F64::new(start + i as f64 * 0.0625))
}

fn get<const N: usize>(values: [F64; N]) -> [f64; N] {
    values.map(F64::get)
}

fn target() -> Value {
    let data = [0.5, -1.25, 2.75, 0.125, -0.375, 0.625, 0.875];
    let s = TargetMsg::new(
        Kind::Joints,
        0x0A0B_0C0D,
        0x0102_0304_0506_0708,
        0x1112_1314_1516_1718,
        data,
    )
    .with_anchor();
    let fields = vec![
        field!(TargetMsg, version, "u8", 1),
        field!(TargetMsg, kind, "u8", 1),
        field!(TargetMsg, flags, "u16", 1),
        field!(TargetMsg, client_id, "u32", 1),
        field!(TargetMsg, seq, "u64", 1),
        field!(TargetMsg, t_send_ns, "u64", 1),
        field!(TargetMsg, data, "f64", 7),
    ];
    let values = json!({
        "version": s.version, "kind": s.kind, "flags": s.flags.get(),
        "client_id": s.client_id.get(), "seq": s.seq.get(), "t_send_ns": s.t_send_ns.get(),
        "data": s.data(),
    });
    message(&s, fields, values)
}

fn state() -> Value {
    let s = StateMsg {
        version: VERSION,
        phase: Phase::Homing as u8,
        robot_mode: robot_mode_code(RobotMode::Reflex),
        has_errors: 1,
        flags: U16::new(FLAG_HOLDING | FLAG_JOINTS),
        client_id: U32::new(0x0A0B_0C0D),
        seq_accepted: U64::new(0x0102_0304_0506_0708),
        t_send_ns_accepted: U64::new(0x1112_1314_1516_1718),
        t_node_ns: U64::new(0x2122_2324_2526_2728),
        robot_time_ms: U64::new(0x3132_3334_3536_3738),
        success_rate: F64::new(0.9375),
        q: floats(1.0),
        dq: floats(2.0),
        tau_ext: floats(3.0),
        o_t_ee: floats(4.0),
        o_f_ext_k: floats(6.0),
        target: floats(7.0),
        accepted: U64::new(0x4142_4344_4546_4748),
        refused: U64::new(0x5152_5354_5556_5758),
        dropped: U64::new(0x6162_6364_6566_6768),
    };
    let fields = vec![
        field!(StateMsg, version, "u8", 1),
        field!(StateMsg, phase, "u8", 1),
        field!(StateMsg, robot_mode, "u8", 1),
        field!(StateMsg, has_errors, "u8", 1),
        field!(StateMsg, flags, "u16", 1),
        field!(StateMsg, client_id, "u32", 1),
        field!(StateMsg, seq_accepted, "u64", 1),
        field!(StateMsg, t_send_ns_accepted, "u64", 1),
        field!(StateMsg, t_node_ns, "u64", 1),
        field!(StateMsg, robot_time_ms, "u64", 1),
        field!(StateMsg, success_rate, "f64", 1),
        field!(StateMsg, q, "f64", 7),
        field!(StateMsg, dq, "f64", 7),
        field!(StateMsg, tau_ext, "f64", 7),
        field!(StateMsg, o_t_ee, "f64", 16),
        field!(StateMsg, o_f_ext_k, "f64", 6),
        field!(StateMsg, target, "f64", 7),
        field!(StateMsg, accepted, "u64", 1),
        field!(StateMsg, refused, "u64", 1),
        field!(StateMsg, dropped, "u64", 1),
    ];
    let values = json!({
        "version": s.version, "phase": s.phase, "robot_mode": s.robot_mode,
        "has_errors": s.has_errors, "flags": s.flags.get(), "client_id": s.client_id.get(),
        "seq_accepted": s.seq_accepted.get(), "t_send_ns_accepted": s.t_send_ns_accepted.get(),
        "t_node_ns": s.t_node_ns.get(), "robot_time_ms": s.robot_time_ms.get(),
        "success_rate": s.success_rate.get(), "q": get(s.q), "dq": get(s.dq),
        "tau_ext": get(s.tau_ext), "o_t_ee": get(s.o_t_ee), "o_f_ext_k": get(s.o_f_ext_k),
        "target": get(s.target), "accepted": s.accepted.get(), "refused": s.refused.get(),
        "dropped": s.dropped.get(),
    });
    message(&s, fields, values)
}

fn gripper() -> Value {
    let s = GripperMsg::new(
        GripperKind::Grasp,
        0x0A0B_0C0D,
        0x0102_0304_0506_0708,
        0x1112_1314_1516_1718,
        0.0425,
        12.5,
    );
    let fields = vec![
        field!(GripperMsg, version, "u8", 1),
        field!(GripperMsg, kind, "u8", 1),
        field!(GripperMsg, _pad, "u16", 1),
        field!(GripperMsg, client_id, "u32", 1),
        field!(GripperMsg, seq, "u64", 1),
        field!(GripperMsg, t_send_ns, "u64", 1),
        field!(GripperMsg, width, "f64", 1),
        field!(GripperMsg, force, "f64", 1),
    ];
    let values = json!({
        "version": s.version, "kind": s.kind, "_pad": s._pad.get(),
        "client_id": s.client_id.get(), "seq": s.seq.get(), "t_send_ns": s.t_send_ns.get(),
        "width": s.width.get(), "force": s.force.get(),
    });
    message(&s, fields, values)
}

fn gripper_state() -> Value {
    let s = GripperStateMsg {
        version: VERSION,
        flags: GRIPPER_CALIBRATED | GRIPPER_GRASPED | GRIPPER_FAULT,
        _pad: U16::ZERO,
        client_id: U32::new(0x0A0B_0C0D),
        t_node_ns: U64::new(0x2122_2324_2526_2728),
        width: F64::new(0.0375),
        commanded: F64::new(0.04),
        max_width: F64::new(0.08),
    };
    let fields = vec![
        field!(GripperStateMsg, version, "u8", 1),
        field!(GripperStateMsg, flags, "u8", 1),
        field!(GripperStateMsg, _pad, "u16", 1),
        field!(GripperStateMsg, client_id, "u32", 1),
        field!(GripperStateMsg, t_node_ns, "u64", 1),
        field!(GripperStateMsg, width, "f64", 1),
        field!(GripperStateMsg, commanded, "f64", 1),
        field!(GripperStateMsg, max_width, "f64", 1),
    ];
    let values = json!({
        "version": s.version, "flags": s.flags, "_pad": s._pad.get(),
        "client_id": s.client_id.get(), "t_node_ns": s.t_node_ns.get(),
        "width": s.width.get(), "commanded": s.commanded.get(), "max_width": s.max_width.get(),
    });
    message(&s, fields, values)
}

/// `UserStopped` as `user_stopped`, the `franka` Python name.
fn snake(name: &str) -> String {
    let mut out = String::new();
    for (i, c) in name.chars().enumerate() {
        if c.is_ascii_uppercase() && i > 0 {
            out.push('_');
        }
        out.push(c.to_ascii_lowercase());
    }
    out
}

fn json_examples() -> Value {
    let full = CmdRequest {
        client_id: 7,
        mode: Some(Kind::Joints),
        speed: Some(0.3),
        episode: Some("pick-0042".into()),
    };
    let episode = EpisodeMsg {
        arm: "fr3".into(),
        recording_id: "fr3-20260912T101500Z".into(),
        file: Some("fr3-20260912T101500Z.rrd".into()),
        t_node_ns: 8_123_456_789,
        phase: EpisodePhase::Start,
    };
    let text = |r: &CmdRequest| serde_json::to_string(r).expect("a request serialises");
    json!({
        "CmdRequest": [text(&CmdRequest::new(7)), text(&full)],
        "CmdReply": [CmdReply::ok().to_json(), CmdReply::err("held by 7").to_json()],
        "EpisodeMsg": [episode.to_json()],
    })
}

fn wire_json() -> Value {
    let phases: Vec<String> = (0u8..)
        .map_while(Phase::from_u8)
        .map(|p| p.to_string())
        .collect();
    let modes: Vec<String> = (0u8..)
        .map_while(RobotMode::from_u8)
        .enumerate()
        .map(|(code, mode)| {
            assert_eq!(usize::from(robot_mode_code(mode)), code);
            snake(&format!("{mode:?}"))
        })
        .collect();
    json!({
        "version": VERSION,
        "messages": {
            "TargetMsg": target(),
            "StateMsg": state(),
            "GripperMsg": gripper(),
            "GripperStateMsg": gripper_state(),
        },
        "flags": {
            "TARGET_ANCHOR": TARGET_ANCHOR,
            "FLAG_HOLDING": FLAG_HOLDING,
            "FLAG_JOINTS": FLAG_JOINTS,
            "GRIPPER_CALIBRATED": GRIPPER_CALIBRATED,
            "GRIPPER_GRASPED": GRIPPER_GRASPED,
            "GRIPPER_MOVING": GRIPPER_MOVING,
            "GRIPPER_FAULT": GRIPPER_FAULT,
        },
        "kinds": {
            Kind::Cartesian.to_string(): Kind::Cartesian as u8,
            Kind::Joints.to_string(): Kind::Joints as u8,
        },
        "gripper_kinds": {"width": GripperKind::Width as u8, "grasp": GripperKind::Grasp as u8},
        "phases": phases,
        "robot_modes": modes,
        "json": json_examples(),
    })
}

#[test]
fn wire_json_matches_the_rust_layout() {
    let expected = wire_json();
    let pretty = serde_json::to_string_pretty(&expected).expect("JSON") + "\n";
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/wire.json");
    if std::env::var_os("FRANKA_NODE_BLESS").is_some() {
        std::fs::write(&path, &pretty).expect("write wire.json");
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; FRANKA_NODE_BLESS=1 writes it", path.display()));
    let actual: Value = serde_json::from_str(&text).expect("wire.json parses");
    assert!(
        actual == expected,
        "{} is not the Rust layout (FRANKA_NODE_BLESS=1 rewrites it); expected:\n{pretty}",
        path.display()
    );
}
