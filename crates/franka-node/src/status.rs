//! The node's status: per-arm atomics the arm thread stores into on every state tick and
//! the transport's target callback counts decode failures into, read by the status publisher
//! once a second into the JSON of `franka/node/<name>/status`.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

use serde::Serialize;

use crate::msg::{
    GripperStateMsg, Kind, Phase, StateMsg, GRIPPER_CALIBRATED, GRIPPER_FAULT, GRIPPER_GRASPED,
};

/// What one arm reports, as plain atomics: no lock between the arm thread and the reader.
#[derive(Default)]
pub struct ArmStats {
    phase: AtomicU8,
    holder: AtomicU32,
    /// A [`Kind`] while a session runs, 0 otherwise.
    mode: AtomicU8,
    accepted: AtomicU64,
    refused: AtomicU64,
    dropped: AtomicU64,
    /// Targets whose bytes were not a `TargetMsg`; counted by the transport.
    pub decode_failures: AtomicU64,
    /// `f64::to_bits` of the robot's `control_command_success_rate`.
    success_rate: AtomicU64,
    /// The file name of the episode being recorded; set at the session's start and end only.
    recording: Mutex<Option<String>>,
    /// The last session's `recording_id`; set at the session's start, kept after its end.
    episode: Mutex<Option<String>>,
    /// A gripper state has been recorded: the arm has a gripper.
    gripper: AtomicBool,
    gripper_flags: AtomicU8,
    /// `f64::to_bits` of the gripper's width.
    gripper_width: AtomicU64,
}

impl ArmStats {
    /// Stores what the gripper's `state` says.
    pub fn record_gripper(&self, state: &GripperStateMsg) {
        self.gripper_flags.store(state.flags, Ordering::Relaxed);
        self.gripper_width
            .store(state.width.get().to_bits(), Ordering::Relaxed);
        self.gripper.store(true, Ordering::Relaxed);
    }

    /// The episode file being recorded, `None` when none is.
    pub fn set_recording(&self, file_name: Option<String>) {
        *self
            .recording
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = file_name;
    }

    /// The id of the episode that started last.
    pub fn set_episode(&self, recording_id: Option<String>) {
        *self.episode.lock().unwrap_or_else(PoisonError::into_inner) = recording_id;
    }

    /// Stores what `state` says; `mode` is the running session's kind, if any.
    pub fn record(&self, state: &StateMsg, mode: Option<Kind>) {
        self.phase.store(state.phase, Ordering::Relaxed);
        self.holder.store(state.client_id.get(), Ordering::Relaxed);
        self.mode
            .store(mode.map_or(0, |kind| kind as u8), Ordering::Relaxed);
        self.accepted.store(state.accepted.get(), Ordering::Relaxed);
        self.refused.store(state.refused.get(), Ordering::Relaxed);
        self.dropped.store(state.dropped.get(), Ordering::Relaxed);
        self.success_rate
            .store(state.success_rate.get().to_bits(), Ordering::Relaxed);
    }

    /// One arm's entry of the status JSON.
    pub fn snapshot(&self, name: &str) -> ArmStatus {
        ArmStatus {
            name: name.to_string(),
            phase: Phase::from_u8(self.phase.load(Ordering::Relaxed)).unwrap_or(Phase::Idle),
            holder: self.holder.load(Ordering::Relaxed),
            mode: Kind::from_u8(self.mode.load(Ordering::Relaxed)),
            accepted: self.accepted.load(Ordering::Relaxed),
            refused: self.refused.load(Ordering::Relaxed),
            dropped: self.dropped.load(Ordering::Relaxed),
            decode_failures: self.decode_failures.load(Ordering::Relaxed),
            success_rate: f64::from_bits(self.success_rate.load(Ordering::Relaxed)),
            recording: self
                .recording
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone(),
            episode: self
                .episode
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone(),
            gripper: self.gripper.load(Ordering::Relaxed).then(|| {
                let flags = self.gripper_flags.load(Ordering::Relaxed);
                GripperStatus {
                    width: f64::from_bits(self.gripper_width.load(Ordering::Relaxed)),
                    calibrated: flags & GRIPPER_CALIBRATED != 0,
                    grasped: flags & GRIPPER_GRASPED != 0,
                    fault: flags & GRIPPER_FAULT != 0,
                }
            }),
        }
    }
}

/// The gripper in [`ArmStatus::gripper`].
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GripperStatus {
    /// m.
    pub width: f64,
    pub calibrated: bool,
    pub grasped: bool,
    pub fault: bool,
}

/// One arm in [`Status::arms`].
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ArmStatus {
    pub name: String,
    pub phase: Phase,
    /// The lease holder, 0 if none.
    pub holder: u32,
    /// The running session's kind; `null` without a session.
    pub mode: Option<Kind>,
    /// Targets since enable.
    pub accepted: u64,
    pub refused: u64,
    pub dropped: u64,
    /// Since the node started.
    pub decode_failures: u64,
    /// The robot's `control_command_success_rate`, meaningful while Active.
    pub success_rate: f64,
    /// The `.rrd` file name of the episode being recorded (the `record` feature); `null`
    /// without one.
    pub recording: Option<String>,
    /// The `recording_id` of the last episode (see `franka/<arm>/episode`); `null` before the
    /// first session.
    pub episode: Option<String>,
    /// The arm's gripper as of its last published state; `null` without one.
    pub gripper: Option<GripperStatus>,
}

/// The JSON of `franka/node/<name>/status`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Status {
    pub node: String,
    /// `CARGO_PKG_VERSION` of the node.
    pub version: &'static str,
    pub uptime_s: u64,
    pub arms: Vec<ArmStatus>,
}

impl Status {
    /// The status of `node` after `uptime_s`, one entry per `(arm name, stats)`.
    pub fn new(node: &str, uptime_s: u64, arms: &[(String, Arc<ArmStats>)]) -> Self {
        Status {
            node: node.to_string(),
            version: env!("CARGO_PKG_VERSION"),
            uptime_s,
            arms: arms
                .iter()
                .map(|(name, stats)| stats.snapshot(name))
                .collect(),
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("strings, integers, a float and enums serialise")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocopy::little_endian::{F64, U32, U64};
    use zerocopy::FromZeros;

    #[test]
    fn stats_start_idle_and_empty() {
        let stats = Arc::new(ArmStats::default());
        let status = Status::new("n", 0, &[("a".to_string(), stats)]);
        assert_eq!(
            status.to_json(),
            format!(
                r#"{{"node":"n","version":"{}","uptime_s":0,"arms":[{{"name":"a","phase":"idle","holder":0,"mode":null,"accepted":0,"refused":0,"dropped":0,"decode_failures":0,"success_rate":0.0,"recording":null,"episode":null,"gripper":null}}]}}"#,
                env!("CARGO_PKG_VERSION")
            )
        );
    }

    #[test]
    fn status_json_follows_the_recorded_state() {
        let mut state = StateMsg::new_zeroed();
        state.phase = Phase::Active as u8;
        state.client_id = U32::new(7);
        state.accepted = U64::new(120);
        state.refused = U64::new(2);
        state.dropped = U64::new(1);
        state.success_rate = F64::new(0.98);
        let left = Arc::new(ArmStats::default());
        left.record(&state, Some(Kind::Cartesian));
        left.decode_failures.fetch_add(3, Ordering::Relaxed);
        left.set_recording(Some("L-20260101T120000Z.rrd".into()));
        left.set_episode(Some("L-20260101T120000Z".into()));
        state.phase = Phase::Homing as u8;
        let right = Arc::new(ArmStats::default());
        right.record(&state, Some(Kind::Joints));
        let mut gripper = GripperStateMsg::new_zeroed();
        gripper.flags = GRIPPER_CALIBRATED | GRIPPER_GRASPED;
        gripper.width = F64::new(0.04);
        right.record_gripper(&gripper);

        let arms = [("L".to_string(), left), ("R".to_string(), right)];
        let status = Status::new("node1", 42, &arms);
        assert_eq!(status.node, "node1");
        assert_eq!(status.version, env!("CARGO_PKG_VERSION"));
        assert_eq!(status.uptime_s, 42);
        assert_eq!(
            status.arms[0],
            ArmStatus {
                name: "L".to_string(),
                phase: Phase::Active,
                holder: 7,
                mode: Some(Kind::Cartesian),
                accepted: 120,
                refused: 2,
                dropped: 1,
                decode_failures: 3,
                success_rate: 0.98,
                recording: Some("L-20260101T120000Z.rrd".into()),
                episode: Some("L-20260101T120000Z".into()),
                gripper: None,
            }
        );
        assert_eq!(
            status.arms[1].gripper,
            Some(GripperStatus {
                width: 0.04,
                calibrated: true,
                grasped: true,
                fault: false,
            })
        );
        let json: serde_json::Value = serde_json::from_str(&status.to_json()).unwrap();
        assert_eq!(json["arms"][0]["phase"], "active");
        assert_eq!(json["arms"][0]["recording"], "L-20260101T120000Z.rrd");
        assert_eq!(json["arms"][1]["recording"], serde_json::Value::Null);
        assert_eq!(json["arms"][0]["mode"], "cartesian");
        assert_eq!(json["arms"][0]["success_rate"], 0.98);
        assert_eq!(json["arms"][1]["name"], "R");
        assert_eq!(json["arms"][1]["phase"], "homing");
        assert_eq!(json["arms"][1]["mode"], "joints");
        assert_eq!(json["arms"][1]["decode_failures"], 0);
        assert_eq!(json["arms"][0]["gripper"], serde_json::Value::Null);
        assert_eq!(json["arms"][1]["gripper"]["width"], 0.04);
        assert_eq!(json["arms"][1]["gripper"]["grasped"], true);
        assert_eq!(json["arms"][1]["gripper"]["fault"], false);
        assert_eq!(json["arms"].as_array().unwrap().len(), 2);
    }
}
