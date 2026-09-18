//! Frames into the arm episode's recording: with the `record` feature a camera whose
//! `record_with` names an arm follows that arm's `franka/<arm>/episode`, and between a `start`
//! and an `end` every frame is logged as a Rerun `EncodedImage` under `<arm>/cam/<name>`. Frames
//! taken outside an episode are published and not recorded. Without the feature [`Recording`]
//! has the same surface, does nothing, and nothing from Rerun is compiled.
//!
//! # The file
//!
//! `<record_dir>/<recording_id>-<arm>-cam.rrd`, written under the episode's own `RecordingId`
//! and `franka_rerun::APPLICATION_ID`, so a viewer handed the arm's file and this one loads them
//! as a single recording. One file per arm episode whatever the number of cameras following that
//! arm: their entities differ, so their rows do not collide, and one writer means one file handle
//! and one batcher. The arm is part of the name because the id need not be: a collector that
//! names one episode for two arms gives both the same id, and two cameras following different
//! arms of it would otherwise want one path.
//!
//! A frame is logged on both of the recording's timelines: `franka_rerun::HOST_TIMELINE`, the
//! capture stamp itself, which is the axis two arms and their cameras all agree on, and the
//! arm's `robot_time`, the same stamp mapped through the clock pairs below. A frame the pairs
//! cannot map is still written, on the host timeline alone: the host stamp is the frame's own
//! and is always valid, and a camera following one arm can only be placed against another arm
//! on it.
//!
//! The archival path is the file, not the wire: a consumer in the cloud gets the preview key
//! live and the episode's files afterwards, which is why the recording exists at full rate
//! while the frame key may not be subscribed at all.
//!
//! # The clock
//!
//! A frame is logged at the time the camera began its exposure, `t_capture_ns`, which the driver
//! stamps on this host's `CLOCK_MONOTONIC` (a stream stamped on any other clock is refused at
//! start, and only such a stream falls back on the dequeue stamp). The recording's timeline is
//! the robot's own clock, so the two need a map, and `franka/<arm>/state` carries both clocks of
//! one instant.
//!
//! It is a map only under two conditions, both enforced by [`Clock`]:
//!
//! - The sample's stamp can be this host's ([`same_host`]). On another host the offset is a
//!   fiction — two unrelated boot counts — so nothing is recorded and one warning says so: an
//!   empty recording is honest, a frame stamped with a robot time it never had is not. A camera
//!   off the arm's host would need a clock synchronisation query, which is not implemented.
//! - A session is running ([`StateHead::session_running`]) and two consecutive samples imply the
//!   same offset to within [`OFFSET_AGREEMENT_NS`]. An idle arm refreshes the state it publishes
//!   `robot_time_ms` from at most once a second, and keeps the last one when its read of the
//!   robot fails, while `t_node_ns` is taken at every encode: such a sample is two clocks read
//!   seconds apart, not a pair, and believing it would move every frame of the episode's start.
//!
//! # What it costs
//!
//! One `Vec<u8>` per frame, a copy of the frame bytes, and only while an episode is open: the
//! sink reads one atomic first, so between episodes the capture thread allocates nothing. That
//! copy is what recording needs in any case, since Rerun's `EncodedImage` owns its blob, and it
//! is taken from the sample the capture thread already built, after the driver's buffer is back
//! in its queue. The copy crosses to a writer thread through a `sync_channel` of
//! [`QUEUE_FRAMES`] with `try_send`, so the capture thread never waits on the Rerun SDK or on
//! the disk; a full queue drops the frame and counts it in the camera's `record_dropped`.
//!
//! Only MJPEG is recorded. A raw or H.264 stream is refused with one warning per camera: YUYV
//! is twenty times the bytes of the same frame as JPEG (18 MB/s at 480p30, a third of an SD
//! card's write rate), and this recorder does not use Rerun's H.264 `VideoStream`, which needs
//! an `ffmpeg` binary in the viewer. The frames still go out on the wire.
//!
//! # When nothing closes the episode
//!
//! The `end` sample is best effort — the arm node's publisher drops rather than blocks, and a
//! killed node sends none at all — so the writer closes the file itself when the arm's states say
//! no session is running ([`SESSION_GONE_NS`]) or stop arriving altogether
//! ([`MAX_EXTRAPOLATION_NS`]). Without that a dropped `end` would leave the file open, and the
//! state topic naming it, until the camera node is stopped.
//!
//! In the other direction, an `end` is honoured only for the episode it names, and a `start`
//! that is refused leaves an open one alone: every sample here comes off the network, and one
//! stray or late message must not end a recording that is running.

use std::time::Duration;

use serde::{Deserialize, Serialize};
use zerocopy::little_endian::{U16, U32, U64};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

use crate::msg::CameraMsg;

/// A frame offered to the recorder: the sample's header and its frame bytes, borrowed from the
/// sample the capture thread built. Called once per frame on the capture thread, after the
/// frame has gone to the publisher, and it must not block: see the module documentation.
pub type RecordSink = Box<dyn Fn(&CameraMsg, &[u8]) + Send>;

/// Frames the hand-off to a writer thread holds. Eight is a quarter of a second at 30 fps and
/// at most 8 frames of memory in flight (a megabyte at 720p), which is the trade: a deeper
/// queue hides a longer disk stall and costs that much more resident memory on a 2 GB board.
pub const QUEUE_FRAMES: usize = 8;

/// How often a writer thread drains the queue and applies the episode events it was handed.
pub const DRAIN_INTERVAL: Duration = Duration::from_millis(100);

/// How far behind the local clock a state sample's stamp may be and still be this host's. A
/// sample crosses Zenoh in well under a millisecond on one host; two hosts' monotonic clocks
/// differ by their boot times, which is seconds at best and days in a lab.
pub const BEHIND_TOLERANCE_NS: u64 = 1_000_000_000;

/// How far ahead of the local clock it may be: the stamp is taken before the sample is sent, so
/// on one host it can only be ahead by the receive path's own scheduling.
pub const AHEAD_TOLERANCE_NS: u64 = 50_000_000;

/// How much two consecutive in-session samples' implied offsets may differ and still be one
/// offset. Both clocks are read in the same encode, one from the control loop's latest cycle,
/// so the difference is that cycle's age plus scheduling: a millisecond or two. Far below the
/// second an idle arm's stale snapshot would show, which is what this catches.
pub const OFFSET_AGREEMENT_NS: u64 = 10_000_000;

/// How far from its pair a stamp is still mapped. The two clocks drift by tens of ppm, so the
/// offset is good to microseconds for as long as the pair holds; past this the pair is too old
/// to answer for, and a robot time is not a thing to guess. (Robot time is the controller's own
/// message count, so it survives an arm-node restart and resets only with the controller.)
pub const MAX_EXTRAPOLATION_NS: u64 = 5_000_000_000;

/// How long an open episode survives an arm that reports no session running. Covers the `end`
/// sample that was dropped while the arm node lives: a session is over long before this.
pub const SESSION_GONE_NS: u64 = 2_000_000_000;

/// How long a followed arm may say nothing at all before the recorder warns once. A
/// `record_with` that names no arm is silent forever, and a typo has no other symptom.
pub const SILENT_ARM_NS: u64 = 10_000_000_000;

/// Whether a `t_node_ns` from another process can be this host's `CLOCK_MONOTONIC`, given the
/// local clock `now_ns` at receipt.
pub fn same_host(t_node_ns: u64, now_ns: u64) -> bool {
    t_node_ns <= now_ns.saturating_add(AHEAD_TOLERANCE_NS)
        && now_ns.saturating_sub(t_node_ns) <= BEHIND_TOLERANCE_NS
}

/// The offset a sample implies: the robot's clock minus the host's, ns.
fn offset_ns(t_node_ns: u64, robot_time_ms: u64) -> i128 {
    robot_time_ms as i128 * 1_000_000 - t_node_ns as i128
}

/// The map from this host's `CLOCK_MONOTONIC` to the robot's clock, and what the arm's state
/// topic says about the session: see the module documentation for both.
///
/// The map is an offset and the slope is 1, because the arm node and this one read one clock.
/// A slope fitted from two samples 10 ms apart, each quantised to the robot's millisecond, would
/// be wrong by up to 10 %, while the real drift is tens of ppm — microseconds over the seconds a
/// pair answers for.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Clock {
    /// The map: the newest in-session sample whose offset the one before it agreed with.
    pair: Option<(u64, u64)>,
    /// The newest in-session sample, which becomes the map when the next one agrees with it.
    candidate: Option<(u64, u64)>,
    /// The local clock at the last sample this version could read, session or not.
    last_state_ns: Option<u64>,
    /// The local clock at the last sample that reported a session running.
    last_running_ns: Option<u64>,
    /// Samples refused because their stamp is not this host's `CLOCK_MONOTONIC`.
    pub foreign: u64,
}

impl Clock {
    /// A state sample received at `now_ns` on the local monotonic clock. `true` when it and the
    /// sample before it agree on one offset, which is when the map exists; see the module
    /// documentation for every reason a sample is not a pair.
    pub fn observe(&mut self, head: &StateHead, now_ns: u64) -> bool {
        if !same_host(head.t_node_ns, now_ns) {
            self.foreign += 1;
            return false;
        }
        self.last_state_ns = Some(now_ns);
        if !head.session_running() {
            // Out of a session the two clocks in a sample were read up to a second apart, so it
            // is not a pair and cannot become the candidate. The map itself is kept: robot time
            // is the controller's own count and runs across sessions, so the offset is still the
            // offset, and `MAX_EXTRAPOLATION_NS` bounds how long it is trusted. Dropping it here
            // would lose the frames of the stop phase, which is part of the episode.
            self.candidate = None;
            return false;
        }
        self.last_running_ns = Some(now_ns);
        // A sample that overtook a newer one, or a repeat, says nothing the last one does not.
        if self
            .candidate
            .is_some_and(|(held, _)| head.t_node_ns <= held)
        {
            return false;
        }
        let offset = offset_ns(head.t_node_ns, head.robot_time_ms);
        let held = self.candidate;
        let agrees = held.is_some_and(|(t, ms)| {
            (offset - offset_ns(t, ms)).abs() <= OFFSET_AGREEMENT_NS as i128
        });
        self.candidate = Some((head.t_node_ns, head.robot_time_ms));
        if agrees {
            self.pair = self.candidate;
        } else if held.is_some() {
            // Two in-session samples that disagree are a stale snapshot or a restarted
            // controller: nothing is mapped until two of them agree again.
            self.pair = None;
        }
        // Neither: the first sample of a session, whose own pair is not established yet. Any map
        // a previous session left is still the offset, so it stands.
        agrees
    }

    /// The robot time in seconds of `t_ns` on this host's monotonic clock; `None` without a
    /// pair or further than [`MAX_EXTRAPOLATION_NS`] from it.
    pub fn robot_time_s(&self, t_ns: u64) -> Option<f64> {
        let (t_node_ns, robot_time_ms) = self.pair?;
        if t_ns.abs_diff(t_node_ns) > MAX_EXTRAPOLATION_NS {
            return None;
        }
        let offset_ns = t_ns as f64 - t_node_ns as f64;
        Some(robot_time_ms as f64 * 1e-3 + offset_ns * 1e-9)
    }

    /// The local clock at the last state sample this version could read, `None` if the arm has
    /// said nothing.
    pub fn last_state_ns(&self) -> Option<u64> {
        self.last_state_ns
    }

    /// The local clock at the last sample that reported a session running.
    pub fn last_running_ns(&self) -> Option<u64> {
        self.last_running_ns
    }
}

/// Start or end of an episode, the `phase` of [`EpisodeMsg`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Phase {
    Start,
    End,
}

/// What this node reads of the arm node's `franka/<arm>/episode` JSON.
///
/// A subset: `file` and anything a later version adds are ignored, so only the id and the
/// stamp are a contract. A phase this version does not know is an error rather than a guess.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EpisodeMsg {
    /// The Rerun `RecordingId` of the arm's episode; the camera file is written under it.
    pub recording_id: String,
    /// The arm host's `CLOCK_MONOTONIC` at publish, the clock of a frame's `t_node_ns`.
    pub t_node_ns: u64,
    pub phase: Phase,
}

/// Bytes of a `franka/<arm>/state` sample the recorder reads: its head.
pub const STATE_PREFIX_SIZE: usize = 42;

/// The protocol version of a state sample this version can read.
pub const STATE_VERSION: u8 = 1;

/// `franka_node::msg::Phase::Active` as its wire byte: a session is running.
pub const PHASE_ACTIVE: u8 = 2;

/// `franka_node::msg::Phase::Homing` as its wire byte: a `home` session is running.
pub const PHASE_HOMING: u8 = 5;

/// The head of a `franka/<arm>/state` sample: the phase and the two clocks of one instant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StateHead {
    /// The arm's state machine phase as its wire byte; [`PHASE_ACTIVE`] and [`PHASE_HOMING`]
    /// are the two in which a session runs.
    pub phase: u8,
    /// The arm host's `CLOCK_MONOTONIC` at encode.
    pub t_node_ns: u64,
    /// `RobotState.time`, ms.
    pub robot_time_ms: u64,
}

/// The head of a state sample, up to and including the two clocks.
///
/// `franka-node` is deliberately not a dependency of a camera host's binary — it is a
/// dev-dependency, for the test that pins these offsets to its own `StateMsg` — so the three
/// fields the recorder needs are read from the prefix every `StateMsg` begins with.
#[repr(C)]
#[derive(FromBytes, IntoBytes, KnownLayout, Immutable, Debug, Default, Clone, Copy)]
struct StatePrefix {
    version: u8,
    phase: u8,
    robot_mode: u8,
    has_errors: u8,
    flags: U16,
    client_id: U32,
    seq_accepted: U64,
    t_send_ns_accepted: U64,
    t_node_ns: U64,
    robot_time_ms: U64,
}

/// The prefix is the head of the arm node's `StateMsg`, byte for byte.
const _: () = assert!(std::mem::size_of::<StatePrefix>() == STATE_PREFIX_SIZE);

impl StateHead {
    /// Whether the arm reports a session running, which is the only phase in which its
    /// `robot_time_ms` and `t_node_ns` were read together.
    pub fn session_running(&self) -> bool {
        matches!(self.phase, PHASE_ACTIVE | PHASE_HOMING)
    }

    /// The head of a state sample, or `None` when `bytes` is not one this version reads.
    ///
    /// The length and the version byte are all there is to check, so any payload of at least
    /// [`STATE_PREFIX_SIZE`] bytes that begins with [`STATE_VERSION`] decodes into something:
    /// what makes it a state is the key expression it arrived on.
    pub fn decode(bytes: &[u8]) -> Option<StateHead> {
        let (prefix, _) = StatePrefix::read_from_prefix(bytes).ok()?;
        (prefix.version == STATE_VERSION).then_some(StateHead {
            phase: prefix.phase,
            t_node_ns: prefix.t_node_ns.get(),
            robot_time_ms: prefix.robot_time_ms.get(),
        })
    }

    /// The head of a state sample carrying this phase and these clocks: what a test publishes
    /// in place of an arm node's 474-byte state.
    pub fn encode(&self) -> [u8; STATE_PREFIX_SIZE] {
        let prefix = StatePrefix {
            version: STATE_VERSION,
            phase: self.phase,
            t_node_ns: U64::new(self.t_node_ns),
            robot_time_ms: U64::new(self.robot_time_ms),
            ..StatePrefix::default()
        };
        let mut bytes = [0u8; STATE_PREFIX_SIZE];
        bytes.copy_from_slice(prefix.as_bytes());
        bytes
    }
}

#[cfg(feature = "record")]
mod live;
#[cfg(feature = "record")]
pub use live::Recording;

/// Without the `record` feature: the same surface, nothing behind it. A `record_dir` is refused
/// by the configuration in such a build, so there is nothing to say here either.
#[cfg(not(feature = "record"))]
mod inert {
    use std::sync::Arc;

    use zenoh::Session;

    use super::RecordSink;
    use crate::config::CamConfig;
    use crate::status::CamStats;

    pub struct Recording;

    impl Recording {
        pub fn start(
            _session: &Session,
            _config: &CamConfig,
            _stats: &[Arc<CamStats>],
        ) -> Recording {
            Recording
        }
        pub fn sink(&mut self, _camera: &str) -> Option<RecordSink> {
            None
        }
        pub fn shutdown(self) {}
    }
}

#[cfg(not(feature = "record"))]
pub use inert::Recording;

#[cfg(test)]
mod tests;
