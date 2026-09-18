//! Episode recording: with the `record` feature every session (`enable` in either mode,
//! `home`) is one `.rrd` in the arm's `record_dir`, written by a `franka_rerun::Recorder`
//! whose `push` is the loop's observer; the arm thread logs the targets it accepts and
//! refuses into the same recording; every cycle carries the torque backend's goal, cap
//! scale and envelope torque (`joints/q_goal`, `joints/dq_goal`, `joints/cap_scale`,
//! `joints/tau_envelope`). Without the feature `Recording` has the same surface and does
//! nothing, and nothing from Rerun is compiled.
//!
//! Every entity is logged under the arm's name, and with an episode token ([`super::Machine`]
//! passes `enable`'s) the recording id is that token and the file is `<token>-<arm>.rrd`: two
//! arms enabled with one token write two files that a viewer loads as one recording, on the
//! host timeline both of them carry.

use std::collections::BTreeMap;
use std::sync::{Mutex, PoisonError};
use std::time::{SystemTime, UNIX_EPOCH};

/// The id of a session that has no file of its own: the stem [`episode_file_name`] would
/// give it, so the two kinds of id read the same. A second session within one second gets
/// `-2`, `-3` and so on, as the file names do.
pub(in crate::arm) fn session_id(arm: &str) -> String {
    static LAST: Mutex<BTreeMap<String, (String, u32)>> = Mutex::new(BTreeMap::new());
    let name = episode_file_name(arm, now_unix_secs());
    let stem = name.trim_end_matches(".rrd").to_string();
    let mut last = LAST.lock().unwrap_or_else(PoisonError::into_inner);
    dedup(arm, stem, &mut last)
}

/// `stem`, or `stem-<n>` when this process has handed that stem out for `arm` before. The
/// count is per arm, so a session of one arm cannot reset another's: two arms alternating
/// within one second would otherwise both be handed their first stem twice.
fn dedup(arm: &str, stem: String, last: &mut BTreeMap<String, (String, u32)>) -> String {
    match last.get_mut(arm) {
        Some((previous, n)) if *previous == stem => {
            *n += 1;
            format!("{stem}-{n}")
        }
        _ => {
            last.insert(arm.to_string(), (stem.clone(), 1));
            stem
        }
    }
}

/// `<arm>-<UTC yyyymmddThhmmssZ>.rrd` for `unix_secs`.
fn episode_file_name(arm: &str, unix_secs: u64) -> String {
    let (days, rest) = (unix_secs / 86_400, unix_secs % 86_400);
    let (y, m, d) = civil_from_days(days as i64);
    format!(
        "{arm}-{y:04}{m:02}{d:02}T{:02}{:02}{:02}Z.rrd",
        rest / 3600,
        rest % 3600 / 60,
        rest % 60
    )
}

/// Proleptic Gregorian date of a day count since 1970-01-01 (Howard Hinnant's algorithm).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + i64::from(m <= 2);
    (y, m, d)
}

fn now_unix_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

#[cfg(feature = "record")]
mod live {
    use std::fmt::Display;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;
    use std::time::Instant;

    use franka::robot::target_control::{CartesianObserver, JointObserver};
    use franka::{CartesianSent, JointSent, Model, RobotCommandLog, RobotState};
    use franka_rerun::flight::cartesian::{AXES, TARGET_PREFIX};
    use franka_rerun::rerun::{EntityPath, RecordingStream, Scalars, TextLog, TextLogLevel};
    use franka_rerun::{
        flight, scene, Layout, MeshChoice, Prefix, Recorder, RecorderOptions, RobotKind, TorqueLog,
        HOST_TIMELINE, TIMELINE,
    };
    use log::{info, warn};

    use super::{episode_file_name, now_unix_secs};
    use crate::arm::RobotSide;
    use crate::config::ArmConfig;
    use crate::msg::{
        GripperStateMsg, Kind, TargetMsg, GRIPPER_FAULT, GRIPPER_GRASPED, GRIPPER_MOVING,
    };

    /// `<episode>-<arm>.rrd`: the file of a session the collector named.
    fn named_file_name(episode: &str, arm: &str) -> String {
        format!("{episode}-{arm}.rrd")
    }

    /// `record_meshes`: unset is the built-in set, `"none"` the skeleton alone, anything else a
    /// directory of meshes.
    fn mesh_choice(meshes: Option<&std::path::Path>) -> MeshChoice {
        match meshes {
            None => MeshChoice::default(),
            Some(path) if path.as_os_str() == "none" => MeshChoice::Off,
            Some(dir) => MeshChoice::Dir(dir.to_path_buf()),
        }
    }

    /// What the observer holds: the recorder, and the last cycle's robot time (ms) and host
    /// clock (ns), for stamping what the arm thread logs at the same instant on both timelines.
    struct Shared {
        recorder: Recorder,
        time_ms: AtomicU64,
        host_ns: AtomicU64,
    }

    impl Shared {
        /// One cycle: the record and the torque backend's log into the recorder, and the
        /// cycle's two clocks left behind for whatever the arm thread logs before the next one.
        /// The host's is read once, here, so that the cycle's own rows and the arm thread's
        /// rows carry the same instant.
        fn cycle(&self, state: &RobotState, command: RobotCommandLog, torque: TorqueLog) {
            let host_ns = crate::monotonic_ns();
            self.time_ms
                .store(state.time.as_millis(), Ordering::Relaxed);
            self.host_ns.store(host_ns, Ordering::Relaxed);
            self.recorder
                .push_torque_at(state, Some(command), torque, host_ns);
        }
    }

    /// The entities an accepted target is logged to, built once per episode.
    struct Entities {
        axes: [EntityPath; 3],
        quaternion: EntityPath,
        q: EntityPath,
        seq: EntityPath,
        client_id: EntityPath,
        gripper_width: EntityPath,
        gripper_commanded: EntityPath,
        gripper_grasped: EntityPath,
        gripper_moving: EntityPath,
        gripper_fault: EntityPath,
    }

    impl Entities {
        fn new(prefix: &Prefix) -> Self {
            let under = |name: &str| prefix.entity(&format!("{TARGET_PREFIX}/{name}"));
            let gripper = |name: &str| prefix.entity(&format!("gripper/{name}"));
            Entities {
                axes: AXES.map(under),
                quaternion: under("quaternion"),
                q: under("q"),
                seq: under("seq"),
                client_id: under("client_id"),
                gripper_width: gripper("width"),
                gripper_commanded: gripper("commanded"),
                gripper_grasped: gripper("grasped"),
                gripper_moving: gripper("moving"),
                gripper_fault: gripper("fault"),
            }
        }
    }

    /// One session's recording.
    struct Episode {
        shared: Arc<Shared>,
        stream: RecordingStream,
        entities: Entities,
        /// The arm's entity prefix, for what the arm thread logs into the 3D scene.
        prefix: Prefix,
        path: PathBuf,
        /// The stream's `RecordingId`: the episode token, or the file's stem without one.
        /// Files written under the same id -- another arm's, a camera's -- load as one
        /// recording with this one.
        id: String,
        kind: Kind,
    }

    /// The arm's recorder state across sessions: the directory, the model (loaded at
    /// construction and kept), the layout every session sends and the open episode. Without a
    /// model there is no recording.
    pub(in crate::arm) struct Recording {
        dir: Option<PathBuf>,
        meshes: MeshChoice,
        model: Option<Arc<Model>>,
        /// This arm's entity prefix and the prefixes of every arm of the node, in config
        /// order: the layout names all of them, so that a recording two arms share lays out
        /// both whichever arm's blueprint the viewer sees.
        prefix: Prefix,
        arms: Vec<Prefix>,
        episode: Option<Episode>,
    }

    impl Recording {
        /// With a `record_dir`, loads the model now, on the node's normal-priority thread and
        /// before any session; a failure is a warning and disables recording for the arm.
        pub(in crate::arm) fn new(
            config: &ArmConfig,
            arms: &[String],
            robot: &impl RobotSide,
        ) -> Self {
            let prefix = Prefix::new(&config.name);
            // At least this arm: a layout that named none would lay out the unprefixed paths
            // nothing writes to, and the whole recording would be invisible.
            let mut layout: Vec<Prefix> = arms.iter().map(Prefix::new).collect();
            if layout.is_empty() {
                layout.push(prefix.clone());
            }
            let mut recording = Recording {
                dir: config.record_dir.clone(),
                meshes: mesh_choice(config.record_meshes.as_deref()),
                model: None,
                prefix,
                arms: layout,
                episode: None,
            };
            if recording.dir.is_some() {
                match robot.load_model() {
                    Ok(model) => recording.model = Some(Arc::new(model)),
                    Err(e) => {
                        warn!("arm {}: no model, not recording: {e}", config.name);
                        recording.dir = None;
                    }
                }
            }
            recording
        }

        /// Opens the episode file of a `kind` session, under the collector's `episode` token
        /// when there is one; a failure is logged and the session runs unrecorded.
        pub(in crate::arm) fn begin(
            &mut self,
            robot: &impl RobotSide,
            arm: &str,
            kind: Kind,
            episode: Option<&str>,
        ) {
            let Some(dir) = self.dir.clone() else {
                return;
            };
            // Every session end closes its episode; a leftover would be a bug, not a leak.
            self.finish(arm);
            match self.open(robot, &dir, arm, kind, episode) {
                Ok(episode) => {
                    info!("arm {arm}: recording to {}", episode.path.display());
                    self.episode = Some(episode);
                }
                Err(e) => warn!("arm {arm}: not recording this session: {e}"),
            }
        }

        fn open(
            &self,
            robot: &impl RobotSide,
            dir: &std::path::Path,
            arm: &str,
            kind: Kind,
            episode: Option<&str>,
        ) -> Result<Episode, String> {
            let model = Arc::clone(self.model.as_ref().ok_or("no model")?);
            let robot_kind = RobotKind::from(robot.fci_version());
            // The recorder thread loads the meshes at its start; probe them here so that a bad
            // directory refuses the session's recording instead of killing that thread.
            self.meshes.resolve(robot_kind).map_err(|e| e.to_string())?;
            std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
            // With a token the collector owns the id, and the file is named after it and the
            // arm; without one the id is the file's own stem.
            let (path, id) = match episode {
                Some(token) => (unique(dir, &named_file_name(token, arm)), token.to_string()),
                None => {
                    let path = unique(dir, &episode_file_name(arm, now_unix_secs()));
                    let id = path
                        .file_stem()
                        .map(|s| s.to_string_lossy().into_owned())
                        .ok_or("no file stem")?;
                    (path, id)
                }
            };
            // A named episode is the only recording several arms share: it gets the layout of
            // all of them, and the time panel on `host_time`, the one axis two controllers'
            // clocks agree on. An unnamed session is this arm's file alone, so it keeps the
            // layout of this arm alone -- the others' views would stay empty for ever -- and
            // opens on the robot's own clock.
            let (arms, timeline) = match episode {
                Some(_) => (self.arms.clone(), HOST_TIMELINE),
                None => (vec![self.prefix.clone()], TIMELINE),
            };
            let options = RecorderOptions {
                flight: flight::FlightOptions {
                    prefix: self.prefix.clone(),
                    meshes: self.meshes.clone(),
                    ..RecorderOptions::default().flight
                },
                layout: Some(Layout {
                    arms,
                    timeline: timeline.to_string(),
                }),
                ..RecorderOptions::default()
            };
            // Under the episode's id as the `RecordingId`: what the arm publishes on the
            // episode topic, so another process, or another arm, can write beside this file.
            let recorder = Recorder::to_file_with_id(&path, &id, model, robot_kind, options)
                .map_err(|e| e.to_string())?;
            let stream = recorder.stream();
            let prefix = self.prefix.clone();
            if kind == Kind::Cartesian {
                flight::log_target_styles(&stream, &prefix).map_err(|e| e.to_string())?;
            }
            // Logged whether or not the arm has a hand: a session without one simply leaves the
            // series empty, and one with a hand gets named, coloured plots rather than paths.
            flight::log_gripper_styles(&stream, &prefix).map_err(|e| e.to_string())?;
            Ok(Episode {
                shared: Arc::new(Shared {
                    recorder,
                    time_ms: AtomicU64::new(0),
                    host_ns: AtomicU64::new(0),
                }),
                stream,
                entities: Entities::new(&prefix),
                prefix,
                path,
                id,
                kind,
            })
        }

        /// The open episode's file name, for the status.
        pub(in crate::arm) fn file_name(&self) -> Option<String> {
            let episode = self.episode.as_ref()?;
            Some(episode.path.file_name()?.to_string_lossy().into_owned())
        }

        /// The open episode's `RecordingId`, for the `episode` topic.
        pub(in crate::arm) fn recording_id(&self) -> Option<String> {
            Some(self.episode.as_ref()?.id.clone())
        }

        /// The Cartesian loop's observer: one `push` per cycle, no allocation.
        pub(in crate::arm) fn cartesian_observer(&self) -> Option<CartesianObserver> {
            let shared = Arc::clone(&self.episode.as_ref()?.shared);
            Some(Box::new(move |state: &RobotState, sent: &CartesianSent| {
                let command = RobotCommandLog {
                    O_T_EE_c: sent.pose,
                    ..RobotCommandLog::default()
                };
                let torque = TorqueLog {
                    q_goal: sent.q_goal,
                    dq_goal: sent.dq_goal,
                    cap_scale: sent.cap_scale,
                    tau_envelope: sent.tau_envelope,
                };
                shared.cycle(state, command, torque);
            }))
        }

        /// The joint loop's observer, likewise.
        pub(in crate::arm) fn joint_observer(&self) -> Option<JointObserver> {
            let shared = Arc::clone(&self.episode.as_ref()?.shared);
            Some(Box::new(move |state: &RobotState, sent: &JointSent| {
                let command = RobotCommandLog {
                    q_c: sent.q,
                    ..RobotCommandLog::default()
                };
                let torque = TorqueLog {
                    q_goal: sent.q_goal,
                    dq_goal: sent.dq_goal,
                    cap_scale: sent.cap_scale,
                    tau_envelope: sent.tau_envelope,
                };
                shared.cycle(state, command, torque);
            }))
        }

        /// An accepted target: `commander/target/*` at the loop's latest robot time.
        pub(in crate::arm) fn accepted(&self, msg: &TargetMsg) {
            let Some(episode) = &self.episode else {
                return;
            };
            let (rec, at) = (&episode.stream, &episode.entities);
            episode.stamp();
            let data = msg.data();
            let result = (|| -> franka_rerun::Result<()> {
                match episode.kind {
                    Kind::Cartesian => {
                        for (entity, value) in at.axes.iter().zip(&data) {
                            rec.log(entity.clone(), &Scalars::single(*value))?;
                        }
                        let quaternion = Scalars::new(data[3..].iter().copied());
                        rec.log(at.quaternion.clone(), &quaternion)?;
                        let position = [data[0], data[1], data[2]];
                        let color = franka_rerun::TARGET;
                        scene::log_point(rec, &episode.prefix, "target", &position, 0.015, color)?;
                    }
                    Kind::Joints => rec.log(at.q.clone(), &Scalars::new(data))?,
                }
                rec.log(at.seq.clone(), &Scalars::single(msg.seq.get() as f64))?;
                let client = Scalars::single(f64::from(msg.client_id.get()));
                rec.log(at.client_id.clone(), &client)?;
                Ok(())
            })();
            if let Err(e) = result {
                warn!("recording the target failed: {e}");
            }
        }

        /// The gripper as the arm thread last read it: `gripper/*` at the loop's latest robot
        /// time, so a replay has the hand beside the arm that carried it.
        ///
        /// The stamp is the arm's, not the hand's: the driver reads the width on its own thread
        /// and the message carries the monotonic time of that read, which is dropped here. A
        /// series that goes flat is therefore a hand that stopped answering as much as one that
        /// stopped moving, and `gripper/fault` is what tells the two apart.
        pub(in crate::arm) fn gripper(&self, msg: &GripperStateMsg) {
            let Some(episode) = &self.episode else {
                return;
            };
            let (rec, at) = (&episode.stream, &episode.entities);
            episode.stamp();
            let flag = |bit: u8| f64::from(u8::from(msg.flags & bit != 0));
            let result = (|| -> franka_rerun::Result<()> {
                rec.log(at.gripper_width.clone(), &Scalars::single(msg.width.get()))?;
                let commanded = Scalars::single(msg.commanded.get());
                rec.log(at.gripper_commanded.clone(), &commanded)?;
                rec.log(
                    at.gripper_grasped.clone(),
                    &Scalars::single(flag(GRIPPER_GRASPED)),
                )?;
                rec.log(
                    at.gripper_moving.clone(),
                    &Scalars::single(flag(GRIPPER_MOVING)),
                )?;
                // A hand that faults mid-episode otherwise leaves no trace in the recording
                // that would explain what the arm did next.
                rec.log(
                    at.gripper_fault.clone(),
                    &Scalars::single(flag(GRIPPER_FAULT)),
                )?;
                Ok(())
            })();
            if let Err(e) = result {
                warn!("recording the gripper failed: {e}");
            }
        }

        /// A refused target: a warning in the `events` log with the reason.
        pub(in crate::arm) fn refused(&self, msg: &TargetMsg, reason: &dyn Display) {
            let Some(episode) = &self.episode else {
                return;
            };
            episode.stamp();
            let text = format!(
                "target seq {} from {} refused: {reason}",
                msg.seq.get(),
                msg.client_id.get()
            );
            let log = TextLog::new(text).with_level(TextLogLevel::WARN);
            if let Err(e) = episode.stream.log(episode.prefix.path("events"), &log) {
                warn!("recording the refusal failed: {e}");
            }
        }

        /// The session is over: the file is flushed and closed and the counts logged. The
        /// loop has been joined, so the observer's clone of the recorder is gone; `finish`
        /// then waits on the recorder thread's last drain and the file sink's flush, both
        /// bounded by the local disk (no viewer is involved).
        pub(in crate::arm) fn finish(&mut self, arm: &str) {
            let Some(episode) = self.episode.take() else {
                return;
            };
            let path = episode.path.display().to_string();
            drop(episode.stream);
            let started = Instant::now();
            match Arc::try_unwrap(episode.shared) {
                Ok(shared) => match shared.recorder.finish() {
                    Ok(stats) => info!(
                        "arm {arm}: recorded {path}: {} pushed, {} dropped, closed in {:?}",
                        stats.pushed,
                        stats.dropped,
                        started.elapsed()
                    ),
                    Err(e) => warn!("arm {arm}: closing {path}: {e}"),
                },
                // The channel closes with the last clone and the recorder thread writes what
                // it has; nothing waits for it.
                Err(_) => warn!("arm {arm}: {path}: the loop still holds the recorder"),
            }
        }

        /// The session never started: the file is closed and removed.
        pub(in crate::arm) fn discard(&mut self, arm: &str) {
            let Some(path) = self.episode.as_ref().map(|e| e.path.clone()) else {
                return;
            };
            self.finish(arm);
            if let Err(e) = std::fs::remove_file(&path) {
                warn!("arm {arm}: removing {}: {e}", path.display());
            }
        }
    }

    impl Episode {
        /// Sets the stream's time on both timelines, to the loop's latest cycle: its robot time
        /// and the host clock read in the same observer call.
        ///
        /// Both stamps are the cycle's, not the arm thread's own: a target or a gripper reading
        /// is logged as this thread handles it, which is after that cycle and, if the thread was
        /// held up, well after it. One instant on both axes is what keeps such a row where the
        /// 1 kHz rows around it are. Before the first cycle (a target refused at a session's very
        /// start) there is nothing to carry and the clock is read here.
        fn stamp(&self) {
            let t = self.shared.time_ms.load(Ordering::Relaxed) as f64 * 1e-3;
            self.stream.set_duration_secs(TIMELINE, t);
            let host = match self.shared.host_ns.load(Ordering::Relaxed) {
                0 => crate::monotonic_ns(),
                ns => ns,
            };
            self.stream
                .set_duration_secs(HOST_TIMELINE, host as f64 * 1e-9);
        }
    }

    /// `dir/<name>`, or `-2`, `-3`, ... before `.rrd` while the name is taken (two sessions
    /// within one second, or one episode token used twice).
    fn unique(dir: &std::path::Path, name: &str) -> PathBuf {
        let stem = name.trim_end_matches(".rrd");
        let mut path = dir.join(name);
        let mut n = 2;
        while path.exists() {
            path = dir.join(format!("{stem}-{n}.rrd"));
            n += 1;
        }
        path
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn a_named_episode_file_is_the_token_and_the_arm() {
            assert_eq!(named_file_name("pick-0042", "L"), "pick-0042-L.rrd");
            assert_eq!(named_file_name("e", "fr3"), "e-fr3.rrd");
        }

        #[test]
        fn record_meshes_default_to_the_builtin_set_and_none_turns_them_off() {
            assert_eq!(mesh_choice(None), MeshChoice::Builtin);
            assert_eq!(mesh_choice(Some("none".as_ref())), MeshChoice::Off);
            let dir = PathBuf::from("/m");
            assert_eq!(mesh_choice(Some(&dir)), MeshChoice::Dir(dir));
        }
    }
}

#[cfg(feature = "record")]
pub(super) use live::Recording;

/// Without the `record` feature: the same surface, nothing behind it.
#[cfg(not(feature = "record"))]
mod inert {
    use std::fmt::Display;

    use franka::robot::target_control::{CartesianObserver, JointObserver};

    use crate::arm::RobotSide;
    use crate::config::ArmConfig;
    use crate::msg::{GripperStateMsg, Kind, TargetMsg};

    pub(in crate::arm) struct Recording;

    impl Recording {
        pub(in crate::arm) fn new(
            _config: &ArmConfig,
            _arms: &[String],
            _robot: &impl RobotSide,
        ) -> Self {
            Recording
        }
        pub(in crate::arm) fn begin(
            &mut self,
            _robot: &impl RobotSide,
            _arm: &str,
            _kind: Kind,
            _episode: Option<&str>,
        ) {
        }
        pub(in crate::arm) fn file_name(&self) -> Option<String> {
            None
        }
        pub(in crate::arm) fn recording_id(&self) -> Option<String> {
            None
        }
        pub(in crate::arm) fn cartesian_observer(&self) -> Option<CartesianObserver> {
            None
        }
        pub(in crate::arm) fn joint_observer(&self) -> Option<JointObserver> {
            None
        }
        pub(in crate::arm) fn accepted(&self, _msg: &TargetMsg) {}
        pub(in crate::arm) fn gripper(&self, _msg: &GripperStateMsg) {}
        pub(in crate::arm) fn refused(&self, _msg: &TargetMsg, _reason: &dyn Display) {}
        pub(in crate::arm) fn finish(&mut self, _arm: &str) {}
        pub(in crate::arm) fn discard(&mut self, _arm: &str) {}
    }
}

#[cfg(not(feature = "record"))]
pub(super) use inert::Recording;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn file_name_is_the_arm_and_a_utc_stamp() {
        assert_eq!(episode_file_name("L", 0), "L-19700101T000000Z.rrd");
        // 2026-09-11 12:17:05 UTC.
        assert_eq!(
            episode_file_name("fr3", 1_789_129_025),
            "fr3-20260911T121705Z.rrd"
        );
        // A leap day, the last second of it.
        assert_eq!(
            episode_file_name("a", 1_709_251_199),
            "a-20240229T235959Z.rrd"
        );
    }

    #[test]
    fn a_session_id_is_the_stem_of_the_file_name() {
        let id = session_id("L");
        assert!(id.starts_with("L-") && id.ends_with('Z'), "{id}");
    }

    #[test]
    fn ids_repeated_within_one_second_are_counted_per_arm() {
        // Two sessions in one second are two ids, as two files in one second are two files;
        // a later second starts over.
        let mut last = BTreeMap::new();
        let stem = "L-19700101T000000Z";
        assert_eq!(dedup("L", stem.into(), &mut last), stem);
        assert_eq!(dedup("L", stem.into(), &mut last), format!("{stem}-2"));
        assert_eq!(dedup("L", stem.into(), &mut last), format!("{stem}-3"));
        assert_eq!(
            dedup("L", "L-19700101T000001Z".into(), &mut last),
            "L-19700101T000001Z"
        );
        // The other arm's sessions must not reset this arm's count: both arms starting a
        // second session in the same second is exactly when the ids would collide.
        let (l, r) = ("L-19700101T000002Z", "R-19700101T000002Z");
        assert_eq!(dedup("L", l.into(), &mut last), l);
        assert_eq!(dedup("R", r.into(), &mut last), r);
        assert_eq!(dedup("L", l.into(), &mut last), format!("{l}-2"));
        assert_eq!(dedup("R", r.into(), &mut last), format!("{r}-2"));
    }
}
