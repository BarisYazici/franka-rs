//! Episode recording against the fake: a real `Recorder` writes into a temporary directory
//! (the fake's model is the built-in FER one), the observer is handed to the fake and never
//! called, the targets are logged from the arm thread. The files are read back for the entity
//! prefix, the recording id and the layout.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use franka::CartesianSent;
use franka_rerun::rerun::external::re_log_encoding::Decoder;
use franka_rerun::rerun::log::{Chunk, LogMsg};
use franka_rerun::rerun::StoreKind;

use super::*;

struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("franka-node-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        TempDir(dir)
    }

    fn rrd_files(&self) -> Vec<PathBuf> {
        let mut files: Vec<PathBuf> = std::fs::read_dir(&self.0)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .map(|e| e.path())
                    .filter(|p| p.extension().is_some_and(|x| x == "rrd"))
                    .collect()
            })
            .unwrap_or_default();
        files.sort();
        files
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn recording_rig(dir: &TempDir) -> Rig {
    rig_with(recording_config(dir, "t"))
}

/// An arm named `name` that records into `dir`.
fn recording_config(dir: &TempDir, name: &str) -> ArmConfig {
    let mut config = arm_config();
    config.name = name.to_string();
    config.record_dir = Some(dir.0.clone());
    config
}

/// An `enable` request naming the episode `token`.
fn episode(token: &str) -> CmdRequest {
    CmdRequest {
        episode: Some(token.to_string()),
        ..CmdRequest::new(CLIENT)
    }
}

/// The recording id and every entity path of the `.rrd` at `path` (the blueprint is a store of
/// its own and is not one of them; the SDK's `__properties` is the recording's, not an arm's).
fn store(path: &Path) -> (String, BTreeSet<String>) {
    let reader = std::io::BufReader::new(std::fs::File::open(path).unwrap());
    let (mut id, mut entities) = (String::new(), BTreeSet::new());
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        match message.unwrap() {
            LogMsg::SetStoreInfo(set) if set.info.store_id.kind() == StoreKind::Recording => {
                id = set.info.store_id.recording_id().to_string();
            }
            LogMsg::ArrowMsg(store, arrow) if store.kind() == StoreKind::Recording => {
                let chunk = Chunk::from_arrow_msg(&arrow).unwrap();
                let entity = chunk.entity_path().to_string();
                let entity = entity.trim_start_matches('/').to_string();
                if !entity.starts_with("__") {
                    entities.insert(entity);
                }
            }
            _ => {}
        }
    }
    (id, entities)
}

/// The layout in the `.rrd` at `path` as text: the blueprint store's chunks, which name the
/// entity paths the views are rooted at and the timeline the time panel opens on.
fn layout(path: &Path) -> String {
    let reader = std::io::BufReader::new(std::fs::File::open(path).unwrap());
    let mut text = String::new();
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::ArrowMsg(store, arrow) = message.unwrap() {
            if store.kind() == StoreKind::Blueprint {
                text.push_str(&Chunk::from_arrow_msg(&arrow).unwrap().to_string());
            }
        }
    }
    text
}

#[test]
fn a_session_is_one_file_opened_at_enable_and_closed_at_stop() {
    let dir = TempDir::new("session");
    let mut rig = recording_rig(&dir);
    // The model is loaded at construction, before any session.
    assert_eq!(rig.fake.calls(), ["load_model"]);
    assert_eq!(rig.stats.snapshot("t").recording, None);
    rig.activate();
    assert_eq!(
        rig.fake.calls(),
        ["load_model", "read_once", "collision", "start"]
    );
    let files = dir.rrd_files();
    assert_eq!(files.len(), 1, "{files:?}");
    let name = files[0].file_name().unwrap().to_str().unwrap().to_string();
    assert!(name.starts_with("t-") && name.ends_with("Z.rrd"), "{name}");
    assert_eq!(rig.stats.snapshot("t").recording, Some(name.clone()));
    // The episode names the file and carries the stream's `RecordingId`, the file stem, so a
    // recorder in another process writes its own file under the same id.
    let episode = rig.episodes.lock().unwrap()[0].clone();
    assert_eq!(episode.phase, EpisodePhase::Start);
    assert_eq!(episode.file.as_deref(), Some(name.as_str()));
    assert_eq!(episode.recording_id, name.trim_end_matches(".rrd"));
    assert_eq!(rig.stats.snapshot("t").episode, Some(episode.recording_id));

    rig.target(1, moved(0.01));
    rig.target(2, [f64::NAN, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]);
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.stats.snapshot("t").recording, None);
    let files = dir.rrd_files();
    assert_eq!(files.len(), 1);
    let size = std::fs::metadata(&files[0]).unwrap().len();
    assert!(size > 1000, "{size} bytes");

    // A second session, in the same second or not, is a second file; the model is kept.
    rig.ok(Verb::Enable, CLIENT);
    assert_eq!(
        rig.fake
            .calls()
            .iter()
            .filter(|c| **c == "load_model")
            .count(),
        1
    );
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(dir.rrd_files().len(), 2);
}

#[test]
fn the_gripper_is_recorded_with_the_arm() {
    use crate::arm::gripper::GripperSide;
    use crate::gripper::{Gripper, GripperState};

    /// A hand that reports a fixed state, enough for the recording to have something to log.
    struct FixedHand;
    impl Gripper for FixedHand {
        fn command(&self, _width_m: f64) {}
        fn grasp(&self, _width_m: f64, _force_n: f64) {}
        fn home(&self) -> Result<(), String> {
            Ok(())
        }
        fn stop(&self) {}
        fn state(&self) -> GripperState {
            GripperState {
                width_m: 0.041,
                commanded_m: 0.04,
                max_width_m: 0.08,
                calibrated: true,
                grasped: true,
                ..GripperState::default()
            }
        }
    }

    let dir = TempDir::new("gripper");
    let mut config = arm_config();
    config.record_dir = Some(dir.0.clone());
    // One gripper sample per state tick, so a few ticks are a few samples.
    config.state_hz = crate::arm::GRIPPER_STATE_HZ;
    let side = GripperSide::new(Box::new(FixedHand), |_| {});
    let mut rig = rig_with_gripper(config, Some(side));
    rig.activate();
    for _ in 0..3 {
        rig.machine.tick();
    }
    rig.ok(Verb::Stop, CLIENT);
    let files = dir.rrd_files();
    assert_eq!(files.len(), 1, "{files:?}");
    // The gripper entities are in the file beside the arm's, under the arm's name, so a
    // replay shows both.
    let (_, entities) = store(&files[0]);
    for entity in [
        "t/gripper/width",
        "t/gripper/commanded",
        "t/gripper/grasped",
        "t/gripper/fault",
    ] {
        assert!(entities.contains(entity), "{entity} is not in {entities:?}");
    }
}

#[test]
fn a_bad_meshes_dir_runs_the_session_unrecorded() {
    let dir = TempDir::new("meshes");
    let mut config = arm_config();
    config.record_dir = Some(dir.0.clone());
    config.record_meshes = Some(dir.0.join("no-meshes-here"));
    let mut rig = rig_with(config);
    rig.activate();
    assert_eq!(rig.stats.snapshot("t").recording, None);
    assert!(dir.rrd_files().is_empty());
    rig.target(1, moved(0.01));
    rig.ok(Verb::Stop, CLIENT);
    assert!(dir.rrd_files().is_empty());
}

#[test]
fn a_failed_start_leaves_no_file() {
    let dir = TempDir::new("failed");
    let mut rig = recording_rig(&dir);
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.fake.start_fails.store(true, Ordering::SeqCst);
    assert_eq!(rig.err(Verb::Enable, CLIENT), "busy");
    assert!(dir.rrd_files().is_empty());
    assert_eq!(rig.stats.snapshot("t").recording, None);
}

#[test]
fn home_records_too_and_a_fault_closes_the_file() {
    let dir = TempDir::new("home");
    let mut rig = recording_rig(&dir);
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    // The fake arm is at READY already: the first tick answers the home query.
    rig.fake.ready_after.store(0, Ordering::SeqCst);
    let reply = rig.request(Verb::Home, CmdRequest::new(CLIENT));
    assert_eq!(rig.phase(), Phase::Homing);
    assert_eq!(dir.rrd_files().len(), 1);
    assert!(rig.stats.snapshot("t").recording.is_some());
    rig.machine.tick();
    assert_eq!(reply.recv().unwrap(), CmdReply::ok());
    assert_eq!(rig.phase(), Phase::Acquired);
    assert_eq!(rig.stats.snapshot("t").recording, None);

    rig.ok(Verb::Enable, CLIENT);
    assert!(rig.stats.snapshot("t").recording.is_some());
    rig.fake.set_target_fails.store(true, Ordering::SeqCst);
    rig.target(1, moved(0.01));
    assert_eq!(rig.phase(), Phase::Faulted);
    assert_eq!(rig.stats.snapshot("t").recording, None);
    assert_eq!(dir.rrd_files().len(), 2);
}

#[test]
fn without_a_token_the_file_and_the_id_are_the_arm_and_a_stamp() {
    let dir = TempDir::new("unnamed");
    // A node with two arms, but a session of this one alone: nothing is shared.
    let arms = ["t".to_string(), "other".to_string()];
    let mut rig = rig_of(recording_config(&dir, "t"), &arms, None);
    rig.activate();
    rig.target(1, moved(0.01));
    rig.target(2, [f64::NAN, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]);
    rig.ok(Verb::Stop, CLIENT);
    let files = dir.rrd_files();
    assert_eq!(files.len(), 1, "{files:?}");
    let name = files[0].file_name().unwrap().to_str().unwrap();
    assert!(name.starts_with("t-") && name.ends_with("Z.rrd"), "{name}");
    let (id, entities) = store(&files[0]);
    assert_eq!(id, name.trim_end_matches(".rrd"));
    // Every entity under the arm's name. The loop never runs against the fake, so these are
    // what the arm thread itself logs; `franka-rerun`'s own tests cover the recorder's.
    for entity in &entities {
        assert!(
            entity.starts_with("t/"),
            "{entity} is not under the arm's name"
        );
    }
    for entity in [
        "t/commander/target/x",
        "t/commander/target/seq",
        "t/gripper/width",
        "t/world/target",
        "t/events",
    ] {
        assert!(
            entities.contains(entity),
            "{entity} missing from {entities:?}"
        );
    }
    // No token: this file is nobody else's, so the layout is this arm's alone (the other arm's
    // views would stay empty for ever) and opens on the robot's own clock.
    let layout = layout(&files[0]);
    assert!(layout.contains("t/world"), "{layout}");
    assert!(!layout.contains("other/"), "{layout}");
    assert!(layout.contains("robot_time"), "{layout}");
    assert!(!layout.contains("host_time"), "{layout}");
}

#[test]
fn one_episode_token_makes_two_arms_one_recording() {
    let dir = TempDir::new("two-arms");
    let arms = ["L".to_string(), "R".to_string()];
    let mut left = rig_of(recording_config(&dir, "L"), &arms, None);
    let mut right = rig_of(recording_config(&dir, "R"), &arms, None);
    left.activate_with(episode("pick-0042"));
    right.activate_with(episode("pick-0042"));
    left.target(1, moved(0.01));
    right.target(1, moved(0.02));
    left.ok(Verb::Stop, CLIENT);
    right.ok(Verb::Stop, CLIENT);

    // Two files, one per arm, named after the episode.
    let files = dir.rrd_files();
    let names: Vec<String> = files
        .iter()
        .map(|p| p.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(names, ["pick-0042-L.rrd", "pick-0042-R.rrd"]);
    let (left_id, left_entities) = store(&files[0]);
    let (right_id, right_entities) = store(&files[1]);
    // One recording id: that is what makes a viewer load the two files as one recording.
    assert_eq!(left_id, "pick-0042");
    assert_eq!(right_id, "pick-0042");
    // And disjoint entity paths, or the two arms would land on one set of series.
    assert!(
        left_entities.iter().all(|e| e.starts_with("L/")),
        "{left_entities:?}"
    );
    assert!(
        right_entities.iter().all(|e| e.starts_with("R/")),
        "{right_entities:?}"
    );
    assert!(left_entities.is_disjoint(&right_entities));
    assert!(
        left_entities.contains("L/commander/target/x"),
        "{left_entities:?}"
    );
    assert!(
        right_entities.contains("R/commander/target/x"),
        "{right_entities:?}"
    );
    // Both arms send the same layout, naming both of them, on the one clock they share.
    for file in &files {
        let layout = layout(file);
        for named in ["L/world", "R/world", "L/joints", "R/joints", "host_time"] {
            assert!(
                layout.contains(named),
                "{} does not name {named}",
                file.display()
            );
        }
    }
    // The episode topic publishes the token as the recording id, for a camera node to follow.
    for rig in [&left, &right] {
        let samples = rig.episodes.lock().unwrap().clone();
        assert_eq!(samples.len(), 2, "{samples:?}");
        assert!(
            samples.iter().all(|s| s.recording_id == "pick-0042"),
            "{samples:?}"
        );
    }
    let published = left.episodes.lock().unwrap()[0].clone();
    assert_eq!(published.file.as_deref(), Some("pick-0042-L.rrd"));
    assert_eq!(
        left.stats.snapshot("L").episode.as_deref(),
        Some("pick-0042")
    );
}

#[test]
fn a_token_that_is_not_key_safe_is_refused_and_starts_nothing() {
    let dir = TempDir::new("bad-token");
    let mut rig = recording_rig(&dir);
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    for token in ["", "../etc", "pick 42", "a/b"] {
        let reply = rig.request(Verb::Enable, episode(token)).recv().unwrap();
        let error = reply.error.expect("refused");
        assert!(error.contains("not [A-Za-z0-9_-]"), "{token}: {error}");
        assert_eq!(rig.phase(), Phase::Acquired);
    }
    let reply = rig.request(Verb::Home, episode("..")).recv().unwrap();
    assert!(reply.error.unwrap().contains("not [A-Za-z0-9_-]"));
    // Nothing was opened, nothing was started, and no episode was announced.
    assert!(dir.rrd_files().is_empty());
    assert_eq!(rig.fake.calls(), ["load_model"]);
    assert!(rig.episodes.lock().unwrap().is_empty());
    // A good token then runs.
    rig.activate_with(episode("pick_1"));
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(dir.rrd_files().len(), 1);
    assert_eq!(store(&dir.rrd_files()[0]).0, "pick_1");
}

/// A token is the collector's to keep unique: the second session's file is kept beside the
/// first, but both carry the token as their `RecordingId`, so a viewer handed the directory
/// loads them as one recording. The node does not rename the episode behind the collector's
/// back -- the id is what a camera node and the other arm were told.
#[test]
fn a_reused_token_keeps_both_files_under_the_one_recording_id() {
    let dir = TempDir::new("same-token");
    let mut rig = recording_rig(&dir);
    for _ in 0..2 {
        rig.activate_with(episode("pick-1"));
        rig.ok(Verb::Stop, CLIENT);
    }
    let names: Vec<String> = dir
        .rrd_files()
        .iter()
        .map(|p| p.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(names, ["pick-1-t-2.rrd", "pick-1-t.rrd"]);
    for file in dir.rrd_files() {
        assert_eq!(store(&file).0, "pick-1");
    }
}

#[test]
fn the_torque_backends_goal_cap_and_envelope_are_recorded() {
    let dir = TempDir::new("torque");
    let mut rig = recording_rig(&dir);
    rig.activate();
    let mut observer = rig
        .fake
        .cartesian_observer
        .lock()
        .unwrap()
        .take()
        .expect("the recording hands the loop an observer");
    let sent = CartesianSent {
        pose: rig.fake.state().O_T_EE,
        orientation: [0.0, 0.0, 0.0, 1.0],
        target: [0.4, 0.0, 0.4],
        target_orientation: [0.0, 0.0, 0.0, 1.0],
        velocity: [0.0; 3],
        acceleration: [0.0; 3],
        angular_velocity: [0.0; 3],
        angular_acceleration: [0.0; 3],
        backstop_alteration: 0.0,
        backstop_angular_alteration: 0.0,
        q_goal: READY,
        dq_goal: [0.1; 7],
        cap_scale: 0.5,
        tau: [0.0; 7],
        tau_envelope: [-1.0; 7],
        ik_error: 0.0,
        leash_alteration: 0.0,
        leash_angular_alteration: 0.0,
    };
    let state = rig.fake.state();
    for _ in 0..3 {
        observer(&state, &sent);
    }
    drop(observer);
    rig.ok(Verb::Stop, CLIENT);
    let (_, entities) = store(&dir.rrd_files()[0]);
    for entity in [
        "t/joints/q_goal",
        "t/joints/dq_goal",
        "t/joints/cap_scale",
        "t/joints/tau_envelope",
    ] {
        assert!(entities.contains(entity), "{entity} is not in {entities:?}");
    }
}
