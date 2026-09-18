//! The writer's own body, driven directly: no camera, no arm and no Zenoh, so the episode
//! lifecycle, the closing rules and the frame sink are tested without either node.

use franka_rerun::rerun::external::re_log_encoding::Decoder;
use franka_rerun::rerun::log::{Chunk, LogMsg};
use franka_rerun::rerun::EntityPath;

use super::*;
use crate::config::CamConfig;
use crate::msg::{Capture, TIMESTAMP_MONOTONIC};
use crate::record::{
    StateHead, MAX_EXTRAPOLATION_NS, PHASE_ACTIVE, SESSION_GONE_NS, SILENT_ARM_NS,
};
use crate::status::Cameras;

const MS: u64 = 1_000_000;

/// A directory of this process's own, removed when the test ends.
struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> TempDir {
        let dir = std::env::temp_dir().join(format!("franka-cam-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        TempDir(dir)
    }

    fn path(&self) -> &Path {
        &self.0
    }

    /// The file names in it, sorted; empty when it does not exist.
    fn files(&self) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(&self.0)
            .into_iter()
            .flatten()
            .flatten()
            .map(|entry| entry.file_name().to_string_lossy().into_owned())
            .collect();
        names.sort();
        names
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The camera `name` recording with the arm `arm`.
fn target(arm: &str, name: &str) -> Arc<Target> {
    let camera = format!("[[camera]]\nname = \"{name}\"\ndevice = \"/dev/video9\"\n")
        .parse::<CamConfig>()
        .expect("config")
        .cameras
        .remove(0);
    Target::new(arm, &camera.name, Arc::new(CamStats::new(&camera)))
}

/// The camera's state as the status thread would publish it.
fn state(target: &Arc<Target>) -> crate::status::CamState {
    Cameras::new(vec![(
        target.name.clone(),
        "fake".into(),
        Arc::clone(&target.stats),
    )])
    .tick()
    .remove(0)
}

/// An in-session state sample, the only kind that maps a frame.
fn running(t_node_ns: u64, robot_time_ms: u64) -> StateHead {
    StateHead {
        phase: PHASE_ACTIVE,
        t_node_ns,
        robot_time_ms,
    }
}

/// A writer with `now` mapped to robot time 1 s, or with no map at all, one camera, and the
/// sender its sink would hold.
fn rig(dir: &TempDir, now: Option<u64>) -> (Writer, Arc<Target>, SyncSender<Shot>) {
    rig_for(dir, now, "L", "wrist")
}

/// [`rig`] following the arm `arm` with a camera named `camera`.
fn rig_for(
    dir: &TempDir,
    now: Option<u64>,
    arm: &str,
    camera: &str,
) -> (Writer, Arc<Target>, SyncSender<Shot>) {
    let target = target(arm, camera);
    let (sender, frames) = mpsc::sync_channel::<Shot>(QUEUE_FRAMES);
    let shared = Arc::new(Shared::default());
    if let Some(now) = now {
        let mut clock = shared.clock.lock().expect("clock");
        // Two samples that agree are what a map takes.
        assert!(!clock.observe(&running(now - 10 * MS, 990), now - 10 * MS));
        assert!(clock.observe(&running(now, 1_000), now));
    }
    let writer = Writer {
        arm: arm.to_string(),
        dir: dir.path().to_path_buf(),
        shared,
        frames,
        targets: vec![Arc::clone(&target)],
        open: Arc::new(AtomicBool::new(false)),
        stop: Arc::new(AtomicBool::new(false)),
        episode: None,
        since: now.unwrap_or(0),
        warned_silent: false,
    };
    (writer, target, sender)
}

/// The non-static chunks of the `.rrd` at `path`.
fn chunks(path: &Path) -> Vec<Chunk> {
    let reader = std::io::BufReader::new(std::fs::File::open(path).expect("the recording"));
    let mut chunks = Vec::new();
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::ArrowMsg(_, arrow) = message.expect("a log message") {
            let chunk = Chunk::from_arrow_msg(&arrow).expect("a chunk");
            if !chunk.is_static() {
                chunks.push(chunk);
            }
        }
    }
    chunks
}

/// The timelines of every non-static chunk at `entity` in the `.rrd` at `path`, each sorted.
/// Rerun keeps rows whose timelines differ in chunks of their own, so a frame with no robot
/// time comes back as a set of its own.
fn timeline_sets(path: &Path, entity: &str) -> Vec<Vec<String>> {
    let entity = EntityPath::from(entity);
    let mut sets: Vec<Vec<String>> = chunks(path)
        .iter()
        .filter(|chunk| *chunk.entity_path() == entity)
        .map(|chunk| {
            let mut names: Vec<String> = chunk.timelines().keys().map(|n| n.to_string()).collect();
            names.sort();
            names
        })
        .collect();
    sets.sort();
    sets
}

/// The entity paths the `.rrd` at `path` has rows at, sorted; they print rooted, `/L/cam/wrist`.
fn entities(path: &Path) -> Vec<String> {
    let mut paths: Vec<String> = chunks(path)
        .iter()
        .map(|chunk| chunk.entity_path().to_string())
        .collect();
    paths.sort();
    paths.dedup();
    paths
}

fn shot(target: &Arc<Target>, captured_ns: u64) -> Shot {
    Shot {
        target: Arc::clone(target),
        captured_ns,
        bytes: vec![0xFF, 0xD8, 0xFF, 0xD9],
    }
}

fn start(id: &str, t_node_ns: u64) -> Event {
    Event::Start {
        t_node_ns,
        id: id.to_string(),
    }
}

fn end(id: &str, t_node_ns: u64) -> Event {
    Event::End {
        t_node_ns,
        id: id.to_string(),
    }
}

#[test]
fn a_start_opens_a_file_and_an_end_closes_it() {
    let dir = TempDir::new("lifecycle");
    // The arm node's stamps are this host's monotonic clock, which is what `begin` checks.
    let now = crate::sys::monotonic_ns();
    let (mut writer, target, _sender) = rig(&dir, Some(now));

    // An end without a start is ignored: no directory, no file, no panic.
    writer.apply(vec![end("L-1", now)], Vec::new());
    assert!(writer.episode.is_none() && !dir.path().exists());

    writer.apply(vec![start("L-1", now)], Vec::new());
    assert!(writer.open.load(Relaxed), "the sinks are not armed");
    assert_eq!(state(&target).recording.as_deref(), Some("L-1-L-cam.rrd"));

    // A frame the camera exposed before the episode began is not part of it.
    let shots = vec![shot(&target, now - 1), shot(&target, now + MS)];
    writer.apply(Vec::new(), shots);
    assert_eq!(writer.episode.as_ref().expect("open").logged, 1);

    // The end takes the frames exposed before it with it, and closes the file.
    let shots = vec![shot(&target, now + 2 * MS), shot(&target, now + 4 * MS)];
    writer.apply(vec![end("L-1", now + 3 * MS)], shots);
    assert!(writer.episode.is_none() && !writer.open.load(Relaxed));
    assert_eq!(state(&target).recording, None);
    assert_eq!(dir.files(), ["L-1-L-cam.rrd"]);

    // A frame after the end belongs to no episode and is not a drop either.
    writer.apply(Vec::new(), vec![shot(&target, now + 5 * MS)]);
    assert_eq!(state(&target).record_dropped, 0);
}

#[test]
fn a_second_start_is_a_second_file() {
    let dir = TempDir::new("second");
    let now = crate::sys::monotonic_ns();
    let (mut writer, target, _sender) = rig(&dir, Some(now));
    for (n, id) in ["L-1", "L-2"].into_iter().enumerate() {
        let at = now + n as u64 * 3 * MS;
        writer.apply(vec![start(id, at)], vec![shot(&target, at + MS)]);
        writer.apply(vec![end(id, at + 2 * MS)], Vec::new());
    }
    assert_eq!(dir.files(), ["L-1-L-cam.rrd", "L-2-L-cam.rrd"]);

    // An arm node that restarts inside a second can repeat an id; the first file stays.
    writer.apply(vec![start("L-1", now + 6 * MS)], Vec::new());
    // And a start while one is open closes it rather than leaking the stream.
    writer.apply(vec![start("L-1", now + 8 * MS)], Vec::new());
    writer.apply(vec![end("L-1", now + 10 * MS)], Vec::new());
    assert_eq!(
        dir.files(),
        [
            "L-1-L-cam-2.rrd",
            "L-1-L-cam-3.rrd",
            "L-1-L-cam.rrd",
            "L-2-L-cam.rrd"
        ]
    );
}

#[test]
fn two_cameras_of_one_arm_share_the_episodes_file() {
    let dir = TempDir::new("shared");
    let now = crate::sys::monotonic_ns();
    let (mut writer, wrist, _sender) = rig(&dir, Some(now));
    let scene = target("L", "scene");
    writer.targets.push(Arc::clone(&scene));

    let shots = vec![shot(&wrist, now + MS), shot(&scene, now + MS)];
    writer.apply(vec![start("L-1", now)], shots);
    // One file, one row each, and both cameras' states name it.
    assert_eq!(writer.episode.as_ref().expect("open").logged, 2);
    for camera in [&wrist, &scene] {
        assert_eq!(state(camera).recording.as_deref(), Some("L-1-L-cam.rrd"));
    }
    writer.apply(vec![end("L-1", now + 2 * MS)], Vec::new());
    assert_eq!(dir.files(), ["L-1-L-cam.rrd"]);
    for camera in [&wrist, &scene] {
        assert_eq!(state(camera).recording, None);
    }
}

#[test]
fn two_cameras_following_two_arms_of_one_episode_do_not_share_a_path() {
    let dir = TempDir::new("named");
    let now = crate::sys::monotonic_ns();
    // One episode token, as a collector that enables both arms with it gives them: the two
    // cameras are handed the same `recording_id` and must still write two files, each under
    // its own arm's entities, where that arm's camera view looks.
    let (mut left, wrist, _l) = rig_for(&dir, Some(now), "L", "wrist");
    let (mut right, scene, _r) = rig_for(&dir, Some(now), "R", "scene");
    left.apply(vec![start("pick-1", now)], vec![shot(&wrist, now + MS)]);
    right.apply(vec![start("pick-1", now)], vec![shot(&scene, now + MS)]);
    assert_eq!(state(&wrist).recording.as_deref(), Some("pick-1-L-cam.rrd"));
    assert_eq!(state(&scene).recording.as_deref(), Some("pick-1-R-cam.rrd"));
    assert_eq!(left.episode.as_ref().expect("open").logged, 1);
    assert_eq!(right.episode.as_ref().expect("open").logged, 1);
    left.apply(vec![end("pick-1", now + 2 * MS)], Vec::new());
    right.apply(vec![end("pick-1", now + 2 * MS)], Vec::new());
    assert_eq!(dir.files(), ["pick-1-L-cam.rrd", "pick-1-R-cam.rrd"]);
    let dir = dir.path();
    assert_eq!(entities(&dir.join("pick-1-L-cam.rrd")), ["/L/cam/wrist"]);
    assert_eq!(entities(&dir.join("pick-1-R-cam.rrd")), ["/R/cam/scene"]);
}

#[test]
fn an_episode_id_that_is_not_a_file_name_is_refused() {
    let dir = TempDir::new("id");
    let now = crate::sys::monotonic_ns();
    let (mut writer, target, _sender) = rig(&dir, Some(now));
    // The id comes off the network and the file is named after it.
    for id in [
        "",
        "../../etc/passwd",
        "/tmp/elsewhere",
        "L 1",
        "L/1",
        "L.1",
    ] {
        writer.open.store(true, Relaxed);
        writer.apply(vec![start(id, now)], Vec::new());
        assert!(writer.episode.is_none(), "{id:?} opened a file");
        // The sinks the callback armed for it are disarmed again.
        assert!(!writer.open.load(Relaxed), "{id:?} left the sinks armed");
    }
    assert!(!dir.path().exists());
    assert_eq!(state(&target).recording, None);
    assert!(usable_id("L-20260101T101500Z") && usable_id("L-20260101T101500Z-2"));

    // A garbage start must not end a good episode: anything on the network can publish one.
    writer.apply(vec![start("L-1", now)], Vec::new());
    writer.apply(vec![start("../../x", now + MS)], Vec::new());
    assert!(writer.open.load(Relaxed));
    assert_eq!(state(&target).recording.as_deref(), Some("L-1-L-cam.rrd"));
    writer.apply(Vec::new(), vec![shot(&target, now + 2 * MS)]);
    assert_eq!(writer.episode.as_ref().expect("still open").logged, 1);
}

#[test]
fn an_end_that_names_another_episode_is_ignored() {
    let dir = TempDir::new("stray-end");
    let now = crate::sys::monotonic_ns();
    let (mut writer, target, _sender) = rig(&dir, Some(now));
    writer.apply(vec![start("L-1", now)], Vec::new());
    // A stray sample, or the late `end` of an episode this one replaced: either would end a
    // recording that is running.
    writer.apply(vec![end("L-0", now + MS)], Vec::new());
    assert!(writer.episode.is_some() && writer.open.load(Relaxed));
    assert_eq!(state(&target).recording.as_deref(), Some("L-1-L-cam.rrd"));
    // Its own `end` closes it.
    writer.apply(vec![end("L-1", now + 2 * MS)], Vec::new());
    assert!(writer.episode.is_none());
    assert_eq!(dir.files(), ["L-1-L-cam.rrd"]);
}

#[test]
fn an_episode_stamped_on_another_hosts_clock_is_not_recorded() {
    let dir = TempDir::new("foreign");
    // An hour of uptime that is not this host's: the offset would be a fiction.
    let (mut writer, target, _sender) = rig(&dir, None);
    writer.open.store(true, Relaxed);
    writer.apply(vec![start("L-1", 3_600 * 1_000_000_000)], Vec::new());
    assert!(writer.episode.is_none() && !dir.path().exists());
    assert!(!writer.open.load(Relaxed));
    assert_eq!(state(&target).recording, None);
}

#[test]
fn a_frame_no_state_pair_can_place_is_kept_on_the_host_timeline() {
    let dir = TempDir::new("unmapped");
    let now = crate::sys::monotonic_ns();
    // No map: the arm's states have not arrived yet, or never will. The frame's own exposure
    // stamp is this host's clock either way, so it is written and not dropped.
    let (mut writer, target, _sender) = rig(&dir, None);
    writer.apply(vec![start("L-1", now)], vec![shot(&target, now + MS)]);
    let episode = writer.episode.as_ref().expect("open");
    assert_eq!((episode.logged, episode.unmapped), (1, 1));
    assert_eq!(state(&target).record_dropped, 0);

    // The pair arrives and the next frame carries the arm's clock as well.
    {
        let mut clock = writer.shared.clock.lock().expect("clock");
        assert!(!clock.observe(&running(now + MS, 1_000), now + MS));
        assert!(clock.observe(&running(now + 2 * MS, 1_001), now + 2 * MS));
    }
    writer.apply(Vec::new(), vec![shot(&target, now + 3 * MS)]);
    let episode = writer.episode.as_ref().expect("open");
    assert_eq!((episode.logged, episode.unmapped), (2, 1));
    writer.apply(vec![end("L-1", now + 4 * MS)], Vec::new());
    assert_eq!(dir.files(), ["L-1-L-cam.rrd"]);

    // Both frames carry the host's clock; only the mapped one carries the arm's, and the
    // unmapped row does not inherit the other's.
    let sets = timeline_sets(&dir.path().join("L-1-L-cam.rrd"), "L/cam/wrist");
    assert_eq!(sets.len(), 2, "{sets:?}");
    let has = |set: &Vec<String>, name: &str| set.iter().any(|n| n == name);
    assert!(sets.iter().all(|set| has(set, HOST_TIMELINE)), "{sets:?}");
    assert_eq!(
        sets.iter().filter(|set| has(set, TIMELINE)).count(),
        1,
        "{sets:?}"
    );
}

#[test]
fn an_episode_no_end_closes_is_closed_by_the_writer() {
    let dir = TempDir::new("stale");
    let now = crate::sys::monotonic_ns();
    let (mut writer, target, _sender) = rig(&dir, Some(now));
    writer.apply(vec![start("L-1", now)], Vec::new());

    // While the arm says a session runs, the episode stays open however long it lasts.
    writer.watch_the_arm(now + MAX_EXTRAPOLATION_NS);
    assert!(writer.episode.is_some() && writer.open.load(Relaxed));

    // The arm node was killed: no `end` was ever published and the states stopped.
    writer.watch_the_arm(now + MAX_EXTRAPOLATION_NS + 1);
    assert!(writer.episode.is_none() && !writer.open.load(Relaxed));
    assert_eq!(state(&target).recording, None);
    assert_eq!(dir.files(), ["L-1-L-cam.rrd"]);

    // And the case the arm lives through: the `end` sample was dropped, so the arm goes on
    // publishing states that say no session is running.
    let (mut writer, target, _sender) = rig(&dir, Some(now));
    writer.apply(vec![start("L-2", now)], Vec::new());
    let idle = StateHead {
        phase: 0,
        t_node_ns: now + MS,
        robot_time_ms: 1_001,
    };
    assert!(!writer
        .shared
        .clock
        .lock()
        .expect("clock")
        .observe(&idle, now + MS));
    writer.watch_the_arm(now + SESSION_GONE_NS);
    assert!(writer.episode.is_some(), "closed while still in the window");
    writer.watch_the_arm(now + SESSION_GONE_NS + 2 * MS);
    assert!(writer.episode.is_none() && !writer.open.load(Relaxed));
    assert_eq!(state(&target).recording, None);
    assert_eq!(dir.files(), ["L-1-L-cam.rrd", "L-2-L-cam.rrd"]);
}

#[test]
fn an_arm_that_never_says_anything_is_warned_about_once() {
    let dir = TempDir::new("silent");
    let now = crate::sys::monotonic_ns();
    // A `record_with` that names no arm: nothing ever arrives, and that is the only symptom.
    let (mut writer, _target, _sender) = rig(&dir, None);
    writer.since = now;
    writer.watch_the_arm(now + SILENT_ARM_NS);
    assert!(!writer.warned_silent);
    writer.watch_the_arm(now + SILENT_ARM_NS + 1);
    assert!(writer.warned_silent);

    // An arm that has said something is not that mistake, however long ago it spoke.
    let (mut writer, _target, _sender) = rig(&dir, Some(now));
    writer.since = now;
    writer.watch_the_arm(now + 10 * SILENT_ARM_NS);
    assert!(!writer.warned_silent);
}

#[test]
fn a_frame_is_logged_at_its_exposure_not_its_dequeue() {
    let header = |flags, t_capture_ns| {
        CameraMsg::new(
            Format::Mjpeg,
            2,
            2,
            Capture {
                flags,
                t_capture_ns,
                t_node_ns: 100,
                ..Capture::default()
            },
        )
    };
    // The driver's start-of-exposure stamp, a frame period or more before the dequeue.
    assert_eq!(captured_ns(&header(TIMESTAMP_MONOTONIC, 67)), 67);
    // A stream on another clock cannot be compared with the arm's: the dequeue stamp, which
    // this node took itself, is what is left. (Such a stream is refused at start.)
    assert_eq!(captured_ns(&header(0, 67)), 100);
    assert_eq!(captured_ns(&header(TIMESTAMP_MONOTONIC, 0)), 100);
}

#[test]
fn the_sink_copies_only_while_an_episode_is_open_and_counts_what_it_drops() {
    let target = target("L", "wrist");
    let (sender, frames) = mpsc::sync_channel::<Shot>(QUEUE_FRAMES);
    let open = Arc::new(AtomicBool::new(false));
    let record = sink(Arc::clone(&target), sender, Arc::clone(&open));
    let header = |format| {
        CameraMsg::new(
            format,
            2,
            2,
            Capture {
                flags: TIMESTAMP_MONOTONIC,
                t_capture_ns: 5,
                t_node_ns: 40,
                ..Capture::default()
            },
        )
    };
    let jpeg = header(Format::Mjpeg);

    // Between episodes nothing is copied and nothing is a drop.
    record(&jpeg, &[1, 2, 3]);
    assert!(frames.try_recv().is_err());
    assert_eq!(state(&target).record_dropped, 0);

    open.store(true, Relaxed);
    record(&jpeg, &[1, 2, 3]);
    let shot = frames.try_recv().expect("a frame");
    assert_eq!((shot.captured_ns, shot.bytes), (5, vec![1, 2, 3]));

    // A format the recording does not take: published, not recorded, counted.
    record(&header(Format::Yuyv), &[0; 8]);
    assert!(frames.try_recv().is_err());
    assert_eq!(state(&target).record_dropped, 1);

    // A full queue drops the frame instead of holding the capture thread.
    for _ in 0..QUEUE_FRAMES + 3 {
        record(&jpeg, &[1, 2, 3]);
    }
    assert_eq!(state(&target).record_dropped, 4);
}
