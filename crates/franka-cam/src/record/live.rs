//! The recorder itself: one writer thread and two subscriptions per followed arm.
//!
//! The thread is what touches the Rerun SDK; a Zenoh callback only leaves an episode boundary
//! or a clock pair behind, and a capture thread only leaves a copy of a frame. See the parent
//! module for the file, the clock and what a recorded frame costs.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use std::sync::mpsc::{self, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

use franka_rerun::rerun::{
    EncodedImage, EntityPath, MediaType, RecordingStream, RecordingStreamBuilder,
};
use franka_rerun::{Prefix, APPLICATION_ID, HOST_TIMELINE, TIMELINE};
use log::{debug, error, info, warn};
use zenoh::key_expr::KeyExpr;
use zenoh::pubsub::Subscriber;
use zenoh::{Session, Wait};

use super::{
    same_host, Clock, EpisodeMsg, Phase, RecordSink, StateHead, DRAIN_INTERVAL,
    MAX_EXTRAPOLATION_NS, QUEUE_FRAMES, SESSION_GONE_NS, SILENT_ARM_NS,
};
use crate::config::CamConfig;
use crate::msg::{CameraMsg, Format, TIMESTAMP_MONOTONIC};
use crate::status::CamStats;
use crate::sys::monotonic_ns;

/// One camera the recorder takes frames from.
struct Target {
    name: String,
    /// `<arm>/cam/<name>`, built once.
    entity: EntityPath,
    stats: Arc<CamStats>,
}

impl Target {
    /// The camera `name` recording with `arm`, at `<arm>/cam/<name>` where that arm's camera
    /// view looks.
    fn new(arm: &str, name: &str, stats: Arc<CamStats>) -> Arc<Target> {
        Arc::new(Target {
            name: name.to_string(),
            entity: Prefix::new(arm).entity(&format!("cam/{name}")),
            stats,
        })
    }
}

/// A frame on its way to the writer thread.
struct Shot {
    target: Arc<Target>,
    /// When the frame belongs on the timeline, on this host's monotonic clock: the driver's
    /// start-of-exposure stamp, which is a frame period or more before the dequeue.
    captured_ns: u64,
    bytes: Vec<u8>,
}

/// When a frame happened: `t_capture_ns` when the driver stamped it on `CLOCK_MONOTONIC` (a
/// stream it stamps otherwise is refused at start, so this is the normal path), else the dequeue
/// stamp, which is late by the transfer and the wakeup.
fn captured_ns(header: &CameraMsg) -> u64 {
    let capture = header.t_capture_ns.get();
    if header.flags.get() & TIMESTAMP_MONOTONIC != 0 && capture > 0 {
        capture
    } else {
        header.t_node_ns.get()
    }
}

/// An episode boundary, as the arm published it.
enum Event {
    Start { t_node_ns: u64, id: String },
    End { t_node_ns: u64, id: String },
}

impl Event {
    fn t_node_ns(&self) -> u64 {
        match self {
            Event::Start { t_node_ns, .. } | Event::End { t_node_ns, .. } => *t_node_ns,
        }
    }
}

/// What one arm's Zenoh callbacks write and its writer thread reads. Neither side does I/O
/// under a lock: the callbacks push an event or a clock pair and return.
#[derive(Default)]
struct Shared {
    clock: Mutex<Clock>,
    events: Mutex<Vec<Event>>,
}

/// One open `.rrd`.
struct Episode {
    stream: RecordingStream,
    path: PathBuf,
    /// The arm episode's `RecordingId`: the `end` that closes this file names it.
    id: String,
    /// The episode's own stamp: a frame captured before it is not part of it.
    start_ns: u64,
    logged: u64,
    /// Frames written on the host timeline alone, because no state pair could map their
    /// capture time to the arm's robot clock.
    unmapped: u64,
}

/// The recorder of a node: one writer thread and two subscriptions per followed arm, and
/// the sinks its cameras have not taken yet.
pub struct Recording {
    arms: Vec<Writing>,
    sinks: BTreeMap<String, RecordSink>,
    /// Undeclared when this drops.
    subscribers: Vec<Subscriber<()>>,
}

/// A running writer thread.
struct Writing {
    arm: String,
    stop: Arc<AtomicBool>,
    /// The thread's arming flag, cleared here if the thread is gone: a sink whose writer
    /// panicked would otherwise copy and count every frame for nothing.
    open: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl Recording {
    /// Subscribes for every arm some camera's `record_with` names and starts its writer
    /// thread. Without a `record_dir`, or with no such camera, nothing is started and every
    /// [`Recording::sink`] is `None`.
    ///
    /// Never fails: publishing frames is this node's job, so an arm whose subscriptions or
    /// thread cannot be had is a warning and a camera that records nothing.
    ///
    /// `stats` are the cameras' counters, in the order of `config.cameras`.
    pub fn start(session: &Session, config: &CamConfig, stats: &[Arc<CamStats>]) -> Recording {
        debug_assert_eq!(
            config.cameras.len(),
            stats.len(),
            "one CamStats per camera, in order"
        );
        let mut recording = Recording {
            arms: Vec::new(),
            sinks: BTreeMap::new(),
            subscribers: Vec::new(),
        };
        let Some(dir) = &config.record_dir else {
            return recording;
        };
        let mut by_arm: BTreeMap<&str, Vec<Arc<Target>>> = BTreeMap::new();
        for (camera, stats) in config.cameras.iter().zip(stats) {
            if let Some(arm) = &camera.record_with {
                by_arm.entry(arm).or_default().push(Target::new(
                    arm,
                    &camera.name,
                    Arc::clone(stats),
                ));
            }
        }
        if by_arm.is_empty() {
            warn!(
                "record_dir {} but no camera has record_with: nothing is recorded",
                dir.display()
            );
            return recording;
        }
        for (arm, targets) in by_arm {
            if let Err(e) = recording.follow(session, dir, arm, targets) {
                warn!("arm {arm}: not recording its episodes: {e}");
            }
        }
        recording
    }

    /// One arm: its two subscriptions, its writer thread and its cameras' sinks. Nothing is
    /// left behind on an error — the sinks are the last thing added.
    fn follow(
        &mut self,
        session: &Session,
        dir: &std::path::Path,
        arm: &str,
        targets: Vec<Arc<Target>>,
    ) -> Result<(), String> {
        let shared = Arc::new(Shared::default());
        let (sender, frames) = mpsc::sync_channel::<Shot>(QUEUE_FRAMES);
        let open = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let names: Vec<&str> = targets.iter().map(|t| t.name.as_str()).collect();
        info!(
            "camera(s) {names:?}: recording with arm {arm} into {}",
            dir.display()
        );
        let episodes = episodes(session, arm, &shared, &open)?;
        let states = states(session, arm, &shared)?;
        let writer = Writer {
            arm: arm.to_string(),
            dir: dir.to_path_buf(),
            shared,
            frames,
            targets: targets.clone(),
            open: Arc::clone(&open),
            stop: Arc::clone(&stop),
            episode: None,
            since: monotonic_ns(),
            warned_silent: false,
        };
        let join = std::thread::Builder::new()
            .name(format!("franka-cam-record-{arm}"))
            .spawn(move || writer.run())
            .map_err(|e| e.to_string())?;
        self.subscribers.push(episodes);
        self.subscribers.push(states);
        for target in &targets {
            let sink = sink(Arc::clone(target), sender.clone(), Arc::clone(&open));
            self.sinks.insert(target.name.clone(), sink);
        }
        self.arms.push(Writing {
            arm: arm.to_string(),
            stop,
            open,
            join: Some(join),
        });
        Ok(())
    }

    /// The frame sink of the camera `camera`, once: it is handed to that camera's capture
    /// thread. `None` for a camera that records nothing.
    pub fn sink(&mut self, camera: &str) -> Option<RecordSink> {
        self.sinks.remove(camera)
    }

    /// Closes any open episode and joins the writer threads. Shut the cameras down first:
    /// their sinks hold the senders, and a frame handed over after this is lost.
    pub fn shutdown(mut self) {
        self.finish();
    }

    fn finish(&mut self) {
        self.sinks.clear();
        for arm in &mut self.arms {
            arm.stop.store(true, Relaxed);
        }
        for arm in &mut self.arms {
            if let Some(join) = arm.join.take() {
                if join.join().is_err() {
                    // Nothing writes for this arm any more: disarm its cameras' sinks, or
                    // every frame would be copied and counted as a drop for ever.
                    arm.open.store(false, Relaxed);
                    error!("arm {}: the recorder thread panicked", arm.arm);
                }
            }
        }
    }
}

impl Drop for Recording {
    fn drop(&mut self) {
        self.finish();
    }
}

/// The per-frame closure: the module's "what it costs".
fn sink(target: Arc<Target>, frames: SyncSender<Shot>, open: Arc<AtomicBool>) -> RecordSink {
    let warned = AtomicBool::new(false);
    Box::new(move |header: &CameraMsg, payload: &[u8]| {
        // Outside an episode there is nothing to record and nothing is copied.
        if !open.load(Relaxed) {
            return;
        }
        if header.format() != Some(Format::Mjpeg) {
            target.stats.record_dropped();
            if !warned.swap(true, Relaxed) {
                let format = header
                    .format()
                    .map_or_else(|| format!("format {}", header.format), |f| f.to_string());
                warn!(
                    "camera {}: a {format} stream is published but not recorded; only mjpeg is",
                    target.name
                );
            }
            return;
        }
        let shot = Shot {
            target: Arc::clone(&target),
            captured_ns: captured_ns(header),
            bytes: payload.to_vec(),
        };
        if let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) = frames.try_send(shot) {
            target.stats.record_dropped();
        }
    })
}

/// The arm's episode topic: the boundary goes to the writer thread, because opening a file
/// and flushing one are not a Zenoh callback's work.
///
/// The sinks are armed here rather than there, so the frames of the first 100 ms of an episode
/// are queued instead of missed; the writer disarms them again if it refuses the episode.
fn episodes(
    session: &Session,
    arm: &str,
    shared: &Arc<Shared>,
    open: &Arc<AtomicBool>,
) -> Result<Subscriber<()>, String> {
    let key = KeyExpr::try_from(format!("franka/{arm}/episode")).map_err(|e| e.to_string())?;
    let shared = Arc::clone(shared);
    let open = Arc::clone(open);
    let arm = arm.to_string();
    session
        .declare_subscriber(key)
        .callback(move |sample| {
            let bytes = sample.payload().to_bytes();
            match serde_json::from_slice::<EpisodeMsg>(&bytes) {
                Ok(msg) => {
                    let event = match msg.phase {
                        Phase::Start => {
                            open.store(true, Relaxed);
                            Event::Start {
                                t_node_ns: msg.t_node_ns,
                                id: msg.recording_id,
                            }
                        }
                        // Not disarmed here: the frames between this sample and the writer's
                        // next drain are the ones the `end` stamp sorts out.
                        Phase::End => Event::End {
                            t_node_ns: msg.t_node_ns,
                            id: msg.recording_id,
                        },
                    };
                    shared
                        .events
                        .lock()
                        .unwrap_or_else(PoisonError::into_inner)
                        .push(event);
                }
                Err(e) => warn!("arm {arm}: episode: {e}"),
            }
        })
        .wait()
        .map_err(|e| e.to_string())
}

/// The arm's state topic, for the clock pairs alone.
fn states(session: &Session, arm: &str, shared: &Arc<Shared>) -> Result<Subscriber<()>, String> {
    let key = KeyExpr::try_from(format!("franka/{arm}/state")).map_err(|e| e.to_string())?;
    let shared = Arc::clone(shared);
    let arm = arm.to_string();
    let unreadable = AtomicBool::new(false);
    session
        .declare_subscriber(key)
        .callback(move |sample| {
            let bytes = sample.payload().to_bytes();
            let Some(head) = StateHead::decode(&bytes) else {
                if !unreadable.swap(true, Relaxed) {
                    warn!(
                        "arm {arm}: its state is not a version this node reads ({} bytes); \
                         frames are published and not recorded",
                        bytes.len()
                    );
                }
                return;
            };
            let mut clock = shared.clock.lock().unwrap_or_else(PoisonError::into_inner);
            let foreign = clock.foreign;
            clock.observe(&head, monotonic_ns());
            if foreign == 0 && clock.foreign == 1 {
                warn!(
                    "arm {arm}: its states are stamped on another host's monotonic clock; \
                     frames are published and not recorded, because the robot time of one \
                     would be a guess"
                );
            }
        })
        .wait()
        .map_err(|e| e.to_string())
}

/// One arm's writer thread.
struct Writer {
    arm: String,
    dir: PathBuf,
    shared: Arc<Shared>,
    frames: Receiver<Shot>,
    targets: Vec<Arc<Target>>,
    /// Whether an episode is open, read by every sink before it copies a frame. Armed by the
    /// episode callback so the first frames of one are not missed, disarmed here.
    open: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    episode: Option<Episode>,
    /// The local clock at this thread's start, the baseline of [`SILENT_ARM_NS`].
    since: u64,
    warned_silent: bool,
}

impl Writer {
    fn run(mut self) {
        // The Rerun SDK's own threads are spawned from this one when an episode opens, so
        // a process started with a realtime policy does not hand them one.
        franka_rerun::recorder::drop_realtime_priority();
        loop {
            std::thread::sleep(DRAIN_INTERVAL);
            // Read before the drain, so a frame handed over before the flag was set is
            // still written.
            let stopping = self.stop.load(Relaxed);
            let events = std::mem::take(
                &mut *self
                    .shared
                    .events
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner),
            );
            let (batch, gone) = self.drain();
            self.apply(events, batch);
            self.watch_the_arm(monotonic_ns());
            if stopping || gone {
                break;
            }
        }
        if self.episode.is_some() {
            warn!("arm {}: the node stopped with an episode open", self.arm);
            self.finish();
        }
    }

    /// What the arm's state topic says about the episode the `end` sample may never close, and
    /// about a `record_with` that names no arm at all.
    fn watch_the_arm(&mut self, now: u64) {
        let clock = *self
            .shared
            .clock
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if clock.last_state_ns().is_none()
            && !self.warned_silent
            && now.saturating_sub(self.since) > SILENT_ARM_NS
        {
            self.warned_silent = true;
            warn!(
                "arm {}: nothing on franka/{}/state in {} s: is that the arm's name?",
                self.arm,
                self.arm,
                SILENT_ARM_NS / 1_000_000_000
            );
        }
        let Some(episode) = &self.episode else {
            return;
        };
        // Until a sample arrives the episode's own stamp is the baseline: it is on this clock,
        // `begin` having checked that.
        let (state_at, running_at) = (
            clock.last_state_ns().unwrap_or(episode.start_ns),
            clock.last_running_ns().unwrap_or(episode.start_ns),
        );
        // A killed arm node sends no `end` and stops publishing altogether. The bound is the
        // one on the map, because a frame past it has no robot time either way.
        let silent = now.saturating_sub(state_at) > MAX_EXTRAPOLATION_NS;
        // A dropped `end` leaves the arm talking and out of its session. Only a sample newer
        // than the last in-session one says so, so a gap in the state stream — which is
        // published with `Drop` like everything else — waits for the bound above instead of
        // ending a session that is still running.
        let over = state_at > running_at && now.saturating_sub(running_at) > SESSION_GONE_NS;
        if silent || over {
            let why = if silent {
                "its states stopped"
            } else {
                "it reports no session running"
            };
            warn!(
                "arm {}: closing {} because {why}; an `end` sample is best effort",
                self.arm,
                file_name(&episode.path)
            );
            self.finish();
        }
    }

    /// Everything queued, and whether the last sink is gone.
    fn drain(&self) -> (Vec<Shot>, bool) {
        let mut batch = Vec::new();
        loop {
            match self.frames.try_recv() {
                Ok(shot) => batch.push(shot),
                Err(TryRecvError::Empty) => return (batch, false),
                Err(TryRecvError::Disconnected) => return (batch, true),
            }
        }
    }

    /// The tick's events and frames on one timeline: a frame captured before an event
    /// belongs to the episode that event has not changed yet, which is what keeps the last
    /// frames of an episode in its own file instead of losing them to the `end`.
    fn apply(&mut self, events: Vec<Event>, mut batch: Vec<Shot>) {
        // By capture time, because two cameras' frames interleave in one queue and a chunk of
        // rows in order is what the SDK would otherwise sort itself. The events arrive in the
        // order the arm published them.
        batch.sort_by_key(|shot| shot.captured_ns);
        let mut shots = batch.into_iter().peekable();
        for event in events {
            while shots
                .peek()
                .is_some_and(|shot| shot.captured_ns < event.t_node_ns())
            {
                let shot = shots.next().expect("peeked");
                self.log(shot);
            }
            match event {
                Event::Start { t_node_ns, id } => self.begin(&id, t_node_ns),
                Event::End { id, .. } => self.end(&id),
            }
        }
        for shot in shots {
            self.log(shot);
        }
    }

    /// Opens the episode `id`. A `start` that is refused leaves an open episode alone, because
    /// anything on the network can publish one; a `start` that is good closes it first.
    fn begin(&mut self, id: &str, t_node_ns: u64) {
        if !same_host(t_node_ns, monotonic_ns()) {
            self.refuse(format!(
                "episode {id} is stamped on another host's monotonic clock; not recording, \
                 because every frame's robot time would be a guess"
            ));
            return;
        }
        if !usable_id(id) {
            self.refuse(format!(
                "episode id {id:?} is not a name a file can take; not recording"
            ));
            return;
        }
        self.finish();
        match self.create(id, t_node_ns) {
            Ok(episode) => {
                let file = file_name(&episode.path);
                info!("arm {}: recording to {file}", self.arm);
                self.announce(Some(&file));
                self.episode = Some(episode);
                self.open.store(true, Relaxed);
            }
            Err(e) => self.refuse(format!("not recording episode {id}: {e}")),
        }
    }

    /// A `start` this thread will not honour: the sinks the callback armed are disarmed again,
    /// unless an episode is open and keeps them armed.
    fn refuse(&self, why: String) {
        if self.episode.is_none() {
            self.open.store(false, Relaxed);
        }
        warn!("arm {}: {why}", self.arm);
    }

    fn create(&self, id: &str, start_ns: u64) -> Result<Episode, String> {
        std::fs::create_dir_all(&self.dir).map_err(|e| format!("{}: {e}", self.dir.display()))?;
        let path = unique(&self.dir, id, &self.arm);
        // The episode's id and this crate's application id: with both, a viewer loads this
        // file and the arm's as one recording.
        let stream = RecordingStreamBuilder::new(APPLICATION_ID)
            .recording_id(id)
            .save(&path)
            .map_err(|e| e.to_string())?;
        Ok(Episode {
            stream,
            path,
            id: id.to_string(),
            start_ns,
            logged: 0,
            unmapped: 0,
        })
    }

    /// The `end` of the episode `id` closes it. An `end` for another one is a stray, or a late
    /// sample of an episode already replaced, and closing on it would end a recording that is
    /// running; an `end` without a `start` is nothing at all.
    fn end(&mut self, id: &str) {
        match &self.episode {
            Some(episode) if episode.id == id => self.finish(),
            Some(episode) => warn!(
                "arm {}: an end for episode {id:?} while {:?} is recording; ignored",
                self.arm, episode.id
            ),
            None => {}
        }
    }

    /// Flushes and closes the open episode.
    fn finish(&mut self) {
        let Some(episode) = self.episode.take() else {
            return;
        };
        // Before the flush, so no capture thread copies another frame for this file.
        self.open.store(false, Relaxed);
        self.announce(None);
        if let Err(e) = episode.stream.flush_blocking() {
            warn!("arm {}: flushing {}: {e}", self.arm, episode.path.display());
        }
        drop(episode.stream);
        let unmapped = if episode.unmapped == 0 {
            String::new()
        } else {
            format!(
                ", {} of them with no robot time of the arm's (the host timeline only)",
                episode.unmapped
            )
        };
        info!(
            "arm {}: recorded {} frame(s) into {}{unmapped}",
            self.arm,
            episode.logged,
            file_name(&episode.path)
        );
    }

    /// The open file in the state topic of every camera of this arm, as the arm node's own
    /// status names its episode.
    fn announce(&self, file: Option<&str>) {
        for target in &self.targets {
            target.stats.set_recording(file.map(str::to_string));
        }
    }

    /// One frame, on both of the recording's timelines: the host's clock, which is the frame's
    /// own exposure stamp and always places it, and the arm's `robot_time` when the clock pairs
    /// map it. A frame from outside the open episode is dropped.
    ///
    /// The host stamp is the frame's *data*, not a translation of it: on that axis this camera,
    /// the arm it follows and every other arm of the episode line up, and a camera following one
    /// arm could not be placed against another arm at all through a single robot clock. A frame
    /// the arm's clock cannot place is therefore still written, on the host timeline alone, and
    /// counted as unmapped rather than dropped.
    fn log(&mut self, shot: Shot) {
        let Some(episode) = &mut self.episode else {
            return;
        };
        if shot.captured_ns < episode.start_ns {
            return;
        }
        let clock = *self
            .shared
            .clock
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        episode
            .stream
            .set_duration_secs(HOST_TIMELINE, shot.captured_ns as f64 * 1e-9);
        match clock.robot_time_s(shot.captured_ns) {
            Some(t) => episode.stream.set_duration_secs(TIMELINE, t),
            None => {
                // Cleared rather than left alone: the previous frame's robot time is not this
                // frame's, and a row must not claim one it does not have.
                episode.stream.disable_timeline(TIMELINE);
                episode.unmapped += 1;
            }
        }
        let image = EncodedImage::new(shot.bytes).with_media_type(MediaType::jpeg());
        match episode.stream.log(shot.target.entity.clone(), &image) {
            Ok(()) => episode.logged += 1,
            Err(e) => {
                shot.target.stats.record_dropped();
                debug!("camera {}: logging a frame: {e}", shot.target.name);
            }
        }
    }
}

/// Whether an episode id can be part of a file name. It arrives over the network and the
/// recording is named after it, so anything but the shape the arm node's stems have (`<arm>-<UTC
/// stamp>`, `[A-Za-z0-9_-]+`) is refused rather than reaching a path: `..` or a leading `/`
/// would write outside `record_dir` entirely.
fn usable_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// `dir/<id>-<arm>-cam.rrd`, or `-2`, `-3`, ... while the name is taken: an arm node that
/// restarts inside one second can hand out an id twice, and the first episode's frames
/// must not be overwritten.
///
/// The arm is in the name because the id need not be: a collector that names one episode for
/// two arms gives both the same, and two cameras following different arms of it would otherwise
/// want one path.
fn unique(dir: &Path, id: &str, arm: &str) -> PathBuf {
    let mut path = dir.join(format!("{id}-{arm}-cam.rrd"));
    let mut n = 2;
    while path.exists() {
        path = dir.join(format!("{id}-{arm}-cam-{n}.rrd"));
        n += 1;
    }
    path
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .into_owned()
}

#[cfg(test)]
mod tests;
