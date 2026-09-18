//! What the node reports: per-camera atomics the capture thread stores into on every frame,
//! read once a second into the JSON of `franka/cam/<name>/state` and, all of them together,
//! `franka/node/<node>/status` — the same branch the arm node publishes its own health on, so
//! one subscription to `franka/node/*/status` sees every node of a lab.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering::Relaxed};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Instant;

use serde::Serialize;

use crate::config::CameraConfig;
use crate::msg::Format;
use crate::source::SourceInfo;

/// One camera's counters, as plain atomics: no lock between its capture thread and the status
/// thread that reads them.
pub struct CamStats {
    /// A [`Format`] as its wire byte.
    format: AtomicU8,
    width: AtomicU32,
    height: AtomicU32,
    /// What the config asked for, which the driver need not have granted.
    fps_set: AtomicU32,
    frames: AtomicU64,
    bytes: AtomicU64,
    /// Gaps in the driver's sequence: frames the camera took that never reached the node.
    driver_dropped: AtomicU64,
    /// Frames the session would not take. **Not** frames a congested link dropped: with
    /// `CongestionControl::Drop` Zenoh discards those without telling the publisher, so they
    /// show only as gaps in `seq` at the consumer.
    publish_dropped: AtomicU64,
    /// Frames the driver flagged, and dequeues that failed.
    errors: AtomicU64,
    /// Frames an open episode did not record; see [`CamState::record_dropped`].
    record_dropped: AtomicU64,
    /// Opens of the device after the first; a camera that flaps counts up here.
    reopens: AtomicU64,
    streaming: AtomicBool,
    ts_monotonic: AtomicBool,
    ts_soe: AtomicBool,
    /// The `.rrd` the frames are going into, set by the recorder when an episode opens.
    recording: Mutex<Option<String>>,
    /// What the source reported it opened, `None` before the first open.
    device: Mutex<Option<String>>,
}

impl CamStats {
    /// The counters of a camera that has not opened yet: the format and size are what the
    /// config asked for until [`CamStats::streaming`] replaces them with what it granted.
    pub fn new(config: &CameraConfig) -> CamStats {
        CamStats {
            format: AtomicU8::new(config.format as u8),
            width: AtomicU32::new(config.width.into()),
            height: AtomicU32::new(config.height.into()),
            fps_set: AtomicU32::new(config.fps),
            frames: AtomicU64::new(0),
            bytes: AtomicU64::new(0),
            driver_dropped: AtomicU64::new(0),
            publish_dropped: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            record_dropped: AtomicU64::new(0),
            reopens: AtomicU64::new(0),
            streaming: AtomicBool::new(false),
            ts_monotonic: AtomicBool::new(false),
            ts_soe: AtomicBool::new(false),
            recording: Mutex::new(None),
            device: Mutex::new(None),
        }
    }

    /// The camera is open and streaming what `info` says.
    pub fn streaming(&self, info: &SourceInfo) {
        *self.device.lock().unwrap_or_else(PoisonError::into_inner) = Some(info.device.clone());
        self.format.store(info.format as u8, Relaxed);
        self.width.store(info.width.into(), Relaxed);
        self.height.store(info.height.into(), Relaxed);
        self.ts_monotonic.store(info.ts_monotonic, Relaxed);
        self.ts_soe.store(info.ts_soe, Relaxed);
        self.streaming.store(true, Relaxed);
    }

    /// The camera is not delivering: it is being opened, or it went away.
    pub fn stopped(&self) {
        self.streaming.store(false, Relaxed);
    }

    /// A frame of `bytes` was published, `gap` frames having been lost by the driver before it;
    /// `error` is the driver's flag on the buffer.
    pub fn frame(&self, bytes: usize, gap: u32, error: bool) {
        self.frames.fetch_add(1, Relaxed);
        self.bytes.fetch_add(bytes as u64, Relaxed);
        if gap != 0 {
            self.driver_dropped.fetch_add(gap.into(), Relaxed);
        }
        if error {
            self.errors.fetch_add(1, Relaxed);
        }
    }

    /// The publisher would not take a frame.
    pub fn publish_dropped(&self) {
        self.publish_dropped.fetch_add(1, Relaxed);
    }

    /// An open episode did not record a frame.
    pub fn record_dropped(&self) {
        self.record_dropped.fetch_add(1, Relaxed);
    }

    /// A dequeue failed, or a frame never arrived.
    pub fn error(&self) {
        self.errors.fetch_add(1, Relaxed);
    }

    /// The device was reopened.
    pub fn reopened(&self) {
        self.reopens.fetch_add(1, Relaxed);
    }

    /// The `.rrd` the frames are being recorded into, `None` when none is.
    pub fn set_recording(&self, file_name: Option<String>) {
        *self
            .recording
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = file_name;
    }

    fn frames(&self) -> u64 {
        self.frames.load(Relaxed)
    }
}

/// One camera in [`Status::cameras`], and the JSON of `franka/cam/<name>/state`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CamState {
    pub name: String,
    /// What the camera actually opened, or the device the config names before the first open.
    pub device: String,
    pub format: Format,
    /// The size the driver granted, or the one asked for before the first open.
    pub width: u16,
    pub height: u16,
    /// What the config asked for.
    pub fps_set: u32,
    /// Frames per second over the last status period.
    pub fps: f64,
    /// Whether the camera is open and delivering.
    pub streaming: bool,
    /// Frames dequeued since the node started, the ones the publisher refused included.
    pub frames: u64,
    /// Gaps in the driver's sequence: frames the camera took that never reached the node.
    pub driver_dropped: u64,
    /// Frames the publisher refused.
    pub publish_dropped: u64,
    /// Frames the driver flagged as erroneous, plus failed dequeues.
    pub errors: u64,
    /// Reopens of the device since the node started.
    pub reopens: u64,
    /// Frames an open episode did not record: the hand-off to the recorder was full, the
    /// format is not one a recording takes (only MJPEG is), or the frame's capture time could
    /// not be placed on the arm's clock. 0 without the `record` feature and outside an episode.
    pub record_dropped: u64,
    /// Published bytes per second over the last status period, header included.
    pub bytes_per_s: f64,
    /// Whether the capture stamps are `CLOCK_MONOTONIC`, and at the start of exposure.
    pub ts_monotonic: bool,
    pub ts_soe: bool,
    /// The `.rrd` the frames go to (the `record` feature); `null` when none.
    pub recording: Option<String>,
}

/// The cameras a status thread watches: their atomics and what it read last, so a rate can be
/// a rate rather than an average since boot.
pub struct Cameras {
    watched: Vec<Watched>,
}

struct Watched {
    name: String,
    device: String,
    stats: Arc<CamStats>,
    frames: u64,
    bytes: u64,
    at: Instant,
}

impl Cameras {
    /// Watches each `(name, device, stats)`.
    pub fn new(cameras: Vec<(String, String, Arc<CamStats>)>) -> Cameras {
        let now = Instant::now();
        Cameras {
            watched: cameras
                .into_iter()
                .map(|(name, device, stats)| Watched {
                    frames: stats.frames(),
                    bytes: stats.bytes.load(Relaxed),
                    name,
                    device,
                    stats,
                    at: now,
                })
                .collect(),
        }
    }

    /// The cameras' names, in the order [`Cameras::tick`] reports them.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.watched.iter().map(|watched| watched.name.as_str())
    }

    /// One camera per entry, with the rates measured against the previous call.
    pub fn tick(&mut self) -> Vec<CamState> {
        let now = Instant::now();
        self.watched
            .iter_mut()
            .map(|watched| watched.state(now))
            .collect()
    }
}

impl Watched {
    fn state(&mut self, now: Instant) -> CamState {
        let stats = &self.stats;
        let (frames, bytes) = (stats.frames(), stats.bytes.load(Relaxed));
        let window = now.duration_since(self.at).as_secs_f64();
        let per_s = |delta: u64| {
            if window > 0.0 {
                delta as f64 / window
            } else {
                0.0
            }
        };
        let state = CamState {
            name: self.name.clone(),
            // What opened, when something has; the config's name until then.
            device: stats
                .device
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone()
                .unwrap_or_else(|| self.device.clone()),
            format: Format::from_u8(stats.format.load(Relaxed)).unwrap_or_default(),
            width: stats.width.load(Relaxed) as u16,
            height: stats.height.load(Relaxed) as u16,
            fps_set: stats.fps_set.load(Relaxed),
            fps: per_s(frames.saturating_sub(self.frames)),
            streaming: stats.streaming.load(Relaxed),
            frames,
            driver_dropped: stats.driver_dropped.load(Relaxed),
            publish_dropped: stats.publish_dropped.load(Relaxed),
            errors: stats.errors.load(Relaxed),
            reopens: stats.reopens.load(Relaxed),
            record_dropped: stats.record_dropped.load(Relaxed),
            bytes_per_s: per_s(bytes.saturating_sub(self.bytes)),
            ts_monotonic: stats.ts_monotonic.load(Relaxed),
            ts_soe: stats.ts_soe.load(Relaxed),
            recording: stats
                .recording
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone(),
        };
        (self.frames, self.bytes, self.at) = (frames, bytes, now);
        state
    }
}

/// The JSON of `franka/node/<node>/status`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Status {
    pub node: String,
    /// `CARGO_PKG_VERSION` of the camera node.
    pub version: &'static str,
    pub uptime_s: u64,
    pub cameras: Vec<CamState>,
}

impl Status {
    /// The status of `node` after `uptime_s`, with the `cameras` of one [`Cameras::tick`].
    pub fn new(node: &str, uptime_s: u64, cameras: Vec<CamState>) -> Status {
        Status {
            node: node.to_string(),
            version: env!("CARGO_PKG_VERSION"),
            uptime_s,
            cameras,
        }
    }

    pub fn to_json(&self) -> String {
        json(self)
    }
}

impl CamState {
    pub fn to_json(&self) -> String {
        json(self)
    }
}

fn json<T: Serialize>(value: &T) -> String {
    serde_json::to_string(value).expect("strings, numbers, bools and an enum serialise")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera() -> CameraConfig {
        "[[camera]]\nname = \"wrist\"\ndevice = \"/dev/video2\"\n"
            .parse::<crate::CamConfig>()
            .expect("config")
            .cameras
            .remove(0)
    }

    fn watch(stats: &Arc<CamStats>) -> Cameras {
        Cameras::new(vec![(
            "wrist".into(),
            "/dev/video2".into(),
            Arc::clone(stats),
        )])
    }

    #[test]
    fn a_camera_that_has_not_opened_reports_what_its_config_asked_for() {
        let stats = Arc::new(CamStats::new(&camera()));
        let mut cameras = watch(&stats);
        assert_eq!(cameras.names().collect::<Vec<_>>(), ["wrist"]);
        let state = cameras.tick().remove(0);
        assert_eq!(state.format, Format::Mjpeg);
        assert_eq!((state.width, state.height, state.fps_set), (640, 480, 30));
        assert!(!state.streaming && !state.ts_monotonic && !state.ts_soe);
        assert_eq!((state.frames, state.errors, state.reopens), (0, 0, 0));
        assert_eq!(state.record_dropped, 0);
        assert_eq!(state.fps, 0.0);
        assert_eq!(state.recording, None);
    }

    #[test]
    fn the_counters_and_the_granted_stream_reach_the_state() {
        let stats = Arc::new(CamStats::new(&camera()));
        let mut cameras = watch(&stats);
        stats.streaming(&SourceInfo {
            device: "/dev/video2".into(),
            format: Format::Yuyv,
            width: 1280,
            height: 720,
            fps: 30,
            ts_monotonic: true,
            ts_soe: true,
        });
        for _ in 0..10 {
            stats.frame(1_000, 0, false);
        }
        stats.frame(1_000, 2, true);
        stats.publish_dropped();
        stats.error();
        stats.reopened();
        stats.record_dropped();
        stats.set_recording(Some("L-20260911T120000Z-cam.rrd".into()));

        let state = cameras.tick().remove(0);
        assert_eq!(state.name, "wrist");
        assert_eq!(state.device, "/dev/video2");
        assert_eq!(state.format, Format::Yuyv);
        assert_eq!((state.width, state.height), (1280, 720));
        assert_eq!((state.fps_set, state.frames), (30, 11));
        assert_eq!(state.driver_dropped, 2);
        assert_eq!(state.publish_dropped, 1);
        // The flagged frame and the failed dequeue.
        assert_eq!(state.errors, 2);
        assert_eq!(state.reopens, 1);
        assert_eq!(state.record_dropped, 1);
        assert!(state.streaming && state.ts_monotonic && state.ts_soe);
        assert_eq!(
            state.recording.as_deref(),
            Some("L-20260911T120000Z-cam.rrd")
        );
        // 11 frames of 1 kB in a window of microseconds: a large rate, not a NaN.
        assert!(state.fps > 0.0 && state.fps.is_finite(), "{}", state.fps);
        assert!(state.bytes_per_s > state.fps, "{}", state.bytes_per_s);

        // The next tick measures the frames since this one.
        let state = cameras.tick().remove(0);
        assert_eq!(state.frames, 11);
        assert_eq!(state.fps, 0.0);
        assert_eq!(state.bytes_per_s, 0.0);
        stats.stopped();
        assert!(!cameras.tick().remove(0).streaming);
    }

    #[test]
    fn the_status_json_is_the_node_and_its_cameras() {
        let stats = Arc::new(CamStats::new(&camera()));
        let mut cameras = watch(&stats);
        let status = Status::new("pi", 42, cameras.tick());
        let json: serde_json::Value = serde_json::from_str(&status.to_json()).expect("json");
        assert_eq!(json["node"], "pi");
        assert_eq!(json["version"], env!("CARGO_PKG_VERSION"));
        assert_eq!(json["uptime_s"], 42);
        assert_eq!(json["cameras"].as_array().expect("cameras").len(), 1);
        let camera = &json["cameras"][0];
        assert_eq!(camera["name"], "wrist");
        assert_eq!(camera["device"], "/dev/video2");
        assert_eq!(camera["format"], "mjpeg");
        assert_eq!(camera["width"], 640);
        assert_eq!(camera["fps_set"], 30);
        assert_eq!(camera["fps"], 0.0);
        assert_eq!(camera["frames"], 0);
        assert_eq!(camera["driver_dropped"], 0);
        assert_eq!(camera["publish_dropped"], 0);
        assert_eq!(camera["record_dropped"], 0);
        assert_eq!(camera["ts_monotonic"], false);
        assert_eq!(camera["recording"], serde_json::Value::Null);
        // The per-camera state topic carries one camera's entry and nothing else.
        let state: serde_json::Value =
            serde_json::from_str(&cameras.tick().remove(0).to_json()).expect("json");
        assert_eq!(&state, camera);
    }
}
