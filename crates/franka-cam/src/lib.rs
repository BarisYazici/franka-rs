//! A Zenoh camera node that belongs next to `franka-node`: V4L2 frames with their capture
//! timestamps on `franka/cam/<name>/frame`, so a consumer can tell which arm state a frame
//! belongs with and one recording can hold both.
//!
//! Exactly, on the host the arm node runs on, because `t_node_ns` is that host's
//! `CLOCK_MONOTONIC` on both wires. From another host only `t_wall_ns` is comparable, which is
//! as good as the two clocks are synchronised.
//!
//! [`run`] is the node: a [`config`] file, one capture thread per camera through [`v4l2`], the
//! publishers of [`transport`], the 1 Hz [`status`] and, with the `record` feature, the
//! [`record`] side that puts the frames of an arm's episode into a `.rrd` beside the arm's own.
//! A separate process from the arm node on purpose — USB is where the flakiness lives, and a
//! camera that stalls must not take the robot's process with it.
//!
//! The frame header is small and alignment-one, so a Python consumer unpacks it with
//! `struct.Struct("<BBHHHIQQQ")` and slices the frame off at [`msg::HEADER_SIZE`].

pub mod capture;
pub mod config;
pub mod msg;
pub mod record;
pub mod source;
pub mod status;
pub mod sys;
pub mod transport;
pub mod v4l2;

pub use capture::{v4l2_factory, Camera, Publish, SourceFactory};
pub use config::{CamConfig, CameraConfig, ConfigError, ZenohConfig};
pub use msg::{
    decode, trim_jpeg, CameraMsg, Capture, DecodeError, Format, FRAME_ERROR, HEADER_SIZE,
    TIMESTAMP_MONOTONIC, TIMESTAMP_SOE, VERSION,
};
pub use record::{RecordSink, Recording};
#[cfg(feature = "fake")]
pub use source::FakeSource;
pub use source::{Frame, FrameSource, SourceInfo, V4l2Source};
pub use status::{CamState, CamStats, Cameras, Status};
pub use sys::{monotonic_ns, wall_ns};
pub use transport::StatusPublisher;

use std::sync::mpsc;
use std::sync::Arc;

use log::info;
use zenoh::Wait;

/// The node: every `[[camera]]` of `config` on its own thread, the frames and the status over
/// Zenoh, and on SIGINT or SIGTERM each camera stopped and the session closed.
pub fn run(config: CamConfig) -> Result<(), String> {
    run_with(config, v4l2_factory())
}

/// [`run`] with the cameras' [`SourceFactory`], which is what lets a test drive the node
/// without a camera.
pub fn run_with(config: CamConfig, factory: SourceFactory) -> Result<(), String> {
    let text = |e: zenoh::Error| e.to_string();
    if config.cameras.is_empty() {
        return Err("no [[camera]] in the config: nothing to publish".into());
    }
    let session = transport::open(&config.zenoh).map_err(text)?;
    info!(
        "node {}: listening on {:?}, connecting to {:?}, multicast scouting {}",
        config.name,
        config.zenoh.listen,
        config.zenoh.connect,
        if config.zenoh.multicast_enabled() {
            "on"
        } else {
            "off"
        }
    );
    // The counters are made here, before the cameras: the recorder counts into the same ones,
    // so a frame it could not take shows up in that camera's state.
    let stats: Vec<Arc<CamStats>> = config
        .cameras
        .iter()
        .map(|camera| Arc::new(CamStats::new(camera)))
        .collect();
    // Never fatal: a camera that cannot record still publishes.
    let mut recording = Recording::start(&session, &config, &stats);
    let mut cameras = Vec::with_capacity(config.cameras.len());
    let mut watched = Vec::with_capacity(config.cameras.len());
    for (camera, stats) in config.cameras.iter().zip(&stats) {
        let device = camera.device.display().to_string();
        let record = recording.sink(&camera.name);
        let attached = transport::attach(
            &session,
            camera.clone(),
            Arc::clone(&factory),
            Arc::clone(stats),
            record,
        )
        .map_err(text)?;
        watched.push((camera.name.clone(), device, Arc::clone(stats)));
        cameras.push(attached);
    }
    let status =
        transport::status_publisher(&session, &config.name, Cameras::new(watched)).map_err(text)?;
    let (signalled, signal) = mpsc::channel();
    ctrlc::set_handler(move || {
        let _ = signalled.send(());
    })
    .map_err(|e| e.to_string())?;
    let _ = signal.recv();
    info!("signal received, stopping {} camera(s)", cameras.len());
    status.shutdown();
    // The cameras first: their sinks hold the recorder's senders, and an open episode is closed
    // once no thread can hand it another frame.
    for camera in cameras {
        camera.shutdown();
    }
    recording.shutdown();
    session.close().wait().map_err(text)
}
