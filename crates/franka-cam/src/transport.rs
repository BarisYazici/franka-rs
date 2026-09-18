//! The Zenoh side: the session from the config, one frame publisher per camera, and the thread
//! that publishes each camera's state and the node's status once a second.
//!
//! Frames are published with `CongestionControl::Drop` and `express(true)`: a frame is tens to
//! hundreds of kilobytes and worth nothing once the next one exists, so a full queue must drop
//! it rather than hold the capture thread, and there is nothing to batch it with.
//!
//! A camera with a `preview_fps` also puts every nth frame on `franka/cam/<name>/preview`, for a
//! consumer that must not pull the full rate over a shared or metered link. It is the same
//! bytes: the payload is turned into a `ZBytes` once and both publishers take a handle to it.

use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use std::sync::atomic::{AtomicU32, Ordering};

use log::{debug, info};
use zenoh::bytes::ZBytes;
use zenoh::key_expr::KeyExpr;
use zenoh::pubsub::Publisher;
use zenoh::qos::CongestionControl;
use zenoh::{Config, Session, Wait};

use crate::capture::{self, Camera, Publish, SourceFactory};
use crate::config::{CameraConfig, ZenohConfig};
use crate::record::RecordSink;
use crate::status::{CamStats, Cameras, Status};

/// How often the state and status topics are published.
pub const STATUS_PERIOD: Duration = Duration::from_secs(1);

/// Opens the session `config` describes: a peer that listens, or a client that dials the routers
/// in `connect` and listens on nothing.
pub fn open(config: &ZenohConfig) -> zenoh::Result<Session> {
    // The file, if there is one, is the base: it carries what this table does not name, such as
    // the TLS material a `tls/` endpoint needs. The keys below are applied on top of it.
    let mut zenoh = match &config.zenoh_config {
        Some(path) => Config::from_file(path)?,
        None => Config::default(),
    };
    zenoh.insert_json5("mode", &serde_json::to_string(config.mode.as_str())?)?;
    zenoh.insert_json5(
        "listen/endpoints",
        &serde_json::to_string(config.listen_endpoints())?,
    )?;
    zenoh.insert_json5(
        "connect/endpoints",
        &serde_json::to_string(&config.connect)?,
    )?;
    zenoh.insert_json5(
        "scouting/multicast/enabled",
        &config.multicast_enabled().to_string(),
    )?;
    if let Some(interface) = &config.scouting_interface {
        zenoh.insert_json5(
            "scouting/multicast/interface",
            &serde_json::to_string(interface)?,
        )?;
    }
    zenoh.insert_json5("transport/link/tx/lease", &config.lease_ms.to_string())?;
    // With `CongestionControl::Drop` the pipeline still waits for room before it gives up, and
    // its defaults are 1 ms for a batch and 50 ms for the fragments of a large message. A frame
    // is many fragments, so on a congested uplink those 50 ms would be spent by the capture
    // thread. Well inside a frame period instead: a dropped frame is the point.
    zenoh.insert_json5(
        "transport/link/tx/queue/congestion_control/drop/wait_before_drop",
        "500",
    )?;
    zenoh.insert_json5(
        "transport/link/tx/queue/congestion_control/drop/max_wait_before_drop_fragments",
        "2000",
    )?;
    zenoh::open(zenoh).wait()
}

/// Spawns the camera of `config` with a source from `factory` and publishes its frames on
/// `franka/cam/<name>/frame`, counted into `stats`. `record` is the camera's recorder sink, from
/// [`crate::record::Recording::sink`], or `None` when it records nothing.
pub fn attach(
    session: &Session,
    config: CameraConfig,
    factory: SourceFactory,
    stats: Arc<CamStats>,
    record: Option<RecordSink>,
) -> zenoh::Result<Camera> {
    let name = config.name.clone();
    let declare = |suffix: &str| -> zenoh::Result<Publisher<'static>> {
        session
            .declare_publisher(KeyExpr::try_from(format!("franka/cam/{name}/{suffix}"))?)
            .congestion_control(CongestionControl::Drop)
            .express(true)
            .wait()
    };
    let frames = declare("frame")?;
    let preview = match config.preview_every() {
        Some(every) => {
            info!(
                "camera {name}: 1 frame in {every} on franka/cam/{name}/preview, {} fps of {}",
                config.fps / every,
                config.fps
            );
            Some((declare("preview")?, every))
        }
        None => None,
    };
    info!(
        "camera {name}: {} on franka/cam/{name}/frame",
        config.device.display()
    );
    let publish = publish(frames, preview);
    let camera = capture::spawn(config, factory, publish, record, stats)?;
    Ok(camera)
}

/// The frame sink: the [`Vec`] is moved into a [`ZBytes`] rather than copied, and with a preview
/// every nth frame goes out on that key too from the same handle, so the second key costs a
/// reference and not a second frame. The transport copies the payload into its own batch, once
/// per key.
fn publish(frames: Publisher<'static>, preview: Option<(Publisher<'static>, u32)>) -> Publish {
    let seen = AtomicU32::new(0);
    Box::new(move |bytes: Vec<u8>| {
        let payload = ZBytes::from(bytes);
        if let Some((preview, every)) = &preview {
            if seen.fetch_add(1, Ordering::Relaxed).is_multiple_of(*every) {
                if let Err(e) = preview.put(payload.clone()).wait() {
                    debug!("preview publish: {e}");
                }
            }
        }
        match frames.put(payload).wait() {
            Ok(()) => true,
            Err(e) => {
                debug!("frame publish: {e}");
                false
            }
        }
    })
}

/// The thread publishing the state and status topics; dropping it stops the thread.
pub struct StatusPublisher {
    stop: Option<Sender<()>>,
    join: Option<JoinHandle<()>>,
}

impl StatusPublisher {
    /// Stops the thread and joins it.
    pub fn shutdown(mut self) {
        self.finish();
    }

    fn finish(&mut self) {
        drop(self.stop.take());
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for StatusPublisher {
    fn drop(&mut self) {
        self.finish();
    }
}

/// Publishes every camera's state on `franka/cam/<name>/state` and the node's own on
/// `franka/node/<node>/status`, at once and then every [`STATUS_PERIOD`].
///
/// The node's health goes where the arm node publishes its own, so one subscription to
/// `franka/node/*/status` sees a whole lab; `franka/cam/<name>/` stays the cameras' own.
pub fn status_publisher(
    session: &Session,
    node: &str,
    mut cameras: Cameras,
) -> zenoh::Result<StatusPublisher> {
    let key = KeyExpr::try_from(format!("franka/node/{node}/status"))?;
    let publisher = session
        .declare_publisher(key.clone())
        .congestion_control(CongestionControl::Drop)
        .wait()?;
    let mut states = Vec::new();
    for name in cameras.names() {
        states.push(
            session
                .declare_publisher(KeyExpr::try_from(format!("franka/cam/{name}/state"))?)
                .congestion_control(CongestionControl::Drop)
                .wait()?,
        );
    }
    info!("node {node}: status on {key}, one state per camera");
    let (stop, stopped) = mpsc::channel::<()>();
    let node = node.to_string();
    let join = std::thread::Builder::new()
        .name("franka-cam-status".into())
        .spawn(move || {
            let started = Instant::now();
            loop {
                let states_now = cameras.tick();
                for (publisher, state) in states.iter().zip(&states_now) {
                    if let Err(e) = publisher.put(state.to_json()).wait() {
                        debug!("state publish: {e}");
                    }
                }
                let status = Status::new(&node, started.elapsed().as_secs(), states_now);
                if let Err(e) = publisher.put(status.to_json()).wait() {
                    debug!("status publish: {e}");
                }
                match stopped.recv_timeout(STATUS_PERIOD) {
                    Err(RecvTimeoutError::Timeout) => {}
                    Ok(()) | Err(RecvTimeoutError::Disconnected) => break,
                }
            }
        })?;
    Ok(StatusPublisher {
        stop: Some(stop),
        join: Some(join),
    })
}
