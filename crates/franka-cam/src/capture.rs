//! One thread per camera: open the device, then dequeue, stamp, publish, count, forever.
//!
//! Normal priority and never `SCHED_FIFO` — the thread copies frames and talks to Zenoh, so
//! it must never be able to hold a realtime loop off a core. Its only affinity is the config's
//! `cpu`, which is how a camera is kept off the cores the arm node's loops run on.
//!
//! One [`Vec`] per frame, the one the header and the frame bytes are written into, and it is
//! handed to the publisher rather than copied again. The dequeued buffer goes back to the
//! driver before the publisher is called, so the queue never waits on Zenoh.
//!
//! With a recorder the frame bytes are copied once more out of that same [`Vec`], which is what
//! [`crate::record`] costs and only while an episode is open; the recorder never blocks the
//! thread either.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use log::{debug, error, info, warn};

use crate::config::CameraConfig;
use crate::msg::{self, CameraMsg, Capture, Format, FRAME_ERROR};
use crate::record::RecordSink;
use crate::source::FrameSource;
use crate::status::CamStats;
use crate::sys;

/// The shortest wait between reopen attempts, doubled up to [`BACKOFF_MAX`].
pub const BACKOFF_MIN: Duration = Duration::from_millis(200);
/// The longest wait between reopen attempts.
pub const BACKOFF_MAX: Duration = Duration::from_secs(5);
/// How long a dequeue may wait before the camera counts as dead, at least. A slower camera
/// gets three of its own frame periods.
pub const DEQUEUE_TIMEOUT: Duration = Duration::from_secs(2);

/// Opens the camera of a config; the capture thread calls it again for every reopen.
///
/// An error of [`io::ErrorKind::InvalidInput`] or [`io::ErrorKind::Unsupported`] is a
/// configuration the device will never satisfy and stops the thread; anything else is the
/// device's and is retried.
pub type SourceFactory =
    Arc<dyn Fn(&CameraConfig) -> io::Result<Box<dyn FrameSource + Send>> + Send + Sync>;

/// The bytes of one frame, moved rather than copied; `false` means the frame did not go out.
pub type Publish = Box<dyn Fn(Vec<u8>) -> bool + Send>;

/// [`V4l2Source`](crate::source::V4l2Source): the factory a node uses.
pub fn v4l2_factory() -> SourceFactory {
    Arc::new(|config| {
        let source = crate::source::V4l2Source::open(config)?;
        Ok(Box::new(source) as Box<dyn FrameSource + Send>)
    })
}

/// A running camera thread; dropping it stops the thread and joins it.
pub struct Camera {
    name: String,
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
    stats: Arc<CamStats>,
}

impl Camera {
    /// The camera's counters, for the status thread.
    pub fn stats(&self) -> &Arc<CamStats> {
        &self.stats
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Stops the thread and joins it.
    ///
    /// The thread checks the flag between frames, so this is immediate on a camera that is
    /// delivering. One that has stopped is sitting in a wait: up to `3 / fps` or
    /// [`DEQUEUE_TIMEOUT`], whichever is longer, in the stream, and up to 5 s while a reopened
    /// camera waits for its first frame. Cameras are shut down one after another, so a node's
    /// worst case is the sum.
    pub fn shutdown(mut self) {
        self.finish();
    }

    fn finish(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            if join.join().is_err() {
                error!("camera {}: the capture thread panicked", self.name);
            }
        }
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        self.finish();
    }
}

/// Spawns the capture thread of `config`: frames from `factory` to `publish` and to `record`
/// when there is one, counted into `stats`.
pub fn spawn(
    config: CameraConfig,
    factory: SourceFactory,
    publish: Publish,
    record: Option<RecordSink>,
    stats: Arc<CamStats>,
) -> io::Result<Camera> {
    let (name, stop) = (config.name.clone(), Arc::new(AtomicBool::new(false)));
    let join = {
        let (stop, stats) = (Arc::clone(&stop), Arc::clone(&stats));
        std::thread::Builder::new()
            .name(format!("franka-cam-{name}"))
            .spawn(move || run(&config, &factory, &publish, record.as_ref(), &stats, &stop))?
    };
    Ok(Camera {
        name,
        stop,
        join: Some(join),
        stats,
    })
}

/// Opens the camera, streams from it, and on an error the device might recover from opens it
/// again after a growing wait, until the flag is set.
fn run(
    config: &CameraConfig,
    factory: &SourceFactory,
    publish: &Publish,
    record: Option<&RecordSink>,
    stats: &Arc<CamStats>,
    stop: &AtomicBool,
) {
    if let Err(e) = sys::pin_to(&config.cpu) {
        warn!("camera {}: cpu {:?}: {e}", config.name, config.cpu);
    }
    let mut backoff = BACKOFF_MIN;
    let mut opened = false;
    while !stop.load(Ordering::Relaxed) {
        match factory(config) {
            Ok(mut source) => {
                if opened {
                    stats.reopened();
                }
                opened = true;
                backoff = BACKOFF_MIN;
                stats.streaming(source.info());
                if let Err(e) = stream(config, source.as_mut(), publish, record, stats, stop) {
                    stats.stopped();
                    if fatal(&e) {
                        error!("camera {}: {e}", config.name);
                        return;
                    }
                    warn!("camera {}: {e}; reopening", config.name);
                } else {
                    stats.stopped();
                }
            }
            Err(e) if fatal(&e) => {
                error!("camera {}: {e}", config.name);
                return;
            }
            Err(e) => {
                warn!(
                    "camera {}: {} {e}; retrying in {backoff:?}",
                    config.name,
                    config.device.display()
                );
                stats.error();
            }
        }
        if !stop.load(Ordering::Relaxed) {
            sleep_until_stopped(backoff, stop);
            backoff = (backoff * 2).min(BACKOFF_MAX);
        }
    }
    info!("camera {}: stopped", config.name);
}

/// An error the device will never recover from: the config asks for something it cannot do.
/// Everything the kernel reports — an unplugged camera's `ENODEV`, a cable's `EIO` — is worth
/// another open.
fn fatal(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::InvalidInput | io::ErrorKind::Unsupported
    )
}

fn sleep_until_stopped(how_long: Duration, stop: &AtomicBool) {
    let deadline = std::time::Instant::now() + how_long;
    while !stop.load(Ordering::Relaxed) {
        let left = deadline.saturating_duration_since(std::time::Instant::now());
        if left.is_zero() {
            return;
        }
        std::thread::sleep(left.min(Duration::from_millis(100)));
    }
}

/// Dequeues, stamps, publishes and counts until the stop flag is set (`Ok`) or the device
/// stops delivering (`Err`).
fn stream(
    config: &CameraConfig,
    source: &mut dyn FrameSource,
    publish: &Publish,
    record: Option<&RecordSink>,
    stats: &Arc<CamStats>,
    stop: &AtomicBool,
) -> io::Result<()> {
    let info = source.info().clone();
    let timeout = (info.period() * 3).max(DEQUEUE_TIMEOUT);
    let mut last_seq: Option<u32> = None;
    while !stop.load(Ordering::Relaxed) {
        let Some(frame) = source.next(timeout).inspect_err(|_| stats.error())? else {
            stats.error();
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                format!("no frame within {timeout:?}"),
            ));
        };
        let capture = Capture {
            seq: frame.seq,
            flags: frame.flags,
            t_capture_ns: frame.t_capture_ns,
            t_node_ns: sys::monotonic_ns(),
            t_wall_ns: sys::wall_ns(),
        };
        let payload = match info.format {
            // Some cameras pad an MJPEG buffer past the end-of-image; a decoder need not care,
            // but the bytes are on the wire and in the recording.
            Format::Mjpeg => msg::trim_jpeg(frame.bytes()),
            _ => frame.bytes(),
        };
        let gap = last_seq.map_or(0, |last| frame.seq.wrapping_sub(last).saturating_sub(1));
        last_seq = Some(frame.seq);
        // The one allocation of the frame path, handed to the publisher rather than copied.
        let header = CameraMsg::new(info.format, info.width, info.height, capture);
        let bytes = header.encode(payload);
        let published = bytes.len();
        drop(frame);
        // The recorder's copy comes out of that sample, with the driver's buffer already back
        // in its queue; it is taken before the publisher moves the sample away.
        if let Some(record) = record {
            record(&header, &bytes[msg::HEADER_SIZE..]);
        }
        if !publish(bytes) {
            stats.publish_dropped();
        }
        stats.frame(published, gap, capture.flags & FRAME_ERROR != 0);
        if gap != 0 {
            debug!("camera {}: the driver dropped {gap} frame(s)", config.name);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
