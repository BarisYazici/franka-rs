//! A live flight recorder: [`Recorder::push`] is called from inside a control callback, on
//! the realtime thread, with every state and the command sent for it; a background thread
//! drains what was pushed every [`RecorderOptions::interval`] and writes it to a Rerun stream
//! with the same [`FlightLogger`] the offline replay uses.
//!
//! # What keeps `push` off the critical path
//!
//! The two threads meet in a `std::sync::mpsc::sync_channel` of [`RecorderOptions::capacity`]
//! records. Bounded (capacity > 0) channels in `std` are backed by a ring of slots allocated
//! once, when the channel is created; `try_send` claims a slot with an atomic
//! compare-and-swap, moves the record into it and returns -- it never blocks and never
//! allocates. When the ring is full the record is dropped and counted instead. The one place
//! `std` can take a lock on the sending side is to wake a receiver that is *blocked* in
//! `recv`; the background thread here never blocks in `recv` (it sleeps, then `try_recv`s
//! until empty), so that path is never taken. `crates/franka-rerun/tests/flight.rs` checks
//! the no-allocation claim with a counting allocator.
//!
//! The Rerun SDK itself is only ever touched from the background thread.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, SyncSender, TrySendError};
use std::thread::JoinHandle;
use std::time::Duration;

use franka::{Model, Record, RobotCommandLog, RobotState};
use rerun::{RecordingStream, RecordingStreamBuilder};

use crate::flight::{send_blueprint, FlightLogger, FlightOptions, Summary};
use crate::{Result, RobotKind};

/// The recorder's tuning.
#[derive(Debug, Clone, PartialEq)]
pub struct RecorderOptions {
    /// Records the channel holds before [`Recorder::push`] starts dropping; 4096 is four
    /// seconds at 1 kHz.
    pub capacity: usize,
    /// How often the background thread drains the channel.
    pub interval: Duration,
    /// How the records are drawn. The default decimates the 3D scene to every 10th record
    /// so the viewer keeps up; the series carry every record.
    pub flight: FlightOptions,
}

impl Default for RecorderOptions {
    fn default() -> Self {
        RecorderOptions {
            capacity: 4096,
            interval: Duration::from_millis(100),
            flight: FlightOptions {
                every: 10,
                ..FlightOptions::default()
            },
        }
    }
}

/// What a recorder did, from [`Recorder::finish`].
#[derive(Debug, Clone, PartialEq)]
pub struct Stats {
    /// Calls to [`Recorder::push`].
    pub pushed: usize,
    /// Of those, records dropped because the channel was full.
    pub dropped: usize,
    /// What reached the stream.
    pub summary: Summary,
}

/// See the module documentation.
pub struct Recorder {
    /// The recording, for [`Recorder::stream`].
    rec: RecordingStream,
    sender: SyncSender<Record>,
    pushed: AtomicUsize,
    dropped: AtomicUsize,
    thread: Option<JoinHandle<std::result::Result<Summary, String>>>,
}

/// `rerun+http://{addr}/proxy` unless `addr` already is a URL.
fn proxy_url(addr: &str) -> String {
    if addr.contains("://") {
        addr.to_string()
    } else {
        format!("rerun+http://{addr}/proxy")
    }
}

impl Recorder {
    /// Records into the `.rrd` at `path`.
    pub fn to_file(
        path: &std::path::Path,
        model: Model,
        kind: RobotKind,
        options: RecorderOptions,
    ) -> Result<Recorder> {
        let rec = RecordingStreamBuilder::new("franka_rs").save(path)?;
        Recorder::with_stream(rec, model, kind, options)
    }

    /// Streams to a viewer listening at `addr` (`host:port`, or a full `rerun+http://.../proxy`
    /// URL). Start it first: `rerun` (the viewer listens on 9876 by default) or
    /// `rerun --serve-web`; nothing here starts one, and a viewer that is not there shows up
    /// as the SDK's connection warnings, not as an error.
    pub fn to_viewer(
        addr: &str,
        model: Model,
        kind: RobotKind,
        options: RecorderOptions,
    ) -> Result<Recorder> {
        let rec = RecordingStreamBuilder::new("franka_rs").connect_grpc_opts(proxy_url(addr))?;
        Recorder::with_stream(rec, model, kind, options)
    }

    /// Both at once: streams to the viewer at `addr` and records into the `.rrd` at `path`
    /// (one stream with two sinks).
    pub fn to_viewer_and_file(
        addr: &str,
        path: &std::path::Path,
        model: Model,
        kind: RobotKind,
        options: RecorderOptions,
    ) -> Result<Recorder> {
        use rerun::external::re_uri::RedapUri;
        use rerun::sink::{FileSink, GrpcSink};
        let url = proxy_url(addr);
        let RedapUri::Proxy(uri) = url.parse::<RedapUri>()? else {
            return Err(format!("{url}: not a viewer proxy URL").into());
        };
        let rec = RecordingStreamBuilder::new("franka_rs")
            .set_sinks((GrpcSink::new(uri), FileSink::new(path)?))?;
        Recorder::with_stream(rec, model, kind, options)
    }

    /// Spawns a `rerun` viewer from `PATH` and streams to it.
    pub fn spawn(model: Model, kind: RobotKind, options: RecorderOptions) -> Result<Recorder> {
        let rec = RecordingStreamBuilder::new("franka_rs").spawn()?;
        Recorder::with_stream(rec, model, kind, options)
    }

    /// Records into any stream, e.g. one with several sinks. Sends the flight recorder's
    /// blueprint first; the static setup (series styles, the base and end effector axes, the
    /// meshes) goes out with the background thread's first batch, so a live viewer shows the
    /// arm as soon as the first records arrive, whatever the sink.
    pub fn with_stream(
        rec: RecordingStream,
        model: Model,
        kind: RobotKind,
        options: RecorderOptions,
    ) -> Result<Recorder> {
        if options.capacity == 0 {
            return Err("RecorderOptions::capacity must be at least 1".into());
        }
        send_blueprint(&rec)?;
        let (sender, receiver) = mpsc::sync_channel::<Record>(options.capacity);
        let thread_rec = rec.clone();
        let thread = std::thread::Builder::new()
            .name("franka-rerun-recorder".into())
            .spawn(move || -> std::result::Result<Summary, String> {
                drop_realtime_priority();
                let rec = thread_rec;
                let mut logger = FlightLogger::new(&rec, &model, kind, options.flight)
                    .map_err(|e| e.to_string())?;
                let mut batch: Vec<Record> = Vec::with_capacity(options.capacity);
                loop {
                    std::thread::sleep(options.interval);
                    let mut disconnected = false;
                    loop {
                        match receiver.try_recv() {
                            Ok(record) => batch.push(record),
                            Err(mpsc::TryRecvError::Empty) => break,
                            Err(mpsc::TryRecvError::Disconnected) => {
                                disconnected = true;
                                break;
                            }
                        }
                    }
                    logger.log_batch(&batch).map_err(|e| e.to_string())?;
                    batch.clear();
                    if disconnected {
                        break;
                    }
                }
                logger.finish(None).map_err(|e| e.to_string())
            })?;
        Ok(Recorder {
            rec,
            sender,
            pushed: AtomicUsize::new(0),
            dropped: AtomicUsize::new(0),
            thread: Some(thread),
        })
    }

    /// Hands one cycle to the background thread: the state received and the command sent
    /// for it (`None` when the cycle sent nothing). Never blocks and never allocates; a full
    /// channel drops the record and counts it.
    pub fn push(&self, state: &RobotState, command: Option<RobotCommandLog>) {
        self.pushed.fetch_add(1, Ordering::Relaxed);
        let record = Record {
            state: *state,
            command,
        };
        if let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) =
            self.sender.try_send(record)
        {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// A handle on the recording, for logging extra entities into it from a thread that is
    /// **not** the realtime one (the SDK allocates and may block): `stream.set_duration_secs(
    /// TIMELINE, t)` then `stream.log(...)`, on the same `robot_time` timeline as everything
    /// else. Handles are cheap `Arc` clones; what is logged through one before
    /// [`Recorder::finish`] is flushed with the rest.
    pub fn stream(&self) -> RecordingStream {
        self.rec.clone()
    }

    /// Records dropped so far.
    pub fn dropped(&self) -> usize {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Records pushed so far.
    pub fn pushed(&self) -> usize {
        self.pushed.load(Ordering::Relaxed)
    }

    /// Closes the channel, waits for the background thread to write what is left and flush
    /// the stream, and returns the counts. Takes up to one [`RecorderOptions::interval`].
    /// Dropping a recorder without calling this closes the channel too, and the thread still
    /// writes what it has, but nothing waits for it.
    pub fn finish(mut self) -> Result<Stats> {
        let thread = self.thread.take().ok_or("recorder already finished")?;
        let (pushed, dropped) = (self.pushed(), self.dropped());
        drop(self);
        let summary = thread
            .join()
            .map_err(|_| "the recorder thread panicked")??;
        Ok(Stats {
            pushed,
            dropped,
            summary,
        })
    }
}

/// Puts the calling thread back on the normal scheduler.
///
/// A control program is often started with a realtime policy for the whole process (for
/// instance `chrt -f 80 ...`), which every thread it spawns inherits. The recorder thread and
/// a commander thread then compete with the 1 kHz control loop at the same priority, and a
/// burst of Rerun serialisation can cost the loop a cycle. Best effort: a failure (no
/// permission, not Linux) is ignored.
pub fn drop_realtime_priority() {
    #[cfg(target_os = "linux")]
    // SAFETY: `sched_setscheduler` on the calling thread with a zeroed `sched_param`; the
    // struct is plain data and the pointer is valid for the call.
    unsafe {
        let param: libc::sched_param = std::mem::zeroed();
        libc::sched_setscheduler(0, libc::SCHED_OTHER, &param);
    }
}

/// Puts the calling thread on `SCHED_FIFO` at `priority`, best effort, returning whether it
/// worked. Call it on the control thread right before the control loop, after the recorder
/// and the Rerun SDK have spawned their threads, so that only the loop runs realtime.
pub fn raise_realtime_priority(priority: i32) -> bool {
    #[cfg(target_os = "linux")]
    // SAFETY: as in `drop_realtime_priority`; `sched_param` is plain data.
    let raised = unsafe {
        let mut param: libc::sched_param = std::mem::zeroed();
        param.sched_priority = priority;
        libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) == 0
    };
    #[cfg(not(target_os = "linux"))]
    let raised = {
        let _ = priority;
        false
    };
    raised
}
