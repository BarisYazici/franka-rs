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
#[derive(Debug, Clone, Copy, PartialEq)]
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

    /// Spawns a `rerun` viewer from `PATH` and streams to it.
    pub fn spawn(model: Model, kind: RobotKind, options: RecorderOptions) -> Result<Recorder> {
        let rec = RecordingStreamBuilder::new("franka_rs").spawn()?;
        Recorder::with_stream(rec, model, kind, options)
    }

    /// Records into any stream, e.g. one with several sinks. Sends the flight recorder's
    /// blueprint first.
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
        let thread = std::thread::Builder::new()
            .name("franka-rerun-recorder".into())
            .spawn(move || -> std::result::Result<Summary, String> {
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
