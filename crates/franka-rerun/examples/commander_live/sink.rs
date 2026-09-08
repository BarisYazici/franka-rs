//! The Rerun side of the commander thread of `commander_live.rs`: the robot clock the
//! callback keeps for it, and a [`Sink`] that logs the raw target as a staircase, its
//! implied speed as a spike, and the commander's events, all stamped with the robot time.
//! Nothing here runs on the realtime thread.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use franka_rerun::flight::cartesian::{TARGET_PREFIX, TARGET_SPEED};
use franka_rerun::{distance, scene, TARGET, TIMELINE};
use rerun::{RecordingStream, Scalars, TextLog, TextLogLevel};

use super::script::Sink;
use super::CYCLE;

/// What the callback tells the commander thread: the latest `state.time` in ms and the start
/// position, one atomic store each (the start once, on the first cycle).
#[derive(Default)]
pub struct Clock {
    pub time_ms: AtomicU64,
    start: [AtomicU64; 3],
}

impl Clock {
    pub fn now(&self) -> f64 {
        self.time_ms.load(Ordering::Relaxed) as f64 * 1e-3
    }
    pub fn set_start(&self, start: &[f64; 3]) {
        for (slot, value) in self.start.iter().zip(start) {
            slot.store(value.to_bits(), Ordering::Relaxed);
        }
    }
    pub fn start(&self) -> [f64; 3] {
        self.start
            .each_ref()
            .map(|slot| f64::from_bits(slot.load(Ordering::Relaxed)))
    }
}

/// The raw target at `t`: the staircase per axis and the 3D point when `position` is given,
/// and the two implied speeds. Non-realtime.
pub fn log_target(rec: &RecordingStream, t: f64, position: Option<&[f64; 3]>, speeds: [f64; 2]) {
    rec.set_duration_secs(TIMELINE, t);
    let log = || -> franka_rerun::Result<()> {
        if let Some(p) = position {
            for (axis, value) in ["x", "y", "z"].into_iter().zip(p) {
                let entity = format!("{TARGET_PREFIX}/{axis}");
                rec.log(entity, &Scalars::single(*value))?;
            }
            scene::log_point(rec, "target", p, 0.015, TARGET)?;
        }
        rec.log(TARGET_SPEED, &Scalars::new(speeds))?;
        Ok(())
    };
    if let Err(e) = log() {
        eprintln!("commander: logging the target failed: {e}");
    }
}

/// The commander thread's side of the recording: the raw target as a staircase (the previous
/// value one cycle before each step), its implied speed as a spike, and the events.
pub struct LiveCommander {
    pub rec: RecordingStream,
    pub clock: Arc<Clock>,
    /// The last target published, absolute, and when.
    pub last: [f64; 3],
    pub last_time: f64,
}

impl Sink for LiveCommander {
    fn published(&mut self, target: [f64; 3]) {
        let t = self.clock.now();
        let start = self.clock.start();
        let absolute: [f64; 3] = std::array::from_fn(|k| start[k] + target[k]);
        let step = distance(&absolute, &self.last);
        let since = (t - self.last_time).max(CYCLE);
        log_target(&self.rec, t - CYCLE, Some(&self.last), [0.0; 2]);
        log_target(&self.rec, t, Some(&absolute), [step / CYCLE, step / since]);
        log_target(&self.rec, t + CYCLE, None, [0.0; 2]);
        self.last = absolute;
        self.last_time = t;
    }

    fn event(&mut self, warning: bool, text: String) {
        eprintln!("commander: {text}");
        let level = if warning {
            TextLogLevel::WARN
        } else {
            TextLogLevel::INFO
        };
        self.rec.set_duration_secs(TIMELINE, self.clock.now());
        let text = format!("commander: {text}");
        if let Err(e) = self
            .rec
            .log("events", &TextLog::new(text).with_level(level))
        {
            eprintln!("commander: logging the event failed: {e}");
        }
    }
}
