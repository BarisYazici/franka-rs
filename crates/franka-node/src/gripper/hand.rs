//! The Franka Hand over [`franka::Gripper`] (FCI port 1338 at the arm's host).
//!
//! A worker thread runs the commands, which block on the hand for up to seconds: the latest
//! width target wins once the worker is free, a queued grasp or home goes first. A reader
//! thread keeps the state fresh meanwhile (`read_once` is the one call that must not run
//! concurrently; the TCP commands may). `stop` drops what is pending and sends the hand's
//! `stop` from a thread of its own, past a running move, as libfranka's does; the caller
//! returns at once. A `stop` racing a move the worker has just dequeued lands before that
//! move reaches the hand, which then runs to its width: the next state shows it, and a
//! second `stop` halts it. Nothing here is realtime.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use franka::{FrankaError, FrankaResult};
use log::{info, warn};

use super::{Gripper, GripperState};
use crate::monotonic_ns;

/// A width target within this of the last commanded width is not sent, m.
pub const WIDTH_DEADBAND: f64 = 0.001;
/// How long [`Gripper::home`] waits for the hand.
pub const HOME_TIMEOUT: Duration = Duration::from_secs(30);
/// `epsilon_inner` and `epsilon_outer` of every grasp, m (libfranka's defaults).
pub const GRASP_EPSILON: f64 = 0.005;
/// The reader's period: about 20 Hz.
const READ_PERIOD: Duration = Duration::from_millis(50);
/// The bits of "no width" in the atomics.
const NONE: u64 = f64::NAN.to_bits();

pub struct FrankaHand {
    shared: Arc<Shared>,
    worker: Option<JoinHandle<()>>,
    reader: Option<JoinHandle<()>>,
}

struct Shared {
    gripper: franka::Gripper,
    speed: f64,
    /// The pending width target as `f64` bits, [`NONE`] without one.
    target: AtomicU64,
    /// The last commanded width as bits; [`NONE`] after a grasp, a home or a stop.
    commanded: AtomicU64,
    moving: AtomicBool,
    /// The command channel failed (a network or protocol error).
    fault: AtomicBool,
    /// The state stream failed.
    read_fault: AtomicBool,
    quit: AtomicBool,
    /// Also the lock `command` takes around its wakeup, so none is lost.
    pending: Mutex<Pending>,
    wake: Condvar,
    /// The last state read and [`monotonic_ns`] of the read.
    state: Mutex<(franka::GripperState, u64)>,
}

#[derive(Default)]
struct Pending {
    grasp: Option<(f64, f64)>,
    home: Option<Sender<Result<(), String>>>,
}

enum Job {
    Move(f64),
    Grasp(f64, f64),
    Home(Sender<Result<(), String>>),
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

impl FrankaHand {
    /// Connects to the hand at `host`, reads a first state and starts the two threads;
    /// `speed` is what every move and grasp runs at, m/s.
    pub fn connect(host: &str, speed: f64) -> FrankaResult<Self> {
        let gripper = franka::Gripper::new(host)?;
        let state = gripper.read_once()?;
        info!(
            "gripper at {host}: server version {}, width {:.4} m, max width {:.4} m",
            gripper.server_version(),
            state.width,
            state.max_width
        );
        let shared = Arc::new(Shared {
            gripper,
            speed,
            target: AtomicU64::new(NONE),
            commanded: AtomicU64::new(NONE),
            moving: AtomicBool::new(false),
            fault: AtomicBool::new(false),
            read_fault: AtomicBool::new(false),
            quit: AtomicBool::new(false),
            pending: Mutex::default(),
            wake: Condvar::new(),
            state: Mutex::new((state, monotonic_ns())),
        });
        let spawn = |name: &str, run: fn(&Shared)| {
            let shared = Arc::clone(&shared);
            std::thread::Builder::new()
                .name(name.into())
                .spawn(move || run(&shared))
                .map_err(|e| FrankaError::Network(format!("gripper thread: {e}")))
        };
        let worker = Some(spawn("franka-hand-cmd", Shared::work)?);
        let reader = Some(spawn("franka-hand-state", Shared::read)?);
        Ok(FrankaHand {
            shared,
            worker,
            reader,
        })
    }
}

impl Drop for FrankaHand {
    fn drop(&mut self) {
        self.shared.quit.store(true, Ordering::SeqCst);
        {
            let _pending = lock(&self.shared.pending);
            self.shared.wake.notify_all();
        }
        for thread in [self.worker.take(), self.reader.take()]
            .into_iter()
            .flatten()
        {
            let _ = thread.join();
        }
    }
}

impl Shared {
    /// The command thread: one job at a time, in the order home, grasp, newest width.
    fn work(&self) {
        while let Some(job) = self.next_job() {
            self.moving.store(true, Ordering::Relaxed);
            let outcome = match job {
                Job::Move(width) => {
                    self.commanded.store(width.to_bits(), Ordering::Relaxed);
                    let moved = self.gripper.move_gripper(width, self.speed);
                    // A move that did not arrive leaves no commanded width, so the same
                    // width can be sent again.
                    if !matches!(moved, Ok(true)) {
                        self.commanded.store(NONE, Ordering::Relaxed);
                    }
                    moved
                }
                Job::Grasp(width, force) => {
                    self.commanded.store(NONE, Ordering::Relaxed);
                    self.gripper
                        .grasp(width, self.speed, force, GRASP_EPSILON, GRASP_EPSILON)
                }
                Job::Home(reply) => {
                    self.commanded.store(NONE, Ordering::Relaxed);
                    let homed = self.gripper.homing();
                    let _ = reply.send(match &homed {
                        Ok(true) => Ok(()),
                        Ok(false) => Err("homing unsuccessful".into()),
                        Err(e) => Err(e.to_string()),
                    });
                    homed
                }
            };
            self.moving.store(false, Ordering::Relaxed);
            self.note(outcome);
        }
    }

    /// Blocks until there is a job or `quit`.
    fn next_job(&self) -> Option<Job> {
        let mut pending = lock(&self.pending);
        loop {
            if self.quit.load(Ordering::SeqCst) {
                return None;
            }
            if let Some(reply) = pending.home.take() {
                return Some(Job::Home(reply));
            }
            if let Some((width, force)) = pending.grasp.take() {
                return Some(Job::Grasp(width, force));
            }
            let target = f64::from_bits(self.target.swap(NONE, Ordering::AcqRel));
            let commanded = f64::from_bits(self.commanded.load(Ordering::Relaxed));
            if !target.is_nan()
                && (commanded.is_nan() || (target - commanded).abs() > WIDTH_DEADBAND)
            {
                return Some(Job::Move(target));
            }
            pending = self
                .wake
                .wait(pending)
                .unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// A command's outcome: a lost channel is the fault, a refused command is a `warn` line.
    fn note(&self, outcome: FrankaResult<bool>) {
        match outcome {
            Ok(true) => self.fault.store(false, Ordering::Relaxed),
            Ok(false) => {
                warn!("gripper: command unsuccessful");
                self.fault.store(false, Ordering::Relaxed);
            }
            Err(e @ (FrankaError::Network(_) | FrankaError::Protocol(_))) => {
                warn!("gripper: {e}");
                self.fault.store(true, Ordering::Relaxed);
            }
            Err(e) => {
                warn!("gripper: {e}");
                self.fault.store(false, Ordering::Relaxed);
            }
        }
    }

    /// The state thread: `read_once` every [`READ_PERIOD`].
    fn read(&self) {
        while !self.quit.load(Ordering::SeqCst) {
            let began = Instant::now();
            match self.gripper.read_once() {
                Ok(state) => {
                    *lock(&self.state) = (state, monotonic_ns());
                    if self.read_fault.swap(false, Ordering::Relaxed) {
                        info!("gripper: state stream back");
                    }
                }
                Err(e) => {
                    if !self.read_fault.swap(true, Ordering::Relaxed) {
                        warn!("gripper: read_once: {e}");
                    }
                }
            }
            if let Some(rest) = READ_PERIOD.checked_sub(began.elapsed()) {
                std::thread::sleep(rest);
            }
        }
    }
}

impl Gripper for FrankaHand {
    fn command(&self, width_m: f64) {
        let _pending = lock(&self.shared.pending);
        self.shared
            .target
            .store(width_m.to_bits(), Ordering::Release);
        self.shared.wake.notify_one();
    }

    fn grasp(&self, width_m: f64, force_n: f64) {
        let mut pending = lock(&self.shared.pending);
        pending.grasp = Some((width_m, force_n));
        self.shared.target.store(NONE, Ordering::Release);
        self.shared.wake.notify_one();
    }

    /// Queues the homing behind a running move and waits for it, up to [`HOME_TIMEOUT`].
    fn home(&self) -> Result<(), String> {
        let (tx, rx) = mpsc::channel();
        {
            let mut pending = lock(&self.shared.pending);
            if pending.home.is_some() {
                return Err("homing already queued".into());
            }
            pending.home = Some(tx);
            self.shared.wake.notify_one();
        }
        match rx.recv_timeout(HOME_TIMEOUT) {
            Ok(outcome) => outcome,
            Err(RecvTimeoutError::Timeout) => Err("homing timed out".into()),
            Err(RecvTimeoutError::Disconnected) => Err("gripper driver gone".into()),
        }
    }

    /// Drops the pending width and grasp and sends `stop` from a thread of its own.
    fn stop(&self) {
        {
            let mut pending = lock(&self.shared.pending);
            pending.grasp = None;
            self.shared.target.store(NONE, Ordering::Release);
        }
        self.shared.commanded.store(NONE, Ordering::Relaxed);
        let shared = Arc::clone(&self.shared);
        let spawned = std::thread::Builder::new()
            .name("franka-hand-stop".into())
            .spawn(move || shared.note(shared.gripper.stop()));
        if let Err(e) = spawned {
            warn!("gripper: stop thread: {e}");
        }
    }

    fn state(&self) -> GripperState {
        let (s, t_ns) = *lock(&self.shared.state);
        let commanded = f64::from_bits(self.shared.commanded.load(Ordering::Relaxed));
        GripperState {
            width_m: s.width,
            commanded_m: if commanded.is_nan() {
                s.width
            } else {
                commanded
            },
            max_width_m: s.max_width,
            calibrated: s.max_width > 0.0,
            grasped: s.is_grasped,
            moving: self.shared.moving.load(Ordering::Relaxed),
            fault: self.shared.fault.load(Ordering::Relaxed)
                || self.shared.read_fault.load(Ordering::Relaxed),
            t_ns,
        }
    }
}
