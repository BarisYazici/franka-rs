//! How a control cycle is recorded: a `RobotSide` delegate that installs a Cartesian
//! observer, and the bounded channel the observer copies into.
//!
//! The observer runs on the 1 kHz thread, so it does what the flight recorder does: one
//! `try_send` into a preallocated bounded channel, no allocation, no lock, and a drop counter
//! behind `try_lock` so even the counter cannot block.

// Shared by two test binaries: the asserting `sim_tuning` suite and the `sim_sweep` campaign
// script, which each use a different part of this module. Rust's dead-code analysis is per
// binary and has no view of the other one, so the parts one of them does not reach are not
// dead -- they are the other's. It is a blanket allow: something that went dead in *both*
// would be silent here, so a reader deleting from this module should check both binaries.
#![allow(dead_code)]

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use franka::model::Model;
use franka::robot::target_control::{
    CartesianSent, JointTargetControlOptions, TargetControlOptions,
};
use franka::{FciVersion, FrankaResult, Robot, RobotState};
use franka_node::{Control, RobotSide};

/// One control cycle, as the observer saw it.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    /// Nanoseconds since the rig started.
    pub t_ns: u64,
    pub q: [f64; 7],
    pub dq: [f64; 7],
    pub q_goal: [f64; 7],
    pub dq_goal: [f64; 7],
    /// What the loop sent: `clamp(law + tau_envelope + tau_position)`.
    pub tau: [f64; 7],
    pub tau_envelope: [f64; 7],
    pub tau_position: [f64; 7],
    /// The generator's translational velocity at the end of the cycle.
    pub velocity: [f64; 3],
    pub cap_scale: f64,
    pub success_rate: f64,
    /// The commanded EE position, and the measured one.
    pub cmd: [f64; 3],
    pub measured: [f64; 3],
}

/// The real robot, with a Cartesian observer spliced into whatever options the node builds.
/// Every other call is a plain delegate: no production code is changed to make this work.
pub struct TapRobot {
    robot: Arc<Robot>,
    tx: SyncSender<Sample>,
    start: Instant,
    dropped: Arc<AtomicU64>,
}

impl RobotSide for TapRobot {
    fn fci_version(&self) -> FciVersion {
        RobotSide::fci_version(&self.robot)
    }

    fn read_once(&self) -> FrankaResult<RobotState> {
        RobotSide::read_once(&self.robot)
    }

    fn set_collision_behavior(&self, torque: f64, force: f64) -> FrankaResult<()> {
        RobotSide::set_collision_behavior(&self.robot, torque, force)
    }

    fn load_model(&self) -> FrankaResult<Model> {
        RobotSide::load_model(&self.robot)
    }

    fn automatic_error_recovery(&self) -> FrankaResult<()> {
        RobotSide::automatic_error_recovery(&self.robot)
    }

    fn start_joints(&self, options: JointTargetControlOptions) -> FrankaResult<Box<dyn Control>> {
        RobotSide::start_joints(&self.robot, options)
    }

    fn start(&self, options: TargetControlOptions) -> FrankaResult<Box<dyn Control>> {
        let (tx, start, dropped) = (self.tx.clone(), self.start, Arc::clone(&self.dropped));
        let options = options.with_observer(move |state: &RobotState, sent: &CartesianSent| {
            let sample = Sample {
                t_ns: start.elapsed().as_nanos() as u64,
                q: state.q,
                dq: state.dq,
                q_goal: sent.q_goal,
                dq_goal: sent.dq_goal,
                tau: sent.tau,
                tau_envelope: sent.tau_envelope,
                tau_position: sent.tau_position,
                velocity: sent.velocity,
                cap_scale: sent.cap_scale,
                success_rate: state.control_command_success_rate,
                cmd: [sent.pose[12], sent.pose[13], sent.pose[14]],
                measured: [state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]],
            };
            if tx.try_send(sample).is_err() {
                dropped.fetch_add(1, Ordering::Relaxed);
            }
        });
        self.robot.start(options)
    }
}

/// The samples the tap produced, drained on an ordinary thread.
pub struct Trace {
    samples: Arc<Mutex<Vec<Sample>>>,
    dropped: Arc<AtomicU64>,
    drain: Option<std::thread::JoinHandle<()>>,
}

impl Trace {
    /// A tap for a run of at most `cycles` control cycles, and the `RobotSide` to hand the
    /// node. The channel and the sink are both sized once, here.
    pub fn tap(robot: Arc<Robot>, start: Instant, cycles: usize) -> (TapRobot, Trace) {
        let (tx, rx) = sync_channel::<Sample>(cycles);
        let dropped = Arc::new(AtomicU64::new(0));
        let tap = TapRobot {
            robot,
            tx,
            start,
            dropped: Arc::clone(&dropped),
        };
        (tap, Trace::drain(rx, dropped, cycles))
    }

    fn drain(rx: Receiver<Sample>, dropped: Arc<AtomicU64>, cycles: usize) -> Trace {
        let samples = Arc::new(Mutex::new(Vec::with_capacity(cycles)));
        let sink = Arc::clone(&samples);
        let drain = std::thread::spawn(move || {
            while let Ok(sample) = rx.recv() {
                sink.lock().unwrap().push(sample);
            }
        });
        Trace {
            samples,
            dropped,
            drain: Some(drain),
        }
    }

    /// The cycles recorded between two rig timestamps.
    pub fn between(&self, from_ns: u64, to_ns: u64) -> Vec<Sample> {
        self.samples
            .lock()
            .unwrap()
            .iter()
            .copied()
            .filter(|s| s.t_ns >= from_ns && s.t_ns <= to_ns)
            .collect()
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Joins the drain thread, so a test cannot tear down while it is still pushing.
    pub fn finish(&mut self) {
        if let Some(drain) = self.drain.take() {
            let _ = drain.join();
        }
    }
}
