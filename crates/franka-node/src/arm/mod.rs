//! The per-arm thread: owns the robot, the control handle, the gate, the state machine and
//! the arm's gripper, if any; fed through an unbounded channel with at most
//! [`CHANNEL_DEPTH`] targets in flight, publishes a [`StateMsg`] every tick, a
//! `GripperStateMsg` every [`gripper::GRIPPER_STATE_HZ`]th of a second and an
//! [`EpisodeMsg`] at every session's start and end.
//!
//! The robot and the control handle sit behind [`RobotSide`] and [`Control`] so the state
//! machine runs against a fake in the unit tests; the gripper behind [`crate::Gripper`].

mod commands;
pub mod gripper;
mod machine;
mod params;
mod record;
mod state;
#[cfg(test)]
mod tests;

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use franka::robot::target_control::{FieldBound, LiveTuning, TuningUpdate, NO_TUNING_MESSAGE};
use franka::{
    CartesianTargetControl, FciVersion, FrankaError, FrankaResult, JointTargetControl,
    JointTargetControlOptions, Model, Robot, RobotState, TargetControlOptions,
};

pub use gripper::{GripperSide, GRIPPER_STATE_HZ};
pub use params::{ParamsVerb, CURRENT_PERIOD};

use crate::config::ArmConfig;
use crate::msg::params::ParamsMsg;
use crate::msg::{CmdReply, CmdRequest, EpisodeMsg, GripperMsg, StateMsg, TargetMsg};
use crate::status::ArmStats;

/// The ready pose `home` drives to, rad.
pub const READY: [f64; 7] = [
    0.0,
    -FRAC_PI_4,
    0.0,
    -3.0 * FRAC_PI_4,
    0.0,
    FRAC_PI_2,
    FRAC_PI_4,
];
/// `home`'s default `speed`, the fraction of the joint limits it moves at.
pub const HOME_SPEED: f64 = 0.2;
/// What a `home` request's `speed` is clamped to.
pub const HOME_SPEED_RANGE: std::ops::RangeInclusive<f64> = 0.05..=0.5;
/// How close, rad, every measured joint must be to [`READY`] for `home` to count as arrived.
pub const HOME_TOLERANCE: f64 = 0.02;
/// Consecutive state ticks within [`HOME_TOLERANCE`] that count as arrived even while the
/// joints still report motion (rest below `REST_JOINT_VELOCITY` arrives at once).
pub const HOME_SETTLE_TICKS: u32 = 10;
/// How long `home` waits for the arrival before it stops and reports a timeout.
pub const HOME_TIMEOUT: Duration = Duration::from_secs(60);
/// Room, rad, the loop's deviation guard gets beyond the travel to [`READY`] during `home`
/// (the guard measures from the start, and the library's default of 1 rad is less than many
/// homes travel).
pub const HOME_DEVIATION_MARGIN: f64 = 0.5;

/// A running target control, Cartesian or joint, as the arm thread drives it.
pub trait Control: Send {
    /// Cartesian: position xyz and quaternion xyzw; joints: the seven positions, rad.
    fn set_target(&self, data: [f64; 7]) -> FrankaResult<()>;
    /// The current target in the same layout.
    fn target(&self) -> [f64; 7];
    fn state(&self) -> RobotState;
    fn is_running(&self) -> bool;
    /// The session's live tuning targets, or why it has none: a joints session's plan is per
    /// joint and its tracking is not what [`LiveTuning`] describes.
    fn tuning(&self) -> FrankaResult<LiveTuning>;
    /// Moves those targets, returning the bounds that had to clamp a value.
    fn tune(&self, update: &TuningUpdate) -> FrankaResult<Vec<&'static FieldBound>>;
    fn stop(self: Box<Self>) -> FrankaResult<()>;
}

impl Control for CartesianTargetControl {
    fn set_target(&self, [x, y, z, a, b, c, d]: [f64; 7]) -> FrankaResult<()> {
        CartesianTargetControl::set_target(self, [x, y, z], [a, b, c, d])
    }

    fn target(&self) -> [f64; 7] {
        let (p, o) = (
            CartesianTargetControl::target(self),
            self.target_orientation(),
        );
        [p[0], p[1], p[2], o[0], o[1], o[2], o[3]]
    }

    fn state(&self) -> RobotState {
        CartesianTargetControl::state(self)
    }

    fn is_running(&self) -> bool {
        CartesianTargetControl::is_running(self)
    }

    fn tuning(&self) -> FrankaResult<LiveTuning> {
        CartesianTargetControl::tuning(self)
    }

    fn tune(&self, update: &TuningUpdate) -> FrankaResult<Vec<&'static FieldBound>> {
        CartesianTargetControl::tune(self, update)
    }

    fn stop(self: Box<Self>) -> FrankaResult<()> {
        CartesianTargetControl::stop(*self)
    }
}

impl Control for JointTargetControl {
    fn set_target(&self, q: [f64; 7]) -> FrankaResult<()> {
        self.set_joints(q)
    }

    fn target(&self) -> [f64; 7] {
        JointTargetControl::target(self)
    }

    fn state(&self) -> RobotState {
        JointTargetControl::state(self)
    }

    fn is_running(&self) -> bool {
        JointTargetControl::is_running(self)
    }

    fn tuning(&self) -> FrankaResult<LiveTuning> {
        Err(FrankaError::InvalidOperation(NO_TUNING_MESSAGE.to_string()))
    }

    fn tune(&self, _update: &TuningUpdate) -> FrankaResult<Vec<&'static FieldBound>> {
        Err(FrankaError::InvalidOperation(NO_TUNING_MESSAGE.to_string()))
    }

    fn stop(self: Box<Self>) -> FrankaResult<()> {
        JointTargetControl::stop(*self)
    }
}

/// The robot, as the arm thread drives it.
pub trait RobotSide: Send + 'static {
    fn fci_version(&self) -> FciVersion;
    fn read_once(&self) -> FrankaResult<RobotState>;
    /// `set_collision_behavior_simple` with one torque, Nm, and one force, N, threshold.
    fn set_collision_behavior(&self, torque: f64, force: f64) -> FrankaResult<()>;
    fn start(&self, options: TargetControlOptions) -> FrankaResult<Box<dyn Control>>;
    fn start_joints(&self, options: JointTargetControlOptions) -> FrankaResult<Box<dyn Control>>;
    fn automatic_error_recovery(&self) -> FrankaResult<()>;
    /// The kinematic model, for the recording.
    fn load_model(&self) -> FrankaResult<Model>;
}

impl RobotSide for Arc<Robot> {
    fn fci_version(&self) -> FciVersion {
        Robot::fci_version(self)
    }

    fn load_model(&self) -> FrankaResult<Model> {
        Robot::load_model(self)
    }

    fn read_once(&self) -> FrankaResult<RobotState> {
        Robot::read_once(self)
    }

    fn set_collision_behavior(&self, torque: f64, force: f64) -> FrankaResult<()> {
        self.set_collision_behavior_simple([torque; 7], [torque; 7], [force; 6], [force; 6])
    }

    fn start(&self, options: TargetControlOptions) -> FrankaResult<Box<dyn Control>> {
        Ok(Box::new(self.start_cartesian_target_control(options)?))
    }

    fn start_joints(&self, options: JointTargetControlOptions) -> FrankaResult<Box<dyn Control>> {
        Ok(Box::new(self.start_joint_target_control(options)?))
    }

    fn automatic_error_recovery(&self) -> FrankaResult<()> {
        Robot::automatic_error_recovery(self)
    }
}

/// The `cmd/*` verbs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    Acquire,
    Release,
    Enable,
    Stop,
    Recover,
    Home,
    /// The gripper's homing; the holder's, answered when it is over.
    GripperHome,
    /// Stops the gripper; anyone's.
    GripperStop,
}

impl Verb {
    /// Every verb, in key order.
    pub const ALL: [Verb; 8] = [
        Verb::Acquire,
        Verb::Release,
        Verb::Enable,
        Verb::Stop,
        Verb::Recover,
        Verb::Home,
        Verb::GripperHome,
        Verb::GripperStop,
    ];

    /// The last key segment of the verb's queryable.
    pub fn key(self) -> &'static str {
        match self {
            Verb::Acquire => "acquire",
            Verb::Release => "release",
            Verb::Enable => "enable",
            Verb::Stop => "stop",
            Verb::Recover => "recover",
            Verb::Home => "home",
            Verb::GripperHome => "gripper_home",
            Verb::GripperStop => "gripper_stop",
        }
    }
}

/// Delivers a command's reply; the transport builds it around the query.
pub type Reply = Box<dyn FnOnce(CmdReply) + Send>;

/// Delivers a `params/*` reply, which is JSON of its own shape rather than a [`CmdReply`].
pub type JsonReply = Box<dyn FnOnce(String) + Send>;

/// What the arm thread receives.
pub enum Event {
    /// A decoded target and the node's [`crate::monotonic_ns`] at receipt.
    Target(TargetMsg, u64),
    /// A command with its request and its reply.
    Cmd(Verb, CmdRequest, Reply),
    /// A `params/*` query with the query's payload (empty where it takes none) and its reply.
    /// Decoded on the arm thread, which is where the values it would change live.
    Params(ParamsVerb, Vec<u8>, JsonReply),
    /// A decoded gripper command; never dropped, the driver keeps the latest.
    Gripper(GripperMsg),
    LeaseAlive(u32),
    LeaseLost(u32),
    Shutdown,
}

/// Targets the channel holds before new ones are dropped.
pub const CHANNEL_DEPTH: usize = 64;

/// Targets in flight and targets dropped at [`CHANNEL_DEPTH`], shared by the senders and
/// the arm thread.
#[derive(Default)]
pub(crate) struct Backlog {
    inflight: AtomicUsize,
    dropped: AtomicU64,
}

impl Backlog {
    /// Takes a slot for a target; `false`, and counted, when none is free.
    fn reserve(&self) -> bool {
        if self.inflight.fetch_add(1, Ordering::AcqRel) >= CHANNEL_DEPTH {
            self.inflight.fetch_sub(1, Ordering::AcqRel);
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        true
    }

    /// The arm thread took a target out of the channel.
    pub(crate) fn release(&self) {
        self.inflight.fetch_sub(1, Ordering::AcqRel);
    }

    pub(crate) fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// The transport's end of an arm's channel: `Clone` for every callback.
#[derive(Clone)]
pub struct ArmSender {
    tx: Sender<Event>,
    backlog: Arc<Backlog>,
}

impl ArmSender {
    /// Queues `event` without blocking. A target beyond [`CHANNEL_DEPTH`] in flight is
    /// dropped and counted; commands and lease events are never dropped.
    pub fn send(&self, event: Event) {
        if matches!(event, Event::Target(..)) && !self.backlog.reserve() {
            return;
        }
        let _ = self.tx.send(event);
    }
}

fn channel() -> (ArmSender, Receiver<Event>) {
    let (tx, rx) = mpsc::channel();
    let backlog = Arc::new(Backlog::default());
    (ArmSender { tx, backlog }, rx)
}

/// The arm thread and its channel.
pub struct ArmHandle {
    sender: ArmSender,
    stats: Arc<ArmStats>,
    join: Option<JoinHandle<()>>,
}

impl ArmHandle {
    /// A sender for a callback.
    pub fn sender(&self) -> ArmSender {
        self.sender.clone()
    }

    /// The atomics the thread stores its phase, holder, mode and counters into every tick.
    pub fn stats(&self) -> &Arc<ArmStats> {
        &self.stats
    }

    /// [`ArmSender::send`].
    pub fn send(&self, event: Event) {
        self.sender.send(event);
    }

    /// Stops the loop if it runs and joins the thread.
    pub fn shutdown(mut self) {
        self.finish();
    }

    fn finish(&mut self) {
        if let Some(join) = self.join.take() {
            self.sender.send(Event::Shutdown);
            let _ = join.join();
        }
    }
}

impl Drop for ArmHandle {
    fn drop(&mut self) {
        self.finish();
    }
}

/// Spawns the thread of one arm; `publish` is called with every encoded state, `episode`
/// with every session's start and end, `gripper` is the arm's gripper with its own
/// publisher, if it has one. `arms` is every arm name of the node, this one included: a
/// recording several arms share has to lay out all of them.
pub fn spawn(
    config: ArmConfig,
    arms: &[String],
    robot: impl RobotSide,
    gripper: Option<GripperSide>,
    publish: impl Fn(&StateMsg) + Send + 'static,
    episode: impl Fn(&EpisodeMsg) + Send + 'static,
    params: impl Fn(&ParamsMsg) + Send + 'static,
) -> std::io::Result<ArmHandle> {
    let (sender, rx) = channel();
    let name = format!("franka-node-{}", config.name);
    let backlog = Arc::clone(&sender.backlog);
    let stats = Arc::new(ArmStats::default());
    let publishers = machine::Publishers {
        state: Box::new(publish),
        episode: Box::new(episode),
        params: Box::new(params),
    };
    let mut machine = machine::Machine::new(
        config,
        arms,
        robot,
        publishers,
        backlog,
        Arc::clone(&stats),
        gripper,
    );
    let join = std::thread::Builder::new()
        .name(name)
        .spawn(move || machine.run(rx))?;
    Ok(ArmHandle {
        sender,
        stats,
        join: Some(join),
    })
}
