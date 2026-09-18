//! The state machine against a fake robot; `thread` covers the channel and the spawned loop,
//! `home` the joints mode and the `home` verb, `gripper` the gripper against a fake driver.

mod gripper;
mod home;
#[cfg(feature = "record")]
mod record;
mod thread;

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use franka::{
    FciVersion, FrankaError, FrankaResult, JointTargetControlOptions, RobotState,
    TargetControlOptions,
};

use super::machine::{Flow, Machine, Publishers};
use super::{
    channel, spawn, ArmSender, Control, Event, GripperSide, RobotSide, Verb, CHANNEL_DEPTH, READY,
};
use crate::config::{ArmConfig, NodeConfig};
use crate::msg::{
    CmdReply, CmdRequest, EpisodeMsg, EpisodePhase, Kind, Phase, StateMsg, TargetMsg, FLAG_HOLDING,
};
use crate::status::ArmStats;

const CLIENT: u32 = 7;
const OTHER: u32 = 9;
/// What the fake Cartesian control starts at: inside the default workspace box.
const START: [f64; 7] = [0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0];
/// Where the fake arm stands: [`READY`] with joint 4 bent 0.3 rad further.
const START_Q: [f64; 7] = [
    READY[0],
    READY[1],
    READY[2],
    READY[3] - 0.3,
    READY[4],
    READY[5],
    READY[6],
];

struct Fake {
    calls: Mutex<Vec<&'static str>>,
    /// Every `set_target` of the current control, whatever its kind.
    targets: Mutex<Vec<[f64; 7]>>,
    running: AtomicBool,
    set_target_fails: AtomicBool,
    stop_fails: AtomicBool,
    start_fails: AtomicBool,
    joints: AtomicBool,
    /// The limit fraction and the `max_deviation` of every `start_joints`.
    fractions: Mutex<Vec<(f64, f64)>>,
    /// Where the fake arm's `O_T_EE` puts it; [`START`]'s position until something pushes it.
    measured: Mutex<[f64; 3]>,
    /// `state()` calls left before the arm reports [`READY`].
    ready_after: AtomicUsize,
    /// Whether the arm still reports motion once at [`READY`].
    moving_at_ready: AtomicBool,
    /// The observer of the running Cartesian control, as the loop would hold it.
    cartesian_observer: Mutex<Option<franka::robot::target_control::CartesianObserver>>,
}

impl Default for Fake {
    fn default() -> Self {
        Fake {
            calls: Mutex::default(),
            targets: Mutex::default(),
            running: AtomicBool::new(false),
            set_target_fails: AtomicBool::new(false),
            stop_fails: AtomicBool::new(false),
            start_fails: AtomicBool::new(false),
            joints: AtomicBool::new(false),
            measured: Mutex::new([START[0], START[1], START[2]]),
            fractions: Mutex::default(),
            ready_after: AtomicUsize::new(usize::MAX),
            moving_at_ready: AtomicBool::new(false),
            cartesian_observer: Mutex::default(),
        }
    }
}

impl Fake {
    fn calls(&self) -> Vec<&'static str> {
        self.calls.lock().unwrap().clone()
    }

    /// One `state()` of the control has passed.
    fn advance(&self) {
        let _ = self
            .ready_after
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |n| {
                (n != 0 && n != usize::MAX).then(|| n - 1)
            });
    }

    /// The arm is dragged to `position`, as a push on the impedance backend drags it.
    fn push(&self, position: [f64; 3]) {
        *self.measured.lock().unwrap() = position;
    }

    fn state(&self) -> RobotState {
        let moving = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let (q, dq) = if self.ready_after.load(Ordering::SeqCst) != 0 {
            (START_Q, moving)
        } else if self.moving_at_ready.load(Ordering::SeqCst) {
            (READY, moving)
        } else {
            (READY, [0.0; 7])
        };
        let [x, y, z] = *self.measured.lock().unwrap();
        RobotState {
            // The fake arm stands on [`START`], the fake control's start target, so the gate's
            // lead check has a coherent reference.
            O_T_EE: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, z, 1.0,
            ],
            control_command_success_rate: 0.97,
            q,
            dq,
            ..RobotState::default()
        }
    }
}

struct FakeControl(Arc<Fake>);

impl Control for FakeControl {
    fn set_target(&self, data: [f64; 7]) -> FrankaResult<()> {
        if self.0.set_target_fails.load(Ordering::SeqCst) {
            return Err(FrankaError::InvalidOperation("ended".into()));
        }
        self.0.targets.lock().unwrap().push(data);
        Ok(())
    }

    fn target(&self) -> [f64; 7] {
        let start = if self.0.joints.load(Ordering::SeqCst) {
            START_Q
        } else {
            START
        };
        self.0
            .targets
            .lock()
            .unwrap()
            .last()
            .copied()
            .unwrap_or(start)
    }

    fn state(&self) -> RobotState {
        let state = self.0.state();
        self.0.advance();
        state
    }

    fn is_running(&self) -> bool {
        self.0.running.load(Ordering::SeqCst)
    }

    fn stop(self: Box<Self>) -> FrankaResult<()> {
        *self.0.cartesian_observer.lock().unwrap() = None;
        self.0.calls.lock().unwrap().push("stop");
        self.0.running.store(false, Ordering::SeqCst);
        if self.0.stop_fails.load(Ordering::SeqCst) {
            return Err(FrankaError::Control(franka::ControlException::new(
                "aborted",
            )));
        }
        Ok(())
    }
}

#[derive(Clone)]
struct FakeRobot(Arc<Fake>);

impl FakeRobot {
    fn start_control(&self, what: &'static str, joints: bool) -> FrankaResult<Box<dyn Control>> {
        let fake = &self.0;
        fake.calls.lock().unwrap().push(what);
        if fake.start_fails.load(Ordering::SeqCst) {
            return Err(FrankaError::InvalidOperation("busy".into()));
        }
        fake.running.store(true, Ordering::SeqCst);
        fake.joints.store(joints, Ordering::SeqCst);
        fake.targets.lock().unwrap().clear();
        Ok(Box::new(FakeControl(Arc::clone(fake))))
    }
}

impl RobotSide for FakeRobot {
    fn fci_version(&self) -> FciVersion {
        FciVersion::V10
    }

    fn read_once(&self) -> FrankaResult<RobotState> {
        self.0.calls.lock().unwrap().push("read_once");
        Ok(self.0.state())
    }

    fn set_collision_behavior(&self, _torque: f64, _force: f64) -> FrankaResult<()> {
        self.0.calls.lock().unwrap().push("collision");
        Ok(())
    }

    fn start(&self, options: TargetControlOptions) -> FrankaResult<Box<dyn Control>> {
        *self.0.cartesian_observer.lock().unwrap() = options.observer;
        self.start_control("start", false)
    }

    fn start_joints(&self, options: JointTargetControlOptions) -> FrankaResult<Box<dyn Control>> {
        let limits = options
            .limits
            .expect("the node always sets the joint limits");
        // The FR3 table at the fraction: joint 1 caps at 2.62 rad/s.
        self.0
            .fractions
            .lock()
            .unwrap()
            .push((limits[0].max_velocity / 2.62, options.max_deviation));
        self.start_control("start_joints", true)
    }

    fn automatic_error_recovery(&self) -> FrankaResult<()> {
        self.0.calls.lock().unwrap().push("recover");
        Ok(())
    }

    fn load_model(&self) -> FrankaResult<franka::Model> {
        self.0.calls.lock().unwrap().push("load_model");
        Ok(franka::Model::native_fer())
    }
}

const CONFIG: &str = r#"
[[arm]]
name = "t"
host = "robot"
realtime = "ignore"
hold_after_ms = 20
stop_after_ms = 60
"#;

struct Rig {
    fake: Arc<Fake>,
    machine: Machine<FakeRobot>,
    published: Arc<Mutex<Vec<StateMsg>>>,
    episodes: Arc<Mutex<Vec<EpisodeMsg>>>,
    stats: Arc<ArmStats>,
    sender: ArmSender,
    rx: Receiver<Event>,
}

fn arm_config() -> ArmConfig {
    CONFIG.parse::<NodeConfig>().unwrap().arms.remove(0)
}

fn rig() -> Rig {
    rig_with(arm_config())
}

fn rig_with(config: ArmConfig) -> Rig {
    rig_with_gripper(config, None)
}

fn rig_with_gripper(config: ArmConfig, gripper: Option<GripperSide>) -> Rig {
    rig_of(config, &[], gripper)
}

/// A rig whose node has the arms `arms` beside this one's config (for the recording's layout).
fn rig_of(config: ArmConfig, arms: &[String], gripper: Option<GripperSide>) -> Rig {
    let fake = Arc::new(Fake::default());
    let published = Arc::new(Mutex::new(Vec::new()));
    let episodes = Arc::new(Mutex::new(Vec::new()));
    let stats = Arc::new(ArmStats::default());
    let (sender, rx) = channel();
    let sink = Arc::clone(&published);
    let episode_sink = Arc::clone(&episodes);
    let names: Vec<String> = if arms.is_empty() {
        vec![config.name.clone()]
    } else {
        arms.to_vec()
    };
    let publishers = Publishers {
        state: Box::new(move |state: &StateMsg| sink.lock().unwrap().push(*state)),
        episode: Box::new(move |episode: &EpisodeMsg| {
            episode_sink.lock().unwrap().push(episode.clone())
        }),
    };
    let machine = Machine::new(
        config,
        &names,
        FakeRobot(Arc::clone(&fake)),
        publishers,
        Arc::clone(&sender.backlog),
        Arc::clone(&stats),
        gripper,
    );
    Rig {
        fake,
        machine,
        published,
        episodes,
        stats,
        sender,
        rx,
    }
}

fn target_msg(seq: u64, data: [f64; 7]) -> TargetMsg {
    TargetMsg::new(Kind::Cartesian, CLIENT, seq, 1000 + seq, data)
}

impl Rig {
    /// Sends `request` for `verb`; the reply arrives on the receiver when the machine answers.
    fn request(&mut self, verb: Verb, request: CmdRequest) -> Receiver<CmdReply> {
        let (tx, rx) = mpsc::channel();
        self.machine.handle(Event::Cmd(
            verb,
            request,
            Box::new(move |reply| tx.send(reply).unwrap()),
        ));
        rx
    }

    fn cmd(&mut self, verb: Verb, client: u32) -> CmdReply {
        self.request(verb, CmdRequest::new(client)).recv().unwrap()
    }

    fn ok(&mut self, verb: Verb, client: u32) {
        let reply = self.cmd(verb, client);
        assert_eq!(reply, CmdReply::ok(), "{verb:?} by {client}");
    }

    fn err(&mut self, verb: Verb, client: u32) -> String {
        let reply = self.cmd(verb, client);
        assert!(!reply.ok, "{verb:?} by {client} was accepted");
        reply.error.unwrap()
    }

    fn state(&mut self) -> StateMsg {
        self.machine.encode()
    }

    fn phase(&mut self) -> Phase {
        Phase::from_u8(self.state().phase).unwrap()
    }

    fn activate(&mut self) {
        self.activate_with(CmdRequest::new(CLIENT));
    }

    /// The lease, `acquire` and an `enable` carrying `request`.
    fn activate_with(&mut self, request: CmdRequest) {
        self.machine.handle(Event::LeaseAlive(CLIENT));
        self.ok(Verb::Acquire, CLIENT);
        let reply = self.request(Verb::Enable, request).recv().unwrap();
        assert_eq!(reply, CmdReply::ok());
        assert_eq!(self.phase(), Phase::Active);
    }

    fn target(&mut self, seq: u64, data: [f64; 7]) {
        self.machine.handle(Event::Target(target_msg(seq, data), 0));
    }

    /// A target with [`crate::msg::TARGET_ANCHOR`] set.
    fn anchored(&mut self, seq: u64, data: [f64; 7]) {
        self.machine
            .handle(Event::Target(target_msg(seq, data).with_anchor(), 0));
    }

    fn phases(&self) -> Vec<Phase> {
        let published = self.published.lock().unwrap();
        published
            .iter()
            .map(|s| Phase::from_u8(s.phase).unwrap())
            .collect()
    }
}

fn moved(dz: f64) -> [f64; 7] {
    [START[0], START[1], START[2] + dz, 0.0, 0.0, 0.0, 1.0]
}

#[test]
fn acquire_needs_an_alive_lease_and_a_free_arm() {
    let mut rig = rig();
    assert_eq!(rig.err(Verb::Acquire, CLIENT), "no lease");
    assert_eq!(rig.phase(), Phase::Idle);
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    assert_eq!(rig.phase(), Phase::Acquired);
    assert_eq!(rig.state().client_id.get(), CLIENT);
    rig.ok(Verb::Acquire, CLIENT);
    rig.machine.handle(Event::LeaseAlive(OTHER));
    assert_eq!(rig.err(Verb::Acquire, OTHER), "held by 7");
    assert_eq!(rig.err(Verb::Enable, OTHER), "client 9 is not the holder");
}

#[test]
fn enable_reads_sets_collision_starts_and_seeds_the_guard() {
    let mut rig = rig();
    assert!(rig.err(Verb::Enable, CLIENT).contains("not the holder"));
    rig.activate();
    assert_eq!(rig.fake.calls(), ["read_once", "collision", "start"]);
    // No record_dir: no model, no recording.
    assert_eq!(rig.stats.snapshot("t").recording, None);
    let state = rig.state();
    assert_eq!(state.target.map(|v| v.get()), START);
    assert_eq!(state.client_id.get(), CLIENT);
    assert_eq!(state.accepted.get(), 0);
    assert_eq!(state.flags.get(), 0);
    assert_eq!(rig.err(Verb::Acquire, CLIENT), "not allowed in active");
    assert_eq!(rig.err(Verb::Enable, CLIENT), "not allowed in active");
    assert_eq!(rig.err(Verb::Release, CLIENT), "not allowed in active");
}

#[test]
fn enable_failure_keeps_the_phase() {
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.fake.start_fails.store(true, Ordering::SeqCst);
    assert_eq!(rig.err(Verb::Enable, CLIENT), "busy");
    assert_eq!(rig.phase(), Phase::Acquired);
}

#[test]
fn targets_outside_active_are_refused() {
    let mut rig = rig();
    rig.target(1, moved(0.01));
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.target(2, moved(0.01));
    assert_eq!(rig.state().refused.get(), 2);
    assert!(rig.fake.targets.lock().unwrap().is_empty());
}

#[test]
fn accepted_targets_reach_the_control_and_the_state() {
    let mut rig = rig();
    rig.activate();
    rig.target(1, moved(0.01));
    rig.target(2, moved(0.02));
    assert_eq!(rig.fake.targets.lock().unwrap().len(), 2);
    let state = rig.state();
    assert_eq!(state.seq_accepted.get(), 2);
    assert_eq!(state.t_send_ns_accepted.get(), 1002);
    assert_eq!(state.target[2].get(), START[2] + 0.02);
    assert_eq!(state.accepted.get(), 2);
    assert_eq!(state.refused.get(), 0);
}

#[test]
fn a_target_that_leads_the_measured_pose_is_refused_and_counted() {
    let mut rig = rig();
    rig.activate();
    // The arm stands still, so the target cannot walk away from it in legal 4 cm steps: the
    // second one leads by 8 cm and is refused, and the arm keeps the target it can reach.
    rig.target(1, moved(0.04));
    rig.target(2, moved(0.08));
    let state = rig.state();
    assert_eq!((state.accepted.get(), state.refused.get()), (1, 1));
    assert_eq!(state.target.map(|v| v.get()), moved(0.04));
    assert_eq!(*rig.fake.targets.lock().unwrap(), [moved(0.04)]);
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn an_anchor_re_anchors_a_commander_the_step_limit_locked_out() {
    let mut rig = rig();
    rig.activate();
    // A push drags the arm 15 cm off the target it is holding; the commander follows the arm,
    // which is a legal lead but more than max_step from the last accepted target, so it is
    // locked out until it anchors once.
    rig.fake.push([START[0], START[1], START[2] - 0.15]);
    rig.machine.tick();
    rig.target(1, moved(-0.14));
    assert_eq!(rig.state().refused.get(), 1);
    rig.anchored(2, moved(-0.14));
    let state = rig.state();
    assert_eq!((state.accepted.get(), state.refused.get()), (1, 1));
    assert_eq!(state.target.map(|v| v.get()), moved(-0.14));
    // An anchor is no licence: one beyond the lead from the pushed arm is refused too.
    rig.anchored(3, moved(0.04));
    let state = rig.state();
    assert_eq!((state.accepted.get(), state.refused.get()), (1, 2));
    assert_eq!(state.target.map(|v| v.get()), moved(-0.14));
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn success_rate_is_the_robots_and_not_the_acceptance_ratio() {
    let mut rig = rig();
    rig.activate();
    rig.target(1, moved(0.01));
    rig.target(1, moved(0.01));
    rig.machine.tick();
    let state = rig.state();
    assert_eq!((state.accepted.get(), state.refused.get()), (1, 1));
    assert_eq!(state.success_rate.get(), 0.97);
}

#[test]
fn refusals_are_counted_and_keep_the_arm_active() {
    let mut rig = rig();
    rig.activate();
    rig.target(1, moved(0.01));
    rig.target(2, [f64::NAN, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]);
    rig.target(3, moved(1.0));
    rig.target(1, moved(0.01));
    let state = rig.state();
    assert_eq!(state.refused.get(), 3);
    assert_eq!(state.accepted.get(), 1);
    assert_eq!(state.seq_accepted.get(), 1);
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn set_target_error_faults_then_recover_returns_to_idle_keeping_the_holder() {
    let mut rig = rig();
    rig.activate();
    rig.fake.set_target_fails.store(true, Ordering::SeqCst);
    rig.target(1, moved(0.01));
    assert_eq!(rig.phase(), Phase::Faulted);
    // A session that ends in a fault still ends its episode.
    let samples = rig.episodes.lock().unwrap().clone();
    assert_eq!(samples.len(), 2, "{samples:?}");
    assert_eq!(samples[1].phase, EpisodePhase::End);
    assert_eq!(samples[1].recording_id, samples[0].recording_id);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert_eq!(rig.err(Verb::Enable, CLIENT), "not allowed in faulted");
    assert_eq!(rig.err(Verb::Stop, CLIENT), "not allowed in faulted");
    rig.ok(Verb::Recover, CLIENT);
    assert_eq!(rig.fake.calls().last(), Some(&"recover"));
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), CLIENT);
    rig.fake.set_target_fails.store(false, Ordering::SeqCst);
    rig.ok(Verb::Enable, CLIENT);
}

#[test]
fn recover_runs_in_every_phase_but_active() {
    let mut rig = rig();
    rig.ok(Verb::Recover, CLIENT);
    assert_eq!(rig.fake.calls(), ["recover"]);
    assert_eq!(rig.phase(), Phase::Idle);
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.ok(Verb::Recover, CLIENT);
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), CLIENT);
    rig.ok(Verb::Enable, CLIENT);
    assert_eq!(rig.err(Verb::Recover, CLIENT), "not allowed in active");
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn release_in_faulted_clears_the_holder_and_keeps_the_phase() {
    let mut rig = rig();
    rig.activate();
    rig.fake.stop_fails.store(true, Ordering::SeqCst);
    assert_eq!(rig.err(Verb::Stop, CLIENT), "aborted");
    assert_eq!(rig.err(Verb::Release, OTHER), "client 9 is not the holder");
    rig.ok(Verb::Release, CLIENT);
    assert_eq!(rig.phase(), Phase::Faulted);
    assert_eq!(rig.state().client_id.get(), 0);
    rig.ok(Verb::Recover, CLIENT);
    assert_eq!(rig.phase(), Phase::Idle);
}

#[test]
fn a_loop_that_ended_is_noticed_on_the_tick() {
    let mut rig = rig();
    rig.activate();
    rig.fake.running.store(false, Ordering::SeqCst);
    rig.fake.stop_fails.store(true, Ordering::SeqCst);
    rig.machine.tick();
    assert_eq!(rig.phase(), Phase::Faulted);
    let reply = rig.cmd(Verb::Stop, CLIENT);
    assert!(!reply.ok);
}

#[test]
fn stop_passes_through_stopping_to_idle_and_keeps_the_holder() {
    let mut rig = rig();
    assert_eq!(rig.err(Verb::Stop, CLIENT), "not allowed in idle");
    rig.activate();
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.phases(), [Phase::Stopping]);
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), CLIENT);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    rig.ok(Verb::Enable, CLIENT);
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn enable_resets_the_counters() {
    let mut rig = rig();
    rig.target(1, moved(0.01));
    rig.activate();
    assert_eq!(rig.state().refused.get(), 0);
    rig.target(1, moved(0.01));
    rig.target(1, moved(0.01));
    for seq in 2..4 {
        rig.sender
            .send(Event::Target(target_msg(seq, moved(0.01)), 0));
    }
    let first = rig.rx.recv().unwrap();
    rig.machine.batch(first, &rig.rx);
    let state = rig.state();
    assert_eq!(
        (
            state.accepted.get(),
            state.refused.get(),
            state.dropped.get(),
            state.seq_accepted.get()
        ),
        (2, 1, 1, 3)
    );
    rig.ok(Verb::Stop, CLIENT);
    rig.target(4, moved(0.01));
    assert_eq!(rig.state().refused.get(), 2);
    rig.ok(Verb::Enable, CLIENT);
    let state = rig.state();
    assert_eq!(
        (
            state.accepted.get(),
            state.refused.get(),
            state.dropped.get(),
            state.seq_accepted.get()
        ),
        (0, 0, 0, 0)
    );
}

#[test]
fn a_failing_stop_faults() {
    let mut rig = rig();
    rig.activate();
    rig.fake.stop_fails.store(true, Ordering::SeqCst);
    assert_eq!(rig.err(Verb::Stop, CLIENT), "aborted");
    assert_eq!(rig.phase(), Phase::Faulted);
}

#[test]
fn watchdog_flags_holding_then_stops_after_silence_and_keeps_the_holder() {
    let mut rig = rig();
    rig.activate();
    rig.target(1, moved(0.01));
    std::thread::sleep(Duration::from_millis(30));
    rig.machine.tick();
    assert_eq!(rig.phase(), Phase::Active);
    assert_eq!(rig.state().flags.get(), FLAG_HOLDING);
    rig.target(2, moved(0.02));
    assert_eq!(rig.state().flags.get(), 0);
    std::thread::sleep(Duration::from_millis(30));
    rig.machine.tick();
    assert_eq!(rig.state().flags.get(), FLAG_HOLDING);
    std::thread::sleep(Duration::from_millis(40));
    rig.machine.tick();
    assert_eq!(rig.phase(), Phase::Idle);
    let state = rig.state();
    assert_eq!(state.flags.get(), 0);
    assert_eq!(state.client_id.get(), CLIENT);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    rig.ok(Verb::Enable, CLIENT);
}

#[test]
fn refused_targets_hold_the_arm_but_do_not_stop_the_session() {
    // hold_after_ms 20, stop_after_ms 60. A commander whose every target is refused -- here
    // for the lead, the case a hand outrunning the arm produces -- is alive, so it is held and
    // not stopped: at 0.5 m/s a hand crosses the lead in 0.16 s, and stopping the session takes
    // the loop away from it for as long as the settle lasts.
    let mut rig = rig();
    rig.activate();
    let mut seq = 0;
    for _ in 0..7 {
        seq += 1;
        rig.target(seq, moved(0.4));
        std::thread::sleep(Duration::from_millis(20));
        rig.machine.tick();
    }
    let state = rig.state();
    assert_eq!(rig.phase(), Phase::Active, "the session was stopped");
    assert_eq!(state.flags.get(), FLAG_HOLDING);
    assert_eq!((state.accepted.get(), state.refused.get()), (0, 7));
    // Silence still stops it, and `holding` still means "nothing accepted".
    std::thread::sleep(Duration::from_millis(70));
    rig.machine.tick();
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
}

#[test]
fn a_refusal_that_is_not_the_commanders_does_not_feed_the_watchdog() {
    // A target from someone who is not the holder, or out of order, says nothing about the
    // commander, so it must not keep a dead session alive.
    let mut rig = rig();
    rig.activate();
    for _ in 0..4 {
        rig.machine.handle(Event::Target(
            TargetMsg::new(Kind::Cartesian, OTHER, 1, 0, moved(0.01)),
            0,
        ));
        std::thread::sleep(Duration::from_millis(20));
        rig.machine.tick();
    }
    assert_eq!(rig.phase(), Phase::Idle);
    assert!(rig.fake.calls().contains(&"stop"));
    // Counted all the same, the last of them as "not active".
    assert_eq!(rig.state().refused.get(), 4);
}

#[test]
fn lease_lost_in_active_stops_and_clears_the_holder() {
    let mut rig = rig();
    rig.activate();
    rig.machine.handle(Event::LeaseLost(OTHER));
    assert_eq!(rig.phase(), Phase::Active);
    rig.machine.handle(Event::LeaseLost(CLIENT));
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), 0);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert_eq!(rig.err(Verb::Acquire, CLIENT), "no lease");
}

#[test]
fn lease_lost_in_acquired_clears_the_holder() {
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.machine.handle(Event::LeaseLost(CLIENT));
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), 0);
}

#[test]
fn release_from_acquired_or_idle_with_holder() {
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    assert_eq!(rig.err(Verb::Release, OTHER), "client 9 is not the holder");
    rig.ok(Verb::Release, CLIENT);
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), 0);
    rig.activate();
    rig.ok(Verb::Stop, CLIENT);
    rig.ok(Verb::Release, CLIENT);
    assert_eq!(rig.state().client_id.get(), 0);
}

#[test]
fn idle_ticks_read_once_per_second_and_publish() {
    let mut rig = rig();
    rig.machine.tick();
    rig.machine.tick();
    assert_eq!(rig.fake.calls(), ["read_once"]);
    assert_eq!(rig.phases(), [Phase::Idle, Phase::Idle]);
    let state = rig.state();
    assert_eq!(state.target.map(|v| v.get()), START);
    assert_eq!(state.o_t_ee[14].get(), START[2]);
}

#[test]
fn a_session_publishes_an_episode_start_and_end() {
    let mut rig = rig();
    assert!(rig.episodes.lock().unwrap().is_empty());
    rig.activate();
    let start = rig.episodes.lock().unwrap().clone();
    assert_eq!(start.len(), 1);
    assert_eq!(start[0].phase, EpisodePhase::Start);
    assert_eq!(start[0].arm, "t");
    // This config records nothing, so there is no file and the id is the generated one;
    // `tests::record` covers the id of a real recording.
    assert!(start[0].file.is_none());
    assert!(start[0].recording_id.starts_with("t-"));
    assert_eq!(
        rig.stats.snapshot("t").episode.as_deref(),
        Some(start[0].recording_id.as_str())
    );
    rig.ok(Verb::Stop, CLIENT);
    let both = rig.episodes.lock().unwrap().clone();
    assert_eq!(both.len(), 2);
    assert_eq!(both[1].phase, EpisodePhase::End);
    assert_eq!(both[1].recording_id, both[0].recording_id);
    assert!(both[1].t_node_ns >= both[0].t_node_ns);
    // The id of the last episode outlives it, so a late subscriber can still find the file.
    assert_eq!(
        rig.stats.snapshot("t").episode.as_deref(),
        Some(both[0].recording_id.as_str())
    );
}

#[test]
fn an_episode_name_becomes_the_sessions_id_and_a_bad_one_is_refused() {
    let named = |token: &str| CmdRequest {
        episode: Some(token.to_string()),
        ..CmdRequest::new(CLIENT)
    };
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    let reply = rig.request(Verb::Enable, named("../x")).recv().unwrap();
    assert!(reply.error.unwrap().contains("not [A-Za-z0-9_-]"));
    assert_eq!(rig.phase(), Phase::Acquired);
    assert!(rig.episodes.lock().unwrap().is_empty());

    // This config records nothing, so there is no file of the arm's; the name is the episode's
    // id all the same, which is what a camera node following this arm records under.
    rig.activate_with(named("pick-0042"));
    let samples = rig.episodes.lock().unwrap().clone();
    assert_eq!(samples.len(), 1);
    assert_eq!(samples[0].recording_id, "pick-0042");
    assert!(samples[0].file.is_none());
    assert_eq!(
        rig.stats.snapshot("t").episode.as_deref(),
        Some("pick-0042")
    );
}

#[test]
fn a_session_that_never_started_publishes_no_episode() {
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.fake.start_fails.store(true, Ordering::SeqCst);
    assert_eq!(rig.err(Verb::Enable, CLIENT), "busy");
    // No `start` went out, so nothing may claim an end either.
    assert!(rig.episodes.lock().unwrap().is_empty());
    assert_eq!(rig.stats.snapshot("t").episode, None);
    assert_eq!(rig.phase(), Phase::Acquired);
}
