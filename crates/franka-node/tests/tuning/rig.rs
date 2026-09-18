//! What has to be standing up: the simulator, the robot with the tap, the node in process, and
//! the two client sessions a panel and a teleop client would each have of their own.

// Shared by two test binaries: the asserting `sim_tuning` suite and the `sim_sweep` campaign
// script, which each use a different part of this module. Rust's dead-code analysis is per
// binary and has no view of the other one, so the parts one of them does not reach are not
// dead -- they are the other's. It is a blanket allow: something that went dead in *both*
// would be silent here, so a reader deleting from this module should check both binaries.
#![allow(dead_code)]

use std::sync::Arc;
use std::time::{Duration, Instant};

use franka::model::Model;
use franka::robot::target_control::LiveTuning;
use franka::robot::target_control::{Backend, ImpedanceOptions};
use franka::{RealtimeConfig, Robot, RobotState};
use franka_node::config::ArmConfig;
use franka_node::{transport, Attached, Kind, NodeConfig, Phase, StateMsg};
use franka_sim_test::{SimConfig, SimServer};
use zenoh::Wait;

use super::tap::Trace;
use crate::{client, ARM, CLIENT, NODE, NODE_ENDPOINT};

/// The node's config for the simulator.
///
/// The lead limits are off. The node's guard measures a lead against `O_T_EE`, and franka-sim's
/// `O_T_EE` is the joint-7 frame, 0.107 m from the pose the loop itself tracks, so every target
/// would be refused for a lead the arm does not have. On a robot the two agree to about 0.5 mm.
/// That is also why nothing here can test the lead guard.
pub fn config(host: &str, max_step: f64, max_deviation: f64) -> NodeConfig {
    format!(
        r#"
name = "{NODE}"

[zenoh]
listen = ["{NODE_ENDPOINT}"]
multicast_scouting = false

[[arm]]
name = "{ARM}"
host = "{host}"
realtime = "ignore"
hold_after_ms = 400
stop_after_ms = 4000
max_step = {max_step}
max_deviation = {max_deviation}
max_lead = 0.0
max_lead_rotation = 0.0
"#
    )
    .parse()
    .expect("test config")
}

pub struct Rig {
    /// Taken by [`Drop`], which stops and joins the 1 kHz thread *before* the container goes.
    attached: Option<Attached>,
    pub trace: Trace,
    pub model: Model,
    /// A state read before the session started: the frames and inertias the model needs, which
    /// do not change while a session runs. [`super::law::expected`] patches `q` and `dq` into it.
    pub template: RobotState,
    /// The impedance options the node's own `ArmConfig` built, so the oracle and the loop cannot
    /// disagree about anything but the gains.
    pub base_impedance: ImpedanceOptions,
    /// What a session of this config starts tunable at: the node's own reading of its TOML.
    pub defaults: LiveTuning,
    pub session: zenoh::Session,
    pub panel: Option<zenoh::Session>,
    pub watch: client::StateWatch,
    start: Instant,
    robot: Arc<Robot>,
    _node: zenoh::Session,
    /// Declared last so that even without the `Drop` below the container outlives the loop.
    _sim: SimServer,
}

impl Rig {
    /// `cycles` sizes the tap once, for the length of run the caller intends.
    pub fn new(max_step: f64, max_deviation: f64, cycles: usize) -> Rig {
        let sim = SimServer::start(SimConfig::nominal().with_motion_limits());
        let robot = Arc::new(Robot::new(sim.host(), RealtimeConfig::Ignore).expect("connect"));
        let template = robot.read_once().expect("read_once");
        let model = robot.load_model().expect("model");
        let start = Instant::now();
        let (tap, trace) = Trace::tap(Arc::clone(&robot), start, cycles);

        let node_config = config(sim.host(), max_step, max_deviation);
        let arm_config: ArmConfig = node_config.arms[0].clone();
        let defaults = arm_config.live_tuning();
        let base_impedance = match arm_config.target_control_options().backend {
            Backend::Impedance(options) => options,
            other => panic!("the node's Cartesian session is not the impedance backend: {other:?}"),
        };
        let node = transport::open(&node_config.zenoh).expect("node session");
        let arms = [ARM.to_string()];
        let attached = transport::attach(&node, arm_config, &arms, tap, None).expect("attach");

        let session = client::open(Some(NODE_ENDPOINT), None).expect("client session");
        let panel = client::open(Some(NODE_ENDPOINT), None).expect("panel session");
        let watch = client::StateWatch::subscribe(&session, ARM).expect("state subscriber");
        Rig {
            attached: Some(attached),
            trace,
            model,
            template,
            base_impedance,
            defaults,
            session,
            panel: Some(panel),
            watch,
            start,
            robot,
            _node: node,
            _sim: sim,
        }
    }

    /// A rig for a wire-only test: no long motion, so a small tap.
    pub fn wire() -> Rig {
        Rig::new(0.20, 0.30, 40_000)
    }

    pub fn now_ns(&self) -> u64 {
        self.start.elapsed().as_nanos() as u64
    }

    /// The panel's session; `None` only after [`kill_panel`](Self::kill_panel).
    pub fn panel(&self) -> &zenoh::Session {
        self.panel.as_ref().expect("the panel session is closed")
    }

    /// Closes the panel's session outright, as a panel process dying would.
    pub fn kill_panel(&mut self) {
        let panel = self.panel.take().expect("the panel session is closed");
        panel.close().wait().expect("close the panel session");
    }

    pub fn torque_limits(&self) -> [f64; 7] {
        self.base_impedance.torque_limits
    }

    /// The FCI version the simulator negotiated, which is what the joint velocity limits are of.
    pub fn fci_version(&self) -> franka::FciVersion {
        self.robot.fci_version()
    }

    /// Lease, enable, and the target the session started holding.
    pub fn engage(&self) -> (zenoh::liveliness::LivelinessToken, [f64; 7]) {
        self.watch
            .wait_for(Duration::from_secs(5), |_| true)
            .expect("no state within 5 s");
        let token = client::lease(&self.session, ARM, CLIENT).expect("lease");
        client::cmd_ok(&self.session, ARM, "enable", CLIENT).expect("enable");
        let state = self
            .watch
            .wait_for(Duration::from_secs(5), |s| {
                Phase::from_u8(s.phase) == Some(Phase::Active)
            })
            .expect("Active after enable");
        (token, state.target.map(|v| v.get()))
    }

    pub fn state(&self) -> StateMsg {
        self.watch.latest().expect("state")
    }

    /// Asserts the arm is still moving under its own session and has not reflexed or stopped.
    pub fn assert_healthy(&self) {
        let s = self.state();
        assert_eq!(
            Phase::from_u8(s.phase),
            Some(Phase::Active),
            "the session is no longer Active"
        );
        assert_eq!(
            s.robot_mode,
            franka_node::msg::robot_mode_code(franka::RobotMode::Move),
            "the robot left Move (reflex or user stop)"
        );
        assert_eq!(s.has_errors, 0, "the robot reports errors");
    }

    /// A publisher on the arm's target key.
    pub fn targets(&self) -> zenoh::pubsub::Publisher<'static> {
        self.session
            .declare_publisher(format!("franka/{ARM}/target"))
            .wait()
            .expect("target publisher")
    }
}

impl Drop for Rig {
    fn drop(&mut self) {
        // Before the container is removed: undeclare the entities, stop the arm, join the
        // 1 kHz thread. Otherwise the loop is joined against a peer that is already gone.
        if let Some(attached) = self.attached.take() {
            attached.shutdown();
        }
        self.trace.finish();
    }
}

/// Publishes one target repeatedly at 50 Hz -- the one publish loop, so there are not four.
pub struct Sender {
    publisher: zenoh::pubsub::Publisher<'static>,
    seq: u64,
}

impl Sender {
    pub fn new(rig: &Rig) -> Sender {
        Sender {
            publisher: rig.targets(),
            seq: 0,
        }
    }

    /// `target` for `hold`, at 50 Hz.
    pub fn hold(&mut self, target: [f64; 7], hold: Duration) {
        let until = Instant::now() + hold;
        while Instant::now() < until {
            self.seq += 1;
            client::publish_target(&self.publisher, Kind::Cartesian, CLIENT, self.seq, target)
                .expect("publish target");
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

/// A background thread tracking a z sinusoid, so the arm has a persistent, nonzero joint error
/// for the law to act on. Stops when dropped.
pub struct Driver {
    stop: Arc<std::sync::atomic::AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Driver {
    pub fn start(rig: &Rig, home: [f64; 7], amplitude: f64, hz: f64) -> Driver {
        let publisher = rig.targets();
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let began = Instant::now();
            let mut seq = 0u64;
            while !flag.load(std::sync::atomic::Ordering::Relaxed) {
                seq += 1;
                let t = began.elapsed().as_secs_f64();
                let mut target = home;
                target[2] = home[2] + amplitude * (2.0 * std::f64::consts::PI * hz * t).sin();
                let _ = client::publish_target(&publisher, Kind::Cartesian, CLIENT, seq, target);
                std::thread::sleep(Duration::from_millis(20));
            }
        });
        Driver {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Driver {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}
