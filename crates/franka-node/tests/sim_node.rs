//! The node against franka-sim, in process: a session listening on the loopback, a client
//! session connecting to it, the lease, the commands, +5 cm of z at 20 Hz, the status and
//! episode topics, the stop, the watchdog, two refusals, the lost lease and a second client
//! while held; then a joints session and `home`; then the gripper through the Franka Hand
//! driver.
//!
//! Run with `FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-node \
//! --test sim_node -- --test-threads=1`.

#[path = "../examples/client/common.rs"]
mod client;

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use franka::{RealtimeConfig, Robot, RobotMode};
use franka_node::msg::robot_mode_code;
use franka_node::{
    transport, CmdReply, CmdRequest, FrankaHand, Gripper, GripperKind, Kind, NodeConfig, Phase,
    StateMsg, FLAG_HOLDING, FLAG_JOINTS, GRIPPER_CALIBRATED, GRIPPER_FAULT, GRIPPER_GRASPED,
    GRIPPER_MOVING, READY,
};
use franka_sim_test::{SimConfig, SimServer};
use serde_json::Value;
use zenoh::pubsub::Subscriber;
use zenoh::Wait;

const ARM: &str = "sim";
const NODE: &str = "test-node";
const NODE_ENDPOINT: &str = "tcp/127.0.0.1:17447";
const FIRST: u32 = 1;
const SECOND: u32 = 2;
const RATE: Duration = Duration::from_millis(50);

fn phase(state: &StateMsg) -> Phase {
    Phase::from_u8(state.phase).expect("phase byte")
}

fn is(expected: Phase) -> impl Fn(&StateMsg) -> bool {
    move |state| phase(state) == expected
}

/// `acquire` for `client`, retried while the node has not yet seen the client's token.
fn acquire(session: &zenoh::Session, client: u32) -> CmdReply {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let reply = client::cmd(session, ARM, "acquire", client).expect("acquire");
        if reply.error.as_deref() != Some("no lease") {
            return reply;
        }
        assert!(Instant::now() < deadline, "acquire kept saying no lease");
        std::thread::sleep(RATE);
    }
}

/// The latest `franka/node/<NODE>/status` sample, decoded.
fn watch_status(session: &zenoh::Session) -> (Arc<Mutex<Option<Value>>>, Subscriber<()>) {
    let latest = Arc::new(Mutex::new(None));
    let sink = Arc::clone(&latest);
    let subscriber = session
        .declare_subscriber(format!("franka/node/{NODE}/status"))
        .callback(move |sample: zenoh::sample::Sample| {
            // A payload that is not JSON shows up as the assertion, not as a swallowed panic.
            *sink.lock().unwrap() = serde_json::from_slice(&sample.payload().to_bytes()).ok();
        })
        .wait()
        .expect("status subscriber");
    (latest, subscriber)
}

/// Every `franka/<ARM>/episode` sample, in arrival order.
fn watch_episodes(session: &zenoh::Session) -> (Arc<Mutex<Vec<Value>>>, Subscriber<()>) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&seen);
    let subscriber = session
        .declare_subscriber(format!("franka/{ARM}/episode"))
        .callback(move |sample: zenoh::sample::Sample| {
            // A payload that is not JSON lands as a `null` the assertions name.
            let json = serde_json::from_slice(&sample.payload().to_bytes()).unwrap_or(Value::Null);
            sink.lock().unwrap().push(json);
        })
        .wait()
        .expect("episode subscriber");
    (seen, subscriber)
}

/// The episode samples once there are `n` of them, or a panic after 2 s.
fn episodes(seen: &Mutex<Vec<Value>>, n: usize) -> Vec<Value> {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let samples = seen.lock().unwrap().clone();
        if samples.len() >= n {
            return samples;
        }
        assert!(
            Instant::now() < deadline,
            "{} episode samples of {n} within 2 s: {samples:?}",
            samples.len()
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Where the `record` feature writes a test's episodes: a fresh directory per test and
/// process, removed when the guard drops.
struct RecordDir(PathBuf);

impl RecordDir {
    fn new(test: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("franka-node-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        RecordDir(dir)
    }

    /// The `.rrd` files with their sizes, sorted by name.
    fn rrd_files(&self) -> Vec<(String, u64)> {
        let mut files: Vec<(String, u64)> = std::fs::read_dir(&self.0)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .filter(|e| e.path().extension().is_some_and(|x| x == "rrd"))
                    .map(|e| {
                        let size = e.metadata().map_or(0, |m| m.len());
                        (e.file_name().to_string_lossy().into_owned(), size)
                    })
                    .collect()
            })
            .unwrap_or_default();
        files.sort();
        files
    }
}

impl Drop for RecordDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The node's config for the simulator: a short watchdog and a step limit with room for
/// rounding above the 5 cm the test commands; with the `record` feature, `record_dir`;
/// `extra` is appended to the arm table.
///
/// The lead limits are off here. The gate measures a lead against `O_T_EE`, and franka-sim's
/// `O_T_EE` is the joint-7 frame, 0.107 m from the pose the loop itself tracks (the model's
/// FK, see `target_control::torque`), so every target would be refused for a lead the arm does
/// not have. On a robot the two agree to about 0.5 mm. Turning this back on in the simulator
/// needs the reference to be the model's own FK, not a looser limit; with the limits off the
/// anchor flag is inert here too.
fn config(host: &str, record_dir: &RecordDir, extra: &str) -> NodeConfig {
    let record = if cfg!(feature = "record") {
        format!("record_dir = {:?}\n", record_dir.0.display().to_string())
    } else {
        String::new()
    };
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
hold_after_ms = 200
stop_after_ms = 1000
max_step = 0.06
max_lead = 0.0
max_lead_rotation = 0.0
{record}{extra}"#
    )
    .parse()
    .expect("test config")
}

#[test]
fn node_serves_one_arm_over_zenoh() {
    let sim = SimServer::start(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(Robot::new(sim.host(), RealtimeConfig::Ignore).expect("connect"));
    let record_dir = RecordDir::new("cartesian");
    let config = config(sim.host(), &record_dir, "");
    let node = transport::open(&config.zenoh).expect("node session");
    let arms = [ARM.to_string()];
    let attached =
        transport::attach(&node, config.arms[0].clone(), &arms, robot, None).expect("attach");
    let status = transport::status_publisher(
        &node,
        &config.name,
        vec![(ARM.to_string(), Arc::clone(attached.stats()))],
    )
    .expect("status publisher");

    let session = client::open(Some(NODE_ENDPOINT), None).expect("client session");
    let watch = client::StateWatch::subscribe(&session, ARM).expect("state subscriber");
    let (latest_status, _status_subscriber) = watch_status(&session);
    let (seen_episodes, _episode_subscriber) = watch_episodes(&session);
    let publisher = session
        .declare_publisher(format!("franka/{ARM}/target"))
        .wait()
        .expect("target publisher");
    let state = watch
        .wait_for(Duration::from_secs(5), |_| true)
        .expect("no state within 5 s");
    assert_eq!(phase(&state), Phase::Idle);
    assert_eq!(state.client_id.get(), 0);

    // Acquire once the token has reached the node.
    let token = session
        .liveliness()
        .declare_token(format!("franka/{ARM}/lease/{FIRST}"))
        .wait()
        .expect("lease token");
    assert_eq!(acquire(&session, FIRST), CmdReply::ok());
    let state = watch
        .wait_for(Duration::from_secs(1), is(Phase::Acquired))
        .expect("Acquired");
    assert_eq!(state.client_id.get(), FIRST);

    // A second client, alive, is refused while the first holds the arm.
    let _second_token = session
        .liveliness()
        .declare_token(format!("franka/{ARM}/lease/{SECOND}"))
        .wait()
        .expect("second token");
    let reply = acquire(&session, SECOND);
    assert!(!reply.ok);
    assert_eq!(reply.error.as_deref(), Some("held by 1"));

    // Enable, then +5 cm in z at 20 Hz for 2 s.
    let enabled_at = Instant::now();
    client::cmd_ok(&session, ARM, "enable", FIRST).expect("enable");
    let state = watch
        .wait_for(Duration::from_secs(2), is(Phase::Active))
        .expect("Active after enable");
    println!("enable took {:?}", enabled_at.elapsed());
    let start = state.target.map(|v| v.get());
    let z0 = state.o_t_ee[14].get();
    let mut raised = start;
    raised[2] += 0.05;
    let mut seq = 0;
    let began = Instant::now();
    while began.elapsed() < Duration::from_secs(2) {
        seq += 1;
        client::publish_target(&publisher, Kind::Cartesian, FIRST, seq, raised).expect("publish");
        std::thread::sleep(RATE);
    }
    let state = watch.latest().expect("state");
    let rtts = watch.take_rtts();
    println!(
        "after 2 s of targets: z {z0:.4} -> {:.4}, robot success rate {:.3}, seq {} of {seq}, \
         accepted {} refused {} dropped {}, rtt min {:?} max {:?} over {} echoes",
        state.o_t_ee[14].get(),
        state.success_rate.get(),
        state.seq_accepted.get(),
        state.accepted.get(),
        state.refused.get(),
        state.dropped.get(),
        rtts.iter().min().map(|ns| Duration::from_nanos(*ns)),
        rtts.iter().max().map(|ns| Duration::from_nanos(*ns)),
        rtts.len()
    );
    assert_eq!(phase(&state), Phase::Active);
    assert!(
        state.o_t_ee[14].get() - z0 >= 0.03,
        "z rose {:.4} m only",
        state.o_t_ee[14].get() - z0
    );
    // The robot's control command success rate: the loop answered its 1 kHz stream in time.
    assert!(
        state.success_rate.get() >= 0.9,
        "{}",
        state.success_rate.get()
    );
    assert!(state.accepted.get() >= 1);
    assert_eq!(state.refused.get(), 0);
    assert_eq!(state.flags.get(), 0);

    // The status topic, at 1 Hz: a sample from the last 2 s names the arm as active.
    let deadline = Instant::now() + Duration::from_secs(2);
    let status_sample = loop {
        let sample = latest_status.lock().unwrap().clone();
        match sample {
            Some(s) if s["arms"][0]["phase"] == "active" => break s,
            _ if Instant::now() >= deadline => panic!("no active status within 2 s: {sample:?}"),
            _ => std::thread::sleep(Duration::from_millis(10)),
        }
    };
    println!("status: {status_sample}");
    assert_eq!(status_sample["node"], NODE);
    assert_eq!(status_sample["version"], env!("CARGO_PKG_VERSION"));
    let arm = &status_sample["arms"][0];
    assert_eq!(arm["name"], ARM);
    assert_eq!(arm["holder"], FIRST);
    assert_eq!(arm["mode"], "cartesian");
    assert!(arm["accepted"].as_u64().unwrap() >= 1);
    assert_eq!(arm["refused"], 0);
    assert_eq!(arm["decode_failures"], 0);
    if cfg!(feature = "record") {
        let recording = arm["recording"].as_str().expect("recording file name");
        assert!(
            recording.starts_with(&format!("{ARM}-")) && recording.ends_with("Z.rrd"),
            "{recording}"
        );
    } else {
        assert_eq!(arm["recording"], Value::Null);
    }

    // Stop → Stopping → Idle, the holder kept.
    let stopped_at = Instant::now();
    client::cmd_ok(&session, ARM, "stop", FIRST).expect("stop");
    let state = watch
        .wait_for(Duration::from_secs(2), is(Phase::Idle))
        .expect("Idle after stop");
    println!("stop took {:?}", stopped_at.elapsed());
    assert_eq!(state.client_id.get(), FIRST);
    // With the `record` feature the session is one `.rrd`, closed by now: about two seconds
    // of 1 kHz records plus the setup, well over 10 kB.
    if cfg!(feature = "record") {
        let files = record_dir.rrd_files();
        println!("recorded: {files:?}");
        assert_eq!(files.len(), 1, "{files:?}");
        assert_eq!(files[0].0, arm["recording"]);
        assert!(files[0].1 > 10_000, "{} bytes", files[0].1);
    }

    // The session's start and end went out on the episode key, under one id.
    let samples = episodes(&seen_episodes, 2);
    println!("episodes: {samples:?}");
    assert_eq!(samples.len(), 2, "{samples:?}");
    assert_eq!(samples[0]["phase"], "start");
    assert_eq!(samples[1]["phase"], "end");
    assert_eq!(samples[0]["arm"], ARM);
    let id = samples[0]["recording_id"]
        .as_str()
        .expect("recording_id")
        .to_string();
    assert_eq!(samples[1]["recording_id"], id);
    let stamp = |s: &Value| s["t_node_ns"].as_u64().expect("t_node_ns");
    assert!(stamp(&samples[1]) >= stamp(&samples[0]), "{samples:?}");
    if cfg!(feature = "record") {
        // The id is the file's stem, which is what lets another process write beside it.
        assert_eq!(samples[0]["file"], format!("{id}.rrd"));
        assert_eq!(samples[0]["file"], arm["recording"]);
    } else {
        assert_eq!(samples[0]["file"], Value::Null);
    }

    // Enable again, ten targets, silence: the watchdog stops the loop, no reflex.
    client::cmd_ok(&session, ARM, "enable", FIRST).expect("second enable");
    watch
        .wait_for(Duration::from_secs(2), is(Phase::Active))
        .expect("Active after second enable");
    for _ in 0..10 {
        seq += 1;
        client::publish_target(&publisher, Kind::Cartesian, FIRST, seq, raised).expect("publish");
        std::thread::sleep(RATE);
    }
    let silent_at = Instant::now();
    let state = watch
        .wait_for(Duration::from_secs(1), |s| {
            s.flags.get() & FLAG_HOLDING != 0
        })
        .expect("holding flagged after hold_after");
    assert_eq!(phase(&state), Phase::Active);
    let state = watch
        .wait_for(Duration::from_secs(3), is(Phase::Idle))
        .expect("Idle after stop_after");
    assert_eq!(state.flags.get(), 0);
    println!(
        "watchdog stopped the loop {:?} after the last target",
        silent_at.elapsed()
    );
    assert!(silent_at.elapsed() >= Duration::from_millis(900));
    assert_ne!(state.robot_mode, robot_mode_code(RobotMode::Reflex));
    assert_eq!(state.has_errors, 0);
    assert_eq!(state.client_id.get(), FIRST);

    // A NaN and a metre are refused and counted; the arm stays active.
    client::cmd_ok(&session, ARM, "enable", FIRST).expect("third enable");
    let state = watch
        .wait_for(Duration::from_secs(2), is(Phase::Active))
        .expect("Active after third enable");
    let refused = state.refused.get();
    let mut nan = raised;
    nan[0] = f64::NAN;
    let mut far = raised;
    far[2] += 1.0;
    // At the commander's rate: two targets in one drain would coalesce into one judgement.
    seq += 1;
    client::publish_target(&publisher, Kind::Cartesian, FIRST, seq, nan).expect("publish");
    std::thread::sleep(RATE);
    seq += 1;
    client::publish_target(&publisher, Kind::Cartesian, FIRST, seq, far).expect("publish");
    let state = watch
        .wait_for(Duration::from_secs(1), |s| s.refused.get() >= refused + 2)
        .expect("two refusals counted");
    assert_eq!(state.refused.get(), refused + 2);
    assert_eq!(phase(&state), Phase::Active);

    // The lease goes away while active: the loop stops and the holder is cleared.
    let lost_at = Instant::now();
    token.undeclare().wait().expect("undeclare");
    let state = watch
        .wait_for(Duration::from_secs(3), |s| {
            phase(s) == Phase::Idle && s.client_id.get() == 0
        })
        .expect("Idle without a holder after the lease was lost");
    println!("lease loss stopped the loop in {:?}", lost_at.elapsed());
    assert_ne!(state.robot_mode, robot_mode_code(RobotMode::Reflex));
    assert_eq!(attached.decode_failures(), 0);

    status.shutdown();
    attached.shutdown();
    if cfg!(feature = "record") {
        // Two more sessions ran: the one the watchdog ended and the one the lease loss did.
        let files = record_dir.rrd_files();
        println!("recorded: {files:?}");
        assert_eq!(files.len(), 3, "{files:?}");
    }
    session.close().wait().expect("close client");
    node.close().wait().expect("close node");
    drop(sim);
}

fn max_abs_difference(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// `enable` in joints mode, then joint targets at 20 Hz until `q` is within `tolerance` of
/// the last of them or `seconds` pass; the state at the end.
fn joints_session(
    session: &zenoh::Session,
    watch: &client::StateWatch,
    publisher: &zenoh::pubsub::Publisher<'_>,
    seq: &mut u64,
    targets: &[[f64; 7]],
    seconds: f64,
) -> StateMsg {
    let enable = CmdRequest {
        mode: Some(Kind::Joints),
        ..CmdRequest::new(FIRST)
    };
    let reply = client::request(session, ARM, "enable", &enable).expect("enable joints");
    assert_eq!(reply, CmdReply::ok());
    let state = watch
        .wait_for(Duration::from_secs(2), is(Phase::Active))
        .expect("Active after enable in joints mode");
    assert_eq!(state.flags.get(), FLAG_JOINTS);
    let began = Instant::now();
    let mut state = state;
    // The step cap is 0.2 rad per target: walk through `targets` in order, one per tick, then
    // keep sending the last.
    let mut i = 0;
    while began.elapsed() < Duration::from_secs_f64(seconds) {
        *seq += 1;
        client::publish_target(publisher, Kind::Joints, FIRST, *seq, targets[i]).expect("publish");
        i = (i + 1).min(targets.len() - 1);
        std::thread::sleep(RATE);
        state = watch.latest().expect("state");
        if max_abs_difference(&state.q.map(|v| v.get()), targets.last().unwrap()) < 0.01 {
            break;
        }
    }
    state
}

#[test]
fn node_serves_joint_targets_and_home() {
    let sim = SimServer::start(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(Robot::new(sim.host(), RealtimeConfig::Ignore).expect("connect"));
    let record_dir = RecordDir::new("joints");
    let config = config(sim.host(), &record_dir, "");
    let node = transport::open(&config.zenoh).expect("node session");
    let arms = [ARM.to_string()];
    let attached =
        transport::attach(&node, config.arms[0].clone(), &arms, robot, None).expect("attach");

    let session = client::open(Some(NODE_ENDPOINT), None).expect("client session");
    let watch = client::StateWatch::subscribe(&session, ARM).expect("state subscriber");
    let publisher = session
        .declare_publisher(format!("franka/{ARM}/target"))
        .wait()
        .expect("target publisher");
    let state = watch
        .wait_for(Duration::from_secs(5), |_| true)
        .expect("no state within 5 s");
    assert_eq!(state.flags.get(), 0);
    let _token = session
        .liveliness()
        .declare_token(format!("franka/{ARM}/lease/{FIRST}"))
        .wait()
        .expect("lease token");
    assert_eq!(acquire(&session, FIRST), CmdReply::ok());
    watch
        .wait_for(Duration::from_secs(1), is(Phase::Acquired))
        .expect("Acquired");

    // Joints mode: +0.1 rad on joint 7 at 20 Hz for 1.5 s.
    let q0 = watch.latest().expect("state").q.map(|v| v.get());
    let mut raised = q0;
    raised[6] += 0.1;
    let mut seq = 0;
    let state = joints_session(&session, &watch, &publisher, &mut seq, &[raised], 1.5);
    let moved = state.q[6].get() - q0[6];
    println!(
        "joints: q7 moved {moved:.4} rad in 1.5 s, accepted {} refused {} success {:.3}",
        state.accepted.get(),
        state.refused.get(),
        state.success_rate.get()
    );
    assert_eq!(phase(&state), Phase::Active);
    assert_eq!(state.flags.get(), FLAG_JOINTS);
    assert_eq!(state.target.map(|v| v.get()), raised);
    assert!(moved >= 0.08, "joint 7 moved {moved:.4} rad only");
    assert_eq!(state.refused.get(), 0);

    // A Cartesian target in joints mode is refused and counted; the arm stays active.
    seq += 1;
    let pose = [0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0];
    client::publish_target(&publisher, Kind::Cartesian, FIRST, seq, pose).expect("publish");
    let state = watch
        .wait_for(Duration::from_secs(1), |s| s.refused.get() >= 1)
        .expect("the Cartesian target was refused");
    assert_eq!(phase(&state), Phase::Active);
    client::cmd_ok(&session, ARM, "stop", FIRST).expect("stop");
    let state = watch
        .wait_for(Duration::from_secs(2), is(Phase::Idle))
        .expect("Idle after stop");
    assert_eq!(state.flags.get(), 0);

    // Move further from the ready pose (franka-sim starts at [0, 0, 0, -1.57, 0, 1.57, 0.785],
    // itself off READY): joint 4 by 0.3 rad in two steps under the 0.2 rad cap, then stop
    // and re-acquire.
    let mut half = q0;
    half[3] += 0.15;
    let mut away = q0;
    away[3] += 0.3;
    assert_eq!(acquire(&session, FIRST), CmdReply::ok());
    let state = joints_session(&session, &watch, &publisher, &mut seq, &[half, away], 6.0);
    println!(
        "moved away: q4 {:.4} (target {:.4})",
        state.q[3].get(),
        away[3]
    );
    assert!((state.q[3].get() - away[3]).abs() < 0.03);
    client::cmd_ok(&session, ARM, "stop", FIRST).expect("stop");
    watch
        .wait_for(Duration::from_secs(2), is(Phase::Idle))
        .expect("Idle after stop");
    assert_eq!(acquire(&session, FIRST), CmdReply::ok());

    // Home: the reply comes when the arm has arrived; Homing is published meanwhile.
    let homed_at = Instant::now();
    let reply = std::thread::scope(|scope| {
        let home = scope.spawn(|| {
            let request = CmdRequest {
                speed: Some(0.3),
                ..CmdRequest::new(FIRST)
            };
            client::request(&session, ARM, "home", &request).expect("home")
        });
        let state = watch
            .wait_for(Duration::from_secs(3), is(Phase::Homing))
            .expect("Homing published");
        assert_eq!(state.flags.get(), FLAG_JOINTS);
        assert_eq!(state.target.map(|v| v.get()), READY);
        // A target during homing is refused.
        seq += 1;
        client::publish_target(&publisher, Kind::Joints, FIRST, seq, away).expect("publish");
        home.join().expect("home thread")
    });
    println!(
        "home took {:?} from {away:.3?}: {reply:?}",
        homed_at.elapsed()
    );
    assert_eq!(reply, CmdReply::ok());
    let state = watch
        .wait_for(Duration::from_secs(1), is(Phase::Acquired))
        .expect("Acquired after home");
    let q = state.q.map(|v| v.get());
    let error = max_abs_difference(&q, &READY);
    println!(
        "home: q within {error:.4} rad of READY, refused {}",
        state.refused.get()
    );
    assert!(error < 0.02, "q {q:?}");
    assert_eq!(state.client_id.get(), FIRST);
    assert_eq!(state.flags.get(), 0);
    // A home is a session: the counters started over, and count the target refused meanwhile.
    assert_eq!((state.accepted.get(), state.refused.get()), (0, 1));
    assert_ne!(state.robot_mode, robot_mode_code(RobotMode::Reflex));

    // Already there: a second home arrives on its first tick.
    let again_at = Instant::now();
    let reply = client::cmd(&session, ARM, "home", FIRST).expect("home again");
    println!("home from READY took {:?}", again_at.elapsed());
    assert_eq!(reply, CmdReply::ok());
    assert!(again_at.elapsed() < Duration::from_secs(3));
    let state = watch
        .wait_for(Duration::from_secs(1), is(Phase::Acquired))
        .expect("Acquired after the second home");
    assert_eq!(state.client_id.get(), FIRST);
    assert_eq!(state.refused.get(), 0);
    assert_eq!(attached.decode_failures(), 0);

    attached.shutdown();
    if cfg!(feature = "record") {
        // Two joints sessions and two homes.
        let files = record_dir.rrd_files();
        println!("recorded: {files:?}");
        assert_eq!(files.len(), 4, "{files:?}");
    }
    session.close().wait().expect("close client");
    node.close().wait().expect("close node");
    drop(sim);
}

/// The sim's kinematic gripper applies widths exactly; the tolerance is `sim_gripper.rs`'s.
const WIDTH_TOL: f64 = 0.005;

/// The gripper through the Franka Hand driver against franka-sim's gripper server: the state
/// stream, a non-holder refused, `gripper_home`, a width, a grasp in thin air (the sim
/// reports it unsuccessful), `gripper_stop` from a second client and the status entry.
#[test]
fn node_serves_a_gripper() {
    // `RUST_LOG=debug` shows the node's refusals with their reasons.
    let _ = env_logger::try_init();
    let sim = SimServer::start(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(Robot::new(sim.host(), RealtimeConfig::Ignore).expect("connect"));
    let record_dir = RecordDir::new("gripper");
    let config = config(sim.host(), &record_dir, "gripper = \"hand\"\n");
    let arm = &config.arms[0];
    assert_eq!(arm.gripper.as_deref(), Some("hand"));
    let hand = FrankaHand::connect(&arm.host, arm.gripper_speed).expect("gripper connect");
    println!("hand at connect: {:?}", hand.state());
    let node = transport::open(&config.zenoh).expect("node session");
    let arms = [ARM.to_string()];
    let attached =
        transport::attach(&node, arm.clone(), &arms, robot, Some(Box::new(hand))).expect("attach");
    let status = transport::status_publisher(
        &node,
        &config.name,
        vec![(ARM.to_string(), Arc::clone(attached.stats()))],
    )
    .expect("status publisher");

    let session = client::open(Some(NODE_ENDPOINT), None).expect("client session");
    let watch = client::StateWatch::subscribe(&session, ARM).expect("state subscriber");
    let gripper = client::GripperWatch::subscribe(&session, ARM).expect("gripper subscriber");
    let (latest_status, _status_subscriber) = watch_status(&session);
    let publisher = session
        .declare_publisher(format!("franka/{ARM}/gripper/target"))
        .wait()
        .expect("gripper target publisher");
    let first = gripper
        .wait_for(Duration::from_secs(5), |_| true)
        .expect("no gripper state within 5 s");
    println!("gripper state: {}", client::describe_gripper(&first));
    assert_eq!(first.client_id.get(), 0);
    assert_eq!(first.flags & (GRIPPER_FAULT | GRIPPER_MOVING), 0);
    // About 20 Hz: at least 8 distinct stamps in the next half second.
    let began = Instant::now();
    let mut stamps = std::collections::BTreeSet::new();
    while began.elapsed() < Duration::from_millis(500) {
        if let Some(s) = gripper.latest() {
            stamps.insert(s.t_node_ns.get());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    println!("{} gripper states in 0.5 s", stamps.len());
    assert!(stamps.len() >= 8, "{} states in 0.5 s", stamps.len());

    // Without a holder every gripper target is refused and counted.
    let refused = watch.latest().expect("state").refused.get();
    client::publish_gripper(&publisher, GripperKind::Width, SECOND, 1, 0.02, 0.0).expect("publish");
    let state = watch
        .wait_for(Duration::from_secs(1), |s| s.refused.get() > refused)
        .expect("the non-holder's gripper target was refused");
    assert_eq!(state.refused.get(), refused + 1);
    assert_eq!(phase(&state), Phase::Idle);

    // The holder: gripper_home, then a width, in Acquired (no arm session).
    let _token = session
        .liveliness()
        .declare_token(format!("franka/{ARM}/lease/{FIRST}"))
        .wait()
        .expect("lease token");
    assert_eq!(acquire(&session, FIRST), CmdReply::ok());
    assert!(
        !client::cmd(&session, ARM, "gripper_home", SECOND)
            .expect("gripper_home by the other")
            .ok
    );
    let homed_at = Instant::now();
    let reply = client::cmd(&session, ARM, "gripper_home", FIRST).expect("gripper_home");
    println!("gripper_home: {reply:?} after {:?}", homed_at.elapsed());
    assert_eq!(reply, CmdReply::ok());
    // The sim reports the stroke before any homing too; the holder shows once a state after
    // the acquire is out.
    let state = gripper
        .wait_for(Duration::from_secs(2), |s| {
            s.client_id.get() == FIRST
                && s.flags & GRIPPER_CALIBRATED != 0
                && (s.max_width.get() - 0.08).abs() < WIDTH_TOL
        })
        .expect("calibrated with max width ~0.08 after gripper_home");
    assert_eq!(state.flags & GRIPPER_FAULT, 0);

    let moved_at = Instant::now();
    client::publish_gripper(&publisher, GripperKind::Width, FIRST, 1, 0.04, 0.0).expect("publish");
    let state = gripper
        .wait_for(Duration::from_secs(5), |s| {
            (s.width.get() - 0.04).abs() < WIDTH_TOL && s.flags & GRIPPER_MOVING == 0
        })
        .expect("width ~0.04 within 5 s");
    println!(
        "width 0.04 reached after {:?}: {}",
        moved_at.elapsed(),
        client::describe_gripper(&state)
    );
    assert_eq!(state.commanded.get(), 0.04);
    assert_eq!(state.flags & GRIPPER_FAULT, 0);
    // The same seq again is refused; the arm's counter says so.
    let refused = watch.latest().expect("state").refused.get();
    client::publish_gripper(&publisher, GripperKind::Width, FIRST, 1, 0.05, 0.0).expect("publish");
    watch
        .wait_for(Duration::from_secs(1), |s| s.refused.get() > refused)
        .expect("a repeated seq was refused");

    // A grasp in thin air: the sim answers "unsuccessful", closes the fingers past the width
    // (to 0, nothing stops them) and reports no grasp.
    client::publish_gripper(&publisher, GripperKind::Grasp, FIRST, 2, 0.02, 10.0).expect("publish");
    let state = gripper
        .wait_for(Duration::from_secs(5), |s| {
            s.width.get() < 0.02 + WIDTH_TOL && s.flags & GRIPPER_MOVING == 0
        })
        .unwrap_or_else(|| {
            panic!(
                "closed to 0.02 or below after the grasp; last {:?}",
                gripper.latest().map(|s| client::describe_gripper(&s))
            )
        });
    println!("after the grasp: {}", client::describe_gripper(&state));
    assert_eq!(state.flags & GRIPPER_GRASPED, 0);
    assert_eq!(state.flags & GRIPPER_FAULT, 0);

    // Stop is anyone's; the arm's own stop is still refused outside a session. The sim's
    // stop re-opens the fingers to the full stroke (a hand halts in place), so only rest and
    // no fault are asserted, and the status is checked against whatever width results.
    assert_eq!(
        client::cmd(&session, ARM, "gripper_stop", SECOND).expect("gripper_stop"),
        CmdReply::ok()
    );
    assert!(!client::cmd(&session, ARM, "stop", FIRST).expect("stop").ok);
    std::thread::sleep(Duration::from_millis(300));
    let state = gripper
        .wait_for(Duration::from_secs(2), |s| s.flags & GRIPPER_MOVING == 0)
        .expect("at rest after gripper_stop");
    println!("after the stop: {}", client::describe_gripper(&state));
    assert_eq!(state.flags & GRIPPER_FAULT, 0);
    let width_now = state.width.get();

    // The status names the gripper; at 1 Hz, the next sample shows the width.
    let deadline = Instant::now() + Duration::from_secs(3);
    let status_sample = loop {
        let sample = latest_status.lock().unwrap().clone();
        let width = sample
            .as_ref()
            .and_then(|s| s["arms"][0]["gripper"]["width"].as_f64());
        match sample {
            Some(s) if width.is_some_and(|w| (w - width_now).abs() < WIDTH_TOL) => break s,
            _ if Instant::now() >= deadline => panic!(
                "no gripper at {width_now} in the status: {sample:?}; gripper {:?}",
                gripper.latest().map(|s| client::describe_gripper(&s))
            ),
            _ => std::thread::sleep(Duration::from_millis(10)),
        }
    };
    println!("status: {status_sample}");
    let entry = &status_sample["arms"][0]["gripper"];
    assert_eq!(entry["calibrated"], true);
    assert_eq!(entry["grasped"], false);
    assert_eq!(entry["fault"], false);
    assert_eq!(attached.decode_failures(), 0);
    // The arm never left Acquired: the gripper works without an arm session.
    assert_eq!(phase(&watch.latest().expect("state")), Phase::Acquired);

    status.shutdown();
    attached.shutdown();
    session.close().wait().expect("close client");
    node.close().wait().expect("close node");
    drop(sim);
}
