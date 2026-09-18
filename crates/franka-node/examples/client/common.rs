//! A commander's side of the node's keys, shared by the `client` example and `tests/sim_node.rs`.

#![allow(dead_code)]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use franka_node::{
    monotonic_ns, transport, CmdReply, CmdRequest, GripperKind, GripperMsg, GripperStateMsg, Kind,
    Phase, StateMsg, TargetMsg, ZenohConfig, FLAG_HOLDING, FLAG_JOINTS, GRIPPER_CALIBRATED,
    GRIPPER_FAULT, GRIPPER_GRASPED, GRIPPER_MOVING,
};
use zenoh::liveliness::LivelinessToken;
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::{Session, Wait};
use zerocopy::little_endian::F64;
use zerocopy::{FromBytes, IntoBytes};

/// How long a command waits for the node's reply; `home` answers when the arm has arrived
/// (the node gives up after 60 s), `gripper_home` when the gripper is homed (30 s).
pub const CMD_TIMEOUT: Duration = Duration::from_secs(3);
pub const HOME_TIMEOUT: Duration = Duration::from_secs(90);

/// A peer session that connects to `connect` if given (and then does not scout), listens on
/// `listen` if given.
pub fn open(connect: Option<&str>, listen: Option<&str>) -> zenoh::Result<Session> {
    let config = ZenohConfig {
        listen: listen.map(String::from).into_iter().collect(),
        connect: connect.map(String::from).into_iter().collect(),
        ..ZenohConfig::default()
    };
    transport::open(&config)
}

/// One `cmd/<verb>` round trip of `request`; the error text is the node's refusal or the
/// transport's.
pub fn request(
    session: &Session,
    arm: &str,
    verb: &str,
    request: &CmdRequest,
) -> Result<CmdReply, String> {
    let payload = serde_json::to_string(request).map_err(|e| e.to_string())?;
    let timeout = if verb.ends_with("home") {
        HOME_TIMEOUT
    } else {
        CMD_TIMEOUT
    };
    let replies = session
        .get(format!("franka/{arm}/cmd/{verb}"))
        .payload(payload)
        .timeout(timeout)
        .wait()
        .map_err(|e| format!("{verb}: {e}"))?;
    let reply = replies
        .recv()
        .map_err(|_| format!("{verb}: no reply from the node"))?;
    let sample = reply.result().map_err(|e| format!("{verb}: {e}"))?;
    serde_json::from_slice(&sample.payload().to_bytes()).map_err(|e| format!("{verb}: {e}"))
}

/// [`request`] with `{"client_id": N}` only.
pub fn cmd(session: &Session, arm: &str, verb: &str, client_id: u32) -> Result<CmdReply, String> {
    request(session, arm, verb, &CmdRequest::new(client_id))
}

/// [`cmd`] that must come back `ok`.
pub fn cmd_ok(session: &Session, arm: &str, verb: &str, client_id: u32) -> Result<(), String> {
    match cmd(session, arm, verb, client_id)? {
        CmdReply { ok: true, .. } => Ok(()),
        CmdReply { error, .. } => Err(format!("{verb} refused: {}", error.unwrap_or_default())),
    }
}

/// Declares the lease token and acquires the arm, retrying while the token is in flight.
pub fn lease(session: &Session, arm: &str, client_id: u32) -> Result<LivelinessToken, String> {
    let token = session
        .liveliness()
        .declare_token(format!("franka/{arm}/lease/{client_id}"))
        .wait()
        .map_err(|e| e.to_string())?;
    println!("client {client_id}: lease franka/{arm}/lease/{client_id} declared");
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        match cmd(session, arm, "acquire", client_id)? {
            reply if reply.ok => break,
            reply if reply.error.as_deref() == Some("no lease") && Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(50));
            }
            reply => {
                return Err(format!(
                    "acquire refused: {}",
                    reply.error.unwrap_or_default()
                ))
            }
        }
    }
    println!("acquired");
    Ok(token)
}

/// `ok`, or the node's refusal.
pub fn verdict(reply: &CmdReply) -> String {
    if reply.ok {
        "ok".to_string()
    } else {
        reply.error.clone().unwrap_or_default()
    }
}

/// Whether `target` holds joint positions.
pub fn joints(state: &StateMsg) -> bool {
    state.flags.get() & FLAG_JOINTS != 0
}

/// One line of what the node reports: `success` is the robot's control command success rate,
/// `acc`/`ref`/`drop` the node's counters since enable, `target` its z or, in a joints
/// session, joint 7.
pub fn describe(state: &StateMsg) -> String {
    let pose = &state.o_t_ee;
    format!(
        "phase {}{} holder {} mode {} errors {} target {} {:.4} xyz {:.4} {:.4} {:.4} \
         success {:.3} acc {} ref {} drop {}",
        Phase::from_u8(state.phase).map_or("?".to_string(), |p| p.to_string()),
        if state.flags.get() & FLAG_HOLDING != 0 {
            " (holding)"
        } else {
            ""
        },
        state.client_id.get(),
        state.robot_mode,
        state.has_errors,
        if joints(state) { "q7" } else { "z" },
        state.target[if joints(state) { 6 } else { 2 }].get(),
        pose[12].get(),
        pose[13].get(),
        pose[14].get(),
        state.success_rate.get(),
        state.accepted.get(),
        state.refused.get(),
        state.dropped.get()
    )
}

/// Per-second extremes over every state received, not only the one printed.
#[derive(Default, Clone, Copy)]
pub struct Peaks {
    /// Largest `|O_F_ext_hat_K|` force, N.
    pub force: f64,
    /// Largest distance between the measured and the target position, m.
    pub deviation: f64,
}

/// The latest `StateMsg`, the round trips of newly accepted targets and the peaks.
pub struct StateWatch {
    latest: Arc<Mutex<Option<StateMsg>>>,
    rtt_ns: Arc<Mutex<Vec<u64>>>,
    peaks: Arc<Mutex<Peaks>>,
    _subscriber: Subscriber<()>,
}

impl StateWatch {
    pub fn subscribe(session: &Session, arm: &str) -> zenoh::Result<Self> {
        let latest = Arc::new(Mutex::new(None::<StateMsg>));
        let rtt_ns = Arc::new(Mutex::new(Vec::new()));
        let peaks = Arc::new(Mutex::new(Peaks::default()));
        let (sink, rtts, extremes) = (Arc::clone(&latest), Arc::clone(&rtt_ns), Arc::clone(&peaks));
        let subscriber = session
            .declare_subscriber(format!("franka/{arm}/state"))
            .callback(move |sample| {
                let Ok(state) = StateMsg::read_from_bytes(&sample.payload().to_bytes()) else {
                    return;
                };
                let mut latest = sink.lock().unwrap_or_else(|p| p.into_inner());
                let previous_seq = latest.map_or(0, |s| s.seq_accepted.get());
                if state.seq_accepted.get() != previous_seq && state.t_send_ns_accepted.get() != 0 {
                    let rtt = monotonic_ns().saturating_sub(state.t_send_ns_accepted.get());
                    rtts.lock().unwrap_or_else(|p| p.into_inner()).push(rtt);
                }
                let force = norm3(&state.o_f_ext_k[..3]);
                // In a joints session the target is not a position; the deviation stays 0.
                let deviation = if joints(&state) {
                    0.0
                } else {
                    norm3(&[
                        F64::new(state.o_t_ee[12].get() - state.target[0].get()),
                        F64::new(state.o_t_ee[13].get() - state.target[1].get()),
                        F64::new(state.o_t_ee[14].get() - state.target[2].get()),
                    ])
                };
                let mut peaks = extremes.lock().unwrap_or_else(|p| p.into_inner());
                peaks.force = peaks.force.max(force);
                peaks.deviation = peaks.deviation.max(deviation);
                *latest = Some(state);
            })
            .wait()?;
        Ok(StateWatch {
            latest,
            rtt_ns,
            peaks,
            _subscriber: subscriber,
        })
    }

    /// The peaks since the last call.
    pub fn take_peaks(&self) -> Peaks {
        std::mem::take(&mut *self.peaks.lock().unwrap_or_else(|p| p.into_inner()))
    }

    pub fn latest(&self) -> Option<StateMsg> {
        *self.latest.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Polls every 10 ms until a state satisfies `accept`, or `timeout` passes.
    pub fn wait_for(
        &self,
        timeout: Duration,
        accept: impl Fn(&StateMsg) -> bool,
    ) -> Option<StateMsg> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(state) = self.latest().filter(&accept) {
                return Some(state);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// The round trips, ns, recorded since the last call.
    pub fn take_rtts(&self) -> Vec<u64> {
        std::mem::take(&mut *self.rtt_ns.lock().unwrap_or_else(|p| p.into_inner()))
    }
}

/// Publishes one target of `kind` stamped with [`monotonic_ns`].
pub fn publish_target(
    publisher: &Publisher<'_>,
    kind: Kind,
    client_id: u32,
    seq: u64,
    data: [f64; 7],
) -> zenoh::Result<()> {
    let msg = TargetMsg::new(kind, client_id, seq, monotonic_ns(), data);
    publisher.put(msg.as_bytes()).wait()
}

fn norm3(v: &[F64]) -> f64 {
    v.iter().map(|x| x.get() * x.get()).sum::<f64>().sqrt()
}

/// Publishes one gripper command stamped with [`monotonic_ns`].
pub fn publish_gripper(
    publisher: &Publisher<'_>,
    kind: GripperKind,
    client_id: u32,
    seq: u64,
    width: f64,
    force: f64,
) -> zenoh::Result<()> {
    let msg = GripperMsg::new(kind, client_id, seq, monotonic_ns(), width, force);
    publisher.put(msg.as_bytes()).wait()
}

/// One line of the gripper's state.
pub fn describe_gripper(state: &GripperStateMsg) -> String {
    let flag = |bit: u8, name: &str| {
        if state.flags & bit != 0 {
            format!(" {name}")
        } else {
            String::new()
        }
    };
    format!(
        "width {:.4} commanded {:.4} max {:.4} holder {}{}{}{}{}",
        state.width.get(),
        state.commanded.get(),
        state.max_width.get(),
        state.client_id.get(),
        flag(GRIPPER_CALIBRATED, "calibrated"),
        flag(GRIPPER_GRASPED, "grasped"),
        flag(GRIPPER_MOVING, "moving"),
        flag(GRIPPER_FAULT, "fault"),
    )
}

/// The latest `GripperStateMsg` of `franka/<arm>/gripper/state`.
pub struct GripperWatch {
    latest: Arc<Mutex<Option<GripperStateMsg>>>,
    _subscriber: Subscriber<()>,
}

impl GripperWatch {
    pub fn subscribe(session: &Session, arm: &str) -> zenoh::Result<Self> {
        let latest = Arc::new(Mutex::new(None));
        let sink = Arc::clone(&latest);
        let subscriber = session
            .declare_subscriber(format!("franka/{arm}/gripper/state"))
            .callback(move |sample| {
                if let Ok(state) = GripperStateMsg::try_from(sample.payload().to_bytes().as_ref()) {
                    *sink.lock().unwrap_or_else(|p| p.into_inner()) = Some(state);
                }
            })
            .wait()?;
        Ok(GripperWatch {
            latest,
            _subscriber: subscriber,
        })
    }

    pub fn latest(&self) -> Option<GripperStateMsg> {
        *self.latest.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Polls every 10 ms until a state satisfies `accept`, or `timeout` passes.
    pub fn wait_for(
        &self,
        timeout: Duration,
        accept: impl Fn(&GripperStateMsg) -> bool,
    ) -> Option<GripperStateMsg> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(state) = self.latest().filter(&accept) {
                return Some(state);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}
