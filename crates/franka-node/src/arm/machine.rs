//! The state machine of one arm and the loop that runs it; the verbs are in `commands`.

use std::collections::BTreeSet;
use std::fmt::Display;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::Arc;
use std::time::{Duration, Instant};

use franka::robot::target_control::REST_JOINT_VELOCITY;
use franka::RobotState;
use log::{debug, info, warn};
use zerocopy::little_endian::{F64, U16, U32, U64};

use super::gripper::{GripperSide, GRIPPER_STATE_HZ};
use super::params::Params;
use super::record::Recording;
use super::state::pose_target;
use super::{
    Backlog, Event, Reply, RobotSide, HOME_SETTLE_TICKS, HOME_TIMEOUT, HOME_TOLERANCE, READY,
};
use crate::config::ArmConfig;
use crate::guard::{Guard, Verdict};
use crate::monotonic_ns;
use crate::msg::params::ParamsMsg;
use crate::msg::{
    robot_mode_code, CmdReply, EpisodeMsg, EpisodePhase, GripperKind, GripperMsg, Kind, Phase,
    StateMsg, TargetMsg, FLAG_HOLDING, FLAG_JOINTS, VERSION,
};
use crate::status::ArmStats;

/// How often an arm that is not active refreshes its snapshot with `read_once`.
const IDLE_READ_PERIOD: Duration = Duration::from_secs(1);

#[derive(Debug, PartialEq, Eq)]
pub(super) enum Flow {
    Continue,
    Stop,
}

#[derive(Default)]
pub(super) struct Counts {
    pub(super) accepted: u64,
    pub(super) refused: u64,
    pub(super) dropped: u64,
}

/// What the arm thread publishes: an encoded state every tick, an [`EpisodeMsg`] at a
/// session's start and end. The transport builds both around its publishers.
pub(super) struct Publishers {
    pub(super) state: Box<dyn Fn(&StateMsg) + Send>,
    pub(super) episode: Box<dyn Fn(&EpisodeMsg) + Send>,
    /// `params/current`, on every change and every
    /// [`CURRENT_PERIOD`](super::CURRENT_PERIOD).
    pub(super) params: Box<dyn Fn(&ParamsMsg) + Send>,
}

/// A `home` in progress: its query is answered when it ends.
pub(super) struct Homing {
    pub(super) reply: Reply,
    pub(super) started: Instant,
    /// Consecutive ticks with every joint within [`HOME_TOLERANCE`].
    pub(super) within: u32,
}

pub(super) struct Machine<R: RobotSide> {
    pub(super) config: ArmConfig,
    pub(super) robot: R,
    publish: Box<dyn Fn(&StateMsg) + Send>,
    publish_episode: Box<dyn Fn(&EpisodeMsg) + Send>,
    pub(super) publish_params_msg: Box<dyn Fn(&ParamsMsg) + Send>,
    pub(super) backlog: Arc<Backlog>,
    /// Stored into with every published state; read by the status publisher.
    pub(super) stats: Arc<ArmStats>,
    /// The episode recorder (inert without the `record` feature).
    pub(super) recording: Recording,
    /// The running session's `episode` sample; its `end` is published from it.
    episode: Option<EpisodeMsg>,
    pub(super) channel_dropped_base: u64,
    pub(super) phase: Phase,
    pub(super) holder: u32,
    pub(super) alive: BTreeSet<u32>,
    pub(super) control: Option<Box<dyn super::Control>>,
    /// `Some` while a session runs (Active, Homing, Stopping); its kind is the session's.
    /// Without one the published target is the snapshot's pose.
    pub(super) guard: Option<Guard>,
    pub(super) homing: Option<Homing>,
    pub(super) snapshot: RobotState,
    pub(super) read_at: Option<Instant>,
    /// When a target was last accepted, which `hold_after_ms` measures from.
    pub(super) last_accept: Instant,
    /// When a target last arrived from the session's own commander, accepted or refused on its
    /// content ([`crate::guard::Reason::from_the_commander`]), which `stop_after_ms` measures
    /// from: a commander whose targets are all refused is alive, and stopping its session would
    /// take the loop away from it for as long as the settle lasts.
    pub(super) last_target: Instant,
    pub(super) holding: bool,
    /// The last accepted target's `seq` and `t_send_ns`.
    pub(super) accepted: (u64, u64),
    pub(super) counts: Counts,
    /// The `params/*` surface's bookkeeping; the values themselves live in the session's slot.
    pub(super) params: Params,
    pub(super) gripper: Option<GripperSide>,
    /// State ticks since the gripper state was last published.
    gripper_ticks: u32,
}

impl<R: RobotSide> Machine<R> {
    pub(super) fn new(
        config: ArmConfig,
        arms: &[String],
        robot: R,
        publishers: Publishers,
        backlog: Arc<Backlog>,
        stats: Arc<ArmStats>,
        gripper: Option<GripperSide>,
    ) -> Self {
        Machine {
            recording: Recording::new(&config, arms, &robot),
            episode: None,
            config,
            robot,
            publish: publishers.state,
            publish_episode: publishers.episode,
            publish_params_msg: publishers.params,
            backlog,
            stats,
            gripper,
            gripper_ticks: 0,
            channel_dropped_base: 0,
            phase: Phase::Idle,
            holder: 0,
            alive: BTreeSet::new(),
            control: None,
            guard: None,
            homing: None,
            snapshot: RobotState::default(),
            read_at: None,
            last_accept: Instant::now(),
            last_target: Instant::now(),
            holding: false,
            accepted: (0, 0),
            counts: Counts::default(),
            params: Params::default(),
        }
    }

    pub(super) fn run(&mut self, rx: Receiver<Event>) {
        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.state_hz));
        let mut next_tick = Instant::now();
        loop {
            let now = Instant::now();
            if now >= next_tick {
                self.tick();
                next_tick = if next_tick + period > now {
                    next_tick + period
                } else {
                    now + period
                };
                continue;
            }
            let wake = self
                .watchdog_deadline()
                .map_or(next_tick, |deadline| deadline.min(next_tick));
            if wake <= now {
                self.watchdog();
                continue;
            }
            match rx.recv_timeout(wake - now) {
                Ok(event) => {
                    if self.batch(event, &rx) == Flow::Stop {
                        break;
                    }
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        let _ = self.end_session("shutdown", Err("shutting down"));
        for event in rx.try_iter() {
            match event {
                Event::Target(..) => self.backlog.release(),
                Event::Cmd(_, _, reply) => reply(CmdReply::err("shutting down")),
                Event::Params(verb, payload, reply) => self.params_query(verb, &payload, reply),
                _ => {}
            }
        }
        self.publish_state();
        info!("arm {}: thread finished", self.config.name);
    }

    /// Handles `first` and everything queued behind it. Commands and lease events run in
    /// arrival order before the newest target, so `LeaseLost` and `stop` always win; of the
    /// targets only the newest is judged, the rest are counted as dropped. Gripper widths
    /// coalesce the same way; a grasp is never superseded and flushes the width before it.
    pub(super) fn batch(&mut self, first: Event, rx: &Receiver<Event>) -> Flow {
        let mut newest = None;
        let mut newest_width: Option<GripperMsg> = None;
        let mut event = first;
        loop {
            match event {
                Event::Target(msg, received_ns) => {
                    self.backlog.release();
                    if newest.replace((msg, received_ns)).is_some() {
                        self.counts.dropped += 1;
                    }
                }
                Event::Gripper(msg) if msg.kind() == Some(GripperKind::Width) => {
                    if newest_width.replace(msg).is_some() {
                        self.counts.dropped += 1;
                    }
                }
                Event::Gripper(msg) => {
                    if let Some(width) = newest_width.take() {
                        self.gripper_target(&width);
                    }
                    self.gripper_target(&msg);
                }
                Event::Shutdown => return Flow::Stop,
                other => self.handle(other),
            }
            match rx.try_recv() {
                Ok(next) => event = next,
                Err(_) => break,
            }
        }
        if let Some(width) = newest_width {
            self.gripper_target(&width);
        }
        if let Some((msg, received_ns)) = newest {
            self.target(&msg, received_ns);
        }
        Flow::Continue
    }

    /// Handles one event that is not a target.
    pub(super) fn handle(&mut self, event: Event) {
        match event {
            Event::Target(msg, received_ns) => self.target(&msg, received_ns),
            Event::Cmd(verb, request, reply) => self.command(verb, request, reply),
            Event::Params(verb, payload, reply) => self.params_query(verb, &payload, reply),
            Event::Gripper(msg) => self.gripper_target(&msg),
            Event::LeaseAlive(client) => {
                self.alive.insert(client);
            }
            Event::LeaseLost(client) => self.lease_lost(client),
            Event::Shutdown => {}
        }
    }

    pub(super) fn target(&mut self, msg: &TargetMsg, received_ns: u64) {
        match self.phase {
            Phase::Active => {}
            Phase::Homing => return self.refuse(msg, "homing"),
            _ => return self.refuse(msg, "not active"),
        }
        // Where the arm is, for the gate's lead check: the measured pose of the latest state,
        // at most one state tick old. A joints session has no Cartesian reference.
        let measured = pose_target(&self.snapshot.O_T_EE);
        let Some((guard, control)) = self.guard.as_mut().zip(self.control.as_ref()) else {
            return self.fault("no control handle");
        };
        let reference = (guard.kind() == Kind::Cartesian).then_some(&measured);
        match guard.check(msg, received_ns, reference) {
            Verdict::Accept => match control.set_target(msg.data()) {
                Ok(()) => {
                    self.counts.accepted += 1;
                    self.accepted = (msg.seq.get(), msg.t_send_ns.get());
                    self.last_accept = Instant::now();
                    self.last_target = self.last_accept;
                    self.recording.accepted(msg);
                    if self.holding {
                        self.holding = false;
                        info!("arm {}: targets resumed", self.config.name);
                    }
                }
                Err(e) => self.fault(&e.to_string()),
            },
            Verdict::Refuse(reason) => {
                if reason.from_the_commander() {
                    self.last_target = Instant::now();
                }
                self.refuse(msg, reason);
            }
        }
    }

    fn refuse(&mut self, msg: &TargetMsg, reason: impl Display) {
        self.counts.refused += 1;
        self.recording.refused(msg, &reason);
        debug!(
            "arm {}: target seq {} from {} refused: {reason}",
            self.config.name,
            msg.seq.get(),
            msg.client_id.get()
        );
    }

    /// A gripper command in any phase, from the holder; a refusal counts as `refused`.
    fn gripper_target(&mut self, msg: &GripperMsg) {
        let holder = self.holder;
        let outcome = match self.gripper.as_mut() {
            Some(gripper) => gripper.target(msg, holder),
            None => Err("no gripper".to_string()),
        };
        if let Err(reason) = outcome {
            self.counts.refused += 1;
            debug!(
                "arm {}: gripper seq {} from {} refused: {reason}",
                self.config.name,
                msg.seq.get(),
                msg.client_id.get()
            );
        }
    }

    pub(super) fn transition(&mut self, to: Phase, why: &str) {
        if self.phase != to {
            info!("arm {}: {} -> {to} ({why})", self.config.name, self.phase);
        }
        self.phase = to;
    }

    /// One state tick: the watchdog while Active, the arrival check while Homing, a snapshot
    /// refresh otherwise, a publish; every `state_hz / 20`th tick the gripper's state too.
    pub(super) fn tick(&mut self) {
        match self.phase {
            Phase::Active => self.tick_active(),
            Phase::Homing => self.tick_homing(),
            _ if self.read_at.is_none_or(|t| t.elapsed() >= IDLE_READ_PERIOD) => {
                // `read_once` blocks up to the library's 1 s UDP timeout, hence the period.
                match self.robot.read_once() {
                    Ok(state) => self.snapshot = state,
                    Err(e) => warn!("arm {}: read_once: {e}", self.config.name),
                }
                self.read_at = Some(Instant::now());
            }
            _ => {}
        }
        self.publish_state();
        self.publish_params_if_due();
        self.gripper_ticks += 1;
        if self.gripper_ticks >= (self.config.state_hz / GRIPPER_STATE_HZ).max(1) {
            self.gripper_ticks = 0;
            if let Some(gripper) = &self.gripper {
                let state = gripper.publish(self.holder, &self.stats);
                // Into the session's recording too, so a replay has the hand beside the arm.
                self.recording.gripper(&state);
            }
        }
    }

    /// The loop's latest state into the snapshot, or a fault if the loop is gone.
    fn refresh_from_control(&mut self) -> bool {
        match self.control.as_ref().filter(|c| c.is_running()) {
            Some(control) => {
                self.snapshot = control.state();
                true
            }
            None => {
                self.fault("the control loop ended");
                false
            }
        }
    }

    fn tick_active(&mut self) {
        if self.refresh_from_control() {
            self.watchdog();
        }
    }

    fn tick_homing(&mut self) {
        if !self.refresh_from_control() {
            return;
        }
        let s = &self.snapshot;
        let within = (0..7).all(|i| (s.q[i] - READY[i]).abs() <= HOME_TOLERANCE);
        let at_rest = s.dq.iter().all(|v| v.abs() < REST_JOINT_VELOCITY);
        let Some(homing) = self.homing.as_mut() else {
            // Unreachable: the query is stored with the transition. Stop rather than run on.
            let _ = self.finish_home("home without a query", Err("no query"));
            return;
        };
        homing.within = if within { homing.within + 1 } else { 0 };
        let arrived = within && (at_rest || homing.within >= HOME_SETTLE_TICKS);
        let timed_out = homing.started.elapsed() >= HOME_TIMEOUT;
        if arrived {
            let _ = self.finish_home("homed", Ok(()));
        } else if timed_out {
            let _ = self.finish_home("home timed out", Err("home timed out"));
        }
    }

    /// When the watchdog next acts, while Active: `hold_after` since the last *accepted*
    /// target, until holding, and `stop_after` since the last one that *arrived*.
    fn watchdog_deadline(&self) -> Option<Instant> {
        if self.phase != Phase::Active {
            return None;
        }
        let stop = self.last_target + Duration::from_millis(self.config.stop_after_ms);
        let hold = self.last_accept + Duration::from_millis(self.config.hold_after_ms);
        Some(if self.holding { stop } else { hold.min(stop) })
    }

    fn watchdog(&mut self) {
        if self.last_target.elapsed() >= Duration::from_millis(self.config.stop_after_ms) {
            let _ = self.stop("no target within stop_after");
        } else if !self.holding
            && self.last_accept.elapsed() >= Duration::from_millis(self.config.hold_after_ms)
        {
            self.holding = true;
            debug!(
                "arm {}: holding, no target accepted within hold_after",
                self.config.name
            );
        }
    }

    pub(super) fn publish_state(&self) {
        let state = self.encode();
        self.stats
            .record(&state, self.guard.as_ref().map(Guard::kind));
        (self.publish)(&state);
    }

    /// A session started: publishes its `start` sample under the recording's id -- the
    /// collector's `episode` token when it named one, else the file's stem, else an id
    /// generated in the same shape because no file is open.
    pub(super) fn begin_episode(&mut self, episode: Option<&str>) {
        let arm = self.config.name.clone();
        let recording_id = self
            .recording
            .recording_id()
            .or_else(|| episode.map(str::to_string))
            .unwrap_or_else(|| super::record::session_id(&arm));
        let msg = EpisodeMsg {
            arm,
            recording_id,
            file: self.recording.file_name(),
            t_node_ns: monotonic_ns(),
            phase: EpisodePhase::Start,
        };
        self.stats.set_episode(Some(msg.recording_id.clone()));
        (self.publish_episode)(&msg);
        self.episode = Some(msg);
    }

    /// The session is over: its `end` sample, if a `start` went out.
    pub(super) fn end_episode(&mut self) {
        if let Some(mut msg) = self.episode.take() {
            msg.t_node_ns = monotonic_ns();
            msg.phase = EpisodePhase::End;
            (self.publish_episode)(&msg);
        }
    }

    /// Whether the running session is a joints one (`enable` in joints mode, or `home`).
    pub(super) fn joints_session(&self) -> bool {
        self.guard
            .as_ref()
            .is_some_and(|guard| guard.kind() == Kind::Joints)
    }

    pub(super) fn encode(&self) -> StateMsg {
        let s = &self.snapshot;
        let target = match &self.guard {
            Some(guard) => guard.previous(),
            None => pose_target(&s.O_T_EE),
        };
        let joints = self.joints_session();
        let mut flags = 0;
        if self.holding {
            flags |= FLAG_HOLDING;
        }
        if joints {
            flags |= FLAG_JOINTS;
        }
        let channel_dropped = self
            .backlog
            .dropped()
            .saturating_sub(self.channel_dropped_base);
        StateMsg {
            version: VERSION,
            phase: self.phase as u8,
            robot_mode: robot_mode_code(s.robot_mode),
            has_errors: u8::from(s.current_errors.any()),
            flags: U16::new(flags),
            client_id: U32::new(self.holder),
            seq_accepted: U64::new(self.accepted.0),
            t_send_ns_accepted: U64::new(self.accepted.1),
            t_node_ns: U64::new(monotonic_ns()),
            robot_time_ms: U64::new(s.time.as_millis()),
            success_rate: F64::new(s.control_command_success_rate),
            q: s.q.map(F64::new),
            dq: s.dq.map(F64::new),
            tau_ext: s.tau_ext_hat_filtered.map(F64::new),
            o_t_ee: s.O_T_EE.map(F64::new),
            o_f_ext_k: s.O_F_ext_hat_K.map(F64::new),
            target: target.map(F64::new),
            accepted: U64::new(self.counts.accepted),
            refused: U64::new(self.counts.refused),
            dropped: U64::new(self.counts.dropped + channel_dropped),
        }
    }
}
