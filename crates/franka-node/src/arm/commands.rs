//! The verbs of one arm, and how a control session (Cartesian, joints or `home`) starts
//! and ends.

use std::time::Instant;

use franka::robot::target_control::joint_position_limits;
use franka::{FrankaError, JointTargetControlOptions};
use log::{debug, info, warn};

use super::machine::{Counts, Homing, Machine};
use super::{
    Control, Reply, RobotSide, Verb, HOME_DEVIATION_MARGIN, HOME_SPEED, HOME_SPEED_RANGE, READY,
};
use crate::guard::{Guard, Reason};
use crate::monotonic_ns;
use crate::msg::{CmdReply, CmdRequest, Kind, Phase};

/// What a session runs: the Cartesian loop, or a joints loop budgeting `fraction` of the
/// limits, travelling to `target` (`home`) or holding where the arm is.
enum Session {
    Cartesian,
    Joints {
        fraction: f64,
        target: Option<[f64; 7]>,
    },
}

impl Session {
    fn kind(&self) -> Kind {
        match self {
            Session::Cartesian => Kind::Cartesian,
            Session::Joints { .. } => Kind::Joints,
        }
    }
}

impl<R: RobotSide> Machine<R> {
    pub(super) fn command(&mut self, verb: Verb, request: CmdRequest, reply: Reply) {
        let client = request.client_id;
        let result = match verb {
            Verb::Acquire => self.acquire(client),
            Verb::Release => self.release(client),
            Verb::Enable => self.enable(&request),
            Verb::Stop => self.stop_command(),
            Verb::Recover => self.recover(),
            Verb::Home => match self.start_home(&request) {
                Ok(()) => {
                    // Answered by `finish_home`.
                    self.homing = Some(Homing {
                        reply,
                        started: Instant::now(),
                        within: 0,
                    });
                    return;
                }
                Err(why) => Err(why),
            },
            Verb::GripperHome => match (&self.gripper, client != 0 && self.holder == client) {
                (_, false) => Err(Reason::NotHolder(client).to_string()),
                (None, true) => Err("no gripper".into()),
                (Some(gripper), true) => {
                    // Answered from the home thread.
                    gripper.home(reply);
                    return;
                }
            },
            Verb::GripperStop => match &self.gripper {
                None => Err("no gripper".into()),
                Some(gripper) => {
                    gripper.stop();
                    Ok(())
                }
            },
        };
        let name = &self.config.name;
        reply(match result {
            Ok(()) => {
                debug!("arm {name}: {} from {client} ok", verb.key());
                CmdReply::ok()
            }
            Err(why) => {
                debug!("arm {name}: {} from {client} refused: {why}", verb.key());
                CmdReply::err(why)
            }
        });
    }

    fn acquire(&mut self, client: u32) -> Result<(), String> {
        if !self.alive.contains(&client) {
            return Err("no lease".into());
        }
        if self.holder != 0 && self.holder != client {
            return Err(format!("held by {}", self.holder));
        }
        match self.phase {
            Phase::Idle => {
                self.holder = client;
                self.transition(Phase::Acquired, &format!("acquired by {client}"));
                Ok(())
            }
            Phase::Acquired => Ok(()),
            other => Err(format!("not allowed in {other}")),
        }
    }

    fn release(&mut self, client: u32) -> Result<(), String> {
        if self.holder != client {
            return Err(Reason::NotHolder(client).to_string());
        }
        if matches!(self.phase, Phase::Active | Phase::Homing) {
            return Err(format!("not allowed in {}", self.phase));
        }
        self.holder = 0;
        if self.phase == Phase::Acquired {
            self.transition(Phase::Idle, "released");
        } else {
            info!("arm {}: released by {client}", self.config.name);
        }
        Ok(())
    }

    fn enable(&mut self, request: &CmdRequest) -> Result<(), String> {
        let client = request.client_id;
        if self.holder != client {
            return Err(Reason::NotHolder(client).to_string());
        }
        if !matches!(self.phase, Phase::Idle | Phase::Acquired) {
            return Err(format!("not allowed in {}", self.phase));
        }
        let episode = request.episode()?;
        let mode = request.mode.unwrap_or_default();
        let session = match mode {
            Kind::Cartesian => Session::Cartesian,
            Kind::Joints => Session::Joints {
                fraction: self.config.joint_budget_fraction,
                target: None,
            },
        };
        let (control, guard) = self.start_session(session, episode)?;
        self.begin(control, guard, client, episode);
        let named = episode.map_or(String::new(), |token| format!(", episode {token}"));
        self.transition(Phase::Active, &format!("enabled, {mode}{named}"));
        Ok(())
    }

    /// Acquired → Homing: a joints session at `speed` of the limits with [`READY`] as its
    /// one target; the reply waits for `finish_home`.
    fn start_home(&mut self, request: &CmdRequest) -> Result<(), String> {
        let client = request.client_id;
        if self.holder != client {
            return Err(Reason::NotHolder(client).to_string());
        }
        if self.phase != Phase::Acquired {
            return Err(format!("not allowed in {}", self.phase));
        }
        let episode = request.episode()?;
        let speed = request.speed.unwrap_or(HOME_SPEED);
        if !speed.is_finite() {
            return Err("speed must be finite".into());
        }
        let speed = speed.clamp(*HOME_SPEED_RANGE.start(), *HOME_SPEED_RANGE.end());
        let session = Session::Joints {
            fraction: speed,
            target: Some(READY),
        };
        let (control, guard) = self.start_session(session, episode)?;
        if let Err(e) = control.set_target(READY) {
            let stopped = control.stop();
            self.recording.discard(&self.config.name);
            return Err(stopped.map_or_else(|s| s.to_string(), |()| e.to_string()));
        }
        self.begin(control, guard, client, episode);
        self.transition(Phase::Homing, &format!("home at {speed:.2} of the limits"));
        Ok(())
    }

    /// Reads the snapshot, sets the collision thresholds, opens the recording (under the
    /// collector's `episode` token, if any) and starts the loop of `session`; the gate is
    /// seeded from the session's target, or from the loop's start target. A failed start
    /// closes and removes the recording.
    fn start_session(
        &mut self,
        session: Session,
        episode: Option<&str>,
    ) -> Result<(Box<dyn Control>, Guard), String> {
        let text = |e: FrankaError| e.to_string();
        self.snapshot = self.robot.read_once().map_err(text)?;
        self.read_at = Some(Instant::now());
        self.robot
            .set_collision_behavior(self.config.collision_torque, self.config.collision_force)
            .map_err(text)?;
        self.recording
            .begin(&self.robot, &self.config.name, session.kind(), episode);
        let started = self.start_loop(session);
        if started.is_err() {
            self.recording.discard(&self.config.name);
        }
        started
    }

    fn start_loop(&mut self, session: Session) -> Result<(Box<dyn Control>, Guard), String> {
        let text = |e: FrankaError| e.to_string();
        let options = self.config.guard_options();
        let (control, guard) = match session {
            Session::Cartesian => {
                let mut loop_options = self.config.target_control_options();
                loop_options.observer = self.recording.cartesian_observer();
                let control = self.robot.start(loop_options).map_err(text)?;
                let seed = control.target();
                (control, Guard::new(options, seed))
            }
            Session::Joints { fraction, target } => {
                let mut loop_options = self.joint_options(fraction, target);
                loop_options.observer = self.recording.joint_observer();
                let control = self.robot.start_joints(loop_options).map_err(text)?;
                let limits = joint_position_limits(self.robot.fci_version());
                let seed = target.unwrap_or_else(|| control.target());
                (control, Guard::joints(options, limits, seed))
            }
        };
        match guard {
            Ok(guard) => Ok((control, guard)),
            Err(reason) => {
                if let Err(e) = control.stop() {
                    warn!(
                        "arm {}: stop after a bad initial target: {e}",
                        self.config.name
                    );
                }
                Err(format!("initial target: {reason}"))
            }
        }
    }

    /// The joint loop's options: `fraction` of the arm's limits and the config's deviation,
    /// priority and cpu. With a `target` to travel to, the deviation guard (measured from the
    /// start) gets room for that travel plus [`HOME_DEVIATION_MARGIN`].
    fn joint_options(&self, fraction: f64, target: Option<[f64; 7]>) -> JointTargetControlOptions {
        let limits = JointTargetControlOptions::scaled_limits(self.robot.fci_version(), fraction);
        let options = self.config.joint_control_options(limits);
        match target {
            Some(target) => {
                let travel = self
                    .snapshot
                    .q
                    .iter()
                    .zip(&target)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f64::max);
                let deviation = options.max_deviation.max(travel + HOME_DEVIATION_MARGIN);
                options.with_max_deviation(deviation)
            }
            None => options,
        }
    }

    /// A session starts: `client` holds the gate, the episode's `start` goes out under the
    /// collector's `episode` token if it named one, the counters and the target bookkeeping
    /// start over.
    fn begin(
        &mut self,
        control: Box<dyn Control>,
        mut guard: Guard,
        client: u32,
        episode: Option<&str>,
    ) {
        guard.set_holder(client, monotonic_ns());
        self.guard = Some(guard);
        self.control = Some(control);
        // The session seeded its tuning from the config, deliberately: an experiment nobody
        // saved is not something a new session should inherit. So whatever a client set during
        // the last one is no longer in force, and `params/current` says so at once.
        self.params_reseeded();
        self.stats.set_recording(self.recording.file_name());
        self.begin_episode(episode);
        self.counts = Counts::default();
        self.channel_dropped_base = self.backlog.dropped();
        self.accepted = (0, 0);
        self.last_accept = Instant::now();
        self.last_target = self.last_accept;
        self.holding = false;
    }

    fn stop_command(&mut self) -> Result<(), String> {
        match self.phase {
            Phase::Active | Phase::Homing => self.end_session("stop command", Err("home stopped")),
            other => Err(format!("not allowed in {other}")),
        }
    }

    /// Ends whatever session runs, for `why`: Active stops to Idle; Homing stops to Acquired
    /// and answers the home query with `outcome`. Nothing without a session.
    pub(super) fn end_session(
        &mut self,
        why: &str,
        outcome: Result<(), &str>,
    ) -> Result<(), String> {
        match self.phase {
            Phase::Active => self.stop(why),
            Phase::Homing => self.finish_home(why, outcome),
            _ => Ok(()),
        }
    }

    /// Ends the homing session for `why` and answers the home query with `outcome`, or with
    /// the stop's error; lands in Acquired. Short of arrival the loop is first re-targeted to
    /// where the arm is, so it decelerates in place instead of on towards [`READY`].
    pub(super) fn finish_home(
        &mut self,
        why: &str,
        outcome: Result<(), &str>,
    ) -> Result<(), String> {
        let homing = self.homing.take();
        if let (Err(_), Some(control)) = (outcome, &self.control) {
            if let Err(e) = control.set_target(control.state().q) {
                warn!(
                    "arm {}: hold in place before the stop: {e}",
                    self.config.name
                );
            }
        }
        let stopped = self.stop_to(why, Phase::Acquired);
        if let Some(homing) = homing {
            (homing.reply)(match (&stopped, outcome) {
                (Ok(()), Ok(())) => CmdReply::ok(),
                (Ok(()), Err(e)) => CmdReply::err(e),
                (Err(e), _) => CmdReply::err(e),
            });
        }
        stopped
    }

    /// Active → Stopping → Idle, or Faulted with the loop's error; the holder is kept.
    pub(super) fn stop(&mut self, why: &str) -> Result<(), String> {
        self.stop_to(why, Phase::Idle)
    }

    fn stop_to(&mut self, why: &str, landing: Phase) -> Result<(), String> {
        let Some(control) = self.control.take() else {
            return Ok(());
        };
        self.transition(Phase::Stopping, why);
        self.publish_state();
        match control.stop() {
            Ok(()) => {
                self.after_loop(landing, "stopped");
                Ok(())
            }
            Err(e) => {
                let message = e.to_string();
                self.after_loop(Phase::Faulted, &message);
                Err(message)
            }
        }
    }

    /// The loop is gone for `why`: stops the handle for its message and goes Faulted.
    pub(super) fn fault(&mut self, why: &str) {
        let message = match self.control.take() {
            Some(control) => control
                .stop()
                .map_or_else(|e| e.to_string(), |()| why.to_string()),
            None => why.to_string(),
        };
        warn!("arm {}: fault: {message}", self.config.name);
        self.after_loop(Phase::Faulted, &message);
    }

    /// The session is over (the loop is joined): the recording is closed and the episode's
    /// `end` published, the published target reverts to the pose, and a home query still
    /// waiting (the loop ended on its own) is answered with `why`.
    fn after_loop(&mut self, phase: Phase, why: &str) {
        self.recording.finish(&self.config.name);
        self.stats.set_recording(None);
        self.end_episode();
        self.read_at = None;
        self.holding = false;
        self.guard = None;
        self.transition(phase, why);
        // The values in force are the config's again: nothing holds a session's live tuning.
        self.params_reseeded();
        if let Some(homing) = self.homing.take() {
            (homing.reply)(CmdReply::err(why));
        }
    }

    /// `automatic_error_recovery` in any phase without a loop, then Idle; the holder is kept.
    fn recover(&mut self) -> Result<(), String> {
        if matches!(self.phase, Phase::Active | Phase::Homing) {
            return Err(format!("not allowed in {}", self.phase));
        }
        self.robot
            .automatic_error_recovery()
            .map_err(|e| e.to_string())?;
        self.transition(Phase::Idle, "recovered");
        Ok(())
    }

    pub(super) fn lease_lost(&mut self, client: u32) {
        self.alive.remove(&client);
        if client != self.holder {
            return;
        }
        let _ = self.end_session("lease lost", Err("lease lost"));
        self.holder = 0;
        if self.phase == Phase::Acquired {
            self.transition(Phase::Idle, "lease lost");
        }
        info!(
            "arm {}: lease of {client} lost, holder cleared",
            self.config.name
        );
    }
}
