//! [`Backend::Impedance`](super::Backend::Impedance), both interfaces: the generator runs on
//! a leashed anchor a [`Tracker`] derives from the measured state, the tracker turns the
//! generator's output into a joint goal and its velocity, and the law's torques go out
//! through [`Robot::control_torques`] after the joint envelope at the measured state.
//!
//! The leash is the torque-mode form of the third OTG rule. There is no echo of a torque
//! command to re-anchor on, so the anchor is the measured state pulled toward the previous
//! desired by at most [`Leash`]: exactly the previous desired while the arm follows, and a
//! bounded distance ahead of an arm that is held back. Once the runner holds, the leash
//! pulls toward the frozen hold instead, every cycle: an arm moved during the hold meets the
//! spring over the leash, never over the whole displacement.
//!
//! When the goal falls short of the generator's step -- cut by the joint velocity limit or held
//! at a joint position limit -- the generator is restarted before the next step at its own
//! end-of-cycle velocity reduced to what went out (and, for a pose, re-anchored on what the goal's
//! step carried): no windup, and no braking lost to a bound that keeps binding. Not at the cut
//! step's mean velocity: once the plan brakes inside a step the mean is above its end velocity,
//! and replanning from it runs past the target.
//!
//! The envelope at the measured state: the law's push along a joint's motion fades out toward
//! the velocity barrier's onset, its push toward a joint limit fades out inside the margin, and
//! the velocity barrier and, inside the position barrier's onset, a spring are added before the
//! clamp.

mod joint;
mod pose;
mod restart;
mod tuning;

use std::sync::Arc;

pub(super) use joint::JointTracker;
pub(super) use pose::PoseTracker;
#[cfg(test)]
pub(super) use restart::{ahead, landed, landing_dwell, left, restart, Pressure, Walls};

use super::impedance::impedance_torques;
use super::position::{spring_ramp, JointLimits};
use super::runner::{Step, REST_JOINT_VELOCITY};
use super::velocity::{barrier_gains, fade_push, velocity_barrier};
use super::{ImpedanceOptions, Leash, LiveTuning, Runner, STOP_TIMEOUT_CYCLES};
use crate::control_types::Torques;
use crate::error::FrankaResult;
use crate::model::{Frame, Model};
use crate::otg::OtgLimits;
use crate::robot::Robot;
use crate::robot_state::RobotState;
use tuning::Tuning;

/// What differs between the Cartesian and the joint torque path: where the generator is
/// anchored, what its step means as a joint goal, the deviation guard and the observer's
/// record. Called in this order every cycle, `anchor` first.
pub(super) trait Tracker<const N: usize, const S: usize> {
    type Sent;
    /// The joint position limits the goal is kept inside.
    fn limits(&self) -> &JointLimits;
    /// The leashed anchor for this cycle, in the slot's coordinates; the first call anchors
    /// the tracker itself on the measured state.
    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; S];
    /// Whether the measured state has strayed further from the start than allowed.
    fn strayed(&self, state: &RobotState) -> bool;
    /// The velocity and acceleration to restart the generator at, in the runner's coordinates,
    /// when the previous goal fell short of its step; `None` when it did not.
    fn restart(&self) -> Option<([f64; N], [f64; N])>;
    /// Whether the goal is held at a position limit as near the target as it gets.
    fn blocked(&self) -> bool;
    /// The joint goal and its velocity after this cycle's step.
    fn goal(&mut self, step: &Step<N, S>) -> ([f64; 7], [f64; 7]);
    fn sent(&self, step: &Step<N, S>, command: &Command) -> Self::Sent;
    /// Writes the tracker's own copies of the tuned options, called before `anchor` on a cycle
    /// something moved. The joint tracker keeps no copy of any of them -- it has no IK, and its
    /// goal velocity is the goal's own finite difference, unfiltered -- so the default is to do
    /// nothing.
    fn retune(&mut self, _tuning: &LiveTuning) {}
    /// The generator's per-axis limits for `tuning`'s budget, in the runner's coordinates.
    /// `None` where the interface plans nothing from a Cartesian budget: the joint one, whose
    /// generator's limits are per joint and are not in [`LiveTuning`] at all.
    fn budget(&self, _tuning: &LiveTuning) -> Option<[OtgLimits; N]> {
        None
    }
}

type Observer<S> = Box<dyn FnMut(&RobotState, &S) + Send>;

/// What the law made of a cycle's goal, for the observer's record.
pub(super) struct Command {
    pub q_goal: [f64; 7],
    pub dq_goal: [f64; 7],
    pub tau: [f64; 7],
    pub tau_envelope: [f64; 7],
    pub tau_position: [f64; 7],
}

/// The loop of [`Backend::Impedance`](super::Backend::Impedance); `cycle` is the
/// [`Robot::control_torques`] callback.
pub(super) struct TorqueLoop<const N: usize, const S: usize, T: Tracker<N, S>> {
    runner: Runner<N, S>,
    model: Arc<Model>,
    impedance: ImpedanceOptions,
    tracker: T,
    observer: Option<Observer<T::Sent>>,
    /// The tracker's limits.
    limits: JointLimits,
    /// The velocity barrier's gain per joint, Nm per rad/s.
    gains: [f64; 7],
    /// Cycles run, for the spring's ramp.
    cycles: u32,
    /// Cycles the runner has reported the finish for while the arm was still moving.
    waited: u32,
    /// The [live tuning](tuning) this session accepts, `None` where it accepts none.
    tuning: Option<Tuning>,
}

impl<const N: usize, const S: usize, T: Tracker<N, S>> TorqueLoop<N, S, T> {
    pub(super) fn new(
        runner: Runner<N, S>,
        model: Arc<Model>,
        impedance: ImpedanceOptions,
        tracker: T,
        observer: Option<Observer<T::Sent>>,
        tuning: Option<LiveTuning>,
    ) -> Self {
        TorqueLoop {
            runner,
            model,
            impedance,
            limits: *tracker.limits(),
            tracker,
            observer,
            gains: barrier_gains(&impedance.torque_limits),
            cycles: 0,
            waited: 0,
            tuning: tuning.map(Tuning::new),
        }
    }

    pub(super) fn cycle(&mut self, state: &RobotState) -> Torques {
        self.retune();
        let commanded = self.tracker.anchor(state, &self.impedance.leash);
        let strayed = self.tracker.strayed(state);
        if let Some((velocity, acceleration)) = self.tracker.restart() {
            self.runner.restart(velocity, acceleration);
        }
        let blocked = self.tracker.blocked();
        let step = self.runner.cycle(state, commanded, strayed, blocked);
        let (q_goal, dq_goal) = self.tracker.goal(&step);
        let law = impedance_torques(
            &self.impedance,
            &self.model.zero_jacobian(Frame::EndEffector, state),
            &q_goal,
            &dq_goal,
            &state.q,
            &state.dq,
            &self.model.coriolis(state),
        );
        let (tau, tau_envelope, tau_position) = self.envelope(&law, state);
        if let Some(observe) = self.observer.as_mut() {
            let command = Command {
                q_goal,
                dq_goal,
                tau,
                tau_envelope,
                tau_position,
            };
            observe(state, &self.tracker.sent(&step, &command));
        }
        let mut output = Torques::new(tau);
        output.motion_finished = step.finished && self.arm_at_rest_or_waited_out(state);
        output
    }

    /// The law under the envelope at the measured state: the torque sent, the velocity
    /// envelope's share and the position envelope's share of it. `kept_v` is the law with its
    /// push along the motion faded, `kept` that with its push toward a limit faded;
    /// `tau = clamp(kept + barrier + spring)`, which is `clamp(law + tau_envelope + tau_position)`.
    fn envelope(&mut self, law: &[f64; 7], state: &RobotState) -> ([f64; 7], [f64; 7], [f64; 7]) {
        let (q, dq) = (&state.q, &state.dq);
        let limits = &self.impedance.torque_limits;
        let (start, onset) = self
            .limits
            .envelope(q, dq, self.impedance.velocity_barrier_fraction);
        let kept_v = fade_push(law, dq, &start, &onset);
        let kept = self.limits.fade_toward(&kept_v, q);
        let barrier = velocity_barrier(dq, &onset, &self.gains, limits);
        let spring = self.limits.spring(q, limits, spring_ramp(self.cycles));
        self.cycles = self.cycles.saturating_add(1);
        (
            std::array::from_fn(|i| {
                (kept[i] + barrier[i] + spring[i]).clamp(-limits[i], limits[i])
            }),
            std::array::from_fn(|i| barrier[i] - (law[i] - kept_v[i])),
            std::array::from_fn(|i| spring[i] - (kept_v[i] - kept[i])),
        )
    }

    /// The runner's hold settles the generator, not the arm, which may still be closing the
    /// lag: finishing then leaves it short, so the finish waits for the arm to rest, up to
    /// [`STOP_TIMEOUT_CYCLES`], with the law kept on the held goal meanwhile.
    fn arm_at_rest_or_waited_out(&mut self, state: &RobotState) -> bool {
        let at_rest = state.dq.iter().all(|v| v.abs() < REST_JOINT_VELOCITY);
        let waited_out = self.waited >= STOP_TIMEOUT_CYCLES;
        self.waited = self.waited.saturating_add(1);
        at_rest || waited_out
    }

    /// Runs the loop on `robot` to its end and returns its result.
    pub(super) fn run(mut self, robot: &Robot, limit_rate: bool) -> FrankaResult<()> {
        let cutoff = self.impedance.cutoff_frequency;
        let result = robot.control_torques(
            |state: &RobotState, _period| self.cycle(state),
            limit_rate,
            cutoff,
        );
        self.finish(result)
    }

    /// The loop's result; see [`Runner::finish`].
    pub(super) fn finish(&self, result: FrankaResult<()>) -> FrankaResult<()> {
        self.runner.finish(result)
    }
}
