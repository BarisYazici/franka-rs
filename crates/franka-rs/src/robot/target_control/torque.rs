//! [`Backend::Impedance`](super::Backend::Impedance), both interfaces: the generator runs on
//! a leashed anchor a [`Tracker`] derives from the measured state, the tracker turns the
//! generator's output into a joint goal and its velocity, and the law's torques go out
//! through [`Robot::control_torques`].
//!
//! The leash is the torque-mode form of the third OTG rule. There is no echo of a torque
//! command to re-anchor on, so the anchor is the measured state pulled toward the previous
//! desired by at most [`Leash`]: exactly the previous desired while the arm follows, and a
//! bounded distance ahead of an arm that is held back. Once the runner holds, the leash
//! pulls toward the frozen hold instead, every cycle: an arm moved during the hold meets the
//! spring over the leash, never over the whole displacement.
//!
//! When the joint velocity cap cuts a goal, the generator is restarted at its own end-of-cycle
//! velocity cut by the same scale, keeping only an acceleration that brakes (and, for a pose, the
//! leash re-anchored on the capped goal's pose) before the next step: no windup, and no braking
//! lost to a cap that keeps binding. Not at the capped step's mean velocity: once the plan brakes
//! inside a step the mean is above its end velocity, and replanning from it runs past the target.

use std::sync::Arc;

use super::cartesian::{self, compose, placement, slot_values, CartesianSent, Placement};
use super::ik::{Ik, IkOptions};
use super::impedance::impedance_torques;
use super::joint::{self, max_abs_difference, JointSent};
use super::rotation::{exp, log, norm};
use super::runner::{Step, REST_JOINT_VELOCITY};
use super::velocity::{cap_step, fade_push, fade_start, velocity_barrier};
use super::{
    ImpedanceOptions, JointTargetControlOptions, Leash, Runner, TargetControlOptions,
    STOP_TIMEOUT_CYCLES,
};
use crate::control_types::Torques;
use crate::error::FrankaResult;
use crate::model::{Frame, Model};
use crate::rate_limiting::DELTA_T;
use crate::robot::Robot;
use crate::robot_state::RobotState;

/// What differs between the Cartesian and the joint torque path: where the generator is
/// anchored, what its step means as a joint goal, the deviation guard and the observer's
/// record. Called in this order every cycle, `anchor` first.
pub(super) trait Tracker<const N: usize, const S: usize> {
    type Sent;
    /// The leashed anchor for this cycle, in the slot's coordinates; the first call anchors
    /// the tracker itself on the measured state.
    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; S];
    /// Whether the measured state has strayed further from the start than allowed.
    fn strayed(&self, state: &RobotState) -> bool;
    /// The velocity to restart the generator at, in the runner's coordinates, when the joint
    /// velocity cap cut the previous goal: its end-of-cycle velocity cut by the cap's scale;
    /// `None` when the cap did not cut.
    fn capped_velocity(&self) -> Option<[f64; N]>;
    /// The joint goal and its velocity after this cycle's step.
    fn goal(&mut self, step: &Step<N, S>) -> ([f64; 7], [f64; 7]);
    fn sent(&self, step: &Step<N, S>, command: &Command) -> Self::Sent;
}

type Observer<S> = Box<dyn FnMut(&RobotState, &S) + Send>;

/// What the law made of a cycle's goal, for the observer's record.
pub(super) struct Command {
    pub q_goal: [f64; 7],
    pub dq_goal: [f64; 7],
    pub tau: [f64; 7],
    pub tau_envelope: [f64; 7],
}

/// The loop of [`Backend::Impedance`](super::Backend::Impedance); `cycle` is the
/// [`Robot::control_torques`] callback.
pub(super) struct TorqueLoop<const N: usize, const S: usize, T: Tracker<N, S>> {
    runner: Runner<N, S>,
    model: Arc<Model>,
    impedance: ImpedanceOptions,
    tracker: T,
    observer: Option<Observer<T::Sent>>,
    /// Where the law's push along a joint's motion starts to fade, rad/s per joint: the cap, or
    /// [`FADE_BAND`](super::FADE_BAND) of the limit under the barrier's onset if that is lower.
    fade_start: [f64; 7],
    /// Where the velocity barrier starts and that push is gone, rad/s per joint.
    barrier_onset: [f64; 7],
    /// Cycles the runner has reported the finish for while the arm was still moving.
    waited: u32,
}

impl<const N: usize, const S: usize, T: Tracker<N, S>> TorqueLoop<N, S, T> {
    pub(super) fn new(
        runner: Runner<N, S>,
        model: Arc<Model>,
        impedance: ImpedanceOptions,
        tracker: T,
        observer: Option<Observer<T::Sent>>,
        max_velocity: [f64; 7],
    ) -> Self {
        TorqueLoop {
            runner,
            model,
            impedance,
            tracker,
            observer,
            fade_start: max_velocity.map(|v| {
                fade_start(
                    v,
                    v * impedance.joint_velocity_fraction,
                    v * impedance.velocity_barrier_fraction,
                )
            }),
            barrier_onset: max_velocity.map(|v| v * impedance.velocity_barrier_fraction),
            waited: 0,
        }
    }

    pub(super) fn cycle(&mut self, state: &RobotState) -> Torques {
        let commanded = self.tracker.anchor(state, &self.impedance.leash);
        let strayed = self.tracker.strayed(state);
        if let Some(velocity) = self.tracker.capped_velocity() {
            self.runner.restart_at_velocity(velocity);
        }
        let step = self.runner.cycle(state, commanded, strayed);
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
        let kept = fade_push(&law, &state.dq, &self.fade_start, &self.barrier_onset);
        let barrier = velocity_barrier(
            &state.dq,
            &self.barrier_onset,
            &self.impedance.torque_limits,
        );
        let limits = &self.impedance.torque_limits;
        let tau: [f64; 7] =
            std::array::from_fn(|i| (kept[i] + barrier[i]).clamp(-limits[i], limits[i]));
        let tau_envelope = std::array::from_fn(|i| barrier[i] - (law[i] - kept[i]));
        if let Some(observe) = self.observer.as_mut() {
            let command = Command {
                q_goal,
                dq_goal,
                tau,
                tau_envelope,
            };
            observe(state, &self.tracker.sent(&step, &command));
        }
        let mut output = Torques::new(tau);
        output.motion_finished = step.finished && self.arm_at_rest_or_waited_out(state);
        output
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

/// `measured` pulled toward `desired` by at most the leash, and what the leash took off: the
/// translation, m, and the rotation, rad. Exactly `desired` within the leash.
fn leashed(desired: &Placement, measured: &Placement, leash: &Leash) -> (Placement, (f64, f64)) {
    let e_t: [f64; 3] = std::array::from_fn(|i| desired.0[i] - measured.0[i]);
    let e_r = log(&(desired.1 * measured.1.transpose()));
    let (dt, dr) = (norm(&e_t), norm(&e_r));
    let s = (leash.translation / dt).min(leash.rotation / dr).min(1.0);
    if s >= 1.0 {
        return (*desired, (0.0, 0.0));
    }
    let position = std::array::from_fn(|i| measured.0[i] + s * e_t[i]);
    let rotation = exp(&e_r.map(|x| s * x)) * measured.1;
    ((position, rotation), ((1.0 - s) * dt, (1.0 - s) * dr))
}

/// The Cartesian tracker: the generator runs from its own last pose, anchored in the first
/// cycle on the model's pose of the measured configuration, and the IK follows it.
pub(super) struct PoseTracker {
    model: Arc<Model>,
    ik: IkOptions,
    posture: Option<[f64; 7]>,
    joint_limits: ([f64; 7], [f64; 7]),
    /// The joint velocity cap, rad/s: the arm's limits times the fraction.
    max_velocity: [f64; 7],
    max_deviation: f64,
    max_angular_deviation: f64,
    anchored: Option<PoseAnchor>,
}

/// What the first cycle anchors on -- the measured pose for the guard, the model's pose of the
/// measured configuration as the first desired pose, that configuration as the IK's start and
/// its default posture -- and the cycle's running state.
struct PoseAnchor {
    start: Placement,
    desired: Placement,
    /// The leashed anchor the runner was given this cycle, which its step composes onto.
    anchor: Placement,
    leash_alteration: (f64, f64),
    /// The pose the runner first held, the leash's reference from then on.
    frozen: Option<Placement>,
    posture: [f64; 7],
    ik: Ik,
    previous_goal: Option<[f64; 7]>,
    pose: [f64; 16],
    ik_error: f64,
    /// The scale the IK's step was cut by this cycle, 1 when it was not.
    cap_scale: f64,
    /// The generator's end-of-cycle twist cut by the cap's scale when the cap cut the goal: its
    /// next velocity.
    capped: Option<[f64; 6]>,
}

impl PoseTracker {
    pub(super) fn new(
        options: &TargetControlOptions,
        impedance: &ImpedanceOptions,
        model: Arc<Model>,
        joint_limits: ([f64; 7], [f64; 7]),
        max_velocity: [f64; 7],
    ) -> Self {
        PoseTracker {
            model,
            ik: impedance.ik,
            posture: impedance.posture,
            joint_limits,
            max_velocity: max_velocity.map(|v| v * impedance.joint_velocity_fraction),
            max_deviation: options.max_deviation,
            max_angular_deviation: options.max_angular_deviation,
            anchored: None,
        }
    }
}

impl Tracker<6, 7> for PoseTracker {
    type Sent = CartesianSent;

    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; 7] {
        let measured = placement(&self.model.pose(Frame::EndEffector, state));
        let (model, ik, posture, limits, cap) = (
            &self.model,
            self.ik,
            self.posture,
            self.joint_limits,
            self.max_velocity,
        );
        let a = self.anchored.get_or_insert_with(|| {
            // The desired pose starts where the IK's own model puts the arm, not at `O_T_EE`:
            // the residual is then zero by construction, whatever the two differ by (the
            // model's accuracy on a robot; 0.107 m on franka-sim, whose `O_T_EE` is the
            // joint-7 frame), and the first torque is that of rest.
            PoseAnchor {
                start: placement(&state.O_T_EE),
                desired: measured,
                anchor: measured,
                leash_alteration: (0.0, 0.0),
                frozen: None,
                posture: posture.unwrap_or(state.q),
                ik: Ik::new(
                    Arc::clone(model),
                    ik,
                    limits,
                    cap,
                    state.q,
                    state.F_T_EE,
                    state.EE_T_K,
                ),
                previous_goal: None,
                pose: [0.0; 16],
                ik_error: 0.0,
                cap_scale: 1.0,
                capped: None,
            }
        });
        let reference = a.frozen.as_ref().unwrap_or(&a.desired);
        (a.anchor, a.leash_alteration) = leashed(reference, &measured, leash);
        slot_values(&a.anchor.0, &a.anchor.1)
    }

    fn strayed(&self, state: &RobotState) -> bool {
        self.anchored.as_ref().is_some_and(|a| {
            cartesian::strayed(
                state,
                &a.start,
                self.max_deviation,
                self.max_angular_deviation,
            )
        })
    }

    fn capped_velocity(&self) -> Option<[f64; 6]> {
        self.anchored.as_ref().and_then(|a| a.capped)
    }

    fn goal(&mut self, step: &Step<6, 7>) -> ([f64; 7], [f64; 7]) {
        let a = self.anchored.as_mut().expect("anchored before goal");
        if step.hold {
            // The runner holds the anchor it was given: that pose is frozen as the leash's
            // reference, and the desired is its leashed form, so an arm moved during the hold
            // takes the desired with it beyond the leash.
            a.frozen.get_or_insert(a.anchor);
            a.desired = a.anchor;
        } else {
            a.desired = compose(step, &a.anchor.1);
        }
        a.pose = cartesian::pose_of(&a.desired);
        // The cap bounds the IK's step from its own last solution: `dq_goal` is under the cap
        // only as long as that is the previous goal. Bitwise, so a NaN goal does not panic here.
        debug_assert!(a
            .previous_goal
            .is_none_or(|previous| previous.map(f64::to_bits) == a.ik.q().map(f64::to_bits)));
        let (q_goal, ik_error) = a.ik.step(&a.pose, &a.posture, DELTA_T);
        a.ik_error = ik_error;
        a.cap_scale = a.ik.cap_scale();
        let dq_goal = match a.previous_goal {
            Some(previous) if !step.hold => {
                std::array::from_fn(|i| (q_goal[i] - previous[i]) / DELTA_T)
            }
            _ => [0.0; 7],
        };
        a.previous_goal = Some(q_goal);
        a.capped = None;
        if a.cap_scale < 1.0 && !step.hold {
            // What went out, not what was planned: the capped goal's pose is the leash's next
            // reference, and the generator's end twist cut by the same scale its next velocity
            // (see the module documentation).
            a.desired = placement(&a.ik.pose_of(&q_goal));
            let scale = a.cap_scale;
            a.capped = Some(step.velocity.map(|v| scale * v));
        }
        (q_goal, dq_goal)
    }

    fn sent(&self, step: &Step<6, 7>, command: &Command) -> CartesianSent {
        let a = self.anchored.as_ref().expect("anchored before sent");
        CartesianSent {
            q_goal: command.q_goal,
            dq_goal: command.dq_goal,
            cap_scale: a.cap_scale,
            tau: command.tau,
            tau_envelope: command.tau_envelope,
            ik_error: a.ik_error,
            leash_alteration: a.leash_alteration.0,
            leash_angular_alteration: a.leash_alteration.1,
            ..cartesian::sent(step, a.pose)
        }
    }
}

/// The joint tracker: the generator runs from its own last output, anchored on the measured
/// configuration in the first cycle; its limits are the budget, and the goal's step is scaled
/// under the joint velocity cap. The hold is not capped: its goal moves only with an arm moved
/// by hand, a leash ahead of it.
pub(super) struct JointTracker {
    max_deviation: f64,
    /// The joint velocity cap as a step per cycle, rad.
    max_step: [f64; 7],
    anchored: Option<JointAnchor>,
}

/// The measured start for the guard, and the cycle's running state.
struct JointAnchor {
    start: [f64; 7],
    /// The previous cycle's goal, the leash's reference until the runner holds.
    goal: [f64; 7],
    /// The goal the runner first held, the leash's reference from then on.
    frozen: Option<[f64; 7]>,
    /// The leashed anchor the runner was given this cycle.
    anchor: [f64; 7],
    leash_alteration: f64,
    /// The scale the goal's step was cut by this cycle, 1 when it was not.
    cap_scale: f64,
    /// The generator's end-of-cycle velocity cut by the cap's scale when the cap cut the goal:
    /// its next velocity.
    capped: Option<[f64; 7]>,
}

impl JointTracker {
    pub(super) fn new(
        options: &JointTargetControlOptions,
        impedance: &ImpedanceOptions,
        max_velocity: [f64; 7],
    ) -> Self {
        JointTracker {
            max_deviation: options.max_deviation,
            max_step: max_velocity.map(|v| v * impedance.joint_velocity_fraction * DELTA_T),
            anchored: None,
        }
    }
}

impl Tracker<7, 7> for JointTracker {
    type Sent = JointSent;

    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; 7] {
        let a = self.anchored.get_or_insert(JointAnchor {
            start: state.q,
            goal: state.q,
            frozen: None,
            anchor: state.q,
            leash_alteration: 0.0,
            cap_scale: 1.0,
            capped: None,
        });
        let reference = a.frozen.unwrap_or(a.goal);
        let mut worst = 0.0f64;
        a.anchor = std::array::from_fn(|i| {
            let ahead = reference[i] - state.q[i];
            if ahead.abs() <= leash.joint {
                reference[i]
            } else {
                worst = worst.max(ahead.abs() - leash.joint);
                state.q[i] + leash.joint.copysign(ahead)
            }
        });
        a.leash_alteration = worst;
        a.anchor
    }

    fn strayed(&self, state: &RobotState) -> bool {
        self.anchored
            .as_ref()
            .is_some_and(|a| max_abs_difference(&state.q, &a.start) > self.max_deviation)
    }

    fn capped_velocity(&self) -> Option<[f64; 7]> {
        self.anchored.as_ref().and_then(|a| a.capped)
    }

    fn goal(&mut self, step: &Step<7, 7>) -> ([f64; 7], [f64; 7]) {
        let a = self.anchored.as_mut().expect("anchored before goal");
        // The goal's own velocity, not the generator's: while the leash binds the goal
        // stands a leash ahead of the arm and the damping feeds nothing forward.
        let (q_goal, dq_goal) = if step.hold {
            a.frozen.get_or_insert(a.anchor);
            a.cap_scale = 1.0;
            (a.anchor, [0.0; 7])
        } else {
            let previous = a.goal;
            let mut q_goal = step.position;
            a.cap_scale = cap_step(&previous, &mut q_goal, &self.max_step);
            (
                q_goal,
                std::array::from_fn(|i| (q_goal[i] - previous[i]) / DELTA_T),
            )
        };
        // The goal is the leash's next reference already; when the cap cut it, the generator's
        // end velocity cut by the same scale is its next one (see the module documentation).
        let scale = a.cap_scale;
        a.capped = (scale < 1.0).then(|| step.velocity.map(|v| scale * v));
        a.goal = q_goal;
        (q_goal, dq_goal)
    }

    fn sent(&self, step: &Step<7, 7>, command: &Command) -> JointSent {
        let a = self.anchored.as_ref().expect("anchored before sent");
        JointSent {
            q_goal: command.q_goal,
            dq_goal: command.dq_goal,
            cap_scale: a.cap_scale,
            tau: command.tau,
            tau_envelope: command.tau_envelope,
            leash_alteration: a.leash_alteration,
            ..joint::sent(step, command.q_goal)
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Matrix3;

    use super::super::rotation::{angle_between, distance};
    use super::*;

    const LEASH: Leash = Leash {
        translation: 0.025,
        rotation: 0.15,
        joint: 0.3,
    };

    #[test]
    fn within_the_leash_the_anchor_is_exactly_the_desired() {
        let measured = ([0.3, 0.0, 0.5], exp(&[0.1, -0.2, 0.3]));
        let desired = ([0.31, 0.02, 0.49], exp(&[0.05, -0.05, 0.1]) * measured.1);
        let (anchor, taken) = leashed(&desired, &measured, &LEASH);
        assert_eq!(anchor.0, desired.0);
        assert_eq!(anchor.1, desired.1);
        assert_eq!(taken, (0.0, 0.0));
        let (same, _) = leashed(&measured, &measured, &LEASH);
        assert_eq!(same.0, measured.0);
    }

    #[test]
    fn beyond_the_leash_the_anchor_sits_on_the_leash_toward_the_desired() {
        let measured = ([0.3, 0.0, 0.5], Matrix3::identity());
        // 10 cm off along a diagonal, no rotation: the anchor is 2.5 cm along it.
        let desired = ([0.36, 0.08, 0.5], Matrix3::identity());
        let (anchor, (dt, dr)) = leashed(&desired, &measured, &LEASH);
        assert!((distance(&anchor.0, &measured.0) - 0.025).abs() < 1e-12);
        assert!((anchor.0[0] - 0.315).abs() < 1e-12 && (anchor.0[1] - 0.02).abs() < 1e-12);
        assert!((dt - 0.075).abs() < 1e-12 && dr == 0.0);
        assert_eq!(anchor.1, Matrix3::identity());
        // 0.6 rad off, no translation: the anchor is 0.15 rad along the same axis.
        let turned = ([0.3, 0.0, 0.5], exp(&[0.0, 0.6, 0.0]));
        let (anchor, (dt, dr)) = leashed(&turned, &measured, &LEASH);
        assert_eq!(anchor.0, measured.0);
        assert!((angle_between(&measured.1, &anchor.1) - 0.15).abs() < 1e-12);
        assert!((angle_between(&anchor.1, &turned.1) - 0.45).abs() < 1e-12);
        assert!(dt == 0.0 && (dr - 0.45).abs() < 1e-12);
        // Both off: one scale, the tighter of the two, keeps the direction.
        let both = ([0.4, 0.0, 0.5], exp(&[0.0, 0.6, 0.0]));
        let (anchor, (dt, dr)) = leashed(&both, &measured, &LEASH);
        let s = 0.15 / 0.6;
        assert!((distance(&anchor.0, &measured.0) - s * 0.1).abs() < 1e-12);
        assert!((angle_between(&measured.1, &anchor.1) - 0.15).abs() < 1e-12);
        assert!(((dt, dr).0 - (1.0 - s) * 0.1).abs() < 1e-12);
        assert!((dr - 0.45).abs() < 1e-12);
    }
}
