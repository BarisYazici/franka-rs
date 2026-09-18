//! Both torque loops as they ran before the joint position guard, the reference of the replays
//! and the plant scenarios: the Cartesian tracker on [`Ik::as_run`] (the 0.02 rad clamp, the
//! whole step scaled under the velocity cap, a cut restarting the generator at the cut
//! velocity), the joint tracker scaled under the cap alone, and the loop's envelope at flat
//! velocity limits with no position fade or spring. Either tracker can be resumed from a
//! snapshot of what it carries between cycles ([`Resume`]).

use std::sync::{mpsc, Arc};

use nalgebra::{SMatrix, SVector};

use super::super::super::cartesian::{
    self, axis_limits, compose, placement, pose_of, slot_values, CartesianSent, Placement,
};
use super::super::super::ik::Ik;
use super::super::super::joint::{self, max_abs_difference, JointSent};
use super::super::super::position::JointLimits;
use super::super::super::rotation::{exp, log, norm};
use super::super::super::runner::{braking, Step};
use super::super::super::torque::{Command, TorqueLoop, Tracker};
use super::super::super::velocity::cap_step;
use super::super::super::*;
use super::super::torque::Rig;
use crate::model::{Frame, Model};
use crate::rate_limiting::DELTA_T;

/// A tracker that a copy can be resumed from: the copy's next cycle is this one's, bit for bit.
pub(in super::super) trait Resume<const N: usize, const S: usize>:
    Tracker<N, S> + Sized
{
    fn resume(&self) -> Self;
}

/// Position limits no joint comes near: the loop's fade, spring and onset stay at their flat,
/// far-from-every-limit values, bit for bit today's.
fn far(impedance: &ImpedanceOptions, velocity: [f64; 7]) -> JointLimits {
    JointLimits {
        position: ([-1e3; 7], [1e3; 7]),
        margin: impedance.joint_position_margin,
        fraction: impedance.joint_velocity_fraction,
        velocity: velocity.into(),
    }
}

/// The Cartesian loop on `rig` as it ran, at the flat velocity limits, and its slot.
pub(in super::super) fn cartesian_loop(
    rig: &Rig,
    mut options: TargetControlOptions,
    impedance: ImpedanceOptions,
    restart: Restart,
) -> (TorqueLoop<6, 7, AsRunPose>, Arc<Shared<7>>) {
    let shared = Arc::new(Shared::<7>::default());
    let (started, _) = mpsc::sync_channel(1);
    let axes = axis_limits(options.limits, options.rotation_limits);
    let chart = cartesian::chart;
    let runner = Runner::new(Arc::clone(&shared), started, axes, options.settle, chart).unwrap();
    let tracker = pose_tracker(rig, &options, impedance, restart);
    let model = Arc::clone(&rig.model);
    let observer = options.observer.take();
    // No live tuning: this harness replays a recorded session bit for bit.
    let torque = TorqueLoop::new(runner, model, impedance, tracker, observer, None);
    (torque, shared)
}

/// How a goal cut under the velocity cap restarted the generator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in super::super) enum Restart {
    /// At its own end-of-cycle velocity cut by the same scale (the cap as committed).
    End,
    /// At the cut goal's own velocity: the twist `J(q_goal) dq_goal`, or `dq_goal` (before).
    Mean,
}

/// The Cartesian tracker as it ran on `rig`.
pub(in super::super) fn pose_tracker(
    rig: &Rig,
    options: &TargetControlOptions,
    impedance: ImpedanceOptions,
    restart: Restart,
) -> AsRunPose {
    let flat = max_joint_velocity(rig.version);
    AsRunPose {
        model: Arc::clone(&rig.model),
        impedance,
        joint_limits: rig.limits,
        cap: flat.map(|v| v * impedance.joint_velocity_fraction),
        limits: far(&impedance, flat),
        max_deviation: (options.max_deviation, options.max_angular_deviation),
        restart,
        anchored: None,
    }
}

/// The joint tracker as it ran on `rig`.
pub(in super::super) fn joint_tracker(
    rig: &Rig,
    options: &JointTargetControlOptions,
    impedance: ImpedanceOptions,
    restart: Restart,
) -> AsRunJoint {
    let flat = max_joint_velocity(rig.version);
    AsRunJoint {
        max_deviation: options.max_deviation,
        max_step: flat.map(|v| v * impedance.joint_velocity_fraction * DELTA_T),
        limits: far(&impedance, flat),
        restart,
        anchored: None,
    }
}

/// `measured` pulled toward `desired` by at most the leash (`torque::pose`'s).
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

/// The Cartesian tracker before the guard.
pub(in super::super) struct AsRunPose {
    model: Arc<Model>,
    impedance: ImpedanceOptions,
    joint_limits: ([f64; 7], [f64; 7]),
    /// The joint velocity cap, rad/s.
    cap: [f64; 7],
    limits: JointLimits,
    max_deviation: (f64, f64),
    restart: Restart,
    anchored: Option<PoseAnchor>,
}

struct PoseAnchor {
    start: Placement,
    desired: Placement,
    anchor: Placement,
    leash_alteration: (f64, f64),
    frozen: Option<Placement>,
    posture: [f64; 7],
    ik: Ik,
    tool: ([f64; 16], [f64; 16]),
    previous_goal: Option<[f64; 7]>,
    pose: [f64; 16],
    ik_error: f64,
    restart: Option<([f64; 6], [f64; 6])>,
}

impl AsRunPose {
    fn ik(&self, q0: [f64; 7], (f_t_ee, ee_t_k): ([f64; 16], [f64; 16])) -> Ik {
        let (model, options) = (Arc::clone(&self.model), self.impedance.ik);
        Ik::as_run(
            model,
            options,
            self.joint_limits,
            self.cap,
            q0,
            f_t_ee,
            ee_t_k,
        )
    }
}

impl Resume<6, 7> for AsRunPose {
    fn resume(&self) -> Self {
        let anchored = self.anchored.as_ref().map(|a| PoseAnchor {
            ik: self.ik(a.ik.q(), a.tool),
            ..*a
        });
        AsRunPose {
            model: Arc::clone(&self.model),
            anchored,
            ..*self
        }
    }
}

impl Tracker<6, 7> for AsRunPose {
    type Sent = CartesianSent;

    fn limits(&self) -> &JointLimits {
        &self.limits
    }

    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; 7] {
        let measured = placement(&self.model.pose(Frame::EndEffector, state));
        if self.anchored.is_none() {
            let tool = (state.F_T_EE, state.EE_T_K);
            self.anchored = Some(PoseAnchor {
                start: placement(&state.O_T_EE),
                desired: measured,
                anchor: measured,
                leash_alteration: (0.0, 0.0),
                frozen: None,
                posture: self.impedance.posture.unwrap_or(state.q),
                ik: self.ik(state.q, tool),
                tool,
                previous_goal: None,
                pose: [0.0; 16],
                ik_error: 0.0,
                restart: None,
            });
        }
        let a = self.anchored.as_mut().expect("anchored");
        let reference = a.frozen.as_ref().unwrap_or(&a.desired);
        (a.anchor, a.leash_alteration) = leashed(reference, &measured, leash);
        slot_values(&a.anchor.0, &a.anchor.1)
    }

    fn strayed(&self, state: &RobotState) -> bool {
        let (translation, rotation) = self.max_deviation;
        self.anchored
            .as_ref()
            .is_some_and(|a| cartesian::strayed(state, &a.start, translation, rotation))
    }

    fn restart(&self) -> Option<([f64; 6], [f64; 6])> {
        self.anchored.as_ref().and_then(|a| a.restart)
    }

    fn blocked(&self) -> bool {
        false
    }

    fn goal(&mut self, step: &Step<6, 7>) -> ([f64; 7], [f64; 7]) {
        let a = self.anchored.as_mut().expect("anchored before goal");
        if step.hold {
            a.frozen.get_or_insert(a.anchor);
            a.desired = a.anchor;
        } else {
            a.desired = compose(step, &a.anchor.1);
        }
        a.pose = pose_of(&a.desired);
        let (q_goal, ik_error) = a.ik.step(&a.pose, &a.posture, DELTA_T);
        a.ik_error = ik_error;
        let dq_goal = match a.previous_goal {
            Some(previous) if !step.hold => {
                std::array::from_fn(|i| (q_goal[i] - previous[i]) / DELTA_T)
            }
            _ => [0.0; 7],
        };
        a.previous_goal = Some(q_goal);
        a.restart = None;
        let scale = a.ik.cap_scale();
        if scale < 1.0 && !step.hold {
            a.desired = placement(&a.ik.pose_of(&q_goal));
            let velocity = match self.restart {
                Restart::End => step.velocity.map(|v| scale * v),
                Restart::Mean => {
                    let (f_t_ee, ee_t_k) = &a.tool;
                    let j = self
                        .model
                        .zero_jacobian_q(Frame::EndEffector, &q_goal, f_t_ee, ee_t_k);
                    (SMatrix::<f64, 6, 7>::from_column_slice(&j) * SVector::from(dq_goal)).into()
                }
            };
            a.restart = Some((velocity, braking(&velocity, &step.acceleration)));
        }
        (q_goal, dq_goal)
    }

    fn sent(&self, step: &Step<6, 7>, command: &Command) -> CartesianSent {
        let a = self.anchored.as_ref().expect("anchored before sent");
        CartesianSent {
            q_goal: command.q_goal,
            dq_goal: command.dq_goal,
            cap_scale: a.ik.cap_scale(),
            tau: command.tau,
            tau_envelope: command.tau_envelope,
            ik_error: a.ik_error,
            leash_alteration: a.leash_alteration.0,
            leash_angular_alteration: a.leash_alteration.1,
            ..cartesian::sent(step, a.pose)
        }
    }
}

/// The joint tracker before the guard: the step scaled as a whole under the cap.
pub(in super::super) struct AsRunJoint {
    max_deviation: f64,
    max_step: [f64; 7],
    limits: JointLimits,
    restart: Restart,
    anchored: Option<JointAnchor>,
}

#[derive(Clone, Copy)]
struct JointAnchor {
    start: [f64; 7],
    goal: [f64; 7],
    frozen: Option<[f64; 7]>,
    anchor: [f64; 7],
    leash_alteration: f64,
    cap_scale: f64,
    restart: Option<([f64; 7], [f64; 7])>,
}

impl Resume<7, 7> for AsRunJoint {
    fn resume(&self) -> Self {
        AsRunJoint { ..*self }
    }
}

impl Tracker<7, 7> for AsRunJoint {
    type Sent = JointSent;

    fn limits(&self) -> &JointLimits {
        &self.limits
    }

    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; 7] {
        let a = self.anchored.get_or_insert(JointAnchor {
            start: state.q,
            goal: state.q,
            frozen: None,
            anchor: state.q,
            leash_alteration: 0.0,
            cap_scale: 1.0,
            restart: None,
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

    fn restart(&self) -> Option<([f64; 7], [f64; 7])> {
        self.anchored.as_ref().and_then(|a| a.restart)
    }

    fn blocked(&self) -> bool {
        false
    }

    fn goal(&mut self, step: &Step<7, 7>) -> ([f64; 7], [f64; 7]) {
        let a = self.anchored.as_mut().expect("anchored before goal");
        let (q_goal, dq_goal) = if step.hold {
            a.frozen.get_or_insert(a.anchor);
            a.cap_scale = 1.0;
            (a.anchor, [0.0; 7])
        } else {
            let previous = a.goal;
            let mut q_goal = step.position;
            a.cap_scale = cap_step(&previous, &mut q_goal, &self.max_step);
            let dq = std::array::from_fn(|i| (q_goal[i] - previous[i]) / DELTA_T);
            (q_goal, dq)
        };
        let scale = a.cap_scale;
        a.restart = (scale < 1.0).then(|| {
            let velocity = match self.restart {
                Restart::End => step.velocity.map(|v| scale * v),
                Restart::Mean => dq_goal,
            };
            (velocity, braking(&velocity, &step.acceleration))
        });
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
