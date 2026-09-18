//! The Cartesian tracker: the generator runs from its own last pose, anchored in the first cycle
//! on the model's pose of the measured configuration, and the IK follows it inside the joint
//! limits' box.
//!
//! The generator is restarted whenever the goal falls short: held at a position limit (the IK's
//! stall flag, or a wall pushed on within the last 20 cycles), a velocity pin, or a backstop cut;
//! how, in [`restart`](super::restart). A stop counts it as landed once the goal is held at a wall
//! the target lies beyond, is within the settle tolerance of the target on every other axis, has
//! restarted slowly enough to stop inside it, and has come no tolerance closer to the target for a
//! [`landing_dwell`](super::restart::landing_dwell).

use std::sync::Arc;

use nalgebra::Matrix4;

use super::super::cartesian::{
    self, axis_limits, compose, placement, slot_values, CartesianSent, Placement,
};
use super::super::ik::{Ik, IkOptions};
use super::super::position::{JointLimits, VelocityLimit};
use super::super::rotation::{exp, log, norm};
use super::super::runner::Step;
use super::super::{ImpedanceOptions, Leash, TargetControlOptions};
use super::restart::{ahead, landed, landing_dwell, left, restart, Pressure, Walls};
use super::{Command, Tracker};
use crate::model::{Frame, Model};
use crate::rate_limiting::DELTA_T;
use crate::robot_state::RobotState;

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

/// See the [module documentation](self).
pub(in super::super) struct PoseTracker {
    model: Arc<Model>,
    ik: IkOptions,
    posture: Option<[f64; 7]>,
    /// [`ImpedanceOptions::velocity_feedforward_cutoff`], handed to each session's anchor.
    feedforward_cutoff: f64,
    limits: JointLimits,
    max_deviation: f64,
    max_angular_deviation: f64,
    /// [`Settle::tolerance`](super::super::Settle::tolerance), within which a stop held at a wall
    /// has slid as far as it can.
    settle: f64,
    /// Cycles the goal must have stood still against a wall before a stop lands on it.
    dwell: u32,
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
    /// `dq_goal` after the feedforward low-pass, the filter's only state. Zero at a session's
    /// start; a hold feeds the filter zero rather than forcing it, so the lead decays over the
    /// filter's own time constant instead of stepping.
    dq_goal_filtered: [f64; 7],
    /// [`ImpedanceOptions::velocity_feedforward_cutoff`], copied so the cycle reads no options.
    feedforward_cutoff: f64,
    pose: [f64; 16],
    ik_error: f64,
    /// The fraction of the generator's step that went out when it is restarted, else 1.
    cap_scale: f64,
    /// The generator's next velocity and acceleration when the goal fell short.
    restart: Option<([f64; 6], [f64; 6])>,
    blocked: bool,
    /// Whether this cycle's goal was held at a wall: the IK stalled, or a wall still within its
    /// life.
    held: bool,
    /// Cycles since the goal last came a settle tolerance closer to `target` on an axis, and how
    /// near it got on each: a stop lands on a wall only once the goal no longer closes on it.
    still: u32,
    closest: [f64; 6],
    target: [f64; 7],
    walls: Walls,
}

impl PoseTracker {
    pub(in super::super) fn new(
        options: &TargetControlOptions,
        impedance: &ImpedanceOptions,
        model: Arc<Model>,
        joint_limits: ([f64; 7], [f64; 7]),
        velocity: impl Into<VelocityLimit>,
    ) -> Self {
        let jerks = axis_limits(options.limits, options.rotation_limits).map(|a| a.max_jerk);
        PoseTracker {
            model,
            ik: impedance.ik,
            posture: impedance.posture,
            feedforward_cutoff: impedance.velocity_feedforward_cutoff,
            limits: JointLimits {
                position: joint_limits,
                margin: impedance.joint_position_margin,
                fraction: impedance.joint_velocity_fraction,
                velocity: velocity.into(),
            },
            max_deviation: options.max_deviation,
            max_angular_deviation: options.max_angular_deviation,
            settle: options.settle.tolerance,
            dwell: landing_dwell(&jerks, options.settle.tolerance),
            anchored: None,
        }
    }
}

impl Tracker<6, 7> for PoseTracker {
    type Sent = CartesianSent;

    fn limits(&self) -> &JointLimits {
        &self.limits
    }

    fn anchor(&mut self, state: &RobotState, leash: &Leash) -> [f64; 7] {
        let measured = placement(&self.model.pose(Frame::EndEffector, state));
        let (model, ik, posture, limits) = (&self.model, self.ik, self.posture, self.limits);
        let feedforward_cutoff = self.feedforward_cutoff;
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
                    state.q,
                    state.F_T_EE,
                    state.EE_T_K,
                ),
                previous_goal: None,
                dq_goal_filtered: [0.0; 7],
                feedforward_cutoff,
                pose: [0.0; 16],
                ik_error: 0.0,
                cap_scale: 1.0,
                restart: None,
                blocked: false,
                held: false,
                still: 0,
                closest: [f64::INFINITY; 6],
                target: [f64::NAN; 7],
                walls: Walls::default(),
            }
        });
        if let Some(old) = a.ik.set_frames(&state.F_T_EE, &state.EE_T_K) {
            // A tool set during the session: the poses so far were the old end effector's.
            let m = |t: &[f64; 16]| Matrix4::from_column_slice(t);
            let shift = m(&old).try_inverse().unwrap_or_else(Matrix4::identity) * m(&state.F_T_EE);
            let start = m(&cartesian::pose_of(&a.start)) * shift;
            a.start = placement(start.as_slice().try_into().expect("16 values"));
            // Where the IK's goal is, not the lagging arm, and no wall of the old tool's.
            let goal = placement(&a.ik.pose_of(&a.ik.q()));
            (a.desired, a.walls) = (goal, Walls::default());
            a.frozen = a.frozen.map(|_| goal);
        }
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

    fn restart(&self) -> Option<([f64; 6], [f64; 6])> {
        self.anchored.as_ref().and_then(|a| a.restart)
    }

    fn blocked(&self) -> bool {
        self.anchored.as_ref().is_some_and(|a| a.blocked)
    }

    fn goal(&mut self, step: &Step<6, 7>) -> ([f64; 7], [f64; 7]) {
        let a = self.anchored.as_mut().expect("anchored before goal");
        let limits = &self.limits;
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
        // The box bounds the IK's step from its own last solution: `dq_goal` is under it only as
        // long as that is the previous goal. Bitwise, so a NaN goal does not panic here.
        debug_assert!(a
            .previous_goal
            .is_none_or(|previous| previous.map(f64::to_bits) == a.ik.q().map(f64::to_bits)));
        let (q_goal, ik_error) = a.ik.step(&a.pose, &a.posture, DELTA_T);
        a.ik_error = ik_error;
        let (stall, cut, pinned) = (a.ik.stall(), a.ik.cap_scale(), a.ik.pinned());
        let velocity_pinned = pinned.iter().any(|pin| pin.abs() == 2);
        let previous = a.previous_goal;
        // A goal that is holding, or has no predecessor, has no velocity at all: the filter is
        // forced to rest rather than allowed to decay. A hold is terminal -- only a stop or a
        // deviation follows it -- and `motion_finished` waits on the *measured* velocity, so a
        // lead decaying into the stop would push the arm off the goal it just landed on for as
        // long as the settle takes.
        let moving = previous.is_some() && !step.hold;
        let raw_dq_goal: [f64; 7] = match previous {
            Some(previous) if moving => {
                std::array::from_fn(|i| (q_goal[i] - previous[i]) / DELTA_T)
            }
            _ => [0.0; 7],
        };
        // The feedforward carries the joint reference's own ripple into the torque, so bound its
        // bandwidth here, where the filter's state lives, rather than in the stateless law.
        let filtered = a.feedforward_cutoff < crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
        let dq_goal = if moving && filtered {
            let gain = crate::lowpass_filter::gain(DELTA_T, a.feedforward_cutoff);
            std::array::from_fn(|i| gain * raw_dq_goal[i] + (1.0 - gain) * a.dq_goal_filtered[i])
        } else {
            raw_dq_goal
        };
        a.dq_goal_filtered = dq_goal;
        a.previous_goal = Some(q_goal);
        (a.cap_scale, a.restart, a.blocked) = (1.0, None, false);
        let held = a.walls.update(&stall);
        a.held = held;
        if !step.hold && (held || velocity_pinned || cut < 1.0) {
            // What went out, not what was planned: the generator restarts from what the goal's
            // step carried.
            let out = placement(&a.ik.pose_of(&q_goal));
            let from = previous.map_or(a.anchor, |q| placement(&a.ik.pose_of(&q)));
            let pressure = if held {
                Pressure::Position {
                    rest: a
                        .walls
                        .blocks
                        .map(|b| b.map_or(0.0, |(w, _)| limits.rest(&q_goal, &w.joints))),
                }
            } else if velocity_pinned {
                Pressure::Velocity
            } else {
                Pressure::None
            };
            let (velocity, acceleration, fractions) =
                restart(step, (&a.anchor, &from), &out, cut, pressure, &a.walls);
            let fraction = fractions[0].min(fractions[1]);
            // Held, the generator restarts on what of its step went out, less its lead into a
            // wall; otherwise as far ahead of the goal as the anchor was of the last goal: the
            // IK's lag stays, where dropping it takes the goal's velocity.
            a.desired = ahead(step, (&a.anchor, &from), &out, held.then_some(&a.walls));
            a.cap_scale = fraction;
            a.restart = Some((velocity, acceleration));
            let tolerance = self.settle;
            // Counted toward this target: a new one is a new stop, whose goal has yet to move.
            let same = a.target.map(f64::to_bits) == step.target.map(f64::to_bits);
            a.target = step.target;
            let remaining = left(&step.target, &out).map(f64::abs);
            let closer = (0..6).any(|i| remaining[i] <= a.closest[i] - tolerance);
            a.still = a.still.saturating_add(1);
            if !same || closer {
                (a.still, a.closest) = (0, remaining);
            }
            a.blocked = held
                && a.still >= self.dwell
                && landed(&step.target, &out, &a.walls, tolerance, &velocity);
        } else if !step.hold {
            // Nothing held the goal back this cycle: whatever it has been doing, it is not a goal
            // a wall has as near its target as it gets.
            (a.still, a.closest) = (0, [f64::INFINITY; 6]);
        }
        (q_goal, dq_goal)
    }

    fn sent(&self, step: &Step<6, 7>, command: &Command) -> CartesianSent {
        let a = self.anchored.as_ref().expect("anchored before sent");
        let stall = a.ik.stall();
        let (ik_step, ik_step_clipped) = a.ik.step_norms();
        CartesianSent {
            q_goal: command.q_goal,
            dq_goal: command.dq_goal,
            cap_scale: a.cap_scale,
            pinned: a.ik.pinned(),
            tau: command.tau,
            tau_envelope: command.tau_envelope,
            tau_position: command.tau_position,
            ik_error: a.ik_error,
            ik_passes: a.ik.passes(),
            stall_pressure: stall.position,
            stalled: stall.stalled,
            ik_step,
            ik_step_clipped,
            ik_blend: a.ik.priority(),
            held: a.held,
            // The age is at most `STALL_QUIET_CYCLES`; -1 is a block with no wall.
            wall_age: a
                .walls
                .blocks
                .map(|b| b.map_or(-1, |(_, age)| i8::try_from(age).unwrap_or(i8::MAX))),
            leash_alteration: a.leash_alteration.0,
            leash_angular_alteration: a.leash_alteration.1,
            ..cartesian::sent(step, a.pose)
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Matrix3;

    use super::super::super::rotation::{angle_between, distance};
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
