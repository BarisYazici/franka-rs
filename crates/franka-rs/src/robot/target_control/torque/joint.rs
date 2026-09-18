//! The joint tracker: the generator runs from its own last output, anchored on the measured
//! configuration in the first cycle; its limits are the budget, and the goal's step is scaled as
//! a whole into the joint limits' box, which keeps the synchronised line. When the box cuts the
//! step, the generator restarts at its end velocity cut by the same scale, and, against a position
//! bound, with only the acceleration that brakes it. The hold is not
//! boxed: its goal moves only with an arm moved by hand, a leash ahead of it.

use super::super::joint::{self as interface, max_abs_difference, JointSent};
use super::super::position::JointLimits;
use super::super::runner::{braking, strictly_braking, Step};
use super::super::velocity::scale_step;
use super::super::{JointTargetControlOptions, Leash};
use super::{Command, Tracker};
use crate::rate_limiting::DELTA_T;
use crate::robot_state::RobotState;

/// See the [module documentation](self).
pub(in super::super) struct JointTracker {
    max_deviation: f64,
    limits: JointLimits,
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
    /// Per joint, the bound that cut the step: 0 none, ∓1 position, ∓2 velocity.
    pinned: [i8; 7],
    /// The generator's next velocity and acceleration when the box cut the goal.
    restart: Option<([f64; 7], [f64; 7])>,
}

impl JointTracker {
    pub(in super::super) fn new(options: &JointTargetControlOptions, limits: JointLimits) -> Self {
        JointTracker {
            max_deviation: options.max_deviation,
            limits,
            anchored: None,
        }
    }
}

impl Tracker<7, 7> for JointTracker {
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
            pinned: [0; 7],
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

    /// A joint target inside the margin is refused, so a joint goal reaches its target as long as
    /// that inset matches the goal's box: never blocked.
    fn blocked(&self) -> bool {
        false
    }

    fn goal(&mut self, step: &Step<7, 7>) -> ([f64; 7], [f64; 7]) {
        let a = self.anchored.as_mut().expect("anchored before goal");
        (a.cap_scale, a.pinned, a.restart) = (1.0, [0; 7], None);
        // The goal's own velocity, not the generator's: while the leash binds the goal
        // stands a leash ahead of the arm and the damping feeds nothing forward.
        let (q_goal, dq_goal) = if step.hold {
            a.frozen.get_or_insert(a.anchor);
            (a.anchor, [0.0; 7])
        } else {
            let previous = a.goal;
            let bounds = self.limits.cycle_box(&previous, DELTA_T);
            let mut q_goal = step.position;
            let scale = scale_step(&previous, &mut q_goal, &bounds.lower, &bounds.upper);
            if scale < 1.0 {
                // The goal is the leash's next reference already; the generator's end velocity
                // cut by the same scale is its next one.
                let velocity = step.velocity.map(|v| scale * v);
                a.cap_scale = scale;
                a.pinned = std::array::from_fn(|i| {
                    let planned = step.position[i] - previous[i];
                    let went = q_goal[i] - previous[i];
                    if planned > 0.0 && went >= bounds.upper[i] - 1e-12 {
                        bounds.pin(i, 1)
                    } else if planned < 0.0 && went <= bounds.lower[i] + 1e-12 {
                        bounds.pin(i, -1)
                    } else {
                        0
                    }
                });
                let acceleration = if a.pinned.iter().any(|pin| pin.abs() == 1) {
                    strictly_braking(&velocity, &step.acceleration)
                } else {
                    braking(&velocity, &step.acceleration)
                };
                a.restart = Some((velocity, acceleration));
            }
            (
                q_goal,
                std::array::from_fn(|i| (q_goal[i] - previous[i]) / DELTA_T),
            )
        };
        a.goal = q_goal;
        (q_goal, dq_goal)
    }

    fn sent(&self, step: &Step<7, 7>, command: &Command) -> JointSent {
        let a = self.anchored.as_ref().expect("anchored before sent");
        JointSent {
            q_goal: command.q_goal,
            dq_goal: command.dq_goal,
            cap_scale: a.cap_scale,
            pinned: a.pinned,
            tau: command.tau,
            tau_envelope: command.tau_envelope,
            tau_position: command.tau_position,
            leash_alteration: a.leash_alteration,
            ..interface::sent(step, command.q_goal)
        }
    }
}
