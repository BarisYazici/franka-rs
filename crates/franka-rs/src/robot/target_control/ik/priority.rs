//! Position first where the pins make the pose unreachable. While the stall flag is on, each
//! iteration solves the translation alone in the box, then the whole task from there with the
//! translation held, in the box that is left: the same active-set solve twice, the second on the
//! weighted task with its translation rows scaled by [`TRANSLATION_PRIORITY`] and its damping
//! centred on the first stage's step. The second stage's weight, target and centre are continuous
//! in the blend `β`, the weighted solve at 0, and `β` eases between 0 and 1 over
//! [`PRIORITY_RAMP_CYCLES`]: where the flag switches, the goal's velocity has no step.

use super::active_set::{Jacobian, Joints, Task};
use super::stall::Stall;

/// Cycles the blend takes from the weighted solve to position first.
pub(in super::super) const PRIORITY_RAMP_CYCLES: u32 = 500;
/// The translation rows' scale in the second stage: the orientation pulls the translation off by
/// ~1e-3 of what it does in the weighted solve (100 leaves active sets unsettled after 12 passes
/// on a stretched arm).
const TRANSLATION_PRIORITY: f64 = 30.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Priority {
    /// 0 to 1, moving at a constant rate toward the flag; `β` is its smoothstep.
    phase: f64,
    /// `β`: 0 the weighted solve, 1 position first.
    pub blend: f64,
    /// The cycles the blend takes, [`PRIORITY_RAMP_CYCLES`] in a session; 0 holds it at the
    /// weighted solve, which only a test asks for.
    #[cfg(test)]
    ramp: u32,
}

impl Default for Priority {
    fn default() -> Self {
        Priority {
            phase: 0.0,
            blend: 0.0,
            #[cfg(test)]
            ramp: PRIORITY_RAMP_CYCLES,
        }
    }
}

impl Priority {
    /// One cycle toward position first while the last step stalled, else back. The smoothstep
    /// is flat at both ends: a flag that drops for a few cycles leaves `β` nearly where it was.
    pub(super) fn update(&mut self, stall: &Stall) {
        let goal = f64::from(u8::from(stall.stalled));
        let step = match self.ramp() {
            0 => 0.0,
            ramp => 1.0 / f64::from(ramp),
        };
        self.phase = if (goal - self.phase).abs() <= step {
            goal
        } else {
            self.phase + step.copysign(goal - self.phase)
        };
        self.blend = self.phase * self.phase * (3.0 - 2.0 * self.phase);
    }

    /// The cycles the blend takes.
    #[cfg(not(test))]
    fn ramp(&self) -> u32 {
        PRIORITY_RAMP_CYCLES
    }

    #[cfg(test)]
    fn ramp(&self) -> u32 {
        self.ramp
    }

    /// Sets them; 0 holds the blend at the weighted solve.
    #[cfg(test)]
    pub(super) fn set_ramp(&mut self, cycles: u32) {
        self.ramp = cycles;
    }

    /// The first stage's task: the weighted task's translation rows alone.
    pub(super) fn position(j: &Jacobian, e: &Task) -> (Jacobian, Task) {
        let (mut j, mut e) = (*j, *e);
        j.fixed_rows_mut::<3>(3).fill(0.0);
        e.fixed_rows_mut::<3>(3).fill(0.0);
        (j, e)
    }

    /// The second stage's task on the step less `s`, the first stage's step scaled by `β`: the
    /// weighted task left after `s`, its translation rows scaled by `TRANSLATION_PRIORITY^β` and
    /// aimed at `1 − β` of what is left of the translation.
    pub(super) fn held(&self, j: &Jacobian, e: &Task, s: &Joints) -> (Jacobian, Task) {
        let beta = self.blend;
        let scale = TRANSLATION_PRIORITY.powf(beta);
        let (mut j, mut left) = (*j, e - j * s);
        left.fixed_rows_mut::<3>(0).scale_mut((1.0 - beta) * scale);
        j.fixed_rows_mut::<3>(0).scale_mut(scale);
        (j, left)
    }
}
