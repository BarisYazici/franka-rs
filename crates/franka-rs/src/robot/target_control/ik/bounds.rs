//! What bounds the IK's step: the guard's cycle box and its posture clamp and fade, or, for the
//! tests, the fixed inset and whole-step cap the solver ran with before the guard.

use super::super::position::{CycleBox, JointLimits};
use super::super::velocity::scale_step;
use super::active_set::{Joints, MAX_ACTIVE_SET_PASSES};

/// How far, rad, beyond the margin the posture is clamped inside the limits and its bias starts
/// to fade as the nearest joint approaches its margin.
pub(in super::super) const POSTURE_LIMIT_DISTANCE: f64 = 0.3;

/// The inset, rad, of the solver before the guard.
#[cfg(test)]
pub(in super::super) const AS_RUN_LIMIT_MARGIN: f64 = 0.02;

pub(super) enum Bounds {
    Guarded(JointLimits),
    #[cfg(test)]
    AsRun {
        lower: [f64; 7],
        upper: [f64; 7],
        max_velocity: [f64; 7],
    },
}

/// A cycle's box on the step, active-set passes, and posture rate, rad per cycle.
pub(super) struct Cycle {
    pub bounds: CycleBox,
    pub passes: u32,
    pub posture: Joints,
}

impl Bounds {
    /// The solver before the guard on `limits`, and `q0` clamped into them.
    #[cfg(test)]
    pub(super) fn as_run(
        limits: ([f64; 7], [f64; 7]),
        max_velocity: [f64; 7],
        q0: [f64; 7],
    ) -> (Self, [f64; 7]) {
        let mut lower = limits.0.map(|l| l + AS_RUN_LIMIT_MARGIN);
        let mut upper = limits.1.map(|u| u - AS_RUN_LIMIT_MARGIN);
        for i in 0..7 {
            if lower[i] > upper[i] {
                lower[i] = 0.5 * (limits.0[i] + limits.1[i]);
                upper[i] = lower[i];
            }
        }
        let q0 = std::array::from_fn(|i| q0[i].clamp(lower[i], upper[i]));
        let bounds = Bounds::AsRun {
            lower,
            upper,
            max_velocity,
        };
        (bounds, q0)
    }

    /// The cycle from `from` toward `posture` at `gain` per cycle.
    pub(super) fn cycle(&self, from: &[f64; 7], posture: &[f64; 7], gain: f64, dt: f64) -> Cycle {
        let q = Joints::from(*from);
        match self {
            Bounds::Guarded(limits) => {
                let (lower, upper) = limits.position;
                let inset = limits.margin + POSTURE_LIMIT_DISTANCE;
                let near = (0..7).fold(f64::INFINITY, |near, i| {
                    near.min(from[i] - lower[i]).min(upper[i] - from[i])
                });
                // `1 − κ`: the bias fades out over the distance before the margin.
                let fade = ((near - limits.margin) / POSTURE_LIMIT_DISTANCE).clamp(0.0, 1.0);
                let post = Joints::from_fn(|i, _| {
                    let (lo, hi) = (lower[i] + inset, upper[i] - inset);
                    if lo <= hi {
                        posture[i].clamp(lo, hi)
                    } else {
                        0.5 * (lower[i] + upper[i])
                    }
                });
                Cycle {
                    bounds: limits.cycle_box(from, dt),
                    passes: MAX_ACTIVE_SET_PASSES,
                    posture: (post - q) * (fade * gain),
                }
            }
            #[cfg(test)]
            Bounds::AsRun {
                lower,
                upper,
                max_velocity,
            } => Cycle {
                bounds: CycleBox {
                    lower: std::array::from_fn(|i| lower[i] - from[i]),
                    upper: std::array::from_fn(|i| upper[i] - from[i]),
                    position_lower: [true; 7],
                    position_upper: [true; 7],
                    velocity_lower: max_velocity.map(|v| -(v * dt)),
                    velocity_upper: max_velocity.map(|v| v * dt),
                },
                passes: 0,
                posture: (Joints::from(*posture) - q) * gain,
            },
        }
    }

    /// Scales `goal`'s step from `from` under the velocity limit; returns the scale. The
    /// guard's box holds that limit but for rounding, so only a real cut is taken.
    pub(super) fn backstop(&self, from: &[f64; 7], goal: &mut [f64; 7], cycle: &Cycle) -> f64 {
        match self {
            Bounds::Guarded(_) => {
                let (mut cut, b) = (*goal, &cycle.bounds);
                match scale_step(from, &mut cut, &b.velocity_lower, &b.velocity_upper) {
                    scale if scale < 1.0 - 1e-9 => {
                        *goal = cut;
                        scale
                    }
                    _ => 1.0,
                }
            }
            #[cfg(test)]
            Bounds::AsRun { .. } => {
                super::super::velocity::cap_step(from, goal, &cycle.bounds.velocity_upper)
            }
        }
    }
}
