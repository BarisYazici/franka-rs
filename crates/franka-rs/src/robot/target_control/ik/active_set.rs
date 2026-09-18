//! One damped-least-squares step inside a box: `min ½‖J̃δ − ẽ‖² + ½λ²‖δ‖²` on `lo ≤ δ ≤ hi`,
//! by a primal active set warm-started from the last solve. Each pass pins the free joint that
//! violates its bound most, or else releases the pinned joint whose multiplier points inward
//! most, and re-solves the free joints with the pinned ones on their bounds: saturation in the
//! nullspace. No dwell: the exact solution is continuous in the task however the set changes.
//! Fixed-size throughout, nothing allocates.

use nalgebra::{Cholesky, SMatrix, SVector, U6};

pub(in super::super) type Jacobian = SMatrix<f64, 6, 7>;
pub(in super::super) type Task = SVector<f64, 6>;
pub(in super::super) type Joints = SVector<f64, 7>;

/// Passes per solve before the step is clipped into the box and counted as not converged.
pub(in super::super) const MAX_ACTIVE_SET_PASSES: u32 = 12;
const VIOLATION_TOLERANCE: f64 = 1e-12;
const MULTIPLIER_TOLERANCE: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(in super::super) struct Solution {
    pub d: Joints,
    /// Passes run; 0 for a plain step clipped into the box.
    pub passes: u32,
    pub converged: bool,
    /// Norm of the step before the box clip, rad.
    pub step: f64,
    /// Norm of what the box clip took off it, rad.
    pub clipped: f64,
}

/// A nullspace bias: a joint rate over the cycle, projected through the free joints'
/// nullspace and scaled as a whole to at most `cap` on any joint.
#[derive(Debug, Clone, Copy)]
pub(in super::super) struct Bias {
    pub rate: Joints,
    pub cap: f64,
}

/// The step on `act` (−1 on the lower bound, +1 on the upper, 0 free, updated in place) with at
/// most `max_passes` passes, `0` for the unconstrained step clipped. The bias is added to the
/// converged step and clipped into the box with it. `None`, `act` unchanged, when the factor
/// or the step is not finite.
pub(in super::super) fn solve(
    j: &Jacobian,
    e: &Task,
    (lo, hi): (&Joints, &Joints),
    bias: Option<Bias>,
    act: &mut [i8; 7],
    damping: f64,
    max_passes: u32,
) -> Option<Solution> {
    let saved = *act;
    let lambda2 = damping * damping;
    let mut passes = 0;
    let mut converged = max_passes == 0;
    let mut free = free_solve(j, e, lo, hi, act, lambda2);
    while passes < max_passes {
        let Some(FreeSolve { d, .. }) = &free else {
            break;
        };
        passes += 1;
        let (worst, violation) = (0..7)
            .filter(|&i| act[i] == 0)
            .map(|i| (i, (lo[i] - d[i]).max(d[i] - hi[i])))
            .fold((0, 0.0), |a, b| if b.1 > a.1 { b } else { a });
        let (changed, side) = if violation > VIOLATION_TOLERANCE {
            (worst, if d[worst] < lo[worst] { -1 } else { 1 })
        } else {
            let mu = j.transpose() * (j * d - e) + d * lambda2;
            let inward = |i: usize| {
                (act[i] < 0 && mu[i] < -MULTIPLIER_TOLERANCE)
                    || (act[i] > 0 && mu[i] > MULTIPLIER_TOLERANCE)
            };
            let Some(release) = (0..7)
                .filter(|&i| inward(i))
                .max_by(|&a, &b| mu[a].abs().total_cmp(&mu[b].abs()))
            else {
                converged = true;
                break;
            };
            (release, 0)
        };
        // The last pass's change is never solved on: the pins stay those of the step returned.
        if passes < max_passes {
            act[changed] = side;
            free = free_solve(j, e, lo, hi, act, lambda2);
        }
    }
    let Some(FreeSolve {
        jf,
        factor,
        free,
        mut d,
    }) = free
    else {
        *act = saved;
        return None;
    };
    if let Some(bias) = bias {
        let rate = Joints::from_fn(|i, _| if free[i] { bias.rate[i] } else { 0.0 });
        let mut pull = rate - jf.transpose() * factor.solve(&(jf * rate));
        let largest = pull.amax();
        if largest > bias.cap {
            pull *= bias.cap / largest;
        }
        d += pull;
    }
    if !d.iter().all(|x| x.is_finite()) {
        *act = saved;
        return None;
    }
    let boxed = Joints::from_fn(|i, _| d[i].max(lo[i]).min(hi[i]));
    Some(Solution {
        step: d.norm(),
        clipped: (d - boxed).norm(),
        d: boxed,
        passes,
        converged,
    })
}

/// A solve on one active set: the free joints' Jacobian (the pinned columns zero), the factor
/// of `J̃_F J̃_Fᵀ + λ²I`, which joints are free, the step.
struct FreeSolve {
    jf: Jacobian,
    factor: Cholesky<f64, U6>,
    free: [bool; 7],
    d: Joints,
}

/// The pinned joints on their bounds and the free ones by damped least squares on the rest of
/// the task.
fn free_solve(
    j: &Jacobian,
    e: &Task,
    lo: &Joints,
    hi: &Joints,
    act: &[i8; 7],
    lambda2: f64,
) -> Option<FreeSolve> {
    let mut jf = *j;
    let mut pinned = Joints::zeros();
    let free = act.map(|side| side == 0);
    for (i, &side) in act.iter().enumerate() {
        if side != 0 {
            pinned[i] = if side < 0 { lo[i] } else { hi[i] };
            jf.column_mut(i).fill(0.0);
        }
    }
    let a = jf * jf.transpose() + SMatrix::<f64, 6, 6>::identity() * lambda2;
    let factor = a.cholesky()?;
    let d = jf.transpose() * factor.solve(&(e - j * pinned)) + pinned;
    d.iter().all(|x| x.is_finite()).then_some(FreeSolve {
        jf,
        factor,
        free,
        d,
    })
}
