//! The torque backend's joint velocity envelope: the cap on the joint goal's step, which keeps
//! what the loop commands under a fraction of the arm's velocity limits; the fade, which takes
//! the law's push off a joint measured faster than the cap (or [`FADE_BAND`] under the barrier's
//! onset), so that catching up on its lag does not drive it past that onset; and the barrier on
//! the measured velocity, which damps what the two cannot: spring-back and overshoot.

use super::impedance::TORQUE_LIMITS;

/// Scales `to - from` as a whole by the largest `s ≤ 1` that keeps every joint's step within
/// `[lower, upper]` (`lower ≤ 0 ≤ upper`), which keeps the step's direction in joint space;
/// returns `s`.
pub(super) fn scale_step(
    from: &[f64; 7],
    to: &mut [f64; 7],
    lower: &[f64; 7],
    upper: &[f64; 7],
) -> f64 {
    let mut scale = 1.0f64;
    for i in 0..7 {
        let step = to[i] - from[i];
        if step > upper[i] {
            scale = scale.min(upper[i] / step);
        } else if step < lower[i] {
            scale = scale.min(lower[i] / step);
        }
    }
    if scale < 1.0 {
        for (q, from) in to.iter_mut().zip(from) {
            *q = from + scale * (*q - from);
        }
    }
    scale
}

/// [`scale_step`] within `±max_step`.
#[cfg(test)]
pub(super) fn cap_step(from: &[f64; 7], to: &mut [f64; 7], max_step: &[f64; 7]) -> f64 {
    scale_step(from, to, &max_step.map(|m| -m), max_step)
}

/// Nm per rad/s of the velocity barrier: the damping a joint meets beyond
/// [`ImpedanceOptions::velocity_barrier_fraction`](super::ImpedanceOptions::velocity_barrier_fraction)
/// of its limit. Stable at 1 kHz behind the 100 Hz low-pass while the law's own joint damping
/// plus this stays under the joint's effective inertia per millisecond; see *The impedance
/// backend* in the book.
pub const VELOCITY_BARRIER_GAIN: f64 = 20.0;

/// The narrowest band, as a fraction of a joint's velocity limit, over which the law's push
/// along the joint's motion fades out under the barrier's onset: the fade starts at the cap or
/// this far under the onset, whichever is lower, and never below rest.
///
/// Away from a position limit the band is this wide and the fade adds up to
/// `|law| / (FADE_BAND × limit)` Nm per rad/s to the loop's gain, about 30 on an FER's wrist at
/// its 11.5 Nm clamp, which the loop settles on 0.03 kg m² and more; a fade from a cap of 0.8
/// rings there. Inside the position margin the cap is 0 -- the braking envelope is 0 at and past
/// the margin -- so the start is 0 and the band is `[0, onset]`. Where the arm's limit toward
/// that side is flat, as the FER's is, that is the narrower band and the figure above is not the
/// bound; what bounds the gain is the position fade
/// ([`POSITION_FADE_BAND`](super::POSITION_FADE_BAND)) tapering the same push to nothing over
/// the same rad. Both vanish linearly, so the product is finite, and it peaks at the margin at
/// `|law| / (velocity_barrier_fraction × E(POSITION_FADE_BAND))`: on the FER 1.16 to 1.61 times
/// the figure above (`tests/torque_position.rs`). The FR3's limit is position-dependent and has
/// itself come down by the margin, so its band is wider there and the figure above does bound
/// it.
pub const FADE_BAND: f64 = 0.15;

/// Where a joint's fade starts, rad/s: at `cap`, or [`FADE_BAND`] of `limit` under `onset` if
/// that is lower, and never below rest, where a barrier under the band would otherwise fade the
/// law at every speed.
pub(super) fn fade_start(limit: f64, cap: f64, onset: f64) -> f64 {
    cap.min(onset - FADE_BAND * limit).max(0.0)
}

/// `law` with every joint's torque along its measured motion faded out linearly as the joint
/// speeds from `start` to `onset` (above it): in full at or below the start, none at or above
/// the onset. A lagging joint's catch-up would otherwise cancel the barrier, and the ramp
/// starts early enough for the torque that reaches the arm, behind the low-pass, to have mostly
/// faded by the onset. Torque against the motion is never scaled; the law's Coriolis term is
/// part of `law` and fades with it.
pub(super) fn fade_push(
    law: &[f64; 7],
    dq: &[f64; 7],
    start: &[f64; 7],
    onset: &[f64; 7],
) -> [f64; 7] {
    std::array::from_fn(|i| {
        let speed = dq[i].abs();
        if law[i] * dq[i] > 0.0 && speed > start[i] {
            law[i] * ((onset[i] - speed) / (onset[i] - start[i])).clamp(0.0, 1.0)
        } else {
            law[i]
        }
    })
}

/// The barrier's gain per joint, Nm per rad/s: [`VELOCITY_BARRIER_GAIN`], lowered in proportion
/// where `torque_limits` clamps a joint below the default clamp (86 Nm on joints 1-4, 11.5 on
/// 5-7), so a lighter clamp does not leave a stiffer barrier to ring on the joint's inertia.
pub(super) fn barrier_gains(torque_limits: &[f64; 7]) -> [f64; 7] {
    std::array::from_fn(|i| VELOCITY_BARRIER_GAIN * (torque_limits[i] / TORQUE_LIMITS[i]).min(1.0))
}

/// The barrier's torque, Nm: `gains` per rad/s by which a joint's measured velocity exceeds
/// `onset`, opposing it and at most that joint's torque limit; zero below.
pub(super) fn velocity_barrier(
    dq: &[f64; 7],
    onset: &[f64; 7],
    gains: &[f64; 7],
    torque_limits: &[f64; 7],
) -> [f64; 7] {
    std::array::from_fn(|i| {
        let excess = dq[i].abs() - onset[i];
        if excess > 0.0 {
            (gains[i] * excess).min(torque_limits[i]).copysign(-dq[i])
        } else {
            0.0
        }
    })
}
