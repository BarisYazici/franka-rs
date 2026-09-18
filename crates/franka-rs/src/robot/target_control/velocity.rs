//! The torque backend's joint velocity envelope: the cap on the joint goal's step, which keeps
//! what the loop commands under a fraction of the arm's velocity limits; the fade, which takes
//! the law's push off a joint measured faster than the cap (or [`FADE_BAND`] under the barrier's
//! onset), so that catching up on its lag does not drive it past that onset; and the barrier on
//! the measured velocity, which damps what the two cannot: spring-back and overshoot.

/// Scales `to - from` as a whole so that no joint moves further than `max_step`, which keeps
/// the step's direction in joint space; returns the scale, 1 when every joint is within its
/// bound.
pub(super) fn cap_step(from: &[f64; 7], to: &mut [f64; 7], max_step: &[f64; 7]) -> f64 {
    let scale = to
        .iter()
        .zip(from)
        .zip(max_step)
        .fold(1.0f64, |scale, ((to, from), max)| {
            let step = (to - from).abs();
            if step > *max {
                scale.min(max / step)
            } else {
                scale
            }
        });
    if scale < 1.0 {
        for (q, from) in to.iter_mut().zip(from) {
            *q = from + scale * (*q - from);
        }
    }
    scale
}

/// Nm per rad/s of the velocity barrier: the damping a joint meets beyond
/// [`ImpedanceOptions::velocity_barrier_fraction`](super::ImpedanceOptions::velocity_barrier_fraction)
/// of its limit. Stable at 1 kHz behind the 100 Hz low-pass while the law's own joint damping
/// plus this stays under the joint's effective inertia per millisecond; see *The impedance
/// backend* in the book.
pub const VELOCITY_BARRIER_GAIN: f64 = 20.0;

/// The narrowest band, as a fraction of a joint's velocity limit, over which the law's push
/// along the joint's motion fades out under the barrier's onset: the fade starts at the cap or
/// this far under the onset, whichever is lower. The fade adds up to `|law| / (FADE_BAND × limit)`
/// Nm per rad/s to the loop's gain there, about 30 on an FER's wrist at its 11.5 Nm clamp, which
/// the loop settles on 0.03 kg m² and more; a fade from a cap of 0.8 rings there.
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

/// The barrier's torque, Nm: [`VELOCITY_BARRIER_GAIN`] per rad/s by which a joint's measured
/// velocity exceeds `onset`, opposing it and at most that joint's torque limit; zero below.
pub(super) fn velocity_barrier(
    dq: &[f64; 7],
    onset: &[f64; 7],
    torque_limits: &[f64; 7],
) -> [f64; 7] {
    std::array::from_fn(|i| {
        let excess = dq[i].abs() - onset[i];
        if excess > 0.0 {
            (VELOCITY_BARRIER_GAIN * excess)
                .min(torque_limits[i])
                .copysign(-dq[i])
        } else {
            0.0
        }
    })
}
