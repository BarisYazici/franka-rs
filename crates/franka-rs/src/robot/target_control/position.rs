//! The joint position limits of the torque backend: the box a cycle's joint goal step is kept
//! in, under the arm's velocity limit and the braking profile `E` of [`super::envelope`].

pub(super) use super::envelope::VelocityLimit;
use super::velocity::fade_start;
use crate::rate_limiting::DELTA_T;

/// Default of the distance, rad, the joint goal keeps from the joint position limits.
pub(super) const DEFAULT_JOINT_POSITION_MARGIN: f64 = 0.05;

/// The joint position limits the goal is kept inside, and how it may approach them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct JointLimits {
    /// (lower, upper), rad.
    pub position: ([f64; 7], [f64; 7]),
    /// How far, rad, inside `position` the goal stops.
    pub margin: f64,
    /// `f`: the fraction of the velocity limit and of `E` the goal may move at.
    pub fraction: f64,
    pub velocity: VelocityLimit,
}

/// A cycle's bounds on the step `δ = q − from`, `lower ≤ 0 ≤ upper`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CycleBox {
    pub lower: [f64; 7],
    pub upper: [f64; 7],
    /// Per side, whether the bound is the braking envelope's rather than the velocity limit's.
    pub position_lower: [bool; 7],
    pub position_upper: [bool; 7],
    /// The velocity limit's bounds alone, `−f·v·Δt` and `f·v·Δt`: the backstop's.
    pub velocity_lower: [f64; 7],
    pub velocity_upper: [f64; 7],
}

impl CycleBox {
    /// The pin of joint `i` on `side` (−1 lower, 0 free, +1 upper): 0 free, ∓1 on a position
    /// bound, ∓2 on a velocity bound.
    pub(super) fn pin(&self, i: usize, side: i8) -> i8 {
        let position = if side < 0 {
            self.position_lower[i]
        } else {
            self.position_upper[i]
        };
        side.signum() * if position { 1 } else { 2 }
    }
}

/// rad: the ramp over which a goal comes to rest on its margin. The braking box's step there is
/// 0.9 % (joint 4) to 3.4 % (wrists) of the distance per cycle, so the rest rises by at most 0.034
/// a cycle whatever the width; on a recorded stall 1e-5 chattered least of 1e-5 to 3e-4, and
/// 1e-3 parks the goal short of its margin.
const REST_BAND: f64 = 1e-5;

/// rad: the band inside the margin over which the law's torque toward a joint limit fades out,
/// from full at the margin to none at the position barrier's onset, `margin − POSITION_FADE_BAND`
/// from the limit. Inside that onset a spring pushes the joint out.
pub const POSITION_FADE_BAND: f64 = 0.02;

/// 1/rad: the position barrier's stiffness as a multiple of the joint's torque clamp, `K_i =
/// POSITION_BARRIER_STIFFNESS × torque_limits_i`: 1075 Nm/rad at the 86 Nm clamp, 144 at 11.5.
pub const POSITION_BARRIER_STIFFNESS: f64 = 12.5;

/// Cycles from the loop's start over which the spring ramps in: an arm started inside the
/// barrier eases out instead of being kicked.
const SPRING_RAMP_CYCLES: u32 = 500;

/// The spring's share, 0 to 1, `cycles` after the loop's start.
pub(super) fn spring_ramp(cycles: u32) -> f64 {
    f64::from(cycles.min(SPRING_RAMP_CYCLES)) / f64::from(SPRING_RAMP_CYCLES)
}

impl JointLimits {
    /// The box of a step `u` from `from` over `dt`: toward each side at most
    /// `min(f·v·dt, max(0, min(x, f·E(x)·dt)))`, `x` the distance to that limit less the margin.
    /// Both limits are taken where the step ends, `u ≤ f·v(from ± u)·dt` and `u ≤ f·E(x − u)·dt`:
    /// each falls as `u` grows, so one more evaluation at the end of the step to the start's
    /// limit gives a step under the limit at its own end, short of the largest by 0.12 % at `f`
    /// 0.7, under 0.25 % at any `f`. A velocity limit that does not depend on the position keeps
    /// its term at `from`.
    pub(super) fn cycle_box(&self, from: &[f64; 7], dt: f64) -> CycleBox {
        let flat = self.velocity.flat();
        let (lower, upper) = self.position;
        let f = self.fraction;
        let start = self.velocity.at(from);
        // The velocity limits where the start's velocity step ends, per side.
        let end = self.velocity.position_dependent().then(|| {
            let toward = |sign: f64, speed: &[f64; 7]| {
                std::array::from_fn(|i| from[i] + sign * f * speed[i] * dt)
            };
            (
                self.velocity.at(&toward(-1.0, &start.0)).0,
                self.velocity.at(&toward(1.0, &start.1)).1,
            )
        });
        let (toward_lower, toward_upper) = end.unwrap_or(start);
        let side = |i: usize, distance: f64, speed: f64| {
            let x = distance - self.margin;
            let velocity = f * speed * dt;
            let braking = f * self.velocity.braking(i, x, flat[i]) * dt;
            let braking = f * self.velocity.braking(i, x - braking, flat[i]) * dt;
            let bound = velocity.min(x.min(braking).max(0.0));
            (bound, velocity, braking < velocity - 1e-12)
        };
        let mut b = CycleBox {
            lower: [0.0; 7],
            upper: [0.0; 7],
            position_lower: [false; 7],
            position_upper: [false; 7],
            velocity_lower: [0.0; 7],
            velocity_upper: [0.0; 7],
        };
        for i in 0..7 {
            let (bound, velocity, position) = side(i, from[i] - lower[i], toward_lower[i]);
            (b.lower[i], b.velocity_lower[i], b.position_lower[i]) = (-bound, -velocity, position);
            let (bound, velocity, position) = side(i, upper[i] - from[i], toward_upper[i]);
            (b.upper[i], b.velocity_upper[i], b.position_upper[i]) = (bound, velocity, position);
        }
        b
    }

    /// How far the goal `q` of the `joints` pinned on a position bound (∓1) rests on that side's
    /// margin, the largest of them: 1 on it, falling linearly to 0 [`REST_BAND`] before it.
    pub(super) fn rest(&self, q: &[f64; 7], joints: &[i8; 7]) -> f64 {
        let (lower, upper) = self.position;
        (0..7)
            .map(|i| match joints[i] {
                -1 => q[i] - lower[i] - self.margin,
                1 => upper[i] - q[i] - self.margin,
                _ => f64::INFINITY,
            })
            .map(|x| (1.0 - x / REST_BAND).clamp(0.0, 1.0))
            .fold(0.0, f64::max)
    }

    /// The distances, rad, of `q` to its lower and its upper limit.
    fn distances(&self, i: usize, q: f64) -> (f64, f64) {
        (q - self.position.0[i], self.position.1[i] - q)
    }

    /// Where the barrier's onset lies, `b = margin − POSITION_FADE_BAND` from a limit.
    fn onset_distance(&self) -> f64 {
        self.margin - POSITION_FADE_BAND
    }

    /// The velocity envelope at measured `q` toward the side `dq` points to, rad/s per joint: the
    /// fade's start and the barrier's onset. The onset is `barrier_fraction` of the arm's limit
    /// there, or of the braking envelope to the barrier's onset if lower; the fade starts at
    /// [`fade_start`] of `fraction` of the limit, or of the envelope to the margin if lower.
    /// Each is the lower of its value at `q` and at `q + dq·Δt`: the next state's velocity is
    /// checked against the limit where that state is, and the fade's start is not monotone in
    /// the limit.
    pub(super) fn envelope(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        barrier_fraction: f64,
    ) -> ([f64; 7], [f64; 7]) {
        let now = self.envelope_at(q, dq, barrier_fraction);
        let next = std::array::from_fn(|i| q[i] + dq[i] * DELTA_T);
        let (start, onset) = self.envelope_at(&next, dq, barrier_fraction);
        (
            std::array::from_fn(|i| now.0[i].min(start[i])),
            std::array::from_fn(|i| now.1[i].min(onset[i])),
        )
    }

    fn envelope_at(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        barrier_fraction: f64,
    ) -> ([f64; 7], [f64; 7]) {
        let flat = self.velocity.flat();
        let (toward_lower, toward_upper) = self.velocity.at(q);
        let (mut start, mut onset) = ([0.0; 7], [0.0; 7]);
        for i in 0..7 {
            let (lower, upper) = self.distances(i, q[i]);
            let (s, v) = if dq[i] < 0.0 {
                (lower, toward_lower[i])
            } else {
                (upper, toward_upper[i])
            };
            let braking = |x: f64| v.min(self.velocity.braking(i, x, flat[i]));
            onset[i] = barrier_fraction * braking(s - self.onset_distance());
            start[i] = fade_start(v, self.fraction * braking(s - self.margin), onset[i]);
        }
        (start, onset)
    }

    /// `torque` with every joint's torque toward a limit faded linearly from full at the margin
    /// to none at the barrier's onset; torque away from it is never scaled.
    pub(super) fn fade_toward(&self, torque: &[f64; 7], q: &[f64; 7]) -> [f64; 7] {
        let b = self.onset_distance();
        std::array::from_fn(|i| {
            let (lower, upper) = self.distances(i, q[i]);
            let s = if torque[i] < 0.0 { lower } else { upper };
            torque[i] * ((s - b) / (self.margin - b)).clamp(0.0, 1.0)
        })
    }

    /// The spring inside the barrier's onset, Nm per joint: `ramp × K_i × depth` out of the limit,
    /// at most the joint's torque clamp, `K_i` [`POSITION_BARRIER_STIFFNESS`] × that clamp.
    pub(super) fn spring(&self, q: &[f64; 7], torque_limits: &[f64; 7], ramp: f64) -> [f64; 7] {
        let b = self.onset_distance();
        std::array::from_fn(|i| {
            let push = |s: f64| {
                let depth = (b - s).max(0.0);
                (ramp * POSITION_BARRIER_STIFFNESS * torque_limits[i] * depth).min(torque_limits[i])
            };
            let (lower, upper) = self.distances(i, q[i]);
            push(lower) - push(upper)
        })
    }
}
