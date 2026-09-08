//! Online trajectory generation: a smooth 1 kHz command from a stream of stepped targets.
//!
//! [`Otg`] turns a target position that may change at any time -- every cycle, in bursts, or
//! not for seconds -- into a per-cycle position command whose velocity, acceleration and jerk
//! never exceed the limits it was built with, whose acceleration is continuous (the trajectory
//! is C2), and which reaches the target and then stays there exactly. It is the causal
//! alternative to a spline, which needs future knots a low-rate commander cannot supply, and to
//! a low-pass filter, whose peak speed grows with the size of the step. [`MultiOtg`] runs one
//! per axis, optionally synchronised; [`CartesianOtg`] is the three-axis one. No allocation.
//!
//! # Algorithm
//! Every call to [`Otg::step`] re-plans from the current state `(p, v, a)` to `(target, 0, 0)`
//! and then follows that plan for one cycle. The plan is the classical seven-segment,
//! jerk-limited profile: a time-optimal *velocity transfer* from `(v, a)` to a peak velocity
//! `v_p` with zero acceleration (jerk `+j_max` up to a peak acceleration, hold it, jerk
//! `-j_max` back to zero -- three segments with closed-form durations), a cruise at `v_p`, and
//! the mirror transfer from `v_p` to rest. The only free parameter is `v_p`: it is `±v_max`
//! with a cruise when the target is far enough, and otherwise a root of
//! `f(v_p) = target - p`, where `f` is the displacement of the two transfers. `f` is
//! increasing in `v_p` except for one hump next to `v_rd = v + a|a| / (2 j_max)`, the velocity
//! reached by ramping the acceleration straight to zero (an unsaturated reversal there costs
//! `2 sqrt(d / j_max)` of extra time at about `v_rd` for a change `d` of peak velocity), so
//! the domain is split at `v_rd` and `v_rd ± a_max² / j_max`, every piece whose ends bracket
//! the target is bisected, and the shortest plan found wins. A target closer than the braking
//! distance gives a root of the opposite sign: the profile passes the target, stops and comes
//! back, all within the limits and without a jerk spike. This is the profile structure of
//! Haschke, Weitnauer and Ritter, *On-line planning of time-optimal, jerk-limited
//! trajectories* (IROS 2008), evaluated one cycle at a time the way Ruckig (Berscheid and
//! Kröger, RSS 2021) does, whose four jerk patterns are the pieces above. The acceleration
//! along the plan is piecewise linear with slope `±j_max` or 0, so the trajectory is C2 and a
//! finite difference of the acceleration over any cycle length, including a 2 ms cycle after a
//! lost packet, stays within `j_max`; the velocity transfer keeps `v_rd` within `±v_max`,
//! which is what makes the velocity bound hold through the transient of every re-plan.
//!
//! The target velocity is always zero: the generator is for positional targets from a planner,
//! a vision loop or a hand, not for velocity tracking. The synchronisation in [`MultiOtg`]
//! stretches the faster axes to the slowest one's duration by lowering their peak velocity (a
//! second bisection, on the scale of `v_p`); an axis whose profile has no peak to lower (one
//! that is exactly braking to its target) keeps its minimum duration.
//!
//! # In a control loop
//! Three rules, learnt from a run on a real FER in which the first version of the bridge in
//! `examples/nonrealtime_commander.rs` was clamped by the rate limiter behind it and then
//! orbited at the velocity cap for twenty seconds. The limits are **per axis**: two axes at
//! full acceleration have a vector norm `sqrt 2` above it, so a budget that is a norm (which is
//! what `limit_rate_cartesian_pose` bounds) needs [`OtgLimits::per_axis_for_norm`]. Step **one
//! nominal cycle per command** (`DELTA_T`), not the measured period: the robot and the rate
//! limiter check every packet against a 1 ms budget, so a 2 ms step after a lost packet is a
//! doubled velocity to them. And **re-anchor on the robot's echo of the position** every
//! cycle with [`Otg::set_position`] (`O_T_EE_c`; not its twist, see [`Otg::set_state`]), so
//! that if anything behind the generator does alter a command, the next plan starts from
//! what was actually sent; a rate limiter that tracks a pose it has fallen behind has no
//! braking logic and never catches up.
//!
//! ```
//! use franka::otg::{Otg, OtgLimits};
//! let limits = OtgLimits { max_velocity: 0.3, max_acceleration: 0.5, max_jerk: 20.0 };
//! let mut otg = Otg::new(0.0, limits).unwrap();
//! otg.set_target(0.05).unwrap();
//! let mut t = 0.0f64;
//! while otg.position() != 0.05 {
//!     otg.step(0.001);
//!     t += 0.001;
//! }
//! assert!((t - 0.658).abs() < 0.01, "a 5 cm S-curve under these limits takes 0.658 s");
//! ```

use crate::error::{FrankaError, FrankaResult};

/// The per-axis limits of an [`Otg`]: all three must be finite and positive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OtgLimits {
    /// Maximum absolute velocity, in the position unit per second.
    pub max_velocity: f64,
    /// Maximum absolute acceleration, per second squared.
    pub max_acceleration: f64,
    /// Maximum absolute jerk, per second cubed.
    pub max_jerk: f64,
}

impl OtgLimits {
    /// Per-axis limits whose vector norm over `axes` axes never exceeds `self`: every limit
    /// divided by `sqrt(axes)`. What to build a [`MultiOtg`] with when the budget is a norm.
    pub fn per_axis_for_norm(self, axes: usize) -> Self {
        let scale = 1.0 / (axes.max(1) as f64).sqrt();
        Self {
            max_velocity: self.max_velocity * scale,
            max_acceleration: self.max_acceleration * scale,
            max_jerk: self.max_jerk * scale,
        }
    }
}

/// One constant-jerk piece of a plan.
#[derive(Debug, Clone, Copy, Default)]
struct Segment {
    jerk: f64,
    duration: f64,
}

/// A seven-segment plan from the current state to rest at the target: the transfer to the
/// peak velocity (three segments), the cruise, and the transfer to rest (three segments).
#[derive(Debug, Clone, Copy, Default)]
struct Plan {
    segments: [Segment; 7],
    peak_velocity: f64,
}

impl Plan {
    fn duration(&self) -> f64 {
        self.segments.iter().map(|s| s.duration).sum()
    }
}

/// Advances `(p, v, a)` by `t` seconds of constant `jerk`.
fn integrate(state: &mut (f64, f64, f64), jerk: f64, t: f64) {
    let (p, v, a) = *state;
    state.0 = p + v * t + a * t * t / 2.0 + jerk * t * t * t / 6.0;
    state.1 = v + a * t + jerk * t * t / 2.0;
    state.2 = a + jerk * t;
}

/// The time-optimal transfer from velocity `v` with acceleration `a` to velocity `v_goal`
/// with zero acceleration under `a_max` and `j_max`, as three constant-jerk segments.
fn transfer(v: f64, a: f64, v_goal: f64, a_max: f64, j_max: f64) -> [Segment; 3] {
    let dv = v_goal - v;
    // Ramping the acceleration straight to zero changes the velocity by a|a| / (2 j_max); if
    // that alone passes the goal, the transfer has to swing the acceleration the other way.
    let s = 1.0f64.copysign(dv - a * a.abs() / (2.0 * j_max));
    // In the frame where the transfer accelerates, dv >= a0|a0| / (2 j_max) holds.
    let (dv, a0) = (s * dv, s * a);
    let a_peak = (j_max * dv + 0.5 * a0 * a0).max(0.0).sqrt().min(a_max);
    let t1 = ((a_peak - a0) / j_max).max(0.0);
    let t3 = a_peak / j_max;
    let t2 = if a_peak > 0.0 {
        ((dv - (2.0 * a_peak * a_peak - a0 * a0) / (2.0 * j_max)) / a_peak).max(0.0)
    } else {
        0.0
    };
    [(s * j_max, t1), (0.0, t2), (-s * j_max, t3)]
        .map(|(jerk, duration)| Segment { jerk, duration })
}

/// A single-axis online trajectory generator; see the [module documentation](self).
#[derive(Debug, Clone, Copy)]
pub struct Otg {
    limits: OtgLimits,
    position: f64,
    velocity: f64,
    acceleration: f64,
    target: f64,
}

impl Otg {
    /// A generator at rest at `position`, whose target is `position`.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] if `position` is not finite or a limit is not finite
    /// and positive.
    pub fn new(position: f64, limits: OtgLimits) -> FrankaResult<Self> {
        let ok = |x: f64| x.is_finite() && x > 0.0;
        let valid = ok(limits.max_velocity) && ok(limits.max_acceleration) && ok(limits.max_jerk);
        if !(valid && position.is_finite()) {
            return Err(FrankaError::InvalidArgument(format!(
                "otg: limits must be finite and positive and the position finite, got \
                 {limits:?} at {position}"
            )));
        }
        Ok(Self {
            limits,
            position,
            velocity: 0.0,
            acceleration: 0.0,
            target: position,
        })
    }

    /// Sets the target position; the next [`step`](Self::step) re-plans towards it.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`], with the target unchanged, if `target` is not finite.
    pub fn set_target(&mut self, target: f64) -> FrankaResult<()> {
        if !target.is_finite() {
            return Err(FrankaError::InvalidArgument(format!(
                "otg: target must be finite, got {target}"
            )));
        }
        self.target = target;
        Ok(())
    }

    /// Re-anchors the generator on the position actually commanded -- the robot's echo of the
    /// last command -- keeping its own velocity, acceleration and target, so that whatever
    /// runs behind it can shape one command but never build up a lag it plans against.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`], with the state unchanged, if `position` is not finite.
    pub fn set_position(&mut self, position: f64) -> FrankaResult<()> {
        self.set_state(position, self.velocity, self.acceleration)
    }

    /// Sets the whole state; the target is kept. The velocity and acceleration are clamped
    /// into the limits. They must be the state at the *end* of the last cycle, as this
    /// generator's are: a finite difference such as the robot's `O_dP_EE_c` is the *mean* over
    /// the cycle, half an acceleration step behind, and re-anchoring on that every cycle
    /// throttles the plan to a crawl -- use [`set_position`](Self::set_position) with an echo.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`], with the state unchanged, if a value is not finite.
    pub fn set_state(
        &mut self,
        position: f64,
        velocity: f64,
        acceleration: f64,
    ) -> FrankaResult<()> {
        if !(position.is_finite() && velocity.is_finite() && acceleration.is_finite()) {
            return Err(FrankaError::InvalidArgument(format!(
                "otg: state must be finite, got {position}, {velocity}, {acceleration}"
            )));
        }
        let (v_max, a_max) = (self.limits.max_velocity, self.limits.max_acceleration);
        self.position = position;
        self.velocity = velocity.clamp(-v_max, v_max);
        self.acceleration = acceleration.clamp(-a_max, a_max);
        Ok(())
    }

    /// Puts the generator at rest at `position`, with the target there too.
    pub fn reset(&mut self, position: f64) {
        (self.position, self.velocity, self.acceleration) = (position, 0.0, 0.0);
        self.target = position;
    }

    /// Advances the state by `dt` seconds along the re-planned profile and returns the new
    /// position. A `dt` that is not positive and finite leaves the state untouched.
    pub fn step(&mut self, dt: f64) -> f64 {
        self.follow(&self.fastest(), dt)
    }

    /// The current position.
    pub fn position(&self) -> f64 {
        self.position
    }

    /// The current velocity.
    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    /// The current acceleration.
    pub fn acceleration(&self) -> f64 {
        self.acceleration
    }

    /// The current target.
    pub fn target(&self) -> f64 {
        self.target
    }

    /// The duration of the time-optimal profile from the current state to rest at the target;
    /// zero when at rest there.
    pub fn duration(&self) -> f64 {
        self.fastest().duration()
    }

    /// The plan through `peak_velocity`, and the displacement its two transfers leave for the
    /// cruise (negative if they alone pass the target, in which case the cruise is empty).
    fn plan(&self, peak_velocity: f64) -> (Plan, f64) {
        let (a_max, j_max) = (self.limits.max_acceleration, self.limits.max_jerk);
        let (v, a) = (self.velocity, self.acceleration);
        let mut plan = Plan {
            peak_velocity,
            ..Plan::default()
        };
        plan.segments[..3].copy_from_slice(&transfer(v, a, peak_velocity, a_max, j_max));
        plan.segments[4..].copy_from_slice(&transfer(peak_velocity, 0.0, 0.0, a_max, j_max));
        let mut state = (self.position, v, a);
        for segment in &plan.segments {
            integrate(&mut state, segment.jerk, segment.duration);
        }
        // Below a nanometre the residual is the integration's rounding, and divided by a peak
        // velocity near zero it would become a cruise of milliseconds; the snap at the end of
        // the plan absorbs it instead.
        let residual = self.target - state.0;
        if peak_velocity != 0.0 && residual.abs() >= 1e-9 {
            plan.segments[3].duration = (residual / peak_velocity).max(0.0);
        }
        (plan, residual)
    }

    /// The time-optimal plan: the fastest peak velocity whose profile lands on the target
    /// (see the module documentation for the pieces).
    fn fastest(&self) -> Plan {
        let OtgLimits {
            max_velocity: v_max,
            max_acceleration: a_max,
            max_jerk: j_max,
        } = self.limits;
        if self.position == self.target && self.velocity == 0.0 && self.acceleration == 0.0 {
            return Plan::default();
        }
        let (plan, residual) = self.plan(v_max);
        if residual >= 0.0 {
            return plan;
        }
        let (plan, residual) = self.plan(-v_max);
        if residual <= 0.0 {
            return plan;
        }
        let v_rd = self.velocity + self.acceleration * self.acceleration.abs() / (2.0 * j_max);
        let width = a_max * a_max / j_max;
        let bounds =
            [-v_max, v_rd - width, v_rd, v_rd + width, v_max].map(|b| b.clamp(-v_max, v_max));
        let mut best: Option<Plan> = None;
        for pair in bounds.windows(2) {
            let (mut lo, mut hi) = (pair[0], pair[1]);
            let (r_lo, r_hi) = (self.plan(lo).1, self.plan(hi).1);
            if lo >= hi || r_lo.signum() == r_hi.signum() {
                continue;
            }
            while hi - lo > 1e-10 * v_max {
                let mid = 0.5 * (lo + hi);
                if self.plan(mid).1.signum() == r_lo.signum() {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let candidate = self.plan(0.5 * (lo + hi)).0;
            if best.is_none_or(|b| candidate.duration() < b.duration()) {
                best = Some(candidate);
            }
        }
        best.unwrap_or(plan)
    }

    /// The plan through a peak velocity scaled down from `plan`'s so that it lasts `duration`
    /// seconds (bisection on the scale; the duration grows without bound as the scale goes to
    /// zero). Returns `plan` itself when it has no peak to lower.
    fn stretched(&self, plan: Plan, duration: f64) -> Plan {
        if plan.peak_velocity == 0.0 || plan.duration() >= duration {
            return plan;
        }
        // A scaled peak is only usable if its transfers do not pass the target (a cruise of
        // non-negative length brings the plan onto it); near the cusp of `f` they can, and
        // such a plan would overshoot. Treat those like plans that are too short.
        let long_enough = |scale: f64| {
            let (candidate, residual) = self.plan(scale * plan.peak_velocity);
            residual * plan.peak_velocity >= 0.0 && candidate.duration() >= duration
        };
        let (mut lo, mut hi) = (1e-9, 1.0);
        if !long_enough(lo) {
            return plan;
        }
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if long_enough(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        self.plan(lo * plan.peak_velocity).0
    }

    /// Follows `plan` for `dt` seconds, snapping to rest at the target when the plan ends
    /// within the cycle.
    fn follow(&mut self, plan: &Plan, dt: f64) -> f64 {
        if !(dt > 0.0 && dt.is_finite()) {
            return self.position;
        }
        if plan.duration() <= dt {
            self.reset(self.target);
            return self.position;
        }
        let mut state = (self.position, self.velocity, self.acceleration);
        let mut left = dt;
        for segment in &plan.segments {
            let t = segment.duration.min(left);
            integrate(&mut state, segment.jerk, t);
            left -= t;
            if left <= 0.0 {
                break;
            }
        }
        let (v_max, a_max) = (self.limits.max_velocity, self.limits.max_acceleration);
        // The plan respects the limits; this only trims rounding in the last bit.
        self.position = state.0;
        self.velocity = state.1.clamp(-v_max, v_max);
        self.acceleration = state.2.clamp(-a_max, a_max);
        self.position
    }
}

/// `N` [`Otg`]s with the same limits, stepped together, optionally synchronised so that all
/// axes arrive at their targets at the same time (the faster axes are slowed down).
#[derive(Debug, Clone, Copy)]
pub struct MultiOtg<const N: usize> {
    axes: [Otg; N],
    synchronize: bool,
}

/// A three-axis [`MultiOtg`] for a Cartesian position.
pub type CartesianOtg = MultiOtg<3>;

impl<const N: usize> MultiOtg<N> {
    /// A generator at rest at `position`, whose target is `position`.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] if a coordinate is not finite or a limit is not finite
    /// and positive.
    pub fn new(position: [f64; N], limits: OtgLimits, synchronize: bool) -> FrankaResult<Self> {
        let mut axes = [Otg::new(0.0, limits)?; N];
        for (axis, p) in axes.iter_mut().zip(position) {
            *axis = Otg::new(p, limits)?;
        }
        Ok(Self { axes, synchronize })
    }

    /// Sets the target position; the next [`step`](Self::step) re-plans towards it.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`], with the target unchanged, if a coordinate is not
    /// finite.
    pub fn set_target(&mut self, target: [f64; N]) -> FrankaResult<()> {
        if target.iter().any(|t| !t.is_finite()) {
            return Err(FrankaError::InvalidArgument(format!(
                "otg: target must be finite, got {target:?}"
            )));
        }
        for (axis, t) in self.axes.iter_mut().zip(target) {
            axis.target = t;
        }
        Ok(())
    }

    /// Advances every axis by `dt` seconds and returns the new position. A `dt` that is not
    /// positive and finite leaves the state untouched.
    pub fn step(&mut self, dt: f64) -> [f64; N] {
        let mut plans = self.axes.map(|axis| axis.fastest());
        let slowest = plans.iter().map(Plan::duration).fold(0.0, f64::max);
        if self.synchronize && slowest > dt {
            for (axis, plan) in self.axes.iter().zip(plans.iter_mut()) {
                *plan = axis.stretched(*plan, slowest);
            }
        }
        let mut position = [0.0; N];
        for ((axis, plan), p) in self.axes.iter_mut().zip(&plans).zip(position.iter_mut()) {
            *p = axis.follow(plan, dt);
        }
        position
    }

    /// The current position.
    pub fn position(&self) -> [f64; N] {
        self.axes.map(|axis| axis.position)
    }

    /// The per-axis generators, for their velocities, accelerations and targets.
    pub fn axes(&self) -> &[Otg; N] {
        &self.axes
    }

    /// Re-anchors every axis on the position actually commanded; see [`Otg::set_position`].
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`], with the state unchanged, if a coordinate is not
    /// finite.
    pub fn set_position(&mut self, position: [f64; N]) -> FrankaResult<()> {
        let (v, a) = (
            self.axes.map(|x| x.velocity),
            self.axes.map(|x| x.acceleration),
        );
        self.set_state(position, v, a)
    }

    /// Sets every axis's state; see [`Otg::set_state`] for what the velocity and
    /// acceleration must be.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`], with the state unchanged, if a value is not finite.
    pub fn set_state(&mut self, p: [f64; N], v: [f64; N], a: [f64; N]) -> FrankaResult<()> {
        if p.iter().chain(&v).chain(&a).any(|x| !x.is_finite()) {
            return Err(FrankaError::InvalidArgument(format!(
                "otg: state must be finite, got {p:?}, {v:?}, {a:?}"
            )));
        }
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_state(p[i], v[i], a[i])?;
        }
        Ok(())
    }

    /// Puts every axis at rest at `position`, with the target there too.
    pub fn reset(&mut self, position: [f64; N]) {
        for (axis, p) in self.axes.iter_mut().zip(position) {
            axis.reset(p);
        }
    }
}

#[cfg(test)]
mod tests;
