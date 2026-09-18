//! Differential inverse kinematics of the Cartesian torque backend: from the previous joint
//! goal, a few damped-least-squares steps toward the desired pose, the orientation weighed by
//! [`IkOptions::rotation_weight`], each solved inside the cycle's box ([`active_set`]): the
//! joint velocity limit, and the braking envelope toward the position limits less the margin.
//! A posture bias rides in the free joints' nullspace, clamped away from the limits and faded
//! out near them. The joint goal so follows the pose stream continuously and, where the pose
//! is unreachable, singular or behind a limit, lags instead of jumping. While a limit stalls the
//! goal, position comes first, to within micrometres ([`priority`]).

pub(super) mod active_set;
mod bounds;
mod priority;
pub(super) mod stall;

use std::sync::Arc;

use nalgebra::{Matrix3, Vector3};

use super::position::JointLimits;
use super::rotation::{log, rotation_of, translation_of};
use crate::error::{FrankaError, FrankaResult};
use crate::model::{Frame, Model};
use active_set::{solve, Bias, Jacobian, Joints, Task};
use bounds::Bounds;
#[cfg(test)]
pub(super) use bounds::{AS_RUN_LIMIT_MARGIN, POSTURE_LIMIT_DISTANCE};
use priority::Priority;
#[cfg(test)]
pub(super) use priority::PRIORITY_RAMP_CYCLES;
use stall::Stall;

/// Options of the Cartesian backend's differential inverse kinematics: every cycle, from the
/// previous joint goal, up to `iterations` damped-least-squares steps toward the pose inside
/// the cycle's joint box, with a posture bias through the nullspace.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IkOptions {
    /// `λ` of the damped least squares `J̃ᵀ (J̃ J̃ᵀ + λ² I)⁻¹` on the weighted Jacobian
    /// `J̃ = W J`, its rotation rows scaled by `rotation_weight`. Default 0.05.
    pub damping: f64,
    /// Gain, 1/s, of the nullspace pull toward the posture; the pull moves no joint faster
    /// than [`MAX_POSTURE_RATE`] whatever the gain and the distance. Default 1.0.
    pub nullspace_gain: f64,
    /// Steps per cycle. Default 3.
    pub iterations: u32,
    /// Weighted residual below which a cycle stops iterating. Default 1e-6.
    pub tolerance: f64,
    /// m per rad: the orientation error's weight against the position error's in the task, the
    /// residual and `tolerance`. At 0.1, 1 mm costs as much as 10 mrad, so a pose the joints
    /// cannot reach gives up orientation before position; while a joint limit stalls the goal,
    /// position comes first, to within micrometres. Within (0, 1]. Default 0.1.
    pub rotation_weight: f64,
}

/// The most, rad/s, the posture bias moves any joint: the pull `nullspace_gain × distance`
/// toward a far posture is scaled down as a whole to this rate, so a posture 2 rad away is
/// approached at 0.5 rad/s, not 2.
pub const MAX_POSTURE_RATE: f64 = 0.5;

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions {
            damping: 0.05,
            nullspace_gain: 1.0,
            iterations: 3,
            tolerance: 1e-6,
            rotation_weight: 0.1,
        }
    }
}

impl IkOptions {
    /// # Errors
    /// [`FrankaError::InvalidArgument`] unless it has a finite, positive damping, finite and
    /// non-negative nullspace_gain and tolerance, a rotation_weight within (0, 1], and at least
    /// one iteration.
    pub fn validate(&self) -> FrankaResult<()> {
        let non_negative = |x: f64| x.is_finite() && x >= 0.0;
        let valid = self.damping.is_finite()
            && self.damping > 0.0
            && non_negative(self.nullspace_gain)
            && non_negative(self.tolerance)
            && self.rotation_weight > 0.0
            && self.rotation_weight <= 1.0
            && self.iterations > 0;
        if !valid {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: ik needs a finite, positive damping, finite and non-negative \
                 nullspace_gain and tolerance, a rotation_weight within (0, 1], and at least \
                 one iteration, got {self:?}"
            )));
        }
        Ok(())
    }
}

/// See the [module documentation](self).
pub(super) struct Ik {
    model: Arc<Model>,
    options: IkOptions,
    bounds: Bounds,
    f_t_ee: [f64; 16],
    ee_t_k: [f64; 16],
    q: [f64; 7],
    /// The weighted Jacobian at `q`, the next cycle's first.
    jacobian: Jacobian,
    act: [i8; 7],
    /// The position stage's active set, warm across its solves.
    position_act: [i8; 7],
    priority: Priority,
    pinned: [i8; 7],
    stall: Stall,
    passes: u32,
    /// The largest raw step norm, rad, over the cycle's solves, before its box clip, and what
    /// that same solve's clip took off. The two are one solve's pair, not two maxima.
    step_norm: f64,
    clipped_norm: f64,
    #[cfg(test)]
    fallbacks: u64,
    cap_scale: f64,
}

impl Ik {
    pub(super) fn new(
        model: Arc<Model>,
        options: IkOptions,
        limits: JointLimits,
        q0: [f64; 7],
        f_t_ee: [f64; 16],
        ee_t_k: [f64; 16],
    ) -> Self {
        let bounds = Bounds::Guarded(limits);
        Ik::with_bounds(model, options, bounds, q0, f_t_ee, ee_t_k)
    }

    /// The solver as it ran before the guard: the start clamped into `limits` inset by
    /// [`AS_RUN_LIMIT_MARGIN`], each step clipped there, the whole move scaled under
    /// `max_velocity` (rad/s), the orientation unweighted, the raw posture, no pins, no stall.
    #[cfg(test)]
    pub(super) fn as_run(
        model: Arc<Model>,
        options: IkOptions,
        limits: ([f64; 7], [f64; 7]),
        max_velocity: [f64; 7],
        q0: [f64; 7],
        f_t_ee: [f64; 16],
        ee_t_k: [f64; 16],
    ) -> Self {
        let (bounds, q0) = Bounds::as_run(limits, max_velocity, q0);
        let options = IkOptions {
            rotation_weight: 1.0,
            ..options
        };
        Ik::with_bounds(model, options, bounds, q0, f_t_ee, ee_t_k)
    }

    fn with_bounds(
        model: Arc<Model>,
        options: IkOptions,
        bounds: Bounds,
        q: [f64; 7],
        f_t_ee: [f64; 16],
        ee_t_k: [f64; 16],
    ) -> Self {
        let mut ik = Ik {
            model,
            options,
            bounds,
            f_t_ee,
            ee_t_k,
            q,
            jacobian: Jacobian::zeros(),
            act: [0; 7],
            position_act: [0; 7],
            priority: Priority::default(),
            pinned: [0; 7],
            stall: Stall::default(),
            passes: 0,
            step_norm: 0.0,
            clipped_norm: 0.0,
            #[cfg(test)]
            fallbacks: 0,
            cap_scale: 1.0,
        };
        ik.jacobian = ik.jacobian_at(&q);
        ik
    }

    /// One cycle toward `pose` (column-major, as `O_T_EE`); returns the solution and its
    /// weighted residual. The first step always runs and carries the posture bias (a rate,
    /// integrated over `dt` once per call); the remaining steps refine the pose until the
    /// residual is below `tolerance`. A non-finite solve keeps the previous goal.
    pub(super) fn step(
        &mut self,
        pose: &[f64; 16],
        posture: &[f64; 7],
        dt: f64,
    ) -> ([f64; 7], f64) {
        let target = (Vector3::from(translation_of(pose)), rotation_of(pose));
        let from = Joints::from(self.q);
        let gain = self.options.nullspace_gain * dt;
        let cycle = self.bounds.cycle(&self.q, posture, gain, dt);
        self.priority.update(&self.stall);
        let saved_act = (self.act, self.position_act);
        let (mut q, mut j) = (from, self.jacobian);
        let mut error = None;
        self.passes = 0;
        (self.step_norm, self.clipped_norm) = (0.0, 0.0);
        for iteration in 0..self.options.iterations {
            let e = self.error(&q, &target);
            if iteration > 0 && e.norm() < self.options.tolerance {
                error = Some(e);
                break;
            }
            if iteration > 0 {
                j = self.jacobian_at(&q.into());
            }
            let lo = Joints::from(cycle.bounds.lower) - (q - from);
            let hi = Joints::from(cycle.bounds.upper) - (q - from);
            let bias = (iteration == 0).then_some(Bias {
                rate: cycle.posture,
                cap: MAX_POSTURE_RATE * dt,
            });
            match self.prioritised(&j, &e, (&lo, &hi), bias, cycle.passes) {
                Some(d) => q += d,
                None => {
                    // The step is undone whole: goal, Jacobian, both active sets and the
                    // passes and step norms they cost, none of which describe the goal that
                    // is kept.
                    (q, j) = (from, self.jacobian);
                    (self.act, self.position_act) = saved_act;
                    self.passes = 0;
                    (self.step_norm, self.clipped_norm) = (0.0, 0.0);
                    #[cfg(test)]
                    {
                        self.fallbacks += 1;
                    }
                    error = Some(self.error(&q, &target));
                    break;
                }
            }
        }
        let mut goal: [f64; 7] = q.into();
        self.cap_scale = self.bounds.backstop(&self.q, &mut goal, &cycle);
        let e = match error {
            Some(e) if self.cap_scale == 1.0 => e,
            _ => self.error(&goal.into(), &target),
        };
        if goal != self.q {
            j = self.jacobian_at(&goal);
        }
        (self.q, self.jacobian) = (goal, j);
        self.pinned = std::array::from_fn(|i| cycle.bounds.pin(i, self.act[i]));
        self.stall
            .update(&j, &e, &self.pinned, self.options.damping);
        (goal, e.norm())
    }

    /// One iteration's step on `j`, `e` in `lo..hi`: the weighted solve, blended toward position
    /// first by [`Priority`]; `None` when a solve is not finite.
    fn prioritised(
        &mut self,
        j: &Jacobian,
        e: &Task,
        (lo, hi): (&Joints, &Joints),
        bias: Option<Bias>,
        passes: u32,
    ) -> Option<Joints> {
        let damping = self.options.damping;
        let mut run = |j: &Jacobian, e: &Task, lo: &Joints, hi: &Joints, bias, act: &mut _| {
            let solution = solve(j, e, (lo, hi), bias, act, damping, passes)?;
            self.passes += solution.passes;
            if solution.step > self.step_norm {
                (self.step_norm, self.clipped_norm) = (solution.step, solution.clipped);
            }
            #[cfg(test)]
            {
                self.fallbacks += u64::from(!solution.converged);
            }
            Some(solution.d)
        };
        if self.priority.blend == 0.0 {
            let d = run(j, e, lo, hi, bias, &mut self.act);
            // The position stage starts from the weighted solve's pins when the blend sets in.
            self.position_act = self.act;
            return d;
        }
        let (jp, ep) = Priority::position(j, e);
        let shift = run(&jp, &ep, lo, hi, None, &mut self.position_act)? * self.priority.blend;
        let (jr, er) = self.priority.held(j, e, &shift);
        let d = run(&jr, &er, &(lo - shift), &(hi - shift), bias, &mut self.act)?;
        Some(d + shift)
    }

    /// The pins of the last step per joint: 0 free, ∓1 on the lower / upper position bound,
    /// ∓2 on the velocity bound.
    pub(super) fn pinned(&self) -> [i8; 7] {
        self.pinned
    }

    /// The stall flag and the pins' pressures after the last step.
    pub(super) fn stall(&self) -> Stall {
        self.stall
    }

    /// Active-set passes of the last step, over its iterations.
    pub(super) fn passes(&self) -> u32 {
        self.passes
    }

    /// The last step's largest raw solve, rad: its norm before the box clip, and what the clip
    /// took off it. While the blend is on, each iteration solves twice and the position stage's
    /// norm is its own, before the blend scales it down.
    pub(super) fn step_norms(&self) -> (f64, f64) {
        (self.step_norm, self.clipped_norm)
    }

    /// Solves that were not finite or did not converge, since the start.
    #[cfg(test)]
    pub(super) fn fallbacks(&self) -> u64 {
        self.fallbacks
    }

    /// The blend toward position first of the last step: 0 the weighted solve, 1 position first.
    pub(super) fn priority(&self) -> f64 {
        self.priority.blend
    }

    /// Sets the cycles the blend takes, [`priority::PRIORITY_RAMP_CYCLES`] by default; 0 holds
    /// it at the weighted solve.
    #[cfg(test)]
    pub(super) fn set_priority_ramp(&mut self, cycles: u32) {
        self.priority.set_ramp(cycles);
    }

    /// The scale the backstop cut the last [`step`](Self::step) by; 1 when it did not.
    pub(super) fn cap_scale(&self) -> f64 {
        self.cap_scale
    }

    /// Takes the end effector and stiffness frames, as `Robot::set_ee` and `set_k` may change them
    /// during a session; the `F_T_EE` it had when they changed.
    pub(super) fn set_frames(
        &mut self,
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> Option<[f64; 16]> {
        let bits = |t: &[f64; 16]| t.map(f64::to_bits);
        if bits(&self.f_t_ee) == bits(f_t_ee) && bits(&self.ee_t_k) == bits(ee_t_k) {
            return None;
        }
        let old = std::mem::replace(&mut self.f_t_ee, *f_t_ee);
        self.ee_t_k = *ee_t_k;
        self.jacobian = self.jacobian_at(&self.q);
        Some(old)
    }

    /// The end effector's pose at `q`, column-major.
    pub(super) fn pose_of(&self, q: &[f64; 7]) -> [f64; 16] {
        self.model
            .pose_q(Frame::EndEffector, q, &self.f_t_ee, &self.ee_t_k)
    }

    /// The solution of the last step, where the next one starts.
    pub(super) fn q(&self) -> [f64; 7] {
        self.q
    }

    /// `W J`, base frame.
    fn jacobian_at(&self, q: &[f64; 7]) -> Jacobian {
        let mut j = Jacobian::from_column_slice(&self.model.zero_jacobian_q(
            Frame::EndEffector,
            q,
            &self.f_t_ee,
            &self.ee_t_k,
        ));
        j.fixed_rows_mut::<3>(3)
            .scale_mut(self.options.rotation_weight);
        j
    }

    /// `W [p_des - p(q); log(R_des R(q)ᵀ)]`, base frame.
    fn error(&self, q: &Joints, (p_des, r_des): &(Vector3<f64>, Matrix3<f64>)) -> Task {
        let fk = self.pose_of(&(*q).into());
        let p = p_des - Vector3::from(translation_of(&fk));
        let turn =
            log(&(r_des * rotation_of(&fk).transpose())).map(|x| x * self.options.rotation_weight);
        Task::new(p[0], p[1], p[2], turn[0], turn[1], turn[2])
    }
}
