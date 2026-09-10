//! Differential inverse kinematics of the Cartesian torque backend: from the previous joint
//! goal, a few damped-least-squares steps toward the desired pose, a posture bias through the
//! nullspace, a cap on the step and a clamp inside the joint position limits. The joint goal
//! so follows the pose stream continuously and, where the pose is unreachable or singular,
//! lags instead of jumping.

use std::sync::Arc;

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use super::rotation::{log, rotation_of, translation_of};
use crate::error::{FrankaError, FrankaResult};
use crate::model::{Frame, Model};

/// Options of the Cartesian backend's differential inverse kinematics: every cycle, from the
/// previous joint goal, up to `iterations` damped-least-squares steps toward the pose, a
/// posture bias through the nullspace, the step capped at `max_step` and clamped inside the
/// joint limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IkOptions {
    /// `λ` of the damped least squares `Jᵀ (J Jᵀ + λ² I)⁻¹`. Default 0.05.
    pub damping: f64,
    /// Gain, 1/s, of the nullspace pull toward the posture; the pull moves no joint faster
    /// than [`MAX_POSTURE_RATE`] whatever the gain and the distance. Default 1.0.
    pub nullspace_gain: f64,
    /// Steps per cycle. Default 3.
    pub iterations: u32,
    /// Residual (m plus rad, one norm) below which a cycle stops iterating. Default 1e-6.
    pub tolerance: f64,
    /// How far, rad, inside the joint position limits the solution is kept, its start (the
    /// measured configuration) included. Default 0.02.
    pub limit_margin: f64,
    /// The most, rad, any joint of the solution moves in one cycle: a larger step is scaled
    /// down as a whole, so an unreachable or singular pose is approached at a bounded rate
    /// instead of jumped at. Default 0.01 (10 rad/s).
    pub max_step: f64,
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
            limit_margin: 0.02,
            max_step: 0.01,
        }
    }
}

impl IkOptions {
    /// # Errors
    /// [`FrankaError::InvalidArgument`] unless `damping` and `max_step` are finite and
    /// positive, `nullspace_gain`, `tolerance` and `limit_margin` finite and non-negative, and
    /// `iterations` at least one.
    pub fn validate(&self) -> FrankaResult<()> {
        let positive = |x: f64| x.is_finite() && x > 0.0;
        let non_negative = |x: f64| x.is_finite() && x >= 0.0;
        let valid = positive(self.damping)
            && positive(self.max_step)
            && non_negative(self.nullspace_gain)
            && non_negative(self.tolerance)
            && non_negative(self.limit_margin)
            && self.iterations > 0;
        if !valid {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: ik needs a finite, positive damping and max_step, finite and \
                 non-negative nullspace_gain, tolerance and limit_margin, and at least one \
                 iteration, got {self:?}"
            )));
        }
        Ok(())
    }
}

/// See the [module documentation](self).
pub(super) struct Ik {
    model: Arc<Model>,
    options: IkOptions,
    lower: [f64; 7],
    upper: [f64; 7],
    f_t_ee: [f64; 16],
    ee_t_k: [f64; 16],
    q: [f64; 7],
}

impl Ik {
    pub(super) fn new(
        model: Arc<Model>,
        options: IkOptions,
        limits: ([f64; 7], [f64; 7]),
        q0: [f64; 7],
        f_t_ee: [f64; 16],
        ee_t_k: [f64; 16],
    ) -> Self {
        let mut lower = limits.0.map(|l| l + options.limit_margin);
        let mut upper = limits.1.map(|u| u - options.limit_margin);
        for i in 0..7 {
            // A margin wider than half a joint's range pins that joint at the range's middle.
            if lower[i] > upper[i] {
                lower[i] = 0.5 * (limits.0[i] + limits.1[i]);
                upper[i] = lower[i];
            }
        }
        Ik {
            model,
            options,
            lower,
            upper,
            f_t_ee,
            ee_t_k,
            q: std::array::from_fn(|i| q0[i].clamp(lower[i], upper[i])),
        }
    }

    /// One cycle toward `pose` (column-major, as `O_T_EE`); returns the solution and the
    /// residual pose error norm. The first step always runs and carries the posture bias (a
    /// rate, integrated over `dt` once per call); the remaining steps refine the pose until
    /// the residual is below `tolerance`; the whole move is then capped at `max_step` on its
    /// largest joint.
    pub(super) fn step(
        &mut self,
        pose: &[f64; 16],
        posture: &[f64; 7],
        dt: f64,
    ) -> ([f64; 7], f64) {
        let p_des = Vector3::from(translation_of(pose));
        let r_des = rotation_of(pose);
        let posture = SVector::<f64, 7>::from(*posture);
        let damping =
            SMatrix::<f64, 6, 6>::identity() * (self.options.damping * self.options.damping);
        let from = self.q;
        let mut residual = 0.0;
        for iteration in 0..=self.options.iterations {
            let error = self.error(&p_des, &r_des);
            residual = error.norm();
            if iteration == self.options.iterations
                || (iteration > 0 && residual < self.options.tolerance)
            {
                break;
            }
            let j = SMatrix::<f64, 6, 7>::from_column_slice(&self.model.zero_jacobian_q(
                Frame::EndEffector,
                &self.q,
                &self.f_t_ee,
                &self.ee_t_k,
            ));
            let Some(inverse) = (j * j.transpose() + damping).try_inverse() else {
                break;
            };
            let pseudo_inverse = j.transpose() * inverse;
            let q = SVector::<f64, 7>::from(self.q);
            let mut next = q + pseudo_inverse * error;
            if iteration == 0 {
                let nullspace = SMatrix::<f64, 7, 7>::identity() - pseudo_inverse * j;
                let mut bias = nullspace * (posture - q) * (self.options.nullspace_gain * dt);
                let (largest, cap) = (bias.amax(), MAX_POSTURE_RATE * dt);
                if largest > cap {
                    bias *= cap / largest;
                }
                next += bias;
            }
            for i in 0..7 {
                self.q[i] = next[i].clamp(self.lower[i], self.upper[i]);
            }
        }
        let largest = (0..7).fold(0.0f64, |m, i| m.max((self.q[i] - from[i]).abs()));
        if largest > self.options.max_step {
            // Scaled as a whole toward `from`, which is within the limits, so it stays there.
            let scale = self.options.max_step / largest;
            for (q, from) in self.q.iter_mut().zip(&from) {
                *q = from + scale * (*q - from);
            }
            residual = self.error(&p_des, &r_des).norm();
        }
        (self.q, residual)
    }

    #[cfg(test)]
    pub(super) fn q(&self) -> [f64; 7] {
        self.q
    }

    /// `[p_des - p(q); log(R_des R(q)ᵀ)]`, base frame.
    fn error(&self, p_des: &Vector3<f64>, r_des: &Matrix3<f64>) -> SVector<f64, 6> {
        let fk = self
            .model
            .pose_q(Frame::EndEffector, &self.q, &self.f_t_ee, &self.ee_t_k);
        let p = p_des - Vector3::from(translation_of(&fk));
        let turn = log(&(r_des * rotation_of(&fk).transpose()));
        SVector::<f64, 6>::new(p[0], p[1], p[2], turn[0], turn[1], turn[2])
    }
}
