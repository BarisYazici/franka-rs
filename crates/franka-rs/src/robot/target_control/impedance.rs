//! The torque backend's impedance law, the structure of DROID / polymetis
//! `HybridJointImpedanceControl` with the damping on the velocity error:
//!
//! ```text
//! Kp  = Jᵀ Kx J + diag(Kq)        Kd = Jᵀ Kxd J + diag(Kqd)
//! tau = Kp (q_goal - q) + Kd (dq_goal - dq) + coriolis        clamped to torque_limits
//! ```
//!
//! `J` is the zero Jacobian at the end-effector frame in the *measured* configuration, so the
//! Cartesian spring acts at the frame `O_T_EE` targets are given in; gravity is the robot's.
//! Without [`ImpedanceOptions::velocity_feedforward`] `dq_goal` is zero and the damping acts on
//! the absolute velocity, which lags a moving goal by `Kd v / Kp` (DROID parity). The joint
//! gains act unprojected, so the end effector feels `Kx` plus the joint springs reflected
//! through `J` (30-60 % stiffer than `Kx` in translation at the ready pose, two to three
//! times in rotation); with [`ImpedanceOptions::project_joint_gains`] they are confined to
//! the nullspace, `N Kq N` with `N = I - J⁺ J`, and the end effector feels exactly `Kx`.

use nalgebra::{SMatrix, SVector};

use super::ik::IkOptions;
use crate::error::{FrankaError, FrankaResult};

/// The stiffness and damping of the law, all finite and non-negative.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpedanceGains {
    /// `Kx`, base frame, translation first: N/m, then Nm/rad.
    pub cartesian_stiffness: [f64; 6],
    /// `Kxd`, base frame, translation first: N s/m, then Nm s/rad.
    pub cartesian_damping: [f64; 6],
    /// `Kq`, per joint, Nm/rad.
    pub joint_stiffness: [f64; 7],
    /// `Kqd`, per joint, Nm s/rad.
    pub joint_damping: [f64; 7],
}

impl ImpedanceGains {
    /// The default for a Cartesian target: DROID's stiffness (a medium spring, 750 N/m) and
    /// soft joint springs that settle the redundancy, with the Cartesian damping raised to
    /// about a damping ratio of 0.8 from the arm's apparent masses at the ready pose (0.94 kg
    /// along x and y, 3.9 kg along z; DROID's 37 N s/m leaves z at 0.34 and ringing).
    pub const CARTESIAN: ImpedanceGains = ImpedanceGains {
        cartesian_stiffness: [750.0, 750.0, 750.0, 15.0, 15.0, 15.0],
        cartesian_damping: [50.0, 50.0, 90.0, 2.0, 2.0, 2.0],
        joint_stiffness: [40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0],
        joint_damping: [4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0],
    };

    /// DROID's production gains as they are, the parity preset: with
    /// [`ImpedanceOptions::velocity_feedforward`] and
    /// [`ImpedanceOptions::project_joint_gains`] both `false` the law is polymetis's
    /// `HybridJointImpedanceControl`, for replaying DROID-trained policies.
    pub const DROID: ImpedanceGains = ImpedanceGains {
        cartesian_stiffness: [750.0, 750.0, 750.0, 15.0, 15.0, 15.0],
        cartesian_damping: [37.0, 37.0, 37.0, 2.0, 2.0, 2.0],
        joint_stiffness: [40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0],
        joint_damping: [4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0],
    };

    /// A joint PD for a joint target, the gains of `examples/fer_joint_impedance.rs`.
    pub const JOINT: ImpedanceGains = ImpedanceGains {
        cartesian_stiffness: [0.0; 6],
        cartesian_damping: [0.0; 6],
        joint_stiffness: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        joint_damping: [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    };

    /// # Errors
    /// [`FrankaError::InvalidArgument`] if a gain is not finite or negative.
    pub fn validate(&self) -> FrankaResult<()> {
        let valid = |gains: &[f64]| gains.iter().all(|g| g.is_finite() && *g >= 0.0);
        if valid(&self.cartesian_stiffness)
            && valid(&self.cartesian_damping)
            && valid(&self.joint_stiffness)
            && valid(&self.joint_damping)
        {
            Ok(())
        } else {
            Err(FrankaError::InvalidArgument(format!(
                "target control: impedance gains must be finite and non-negative, got {self:?}"
            )))
        }
    }
}

/// How far the desired state may run ahead of the measured one. There is no echo of a torque
/// command to re-anchor the generator on, so every cycle it is anchored on the measured state
/// pulled toward the previous desired by at most this: while the arm follows, that is exactly
/// the previous desired and nothing changes; held back (a hand, an obstacle, an unreachable
/// target) the desired stays within the leash, the force on the arm is bounded by the
/// stiffness times the leash, and on release the generator resumes from where the arm is
/// under its budget. Once a stop holds, the leash pulls toward the held state instead, so an
/// arm moved during the hold meets the same bound. The observer reports what the leash took
/// off as `leash_alteration`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Leash {
    /// Cartesian interface, translation, m. Default 0.025. The force at the leash is about
    /// the *felt* stiffness times the leash: at the default gains at the ready pose roughly
    /// 25-30 N (the unprojected joint springs add to `Kx`), 18.75 N with
    /// [`project_joint_gains`](ImpedanceOptions::project_joint_gains). Set the collision
    /// thresholds accordingly.
    pub translation: f64,
    /// Cartesian interface, rotation, rad. Default 0.15.
    pub rotation: f64,
    /// Joint interface, per joint, rad. Default 0.1. On the joint interface the torque clamp,
    /// not the leash, bounds the torque: at the leash the [`JOINT`](ImpedanceGains::JOINT)
    /// preset pulls 60 Nm on joints 1 to 4 (under their 86 Nm clamp) and 25 / 15 / 5 Nm on the
    /// wrist, where joints 5 and 6 meet the 11.5 Nm clamp first; the examples' 20 Nm joint
    /// collision threshold is reached at 0.033 rad on joints 1 to 4.
    pub joint: f64,
}

impl Default for Leash {
    fn default() -> Self {
        Leash {
            translation: 0.025,
            rotation: 0.15,
            joint: 0.1,
        }
    }
}

/// Options of [`Backend::Impedance`]; [`cartesian`](Self::cartesian) and
/// [`joint`](Self::joint) are the documented starting points and the `with_*` methods change
/// one field each.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpedanceOptions {
    /// The gains of the law.
    pub gains: ImpedanceGains,
    /// Per-joint clamp on the commanded torque, Nm, at most [`RATED_TORQUES`]. Default 86 on
    /// joints 1-4, 11.5 on 5-7.
    pub torque_limits: [f64; 7],
    /// Cutoff, Hz, of the crate's first-order low-pass on the commanded torque;
    /// [`MAX_CUTOFF_FREQUENCY`](crate::lowpass_filter::MAX_CUTOFF_FREQUENCY) switches it off.
    /// Default 100.
    pub cutoff_frequency: f64,
    /// The configuration the Cartesian IK's nullspace bias pulls toward; `None`, the default,
    /// is the configuration the loop starts in. Ignored by the joint interface.
    pub posture: Option<[f64; 7]>,
    /// The Cartesian interface's inverse kinematics. Ignored by the joint interface.
    pub ik: IkOptions,
    /// Whether the damping acts on the velocity *error*, `Kd (dq_goal - dq)`, with `dq_goal`
    /// the goal's velocity (the generator's on the joint interface, the IK solution's finite
    /// difference on the Cartesian one); `false` damps the absolute velocity, DROID parity,
    /// and a goal moving at `v` is then tracked `Kd v / Kp` behind. Default `true`.
    pub velocity_feedforward: bool,
    /// How far the desired state may run ahead of the measured one; see [`Leash`].
    pub leash: Leash,
    /// Whether the joint gains are projected into the nullspace of the Jacobian (`N Kq N`,
    /// `N Kqd N`, `N = I - J⁺ J` with the damped pseudoinverse of [`IkOptions::damping`]), so
    /// that the Cartesian gains alone are felt at the end effector; `false` leaves them
    /// unprojected, DROID parity. For the Cartesian interface: projected joint gains hold the
    /// nullspace only, so with zero Cartesian gains the end effector would be free. Default
    /// `false`.
    pub project_joint_gains: bool,
}

/// The DROID clamp, a Nm and a half under the rated torques.
const TORQUE_LIMITS: [f64; 7] = [86.0, 86.0, 86.0, 86.0, 11.5, 11.5, 11.5];

/// The FR3's and FER's rated joint torques, Nm, the most `torque_limits` may allow.
pub const RATED_TORQUES: [f64; 7] = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0];

impl ImpedanceOptions {
    /// The defaults of the Cartesian interface: [`ImpedanceGains::CARTESIAN`].
    pub fn cartesian() -> Self {
        Self::with_default_gains(ImpedanceGains::CARTESIAN)
    }

    /// The defaults of the joint interface: [`ImpedanceGains::JOINT`].
    pub fn joint() -> Self {
        Self::with_default_gains(ImpedanceGains::JOINT)
    }

    fn with_default_gains(gains: ImpedanceGains) -> Self {
        ImpedanceOptions {
            gains,
            torque_limits: TORQUE_LIMITS,
            cutoff_frequency: 100.0,
            posture: None,
            ik: IkOptions::default(),
            velocity_feedforward: true,
            leash: Leash::default(),
            project_joint_gains: false,
        }
    }

    /// Sets the gains.
    pub fn with_gains(mut self, gains: ImpedanceGains) -> Self {
        self.gains = gains;
        self
    }

    /// Sets the per-joint torque clamp, Nm.
    pub fn with_torque_limits(mut self, limits: [f64; 7]) -> Self {
        self.torque_limits = limits;
        self
    }

    /// Sets the torque low-pass cutoff, Hz.
    pub fn with_cutoff_frequency(mut self, hertz: f64) -> Self {
        self.cutoff_frequency = hertz;
        self
    }

    /// Sets the IK's posture (`None`: the start configuration).
    pub fn with_posture(mut self, posture: Option<[f64; 7]>) -> Self {
        self.posture = posture;
        self
    }

    /// Sets the IK options.
    pub fn with_ik(mut self, ik: IkOptions) -> Self {
        self.ik = ik;
        self
    }

    /// Switches the damping between the velocity error (`true`) and the absolute velocity.
    pub fn with_velocity_feedforward(mut self, on: bool) -> Self {
        self.velocity_feedforward = on;
        self
    }

    /// Sets the leash.
    pub fn with_leash(mut self, leash: Leash) -> Self {
        self.leash = leash;
        self
    }

    /// Switches the projection of the joint gains into the Jacobian's nullspace.
    pub fn with_project_joint_gains(mut self, on: bool) -> Self {
        self.project_joint_gains = on;
        self
    }

    /// Checks the options without starting anything.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] naming the field: a gain that is not finite or
    /// negative, a torque limit, cutoff frequency or leash that is not finite and positive, a
    /// posture that is not finite, or invalid [`IkOptions`].
    pub fn validate(&self) -> FrankaResult<()> {
        self.gains.validate()?;
        let positive = |x: f64| x.is_finite() && x > 0.0;
        let rated = self.torque_limits.iter().zip(&RATED_TORQUES);
        if !rated.into_iter().all(|(t, r)| positive(*t) && t <= r) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: torque_limits must be positive and at most the rated \
                 {RATED_TORQUES:?} Nm, got {:?}",
                self.torque_limits
            )));
        }
        if !positive(self.cutoff_frequency) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: cutoff_frequency must be finite and positive, got {}",
                self.cutoff_frequency
            )));
        }
        let Leash {
            translation,
            rotation,
            joint,
        } = self.leash;
        if !(positive(translation) && positive(rotation) && positive(joint)) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: leash must be finite and positive, got {:?}",
                self.leash
            )));
        }
        if let Some(posture) = self.posture {
            if posture.iter().any(|q| !q.is_finite()) {
                return Err(FrankaError::InvalidArgument(format!(
                    "target control: posture must be finite, got {posture:?}"
                )));
            }
        }
        self.ik.validate()
    }
}

/// What tracks the target: the robot's controller or the crate's torques.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // options, copied once at the start; boxing would cost `Copy`
pub enum Backend {
    /// The robot's own impedance controller tracks a pose / joint-position stream.
    RobotController,
    /// Torques from the crate's impedance law (`control_torques`).
    Impedance(ImpedanceOptions),
}

/// The torque backend's law, offline:
///
/// ```text
/// Kp  = Jᵀ Kx J + diag(Kq)        Kd = Jᵀ Kxd J + diag(Kqd)
/// tau = Kp (q_goal - q) + Kd (dq_goal - dq) + coriolis        clamped to torque_limits
/// ```
///
/// with the gains of `options`; `dq_goal` is taken as zero without
/// [`velocity_feedforward`](ImpedanceOptions::velocity_feedforward), and the joint gains are
/// confined to the nullspace of `J` with
/// [`project_joint_gains`](ImpedanceOptions::project_joint_gains). `jacobian` is the zero
/// Jacobian at the end effector in the measured configuration, column-major 6x7 as
/// [`Model::zero_jacobian`](crate::Model::zero_jacobian) returns it.
pub fn impedance_torques(
    options: &ImpedanceOptions,
    jacobian: &[f64; 42],
    q_goal: &[f64; 7],
    dq_goal: &[f64; 7],
    q: &[f64; 7],
    dq: &[f64; 7],
    coriolis: &[f64; 7],
) -> [f64; 7] {
    let gains = &options.gains;
    let j = SMatrix::<f64, 6, 7>::from_column_slice(jacobian);
    let kx = SVector::<f64, 6>::from(gains.cartesian_stiffness);
    let kxd = SVector::<f64, 6>::from(gains.cartesian_damping);
    let kq = SVector::<f64, 7>::from(gains.joint_stiffness);
    let kqd = SVector::<f64, 7>::from(gains.joint_damping);
    let error = SVector::<f64, 7>::from(*q_goal) - SVector::<f64, 7>::from(*q);
    let mut velocity_error = -SVector::<f64, 7>::from(*dq);
    if options.velocity_feedforward {
        velocity_error += SVector::<f64, 7>::from(*dq_goal);
    }
    // Jᵀ Kx J e as Jᵀ (Kx ∘ J e): the diagonal gains never form a 7x7.
    let mut tau = j.transpose()
        * (kx.component_mul(&(j * error)) + kxd.component_mul(&(j * velocity_error)))
        + SVector::<f64, 7>::from(*coriolis);
    let projector = options
        .project_joint_gains
        .then(|| nullspace_projector(&j, options.ik.damping));
    match projector {
        // N is symmetric, so N Kq N e = N (Kq ∘ N e).
        Some(n) => {
            tau += n * (kq.component_mul(&(n * error)) + kqd.component_mul(&(n * velocity_error)))
        }
        None => tau += kq.component_mul(&error) + kqd.component_mul(&velocity_error),
    }
    let limits = &options.torque_limits;
    std::array::from_fn(|i| tau[i].clamp(-limits[i], limits[i]))
}

/// `N = I - J⁺ J` with the damped pseudoinverse `J⁺ = Jᵀ (J Jᵀ + λ² I)⁻¹`. The 6x6 is
/// positive definite for `λ > 0`, so the inverse only fails on non-finite input, where the
/// identity (nothing projected) is the safe answer.
fn nullspace_projector(j: &SMatrix<f64, 6, 7>, damping: f64) -> SMatrix<f64, 7, 7> {
    let regularised = j * j.transpose() + SMatrix::<f64, 6, 6>::identity() * (damping * damping);
    match regularised.try_inverse() {
        Some(inverse) => SMatrix::<f64, 7, 7>::identity() - j.transpose() * inverse * j,
        None => SMatrix::<f64, 7, 7>::identity(),
    }
}
