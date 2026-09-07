//! Kinematics and dynamics of the arm, mirroring libfranka's `franka::Model`.
//!
//! [`Model`] is a port of `franka::Model` (`include/franka/model.h`,
//! `src/model.cpp`); it is a thin dispatch layer over a [`RobotModelBackend`],
//! the equivalent of libfranka's `franka::RobotModelBase`. The default backend
//! is [`native_backend::NativeBackend`], a serial-chain implementation that
//! reproduces libfranka's Pinocchio-based `franka::RobotModel` bit-for-bit
//! within 1e-9 (kinematics) and 1e-6 (dynamics).
//!
//! # Matrix layouts
//!
//! * 4x4 poses are vectorised column-major (`[f64; 16]`).
//! * Jacobians are 6x7 column-major (`[f64; 42]`); within a column the rows are
//!   the linear parts `vx, vy, vz` first and the angular parts `wx, wy, wz`
//!   second. This is Pinocchio's `Motion` layout, which is what libfranka
//!   copies out of Eigen verbatim.
//! * The mass matrix is 7x7 column-major (`[f64; 49]`); it is symmetric.
//!
//! # Example
//!
//! ```no_run
//! use franka::model::{Frame, Model};
//! use franka::robot_state::RobotState;
//!
//! let urdf = std::fs::read_to_string("fr3.urdf").unwrap();
//! let model = Model::from_urdf(&urdf).unwrap();
//! let state = RobotState::default();
//! let o_t_ee = model.pose(Frame::EndEffector, &state);
//! let gravity = model.gravity(&state);
//! # let _ = (o_t_ee, gravity);
//! ```

mod spatial;

pub mod native_backend;

/// Downloading and loading the FCI v5 (Franka Emika Robot, FER) model library.
///
/// [`model_library::load_from_robot`] exists in every configuration so that
/// callers need no `cfg` of their own; the `dlopen`-based backend behind it is
/// the default-on `model-library` cargo feature, which pulls in `libloading`.
/// Without that feature the crate builds with the URDF-based
/// [`native_backend`] only and `load_from_robot` reports a
/// [`crate::FrankaError::Model`].
pub mod model_library;

#[cfg(feature = "model-library")]
pub mod so_backend;

use crate::error::FrankaResult;
use crate::robot_state::RobotState;

pub use model_library::load_from_robot;
pub use native_backend::NativeBackend;
#[cfg(feature = "model-library")]
pub use so_backend::SoModelBackend;

/// Earth's gravity used by [`Model::coriolis`], matching libfranka's
/// `RobotModel::coriolis` fallback of `{0, 0, -9.81}`.
pub const DEFAULT_GRAVITY_EARTH: [f64; 3] = [0.0, 0.0, -9.81];

/// The Franka Emika Robot (FER)'s arm, as a URDF the [`native_backend`] can evaluate.
///
/// An FER has no `GetRobotModel`, so this file is *not* something the robot
/// serves: it is a mesh-free, arm-only description built here — the joint
/// origins and axes of `franka_description`'s `fer_joint1..8`, plus link
/// inertial parameters identified from an FER's own `libfcimodels_x64.so` by
/// `tools/fer-model-fit`. [`Model::native_fer`] parses it.
///
/// The ten inertial parameters per link are a *base-parameter-equivalent* set,
/// chosen to reproduce the robot's model, and not a physically realisable
/// description of each casting; see `docs/book/src/fer.md`.
pub const FER_URDF: &str = include_str!("../../tests/data/fer.urdf");

/// The seven joints, the flange, the end effector and the stiffness frame.
///
/// Port of `franka::Frame` (`include/franka/model.h`); the variants are in the
/// same order as the C++ enumerators `kJoint1 .. kStiffness`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Frame {
    /// Frame of joint 1.
    Joint1,
    /// Frame of joint 2.
    Joint2,
    /// Frame of joint 3.
    Joint3,
    /// Frame of joint 4.
    Joint4,
    /// Frame of joint 5.
    Joint5,
    /// Frame of joint 6.
    Joint6,
    /// Frame of joint 7.
    Joint7,
    /// Flange frame (the `link8` frame of the URDF).
    Flange,
    /// End-effector frame, i.e. the flange frame post-multiplied by `F_T_EE`.
    EndEffector,
    /// Stiffness frame K, i.e. the end-effector frame post-multiplied by `EE_T_K`.
    Stiffness,
}

impl Frame {
    /// All frames, in the order of the C++ enumerators.
    pub const ALL: [Frame; 10] = [
        Frame::Joint1,
        Frame::Joint2,
        Frame::Joint3,
        Frame::Joint4,
        Frame::Joint5,
        Frame::Joint6,
        Frame::Joint7,
        Frame::Flange,
        Frame::EndEffector,
        Frame::Stiffness,
    ];

    /// The joint index (`1..=7`) of a joint frame, `None` for the others.
    pub fn joint_index(self) -> Option<usize> {
        match self {
            Frame::Joint1 => Some(1),
            Frame::Joint2 => Some(2),
            Frame::Joint3 => Some(3),
            Frame::Joint4 => Some(4),
            Frame::Joint5 => Some(5),
            Frame::Joint6 => Some(6),
            Frame::Joint7 => Some(7),
            _ => None,
        }
    }
}

/// Rigid-body model backend.
///
/// Port of libfranka's `franka::RobotModelBase` (`src/robot_model_base.h`).
/// The method set is deliberately identical so an alternative backend can be
/// slotted in the way libfranka's tests slot in a mocked `RobotModelBase`.
///
/// All `joint_index` arguments are 1-based and must be in `1..=7`.
pub trait RobotModelBackend {
    /// Coriolis force vector `C(q, dq) * dq`, in Nm.
    ///
    /// Port of `RobotModelBase::coriolis` with a gravity argument, which
    /// libfranka evaluates as `rnea(q, dq, 0) - generalized_gravity(q)`.
    fn coriolis(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
        gravity_earth: &[f64; 3],
    ) -> [f64; 7];

    /// Gravity torque vector, in Nm. Port of `RobotModelBase::gravity`.
    fn gravity(
        &self,
        q: &[f64; 7],
        gravity_earth: &[f64; 3],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [f64; 7];

    /// 7x7 joint-space inertia matrix, column-major. Port of `RobotModelBase::mass`.
    fn mass(
        &self,
        q: &[f64; 7],
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [f64; 49];

    /// Pose of a joint frame in the base frame. Port of `RobotModelBase::pose`.
    fn pose(&self, q: &[f64; 7], joint_index: usize) -> [f64; 16];

    /// Pose of the flange frame. Port of `RobotModelBase::poseFlange`.
    fn pose_flange(&self, q: &[f64; 7]) -> [f64; 16];

    /// Pose of the end-effector frame. Port of `RobotModelBase::poseEe`.
    fn pose_ee(&self, q: &[f64; 7], f_t_ee: &[f64; 16]) -> [f64; 16];

    /// Pose of the stiffness frame. Port of `RobotModelBase::poseStiffness`.
    fn pose_stiffness(&self, q: &[f64; 7], f_t_ee: &[f64; 16], ee_t_k: &[f64; 16]) -> [f64; 16];

    /// Body (`LOCAL`) Jacobian of a joint frame. Port of `RobotModelBase::bodyJacobian`.
    fn body_jacobian(&self, q: &[f64; 7], joint_index: usize) -> [f64; 42];

    /// Body Jacobian of the flange frame. Port of `RobotModelBase::bodyJacobianFlange`.
    fn body_jacobian_flange(&self, q: &[f64; 7]) -> [f64; 42];

    /// Body Jacobian of the end-effector frame. Port of `RobotModelBase::bodyJacobianEe`.
    fn body_jacobian_ee(&self, q: &[f64; 7], f_t_ee: &[f64; 16]) -> [f64; 42];

    /// Body Jacobian of the stiffness frame. Port of `RobotModelBase::bodyJacobianStiffness`.
    fn body_jacobian_stiffness(
        &self,
        q: &[f64; 7],
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> [f64; 42];

    /// Zero (`LOCAL_WORLD_ALIGNED`) Jacobian of a joint frame. Port of
    /// `RobotModelBase::zeroJacobian`.
    fn zero_jacobian(&self, q: &[f64; 7], joint_index: usize) -> [f64; 42];

    /// Zero Jacobian of the flange frame. Port of `RobotModelBase::zeroJacobianFlange`.
    fn zero_jacobian_flange(&self, q: &[f64; 7]) -> [f64; 42];

    /// Zero Jacobian of the end-effector frame. Port of `RobotModelBase::zeroJacobianEe`.
    fn zero_jacobian_ee(&self, q: &[f64; 7], f_t_ee: &[f64; 16]) -> [f64; 42];

    /// Zero Jacobian of the stiffness frame. Port of `RobotModelBase::zeroJacobianStiffness`.
    fn zero_jacobian_stiffness(
        &self,
        q: &[f64; 7],
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> [f64; 42];
}

/// Poses of the joints and dynamic properties of the robot.
///
/// Port of `franka::Model`. Obtain one from `Robot::load_model`, or build one
/// directly from a URDF with [`Model::from_urdf`].
pub struct Model {
    backend: Box<dyn RobotModelBackend + Send + Sync>,
}

impl std::fmt::Debug for Model {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Model").finish_non_exhaustive()
    }
}

impl Model {
    /// Builds a model from a URDF string using the native backend.
    ///
    /// Port of `franka::Model::Model(const std::string&)`.
    ///
    /// # Errors
    ///
    /// [`crate::FrankaError::Model`] if the URDF cannot be parsed or does not
    /// describe a seven-axis arm ending in a `link8` flange frame.
    pub fn from_urdf(urdf: &str) -> FrankaResult<Model> {
        Ok(Model::from_backend(Box::new(NativeBackend::from_urdf(
            urdf,
        )?)))
    }

    /// Builds the Franka Emika Robot (FER)'s model, without asking a robot for it.
    ///
    /// This is the FCI v5 counterpart of what [`Model::from_urdf`] does with an
    /// FR3's `GetRobotModel` answer: it is the native backend over
    /// [`FER_URDF`], whose parameters were identified from an FER's own
    /// `libfcimodels_x64.so`. It needs no download, no `dlopen`, no
    /// `model-library` feature and no x86-64 Linux host, and it is what
    /// [`crate::Robot::load_model`] returns on an FER.
    ///
    /// Agreement with the shared object, over 208 joint configurations
    /// (see `docs/book/src/model.md`): poses and both Jacobians to 9e-16 at
    /// all ten frames, gravity to 5e-14 with any payload, and the mass matrix
    /// and Coriolis vector to 4e-14 with no payload. With a payload the mass
    /// matrix and Coriolis vector differ by up to 3e-3 and 5e-2, because the
    /// robot's own `M_NE` is not affine in `m_load` and therefore is not a
    /// rigid-body model of the payload; see `docs/book/src/fer.md`. Use
    /// [`crate::Robot::load_model_from_robot`] when you need the library's own
    /// answer to the last bit.
    ///
    /// # Panics
    ///
    /// Never: [`FER_URDF`] is compiled in and a unit test parses it.
    pub fn native_fer() -> Model {
        Model::from_urdf(FER_URDF).expect("the built-in FER URDF parses")
    }

    /// Builds a model around an explicit backend.
    ///
    /// Port of `franka::Model::Model(std::unique_ptr<RobotModelBase>)`, which
    /// libfranka provides for its own tests; the conformance suite uses it to
    /// run the evaluation backends through the same API.
    pub fn from_backend(backend: Box<dyn RobotModelBackend + Send + Sync>) -> Model {
        Model { backend }
    }

    /// Borrows the underlying backend.
    pub fn backend(&self) -> &(dyn RobotModelBackend + Send + Sync) {
        self.backend.as_ref()
    }

    /// 4x4 pose matrix of `frame` in the base frame, column-major.
    ///
    /// Port of `franka::Model::pose(Frame, const RobotState&)`.
    #[allow(non_snake_case)]
    pub fn pose(&self, frame: Frame, state: &RobotState) -> [f64; 16] {
        self.pose_q(frame, &state.q, &state.F_T_EE, &state.EE_T_K)
    }

    /// 4x4 pose matrix of `frame` in the base frame, column-major.
    ///
    /// Port of `franka::Model::pose(Frame, q, F_T_EE, EE_T_K)`.
    #[allow(non_snake_case)]
    pub fn pose_q(
        &self,
        frame: Frame,
        q: &[f64; 7],
        F_T_EE: &[f64; 16],
        EE_T_K: &[f64; 16],
    ) -> [f64; 16] {
        match frame {
            Frame::Flange => self.backend.pose_flange(q),
            Frame::EndEffector => self.backend.pose_ee(q, F_T_EE),
            Frame::Stiffness => self.backend.pose_stiffness(q, F_T_EE, EE_T_K),
            joint => self
                .backend
                .pose(q, joint.joint_index().expect("joint frame")),
        }
    }

    /// 6x7 body Jacobian of `frame`, column-major, relative to `frame` itself.
    ///
    /// Port of `franka::Model::bodyJacobian(Frame, const RobotState&)`.
    #[allow(non_snake_case)]
    pub fn body_jacobian(&self, frame: Frame, state: &RobotState) -> [f64; 42] {
        self.body_jacobian_q(frame, &state.q, &state.F_T_EE, &state.EE_T_K)
    }

    /// 6x7 body Jacobian of `frame`, column-major, relative to `frame` itself.
    ///
    /// Port of `franka::Model::bodyJacobian(Frame, q, F_T_EE, EE_T_K)`.
    #[allow(non_snake_case)]
    pub fn body_jacobian_q(
        &self,
        frame: Frame,
        q: &[f64; 7],
        F_T_EE: &[f64; 16],
        EE_T_K: &[f64; 16],
    ) -> [f64; 42] {
        match frame {
            Frame::Flange => self.backend.body_jacobian_flange(q),
            Frame::EndEffector => self.backend.body_jacobian_ee(q, F_T_EE),
            Frame::Stiffness => self.backend.body_jacobian_stiffness(q, F_T_EE, EE_T_K),
            joint => self
                .backend
                .body_jacobian(q, joint.joint_index().expect("joint frame")),
        }
    }

    /// 6x7 zero Jacobian of `frame`, column-major, relative to the base frame.
    ///
    /// Port of `franka::Model::zeroJacobian(Frame, const RobotState&)`.
    #[allow(non_snake_case)]
    pub fn zero_jacobian(&self, frame: Frame, state: &RobotState) -> [f64; 42] {
        self.zero_jacobian_q(frame, &state.q, &state.F_T_EE, &state.EE_T_K)
    }

    /// 6x7 zero Jacobian of `frame`, column-major, relative to the base frame.
    ///
    /// Port of `franka::Model::zeroJacobian(Frame, q, F_T_EE, EE_T_K)`.
    #[allow(non_snake_case)]
    pub fn zero_jacobian_q(
        &self,
        frame: Frame,
        q: &[f64; 7],
        F_T_EE: &[f64; 16],
        EE_T_K: &[f64; 16],
    ) -> [f64; 42] {
        match frame {
            Frame::Flange => self.backend.zero_jacobian_flange(q),
            Frame::EndEffector => self.backend.zero_jacobian_ee(q, F_T_EE),
            Frame::Stiffness => self.backend.zero_jacobian_stiffness(q, F_T_EE, EE_T_K),
            joint => self
                .backend
                .zero_jacobian(q, joint.joint_index().expect("joint frame")),
        }
    }

    /// 7x7 mass matrix, column-major, in `kg * m^2`.
    ///
    /// Port of `franka::Model::mass(const RobotState&)`.
    pub fn mass(&self, state: &RobotState) -> [f64; 49] {
        self.mass_q(&state.q, &state.I_total, state.m_total, &state.F_x_Ctotal)
    }

    /// 7x7 mass matrix, column-major, in `kg * m^2`.
    ///
    /// Port of `franka::Model::mass(q, I_total, m_total, F_x_Ctotal)`.
    #[allow(non_snake_case)]
    pub fn mass_q(
        &self,
        q: &[f64; 7],
        I_total: &[f64; 9],
        m_total: f64,
        F_x_Ctotal: &[f64; 3],
    ) -> [f64; 49] {
        self.backend.mass(q, I_total, m_total, F_x_Ctotal)
    }

    /// Coriolis force vector, in Nm, with Earth's gravity `{0, 0, -9.81}`.
    ///
    /// Port of `franka::Model::coriolis(const RobotState&)`, which forwards to
    /// the deprecated `RobotModel::coriolis` overload that hard-codes that
    /// gravity vector.
    pub fn coriolis(&self, state: &RobotState) -> [f64; 7] {
        self.coriolis_q(
            &state.q,
            &state.dq,
            &state.I_total,
            state.m_total,
            &state.F_x_Ctotal,
            &DEFAULT_GRAVITY_EARTH,
        )
    }

    /// Coriolis force vector, in Nm.
    ///
    /// Port of `franka::Model::coriolis(q, dq, I_total, m_total, F_x_Ctotal, gravity_earth)`.
    #[allow(non_snake_case)]
    pub fn coriolis_q(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        I_total: &[f64; 9],
        m_total: f64,
        F_x_Ctotal: &[f64; 3],
        gravity_earth: &[f64; 3],
    ) -> [f64; 7] {
        self.backend
            .coriolis(q, dq, I_total, m_total, F_x_Ctotal, gravity_earth)
    }

    /// Gravity torque vector, in Nm, using the state's `O_ddP_O` as Earth's gravity.
    ///
    /// Port of `franka::Model::gravity(const RobotState&)`.
    pub fn gravity(&self, state: &RobotState) -> [f64; 7] {
        self.gravity_q(&state.q, state.m_total, &state.F_x_Ctotal, &state.O_ddP_O)
    }

    /// Gravity torque vector, in Nm.
    ///
    /// Port of `franka::Model::gravity(q, m_total, F_x_Ctotal, gravity_earth)`.
    #[allow(non_snake_case)]
    pub fn gravity_q(
        &self,
        q: &[f64; 7],
        m_total: f64,
        F_x_Ctotal: &[f64; 3],
        gravity_earth: &[f64; 3],
    ) -> [f64; 7] {
        self.backend.gravity(q, gravity_earth, m_total, F_x_Ctotal)
    }
}

#[cfg(test)]
mod tests;
