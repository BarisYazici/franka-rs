//! `franka.Model`: [`franka::Model`] over numpy arrays; pure compute, no GIL release.

use nalgebra::{Matrix4, Rotation3, Vector3};
use numpy::ndarray::Array3;
use numpy::{
    AllowTypeChange, IntoPyArray, PyArray1, PyArray2, PyArray3, PyArrayLike1, PyArrayLike2,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use franka::model::DEFAULT_GRAVITY_EARTH;
use franka::robot_state::IDENTITY_TRANSFORM;
use franka::Frame;

use crate::state::to_matrix;

/// Yaw of franka_description's hand frame about the flange's `z`, rad (`franka-rerun`'s
/// `meshes::HAND_YAW`).
const HAND_YAW: f64 = -std::f64::consts::FRAC_PI_4;

const FRAMES: [(&str, Frame); 10] = [
    ("joint1", Frame::Joint1),
    ("joint2", Frame::Joint2),
    ("joint3", Frame::Joint3),
    ("joint4", Frame::Joint4),
    ("joint5", Frame::Joint5),
    ("joint6", Frame::Joint6),
    ("joint7", Frame::Joint7),
    ("flange", Frame::Flange),
    ("ee", Frame::EndEffector),
    ("stiffness", Frame::Stiffness),
];

type Vector<'py> = PyArrayLike1<'py, f64, AllowTypeChange>;
type Matrix<'py> = PyArrayLike2<'py, f64, AllowTypeChange>;

fn parse_frame(name: &str) -> PyResult<Frame> {
    FRAMES
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, frame)| *frame)
        .ok_or_else(|| {
            let names: Vec<&str> = FRAMES.iter().map(|(n, _)| *n).collect();
            PyValueError::new_err(format!(
                "franka: frame must be one of {names:?}, got {name:?}"
            ))
        })
}

/// A vector of `N` elements; `None` is `default`.
fn vector<const N: usize>(
    v: Option<&Vector<'_>>,
    default: [f64; N],
    what: &str,
) -> PyResult<[f64; N]> {
    let Some(v) = v else { return Ok(default) };
    let v = v.as_array();
    <[f64; N]>::try_from(v.to_vec()).map_err(|_| {
        PyValueError::new_err(format!(
            "franka: {what} has {N} elements, got shape {:?}",
            v.shape()
        ))
    })
}

/// A row-major `(rows, N / rows)` matrix as a column-major array; `None` is `default`.
fn matrix<const N: usize>(
    m: Option<&Matrix<'_>>,
    rows: usize,
    default: [f64; N],
    what: &str,
) -> PyResult<[f64; N]> {
    let Some(m) = m else { return Ok(default) };
    let m = m.as_array();
    if m.shape() != [rows, N / rows] {
        return Err(PyValueError::new_err(format!(
            "franka: {what} has shape ({rows}, {}), got {:?}",
            N / rows,
            m.shape()
        )));
    }
    Ok(std::array::from_fn(|i| m[(i % rows, i / rows)]))
}

/// The `(frame, q, F_T_EE, EE_T_K)` arguments of the kinematic methods.
type FrameArgs = (Frame, [f64; 7], [f64; 16], [f64; 16]);

#[allow(non_snake_case)]
fn frame_args(
    frame: &str,
    q: &Vector<'_>,
    F_T_EE: Option<&Matrix<'_>>,
    EE_T_K: Option<&Matrix<'_>>,
) -> PyResult<FrameArgs> {
    Ok((
        parse_frame(frame)?,
        vector(Some(q), [0.0; 7], "q")?,
        matrix(F_T_EE, 4, IDENTITY_TRANSFORM, "F_T_EE")?,
        matrix(EE_T_K, 4, IDENTITY_TRANSFORM, "EE_T_K")?,
    ))
}

/// The `(I_total, F_x_Ctotal)` payload arguments of the dynamic methods.
#[allow(non_snake_case)]
fn load_args(
    I_total: Option<&Matrix<'_>>,
    F_x_Ctotal: Option<&Vector<'_>>,
) -> PyResult<([f64; 9], [f64; 3])> {
    Ok((
        matrix(I_total, 3, [0.0; 9], "I_total")?,
        vector(F_x_Ctotal, [0.0; 3], "F_x_Ctotal")?,
    ))
}

/// The kinematics and dynamics of the connected robot (`Robot.model()`), computed natively
/// on both an FR3 and an FER. Frames are `'joint1'..'joint7'`, `'flange'`, `'ee'` and
/// `'stiffness'`; the optional transforms default to the identity (pass `state.F_T_EE`
/// and `state.EE_T_K` for the mounted tool), the payload to nothing attached (pass
/// `state.I_total`, `state.m_total`, `state.F_x_Ctotal`) and gravity to `(0, 0, -9.81)`.
/// Jacobians are `(6, 7)`, linear rows first; poses `(4, 4)` as `RobotState.O_T_EE`.
#[pyclass(frozen, name = "Model", module = "franka")]
pub struct Model {
    inner: franka::Model,
}

impl From<franka::Model> for Model {
    fn from(inner: franka::Model) -> Self {
        Model { inner }
    }
}

#[pymethods]
impl Model {
    /// Pose of `frame` in the base frame, `(4, 4)`.
    #[pyo3(signature = (frame, q, F_T_EE = None, EE_T_K = None))]
    fn pose<'py>(
        &self,
        py: Python<'py>,
        frame: &str,
        q: Vector<'py>,
        F_T_EE: Option<Matrix<'py>>,
        EE_T_K: Option<Matrix<'py>>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let (frame, q, f_t_ee, ee_t_k) = frame_args(frame, &q, F_T_EE.as_ref(), EE_T_K.as_ref())?;
        Ok(to_matrix(
            py,
            &self.inner.pose_q(frame, &q, &f_t_ee, &ee_t_k),
            4,
        ))
    }

    /// Body Jacobian of `frame`, `(6, 7)`, expressed in `frame` itself.
    #[pyo3(signature = (frame, q, F_T_EE = None, EE_T_K = None))]
    fn body_jacobian<'py>(
        &self,
        py: Python<'py>,
        frame: &str,
        q: Vector<'py>,
        F_T_EE: Option<Matrix<'py>>,
        EE_T_K: Option<Matrix<'py>>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let (frame, q, f_t_ee, ee_t_k) = frame_args(frame, &q, F_T_EE.as_ref(), EE_T_K.as_ref())?;
        let j = self.inner.body_jacobian_q(frame, &q, &f_t_ee, &ee_t_k);
        Ok(to_matrix(py, &j, 6))
    }

    /// Zero Jacobian of `frame`, `(6, 7)`, expressed in the base frame.
    #[pyo3(signature = (frame, q, F_T_EE = None, EE_T_K = None))]
    fn zero_jacobian<'py>(
        &self,
        py: Python<'py>,
        frame: &str,
        q: Vector<'py>,
        F_T_EE: Option<Matrix<'py>>,
        EE_T_K: Option<Matrix<'py>>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let (frame, q, f_t_ee, ee_t_k) = frame_args(frame, &q, F_T_EE.as_ref(), EE_T_K.as_ref())?;
        let j = self.inner.zero_jacobian_q(frame, &q, &f_t_ee, &ee_t_k);
        Ok(to_matrix(py, &j, 6))
    }

    /// Mass matrix, `(7, 7)`, kg m^2.
    #[pyo3(signature = (q, I_total = None, m_total = 0.0, F_x_Ctotal = None))]
    fn mass<'py>(
        &self,
        py: Python<'py>,
        q: Vector<'py>,
        I_total: Option<Matrix<'py>>,
        m_total: f64,
        F_x_Ctotal: Option<Vector<'py>>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let q = vector(Some(&q), [0.0; 7], "q")?;
        let (i_total, f_x_ctotal) = load_args(I_total.as_ref(), F_x_Ctotal.as_ref())?;
        let m = self.inner.mass_q(&q, &i_total, m_total, &f_x_ctotal);
        Ok(to_matrix(py, &m, 7))
    }

    /// Coriolis torques `C(q, dq) dq`, `(7,)`, Nm.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (q, dq, I_total = None, m_total = 0.0, F_x_Ctotal = None, gravity_earth = None))]
    fn coriolis<'py>(
        &self,
        py: Python<'py>,
        q: Vector<'py>,
        dq: Vector<'py>,
        I_total: Option<Matrix<'py>>,
        m_total: f64,
        F_x_Ctotal: Option<Vector<'py>>,
        gravity_earth: Option<Vector<'py>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = vector(Some(&q), [0.0; 7], "q")?;
        let dq = vector(Some(&dq), [0.0; 7], "dq")?;
        let (i_total, f_x_ctotal) = load_args(I_total.as_ref(), F_x_Ctotal.as_ref())?;
        let g = vector(
            gravity_earth.as_ref(),
            DEFAULT_GRAVITY_EARTH,
            "gravity_earth",
        )?;
        let c = self
            .inner
            .coriolis_q(&q, &dq, &i_total, m_total, &f_x_ctotal, &g);
        Ok(PyArray1::from_slice(py, &c))
    }

    /// Gravity torques, `(7,)`, Nm.
    #[pyo3(signature = (q, m_total = 0.0, F_x_Ctotal = None, gravity_earth = None))]
    fn gravity<'py>(
        &self,
        py: Python<'py>,
        q: Vector<'py>,
        m_total: f64,
        F_x_Ctotal: Option<Vector<'py>>,
        gravity_earth: Option<Vector<'py>>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let q = vector(Some(&q), [0.0; 7], "q")?;
        let f_x_ctotal = vector(F_x_Ctotal.as_ref(), [0.0; 3], "F_x_Ctotal")?;
        let g = vector(
            gravity_earth.as_ref(),
            DEFAULT_GRAVITY_EARTH,
            "gravity_earth",
        )?;
        let tau = self.inner.gravity_q(&q, m_total, &f_x_ctotal, &g);
        Ok(PyArray1::from_slice(py, &tau))
    }

    /// The frames franka_description hangs `link0..link7`'s meshes on, `(8, 4, 4)`: the
    /// base (identity) and `joint1..joint7`.
    fn link_poses<'py>(
        &self,
        py: Python<'py>,
        q: Vector<'py>,
    ) -> PyResult<Bound<'py, PyArray3<f64>>> {
        let q = vector(Some(&q), [0.0; 7], "q")?;
        let poses: [[f64; 16]; 8] = std::array::from_fn(|k| match k {
            0 => IDENTITY_TRANSFORM,
            _ => self.joint_pose(Frame::ALL[k - 1], &q),
        });
        Ok(Array3::from_shape_fn((8, 4, 4), |(k, r, c)| poses[k][c * 4 + r]).into_pyarray(py))
    }

    /// The frame franka_description hangs the hand mesh on: the flange yawed by -45 deg
    /// about its `z`, `(4, 4)`.
    fn hand_pose<'py>(
        &self,
        py: Python<'py>,
        q: Vector<'py>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let q = vector(Some(&q), [0.0; 7], "q")?;
        let flange = Matrix4::from_column_slice(&self.joint_pose(Frame::Flange, &q));
        let yaw = Rotation3::from_axis_angle(&Vector3::z_axis(), HAND_YAW).to_homogeneous();
        Ok(to_matrix(py, (flange * yaw).as_slice(), 4))
    }

    fn __repr__(&self) -> &'static str {
        "Model()"
    }
}

impl Model {
    /// A frame's pose with nothing mounted on the flange.
    fn joint_pose(&self, frame: Frame, q: &[f64; 7]) -> [f64; 16] {
        self.inner
            .pose_q(frame, q, &IDENTITY_TRANSFORM, &IDENTITY_TRANSFORM)
    }
}
