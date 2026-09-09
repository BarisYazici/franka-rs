//! `franka.RobotState`: [`franka::RobotState`] with numpy fields and `flat()`.

use nalgebra::{Matrix3, Matrix4, Rotation3, UnitQuaternion};
use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray1, PyArray2};
use pyo3::prelude::*;
use pyo3::types::PyList;

/// The slices of `flat()` as `(name, start, stop)`; fixed for the life of the bindings.
pub const FLAT_LAYOUT: &[(&str, usize, usize)] = &[
    ("q", 0, 7),
    ("dq", 7, 14),
    ("tau_J", 14, 21),
    ("tau_ext_hat_filtered", 21, 28),
    ("O_T_EE_position", 28, 31),
    ("O_T_EE_quaternion", 31, 35),
    ("O_F_ext_hat_K", 35, 41),
    ("q_d", 41, 48),
    ("O_T_EE_c_position", 48, 51),
    ("O_T_EE_c_quaternion", 51, 55),
    ("joint_contact", 55, 62),
    ("cartesian_contact", 62, 68),
    ("time", 68, 69),
];

/// Length of `flat()`.
pub const FLAT_LEN: usize = 69;

pub fn flat_layout(py: Python<'_>) -> PyResult<Bound<'_, PyList>> {
    PyList::new(py, FLAT_LAYOUT.iter().copied())
}

/// A column-major `[f64; R*C]` as a numpy `(R, C)` matrix, `M[i, j]` = row `i`, column `j`.
pub(crate) fn to_matrix<'py>(py: Python<'py>, m: &[f64], rows: usize) -> Bound<'py, PyArray2<f64>> {
    let cols = m.len() / rows;
    Array2::from_shape_fn((rows, cols), |(r, c)| m[c * rows + r]).into_pyarray(py)
}

fn accelerometers<'py>(py: Python<'py>, a: &[[f64; 3]; 6]) -> Bound<'py, PyArray2<f64>> {
    Array2::from_shape_fn((6, 3), |(i, j)| a[i][j]).into_pyarray(py)
}

/// Translation and unit quaternion `(x, y, z, w)` with `w >= 0` of a column-major pose.
fn translation_and_quaternion(pose: &[f64; 16]) -> ([f64; 3], [f64; 4]) {
    let rotation: Matrix3<f64> = Matrix4::from_column_slice(pose)
        .fixed_view::<3, 3>(0, 0)
        .into_owned();
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
    let sign = if q.w < 0.0 { -1.0 } else { 1.0 };
    (
        [pose[12], pose[13], pose[14]],
        [sign * q.i, sign * q.j, sign * q.k, sign * q.w],
    )
}

fn mode_name(mode: franka::RobotMode) -> &'static str {
    use franka::RobotMode::*;
    match mode {
        Other => "other",
        Idle => "idle",
        Move => "move",
        Guiding => "guiding",
        Reflex => "reflex",
        UserStopped => "user_stopped",
        AutomaticErrorRecovery => "automatic_error_recovery",
    }
}

/// One robot state (libfranka's `RobotState`). Arrays are float64 numpy arrays, 4x4 poses
/// `(4, 4)` matrices with the translation in `M[:3, 3]` (m); `flat()` follows `FLAT_LAYOUT`.
#[pyclass(frozen, name = "RobotState", module = "franka")]
pub struct RobotState {
    inner: franka::RobotState,
}

impl From<franka::RobotState> for RobotState {
    fn from(inner: franka::RobotState) -> Self {
        RobotState { inner }
    }
}

/// One `#[pymethods]` block (a second needs pyo3's `multiple-pymethods`): a getter per
/// listed field, then the hand-written methods.
macro_rules! robot_state_methods {
    (
        arrays: [$($arr:ident),*],
        matrices: [$($mat:ident: $rows:literal),*],
        scalars: [$($scalar:ident),*]
    ) => {
        #[pymethods]
        impl RobotState {
            $(
                #[getter]
                fn $arr<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
                    PyArray1::from_slice(py, &self.inner.$arr)
                }
            )*
            $(
                #[getter]
                fn $mat<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
                    to_matrix(py, &self.inner.$mat, $rows)
                }
            )*
            $(
                #[getter]
                fn $scalar(&self) -> f64 {
                    self.inner.$scalar
                }
            )*

            #[classattr]
            fn FLAT_LAYOUT(py: Python<'_>) -> PyResult<Bound<'_, PyList>> {
                flat_layout(py)
            }

            #[classattr]
            fn FLAT_LEN() -> usize {
                FLAT_LEN
            }

            /// `(6, 3)`, m/s^2; zero on an FER.
            #[getter]
            fn accelerometer_top<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
                accelerometers(py, &self.inner.accelerometer_top)
            }

            /// `(6, 3)`, m/s^2; zero on an FER.
            #[getter]
            fn accelerometer_bottom<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
                accelerometers(py, &self.inner.accelerometer_bottom)
            }

            /// Names of the errors active on the robot.
            #[getter]
            fn current_errors(&self) -> Vec<&'static str> {
                self.inner.current_errors.names()
            }

            /// Names of the errors that aborted the last motion.
            #[getter]
            fn last_motion_errors(&self) -> Vec<&'static str> {
                self.inner.last_motion_errors.names()
            }

            /// 'other', 'idle', 'move', 'guiding', 'reflex', 'user_stopped' or
            /// 'automatic_error_recovery'.
            #[getter]
            fn robot_mode(&self) -> &'static str {
                mode_name(self.inner.robot_mode)
            }

            /// The robot's timestamp, seconds.
            #[getter]
            fn time(&self) -> f64 {
                self.inner.time.as_secs_f64()
            }

            /// The state as a 1-D float64 array of length `FLAT_LEN`, laid out as `FLAT_LAYOUT`.
            fn flat<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
                let s = &self.inner;
                let (ee_p, ee_q) = translation_and_quaternion(&s.O_T_EE);
                let (c_p, c_q) = translation_and_quaternion(&s.O_T_EE_c);
                let parts: [&[f64]; 13] = [
                    &s.q, &s.dq, &s.tau_J, &s.tau_ext_hat_filtered, &ee_p, &ee_q,
                    &s.O_F_ext_hat_K, &s.q_d, &c_p, &c_q, &s.joint_contact,
                    &s.cartesian_contact, &[s.time.as_secs_f64()],
                ];
                let out = parts.concat();
                debug_assert_eq!(out.len(), FLAT_LEN);
                PyArray1::from_slice(py, &out)
            }

            fn __repr__(&self) -> String {
                let p = &self.inner.O_T_EE;
                format!(
                    "RobotState(robot_mode={:?}, time={:.3}, ee=[{:.4}, {:.4}, {:.4}])",
                    mode_name(self.inner.robot_mode), self.inner.time.as_secs_f64(),
                    p[12], p[13], p[14]
                )
            }
        }
    };
}

robot_state_methods! {
    arrays: [
        q, q_d, dq, dq_d, ddq_d, tau_J, tau_J_d, dtau_J, theta, dtheta,
        joint_contact, joint_collision, cartesian_contact, cartesian_collision,
        tau_ext_hat_filtered, O_F_ext_hat_K, K_F_ext_hat_K,
        O_dP_EE_d, O_dP_EE_c, O_ddP_EE_c, O_ddP_O,
        elbow, elbow_d, elbow_c, delbow_c, ddelbow_c,
        F_x_Cee, F_x_Cload, F_x_Ctotal
    ],
    matrices: [
        O_T_EE: 4, O_T_EE_d: 4, O_T_EE_c: 4, F_T_EE: 4, F_T_NE: 4, NE_T_EE: 4, EE_T_K: 4,
        I_ee: 3, I_load: 3, I_total: 3
    ],
    scalars: [m_ee, m_load, m_total, control_command_success_rate]
}
