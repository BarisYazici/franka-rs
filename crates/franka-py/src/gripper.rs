//! `franka.Gripper`: the Franka Hand on port 1338; every command releases the GIL.

use pyo3::prelude::*;

use crate::IntoPyResult;

/// One gripper state: `width` and `max_width` (m), `is_grasped`, `temperature` (C), `time` (s).
#[pyclass(frozen, name = "GripperState", module = "franka")]
pub struct GripperState {
    #[pyo3(get)]
    width: f64,
    #[pyo3(get)]
    max_width: f64,
    #[pyo3(get)]
    is_grasped: bool,
    #[pyo3(get)]
    temperature: u16,
    #[pyo3(get)]
    time: f64,
}

#[pymethods]
impl GripperState {
    fn __repr__(&self) -> String {
        format!(
            "GripperState(width={:.4}, max_width={:.4}, is_grasped={}, temperature={}, time={:.3})",
            self.width, self.max_width, self.is_grasped, self.temperature, self.time
        )
    }
}

impl From<franka::GripperState> for GripperState {
    fn from(s: franka::GripperState) -> Self {
        GripperState {
            width: s.width,
            max_width: s.max_width,
            is_grasped: s.is_grasped,
            temperature: s.temperature,
            time: s.time.as_secs_f64(),
        }
    }
}

/// The Franka Hand. Commands return `True` on success, `False` when the hand reports
/// "unsuccessful" (a grasp that did not hold), and raise `FrankaError` when they fail.
#[pyclass(frozen, name = "Gripper", module = "franka")]
pub struct Gripper {
    inner: franka::Gripper,
}

impl Gripper {
    pub(crate) fn connect(py: Python<'_>, address: &str) -> PyResult<Self> {
        let inner = py.detach(|| franka::Gripper::new(address)).into_py(py)?;
        Ok(Gripper { inner })
    }
}

#[pymethods]
impl Gripper {
    /// Connects to the hand of the robot at `address` (`"host"` or `"host:port"`, default 1338).
    #[new]
    fn new(py: Python<'_>, address: String) -> PyResult<Self> {
        Gripper::connect(py, &address)
    }

    #[getter]
    fn server_version(&self) -> u16 {
        self.inner.server_version()
    }

    /// Homes the hand and estimates `max_width`; needed after changing the fingers.
    fn homing(&self, py: Python<'_>) -> PyResult<bool> {
        py.detach(|| self.inner.homing()).into_py(py)
    }

    /// Moves the fingers to `width` (m) at `speed` (m/s).
    #[pyo3(name = "move")]
    fn move_(&self, py: Python<'_>, width: f64, speed: f64) -> PyResult<bool> {
        py.detach(|| self.inner.move_gripper(width, speed))
            .into_py(py)
    }

    /// Grasps at `width` (m), `speed` (m/s), `force` (N); success when the fingers end within
    /// `[width - epsilon_inner, width + epsilon_outer]`.
    #[pyo3(signature = (width, speed, force, epsilon_inner = 0.005, epsilon_outer = 0.005))]
    fn grasp(
        &self,
        py: Python<'_>,
        width: f64,
        speed: f64,
        force: f64,
        epsilon_inner: f64,
        epsilon_outer: f64,
    ) -> PyResult<bool> {
        py.detach(|| {
            self.inner
                .grasp(width, speed, force, epsilon_inner, epsilon_outer)
        })
        .into_py(py)
    }

    /// Stops a running move or grasp.
    fn stop(&self, py: Python<'_>) -> PyResult<bool> {
        py.detach(|| self.inner.stop()).into_py(py)
    }

    /// Waits for the next gripper state.
    fn read_once(&self, py: Python<'_>) -> PyResult<GripperState> {
        py.detach(|| self.inner.read_once())
            .into_py(py)
            .map(GripperState::from)
    }

    fn __repr__(&self) -> String {
        format!("Gripper(server_version={})", self.inner.server_version())
    }
}
