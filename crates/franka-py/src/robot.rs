//! `franka.Robot`: the connection, the setters and the two `*_targets` starters.

use std::sync::Arc;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use franka::{
    ControllerMode, FciVersion, JointTargetControlOptions, OtgLimits, RealtimeConfig,
    TargetControlOptions,
};

use crate::gripper::Gripper;
use crate::model::Model;
use crate::state::RobotState;
use crate::targets::{CartesianTargets, JointTargets};
use crate::IntoPyResult;

/// `realtime=`, else `FRANKA_REALTIME` (as the examples read it), else `enforce`.
fn realtime_config(realtime: Option<String>) -> PyResult<RealtimeConfig> {
    let value = realtime.or_else(|| std::env::var("FRANKA_REALTIME").ok());
    match value.as_deref() {
        None | Some("enforce") => Ok(RealtimeConfig::Enforce),
        Some("ignore") => Ok(RealtimeConfig::Ignore),
        Some(other) => Err(PyValueError::new_err(format!(
            "franka: realtime must be 'enforce' or 'ignore' (also read from FRANKA_REALTIME), got {other:?}"
        ))),
    }
}

fn parse_controller_mode(name: &str) -> PyResult<ControllerMode> {
    match name {
        "joint_impedance" => Ok(ControllerMode::JointImpedance),
        "cartesian_impedance" => Ok(ControllerMode::CartesianImpedance),
        other => Err(PyValueError::new_err(format!(
            "franka: controller_mode must be 'joint_impedance' or 'cartesian_impedance', got {other:?}"
        ))),
    }
}

/// A connection to a robot's control box. `realtime` is 'enforce' (the default, as
/// libfranka: needs PREEMPT_RT and SCHED_FIFO) or 'ignore'; unset, `FRANKA_REALTIME` decides.
/// Every method that talks to the robot releases the GIL.
#[pyclass(frozen, name = "Robot", module = "franka")]
pub struct Robot {
    inner: Arc<franka::Robot>,
    address: String,
}

/// Runs `f` with the GIL released and maps its error.
fn detached<T: Send>(
    py: Python<'_>,
    robot: &Arc<franka::Robot>,
    f: impl FnOnce(&Arc<franka::Robot>) -> franka::FrankaResult<T> + Send,
) -> PyResult<T> {
    let robot = Arc::clone(robot);
    py.detach(move || f(&robot)).into_py(py)
}

#[pymethods]
impl Robot {
    #[new]
    #[pyo3(signature = (address, *, realtime = None, log_size = None))]
    fn new(
        py: Python<'_>,
        address: String,
        realtime: Option<String>,
        log_size: Option<usize>,
    ) -> PyResult<Self> {
        let config = realtime_config(realtime)?;
        let robot = {
            let address = address.clone();
            py.detach(move || match log_size {
                Some(size) => franka::Robot::new_with_log_size(&address, config, size),
                None => franka::Robot::new(&address, config),
            })
            .into_py(py)?
        };
        Ok(Robot {
            inner: Arc::new(robot),
            address,
        })
    }

    #[getter]
    fn address(&self) -> &str {
        &self.address
    }

    /// The negotiated FCI protocol version: 10 (FR3) or 5 (FER).
    #[getter]
    fn fci_version(&self) -> u16 {
        match self.inner.fci_version() {
            FciVersion::V5 => 5,
            FciVersion::V10 => 10,
        }
    }

    /// The software version the control box reported.
    #[getter]
    fn server_version(&self) -> u16 {
        self.inner.server_version()
    }

    /// Waits for the next robot state (not while a target control runs: use its `state()`).
    fn read_once(&self, py: Python<'_>) -> PyResult<RobotState> {
        detached(py, &self.inner, |r| r.read_once()).map(RobotState::from)
    }

    /// Clears the errors of a reflex or collision so that a new motion can start.
    fn automatic_error_recovery(&self, py: Python<'_>) -> PyResult<()> {
        detached(py, &self.inner, |r| r.automatic_error_recovery())
    }

    /// Stops the running motion from any thread; a target control's `stop()` then reports
    /// the preemption as a `ControlException`.
    fn stop(&self, py: Python<'_>) -> PyResult<()> {
        detached(py, &self.inner, |r| r.stop())
    }

    /// Torque (Nm, 7 each) and force (N/Nm, 6 each) thresholds, lower/upper, for the
    /// acceleration and the nominal phase; the Rust `set_collision_behavior` arguments.
    #[allow(clippy::too_many_arguments)]
    fn set_collision_behavior(
        &self,
        py: Python<'_>,
        lower_torque_thresholds_acceleration: [f64; 7],
        upper_torque_thresholds_acceleration: [f64; 7],
        lower_torque_thresholds_nominal: [f64; 7],
        upper_torque_thresholds_nominal: [f64; 7],
        lower_force_thresholds_acceleration: [f64; 6],
        upper_force_thresholds_acceleration: [f64; 6],
        lower_force_thresholds_nominal: [f64; 6],
        upper_force_thresholds_nominal: [f64; 6],
    ) -> PyResult<()> {
        detached(py, &self.inner, |r| {
            r.set_collision_behavior(
                lower_torque_thresholds_acceleration,
                upper_torque_thresholds_acceleration,
                lower_torque_thresholds_nominal,
                upper_torque_thresholds_nominal,
                lower_force_thresholds_acceleration,
                upper_force_thresholds_acceleration,
                lower_force_thresholds_nominal,
                upper_force_thresholds_nominal,
            )
        })
    }

    /// `set_collision_behavior` with the same thresholds for both phases.
    fn set_collision_behavior_simple(
        &self,
        py: Python<'_>,
        lower_torque_thresholds: [f64; 7],
        upper_torque_thresholds: [f64; 7],
        lower_force_thresholds: [f64; 6],
        upper_force_thresholds: [f64; 6],
    ) -> PyResult<()> {
        detached(py, &self.inner, |r| {
            r.set_collision_behavior_simple(
                lower_torque_thresholds,
                upper_torque_thresholds,
                lower_force_thresholds,
                upper_force_thresholds,
            )
        })
    }

    /// Joint impedance of the internal controller, Nm/rad, 7 values.
    fn set_joint_impedance(&self, py: Python<'_>, K_theta: [f64; 7]) -> PyResult<()> {
        detached(py, &self.inner, |r| r.set_joint_impedance(K_theta))
    }

    /// Cartesian impedance of the internal controller (x, y, z, roll, pitch, yaw).
    fn set_cartesian_impedance(&self, py: Python<'_>, K_x: [f64; 6]) -> PyResult<()> {
        detached(py, &self.inner, |r| r.set_cartesian_impedance(K_x))
    }

    /// Starts a Cartesian target loop on a Rust thread. The budgets are norms: translation
    /// in m/s, m/s^2, m/s^3, rotation in rad/s, rad/s^2, rad/s^3 (`None`: the Rust defaults);
    /// `max_deviation` (m) and `max_angular_deviation` (rad) end the loop with a
    /// `ControlException` when the measured pose strays that far from the start.
    #[pyo3(signature = (
        *, max_velocity = 0.3, max_acceleration = 0.5, max_jerk = 20.0, max_deviation = 0.30,
        max_angular_velocity = None, max_angular_acceleration = None, max_angular_jerk = None,
        max_angular_deviation = None, controller_mode = "cartesian_impedance", limit_rate = true,
        realtime_priority = None
    ))]
    #[allow(clippy::too_many_arguments)]
    fn cartesian_targets(
        &self,
        py: Python<'_>,
        max_velocity: f64,
        max_acceleration: f64,
        max_jerk: f64,
        max_deviation: f64,
        max_angular_velocity: Option<f64>,
        max_angular_acceleration: Option<f64>,
        max_angular_jerk: Option<f64>,
        max_angular_deviation: Option<f64>,
        controller_mode: &str,
        limit_rate: bool,
        realtime_priority: Option<i32>,
    ) -> PyResult<CartesianTargets> {
        let defaults = TargetControlOptions::default();
        let rotation = OtgLimits {
            max_velocity: max_angular_velocity.unwrap_or(defaults.rotation_limits.max_velocity),
            max_acceleration: max_angular_acceleration
                .unwrap_or(defaults.rotation_limits.max_acceleration),
            max_jerk: max_angular_jerk.unwrap_or(defaults.rotation_limits.max_jerk),
        };
        let angular_deviation = max_angular_deviation.unwrap_or(defaults.max_angular_deviation);
        let options = defaults
            .with_limits(OtgLimits {
                max_velocity,
                max_acceleration,
                max_jerk,
            })
            .with_rotation_limits(rotation)
            .with_max_deviation(max_deviation)
            .with_max_angular_deviation(angular_deviation)
            .with_controller_mode(parse_controller_mode(controller_mode)?)
            .with_limit_rate(limit_rate)
            .with_realtime_priority(realtime_priority);
        detached(py, &self.inner, |r| {
            CartesianTargets::start(r.start_cartesian_target_control(options)?)
        })
    }

    /// Starts a joint target loop on a Rust thread with `fraction` of the robot's joint
    /// velocity, acceleration and jerk limits as its budget; `max_deviation` in rad. Use the
    /// result as a context manager.
    #[pyo3(signature = (
        *, fraction = 0.2, max_deviation = 1.0, controller_mode = "joint_impedance",
        limit_rate = true, realtime_priority = None
    ))]
    fn joint_targets(
        &self,
        py: Python<'_>,
        fraction: f64,
        max_deviation: f64,
        controller_mode: &str,
        limit_rate: bool,
        realtime_priority: Option<i32>,
    ) -> PyResult<JointTargets> {
        if !(fraction.is_finite() && fraction > 0.0) {
            return Err(PyValueError::new_err(format!(
                "franka: fraction must be positive, got {fraction}"
            )));
        }
        let limits = JointTargetControlOptions::scaled_limits(self.inner.fci_version(), fraction);
        let options = JointTargetControlOptions::default()
            .with_limits(limits)
            .with_max_deviation(max_deviation)
            .with_controller_mode(parse_controller_mode(controller_mode)?)
            .with_limit_rate(limit_rate)
            .with_realtime_priority(realtime_priority);
        detached(py, &self.inner, |r| {
            JointTargets::start(r.start_joint_target_control(options)?)
        })
    }

    /// The robot's kinematic and dynamic model: an FR3's URDF is fetched from the robot, an
    /// FER's is built in; nothing is downloaded and executed on either.
    fn model(&self, py: Python<'_>) -> PyResult<Model> {
        detached(py, &self.inner, |r| r.load_model()).map(Model::from)
    }

    /// Connects to this robot's Franka Hand (port 1338 of the same host).
    fn gripper(&self, py: Python<'_>) -> PyResult<Gripper> {
        let host = match self.address.rsplit_once(':') {
            Some((host, port)) if port.parse::<u16>().is_ok() => host,
            _ => self.address.as_str(),
        };
        Gripper::connect(py, host)
    }

    fn __repr__(&self) -> String {
        format!(
            "Robot({:?}, fci_version={}, server_version={})",
            self.address,
            self.fci_version(),
            self.inner.server_version()
        )
    }
}
