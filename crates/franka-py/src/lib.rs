//! Python bindings for `franka-rs` (`import franka`): the extension module `franka._franka`.
//!
//! The 1 kHz loop is [`franka::robot::target_control`] on a Rust thread that never takes the
//! GIL; every Python call is a write into its slot or a copy out of a mutex.

#![allow(non_snake_case)]

use pyo3::exceptions::PyException;
use pyo3::prelude::*;

mod gripper;
mod model;
mod robot;
mod state;
mod targets;

pyo3::create_exception!(
    franka,
    FrankaError,
    PyException,
    "Any franka-rs error; `str(e)` is the Rust message, `e.kind` names the variant."
);

pyo3::create_exception!(
    franka,
    ControlException,
    FrankaError,
    "A motion ended abnormally; `e.reason` is the reflex reason, `e.errors` its flag names."
);

fn kind(e: &franka::FrankaError) -> &'static str {
    use franka::FrankaError::*;
    match e {
        Network(_) => "network",
        Protocol(_) => "protocol",
        IncompatibleVersion { .. } => "incompatible_version",
        Command(_) => "command",
        Control(_) => "control",
        Realtime(_) => "realtime",
        InvalidOperation(_) => "invalid_operation",
        InvalidArgument(_) => "invalid_argument",
        Model(_) => "model",
    }
}

/// Maps a Rust error to its Python exception; attribute writes on the instance cannot fail.
pub(crate) fn to_py(py: Python<'_>, e: franka::FrankaError) -> PyErr {
    let kind = kind(&e);
    let message = e.to_string();
    let err = match e {
        franka::FrankaError::Control(exception) => {
            let err = PyErr::new::<ControlException, _>(message);
            let value = err.value(py);
            let _ = value.setattr("reason", exception.last_motion_errors.to_string());
            let _ = value.setattr("errors", exception.last_motion_errors.names());
            let _ = value.setattr(
                "move_status",
                exception.move_status.map(|status| format!("{status:?}")),
            );
            err
        }
        _ => PyErr::new::<FrankaError, _>(message),
    };
    let _ = err.value(py).setattr("kind", kind);
    err
}

/// `?` for a [`franka::FrankaResult`] inside a `#[pymethods]` body.
pub(crate) trait IntoPyResult<T> {
    fn into_py(self, py: Python<'_>) -> PyResult<T>;
}

impl<T> IntoPyResult<T> for franka::FrankaResult<T> {
    fn into_py(self, py: Python<'_>) -> PyResult<T> {
        self.map_err(|e| to_py(py, e))
    }
}

#[pymodule]
fn _franka(m: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = m.py();
    m.add("FrankaError", py.get_type::<FrankaError>())?;
    m.add("ControlException", py.get_type::<ControlException>())?;
    m.add_class::<robot::Robot>()?;
    m.add_class::<model::Model>()?;
    m.add_class::<state::RobotState>()?;
    m.add_class::<targets::CartesianTargets>()?;
    m.add_class::<targets::JointTargets>()?;
    m.add_class::<gripper::Gripper>()?;
    m.add_class::<gripper::GripperState>()?;
    m.add("FLAT_LAYOUT", state::flat_layout(py)?)?;
    m.add("FLAT_LEN", state::FLAT_LEN)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
