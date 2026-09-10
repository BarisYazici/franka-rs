//! The `backend=` keyword arguments of `Robot.cartesian_targets` / `joint_targets`, parsed
//! into a [`franka::Backend`]. `None` keeps the Rust default; a wrong shape is a `ValueError`.

use numpy::{AllowTypeChange, PyArrayLike1};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use franka::{Backend, ImpedanceOptions, Leash};

/// The impedance keyword arguments as Python objects; `joint_targets` leaves the Cartesian
/// gains and the posture `None`. `leash` is `(metres, radians)` on the Cartesian interface
/// and one float, radians, on the joint interface (`joint_leash`).
pub(crate) struct ImpedanceArgs<'py> {
    pub cartesian_stiffness: Option<Bound<'py, PyAny>>,
    pub cartesian_damping: Option<Bound<'py, PyAny>>,
    pub joint_stiffness: Option<Bound<'py, PyAny>>,
    pub joint_damping: Option<Bound<'py, PyAny>>,
    pub torque_limits: Option<Bound<'py, PyAny>>,
    pub posture: Option<Bound<'py, PyAny>>,
    pub torque_cutoff: Option<f64>,
    pub velocity_feedforward: bool,
    pub leash: Option<Bound<'py, PyAny>>,
    pub joint_leash: bool,
    pub project_joint_gains: bool,
}

impl Default for ImpedanceArgs<'_> {
    fn default() -> Self {
        ImpedanceArgs {
            cartesian_stiffness: None,
            cartesian_damping: None,
            joint_stiffness: None,
            joint_damping: None,
            torque_limits: None,
            posture: None,
            torque_cutoff: None,
            velocity_feedforward: true,
            leash: None,
            joint_leash: false,
            project_joint_gains: false,
        }
    }
}

/// `value` as the leash: `(metres, radians)` for a pose, one float (radians) for joints.
fn leash(value: &Bound<'_, PyAny>, joint: bool, default: Leash) -> PyResult<Leash> {
    if joint {
        let radians: f64 = value.extract().map_err(|e| {
            PyValueError::new_err(format!(
                "franka: leash takes one float (rad) for joints: {e}"
            ))
        })?;
        return Ok(Leash {
            joint: radians,
            ..default
        });
    }
    let [translation, rotation] = vector::<2>("leash", "metres, radians", value, None)?;
    Ok(Leash {
        translation,
        rotation,
        ..default
    })
}

const CARTESIAN_UNIT: &str = "x, y, z, then three rotational";

/// `value` as `N` floats. With `translational` set, one float is also accepted: it replaces
/// the first three entries of that default and keeps the rotational three.
fn vector<const N: usize>(
    name: &str,
    unit: &str,
    value: &Bound<'_, PyAny>,
    translational: Option<[f64; N]>,
) -> PyResult<[f64; N]> {
    let shape = || {
        let scalar = if translational.is_some() {
            ", or one float for the three translational entries"
        } else {
            ""
        };
        format!("franka: {name} takes {N} values ({unit}){scalar}")
    };
    let wrap = |e: PyErr| PyValueError::new_err(format!("{}: {e}", shape()));
    if let Some(default) = translational {
        if value.len().is_err() {
            let mut out = default;
            out[..3].fill(value.extract().map_err(wrap)?);
            return Ok(out);
        }
    }
    let array: PyArrayLike1<'_, f64, AllowTypeChange> = value.extract().map_err(wrap)?;
    <[f64; N]>::try_from(array.as_array().to_vec())
        .map_err(|got| PyValueError::new_err(format!("{}, got {} values", shape(), got.len())))
}

impl ImpedanceArgs<'_> {
    /// `defaults` with every given argument applied through its `with_*` builder.
    fn options(&self, defaults: ImpedanceOptions) -> PyResult<ImpedanceOptions> {
        let mut gains = defaults.gains;
        if let Some(v) = &self.cartesian_stiffness {
            let unit = format!("{CARTESIAN_UNIT}, N/m and Nm/rad");
            gains.cartesian_stiffness = vector(
                "cartesian_stiffness",
                &unit,
                v,
                Some(gains.cartesian_stiffness),
            )?;
        }
        if let Some(v) = &self.cartesian_damping {
            let unit = format!("{CARTESIAN_UNIT}, Ns/m and Nms/rad");
            gains.cartesian_damping =
                vector("cartesian_damping", &unit, v, Some(gains.cartesian_damping))?;
        }
        if let Some(v) = &self.joint_stiffness {
            gains.joint_stiffness = vector("joint_stiffness", "Nm/rad", v, None)?;
        }
        if let Some(v) = &self.joint_damping {
            gains.joint_damping = vector("joint_damping", "Nms/rad", v, None)?;
        }
        let mut options = defaults.with_gains(gains);
        if let Some(v) = &self.torque_limits {
            options = options.with_torque_limits(vector("torque_limits", "Nm", v, None)?);
        }
        if let Some(v) = &self.posture {
            options = options.with_posture(Some(vector("posture", "rad", v, None)?));
        }
        if let Some(hz) = self.torque_cutoff {
            options = options.with_cutoff_frequency(hz);
        }
        if let Some(v) = &self.leash {
            options = options.with_leash(leash(v, self.joint_leash, options.leash)?);
        }
        Ok(options
            .with_velocity_feedforward(self.velocity_feedforward)
            .with_project_joint_gains(self.project_joint_gains))
    }

    /// The first argument that was given, for the `backend='robot'` refusal.
    fn first_given(&self) -> Option<&'static str> {
        [
            ("cartesian_stiffness", self.cartesian_stiffness.is_some()),
            ("cartesian_damping", self.cartesian_damping.is_some()),
            ("joint_stiffness", self.joint_stiffness.is_some()),
            ("joint_damping", self.joint_damping.is_some()),
            ("torque_limits", self.torque_limits.is_some()),
            ("posture", self.posture.is_some()),
            ("torque_cutoff", self.torque_cutoff.is_some()),
            ("leash", self.leash.is_some()),
            ("velocity_feedforward=False", !self.velocity_feedforward),
            ("project_joint_gains=True", self.project_joint_gains),
        ]
        .into_iter()
        .find_map(|(name, given)| given.then_some(name))
    }

    /// `backend` is `'impedance'` (the crate's torques: `defaults` plus these arguments) or
    /// `'robot'` (the robot's own controller; every impedance argument must then be `None`).
    pub(crate) fn backend(&self, backend: &str, defaults: ImpedanceOptions) -> PyResult<Backend> {
        match backend {
            "impedance" => Ok(Backend::Impedance(self.options(defaults)?)),
            "robot" => match self.first_given() {
                Some(name) => Err(PyValueError::new_err(format!(
                    "franka: {name} applies to backend='impedance' only, not to backend='robot'"
                ))),
                None => Ok(Backend::RobotController),
            },
            other => Err(PyValueError::new_err(format!(
                "franka: backend must be 'impedance' or 'robot', got {other:?}"
            ))),
        }
    }
}
