//! `franka.CartesianTargets` / `franka.JointTargets`: the Python side of a running
//! [`franka::robot::target_control`] loop, over one generic slot feeder ([`Core`]).
//!
//! Lock order everywhere: the follow state, then the control handle. The worker publishes a
//! row under the follow lock, so a `move_*` that cleared the chunk cannot be overtaken.

use std::ffi::CString;
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use numpy::{AllowTypeChange, PyArray1, PyArray2, PyArrayLike1, PyArrayLike2};
use pyo3::exceptions::{PyRuntimeWarning, PyValueError};
use pyo3::prelude::*;

use franka::{CartesianTargetControl, FrankaError, FrankaResult, JointTargetControl};

use crate::state::{to_matrix, RobotState};
use crate::{to_py, IntoPyResult};

const STOPPED_MESSAGE: &str = "franka: this target control was stopped; start a new one with \
     Robot.cartesian_targets() or Robot.joint_targets().";

/// What the two Rust handles have in common: a full `Target` in the slot, an absolute
/// input `Row` and a relative `Delta`, each applied to the current target.
pub(crate) trait Handle: Send + 'static {
    type Target: Copy + Send + 'static;
    type Row: Copy + Send + 'static;
    type Delta: Copy;
    fn parse(row: &[f64]) -> PyResult<Self::Row>;
    fn parse_delta(row: &[f64]) -> PyResult<Self::Delta>;
    fn absolute(current: Self::Target, row: Self::Row) -> Self::Target;
    fn relative(current: Self::Target, delta: Self::Delta) -> Self::Target;
    fn set(&self, target: Self::Target) -> FrankaResult<()>;
    fn target(&self) -> Self::Target;
    fn state(&self) -> franka::RobotState;
    fn is_running(&self) -> bool;
    fn stop(self) -> FrankaResult<()>;
}

fn orientation(target: &[f64; 7]) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(target[6], target[3], target[4], target[5]))
}

fn pose(p: [f64; 3], q: UnitQuaternion<f64>) -> [f64; 7] {
    [p[0], p[1], p[2], q.i, q.j, q.k, q.w]
}

/// Position + quaternion `[x, y, z, w]`; a row is a position and an optional orientation,
/// a delta a translation and an optional rotation vector.
impl Handle for CartesianTargetControl {
    type Target = [f64; 7];
    type Row = ([f64; 3], Option<UnitQuaternion<f64>>);
    type Delta = ([f64; 3], Option<Vector3<f64>>);

    fn parse(row: &[f64]) -> PyResult<Self::Row> {
        match row {
            [x, y, z] => Ok(([*x, *y, *z], None)),
            [x, y, z, i, j, k, w] => {
                let q = UnitQuaternion::try_new(Quaternion::new(*w, *i, *j, *k), 1e-12)
                    .ok_or_else(|| {
                        PyValueError::new_err("franka: the orientation quaternion has zero length")
                    })?;
                Ok(([*x, *y, *z], Some(q)))
            }
            _ => Err(PyValueError::new_err(format!(
                "franka: a Cartesian target has 3 elements (x, y, z in m) or 7 (plus a unit \
                 quaternion x, y, z, w), got {}",
                row.len()
            ))),
        }
    }

    fn parse_delta(row: &[f64]) -> PyResult<Self::Delta> {
        match row {
            [x, y, z] => Ok(([*x, *y, *z], None)),
            [x, y, z, a, b, c] => Ok(([*x, *y, *z], Some(Vector3::new(*a, *b, *c)))),
            _ => Err(PyValueError::new_err(format!(
                "franka: a Cartesian delta has 3 elements (dx, dy, dz in m) or 6 (plus a \
                 rotation vector in rad), got {}",
                row.len()
            ))),
        }
    }

    fn absolute(current: [f64; 7], (position, q): Self::Row) -> [f64; 7] {
        pose(position, q.unwrap_or_else(|| orientation(&current)))
    }

    /// The rotation is composed onto the target orientation in the base frame: `exp(r) * q`.
    fn relative(current: [f64; 7], (delta, rotation): Self::Delta) -> [f64; 7] {
        let q = rotation.map_or_else(
            || orientation(&current),
            |r| UnitQuaternion::from_scaled_axis(r) * orientation(&current),
        );
        pose(std::array::from_fn(|i| current[i] + delta[i]), q)
    }

    fn set(&self, t: [f64; 7]) -> FrankaResult<()> {
        self.set_target([t[0], t[1], t[2]], [t[3], t[4], t[5], t[6]])
    }

    fn target(&self) -> [f64; 7] {
        let (p, q) = (
            CartesianTargetControl::target(self),
            self.target_orientation(),
        );
        [p[0], p[1], p[2], q[0], q[1], q[2], q[3]]
    }

    fn state(&self) -> franka::RobotState {
        CartesianTargetControl::state(self)
    }
    fn is_running(&self) -> bool {
        CartesianTargetControl::is_running(self)
    }
    fn stop(self) -> FrankaResult<()> {
        CartesianTargetControl::stop(self)
    }
}

impl Handle for JointTargetControl {
    type Target = [f64; 7];
    type Row = [f64; 7];
    type Delta = [f64; 7];

    fn parse(row: &[f64]) -> PyResult<[f64; 7]> {
        <[f64; 7]>::try_from(row).map_err(|_| {
            PyValueError::new_err(format!(
                "franka: a joint target has 7 elements (rad), got {}",
                row.len()
            ))
        })
    }
    fn parse_delta(row: &[f64]) -> PyResult<[f64; 7]> {
        Self::parse(row)
    }
    fn absolute(_current: [f64; 7], row: [f64; 7]) -> [f64; 7] {
        row
    }
    fn relative(current: [f64; 7], delta: [f64; 7]) -> [f64; 7] {
        std::array::from_fn(|i| current[i] + delta[i])
    }
    fn set(&self, t: [f64; 7]) -> FrankaResult<()> {
        self.set_joints(t)
    }
    fn target(&self) -> [f64; 7] {
        JointTargetControl::target(self)
    }
    fn state(&self) -> franka::RobotState {
        JointTargetControl::state(self)
    }
    fn is_running(&self) -> bool {
        JointTargetControl::is_running(self)
    }
    fn stop(self) -> FrankaResult<()> {
        JointTargetControl::stop(self)
    }
}

/// The chunk being fed: `rows[next..]` are due, row `i` at `started + i * dt` (`dt` in s).
struct Follow<R> {
    rows: Vec<R>,
    next: usize,
    dt: f64,
    started: Instant,
    quit: bool,
}

/// Shared by the Python object, its worker thread and any calling Python thread.
pub(crate) struct Core<H: Handle> {
    control: Mutex<Option<H>>,
    follow: Mutex<Follow<H::Row>>,
    wake: Condvar,
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

fn stopped() -> FrankaError {
    FrankaError::InvalidOperation(STOPPED_MESSAGE.to_string())
}

impl<H: Handle> Core<H> {
    /// Wraps a started handle and spawns its worker thread.
    fn start(handle: H, name: &str) -> FrankaResult<Arc<Self>> {
        let core = Arc::new(Core {
            control: Mutex::new(Some(handle)),
            follow: Mutex::new(Follow {
                rows: Vec::new(),
                next: 0,
                dt: 0.0,
                started: Instant::now(),
                quit: false,
            }),
            wake: Condvar::new(),
        });
        let worker = Arc::clone(&core);
        std::thread::Builder::new()
            .name(format!("franka-py-{name}-follow"))
            .spawn(move || worker.run())
            .map_err(|e| {
                FrankaError::InvalidOperation(format!(
                    "franka: cannot spawn the follow thread: {e}"
                ))
            })?;
        Ok(core)
    }

    /// The worker: waits until a row is due, publishes it, repeats. A refused row (the loop
    /// ended) drops the rest of the chunk; `stop()` has the reason.
    fn run(&self) {
        let mut follow = lock(&self.follow);
        loop {
            if follow.quit {
                return;
            }
            if follow.next >= follow.rows.len() {
                follow = self
                    .wake
                    .wait(follow)
                    .unwrap_or_else(PoisonError::into_inner);
                continue;
            }
            let wait = follow.dt * follow.next as f64 - follow.started.elapsed().as_secs_f64();
            if wait > 0.0 {
                let timeout = Duration::try_from_secs_f64(wait).unwrap_or(Duration::MAX);
                let waited = self.wake.wait_timeout(follow, timeout);
                follow = waited.unwrap_or_else(PoisonError::into_inner).0;
                continue;
            }
            let row = follow.rows[follow.next];
            follow.next += 1;
            let published = lock(&self.control)
                .as_ref()
                .map(|handle| handle.set(H::absolute(handle.target(), row)));
            if !matches!(published, Some(Ok(()))) {
                follow.rows.clear();
            }
        }
    }

    /// Cancels the chunk and sets `f(current target)`, atomically with respect to the worker.
    fn update(&self, f: impl FnOnce(H::Target) -> H::Target) -> FrankaResult<()> {
        let mut follow = lock(&self.follow);
        follow.rows.clear();
        let control = lock(&self.control);
        let handle = control.as_ref().ok_or_else(stopped)?;
        handle.set(f(handle.target()))
    }

    /// Replaces the chunk: row 0 as soon as the worker wakes, row `i` at `i * dt`.
    fn follow(&self, rows: Vec<H::Row>, dt: f64) -> FrankaResult<()> {
        let mut follow = lock(&self.follow);
        if lock(&self.control).is_none() {
            return Err(stopped());
        }
        follow.rows = rows;
        follow.next = 0;
        follow.dt = dt;
        follow.started = Instant::now();
        drop(follow);
        self.wake.notify_one();
        Ok(())
    }

    fn with_handle<T>(&self, f: impl FnOnce(&H) -> T) -> FrankaResult<T> {
        lock(&self.control).as_ref().map(f).ok_or_else(stopped)
    }

    fn is_running(&self) -> bool {
        self.with_handle(Handle::is_running).unwrap_or(false)
    }

    /// Cancels the chunk, ends the worker and takes the handle out; `None` once taken.
    fn take(&self) -> Option<H> {
        let mut follow = lock(&self.follow);
        follow.rows.clear();
        follow.quit = true;
        let handle = lock(&self.control).take();
        drop(follow);
        self.wake.notify_all();
        handle
    }

    /// Stops the loop and returns its result; `Ok(())` when already stopped.
    fn stop(&self) -> FrankaResult<()> {
        self.take().map_or(Ok(()), Handle::stop)
    }
}

fn parse_dt(dt: f64) -> PyResult<f64> {
    if !(dt.is_finite() && dt > 0.0) {
        return Err(PyValueError::new_err(format!(
            "franka: dt must be positive seconds, got {dt}"
        )));
    }
    Ok(dt)
}

/// Collects a row and parses it with `parse` once every element is finite.
fn parse_row<T>(row: impl Iterator<Item = f64>, parse: fn(&[f64]) -> PyResult<T>) -> PyResult<T> {
    let row: Vec<f64> = row.collect();
    if row.iter().any(|v| !v.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "franka: a target must be finite, got {row:?}"
        )));
    }
    parse(&row)
}

/// The two pyclasses share every method; the macro writes both `#[pymethods]` blocks, with
/// the class-specific `extra` methods spliced in.
macro_rules! targets_class {
    (
        $(#[$doc:meta])*
        $name:ident = $pyname:literal, $handle:ty,
        move_to: $move_to_doc:literal, move_by: $move_by_doc:literal,
        follow: $follow_doc:literal, target: $target_doc:literal,
        extra: { $($extra:tt)* }
    ) => {
        $(#[$doc])*
        #[pyclass(frozen, name = $pyname, module = "franka")]
        pub struct $name {
            core: Arc<Core<$handle>>,
        }

        impl $name {
            pub(crate) fn start(handle: $handle) -> FrankaResult<Self> {
                Ok($name { core: Core::start(handle, $pyname)? })
            }
        }

        impl Drop for $name {
            /// Drops the handle without joining; its own `Drop` requests the loop's stop.
            fn drop(&mut self) {
                drop(self.core.take());
            }
        }

        #[pymethods]
        impl $name {
            #[doc = $move_to_doc]
            fn move_to(&self, py: Python<'_>, target: PyArrayLike1<'_, f64, AllowTypeChange>) -> PyResult<()> {
                let row = parse_row(target.as_array().iter().copied(), <$handle>::parse)?;
                self.core.update(|current| <$handle>::absolute(current, row)).into_py(py)
            }

            #[doc = $move_by_doc]
            fn move_by(&self, py: Python<'_>, delta: PyArrayLike1<'_, f64, AllowTypeChange>) -> PyResult<()> {
                let delta = parse_row(delta.as_array().iter().copied(), <$handle>::parse_delta)?;
                self.core.update(|current| <$handle>::relative(current, delta)).into_py(py)
            }

            #[doc = $follow_doc]
            #[pyo3(signature = (chunk, dt = 0.02))]
            fn follow(&self, py: Python<'_>, chunk: PyArrayLike2<'_, f64, AllowTypeChange>, dt: f64) -> PyResult<()> {
                let dt = parse_dt(dt)?;
                let rows = chunk
                    .as_array()
                    .rows()
                    .into_iter()
                    .map(|row| parse_row(row.iter().copied(), <$handle>::parse))
                    .collect::<PyResult<Vec<_>>>()?;
                self.core.follow(rows, dt).into_py(py)
            }

            #[doc = $target_doc]
            fn target<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray1<f64>>> {
                let target = self.core.with_handle(Handle::target).into_py(py)?;
                Ok(PyArray1::from_slice(py, &target))
            }

            /// The latest robot state the loop received; never blocks on the network.
            fn state(&self, py: Python<'_>) -> PyResult<RobotState> {
                self.core.with_handle(Handle::state).into_py(py).map(RobotState::from)
            }

            /// `False` once the loop ended for any reason or `stop()` was called.
            #[getter]
            fn running(&self) -> bool {
                self.core.is_running()
            }

            /// Cancels a `follow`, settles on the last target, finishes the motion and returns
            /// the loop's result (GIL released). A second call is a no-op.
            fn stop(&self, py: Python<'_>) -> PyResult<()> {
                let core = Arc::clone(&self.core);
                py.detach(move || core.stop()).into_py(py)
            }

            fn __enter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
                slf
            }

            /// Stops the loop; an exception from the block is re-raised as it is and a
            /// `stop()` failure is then only a `RuntimeWarning`.
            fn __exit__(
                &self,
                py: Python<'_>,
                exc_type: &Bound<'_, PyAny>,
                _exc_value: &Bound<'_, PyAny>,
                _traceback: &Bound<'_, PyAny>,
            ) -> PyResult<bool> {
                let core = Arc::clone(&self.core);
                match py.detach(move || core.stop()) {
                    Ok(()) => {}
                    Err(e) if exc_type.is_none() => return Err(to_py(py, e)),
                    Err(e) => {
                        let message = CString::new(format!("franka: stop() failed: {e}")).unwrap_or_default();
                        PyErr::warn(py, &py.get_type::<PyRuntimeWarning>(), &message, 1)?;
                    }
                }
                Ok(false)
            }

            fn __repr__(&self) -> String {
                format!("{}(running={})", $pyname, self.core.is_running())
            }

            $($extra)*
        }
    };
}

targets_class! {
    /// A running Cartesian target control: a pose target in the base frame under the budget
    /// of `Robot.cartesian_targets`. A target is `(x, y, z)` in m, optionally followed by a
    /// unit quaternion `(x, y, z, w)`; a delta is `(dx, dy, dz)` in m, optionally followed
    /// by a rotation vector (axis times angle, rad) in the base frame.
    CartesianTargets = "CartesianTargets", CartesianTargetControl,
    move_to: "Absolute target: 3 elements keep the target orientation, 7 set it (unit \
              quaternion `x, y, z, w`, normalised). Cancels a `follow`; never blocks.",
    move_by: "Moves the *target* (not the measured pose) by (dx, dy, dz) m and, with 6 \
              elements, rotates it by the rotation vector in the base frame. Cancels a `follow`.",
    follow: "Feeds an `(N, 3)` or `(N, 7)` chunk of absolute targets from a Rust thread: row 0 \
             now, row i at `i * dt` s. A new `follow` or `move_*` replaces it. Returns at once.",
    target: "The current target: position (m) and unit quaternion `(x, y, z, w)`, shape `(7,)`.",
    extra: {
        /// The current target pose as a `(4, 4)` matrix, the convention of `RobotState.O_T_EE`.
        fn target_pose<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
            let pose = self.core.with_handle(CartesianTargetControl::target_pose).into_py(py)?;
            Ok(to_matrix(py, &pose, 4))
        }
    }
}

targets_class! {
    /// A running joint target control: a joint position target (rad) under the per-joint
    /// limits of `Robot.joint_targets`.
    JointTargets = "JointTargets", JointTargetControl,
    move_to: "Absolute joint target, 7 values, rad. Cancels a `follow`; never blocks.",
    move_by: "Moves the *target* by `dq` (7 values, rad). Cancels a `follow`.",
    follow: "Feeds an `(N, 7)` chunk of absolute joint targets from a Rust thread: row 0 now, \
             row i at `i * dt` s. A new `follow` or `move_*` replaces it. Returns at once.",
    target: "The current target, rad, shape `(7,)`.",
    extra: {}
}
