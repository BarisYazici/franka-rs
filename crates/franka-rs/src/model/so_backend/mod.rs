//! [`RobotModelBackend`] over the robot-served `libfcimodels` shared object.
//!
//! On FCI v5 (Franka Emika Robot, FER, libfranka 0.9.2) the robot does not publish a
//! URDF. It serves a *compiled* model instead: `Robot::loadModel` issues a
//! `LoadModelLibrary` command, libfranka writes the returned bytes to a
//! temporary file, `dlopen`s it and calls plain C functions out of it
//! (`src/library_downloader.cpp`, `src/library_loader.h`, `src/model_library.h`,
//! `src/libfcimodels.h`, `src/model.cpp`, all of 0.9.2).
//!
//! This module is the Rust equivalent of `franka::ModelLibrary` +
//! `franka::LibraryLoader`: [`SoModelBackend::open`] / [`SoModelBackend::from_bytes`]
//! bind all thirty exported symbols once, up front, exactly like
//! `ModelLibrary::ModelLibrary` does in its initialiser list, and the
//! [`RobotModelBackend`] impl dispatches the ten [`crate::model::Frame`] values
//! onto them the way `franka::Model` does in `model.cpp`.
//!
//! # Symbols and frames
//!
//! `libfcimodels.h` (0.9.2) declares, all `extern "C"` and all column-major:
//!
//! | symbol | signature | used for |
//! |---|---|---|
//! | `O_T_J1 .. O_T_J8` | `(const double q[7], double out[16])` | pose of joints 1..7 and the flange |
//! | `O_T_J9` | `(const double q[7], const double F_T_EE[16], double out[16])` | pose of the end-effector / stiffness frame |
//! | `Ji_J_J1` | `(double out[42])` | body Jacobian of joint 1 — **no `q`** |
//! | `Ji_J_J2 .. Ji_J_J8` | `(const double q[7], double out[42])` | body Jacobian of joints 2..7 and the flange |
//! | `Ji_J_J9` | `(const double q[7], const double F_T_EE[16], double out[42])` | body Jacobian of the end-effector / stiffness frame |
//! | `O_J_J1` | `(double out[42])` | zero Jacobian of joint 1 — **no `q`** |
//! | `O_J_J2 .. O_J_J8` | `(const double q[7], double out[42])` | zero Jacobian of joints 2..7 and the flange |
//! | `O_J_J9` | `(const double q[7], const double F_T_EE[16], double out[42])` | zero Jacobian of the end-effector / stiffness frame |
//! | `M_NE` | `(const double q[7], const double I_load[9], double m_load, const double F_x_Cload[3], double out[49])` | mass matrix |
//! | `c_NE` | `(const double q[7], const double dq[7], const double I_load[9], double m_load, const double F_x_Cload[3], double out[7])` | Coriolis vector |
//! | `g_NE` | `(const double q[7], const double g_earth[3], double m_load, const double F_x_Cload[3], double out[7])` | gravity vector |
//!
//! The frame mapping is `model.cpp`'s: `Joint1..Joint7` are `J1..J7`, `Flange`
//! is `J8`, `EndEffector` is `J9` evaluated with `F_T_EE`, and `Stiffness` is
//! `J9` evaluated with the column-major product `F_T_EE * EE_T_K` — libfranka
//! forms exactly that product with
//! `Eigen::Matrix4d(F_T_EE.data()) * Eigen::Matrix4d(EE_T_K.data())`, and Eigen
//! is column-major by default.
//!
//! # Load parameters
//!
//! The `_load` arguments are named after the *payload* in the C header, but
//! `franka::Model::mass/coriolis/gravity` pass `robot_state.I_total`,
//! `robot_state.m_total` and `robot_state.F_x_Ctotal`, i.e. the **combined**
//! end-effector-plus-payload body. This backend therefore forwards the trait's
//! `i_total / m_total / f_x_ctotal` unchanged, which is what libfranka does.
//!
//! # Gravity and Coriolis
//!
//! `c_NE` takes no gravity vector: the FCI v5 `franka::Model::coriolis`
//! (`model.cpp`, 0.9.2) has no `gravity_earth` parameter at all, unlike the FCI
//! v10 `franka::RobotModel::coriolis` this crate's trait is shaped after.
//! [`RobotModelBackend::coriolis`]'s `gravity_earth` argument is consequently
//! **ignored** here; the Coriolis vector the FER's own model returns is
//! gravity-free by construction. `g_NE` does take `g_earth` and receives the
//! trait's `gravity_earth` verbatim.
//!
//! # Platform
//!
//! The robot serves a native shared object, so the process that loads it must
//! match the `(architecture, system)` pair it asked for. In practice that is
//! `libfcimodels_x64.so` on x86-64 Linux; see
//! [`crate::model::model_library::load_from_robot`].

mod symbols;
mod temp_file;

use symbols::{symbol, ConstJacobianFn, JacobianEeFn, JacobianFn, PoseEeFn, PoseFn, Symbols};
use temp_file::{write_temp_library, TempLibraryFile};

use std::path::Path;

use libloading::Library;

use crate::error::{FrankaError, FrankaResult};
use crate::model::native_backend::DOF;
use crate::model::RobotModelBackend;

/// [`RobotModelBackend`] backed by the robot's own `libfcimodels` shared object.
///
/// Obtain one from [`crate::model::model_library::load_from_robot`] (online) or
/// from [`SoModelBackend::open`] / [`SoModelBackend::from_bytes`] (offline,
/// e.g. from a captured library).
pub struct SoModelBackend {
    /// Resolved function pointers into `library`. Declared first so they are
    /// dropped (a no-op — they are plain `fn` pointers) before the `dlclose`.
    symbols: Symbols,
    /// The `dlopen` handle. Never read after construction, but dropping it
    /// `dlclose`s the library, which invalidates every pointer in `symbols`;
    /// nothing may outlive it.
    _library: Library,
    /// A handle keeping `libm.so.6` mapped in the process's *global* symbol
    /// scope, if one was opened; see [`open_libm_global`]. `libloading::Library::new`
    /// uses `RTLD_LAZY`, so `_library`'s `sin`/`cos`/`sincos` references are not
    /// resolved until first called — this handle must therefore outlive every
    /// call through `symbols`, not just the `dlopen`. `None` when `libm.so.6`
    /// could not be opened; the model library load is then left to fail (or
    /// succeed, if libm was already global by some other path) on its own.
    #[cfg(unix)]
    _libm: Option<libloading::os::unix::Library>,
    /// The temporary file the library was written to, if this backend owns one.
    /// Declared last so the unlink happens after the `dlclose`.
    _temp_file: Option<TempLibraryFile>,
}

impl std::fmt::Debug for SoModelBackend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SoModelBackend").finish_non_exhaustive()
    }
}

// `libloading::Library` wraps a `dlopen` handle, which is `Send + Sync` on every platform
// this crate supports; the resolved symbols are bare `fn` pointers, which are `Send + Sync`
// unconditionally. The generated `libfcimodels` code is pure arithmetic on caller-provided
// buffers with no mutable global state, so concurrent calls do not race. libfranka relies on
// the same property: `franka::Model` is handed out by value and used from control threads.
const _: () = {
    const fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Library>();
    #[cfg(unix)]
    assert_send_sync::<libloading::os::unix::Library>();
};

/// Opens `libm.so.6` into the process's *global* symbol scope, so a later
/// `dlopen` of a library with no `DT_NEEDED` of its own can still resolve
/// libm symbols against it.
///
/// # Why this exists
///
/// The robot-served `libfcimodels_x64.so` calls `sin`, `cos` and `sincos` but
/// carries **no `DT_NEEDED` entries at all** (`readelf -d` on a captured copy
/// prints nothing under `NEEDED`), so the dynamic linker can only resolve
/// those symbols from the *global* scope of the process that `dlopen`s it —
/// confirmed by `nm -D --undefined-only`, which lists `sin`, `cos` and
/// `sincos` as undefined. libfranka's C++ clients get libm for free because
/// libstdc++ itself has a `DT_NEEDED` on it, loaded into the main
/// executable's global scope by the dynamic linker at process start. A Rust
/// binary has no such guarantee: `libloading::Library::new` opens the model
/// library with `RTLD_LAZY | RTLD_LOCAL`
/// (`libloading::os::unix::Library::new`), and if the binary itself never
/// references an libm symbol that survives `--as-needed`, `libm.so.6` may
/// not be mapped at all, or may be mapped without its symbols published
/// globally — either way the first call into the model library fails with
/// `undefined symbol: sincos`.
///
/// Calling this before [`SoModelBackend::load`] opens `libm.so.6` itself with
/// `RTLD_NOW | RTLD_GLOBAL`, publishing its symbols into the scope every
/// subsequent `dlopen` can see — the same effect libstdc++'s `DT_NEEDED`
/// has — independent of whether this binary happens to use libm anywhere
/// else. If `libm.so.6` is already loaded (the common case, since most
/// binaries pull it in transitively), `dlopen` just bumps its reference
/// count and, per POSIX, promotes it to global scope if it was not already.
///
/// # Fallback
///
/// If `libm.so.6` cannot be opened (an unusual libc, a fully static build,
/// ...) this returns `None` rather than panicking; [`SoModelBackend::load`]
/// continues regardless; and lets the model library open — or the first call
/// through it — fail with its own ordinary error if libm truly is not
/// reachable some other way.
#[cfg(unix)]
fn open_libm_global() -> Option<libloading::os::unix::Library> {
    use libloading::os::unix::{Library as UnixLibrary, RTLD_GLOBAL, RTLD_NOW};

    // SAFETY: `libm.so.6` is glibc's math library; opening it runs no code the caller
    // does not already trust the C runtime to run, and publishing its already-defined
    // symbols into the global scope cannot invalidate anything else that is loaded.
    unsafe { UnixLibrary::open(Some("libm.so.6"), RTLD_NOW | RTLD_GLOBAL) }.ok()
}

impl SoModelBackend {
    /// Loads a `libfcimodels` shared object from an existing file.
    ///
    /// The file is *not* removed when the backend is dropped; use
    /// [`SoModelBackend::from_bytes`] for a downloaded library.
    ///
    /// Port of `franka::LibraryLoader` plus `franka::ModelLibrary`'s symbol
    /// binding (`src/library_loader.cpp`, `src/model_library.cpp`, 0.9.2).
    ///
    /// # Safety
    ///
    /// This `dlopen`s `path`, which runs the shared object's `DT_INIT`
    /// constructors and then calls function pointers resolved out of it — i.e.
    /// it executes the file's code in this process, with no sandbox and no
    /// validation of its contents.
    ///
    /// The caller asserts that `path` is a **trusted** `libfcimodels` build for
    /// the current platform: it came from a source the caller is willing to run
    /// arbitrary native code from (a Franka control unit over the FCI command
    /// socket, or a library captured from one and stored somewhere only trusted
    /// principals can write), the file cannot be swapped between this call and
    /// the load, and its thirty exported entry points have the signatures
    /// `libfcimodels.h` of libfranka 0.9.2 declares. Loading anything else is
    /// undefined behaviour.
    ///
    /// # Errors
    ///
    /// [`FrankaError::Model`] when the file cannot be `dlopen`ed (wrong
    /// architecture, missing dependency, not a shared object) or when any of
    /// the thirty expected symbols is absent — libfranka reports the same two
    /// conditions as `ModelException("libfranka: Cannot load model library: ...")`
    /// and `ModelException("libfranka: Symbol cannot be found: ...")`.
    pub unsafe fn open(path: &Path) -> FrankaResult<SoModelBackend> {
        // SAFETY: delegated to this function's own contract — the caller asserts `path`
        // is a trusted `libfcimodels` build for this platform.
        unsafe { SoModelBackend::load(path, None) }
    }

    /// Writes `bytes` to a fresh, `0600`, uniquely named file under
    /// [`std::env::temp_dir`], loads it, and removes the file when the returned
    /// backend is dropped.
    ///
    /// This is what libfranka does with the `LoadModelLibrary` response
    /// (`LibraryDownloader::LibraryDownloader`, which names the file with
    /// `Poco::TemporaryFile::tempName()` and unlinks it in its destructor).
    ///
    /// # Safety
    ///
    /// `bytes` are written to disk and `dlopen`ed, so this executes them in
    /// this process; see [`SoModelBackend::open`] for what that means. The
    /// caller asserts that `bytes` are a **trusted** `libfcimodels` build for
    /// the current platform — in the online path, that trust is the FCI peer
    /// itself, which already commands the arm.
    ///
    /// # Errors
    ///
    /// [`FrankaError::Model`] when the file cannot be created or written
    /// (libfranka: `"libfranka: Cannot save model library."`), or for the
    /// reasons listed on [`SoModelBackend::open`].
    pub unsafe fn from_bytes(bytes: &[u8]) -> FrankaResult<SoModelBackend> {
        let temp_file = write_temp_library(bytes)?;
        let path = temp_file.path.clone();
        // SAFETY: delegated to this function's own contract — the caller asserts `bytes`
        // are a trusted `libfcimodels` build for this platform.
        unsafe { SoModelBackend::load(&path, Some(temp_file)) }
    }

    /// # Safety
    ///
    /// See [`SoModelBackend::open`]: `path` must name a trusted `libfcimodels`
    /// build for the current platform.
    unsafe fn load(
        path: &Path,
        temp_file: Option<TempLibraryFile>,
    ) -> FrankaResult<SoModelBackend> {
        // Publish libm's symbols into the global scope before the model library is
        // opened, so its unresolved `sin`/`cos`/`sincos` references can find them even
        // though it has no `DT_NEEDED` of its own; see `open_libm_global`. Best effort:
        // if this fails, fall through and let the model library open — or its first
        // call — report its own error.
        #[cfg(unix)]
        let libm = open_libm_global();

        // SAFETY: `dlopen` runs the shared object's initialisers, which is arbitrary code.
        // Discharged by this function's contract, which `open` and `from_bytes` re-export
        // as their own `# Safety` sections: the caller asserts the file is a trusted
        // `libfcimodels` build for this platform. In the online path the bytes come from
        // the robot over the FCI command socket, which is exactly the trust boundary
        // libfranka's `LibraryLoader` sits on.
        let library = unsafe { Library::new(path) }.map_err(|e| {
            FrankaError::Model(format!("libfranka: Cannot load model library: {e}"))
        })?;

        // SAFETY: every name below is declared in `src/libfcimodels.h` with the signature
        // the corresponding Rust type alias spells out, and the pointers are stored next to
        // the `Library` that owns them, so they cannot outlive it.
        let symbols = unsafe {
            Symbols {
                o_t_j: [
                    symbol(&library, "O_T_J1")?,
                    symbol(&library, "O_T_J2")?,
                    symbol(&library, "O_T_J3")?,
                    symbol(&library, "O_T_J4")?,
                    symbol(&library, "O_T_J5")?,
                    symbol(&library, "O_T_J6")?,
                    symbol(&library, "O_T_J7")?,
                    symbol(&library, "O_T_J8")?,
                ],
                o_t_j9: symbol(&library, "O_T_J9")?,
                ji_j_j1: symbol(&library, "Ji_J_J1")?,
                ji_j: [
                    symbol(&library, "Ji_J_J2")?,
                    symbol(&library, "Ji_J_J3")?,
                    symbol(&library, "Ji_J_J4")?,
                    symbol(&library, "Ji_J_J5")?,
                    symbol(&library, "Ji_J_J6")?,
                    symbol(&library, "Ji_J_J7")?,
                    symbol(&library, "Ji_J_J8")?,
                ],
                ji_j_j9: symbol(&library, "Ji_J_J9")?,
                o_j_j1: symbol(&library, "O_J_J1")?,
                o_j: [
                    symbol(&library, "O_J_J2")?,
                    symbol(&library, "O_J_J3")?,
                    symbol(&library, "O_J_J4")?,
                    symbol(&library, "O_J_J5")?,
                    symbol(&library, "O_J_J6")?,
                    symbol(&library, "O_J_J7")?,
                    symbol(&library, "O_J_J8")?,
                ],
                o_j_j9: symbol(&library, "O_J_J9")?,
                mass: symbol(&library, "M_NE")?,
                coriolis: symbol(&library, "c_NE")?,
                gravity: symbol(&library, "g_NE")?,
            }
        };

        Ok(SoModelBackend {
            symbols,
            _library: library,
            #[cfg(unix)]
            _libm: libm,
            _temp_file: temp_file,
        })
    }

    /// `O_T_J1 .. O_T_J8`.
    fn call_pose(f: PoseFn, q: &[f64; DOF]) -> [f64; 16] {
        let mut out = [0.0; 16];
        // SAFETY: `f` has type `PoseFn`, so it was resolved as a
        // `(const double[7], double[16])` entry point of the model library; `q` and `out`
        // are live arrays of exactly those lengths. The library holds no state across calls.
        unsafe { f(q, &mut out) };
        out
    }

    /// `O_T_J9`, which takes the flange-to-frame offset.
    fn call_pose_ee(f: PoseEeFn, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 16] {
        let mut out = [0.0; 16];
        // SAFETY: `f` has type `PoseEeFn`, i.e. a
        // `(const double[7], const double[16], double[16])` entry point; `q`, `f_t_ee` and
        // `out` are live arrays of exactly those lengths.
        unsafe { f(q, f_t_ee, &mut out) };
        out
    }

    /// `Ji_J_J2 .. Ji_J_J8` and `O_J_J2 .. O_J_J8`.
    fn call_jacobian(f: JacobianFn, q: &[f64; DOF]) -> [f64; 42] {
        let mut out = [0.0; 42];
        // SAFETY: `f` has type `JacobianFn`, i.e. a `(const double[7], double[42])` entry
        // point; `q` and `out` are live arrays of exactly those lengths.
        unsafe { f(q, &mut out) };
        out
    }

    /// `Ji_J_J9` / `O_J_J9`, which take the flange-to-frame offset.
    fn call_jacobian_ee(f: JacobianEeFn, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 42] {
        let mut out = [0.0; 42];
        // SAFETY: `f` has type `JacobianEeFn`, i.e. a
        // `(const double[7], const double[16], double[42])` entry point; `q`, `f_t_ee` and
        // `out` are live arrays of exactly those lengths.
        unsafe { f(q, f_t_ee, &mut out) };
        out
    }

    /// `Ji_J_J1` / `O_J_J1`, which are constant and take no `q`.
    fn call_const_jacobian(f: ConstJacobianFn) -> [f64; 42] {
        let mut out = [0.0; 42];
        // SAFETY: `f` has type `ConstJacobianFn`, i.e. a `(double[42])` entry point; `out`
        // is a live `[f64; 42]`.
        unsafe { f(&mut out) };
        out
    }
}

/// Column-major 4x4 product `a * b`.
///
/// The exact operation `model.cpp` performs for [`crate::model::Frame::Stiffness`]:
/// `Eigen::Matrix4d(F_T_EE.data()) * Eigen::Matrix4d(EE_T_K.data())`, with Eigen's
/// default column-major storage on both sides.
fn mat4_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for column in 0..4 {
        for row in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[row + 4 * k] * b[k + 4 * column];
            }
            out[row + 4 * column] = sum;
        }
    }
    out
}

impl RobotModelBackend for SoModelBackend {
    /// `c_NE(q, dq, I_total, m_total, F_x_Ctotal)`.
    ///
    /// `gravity_earth` is **ignored**: the FCI v5 model library's Coriolis
    /// entry point takes no gravity vector and `franka::Model::coriolis`
    /// (0.9.2) has no such parameter either. See the module documentation.
    fn coriolis(
        &self,
        q: &[f64; DOF],
        dq: &[f64; DOF],
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
        _gravity_earth: &[f64; 3],
    ) -> [f64; DOF] {
        let mut out = [0.0; DOF];
        // SAFETY: `c_NE` was resolved with the signature `CoriolisFn` spells out; all five
        // inputs are live arrays of the declared lengths and `out` is a live `[f64; 7]`.
        unsafe { (self.symbols.coriolis)(q, dq, i_total, m_total, f_x_ctotal, &mut out) };
        out
    }

    /// `g_NE(q, gravity_earth, m_total, F_x_Ctotal)`.
    fn gravity(
        &self,
        q: &[f64; DOF],
        gravity_earth: &[f64; 3],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [f64; DOF] {
        let mut out = [0.0; DOF];
        // SAFETY: `g_NE` was resolved with the signature `GravityFn` spells out; all inputs
        // are live arrays of the declared lengths and `out` is a live `[f64; 7]`.
        unsafe { (self.symbols.gravity)(q, gravity_earth, m_total, f_x_ctotal, &mut out) };
        out
    }

    /// `M_NE(q, I_total, m_total, F_x_Ctotal)`.
    fn mass(
        &self,
        q: &[f64; DOF],
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [f64; 49] {
        let mut out = [0.0; 49];
        // SAFETY: `M_NE` was resolved with the signature `MassFn` spells out; all inputs are
        // live arrays of the declared lengths and `out` is a live `[f64; 49]`.
        unsafe { (self.symbols.mass)(q, i_total, m_total, f_x_ctotal, &mut out) };
        out
    }

    fn pose(&self, q: &[f64; DOF], joint_index: usize) -> [f64; 16] {
        debug_assert!((1..=DOF).contains(&joint_index));
        SoModelBackend::call_pose(self.symbols.o_t_j[joint_index.clamp(1, DOF) - 1], q)
    }

    fn pose_flange(&self, q: &[f64; DOF]) -> [f64; 16] {
        SoModelBackend::call_pose(self.symbols.o_t_j[7], q)
    }

    fn pose_ee(&self, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 16] {
        SoModelBackend::call_pose_ee(self.symbols.o_t_j9, q, f_t_ee)
    }

    fn pose_stiffness(&self, q: &[f64; DOF], f_t_ee: &[f64; 16], ee_t_k: &[f64; 16]) -> [f64; 16] {
        SoModelBackend::call_pose_ee(self.symbols.o_t_j9, q, &mat4_mul(f_t_ee, ee_t_k))
    }

    fn body_jacobian(&self, q: &[f64; DOF], joint_index: usize) -> [f64; 42] {
        debug_assert!((1..=DOF).contains(&joint_index));
        match joint_index.clamp(1, DOF) {
            1 => SoModelBackend::call_const_jacobian(self.symbols.ji_j_j1),
            joint => SoModelBackend::call_jacobian(self.symbols.ji_j[joint - 2], q),
        }
    }

    fn body_jacobian_flange(&self, q: &[f64; DOF]) -> [f64; 42] {
        SoModelBackend::call_jacobian(self.symbols.ji_j[6], q)
    }

    fn body_jacobian_ee(&self, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 42] {
        SoModelBackend::call_jacobian_ee(self.symbols.ji_j_j9, q, f_t_ee)
    }

    fn body_jacobian_stiffness(
        &self,
        q: &[f64; DOF],
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> [f64; 42] {
        SoModelBackend::call_jacobian_ee(self.symbols.ji_j_j9, q, &mat4_mul(f_t_ee, ee_t_k))
    }

    fn zero_jacobian(&self, q: &[f64; DOF], joint_index: usize) -> [f64; 42] {
        debug_assert!((1..=DOF).contains(&joint_index));
        match joint_index.clamp(1, DOF) {
            1 => SoModelBackend::call_const_jacobian(self.symbols.o_j_j1),
            joint => SoModelBackend::call_jacobian(self.symbols.o_j[joint - 2], q),
        }
    }

    fn zero_jacobian_flange(&self, q: &[f64; DOF]) -> [f64; 42] {
        SoModelBackend::call_jacobian(self.symbols.o_j[6], q)
    }

    fn zero_jacobian_ee(&self, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 42] {
        SoModelBackend::call_jacobian_ee(self.symbols.o_j_j9, q, f_t_ee)
    }

    fn zero_jacobian_stiffness(
        &self,
        q: &[f64; DOF],
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> [f64; 42] {
        SoModelBackend::call_jacobian_ee(self.symbols.o_j_j9, q, &mat4_mul(f_t_ee, ee_t_k))
    }
}

#[cfg(test)]
mod tests;
