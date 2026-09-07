//! The C entry points of `libfcimodels` and the load-time symbol resolution.

use libloading::{Library, Symbol};

use crate::error::{FrankaError, FrankaResult};

// -- C signatures from `src/libfcimodels.h` (libfranka 0.9.2) --------------------------------
//
// Every array parameter is spelled as a pointer to a *sized* array rather than a bare
// `*const f64` / `*mut f64`. The ABI is identical — C decays `double q[7]` to `double*`
// anyway — but it puts the length into the function-pointer type, so a 16-double pose
// entry point and a 42-double Jacobian entry point are no longer the same Rust type and
// cannot be swapped by an index mix-up without a compile error.

/// `void O_T_J1..O_T_J8(const double q[7], double out[16])`.
pub(super) type PoseFn = unsafe extern "C" fn(*const [f64; 7], *mut [f64; 16]);

/// `void O_T_J9(const double q[7], const double F_T_EE[16], double out[16])`.
pub(super) type PoseEeFn = unsafe extern "C" fn(*const [f64; 7], *const [f64; 16], *mut [f64; 16]);

/// `void Ji_J_J2..Ji_J_J8(const double q[7], double out[42])`, and the same for
/// `O_J_J2..O_J_J8`.
pub(super) type JacobianFn = unsafe extern "C" fn(*const [f64; 7], *mut [f64; 42]);

/// `void Ji_J_J9(const double q[7], const double F_T_EE[16], double out[42])`, and the
/// same for `O_J_J9`.
pub(super) type JacobianEeFn =
    unsafe extern "C" fn(*const [f64; 7], *const [f64; 16], *mut [f64; 42]);

/// `void Ji_J_J1(double out[42])` / `void O_J_J1(double out[42])` — constant, no `q`.
pub(super) type ConstJacobianFn = unsafe extern "C" fn(*mut [f64; 42]);

/// `void M_NE(const double q[7], const double I_load[9], double m_load,
///            const double F_x_Cload[3], double out[49])`
pub(super) type MassFn =
    unsafe extern "C" fn(*const [f64; 7], *const [f64; 9], f64, *const [f64; 3], *mut [f64; 49]);

/// `void c_NE(const double q[7], const double dq[7], const double I_load[9],
///            double m_load, const double F_x_Cload[3], double out[7])`
pub(super) type CoriolisFn = unsafe extern "C" fn(
    *const [f64; 7],
    *const [f64; 7],
    *const [f64; 9],
    f64,
    *const [f64; 3],
    *mut [f64; 7],
);

/// `void g_NE(const double q[7], const double g_earth[3], double m_load,
///            const double F_x_Cload[3], double out[7])`
pub(super) type GravityFn =
    unsafe extern "C" fn(*const [f64; 7], *const [f64; 3], f64, *const [f64; 3], *mut [f64; 7]);

/// The thirty resolved entry points, bound once at construction.
///
/// Port of the `std::function` members of `franka::ModelLibrary`
/// (`src/model_library.h`), which are likewise all resolved in the constructor
/// so that a missing symbol is an error at load time rather than at the first
/// control cycle.
pub(super) struct Symbols {
    /// `O_T_J1 .. O_T_J8`, indexed by `joint - 1`.
    pub(super) o_t_j: [PoseFn; 8],
    /// `O_T_J9`.
    pub(super) o_t_j9: PoseEeFn,
    /// `Ji_J_J1`.
    pub(super) ji_j_j1: ConstJacobianFn,
    /// `Ji_J_J2 .. Ji_J_J8`, indexed by `joint - 2`.
    pub(super) ji_j: [JacobianFn; 7],
    /// `Ji_J_J9`.
    pub(super) ji_j_j9: JacobianEeFn,
    /// `O_J_J1`.
    pub(super) o_j_j1: ConstJacobianFn,
    /// `O_J_J2 .. O_J_J8`, indexed by `joint - 2`.
    pub(super) o_j: [JacobianFn; 7],
    /// `O_J_J9`.
    pub(super) o_j_j9: JacobianEeFn,
    /// `M_NE`.
    pub(super) mass: MassFn,
    /// `c_NE`.
    pub(super) coriolis: CoriolisFn,
    /// `g_NE`.
    pub(super) gravity: GravityFn,
}

/// Reads a `Symbol` out of `library` and copies the function pointer out of it.
///
/// # Safety
///
/// The caller asserts that the symbol named `name` in `library` really has type
/// `T`, and that the returned pointer is only called while `library` is alive.
pub(super) unsafe fn symbol<T: Copy>(library: &Library, name: &str) -> FrankaResult<T> {
    let mut nul_terminated = Vec::with_capacity(name.len() + 1);
    nul_terminated.extend_from_slice(name.as_bytes());
    nul_terminated.push(0);
    // SAFETY: delegated to this function's own contract; `nul_terminated` is a valid
    // NUL-terminated symbol name.
    let resolved: Symbol<T> = unsafe { library.get(&nul_terminated) }.map_err(|e| {
        FrankaError::Model(format!("libfranka: Symbol cannot be found: {name}: {e}"))
    })?;
    Ok(*resolved)
}
