//! Least-squares identification of the Franka Emika Robot (FER)'s link inertial parameters from
//! the robot's own `libfcimodels_x64.so`.
//!
//! Rigid-body dynamics is **linear** in the ten inertial parameters of each
//! link, so with the kinematics fixed (and already known to agree with the
//! shared object to 4e-16) the mass matrix and the gravity torques are an
//! affine function of the 70 unknowns:
//!
//! ```text
//! y(p) = y(p0) + A (p - p0)
//! ```
//!
//! `A` is obtained a column at a time by evaluating the *native backend itself*
//! at `p0 + e_k` — no second implementation of CRBA or RNEA is written here,
//! and the map is exactly linear, so the differences are the exact columns up to
//! round-off. The system is solved with a truncated SVD, which gives the
//! minimum-norm correction to `p0`: the unidentifiable directions (link 1's
//! mass, for instance) simply keep their published values.

use franka::model::Model;
use nalgebra::{DMatrix, DVector};

use crate::fer::{self, Params, DOF};

/// The three gravity directions the fit observes, so the identified parameters
/// reproduce `gravity_q` for any `gravity_earth`, not only a vertical one.
const GRAVITY_BASIS: [[f64; 3]; 3] = [[9.81, 0.0, 0.0], [0.0, 9.81, 0.0], [0.0, 0.0, -9.81]];

/// Rows contributed by one joint configuration: the 28 unique entries of the
/// symmetric mass matrix plus 7 gravity torques per gravity direction.
const ROWS_PER_SAMPLE: usize = 28 + 7 * GRAVITY_BASIS.len();

/// The observation vector: what the identification matches, with no load.
fn observation(model: &Model, qs: &[[f64; 7]]) -> DVector<f64> {
    let mut out = DVector::zeros(qs.len() * ROWS_PER_SAMPLE);
    let mut row = 0;
    for q in qs {
        let mass = model.mass_q(q, &[0.0; 9], 0.0, &[0.0; 3]);
        for i in 0..7 {
            for j in i..7 {
                out[row] = mass[i + 7 * j];
                row += 1;
            }
        }
        for g in GRAVITY_BASIS {
            for value in model.gravity_q(q, 0.0, &[0.0; 3], &g) {
                out[row] = value;
                row += 1;
            }
        }
    }
    out
}

/// Builds the native model carrying `params`.
fn native(params: &Params) -> Model {
    Model::from_urdf(&fer::urdf(params, "")).expect("the generated URDF loads")
}

/// The outcome of one identification.
pub struct Identified {
    pub params: Params,
    /// Largest residual of the fitted model on the observation vector.
    pub residual: f64,
    /// Largest residual of the starting point, for comparison.
    pub prior_residual: f64,
    /// Numerical rank of the regressor at the truncation used.
    pub rank: usize,
    /// Largest and smallest retained singular value of the scaled regressor.
    pub singular_values: (f64, f64),
    /// Smallest eigenvalue of any link's pseudo-inertia matrix; positive would
    /// mean every link is a physically realisable rigid body.
    pub min_pseudo_inertia_eigenvalue: f64,
    /// The best that quantity can be made anywhere in the affine set of exact
    /// fits, by alternating projections. Still negative means *no* exact fit is
    /// physically consistent, i.e. the shared object's own parameters are not.
    pub best_pseudo_inertia_eigenvalue: f64,
}

/// Identifies the seven links' inertial parameters against `so`.
pub fn identify(so: &Model, prior: &Params, qs: &[[f64; 7]]) -> Identified {
    let target = observation(so, qs);
    let base = observation(&native(prior), qs);
    let rows = target.len();
    let columns = DOF * 10;

    let mut a = DMatrix::zeros(rows, columns);
    for k in 0..columns {
        let mut perturbed = *prior;
        perturbed[k / 10][k % 10] += 1.0;
        let column = observation(&native(&perturbed), qs) - &base;
        a.set_column(k, &column);
    }

    // Column scaling: the parameters differ by four orders of magnitude
    // (kilograms against kilogram metres squared), and the minimum-norm
    // criterion should not be dominated by the largest-scale ones.
    let mut scale = vec![1.0_f64; columns];
    for (k, entry) in scale.iter_mut().enumerate() {
        let norm = a.column(k).norm();
        if norm > 0.0 {
            *entry = norm;
            let inverse = 1.0 / norm;
            let mut column = a.column_mut(k);
            column *= inverse;
        }
    }

    let residual_target = &target - &base;
    let svd = nalgebra::SVD::new(a.clone(), true, true);
    let largest = svd.singular_values.max();
    let cutoff = 1e-12;
    let rank = svd
        .singular_values
        .iter()
        .filter(|s| **s > cutoff * largest)
        .count();
    let smallest_kept = svd
        .singular_values
        .iter()
        .copied()
        .filter(|s| *s > cutoff * largest)
        .fold(f64::INFINITY, f64::min);
    let scaled_delta = svd
        .solve(&residual_target, cutoff * largest)
        .expect("the regressor has a least-squares solution");

    let mut params = *prior;
    for k in 0..columns {
        params[k / 10][k % 10] += scaled_delta[k] / scale[k];
    }
    let params = params;

    // The regressor is rank deficient — 45 of 70 directions, the classical base
    // parameters — so the solution above is one of infinitely many exact fits,
    // and it is the one closest to `prior`. Report how physical the whole family
    // can be made; see `nearest_physically_consistent`.
    let null = null_space(&svd, columns, rank);
    let best = nearest_physically_consistent(&params, &null, &scale);

    let fitted = observation(&native(&params), qs);
    let residual = (&fitted - &target).amax();
    let prior_residual = (&base - &target).amax();

    Identified {
        params,
        residual,
        prior_residual,
        rank,
        singular_values: (largest, smallest_kept),
        min_pseudo_inertia_eigenvalue: min_pseudo_inertia_eigenvalue(&params),
        best_pseudo_inertia_eigenvalue: best,
    }
}

/// The regressor's null space, in the *scaled* coordinates the SVD was taken
/// in, where the basis is orthonormal.
///
/// A step along any of these vectors leaves every mass matrix and every gravity
/// torque unchanged: they are the directions the identification cannot see.
fn null_space(
    svd: &nalgebra::SVD<f64, nalgebra::Dyn, nalgebra::Dyn>,
    columns: usize,
    rank: usize,
) -> Vec<Vec<f64>> {
    let v_t = svd.v_t.as_ref().expect("right singular vectors");
    let mut out = Vec::new();
    for row in rank..v_t.nrows() {
        out.push((0..columns).map(|k| v_t[(row, k)]).collect());
    }
    out
}

/// The pseudo-inertia matrix of one link (Wensing, Kim and Slotine, 2018).
///
/// `J = [[tr(I)/2 * E - I, h], [h^T, m]]` with `I` about the joint frame's
/// origin, which is exactly how [`Params`] stores it. `J` is positive
/// semi-definite if and only if the ten parameters are those of a real rigid
/// body with non-negative mass density.
fn pseudo_inertia(p: &[f64; 10]) -> nalgebra::Matrix4<f64> {
    let (m, hx, hy, hz) = (p[0], p[1], p[2], p[3]);
    let (ixx, ixy, ixz, iyy, iyz, izz) = (p[4], p[5], p[6], p[7], p[8], p[9]);
    let half_trace = 0.5 * (ixx + iyy + izz);
    nalgebra::Matrix4::new(
        half_trace - ixx,
        -ixy,
        -ixz,
        hx,
        -ixy,
        half_trace - iyy,
        -iyz,
        hy,
        -ixz,
        -iyz,
        half_trace - izz,
        hz,
        hx,
        hy,
        hz,
        m,
    )
}

/// Smallest eigenvalue of any link's pseudo-inertia matrix.
fn min_pseudo_inertia_eigenvalue(params: &Params) -> f64 {
    params
        .iter()
        .map(|p| {
            nalgebra::SymmetricEigen::new(pseudo_inertia(p))
                .eigenvalues
                .min()
        })
        .fold(f64::INFINITY, f64::min)
}

/// Searches the regressor's null space for the exact fit whose links come
/// closest to being real rigid bodies, and returns the smallest pseudo-inertia
/// eigenvalue it could reach.
///
/// Alternating projections between two convex sets: the affine set of exact
/// fits (`start` plus the regressor's null space) and the cone of physically
/// realisable ten-parameter sets (`J_i >= 0`). Both projections are exact, so
/// this converges to a point in the intersection whenever there is one — and
/// there is one only if the shared object's own parameters are those of a real
/// rigid body. Predictions never move: the affine projection restores them
/// after every eigenvalue clip.
fn nearest_physically_consistent(start: &Params, null: &[Vec<f64>], scale: &[f64]) -> f64 {
    /// How far inside the cone to aim, in the units of the pseudo-inertia's
    /// eigenvalues (kg and kg m^2).
    const MARGIN: f64 = 1e-6;
    const ITERATIONS: usize = 5_000;

    let columns = DOF * 10;
    // Work in the scaled coordinates the null-space basis is orthonormal in.
    let flatten =
        |p: &Params| -> Vec<f64> { (0..columns).map(|k| p[k / 10][k % 10] * scale[k]).collect() };
    let unflatten = |x: &[f64]| -> Params {
        let mut p = [[0.0; 10]; DOF];
        for k in 0..columns {
            p[k / 10][k % 10] = x[k] / scale[k];
        }
        p
    };

    let anchor = flatten(start);
    let mut x = anchor.clone();
    let mut best_eigenvalue = min_pseudo_inertia_eigenvalue(start);

    for _ in 0..ITERATIONS {
        if best_eigenvalue >= 0.0 {
            break;
        }
        // Project each link onto the positive semi-definite cone.
        let mut params = unflatten(&x);
        for p in params.iter_mut() {
            let eigen = nalgebra::SymmetricEigen::new(pseudo_inertia(p));
            if eigen.eigenvalues.min() >= MARGIN {
                continue;
            }
            let mut clipped = nalgebra::Vector4::zeros();
            for (i, lambda) in eigen.eigenvalues.iter().enumerate() {
                clipped[i] = lambda.max(MARGIN);
            }
            let j = eigen.eigenvectors
                * nalgebra::Matrix4::from_diagonal(&clipped)
                * eigen.eigenvectors.transpose();
            *p = params_from_pseudo_inertia(&j);
        }
        // Project back onto the affine set of exact fits.
        let candidate = flatten(&params);
        let mut next = anchor.clone();
        for vector in null {
            let dot: f64 = (0..columns)
                .map(|k| (candidate[k] - anchor[k]) * vector[k])
                .sum();
            for k in 0..columns {
                next[k] += dot * vector[k];
            }
        }
        x = next;

        let eigenvalue = min_pseudo_inertia_eigenvalue(&unflatten(&x));
        best_eigenvalue = best_eigenvalue.max(eigenvalue);
    }
    best_eigenvalue
}

/// The inverse of [`pseudo_inertia`]: `I = tr(M3) * E - M3`, `h` and `m` read
/// off directly.
fn params_from_pseudo_inertia(j: &nalgebra::Matrix4<f64>) -> [f64; 10] {
    let m3 = j.fixed_view::<3, 3>(0, 0);
    let trace = m3[(0, 0)] + m3[(1, 1)] + m3[(2, 2)];
    [
        j[(3, 3)],
        j[(0, 3)],
        j[(1, 3)],
        j[(2, 3)],
        trace - m3[(0, 0)],
        -m3[(0, 1)],
        -m3[(0, 2)],
        trace - m3[(1, 1)],
        -m3[(1, 2)],
        trace - m3[(2, 2)],
    ]
}
