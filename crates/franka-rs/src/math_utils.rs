//! Small numeric helpers shared by the ports of libfranka's Eigen-based code.
//!
//! These live in one place because they are subtle: [`cmin`]/[`cmax`] reproduce `std::min` /
//! `std::max` (which differ from `f64::min`/`f64::max` for NaN operands) and
//! [`orthonormalized_rotation`] reproduces `Eigen::Transform<Affine>::rotation()`, which is a
//! polar decomposition rather than a plain read of the linear block. Two copies of either could
//! silently drift apart.

use nalgebra::{Matrix3, Matrix4, Vector3};

/// `std::min` semantics (`b < a ? b : a`), which differ from `f64::min` for NaN operands.
#[inline]
pub(crate) fn cmin(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max` semantics (`a < b ? b : a`), which differ from `f64::max` for NaN operands.
#[inline]
pub(crate) fn cmax(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// The linear (upper-left 3x3) block of a column-major homogeneous transformation.
///
/// Port of `Eigen::Transform::linear()`.
pub(crate) fn linear_of(pose: &Matrix4<f64>) -> Matrix3<f64> {
    pose.fixed_view::<3, 3>(0, 0).into_owned()
}

/// The translation (upper-right 3x1) block of a column-major homogeneous transformation.
///
/// Port of `Eigen::Transform::translation()`.
pub(crate) fn translation_of(pose: &Matrix4<f64>) -> Vector3<f64> {
    Vector3::new(pose[(0, 3)], pose[(1, 3)], pose[(2, 3)])
}

/// Rotation factor of the polar decomposition of `m`.
///
/// Port of Eigen's `Transform<Affine>::rotation()` / `computeRotationScaling()`: `U * V^T` from
/// the SVD, with the last column of `U` negated when that product would have a negative
/// determinant. Unlike `linear()`, this orthonormalizes the matrix, which is what lets
/// `cartesian_low_pass_filter` smooth barely-orthonormal input poses.
pub(crate) fn orthonormalized_rotation(m: &Matrix3<f64>) -> Matrix3<f64> {
    let svd = m.svd(true, true);
    let (Some(mut u), Some(v_t)) = (svd.u, svd.v_t) else {
        return *m;
    };
    if (u * v_t).determinant() < 0.0 {
        for row in 0..3 {
            u[(row, 2)] = -u[(row, 2)];
        }
    }
    u * v_t
}

/// Assembles a column-major 4x4 homogeneous transformation matrix.
pub(crate) fn pose_to_array(rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> [f64; 16] {
    let mut out = [0.0; 16];
    for col in 0..3 {
        for row in 0..3 {
            out[col * 4 + row] = rotation[(row, col)];
        }
    }
    out[12] = translation[0];
    out[13] = translation[1];
    out[14] = translation[2];
    out[15] = 1.0;
    out
}

/// Twist between two column-major poses.
///
/// Port of `differentiateOneSample(std::array<double, 16>, std::array<double, 16>, double)` from
/// libfranka's `test/helpers.cpp`; used by the ported rate-limiting and low-pass filter tests.
#[cfg(test)]
pub(crate) fn differentiate_one_sample_pose(
    value: &[f64; 16],
    last_value: &[f64; 16],
    delta_t: f64,
) -> [f64; 6] {
    let pose = Matrix4::from_column_slice(value);
    let last_pose = Matrix4::from_column_slice(last_value);
    let translational = (translation_of(&pose) - translation_of(&last_pose)) / delta_t;
    let delta_rotation = (linear_of(&pose) - linear_of(&last_pose)) / delta_t;
    let rotational_twist = delta_rotation * linear_of(&last_pose).transpose();
    [
        translational[0],
        translational[1],
        translational[2],
        rotational_twist[(2, 1)],
        rotational_twist[(0, 2)],
        rotational_twist[(1, 0)],
    ]
}
