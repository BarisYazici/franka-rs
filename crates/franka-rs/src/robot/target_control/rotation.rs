//! The rotation arithmetic of the Cartesian interface: column-major poses, unit quaternions
//! in `[x, y, z, w]` order and rotation vectors (unit axis times angle), on nalgebra.

use nalgebra::{Matrix3, Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3};

use crate::error::{FrankaError, FrankaResult};
use crate::math_utils::{linear_of, orthonormalized_rotation, pose_to_array};
use crate::rate_limiting::scaled_axis;

/// How far from orthonormal (the largest entry of `|R^T R - I|`, or `|det R - 1|`) the
/// rotation block of a target pose may be: within this it is re-orthonormalised, beyond it
/// refused.
pub const ORTHONORMAL_TOLERANCE: f64 = 1e-3;

/// How far from unit length a target quaternion may be before it is refused; within this it
/// is normalised.
pub const UNIT_QUATERNION_TOLERANCE: f64 = 1e-3;

/// The rotation block of a column-major 4x4 pose, as it is (not orthonormalised).
pub(super) fn rotation_of(pose: &[f64; 16]) -> Matrix3<f64> {
    linear_of(&Matrix4::from_column_slice(pose))
}

/// The translation of a column-major 4x4 pose.
pub(super) fn translation_of(pose: &[f64; 16]) -> [f64; 3] {
    [pose[12], pose[13], pose[14]]
}

/// A column-major 4x4 pose from a rotation and a translation.
pub(super) fn pose_from(rotation: &Matrix3<f64>, translation: &[f64; 3]) -> [f64; 16] {
    pose_to_array(rotation, &Vector3::from(*translation))
}

/// How far `m` is from a rotation: the largest entry of `|m^T m - I|`, or `|det m - 1|` if
/// that is larger (a reflection is orthonormal but no rotation).
pub(super) fn orthonormality_error(m: &Matrix3<f64>) -> f64 {
    let gram = m.transpose() * m - Matrix3::identity();
    let worst = gram.iter().fold(0.0f64, |worst, x| worst.max(x.abs()));
    worst.max((m.determinant() - 1.0).abs())
}

/// The unit quaternion of a rotation matrix, `[x, y, z, w]` with `w >= 0`.
pub(super) fn to_quaternion(rotation: &Matrix3<f64>) -> [f64; 4] {
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*rotation));
    let sign = if q.w < 0.0 { -1.0 } else { 1.0 };
    [sign * q.i, sign * q.j, sign * q.k, sign * q.w]
}

/// The rotation matrix of a quaternion `[x, y, z, w]`, normalised on the way.
pub(super) fn from_quaternion(q: &[f64; 4]) -> Matrix3<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2]))
        .to_rotation_matrix()
        .into_inner()
}

/// The rotation vector of a rotation matrix (quaternion-based: accurate for small angles).
pub(super) fn log(rotation: &Matrix3<f64>) -> [f64; 3] {
    let v = scaled_axis(rotation);
    [v[0], v[1], v[2]]
}

/// The rotation matrix of a rotation vector (Rodrigues); the identity for the zero vector.
pub(super) fn exp(v: &[f64; 3]) -> Matrix3<f64> {
    Rotation3::from_scaled_axis(Vector3::from(*v)).into_inner()
}

/// The angle, rad, that takes `a` to `b`.
pub(super) fn angle_between(a: &Matrix3<f64>, b: &Matrix3<f64>) -> f64 {
    norm(&log(&(b * a.transpose())))
}

pub(super) fn norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub(super) fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    norm(&[a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

/// `q` as a unit quaternion.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if a component is not finite or the norm is further than
/// [`UNIT_QUATERNION_TOLERANCE`] from one.
pub(super) fn unit_quaternion(q: [f64; 4]) -> FrankaResult<[f64; 4]> {
    if q.iter().any(|x| !x.is_finite()) {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: the orientation must be finite, got {q:?}"
        )));
    }
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if (norm - 1.0).abs() > UNIT_QUATERNION_TOLERANCE {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: the orientation {q:?} is not a unit quaternion (norm {norm}); \
             the order is [x, y, z, w]"
        )));
    }
    Ok(q.map(|x| x / norm))
}

/// The translation and the re-orthonormalised rotation of a column-major pose.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if an entry is not finite, the last row is not
/// `[0, 0, 0, 1]` or the rotation block is further from orthonormal than the tolerance.
pub(super) fn checked_pose(pose: &[f64; 16]) -> FrankaResult<([f64; 3], Matrix3<f64>)> {
    if pose.iter().any(|x| !x.is_finite()) {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: the pose must be finite, got {pose:?}"
        )));
    }
    let last_row = [pose[3], pose[7], pose[11], pose[15]];
    if last_row
        .iter()
        .zip(&[0.0, 0.0, 0.0, 1.0])
        .any(|(a, b)| (a - b).abs() > 1e-6)
    {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: the pose must be a column-major homogeneous transform with the \
             last row [0, 0, 0, 1], got {last_row:?}"
        )));
    }
    let rotation = rotation_of(pose);
    let error = orthonormality_error(&rotation);
    if error > ORTHONORMAL_TOLERANCE {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: the rotation block of the pose is not orthonormal (error {error:.3e} \
             against a tolerance of {ORTHONORMAL_TOLERANCE:.0e}); a rotation matrix's columns \
             must be unit length and mutually perpendicular, and the pose column-major"
        )));
    }
    Ok((translation_of(pose), orthonormalized_rotation(&rotation)))
}
