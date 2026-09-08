//! Cartesian rate limiting: the `O_dP_EE_c` (twist) and `O_T_EE_c` (pose) `limitRate` overloads
//! and the `Eigen::Vector3d` helper they share.
//!
//! Split out of libfranka's `src/rate_limiting.cpp`; the constants these functions are normally
//! called with live in [the parent module](super).

use nalgebra::{Matrix3, Matrix4, Rotation3, UnitQuaternion, Vector3};

use super::{DELTA_T, FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE, NORM_EPS};
use crate::control_types::is_homogeneous_transformation;
use crate::error::{FrankaError, FrankaResult};
use crate::math_utils::{
    cmax, cmin, linear_of, orthonormalized_rotation, pose_to_array, translation_of,
};

/// Limits the rate of a Cartesian (translational or rotational) velocity vector.
///
/// Port of the anonymous-namespace `limitRate(..., const Eigen::Vector3d&, ...)` in
/// `rate_limiting.cpp`.
fn limit_rate_vector3(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: &Vector3<f64>,
    last_commanded_velocity: &Vector3<f64>,
    last_commanded_acceleration: &Vector3<f64>,
) -> Vector3<f64> {
    // Differentiate to get jerk
    let commanded_jerk = (((commanded_velocity - last_commanded_velocity) / DELTA_T)
        - last_commanded_acceleration)
        / DELTA_T;

    // Limit jerk and integrate to get desired acceleration
    let mut commanded_acceleration = *last_commanded_acceleration;
    let jerk_norm = commanded_jerk.norm();
    if jerk_norm > NORM_EPS {
        commanded_acceleration +=
            (commanded_jerk / jerk_norm) * cmax(cmin(jerk_norm, max_jerk), -max_jerk) * DELTA_T;
    }

    // Compute Euclidean distance to the max velocity vector that would be reached starting from
    // last_commanded_velocity with the direction of the desired acceleration
    let unit_commanded_acceleration = commanded_acceleration / commanded_acceleration.norm();
    let dot_product = unit_commanded_acceleration.dot(last_commanded_velocity);
    let distance_to_max_velocity = -dot_product
        + (dot_product.powi(2) - last_commanded_velocity.norm_squared() + max_velocity.powi(2))
            .sqrt();

    // Compute safe acceleration limits
    let safe_max_acceleration = cmin(
        (max_jerk / max_acceleration) * distance_to_max_velocity,
        max_acceleration,
    );

    // Limit acceleration and integrate to get desired velocities
    let mut limited_commanded_velocity = *last_commanded_velocity;
    let acceleration_norm = commanded_acceleration.norm();
    if acceleration_norm > NORM_EPS {
        limited_commanded_velocity +=
            unit_commanded_acceleration * cmin(acceleration_norm, safe_max_acceleration) * DELTA_T;
    }

    limited_commanded_velocity
}

/// Limits the rate of a desired Cartesian velocity considering the limits provided.
///
/// Port of the `std::array<double, 6>` overload of `franka::limitRate`.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if an element of `O_dP_EE_c` is infinite or NaN.
///
/// # Note
/// FCI filters must be deactivated to work properly.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_cartesian_velocity(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    O_dP_EE_c: &[f64; 6],
    last_O_dP_EE_c: &[f64; 6],
    last_O_ddP_EE_c: &[f64; 6],
) -> FrankaResult<[f64; 6]> {
    if !O_dP_EE_c.iter().all(|v| v.is_finite()) {
        return Err(FrankaError::InvalidArgument(
            "O_dP_EE_c is infinite or NaN.".to_string(),
        ));
    }
    let head = |v: &[f64; 6]| Vector3::new(v[0], v[1], v[2]);
    let tail = |v: &[f64; 6]| Vector3::new(v[3], v[4], v[5]);

    let translation = limit_rate_vector3(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        &head(O_dP_EE_c),
        &head(last_O_dP_EE_c),
        &head(last_O_ddP_EE_c),
    );
    let rotation = limit_rate_vector3(
        max_rotational_velocity,
        max_rotational_acceleration,
        max_rotational_jerk,
        &tail(O_dP_EE_c),
        &tail(last_O_dP_EE_c),
        &tail(last_O_ddP_EE_c),
    );

    Ok([
        translation[0],
        translation[1],
        translation[2],
        rotation[0],
        rotation[1],
        rotation[2],
    ])
}

/// Limits the rate of a desired Cartesian pose considering the limits provided.
///
/// Port of the `std::array<double, 16>` overload of `franka::limitRate`. `O_T_EE_c` and the
/// return value are column-major 4x4 homogeneous transformation matrices.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if an element of `O_T_EE_c` is infinite or NaN, or if
/// `O_T_EE_c` is not a homogeneous transformation matrix.
///
/// # Note
/// FCI filters must be deactivated to work properly.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_cartesian_pose(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    O_T_EE_c: &[f64; 16],
    last_O_T_EE_c: &[f64; 16],
    last_O_dP_EE_c: &[f64; 6],
    last_O_ddP_EE_c: &[f64; 6],
) -> FrankaResult<[f64; 16]> {
    if !O_T_EE_c.iter().all(|v| v.is_finite()) {
        return Err(FrankaError::InvalidArgument(
            "O_T_EE_c is infinite or NaN.".to_string(),
        ));
    }
    if !is_homogeneous_transformation(O_T_EE_c) {
        return Err(FrankaError::InvalidArgument(
            "O_T_EE_c is invalid transformation matrix. Has to be column major!".to_string(),
        ));
    }

    let commanded_pose = Matrix4::from_column_slice(O_T_EE_c);
    let last_commanded_pose = Matrix4::from_column_slice(last_O_T_EE_c);
    let commanded_translation = translation_of(&commanded_pose);
    let last_commanded_translation = translation_of(&last_commanded_pose);
    // Eigen's Affine3d::rotation() is the rotation factor of the polar decomposition, i.e. it
    // orthonormalizes the linear part.
    let commanded_rotation = orthonormalized_rotation(&linear_of(&commanded_pose));
    let last_commanded_rotation = orthonormalized_rotation(&linear_of(&last_commanded_pose));

    // Compute translational velocity
    let translational_velocity = (commanded_translation - last_commanded_translation) / DELTA_T;

    // Compute rotational velocity
    let rot_difference = commanded_rotation * last_commanded_rotation.transpose();
    let rotational_velocity = scaled_axis(&rot_difference) / DELTA_T;

    // Limit the rate of the twist
    let commanded_twist = [
        translational_velocity[0],
        translational_velocity[1],
        translational_velocity[2],
        rotational_velocity[0],
        rotational_velocity[1],
        rotational_velocity[2],
    ];
    let limited_twist = limit_rate_cartesian_velocity(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_velocity,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_acceleration,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_jerk,
        &commanded_twist,
        last_O_dP_EE_c,
        last_O_ddP_EE_c,
    )?;
    let dx_head = Vector3::new(limited_twist[0], limited_twist[1], limited_twist[2]);
    let dx_tail = Vector3::new(limited_twist[3], limited_twist[4], limited_twist[5]);

    // Integrate limited twist
    let limited_translation = last_commanded_translation + dx_head * DELTA_T;
    let mut limited_rotation = last_commanded_rotation;
    let angular_speed = dx_tail.norm();
    if angular_speed > NORM_EPS {
        let w_norm = dx_tail / angular_speed;
        let theta = DELTA_T * angular_speed;
        let omega_skew = Matrix3::new(
            0.0, -w_norm[2], w_norm[1], //
            w_norm[2], 0.0, -w_norm[0], //
            -w_norm[1], w_norm[0], 0.0,
        );
        let rotation = Matrix3::identity()
            + theta.sin() * omega_skew
            + (1.0 - theta.cos()) * (omega_skew * omega_skew);
        limited_rotation = rotation * last_commanded_rotation;
    }

    Ok(pose_to_array(&limited_rotation, &limited_translation))
}

/// `axis * angle` of a rotation matrix.
///
/// Port of `Eigen::AngleAxisd(rotation).axis() * Eigen::AngleAxisd(rotation).angle()`, including
/// Eigen's quaternion-based extraction (which yields the zero vector for the identity rotation).
pub(crate) fn scaled_axis(rotation: &Matrix3<f64>) -> Vector3<f64> {
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*rotation));
    let vec = Vector3::new(q.i, q.j, q.k);
    let mut n = vec.norm();
    if n == 0.0 {
        return Vector3::zeros();
    }
    let angle = 2.0 * n.atan2(q.w.abs());
    if q.w < 0.0 {
        n = -n;
    }
    (vec / n) * angle
}
