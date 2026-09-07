//! The Cartesian-impedance pieces shared by `cartesian_impedance_active_control` and
//! `cartesian_impedance_figure_eight`.
//!
//! Both examples are the same control law with different setpoints, so the parts that would
//! otherwise be copied verbatim -- unpacking libfranka's column-major `O_T_EE`, building the
//! diagonal stiffness/damping pair, and the quaternion orientation error with its
//! hemisphere fix -- live here instead. All three are ports of the corresponding lines of
//! libfranka's `examples/cartesian_impedance_control.cpp`.

use nalgebra::{Matrix3, Matrix4, Rotation3, SMatrix, UnitQuaternion, Vector3};

/// Splits a column-major 4x4 pose (`RobotState::O_T_EE` and friends) into its translation and
/// its rotation.
pub fn decompose(pose: &[f64; 16]) -> (Vector3<f64>, Matrix3<f64>) {
    let transform = Matrix4::from_column_slice(pose);
    let translation = transform.fixed_view::<3, 1>(0, 3).into_owned();
    let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
    (translation, rotation)
}

/// The compliance parameters as the two 6x6 matrices of the C++ example: a diagonal stiffness
/// with `translational` on the first three axes and `rotational` on the last three, and the
/// damping of a critically damped spring, `2 sqrt(k)`.
pub fn compliance(translational: f64, rotational: f64) -> (SMatrix<f64, 6, 6>, SMatrix<f64, 6, 6>) {
    let mut stiffness = SMatrix::<f64, 6, 6>::zeros();
    let mut damping = SMatrix::<f64, 6, 6>::zeros();
    for axis in 0..3 {
        stiffness[(axis, axis)] = translational;
        stiffness[(axis + 3, axis + 3)] = rotational;
        damping[(axis, axis)] = 2.0 * translational.sqrt();
        damping[(axis + 3, axis + 3)] = 2.0 * rotational.sqrt();
    }
    (stiffness, damping)
}

/// The rotational half of the impedance error, `-R * vec(q_ee^-1 q_d)`, in the base frame.
///
/// `rotation` is the end effector's current orientation as a rotation matrix (the one
/// [`decompose`] returns) and `orientation_d` the setpoint. A unit quaternion and its negation
/// are the same rotation, so the current orientation is first moved onto the same hemisphere
/// as the setpoint: without that the controller takes the long way round for half of all
/// setpoints.
pub fn orientation_error(
    rotation: &Matrix3<f64>,
    orientation_d: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let mut orientation =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*rotation));
    if orientation_d.coords.dot(&orientation.coords) < 0.0 {
        orientation = UnitQuaternion::new_unchecked(-orientation.into_inner());
    }
    let error_quaternion = orientation.inverse() * orientation_d;
    -(rotation * error_quaternion.vector())
}
