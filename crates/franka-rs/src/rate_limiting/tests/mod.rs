//! Port of libfranka's `test/rate_limiting_tests.cpp`.
//!
//! The shared helpers (`integrate_one_sample`, `violates_rate_limits_*`, the value generators)
//! live here; the per-quantity `TEST(RateLimiting, ...)` cases are in the [`torque`], [`joint`]
//! and [`cartesian`] submodules, which reach the helpers through `use super::*`.
#![allow(clippy::too_many_arguments)]

mod cartesian;
mod joint;
mod torque;

use nalgebra::{Matrix3, Matrix4, Vector3};

use super::*;
use crate::error::FrankaError;
use crate::math_utils::{
    cmax, cmin, differentiate_one_sample_pose, linear_of, orthonormalized_rotation, pose_to_array,
    translation_of,
};

const NO_LIMIT: f64 = f64::MAX;
const NO_LOWER_LIMIT: f64 = f64::MIN;
const JOINTS_NO_LIMIT: [f64; 7] = [NO_LIMIT; 7];
const JOINTS_NO_LOWER_LIMIT: [f64; 7] = [NO_LOWER_LIMIT; 7];
const IDENTITY_POSE: [f64; 16] = [
    1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
];

// ---- helpers ported from test/rate_limiting_tests.cpp and test/helpers.cpp ----

fn integrate_one_sample<const N: usize>(
    last_value: &[f64; N],
    derivative: &[f64; N],
    delta_t: f64,
) -> [f64; N] {
    let mut result = [0.0; N];
    for i in 0..N {
        result[i] = last_value[i] + derivative[i] * delta_t;
    }
    result
}

fn differentiate_one_sample<const N: usize>(
    value: &[f64; N],
    last_value: &[f64; N],
    delta_t: f64,
) -> [f64; N] {
    let mut result = [0.0; N];
    for i in 0..N {
        result[i] = (value[i] - last_value[i]) / delta_t;
    }
    result
}

/// Port of `integrateOneSample(std::array<double, 16>, std::array<double, 6>, double)`.
fn integrate_one_sample_pose(last_pose: &[f64; 16], twist: &[f64; 6], delta_t: f64) -> [f64; 16] {
    let pose = Matrix4::from_column_slice(last_pose);
    let rotation = orthonormalized_rotation(&linear_of(&pose));
    let omega_skew = Matrix3::new(
        0.0, -twist[5], twist[4], //
        twist[5], 0.0, -twist[3], //
        -twist[4], twist[3], 0.0,
    );
    let linear = rotation + omega_skew * rotation * delta_t;
    let translation = translation_of(&pose) + Vector3::new(twist[0], twist[1], twist[2]) * delta_t;
    pose_to_array(&linear, &translation)
}

fn violates_limit(desired_value: f64, max_value: f64) -> bool {
    desired_value.abs() > max_value
}

fn violates_limits(values: &[f64; 7], max_values: &[f64; 7]) -> bool {
    (0..7).any(|i| violates_limit(values[i], max_values[i]))
}

fn violates_rate_limits_1(
    max_derivatives: &[f64; 7],
    values: &[f64; 7],
    last_desired_values: &[f64; 7],
    delta_t: f64,
) -> bool {
    violates_limits(
        &differentiate_one_sample(values, last_desired_values, delta_t),
        max_derivatives,
    )
}

fn violates_rate_limits_3(
    max_values: &[f64; 7],
    max_derivatives: &[f64; 7],
    max_dderivatives: &[f64; 7],
    values: &[f64; 7],
    last_values: &[f64; 7],
    last_dvalues: &[f64; 7],
    delta_t: f64,
) -> bool {
    let desired_derivatives = differentiate_one_sample(values, last_values, delta_t);
    violates_limits(values, max_values)
        || violates_rate_limits_1(max_derivatives, values, last_values, delta_t)
        || violates_rate_limits_1(
            max_dderivatives,
            &desired_derivatives,
            last_dvalues,
            delta_t,
        )
}

fn violates_rate_limits_cartesian(
    max_translational_dx: f64,
    max_translational_ddx: f64,
    max_translational_dddx: f64,
    max_rotational_dx: f64,
    max_rotational_ddx: f64,
    max_rotational_dddx: f64,
    cmd_dx: &[f64; 6],
    O_dP_EE_c: &[f64; 6],
    O_ddP_EE_c: &[f64; 6],
    delta_t: f64,
) -> bool {
    let mut ddx = [0.0; 6];
    let mut dddx = [0.0; 6];
    for i in 0..6 {
        ddx[i] = (cmd_dx[i] - O_dP_EE_c[i]) / delta_t;
        dddx[i] = (ddx[i] - O_ddP_EE_c[i]) / delta_t;
    }
    let head = |v: &[f64; 6]| Vector3::new(v[0], v[1], v[2]).norm();
    let tail = |v: &[f64; 6]| Vector3::new(v[3], v[4], v[5]).norm();
    violates_limit(head(cmd_dx), max_translational_dx)
        || violates_limit(head(&ddx), max_translational_ddx)
        || violates_limit(head(&dddx), max_translational_dddx)
        || violates_limit(tail(cmd_dx), max_rotational_dx)
        || violates_limit(tail(&ddx), max_rotational_ddx)
        || violates_limit(tail(&dddx), max_rotational_dddx)
}

fn generate_values_into_limits(
    last_cmd_values: &[f64; 7],
    max_derivatives: &[f64; 7],
    eps: f64,
    delta_t: f64,
) -> [f64; 7] {
    let mut cmd_value = [0.0; 7];
    for i in 0..7 {
        // Make sure that the integration yields a value into limits
        cmd_value[i] = last_cmd_values[i]
            + (max_derivatives[i] - cmin(cmax(eps.abs(), 0.0), 2.0 * max_derivatives[i])) * delta_t;
    }
    cmd_value
}

fn generate_values_outside_limits(
    last_cmd_values: &[f64; 7],
    max_derivatives: &[f64; 7],
    eps: f64,
    delta_t: f64,
) -> [f64; 7] {
    let mut cmd_value = [0.0; 7];
    for i in 0..7 {
        // Make sure that diff yields a value outside limits
        cmd_value[i] =
            last_cmd_values[i] + (max_derivatives[i] + cmax(eps.abs(), LIMIT_EPS)) * delta_t;
    }
    cmd_value
}

fn generate_cartesian_values_into_limits(
    last_cmd_values: &[f64; 6],
    max_translational_derivative: f64,
    max_rotational_derivative: f64,
    eps: f64,
    delta_t: f64,
) -> [f64; 6] {
    // unit_vector is (1, 0, 0), so only the x components change.
    let mut result = *last_cmd_values;
    result[0] += (max_translational_derivative
        - cmin(cmax(eps.abs(), 0.0), 2.0 * max_translational_derivative))
        * delta_t;
    result[3] += (max_rotational_derivative
        - cmin(cmax(eps.abs(), 0.0), 2.0 * max_rotational_derivative))
        * delta_t;
    result
}

fn generate_cartesian_values_outside_limits(
    last_cmd_values: &[f64; 6],
    max_translational_derivative: f64,
    max_rotational_derivative: f64,
    eps: f64,
    delta_t: f64,
) -> [f64; 6] {
    let mut result = *last_cmd_values;
    result[0] += (max_translational_derivative + cmax(eps.abs(), LIMIT_EPS)) * delta_t;
    result[3] += (max_rotational_derivative + cmax(eps.abs(), LIMIT_EPS)) * delta_t;
    result
}

// ---- TEST(RateLimiting, MaxDerivative) ----
#[test]
fn non_finite_inputs_are_rejected() {
    let message = |e: FrankaError| match e {
        FrankaError::InvalidArgument(msg) => msg,
        other => panic!("expected InvalidArgument, got {other:?}"),
    };
    let mut bad7 = [0.0; 7];
    bad7[2] = f64::NAN;
    assert_eq!(
        message(limit_rate_torques(&MAX_TORQUE_RATE, &bad7, &[0.0; 7]).unwrap_err()),
        "Commanding value is infinite or NaN."
    );
    assert_eq!(
        message(
            limit_rate_joint_velocities(
                &JOINTS_NO_LIMIT,
                &JOINTS_NO_LOWER_LIMIT,
                &MAX_JOINT_ACCELERATION,
                &MAX_JOINT_JERK,
                &bad7,
                &[0.0; 7],
                &[0.0; 7]
            )
            .unwrap_err()
        ),
        "commanded_velocities is infinite or NaN."
    );
    bad7[2] = f64::INFINITY;
    assert_eq!(
        message(
            limit_rate_joint_positions(
                &JOINTS_NO_LIMIT,
                &JOINTS_NO_LOWER_LIMIT,
                &MAX_JOINT_ACCELERATION,
                &MAX_JOINT_JERK,
                &bad7,
                &[0.0; 7],
                &[0.0; 7],
                &[0.0; 7]
            )
            .unwrap_err()
        ),
        "commanded_positions is infinite or NaN."
    );
    let mut bad6 = [0.0; 6];
    bad6[5] = f64::NAN;
    assert_eq!(
        message(
            limit_rate_cartesian_velocity(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &bad6,
                &[0.0; 6],
                &[0.0; 6]
            )
            .unwrap_err()
        ),
        "O_dP_EE_c is infinite or NaN."
    );
    let mut bad_pose = IDENTITY_POSE;
    bad_pose[0] = f64::NAN;
    assert_eq!(
        message(
            limit_rate_cartesian_pose(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &bad_pose,
                &IDENTITY_POSE,
                &[0.0; 6],
                &[0.0; 6]
            )
            .unwrap_err()
        ),
        "O_T_EE_c is infinite or NaN."
    );
    let mut bad_pose = IDENTITY_POSE;
    bad_pose[0] = 2.0;
    assert_eq!(
        message(
            limit_rate_cartesian_pose(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &bad_pose,
                &IDENTITY_POSE,
                &[0.0; 6],
                &[0.0; 6]
            )
            .unwrap_err()
        ),
        "O_T_EE_c is invalid transformation matrix. Has to be column major!"
    );
}

#[test]
fn constants_match_libfranka() {
    let close = |a: f64, b: f64| (a - b).abs() < 1e-9;
    assert_eq!(DELTA_T, 1e-3);
    assert_eq!(LIMIT_EPS, 1e-3);
    assert_eq!(NORM_EPS, f64::EPSILON);
    assert_eq!(TOL_NUMBER_PACKETS_LOST, 0.0);
    assert_eq!(FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE, 0.99);
    assert!(MAX_TORQUE_RATE.iter().all(|v| close(*v, 999.999)));
    assert!(MAX_JOINT_JERK.iter().all(|v| close(*v, 4999.999)));
    assert!(MAX_JOINT_ACCELERATION.iter().all(|v| close(*v, 9.999)));
    assert!(JOINT_VELOCITY_LIMITS_TOLERANCE
        .iter()
        .all(|v| close(*v, 1e-3)));
    assert!(close(MAX_TRANSLATIONAL_JERK, 4499.999));
    assert!(close(MAX_TRANSLATIONAL_ACCELERATION, 8.999));
    assert!(close(MAX_TRANSLATIONAL_VELOCITY, 2.999));
    assert!(close(MAX_ROTATIONAL_JERK, 8499.999));
    assert!(close(MAX_ROTATIONAL_ACCELERATION, 16.999));
    assert!(close(MAX_ROTATIONAL_VELOCITY, 2.499));
    assert!(close(MAX_ELBOW_JERK, 4999.999));
    assert!(close(MAX_ELBOW_ACCELERATION, 9.999));
    assert!(close(MAX_ELBOW_VELOCITY, 1.499));
    assert!(close(MIN_ELBOW_VELOCITY, -1.499));
}

/// Pins the FER constants against the literal expressions of libfranka 0.9.2
/// `include/franka/rate_limiting.h`.
#[test]
fn fer_constants_match_the_0_9_2_header() {
    use super::fer;

    let close = |a: f64, b: f64| (a - b).abs() < 1e-9;

    assert_eq!(fer::TOL_NUMBER_PACKETS_LOST, 3.0);
    assert_eq!(fer::DELTA_T, 1e-3);
    assert_eq!(fer::LIMIT_EPS, 1e-3);

    // rate_limiting.h:57-64, spelled out exactly as libfranka 0.9.2 writes them.
    assert_eq!(
        fer::MAX_JOINT_VELOCITY[0],
        2.175 - 1e-3 - 3.0 * 1e-3 * (15.0 - 1e-3)
    );
    assert_eq!(
        fer::MAX_JOINT_VELOCITY[1],
        2.175 - 1e-3 - 3.0 * 1e-3 * (7.5 - 1e-3)
    );
    assert_eq!(
        fer::MAX_JOINT_VELOCITY[4],
        2.610 - 1e-3 - 3.0 * 1e-3 * (15.0 - 1e-3)
    );
    assert_eq!(
        fer::MAX_JOINT_VELOCITY[6],
        2.610 - 1e-3 - 3.0 * 1e-3 * (20.0 - 1e-3)
    );
    assert_eq!(fer::MIN_JOINT_VELOCITY[0], -fer::MAX_JOINT_VELOCITY[0]);

    // rate_limiting.h:76-77 and 89-90.
    assert_eq!(
        fer::MAX_TRANSLATIONAL_VELOCITY,
        2.0 - 1e-3 - 3.0 * 1e-3 * (13.0 - 1e-3)
    );
    assert_eq!(
        fer::MAX_ROTATIONAL_VELOCITY,
        2.5 - 1e-3 - 3.0 * 1e-3 * (25.0 - 1e-3)
    );
    assert_eq!(
        fer::MAX_ELBOW_VELOCITY,
        2.175 - 1e-3 - 3.0 * 1e-3 * (10.0 - 1e-3)
    );
    assert_eq!(fer::MIN_ELBOW_VELOCITY, -fer::MAX_ELBOW_VELOCITY);

    // The jerks and accelerations are per-joint on the FER, unlike the FR3's flat arrays.
    assert!(close(fer::MAX_JOINT_JERK[0], 7499.999));
    assert!(close(fer::MAX_JOINT_JERK[5], 9999.999));
    assert!(close(fer::MAX_JOINT_ACCELERATION[1], 7.499));
    assert!(fer::MAX_TORQUE_RATE.iter().all(|v| close(*v, 999.999)));
    assert!(close(fer::MAX_TRANSLATIONAL_JERK, 6499.999));
    assert!(close(fer::MAX_TRANSLATIONAL_ACCELERATION, 12.999));
    assert!(close(fer::MAX_ROTATIONAL_JERK, 12499.999));
    assert!(close(fer::MAX_ROTATIONAL_ACCELERATION, 24.999));
    assert!(close(fer::MAX_ELBOW_JERK, 4999.999));
    assert!(close(fer::MAX_ELBOW_ACCELERATION, 9.999));
    assert_eq!(fer::NORM_EPS, f64::EPSILON);
    assert_eq!(fer::FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE, 0.99);
}
