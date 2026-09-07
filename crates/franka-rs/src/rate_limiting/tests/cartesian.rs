//! `TEST(RateLimiting, CartesianVelocity)`, `CartesianPose` and
//! `CartesianPoseIntegrationAndDifferentation`.

use super::*;

// ---- TEST(RateLimiting, CartesianVelocity) ----
#[test]
fn cartesian_velocity() {
    let last_cmd_velocity = [0.0; 6];
    let last_cmd_acceleration = [0.0; 6];
    let max_translational_acceleration = 10.0;
    let max_translational_jerk = 100.0;
    let max_rotational_acceleration = 5.0;
    let max_rotational_jerk = 50.0;
    let eps = 1e-2;

    // Into limits (rotational and translational jerk)
    let cartesian_velocity_into_limits = integrate_one_sample(
        &last_cmd_velocity,
        &generate_cartesian_values_into_limits(
            &last_cmd_acceleration,
            max_translational_jerk,
            max_rotational_jerk,
            eps,
            DELTA_T,
        ),
        DELTA_T,
    );
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &cartesian_velocity_into_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_eq!(
        cartesian_velocity_into_limits,
        limit_rate_cartesian_velocity(
            NO_LIMIT,
            NO_LIMIT,
            max_translational_jerk,
            NO_LIMIT,
            NO_LIMIT,
            max_rotational_jerk,
            &cartesian_velocity_into_limits,
            &last_cmd_velocity,
            &last_cmd_acceleration
        )
        .unwrap()
    );

    // Into limits (rotational and translational acceleration)
    let cartesian_velocity_into_limits = generate_cartesian_values_into_limits(
        &last_cmd_velocity,
        max_translational_acceleration,
        max_rotational_acceleration,
        eps,
        DELTA_T,
    );
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &cartesian_velocity_into_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_eq!(
        cartesian_velocity_into_limits,
        limit_rate_cartesian_velocity(
            NO_LIMIT,
            max_translational_acceleration,
            NO_LIMIT,
            NO_LIMIT,
            max_rotational_acceleration,
            NO_LIMIT,
            &cartesian_velocity_into_limits,
            &last_cmd_velocity,
            &last_cmd_acceleration
        )
        .unwrap()
    );

    // Outside limits (rotational and translational jerk violation)
    let cartesian_velocity_outside_limits = integrate_one_sample(
        &last_cmd_velocity,
        &generate_cartesian_values_outside_limits(
            &last_cmd_acceleration,
            max_translational_jerk,
            max_rotational_jerk,
            eps,
            DELTA_T,
        ),
        DELTA_T,
    );
    let limited_cartesian_velocity = limit_rate_cartesian_velocity(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &cartesian_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_cartesian(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &cartesian_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(
        cartesian_velocity_outside_limits,
        limited_cartesian_velocity
    );
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &limited_cartesian_velocity,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));

    // Outside limits (rotational and translational acceleration violation)
    let cartesian_velocity_outside_limits = generate_cartesian_values_outside_limits(
        &last_cmd_velocity,
        max_translational_acceleration,
        max_rotational_acceleration,
        eps,
        DELTA_T,
    );
    let limited_cartesian_velocity = limit_rate_cartesian_velocity(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &cartesian_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_cartesian(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &cartesian_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(
        cartesian_velocity_outside_limits,
        limited_cartesian_velocity
    );
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &limited_cartesian_velocity,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
}

// ---- TEST(RateLimiting, CartesianPose) ----
#[test]
fn cartesian_pose() {
    let last_cmd_pose = IDENTITY_POSE;
    let last_cmd_velocity = [0.0; 6];
    let last_cmd_acceleration = [0.0; 6];
    let max_translational_acceleration = 10.0;
    let max_translational_jerk = 100.0;
    let max_rotational_acceleration = 5.0;
    let max_rotational_jerk = 50.0;
    let eps = 1e-2;

    // Into limits (rotational and translational jerk)
    let cartesian_pose_into_limits = integrate_one_sample_pose(
        &last_cmd_pose,
        &integrate_one_sample(
            &last_cmd_velocity,
            &generate_cartesian_values_into_limits(
                &last_cmd_acceleration,
                max_translational_jerk,
                max_rotational_jerk,
                eps,
                DELTA_T,
            ),
            DELTA_T,
        ),
        DELTA_T,
    );
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &differentiate_one_sample_pose(&cartesian_pose_into_limits, &last_cmd_pose, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    let cartesian_pose_limited = limit_rate_cartesian_pose(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &cartesian_pose_into_limits,
        &last_cmd_pose,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    for i in 0..16 {
        assert!(
            (cartesian_pose_into_limits[i] - cartesian_pose_limited[i]).abs() < 1e-6,
            "element {i}: {} vs {}",
            cartesian_pose_into_limits[i],
            cartesian_pose_limited[i]
        );
    }

    // Into limits (rotational and translational acceleration)
    let cartesian_pose_into_limits = integrate_one_sample_pose(
        &last_cmd_pose,
        &generate_cartesian_values_into_limits(
            &last_cmd_velocity,
            max_translational_acceleration,
            max_rotational_acceleration,
            eps,
            DELTA_T,
        ),
        DELTA_T,
    );
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &differentiate_one_sample_pose(&cartesian_pose_into_limits, &last_cmd_pose, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    let cartesian_pose_limited = limit_rate_cartesian_pose(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &cartesian_pose_into_limits,
        &last_cmd_pose,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    for i in 0..16 {
        assert!(
            (cartesian_pose_into_limits[i] - cartesian_pose_limited[i]).abs() < 1e-6,
            "element {i}: {} vs {}",
            cartesian_pose_into_limits[i],
            cartesian_pose_limited[i]
        );
    }

    // Outside limits (rotational and translational jerk violation)
    let cartesian_pose_outside_limits = integrate_one_sample_pose(
        &last_cmd_pose,
        &integrate_one_sample(
            &last_cmd_velocity,
            &generate_cartesian_values_outside_limits(
                &last_cmd_acceleration,
                max_translational_jerk,
                max_rotational_jerk,
                eps,
                DELTA_T,
            ),
            DELTA_T,
        ),
        DELTA_T,
    );
    let limited_cartesian_pose = limit_rate_cartesian_pose(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &cartesian_pose_outside_limits,
        &last_cmd_pose,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_cartesian(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &differentiate_one_sample_pose(&cartesian_pose_outside_limits, &last_cmd_pose, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(cartesian_pose_outside_limits, limited_cartesian_pose);
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        NO_LIMIT,
        max_translational_jerk,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_jerk,
        &differentiate_one_sample_pose(&limited_cartesian_pose, &last_cmd_pose, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));

    // Outside limits (rotational and translational acceleration violation)
    let cartesian_pose_outside_limits = integrate_one_sample_pose(
        &last_cmd_pose,
        &generate_cartesian_values_outside_limits(
            &last_cmd_velocity,
            max_translational_acceleration,
            max_rotational_acceleration,
            eps,
            DELTA_T,
        ),
        DELTA_T,
    );
    let limited_cartesian_pose = limit_rate_cartesian_pose(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &cartesian_pose_outside_limits,
        &last_cmd_pose,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_cartesian(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &differentiate_one_sample_pose(&cartesian_pose_outside_limits, &last_cmd_pose, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(cartesian_pose_outside_limits, limited_cartesian_pose);
    assert!(!violates_rate_limits_cartesian(
        NO_LIMIT,
        max_translational_acceleration,
        NO_LIMIT,
        NO_LIMIT,
        max_rotational_acceleration,
        NO_LIMIT,
        &differentiate_one_sample_pose(&limited_cartesian_pose, &last_cmd_pose, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
}

// ---- TEST(RateLimiting, CartesianPoseIntegrationAndDifferentation) ----
#[test]
fn cartesian_pose_integration_and_differentiation() {
    let last_cmd_pose = [
        0., 1., 0., 0., -1., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
    ];
    let last_cmd_velocity = [1.0, 2.0, 3.0, 0.4, 0.5, 0.3];

    let cartesian_pose = integrate_one_sample_pose(&last_cmd_pose, &last_cmd_velocity, DELTA_T);
    let twist = differentiate_one_sample_pose(&cartesian_pose, &last_cmd_pose, DELTA_T);
    for i in 0..6 {
        assert!(
            (twist[i] - last_cmd_velocity[i]).abs() < 1e-6,
            "twist[{i}] = {}, expected {}",
            twist[i],
            last_cmd_velocity[i]
        );
    }
}

// ---- additional tests: the invalid-argument paths ----
