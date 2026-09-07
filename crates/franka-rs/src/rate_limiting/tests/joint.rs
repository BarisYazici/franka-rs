//! `TEST(RateLimiting, JointVelocity)` and `TEST(RateLimiting, JointPosition)`.

use super::*;

// ---- TEST(RateLimiting, JointVelocity) ----
#[test]
fn joint_velocity() {
    let last_cmd_velocity = [0.0; 7];
    let last_cmd_acceleration = [0.0; 7];
    let max_acceleration = [10.0; 7];
    let max_jerk = [100.0; 7];
    let eps = 1e-2;

    // Desired values are into limits and unchanged after limitRate (jerk)
    let joint_velocity_into_limits = integrate_one_sample(
        &last_cmd_velocity,
        &generate_values_into_limits(&last_cmd_acceleration, &max_jerk, eps, DELTA_T),
        DELTA_T,
    );
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &joint_velocity_into_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_eq!(
        joint_velocity_into_limits,
        limit_rate_joint_velocities(
            &JOINTS_NO_LIMIT,
            &JOINTS_NO_LOWER_LIMIT,
            &JOINTS_NO_LIMIT,
            &max_jerk,
            &joint_velocity_into_limits,
            &last_cmd_velocity,
            &last_cmd_acceleration
        )
        .unwrap()
    );

    // Desired values are into limits and unchanged after limitRate (acceleration)
    let joint_velocity_into_limits =
        generate_values_into_limits(&last_cmd_velocity, &max_acceleration, eps, DELTA_T);
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &joint_velocity_into_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_eq!(
        joint_velocity_into_limits,
        limit_rate_joint_velocities(
            &JOINTS_NO_LIMIT,
            &JOINTS_NO_LOWER_LIMIT,
            &max_acceleration,
            &JOINTS_NO_LIMIT,
            &joint_velocity_into_limits,
            &last_cmd_velocity,
            &last_cmd_acceleration
        )
        .unwrap()
    );

    // Desired values are outside limits (jerk violation) and limited after limitRate
    let joint_velocity_outside_limits = integrate_one_sample(
        &last_cmd_velocity,
        &generate_values_outside_limits(&last_cmd_acceleration, &max_jerk, eps, DELTA_T),
        DELTA_T,
    );
    let limited_joint_velocity = limit_rate_joint_velocities(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LOWER_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &joint_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &joint_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(joint_velocity_outside_limits, limited_joint_velocity);
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &limited_joint_velocity,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));

    // Desired values are outside limits (acceleration violation) and limited after limitRate
    let joint_velocity_outside_limits =
        generate_values_outside_limits(&last_cmd_velocity, &max_acceleration, eps, DELTA_T);
    let limited_joint_velocity = limit_rate_joint_velocities(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LOWER_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &joint_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &joint_velocity_outside_limits,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(joint_velocity_outside_limits, limited_joint_velocity);
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &limited_joint_velocity,
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
}

// ---- TEST(RateLimiting, JointPosition) ----
#[test]
fn joint_position() {
    let last_cmd_position = [0.0; 7];
    let last_cmd_velocity = [0.0; 7];
    let last_cmd_acceleration = [0.0; 7];
    let max_acceleration = [10.0; 7];
    let max_jerk = [100.0; 7];
    let eps = 1e-2;

    // Desired values are into limits and unchanged after limitRate (jerk)
    let joint_position_into_limits = integrate_one_sample(
        &last_cmd_position,
        &integrate_one_sample(
            &last_cmd_velocity,
            &generate_values_into_limits(&last_cmd_acceleration, &max_jerk, eps, DELTA_T),
            DELTA_T,
        ),
        DELTA_T,
    );
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &differentiate_one_sample(&joint_position_into_limits, &last_cmd_position, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_eq!(
        joint_position_into_limits,
        limit_rate_joint_positions(
            &JOINTS_NO_LIMIT,
            &JOINTS_NO_LOWER_LIMIT,
            &JOINTS_NO_LIMIT,
            &max_jerk,
            &joint_position_into_limits,
            &last_cmd_position,
            &last_cmd_velocity,
            &last_cmd_acceleration
        )
        .unwrap()
    );

    // Desired values are into limits and unchanged after limitRate (acceleration)
    let joint_position_into_limits = integrate_one_sample(
        &last_cmd_position,
        &generate_values_into_limits(&last_cmd_velocity, &max_acceleration, eps, DELTA_T),
        DELTA_T,
    );
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &differentiate_one_sample(&joint_position_into_limits, &last_cmd_position, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_eq!(
        joint_position_into_limits,
        limit_rate_joint_positions(
            &JOINTS_NO_LIMIT,
            &JOINTS_NO_LOWER_LIMIT,
            &max_acceleration,
            &JOINTS_NO_LIMIT,
            &joint_position_into_limits,
            &last_cmd_position,
            &last_cmd_velocity,
            &last_cmd_acceleration
        )
        .unwrap()
    );

    // Desired values are outside limits (jerk violation) and limited after limitRate
    let joint_position_outside_limits = integrate_one_sample(
        &last_cmd_position,
        &integrate_one_sample(
            &last_cmd_velocity,
            &generate_values_outside_limits(&last_cmd_acceleration, &max_jerk, eps, DELTA_T),
            DELTA_T,
        ),
        DELTA_T,
    );
    let limited_joint_position = limit_rate_joint_positions(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LOWER_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &joint_position_outside_limits,
        &last_cmd_position,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &differentiate_one_sample(&joint_position_outside_limits, &last_cmd_position, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(joint_position_outside_limits, limited_joint_position);
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LIMIT,
        &max_jerk,
        &differentiate_one_sample(&limited_joint_position, &last_cmd_position, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));

    // Desired values outside limits (acceleration violation) and limited after limitRate
    let joint_position_outside_limits = integrate_one_sample(
        &last_cmd_position,
        &generate_values_outside_limits(&last_cmd_velocity, &max_acceleration, eps, DELTA_T),
        DELTA_T,
    );
    let limited_joint_position = limit_rate_joint_positions(
        &JOINTS_NO_LIMIT,
        &JOINTS_NO_LOWER_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &joint_position_outside_limits,
        &last_cmd_position,
        &last_cmd_velocity,
        &last_cmd_acceleration,
    )
    .unwrap();
    assert!(violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &differentiate_one_sample(&joint_position_outside_limits, &last_cmd_position, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
    assert_ne!(joint_position_outside_limits, limited_joint_position);
    assert!(!violates_rate_limits_3(
        &JOINTS_NO_LIMIT,
        &max_acceleration,
        &JOINTS_NO_LIMIT,
        &differentiate_one_sample(&limited_joint_position, &last_cmd_position, DELTA_T),
        &last_cmd_velocity,
        &last_cmd_acceleration,
        DELTA_T
    ));
}

// ---- TEST(RateLimiting, CartesianVelocity) ----
