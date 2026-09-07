//! `TEST(RateLimiting, MaxDerivative)` — the torque overload.

use super::*;

// ---- TEST(RateLimiting, MaxDerivative) ----
#[test]
fn max_derivative() {
    let max_derivatives = [100.0; 7];
    let last_cmd_values = [0.0; 7];
    let eps = 1e-2;

    // Desired values are into limits and unchanged after limitRate
    let values_into_limits =
        generate_values_into_limits(&last_cmd_values, &max_derivatives, eps, DELTA_T);
    assert!(!violates_rate_limits_1(
        &max_derivatives,
        &values_into_limits,
        &last_cmd_values,
        DELTA_T
    ));
    assert_eq!(
        values_into_limits,
        limit_rate_torques(&max_derivatives, &values_into_limits, &last_cmd_values).unwrap()
    );

    // Desired values are outside limits and limited after limitRate
    let values_outside_limits =
        generate_values_outside_limits(&last_cmd_values, &max_derivatives, eps, DELTA_T);
    let limited_values =
        limit_rate_torques(&max_derivatives, &values_outside_limits, &last_cmd_values).unwrap();
    assert!(violates_rate_limits_1(
        &max_derivatives,
        &values_outside_limits,
        &last_cmd_values,
        DELTA_T
    ));
    assert_ne!(values_outside_limits, limited_values);
    assert!(!violates_rate_limits_1(
        &max_derivatives,
        &limited_values,
        &last_cmd_values,
        DELTA_T
    ));
}

// ---- TEST(RateLimiting, JointVelocity) ----
