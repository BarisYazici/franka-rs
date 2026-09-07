//! Torque conversion: rate limiting against `tau_J_d`, the `float32` margin and the v5
//! envelope.

use super::*;
use crate::control_types::Torques;
use crate::rate_limiting::{limit_rate_torques, DELTA_T};
use crate::wire::robot::ControllerCommand;

// Port of ControlLoops.SpinOnceWithMotionAndControllerCallback for the torque half:
// with the rate limiter on, a first command of {0,1,2,...} is clipped to a 1 Nm step.
#[test]
fn torques_are_rate_limited_against_tau_j_d() {
    let state = valid_state();
    let mut command = ControllerCommand::default();
    convert_torques(
        &Torques::new([0., 1., 2., 3., 4., 5., 6.]),
        &state,
        &V10_RATE,
        true,
        MAX_CUTOFF_FREQUENCY,
        &mut command,
    )
    .unwrap();
    let limited = f64s_to_f64(&command.tau_J_d);
    let expected = [0., 1., 1., 1., 1., 1., 1.];
    for i in 0..7 {
        assert!(
            (limited[i] - expected[i]).abs() < 1e-5,
            "tau_J_d[{i}] = {} expected {}",
            limited[i],
            expected[i]
        );
    }
}

/// The float32 trap: at |τ| ≥ 64 Nm the robot's `tau_J_d` reaches us quantised to
/// `f32`, but the robot judges our command's rate against the `f64` it last received. A
/// saturated command limited against the quantised value would read as up to ≈ 1000.0028
/// Nm/s there — over the nominal 1000 Nm/s. [`torque_rate_margin`] gives that budget back.
///
/// The sweep covers both signs and both interesting endpoints (64 Nm, where the f32
/// ULP jumps to 2⁻¹⁷, and 87 Nm, the FR3's joint-1 torque limit), simulating the rounding
/// the robot performs.
#[test]
fn saturated_torques_stay_below_the_nominal_rate_after_f32_quantisation() {
    for magnitude in [64.0_f64, 87.0] {
        for sign in [1.0_f64, -1.0] {
            for step in 0..1000 {
                // A true (f64) last command the robot holds, and what it publishes for it.
                let tau_true = sign * (magnitude + f64::from(step) * 1e-3);
                let tau_published = f64::from(tau_true as f32);

                let mut state = valid_state();
                state.tau_J_d = [tau_published; 7];
                let margin = torque_rate_margin(tau_published);

                // Saturate the limiter in both directions.
                for direction in [1.0_f64, -1.0] {
                    let mut command = ControllerCommand::default();
                    convert_torques(
                        &Torques::new([tau_published + direction * 100.0; 7]),
                        &state,
                        &V10_RATE,
                        true,
                        MAX_CUTOFF_FREQUENCY,
                        &mut command,
                    )
                    .unwrap();
                    let limited = f64s_to_f64(&command.tau_J_d)[0];

                    // 1. The step we take is the shrunk budget, not the nominal one.
                    let delta = (limited - tau_published).abs();
                    let allowed = (V10_RATE.max_torque_rate[0] - margin) * DELTA_T;
                    assert!(
                        delta <= allowed + 1e-12,
                        "tau_J_d {tau_published}: step {delta} exceeds the allowed {allowed}"
                    );

                    // 2. The rate the robot measures — our command against the f64 value it
                    //    really holds — stays below the nominal 1000 Nm/s.
                    let rate_at_the_robot = (limited - tau_true).abs() / DELTA_T;
                    assert!(
                        rate_at_the_robot < 1000.0,
                        "tau_J_d {tau_true} (published {tau_published}): the robot measures \
                         {rate_at_the_robot} Nm/s, over the nominal 1000 Nm/s"
                    );
                }
            }
        }
    }
}

/// Without the margin the same sweep would breach 1000 Nm/s, i.e. the margin is load
/// bearing rather than decorative.
#[test]
fn without_the_margin_f32_quantisation_would_breach_the_nominal_rate() {
    let mut worst: f64 = 0.0;
    for step in 0..200_000 {
        let tau_true = 64.0 + f64::from(step) * 1e-4;
        let tau_published = f64::from(tau_true as f32);
        // The unmargined libfranka computation: saturate MAX_TORQUE_RATE against the
        // published value.
        let limited = limit_rate_torques(
            &V10_RATE.max_torque_rate,
            &[tau_published + 100.0; 7],
            &[tau_published; 7],
        )
        .unwrap()[0];
        worst = worst.max((limited - tau_true) / DELTA_T);
    }
    assert!(
        worst > 1000.0,
        "expected the unmargined rate to exceed 1000 Nm/s, worst was {worst}"
    );
    assert!(
        torque_rate_margin(87.0) > worst - 1000.0,
        "the margin at 87 Nm must cover the worst observed overshoot"
    );
}

#[test]
fn the_torque_rate_margin_is_zero_at_zero_and_half_an_f32_ulp_otherwise() {
    assert_eq!(torque_rate_margin(0.0), 0.0);
    // At 64 Nm: half of f32::EPSILON * 64 = 2^-23 * 64 / 2 Nm, per 1 ms.
    let expected = f64::from(f32::EPSILON) * 64.0 / 2.0 / DELTA_T;
    assert!((torque_rate_margin(-64.0) - expected).abs() < 1e-15);
    assert!(
        torque_rate_margin(64.0) > 1e-3,
        "the margin must exceed LIMIT_EPS at 64 Nm"
    );
}

/// The FER envelope comes from libfranka 0.9.2's `rate_limiting.h` and, unlike the FR3's,
/// carries no `float32` `tau_J_d` margin: the FCI v5 state publishes `tau_J_d` as a
/// `double`, so shrinking the budget would take torque away for no reason.
#[test]
fn the_v5_envelope_is_the_fer_one_and_skips_the_f32_torque_margin() {
    const { assert!(!V5_RATE.torque_f32_margin) };
    const { assert!(V10_RATE.torque_f32_margin) };

    assert_eq!(
        V5_RATE.max_joint_acceleration,
        crate::rate_limiting::fer::MAX_JOINT_ACCELERATION
    );
    assert_eq!(
        V5_RATE.max_joint_jerk,
        crate::rate_limiting::fer::MAX_JOINT_JERK
    );
    assert_eq!(
        V5_RATE.max_elbow_velocity,
        crate::rate_limiting::fer::MAX_ELBOW_VELOCITY
    );
    assert_eq!(
        V5_RATE.min_elbow_velocity,
        -crate::rate_limiting::fer::MAX_ELBOW_VELOCITY
    );
    assert_eq!(
        V5_RATE.max_translational_velocity,
        crate::rate_limiting::fer::MAX_TRANSLATIONAL_VELOCITY
    );
    assert_eq!(
        V5_RATE.max_rotational_velocity,
        crate::rate_limiting::fer::MAX_ROTATIONAL_VELOCITY
    );
    assert_eq!(
        V5_RATE.max_torque_rate,
        crate::rate_limiting::fer::MAX_TORQUE_RATE
    );

    // The two envelopes really differ: the FR3's joint 1 jerk budget is not the FER's.
    assert_ne!(V5_RATE.max_joint_jerk[0], V10_RATE.max_joint_jerk[0]);
}

/// With the margin off, a saturated FER torque command takes exactly the full
/// `kMaxTorqueRate` step even at a `tau_J_d` where the FR3 path would give budget back.
#[test]
fn v5_torques_are_limited_against_the_full_fer_torque_rate() {
    let mut state = valid_state();
    state.tau_J_d = [87.0; 7];

    let mut command = ControllerCommand::default();
    convert_torques(
        &Torques::new([187.0; 7]),
        &state,
        &V5_RATE,
        true,
        MAX_CUTOFF_FREQUENCY,
        &mut command,
    )
    .unwrap();
    let step = f64s_to_f64(&command.tau_J_d)[0] - 87.0;
    assert!(
        (step - V5_RATE.max_torque_rate[0] * DELTA_T).abs() < 1e-12,
        "v5 step {step} must be the full budget"
    );

    let mut command = ControllerCommand::default();
    convert_torques(
        &Torques::new([187.0; 7]),
        &state,
        &V10_RATE,
        true,
        MAX_CUTOFF_FREQUENCY,
        &mut command,
    )
    .unwrap();
    let v10_step = f64s_to_f64(&command.tau_J_d)[0] - 87.0;
    assert!(v10_step < step, "the v10 margin must shrink the step");
}

#[test]
fn torques_are_passed_through_without_limiting_or_filtering() {
    let state = valid_state();
    let mut command = ControllerCommand::default();
    let torques = Torques::new([0., 1., 2., 3., 4., 5., 6.]);
    convert_torques(
        &torques,
        &state,
        &V10_RATE,
        false,
        MAX_CUTOFF_FREQUENCY,
        &mut command,
    )
    .unwrap();
    assert_eq!(f64s_to_f64(&command.tau_J_d), torques.tau_J);
}

#[test]
fn torques_are_filtered_towards_the_last_command() {
    let state = valid_state();
    let mut command = ControllerCommand::default();
    let torques = Torques::new([0., 1., 2., 3., 4., 5., 6.]);
    convert_torques(
        &torques,
        &state,
        &V10_RATE,
        false,
        crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY,
        &mut command,
    )
    .unwrap();
    // state.tau_J_d is zero, so every filtered value stays below the commanded one.
    for (filtered, commanded) in f64s_to_f64(&command.tau_J_d).iter().zip(torques.tau_J) {
        assert!(*filtered <= commanded);
    }
}

#[test]
fn invalid_torques_are_rejected() {
    let state = valid_state();
    let mut command = ControllerCommand::default();
    for invalid in [f64::NAN, f64::INFINITY] {
        let result = convert_torques(
            &Torques::new([0., invalid, 2., 3., 4., 5., 6.]),
            &state,
            &V10_RATE,
            false,
            MAX_CUTOFF_FREQUENCY,
            &mut command,
        );
        assert_eq!(
            invalid_argument(result),
            "Commanding value is infinite or NaN."
        );
    }
}
