//! The impedance law and its options, offline.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use nalgebra::{SMatrix, SVector};

use super::super::ik::IkOptions;
use super::super::impedance::*;
use super::is_invalid_argument;

const Q: [f64; 7] = [0.1, -0.6, 0.2, -2.1, 0.0, 1.7, 0.9];
const DELTA: [f64; 7] = [0.01, -0.02, 0.03, 0.005, -0.01, 0.02, -0.03];
const DQ: [f64; 7] = [0.1, -0.2, 0.3, 0.05, -0.1, 0.2, -0.3];
const ZERO: [f64; 7] = [0.0; 7];

/// A deterministic 6x7 with entries in `[-1, 1)`, column-major.
fn random_jacobian() -> [f64; 42] {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    std::array::from_fn(|_| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 52) as f64 * 2.0 - 1.0
    })
}

/// The Cartesian options with `gains` and no clamp.
fn unclamped(gains: ImpedanceGains) -> ImpedanceOptions {
    ImpedanceOptions::cartesian()
        .with_gains(gains)
        .with_torque_limits([1e6; 7])
}

fn goal() -> [f64; 7] {
    std::array::from_fn(|i| Q[i] + DELTA[i])
}

fn assert_close(actual: &[f64; 7], expected: &[f64; 7], tolerance: f64) {
    for i in 0..7 {
        assert!(
            (actual[i] - expected[i]).abs() <= tolerance,
            "joint {i}: {} vs {}",
            actual[i],
            expected[i]
        );
    }
}

/// `Kp e + Kd de` with the gains as full matrices, the reference for the law.
fn reference(
    gains: &ImpedanceGains,
    jacobian: &[f64; 42],
    e: &[f64; 7],
    de: &[f64; 7],
) -> [f64; 7] {
    let j = SMatrix::<f64, 6, 7>::from_column_slice(jacobian);
    let diag6 = |d: [f64; 6]| SMatrix::<f64, 6, 6>::from_diagonal(&SVector::from(d));
    let diag7 = |d: [f64; 7]| SMatrix::<f64, 7, 7>::from_diagonal(&SVector::from(d));
    let kp = j.transpose() * diag6(gains.cartesian_stiffness) * j + diag7(gains.joint_stiffness);
    let kd = j.transpose() * diag6(gains.cartesian_damping) * j + diag7(gains.joint_damping);
    (kp * SVector::<f64, 7>::from(*e) + kd * SVector::<f64, 7>::from(*de)).into()
}

#[test]
fn zero_torque_at_the_goal_at_rest() {
    let options = ImpedanceOptions::cartesian();
    let tau = impedance_torques(&options, &random_jacobian(), &Q, &ZERO, &Q, &ZERO, &ZERO);
    assert_eq!(tau, ZERO);
}

#[test]
fn coriolis_passes_through() {
    let coriolis = [0.3, -1.2, 0.5, 2.0, -0.1, 0.05, 0.7];
    let options = ImpedanceOptions::cartesian();
    let tau = impedance_torques(
        &options,
        &random_jacobian(),
        &Q,
        &ZERO,
        &Q,
        &ZERO,
        &coriolis,
    );
    assert_eq!(tau, coriolis);
}

#[test]
fn clamp_binds_symmetrically() {
    let limits = [10.0, 20.0, 30.0, 40.0, 5.0, 6.0, 7.0];
    let options = ImpedanceOptions::joint().with_torque_limits(limits);
    let far: [f64; 7] = std::array::from_fn(|i| Q[i] + 100.0);
    let up = impedance_torques(&options, &[0.0; 42], &far, &ZERO, &Q, &ZERO, &ZERO);
    let down = impedance_torques(&options, &[0.0; 42], &Q, &ZERO, &far, &ZERO, &ZERO);
    assert_eq!(up, limits);
    assert_eq!(down, limits.map(|t| -t));
}

#[test]
fn cartesian_stiffness_alone_is_jt_kx_j() {
    let gains = ImpedanceGains {
        cartesian_stiffness: [750.0, 700.0, 650.0, 15.0, 14.0, 13.0],
        cartesian_damping: [0.0; 6],
        joint_stiffness: [0.0; 7],
        joint_damping: [0.0; 7],
    };
    let jacobian = random_jacobian();
    let tau = impedance_torques(
        &unclamped(gains),
        &jacobian,
        &goal(),
        &ZERO,
        &Q,
        &ZERO,
        &ZERO,
    );
    assert_close(&tau, &reference(&gains, &jacobian, &DELTA, &ZERO), 1e-9);
}

#[test]
fn damping_and_joint_gains_follow_the_law() {
    let gains = ImpedanceGains::CARTESIAN;
    let jacobian = random_jacobian();
    let tau = impedance_torques(&unclamped(gains), &jacobian, &goal(), &ZERO, &Q, &DQ, &ZERO);
    assert_close(
        &tau,
        &reference(&gains, &jacobian, &DELTA, &DQ.map(|v| -v)),
        1e-9,
    );
}

#[test]
fn the_damping_acts_on_the_velocity_error_unless_feedforward_is_off() {
    let gains = ImpedanceGains::CARTESIAN;
    let jacobian = random_jacobian();
    let dq_goal = [0.2, -0.1, 0.25, 0.15, 0.0, 0.1, -0.2];
    let on = unclamped(gains).with_velocity_feedforward(true);
    let with = impedance_torques(&on, &jacobian, &goal(), &dq_goal, &Q, &DQ, &ZERO);
    let de: [f64; 7] = std::array::from_fn(|i| dq_goal[i] - DQ[i]);
    assert_close(&with, &reference(&gains, &jacobian, &DELTA, &de), 1e-9);
    // Moving with the goal, the damping is silent.
    let riding = impedance_torques(&on, &jacobian, &Q, &DQ, &Q, &DQ, &ZERO);
    assert_close(&riding, &ZERO, 1e-12);
    let parity = unclamped(gains).with_velocity_feedforward(false);
    let without = impedance_torques(&parity, &jacobian, &goal(), &dq_goal, &Q, &DQ, &ZERO);
    assert_close(
        &without,
        &reference(&gains, &jacobian, &DELTA, &DQ.map(|v| -v)),
        1e-9,
    );
}

#[test]
fn the_feedforward_gain_interpolates_between_riding_the_goal_and_damping_on_measurement() {
    let gains = ImpedanceGains::CARTESIAN;
    let jacobian = random_jacobian();
    let dq_goal = [0.3, -0.2, 0.1, 0.4, -0.1, 0.2, -0.3];
    let on = unclamped(gains).with_velocity_feedforward(true);
    let at = |g: f64| {
        let options = on.with_velocity_feedforward_gain(g);
        impedance_torques(&options, &jacobian, &goal(), &dq_goal, &Q, &DQ, &ZERO)
    };
    // Gain 1 is the velocity error, gain 0 is `-dq` alone -- i.e. the same law the boolean
    // switches off -- so the gain subsumes it rather than adding a second way to say it.
    let full = impedance_torques(&on, &jacobian, &goal(), &dq_goal, &Q, &DQ, &ZERO);
    assert_close(&at(1.0), &full, 1e-12);
    let off = unclamped(gains).with_velocity_feedforward(false);
    let none = impedance_torques(&off, &jacobian, &goal(), &dq_goal, &Q, &DQ, &ZERO);
    assert_close(&at(0.0), &none, 1e-12);
    // And it is linear in between: half the feedforward is half way between the two.
    let half: [f64; 7] = std::array::from_fn(|i| 0.5 * (full[i] + none[i]));
    assert_close(&at(0.5), &half, 1e-9);
}

#[test]
fn the_feedforward_cutoff_is_off_at_the_maximum_and_refused_outside_it() {
    let options = ImpedanceOptions::cartesian();
    assert_eq!(
        options.velocity_feedforward_cutoff,
        crate::lowpass_filter::MAX_CUTOFF_FREQUENCY
    );
    assert_eq!(options.velocity_feedforward_gain, 1.0);
    options.validate().unwrap();
    for bad in [0.0, -1.0, f64::NAN] {
        let why = format!(
            "{:?}",
            options
                .with_velocity_feedforward_cutoff(bad)
                .validate()
                .unwrap_err()
        );
        assert!(why.contains("velocity_feedforward_cutoff"), "{bad}: {why}");
    }
    for bad in [-0.1, 1.1, f64::NAN] {
        let why = format!(
            "{:?}",
            options
                .with_velocity_feedforward_gain(bad)
                .validate()
                .unwrap_err()
        );
        assert!(why.contains("velocity_feedforward_gain"), "{bad}: {why}");
    }
}

#[test]
fn projected_joint_gains_act_in_the_nullspace_only() {
    let gains = ImpedanceGains::CARTESIAN;
    let jacobian = random_jacobian();
    let j = SMatrix::<f64, 6, 7>::from_column_slice(&jacobian);
    // A damping of 1e-9 makes the projector exact to rounding.
    let options = unclamped(gains)
        .with_project_joint_gains(true)
        .with_ik(IkOptions {
            damping: 1e-9,
            ..IkOptions::default()
        });
    let law = |e: &[f64; 7], de: &[f64; 7]| {
        let q_goal: [f64; 7] = std::array::from_fn(|i| Q[i] + e[i]);
        let dq: [f64; 7] = de.map(|v| -v);
        impedance_torques(&options, &jacobian, &q_goal, &ZERO, &Q, &dq, &ZERO)
    };
    let relative = |actual: &[f64; 7], expected: &[f64; 7]| {
        let scale = expected.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(scale > 0.1, "a trivial reference: {expected:?}");
        assert_close(actual, expected, 1e-9 * scale);
    };
    // An error in the range of Jᵀ: the joint terms contribute nothing beyond Jᵀ Kx J e.
    let v = SVector::<f64, 6>::new(0.01, -0.02, 0.015, 0.1, -0.05, 0.08);
    let e: [f64; 7] = (j.transpose() * v).into();
    let cartesian_only = ImpedanceGains {
        joint_stiffness: [0.0; 7],
        joint_damping: [0.0; 7],
        ..gains
    };
    relative(
        &law(&e, &ZERO),
        &reference(&cartesian_only, &jacobian, &e, &ZERO),
    );
    relative(
        &law(&ZERO, &e),
        &reference(&cartesian_only, &jacobian, &ZERO, &e),
    );
    // An error in the nullspace of J: the Cartesian terms are zero and N Kq N e acts.
    let exact = SMatrix::<f64, 7, 7>::identity()
        - j.transpose() * (j * j.transpose()).try_inverse().unwrap() * j;
    let e: [f64; 7] = (exact * SVector::<f64, 7>::from(DELTA) * 10.0).into();
    assert!((j * SVector::<f64, 7>::from(e)).norm() < 1e-12);
    let kq = SVector::<f64, 7>::from(gains.joint_stiffness);
    let kqd = SVector::<f64, 7>::from(gains.joint_damping);
    let se = SVector::<f64, 7>::from(e);
    relative(&law(&e, &ZERO), &(exact * kq.component_mul(&se)).into());
    relative(&law(&ZERO, &e), &(exact * kqd.component_mul(&se)).into());
    // Unprojected, the same nullspace error meets the full joint spring.
    let unprojected = unclamped(gains);
    let q_goal: [f64; 7] = std::array::from_fn(|i| Q[i] + e[i]);
    let tau = impedance_torques(&unprojected, &jacobian, &q_goal, &ZERO, &Q, &ZERO, &ZERO);
    assert_close(&tau, &kq.component_mul(&se).into(), 1e-9);
}

/// The FER at the ready configuration, the goal 1 cm along x through the exact pseudo-inverse,
/// the arm at rest: the torques map back to a force along +x of `Kx e` = 7.5 N plus the
/// unprojected joint springs' share (4.25 N at these gains, with 1.2 N of coupling into z),
/// exactly 7.5 N projected.
#[test]
fn a_centimetre_along_x_pulls_the_ready_arm_with_kx_e_plus_the_joint_springs() {
    use crate::model::{Frame, Model};
    let ready = [
        0.0,
        -FRAC_PI_4,
        0.0,
        -3.0 * FRAC_PI_4,
        0.0,
        FRAC_PI_2,
        FRAC_PI_4,
    ];
    let identity: [f64; 16] = {
        let mut t = [0.0; 16];
        for i in 0..4 {
            t[5 * i] = 1.0;
        }
        t
    };
    let model = Model::native_fer();
    let jacobian = model.zero_jacobian_q(Frame::EndEffector, &ready, &identity, &identity);
    let j = SMatrix::<f64, 6, 7>::from_column_slice(&jacobian);
    let jjt_inverse = (j * j.transpose()).try_inverse().unwrap();
    let pseudo_inverse = j.transpose() * jjt_inverse;
    let twist = SVector::<f64, 6>::new(0.01, 0.0, 0.0, 0.0, 0.0, 0.0);
    let q_goal: [f64; 7] = (SVector::<f64, 7>::from(ready) + pseudo_inverse * twist).into();
    let wrench_of = |options: &ImpedanceOptions| -> SVector<f64, 6> {
        let tau = impedance_torques(options, &jacobian, &q_goal, &ZERO, &ready, &ZERO, &ZERO);
        // `tau = Jᵀ w` for the wrench `w` the arm feels: `w = (J Jᵀ)⁻¹ J tau`.
        jjt_inverse * j * SVector::<f64, 7>::from(tau)
    };
    let along_x = |w: &SVector<f64, 6>| {
        let force = [w[0], w[1], w[2]];
        assert!(
            force[0] > 0.0 && force[1].abs() < 0.15 * force[0] && force[2].abs() < 0.15 * force[0],
            "the force does not point along +x: {force:?}"
        );
        (force[0] * force[0] + force[1] * force[1] + force[2] * force[2]).sqrt()
    };
    let unprojected = along_x(&wrench_of(&ImpedanceOptions::cartesian()));
    assert!(
        (unprojected - 11.75).abs() < 0.05 * 11.75,
        "unprojected: {unprojected} N"
    );
    let projected = along_x(&wrench_of(
        &ImpedanceOptions::cartesian().with_project_joint_gains(true),
    ));
    assert!(
        (projected - 7.5).abs() < 0.05 * 7.5,
        "projected: {projected} N"
    );
    assert!(unprojected > projected + 0.5);
}

#[test]
fn defaults_validate() {
    ImpedanceGains::CARTESIAN.validate().unwrap();
    ImpedanceGains::DROID.validate().unwrap();
    ImpedanceGains::JOINT.validate().unwrap();
    let cartesian = ImpedanceOptions::cartesian();
    cartesian.validate().unwrap();
    ImpedanceOptions::joint().validate().unwrap();
    assert_eq!(cartesian.gains, ImpedanceGains::CARTESIAN);
    assert_eq!(ImpedanceOptions::joint().gains, ImpedanceGains::JOINT);
    // Feedforward is off by default on both interfaces: on, it shook real arms.
    assert!(!cartesian.velocity_feedforward);
    assert!(!ImpedanceOptions::joint().velocity_feedforward);
    assert!(!cartesian.project_joint_gains);
    // The default is DROID's preset with the Cartesian damping raised, nothing else.
    assert_eq!(
        ImpedanceGains::CARTESIAN.cartesian_damping,
        [50.0, 50.0, 90.0, 2.0, 2.0, 2.0]
    );
    assert_eq!(
        ImpedanceGains::DROID.cartesian_damping,
        [37.0, 37.0, 37.0, 2.0, 2.0, 2.0]
    );
    assert_eq!(
        ImpedanceGains {
            cartesian_damping: ImpedanceGains::DROID.cartesian_damping,
            ..ImpedanceGains::CARTESIAN
        },
        ImpedanceGains::DROID
    );
    // The joint velocity cap and the barrier are on by default, on both interfaces.
    for options in [cartesian, ImpedanceOptions::joint()] {
        assert_eq!(options.joint_velocity_fraction, 0.7);
        assert_eq!(options.velocity_barrier_fraction, 0.85);
    }
}

#[test]
fn validate_rejects_bad_fields() {
    let mut gains = ImpedanceGains::CARTESIAN;
    gains.joint_damping[2] = -1.0;
    assert!(gains.validate().is_err());
    gains.joint_damping[2] = f64::NAN;
    assert!(gains.validate().is_err());
    let options = ImpedanceOptions::cartesian();
    assert!(options.with_gains(gains).validate().is_err());
    assert!(options.with_torque_limits([0.0; 7]).validate().is_err());
    assert!(options
        .with_torque_limits([f64::INFINITY; 7])
        .validate()
        .is_err());
    // The clamp may reach the rated torque and not exceed it.
    assert!(options.with_torque_limits(RATED_TORQUES).validate().is_ok());
    let mut over = RATED_TORQUES;
    over[0] += 0.5;
    assert!(options.with_torque_limits(over).validate().is_err());
    let mut over = RATED_TORQUES;
    over[6] += 0.5;
    assert!(options.with_torque_limits(over).validate().is_err());
    assert!(options.with_cutoff_frequency(0.0).validate().is_err());
    assert!(options.with_cutoff_frequency(f64::NAN).validate().is_err());
    assert!(options
        .with_posture(Some([f64::NAN; 7]))
        .validate()
        .is_err());
    assert!(options.with_posture(Some(Q)).validate().is_ok());
    let ik = IkOptions {
        iterations: 0,
        ..IkOptions::default()
    };
    assert!(options.with_ik(ik).validate().is_err());
    for bad in [0.0, -0.1, f64::NAN, f64::INFINITY] {
        for field in 0..3 {
            let mut leash = Leash::default();
            *[
                &mut leash.translation,
                &mut leash.rotation,
                &mut leash.joint,
            ][field] = bad;
            assert!(options.with_leash(leash).validate().is_err(), "{leash:?}");
        }
    }
    assert_eq!(
        options.leash,
        Leash {
            translation: 0.025,
            rotation: 0.15,
            joint: 0.1
        }
    );
    for bad in [0.0, -0.1, 1.01, f64::NAN, f64::INFINITY] {
        let result = options.with_joint_velocity_fraction(bad).validate();
        assert!(
            is_invalid_argument(result, "joint_velocity_fraction"),
            "{bad}"
        );
    }
    // The barrier sits at or above the cap and at most at the limit.
    for bad in [0.69, 1.01, f64::NAN] {
        let result = options.with_velocity_barrier_fraction(bad).validate();
        assert!(
            is_invalid_argument(result, "velocity_barrier_fraction"),
            "{bad}"
        );
    }
    assert!(options.with_joint_velocity_fraction(0.8).validate().is_ok());
    assert!(options
        .with_velocity_barrier_fraction(0.8)
        .validate()
        .is_ok());
    let full = options
        .with_joint_velocity_fraction(1.0)
        .with_velocity_barrier_fraction(1.0);
    assert!(full.validate().is_ok());
    // The margin: 0.05 by default, from over the tracking overshoot and the FR3's envelope to half
    // a radian.
    assert_eq!(options.joint_position_margin, 0.05);
    for good in [0.035, 0.5] {
        assert!(options.with_joint_position_margin(good).validate().is_ok());
    }
    for bad in [0.034, 0.51, f64::NAN, f64::INFINITY] {
        let result = options.with_joint_position_margin(bad).validate();
        assert!(
            is_invalid_argument(result, "joint_position_margin"),
            "{bad}"
        );
    }
}
