//! Conformance suite for `franka::Model` against libfranka's own numbers.
//!
//! `tests/data/model_reference_fr3.json` is produced by `tools/model-reference`,
//! a C++ program linked against libfranka 0.20.4 that drives the real
//! `franka::Model` wrapped around libfranka's Pinocchio-based
//! `franka::RobotModel`. Every quantity the public API exposes is compared
//! against it for 128 joint configurations under three load configurations.
//!
//! Tolerances: 1e-9 absolute for kinematics (poses, both Jacobians), 1e-6
//! absolute for dynamics (mass matrix, Coriolis, gravity).

use franka::model::{Frame, Model};
use franka::robot_state::RobotState;

const KINEMATIC_TOLERANCE: f64 = 1e-9;
const DYNAMIC_TOLERANCE: f64 = 1e-6;

mod conformance;
#[path = "model_conformance/fixture.rs"]
mod fixture;
/// SHA-256, so the fixture and the URDF it was generated from cannot drift
/// apart; shared with `fer_native_conformance.rs`.
mod sha256;

use conformance::{compare, MaxError};
use fixture::{array16, array3, array7, array9, load_fixture, load_urdf};

// --------------------------------------------------------------------------
// The suite
// --------------------------------------------------------------------------

#[test]
fn fixture_matches_the_committed_urdf() {
    let fixture = load_fixture();
    let digest = sha256::hex(load_urdf().as_bytes());
    assert_eq!(
        digest, fixture.meta.urdf_sha256,
        "tests/data/fr3.urdf no longer matches the URDF the fixture was generated from; \
         regenerate the fixture with tools/model-reference"
    );
    assert_eq!(fixture.meta.libfranka_version, "0.20.4");
    assert_eq!(fixture.meta.rng_seed, 20260904);
    assert!(fixture.meta.random_sample_count >= 100);
    // The suite slices `pose` / `body_jacobian` / `zero_jacobian` frame-major in
    // `Frame::ALL` order, so the fixture's frame list must be exactly that order.
    assert_eq!(
        fixture.meta.frames,
        [
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
            "Joint7",
            "Flange",
            "EndEffector",
            "Stiffness"
        ]
    );
    assert_eq!(fixture.meta.frames.len(), Frame::ALL.len());
    assert_eq!(fixture.load_configs.len(), 3);
    assert_eq!(fixture.samples.len(), fixture.meta.random_sample_count + 8);
}

/// libfranka copies Pinocchio's `Motion` layout out of Eigen verbatim, so the
/// rows of a 6x7 Jacobian column are `vx, vy, vz, wx, wy, wz`. Pin that down
/// against the fixture rather than trusting the documentation: at `q = 0` the
/// zero Jacobian of joint 1 must be a pure rotation about the world `z` axis
/// through the joint 1 origin, i.e. column 1 is `[0, 0, 0, 0, 0, 1]`.
#[test]
fn jacobian_rows_are_linear_first() {
    let fixture = load_fixture();
    let zero = fixture
        .samples
        .iter()
        .find(|s| s.kind == "zero")
        .expect("a zero-configuration sample");
    let case = &zero.cases[0];
    // Frame index 0 is Joint1; its first column starts at offset 0.
    let column = &case.zero_jacobian[0..6];
    let expected = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    for (i, (actual, want)) in column.iter().zip(expected).enumerate() {
        assert!(
            (actual - want).abs() < KINEMATIC_TOLERANCE,
            "row {i} of the joint 1 zero Jacobian is {actual}, expected {want}; \
             the fixture's row order is not linear-first: {column:?}"
        );
    }
}

#[test]
fn native_backend_matches_libfranka() {
    let fixture = load_fixture();
    let model = Model::from_urdf(&load_urdf()).expect("model builds");

    let gravity_earth = array3(&fixture.meta.gravity_earth);
    let gravity_earth_alt = array3(&fixture.meta.gravity_earth_alt);

    let mut pose_error = MaxError::default();
    let mut body_error = MaxError::default();
    let mut zero_error = MaxError::default();
    let mut mass_error = MaxError::default();
    let mut coriolis_error = MaxError::default();
    let mut gravity_error = MaxError::default();
    let mut gravity_alt_error = MaxError::default();

    for sample in &fixture.samples {
        let q = array7(&sample.q);
        let dq = array7(&sample.dq);

        for case in &sample.cases {
            let cfg = &fixture.load_configs[case.config];
            let f_t_ee = array16(&cfg.F_T_EE);
            let ee_t_k = array16(&cfg.EE_T_K);
            let i_total = array9(&cfg.I_total);
            let f_x_ctotal = array3(&cfg.F_x_Ctotal);

            for (f, frame) in Frame::ALL.iter().enumerate() {
                compare(
                    &mut pose_error,
                    &model.pose_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.pose[f * 16..(f + 1) * 16],
                    &format!("pose[{}]", fixture.meta.frames[f]),
                    sample.index,
                    case.config,
                );
                compare(
                    &mut body_error,
                    &model.body_jacobian_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.body_jacobian[f * 42..(f + 1) * 42],
                    &format!("body_jacobian[{}]", fixture.meta.frames[f]),
                    sample.index,
                    case.config,
                );
                compare(
                    &mut zero_error,
                    &model.zero_jacobian_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.zero_jacobian[f * 42..(f + 1) * 42],
                    &format!("zero_jacobian[{}]", fixture.meta.frames[f]),
                    sample.index,
                    case.config,
                );
            }

            compare(
                &mut mass_error,
                &model.mass_q(&q, &i_total, cfg.m_total, &f_x_ctotal),
                &case.mass,
                "mass",
                sample.index,
                case.config,
            );
            compare(
                &mut coriolis_error,
                &model.coriolis_q(&q, &dq, &i_total, cfg.m_total, &f_x_ctotal, &gravity_earth),
                &case.coriolis,
                "coriolis",
                sample.index,
                case.config,
            );
            compare(
                &mut gravity_error,
                &model.gravity_q(&q, cfg.m_total, &f_x_ctotal, &gravity_earth),
                &case.gravity,
                "gravity",
                sample.index,
                case.config,
            );
            compare(
                &mut gravity_alt_error,
                &model.gravity_q(&q, cfg.m_total, &f_x_ctotal, &gravity_earth_alt),
                &case.gravity_alt,
                "gravity (alternative g)",
                sample.index,
                case.config,
            );
        }
    }

    println!(
        "native backend max |error|: pose {:.3e}, body_jacobian {:.3e}, zero_jacobian {:.3e}, \
         mass {:.3e}, coriolis {:.3e}, gravity {:.3e}, gravity_alt {:.3e}",
        pose_error.value,
        body_error.value,
        zero_error.value,
        mass_error.value,
        coriolis_error.value,
        gravity_error.value,
        gravity_alt_error.value
    );

    let failures: Vec<String> = [
        pose_error.check("pose", KINEMATIC_TOLERANCE),
        body_error.check("body_jacobian", KINEMATIC_TOLERANCE),
        zero_error.check("zero_jacobian", KINEMATIC_TOLERANCE),
        mass_error.check("mass", DYNAMIC_TOLERANCE),
        coriolis_error.check("coriolis", DYNAMIC_TOLERANCE),
        gravity_error.check("gravity", DYNAMIC_TOLERANCE),
        gravity_alt_error.check("gravity_alt", DYNAMIC_TOLERANCE),
    ]
    .into_iter()
    .flatten()
    .collect();

    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The `&RobotState` overloads must forward exactly like `franka::Model` does:
/// `pose`/`body_jacobian`/`zero_jacobian` take `q`, `F_T_EE` and `EE_T_K` from
/// the state, `mass`/`coriolis` take the combined load, `coriolis` pins gravity
/// to `{0, 0, -9.81}` and `gravity` reads it from `O_ddP_O`.
#[test]
fn robot_state_overloads_forward_like_libfranka() {
    let fixture = load_fixture();
    let model = Model::from_urdf(&load_urdf()).expect("model builds");
    let gravity_earth_alt = array3(&fixture.meta.gravity_earth_alt);

    let sample = &fixture.samples[3];
    let case = sample
        .cases
        .iter()
        .find(|c| c.config == 2)
        .expect("the loaded configuration");
    let cfg = &fixture.load_configs[2];

    let state = RobotState {
        q: array7(&sample.q),
        dq: array7(&sample.dq),
        F_T_EE: array16(&cfg.F_T_EE),
        EE_T_K: array16(&cfg.EE_T_K),
        m_total: cfg.m_total,
        I_total: array9(&cfg.I_total),
        F_x_Ctotal: array3(&cfg.F_x_Ctotal),
        O_ddP_O: gravity_earth_alt,
        ..Default::default()
    };

    for (f, frame) in Frame::ALL.iter().enumerate() {
        let pose = model.pose(*frame, &state);
        for (i, value) in pose.iter().enumerate() {
            assert!(
                (value - case.pose[f * 16 + i]).abs() < KINEMATIC_TOLERANCE,
                "pose[{}][{i}]",
                fixture.meta.frames[f]
            );
        }
        let body = model.body_jacobian(*frame, &state);
        let zero = model.zero_jacobian(*frame, &state);
        for i in 0..42 {
            assert!((body[i] - case.body_jacobian[f * 42 + i]).abs() < KINEMATIC_TOLERANCE);
            assert!((zero[i] - case.zero_jacobian[f * 42 + i]).abs() < KINEMATIC_TOLERANCE);
        }
    }

    let mass = model.mass(&state);
    for (i, value) in mass.iter().enumerate() {
        assert!(
            (value - case.mass[i]).abs() < DYNAMIC_TOLERANCE,
            "mass[{i}]"
        );
    }
    let coriolis = model.coriolis(&state);
    for (i, value) in coriolis.iter().enumerate() {
        assert!(
            (value - case.coriolis[i]).abs() < DYNAMIC_TOLERANCE,
            "coriolis[{i}]"
        );
    }
    // `Model::gravity(&state)` must use `O_ddP_O`, i.e. the alternative gravity.
    let gravity = model.gravity(&state);
    for (i, value) in gravity.iter().enumerate() {
        assert!(
            (value - case.gravity_alt[i]).abs() < DYNAMIC_TOLERANCE,
            "gravity[{i}]"
        );
    }
}

/// The load configurations in the fixture must be the three the reference dump is
/// generated with, with `m_total` / `F_x_Ctotal` / `I_total` derived by libfranka's
/// own `combineCenterOfMass` / `combineInertiaTensor`.
#[test]
fn load_configurations_are_the_expected_ones() {
    let fixture = load_fixture();
    let names: Vec<&str> = fixture
        .load_configs
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(
        names,
        ["no_load", "hand_and_load", "hand_load_and_stiffness"]
    );
    assert_eq!(fixture.load_configs[0].m_total, 0.0);
    assert!((fixture.load_configs[1].m_total - 1.23).abs() < 1e-12);
    assert_eq!(fixture.load_configs[1].F_T_EE[14], 0.1034);
    assert_eq!(fixture.load_configs[2].EE_T_K[14], 0.05);
}
