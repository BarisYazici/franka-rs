//! Unit tests for [`Model`] against libfranka baselines at fixed configurations.

use super::*;

const FR3_URDF: &str = include_str!("../../tests/data/fr3.urdf");

fn model() -> Model {
    Model::from_urdf(FR3_URDF).expect("fr3.urdf loads")
}

/// `franka_test_utils::kEps` from libfranka's test suite.
const K_EPS: f64 = 1e-5;

const I_ZERO: [f64; 9] = [0.0; 9];
const IDENTITY_16: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
const F_X_C_ZERO: [f64; 3] = [0.0; 3];
const G_EARTH: [f64; 3] = [0.0, 0.0, -9.81];
// Franka end-effector mechanical properties, from robot_model_tests.cpp.
const EE_I_TOTAL: [f64; 9] = [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017];
const EE_F_X_CTOTAL: [f64; 3] = [0.01, 0.0, 0.03];
const EE_M_TOTAL: f64 = 0.73;

/// Port of `RobotModelTest.TestCoriolisWithGravity`.
#[test]
fn coriolis_at_rest_is_exactly_zero() {
    let m = model();
    let c = m.coriolis_q(&[0.0; 7], &[0.0; 7], &I_ZERO, 0.0, &F_X_C_ZERO, &G_EARTH);
    for value in c {
        assert_eq!(value, 0.0);
    }
}

/// Port of `RobotModelTest.TestGravity`.
#[test]
fn gravity_at_zero_configuration() {
    let m = model();
    let g = m.gravity_q(&[0.0; 7], 0.0, &F_X_C_ZERO, &G_EARTH);
    let expected = [0.0, -3.52387, 0.0, -3.44254, 0.0, 1.63362, -2.47128e-17];
    for (i, e) in expected.iter().enumerate() {
        assert!((g[i] - e).abs() < K_EPS, "joint {i}: {} vs {e}", g[i]);
    }
}

/// Port of `RobotModelTest.TestGravityWithAddedFrankaEndEffectorInertiaToLastLink`.
#[test]
fn gravity_at_zero_configuration_with_end_effector() {
    let m = model();
    let g = m.gravity_q(&[0.0; 7], EE_M_TOTAL, &EE_F_X_CTOTAL, &G_EARTH);
    let expected = [0.0, -4.22568, 0.0, -3.33154, 0.0, 2.33543, -8.83179e-17];
    for (i, e) in expected.iter().enumerate() {
        assert!((g[i] - e).abs() < K_EPS, "joint {i}: {} vs {e}", g[i]);
    }
}

/// Port of `RobotModelTest.TestMass`.
#[test]
fn mass_at_zero_configuration() {
    let m = model();
    let mass = m.mass_q(&[0.0; 7], &I_ZERO, 0.0, &F_X_C_ZERO);
    #[rustfmt::skip]
    let expected: [f64; 49] = [
        0.132725, -0.0414226, 0.10001, 0.0120435, 0.0446215, -0.0022471, -0.000369021,
        -0.0414226, 2.73892, -0.0437247, -1.15444, -0.0372567, 0.0137625, -0.00396178,
        0.10001, -0.0437247, 0.10001, 0.0120435, 0.0446215, -0.0022471, -0.000369021,
        0.0120435, -1.15444, 0.0120435, 0.644215, 0.0116149, -0.00714566, 0.00225227,
        0.0446215, -0.0372567, 0.0446215, 0.0116149, 0.0446215, -0.0022471, -0.000369021,
        -0.0022471, 0.0137625, -0.0022471, -0.00714566, -0.0022471, 0.0313282, 0.000174882,
        -0.000369021, -0.00396178, -0.000369021, 0.00225227, -0.000369021, 0.000174882,
        0.000119426,
    ];
    for (i, e) in expected.iter().enumerate() {
        assert!((mass[i] - e).abs() < K_EPS, "entry {i}: {} vs {e}", mass[i]);
    }
}

/// Port of `RobotModelTest.TestMassWithAddedFrankaEndEffectorInertiaToLastLink`.
#[test]
fn mass_at_zero_configuration_with_end_effector() {
    let m = model();
    let mass = m.mass_q(&[0.0; 7], &EE_I_TOTAL, EE_M_TOTAL, &EE_F_X_CTOTAL);
    #[rustfmt::skip]
    let expected: [f64; 49] = [
        0.141436, -0.0414226, 0.108721, 0.0120435, 0.0533324, -0.0022471, -0.00278442,
        -0.0414226, 2.97982, -0.0437247, -1.25956, -0.0372567, 0.0605572, -0.00396178,
        0.108721, -0.0437247, 0.108721, 0.0120435, 0.0533324, -0.0022471, -0.00278442,
        0.0120435, -1.25956, 0.0120435, 0.691427, 0.0116149, -0.0282393, 0.00225227,
        0.0533324, -0.0372567, 0.0533324, 0.0116149, 0.0533324, -0.0022471, -0.00278442,
        -0.0022471, 0.0605572, -0.0022471, -0.0282393, -0.0022471, 0.0545405, 0.000174882,
        -0.00278442, -0.00396178, -0.00278442, 0.00225227, -0.00278442, 0.000174882,
        0.00189243,
    ];
    for (i, e) in expected.iter().enumerate() {
        assert!((mass[i] - e).abs() < K_EPS, "entry {i}: {} vs {e}", mass[i]);
    }
}

/// Port of `RobotModelTest.TestCoriolisWithAddedFrankaEndEffectorInertiaToLastLink`.
#[test]
fn coriolis_with_unit_velocity_and_end_effector() {
    let m = model();
    let c = m.coriolis_q(
        &[0.0; 7],
        &[1.0; 7],
        &EE_I_TOTAL,
        EE_M_TOTAL,
        &EE_F_X_CTOTAL,
        &G_EARTH,
    );
    let expected = [
        0.25211,
        -1.97502,
        0.254311,
        0.987115,
        0.122285,
        -0.256464,
        -0.000634562,
    ];
    for (i, e) in expected.iter().enumerate() {
        assert!((c[i] - e).abs() < K_EPS, "joint {i}: {} vs {e}", c[i]);
    }
}

/// Port of `RobotModelTest.givenNoLoad_whenComputingGravity_thenCloseToBaseline`.
#[test]
fn gravity_at_default_configuration_close_to_baseline() {
    const DELTA_GRAVITY: f64 = 0.2;
    let m = model();
    let q = [
        0.0,
        0.0,
        0.0,
        -0.75 * std::f64::consts::PI,
        0.0,
        0.75 * std::f64::consts::PI,
        0.0,
    ];
    let g = m.gravity_q(&q, 0.0, &F_X_C_ZERO, &G_EARTH);
    let expected = [0.0, -24.5858, 0.0, 17.6880, 0.5095, 1.6428, 0.0];
    for (i, e) in expected.iter().enumerate() {
        assert!(
            (g[i] - e).abs() < DELTA_GRAVITY,
            "joint {i}: {} vs {e}",
            g[i]
        );
    }
}

/// Port of `RobotModelTest.givenLoad_whenComputingGravity_thenCloseToBaseline`.
#[test]
fn gravity_at_moved_configuration_with_load_close_to_baseline() {
    const DELTA_GRAVITY: f64 = 0.2;
    let m = model();
    let q = [0.0010, 0.0010, 0.0010, -2.3552, 0.0010, 2.3572, 0.0010];
    let g = m.gravity_q(&q, EE_M_TOTAL, &EE_F_X_CTOTAL, &G_EARTH);
    let expected = [0.0, -28.2417, 0.0, 20.7531, 0.5095, 2.3446, 0.0];
    for (i, e) in expected.iter().enumerate() {
        assert!(
            (g[i] - e).abs() < DELTA_GRAVITY,
            "joint {i}: {} vs {e}",
            g[i]
        );
    }
}

/// Port of
/// `RobotModelTest.givenNonZeroJointVelocity_whenComputingCoriolis_thenNonZeroCoriolis`.
#[test]
fn coriolis_at_default_configuration_close_to_baseline() {
    const DELTA_CORIOLIS: f64 = 0.2;
    let m = model();
    let q = [
        0.0,
        0.0,
        0.0,
        -0.75 * std::f64::consts::PI,
        0.0,
        0.75 * std::f64::consts::PI,
        0.0,
    ];
    let c = m.coriolis_q(&q, &[1.0; 7], &I_ZERO, 0.0, &F_X_C_ZERO, &G_EARTH);
    let expected = [2.4562, -0.8199, 2.4532, -2.3450, -0.1492, -0.1801, 0.0164];
    for (i, e) in expected.iter().enumerate() {
        assert!(
            (c[i] - e).abs() < DELTA_CORIOLIS,
            "joint {i}: {} vs {e}",
            c[i]
        );
    }
}

/// Port of `RobotModelTest.givenWithLoad_whenComputingInertia_thenCloseToBaseline`.
#[test]
fn mass_at_moved_configuration_with_load_close_to_baseline() {
    const DELTA_INERTIA: f64 = 0.1;
    let m = model();
    let q = [0.0010, 0.0010, 0.0010, -2.3552, 0.0010, 2.3572, 0.0010];
    let mass = m.mass_q(&q, &EE_I_TOTAL, EE_M_TOTAL, &EE_F_X_CTOTAL);
    #[rustfmt::skip]
    let expected: [f64; 49] = [
        1.1749, -0.0085, 1.1660, -0.0247, -0.0261, -0.0033, -0.0057,
        -0.0085, 1.7281, 0.0046, -0.8304, -0.0142, -0.1209, -0.0005,
        1.1660, 0.0046, 1.1660, -0.0247, -0.0261, -0.0033, -0.0057,
        -0.0247, -0.8304, -0.0247, 1.0399, 0.0263, 0.1526, -0.0023,
        -0.0261, -0.0142, -0.0261, 0.0263, 0.0112, 0.0042, 0.0012,
        -0.0033, -0.1209, -0.0033, 0.1526, 0.0042, 0.0390, -0.0004,
        -0.0057, -0.0005, -0.0057, -0.0023, 0.0012, -0.0004, 0.0019,
    ];
    for (i, e) in expected.iter().enumerate() {
        assert!(
            (mass[i] - e).abs() < DELTA_INERTIA,
            "entry {i}: {} vs {e}",
            mass[i]
        );
    }
}

/// Port of `RobotModelTest.givenNoLoad_whenComputingInertia_thenCloseToBaseline`.
#[test]
fn mass_at_default_configuration_close_to_baseline() {
    const DELTA_INERTIA: f64 = 0.1;
    let m = model();
    let q = [
        0.0,
        0.0,
        0.0,
        -0.75 * std::f64::consts::PI,
        0.0,
        0.75 * std::f64::consts::PI,
        0.0,
    ];
    let mass = m.mass_q(&q, &I_ZERO, 0.0, &F_X_C_ZERO);
    #[rustfmt::skip]
    let expected: [f64; 49] = [
        0.9831, -0.0078, 0.9742, -0.0254, -0.0351, -0.0040, -0.0004,
        -0.0078, 1.5390, 0.0053, -0.6641, -0.0147, -0.0828, -0.0012,
        0.9742, 0.0053, 0.9742, -0.0254, -0.0351, -0.0040, -0.0004,
        -0.0254, -0.6641, -0.0254, 0.8185, 0.0268, 0.0888, -0.0016,
        -0.0351, -0.0147, -0.0351, 0.0268, 0.0114, 0.0047, 0.0002,
        -0.0040, -0.0828, -0.0040, 0.0888, 0.0047, 0.0202, 0.0003,
        -0.0004, -0.0012, -0.0004, -0.0016, 0.0002, 0.0003, 0.0002,
    ];
    for (i, e) in expected.iter().enumerate() {
        assert!(
            (mass[i] - e).abs() < DELTA_INERTIA,
            "entry {i}: {} vs {e}",
            mass[i]
        );
    }
}

/// Port of
/// `RobotModelTest.givenNonZeroJointVelocityWithLoad_whenComputingCoriolis_thenCloseToBaseline`.
#[test]
fn coriolis_at_moved_configuration_with_load_close_to_baseline() {
    const DELTA_CORIOLIS: f64 = 0.2;
    let m = model();
    let q = [0.0010, 0.0010, 0.0010, -2.3552, 0.0010, 2.3572, 0.0010];
    let c = m.coriolis_q(
        &q,
        &[1.0; 7],
        &EE_I_TOTAL,
        EE_M_TOTAL,
        &EE_F_X_CTOTAL,
        &G_EARTH,
    );
    let expected = [3.1468, -0.6225, 3.1439, -3.1063, -0.0658, -0.5416, 0.0089];
    for (i, e) in expected.iter().enumerate() {
        assert!(
            (c[i] - e).abs() < DELTA_CORIOLIS,
            "joint {i}: {} vs {e}",
            c[i]
        );
    }
}

#[test]
fn mass_matrix_is_symmetric_and_positive_definite_diagonal() {
    let m = model();
    let q = [0.1, -0.6, 0.3, -1.9, 0.2, 1.7, -0.4];
    let mass = m.mass_q(&q, &I_ZERO, 0.0, &F_X_C_ZERO);
    for i in 0..7 {
        assert!(mass[i * 7 + i] > 0.0);
        for j in 0..7 {
            assert_eq!(mass[i * 7 + j], mass[j * 7 + i]);
        }
    }
}

#[test]
fn identity_end_effector_frames_coincide_with_flange() {
    let m = model();
    let q = [0.1, -0.6, 0.3, -1.9, 0.2, 1.7, -0.4];
    let identity = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    let flange = m.pose_q(Frame::Flange, &q, &identity, &identity);
    let ee = m.pose_q(Frame::EndEffector, &q, &identity, &identity);
    let k = m.pose_q(Frame::Stiffness, &q, &identity, &identity);
    for i in 0..16 {
        assert!((flange[i] - ee[i]).abs() < 1e-15);
        assert!((flange[i] - k[i]).abs() < 1e-15);
    }
}

#[test]
fn the_built_in_fer_model_parses_and_answers() {
    let m = Model::native_fer();
    let q = [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.7];
    let g = m.gravity_q(&q, 0.0, &F_X_C_ZERO, &G_EARTH);
    assert!(g.iter().all(|value| value.is_finite()));
    // The arm hangs out of the vertical, so the pitch joints carry weight.
    assert!(g[1].abs() > 1.0, "gravity {g:?}");
    let mass = m.mass_q(&q, &I_ZERO, 0.0, &F_X_C_ZERO);
    for i in 0..7 {
        assert!(
            mass[i * 7 + i] > 0.0,
            "mass diagonal {i}: {}",
            mass[i * 7 + i]
        );
    }
    // `Frame::Flange` is 0.107 m along joint 7's own z axis on an FER.
    let joint7 = m.pose_q(Frame::Joint7, &q, &IDENTITY_16, &IDENTITY_16);
    let flange = m.pose_q(Frame::Flange, &q, &IDENTITY_16, &IDENTITY_16);
    let offset = [
        flange[12] - joint7[12],
        flange[13] - joint7[13],
        flange[14] - joint7[14],
    ];
    let along_z = offset[0] * joint7[8] + offset[1] * joint7[9] + offset[2] * joint7[10];
    assert!((along_z - 0.107).abs() < 1e-12, "flange offset {along_z}");
}

#[test]
fn urdf_without_link8_is_rejected() {
    let err =
        Model::from_urdf("<robot name=\"x\"><link name=\"base\"/></robot>").expect_err("must fail");
    assert!(format!("{err}").contains("link8"), "{err}");
}
