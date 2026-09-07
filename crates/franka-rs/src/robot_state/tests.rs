//! Unit tests for the wire-to-`RobotState` conversion of both FCI versions.

use super::*;
use crate::wire::f64s_to_wire;
use zerocopy::little_endian::{F32, U64};

fn f32s<const N: usize>(values: [f64; N]) -> [F32; N] {
    let mut out = [F32::new(0.0); N];
    for (o, v) in out.iter_mut().zip(values.iter()) {
        *o = F32::new(*v as f32);
    }
    out
}

/// Expected values from libfranka's own `CalculationTest.CombineCenterOfMassEEWithLoad` and
/// `CalculationTest.CombineInertiaTensorEEWithLoad` (`test/calculations_tests.cpp`).
#[test]
// Keeps libfranka's literals verbatim.
#[allow(clippy::excessive_precision)]
fn load_combination_matches_libfranka_reference_values() {
    let m_ee = 0.73;
    let F_x_Cee = [-0.01, 0.0, -0.03];
    let I_ee = [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017];
    let m_load = 0.5;
    let F_x_Cload = [0.01, -0.2, 0.03];
    let I_load = [0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3];

    let F_x_Ctotal = combine_center_of_mass(m_ee, &F_x_Cee, m_load, &F_x_Cload);
    let expected_com = [
        -0.001_869_918_699_186_99,
        -0.081_300_813_008_130_09,
        -0.005_609_756_097_560_98,
    ];
    for (a, b) in F_x_Ctotal.iter().zip(expected_com.iter()) {
        assert!((a - b).abs() < 1e-15, "{a} != {b}");
    }

    let I_total = combine_inertia_tensor(
        m_ee,
        &F_x_Cee,
        &I_ee,
        m_load,
        &F_x_Cload,
        &I_load,
        m_ee + m_load,
        &F_x_Ctotal,
    );
    let expected = [
        1.493_821_138_211_38e-2,
        1.186_991_869_918_70e-3,
        -3.560_975_609_756_10e-4,
        1.186_991_869_918_70e-3,
        // libfranka's own literal here is 2.36869918699187e-2
        // (test/calculations_tests.cpp:123), but its test only compares the first three
        // elements (`for (int i = 0; i < 3; i++)`), so that value was never exercised and
        // is off by exactly 0.005. The parallel-axis result is 2.868699186991870e-2.
        2.868_699_186_991_87e-2,
        3.560_975_609_756_10e-3,
        -3.560_975_609_756_10e-4,
        3.560_975_609_756_10e-3,
        3.136_886_178_861_79e-1,
    ];
    for (a, b) in I_total.iter().zip(expected.iter()) {
        assert!((a - b).abs() < 1e-14, "{a} != {b}");
    }
}

/// `robot_impl.cpp:516` is `converted.m_total = robot_state.m_ee + robot_state.m_load;`
/// over the two wire `float`s: the addition happens in `float` and only the result is
/// widened. With a load small enough to be swallowed by the end effector's mantissa the
/// two orders of operation give different doubles, and only the C++ one is correct here.
#[test]
fn m_total_is_the_f32_sum_widened_not_the_sum_of_the_widened_values() {
    let wire = crate::wire::robot::RobotState {
        m_ee: F32::new(1.0),
        m_load: F32::new(1e-8),
        ..Default::default()
    };
    let state = RobotState::from_wire(&wire);

    assert_eq!(state.m_total, f64::from(1.0f32 + 1e-8f32));
    assert_eq!(state.m_total, 1.0);
    assert_ne!(state.m_total, state.m_ee + state.m_load);
}

#[test]
fn zero_mass_gives_zero_load_properties() {
    assert_eq!(
        combine_center_of_mass(0.0, &[1.0; 3], 0.0, &[2.0; 3]),
        [0.0; 3]
    );
    assert_eq!(
        combine_inertia_tensor(
            0.0, &[1.0; 3], &[1.0; 9], 0.0, &[2.0; 3], &[2.0; 9], 0.0, &[0.0; 3]
        ),
        [0.0; 9]
    );
}

#[test]
#[allow(clippy::field_reassign_with_default, clippy::excessive_precision)]
fn from_wire_copies_every_field() {
    use zerocopy::{FromBytes, IntoBytes};

    let mut wire = crate::wire::robot::RobotState::default();
    wire.message_id = U64::new(1234);
    wire.q = f32s([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]);
    wire.dq = f32s([-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]);
    wire.O_T_EE = f32s(crate::robot_state::IDENTITY_TRANSFORM);
    wire.m_ee = F32::new(0.73);
    wire.F_x_Cee = f32s([-0.01, 0.0, -0.03]);
    wire.I_ee = f32s([0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017]);
    wire.m_load = F32::new(0.5);
    wire.F_x_Cload = f32s([0.01, -0.2, 0.03]);
    wire.I_load = f32s([0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3]);
    wire.accelerometer_top[2] = f32s([1.0, 2.0, 3.0]);
    wire.robot_mode = crate::wire::robot::RobotMode::Reflex.to_u8();
    wire.control_command_success_rate = F32::new(0.5);
    wire.errors[3] = 1;
    wire.reflex_reason[25] = 1;

    // Round-trip through raw bytes to prove the conversion reads the packed layout.
    let bytes = wire.as_bytes().to_vec();
    assert_eq!(bytes.len(), 1377);
    let parsed = crate::wire::robot::RobotState::read_from_bytes(&bytes).unwrap();
    let state = RobotState::from_wire(&parsed);

    assert_eq!(state.time, Duration::from_millis(1234));
    assert!((state.q[6] - 0.7).abs() < 1e-6);
    assert!((state.dq[0] + 0.1).abs() < 1e-6);
    assert_eq!(state.O_T_EE, IDENTITY_TRANSFORM);
    assert_eq!(state.accelerometer_top[2], [1.0, 2.0, 3.0]);
    assert_eq!(state.robot_mode, RobotMode::Reflex);
    assert!((state.control_command_success_rate - 0.5).abs() < 1e-12);
    assert!(state.current_errors.get("joint_velocity_violation"));
    assert!(state
        .last_motion_errors
        .get("communication_constraints_violation"));
    assert!(!state
        .current_errors
        .get("communication_constraints_violation"));

    // m_total / F_x_Ctotal / I_total are combined exactly like convertRobotState does; the
    // f32 wire round-trip costs about six digits, hence the loose tolerance.
    //
    // `m_total` in particular is the *f32* sum widened, not the sum of the two widened
    // doubles (`robot_impl.cpp:516` adds two `float`s) -- with these inputs the two differ
    // in the 8th digit, which this asserts exactly.
    assert_eq!(state.m_total, f64::from(0.73f32 + 0.5f32));
    assert!((state.m_total - 1.23).abs() < 1e-6);
    assert!((state.I_total[0] - 1.493_821_138_211_38e-2).abs() < 1e-8);
    assert!((state.I_total[8] - 3.136_886_178_861_79e-1).abs() < 1e-7);
    assert!((state.F_x_Ctotal[1] + 0.081_300_813_008_130_09).abs() < 1e-7);

    // f64s_to_wire is the inverse helper used by the command path.
    assert_eq!(f64s_to_wire(&[1.0f64; 2])[0].get(), 1.0);
}

#[test]
#[allow(clippy::field_reassign_with_default, clippy::excessive_precision)]
fn from_wire_v5_copies_every_field() {
    use zerocopy::{FromBytes, IntoBytes};

    let mut wire = crate::wire::robot::v5::RobotState::default();
    wire.message_id = U64::new(4321);
    wire.q = f64s_to_wire(&[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]);
    wire.dq = f64s_to_wire(&[-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]);
    wire.O_T_EE = f64s_to_wire(&crate::robot_state::IDENTITY_TRANSFORM);
    wire.EE_T_K = f64s_to_wire(&crate::robot_state::IDENTITY_TRANSFORM);
    wire.m_ee = zerocopy::little_endian::F64::new(0.73);
    wire.F_x_Cee = f64s_to_wire(&[-0.01, 0.0, -0.03]);
    wire.I_ee = f64s_to_wire(&[0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017]);
    wire.m_load = zerocopy::little_endian::F64::new(0.5);
    wire.F_x_Cload = f64s_to_wire(&[0.01, -0.2, 0.03]);
    wire.I_load = f64s_to_wire(&[0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3]);
    wire.robot_mode = crate::wire::robot::v5::RobotMode::Reflex.to_u8();
    wire.control_command_success_rate = zerocopy::little_endian::F64::new(0.5);
    wire.errors[3] = 1;
    wire.reflex_reason[25] = 1;

    // Round-trip through raw bytes to prove the conversion reads the packed layout.
    let bytes = wire.as_bytes().to_vec();
    assert_eq!(bytes.len(), 2373);
    let parsed = crate::wire::robot::v5::RobotState::read_from_bytes(&bytes).unwrap();
    let state = RobotState::from_wire_v5(&parsed);

    assert_eq!(state.time, Duration::from_millis(4321));
    assert_eq!(state.q, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]);
    assert_eq!(state.dq, [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]);
    assert_eq!(state.O_T_EE, IDENTITY_TRANSFORM);
    assert_eq!(state.EE_T_K, IDENTITY_TRANSFORM);
    assert_eq!(state.robot_mode, RobotMode::Reflex);
    assert_eq!(state.control_command_success_rate, 0.5);
    assert!(state.current_errors.get("joint_velocity_violation"));
    assert!(state
        .last_motion_errors
        .get("communication_constraints_violation"));

    // FCI v5 carries no accelerometers.
    assert_eq!(state.accelerometer_top, [[0.0; 3]; 6]);
    assert_eq!(state.accelerometer_bottom, [[0.0; 3]; 6]);

    // The load combination is the same as on v10, but exact here: the wire is f64.
    assert_eq!(state.m_total, 1.23);
    assert!((state.I_total[0] - 1.493_821_138_211_38e-2).abs() < 1e-14);
    assert!((state.I_total[8] - 3.136_886_178_861_79e-1).abs() < 1e-14);
    assert!((state.F_x_Ctotal[1] + 0.081_300_813_008_130_09).abs() < 1e-15);
}
