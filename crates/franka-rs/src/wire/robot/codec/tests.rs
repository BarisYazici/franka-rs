//! Unit tests for the version seam.

use super::*;
use crate::error::{FrankaError, MoveStatus};
use crate::wire::robot::{v10, v5};
use zerocopy::{FromBytes, IntoBytes};

#[test]
fn connect_version_and_sizes_per_version() {
    assert_eq!(connect_version(FciVersion::V5), 5);
    assert_eq!(connect_version(FciVersion::V10), 10);

    assert_eq!(state_size(FciVersion::V5), 2373);
    assert_eq!(state_size(FciVersion::V10), 1377);
    assert_eq!(command_size(FciVersion::V5), 370);
    assert_eq!(command_size(FciVersion::V10), 371);

    assert_eq!(MOVE_REQUEST_MAX_LEN, 113);
    assert_eq!(ROBOT_COMMAND_MAX_LEN, 371);
}

#[test]
fn command_ids_follow_the_version_numbering() {
    use CommandKind::*;

    // The two numberings agree only on the first three commands.
    for kind in [Connect, Move, StopMove] {
        assert_eq!(
            command_id(FciVersion::V5, kind),
            command_id(FciVersion::V10, kind)
        );
    }
    assert_eq!(command_id(FciVersion::V5, Connect), Some(0));
    assert_eq!(command_id(FciVersion::V5, Move), Some(1));
    assert_eq!(command_id(FciVersion::V5, StopMove), Some(2));

    // v5: GetCartesianLimit at 3 shifts every setter by one, SetFilters is 11.
    assert_eq!(command_id(FciVersion::V5, GetCartesianLimit), Some(3));
    assert_eq!(command_id(FciVersion::V5, SetCollisionBehavior), Some(4));
    assert_eq!(command_id(FciVersion::V5, SetJointImpedance), Some(5));
    assert_eq!(command_id(FciVersion::V5, SetCartesianImpedance), Some(6));
    assert_eq!(command_id(FciVersion::V5, SetGuidingMode), Some(7));
    assert_eq!(command_id(FciVersion::V5, SetEEToK), Some(8));
    assert_eq!(command_id(FciVersion::V5, SetNEToEE), Some(9));
    assert_eq!(command_id(FciVersion::V5, SetLoad), Some(10));
    assert_eq!(command_id(FciVersion::V5, SetFilters), Some(11));
    assert_eq!(command_id(FciVersion::V5, AutomaticErrorRecovery), Some(12));
    assert_eq!(command_id(FciVersion::V5, LoadModelLibrary), Some(13));
    assert_eq!(command_id(FciVersion::V5, GetRobotModel), None);

    assert_eq!(command_id(FciVersion::V10, SetCollisionBehavior), Some(3));
    assert_eq!(command_id(FciVersion::V10, SetLoad), Some(9));
    assert_eq!(
        command_id(FciVersion::V10, AutomaticErrorRecovery),
        Some(10)
    );
    assert_eq!(command_id(FciVersion::V10, GetRobotModel), Some(11));
    assert_eq!(command_id(FciVersion::V10, GetCartesianLimit), None);
    assert_eq!(command_id(FciVersion::V10, SetFilters), None);
    assert_eq!(command_id(FciVersion::V10, LoadModelLibrary), None);
}

#[test]
fn move_request_v5_is_56_bytes_without_the_async_tail() {
    let mut out = [0xAAu8; MOVE_REQUEST_MAX_LEN];
    let len = move_request(
        FciVersion::V5,
        v10::MoveControllerMode::ExternalController,
        v10::MoveMotionGeneratorMode::JointVelocity,
        v10::Deviation::new(1.0, 2.0, 3.0),
        v10::Deviation::new(4.0, 5.0, 6.0),
        &mut out,
    )
    .unwrap();

    assert_eq!(len, 56);
    assert_eq!(&out[0..4], &2u32.to_le_bytes());
    assert_eq!(&out[4..8], &1u32.to_le_bytes());
    assert_eq!(&out[8..16], &1.0f64.to_le_bytes());
    assert_eq!(&out[16..24], &2.0f64.to_le_bytes());
    assert_eq!(&out[24..32], &3.0f64.to_le_bytes());
    assert_eq!(&out[32..40], &4.0f64.to_le_bytes());
    assert_eq!(&out[48..56], &6.0f64.to_le_bytes());
    // Nothing past the 56th byte was touched.
    assert!(out[56..].iter().all(|&b| b == 0xAA));
}

#[test]
fn move_request_v10_is_113_bytes_and_matches_the_struct() {
    let mut out = [0xAAu8; MOVE_REQUEST_MAX_LEN];
    let len = move_request(
        FciVersion::V10,
        v10::MoveControllerMode::JointImpedance,
        v10::MoveMotionGeneratorMode::None,
        v10::Deviation::new(1.0, 2.0, 3.0),
        v10::Deviation::new(4.0, 5.0, 6.0),
        &mut out,
    )
    .unwrap();

    assert_eq!(len, 113);
    let expected = v10::MoveRequest::new(
        v10::MoveControllerMode::JointImpedance,
        v10::MoveMotionGeneratorMode::None,
        v10::Deviation::new(1.0, 2.0, 3.0),
        v10::Deviation::new(4.0, 5.0, 6.0),
    );
    assert_eq!(&out[..], expected.as_bytes());
}

#[test]
fn move_request_v5_rejects_the_none_motion_generator_mode() {
    let mut out = [0u8; MOVE_REQUEST_MAX_LEN];
    let error = move_request(
        FciVersion::V5,
        v10::MoveControllerMode::ExternalController,
        v10::MoveMotionGeneratorMode::None,
        v10::Deviation::new(1.0, 2.0, 3.0),
        v10::Deviation::new(4.0, 5.0, 6.0),
        &mut out,
    )
    .unwrap_err();

    assert!(matches!(error, FrankaError::InvalidArgument(_)), "{error}");
    assert!(error.to_string().contains("FCI version 5"), "{error}");
    // Nothing was written.
    assert!(out.iter().all(|&b| b == 0));
}

fn sample_command() -> RobotCommandData {
    RobotCommandData {
        message_id: 42,
        q_c: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        dq_c: [0.0; 7],
        O_T_EE_c: [0.5; 16],
        O_dP_EE_c: [0.25; 6],
        elbow_c: [-1.0, 1.0],
        valid_elbow: true,
        motion_generation_finished: true,
        tau_J_d: [11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0],
        torque_command_finished: true,
    }
}

#[test]
fn robot_command_v5_writes_370_bytes_with_tau_j_d_at_314() {
    let mut out = [0xAAu8; ROBOT_COMMAND_MAX_LEN];
    let len = robot_command(FciVersion::V5, &sample_command(), &mut out);

    assert_eq!(len, 370);
    assert_eq!(&out[0..8], &42u64.to_le_bytes());
    assert_eq!(&out[8..16], &1.0f64.to_le_bytes());
    // MotionGeneratorCommand is 306 bytes; its two flags are its last two.
    assert_eq!(out[8 + 304], 1); // valid_elbow
    assert_eq!(out[8 + 305], 1); // motion_generation_finished
                                 // ControllerCommand starts right after, and on v5 it is only tau_J_d.
    assert_eq!(&out[314..322], &11.0f64.to_le_bytes());
    assert_eq!(&out[362..370], &17.0f64.to_le_bytes());
    // No torque_command_finished byte: byte 370 was never written.
    assert_eq!(out[370], 0xAA);
}

#[test]
fn robot_command_v10_writes_371_bytes_and_keeps_the_torque_flag() {
    let mut out = [0xAAu8; ROBOT_COMMAND_MAX_LEN];
    let len = robot_command(FciVersion::V10, &sample_command(), &mut out);

    assert_eq!(len, 371);
    assert_eq!(&out[0..8], &42u64.to_le_bytes());
    assert_eq!(&out[314..322], &11.0f64.to_le_bytes());
    assert_eq!(out[370], 1); // torque_command_finished
}

#[test]
fn robot_command_roundtrips_through_the_wire_structs() {
    let data = sample_command();

    let mut out = [0u8; ROBOT_COMMAND_MAX_LEN];
    let len = robot_command(FciVersion::V5, &data, &mut out);
    let parsed = v5::RobotCommand::read_from_bytes(&out[..len]).unwrap();
    assert_eq!(parsed.message_id.get(), 42);
    assert_eq!(crate::wire::f64s_to_f64(&parsed.motion.q_c), data.q_c);
    assert_eq!(
        crate::wire::f64s_to_f64(&parsed.control.tau_J_d),
        data.tau_J_d
    );
    assert_eq!(parsed.motion.valid_elbow, 1);

    let mut out = [0u8; ROBOT_COMMAND_MAX_LEN];
    let len = robot_command(FciVersion::V10, &data, &mut out);
    let parsed = v10::RobotCommand::read_from_bytes(&out[..len]).unwrap();
    assert_eq!(
        crate::wire::f64s_to_f64(&parsed.control.tau_J_d),
        data.tau_J_d
    );
    assert_eq!(parsed.control.torque_command_finished, 1);
}

#[test]
fn parse_robot_state_accepts_only_the_exact_size_of_its_version() {
    let mut wire = v5::RobotState::default();
    wire.message_id = zerocopy::little_endian::U64::new(7);
    wire.q = crate::wire::f64s_to_wire(&[0.5; 7]);
    let bytes = wire.as_bytes();
    assert_eq!(bytes.len(), 2373);

    let state = parse_robot_state(FciVersion::V5, bytes).unwrap();
    assert_eq!(state.q, [0.5; 7]);
    assert_eq!(state.time, crate::duration::Duration::from_millis(7));

    // The v10 parser rejects a v5 datagram and vice versa, with libfranka's UDP text.
    let error = parse_robot_state(FciVersion::V10, bytes).unwrap_err();
    assert_eq!(error.to_string(), "libfranka: incorrect object size");
    let error = parse_robot_state(FciVersion::V5, &bytes[..2372]).unwrap_err();
    assert_eq!(error.to_string(), "libfranka: incorrect object size");

    let v10_wire = v10::RobotState::default();
    let v10_bytes = v10_wire.as_bytes();
    assert_eq!(v10_bytes.len(), 1377);
    assert!(parse_robot_state(FciVersion::V10, v10_bytes).is_ok());
    assert!(parse_robot_state(FciVersion::V5, v10_bytes).is_err());
}

#[test]
fn move_statuses_map_by_name_on_both_versions() {
    // v5 (service_types.h:137-148) drops the two safety-function values of v10, so every
    // value from kCommandNotPossibleRejected on is shifted down by two.
    let v5_expected = [
        (0u8, MoveStatus::Success),
        (1, MoveStatus::MotionStarted),
        (2, MoveStatus::Preempted),
        (3, MoveStatus::CommandNotPossibleRejected),
        (4, MoveStatus::StartAtSingularPoseRejected),
        (5, MoveStatus::InvalidArgumentRejected),
        (6, MoveStatus::ReflexAborted),
        (7, MoveStatus::EmergencyAborted),
        (8, MoveStatus::InputErrorAborted),
        (9, MoveStatus::Aborted),
    ];
    for (byte, expected) in v5_expected {
        assert_eq!(
            parse_move_status(FciVersion::V5, byte, CommandKind::Move.name()).unwrap(),
            expected
        );
    }
    let error = parse_move_status(FciVersion::V5, 10, CommandKind::Move.name()).unwrap_err();
    assert!(matches!(error, FrankaError::Protocol(_)));
    assert_eq!(
        error.to_string(),
        "libfranka: Unexpected response while handling Move command!"
    );

    for byte in 0..=11u8 {
        assert_eq!(
            parse_move_status(FciVersion::V10, byte, CommandKind::Move.name()).unwrap(),
            MoveStatus::from_u8(byte).unwrap()
        );
    }
    assert!(parse_move_status(FciVersion::V10, 12, CommandKind::Move.name()).is_err());
}

/// The unknown-byte text is libfranka's `handleCommandResponse` string verbatim, with the
/// name of the *command* whose response was parsed — not of the status family. Every
/// parser takes the name, so a getter/setter byte reports `Set Load`, not `Getter/Setter`.
#[test]
fn unknown_status_text_names_the_command_verbatim() {
    let cases: Vec<(FrankaError, &str)> = vec![
        (
            parse_move_status(FciVersion::V5, 200, CommandKind::Move.name()).unwrap_err(),
            "libfranka: Unexpected response while handling Move command!",
        ),
        (
            parse_stop_move_status(FciVersion::V5, 200, CommandKind::StopMove.name()).unwrap_err(),
            "libfranka: Unexpected response while handling Stop Move command!",
        ),
        (
            parse_automatic_error_recovery_status(
                FciVersion::V10,
                200,
                CommandKind::AutomaticErrorRecovery.name(),
            )
            .unwrap_err(),
            "libfranka: Unexpected response while handling Automatic Error Recovery command!",
        ),
        (
            parse_getter_setter_status(FciVersion::V5, 3, CommandKind::SetLoad.name()).unwrap_err(),
            "libfranka: Unexpected response while handling Set Load command!",
        ),
        (
            parse_getter_setter_status(FciVersion::V5, 3, CommandKind::SetFilters.name())
                .unwrap_err(),
            "libfranka: Unexpected response while handling Set Filters command!",
        ),
        (
            parse_getter_setter_status(FciVersion::V5, 3, CommandKind::GetCartesianLimit.name())
                .unwrap_err(),
            "libfranka: Unexpected response while handling Get Cartesian Limit command!",
        ),
        (
            parse_command_status(FciVersion::V5, 2, CommandKind::GetRobotModel.name()).unwrap_err(),
            "libfranka: Unexpected response while handling Get Robot Model command!",
        ),
    ];
    for (error, expected) in cases {
        assert!(matches!(error, FrankaError::Protocol(_)), "{error}");
        assert_eq!(error.to_string(), expected);
    }
}

/// The names are `research_interface::robot::CommandTraits<T>::kName`
/// (`service_traits.h`), which is what `robot::commands::CommandName::as_str` already
/// prints on FCI v10.
#[test]
fn command_kind_names_match_the_libfranka_traits() {
    use CommandKind::*;

    assert_eq!(Move.name(), "Move");
    assert_eq!(StopMove.name(), "Stop Move");
    assert_eq!(GetCartesianLimit.name(), "Get Cartesian Limit");
    assert_eq!(SetCollisionBehavior.name(), "Set Collision Behavior");
    assert_eq!(SetJointImpedance.name(), "Set Joint Impedance");
    assert_eq!(SetCartesianImpedance.name(), "Set Cartesian Impedance");
    assert_eq!(SetGuidingMode.name(), "Set Guiding Mode");
    assert_eq!(SetEEToK.name(), "Set EE to K");
    assert_eq!(SetNEToEE.name(), "Set NE to EE");
    assert_eq!(SetLoad.name(), "Set Load");
    assert_eq!(SetFilters.name(), "Set Filters");
    assert_eq!(AutomaticErrorRecovery.name(), "Automatic Error Recovery");
    assert_eq!(GetRobotModel.name(), "Get Robot Model");
    // No CommandTraits specialisation in libfranka; this crate's own spellings.
    assert_eq!(Connect.name(), "Connect");
    assert_eq!(LoadModelLibrary.name(), "Load Model Library");
}

#[test]
fn stop_move_statuses_map_by_name_on_both_versions() {
    use v10::StopMoveStatus as S;

    let v5_expected = [
        (0u8, S::Success),
        (1, S::CommandNotPossibleRejected),
        (2, S::EmergencyAborted),
        (3, S::ReflexAborted),
        (4, S::Aborted),
    ];
    let name = CommandKind::StopMove.name();
    for (byte, expected) in v5_expected {
        assert_eq!(
            parse_stop_move_status(FciVersion::V5, byte, name).unwrap(),
            expected
        );
    }
    assert!(parse_stop_move_status(FciVersion::V5, 5, name).is_err());

    // v10 keeps its own numbering, in which 2 is the safety-function rejection.
    assert_eq!(
        parse_stop_move_status(FciVersion::V10, 2, name).unwrap(),
        S::CommandRejectedDueToActivatedSafetyFunctions
    );
    assert_eq!(
        parse_stop_move_status(FciVersion::V10, 5, name).unwrap(),
        S::Aborted
    );
    assert!(parse_stop_move_status(FciVersion::V10, 6, name).is_err());
}

#[test]
fn automatic_error_recovery_statuses_map_by_name_on_both_versions() {
    use v10::AutomaticErrorRecoveryStatus as S;

    let v5_expected = [
        (0u8, S::Success),
        (1, S::CommandNotPossibleRejected),
        (2, S::ManualErrorRecoveryRequiredRejected),
        (3, S::ReflexAborted),
        (4, S::EmergencyAborted),
        (5, S::Aborted),
    ];
    let name = CommandKind::AutomaticErrorRecovery.name();
    for (byte, expected) in v5_expected {
        assert_eq!(
            parse_automatic_error_recovery_status(FciVersion::V5, byte, name).unwrap(),
            expected
        );
    }
    assert!(parse_automatic_error_recovery_status(FciVersion::V5, 6, name).is_err());

    assert_eq!(
        parse_automatic_error_recovery_status(FciVersion::V10, 3, name).unwrap(),
        S::ManualErrorRecoveryRequiredRejected
    );
    assert_eq!(
        parse_automatic_error_recovery_status(FciVersion::V10, 6, name).unwrap(),
        S::Aborted
    );
    assert!(parse_automatic_error_recovery_status(FciVersion::V10, 7, name).is_err());
}

#[test]
fn getter_setter_and_command_statuses_stop_earlier_on_v5() {
    use v10::{CommandStatus as C, GetterSetterStatus as G};

    let setter = CommandKind::SetLoad.name();
    let base = CommandKind::GetRobotModel.name();

    assert_eq!(
        parse_getter_setter_status(FciVersion::V5, 2, setter).unwrap(),
        G::InvalidArgumentRejected
    );
    // Byte 3 is kCommandRejectedDueToActivatedSafetyFunctions, v10 only.
    assert!(parse_getter_setter_status(FciVersion::V5, 3, setter).is_err());
    assert_eq!(
        parse_getter_setter_status(FciVersion::V10, 3, setter).unwrap(),
        G::CommandRejectedDueToActivatedSafetyFunctions
    );

    assert_eq!(
        parse_command_status(FciVersion::V5, 1, base).unwrap(),
        C::CommandNotPossibleRejected
    );
    // Byte 2 is kCommandRejectedDueToActivatedSafetyFunctions, v10 only.
    assert!(parse_command_status(FciVersion::V5, 2, base).is_err());
    assert_eq!(
        parse_command_status(FciVersion::V10, 2, base).unwrap(),
        C::CommandRejectedDueToActivatedSafetyFunctions
    );
    assert!(parse_command_status(FciVersion::V10, 3, base).is_err());
}
