//! Byte-layout conformance of the wire structs against libfranka 0.21.2.
//!
//! Every expectation below is derived from the C++ declarations in
//! `reference/libfranka/common/include/research_interface/robot/{service_types.h,rbk_types.h}`
//! and `.../gripper/types.h`, which are all inside `#pragma pack(push, 1)`. These tests need no
//! simulator.

use std::mem::{offset_of, size_of};

use franka::wire::gripper::{
    GraspRequest, GripperCommandHeader, GripperCommandResponse, GripperConnectRequest,
    GripperConnectResponse, GripperMoveRequest, GripperState,
};
use franka::wire::robot::{
    AutomaticErrorRecoveryResponse, CommandResponse, CommandStatus, ConnectRequest,
    ConnectResponse, ControllerCommand, Deviation, GetterSetterResponse, GetterSetterStatus,
    MotionGeneratorCommand, MoveRequest, MoveResponse, RobotCommand, RobotState,
    SetCartesianImpedanceRequest, SetCollisionBehaviorRequest, SetEEToKRequest,
    SetGuidingModeRequest, SetJointImpedanceRequest, SetLoadRequest, SetNEToEERequest,
    StopMoveResponse,
};
use franka::wire::{CommandHeader, GRIPPER_HEADER_LEN, ROBOT_HEADER_LEN};

#[test]
fn robot_state_and_command_sizes() {
    // rbk_types.h: uint64_t + 320 floats + 2 mode bytes + 2*41 bools + 1 mode byte + 1 float.
    assert_eq!(size_of::<RobotState>(), 1377);
    assert_eq!(size_of::<MotionGeneratorCommand>(), 306);
    assert_eq!(size_of::<ControllerCommand>(), 57);
    assert_eq!(size_of::<RobotCommand>(), 371);
}

#[test]
fn robot_state_tail_offsets() {
    // The 320 floats end at 8 + 320*4 = 1288; the trailing scalars follow with no padding.
    assert_eq!(offset_of!(RobotState, motion_generator_mode), 1288);
    assert_eq!(offset_of!(RobotState, controller_mode), 1289);
    assert_eq!(offset_of!(RobotState, errors), 1290);
    assert_eq!(offset_of!(RobotState, reflex_reason), 1331);
    assert_eq!(offset_of!(RobotState, robot_mode), 1372);
    assert_eq!(offset_of!(RobotState, control_command_success_rate), 1373);
}

#[test]
fn robot_state_head_offsets() {
    // The six 4x4 transforms come first, in the header's order: O_T_EE, O_T_EE_d, F_T_EE,
    // EE_T_K, F_T_NE, NE_T_EE (rbk_types.h lines 88-93).
    assert_eq!(offset_of!(RobotState, message_id), 0);
    assert_eq!(offset_of!(RobotState, O_T_EE), 8);
    assert_eq!(offset_of!(RobotState, O_T_EE_d), 8 + 64);
    assert_eq!(offset_of!(RobotState, F_T_EE), 8 + 128);
    assert_eq!(offset_of!(RobotState, EE_T_K), 8 + 192);
    assert_eq!(offset_of!(RobotState, F_T_NE), 8 + 256);
    assert_eq!(offset_of!(RobotState, NE_T_EE), 8 + 320);
    assert_eq!(offset_of!(RobotState, m_ee), 8 + 384);
    assert_eq!(offset_of!(RobotState, q), 8 + 4 * 147);
    assert_eq!(offset_of!(RobotState, accelerometer_top), 8 + 4 * 284);
    assert_eq!(offset_of!(RobotState, accelerometer_bottom), 8 + 4 * 302);
}

#[test]
fn robot_command_offsets() {
    assert_eq!(offset_of!(RobotCommand, message_id), 0);
    assert_eq!(offset_of!(RobotCommand, motion), 8);
    assert_eq!(offset_of!(RobotCommand, control), 8 + 306);
    assert_eq!(offset_of!(MotionGeneratorCommand, valid_elbow), 304);
    assert_eq!(
        offset_of!(MotionGeneratorCommand, motion_generation_finished),
        305
    );
    assert_eq!(offset_of!(ControllerCommand, torque_command_finished), 56);
}

#[test]
fn robot_service_type_sizes() {
    // service_types.h: CommandHeader is Command(uint32) + command_id + size.
    assert_eq!(size_of::<CommandHeader>(), 12);
    assert_eq!(ROBOT_HEADER_LEN, 12);

    assert_eq!(size_of::<ConnectRequest>(), 4);
    assert_eq!(size_of::<ConnectResponse>(), 3);

    // Move::Request = 2 enums (uint32) + 2 Deviations + bool + array<double, 7>.
    assert_eq!(size_of::<Deviation>(), 24);
    assert_eq!(size_of::<MoveRequest>(), 113);
    assert_eq!(size_of::<MoveResponse>(), 1);
    assert_eq!(size_of::<StopMoveResponse>(), 1);
    assert_eq!(size_of::<AutomaticErrorRecoveryResponse>(), 1);
    assert_eq!(size_of::<GetterSetterResponse>(), 1);
    assert_eq!(size_of::<CommandResponse>(), 1);

    assert_eq!(size_of::<SetCollisionBehaviorRequest>(), 416);
    assert_eq!(size_of::<SetJointImpedanceRequest>(), 56);
    assert_eq!(size_of::<SetCartesianImpedanceRequest>(), 48);
    assert_eq!(size_of::<SetGuidingModeRequest>(), 7);
    assert_eq!(size_of::<SetEEToKRequest>(), 128);
    assert_eq!(size_of::<SetNEToEERequest>(), 128);
    assert_eq!(size_of::<SetLoadRequest>(), 104);
}

#[test]
fn move_request_field_offsets() {
    assert_eq!(offset_of!(MoveRequest, controller_mode), 0);
    assert_eq!(offset_of!(MoveRequest, motion_generator_mode), 4);
    assert_eq!(offset_of!(MoveRequest, maximum_path_deviation), 8);
    assert_eq!(offset_of!(MoveRequest, maximum_goal_pose_deviation), 32);
    assert_eq!(offset_of!(MoveRequest, use_async_motion_generator), 56);
    assert_eq!(offset_of!(MoveRequest, maximum_velocity), 57);
}

#[test]
fn set_collision_behavior_field_offsets() {
    // 4 torque arrays of 7 doubles, then 4 force arrays of 6 doubles.
    assert_eq!(
        offset_of!(
            SetCollisionBehaviorRequest,
            lower_torque_thresholds_acceleration
        ),
        0
    );
    assert_eq!(
        offset_of!(
            SetCollisionBehaviorRequest,
            upper_torque_thresholds_acceleration
        ),
        56
    );
    assert_eq!(
        offset_of!(SetCollisionBehaviorRequest, lower_torque_thresholds_nominal),
        112
    );
    assert_eq!(
        offset_of!(SetCollisionBehaviorRequest, upper_torque_thresholds_nominal),
        168
    );
    assert_eq!(
        offset_of!(
            SetCollisionBehaviorRequest,
            lower_force_thresholds_acceleration
        ),
        224
    );
    assert_eq!(
        offset_of!(
            SetCollisionBehaviorRequest,
            upper_force_thresholds_acceleration
        ),
        272
    );
    assert_eq!(
        offset_of!(SetCollisionBehaviorRequest, lower_force_thresholds_nominal),
        320
    );
    assert_eq!(
        offset_of!(SetCollisionBehaviorRequest, upper_force_thresholds_nominal),
        368
    );
    assert_eq!(offset_of!(SetLoadRequest, m_load), 0);
    assert_eq!(offset_of!(SetLoadRequest, F_x_Cload), 8);
    assert_eq!(offset_of!(SetLoadRequest, I_load), 32);
}

#[test]
fn gripper_sizes() {
    // gripper/types.h: CommandHeader is Command(uint16) + command_id + size.
    assert_eq!(size_of::<GripperCommandHeader>(), 10);
    assert_eq!(GRIPPER_HEADER_LEN, 10);
    assert_eq!(size_of::<GripperConnectRequest>(), 4);
    assert_eq!(size_of::<GripperConnectResponse>(), 4);
    assert_eq!(size_of::<GripperCommandResponse>(), 2);
    assert_eq!(size_of::<GripperMoveRequest>(), 16);
    assert_eq!(size_of::<GraspRequest>(), 40);
    assert_eq!(size_of::<GripperState>(), 23);

    assert_eq!(offset_of!(GripperState, message_id), 0);
    assert_eq!(offset_of!(GripperState, width), 4);
    assert_eq!(offset_of!(GripperState, max_width), 12);
    assert_eq!(offset_of!(GripperState, is_grasped), 20);
    assert_eq!(offset_of!(GripperState, temperature), 21);
}

#[test]
fn move_request_serializes_to_113_little_endian_bytes() {
    use franka::wire::robot::{MoveControllerMode, MoveMotionGeneratorMode};
    use zerocopy::IntoBytes;

    let request = MoveRequest::new(
        MoveControllerMode::ExternalController,
        MoveMotionGeneratorMode::JointVelocity,
        Deviation::new(1.0, 2.0, 3.0),
        Deviation::new(4.0, 5.0, 6.0),
    );
    let bytes = request.as_bytes();
    assert_eq!(bytes.len(), 113);
    assert_eq!(&bytes[0..4], &2u32.to_le_bytes());
    assert_eq!(&bytes[4..8], &1u32.to_le_bytes());
    assert_eq!(&bytes[8..16], &1.0f64.to_le_bytes());
    assert_eq!(&bytes[32..40], &4.0f64.to_le_bytes());
    assert_eq!(bytes[56], 0);
    assert!(bytes[57..].iter().all(|&b| b == 0));
}

/// `CommandBase::Status` (service_types.h:158-162) and `GetterSetterCommandBase::Status`
/// (service_types.h:173-178) agree on 0 and 1 only: value 2 is
/// `kCommandRejectedDueToActivatedSafetyFunctions` in the first and `kInvalidArgumentRejected`
/// in the second.
#[test]
fn command_and_getter_setter_statuses_diverge_at_two() {
    assert_eq!(CommandStatus::from_u8(0), Some(CommandStatus::Success));
    assert_eq!(
        CommandStatus::from_u8(1),
        Some(CommandStatus::CommandNotPossibleRejected)
    );
    assert_eq!(
        CommandStatus::from_u8(2),
        Some(CommandStatus::CommandRejectedDueToActivatedSafetyFunctions)
    );
    assert_eq!(CommandStatus::from_u8(3), None);

    assert_eq!(
        GetterSetterStatus::from_u8(2),
        Some(GetterSetterStatus::InvalidArgumentRejected)
    );
    assert_eq!(
        GetterSetterStatus::from_u8(3),
        Some(GetterSetterStatus::CommandRejectedDueToActivatedSafetyFunctions)
    );
    assert_eq!(GetterSetterStatus::from_u8(4), None);
}
