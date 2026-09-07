//! Byte-layout conformance of the FCI v5 wire structs against libfranka 0.9.2.
//!
//! Every expectation below is derived from the C++ declarations in
//! `reference/libfranka-0.9.2/common/include/research_interface/robot/{service_types.h,rbk_types.h}`,
//! which are all inside `#pragma pack(push, 1)`. These tests need no simulator.
//!
//! The sibling `wire_sizes.rs` does the same for FCI v10; where a v5 struct is byte-identical
//! to its v10 counterpart the v5 module re-exports it, and the size is pinned in both files.

use std::mem::{offset_of, size_of};

use franka::wire::robot::v5::{
    AutomaticErrorRecoveryResponse, AutomaticErrorRecoveryStatus, Command, CommandResponse,
    CommandStatus, ConnectRequest, ConnectResponse, ConnectStatus, ControllerCommand,
    ControllerMode, Deviation, GetCartesianLimitRequest, GetCartesianLimitResponse,
    GetterSetterResponse, GetterSetterStatus, LoadModelLibraryArchitecture,
    LoadModelLibraryRequest, LoadModelLibraryResponse, LoadModelLibraryStatus,
    LoadModelLibrarySystem, MotionGeneratorCommand, MotionGeneratorMode, MoveControllerMode,
    MoveMotionGeneratorMode, MoveRequest, MoveResponse, MoveStatus, RobotCommand, RobotMode,
    RobotState, SetCartesianImpedanceRequest, SetCollisionBehaviorRequest, SetEEToKRequest,
    SetFiltersRequest, SetGuidingModeRequest, SetJointImpedanceRequest, SetLoadRequest,
    SetNEToEERequest, StopMoveResponse, StopMoveStatus, ROBOT_VERSION,
};
use franka::wire::ROBOT_HEADER_LEN;

/// Number of `double`s between `message_id` and `motion_generator_mode` in the v5 state
/// (`rbk_types.h:40-78`), counted field by field:
///
/// | fields | doubles |
/// |---|---|
/// | `O_T_EE`, `O_T_EE_d`, `F_T_EE`, `EE_T_K`, `F_T_NE`, `NE_T_EE` | 6*16 = 96 |
/// | `m_ee`, `I_ee`, `F_x_Cee` | 1 + 9 + 3 = 13 |
/// | `m_load`, `I_load`, `F_x_Cload` | 1 + 9 + 3 = 13 |
/// | `elbow`, `elbow_d` | 2 + 2 = 4 |
/// | `tau_J`, `tau_J_d`, `dtau_J`, `q`, `q_d`, `dq`, `dq_d`, `ddq_d`, `joint_contact` | 9*7 = 63 |
/// | `cartesian_contact`, `joint_collision`, `cartesian_collision` | 6 + 7 + 6 = 19 |
/// | `tau_ext_hat_filtered`, `O_F_ext_hat_K`, `K_F_ext_hat_K`, `O_dP_EE_d`, `O_ddP_O` | 7 + 6 + 6 + 6 + 3 = 28 |
/// | `elbow_c`, `delbow_c`, `ddelbow_c` | 3*2 = 6 |
/// | `O_T_EE_c`, `O_dP_EE_c`, `O_ddP_EE_c` | 16 + 6 + 6 = 28 |
/// | `theta`, `dtheta` | 2*7 = 14 |
///
/// 96 + 13 + 13 + 4 + 63 + 19 + 28 + 6 + 28 + 14 = 284.
const STATE_DOUBLES: usize = 284;

/// Offset of the first non-`double` field: `message_id` plus the 284 doubles above.
const MODE_OFFSET: usize = 8 + 8 * STATE_DOUBLES;

#[test]
fn robot_state_and_command_sizes() {
    // rbk_types.h:38-85: uint64_t + 284 doubles + 2 mode bytes + 2*41 bools + 1 mode byte +
    // 1 double.
    assert_eq!(MODE_OFFSET, 2280);
    assert_eq!(
        size_of::<RobotState>(),
        MODE_OFFSET + 1 + 1 + 41 + 41 + 1 + 8
    );
    assert_eq!(size_of::<RobotState>(), 2373);

    // rbk_types.h:87-105. ControllerCommand has no torque_command_finished on v5, so it is one
    // byte shorter than v10's 57 and RobotCommand is 370 instead of 371.
    assert_eq!(size_of::<MotionGeneratorCommand>(), 306);
    assert_eq!(size_of::<ControllerCommand>(), 56);
    assert_eq!(size_of::<RobotCommand>(), 370);
}

/// Every field of the v5 `RobotState`, in `rbk_types.h:38-85` order, as
/// `(name, offset_of!, size in bytes)`.
///
/// The point of listing all 46 is that the offsets are then checked against a running sum
/// rather than against hand-computed constants, so **any** reordering fails — including a
/// swap of two same-shape siblings (`tau_J_d` <-> `dtau_J`, `O_F_ext_hat_K` <->
/// `K_F_ext_hat_K`, `elbow` <-> `elbow_d`, `theta` <-> `dtheta`, ...), which no size or
/// spot-check assertion can see and which would quietly feed the rate limiter, the
/// collision flags or the motor-side angles the wrong signal on a real FER.
fn robot_state_fields() -> Vec<(&'static str, usize, usize)> {
    macro_rules! field {
        ($name:ident, $bytes:expr) => {
            (stringify!($name), offset_of!(RobotState, $name), $bytes)
        };
    }
    vec![
        field!(message_id, 8),
        // The six 4x4 transforms come first, in the header's order (rbk_types.h:40-45) —
        // 128 bytes each, not the 64 of v10's float matrices.
        field!(O_T_EE, 8 * 16),
        field!(O_T_EE_d, 8 * 16),
        field!(F_T_EE, 8 * 16),
        field!(EE_T_K, 8 * 16),
        field!(F_T_NE, 8 * 16),
        field!(NE_T_EE, 8 * 16),
        field!(m_ee, 8),
        field!(I_ee, 8 * 9),
        field!(F_x_Cee, 8 * 3),
        field!(m_load, 8),
        field!(I_load, 8 * 9),
        field!(F_x_Cload, 8 * 3),
        field!(elbow, 8 * 2),
        field!(elbow_d, 8 * 2),
        field!(tau_J, 8 * 7),
        field!(tau_J_d, 8 * 7),
        field!(dtau_J, 8 * 7),
        field!(q, 8 * 7),
        field!(q_d, 8 * 7),
        field!(dq, 8 * 7),
        field!(dq_d, 8 * 7),
        field!(ddq_d, 8 * 7),
        field!(joint_contact, 8 * 7),
        field!(cartesian_contact, 8 * 6),
        field!(joint_collision, 8 * 7),
        field!(cartesian_collision, 8 * 6),
        field!(tau_ext_hat_filtered, 8 * 7),
        field!(O_F_ext_hat_K, 8 * 6),
        field!(K_F_ext_hat_K, 8 * 6),
        field!(O_dP_EE_d, 8 * 6),
        field!(O_ddP_O, 8 * 3),
        field!(elbow_c, 8 * 2),
        field!(delbow_c, 8 * 2),
        field!(ddelbow_c, 8 * 2),
        field!(O_T_EE_c, 8 * 16),
        field!(O_dP_EE_c, 8 * 6),
        field!(O_ddP_EE_c, 8 * 6),
        field!(theta, 8 * 7),
        field!(dtheta, 8 * 7),
        // The tail: two mode bytes, two 41-element `std::array<bool>`s, the robot mode and
        // one `double` success rate (a `float` on v10).
        field!(motion_generator_mode, 1),
        field!(controller_mode, 1),
        field!(errors, 41),
        field!(reflex_reason, 41),
        field!(robot_mode, 1),
        field!(control_command_success_rate, 8),
    ]
}

#[test]
fn every_robot_state_field_offset_is_pinned() {
    let fields = robot_state_fields();
    assert_eq!(fields.len(), 46, "rbk_types.h:38-85 declares 46 fields");

    let mut expected = 0usize;
    let mut doubles = 0usize;
    for (name, actual, size) in &fields {
        assert_eq!(
            *actual, expected,
            "RobotState::{name} is at offset {actual}, expected {expected} — the field \
             order no longer matches rbk_types.h:38-85 of libfranka 0.9.2"
        );
        expected += size;
        if *name != "message_id" && size % 8 == 0 && expected <= MODE_OFFSET {
            doubles += size / 8;
        }
    }

    assert_eq!(
        expected,
        size_of::<RobotState>(),
        "the field table does not cover the whole struct"
    );
    assert_eq!(expected, 2373);
    assert_eq!(
        doubles, STATE_DOUBLES,
        "284 doubles between message_id and the modes"
    );

    // The named boundaries the rest of this file and the parser rely on.
    let at = |name: &str| {
        fields
            .iter()
            .find(|(n, _, _)| *n == name)
            .unwrap_or_else(|| panic!("no field {name}"))
            .1
    };
    assert_eq!(at("O_T_EE"), 8);
    assert_eq!(at("tau_J"), 8 + 8 * 126);
    assert_eq!(at("q"), 8 + 8 * 147);
    assert_eq!(at("theta"), 8 + 8 * 270);
    assert_eq!(at("motion_generator_mode"), MODE_OFFSET);
    assert_eq!(at("motion_generator_mode"), 2280);
    assert_eq!(at("errors"), 2282);
    assert_eq!(at("reflex_reason"), 2323);
    assert_eq!(at("robot_mode"), 2364);
    assert_eq!(at("control_command_success_rate"), 2365);
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
    // tau_J_d is the whole of ControllerCommand on v5.
    assert_eq!(offset_of!(ControllerCommand, tau_J_d), 0);
    assert_eq!(
        offset_of!(RobotCommand, control) + offset_of!(ControllerCommand, tau_J_d),
        314
    );
}

#[test]
fn robot_service_type_sizes() {
    // The 12-byte robot CommandHeader is unchanged between the versions (service_types.h:37-45).
    assert_eq!(ROBOT_HEADER_LEN, 12);
    assert_eq!(ROBOT_VERSION, 5);

    assert_eq!(size_of::<ConnectRequest>(), 4);
    assert_eq!(size_of::<ConnectResponse>(), 3);

    // Move::Request = 2 enums (uint32) + 2 Deviations; no async tail, unlike v10's 113 bytes.
    assert_eq!(size_of::<Deviation>(), 24);
    assert_eq!(size_of::<MoveRequest>(), 56);
    assert_eq!(size_of::<MoveResponse>(), 1);
    assert_eq!(size_of::<StopMoveResponse>(), 1);
    assert_eq!(size_of::<AutomaticErrorRecoveryResponse>(), 1);
    assert_eq!(size_of::<GetterSetterResponse>(), 1);
    assert_eq!(size_of::<CommandResponse>(), 1);

    // Identical to v10 (service_types.h:210-299).
    assert_eq!(size_of::<SetCollisionBehaviorRequest>(), 416);
    assert_eq!(size_of::<SetJointImpedanceRequest>(), 56);
    assert_eq!(size_of::<SetCartesianImpedanceRequest>(), 48);
    assert_eq!(size_of::<SetGuidingModeRequest>(), 7);
    assert_eq!(size_of::<SetEEToKRequest>(), 128);
    assert_eq!(size_of::<SetNEToEERequest>(), 128);
    assert_eq!(size_of::<SetLoadRequest>(), 104);

    // v5-only commands.
    assert_eq!(size_of::<SetFiltersRequest>(), 40);
    assert_eq!(size_of::<GetCartesianLimitRequest>(), 4);
    assert_eq!(size_of::<GetCartesianLimitResponse>(), 154);
    assert_eq!(size_of::<LoadModelLibraryRequest>(), 2);
    assert_eq!(size_of::<LoadModelLibraryResponse>(), 1);
}

#[test]
fn v5_only_request_field_offsets() {
    assert_eq!(offset_of!(MoveRequest, controller_mode), 0);
    assert_eq!(offset_of!(MoveRequest, motion_generator_mode), 4);
    assert_eq!(offset_of!(MoveRequest, maximum_path_deviation), 8);
    assert_eq!(offset_of!(MoveRequest, maximum_goal_pose_deviation), 32);

    assert_eq!(
        offset_of!(SetFiltersRequest, joint_position_filter_frequency),
        0
    );
    assert_eq!(
        offset_of!(SetFiltersRequest, joint_velocity_filter_frequency),
        8
    );
    assert_eq!(
        offset_of!(SetFiltersRequest, cartesian_position_filter_frequency),
        16
    );
    assert_eq!(
        offset_of!(SetFiltersRequest, cartesian_velocity_filter_frequency),
        24
    );
    assert_eq!(
        offset_of!(SetFiltersRequest, controller_filter_frequency),
        32
    );

    assert_eq!(offset_of!(GetCartesianLimitRequest, id), 0);
    assert_eq!(offset_of!(GetCartesianLimitResponse, status), 0);
    assert_eq!(offset_of!(GetCartesianLimitResponse, object_world_size), 1);
    assert_eq!(offset_of!(GetCartesianLimitResponse, object_frame), 25);
    assert_eq!(
        offset_of!(GetCartesianLimitResponse, object_activation),
        153
    );

    assert_eq!(offset_of!(LoadModelLibraryRequest, architecture), 0);
    assert_eq!(offset_of!(LoadModelLibraryRequest, system), 1);
}

/// The FCI v5 numbering (`service_types.h:20-35`), which differs from v10's from
/// `GetCartesianLimit` on.
#[test]
fn command_numbering() {
    let all = [
        (Command::Connect, 0u32),
        (Command::Move, 1),
        (Command::StopMove, 2),
        (Command::GetCartesianLimit, 3),
        (Command::SetCollisionBehavior, 4),
        (Command::SetJointImpedance, 5),
        (Command::SetCartesianImpedance, 6),
        (Command::SetGuidingMode, 7),
        (Command::SetEEToK, 8),
        (Command::SetNEToEE, 9),
        (Command::SetLoad, 10),
        (Command::SetFilters, 11),
        (Command::AutomaticErrorRecovery, 12),
        (Command::LoadModelLibrary, 13),
    ];
    for (command, value) in all {
        assert_eq!(command.to_u32(), value);
        assert_eq!(Command::from_u32(value), Some(command));
    }
    assert_eq!(Command::from_u32(14), None);

    // Two command ids worth pinning by hand as well.
    assert_eq!(Command::SetCollisionBehavior.to_u32(), 4);
    assert_eq!(Command::LoadModelLibrary.to_u32(), 13);
}

/// Every v5 status enum round-trips through its wire byte and rejects the first value past its
/// end — which is exactly where the v10 enums keep going.
#[test]
fn status_enums_roundtrip() {
    let move_statuses = [
        MoveStatus::Success,
        MoveStatus::MotionStarted,
        MoveStatus::Preempted,
        MoveStatus::CommandNotPossibleRejected,
        MoveStatus::StartAtSingularPoseRejected,
        MoveStatus::InvalidArgumentRejected,
        MoveStatus::ReflexAborted,
        MoveStatus::EmergencyAborted,
        MoveStatus::InputErrorAborted,
        MoveStatus::Aborted,
    ];
    for (i, status) in move_statuses.into_iter().enumerate() {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(MoveStatus::from_u8(i as u8), Some(status));
    }
    assert_eq!(MoveStatus::from_u8(10), None);

    let stop_move_statuses = [
        StopMoveStatus::Success,
        StopMoveStatus::CommandNotPossibleRejected,
        StopMoveStatus::EmergencyAborted,
        StopMoveStatus::ReflexAborted,
        StopMoveStatus::Aborted,
    ];
    for (i, status) in stop_move_statuses.into_iter().enumerate() {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(StopMoveStatus::from_u8(i as u8), Some(status));
    }
    assert_eq!(StopMoveStatus::from_u8(5), None);

    let recovery_statuses = [
        AutomaticErrorRecoveryStatus::Success,
        AutomaticErrorRecoveryStatus::CommandNotPossibleRejected,
        AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected,
        AutomaticErrorRecoveryStatus::ReflexAborted,
        AutomaticErrorRecoveryStatus::EmergencyAborted,
        AutomaticErrorRecoveryStatus::Aborted,
    ];
    for (i, status) in recovery_statuses.into_iter().enumerate() {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(AutomaticErrorRecoveryStatus::from_u8(i as u8), Some(status));
    }
    assert_eq!(AutomaticErrorRecoveryStatus::from_u8(6), None);

    let getter_setter_statuses = [
        GetterSetterStatus::Success,
        GetterSetterStatus::CommandNotPossibleRejected,
        GetterSetterStatus::InvalidArgumentRejected,
    ];
    for (i, status) in getter_setter_statuses.into_iter().enumerate() {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(GetterSetterStatus::from_u8(i as u8), Some(status));
    }
    // v10 has kCommandRejectedDueToActivatedSafetyFunctions here; v5 does not.
    assert_eq!(GetterSetterStatus::from_u8(3), None);

    for (i, status) in [
        CommandStatus::Success,
        CommandStatus::CommandNotPossibleRejected,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(CommandStatus::from_u8(i as u8), Some(status));
    }
    assert_eq!(CommandStatus::from_u8(2), None);

    for (i, status) in [
        ConnectStatus::Success,
        ConnectStatus::IncompatibleLibraryVersion,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(ConnectStatus::from_u8(i as u8), Some(status));
    }

    for (i, status) in [
        LoadModelLibraryStatus::Success,
        LoadModelLibraryStatus::Error,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(status.to_u8() as usize, i);
        assert_eq!(LoadModelLibraryStatus::from_u8(i as u8), Some(status));
    }
    assert_eq!(LoadModelLibraryStatus::from_u8(2), None);
}

/// The mode enums: the `Move` motion generator has no `kNone` and the state's has no `None`
/// either, unlike FCI v10.
#[test]
fn mode_enums_roundtrip() {
    let move_modes = [
        MoveMotionGeneratorMode::JointPosition,
        MoveMotionGeneratorMode::JointVelocity,
        MoveMotionGeneratorMode::CartesianPosition,
        MoveMotionGeneratorMode::CartesianVelocity,
    ];
    for (i, mode) in move_modes.into_iter().enumerate() {
        assert_eq!(mode.to_u32() as usize, i);
        assert_eq!(MoveMotionGeneratorMode::from_u32(i as u32), Some(mode));
    }
    assert_eq!(MoveMotionGeneratorMode::from_u32(4), None);

    let controller_modes = [
        MoveControllerMode::JointImpedance,
        MoveControllerMode::CartesianImpedance,
        MoveControllerMode::ExternalController,
    ];
    for (i, mode) in controller_modes.into_iter().enumerate() {
        assert_eq!(mode.to_u32() as usize, i);
    }

    let state_modes = [
        MotionGeneratorMode::Idle,
        MotionGeneratorMode::JointPosition,
        MotionGeneratorMode::JointVelocity,
        MotionGeneratorMode::CartesianPosition,
        MotionGeneratorMode::CartesianVelocity,
    ];
    for (i, mode) in state_modes.into_iter().enumerate() {
        assert_eq!(mode.to_u8() as usize, i);
        assert_eq!(MotionGeneratorMode::from_u8(i as u8), Some(mode));
    }
    // v10 has None = 5 here; v5 does not.
    assert_eq!(MotionGeneratorMode::from_u8(5), None);

    for (i, mode) in [
        ControllerMode::JointImpedance,
        ControllerMode::CartesianImpedance,
        ControllerMode::ExternalController,
        ControllerMode::Other,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(mode.to_u8() as usize, i);
        assert_eq!(ControllerMode::from_u8(i as u8), Some(mode));
    }
    assert_eq!(ControllerMode::from_u8(4), None);

    for (i, mode) in [
        RobotMode::Other,
        RobotMode::Idle,
        RobotMode::Move,
        RobotMode::Guiding,
        RobotMode::Reflex,
        RobotMode::UserStopped,
        RobotMode::AutomaticErrorRecovery,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(mode.to_u8() as usize, i);
        assert_eq!(RobotMode::from_u8(i as u8), Some(mode));
    }
    assert_eq!(RobotMode::from_u8(7), None);

    for (i, arch) in [
        LoadModelLibraryArchitecture::X64,
        LoadModelLibraryArchitecture::X86,
        LoadModelLibraryArchitecture::ARM,
        LoadModelLibraryArchitecture::ARM64,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(arch.to_u8() as usize, i);
        assert_eq!(LoadModelLibraryArchitecture::from_u8(i as u8), Some(arch));
    }
    assert_eq!(LoadModelLibraryArchitecture::from_u8(4), None);

    for (i, system) in [
        LoadModelLibrarySystem::Linux,
        LoadModelLibrarySystem::Windows,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(system.to_u8() as usize, i);
        assert_eq!(LoadModelLibrarySystem::from_u8(i as u8), Some(system));
    }
    assert_eq!(LoadModelLibrarySystem::from_u8(2), None);
}

#[test]
fn move_request_serializes_to_56_little_endian_bytes() {
    use zerocopy::IntoBytes;

    let request = MoveRequest::new(
        MoveControllerMode::ExternalController,
        MoveMotionGeneratorMode::JointVelocity,
        Deviation::new(1.0, 2.0, 3.0),
        Deviation::new(4.0, 5.0, 6.0),
    );
    let bytes = request.as_bytes();
    assert_eq!(bytes.len(), 56);
    assert_eq!(&bytes[0..4], &2u32.to_le_bytes());
    assert_eq!(&bytes[4..8], &1u32.to_le_bytes());
    assert_eq!(&bytes[8..16], &1.0f64.to_le_bytes());
    assert_eq!(&bytes[32..40], &4.0f64.to_le_bytes());
    assert_eq!(&bytes[48..56], &6.0f64.to_le_bytes());
}

#[test]
fn set_filters_and_load_model_library_requests_serialize() {
    use zerocopy::IntoBytes;

    let request = SetFiltersRequest::new(10.0, 20.0, 30.0, 40.0, 50.0);
    let bytes = request.as_bytes();
    assert_eq!(bytes.len(), 40);
    for (i, expected) in [10.0f64, 20.0, 30.0, 40.0, 50.0].into_iter().enumerate() {
        assert_eq!(&bytes[i * 8..i * 8 + 8], &expected.to_le_bytes());
    }

    let request = LoadModelLibraryRequest::new(
        LoadModelLibraryArchitecture::X64,
        LoadModelLibrarySystem::Linux,
    );
    assert_eq!(request.as_bytes(), &[0u8, 0u8]);

    let request = GetCartesianLimitRequest::new(-3);
    assert_eq!(request.as_bytes(), &(-3i32).to_le_bytes());
}
