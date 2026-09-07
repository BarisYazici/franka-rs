//! Encoding: the `Move::Request` and the UDP `RobotCommand`, both version-agnostic at the
//! call site.

use zerocopy::IntoBytes;

use crate::error::{FrankaError, FrankaResult};
use crate::wire::robot::{v10, v5};

use super::{FciVersion, MOVE_REQUEST_MAX_LEN, ROBOT_COMMAND_MAX_LEN};

/// Encodes a `Move::Request` into `out` and returns its length: 56 bytes on FCI v5, 113 on
/// FCI v10.
///
/// `motion_generator_mode` is the FCI v10 enum, which is the superset of the two.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] when `version` is [`FciVersion::V5`] and
/// `motion_generator_mode` is [`v10::MoveMotionGeneratorMode::None`]: FCI v5 has no torque-only
/// motion generator mode (`service_types.h:130-135`, libfranka 0.9.2). Torque control on a
/// FER runs a joint-velocity generator commanding zero velocities instead.
pub(crate) fn move_request(
    version: FciVersion,
    controller_mode: v10::MoveControllerMode,
    motion_generator_mode: v10::MoveMotionGeneratorMode,
    maximum_path_deviation: v10::Deviation,
    maximum_goal_pose_deviation: v10::Deviation,
    out: &mut [u8; MOVE_REQUEST_MAX_LEN],
) -> FrankaResult<usize> {
    match version {
        FciVersion::V5 => {
            let mode = match motion_generator_mode {
                v10::MoveMotionGeneratorMode::JointPosition => {
                    v5::MoveMotionGeneratorMode::JointPosition
                }
                v10::MoveMotionGeneratorMode::JointVelocity => {
                    v5::MoveMotionGeneratorMode::JointVelocity
                }
                v10::MoveMotionGeneratorMode::CartesianPosition => {
                    v5::MoveMotionGeneratorMode::CartesianPosition
                }
                v10::MoveMotionGeneratorMode::CartesianVelocity => {
                    v5::MoveMotionGeneratorMode::CartesianVelocity
                }
                v10::MoveMotionGeneratorMode::None => {
                    return Err(FrankaError::InvalidArgument(
                        "libfranka: Move: motion generator mode None is not available on FCI \
                         version 5."
                            .to_string(),
                    ))
                }
            };
            let request = v5::MoveRequest::new(
                controller_mode,
                mode,
                maximum_path_deviation,
                maximum_goal_pose_deviation,
            );
            let bytes = request.as_bytes();
            out[..bytes.len()].copy_from_slice(bytes);
            Ok(bytes.len())
        }
        FciVersion::V10 => {
            let request = v10::MoveRequest::new(
                controller_mode,
                motion_generator_mode,
                maximum_path_deviation,
                maximum_goal_pose_deviation,
            );
            out.copy_from_slice(request.as_bytes());
            Ok(out.len())
        }
    }
}

/// The fields of one UDP `RobotCommand`, in a version-agnostic form.
///
/// `torque_command_finished` is ignored on FCI v5, whose `ControllerCommand` has no such flag
/// (`rbk_types.h:97-99`, libfranka 0.9.2).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct RobotCommandData {
    /// `message_id` of the state this command answers.
    pub message_id: u64,
    pub q_c: [f64; 7],
    pub dq_c: [f64; 7],
    pub O_T_EE_c: [f64; 16],
    pub O_dP_EE_c: [f64; 6],
    pub elbow_c: [f64; 2],
    pub valid_elbow: bool,
    pub motion_generation_finished: bool,
    pub tau_J_d: [f64; 7],
    /// FCI v10 only; dropped on FCI v5.
    pub torque_command_finished: bool,
}

/// Encodes a UDP `RobotCommand` into `out` and returns its length: 370 bytes on FCI v5, 371 on
/// FCI v10.
pub(crate) fn robot_command(
    version: FciVersion,
    data: &RobotCommandData,
    out: &mut [u8; ROBOT_COMMAND_MAX_LEN],
) -> usize {
    use crate::wire::f64s_to_wire;
    use zerocopy::little_endian::U64;

    match version {
        FciVersion::V5 => {
            let command = v5::RobotCommand {
                message_id: U64::new(data.message_id),
                motion: v5::MotionGeneratorCommand {
                    q_c: f64s_to_wire(&data.q_c),
                    dq_c: f64s_to_wire(&data.dq_c),
                    O_T_EE_c: f64s_to_wire(&data.O_T_EE_c),
                    O_dP_EE_c: f64s_to_wire(&data.O_dP_EE_c),
                    elbow_c: f64s_to_wire(&data.elbow_c),
                    valid_elbow: u8::from(data.valid_elbow),
                    motion_generation_finished: u8::from(data.motion_generation_finished),
                },
                control: v5::ControllerCommand {
                    tau_J_d: f64s_to_wire(&data.tau_J_d),
                },
            };
            let bytes = command.as_bytes();
            out[..bytes.len()].copy_from_slice(bytes);
            bytes.len()
        }
        FciVersion::V10 => {
            let command = v10::RobotCommand {
                message_id: U64::new(data.message_id),
                motion: v10::MotionGeneratorCommand {
                    q_c: f64s_to_wire(&data.q_c),
                    dq_c: f64s_to_wire(&data.dq_c),
                    O_T_EE_c: f64s_to_wire(&data.O_T_EE_c),
                    O_dP_EE_c: f64s_to_wire(&data.O_dP_EE_c),
                    elbow_c: f64s_to_wire(&data.elbow_c),
                    valid_elbow: u8::from(data.valid_elbow),
                    motion_generation_finished: u8::from(data.motion_generation_finished),
                },
                control: v10::ControllerCommand {
                    tau_J_d: f64s_to_wire(&data.tau_J_d),
                    torque_command_finished: u8::from(data.torque_command_finished),
                },
            };
            out.copy_from_slice(command.as_bytes());
            out.len()
        }
    }
}
