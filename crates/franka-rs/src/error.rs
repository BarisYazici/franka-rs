//! Error types (mirror libfranka's exception hierarchy).

use crate::errors::Errors;
use crate::robot_state::RobotState;

/// Result alias used throughout the crate.
pub type FrankaResult<T> = Result<T, FrankaError>;

/// Terminal status of a `Move` command (`research_interface::robot::Move::Status`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MoveStatus {
    /// The motion ended regularly; this is the only reply that makes a control loop return
    /// `Ok(())` (`kSuccess`).
    Success,
    /// The robot accepted the `Move` and the motion is now running (`kMotionStarted`). Sent
    /// once, as the first reply.
    MotionStarted,
    /// The motion was pre-empted by another command, typically `StopMove` (`kPreempted`).
    Preempted,
    /// The motion was pre-empted because a safety function became active
    /// (`kPreemptedDueToActivatedSafetyFunctions`, FCI v10 only).
    PreemptedDueToActivatedSafetyFunctions,
    /// The `Move` was refused because a safety function is active
    /// (`kCommandRejectedDueToActivatedSafetyFunctions`, FCI v10 only).
    CommandRejectedDueToActivatedSafetyFunctions,
    /// The robot is not in a mode that allows a motion, e.g. it is still in a reflex or guiding
    /// mode (`kCommandNotPossibleRejected`).
    CommandNotPossibleRejected,
    /// The `Move` was refused because the robot starts at a singular pose
    /// (`kStartAtSingularPoseRejected`).
    StartAtSingularPoseRejected,
    /// The `Move` arguments were out of range (`kInvalidArgumentRejected`).
    InvalidArgumentRejected,
    /// A reflex (collision, joint or Cartesian limit violation) aborted the motion
    /// (`kReflexAborted`); [`ControlException::last_motion_errors`] names which.
    ReflexAborted,
    /// The external emergency stop or the user stop aborted the motion (`kEmergencyAborted`).
    EmergencyAborted,
    /// The robot rejected a command the client sent during the motion (`kInputErrorAborted`),
    /// e.g. a setpoint that violates the rate limits.
    InputErrorAborted,
    /// The motion was aborted for a reason none of the above covers (`kAborted`).
    Aborted,
}

impl MoveStatus {
    /// Wire value (`u8`).
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses the wire value.
    pub const fn from_u8(v: u8) -> Option<MoveStatus> {
        Some(match v {
            0 => MoveStatus::Success,
            1 => MoveStatus::MotionStarted,
            2 => MoveStatus::Preempted,
            3 => MoveStatus::PreemptedDueToActivatedSafetyFunctions,
            4 => MoveStatus::CommandRejectedDueToActivatedSafetyFunctions,
            5 => MoveStatus::CommandNotPossibleRejected,
            6 => MoveStatus::StartAtSingularPoseRejected,
            7 => MoveStatus::InvalidArgumentRejected,
            8 => MoveStatus::ReflexAborted,
            9 => MoveStatus::EmergencyAborted,
            10 => MoveStatus::InputErrorAborted,
            11 => MoveStatus::Aborted,
            _ => return None,
        })
    }
}

/// The robot command that was sent in one cycle, as recorded in the control log.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RobotCommandLog {
    /// Commanded joint positions, in rad.
    pub q_c: [f64; 7],
    /// Commanded joint velocities, in rad/s.
    pub dq_c: [f64; 7],
    /// Commanded end-effector pose in base frame, a column-major 4x4 homogeneous transform
    /// (translation in m).
    pub O_T_EE_c: [f64; 16],
    /// Commanded end-effector twist in base frame: linear m/s then angular rad/s.
    pub O_dP_EE_c: [f64; 6],
    /// Commanded elbow configuration: joint-3 position in rad and the sign of joint 4.
    pub elbow_c: [f64; 2],
    /// Commanded joint torques without gravity and friction, in Nm.
    pub tau_J_d: [f64; 7],
}

/// One entry of the control log attached to a [`ControlException`] (mirrors `franka::Record`).
///
/// With the `serde` feature a `Vec<Record>` is what a control log looks like on disk.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Record {
    /// The robot state received in this cycle.
    pub state: RobotState,
    /// The command sent in this cycle, or `None` when the cycle sent nothing.
    pub command: Option<RobotCommandLog>,
}

/// A motion ended abnormally (mirrors `franka::ControlException`).
///
/// With the `serde` feature the whole exception serialises, log included; nothing in it is
/// unserialisable.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ControlException {
    /// Full libfranka-style message, including the error names and success-rate lines when
    /// the motion was aborted by a reflex.
    pub message: String,
    /// Terminal `Move` status when the exception was produced by a `Move` response.
    pub move_status: Option<MoveStatus>,
    /// `reflex_reason` of the last received state.
    pub last_motion_errors: Errors,
    /// The last states and commands before the exception (newest last).
    pub log: Vec<Record>,
}

impl ControlException {
    /// Convenience constructor for an exception without a log.
    pub fn new(message: impl Into<String>) -> Self {
        ControlException {
            message: message.into(),
            move_status: None,
            last_motion_errors: Errors::default(),
            log: Vec::new(),
        }
    }
}

impl std::fmt::Display for ControlException {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ControlException {}

/// Every error the crate can return (mirrors libfranka's exception types).
#[derive(Debug, Clone, thiserror::Error)]
pub enum FrankaError {
    /// Connection or socket failure (`franka::NetworkException`).
    #[error("{0}")]
    Network(String),
    /// Malformed or unexpected protocol data (`franka::ProtocolException`).
    #[error("{0}")]
    Protocol(String),
    /// The server speaks another FCI version (`franka::IncompatibleVersionException`).
    #[error("libfranka: Incompatible library version (server version: {server_version}, library version: {library_version}).")]
    IncompatibleVersion {
        /// FCI version the control box announced in its `Connect` reply.
        server_version: u16,
        /// FCI version this client requested (5 or 10).
        library_version: u16,
    },
    /// A TCP command was rejected (`franka::CommandException`).
    #[error("{0}")]
    Command(String),
    /// A motion ended abnormally (`franka::ControlException`).
    #[error("{0}")]
    Control(ControlException),
    /// Realtime priority or kernel requirements not met (`franka::RealtimeException`).
    #[error("{0}")]
    Realtime(String),
    /// Operation not allowed in the current state (`franka::InvalidOperationException`).
    #[error("{0}")]
    InvalidOperation(String),
    /// Invalid user input (`std::invalid_argument` in libfranka).
    #[error("{0}")]
    InvalidArgument(String),
    /// Model loading or evaluation failure (`franka::ModelException`).
    #[error("{0}")]
    Model(String),
}

impl From<ControlException> for FrankaError {
    fn from(e: ControlException) -> Self {
        FrankaError::Control(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn incompatible_version_text_matches_libfranka() {
        let e = FrankaError::IncompatibleVersion {
            server_version: 10,
            library_version: 9,
        };
        assert_eq!(
            e.to_string(),
            "libfranka: Incompatible library version (server version: 10, library version: 9)."
        );
    }

    #[test]
    fn move_status_roundtrip() {
        for v in 0..=11u8 {
            let s = MoveStatus::from_u8(v).unwrap();
            assert_eq!(s.to_u8(), v);
        }
        assert!(MoveStatus::from_u8(12).is_none());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn record_and_exception_round_trip_through_json() {
        use crate::duration::Duration;
        use crate::robot_state::RobotMode;

        let mut state = RobotState {
            time: Duration::from_millis(123_456),
            robot_mode: RobotMode::Reflex,
            ..RobotState::default()
        };
        state.q = [0.1, -0.2, 0.3, -2.0, 0.5, 1.6, 0.7];
        state.joint_contact[3] = 1.0;
        state.O_F_ext_hat_K = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3];
        state.current_errors.0[Errors::index_of("cartesian_reflex").unwrap()] = true;
        let record = Record {
            state,
            command: Some(RobotCommandLog {
                q_c: [0.1, -0.2, 0.3, -2.0, 0.5, 1.6, 0.7],
                ..RobotCommandLog::default()
            }),
        };
        let json = serde_json::to_string(&record).unwrap();
        assert!(
            json.contains("\"current_errors\":[\"cartesian_reflex\"]"),
            "{json}"
        );
        assert!(json.contains("\"time\":123456"), "{json}");
        let back: Record = serde_json::from_str(&json).unwrap();
        assert_eq!(back, record);
        assert_eq!(back.state.q[3], -2.0);
        assert_eq!(back.state.joint_contact[3], 1.0);
        assert_eq!(back.state.current_errors, state.current_errors);
        assert_eq!(back.command.unwrap().q_c[5], 1.6);

        let exception = ControlException {
            message: "libfranka: Move command aborted: motion aborted by reflex!".into(),
            move_status: Some(MoveStatus::ReflexAborted),
            last_motion_errors: state.current_errors,
            log: vec![record.clone(), record],
        };
        let json = serde_json::to_string(&exception).unwrap();
        let back: ControlException = serde_json::from_str(&json).unwrap();
        assert_eq!(back.message, exception.message);
        assert_eq!(back.move_status, Some(MoveStatus::ReflexAborted));
        assert_eq!(back.last_motion_errors, exception.last_motion_errors);
        assert_eq!(back.log, exception.log);
    }
}
