//! The request and response payload structs of the FCI v10 TCP commands.

use zerocopy::little_endian::{F64, U16, U32};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

use super::{ConnectStatus, MoveControllerMode, MoveMotionGeneratorMode};

/// `Connect::Request` — 4 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct ConnectRequest {
    /// Library version; this crate always sends [`crate::wire::ROBOT_VERSION`].
    pub version: U16,
    /// Port the client's UDP socket is bound to.
    pub udp_port: U16,
}

impl ConnectRequest {
    /// Builds a request announcing `version` and the client's `udp_port`.
    pub fn new(version: u16, udp_port: u16) -> Self {
        ConnectRequest {
            version: U16::new(version),
            udp_port: U16::new(udp_port),
        }
    }
}

/// `Connect::Response` — 3 bytes (`status` then the server's version).
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct ConnectResponse {
    /// [`ConnectStatus`] as a `u8`.
    pub status: u8,
    /// FCI version implemented by the server.
    pub version: U16,
}

impl ConnectResponse {
    /// Builds a response (used by the tests' mock servers).
    pub fn new(status: ConnectStatus, version: u16) -> Self {
        ConnectResponse {
            status: status.to_u8(),
            version: U16::new(version),
        }
    }
}

/// `Move::Deviation` — three doubles.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct Deviation {
    /// Maximum translational deviation from the commanded path, in m.
    pub translation: F64,
    /// Maximum rotational deviation from the commanded path, in rad.
    pub rotation: F64,
    /// Maximum elbow deviation from the commanded path, in rad.
    pub elbow: F64,
}

impl Deviation {
    /// Builds a deviation triple.
    pub fn new(translation: f64, rotation: f64, elbow: f64) -> Self {
        Deviation {
            translation: F64::new(translation),
            rotation: F64::new(rotation),
            elbow: F64::new(elbow),
        }
    }
}

/// `Move::Request` — 113 bytes.
///
/// libfranka always transmits the whole struct, including the asynchronous-motion-generator
/// tail (`use_async_motion_generator` + `maximum_velocity`), even when the feature is unused.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct MoveRequest {
    /// [`MoveControllerMode`] as a `u32`.
    pub controller_mode: U32,
    /// [`MoveMotionGeneratorMode`] as a `u32`.
    pub motion_generator_mode: U32,
    /// Allowed deviation from the commanded path while the motion runs.
    pub maximum_path_deviation: Deviation,
    /// Allowed deviation from the commanded goal pose.
    pub maximum_goal_pose_deviation: Deviation,
    /// C++ `bool`.
    pub use_async_motion_generator: u8,
    /// Only meaningful when `use_async_motion_generator` is set.
    pub maximum_velocity: [F64; 7],
}

impl MoveRequest {
    /// Builds the request libfranka's `Robot::Impl::startMotion` sends: no asynchronous motion
    /// generator, zero maximum velocities.
    pub fn new(
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
        maximum_path_deviation: Deviation,
        maximum_goal_pose_deviation: Deviation,
    ) -> Self {
        MoveRequest {
            controller_mode: U32::new(controller_mode.to_u32()),
            motion_generator_mode: U32::new(motion_generator_mode.to_u32()),
            maximum_path_deviation,
            maximum_goal_pose_deviation,
            use_async_motion_generator: 0,
            maximum_velocity: [F64::new(0.0); 7],
        }
    }
}

/// `Move::Response` — a single status byte.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct MoveResponse {
    /// [`super::MoveStatus`] as a `u8`.
    pub status: u8,
}

/// `StopMove::Response` — a single status byte.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct StopMoveResponse {
    /// [`super::StopMoveStatus`] as a `u8`.
    pub status: u8,
}

/// `AutomaticErrorRecovery::Response` — a single status byte.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct AutomaticErrorRecoveryResponse {
    /// [`super::AutomaticErrorRecoveryStatus`] as a `u8`.
    pub status: u8,
}

/// The response of every getter/setter command (`ResponseBase` with a
/// [`super::GetterSetterStatus`]) — a single status byte.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct GetterSetterResponse {
    /// [`super::GetterSetterStatus`] as a `u8`.
    pub status: u8,
}

/// The response of a command that keeps `CommandBase::Status` — in FCI v10 that is
/// `GetRobotModel`, whose reply is this status byte followed by the URDF.
///
/// It is byte-identical to [`GetterSetterResponse`] but carries a different enum, which is why
/// it exists as its own type: status `2` means
/// `CommandRejectedDueToActivatedSafetyFunctions` here, not `InvalidArgumentRejected`.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct CommandResponse {
    /// [`super::CommandStatus`] as a `u8`.
    pub status: u8,
}

/// `SetCollisionBehavior::Request` — 416 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetCollisionBehaviorRequest {
    /// Lower joint torque contact thresholds during acceleration, in Nm.
    pub lower_torque_thresholds_acceleration: [F64; 7],
    /// Upper joint torque collision thresholds during acceleration, in Nm.
    pub upper_torque_thresholds_acceleration: [F64; 7],
    /// Lower joint torque contact thresholds at nominal speed, in Nm.
    pub lower_torque_thresholds_nominal: [F64; 7],
    /// Upper joint torque collision thresholds at nominal speed, in Nm.
    pub upper_torque_thresholds_nominal: [F64; 7],
    /// Lower Cartesian force contact thresholds during acceleration: N then Nm.
    pub lower_force_thresholds_acceleration: [F64; 6],
    /// Upper Cartesian force collision thresholds during acceleration: N then Nm.
    pub upper_force_thresholds_acceleration: [F64; 6],
    /// Lower Cartesian force contact thresholds at nominal speed: N then Nm.
    pub lower_force_thresholds_nominal: [F64; 6],
    /// Upper Cartesian force collision thresholds at nominal speed: N then Nm.
    pub upper_force_thresholds_nominal: [F64; 6],
}

impl SetCollisionBehaviorRequest {
    /// Builds the request from the eight threshold arrays, in the order of libfranka's
    /// `Robot::setCollisionBehavior`.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        lower_torque_thresholds_acceleration: &[f64; 7],
        upper_torque_thresholds_acceleration: &[f64; 7],
        lower_torque_thresholds_nominal: &[f64; 7],
        upper_torque_thresholds_nominal: &[f64; 7],
        lower_force_thresholds_acceleration: &[f64; 6],
        upper_force_thresholds_acceleration: &[f64; 6],
        lower_force_thresholds_nominal: &[f64; 6],
        upper_force_thresholds_nominal: &[f64; 6],
    ) -> Self {
        use crate::wire::f64s_to_wire;
        SetCollisionBehaviorRequest {
            lower_torque_thresholds_acceleration: f64s_to_wire(
                lower_torque_thresholds_acceleration,
            ),
            upper_torque_thresholds_acceleration: f64s_to_wire(
                upper_torque_thresholds_acceleration,
            ),
            lower_torque_thresholds_nominal: f64s_to_wire(lower_torque_thresholds_nominal),
            upper_torque_thresholds_nominal: f64s_to_wire(upper_torque_thresholds_nominal),
            lower_force_thresholds_acceleration: f64s_to_wire(lower_force_thresholds_acceleration),
            upper_force_thresholds_acceleration: f64s_to_wire(upper_force_thresholds_acceleration),
            lower_force_thresholds_nominal: f64s_to_wire(lower_force_thresholds_nominal),
            upper_force_thresholds_nominal: f64s_to_wire(upper_force_thresholds_nominal),
        }
    }
}

/// `SetJointImpedance::Request` — 56 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetJointImpedanceRequest {
    /// Joint impedance stiffness, in Nm/rad.
    pub K_theta: [F64; 7],
}

impl SetJointImpedanceRequest {
    /// Builds the request.
    pub fn new(K_theta: &[f64; 7]) -> Self {
        SetJointImpedanceRequest {
            K_theta: crate::wire::f64s_to_wire(K_theta),
        }
    }
}

/// `SetCartesianImpedance::Request` — 48 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetCartesianImpedanceRequest {
    /// Cartesian impedance stiffness: N/m for translation, Nm/rad for rotation.
    pub K_x: [F64; 6],
}

impl SetCartesianImpedanceRequest {
    /// Builds the request.
    pub fn new(K_x: &[f64; 6]) -> Self {
        SetCartesianImpedanceRequest {
            K_x: crate::wire::f64s_to_wire(K_x),
        }
    }
}

/// `SetGuidingMode::Request` — 7 bytes (six C++ `bool`s plus the nullspace flag).
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetGuidingModeRequest {
    /// C++ `bool`s: which of the six Cartesian directions hand guiding may move in.
    pub guiding_mode: [u8; 6],
    /// C++ `bool`: whether the elbow may move in the nullspace.
    pub nullspace: u8,
}

impl SetGuidingModeRequest {
    /// Builds the request.
    pub fn new(guiding_mode: &[bool; 6], nullspace: bool) -> Self {
        let mut modes = [0u8; 6];
        for (m, &g) in modes.iter_mut().zip(guiding_mode.iter()) {
            *m = u8::from(g);
        }
        SetGuidingModeRequest {
            guiding_mode: modes,
            nullspace: u8::from(nullspace),
        }
    }
}

/// `SetEEToK::Request` — 128 bytes, column-major 4x4 transform.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetEEToKRequest {
    /// End-effector-to-stiffness-frame transform, column-major 4x4 (m).
    pub EE_T_K: [F64; 16],
}

impl SetEEToKRequest {
    /// Builds the request.
    pub fn new(EE_T_K: &[f64; 16]) -> Self {
        SetEEToKRequest {
            EE_T_K: crate::wire::f64s_to_wire(EE_T_K),
        }
    }
}

/// `SetNEToEE::Request` — 128 bytes, column-major 4x4 transform.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetNEToEERequest {
    /// Nominal-end-effector-to-end-effector transform, column-major 4x4 (m).
    pub NE_T_EE: [F64; 16],
}

impl SetNEToEERequest {
    /// Builds the request.
    pub fn new(NE_T_EE: &[f64; 16]) -> Self {
        SetNEToEERequest {
            NE_T_EE: crate::wire::f64s_to_wire(NE_T_EE),
        }
    }
}

/// `SetLoad::Request` — 104 bytes.
#[derive(Debug, Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C, packed)]
pub struct SetLoadRequest {
    /// Mass of the load, in kg.
    pub m_load: F64,
    /// Centre of mass of the load in flange frame, in m.
    pub F_x_Cload: [F64; 3],
    /// Inertia matrix of the load about its centre of mass, column-major 3x3, in kg*m^2.
    pub I_load: [F64; 9],
}

impl SetLoadRequest {
    /// Builds the request.
    pub fn new(m_load: f64, F_x_Cload: &[f64; 3], I_load: &[f64; 9]) -> Self {
        SetLoadRequest {
            m_load: F64::new(m_load),
            F_x_Cload: crate::wire::f64s_to_wire(F_x_Cload),
            I_load: crate::wire::f64s_to_wire(I_load),
        }
    }
}
