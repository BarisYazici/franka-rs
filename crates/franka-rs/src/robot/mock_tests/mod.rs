//! Unit tests for [`RobotImpl`] against a loopback mock FCI server.
//!
//! Ports of the cases in libfranka 0.21.2 `test/robot_impl_tests.cpp` that do not depend on the
//! internals of the C++ `MockServer`. The mock here answers `Connect` and `GetRobotModel`
//! automatically, lets a test queue or schedule TCP responses per command, and sends robot
//! states on demand.
//!
//! The mock server itself lives in [`server`]; the cases are grouped by what they exercise:
//! [`motion`] (the FCI v10 motion lifecycle), [`torque`] (torque-only control and the
//! cancel-on-drop paths), [`fer`] (FCI v5) and [`model_library`].

mod fer;
#[cfg(feature = "model-library")]
mod model_library;
mod motion;
mod server;
mod torque;

use server::MockServer;

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration as StdDuration;

use zerocopy::{FromBytes, IntoBytes};

use crate::error::{FrankaError, MoveStatus};
use crate::realtime::RealtimeConfig;
use crate::robot::robot_impl::RobotImpl;
use crate::robot::VersionPolicy;
use crate::wire::robot::codec::{self, CommandKind, FciVersion};
use crate::wire::robot::v5;
use crate::wire::robot::{
    Command, ConnectStatus, ControllerMode as StateControllerMode, Deviation,
    MotionGeneratorCommand, MotionGeneratorMode as StateMotionGeneratorMode, MoveControllerMode,
    MoveMotionGeneratorMode, RobotCommand, RobotMode as WireRobotMode,
    RobotState as WireRobotState, StopMoveStatus,
};
use crate::wire::HeaderLayout;

/// Minimal URDF carrying everything `JointVelocityLimitsConfig` needs.
const TEST_URDF: &str = r#"<?xml version="1.0"?>
<robot name="fr3">
  <joint name="fr3_joint1" type="revolute">
    <limit effort="87.0" lower="-2.7437" upper="2.7437" velocity="2.62"/>
    <position_based_velocity_limits deceleration_limit="6.0" velocity_offset="0.30"/>
  </joint>
  <joint name="fr3_joint2" type="revolute">
    <limit effort="87.0" lower="-1.7837" upper="1.7837" velocity="2.62"/>
    <position_based_velocity_limits deceleration_limit="2.585" velocity_offset="0.20"/>
  </joint>
  <joint name="fr3_joint3" type="revolute">
    <limit effort="87.0" lower="-2.9007" upper="2.9007" velocity="2.62"/>
    <position_based_velocity_limits deceleration_limit="6.0" velocity_offset="0.20"/>
  </joint>
  <joint name="fr3_joint4" type="revolute">
    <limit effort="87.0" lower="-3.0421" upper="-0.1518" velocity="2.62"/>
    <position_based_velocity_limits deceleration_limit="2.585" velocity_offset="0.30"/>
  </joint>
  <joint name="fr3_joint5" type="revolute">
    <limit effort="12.0" lower="-2.8065" upper="2.8065" velocity="5.26"/>
    <position_based_velocity_limits deceleration_limit="6.0" velocity_offset="0.35"/>
  </joint>
  <joint name="fr3_joint6" type="revolute">
    <limit effort="12.0" lower="0.5445" upper="4.5169" velocity="4.18"/>
    <position_based_velocity_limits deceleration_limit="6.0" velocity_offset="0.35"/>
  </joint>
  <joint name="fr3_joint7" type="revolute">
    <limit effort="12.0" lower="-3.0159" upper="3.0159" velocity="5.26"/>
    <position_based_velocity_limits deceleration_limit="6.0" velocity_offset="0.35"/>
  </joint>
</robot>
"#;

/// A state with everything idle.
fn idle_state() -> WireRobotState {
    WireRobotState {
        motion_generator_mode: StateMotionGeneratorMode::Idle.to_u8(),
        controller_mode: StateControllerMode::JointImpedance.to_u8(),
        robot_mode: WireRobotMode::Idle.to_u8(),
        ..WireRobotState::default()
    }
}

/// [`idle_state`] as an FCI v5 state (2373 bytes of `double`).
fn idle_state_v5() -> v5::RobotState {
    v5::RobotState {
        motion_generator_mode: StateMotionGeneratorMode::Idle.to_u8(),
        controller_mode: StateControllerMode::JointImpedance.to_u8(),
        robot_mode: WireRobotMode::Idle.to_u8(),
        ..v5::RobotState::default()
    }
}

/// A state reporting a running motion with the given modes.
fn moving_state(
    motion: StateMotionGeneratorMode,
    controller: StateControllerMode,
) -> WireRobotState {
    WireRobotState {
        motion_generator_mode: motion.to_u8(),
        controller_mode: controller.to_u8(),
        robot_mode: WireRobotMode::Move.to_u8(),
        ..WireRobotState::default()
    }
}

/// The deviations the C++ tests pass to `startMotion`.
fn deviation() -> Deviation {
    Deviation::new(0.0, 1.0, 2.0)
}

fn start_joint_velocity_motion(robot: &RobotImpl) -> crate::error::FrankaResult<u32> {
    robot.start_motion(
        MoveControllerMode::JointImpedance,
        MoveMotionGeneratorMode::JointVelocity,
        deviation(),
        deviation(),
    )
}
