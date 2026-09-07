//! A pure-Rust client for the Franka Control Interface (FCI), the 1 kHz realtime protocol a
//! Franka robot's control box speaks over Ethernet.
//!
//! # Supported robots
//! * **Franka Research 3** — FCI v10, mirroring libfranka 0.21.2.
//! * **Franka Emika Robot (FER)** — FCI v5, mirroring libfranka 0.9.2.
//!
//! The version is negotiated during the `Connect` handshake ([`VersionPolicy`]): [`Robot`]
//! probes v10 first and falls back to v5 when the control box rejects it and reports version 5,
//! so the same binary drives either arm. Every byte-level difference is funnelled through
//! [`wire::robot::codec`]; nothing above it names a protocol version.
//!
//! # Three ways to control the robot
//! 1. **Callbacks** — [`Robot::control_joint_positions`] and its siblings run libfranka's
//!    `ControlLoop`: your closure is called once per millisecond and returns the next setpoint
//!    (see the [`robot`] module).
//! 2. **`ActiveControl`** — [`Robot::start_torque_control`] and its siblings hand back a
//!    [`ActiveTorqueControl`] / [`ActiveMotionGenerator`] handle you drive yourself with
//!    `read_once` / `write_once` (see [`robot::active_control`]).
//! 3. **Read-only** — [`Robot::read_once`] and [`Robot::read`] stream [`RobotState`] without
//!    commanding anything.
//!
//! # Model and gripper
//! [`Model`] answers forward kinematics, both Jacobians, the mass matrix, Coriolis and gravity.
//! It runs on a native URDF-based backend by default and can also load the robot's own
//! `libfcimodels` shared object ([`Robot::load_model_from_robot`]). [`Gripper`] is the separate
//! Franka Hand client, on its own TCP/UDP session.
//!
//! # Realtime requirements
//! A control loop must complete within 1 ms. [`RealtimeConfig::Enforce`] (libfranka's default)
//! raises the calling thread to `SCHED_FIFO` priority 40 and fails when it cannot; run on a
//! `PREEMPT_RT` kernel with the process allowed to lock memory and to set realtime priorities.
//! [`RealtimeConfig::Ignore`] downgrades that to a warning, which is what the simulator tests
//! and the examples use.
//!
//! # Example
//! ```no_run
//! use franka::{ControllerMode, JointPositions, RealtimeConfig, Robot, DEFAULT_CUTOFF_FREQUENCY};
//!
//! # fn main() -> franka::FrankaResult<()> {
//! let robot = Robot::new("192.168.0.1", RealtimeConfig::Enforce)?;
//! let initial = robot.read_once()?.q_d;
//! let mut time = 0.0;
//! robot.control_joint_positions(
//!     |_state, period| {
//!         time += period.as_secs_f64();
//!         let mut q = initial;
//!         q[3] += std::f64::consts::PI / 8.0 * (1.0 - (std::f64::consts::PI / 2.5 * time).cos());
//!         let mut output = JointPositions::new(q);
//!         output.motion_finished = time >= 5.0;
//!         output
//!     },
//!     ControllerMode::JointImpedance,
//!     true,
//!     DEFAULT_CUTOFF_FREQUENCY,
//! )?;
//! # Ok(())
//! # }
//! ```
//!
//! # Credit
//! This crate is an independent implementation whose API shape was informed by
//! [marcbone's libfranka-rs](https://github.com/marcbone/libfranka-rs) (no code shared);
//! [libfranka](https://github.com/frankarobotics/libfranka) itself is the reference every
//! type, constant, error text and wire layout here is checked against; the doc comments name
//! the C++ counterpart wherever there is one.
//!
//! # Module layout
//! Following libfranka: [`wire`] (packed protocol structs), [`network`] (the TCP command
//! channel with command-id demultiplexing and the UDP state/command channel), [`robot`] (the
//! public [`Robot`] API, control loops and `ActiveControl`), [`rate_limiting`], [`model`],
//! [`gripper`].

#![warn(missing_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![allow(non_snake_case)]

pub mod control_types;
pub mod duration;
pub mod error;
pub mod errors;
pub mod gripper;
pub mod joint_velocity_limits;
pub mod lowpass_filter;
pub(crate) mod math_utils;
pub mod model;
pub mod network;
pub mod rate_limiting;
pub mod realtime;
pub mod robot;
pub mod robot_state;
pub mod wire;

// -- Re-exports of the public API, mirroring `#include <franka/...>` in libfranka ------------

pub use control_types::{
    motion_finished, CartesianPose, CartesianVelocities, ControllerMode, Finishable,
    JointPositions, JointVelocities, MotionGenerator, MotionGeneratorKind, Torques,
};
pub use duration::Duration;
pub use error::{ControlException, FrankaError, FrankaResult, MoveStatus, Record, RobotCommandLog};
pub use errors::{Errors, ERROR_NAMES};
pub use gripper::{Gripper, GripperState};
pub use joint_velocity_limits::JointVelocityLimitsConfig;
pub use lowpass_filter::{
    cartesian_low_pass_filter, low_pass_filter, DEFAULT_CUTOFF_FREQUENCY, MAX_CUTOFF_FREQUENCY,
};
pub use model::{Frame, Model};
/// libfranka puts the `limitRate` overloads and the `kMax*` rate constants of the **FR3**
/// (FCI v10) in the bare `franka::` namespace (`include/franka/rate_limiting.h`), so they are
/// re-exported here; every item also keeps its `franka::rate_limiting::` path.
///
/// The FER (FCI v5) constants are deliberately **not** part of this list: they live at
/// `franka::rate_limiting::fer::*` only, so that `franka::MAX_JOINT_ACCELERATION` and its
/// neighbours unambiguously mean the FR3's and the two envelopes can never be mixed up at a
/// glance. The control loop picks between them from the negotiated [`FciVersion`]; user code
/// rarely needs either.
pub use rate_limiting::{
    compute_lower_limits_joint_velocity, compute_upper_limits_joint_velocity,
    limit_rate_cartesian_pose, limit_rate_cartesian_velocity, limit_rate_joint_position,
    limit_rate_joint_positions, limit_rate_joint_velocities, limit_rate_joint_velocity,
    limit_rate_torques, DELTA_T, FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE,
    JOINT_VELOCITY_LIMITS_TOLERANCE, LIMIT_EPS, MAX_ELBOW_ACCELERATION, MAX_ELBOW_JERK,
    MAX_ELBOW_VELOCITY, MAX_JOINT_ACCELERATION, MAX_JOINT_JERK, MAX_ROTATIONAL_ACCELERATION,
    MAX_ROTATIONAL_JERK, MAX_ROTATIONAL_VELOCITY, MAX_TORQUE_RATE, MAX_TRANSLATIONAL_ACCELERATION,
    MAX_TRANSLATIONAL_JERK, MAX_TRANSLATIONAL_VELOCITY, MIN_ELBOW_VELOCITY, NORM_EPS,
    TOL_NUMBER_PACKETS_LOST,
};
pub use realtime::RealtimeConfig;
pub use robot::{
    ActiveMotionGenerator, ActiveMotionInput, ActiveTorqueControl, Robot, RobotOptions,
    VersionPolicy, VirtualWallCuboid,
};
pub use robot_state::{RobotMode, RobotState};
/// The FCI protocol version a connection speaks: v5 (FER) or v10 (FR3).
pub use wire::robot::codec::FciVersion;
/// The controller a motion is started with; unlike [`ControllerMode`] this includes the
/// external controller, which is what `Robot::start_*_control` needs.
pub use wire::robot::MoveControllerMode;

/// Checks that the README's "Quick example" block and `examples/readme_joint_move.rs` cannot
/// drift apart, and that the block stays within its 20-line budget.
///
/// This lives in the library (rather than in `tests/`) so that it runs as part of the
/// Docker-free `cargo test -p franka-rs --lib` that CI's `check` job executes.
#[cfg(test)]
mod readme_sync;
