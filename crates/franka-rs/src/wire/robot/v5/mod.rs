//! FCI version 5 wire structs (Franka Emika Robot, FER), ported from libfranka 0.9.2
//! (`common/include/research_interface/robot/{service_types.h,rbk_types.h}`).
//!
//! Differences from [`crate::wire::robot::v10`] at a glance:
//!
//! - the `Command` numbering differs (`GetCartesianLimit` and `SetFilters` exist,
//!   `GetRobotModel` does not, `LoadModelLibrary` replaces it),
//! - `Move::Request` is 56 bytes instead of 113 and has no `kNone` motion generator mode,
//! - every status enum is shorter (no safety-function values),
//! - the UDP `RobotState` is 2373 bytes of `double` with no accelerometers (v10: 1377 bytes of
//!   `float`), and `RobotCommand` is 370 bytes because `ControllerCommand` has no
//!   `torque_command_finished` flag.

pub mod rbk_types;
pub mod service_types;

pub use rbk_types::*;
pub use service_types::*;

/// FCI library version implemented by this module (`research_interface::robot::kVersion`,
/// `service_types.h:17`).
pub const ROBOT_VERSION: u16 = 5;
