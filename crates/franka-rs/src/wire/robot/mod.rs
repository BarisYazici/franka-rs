//! Robot protocol wire structs, one module per FCI version.
//!
//! FCI version 10 (Franka Research 3, libfranka 0.21.2) lives in [`v10`] and is re-exported
//! here, so that the rest of the crate can refer to `wire::robot::RobotState` without naming a
//! version. FCI version 5 (Franka Emika Robot, FER, libfranka 0.9.2) lives in [`v5`] and is always
//! named explicitly.
//!
//! [`codec`] is the seam between the two: it turns an [`codec::FciVersion`] plus
//! version-agnostic arguments into the right bytes, and version-specific bytes back into the
//! shared Rust types.

pub mod codec;
pub mod v10;
pub mod v5;

pub use v10::*;
