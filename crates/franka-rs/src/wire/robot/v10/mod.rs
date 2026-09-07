//! FCI version 10 wire structs, ported from libfranka 0.21.2
//! (`common/include/research_interface/robot/{service_types.h,rbk_types.h}`).

pub mod rbk_types;
pub mod service_types;

pub use rbk_types::*;
pub use service_types::*;
