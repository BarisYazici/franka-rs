//! Shared instrumentation for the `franka-rs` benchmark binaries.
//!
//! `bench_joint_velocity`, `bench_model_control` and `robot_probe` all need the same small
//! set of process-introspection helpers (a monotonic clock read, latency statistics, scheduler
//! and `mlockall` reporting, `getrusage` deltas). Rather than duplicate them per binary, they
//! live here and each binary pulls in only what it uses.

pub mod sched;
pub mod stats;
pub mod time;
