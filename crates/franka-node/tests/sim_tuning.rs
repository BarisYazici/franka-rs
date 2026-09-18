//! Live tuning against franka-sim, end to end: the node in process, `params/*` driven over
//! Zenoh, and every claim asserted against the torque the 1 kHz loop actually produced.
//!
//! The node is attached to a `RobotSide` delegate that installs a Cartesian observer on whatever
//! options the node builds (`tuning::tap`), so a cycle's `tau`, `q_goal` and measured state are
//! recorded without a line of production code changing. The oracle is the loop's own
//! `impedance_torques`, fed the gains parsed out of the node's reply JSON — so the *law* is not
//! under test here, the *gain path* is. `tuning::law` refuses any window on which that residual
//! would not discriminate.
//!
//! The suite costs **about 50 s and ten containers**, one per `#[test]`: the price of a failure
//! naming itself instead of dropping a wall of output on the reader. Its torque-side claims
//! discriminate to about ±8-10% of a gain — see `tuning/law.rs` — so a pass says the value
//! arrived, not that it arrived exactly.
//!
//! One FCI client slot per host. Nothing in this file enforces `--test-threads=1`: it is
//! `franka-sim-test`'s process-wide `LOCK`, held for a `SimServer`'s lifetime, that serialises
//! the containers. Run it as the other simulator suites are run.
//!
//! ```sh
//! FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-node \
//!   --test sim_tuning -- --test-threads=1 --nocapture
//! ```
//!
//! `--nocapture` only so a passing run prints the figures; the assertions do not need it.
//!
//! The bounds *sweep* — how each of `LiveTuning::BOUNDS`' ceilings behaves between its default
//! and its limit — is not here. It is a campaign script, `tests/sim_sweep.rs`, because it
//! measures rather than asserts.

#[path = "../examples/client/common.rs"]
pub mod client;

#[path = "tuning/arrival.rs"]
mod arrival;
#[path = "tuning/bounds.rs"]
mod bounds;
#[path = "tuning/contract.rs"]
mod contract;
#[path = "tuning/degradation.rs"]
mod degradation;
#[path = "tuning/law.rs"]
mod law;
#[path = "tuning/metrics.rs"]
mod metrics;
#[path = "tuning/params.rs"]
mod params;
#[path = "tuning/rig.rs"]
mod rig;
#[path = "tuning/table.rs"]
mod table;
#[path = "tuning/tap.rs"]
mod tap;

/// The simulated arm's name, the node's, and where the node listens.
pub const ARM: &str = "sim";
pub const NODE: &str = "tune-node";
pub const NODE_ENDPOINT: &str = "tcp/127.0.0.1:17451";
/// The client id every request in this suite carries; echoed back as `origin.by`.
pub const CLIENT: u32 = 7;
