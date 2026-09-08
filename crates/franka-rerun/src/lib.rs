//! Replays of `franka-rs` logs as [Rerun](https://rerun.io) recordings, and a live recorder.
//!
//! The pieces are independent of where the data comes from: [`series`] logs time series from
//! slices of times and positions, [`scene`] logs the 3D arm from a [`franka::Model`] and joint
//! angles, [`commander`] reads the CSV that `examples/nonrealtime_commander.rs --log` writes
//! and records all of it, [`flight`] replays a control log (`Vec<franka::Record>`, what a
//! [`franka::ControlException`] carries) as a flight recording -- contact and collision flags,
//! external wrench, commanded versus measured, errors, and where on the arm the external
//! torques say it was touched ([`flight::contact`]) -- [`recorder`] streams the same from
//! inside a running control loop, [`meshes`] draws the arm with Franka's link meshes, and
//! [`demo`] is a commander layout made for a screen capture. The `franka-rerun` binary is
//! the command line over the two replays.
//!
//! Everything is logged on the [`TIMELINE`] `robot_time`: seconds since the motion started
//! for a commander CSV, the robot's own clock for a control log.

pub mod commander;
pub mod demo;
pub mod flight;
pub mod meshes;
pub mod recorder;
pub mod scene;
pub mod series;

pub use commander::CommanderLog;
pub use flight::{
    load_records, log_records, replay_exception, save_records, ContactEstimate, ContactOptions,
    FlightLogger, FlightOptions, Summary,
};
pub use meshes::Meshes;
pub use recorder::{Recorder, RecorderOptions, Stats};
pub use series::{Limits, Peaks};

/// The timeline every entity is logged on: the robot's time, seconds since the motion started.
pub const TIMELINE: &str = "robot_time";

/// Series colours as `0xRRGGBBAA`: target orange, commanded blue, measured green (the same
/// three as `bench/commander/plot.py`), limit grey, and budget red.
pub const TARGET: u32 = 0xeb68_34ff;
pub const COMMANDED: u32 = 0x2a78_d6ff;
pub const MEASURED: u32 = 0x1baf_7aff;
pub const LIMIT: u32 = 0x9a9a_9aff;
/// The red of the budget lines in the demo layout.
pub const BUDGET: u32 = 0xe548_4dff;

/// `Box<dyn Error>` for everything: the SDK's serialization, stream and sink errors, the
/// model's, and the CSV's.
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Which robot's limits and model apply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotKind {
    /// Franka Research 3 (FCI v10).
    Fr3,
    /// Franka Emika Robot / Panda (FCI v5).
    Fer,
}

impl From<franka::FciVersion> for RobotKind {
    /// The robot behind a negotiated FCI version: v10 is an FR3, v5 an FER.
    fn from(version: franka::FciVersion) -> RobotKind {
        match version {
            franka::FciVersion::V10 => RobotKind::Fr3,
            franka::FciVersion::V5 => RobotKind::Fer,
        }
    }
}

impl RobotKind {
    /// The rate limiter's translational limits for this robot.
    pub fn limits(self) -> Limits {
        match self {
            RobotKind::Fr3 => Limits {
                speed: franka::MAX_TRANSLATIONAL_VELOCITY,
                acceleration: franka::MAX_TRANSLATIONAL_ACCELERATION,
                jerk: franka::MAX_TRANSLATIONAL_JERK,
            },
            RobotKind::Fer => Limits {
                speed: franka::rate_limiting::fer::MAX_TRANSLATIONAL_VELOCITY,
                acceleration: franka::rate_limiting::fer::MAX_TRANSLATIONAL_ACCELERATION,
                jerk: franka::rate_limiting::fer::MAX_TRANSLATIONAL_JERK,
            },
        }
    }
}

/// Euclidean norm.
pub fn norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Euclidean distance between two positions.
pub fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    norm(&[a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}
