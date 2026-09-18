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
//! for a commander CSV, the robot's own clock for a control log. A record taken live carries
//! the host's clock as well and is logged on [`HOST_TIMELINE`] too; that is the axis on which
//! two robots, each with its own controller and so its own `robot_time`, line up. A [`Prefix`]
//! puts everything one recorder writes under a name, which is what lets them share a recording.

pub mod commander;
pub mod demo;
pub mod flight;
pub mod meshes;
mod prefix;
pub mod recorder;
pub mod scene;
pub mod series;

pub use commander::CommanderLog;
pub use flight::{
    load_records, log_records, replay_exception, save_records, ContactEstimate, ContactOptions,
    FlightLogger, FlightOptions, Layout, Stamped, Summary, TorqueLog,
};
pub use meshes::{MeshChoice, Meshes};
pub use prefix::Prefix;
pub use recorder::{Recorder, RecorderOptions, Stats};
/// The SDK behind [`Recorder::stream`], at the version this crate pins, for dependants that
/// log their own entities.
pub use rerun;
pub use series::{Limits, Peaks};

/// The timeline every entity is logged on: the robot's time, seconds since the motion started.
///
/// It is the controller's own millisecond counter, so two robots' values are unrelated: a
/// recording of two of them is synchronised on [`HOST_TIMELINE`], not on this one.
pub const TIMELINE: &str = "robot_time";

/// The second timeline of a live recording: the host's `CLOCK_MONOTONIC` in seconds, stamped
/// where the record was taken ([`Recorder::push`]).
///
/// Every process on one host reads the same clock, so an arm, a second arm and a camera node
/// all put their rows on one axis without exchanging anything. An offline replay has no host
/// clock for a record and writes only [`TIMELINE`].
pub const HOST_TIMELINE: &str = "host_time";

/// The Rerun application id of every stream this crate opens.
///
/// A viewer keys a store by the application id *and* the recording id, so a second process
/// that wants its file to load as part of one of ours has to name this same id alongside the
/// recording id it was given.
pub const APPLICATION_ID: &str = "franka_rs";

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
