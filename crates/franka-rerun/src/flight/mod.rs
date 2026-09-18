//! The flight recorder: a control log (`Vec<franka::Record>`, what a
//! [`franka::ControlException`] carries) replayed as a Rerun recording, so that what happened
//! before a reflex can be scrubbed through -- which joint reported contact or collision, the
//! estimated external wrench, commanded versus measured, and the error that fired.
//!
//! [`log_records`] writes one log to a stream, [`replay_exception`] wraps it for an exception,
//! [`save_records`] / [`load_records`] move a log through JSON, and [`FlightLogger`] is the
//! streaming core the [`crate::Recorder`] drives live. Everything is on the [`crate::TIMELINE`]
//! `robot_time`: `state.time` in seconds (the robot's millisecond counter), or the record index
//! in milliseconds when `time` never changes over the log. A record that carries the host's
//! clock ([`Stamped::host_ns`], which only a live [`crate::Recorder`] has) is logged on
//! [`crate::HOST_TIMELINE`] as well.
//!
//! What a recording holds, every entity under [`FlightOptions::prefix`] when there is one:
//!
//! | entity | content |
//! |---|---|
//! | `joints/q`, `joints/q_d` | measured joint positions; commanded ones (the sent command's `q_c` when the cycle sent a joint command, else the robot's `q_d`) |
//! | `joints/dq`, `joints/tau_J`, `joints/tau_J_d`, `joints/tau_ext` | joint velocities, measured torques, desired torques, `tau_ext_hat_filtered` |
//! | `joints/q_goal`, `joints/dq_goal`, `joints/cap_scale`, `joints/tau_envelope` | for records pushed with a [`TorqueLog`]: target control's joint goal and its velocity, the scale the joint velocity cap cut the goal's step by (1 when it did not), the velocity envelope's torque: the barrier's, less the law's torque faded out along a joint's motion above the fade's start |
//! | `ee/F_ext` | `O_F_ext_hat_K`: force (N) and torque (Nm) on the stiffness frame in the base frame |
//! | `ee/position` | measured `O_T_EE` translation (`x`, `y`, `z`) against the commanded `O_T_EE_c` (`x_c`, ...) |
//! | `ee/orientation` | the rotations of the same two poses as quaternions `xyzw` (`qx` .. `qw`, `qx_c` .. `qw_c`), every cycle |
//! | `ee/position/{x,y,z}`, `ee/derivatives/{speed,acceleration,jerk}` | for records whose command carries an `O_T_EE_c`: the sent position against the measured one per axis, and the norms of its finite differences against the limits (see [`cartesian`]) |
//! | `flags/joint_contact`, `flags/joint_collision` | seven 0/1 series each, amber and red |
//! | `flags/cartesian_contact`, `flags/cartesian_collision` | six 0/1 series each (`Fx`..`Tz`) |
//! | `world/*` | the arm, a sphere per joint (grey, amber on contact, red on collision, growing with `\|tau_ext\|`), the external force as an arrow from the end effector, the end effector axes, a faint workspace box, the sent position of a Cartesian command |
//! | `world/links/*` | unless [`FlightOptions::meshes`] is off: Franka's link meshes on the link frames (see [`crate::meshes`]) |
//! | `world/contact/estimate`, `world/contact/force` | where the external joint torques say the arm is being touched, and the force there (see [`contact`]); present while a flag is set or a torque exceeds the noise floor |
//! | `contact/link` | the estimated link over time |
//! | `events` | every change of `current_errors` and `robot_mode`, the first rising edge of every flag, the first contact estimate and the one at the collision, and `motion aborted: ...` at the end |

mod blueprint;
pub mod cartesian;
pub mod contact;
mod logger;
mod skeleton;
mod style;
mod tare;
mod view;

use std::fmt;
use std::path::Path;

use franka::{ControlException, Errors, Model, Record};
use rerun::RecordingStream;

pub use blueprint::{send_blueprint, send_commander_blueprint, Layout};
pub use cartesian::{log_gripper_styles, log_target_styles};
pub use contact::{ContactEstimate, ContactOptions};
pub use logger::FlightLogger;
pub use style::{COLLISION, CONTACT, QUIET};
pub use tare::Tare;

use crate::series::Peaks;
use crate::{MeshChoice, Prefix, Result, RobotKind};

/// `ee/orientation`: the measured and the commanded end effector rotation as quaternions
/// `xyzw`, every cycle.
pub const ORIENTATION: &str = "ee/orientation";

/// A record with the host's clock at the moment it was taken: what the live
/// [`crate::Recorder`] hands its writer thread, and what puts a row on
/// [`crate::HOST_TIMELINE`] beside [`crate::TIMELINE`].
#[derive(Debug, Clone, PartialEq)]
pub struct Stamped {
    pub record: Record,
    /// The host's `CLOCK_MONOTONIC` when the record was taken, ns. `None` for a log replayed
    /// offline: it carries the robot's clock and nothing else, and a stamp taken at the replay
    /// would be a fiction.
    pub host_ns: Option<u64>,
    /// What target control's torque backend made of the cycle, when the producer had it
    /// ([`crate::Recorder::push_torque_at`]); `None` otherwise.
    pub torque: Option<TorqueLog>,
}

impl From<Record> for Stamped {
    /// A record with no host clock.
    fn from(record: Record) -> Stamped {
        Stamped {
            record,
            host_ns: None,
            torque: None,
        }
    }
}

impl Stamped {
    /// A record taken at `host_ns` on the host's `CLOCK_MONOTONIC`.
    pub fn at(record: Record, host_ns: u64) -> Stamped {
        Stamped {
            record,
            host_ns: Some(host_ns),
            torque: None,
        }
    }

    /// [`Stamped::host_ns`] in seconds.
    pub fn host_secs(&self) -> Option<f64> {
        self.host_ns.map(|ns| ns as f64 * 1e-9)
    }
}

/// What the torque backend of `franka`'s target control made of one cycle, from its observer's
/// record: the joint goal, its velocity, the scale the joint velocity cap cut the goal's step
/// by (1 when it did not) and the velocity envelope's torque: the barrier's, less the law's
/// torque faded out along a joint's motion above the fade's start.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TorqueLog {
    pub q_goal: [f64; 7],
    pub dq_goal: [f64; 7],
    pub cap_scale: f64,
    pub tau_envelope: [f64; 7],
}

/// How a log is drawn.
#[derive(Debug, Clone, PartialEq)]
pub struct FlightOptions {
    /// What every entity path is put under, so that several robots can write into one
    /// recording. Empty by default; the node sets it to the arm's name.
    pub prefix: Prefix,
    /// Length of the external force arrow, m per N. 0.01 draws 10 N as 10 cm.
    pub force_scale: f64,
    /// Log every `every`-th record to the 3D scene (the series always carry every record).
    /// The contact estimate follows it too, except that a record with a contact or collision
    /// flag set is always estimated.
    pub every: usize,
    /// The link meshes the arm is drawn with besides the skeleton: the built-in set by default
    /// (feature `builtin-meshes`).
    pub meshes: MeshChoice,
    /// The contact estimator's noise floor and grid.
    pub contact: ContactOptions,
}

impl Default for FlightOptions {
    fn default() -> Self {
        FlightOptions {
            prefix: Prefix::none(),
            force_scale: 0.01,
            every: 1,
            meshes: MeshChoice::default(),
            contact: ContactOptions::default(),
        }
    }
}

/// What a replay recorded, for printing.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Summary {
    /// Records written.
    pub records: usize,
    /// Timeline value of the first and the last record, s.
    pub first_time: f64,
    pub last_time: f64,
    /// Rising edges of the flags, summed over the joints or axes.
    pub joint_contacts: usize,
    pub joint_collisions: usize,
    pub cartesian_contacts: usize,
    pub cartesian_collisions: usize,
    /// Records on which `current_errors` or `robot_mode` differed from the previous one.
    pub error_changes: usize,
    pub mode_changes: usize,
    /// Largest `|O_F_ext_hat_K|` force norm, N, and when.
    pub peak_force: f64,
    pub peak_force_time: f64,
    /// Largest `|tau_ext_hat_filtered|`, Nm, and on which joint (1-based; 0 if never above 0).
    pub peak_tau_ext: f64,
    pub peak_tau_ext_joint: usize,
    /// Lines written to `events`.
    pub events: usize,
    /// The names in the `motion aborted` line, empty when there was none.
    pub motion_errors: Vec<&'static str>,
    /// Records that got a contact estimate, and the last estimate.
    pub contact_estimates: usize,
    pub last_contact: Option<ContactEstimate>,
    /// The peak speed, acceleration and jerk norms of the sent position, when any record
    /// carried a Cartesian command (see [`cartesian`]).
    pub commanded_peaks: Option<Peaks>,
}

impl Summary {
    /// Seconds between the first and the last record.
    pub fn span(&self) -> f64 {
        self.last_time - self.first_time
    }
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "{} records over {:.3} s (robot_time {:.3} .. {:.3}), {} events",
            self.records,
            self.span(),
            self.first_time,
            self.last_time,
            self.events
        )?;
        writeln!(
            f,
            "contacts: {} joint, {} cartesian; collisions: {} joint, {} cartesian; {} error \
             change(s), {} mode change(s)",
            self.joint_contacts,
            self.cartesian_contacts,
            self.joint_collisions,
            self.cartesian_collisions,
            self.error_changes,
            self.mode_changes
        )?;
        write!(
            f,
            "peak |F_ext| {:.2} N at robot_time {:.3}; peak |tau_ext| {:.2} Nm on joint {}",
            self.peak_force, self.peak_force_time, self.peak_tau_ext, self.peak_tau_ext_joint
        )?;
        if !self.motion_errors.is_empty() {
            write!(f, "\nmotion aborted: {}", self.motion_errors.join(", "))?;
        }
        if let Some(peaks) = &self.commanded_peaks {
            write!(
                f,
                "\npeak sent position derivatives: {:.3} m/s, {:.2} m/s^2, {:.0} m/s^3",
                peaks.speed, peaks.acceleration, peaks.jerk
            )?;
        }
        if let Some(contact) = &self.last_contact {
            let [x, y, z] = contact.point;
            let [fx, fy, fz] = contact.force;
            write!(
                f,
                "\nlast of {} contact estimates: {contact}; point [{x:.3}, {y:.3}, {z:.3}] m, \
                 force [{fx:.1}, {fy:.1}, {fz:.1}] N",
                self.contact_estimates
            )?;
        }
        Ok(())
    }
}

/// Writes `records` to `rec` (series, 3D scene, events; see the module documentation) and,
/// if `last_motion_errors` has a flag set, a `motion aborted` line at the last record. Does
/// not send a blueprint; see [`send_blueprint`].
pub fn log_records(
    rec: &RecordingStream,
    records: &[Record],
    model: &Model,
    kind: RobotKind,
    options: &FlightOptions,
    last_motion_errors: Option<&Errors>,
) -> Result<Summary> {
    let mut logger = FlightLogger::new(rec, model, kind, options.clone())?;
    // Decided here rather than per batch: the record index stands in for a `time` that never
    // changes over the *whole* log, and one batch of it cannot tell that from a log whose time
    // is merely slow to move.
    let first = records[0].state.time;
    let constant = records.len() >= 2 && records.iter().all(|r| r.state.time == first);
    logger.set_timebase(constant);
    // In chunks, so that a long log is not copied into one second `Vec` of stamped records.
    let mut batch: Vec<Stamped> = Vec::with_capacity(BATCH.min(records.len()));
    for chunk in records.chunks(BATCH) {
        batch.clear();
        batch.extend(chunk.iter().cloned().map(Stamped::from));
        logger.log_batch(&batch)?;
    }
    logger.finish(last_motion_errors)
}

/// Records [`log_records`] hands the logger at a time.
const BATCH: usize = 1024;

/// Writes `exception.log` to the `.rrd` at `path`, with `exception.last_motion_errors` as the
/// abort line and the default layout.
pub fn replay_exception(
    path: &Path,
    exception: &ControlException,
    model: &Model,
    kind: RobotKind,
    options: &FlightOptions,
) -> Result<Summary> {
    if exception.log.is_empty() {
        return Err(format!(
            "{}: the exception carries no control log (was the robot opened with a log size \
             of 0?)",
            path.display()
        )
        .into());
    }
    let rec = rerun::RecordingStreamBuilder::new(crate::APPLICATION_ID).save(path)?;
    let errors = Some(&exception.last_motion_errors);
    let summary = log_records(&rec, &exception.log, model, kind, options, errors)?;
    send_blueprint(&rec, &Layout::single(options.prefix.clone()))?;
    rec.flush_blocking()?;
    Ok(summary)
}

/// Saves a control log as a JSON array of [`franka::Record`]s (the `serde` feature of
/// `franka-rs` defines the layout; `Errors` are lists of names, `time` is milliseconds).
pub fn save_records(path: &Path, records: &[Record]) -> Result<()> {
    let file = std::fs::File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut writer = std::io::BufWriter::new(file);
    serde_json::to_writer(&mut writer, records).map_err(|e| format!("{}: {e}", path.display()))?;
    std::io::Write::flush(&mut writer).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(())
}

/// Reads a control log written by [`save_records`].
pub fn load_records(path: &Path) -> Result<Vec<Record>> {
    let file = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let reader = std::io::BufReader::new(file);
    let records: Vec<Record> =
        serde_json::from_reader(reader).map_err(|e| format!("{}: {e}", path.display()))?;
    if records.is_empty() {
        return Err(format!("{}: no records", path.display()).into());
    }
    Ok(records)
}
