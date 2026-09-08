//! The flight recorder: a control log (`Vec<franka::Record>`, what a
//! [`franka::ControlException`] carries) replayed as a Rerun recording, so that what happened
//! before a reflex can be scrubbed through -- which joint reported contact or collision, the
//! estimated external wrench, commanded versus measured, and the error that fired.
//!
//! [`log_records`] writes one log to a stream, [`replay_exception`] wraps it for an exception,
//! [`save_records`] / [`load_records`] move a log through JSON, and [`FlightLogger`] is the
//! streaming core the [`crate::Recorder`] drives live. Everything is on the [`TIMELINE`]
//! `robot_time`: `state.time` in seconds (the robot's millisecond counter), or the record index
//! in milliseconds when `time` never changes over the log.
//!
//! What a recording holds:
//!
//! | entity | content |
//! |---|---|
//! | `joints/q`, `joints/q_d` | measured joint positions; commanded ones (the sent command's `q_c` when the cycle sent a joint command, else the robot's `q_d`) |
//! | `joints/dq`, `joints/tau_J`, `joints/tau_J_d`, `joints/tau_ext` | joint velocities, measured torques, desired torques, `tau_ext_hat_filtered` |
//! | `ee/F_ext` | `O_F_ext_hat_K`: force (N) and torque (Nm) on the stiffness frame in the base frame |
//! | `ee/position` | measured `O_T_EE` translation (`x`, `y`, `z`) against the commanded `O_T_EE_c` (`x_c`, ...) |
//! | `flags/joint_contact`, `flags/joint_collision` | seven 0/1 series each, amber and red |
//! | `flags/cartesian_contact`, `flags/cartesian_collision` | six 0/1 series each (`Fx`..`Tz`) |
//! | `world/*` | the arm, a sphere per joint (grey, amber on contact, red on collision, growing with `\|tau_ext\|`), the external force as an arrow from the end effector, the end effector axes |
//! | `events` | every change of `current_errors` and `robot_mode`, the first rising edge of every flag, and `motion aborted: ...` at the end |

mod logger;
mod style;

use std::fmt;
use std::path::Path;

use franka::{ControlException, Errors, Model, Record};
use rerun::blueprint::{
    Blueprint, BlueprintActivation, Grid, Horizontal, Spatial3DView, Tabs, TextLogView, TimePanel,
    TimeSeriesView, Vertical,
};
use rerun::RecordingStream;

pub use logger::FlightLogger;
pub use style::{COLLISION, CONTACT, QUIET};

use crate::{Result, RobotKind, TIMELINE};

/// How a log is drawn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightOptions {
    /// Length of the external force arrow, m per N. 0.01 draws 10 N as 10 cm.
    pub force_scale: f64,
    /// Log every `every`-th record to the 3D scene (the series always carry every record).
    pub every: usize,
}

impl Default for FlightOptions {
    fn default() -> Self {
        FlightOptions {
            force_scale: 0.01,
            every: 1,
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
    let mut logger = FlightLogger::new(rec, model, kind, *options)?;
    logger.log_batch(records)?;
    logger.finish(last_motion_errors)
}

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
    let rec = rerun::RecordingStreamBuilder::new("franka_rs").save(path)?;
    let errors = Some(&exception.last_motion_errors);
    let summary = log_records(&rec, &exception.log, model, kind, options, errors)?;
    send_blueprint(&rec)?;
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

/// The default layout: the 3D scene on the left; the joint, end effector and flag plots on
/// the right (commanded and measured `q` share one plot, the torques and the flags are tabs);
/// the event log along the bottom; the time panel on `robot_time`.
pub fn send_blueprint(rec: &RecordingStream) -> Result<()> {
    let plot = |name: &str, origin: &str| TimeSeriesView::new(name).with_origin(origin);
    let q = TimeSeriesView::new("q vs q_d")
        .with_origin("joints")
        .with_contents(["+ $origin/q", "+ $origin/q_d"]);
    let velocity = plot("dq", "joints/dq");
    let torques = Tabs::new([
        plot("tau_ext", "joints/tau_ext").into(),
        plot("tau_J", "joints/tau_J").into(),
        plot("tau_J_d", "joints/tau_J_d").into(),
    ]);
    let flags = Tabs::new([
        plot("joint contact", "flags/joint_contact").into(),
        plot("joint collision", "flags/joint_collision").into(),
        plot("cartesian contact", "flags/cartesian_contact").into(),
        plot("cartesian collision", "flags/cartesian_collision").into(),
    ]);
    let plots = Grid::new([
        q.into(),
        torques.into(),
        plot("F_ext", "ee/F_ext").into(),
        plot("position", "ee/position").into(),
        velocity.into(),
        flags.into(),
    ])
    .with_grid_columns(2);
    let top = Horizontal::new([
        Spatial3DView::new("arm").with_origin("world").into(),
        plots.into(),
    ])
    .with_column_shares([2.0, 3.0]);
    let events = TextLogView::new("events").with_origin("events");
    let root = Vertical::new([top.into(), events.into()]).with_row_shares([4.0, 1.0]);
    Blueprint::new(root)
        .with_time_panel(TimePanel::new().with_timeline(TIMELINE))
        .send(rec, BlueprintActivation::default())?;
    Ok(())
}
