//! The CSV that `examples/nonrealtime_commander.rs --log` writes -- one row per 1 kHz cycle,
//! columns selected by name -- and its replay: positions, derivatives against the limits, the
//! commander's events, the 3D arm, and a default viewer layout.

use std::fmt;
use std::path::Path;

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::Model;
use rerun::blueprint::{
    Blueprint, BlueprintActivation, Grid, Horizontal, Spatial3DView, Tabs, TextLogView, TimePanel,
    TimeSeriesView, Vertical,
};
use rerun::{RecordingStream, TextLog, TextLogLevel};

use crate::series::{Limits, Peaks, PositionSeries};
use crate::{distance, scene, series, Result, COMMANDED, MEASURED, TARGET, TIMELINE};

/// A gap between two target changes longer than this is a stall, s.
pub const STALL_SECONDS: f64 = 1.5;
/// This many target changes inside [`BURST_WINDOW_SECONDS`] are a burst.
pub const BURST_COUNT: usize = 5;
pub const BURST_WINDOW_SECONDS: f64 = 0.1;
/// The example's own settling tolerance: a log whose last commanded position is farther than
/// this from the target ended before the motion settled, i.e. the robot aborted it.
pub const SETTLE_TOLERANCE: f64 = 1e-3;

/// One `--log` CSV, column-wise. Positions are in metres in the robot's base frame.
#[derive(Debug, Clone, PartialEq)]
pub struct CommanderLog {
    /// The robot's time since the motion started, s.
    pub t: Vec<f64>,
    /// The raw commander target: the staircase.
    pub target: Vec<[f64; 3]>,
    /// The commanded position the robot echoed back, `O_T_EE_c`.
    pub commanded: Vec<[f64; 3]>,
    /// The measured position, `O_T_EE`.
    pub measured: Vec<[f64; 3]>,
    /// The measured joint angles, rad; `None` for a CSV written before the `q0..q6` columns
    /// existed, in which case the 3D arm is not drawn.
    pub q: Option<Vec<[f64; 7]>>,
}

/// What a replay recorded, for the caller's summary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Summary {
    /// Peaks of the commanded position's derivatives.
    pub commanded: Peaks,
    /// Peak of the raw target's implied speed (50 m/s at a 5 cm step).
    pub target_speed: f64,
    /// Number of target changes.
    pub changes: usize,
    /// Translation of the tool offset `F_T_EE` identified from the first row's joint angles
    /// and measured `O_T_EE`, m; `None` without joint angles.
    pub tool_offset: Option<[f64; 3]>,
    /// Largest distance between the model's end effector (from `q` and that tool offset) and
    /// the measured `O_T_EE` over the log: the model's residual; `None` without joint angles.
    pub fk_gap: Option<f64>,
}

/// Anything wrong with the CSV, with the path and line in the message.
#[derive(Debug)]
pub struct ParseError(String);

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ParseError {}

impl CommanderLog {
    /// Reads `path`; see [`CommanderLog::parse`].
    pub fn read(path: &Path) -> std::result::Result<CommanderLog, ParseError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| ParseError(format!("{}: {e}", path.display())))?;
        CommanderLog::parse(&text).map_err(|e| ParseError(format!("{}: {e}", path.display())))
    }

    /// Parses CSV text. The header names the columns; `t`, `target_*`, `cmd_*` and `meas_*`
    /// are required, `q0..q6` optional, anything else is ignored.
    pub fn parse(text: &str) -> std::result::Result<CommanderLog, ParseError> {
        let mut lines = text.lines().enumerate();
        let (_, header) = lines
            .next()
            .ok_or_else(|| ParseError("empty file".into()))?;
        let names: Vec<&str> = header.split(',').map(str::trim).collect();
        let column = |name: &str| {
            names
                .iter()
                .position(|candidate| *candidate == name)
                .ok_or_else(|| ParseError(format!("no column {name:?} in header {header:?}")))
        };
        let triple = |prefix: &str| -> std::result::Result<[usize; 3], ParseError> {
            Ok([
                column(&format!("{prefix}x"))?,
                column(&format!("{prefix}y"))?,
                column(&format!("{prefix}z"))?,
            ])
        };
        let t = column("t")?;
        let (target, commanded, measured) = (triple("target_")?, triple("cmd_")?, triple("meas_")?);
        let q: Option<[usize; 7]> = (0..7)
            .map(|joint| column(&format!("q{joint}")).ok())
            .collect::<Option<Vec<usize>>>()
            .and_then(|columns| columns.try_into().ok());

        let mut log = CommanderLog {
            t: Vec::new(),
            target: Vec::new(),
            commanded: Vec::new(),
            measured: Vec::new(),
            q: q.map(|_| Vec::new()),
        };
        let mut fields: Vec<f64> = Vec::with_capacity(names.len());
        for (index, line) in lines {
            if line.trim().is_empty() {
                continue;
            }
            fields.clear();
            for field in line.split(',') {
                let value = field
                    .trim()
                    .parse::<f64>()
                    .map_err(|e| ParseError(format!("line {}: field {field:?}: {e}", index + 1)))?;
                fields.push(value);
            }
            if fields.len() != names.len() {
                return Err(ParseError(format!(
                    "line {}: {} fields, the header has {}",
                    index + 1,
                    fields.len(),
                    names.len()
                )));
            }
            log.t.push(fields[t]);
            log.target.push(target.map(|column| fields[column]));
            log.commanded.push(commanded.map(|column| fields[column]));
            log.measured.push(measured.map(|column| fields[column]));
            if let (Some(columns), Some(rows)) = (q, log.q.as_mut()) {
                rows.push(columns.map(|column| fields[column]));
            }
        }
        if log.t.is_empty() {
            return Err(ParseError("no data rows".into()));
        }
        Ok(log)
    }

    /// Number of rows.
    pub fn rows(&self) -> usize {
        self.t.len()
    }

    /// Indices of the rows on which the raw target differs from the previous row's.
    pub fn target_changes(&self) -> Vec<usize> {
        (1..self.rows())
            .filter(|&i| self.target[i] != self.target[i - 1])
            .collect()
    }

    /// Records everything: `position/*`, `derivatives/*`, `events`, `world/*`. `every`
    /// decimates the 3D scene (1 logs every row).
    pub fn record(
        &self,
        rec: &RecordingStream,
        model: &Model,
        limits: &Limits,
        every: usize,
    ) -> Result<Summary> {
        let traces = [
            ("target (raw)", TARGET, 1.0, &self.target),
            ("commanded O_T_EE_c", COMMANDED, 2.0, &self.commanded),
            ("measured O_T_EE", MEASURED, 1.2, &self.measured),
        ]
        .map(|(name, color, width, positions)| PositionSeries {
            name,
            color,
            width,
            positions,
        });
        series::log_positions(rec, "position", &self.t, &traces)?;
        let commanded = series::log_derivatives(
            rec,
            "derivatives",
            &self.t,
            &self.commanded,
            COMMANDED,
            limits,
        )?;
        let target_speed = series::differences(&self.t, &self.target, 1);
        let entity = "derivatives/target_speed";
        let label = "raw target |v| [m/s]";
        series::log_scalar(rec, entity, label, TARGET, &self.t, &target_speed, None)?;
        let changes = self.log_events(rec)?;
        let trail = (
            "measured_path",
            self.measured.as_slice(),
            MEASURED & 0xffff_ff40,
        );
        scene::log_static(rec, Some(trail))?;
        let f_t_ee = match self.q.as_ref() {
            Some(q) => scene::tool_offset(model, &q[0], &self.measured[0]),
            None => IDENTITY_TRANSFORM,
        };
        let fk_gap = self.log_scene(rec, model, &f_t_ee, every)?;
        Ok(Summary {
            commanded,
            target_speed: target_speed.iter().copied().fold(0.0, f64::max),
            changes,
            tool_offset: self.q.as_ref().map(|_| scene::translation(&f_t_ee)),
            fk_gap,
        })
    }

    /// Every `every`-th row into the 3D scene; returns the model-vs-measured end effector gap.
    fn log_scene(
        &self,
        rec: &RecordingStream,
        model: &Model,
        f_t_ee: &[f64; 16],
        every: usize,
    ) -> Result<Option<f64>> {
        let mut fk_gap = None;
        for i in (0..self.rows()).step_by(every.max(1)) {
            rec.set_duration_secs(TIMELINE, self.t[i]);
            if let Some(q) = self.q.as_ref().map(|rows| &rows[i]) {
                let ee = scene::log_arm(rec, model, q, f_t_ee)?;
                let gap = distance(&ee, &self.measured[i]);
                fk_gap = Some(fk_gap.map_or(gap, |g: f64| g.max(gap)));
            }
            scene::log_point(rec, "target", &self.target[i], 0.015, TARGET)?;
            scene::log_point(rec, "commanded", &self.commanded[i], 0.008, COMMANDED)?;
            scene::log_point(rec, "measured", &self.measured[i], 0.006, MEASURED)?;
        }
        Ok(fk_gap)
    }

    /// `events`: every target step, stalls, bursts, and an abort if the log ends unsettled.
    /// Returns the number of target changes.
    pub fn log_events(&self, rec: &RecordingStream) -> Result<usize> {
        let event = |t: f64, level: &str, text: String| -> Result<()> {
            rec.set_duration_secs(TIMELINE, t);
            rec.log("events", &TextLog::new(text).with_level(level))?;
            Ok(())
        };
        let changes = self.target_changes();
        let mut last_change = self.t[0];
        let mut window_start = 0;
        let mut in_burst = false;
        for (k, &row) in changes.iter().enumerate() {
            let t = self.t[row];
            let gap = t - last_change;
            if gap > STALL_SECONDS {
                let text = format!("stall: no new target for {gap:.2} s");
                event(last_change + STALL_SECONDS, TextLogLevel::WARN, text)?;
            }
            let [x, y, z] = self.target[row];
            event(
                t,
                TextLogLevel::INFO,
                format!("target step to [{x:.3}, {y:.3}, {z:.3}]"),
            )?;
            while t - self.t[changes[window_start]] > BURST_WINDOW_SECONDS {
                window_start += 1;
            }
            let in_window = k - window_start + 1;
            if in_window >= BURST_COUNT && !in_burst {
                let window_ms = BURST_WINDOW_SECONDS * 1e3;
                let text = format!("burst: {in_window} targets within {window_ms:.0} ms");
                event(t, TextLogLevel::WARN, text)?;
            }
            in_burst = in_window >= BURST_COUNT;
            last_change = t;
        }
        let last = self.rows() - 1;
        let unsettled = distance(&self.commanded[last], &self.target[last]);
        if unsettled > SETTLE_TOLERANCE {
            let text = format!(
                "motion aborted by the robot: the log ends with the command {:.1} mm from the target",
                unsettled * 1e3
            );
            event(self.t[last], TextLogLevel::ERROR, text)?;
        }
        Ok(changes.len())
    }
}

/// The default layout for a commander replay: the 3D scene on the left, the positions and
/// the derivatives in a grid on the right, the event log along the bottom, the time panel on
/// `robot_time`.
pub fn send_blueprint(rec: &RecordingStream) -> Result<()> {
    let plot = |name: &str, origin: &str| TimeSeriesView::new(name).with_origin(origin).into();
    let speed = Tabs::new([
        plot("speed", "derivatives/speed"),
        plot("raw target speed", "derivatives/target_speed"),
    ]);
    let plots = Grid::new([
        plot("x", "position/x"),
        speed.into(),
        plot("y", "position/y"),
        plot("acceleration", "derivatives/acceleration"),
        plot("z", "position/z"),
        plot("jerk", "derivatives/jerk"),
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

#[cfg(test)]
mod tests {
    use super::CommanderLog;

    const CSV: &str = "t,target_x,target_y,target_z,cmd_x,cmd_y,cmd_z,meas_x,meas_y,meas_z,\
                       q0,q1,q2,q3,q4,q5,q6\n\
                       0.0000,0.3,0,0.5,0.3,0,0.5,0.3,0,0.5,0,-0.78,0,-2.35,0,1.57,0.78\n\
                       0.0010,0.35,0,0.5,0.3,0,0.5,0.3,0,0.5,0,-0.78,0,-2.35,0,1.57,0.78\n";

    #[test]
    fn parses_columns_by_name_and_finds_target_changes() {
        let log = CommanderLog::parse(CSV).unwrap();
        assert_eq!(log.rows(), 2);
        assert_eq!(log.target[1], [0.35, 0.0, 0.5]);
        assert_eq!(log.q.as_ref().unwrap()[0][3], -2.35);
        assert_eq!(log.target_changes(), vec![1]);
    }

    #[test]
    fn joint_angles_are_optional_and_missing_positions_are_not() {
        let without_q: String = CSV
            .lines()
            .map(|line| line.rsplitn(8, ',').last().unwrap().to_string() + "\n")
            .collect();
        assert!(CommanderLog::parse(&without_q).unwrap().q.is_none());
        let error = CommanderLog::parse("t,target_x\n0,0\n").unwrap_err();
        assert!(error.to_string().contains("target_y"), "{error}");
    }
}
