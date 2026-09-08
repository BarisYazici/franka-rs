//! The Cartesian half of the flight recorder, for records whose command carries an
//! `O_T_EE_c` (what a Cartesian pose loop sends): the *sent* position against the measured
//! one per axis, the norms of its first, second and third finite differences against the
//! robot's rate limits, and the sent position as a point in the 3D scene -- what a live
//! commander demo needs to show next to the raw target the commander logs itself under
//! [`TARGET_PREFIX`] and [`TARGET_SPEED`] (styled by [`log_target_styles`], laid out by
//! [`super::send_commander_blueprint`]).

use franka::Record;
use rerun::components::AggregationPolicy;
use rerun::{Color, RecordingStream, Scalars, SeriesLines, TimeColumn};

use super::logger::FlightLogger;
use crate::series::Peaks;
use crate::{norm, scene, Result, RobotKind, COMMANDED, LIMIT, MEASURED, TARGET, TIMELINE};

/// `ee/position/{x,y,z}`: the sent position and the measured one, per axis.
pub const POSITION_PREFIX: &str = "ee/position";
/// `ee/derivatives/{speed,acceleration,jerk}`, each with the limit at `.../limit`.
pub const DERIVATIVES_PREFIX: &str = "ee/derivatives";
/// `commander/target/{x,y,z}`: the raw target, absolute, logged by the commander itself.
pub const TARGET_PREFIX: &str = "commander/target";
/// The raw target's implied speed, logged by the commander: the step over one 1 ms cycle
/// (what the robot would see) and the step over the time since the previous target.
pub const TARGET_SPEED: &str = "commander/target_speed";
/// `world/commanded`: the sent position as a point in the 3D scene.
pub const COMMANDED_POINT: &str = "commanded";
pub const AXES: [&str; 3] = ["x", "y", "z"];
/// Entity, legend name and unit of the three derivatives.
const DERIVATIVES: [(&str, &str, &str); 3] = [
    ("speed", "|v| [m/s]", "m/s"),
    ("acceleration", "|a| [m/s^2]", "m/s^2"),
    ("jerk", "|j| [m/s^3]", "m/s^3"),
];
/// The paler orange of the second target speed series.
const TARGET_PALE: u32 = 0xf5b7_8aff;

/// The finite-difference state carried across batches.
#[derive(Debug, Default, Clone, Copy)]
pub(super) struct Cartesian {
    /// The last Cartesian record's time and sent position, velocity, acceleration and jerk.
    last: Option<(f64, [[f64; 3]; 4])>,
    /// The largest norms so far.
    pub(super) peaks: Peaks,
    /// Whether any record carried a Cartesian command.
    pub(super) seen: bool,
}

/// The sent position of a record whose command carries a pose (the default, all-zero
/// `O_T_EE_c` is no pose).
fn sent(record: &Record) -> Option<[f64; 3]> {
    let c = record.command.as_ref()?.O_T_EE_c;
    c.iter().any(|&v| v != 0.0).then_some([c[12], c[13], c[14]])
}

fn difference(a: &[f64; 3], b: &[f64; 3], dt: f64) -> [f64; 3] {
    std::array::from_fn(|k| (a[k] - b[k]) / dt)
}

impl FlightLogger<'_> {
    /// The per-axis positions, the derivatives and the 3D point for the records of a batch
    /// (starting at index `first`, at times `t`) that carry a Cartesian command; the rest are
    /// skipped. A zero time step repeats the previous derivatives.
    pub(super) fn log_cartesian(
        &mut self,
        first: usize,
        t: &[f64],
        records: &[Record],
    ) -> Result<()> {
        let mut times = Vec::new();
        let mut positions: Vec<[f64; 6]> = Vec::new();
        let mut derivatives: Vec<[f64; 3]> = Vec::new();
        for (i, (record, &time)) in records.iter().zip(t).enumerate() {
            let Some(p) = sent(record) else {
                continue;
            };
            let [v, a, j] = match self.cartesian.last {
                Some((t0, [p0, v0, a0, _])) if time > t0 => {
                    let dt = time - t0;
                    let v = difference(&p, &p0, dt);
                    let a = difference(&v, &v0, dt);
                    [v, a, difference(&a, &a0, dt)]
                }
                Some((_, [_, v0, a0, j0])) => [v0, a0, j0],
                None => [[0.0; 3]; 3],
            };
            self.cartesian.last = Some((time, [p, v, a, j]));
            self.cartesian.seen = true;
            let norms = [norm(&v), norm(&a), norm(&j)];
            let peaks = &mut self.cartesian.peaks;
            peaks.speed = peaks.speed.max(norms[0]);
            peaks.acceleration = peaks.acceleration.max(norms[1]);
            peaks.jerk = peaks.jerk.max(norms[2]);
            let m = &record.state.O_T_EE;
            positions.push([p[0], p[1], p[2], m[12], m[13], m[14]]);
            derivatives.push(norms);
            times.push(time);
            if (first + i).is_multiple_of(self.options.every) {
                self.rec.set_duration_secs(TIMELINE, time);
                scene::log_point(&self.rec, COMMANDED_POINT, &p, 0.008, COMMANDED)?;
            }
        }
        if times.is_empty() {
            return Ok(());
        }
        self.summary.commanded_peaks = Some(self.cartesian.peaks);
        let column = || TimeColumn::new_duration_secs(TIMELINE, times.iter().copied());
        for (k, axis) in AXES.iter().enumerate() {
            let values = positions.iter().flat_map(|row| [row[k], row[k + 3]]);
            let columns = Scalars::new(values).columns(std::iter::repeat_n(2, times.len()))?;
            let entity = format!("{POSITION_PREFIX}/{axis}");
            self.rec.send_columns(entity, [column()], columns)?;
        }
        let limits = self.kind.limits();
        let limit_values = [limits.speed, limits.acceleration, limits.jerk];
        let ends = [times[0], times[times.len() - 1]];
        for (k, (name, _, _)) in DERIVATIVES.iter().enumerate() {
            let values = derivatives.iter().map(|row| row[k]);
            let columns = Scalars::new(values).columns_of_unit_batches()?;
            let entity = format!("{DERIVATIVES_PREFIX}/{name}");
            self.rec
                .send_columns(entity.as_str(), [column()], columns)?;
            let limit = Scalars::new([limit_values[k]; 2]).columns_of_unit_batches()?;
            let column = TimeColumn::new_duration_secs(TIMELINE, ends);
            self.rec
                .send_columns(format!("{entity}/limit"), [column], limit)?;
        }
        Ok(())
    }
}

fn lines(names: &[&str], colors: &[u32], widths: &[f32]) -> SeriesLines {
    SeriesLines::new()
        .with_names(names.iter().copied())
        .with_colors(colors.iter().map(|&c| Color::from_u32(c)))
        .with_widths(widths.iter().copied())
}

fn style(
    rec: &RecordingStream,
    entity: &str,
    names: &[&str],
    colors: &[u32],
    widths: &[f32],
) -> Result<()> {
    rec.log_static(entity, &lines(names, colors, widths))?;
    Ok(())
}

/// A style for a series of a few samples a millisecond apart -- the staircase's corner
/// points, a spike -- which the viewer's default per-pixel aggregation would collapse into
/// one point each and join with diagonals: aggregation off, every sample drawn.
fn unaggregated(
    rec: &RecordingStream,
    entity: &str,
    names: &[&str],
    colors: &[u32],
    widths: &[f32],
) -> Result<()> {
    let style = lines(names, colors, widths).with_aggregation_policy(AggregationPolicy::Off);
    rec.log_static(entity, &style)?;
    Ok(())
}

/// The legend names and colours of the per-axis positions and the derivatives (the limit
/// lines carry `kind`'s values in their names).
pub(super) fn log_cartesian_styles(rec: &RecordingStream, kind: RobotKind) -> Result<()> {
    for axis in AXES {
        let entity = format!("{POSITION_PREFIX}/{axis}");
        let names = ["commanded (sent)", "measured O_T_EE"];
        style(rec, &entity, &names, &[COMMANDED, MEASURED], &[2.0, 1.2])?;
    }
    let limits = kind.limits();
    let values = [limits.speed, limits.acceleration, limits.jerk];
    for ((name, label, unit), limit) in DERIVATIVES.iter().zip(values) {
        let entity = format!("{DERIVATIVES_PREFIX}/{name}");
        style(rec, &entity, &[label], &[COMMANDED], &[2.0])?;
        let label = format!("limit {limit:.1} {unit}");
        style(rec, &format!("{entity}/limit"), &[&label], &[LIMIT], &[1.0])?;
    }
    Ok(())
}

/// The legend names and colours of the commander's own entities, [`TARGET_PREFIX`] and
/// [`TARGET_SPEED`], for a program that logs them from its commander thread.
pub fn log_target_styles(rec: &RecordingStream) -> Result<()> {
    for axis in AXES {
        let entity = format!("{TARGET_PREFIX}/{axis}");
        unaggregated(rec, &entity, &["target (raw)"], &[TARGET], &[1.5])?;
    }
    let names = [
        "raw step over 1 ms [m/s]",
        "step over time since last [m/s]",
    ];
    unaggregated(
        rec,
        TARGET_SPEED,
        &names,
        &[TARGET, TARGET_PALE],
        &[2.0, 1.5],
    )
}
