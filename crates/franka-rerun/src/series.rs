//! Time series on the `robot_time` timeline, from slices of times and values: positions per
//! axis, finite-difference derivatives of a position against the robot's limits, and plain
//! scalar series -- lines, staircases -- with an optional horizontal limit line.

use rerun::components::AggregationPolicy;
use rerun::{Color, RecordingStream, Scalars, SeriesLines, TimeColumn};

use crate::{norm, Result, LIMIT, TIMELINE};

/// The rate limiter's translational limits, m/s, m/s^2 and m/s^3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub speed: f64,
    pub acceleration: f64,
    pub jerk: f64,
}

/// The largest norms the finite differences of a position reached.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Peaks {
    pub speed: f64,
    pub acceleration: f64,
    pub jerk: f64,
}

/// One named, coloured position trace for [`log_positions`].
#[derive(Debug, Clone, Copy)]
pub struct PositionSeries<'a> {
    /// The legend name.
    pub name: &'a str,
    /// `0xRRGGBBAA`.
    pub color: u32,
    /// Stroke width, UI points.
    pub width: f32,
    /// One position per time.
    pub positions: &'a [[f64; 3]],
}

fn times(t: &[f64]) -> TimeColumn {
    TimeColumn::new_duration_secs(TIMELINE, t.iter().copied())
}

/// `{prefix}/x`, `{prefix}/y`, `{prefix}/z`, each carrying all of `series` -- one entity per
/// axis so each gets its own plot and y-axis. Every series must have `t.len()` positions.
pub fn log_positions(
    rec: &RecordingStream,
    prefix: &str,
    t: &[f64],
    series: &[PositionSeries<'_>],
) -> Result<()> {
    for (axis, name) in ["x", "y", "z"].into_iter().enumerate() {
        let entity = format!("{prefix}/{name}");
        let style = SeriesLines::new()
            .with_names(series.iter().map(|s| s.name))
            .with_colors(series.iter().map(|s| Color::from_u32(s.color)))
            .with_widths(series.iter().map(|s| s.width));
        rec.log_static(entity.as_str(), &style)?;
        let values = (0..t.len()).flat_map(|i| series.iter().map(move |s| s.positions[i][axis]));
        let columns = Scalars::new(values).columns(std::iter::repeat_n(series.len(), t.len()))?;
        rec.send_columns(entity.as_str(), [times(t)], columns)?;
    }
    Ok(())
}

/// The `order`-th finite difference of `positions` over `t`, per axis (1: velocity, 2:
/// acceleration, 3: jerk). The first `order` samples are zero; a non-positive time step
/// repeats the previous derivative instead of dividing by it.
pub fn difference_vectors(t: &[f64], positions: &[[f64; 3]], order: usize) -> Vec<[f64; 3]> {
    let mut derivative = positions.to_vec();
    for _ in 0..order {
        let previous = derivative.clone();
        for i in 1..t.len() {
            let dt = t[i] - t[i - 1];
            derivative[i] = if dt > 0.0 {
                std::array::from_fn(|k| (previous[i][k] - previous[i - 1][k]) / dt)
            } else {
                derivative[i - 1]
            };
        }
        if let Some(first) = derivative.first_mut() {
            *first = [0.0; 3];
        }
    }
    derivative
}

/// Norms of [`difference_vectors`] (1: speed, 2: acceleration, 3: jerk).
pub fn differences(t: &[f64], positions: &[[f64; 3]], order: usize) -> Vec<f64> {
    difference_vectors(t, positions, order)
        .iter()
        .map(norm)
        .collect()
}

/// One scalar series at `entity` named `name`, in `color`, `width` UI points wide. A
/// `staircase` -- a target that jumps and holds, its corners a millisecond apart -- is
/// logged with the viewer's per-pixel aggregation off, so that the corners stay corners at
/// every zoom instead of averaging into a slope.
#[allow(clippy::too_many_arguments)]
pub fn log_line(
    rec: &RecordingStream,
    entity: &str,
    name: &str,
    color: u32,
    width: f32,
    staircase: bool,
    t: &[f64],
    values: &[f64],
) -> Result<()> {
    let mut style = SeriesLines::new()
        .with_names([name])
        .with_colors([Color::from_u32(color)])
        .with_widths([width]);
    if staircase {
        style = style.with_aggregation_policy(AggregationPolicy::Off);
    }
    rec.log_static(entity, &style)?;
    let scalars = Scalars::new(values.iter().copied()).columns_of_unit_batches()?;
    rec.send_columns(entity, [times(t)], scalars)?;
    Ok(())
}

/// `limit 0.3 m/s`, `limit 20 m/s³`: a limit line's legend name, the value without
/// trailing zeros.
pub fn limit_label(limit: f64, unit: &str) -> String {
    let value = format!("{limit:.3}");
    let value = value.trim_end_matches('0').trim_end_matches('.');
    format!("limit {value} {unit}")
}

/// One scalar series at `entity`, with an optional grey limit line -- two samples, at the
/// first and the last time -- at `entity/limit`, so a plot of `entity` shows both.
pub fn log_scalar(
    rec: &RecordingStream,
    entity: &str,
    name: &str,
    color: u32,
    t: &[f64],
    values: &[f64],
    limit: Option<(f64, &str)>,
) -> Result<()> {
    log_line(rec, entity, name, color, 2.0, false, t, values)?;
    if let (Some((limit, label)), Some(&first), Some(&last)) = (limit, t.first(), t.last()) {
        let entity = format!("{entity}/limit");
        let style = SeriesLines::new()
            .with_names([label])
            .with_colors([Color::from_u32(LIMIT)])
            .with_widths([1.0]);
        rec.log_static(entity.as_str(), &style)?;
        let column = TimeColumn::new_duration_secs(TIMELINE, [first, last]);
        let values = Scalars::new([limit, limit]).columns_of_unit_batches()?;
        rec.send_columns(entity.as_str(), [column], values)?;
    }
    Ok(())
}

fn max(values: &[f64]) -> f64 {
    values.iter().copied().fold(0.0, f64::max)
}

/// `{prefix}/speed`, `{prefix}/acceleration` and `{prefix}/jerk`: the norms of the first,
/// second and third finite differences of `positions`, each with its limit as a grey line.
pub fn log_derivatives(
    rec: &RecordingStream,
    prefix: &str,
    t: &[f64],
    positions: &[[f64; 3]],
    color: u32,
    limits: &Limits,
) -> Result<Peaks> {
    let entries = [
        ("speed", "|v| [m/s]", 1, limits.speed, "m/s"),
        (
            "acceleration",
            "|a| [m/s^2]",
            2,
            limits.acceleration,
            "m/s^2",
        ),
        ("jerk", "|j| [m/s^3]", 3, limits.jerk, "m/s^3"),
    ];
    let mut peaks = [0.0; 3];
    for (peak, (name, label, order, limit, unit)) in peaks.iter_mut().zip(entries) {
        let values = differences(t, positions, order);
        *peak = max(&values);
        let entity = format!("{prefix}/{name}");
        let limit_label = format!("limit {limit:.1} {unit}");
        log_scalar(
            rec,
            &entity,
            label,
            color,
            t,
            &values,
            Some((limit, &limit_label)),
        )?;
    }
    Ok(Peaks {
        speed: peaks[0],
        acceleration: peaks[1],
        jerk: peaks[2],
    })
}

#[cfg(test)]
mod tests {
    use super::{difference_vectors, differences, limit_label};

    #[test]
    fn limit_labels_drop_trailing_zeros() {
        assert_eq!(limit_label(0.3, "m/s"), "limit 0.3 m/s");
        assert_eq!(limit_label(20.0, "m/s³"), "limit 20 m/s³");
        assert_eq!(limit_label(1.961, "m/s"), "limit 1.961 m/s");
    }

    #[test]
    fn finite_differences_of_a_step() {
        let t = [0.0, 0.001, 0.002, 0.003];
        let p = [[0.0; 3], [0.0; 3], [0.05, 0.0, 0.0], [0.05, 0.0, 0.0]];
        let velocity = difference_vectors(&t, &p, 1);
        assert!((velocity[2][0] - 50.0).abs() < 1e-9, "{velocity:?}");
        assert_eq!(velocity[2][1..], [0.0, 0.0]);
        let speed = differences(&t, &p, 1);
        assert_eq!(speed[0], 0.0);
        assert!((speed[2] - 50.0).abs() < 1e-9, "{speed:?}");
        assert_eq!(speed[3], 0.0);
        let acceleration = differences(&t, &p, 2);
        assert!(
            (acceleration[2] - 50_000.0).abs() < 1e-6,
            "{acceleration:?}"
        );
        assert!(
            (acceleration[3] - 50_000.0).abs() < 1e-6,
            "{acceleration:?}"
        );
    }

    #[test]
    fn a_zero_time_step_repeats_the_previous_derivative() {
        let t = [0.0, 0.001, 0.001, 0.002];
        let p = [
            [0.0; 3],
            [0.001, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.003, 0.0, 0.0],
        ];
        let speed = differences(&t, &p, 1);
        assert!((speed[1] - 1.0).abs() < 1e-9);
        assert_eq!(speed[2], speed[1]);
    }
}
