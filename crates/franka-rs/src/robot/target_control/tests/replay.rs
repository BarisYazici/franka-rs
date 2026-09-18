//! Replays a Cartesian recording of your own, given by the `REPLAY_*` variables, through the
//! real torque loop with the joint velocity cap on and, as the reference, off. `#[ignore]`d:
//!
//! ```text
//! REPLAY_STATES=$PWD/states.bin REPLAY_TARGETS=$PWD/targets.bin cargo test --release \
//!     -p franka-rs --lib replay_a_recorded_session -- --ignored --nocapture
//! ```
//!
//! Paths are absolute or relative to the crate, where the test runs.
//! `REPLAY_STATES`: little-endian f64 rows `t, q (7), dq (7), O_T_EE (16)`, one per cycle;
//! `REPLAY_TARGETS`: rows `t, x, y, z, qx, qy, qz, qw`, the accepted targets at robot time.
//! The run's configuration, the library's defaults when unset: `REPLAY_BUDGET` and `REPLAY_ROT`
//! (`v,a,j` norms), `REPLAY_STIFFNESS` (N/m), `REPLAY_LEASH` (`m,rad`), `REPLAY_FRACTION`, and
//! `REPLAY_HAND=0` without the Franka Hand's `F_T_EE`. The reference is the same run at an
//! infinite fraction, which switches off the cap, the barrier and, with them, the guard's
//! braking envelope: it bounds what the whole guard costs, not the cap alone. Chatter is
//! [`chatter`]'s, judged on the recorded arm here, so the torque rate is judged against that
//! reference rather than the absolute floor the plant runs are held to.
//! `REPLAY_OUT` writes the capped run, 60 f64 per cycle (see
//! [`write_out`]). Recorded sessions as CSV, commanded or on the plant: [`csv`].

mod chatter;
pub(super) mod csv;
mod envelope;
mod fr3;
mod joint;
mod limits;
mod sync;
mod yaw;

use std::io::Write;

use nalgebra::Matrix3;

use super::super::rotation::{
    angle_between, distance, from_quaternion, log, rotation_of, translation_of,
};
use super::super::*;
use chatter::{Chatter, HALF, NEAR_RMS, SLOW, SPEED_HALF};

use super::plant::HAND;
use super::recording;
use super::torque::cartesian_loop;
use crate::model::{Frame, Model};
use crate::otg::OtgLimits;
use crate::rate_limiting::fer::JOINT_POSITION_LIMITS;
use crate::rate_limiting::DELTA_T;
use crate::wire::robot::codec::FciVersion;

/// The rows of `width` f64 in the file `var` names.
fn rows(var: &str, width: usize) -> Vec<Vec<f64>> {
    let path = std::env::var(var).unwrap_or_else(|_| panic!("{var} must name a file"));
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    assert_eq!(
        bytes.len() % (8 * width),
        0,
        "{path}: not rows of {width} f64"
    );
    let values: Vec<f64> = bytes
        .chunks_exact(8)
        .map(|c| f64::from_le_bytes(c.try_into().unwrap()))
        .collect();
    values.chunks_exact(width).map(<[f64]>::to_vec).collect()
}

/// The comma-separated numbers of `var`, or `default`.
fn numbers<const N: usize>(var: &str, default: [f64; N]) -> [f64; N] {
    let Ok(text) = std::env::var(var) else {
        return default;
    };
    let values: Vec<f64> = text.split(',').map(|x| x.trim().parse().unwrap()).collect();
    values
        .try_into()
        .unwrap_or_else(|v| panic!("{var} takes {N} numbers, got {v:?}"))
}

fn limits([max_velocity, max_acceleration, max_jerk]: [f64; 3]) -> OtgLimits {
    OtgLimits {
        max_velocity,
        max_acceleration,
        max_jerk,
    }
}

fn triple(limits: OtgLimits) -> [f64; 3] {
    [
        limits.max_velocity,
        limits.max_acceleration,
        limits.max_jerk,
    ]
}

/// One replay's configuration.
#[derive(Clone, Copy)]
pub(super) struct Config {
    pub budget: OtgLimits,
    pub rotation: OtgLimits,
    pub stiffness: f64,
    pub leash: Leash,
    pub fraction: f64,
    pub hand: bool,
}

/// The loop's options for `config`, with `observer`.
pub(super) fn loop_options(
    config: Config,
    observer: impl FnMut(&RobotState, &CartesianSent) + Send + 'static,
) -> (TargetControlOptions, ImpedanceOptions) {
    let preset = ImpedanceGains::CARTESIAN;
    let ratio = config.stiffness / preset.cartesian_stiffness[0];
    let gains = ImpedanceGains {
        cartesian_stiffness: preset.cartesian_stiffness.map(|k| k * ratio),
        cartesian_damping: preset.cartesian_damping.map(|d| d * ratio.sqrt()),
        ..preset
    };
    // An infinite fraction switches the cap and the barrier off: the reference's switch, which
    // `validate` refuses, so no started loop can have it.
    let barrier = match config.fraction {
        f if f.is_finite() => ImpedanceOptions::cartesian()
            .velocity_barrier_fraction
            .max(f),
        _ => f64::INFINITY,
    };
    let impedance = ImpedanceOptions::cartesian()
        .with_gains(gains)
        .with_leash(config.leash)
        .with_joint_velocity_fraction(config.fraction)
        .with_velocity_barrier_fraction(barrier);
    // The deviation guard is the commander's business: a replay must not end on it.
    let options = TargetControlOptions::default()
        .with_limits(config.budget)
        .with_rotation_limits(config.rotation)
        .with_max_deviation(100.0)
        .with_max_angular_deviation(100.0)
        .with_observer(observer);
    (options, impedance)
}

/// Every record of the loop over `states` with the accepted targets published at their robot
/// time, and how far the model's pose of each recorded `q` is from the recorded `O_T_EE`, m.
fn replay(
    states: &[Vec<f64>],
    targets: &[Vec<f64>],
    config: Config,
) -> (Vec<CartesianSent>, Vec<f64>) {
    let (records, observer) = recording::<CartesianSent>();
    let (options, impedance) = loop_options(config, observer);
    let (mut torque, shared, _first) = cartesian_loop(options, impedance);
    let model = Model::native_fer();
    let mut state = RobotState::default();
    if config.hand {
        state.F_T_EE = HAND.f_t_ee;
    }
    let (mut next, mut fk_error) = (0, Vec::with_capacity(states.len()));
    for (i, row) in states.iter().enumerate() {
        state.q.copy_from_slice(&row[1..8]);
        state.dq.copy_from_slice(&row[8..15]);
        state.O_T_EE.copy_from_slice(&row[15..31]);
        // The first cycle anchors; a target recorded before a cycle's time is in the slot for it.
        while i > 0 && next < targets.len() && targets[next][0] < row[0] - 1e-7 {
            let t = &targets[next];
            let n = t[4..8].iter().map(|c| c * c).sum::<f64>().sqrt();
            shared
                .slot
                .publish([t[1], t[2], t[3], t[4] / n, t[5] / n, t[6] / n, t[7] / n]);
            next += 1;
        }
        let fk = model.pose(Frame::EndEffector, &state);
        fk_error.push(distance(
            &translation_of(&fk),
            &translation_of(&state.O_T_EE),
        ));
        torque.cycle(&state);
    }
    drop(torque);
    let sent = std::mem::take(&mut *records.lock().unwrap());
    (sent, fk_error)
}

/// The cycles within (-10, +100) ms of a target that moved: the engaged time.
fn engaged(times: &[f64], targets: &[Vec<f64>]) -> Vec<bool> {
    let mut mask = vec![false; times.len()];
    for (k, row) in targets.iter().enumerate() {
        let moved = k == 0
            || row[1..]
                .iter()
                .zip(&targets[k - 1][1..])
                .any(|(a, b)| (a - b).abs() > 1e-6);
        if moved {
            let from = times.partition_point(|&t| t < row[0] - 0.01);
            let to = times.partition_point(|&t| t < row[0] + 0.1);
            mask[from..to].fill(true);
        }
    }
    mask
}

/// `ys` at every `at`, linear between `xs` and held flat outside them.
fn interpolate(xs: &[f64], ys: &[f64], at: &[f64]) -> Vec<f64> {
    at.iter()
        .map(|&t| match xs.partition_point(|&x| x <= t) {
            0 => ys[0],
            k if k == xs.len() => ys[k - 1],
            k => ys[k - 1] + (ys[k] - ys[k - 1]) * (t - xs[k - 1]) / (xs[k] - xs[k - 1]),
        })
        .collect()
}

/// The shift, ms, minimising the spread of `a[i - lag] - b[i]` (its mean removed) over the
/// masked cycles: how far `b` follows `a`.
fn best_lag(a: &[f64], b: &[f64], mask: &[bool], max_lag: usize) -> usize {
    let cycles: Vec<usize> = (max_lag + 1..a.len()).filter(|&i| mask[i]).collect();
    let n = cycles.len() as f64;
    let spread = |lag: usize| {
        let mean = cycles.iter().map(|&i| a[i - lag] - b[i]).sum::<f64>() / n;
        cycles
            .iter()
            .map(|&i| (a[i - lag] - b[i] - mean).powi(2))
            .sum::<f64>()
    };
    (0..=max_lag)
        .map(|lag| (spread(lag), lag))
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .unwrap()
        .1
}

/// numpy's default (linear) percentile.
fn percentile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(f64::total_cmp);
    let rank = q / 100.0 * (values.len() - 1) as f64;
    let low = rank.floor() as usize;
    let high = (low + 1).min(values.len() - 1);
    values[low] + rank.fract() * (values[high] - values[low])
}

/// A run's numbers: the target -> command lag per axis (x, y, z, then the rotation vector
/// about the first measured orientation), ms, and the command-side tracking error's p50 and p99
/// over the engaged cycles, (m, rad).
struct Metrics {
    lag_ms: [usize; 6],
    p50: (f64, f64),
    p99: (f64, f64),
}

fn metrics(
    states: &[Vec<f64>],
    targets: &[Vec<f64>],
    sent: &[CartesianSent],
    engaged: &[bool],
) -> Metrics {
    let times: Vec<f64> = states.iter().map(|r| r[0]).collect();
    let target_times: Vec<f64> = targets.iter().map(|r| r[0]).collect();
    let mut first = [0.0; 16];
    first.copy_from_slice(&states[0][15..31]);
    let about_first = rotation_of(&first).transpose();
    let vector = |r: Matrix3<f64>| log(&(r * about_first));
    let target_rv: Vec<[f64; 3]> = targets
        .iter()
        .map(|t| vector(from_quaternion(&[t[4], t[5], t[6], t[7]])))
        .collect();
    let command_rv: Vec<[f64; 3]> = sent.iter().map(|s| vector(rotation_of(&s.pose))).collect();
    let lag_ms = std::array::from_fn(|axis| {
        let (wanted, commanded): (Vec<f64>, Vec<f64>) = if axis < 3 {
            (
                targets.iter().map(|t| t[1 + axis]).collect(),
                sent.iter().map(|s| s.pose[12 + axis]).collect(),
            )
        } else {
            (
                target_rv.iter().map(|v| v[axis - 3]).collect(),
                command_rv.iter().map(|v| v[axis - 3]).collect(),
            )
        };
        let wanted = interpolate(&target_times, &wanted, &times);
        best_lag(&wanted, &commanded, engaged, 400)
    });
    let (mut position, mut rotation): (Vec<f64>, Vec<f64>) = sent
        .iter()
        .zip(engaged)
        .filter(|(_, e)| **e)
        .map(|(s, _)| {
            let target = from_quaternion(&s.target_orientation);
            (
                distance(&s.target, &translation_of(&s.pose)),
                angle_between(&target, &rotation_of(&s.pose)),
            )
        })
        .unzip();
    Metrics {
        lag_ms,
        p50: (
            percentile(&mut position, 50.0),
            percentile(&mut rotation, 50.0),
        ),
        p99: (
            percentile(&mut position, 99.0),
            percentile(&mut rotation, 99.0),
        ),
    }
}

/// Per cycle, whether the target's position moves slower than [`chatter::SLOW`] over the centred
/// window: [`chatter`]'s rule, so that intended motion is not read as chatter.
fn slow(times: &[f64], targets: &[Vec<f64>]) -> Vec<bool> {
    let n = times.len();
    let at = |k: usize| {
        let i = targets.partition_point(|t| t[0] <= times[k]);
        i.checked_sub(1).map(|i| &targets[i])
    };
    (0..n)
        .map(|k| {
            let (from, to) = (k.saturating_sub(SPEED_HALF), (k + SPEED_HALF).min(n - 1));
            match (at(from), at(to)) {
                (Some(a), Some(b)) => {
                    let moved = (1..4).map(|i| (b[i] - a[i]).powi(2)).sum::<f64>().sqrt();
                    moved < SLOW * (to - from) as f64 * DELTA_T
                }
                _ => true,
            }
        })
        .collect()
}

/// The cycles within `margin` of one the generator was restarted on.
fn capped_windows(sent: &[CartesianSent], margin: usize) -> Vec<bool> {
    let mut mask = vec![false; sent.len()];
    for i in (0..sent.len()).filter(|&i| sent[i].cap_scale < 1.0) {
        mask[i.saturating_sub(margin)..(i + margin + 1).min(sent.len())].fill(true);
    }
    mask
}

/// The largest second difference of any joint's `dq_goal` over the masked cycles, rad/s: how
/// hard the goal's velocity chatters.
fn chatter(sent: &[CartesianSent], mask: &[bool]) -> f64 {
    (1..sent.len().saturating_sub(1))
        .filter(|&k| mask[k])
        .flat_map(|k| {
            let [before, now, after] = [&sent[k - 1], &sent[k], &sent[k + 1]].map(|r| r.dq_goal);
            (0..7).map(move |j| (after[j] - 2.0 * now[j] + before[j]).abs())
        })
        .fold(0.0, f64::max)
}

/// The capped run, 60 f64 per cycle: t, pose xyz, orientation xyzw, velocity, angular velocity,
/// acceleration, angular acceleration, target xyz, target xyzw, q_goal, tau, the two leash
/// alterations, ik_error and the FK error, then dq_goal, cap_scale and tau_envelope.
fn write_out(path: &str, times: &[f64], sent: &[CartesianSent], fk_error: &[f64]) {
    let mut out = std::io::BufWriter::new(std::fs::File::create(path).unwrap());
    for ((t, r), e) in times.iter().zip(sent).zip(fk_error) {
        let scalars = [
            r.leash_alteration,
            r.leash_angular_alteration,
            r.ik_error,
            *e,
        ];
        let scale = [r.cap_scale];
        let row: [&[f64]; 15] = [
            std::slice::from_ref(t),
            &r.pose[12..15],
            &r.orientation,
            &r.velocity,
            &r.angular_velocity,
            &r.acceleration,
            &r.angular_acceleration,
            &r.target,
            &r.target_orientation,
            &r.q_goal,
            &r.tau,
            &scalars,
            &r.dq_goal,
            &scale,
            &r.tau_envelope,
        ];
        for value in row.iter().flat_map(|part| part.iter()) {
            out.write_all(&value.to_le_bytes()).unwrap();
        }
    }
}

#[test]
#[ignore]
fn replay_a_recorded_session() {
    let states = rows("REPLAY_STATES", 31);
    let targets = rows("REPLAY_TARGETS", 8);
    let defaults = TargetControlOptions::default();
    let [translation, rotation] = numbers(
        "REPLAY_LEASH",
        [Leash::default().translation, Leash::default().rotation],
    );
    let run = Config {
        budget: limits(numbers("REPLAY_BUDGET", triple(defaults.limits))),
        rotation: limits(numbers("REPLAY_ROT", triple(defaults.rotation_limits))),
        stiffness: numbers(
            "REPLAY_STIFFNESS",
            [ImpedanceGains::CARTESIAN.cartesian_stiffness[0]],
        )[0],
        leash: Leash {
            translation,
            rotation,
            ..Leash::default()
        },
        fraction: numbers(
            "REPLAY_FRACTION",
            [ImpedanceOptions::cartesian().joint_velocity_fraction],
        )[0],
        hand: std::env::var("REPLAY_HAND").as_deref() != Ok("0"),
    };
    let reference = Config {
        fraction: f64::INFINITY,
        ..run
    };
    let (capped, fk_error) = replay(&states, &targets, run);
    let (free, _) = replay(&states, &targets, reference);
    let times: Vec<f64> = states.iter().map(|r| r[0]).collect();
    let engaged = engaged(&times, &targets);
    let ours = metrics(&states, &targets, &capped, &engaged);
    let theirs = metrics(&states, &targets, &free, &engaged);

    let caps = max_joint_velocity(FciVersion::V5).map(|v| v * run.fraction);
    let peak: [f64; 7] = std::array::from_fn(|j| {
        capped
            .iter()
            .map(|r| r.dq_goal[j].abs() / caps[j])
            .fold(0.0, f64::max)
    });
    let engaged_cycles = engaged.iter().filter(|e| **e).count();
    let active = capped
        .iter()
        .zip(&engaged)
        .filter(|(r, e)| **e && r.cap_scale < 1.0)
        .count() as f64
        / engaged_cycles as f64;
    let first = capped.iter().position(|r| r.cap_scale < 1.0);
    let (apart_m, apart_rad) =
        (0..first.unwrap_or(capped.len())).fold((0.0f64, 0.0f64), |(m, r), i| {
            let (a, b) = (&capped[i].pose, &free[i].pose);
            (
                m.max(distance(&translation_of(a), &translation_of(b))),
                r.max(angle_between(&rotation_of(a), &rotation_of(b))),
            )
        });
    let min_scale = capped.iter().map(|r| r.cap_scale).fold(1.0, f64::min);
    let windows = capped_windows(&capped, 5);
    let (jitter, reference_jitter) = (chatter(&capped, &windows), chatter(&free, &windows));
    // `jitter` is printed, not judged: `cap_scale` marks every restart, the position envelope's
    // braking included, and its "one kink" floor is the cap flattening a goal at the arm's
    // acceleration limit, not a proportional cut of a fast step. Chatter is judged on
    // [`chatter`]'s quantities instead: the torque rate against the same run with the guard off,
    // because a recording's own rate is not the guard's doing, and the near-limit RMS against
    // its absolute floor where the session goes near a limit at all.
    let q: Vec<[f64; 7]> = states
        .iter()
        .map(|r| std::array::from_fn(|i| r[1 + i]))
        .collect();
    let slow = slow(&times, &targets);
    let jitters = Chatter::of(&capped, &q, &slow, &JOINT_POSITION_LIMITS);
    let reference_jitters = Chatter::of(&free, &q, &slow, &JOINT_POSITION_LIMITS);

    let mut checks = vec![
        (
            "dq_goal within the cap every cycle".to_string(),
            peak.iter().all(|p| *p <= 1.0 + 1e-9),
        ),
        (
            "cap active at most 0.5 % of engaged time".to_string(),
            active <= 0.005,
        ),
        (
            "p50 position at most the reference's + 5 %".to_string(),
            ours.p50.0 <= 1.05 * theirs.p50.0,
        ),
        (
            "p50 rotation at most the reference's + 5 %".to_string(),
            ours.p50.1 <= 1.05 * theirs.p50.1,
        ),
        (
            "p99 position at most the reference's + 5 %".to_string(),
            ours.p99.0 <= 1.05 * theirs.p99.0,
        ),
        (
            "p99 rotation at most the reference's + 5 %".to_string(),
            ours.p99.1 <= 1.05 * theirs.p99.1,
        ),
        (
            "commanded torque rate p99 at most the reference's + 5 %".to_string(),
            jitters.rate_p99 <= 1.05 * reference_jitters.rate_p99,
        ),
    ];
    // The near-limit RMS needs a high-pass window of slow cycles near a limit to mean anything.
    if jitters.counts[0] > 2 * HALF {
        checks.push((
            format!("near-limit dq_goal RMS at most {NEAR_RMS} rad/s"),
            jitters.near.iter().all(|rms| *rms <= NEAR_RMS),
        ));
    }
    for (axis, name) in ["x", "y", "z", "rx", "ry", "rz"].iter().enumerate() {
        let ok = ours.lag_ms[axis] <= theirs.lag_ms[axis] + 5;
        checks.push((format!("lag {name} at most the reference's + 5 ms"), ok));
    }
    checks.push((
        "identical to the reference until the cap engages".to_string(),
        apart_m <= 1e-5 && apart_rad <= 1e-5,
    ));
    let failed: Vec<&str> = checks
        .iter()
        .filter(|(_, ok)| !ok)
        .map(|(name, _)| name.as_str())
        .collect();

    let fk = fk_error.iter().copied().fold(0.0, f64::max);
    println!(
        "replay: {} cycles, {} targets, engaged {:.2} s, FK vs O_T_EE at most {fk:.2e} m",
        states.len(),
        targets.len(),
        engaged_cycles as f64 * 1e-3
    );
    println!(
        "replay: cap {} of the limits, active {active:.4} of engaged time, first at {:?} s, \
         smallest scale {min_scale:.3}, peak |dq_goal| / cap {peak:.3?}",
        run.fraction,
        first.map(|i| times[i] - times[0])
    );
    println!(
        "replay: until the cap engaged the command stayed within {apart_m:.2e} m and \
         {apart_rad:.2e} rad of the reference"
    );
    println!("replay: {jitters}");
    println!("replay: reference {reference_jitters}");
    println!(
        "replay-summary budget={:?} rotation={:?} fraction={} active={active:.4} lag_ms={:?} \
         reference_lag_ms={:?} p50={:.5?} reference_p50={:.5?} p99={:.5?} reference_p99={:.5?} \
         restart_jitter={jitter:.4} reference_jitter={reference_jitter:.4} \
         near_rms={:.4} rate_p99={:.0} reference_rate_p99={:.0} pass={}",
        triple(run.budget),
        triple(run.rotation),
        run.fraction,
        ours.lag_ms,
        theirs.lag_ms,
        ours.p50,
        theirs.p50,
        ours.p99,
        theirs.p99,
        jitters.near.iter().copied().fold(0.0, f64::max),
        jitters.rate_p99,
        reference_jitters.rate_p99,
        failed.is_empty()
    );
    if let Ok(path) = std::env::var("REPLAY_OUT") {
        write_out(&path, &times, &capped, &fk_error);
    }
    assert!(failed.is_empty(), "failed: {failed:?}");
}
