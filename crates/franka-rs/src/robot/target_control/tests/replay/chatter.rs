//! Joint-goal chatter near the limits: on the Panda the goal velocity of joints 1, 3 and 7
//! chattered at 0.08 to 0.23 rad/s while a joint was near a limit, which drove the torque rate to
//! the robot's 1000 Nm/s cap. Measured as the high-pass (less the centred 21-cycle mean) RMS of
//! `dq_goal` per joint over the slow cycles (the target under 0.02 m/s, so that intended motion
//! does not count) any joint is within 0.1 rad of a limit, and the p99 of the torque rate, over a
//! whole session. The rate is of the law's clamped output, before the robot's low-pass and rate
//! limit act on it.
//!
//! The torque rate is the acceptance test, because it is the hardware symptom. The near-limit RMS
//! is judged too, but against an absolute floor: it moves 4× between variants that behave the same
//! on the arm, so a tight gate on it would accept and refuse on the metric's own noise.
//!
//! The guard is judged on the plant, not on the recorded arm: fed the recorded `q`, the leash
//! carries the recorded arm's own chatter into the goals. It is judged on two plants, the ideal
//! one and one under the recorded external torques with encoder noise on its measured joints. A
//! single pin entry can move the RMS, so the guard runs on three targets offset by a fraction of
//! a picometre and the worst counts.

use super::super::super::*;
use super::super::plant::as_run::Restart;
use super::super::plant::Limits;
use super::csv::{cartesian, Arm, Drive, Mode, Session};
use super::Config;
use crate::rate_limiting::fer::JOINT_POSITION_LIMITS;
use crate::rate_limiting::DELTA_T;

/// A joint closer than this to a limit, rad, puts its cycle near the limits.
const NEAR: f64 = 0.1;
/// Half the high-pass's centred mean, cycles.
pub(super) const HALF: usize = 10;
/// The most, rad/s, any joint's near-limit RMS may be. An absolute floor, not a tight one: the
/// metric moves 4× between variants that behave the same on the arm (on `wrist-b-2`: 0.006 to
/// 0.054 over five combinations of the same three rules), so it is informative, not the test. This
/// is under the 0.08 rad/s the Panda chattered at and well over the worst guarded run, 0.026; the
/// recordings replayed as they ran give 0.13 to 0.26.
pub(super) const NEAR_RMS: f64 = 0.05;
/// The most, Nm/s, the commanded torque rate's p99 may be: the acceptance test, because it is the
/// hardware symptom -- on the Panda the rate saturated at the robot's 1000 Nm/s limit. Guarded
/// runs measure 96 to 252 Nm/s, so this is a gate with room, far under the limiter.
pub(super) const RATE_P99: f64 = 400.0;
/// A cycle whose target moves slower than this, m/s, is slow.
pub(super) const SLOW: f64 = 0.02;
/// Half the window a target's speed is taken over, cycles.
pub(super) const SPEED_HALF: usize = 25;
/// The offsets, m, added to every target's position for the guarded runs.
const OFFSETS: [f64; 3] = [1e-13, -1e-13, 3e-13];
/// The white noise, rad, put on the measured joints of the disturbed plant: the arm's encoders.
const ENCODER_NOISE: f64 = 1e-5;

pub(super) struct Chatter {
    /// The high-pass RMS of `dq_goal` per joint near the limits and away from them, rad/s.
    pub near: [f64; 7],
    pub away: [f64; 7],
    /// The slow cycles near the limits and away from them.
    pub counts: [usize; 2],
    /// The largest joint's p99 of `|Δτ| / Δt`, Nm/s.
    pub rate_p99: f64,
}

impl Chatter {
    /// Of the records `sent` of a run whose arm was at `q`, one of each per cycle, over the
    /// cycles `slow` marks.
    pub(super) fn of(
        sent: &[CartesianSent],
        q: &[[f64; 7]],
        slow: &[bool],
        limits: &Limits,
    ) -> Chatter {
        let n = sent.len().min(q.len());
        let near =
            |k: usize| (0..7).any(|i| (q[k][i] - limits.0[i]).min(limits.1[i] - q[k][i]) < NEAR);
        let mut sums = [[0.0; 7]; 2];
        let mut counts = [0usize; 2];
        for k in (HALF..n.saturating_sub(HALF)).filter(|&k| slow[k]) {
            let window = &sent[k - HALF..=k + HALF];
            let side = usize::from(!near(k));
            counts[side] += 1;
            for (i, sum) in sums[side].iter_mut().enumerate() {
                let mean = window.iter().map(|r| r.dq_goal[i]).sum::<f64>() / window.len() as f64;
                *sum += (sent[k].dq_goal[i] - mean).powi(2);
            }
        }
        let rms = |side: usize| sums[side].map(|s| (s / counts[side].max(1) as f64).sqrt());
        let rate_p99 = (0..7)
            .map(|i| {
                let mut rates: Vec<f64> = sent[..n]
                    .windows(2)
                    .map(|w| (w[1].tau[i] - w[0].tau[i]).abs() / DELTA_T)
                    .collect();
                rates.sort_by(f64::total_cmp);
                rates
                    .get((rates.len().saturating_sub(1)) * 99 / 100)
                    .copied()
                    .unwrap_or(0.0)
            })
            .fold(0.0, f64::max);
        Chatter {
            near: rms(0),
            away: rms(1),
            counts,
            rate_p99,
        }
    }

    /// The largest joint's near-limit RMS, rad/s.
    fn peak(&self) -> f64 {
        self.near.iter().copied().fold(0.0, f64::max)
    }

    /// The largest ratio over the joints of the near RMS to the away one, NaN if any is.
    pub(super) fn ratio(&self) -> f64 {
        (0..7)
            .map(|i| self.near[i] / self.away[i])
            .fold(0.0, |a, r| if r.is_nan() || r > a { r } else { a })
    }
}

impl std::fmt::Display for Chatter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let round = |x: [f64; 7]| x.map(|v| (v * 1e4).round() / 1e4);
        write!(
            f,
            "dq_goal HP RMS near {:?} ({} cycles), away {:?} ({}) rad/s (ratio {:.2}), torque \
             rate p99 {:.0} Nm/s",
            round(self.near),
            self.counts[0],
            round(self.away),
            self.counts[1],
            self.ratio(),
            self.rate_p99
        )
    }
}

/// Per cycle of `session`, whether its target's position moves slower than [`SLOW`] over the
/// centred window; before the first target it holds.
fn slow(session: &Session) -> Vec<bool> {
    let n = session.t.len();
    let at = |k: usize| {
        let i = session.targets.partition_point(|t| t.0 <= k);
        session.targets.get(i.wrapping_sub(1)).map(|t| t.1)
    };
    (0..n)
        .map(|k| {
            let (from, to) = (k.saturating_sub(SPEED_HALF), (k + SPEED_HALF).min(n - 1));
            match (at(from), at(to)) {
                (Some(a), Some(b)) => {
                    let moved = (0..3).map(|i| (b[i] - a[i]).powi(2)).sum::<f64>().sqrt();
                    moved < SLOW * (to - from) as f64 * DELTA_T
                }
                _ => true,
            }
        })
        .collect()
}

/// The loop at `config` commanded over the whole of `session` on the FER with `targets`: as run
/// restarting as `restart` on the recorded arm, and guarded on the recorded arm and on two plants,
/// the ideal one and one under the recorded external torques with encoder noise on its `q`, at each
/// of [`OFFSETS`]. All are printed, ratios and as-run numbers included. Both plants are judged:
/// on the torque rate always, and on the near-limit RMS where the run has at least a high-pass
/// window of slow cycles near the limits, so that no run is left unjudged.
pub(super) fn check(
    name: &str,
    session: &Session,
    targets: &[(usize, [f64; 7])],
    (config, restart): (Config, Restart),
    failures: &mut Vec<String>,
) {
    let slow = slow(session);
    let run = |mode, targets: &[(usize, [f64; 7])], how| {
        let all = 0..session.t.len();
        let (sent, driven) = cartesian(session, targets, &Arm::fer(), config, mode, all, how);
        Chatter::of(&sent, &driven.q, &slow, &JOINT_POSITION_LIMITS)
    };
    let as_run = run(Mode::AsRun(restart), targets, Drive::Commanded);
    println!("{name}: chatter as run, recorded arm: {as_run}");
    let plants = [
        ("plant", Drive::plant(0.0)),
        (
            "plant, recorded disturbance and encoder noise",
            Drive::Closed {
                disturbance: 1.0,
                noise: ENCODER_NOISE,
            },
        ),
    ];
    for (arm, how) in [("recorded arm", Drive::Commanded)]
        .into_iter()
        .chain(plants)
    {
        let runs: Vec<Chatter> = OFFSETS
            .iter()
            .map(|offset| {
                let mut moved = targets.to_vec();
                moved
                    .iter_mut()
                    .for_each(|t| t.1[..3].iter_mut().for_each(|x| *x += offset));
                run(Mode::Guarded(0.1), &moved, how)
            })
            .collect();
        let peaks = runs.iter().map(Chatter::peak);
        let (low, high) = peaks.fold((f64::INFINITY, 0.0f64), |(l, h), p| (l.min(p), h.max(p)));
        let worst = runs
            .iter()
            .fold(&runs[0], |w, r| if r.peak() > w.peak() { r } else { w });
        let rate = runs.iter().map(|r| r.rate_p99).fold(0.0, f64::max);
        println!(
            "{name}: chatter guarded, {arm}: worst of {} offsets {worst}; near peak {low:.4} to \
             {high:.4} rad/s, rate p99 at most {rate:.0} Nm/s",
            OFFSETS.len()
        );
        if !matches!(how, Drive::Closed { .. }) {
            continue;
        }
        let judged = worst.counts[0] > 2 * HALF;
        if !judged {
            println!(
                "{name}, {arm}: under a high-pass window of slow cycles near the limits, judged \
                 on the torque rate alone"
            );
        }
        if (judged && high > NEAR_RMS) || rate > RATE_P99 {
            failures.push(format!("{name}, {arm}: guarded chatter {worst}"));
        }
    }
}
