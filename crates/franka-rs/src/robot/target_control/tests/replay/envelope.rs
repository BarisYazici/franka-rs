//! Joint velocities against an arm's limit: the share of the limit per joint, and what the
//! recordings did against the FER's flat limit, which is the limit the robot itself
//! checks. A recording that goes over it without a reflex would put the flat limit in doubt, so
//! the shares are asserted, with the one recorded `joint_velocity_violation` allowed for.

use std::ops::Range;

use super::super::super::position::VelocityLimit;
use super::csv::Session;
use crate::wire::robot::codec::FciVersion;

/// Per joint `|v| / limit` at `q`, the limit toward `v`'s side: 0 at rest, infinite where that
/// limit is 0.
fn share(velocity: &VelocityLimit, q: &[f64; 7], v: &[f64; 7]) -> [f64; 7] {
    let (lower, upper) = velocity.at(q);
    std::array::from_fn(|i| {
        let limit = if v[i] < 0.0 { lower[i] } else { upper[i] };
        match v[i].abs() {
            0.0 => 0.0,
            speed if limit > 0.0 => speed / limit,
            _ => f64::INFINITY,
        }
    })
}

/// Per joint, the largest share of `version`'s limit over `cycles` of `q` and `v`, the end of
/// the step `v` came from.
pub(super) fn shares(
    version: FciVersion,
    q: &[[f64; 7]],
    v: &[[f64; 7]],
    cycles: Range<usize>,
) -> [f64; 7] {
    let velocity = VelocityLimit::of(version);
    cycles.fold([0.0; 7], |worst, k| {
        let share = share(&velocity, &q[k], &v[k]);
        std::array::from_fn(|i| worst[i].max(share[i]))
    })
}

/// Over `cycles` of the recorded arm: the largest share of the FER's flat limit per joint, the
/// cycles at or above 1, and the first such cycle and joint.
fn recorded(
    session: &Session,
    cycles: Range<usize>,
) -> ([f64; 7], [usize; 7], Option<(usize, usize)>) {
    let velocity = VelocityLimit::of(FciVersion::V5);
    let (mut worst, mut over, mut first) = ([0.0f64; 7], [0; 7], None);
    for k in cycles {
        let share = share(&velocity, &session.q[k], &session.dq[k]);
        for i in 0..7 {
            worst[i] = worst[i].max(share[i]);
            if share[i] >= 1.0 {
                over[i] += 1;
                first.get_or_insert((k, i));
            }
        }
    }
    (worst, over, first)
}

/// The share of the flat limit a recorded arm may reach. None of these sessions comes near it
/// (the worst is 0.75), and the one recording ending in a `joint_velocity_violation` (joint 5 at
/// 1.36 of this limit) is the robot checking it. A recording that
/// ran over it without a reflex would mean the arm checks something else.
const RECORDED_ALLOWANCE: f64 = 1.0;

/// Prints the recorded arm of `name` against the FER's flat limit — over the last 2 s before the
/// session's end (its fault), over the whole session, and over each of `windows` (s) — and fails
/// on a session that went over it.
pub(super) fn recorded_on_the_fer(
    name: &str,
    session: &Session,
    windows: &[(f64, f64)],
    failures: &mut Vec<String>,
) {
    let n = session.t.len();
    let (last, _, _) = recorded(session, n.saturating_sub(2000)..n);
    let (whole, over, first) = recorded(session, 0..n);
    let first = first.map(|(k, i)| {
        format!(
            "joint {} {} ms before the end at {:.3} rad/s",
            i + 1,
            n - 1 - k,
            session.dq[k][i]
        )
    });
    println!(
        "{name}: recorded share of the FER's flat limit: last 2 s {last:.3?}, whole session \
         {whole:.3?}, cycles at or above 1 {over:?}, first {first:?}"
    );
    let worst = whole.into_iter().fold(0.0, f64::max);
    if worst > RECORDED_ALLOWANCE {
        failures.push(format!(
            "{name}: recorded at {worst:.3} of the FER's flat limit with no reflex: the flat limit \
             is not what the robot checks"
        ));
    }
    for &(t0, t1) in windows {
        let at = |t: f64| session.t.partition_point(|x| *x < t);
        let (worst, _, _) = recorded(session, at(t0)..at(t1));
        println!("{name} {t0:.2}-{t1:.2}: recorded share of the FER's flat limit {worst:.3?}");
    }
}
