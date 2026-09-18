//! The torque backend's joint velocity envelope, offline: the cap on the goal's step and the
//! barrier on the measured velocity.

use super::super::velocity::*;
use crate::lowpass_filter::low_pass_filter;
use crate::rate_limiting::fer::{MAX_JOINT_VELOCITY, MAX_TORQUE_RATE};
use crate::rate_limiting::{limit_rate_torques, DELTA_T};

const FROM: [f64; 7] = [0.1, -0.6, 0.2, -2.1, 0.0, 1.7, 0.9];
const MAX: [f64; 7] = [2e-3, 2e-3, 2e-3, 2e-3, 3e-3, 3e-3, 3e-3];

#[test]
fn a_step_inside_every_bound_is_left_alone() {
    let mut to: [f64; 7] = std::array::from_fn(|i| FROM[i] + 0.9 * MAX[i]);
    let before = to;
    assert_eq!(cap_step(&FROM, &mut to, &MAX), 1.0);
    assert_eq!(to, before);
    // Exactly at the bound is inside (joint 5 starts at zero, so the step is exact).
    let mut edge = FROM;
    edge[4] += MAX[4];
    assert_eq!(cap_step(&FROM, &mut edge, &MAX), 1.0);
}

#[test]
fn a_step_past_a_bound_is_scaled_whole_onto_it_and_keeps_its_direction() {
    let delta = [1e-3, -5e-3, 0.5e-3, 0.0, 9e-3, -1e-3, 4e-3];
    let mut to: [f64; 7] = std::array::from_fn(|i| FROM[i] + delta[i]);
    let scale = cap_step(&FROM, &mut to, &MAX);
    // Joint 5 binds: 3e-3 of 9e-3; joint 2's 2e-3 of 5e-3 is looser.
    assert!((scale - 3e-3 / 9e-3).abs() < 1e-12, "{scale}");
    for i in 0..7 {
        let step = to[i] - FROM[i];
        assert!(
            step.abs() <= MAX[i] + 1e-12,
            "joint {i}: {step} against {}",
            MAX[i]
        );
        assert!(
            (step - scale * delta[i]).abs() < 1e-12,
            "joint {i} left the direction"
        );
    }
    assert!(((to[4] - FROM[4]).abs() - MAX[4]).abs() < 1e-12);
}

#[test]
fn an_infinite_bound_never_caps() {
    let mut to = [1e3; 7];
    assert_eq!(cap_step(&FROM, &mut to, &[f64::INFINITY; 7]), 1.0);
    assert_eq!(to, [1e3; 7]);
}

#[test]
fn the_barrier_opposes_a_joint_past_its_onset_in_proportion_within_the_torque_limit() {
    let onset = [2.0; 7];
    let limits = [86.0, 86.0, 86.0, 86.0, 11.5, 11.5, 11.5];
    let dq = [1.99, 2.0, 2.1, -2.1, 2.5, -3.0, 10.0];
    let tau = velocity_barrier(&dq, &onset, &limits);
    assert_eq!(
        (tau[0], tau[1]),
        (0.0, 0.0),
        "at or below the onset, nothing"
    );
    assert!(
        (tau[2] + VELOCITY_BARRIER_GAIN * 0.1).abs() < 1e-9,
        "{}",
        tau[2]
    );
    assert!(
        (tau[3] - VELOCITY_BARRIER_GAIN * 0.1).abs() < 1e-9,
        "{}",
        tau[3]
    );
    assert!(
        (tau[4] + VELOCITY_BARRIER_GAIN * 0.5).abs() < 1e-9,
        "{}",
        tau[4]
    );
    assert_eq!(tau[5], 11.5, "20 Nm opposing, clamped to the joint's limit");
    assert_eq!(tau[6], -11.5);
    assert_eq!(velocity_barrier(&[f64::NAN; 7], &onset, &limits), [0.0; 7]);
}

#[test]
fn the_law_along_a_joints_motion_fades_from_the_start_to_none_at_the_onset() {
    let (start, onset) = ([2.0; 7], [3.0; 7]);
    // Along the motion: below the start, at it, half-way (negative), a quarter from the onset,
    // at the onset, past it (negative); and NaN.
    let law = [4.0, 4.0, -4.0, 4.0, 4.0, -4.0, 4.0];
    let dq = [1.0, 2.0, -2.5, 2.75, 3.0, -3.5, f64::NAN];
    assert_eq!(
        fade_push(&law, &dq, &start, &onset),
        [4.0, 4.0, -2.0, 1.0, 0.0, 0.0, 4.0]
    );
    // Against the motion, both ways, however fast: never scaled.
    let law = [4.0, -4.0, 4.0, -4.0, 0.0, 0.0, 0.0];
    let dq = [-3.5, 3.5, -2.5, 2.5, 3.5, -3.5, 0.0];
    assert_eq!(fade_push(&law, &dq, &start, &onset), law);
    // An envelope switched off, as the replay's reference has it: never scaled.
    let infinite = [f64::INFINITY; 7];
    let law = [4.0; 7];
    assert_eq!(fade_push(&law, &[1e3; 7], &infinite, &infinite), law);
}

#[test]
fn a_barrier_under_the_band_fades_from_rest_not_below_it() {
    // Cap and barrier at 0.1 of a 2 rad/s limit: the band would start the fade at -0.1 rad/s,
    // taking a third of the law's push off the moment the joint moves.
    let (limit, onset) = (2.0, 0.2);
    let start = fade_start(limit, onset, onset);
    assert_eq!(start, 0.0);
    let law = [4.0; 7];
    let fade = |speed: f64| fade_push(&law, &[speed; 7], &[start; 7], &[onset; 7])[0];
    assert_eq!(fade(0.0), 4.0);
    assert!((fade(1e-9) - 4.0).abs() < 1e-6, "{}", fade(1e-9));
    assert_eq!(fade(0.1), 2.0);
    assert_eq!(fade(0.2), 0.0);
    // Otherwise the cap, or the band under the onset where that is lower.
    assert_eq!(fade_start(limit, 1.4, 1.7), 1.4);
    assert!((fade_start(limit, 1.6, 1.7) - 1.4).abs() < 1e-12);
}

const WRIST: usize = 6;
const CLAMP: f64 = 11.5;

/// Joint 7 of an FER behind what the robot does with a torque command, for `cycles`: the law's
/// torque for the velocity measured a cycle before it is applied, faded from `fade_start` (a
/// fraction of the limit) to the barrier's onset at 0.85 when there is one, and with `barrier`
/// the barrier added; clamped, low-passed at 100 Hz against the last command, rate limited,
/// then applied for a cycle with `push` Nm from outside during the first 500. The velocity and
/// the applied torque after every cycle.
fn wrist(
    inertia: f64,
    law: impl Fn(f64) -> f64,
    fade_start: Option<f64>,
    barrier: bool,
    push: f64,
    cycles: usize,
) -> Vec<[f64; 2]> {
    let limit = MAX_JOINT_VELOCITY[WRIST];
    let onset = [0.85 * limit; 7];
    let (mut velocity, mut measured, mut last) = (0.0f64, 0.0f64, 0.0f64);
    (0..cycles)
        .map(|cycle| {
            let mut dq = [0.0; 7];
            dq[WRIST] = measured;
            let mut torque = [0.0; 7];
            torque[WRIST] = law(measured);
            if let Some(start) = fade_start {
                torque = fade_push(&torque, &dq, &[start * limit; 7], &onset);
            }
            if barrier {
                torque[WRIST] += velocity_barrier(&dq, &onset, &[CLAMP; 7])[WRIST];
            }
            let command = torque[WRIST].clamp(-CLAMP, CLAMP);
            let filtered = low_pass_filter(DELTA_T, command, last, 100.0).unwrap();
            last = limit_rate_torques(&MAX_TORQUE_RATE, &[filtered; 7], &[last; 7]).unwrap()[WRIST];
            measured = velocity;
            let outside = if cycle < 500 { push } else { 0.0 };
            velocity += (last + outside) / inertia * DELTA_T;
            [velocity, last]
        })
        .collect()
}

/// The largest magnitude of column `k` of `run`.
fn peak(run: &[[f64; 2]], k: usize) -> f64 {
    run.iter().fold(0.0, |m, s| m.max(s[k].abs()))
}

/// The peak-to-peak of column `k` of `run`.
fn spread(run: &[[f64; 2]], k: usize) -> f64 {
    let (lo, hi) = run.iter().fold((f64::MAX, f64::MIN), |(lo, hi), s| {
        (lo.min(s[k]), hi.max(s[k]))
    });
    hi - lo
}

#[test]
fn the_barrier_settles_a_pushed_wrist_under_its_limit_and_lets_it_rest() {
    let limit = MAX_JOINT_VELOCITY[WRIST];
    let onset = 0.85 * limit;
    // 0.04 kg m²: half the wrist armature of MuJoCo's FR3, over ten times the link's own. The
    // Cartesian preset's joint damping on the wrist, without and with a Cartesian share.
    // A transient overshoot past the limit is expected; the barrier does not bound it.
    for (damping, push) in [(1.0, 6.0), (3.0, 11.0)] {
        let free = wrist(0.04, |v| -damping * v, None, false, push, 1000);
        assert!(
            free[499][0] > limit,
            "the push alone drives it to {} rad/s",
            free[499][0]
        );
        let held = wrist(0.04, |v| -damping * v, None, true, push, 1000);
        // The static balance push = damping v + gain (v - onset), below the limit.
        let expected = (push + VELOCITY_BARRIER_GAIN * onset) / (damping + VELOCITY_BARRIER_GAIN);
        assert!(
            (held[499][0] - expected).abs() < 0.02,
            "{} against {expected}",
            held[499][0]
        );
        assert!(held[499][0] < limit);
        // Released, it settles: nothing rings on.
        let (tail, pushed) = (peak(&held[900..], 0), peak(&held[..500], 0));
        assert!(tail < 0.01 * pushed, "{tail} rad/s left of {pushed}");
    }
}

#[test]
fn the_same_wrist_rings_when_the_loop_gain_passes_the_bound() {
    // 15 Nm s/rad on 0.01 kg m² is 1.5 of damping per cycle, past the 1 the loop tolerates:
    // the harness sees it, so the case above is evidence.
    let ringing = wrist(0.01, |v| -15.0 * v, None, true, 11.0, 1000);
    let (tail, pushed) = (peak(&ringing[900..], 0), peak(&ringing[..500], 0));
    assert!(tail > 0.5 * pushed, "{tail} rad/s left of {pushed}");
}

/// A wrist pulled `pull` Nm along its motion (lagging its goal) and damped by `damping` on its
/// velocity error to that goal, which moves at the cap, with the fade from `fade_start` and the
/// barrier: the last 1000 of 4000 cycles.
fn pulled(inertia: f64, damping: f64, pull: f64, cap: f64, fade_start: f64) -> Vec<[f64; 2]> {
    let goal = cap * MAX_JOINT_VELOCITY[WRIST];
    let law = |v: f64| pull + damping * (goal - v);
    wrist(inertia, law, Some(fade_start), true, 0.0, 4000).split_off(3000)
}

#[test]
fn a_wrist_pulled_along_its_motion_settles_at_most_at_the_onset_through_the_fade() {
    let limit = MAX_JOINT_VELOCITY[WRIST];
    // The loop's fade start for a cap of 0.7 and of 0.8 under the onset at 0.85: 0.7 both.
    for cap in [0.7f64, 0.8] {
        let start = cap.min(0.85 - FADE_BAND);
        for inertia in [0.03, 0.04] {
            for damping in [1.0, 5.0, 15.0] {
                for pull in [5.0, 11.5] {
                    let tail = pulled(inertia, damping, pull, cap, start);
                    let case =
                        format!("cap {cap}, {inertia} kg m², damping {damping}, pull {pull}");
                    assert!(
                        spread(&tail, 1) < 0.01 * pull,
                        "{case}: {} Nm left ringing",
                        spread(&tail, 1)
                    );
                    // At the onset, or under it where the damping balances the pull first.
                    let speed = peak(&tail, 0) / limit;
                    assert!(speed < 0.85 + 1e-3, "{case}: at {speed} of the limit");
                    assert!(spread(&tail, 0) < 1e-3 * limit, "{case}: still moving");
                }
            }
        }
    }
}

#[test]
fn the_same_pull_rings_on_a_fade_narrower_than_the_band() {
    // Fading from a cap of 0.8 to the onset, a third of the band, adds 90 Nm per rad/s at the
    // full clamp: past what 0.03 kg m² tolerates, so the case above is evidence.
    let tail = pulled(0.03, 1.0, 11.5, 0.8, 0.8);
    assert!(spread(&tail, 1) > 5.0, "{} Nm", spread(&tail, 1));
}
