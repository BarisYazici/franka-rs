//! The position envelope of the torque loop: the fade toward a limit, the spring, the onset and
//! fade start at the measured state, the barrier's gain, offline; a joint driven into its
//! margin behind what the robot does with a torque command. The loops against a limit in
//! [`scenarios`], a Cartesian target beyond one in [`fold`].

mod fold;
mod scenarios;
mod slide;
mod wrist;

use nalgebra::Matrix3;

use super::super::ik::stall::{Stall, Wall};
use super::super::position::{spring_ramp, JointLimits, VelocityLimit};
use super::super::runner::Step;
use super::super::torque::{ahead, landed, landing_dwell, left, restart, Pressure, Walls};
use super::super::velocity::{barrier_gains, fade_push, fade_start, velocity_barrier};
use super::super::*;
use crate::lowpass_filter::low_pass_filter;
use crate::rate_limiting::fer::MAX_TORQUE_RATE;
use crate::rate_limiting::{limit_rate_torques, DELTA_T};

const MARGIN: f64 = 0.05;
const ONSET: f64 = MARGIN - POSITION_FADE_BAND;

fn limits(version: FciVersion) -> JointLimits {
    JointLimits {
        position: joint_position_limits(version),
        margin: MARGIN,
        fraction: 0.7,
        velocity: VelocityLimit::of(version),
    }
}

/// Mid-range on every joint, `s` from joint `i`'s lower limit.
fn at_lower(limits: &JointLimits, i: usize, s: f64) -> [f64; 7] {
    let (lower, upper) = limits.position;
    let mut q: [f64; 7] = std::array::from_fn(|j| 0.5 * (lower[j] + upper[j]));
    q[i] = lower[i] + s;
    q
}

#[test]
fn the_push_toward_a_limit_fades_linearly_over_the_band_and_away_is_left_alone() {
    let fer = limits(FciVersion::V5);
    for (s, kept) in [
        (0.2, 1.0),
        (MARGIN, 1.0),
        (0.04, 0.5),
        (ONSET, 0.0),
        (0.01, 0.0),
    ] {
        let q = at_lower(&fer, 3, s);
        let mut toward = [0.0; 7];
        toward[3] = -4.0;
        let faded = fer.fade_toward(&toward, &q);
        assert!((faded[3] + 4.0 * kept).abs() < 1e-9, "at {s}: {faded:?}");
        let away = toward.map(|t| -t);
        assert_eq!(fer.fade_toward(&away, &q), away, "at {s}");
        // The upper side of joint 1, mirrored.
        let (_, upper) = fer.position;
        let mut q = q;
        q[0] = upper[0] - s;
        let push = [4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!((fer.fade_toward(&push, &q)[0] - 4.0 * kept).abs() < 1e-9);
        assert_eq!(fer.fade_toward(&push.map(|t| -t), &q)[0], -4.0);
    }
}

/// The two fades multiply, and on the FER the velocity fade's own band is the narrower one
/// inside the margin, so the bound on the composed gain is the position fade's: see *The
/// impedance backend* in the book.
#[test]
fn the_two_fades_compose_to_a_finite_gain_peaking_at_the_margin() {
    let fer = limits(FciVersion::V5);
    let flat = fer.velocity.flat();
    let options = ImpedanceOptions::cartesian();
    let (clamp, vbf) = (options.torque_limits, options.velocity_barrier_fraction);
    // The fade's start and the Nm per rad/s the composed fade takes off a law of `clamp[i]`
    // pushing joint `i` toward its lower limit, `s` away from it: the secant between two speeds
    // inside the band, which `fade_start < onset` keeps non-empty wherever `onset > 0`.
    let fade = |i: usize, s: f64| {
        let q = at_lower(&fer, i, s);
        // Barely moving toward the limit: it picks that side, and at the margin the cycle's
        // lookahead (which only lowers the onset there, so only raises the gain) is out of it.
        let (start, onset) = fer.envelope(&q, &[-1e-9; 7], vbf);
        assert!(onset[i] > start[i], "joint {i} at {s}: the band is empty");
        let kept = |speed: f64| {
            let (mut law, mut dq) = ([0.0; 7], [0.0; 7]);
            (law[i], dq[i]) = (-clamp[i], -speed);
            fer.fade_toward(&fade_push(&law, &dq, &start, &onset), &q)[i]
        };
        let quarter = 0.25 * (onset[i] - start[i]);
        let (a, b) = (start[i] + quarter, start[i] + 3.0 * quarter);
        (start[i], (kept(a) - kept(b)).abs() / (b - a))
    };
    let gain = |i: usize, s: f64| fade(i, s).1;
    // Joints 2 and 4, the FER's two extremes of the factor below.
    for (i, peak_expected, factor_expected) in [(1, 428.9, 1.61), (3, 357.4, 1.33)] {
        // Inside the margin the cap is 0, so the fade starts at rest: no `FADE_BAND` floor.
        assert!(fade(i, MARGIN).0 < 1e-12);
        assert!(fade(i, ONSET + 1e-4).0 < 1e-12);
        let peak = gain(i, MARGIN);
        let closed = clamp[i] / (vbf * fer.velocity.braking(i, POSITION_FADE_BAND, flat[i]));
        assert!((peak - closed).abs() < 1e-9 * closed, "joint {i}: {peak}");
        assert!((peak - peak_expected).abs() < 0.05, "joint {i}: {peak}");
        // Finite, and no sample of the margin and beyond it beats the margin itself.
        for k in 1..=500 {
            let s = ONSET + f64::from(k) * (0.12 - ONSET) / 500.0;
            let at = gain(i, s);
            assert!(
                at.is_finite() && at <= peak * (1.0 + 1e-9),
                "joint {i} at {s}: {at}"
            );
        }
        // Above the band alone, which away from a limit is what bounds the gain.
        let band_alone = clamp[i] / (FADE_BAND * flat[i]);
        assert!(
            (peak / band_alone - factor_expected).abs() < 0.005,
            "joint {i}: {}",
            peak / band_alone
        );
    }
}

#[test]
fn the_spring_pushes_out_inside_the_onset_within_the_clamp_and_ramps_in() {
    let fer = limits(FciVersion::V5);
    let clamp = ImpedanceOptions::cartesian().torque_limits;
    for (i, s, expected) in [
        (3, ONSET, 0.0),
        (3, ONSET + 0.01, 0.0),
        (3, ONSET - 0.01, POSITION_BARRIER_STIFFNESS * 86.0 * 0.01),
        (6, ONSET - 0.01, POSITION_BARRIER_STIFFNESS * 11.5 * 0.01),
        (3, -0.2, 86.0),
    ] {
        let q = at_lower(&fer, i, s);
        let spring = fer.spring(&q, &clamp, 1.0);
        assert!(
            (spring[i] - expected).abs() < 1e-9,
            "joint {i} at {s}: {spring:?}"
        );
        let mut others = spring;
        others[i] = 0.0;
        assert_eq!(others, [0.0; 7]);
        // The upper side pushes down, the same amount.
        let mut q = q;
        q[i] = fer.position.1[i] - s;
        q[3] = if i == 3 {
            q[i]
        } else {
            at_lower(&fer, 3, 0.5)[3]
        };
        assert!((fer.spring(&q, &clamp, 1.0)[i] + expected).abs() < 1e-9);
        // Half way through the ramp, half of it; none at the start.
        let half = fer.spring(&at_lower(&fer, i, s), &clamp, spring_ramp(250))[i];
        let expected = if s < 0.0 { clamp[i] } else { 0.5 * expected };
        assert!((half - expected).abs() < 1e-9, "{half}");
    }
    assert_eq!(spring_ramp(0), 0.0);
    assert_eq!((spring_ramp(500), spring_ramp(u32::MAX)), (1.0, 1.0));
}

#[test]
fn the_onset_and_the_fade_start_come_down_toward_the_limit_the_joint_moves_to() {
    let fer = limits(FciVersion::V5);
    let flat = max_joint_velocity(FciVersion::V5);
    let (vbf, f) = (0.85, fer.fraction);
    // Far from every limit, bit for bit what the loop used before the guard.
    let q = at_lower(&fer, 3, 0.5 * (fer.position.1[3] - fer.position.0[3]));
    for sign in [1.0, -1.0] {
        let (start, onset) = fer.envelope(&q, &[sign * 0.1; 7], vbf);
        for i in [0, 4, 6] {
            assert_eq!(onset[i].to_bits(), (flat[i] * vbf).to_bits());
            let today = fade_start(flat[i], flat[i] * f, flat[i] * vbf);
            assert_eq!(start[i].to_bits(), today.to_bits());
        }
    }
    // Joint 4 0.1 from its lower limit: brought down toward it, the lower of here and where the
    // cycle's velocity takes it, flat away from it; zero at the barrier's onset.
    let q = at_lower(&fer, 3, 0.1);
    let (start, onset) = fer.envelope(&q, &[-0.1; 7], vbf);
    let at = |s: f64| {
        let v = fer.velocity.at(&at_lower(&fer, 3, s)).0[3];
        let onset = vbf * v.min(fer.velocity.braking(3, s - ONSET, flat[3]));
        let cap = f * v.min(fer.velocity.braking(3, s - MARGIN, flat[3]));
        (fade_start(v, cap, onset), onset, cap)
    };
    let (now, next) = (at(0.1), at(0.1 - 0.1 * DELTA_T));
    assert!((onset[3] - now.1.min(next.1)).abs() < 1e-12, "{}", onset[3]);
    assert!((start[3] - now.0.min(next.0)).abs() < 1e-12, "{}", start[3]);
    assert!(onset[3] < 0.85 * flat[3] && start[3] < now.2 + 1e-15);
    assert_eq!(fer.envelope(&q, &[0.1; 7], vbf).1[3], vbf * flat[3]);
    let (start, onset) = fer.envelope(&at_lower(&fer, 3, ONSET), &[-0.1; 7], vbf);
    assert_eq!((start[3], onset[3]), (0.0, 0.0));
}

#[test]
fn the_onset_and_fade_start_stay_under_the_envelope_now_and_a_cycle_on() {
    for version in [FciVersion::V5, FciVersion::V10] {
        let arm = limits(version);
        let (lower, upper) = arm.position;
        // The limit toward `dq` at `q` and where `dq` takes it in a cycle, the lower of the two.
        let limit = |q: &[f64; 7], dq: &[f64; 7], i: usize| {
            let next: [f64; 7] = std::array::from_fn(|j| q[j] + dq[j] * DELTA_T);
            let toward = |q: &[f64; 7]| {
                let (lower, upper) = arm.velocity.at(q);
                if dq[i] < 0.0 {
                    lower[i]
                } else {
                    upper[i]
                }
            };
            toward(q).min(toward(&next))
        };
        for i in 0..7 {
            for speed in [0.1, 2.0] {
                for k in 0..=2000 {
                    let s = f64::from(k) * (upper[i] - lower[i]) / 2000.0;
                    for toward_upper in [false, true] {
                        let mut q = at_lower(&arm, i, s);
                        let mut dq = [0.0; 7];
                        (q[i], dq[i]) = if toward_upper {
                            (upper[i] - s, speed)
                        } else {
                            (q[i], -speed)
                        };
                        let (start, onset) = arm.envelope(&q, &dq, 0.85);
                        let v = limit(&q, &dq, i);
                        let case = format!("{version:?} joint {i} at {s}, {dq:?}");
                        assert!(onset[i] <= 0.85 * v + 1e-12, "{case}");
                        assert!(start[i] <= 0.7 * v + 1e-12, "{case}");
                    }
                }
            }
        }
    }
}

#[test]
fn the_barrier_gain_is_the_constant_at_the_default_clamps_and_lowered_under_them() {
    let presets = ImpedanceOptions::cartesian().torque_limits;
    assert_eq!(barrier_gains(&presets), [VELOCITY_BARRIER_GAIN; 7]);
    assert_eq!(barrier_gains(&RATED_TORQUES), [VELOCITY_BARRIER_GAIN; 7]);
    let gains = barrier_gains(&[43.0, 86.0, 86.0, 86.0, 11.5, 11.5, 1.0]);
    assert_eq!(gains[0], 0.5 * VELOCITY_BARRIER_GAIN);
    assert!((gains[6] - VELOCITY_BARRIER_GAIN / 11.5).abs() < 1e-12);
}

/// One joint of an arm behind what the robot does with a torque command, as
/// [`velocity`](super::velocity)'s wrist: `inertia`, moving toward its lower limit at `entry`
/// rad/s from where the goal would still move that fast, with the law pushing `push` Nm toward
/// it and damping `damping` Nm s/rad, the envelope and the spring on. Every cycle's distance to
/// the limit, velocity and applied torque.
fn entering(
    limits: &JointLimits,
    joint: usize,
    inertia: f64,
    entry: f64,
    push: f64,
    damping: f64,
    clamp: [f64; 7],
) -> Vec<[f64; 3]> {
    let lower = limits.position.0[joint];
    let gains = barrier_gains(&clamp);
    let flat = limits.velocity.flat()[joint];
    let braking = |x: f64| limits.fraction * limits.velocity.braking(joint, x, flat);
    let from = (0..)
        .map(|k| f64::from(k) * 1e-4)
        .find(|&x| braking(x) >= entry)
        .unwrap();
    let mut q = at_lower(limits, joint, MARGIN + from);
    let (mut velocity, mut measured, mut last) = (-entry, (q[joint], -entry), 0.0f64);
    (0..8000)
        .map(|cycle| {
            let (mut seen, mut dq) = (q, [0.0; 7]);
            (seen[joint], dq[joint]) = measured;
            let mut law = [0.0; 7];
            law[joint] = (-push - damping * dq[joint]).clamp(-clamp[joint], clamp[joint]);
            let (start, onset) = limits.envelope(&seen, &dq, 0.85);
            let kept = limits.fade_toward(&fade_push(&law, &dq, &start, &onset), &seen);
            let barrier = velocity_barrier(&dq, &onset, &gains, &clamp);
            let spring = limits.spring(&seen, &clamp, spring_ramp(cycle + 500));
            let command =
                (kept[joint] + barrier[joint] + spring[joint]).clamp(-clamp[joint], clamp[joint]);
            let filtered = low_pass_filter(DELTA_T, command, last, 100.0).unwrap();
            last = limit_rate_torques(&MAX_TORQUE_RATE, &[filtered; 7], &[last; 7]).unwrap()[joint];
            measured = (q[joint], velocity);
            velocity += last / inertia * DELTA_T;
            q[joint] += velocity * DELTA_T;
            [q[joint] - lower, velocity, last]
        })
        .collect()
}

/// What `limits` lets `joint` move at toward its lower limit at `s` from it: the arm's own limit
/// there -- flat on the FER, the envelope on the FR3 -- and the braking profile to the limit
/// itself, the speed the joint can still come to rest from before it.
fn toward_lower(limits: &JointLimits, joint: usize, s: f64) -> f64 {
    let flat = limits.velocity.flat()[joint];
    limits.velocity.at(&at_lower(limits, joint, s)).0[joint]
        .min(limits.velocity.braking(joint, s, flat))
}

/// The distance's minimum and the applied torque's spread over the last 500 cycles of `run`.
fn closest_and_tail_spread(run: &[[f64; 3]]) -> (f64, f64) {
    let closest = run.iter().map(|r| r[0]).fold(f64::INFINITY, f64::min);
    let tail = run[run.len() - 500..].iter().map(|r| r[2]);
    let (lo, hi) = tail.fold((f64::MAX, f64::MIN), |(lo, hi), t| (lo.min(t), hi.max(t)));
    (closest, hi - lo)
}

#[test]
fn a_joint_driven_into_its_margin_stops_short_of_the_limit_and_rests() {
    // Joint 2 at 0.25 rad/s, joint 4 on its link inertia and more, the wrists at their armature;
    // the law's push from a trace to the clamp. It settles at the barrier's onset, where the
    // spring balances the push.
    let clamp = ImpedanceOptions::cartesian().torque_limits;
    for version in [FciVersion::V5, FciVersion::V10] {
        let limits = limits(version);
        for (joint, inertia, damping, entries) in [
            (1, 0.44, 25.0, &[0.25][..]),
            (3, 0.2, 25.0, &[0.3, 0.63, 1.26][..]),
            (3, 0.44, 25.0, &[0.3, 0.63, 1.26][..]),
            (3, 1.0, 25.0, &[0.3, 0.63][..]),
            (4, 0.03, 6.0, &[0.3, 1.5][..]),
            (6, 0.04, 6.0, &[0.3, 1.5][..]),
        ] {
            for &entry in entries {
                for push in [1.6, 11.0, clamp[joint]] {
                    let run = entering(&limits, joint, inertia, entry, push, damping, clamp);
                    let case = format!(
                        "{version:?} joint {} {inertia} kg m² at {entry} rad/s, {push} Nm",
                        joint + 1
                    );
                    let (closest, spread) = closest_and_tail_spread(&run);
                    assert!(closest >= 0.010, "{case}: {closest} rad from the limit");
                    assert!(spread < 0.01 * push, "{case}: {spread} Nm left ringing");
                    // Under the limit, but where the push accelerates the joint at 300 rad/s²
                    // and more (15 times either arm's limit, so not a motion either can make):
                    // the fade lags behind the low-pass, 0.026 rad/s over it on the FR3's joint
                    // 4 at 0.2 kg m² and 86 Nm, the only case that leaves it. The slack is not
                    // that measurement but the 0.046 rad/s the robot's own limit keeps in hand,
                    // `LIMIT_EPS` plus three lost packets at the FER's joint acceleration limit.
                    let slack = if push / inertia < 300.0 { 1e-9 } else { 0.045 };
                    for (k, r) in run.iter().enumerate() {
                        let bound = toward_lower(&limits, joint, r[0]);
                        assert!(
                            -r[1] <= bound + slack,
                            "{case}: cycle {k} {r:?} over {bound} by more than {slack}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn the_same_joint_rings_when_the_barrier_passes_the_loop_bound() {
    // The barrier's 20 Nm s/rad on 0.005 kg m² is 4 of damping per cycle, the law's own 0.2: the
    // harness sees it, so the case above is evidence.
    let clamp = ImpedanceOptions::cartesian().torque_limits;
    let run = entering(&limits(FciVersion::V5), 6, 0.005, 1.5, 11.0, 1.0, clamp);
    let (_, spread) = closest_and_tail_spread(&run);
    assert!(spread > 1.0, "{spread} Nm");
}

/// A step of the six-axis generator from the origin that planned `plan` and moves at `v`, `a`.
fn planned(plan: [f64; 3], v: [f64; 3], a: [f64; 3]) -> Step<6, 7> {
    let six = |x: [f64; 3]| [x[0], x[1], x[2], 0.0, 0.0, 0.0];
    Step {
        position: six(plan),
        velocity: six(v),
        acceleration: six(a),
        target: [0.0; 7],
        hold: false,
        finished: false,
    }
}

/// Held, a joint still braking toward its margin.
const HELD: Pressure = Pressure::Position { rest: [0.0; 2] };

/// Joint 4 on its lower bound, withholding along `n`.
fn joint_4(n: [f64; 3]) -> Wall {
    Wall {
        normal: nalgebra::Vector3::from(n),
        joints: [0, 0, 0, -1, 0, 0, 0],
    }
}

/// Held against a wall along `n` in the translation block.
fn wall(n: [f64; 3]) -> Walls {
    Walls {
        blocks: [Some((joint_4(n), 0)), None],
    }
}

/// [`restart`]'s translation block from the origin to `went`, on no wall.
fn restarted(
    step: &Step<6, 7>,
    went: [f64; 3],
    cut: f64,
    pressure: Pressure,
) -> ([f64; 3], [f64; 3], f64) {
    restarted_on(&Walls::default(), step, went, cut, pressure)
}

/// [`restarted`] against `walls`.
fn restarted_on(
    walls: &Walls,
    step: &Step<6, 7>,
    went: [f64; 3],
    cut: f64,
    pressure: Pressure,
) -> ([f64; 3], [f64; 3], f64) {
    let origin = ([0.0; 3], Matrix3::identity());
    let out = (went, Matrix3::identity());
    let (v, a, fractions) = restart(step, (&origin, &origin), &out, cut, pressure, walls);
    let fraction = fractions[0].min(fractions[1]);
    assert!(
        v[3..].iter().chain(&a[3..]).all(|x| *x == 0.0),
        "the rotation planned nothing"
    );
    ([v[0], v[1], v[2]], [a[0], a[1], a[2]], fraction)
}

#[test]
fn against_a_position_limit_the_velocity_into_it_is_taken_out_and_the_rest_slides_on() {
    // Planned diagonally, held on y: y's velocity and acceleration go, x's stay.
    let y = wall([0.0, 1.0, 0.0]);
    let step = planned([1e-3, 1e-3, 0.0], [0.7, 0.7, 0.0], [2.0, 2.0, 0.0]);
    let (v, a, fraction) = restarted_on(&y, &step, [1e-3, 0.0, 0.0], 1.0, HELD);
    assert!((v[0] - 0.7).abs() < 1e-12 && v[1].abs() < 1e-12, "{v:?}");
    assert_eq!(a, [2.0, 0.0, 0.0]);
    assert!((fraction - 0.5).abs() < 1e-12);
    // Away from it, the acceleration is kept.
    let step = planned([1e-3, 1e-3, 0.0], [0.7, 0.7, 0.0], [-2.0, -2.0, 0.0]);
    let (_, a, _) = restarted_on(&y, &step, [1e-3, 0.0, 0.0], 1.0, HELD);
    assert_eq!(a, [-2.0, -2.0, 0.0]);
    // Straight into it: at rest, nothing to accelerate with.
    let x = wall([1.0, 0.0, 0.0]);
    let step = planned([1e-3, 0.0, 0.0], [0.7, 0.0, 0.0], [2.0, 0.0, 0.0]);
    let (v, a, fraction) = restarted_on(&x, &step, [0.0; 3], 1.0, HELD);
    assert_eq!((v, a, fraction), ([0.0; 3], [0.0; 3], 0.0));
}

#[test]
fn what_went_out_is_the_goal_s_own_step_not_its_lag_behind_the_generator() {
    // The goal a step behind the anchor made this cycle's whole step: from the anchor nothing went
    // out, which would take the velocity into the wall, or on a velocity pin all of it.
    let x = wall([1.0, 0.0, 0.0]);
    let step = planned([1e-4, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0; 3]);
    let anchor = ([0.0; 3], Matrix3::identity());
    let last = ([-1e-4, 0.0, 0.0], Matrix3::identity());
    for (walls, pressure) in [(x, HELD), (Walls::default(), Pressure::Velocity)] {
        let (v, _, fraction) = restart(&step, (&anchor, &last), &anchor, 1.0, pressure, &walls);
        assert_eq!((v[0], fraction[0]), (0.1, 1.0), "{pressure:?}");
        let (v, _, fraction) = restart(&step, (&anchor, &anchor), &anchor, 1.0, pressure, &walls);
        assert_eq!((v[0], fraction[0]), (0.0, 0.0), "{pressure:?}");
    }
}

/// The Cartesian budget's jerk per axis, as a session's generator has it.
fn jerks() -> [f64; 6] {
    let options = TargetControlOptions::default();
    super::super::cartesian::axis_limits(options.limits, options.rotation_limits)
        .map(|a| a.max_jerk)
}

#[test]
fn a_stop_lands_on_a_wall_the_target_lies_beyond_once_the_goal_has_stood_still_against_it() {
    let x = wall([0.8, 0.0, 0.6]);
    let out = ([0.0; 3], Matrix3::identity());
    let target = |p: [f64; 3]| [p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0];
    let rest = [0.0; 6];
    let at = |t: [f64; 3], w: &Walls| landed(&target(t), &out, w, 1e-3, &rest);
    // 2 cm beyond the wall on x and z and 1.1 mm along it: landed, wherever the rest lies, as
    // long as the target is on the far side of the wall. The dwell -- that the goal has stood
    // still against the wall -- lives at the one call site, not here.
    assert!(at([0.02, 1.1e-3, 0.001], &x));
    // Without a wall, or with the target on its near side, only the tolerance lands it.
    assert!(!at([0.02, 0.0, 0.0], &Walls::default()));
    assert!(!at([9e-4, 0.0, 0.0], &Walls::default()));
    assert!(!at([-0.02, 0.0, 0.0], &x));
    // A wall tilted under 30 degrees off an axis lands the same: no axis has to hold the rest.
    let tilted = wall([0.94, 0.34, 0.0]);
    assert!(at([0.02, 0.00723, 0.0], &tilted));
    // A wall orthogonal to what is left -- a rotation-only target, say, the translation reached:
    // the block is not beyond it, so it is judged on the tolerance, and the wall still holds the
    // generator, which is what lands the stop. Judged on being beyond alone it never lands and
    // the stop runs its whole timeout.
    let y = wall([0.0, 1.0, 0.0]);
    assert!(at([9e-4, 0.0, 0.0], &y));
    // And a wall that holds back only rounding does not excuse the rest: the floor on the normal.
    assert!(!at([0.02, 1e-9, 0.0], &y));
    // Still running: beyond the wall, but faster than the landing speed.
    let moving = [0.02, 0.0, 0.015, 0.0, 0.0, 0.0];
    assert!(!landed(
        &target([0.02, 0.0, 0.015]),
        &out,
        &x,
        1e-3,
        &moving
    ));
    let creeping = moving.map(|v| v * 1e-2);
    assert!(landed(
        &target([0.02, 0.0, 0.015]),
        &out,
        &x,
        1e-3,
        &creeping
    ));
}

#[test]
fn a_stop_waits_out_a_dwell_a_generator_from_rest_covers_two_tolerances_in() {
    let jerks = jerks();
    let dwell = landing_dwell(&jerks, 1e-3);
    println!("landing dwell {dwell} cycles");
    let (mut acceleration, mut speed, mut moved) = (0.0, 0.0, 0.0);
    for _ in 0..dwell {
        acceleration += jerks[0] * DELTA_T;
        speed += acceleration * DELTA_T;
        moved += speed * DELTA_T;
    }
    println!("a generator from rest covers {moved:.5} m in it");
    assert!((1.5e-3..3e-3).contains(&moved), "{moved} m");
    // What is left to the target, per axis: the translation, then the rotation.
    let turned = super::super::rotation::exp(&[0.0, 0.0, 0.02]);
    let out = ([0.01, 0.0, 0.0], turned);
    let upright = [0.012, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    let left = left(&upright, &out);
    assert!(
        (left[0] - 0.002).abs() < 1e-9 && (left[5] + 0.02).abs() < 1e-9,
        "{left:?}"
    );
}

#[test]
fn without_a_deficit_or_on_a_velocity_pin_the_restart_is_the_cap_s() {
    // A backstop cut on the line: the velocity cut by its scale, the acceleration as it was where
    // it brakes or the velocity is zero, whatever the pressure.
    let step = planned([1e-3, 0.0, 1e-3], [0.7, 0.0, -0.2], [2.0, 1.0, 3.0]);
    let x = wall([1.0, 0.0, 0.0]);
    for (walls, pressure) in [(Walls::default(), Pressure::None), (x, HELD)] {
        let (v, a, fraction) = restarted_on(&walls, &step, [8e-4, 0.0, 8e-4], 0.8, pressure);
        assert!(
            (v[0] - 0.56).abs() < 1e-12 && (v[2] + 0.16).abs() < 1e-12,
            "{v:?}"
        );
        assert_eq!(a, [0.0, 1.0, 3.0]);
        assert!((fraction - 0.8).abs() < 1e-12);
    }
    // A velocity pin that let half of the step out: half the velocity.
    let (v, a, fraction) = restarted(&step, [5e-4, 0.0, 5e-4], 1.0, Pressure::Velocity);
    assert!(
        (v[0] - 0.35).abs() < 1e-12 && (fraction - 0.5).abs() < 1e-12,
        "{v:?}"
    );
    assert_eq!(a, [0.0, 1.0, 3.0]);
    // Everything went out: exactly 1.
    let (_, _, fraction) = restarted(&step, [1e-3, 0.0, 1e-3], 1.0, Pressure::Velocity);
    assert_eq!(fraction, 1.0);
}

#[test]
fn a_wall_holds_the_generator_on_every_held_cycle_and_leaves_a_reversal_alone() {
    let y = wall([0.0, 1.0, 0.0]);
    let held = HELD;
    // Its deficit elsewhere: the push into the wall goes, a braking acceleration stays.
    let step = planned([0.0, 1e-4, 0.0], [0.0, 0.2, 0.0], [0.0, -3.0, 0.0]);
    let (v, a, _) = restarted_on(&y, &step, [1e-4, 0.0, 0.0], 1.0, held);
    assert_eq!((v, a), ([0.0; 3], [0.0, -3.0, 0.0]));
    let step = planned([0.0, 1e-4, 0.0], [0.0, 0.2, 0.0], [0.0, 3.0, 0.0]);
    let (v, a, _) = restarted_on(&y, &step, [0.0; 3], 1.0, held);
    assert_eq!((v, a), ([0.0; 3], [0.0; 3]));
    // A joint still braking toward the limit let 90 % of the step along the wall out: 90 % of the
    // velocity into it stays. The step and the velocity are a generator's: their difference is
    // under a cycle of the acceleration, which the hold leaves the generator either way.
    let step = planned([0.0, 1e-4, 0.0], [0.0, 0.1, 0.0], [0.0, 3.0, 0.0]);
    let (v, _, _) = restarted_on(&y, &step, [0.0, 9e-5, 0.0], 1.0, held);
    assert!((v[1] - 0.09).abs() < 1e-12, "{v:?}");
    // Once that joint rests on its margin, none.
    let resting = Pressure::Position { rest: [1.0, 0.0] };
    let (v, _, _) = restarted_on(&y, &step, [0.0, 9e-5, 0.0], 1.0, resting);
    assert_eq!(v, [0.0; 3]);
    // Away from it, or along it with nothing withheld: left as it is.
    let back = planned([0.0, -1e-4, 1e-4], [0.0, -0.2, 0.1], [0.0, -3.0, 1.0]);
    let (v, a, fraction) = restarted_on(&y, &back, [0.0, -1e-4, 1e-4], 1.0, held);
    assert_eq!((v, a, fraction), ([0.0, -0.2, 0.1], [0.0, -3.0, 1.0], 1.0));
}

#[test]
fn a_partial_hold_takes_the_generator_down_to_what_went_out() {
    // Planned along a wall, nothing into it, and the goal made half of that: the generator
    // restarts at the speed the goal's step carried, not at its own, but for the cycle of its
    // acceleration that separates a step from the velocity it ends at.
    let y = wall([0.0, 1.0, 0.0]);
    let step = planned([1e-4, 0.0, 0.0], [0.1, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let (v, _, fraction) = restarted_on(&y, &step, [5e-5, 0.0, 0.0], 1.0, HELD);
    assert!((v[0] - 0.051).abs() < 1e-12, "{v:?}");
    assert!((fraction - 0.5).abs() < 1e-12);
    // The goal's step across the plan: nothing of the plan went out, and a cycle of the
    // acceleration is all the generator keeps, across it as along it.
    let (v, _, fraction) = restarted_on(&y, &step, [0.0, 0.0, 2e-5], 1.0, HELD);
    assert!((v[0] - 1e-3).abs() < 1e-12 && v[2] == 0.0, "{v:?}");
    assert_eq!(fraction, 0.0);
}

/// The share of what it is behind the desired pose that the goal closes in a cycle: the damped
/// solve's own tracking, near a wrist singularity or with a joint braking into its margin.
const CLOSED_PER_CYCLE: f64 = 0.5;

/// A generator held for twenty cycles against a wall across its motion (a pin elsewhere withholds
/// nothing of it) while the goal closes [`CLOSED_PER_CYCLE`] of what it is behind: its speed over
/// the hold as a share of the speed it had going in, and the largest change of that speed per
/// cycle over the hold and the eighty cycles after it. `snapped` re-anchors the generator on the
/// goal's pose every held cycle and leaves its velocity alone, as the restart did before it took
/// the hold's shortfall into account.
fn held_against_a_wall(snapped: bool) -> (f64, f64) {
    let budget = crate::otg::OtgLimits {
        max_velocity: 1.0,
        max_acceleration: 8.0,
        max_jerk: 400.0,
    };
    let mut otg = crate::otg::MultiOtg::<6>::with_limits([0.0; 6], [budget; 6], true).unwrap();
    let place = |p: [f64; 3]| (p, Matrix3::identity());
    let (mut goal, mut desired) = ([0.0; 3], [0.0; 3]);
    let mut speeds = Vec::new();
    for cycle in 0..300 {
        let anchor = desired;
        let _ = otg.set_position([anchor[0], anchor[1], anchor[2], 0.0, 0.0, 0.0]);
        let _ = otg.set_target([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let position = otg.step(DELTA_T);
        let axes = otg.axes();
        let step = Step::<6, 7> {
            position,
            velocity: std::array::from_fn(|i| axes[i].velocity()),
            acceleration: std::array::from_fn(|i| axes[i].acceleration()),
            target: [0.0; 7],
            hold: false,
            finished: false,
        };
        let from = goal;
        (0..3).for_each(|i| goal[i] += CLOSED_PER_CYCLE * (position[i] - goal[i]));
        speeds.push((goal[0] - from[0]) / DELTA_T);
        desired = position[..3].try_into().expect("three values");
        // A wall across the motion, latched over cycles 100 to 119 as a pin's twenty quiet ones.
        if !(100..120).contains(&cycle) {
            continue;
        }
        let walls = wall([0.0, 1.0, 0.0]);
        let anchors = (place(anchor), place(from));
        if snapped {
            desired = goal;
            continue;
        }
        let (v, a, _) = restart(
            &step,
            (&anchors.0, &anchors.1),
            &place(goal),
            1.0,
            HELD,
            &walls,
        );
        otg.set_state(otg.position(), v, a).expect("finite");
        desired = ahead(&step, (&anchors.0, &anchors.1), &place(goal), Some(&walls)).0;
    }
    let jump = (100..200)
        .map(|c| (speeds[c + 1] - speeds[c]).abs())
        .fold(0.0, f64::max);
    let behind = (100..120).map(|c| speeds[c]).fold(f64::INFINITY, f64::min) / speeds[99];
    (behind, jump)
}

// Measured: snapped, the goal ran at 0.52 of the generator's speed over the hold and its speed
// jumped 0.35 m/s in a cycle once the wall went; restarted on what went out, 1.01 and 0.010.
#[test]
fn a_hold_the_goal_only_partly_follows_leaves_the_generator_no_windup() {
    let (behind, jump) = held_against_a_wall(true);
    println!("snapped: the goal held {behind:.2} of its speed, largest jump {jump:.3} m/s");
    assert!(
        behind < 0.6 && jump > 0.05,
        "no windup to fix: {behind}, {jump}"
    );
    let (behind, jump) = held_against_a_wall(false);
    println!("restarted on what went out: {behind:.2} of its speed, largest jump {jump:.3} m/s");
    assert!(behind > 0.9, "the goal ran at {behind} of its speed");
    assert!(jump < 0.02, "the goal's speed jumped {jump} m/s");
}

#[test]
fn the_restart_keeps_the_lag_along_a_wall_drops_the_lead_into_it_and_never_outruns_the_goal() {
    let place = |p: [f64; 3]| (p, Matrix3::identity());
    let (anchor, from) = (place([1e-3, 1e-3, 0.0]), place([0.0; 3]));
    // The goal made the generator's whole step: with no wall the generator stays as far ahead of
    // it as it was, lag and all.
    let step = planned([1.2e-3, 1.2e-3, 0.0], [0.2, 0.2, 0.0], [0.0; 3]);
    let out = place([2e-4, 2e-4, 0.0]);
    let free = ahead(&step, (&anchor, &from), &out, None).0;
    assert!(
        (0..2).all(|i| (free[i] - out.0[i] - 1e-3).abs() < 1e-15),
        "{free:?}"
    );
    // Held at a wall along x: the lead into it goes, the lag along it stays.
    let walls = wall([1.0, 0.0, 0.0]);
    let held = ahead(&step, (&anchor, &from), &out, Some(&walls)).0;
    assert!((held[0] - out.0[0]).abs() < 1e-15, "{held:?}");
    assert!((held[1] - out.0[1] - 1e-3).abs() < 1e-15, "{held:?}");
    // A goal closing a lag of its own moves further than the generator planned: the restart
    // advances by no more than that plan, so the desired is not dragged along with the goal.
    let closing = place([1e-2, 0.0, 0.0]);
    let raced = ahead(&step, (&anchor, &from), &closing, Some(&Walls::default())).0;
    let planned = (0..3)
        .map(|i| (step.position[i] - anchor.0[i]).powi(2))
        .sum::<f64>()
        .sqrt();
    let advanced = raced[0] - anchor.0[0];
    assert!(
        (advanced - planned).abs() < 1e-15,
        "{advanced} of {planned}"
    );
}

#[test]
fn a_wall_is_held_while_its_pins_push_and_goes_twenty_cycles_after_the_last_push() {
    let (mut walls, mut stall) = (Walls::default(), Stall::default());
    // A pin that does not push holds nothing.
    assert!(!walls.update(&stall));
    let x = joint_4([1.0, 0.0, 0.0]);
    stall.withheld[0] = Some(x);
    for _ in 0..100 {
        assert!(walls.update(&stall));
    }
    // Pushing in rotation too: that block's wall is its own, the translation's stays.
    let turn = joint_4([0.0, 0.0, 1.0]);
    stall.withheld = [None, Some(turn)];
    assert!(walls.update(&stall));
    assert_eq!(walls.blocks, [Some((x, 1)), Some((turn, 0))]);
    // A push dropped for a cycle is bridged; the translation's wall goes 20 cycles after its last
    // push while the rotation's is still pushed on, then that one too.
    stall.withheld[1] = None;
    assert!(walls.update(&stall));
    stall.withheld[1] = Some(turn);
    for k in 3..=21 {
        assert!(walls.update(&stall), "cycle {k}");
        assert_eq!(walls.blocks[0].is_some(), k <= 20, "cycle {k}");
    }
    stall.withheld[1] = None;
    for k in 1..=21 {
        assert_eq!(walls.update(&stall), k <= 20, "cycle {k}");
    }
    assert_eq!(walls, Walls::default());
    // The stall flag alone holds, against no wall.
    stall.record(3e-5, 0.0);
    assert!(walls.update(&stall) && walls == Walls::default());
}

#[test]
fn a_goal_comes_to_rest_on_the_margin_of_the_pins_of_its_wall_over_a_ramp() {
    let fer = limits(FciVersion::V5);
    let q = at_lower(&fer, 3, MARGIN + 5e-6);
    let rest = |joints: [i8; 7]| fer.rest(&q, &joints);
    assert!((rest([0, 0, 0, -1, 0, 0, 0]) - 0.5).abs() < 1e-9);
    // Another side, another joint, or none: not resting.
    assert_eq!(rest([0, 0, 0, 1, 0, 0, 0]), 0.0);
    assert_eq!(rest([-1, 0, 0, 0, 0, 0, 0]), 0.0);
    assert_eq!(rest([0; 7]), 0.0);
    assert_eq!(
        fer.rest(&at_lower(&fer, 3, MARGIN), &[0, 0, 0, -1, 0, 0, 0]),
        1.0
    );
    assert_eq!(
        fer.rest(&at_lower(&fer, 3, MARGIN + 2e-5), &[0, 0, 0, -1, 0, 0, 0]),
        0.0
    );
}

#[test]
fn a_block_planning_less_than_at_rest_is_left_as_it_is_with_fraction_one() {
    // A turn with a nanometre of translation planned, the goal's pose a little off it.
    let step = Step {
        position: [1e-9, 0.0, 0.0, 0.0, 0.0, 1e-3],
        velocity: [1e-6, 0.0, 0.0, 0.0, 0.0, 1.0],
        acceleration: [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0],
        target: [0.0; 7],
        hold: false,
        finished: false,
    };
    let origin = ([0.0; 3], Matrix3::identity());
    let out = (
        [0.0, 5e-7, 0.0],
        super::super::rotation::exp(&[0.0, 0.0, 1e-3]),
    );
    for (pressure, kept) in [(Pressure::Velocity, 0.0), (HELD, 1e-3)] {
        let walls = wall([0.0, 1.0, 0.0]);
        let (v, a, fraction) = restart(&step, (&origin, &origin), &out, 1.0, pressure, &walls);
        assert_eq!(fraction, [1.0; 2], "{pressure:?}");
        assert_eq!(v[..3], [1e-6, 0.0, 0.0], "{pressure:?}");
        assert_eq!(a[..3], [kept, 0.0, 0.0], "{pressure:?}");
    }
}
