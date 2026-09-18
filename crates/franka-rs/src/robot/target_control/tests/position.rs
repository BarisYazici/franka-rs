//! The cycle box under the braking envelope and the velocity envelope, offline, on both arms.

use super::super::position::*;
use super::super::*;
use crate::rate_limiting::DELTA_T;

const FRACTION: f64 = 0.7;
const MARGIN: f64 = 0.05;

fn arms() -> [(&'static str, JointLimits); 2] {
    let limits = |version| JointLimits {
        position: joint_position_limits(version),
        margin: MARGIN,
        fraction: FRACTION,
        velocity: VelocityLimit::of(version),
    };
    [
        ("FER", limits(FciVersion::V5)),
        ("FR3", limits(FciVersion::V10)),
    ]
}

#[test]
fn the_box_contains_zero_and_is_under_the_velocity_limit_everywhere() {
    for (name, limits) in arms() {
        let (lower, upper) = limits.position;
        let flat = limits.velocity.flat();
        for k in 0..=400 {
            // From 0.1 rad outside the lower limit to 0.1 outside the upper, every joint at once.
            let from: [f64; 7] = std::array::from_fn(|i| {
                lower[i] - 0.1 + f64::from(k) / 400.0 * (upper[i] - lower[i] + 0.2)
            });
            let b = limits.cycle_box(&from, DELTA_T);
            let (toward_lower, toward_upper) = limits.velocity.at(&from);
            for i in 0..7 {
                assert!(
                    b.lower[i] <= 0.0 && b.upper[i] >= 0.0,
                    "{name} joint {i}: {b:?}"
                );
                assert!(-b.lower[i] <= FRACTION * toward_lower[i] * DELTA_T);
                assert!(b.upper[i] <= FRACTION * toward_upper[i] * DELTA_T);
                assert!(b.velocity_upper[i] <= FRACTION * toward_upper[i] * DELTA_T);
                assert!(b.velocity_lower[i] >= -FRACTION * toward_lower[i] * DELTA_T);
                // Mid-range the velocity term is the flat limit, bit for bit.
                if (from[i] - 0.5 * (lower[i] + upper[i])).abs() < 0.05 {
                    assert_eq!(b.velocity_upper[i], FRACTION * flat[i] * DELTA_T, "{name}");
                    assert_eq!(b.velocity_lower[i], -FRACTION * flat[i] * DELTA_T, "{name}");
                }
                // Inside the margin nothing moves toward that limit.
                if from[i] - lower[i] <= MARGIN {
                    assert_eq!(b.lower[i], 0.0, "{name} joint {i}");
                }
                if upper[i] - from[i] <= MARGIN {
                    assert_eq!(b.upper[i], 0.0, "{name} joint {i}");
                }
            }
        }
    }
}

#[test]
fn a_goal_driven_along_its_box_reaches_the_margin_without_crossing_it() {
    for (name, limits) in arms() {
        let (lower, upper) = limits.position;
        let flat = limits.velocity.flat();
        for i in 0..7 {
            for toward_upper in [false, true] {
                let mut q: [f64; 7] = std::array::from_fn(|j| 0.5 * (lower[j] + upper[j]));
                let mut reached = None;
                for cycle in 0..20_000 {
                    let b = limits.cycle_box(&q, DELTA_T);
                    let s = if toward_upper {
                        upper[i] - q[i]
                    } else {
                        q[i] - lower[i]
                    };
                    let step = if toward_upper {
                        b.upper[i]
                    } else {
                        -b.lower[i]
                    };
                    let bound =
                        FRACTION * limits.velocity.braking(i, s - MARGIN, flat[i]) * DELTA_T;
                    assert!(step <= bound + 1e-15, "{name} joint {i}: {step} > {bound}");
                    q[i] += if toward_upper { step } else { -step };
                    let s = if toward_upper {
                        upper[i] - q[i]
                    } else {
                        q[i] - lower[i]
                    };
                    assert!(s >= MARGIN - 1e-12, "{name} joint {i} crossed: {s}");
                    if reached.is_none() && s <= MARGIN + 1e-3 {
                        reached = Some(cycle);
                    }
                }
                assert!(
                    reached.is_some(),
                    "{name} joint {i} upper {toward_upper}: {q:?}"
                );
            }
        }
    }
}

#[test]
fn the_box_shrinks_to_the_envelope_toward_every_limit() {
    for (name, arm) in arms() {
        let (lower, upper) = arm.position;
        for i in 0..7 {
            for toward_upper in [false, true] {
                let mut from: [f64; 7] = std::array::from_fn(|j| 0.5 * (lower[j] + upper[j]));
                for k in 0..=1000 {
                    let s = MARGIN + f64::from(k) * 1e-3;
                    from[i] = if toward_upper {
                        upper[i] - s
                    } else {
                        lower[i] + s
                    };
                    let b = arm.cycle_box(&from, DELTA_T);
                    let (vl, vu) = arm.velocity.at(&from);
                    let (bound, v) = if toward_upper {
                        (b.upper[i], vu[i])
                    } else {
                        (-b.lower[i], vl[i])
                    };
                    assert!(
                        bound <= FRACTION * v * DELTA_T,
                        "{name} joint {i} at {s}: {bound} > {v}"
                    );
                    // Within 0.3 rad of the limit the box is under the flat limit, whether
                    // the arm's own limit narrows (the FR3) or only the braking profile does.
                    let flat = FRACTION * arm.velocity.flat()[i] * DELTA_T;
                    assert!(bound < flat || s > 0.3, "{name} joint {i} at {s}");
                    if k == 0 {
                        assert!(bound < 1e-15, "{name} joint {i} at the margin: {bound}");
                    }
                }
            }
        }
    }
}

#[test]
fn every_box_step_meets_the_velocity_limit_and_the_envelope_where_it_ends() {
    for (name, arm) in arms() {
        let (lower, upper) = arm.position;
        let flat = arm.velocity.flat();
        for i in 0..7 {
            for toward_upper in [false, true] {
                let mut from: [f64; 7] = std::array::from_fn(|j| 0.5 * (lower[j] + upper[j]));
                let sign = if toward_upper { 1.0 } else { -1.0 };
                for k in 0..=1600 {
                    // From the margin to where both limits are flat, through where they narrow.
                    let s = MARGIN + f64::from(k) * 1e-3;
                    from[i] = if toward_upper {
                        upper[i] - s
                    } else {
                        lower[i] + s
                    };
                    let s = if toward_upper {
                        upper[i] - from[i]
                    } else {
                        from[i] - lower[i]
                    };
                    let b = arm.cycle_box(&from, DELTA_T);
                    let (step, velocity) = if toward_upper {
                        (b.upper[i], b.velocity_upper[i])
                    } else {
                        (-b.lower[i], -b.velocity_lower[i])
                    };
                    let limit = |u: f64| {
                        let mut end = from;
                        end[i] += sign * u;
                        let (vl, vu) = arm.velocity.at(&end);
                        FRACTION * if toward_upper { vu[i] } else { vl[i] } * DELTA_T
                    };
                    let braking =
                        FRACTION * arm.velocity.braking(i, s - MARGIN - step, flat[i]) * DELTA_T;
                    let end = limit(step).min(braking);
                    assert!(step <= end, "{name} joint {i} at {s}: {step} > {end}");
                    assert!(
                        velocity <= limit(velocity),
                        "{name} joint {i} at {s}: backstop"
                    );
                    // Conservative, not slower than the limit at the end by more than 0.25 %.
                    assert!(
                        end - step <= 2.5e-3 * step + 1e-15,
                        "{name} joint {i} at {s}"
                    );
                }
            }
        }
    }
}

#[test]
fn on_the_fer_the_velocity_bound_is_flat_and_the_braking_profile_holds_the_goal_back() {
    let (_, fer) = arms()[0];
    let (_, upper) = fer.position;
    let flat = fer.velocity.flat();
    let mut from: [f64; 7] = std::array::from_fn(|i| 0.5 * (fer.position.0[i] + upper[i]));
    // The robot's own limit does not narrow: the velocity bound is the flat one at every
    // distance, bit for bit, on both sides.
    for s in [0.2, 0.5, 1.0] {
        from[4] = upper[4] - s;
        let b = fer.cycle_box(&from, DELTA_T);
        assert_eq!(b.velocity_upper[4], FRACTION * flat[4] * DELTA_T);
        assert_eq!(-b.velocity_lower[4], FRACTION * flat[4] * DELTA_T);
    }
    // Joint 5 0.2 rad before its upper limit: the braking profile leaves it 1.19 rad/s of the
    // 2.56 flat limit, so it is the braking bound that binds, and it is a position pin.
    from[4] = upper[4] - 0.2;
    let b = fer.cycle_box(&from, DELTA_T);
    let braking = fer.velocity.braking(4, 0.2 - MARGIN, flat[4]);
    assert!((braking - 1.19).abs() < 0.01, "{braking}");
    assert!(
        b.position_upper[4] && b.upper[4] < b.velocity_upper[4],
        "{b:?}"
    );
    assert!(b.upper[4] <= FRACTION * braking * DELTA_T);
    assert!(b.upper[4] > 0.99 * FRACTION * braking * DELTA_T);
}

#[test]
fn a_bound_is_a_position_pin_near_the_limit_and_a_velocity_pin_away_from_it() {
    for (name, limits) in arms() {
        let (lower, upper) = limits.position;
        let mid: [f64; 7] = std::array::from_fn(|j| 0.5 * (lower[j] + upper[j]));
        let b = limits.cycle_box(&mid, DELTA_T);
        // Mid-range the velocity limit binds on every joint but the short ranges' envelope.
        assert!(b.position_upper.iter().any(|p| !p), "{name}: {b:?}");
        let mut near = mid;
        near[3] = lower[3] + MARGIN + 0.01;
        let b = limits.cycle_box(&near, DELTA_T);
        assert!(b.position_lower[3] && !b.position_upper[3], "{name}: {b:?}");
        assert!(-b.lower[3] < b.velocity_upper[3], "{name}: {b:?}");
    }
}
