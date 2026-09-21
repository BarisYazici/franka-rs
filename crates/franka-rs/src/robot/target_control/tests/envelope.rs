//! The arms' velocity limits — the FR3's published envelope, the FER's published flat limit —
//! and the braking profile each arm approaches a position limit with.

use super::super::envelope::{Envelope, FR3};
use super::super::position::{JointLimits, VelocityLimit};
use super::super::*;
use crate::rate_limiting::{fer, DELTA_T};

/// The specifications page's form toward the upper limit, as a signed velocity.
fn published_upper(q: f64, q_max: f64, dq_max: f64, offset: f64, dec: f64) -> f64 {
    dq_max.min((-offset + (2.0 * dec * (q_max - q)).max(0.0).sqrt()).max(0.0))
}

/// ... and toward the lower limit.
fn published_lower(q: f64, q_min: f64, dq_max: f64, offset: f64, dec: f64) -> f64 {
    (-dq_max).max((offset - (2.0 * dec * (q - q_min)).max(0.0).sqrt()).min(0.0))
}

/// The FER's nominal joint acceleration limits, rad/s², from the specifications page.
const FER_DDQ_MAX: [f64; 7] = [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0];

#[test]
fn the_fr3_envelope_has_the_published_form_and_parameters() {
    let (q_min, q_max, dq_max, offset, dec) = (
        [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508],
        [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508],
        [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],
        [0.6599, 0.2517, 0.2000, 0.3533, 0.5757, 0.4878, 0.4628],
        [6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0],
    );
    let envelope: &Envelope = &FR3;
    let tol = envelope.tolerance;
    assert_eq!(tol, rate_limiting::JOINT_VELOCITY_LIMITS_TOLERANCE);
    let limit = VelocityLimit::of(FciVersion::V10);
    assert!(limit.position_dependent());
    for i in 0..7 {
        for k in -50..=1050 {
            let q = q_min[i] + f64::from(k) / 1000.0 * (q_max[i] - q_min[i]);
            let mut at = [0.0; 7];
            at[i] = q;
            let (lower, upper) = limit.at(&at);
            let up = published_upper(q, q_max[i], dq_max[i], offset[i], dec[i]) - tol[i];
            let lo = published_lower(q, q_min[i], dq_max[i], offset[i], dec[i]) + tol[i];
            assert!((upper[i] - up.max(0.0)).abs() < 1e-12, "joint {i} at {q}");
            assert!((lower[i] + lo.min(0.0)).abs() < 1e-12, "joint {i} at {q}");
        }
    }
}

#[test]
fn the_fr3_envelope_falls_toward_each_limit_to_zero_at_it_and_nan_is_zero() {
    let envelope: &Envelope = &FR3;
    for i in 0..7 {
        let (q_min, q_max) = (envelope.q_min[i], envelope.q_max[i]);
        let (mut last_up, mut last_down) = (f64::INFINITY, f64::INFINITY);
        for k in 0..=4000 {
            let s = f64::from(k) / 4000.0 * (q_max - q_min);
            let (mut up, mut down) = ([0.0; 7], [0.0; 7]);
            (up[i], down[i]) = (q_min + s, q_max - s);
            let toward_upper = envelope.at(&up).1[i];
            let toward_lower = envelope.at(&down).0[i];
            assert!(toward_upper <= last_up && toward_upper >= 0.0, "joint {i}");
            assert!(
                toward_lower <= last_down && toward_lower >= 0.0,
                "joint {i}"
            );
            (last_up, last_down) = (toward_upper, toward_lower);
        }
        let (mut at, mut past) = ([0.0; 7], [0.0; 7]);
        (at[i], past[i]) = (q_max, q_max + 0.1);
        assert_eq!((envelope.at(&at).1[i], envelope.at(&past).1[i]), (0.0, 0.0));
        (at[i], past[i]) = (q_min, q_min - 0.1);
        assert_eq!((envelope.at(&at).0[i], envelope.at(&past).0[i]), (0.0, 0.0));
    }
    assert_eq!(envelope.at(&[f64::NAN; 7]), ([0.0; 7], [0.0; 7]));
}

#[test]
fn the_fer_velocity_limit_is_the_published_flat_one_everywhere() {
    // Franka publishes no position-based velocity rows for the FER, and an envelope built on
    // half its acceleration limit is not what the robot checks, so the robot's own check is
    // libfranka 0.9.2's flat limit: 2.175 rad/s on
    // joints 1 to 4 and 2.61 on the wrist, less the tolerance, wherever the joint is.
    let limit = VelocityLimit::of(FciVersion::V5);
    assert!(!limit.position_dependent());
    assert_eq!(limit.flat(), fer::MAX_JOINT_VELOCITY);
    assert!((fer::MAX_JOINT_VELOCITY[0] - (2.175 - 1e-3 - 3e-3 * (15.0 - 1e-3))).abs() < 1e-15);
    let (lower, upper) = joint_position_limits(FciVersion::V5);
    for i in 0..7 {
        for k in -50..=1050 {
            let mut q = [0.0; 7];
            q[i] = lower[i] + f64::from(k) / 1000.0 * (upper[i] - lower[i]);
            assert_eq!(
                limit.at(&q),
                (fer::MAX_JOINT_VELOCITY, fer::MAX_JOINT_VELOCITY)
            );
        }
    }
    assert_eq!(
        limit.at(&[f64::NAN; 7]),
        (fer::MAX_JOINT_VELOCITY, fer::MAX_JOINT_VELOCITY)
    );
}

#[test]
fn each_arm_brakes_at_half_its_acceleration_limit_or_the_published_deceleration() {
    // E(x) = min(flat, sqrt(k x + c²) − c) with k = 2a: the FR3's a is the specifications'
    // ddq_dec, the FER's half its acceleration limit, and E stays under sqrt(2 a x), the profile
    // of a joint braking at a, and under the flat limit.
    for (name, version, a) in [
        ("FER", FciVersion::V5, FER_DDQ_MAX.map(|a| 0.5 * a)),
        ("FR3", FciVersion::V10, FR3.ddq_dec),
    ] {
        let limit = VelocityLimit::of(version);
        let flat = limit.flat();
        for (i, &flat) in flat.iter().enumerate() {
            assert_eq!(limit.braking(i, 0.0, flat), 0.0);
            assert_eq!(limit.braking(i, -0.1, flat), 0.0);
            assert_eq!(limit.braking(i, f64::NAN, flat), 0.0);
            let mut last = 0.0;
            for k in 1..=3000 {
                let x = f64::from(k) * 1e-3;
                let e = limit.braking(i, x, flat);
                let stops = (2.0 * a[i] * x).sqrt();
                assert!(e > 0.0 && e >= last && e <= flat, "{name} joint {i} at {x}");
                assert!(e <= stops, "{name} joint {i} at {x}: {e} over {stops}");
                last = e;
            }
        }
        // 10 mrad before the stop, rad/s.
        let ten = flat
            .iter()
            .enumerate()
            .map(|(i, &flat)| limit.braking(i, 0.01, flat))
            .collect::<Vec<_>>();
        let expected = match version {
            FciVersion::V5 => [0.1899, 0.1391, 0.1742, 0.1637, 0.1720, 0.2179, 0.2179],
            FciVersion::V10 => [0.1583, 0.1028, 0.1317, 0.1123, 0.3301, 0.1322, 0.3301],
        };
        for i in 0..7 {
            assert!((ten[i] - expected[i]).abs() < 5e-5, "{name}: {ten:.4?}");
        }
    }
    // The FER's wrists brake at 7.5 and 10 rad/s², inside their 15 and 20 rating, where the FR3's
    // 17 was above joint 5's rating; joints 1 to 4 and 6 brake harder than the FR3 constants did.
    let (fer, fr3) = (
        VelocityLimit::of(FciVersion::V5),
        VelocityLimit::of(FciVersion::V10),
    );
    for i in [4, 6] {
        assert!(fer.braking(i, 0.01, 10.0) < 0.7 * fr3.braking(i, 0.01, 10.0));
    }
    for i in [0, 1, 2, 3, 5] {
        assert!(fer.braking(i, 0.01, 10.0) > 1.1 * fr3.braking(i, 0.01, 10.0));
    }
}

#[test]
fn the_braking_profile_governs_the_fer_near_a_limit_and_the_lower_of_the_two_on_the_fr3() {
    let margin = 0.05;
    for (name, version) in [("FER", FciVersion::V5), ("FR3", FciVersion::V10)] {
        let limits = JointLimits {
            position: joint_position_limits(version),
            margin,
            fraction: 0.7,
            velocity: VelocityLimit::of(version),
        };
        let (lower, upper) = limits.position;
        let flat = limits.velocity.flat();
        for i in 0..7 {
            for k in 0..=500 {
                let s = margin + f64::from(k) * 1e-3;
                let mut from: [f64; 7] = std::array::from_fn(|j| 0.5 * (lower[j] + upper[j]));
                from[i] = upper[i] - s;
                let b = limits.cycle_box(&from, DELTA_T);
                let braking = 0.7 * limits.velocity.braking(i, s - margin, flat[i]) * DELTA_T;
                // Within a rad of a limit the braking profile is what holds the goal back: the
                // step is under it, and under the flat limit it would otherwise be allowed.
                assert!(b.upper[i] <= braking + 1e-15, "{name} joint {i} at {s}");
                if k > 0 {
                    assert!(b.upper[i] > 0.0, "{name} joint {i} at {s}");
                }
                if s < 0.3 {
                    assert!(
                        b.upper[i] < 0.7 * flat[i] * DELTA_T && b.position_upper[i],
                        "{name} joint {i} at {s}: {:.3e}",
                        b.upper[i]
                    );
                }
            }
        }
    }
}
