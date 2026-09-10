//! The pose path of the runner: six axes on seven slot values, against an exact robot.

use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::sync::Arc;

use super::super::cartesian::{axis_limits, chart, slot_values};
use super::super::rotation::{angle_between, exp, norm};
use super::super::*;
use super::runner::LIMITS;
use crate::otg::OtgLimits;
use crate::rate_limiting::DELTA_T;
use nalgebra::Matrix3;

const ROTATION_LIMITS: OtgLimits = OtgLimits {
    max_velocity: 0.5,
    max_acceleration: 1.0,
    max_jerk: 20.0,
};

/// A pose runner (six axes, seven slot values) with a fresh slot.
fn pose_runner(settle: Settle) -> (Runner<6, 7>, Arc<Shared<7>>) {
    let shared = Arc::new(Shared::<7>::default());
    let (started, _first) = mpsc::sync_channel(1);
    let limits = axis_limits(LIMITS, ROTATION_LIMITS);
    let runner = Runner::new(Arc::clone(&shared), started, limits, settle, chart).unwrap();
    (runner, shared)
}

/// One cycle of a pose runner against an exact robot: the echo is the composed pose itself.
fn pose_cycle(
    runner: &mut Runner<6, 7>,
    position: &mut [f64; 3],
    rotation: &mut Matrix3<f64>,
) -> runner::Step<6, 7> {
    let step = runner.cycle(
        &RobotState::default(),
        slot_values(position, rotation),
        false,
    );
    if !step.hold {
        *position = [step.position[0], step.position[1], step.position[2]];
        *rotation = exp(&[step.position[3], step.position[4], step.position[5]]) * *rotation;
    }
    step
}

#[test]
fn the_rotational_generator_keeps_its_limits_and_converges_on_random_targets() {
    let mut rng = crate::otg::tests::Rng(0x9E37_79B9_7F4A_7C15);
    let per_axis = ROTATION_LIMITS.per_axis_for_norm(3);
    let (v_max, a_max, j_max) = (
        ROTATION_LIMITS.max_velocity,
        ROTATION_LIMITS.max_acceleration,
        ROTATION_LIMITS.max_jerk,
    );
    for sequence in 0..6 {
        let (mut runner, shared) = pose_runner(Settle::default());
        let (mut position, mut rotation) = ([0.3, 0.0, 0.5], exp(&[0.1, 0.2, -0.3]));
        pose_cycle(&mut runner, &mut position, &mut rotation);
        let mut target = rotation;
        let mut last_acceleration = [0.0; 3];
        let mut t = 0.0;
        let mut next_change = rng.uniform(0.0, 0.3);
        let mut changes = 0;
        // Steps, one burst, gaps; then a hold long enough for any of them to land.
        while changes < 7 || t < next_change + 6.0 {
            if t >= next_change && changes < 7 {
                let axis = [
                    rng.uniform(-1.0, 1.0),
                    rng.uniform(-1.0, 1.0),
                    rng.uniform(-1.0, 1.0),
                ];
                let angle = rng.uniform(0.05, 1.0) / norm(&axis);
                target = exp(&axis.map(|x| x * angle)) * rotation;
                shared.slot.publish(slot_values(&position, &target));
                changes += 1;
                next_change = t + if changes == 3 {
                    5.0 * DELTA_T
                } else {
                    rng.uniform(0.01, 3.0)
                };
            }
            let before = rotation;
            let step = pose_cycle(&mut runner, &mut position, &mut rotation);
            t += DELTA_T;
            let what = format!("sequence {sequence} at {t:.3} s");
            let turned = angle_between(&before, &rotation) / DELTA_T;
            assert!(turned <= v_max + 1e-9, "{what}: turned at {turned} rad/s");
            let velocity = [step.velocity[3], step.velocity[4], step.velocity[5]];
            let acceleration = [
                step.acceleration[3],
                step.acceleration[4],
                step.acceleration[5],
            ];
            assert!(norm(&velocity) <= v_max + 1e-9, "{what}: {velocity:?}");
            assert!(
                norm(&acceleration) <= a_max + 1e-9,
                "{what}: {acceleration:?}"
            );
            for k in 0..3 {
                assert!(velocity[k].abs() <= per_axis.max_velocity + 1e-9, "{what}");
                let jerk = (acceleration[k] - last_acceleration[k]) / DELTA_T;
                assert!(
                    jerk.abs() <= per_axis.max_jerk + 1e-6,
                    "{what}: jerk {jerk}"
                );
            }
            let jerk: [f64; 3] =
                std::array::from_fn(|k| (acceleration[k] - last_acceleration[k]) / DELTA_T);
            assert!(norm(&jerk) <= j_max + 1e-6, "{what}: {jerk:?}");
            last_acceleration = acceleration;
            assert_eq!(position, [0.3, 0.0, 0.5], "{what}: the position moved");
        }
        let error = angle_between(&rotation, &target);
        assert!(
            error < 1e-9,
            "sequence {sequence}: {error} rad from the target"
        );
        assert_eq!(last_acceleration, [0.0; 3]);
    }
}

#[test]
fn the_cartesian_stop_waits_for_the_orientation_to_land_and_holds_the_echo() {
    let settle = Settle::default();
    let (mut runner, shared) = pose_runner(settle);
    let (mut position, mut rotation) = ([0.0; 3], Matrix3::identity());
    pose_cycle(&mut runner, &mut position, &mut rotation);
    // A translation and a rotation together: synchronised, they land in the same cycle.
    let target = exp(&[0.0, 0.0, 0.3]);
    shared.slot.publish(slot_values(&[0.05, 0.0, 0.0], &target));
    shared.stop.store(true, Ordering::SeqCst);
    let (mut cycles, mut first_hold) = (0u32, None);
    let (mut translated, mut turned) = (None, None);
    loop {
        let step = pose_cycle(&mut runner, &mut position, &mut rotation);
        cycles += 1;
        if translated.is_none() && (position[0] - 0.05).abs() < 1e-4 {
            translated = Some(cycles);
        }
        if turned.is_none() && angle_between(&rotation, &target) < 1e-4 {
            turned = Some(cycles);
        }
        if step.hold && first_hold.is_none() {
            first_hold = Some(cycles);
            assert_eq!(
                step.position,
                [position[0], position[1], position[2], 0.0, 0.0, 0.0]
            );
        }
        if step.finished {
            break;
        }
        assert!(cycles < STOP_TIMEOUT_CYCLES);
    }
    // 0.3 rad about z under 0.289 rad/s per axis takes 1.6 s; the 5 cm alone would take 0.85 s
    // and is stretched to arrive with it.
    let (first_hold, translated, turned) =
        (first_hold.unwrap(), translated.unwrap(), turned.unwrap());
    assert!(
        (1400..1700).contains(&first_hold),
        "held after {first_hold} cycles"
    );
    assert!(
        translated.abs_diff(turned) < 50,
        "landed at {translated} and {turned}"
    );
    assert!(first_hold >= turned.max(translated));
    assert_eq!(cycles, first_hold + settle.cycles);
    assert!(angle_between(&rotation, &target) < settle.tolerance);
    assert!((position[0] - 0.05).abs() < settle.tolerance);
}
