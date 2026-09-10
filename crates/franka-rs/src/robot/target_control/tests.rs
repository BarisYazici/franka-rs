//! Option validation and the generator runner, without a robot.

use std::sync::mpsc;
use std::sync::Arc;

use super::cartesian::{axis_limits, chart, slot_values};
use super::joint::DEFAULT_LIMIT_FRACTION;
use super::rotation::{
    angle_between, checked_pose, exp, from_quaternion, log, norm, orthonormality_error, pose_from,
    to_quaternion, unit_quaternion,
};
use super::runner::identity;
use super::*;
use crate::otg::OtgLimits;
use crate::rate_limiting::{self, DELTA_T};
use crate::wire::robot::codec::FciVersion;
use crate::ControllerMode;
use nalgebra::Matrix3;

fn is_invalid_argument(result: FrankaResult<()>, needle: &str) -> bool {
    matches!(result, Err(FrankaError::InvalidArgument(m)) if m.contains(needle))
}

#[test]
fn default_options_are_valid_and_documented() {
    let cartesian = TargetControlOptions::default();
    assert!(cartesian.validate().is_ok());
    assert_eq!(cartesian.limits.max_velocity, 0.3);
    assert_eq!(cartesian.limits.max_acceleration, 0.5);
    assert_eq!(cartesian.limits.max_jerk, 20.0);
    assert_eq!(cartesian.rotation_limits.max_velocity, 0.5);
    assert_eq!(cartesian.rotation_limits.max_acceleration, 1.0);
    assert_eq!(cartesian.rotation_limits.max_jerk, 20.0);
    assert_eq!(cartesian.max_deviation, 0.30);
    assert_eq!(cartesian.max_angular_deviation, 0.5);
    assert_eq!(cartesian.settle, Settle::default());
    assert!(cartesian.limit_rate);
    assert_eq!(cartesian.realtime_priority, None);
    assert!(cartesian.observer.is_none());
    assert!(format!("{cartesian:?}").contains("observer: false"));

    let joint = JointTargetControlOptions::default();
    assert!(joint.validate().is_ok());
    assert_eq!(joint.limits, None);
    assert_eq!(joint.max_deviation, 1.0);
    assert!(joint.limit_rate);
}

#[test]
fn cartesian_options_reject_bad_fields() {
    let bad_limits = TargetControlOptions::default().with_limits(OtgLimits {
        max_velocity: 0.0,
        max_acceleration: 0.5,
        max_jerk: 20.0,
    });
    assert!(is_invalid_argument(bad_limits.validate(), "otg: limits"));
    let bad_rotation = TargetControlOptions::default().with_rotation_limits(OtgLimits {
        max_velocity: 0.5,
        max_acceleration: f64::NAN,
        max_jerk: 20.0,
    });
    assert!(is_invalid_argument(
        bad_rotation.validate(),
        "rotation_limits"
    ));
    let deviation = TargetControlOptions::default().with_max_deviation(f64::NAN);
    assert!(is_invalid_argument(deviation.validate(), "max_deviation"));
    let angular = TargetControlOptions::default().with_max_angular_deviation(0.0);
    assert!(is_invalid_argument(
        angular.validate(),
        "max_angular_deviation"
    ));
    let settle = TargetControlOptions::default().with_settle(Settle {
        tolerance: 1e-3,
        cycles: 0,
    });
    assert!(is_invalid_argument(settle.validate(), "settle"));
    let settle = TargetControlOptions::default().with_settle(Settle {
        tolerance: -1.0,
        cycles: 10,
    });
    assert!(is_invalid_argument(settle.validate(), "settle"));
    for priority in [0, 100, -5] {
        let options = TargetControlOptions::default().with_realtime_priority(Some(priority));
        assert!(is_invalid_argument(options.validate(), "realtime_priority"));
    }
    let fine = TargetControlOptions::default()
        .with_realtime_priority(Some(80))
        .with_controller_mode(ControllerMode::JointImpedance)
        .with_limit_rate(false)
        .with_observer(|_, _| {});
    assert!(fine.validate().is_ok());
    assert!(fine.observer.is_some());
}

#[test]
fn joint_options_reject_bad_fields() {
    let mut limits = JointTargetControlOptions::scaled_limits(FciVersion::V10, 0.2);
    limits[6].max_jerk = f64::INFINITY;
    let bad = JointTargetControlOptions::default().with_limits(limits);
    assert!(is_invalid_argument(bad.validate(), "otg: limits"));
    let deviation = JointTargetControlOptions::default().with_max_deviation(0.0);
    assert!(is_invalid_argument(deviation.validate(), "max_deviation"));
    let priority = JointTargetControlOptions::default().with_realtime_priority(Some(200));
    assert!(is_invalid_argument(
        priority.validate(),
        "realtime_priority"
    ));
}

#[test]
fn scaled_joint_limits_follow_the_version() {
    let fr3 = JointTargetControlOptions::scaled_limits(FciVersion::V10, DEFAULT_LIMIT_FRACTION);
    assert!((fr3[0].max_velocity - 0.2 * 2.62).abs() < 1e-12);
    assert!((fr3[4].max_velocity - 0.2 * 5.26).abs() < 1e-12);
    assert!(
        (fr3[0].max_acceleration - 0.2 * rate_limiting::MAX_JOINT_ACCELERATION[0]).abs() < 1e-12
    );
    assert!((fr3[0].max_jerk - 0.2 * rate_limiting::MAX_JOINT_JERK[0]).abs() < 1e-12);

    let fer = JointTargetControlOptions::scaled_limits(FciVersion::V5, 0.5);
    for (i, limits) in fer.iter().enumerate() {
        let (v, a, j) = (
            rate_limiting::fer::MAX_JOINT_VELOCITY[i],
            rate_limiting::fer::MAX_JOINT_ACCELERATION[i],
            rate_limiting::fer::MAX_JOINT_JERK[i],
        );
        assert!((limits.max_velocity - 0.5 * v).abs() < 1e-12);
        assert!((limits.max_acceleration - 0.5 * a).abs() < 1e-12);
        assert!((limits.max_jerk - 0.5 * j).abs() < 1e-12);
    }
    for version in [FciVersion::V5, FciVersion::V10] {
        let options = JointTargetControlOptions::default()
            .with_limits(JointTargetControlOptions::scaled_limits(version, 0.2));
        assert!(options.validate().is_ok());
    }
}

const LIMITS: OtgLimits = OtgLimits {
    max_velocity: 0.3,
    max_acceleration: 0.5,
    max_jerk: 20.0,
};

/// A runner with a fresh slot and a `started` receiver; the "robot" echoes each command.
fn runner(settle: Settle) -> (Runner<3, 3>, Arc<Shared<3>>, mpsc::Receiver<()>) {
    let shared = Arc::new(Shared::<3>::default());
    let (started, first_cycle) = mpsc::sync_channel(1);
    let runner = Runner::new(Arc::clone(&shared), started, [LIMITS; 3], settle, identity).unwrap();
    (runner, shared, first_cycle)
}

#[test]
fn the_runner_anchors_on_the_first_echo_and_signals_the_start() {
    let (mut runner, shared, first_cycle) = runner(Settle::default());
    let mut state = RobotState::default();
    state.q[0] = 0.5;
    let step = runner.cycle(&state, [0.4, 0.1, 0.3], false);
    assert_eq!(step.position, [0.4, 0.1, 0.3]);
    assert_eq!(step.target, [0.4, 0.1, 0.3]);
    assert_eq!(step.velocity, [0.0; 3]);
    assert!(!step.finished);
    assert!(
        first_cycle.try_recv().is_ok(),
        "the start must be signalled"
    );
    let mut target = [0.0; 3];
    assert!(shared.slot.load(&mut target));
    assert_eq!(target, [0.4, 0.1, 0.3], "the slot starts at the anchor");
    assert_eq!(
        shared.state.lock().unwrap().q[0],
        0.5,
        "the state is published"
    );
}

#[test]
fn the_runner_follows_the_slot_settles_after_a_stop_and_finishes() {
    let settle = Settle {
        tolerance: 1e-3,
        cycles: 20,
    };
    let (mut runner, shared, _first) = runner(settle);
    let state = RobotState::default();
    let mut echo = [0.0; 3];
    runner.cycle(&state, echo, false);
    shared.slot.publish([0.05, 0.0, 0.0]);
    let mut cycles = 0;
    let mut peak_speed = 0.0f64;
    let mut finished = false;
    while !finished {
        let step = runner.cycle(&state, echo, false);
        peak_speed = peak_speed.max(step.velocity[0].abs());
        echo = step.position;
        cycles += 1;
        if cycles == 300 {
            shared.stop.store(true, Ordering::SeqCst);
        }
        finished = step.finished;
        assert!(cycles < 2000, "the runner never finished");
    }
    // The hold repeats the echo of the command before the generator counted as landed, in
    // the last 4 ms of the profile at most: under a micrometre from the target.
    assert!(
        (echo[0] - 0.05).abs() < 1e-6 && echo[1..] == [0.0, 0.0],
        "{echo:?}"
    );
    assert!(peak_speed <= LIMITS.max_velocity / 3f64.sqrt() + 1e-9);
    // The 5 cm S-curve takes 0.658 s, then the hold of `settle.cycles` and the finish.
    assert!(
        (640..=720).contains(&cycles),
        "finished after {cycles} cycles"
    );
    assert!(runner.finish(Ok(())).is_ok());
}

#[test]
fn the_runner_freezes_the_target_on_a_deviation_and_reports_it() {
    let (mut runner, shared, _first) = runner(Settle {
        tolerance: 1e-3,
        cycles: 5,
    });
    let state = RobotState::default();
    let mut echo = [0.0; 3];
    runner.cycle(&state, echo, false);
    shared.slot.publish([0.10, 0.0, 0.0]);
    for _ in 0..200 {
        echo = runner.cycle(&state, echo, false).position;
    }
    let moving = runner.cycle(&state, echo, true);
    assert!(
        moving.target[0] < 0.10 && moving.target[0] > 0.0,
        "frozen where the command is"
    );
    let frozen = moving.target;
    shared.slot.publish([0.0, 0.0, 0.0]); // ignored from now on
    let mut cycles = 0;
    loop {
        let step = runner.cycle(&state, echo, true);
        echo = step.position;
        cycles += 1;
        assert_eq!(step.target, frozen);
        if step.finished {
            break;
        }
        assert!(cycles < 2000);
    }
    assert!((echo[0] - frozen[0]).abs() < 1e-3);
    match runner.finish(Ok(())) {
        Err(FrankaError::Control(e)) => assert_eq!(e.message, DEVIATION_MESSAGE),
        other => panic!("expected the deviation error, got {other:?}"),
    }
    match runner.finish(Err(FrankaError::Network("gone".into()))) {
        Err(FrankaError::Network(m)) => assert_eq!(m, "gone"),
        other => panic!("a loop error wins over the deviation, got {other:?}"),
    }
}

/// Runs a stopping runner against `echo_of` (the "robot") until it finishes, returning every
/// sent position; asserts the last `settle.cycles + 1` are bit-identical and sent at rest.
fn run_to_finish<const N: usize>(
    runner: &mut Runner<N, N>,
    shared: &Shared<N>,
    target: [f64; N],
    settle: Settle,
    mut echo_of: impl FnMut([f64; N]) -> [f64; N],
) -> Vec<[f64; N]> {
    let state = RobotState::default();
    // Anchor first (which publishes the anchor into the slot), then set the target.
    let mut echo = runner.cycle(&state, [0.0; N], false).position;
    shared.slot.publish(target);
    shared.stop.store(true, Ordering::SeqCst);
    let mut sent = vec![echo];
    loop {
        let step = runner.cycle(&state, echo_of(echo), false);
        echo = step.position;
        sent.push(step.position);
        assert!(sent.len() < 8000, "the runner never finished");
        if step.hold {
            assert_eq!(step.velocity, [0.0; N]);
            assert_eq!(step.acceleration, [0.0; N]);
        }
        if step.finished {
            break;
        }
    }
    let window = settle.cycles as usize + 1;
    let held = &sent[sent.len() - window..];
    let bits = |p: &[f64; N]| p.map(f64::to_bits);
    assert!(
        held.iter().all(|p| bits(p) == bits(&held[0])),
        "the last {window} commands are not bit-identical"
    );
    sent
}

/// The commands shortly before the held window were still moving: the hold began at the
/// window (the held echo equals the one command before it, never the ten before).
fn assert_hold_began_at_window<const N: usize>(sent: &[[f64; N]], settle: Settle) {
    let window = settle.cycles as usize + 1;
    let bits = |p: &[f64; N]| p.map(f64::to_bits);
    assert_ne!(
        bits(&sent[sent.len() - window - 10]),
        bits(&sent[sent.len() - 1]),
        "the hold started earlier than the settle window"
    );
}

#[test]
fn a_stop_that_never_lands_times_out_and_still_holds_before_finishing() {
    let settle = Settle::default();
    let (mut runner, shared, _first) = runner(settle);
    // The "robot" echoes a position a metre away every cycle, so the generator never lands.
    let sent = run_to_finish(&mut runner, &shared, [0.0; 3], settle, |_| [1.0; 3]);
    assert_eq!(sent.len() as u32, 1 + STOP_TIMEOUT_CYCLES + settle.cycles);
}

#[test]
fn the_cartesian_stop_holds_an_identical_pose_for_the_settle_window_then_finishes() {
    let settle = Settle::default();
    let (mut runner, shared, _first) = runner(settle);
    let target = [0.05, -0.02, 0.01];
    let sent = run_to_finish(&mut runner, &shared, target, settle, |p| p);
    assert_hold_began_at_window(&sent, settle);
    let last = sent[sent.len() - 1];
    // The echo of a command in the last 4 ms of the profile: under a micrometre from the target.
    for (a, b) in last.iter().zip(&target) {
        assert!((a - b).abs() < 1e-6, "held away from the target: {last:?}");
    }
    // Landing plus the hold: well before the timeout.
    assert!(
        (sent.len() as u32) < 1200 + settle.cycles,
        "{} cycles",
        sent.len()
    );
}

#[test]
fn the_stop_lands_through_a_float32_echo_with_reference_noise() {
    // What an FR3 does to the command: float32 on the wire, an ulp of dither on top.
    let settle = Settle::default();
    let (mut runner, shared, _first) = runner(settle);
    let mut k = 0u32;
    let sent = run_to_finish(&mut runner, &shared, [0.4, 0.0, 0.5], settle, |p| {
        k += 1;
        let noise = if k % 3 == 0 { 3e-8 } else { -2e-8 };
        p.map(|x| f64::from(x as f32) + noise)
    });
    let last = sent[sent.len() - 1];
    assert!(
        (last[0] - 0.4).abs() < 1e-6 && (last[2] - 0.5).abs() < 1e-6,
        "{last:?}"
    );
    assert_hold_began_at_window(&sent, settle);
    // 0.64 m at 0.173 m/s per axis is about 2.3 s; the timeout would be 5 s more.
    assert!(
        (sent.len() as u32) < STOP_TIMEOUT_CYCLES,
        "took the timeout: {} cycles",
        sent.len()
    );
}

#[test]
fn the_joint_stop_holds_an_identical_configuration_then_finishes() {
    let settle = Settle {
        tolerance: 1e-3,
        cycles: 100,
    };
    let shared = Arc::new(Shared::<7>::default());
    let (started, _first) = mpsc::sync_channel(1);
    let limits = JointTargetControlOptions::scaled_limits(FciVersion::V10, 0.2);
    let mut runner = Runner::new(Arc::clone(&shared), started, limits, settle, identity).unwrap();
    let target = [0.3, -0.1, 0.0, -0.2, 0.0, 0.4, 0.05];
    let sent = run_to_finish(&mut runner, &shared, target, settle, |q| {
        q.map(|x| f64::from(x as f32))
    });
    assert_hold_began_at_window(&sent, settle);
    let last = sent[sent.len() - 1];
    for (a, b) in last.iter().zip(&target) {
        assert!((a - b).abs() < 1e-6, "{last:?}");
    }
    assert!((sent.len() as u32) < 3000, "{} cycles", sent.len());
}

#[test]
fn the_handle_rejects_targets_once_the_loop_has_ended() {
    let shared = Arc::new(Shared::<3>::default());
    shared.slot.publish([1.0, 2.0, 3.0]);
    let handle = Handle {
        shared: Arc::clone(&shared),
        thread: None,
    };
    assert!(!handle.is_running());
    assert_eq!(handle.target(), [1.0, 2.0, 3.0]);
    match handle.set_target([0.0; 3]) {
        Err(FrankaError::InvalidOperation(m)) => assert_eq!(m, ENDED_MESSAGE),
        other => panic!("expected InvalidOperation, got {other:?}"),
    }
    shared.running.store(true, Ordering::SeqCst);
    assert!(is_invalid_argument(
        handle.set_target([f64::NAN, 0.0, 0.0]),
        "must be finite"
    ));
    handle.set_target([4.0, 5.0, 6.0]).unwrap();
    assert_eq!(handle.target(), [4.0, 5.0, 6.0]);
    assert!(handle.stop().is_ok(), "no thread to join is a regular end");
}

#[test]
fn rotation_vectors_quaternions_and_matrices_round_trip() {
    let mut rng = crate::otg::tests::Rng(0x2545_F491_4F6C_DD1D);
    for _ in 0..500 {
        let v = [
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
        ];
        let angle = rng.uniform(1e-9, 3.1);
        let v = v.map(|x| x * angle / norm(&v));
        let r = exp(&v);
        assert!(orthonormality_error(&r) < 1e-12);
        let back = log(&r);
        assert!(
            norm(&[back[0] - v[0], back[1] - v[1], back[2] - v[2]]) < 1e-9,
            "{v:?} -> {back:?}"
        );
        let q = to_quaternion(&r);
        assert!(q[3] >= 0.0 && (norm(&[q[0], q[1], q[2]]).hypot(q[3]) - 1.0).abs() < 1e-12);
        assert!(angle_between(&r, &from_quaternion(&q)) < 1e-12);
        assert!((angle_between(&Matrix3::identity(), &r) - angle).abs() < 1e-9);
    }
    assert_eq!(log(&Matrix3::identity()), [0.0; 3]);
    assert_eq!(exp(&[0.0; 3]), Matrix3::identity());
    assert_eq!(to_quaternion(&Matrix3::identity()), [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn poses_and_quaternions_are_repaired_within_tolerance_and_refused_beyond() {
    let r = exp(&[0.3, -0.2, 0.5]);
    let pose = pose_from(&r, &[0.4, 0.0, 0.5]);
    let (position, checked) = checked_pose(&pose).unwrap();
    assert_eq!(position, [0.4, 0.0, 0.5]);
    assert!(angle_between(&r, &checked) < 1e-12);

    let scaled = |factor: f64| pose_from(&(r * factor), &[0.4, 0.0, 0.5]);
    let (_, repaired) = checked_pose(&scaled(1.0003)).unwrap();
    assert!(orthonormality_error(&repaired) < 1e-12 && angle_between(&r, &repaired) < 1e-9);
    assert!(is_invalid_argument(
        checked_pose(&scaled(1.01)).map(drop),
        "not orthonormal"
    ));
    let mut reflected = pose;
    reflected[0..3].iter_mut().for_each(|x| *x = -*x);
    assert!(is_invalid_argument(
        checked_pose(&reflected).map(drop),
        "not orthonormal"
    ));
    let mut bad_row = pose;
    bad_row[3] = 0.1;
    assert!(is_invalid_argument(
        checked_pose(&bad_row).map(drop),
        "last row"
    ));
    let mut nan = pose;
    nan[5] = f64::NAN;
    assert!(is_invalid_argument(checked_pose(&nan).map(drop), "finite"));

    assert_eq!(
        unit_quaternion([0.0, 0.0, 0.0, 1.0005]).unwrap(),
        [0.0, 0.0, 0.0, 1.0]
    );
    assert!(is_invalid_argument(
        unit_quaternion([0.0, 0.0, 0.0, 2.0]).map(drop),
        "[x, y, z, w]"
    ));
    assert!(is_invalid_argument(
        unit_quaternion([f64::INFINITY, 0.0, 0.0, 1.0]).map(drop),
        "finite"
    ));
}

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
