//! The generator runner against a mock robot that echoes each command.

use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::sync::Arc;

use super::super::runner::identity;
use super::super::*;
use super::is_invalid_argument;
use crate::otg::OtgLimits;
use crate::wire::robot::codec::FciVersion;

pub(super) const LIMITS: OtgLimits = OtgLimits {
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
