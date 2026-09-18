//! The fault replays (`#[ignore]`, `REPLAY_DIR`): the `fold-*` recordings, joint 4 folded past
//! its lower limit in two VR sessions, and the `wrist-*` ones, joint 2 past its upper limit with
//! the wrist pinned. Each first
//! reproduces the recorded goals as the loop ran (the oracle), then runs the guard commanded on
//! the recorded arm and, from 3 s before the fault, both loops on the plant: the one as it ran
//! must reproduce the fault (joint 4 at its clamp, joint 2 past its limit) or the replay fails.
//!
//! ```text
//! REPLAY_DIR=/path/to/recordings cargo test --release -p franka-rs --lib \
//!     target_control::tests::replay -- --ignored --nocapture --test-threads=1
//! ```

use super::super::super::ik::AS_RUN_LIMIT_MARGIN as CLAMP;
use super::super::super::position::VelocityLimit;
use super::super::super::*;
use super::super::plant::as_run::Restart;
use super::super::plant::nearest;
use super::chatter;
use super::csv::{cartesian, pinned_run, Arm, Drive, Driven, Mode, Session};
use super::envelope::{recorded_on_the_fer, shares};
use super::sync::reproduced;
use super::Config;
use crate::rate_limiting::fer::JOINT_POSITION_LIMITS;
use crate::rate_limiting::DELTA_T;

/// The plant runs from this many cycles before the fault.
const BEFORE: usize = 3000;

type Targets = Vec<(usize, [f64; 7])>;

/// The guard commanded on the recorded arm: every goal inside the margin, every goal step under
/// the fraction of the FER's flat limit and of the braking profile where it ends; the fraction of
/// cycles it engaged. The recorded arm itself is judged against the flat limit.
fn commanded(name: &str, session: &Session, targets: &Targets, config: Config) -> Vec<String> {
    let n = session.t.len();
    let (sent, _) = cartesian(
        session,
        targets,
        &Arm::fer(),
        config,
        Mode::Guarded(0.1),
        0..n,
        Drive::Commanded,
    );
    let goals: Vec<[f64; 7]> = sent.iter().map(|r| r.q_goal).collect();
    let (clearance, joint) = nearest(&goals, &JOINT_POSITION_LIMITS);
    let dq: Vec<[f64; 7]> = sent.iter().map(|r| r.dq_goal).collect();
    let fastest = shares(FciVersion::V5, &goals, &dq, 1..n)
        .into_iter()
        .fold(0.0, f64::max)
        / config.fraction;
    // ... and of the braking profile to the margin, where the step ends: the goal's own bound
    // toward the limit, the only one that narrows on the FER.
    let velocity = VelocityLimit::of(FciVersion::V5);
    let flat = velocity.flat();
    let margin = ImpedanceOptions::cartesian().joint_position_margin;
    let braking = sent
        .windows(2)
        .map(|w| {
            (0..7)
                .map(|i| {
                    let (from, to) = (w[0].q_goal[i], w[1].q_goal[i]);
                    let s = if to < from {
                        to - JOINT_POSITION_LIMITS.0[i]
                    } else {
                        JOINT_POSITION_LIMITS.1[i] - to
                    };
                    let bound = velocity.braking(i, s - margin, flat[i]) * DELTA_T;
                    // A goal resting on its margin moves by 1e-14 rad against a bound of the
                    // same size: nothing under a nrad a cycle (1e-6 rad/s) is judged.
                    match (to - from).abs() {
                        step if step < 1e-9 => 0.0,
                        step if bound > 0.0 => step / bound,
                        _ => f64::INFINITY,
                    }
                })
                .fold(0.0, f64::max)
        })
        .fold(0.0, f64::max)
        / config.fraction;
    let engaged = sent
        .iter()
        .filter(|r| r.pinned != [0; 7] || r.cap_scale < 1.0 || r.tau_position != [0.0; 7])
        .count();
    println!(
        "{name}: guarded commanded: goals {:.2} mrad from a limit (joint {}), |dq_goal| at most \
         {fastest:.4} of the cap on the FER's flat limit and {braking:.4} of the braking \
         profile's, engaged {:.4} of the cycles",
        clearance * 1e3,
        joint + 1,
        engaged as f64 / n as f64
    );
    let mut failures = Vec::new();
    if clearance < margin - 1e-12 {
        failures.push(format!(
            "{name}: commanded goal {:.3} mrad from a limit",
            clearance * 1e3
        ));
    }
    if fastest > 1.0 + 1e-9 {
        failures.push(format!("{name}: commanded goal at {fastest} of the cap"));
    }
    if braking > 1.0 + 1e-9 {
        failures.push(format!(
            "{name}: commanded goal at {braking} of the braking profile's cap"
        ));
    }
    recorded_on_the_fer(name, session, &[], &mut failures);
    failures
}

/// Both loops on the plant over `cycles` under `scale` times the recorded external torques.
fn on_plant(
    session: &Session,
    targets: &Targets,
    config: Config,
    mode: Mode,
    cycles: std::ops::Range<usize>,
    scale: f64,
) -> (Vec<CartesianSent>, Driven) {
    cartesian(
        session,
        targets,
        &Arm::fer(),
        config,
        mode,
        cycles,
        Drive::plant(scale),
    )
}

/// The hand's RMS distance to the target in force, m, over `cycles` of a plant run from `start`.
fn hand_rms(q: &[[f64; 7]], targets: &Targets, start: usize, cycles: usize) -> f64 {
    let arm = Arm::fer();
    let sum: f64 = (0..cycles)
        .map(|k| {
            let i = targets.partition_point(|t| t.0 <= start + k).max(1) - 1;
            let pose = arm.pose(&q[k]);
            (0..3)
                .map(|a| (pose[12 + a] - targets[i].1[a]).powi(2))
                .sum::<f64>()
        })
        .sum();
    (sum / cycles as f64).sqrt()
}

/// A fold replay on `name`, whose goal of joint 4 was pinned at the clamp for `pinned` cycles.
fn fold(name: &str, pinned: usize) {
    let (config, restart) = (Config::teleop(), Restart::Mean);
    let mut failures = Vec::new();
    let Some((session, (targets, goals))) =
        reproduced(name, &Arm::fer(), config, restart, &mut failures)
    else {
        return;
    };
    let lower = JOINT_POSITION_LIMITS.0[3];
    let recorded = pinned_run(&session.q_goal, 3, lower + CLAMP, 1e-9);
    let ours = pinned_run(&goals, 3, lower + CLAMP, 1e-9);
    println!("{name}: joint 4 pinned at the clamp (start, cycles): recorded {recorded:?}, as run {ours:?}");
    if ours.map(|p| p.1) != Some(pinned) {
        failures.push(format!(
            "{name}: as run pinned {ours:?}, expected {pinned} cycles"
        ));
    }
    failures.extend(commanded(name, &session, &targets, config));
    chatter::check(name, &session, &targets, (config, restart), &mut failures);

    // From 3 s before the fault, then those 3 s of targets played backwards and 0.5 s held.
    let n = session.t.len();
    let start = n.saturating_sub(BEFORE);
    let mut played = targets.clone();
    let back = targets.iter().rev().take_while(|t| t.0 >= start);
    played.extend(back.map(|t| (2 * n - t.0, t.1)));
    let cycles = start..2 * n - start + 500;
    let (_, as_run) = on_plant(
        &session,
        &played,
        config,
        Mode::AsRun(restart),
        cycles.clone(),
        0.0,
    );
    let (_, guarded) = on_plant(&session, &played, config, Mode::Guarded(0.1), cycles, 0.0);
    let (clearance, joint) = nearest(&guarded.q, &JOINT_POSITION_LIMITS);
    // Credible iff the plant as it ran brings joint 4 to its clamp or past its limit first.
    let fault = n - start;
    let as_run_joint_4 = as_run.q[..fault]
        .iter()
        .map(|q| q[3] - lower)
        .fold(f64::INFINITY, f64::min);
    let credible = match as_run.violation {
        Some((_, joint)) => joint == 3,
        None => as_run_joint_4 <= CLAMP + 1e-3,
    };
    let rms = |q: &[[f64; 7]], cycles: usize| hand_rms(q, &played, start, cycles);
    let (whole, as_run_whole) = (
        rms(&guarded.q, guarded.q.len()),
        rms(&as_run.q, as_run.q.len()),
    );
    println!(
        "{name}: plant from {start}: as run joint 4 {:.2} mrad from its limit, violation {:?}: \
         credible {credible}; guarded {:.2} mrad (joint {}), violation {:?}; hand RMS guarded \
         {:.2} mm to the fault, {:.2} mm over the run, as run {:.2}, {:.2} mm",
        as_run_joint_4 * 1e3,
        as_run.violation,
        clearance * 1e3,
        joint + 1,
        guarded.violation,
        rms(&guarded.q, fault) * 1e3,
        whole * 1e3,
        rms(&as_run.q, fault) * 1e3,
        as_run_whole * 1e3
    );
    if !credible {
        failures.push(format!(
            "{name}: the plant as run does not reproduce the joint 4 fault -- {:.2} mrad from its \
             limit, violation {:?}; open harness item, not a guard result",
            as_run_joint_4 * 1e3,
            as_run.violation
        ));
    }
    if clearance < 0.02 || guarded.violation.is_some() {
        failures.push(format!("{name}: guarded plant {:.2} mrad", clearance * 1e3));
    }
    if whole > 1.25 * as_run_whole {
        failures.push(format!(
            "{name}: hand RMS {whole} against {as_run_whole} as run"
        ));
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn joint_4_folds_in_fold_a() {
    fold("fold-a", 683);
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn joint_4_folds_in_fold_a_6() {
    fold("fold-a-6", 289);
}

/// A wrist replay on `name`, whose wrist pinned `wrist` (0-based) while joint 2 went past its
/// upper limit.
fn wrist(name: &str, wrist: &[usize]) {
    let config = Config::wrist_limit();
    let mut failures = Vec::new();
    let Some((session, (targets, _))) =
        reproduced(name, &Arm::fer(), config, Restart::End, &mut failures)
    else {
        return;
    };
    failures.extend(commanded(name, &session, &targets, config));
    chatter::check(
        name,
        &session,
        &targets,
        (config, Restart::End),
        &mut failures,
    );
    let n = session.t.len();
    let start = n.saturating_sub(BEFORE);
    let cycles = start..n + 300;
    let (lower, upper) = JOINT_POSITION_LIMITS;
    let (sent, as_run) = on_plant(
        &session,
        &targets,
        config,
        Mode::AsRun(Restart::End),
        cycles.clone(),
        1.0,
    );
    // Credible iff the loop as it ran pins the recorded set, brings joint 2 within 2 mrad of its
    // upper limit within 300 ms of the fault and first leaves the limits on joint 2.
    let near_fault = BEFORE.min(n) - 300..sent.len();
    let clamped = |j: usize| {
        near_fault.clone().any(|k| {
            let g = sent[k].q_goal[j];
            (g - (lower[j] + CLAMP)).abs() < 1e-9 || (g - (upper[j] - CLAMP)).abs() < 1e-9
        })
    };
    let set: Vec<usize> = std::iter::once(1).chain(wrist.iter().copied()).collect();
    let pins: Vec<bool> = set.iter().map(|&j| clamped(j)).collect();
    let closest = near_fault
        .clone()
        .map(|k| upper[1] - as_run.q[k][1])
        .fold(f64::INFINITY, f64::min);
    let on_joint_2 = as_run.violation.is_some_and(|(_, joint)| joint == 1);
    let credible = pins.iter().all(|p| *p) && closest <= 0.002 && on_joint_2;
    println!(
        "{name}: plant as run: joints {:?} clamped {pins:?}, joint 2 {:.2} mrad from its upper \
         limit, violation {:?}: credible {credible}",
        set.iter().map(|j| j + 1).collect::<Vec<_>>(),
        closest * 1e3,
        as_run.violation
    );
    if !credible {
        failures.push(format!(
            "{name}: the plant as run does not reproduce the joint 2 fault -- joints {:?} clamped \
             {pins:?}, joint 2 {:.2} mrad from its upper limit (2 wanted), violation {:?}; open \
             harness item, not a guard result",
            set.iter().map(|j| j + 1).collect::<Vec<_>>(),
            closest * 1e3,
            as_run.violation
        ));
    }
    let velocity = VelocityLimit::of(FciVersion::V5);
    let (f, flat) = (config.fraction, velocity.flat());
    let margin = ImpedanceOptions::cartesian().joint_position_margin;
    for (scale, clearance) in [(1.0, 0.02), (2.0, 0.01)] {
        let (sent, guarded) = on_plant(
            &session,
            &targets,
            config,
            Mode::Guarded(0.1),
            cycles.clone(),
            scale,
        );
        let (nearest_q, joint) = nearest(&guarded.q, &JOINT_POSITION_LIMITS);
        let steepest = sent
            .windows(2)
            .map(|w| {
                let (from, to) = (w[0].q_goal[1], w[1].q_goal[1]);
                let x = upper[1] - from - margin;
                (to - from) - f * velocity.braking(1, x, flat[1]) * DELTA_T
            })
            .fold(f64::NEG_INFINITY, f64::max);
        println!(
            "{name}: plant guarded, disturbance x{scale}: {:.2} mrad from a limit (joint {}), \
             violation {:?}, joint 2's step at most {steepest:.2e} rad over f·E·Δt",
            nearest_q * 1e3,
            joint + 1,
            guarded.violation
        );
        if credible && (nearest_q < clearance || guarded.violation.is_some() || steepest > 1e-15) {
            failures.push(format!(
                "{name}: guarded x{scale}: {:.2} mrad, step {steepest:.2e}",
                nearest_q * 1e3
            ));
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn joint_2_over_its_limit_in_wrist_a() {
    wrist("wrist-a", &[4]);
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn joint_2_over_its_limit_in_wrist_b() {
    wrist("wrist-b", &[4, 6]);
}
