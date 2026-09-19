//! The `limit-*` recordings, joint 4's lower limit approached slowly (`#[ignore]`, `REPLAY_DIR`):
//! in `limit-joint` the goals hold 0.30 to 0.05 rad above it and 0.03 is refused; in `limit-cart`
//! the hand folds until the workspace box stops it at 0.204 rad. Each reproduces the recorded
//! goals as the loop ran; the joint goals then run guarded on the plant, the fold commanded.

use super::super::super::*;
use super::super::plant::as_run::Restart;
use super::super::plant::nearest;
use super::super::torque::joint_loop_on;
use super::csv::{cartesian, drive, load, recording_path, Arm, Drive, Mode, Session};
use super::sync::{check_oracle, joint_as_run, reproduced};
use super::Config;
use crate::rate_limiting::fer::JOINT_POSITION_LIMITS;

/// The node's joint session: 0.2 of the limits, the joint preset, feedforward on as it was
/// when the recordings were taken.
fn joint_session() -> ([crate::otg::OtgLimits; 7], ImpedanceOptions) {
    let budget = JointTargetControlOptions::scaled_limits(FciVersion::V5, DEFAULT_LIMIT_FRACTION);
    (
        budget,
        ImpedanceOptions::joint().with_velocity_feedforward(true),
    )
}

/// The runs of targets that hold joint 4 for at least `hold` cycles: first and last cycle, and
/// the target.
fn plateaus(session: &Session, hold: usize) -> Vec<(usize, usize, [f64; 7])> {
    let targets = &session.joint_targets;
    let mut runs = Vec::new();
    let mut i = 0;
    while i < targets.len() {
        let j = (i..targets.len())
            .take_while(|&j| targets[j].1[3] == targets[i].1[3])
            .last()
            .unwrap();
        let end = targets.get(j + 1).map_or(session.t.len(), |t| t.0) - 1;
        if end - targets[i].0 >= hold {
            runs.push((targets[i].0, end, targets[i].1));
        }
        i = j + 1;
    }
    runs
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn joint_4_goals_toward_its_limit_in_limit_joint() {
    let Some(path) = recording_path("limit-joint") else {
        println!("limit-joint: REPLAY_DIR is not set, skipped");
        return;
    };
    let session = load(&path);
    let mut failures = Vec::new();
    let (budget, impedance) = joint_session();
    let (picked, goals) = joint_as_run(&session, budget, impedance);
    check_oracle("limit-joint", &goals, &session.q_goal, &mut failures);

    // Guarded on the plant, the targets the guarded handle refuses dropped.
    let arm = Arm::fer();
    let rig = &arm.rig;
    let inset = joint_limit_inset(&Backend::Impedance(impedance));
    let accepted: Vec<(usize, [f64; 7])> = picked
        .iter()
        .filter(|t| check_joint_limits(&t.1, &rig.limits, inset, "target").is_ok())
        .copied()
        .collect();
    let options = JointTargetControlOptions::default().with_max_deviation(10.0);
    let (mut torque, shared) = joint_loop_on(rig, options, budget, impedance);
    let n = session.t.len();
    let driven = drive(
        &mut torque,
        &shared,
        &session,
        0..n,
        &accepted,
        &arm,
        Drive::plant(0.0),
    );
    let lower = rig.limits.0[3];
    for (from, to, target) in plateaus(&session, 4000) {
        let above = target[3] - lower;
        let refused = check_joint_limits(&target, &rig.limits, inset, "target").is_err();
        let (end, earlier) = (driven.q[to][3], driven.q[to - 4000][3]);
        println!(
            "limit-joint: goal {above:.3} rad above the limit, cycles {from}..{to}: refused \
             {refused}, joint 4 at {:.4} mrad from it, drift {:.4} mrad over 4 s",
            (end - target[3]) * 1e3,
            (end - earlier) * 1e3
        );
        if above < impedance.joint_position_margin - 1e-9 {
            if !refused {
                failures.push(format!("goal {above:.3} was not refused"));
            }
        } else if refused || (end - target[3]).abs() > 1e-3 || (end - earlier).abs() >= 5e-5 {
            failures.push(format!(
                "goal {above:.3}: refused {refused}, off {:.4} mrad, drift {:.4} mrad",
                (end - target[3]) * 1e3,
                (end - earlier) * 1e3
            ));
        }
    }
    // The goals the arm was sent inside the margin, down to 0.03, the deepest, which ended in a
    // reflex: all refused.
    let inside = picked.len() - accepted.len();
    let deepest = picked
        .iter()
        .map(|t| t.1[3] - lower)
        .fold(f64::INFINITY, f64::min);
    let mut at = picked[0].1;
    at[3] = lower + 0.03;
    let refused = check_joint_limits(&at, &rig.limits, inset, "target").is_err();
    println!(
        "limit-joint: {inside} goals refused, the deepest {deepest:.4} rad; 0.03 refused {refused}"
    );
    if !refused
        || picked.iter().any(|t| {
            (t.1[3] - lower < impedance.joint_position_margin - 1e-12) == accepted.contains(t)
        })
    {
        failures.push(format!(
            "the refusals inside the margin: {inside}, 0.03 refused {refused}"
        ));
    }
    if driven.violation.is_some() {
        failures.push(format!("violation {:?}", driven.violation));
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn a_fold_stopped_by_the_workspace_in_limit_cart() {
    let name = "limit-cart";
    let config = Config::node();
    let mut failures = Vec::new();
    let Some((session, (targets, goals))) =
        reproduced(name, &Arm::fer(), config, Restart::End, &mut failures)
    else {
        return;
    };
    let n = session.t.len();
    let limits = JOINT_POSITION_LIMITS;
    // Identical at a rotation weight of 1 until the posture fade could engage.
    let reach = ImpedanceOptions::cartesian().joint_position_margin + 0.3;
    let engaged = (0..n)
        .find(|&k| nearest(&goals[k..=k], &limits).0 < reach)
        .unwrap_or(n);
    let arm = Arm::fer();
    let (unit, _) = cartesian(
        &session,
        &targets,
        &arm,
        config,
        Mode::Guarded(1.0),
        0..n,
        Drive::Commanded,
    );
    let apart = (0..engaged)
        .map(|k| {
            (0..7)
                .map(|i| (unit[k].q_goal[i] - goals[k][i]).abs())
                .fold(0.0, f64::max)
        })
        .fold(0.0, f64::max);
    let (sent, _) = cartesian(
        &session,
        &targets,
        &arm,
        config,
        Mode::Guarded(0.1),
        0..n,
        Drive::Commanded,
    );
    let margin4 = |g: &mut dyn Iterator<Item = [f64; 7]>| {
        g.map(|q| q[3] - limits.0[3]).fold(f64::INFINITY, f64::min)
    };
    let (guarded, recorded) = (
        margin4(&mut sent.iter().map(|r| r.q_goal)),
        margin4(&mut session.q_goal.iter().copied()),
    );
    let measured = margin4(&mut session.q.iter().copied());
    println!(
        "{name}: at weight 1 within {apart:.2e} rad of as run until cycle {engaged}; joint 4's goal \
         at least {guarded:.4} rad above its limit guarded, {recorded:.4} recorded ({measured:.4} \
         measured)"
    );
    if apart > 1e-6 {
        failures.push(format!(
            "{name}: {apart:.2e} rad from as run before cycle {engaged}"
        ));
    }
    if (guarded - recorded).abs() > 2e-3 {
        failures.push(format!(
            "{name}: joint 4 {guarded:.4} guarded, {recorded:.4} recorded"
        ));
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
