//! Joint 4 parked at its margin, the arm following the goal one cycle behind: a move that does
//! not need joint 4 further in keeps the speed it has with the limits far, one that does costs no
//! more than from rest, whatever the tool; a stop toward it lands on it, or slides as far as it can.

use std::sync::atomic::Ordering;

use super::super::super::cartesian::{placement, pose_of};
use super::super::super::rotation::{angle_between, exp};
use super::super::plant::{teleop, Tool, FLANGE, HAND, LONG_TOOL};
use super::super::torque::{cycles_to_finish, Rig};
use super::super::READY;
use super::fold::{folded, folding, Fold};
use super::*;

/// `pose` moved `d` in the base frame and turned by the rotation vector `turn` in its own.
fn moved(pose: &[f64; 16], d: [f64; 3], turn: [f64; 3]) -> [f64; 16] {
    let (mut p, r) = placement(pose);
    (0..3).for_each(|i| p[i] += d[i]);
    pose_of(&(p, r * exp(&turn)))
}

/// Toward `target` for `cycles`: the goal pose's peak speed, m/s, or rad/s if `turn`, and the
/// cycles it takes to cover 99 % of the way.
fn speed(fold: &mut Fold, target: &[f64; 16], cycles: usize, turn: bool) -> (f64, usize) {
    let start = fold.records.lock().unwrap().len();
    fold.to(target, cycles);
    let records = fold.records.lock().unwrap();
    let to = placement(target);
    let away = |q: &[f64; 7]| {
        let (p, r) = placement(&fold.pose_of(q));
        if turn {
            angle_between(&r, &to.1)
        } else {
            (0..3).map(|i| (p[i] - to.0[i]).powi(2)).sum::<f64>().sqrt()
        }
    };
    let away: Vec<f64> = records[start - 1..]
        .iter()
        .map(|r| away(&r.q_goal))
        .collect();
    let peak = away
        .windows(2)
        .map(|w| (w[0] - w[1]) / DELTA_T)
        .fold(0.0, f64::max);
    let arrival = away
        .iter()
        .position(|d| *d < 0.01 * away[0])
        .unwrap_or(usize::MAX);
    (peak, arrival)
}

/// Whether joint 4's goal was pinned on its lower bound within the last cycles of `fold`: the
/// exact solve lets a pin go for a cycle here and there, which the wall's latch bridges.
fn parked_at_its_margin(fold: &Fold, tool: &Tool) {
    let records = fold.records.lock().unwrap();
    let last = records.len().saturating_sub(21);
    assert!(
        records[last..].iter().any(|r| r.pinned[3] == -1),
        "{}: {:?}",
        tool.name,
        records[records.len() - 1]
    );
}

/// Joint 4 held at its margin by a fold beyond reach: the fold, and where the goal rests.
fn parked(tool: &Tool) -> (Fold, [f64; 7]) {
    let mut fold = Fold::new(&folding(&Rig::fer()), READY, tool.f_t_ee);
    fold.toward(&folded(), 2000);
    parked_at_its_margin(&fold, tool);
    let q = fold.last().q_goal;
    (fold, q)
}

/// Each axis both ways: 5 cm along the base's axes, 0.3 rad about the tool's.
const MOVES: [(&str, [f64; 3], [f64; 3]); 12] = [
    ("+x", [0.05, 0.0, 0.0], [0.0; 3]),
    ("-x", [-0.05, 0.0, 0.0], [0.0; 3]),
    ("+y", [0.0, 0.05, 0.0], [0.0; 3]),
    ("-y", [0.0, -0.05, 0.0], [0.0; 3]),
    ("+z", [0.0, 0.0, 0.05], [0.0; 3]),
    ("-z", [0.0, 0.0, -0.05], [0.0; 3]),
    ("+roll", [0.0; 3], [0.3, 0.0, 0.0]),
    ("-roll", [0.0; 3], [-0.3, 0.0, 0.0]),
    ("+pitch", [0.0; 3], [0.0, 0.3, 0.0]),
    ("-pitch", [0.0; 3], [0.0, -0.3, 0.0]),
    ("+yaw", [0.0; 3], [0.0, 0.0, 0.3]),
    ("-yaw", [0.0; 3], [0.0, 0.0, -0.3]),
];

/// `a` at least as fast as `b`, but for 10 % and 0.01 m/s or rad/s, and arriving when `b` does.
fn as_fast(a: (f64, usize), b: (f64, usize)) -> bool {
    a.0 >= 0.9 * b.0 - 0.01 && (b.1 == usize::MAX || 0.9 * a.1 as f64 <= b.1 as f64)
}

// Measured: ±y, +x, +z, ±yaw and -pitch keep the speed on every tool; -x, -z, +roll, +pitch need
// joint 4 further in (with the hand also -roll); after the fold none is slower than from rest.
#[test]
fn a_joint_parked_at_its_margin_costs_a_move_that_does_not_need_it_no_speed() {
    let mut slow = Vec::new();
    for tool in [FLANGE, HAND, LONG_TOOL] {
        let (_, q) = parked(&tool);
        for (name, d, turn) in MOVES {
            let mut free = Fold::new(&Rig::fer(), q, tool.f_t_ee);
            let target = moved(&free.pose_of(&q), d, turn);
            let turning = turn != [0.0; 3];
            let far = speed(&mut free, &target, 800, turning);
            let (mut held, _) = parked(&tool);
            let after_fold = speed(&mut held, &target, 800, turning);
            let mut rest = Fold::new(&folding(&Rig::fer()), q, tool.f_t_ee);
            let from_rest = speed(&mut rest, &target, 800, turning);
            let case = format!("{} {name}", tool.name);
            println!("{case}: (peak, arrival) {far:.3?} far, {after_fold:.3?} after the fold, {from_rest:.3?} from rest");
            let free_move = ["+x", "+y", "-y", "+z", "-pitch", "+yaw", "-yaw"].contains(&name);
            if !as_fast(after_fold, from_rest) || free_move && !as_fast(after_fold, far) {
                slow.push(case);
            }
        }
    }
    assert!(slow.is_empty(), "{slow:?}");
}

#[test]
fn a_stop_toward_a_reachable_target_lands_on_it_with_a_joint_parked_at_its_margin() {
    for tool in [FLANGE, HAND, LONG_TOOL] {
        let (mut fold, q) = parked(&tool);
        let target = moved(&fold.pose_of(&q), [0.0, 0.02, 0.0], [0.0; 3]);
        fold.to(&target, 0);
        fold.shared.stop.store(true, Ordering::SeqCst);
        let cycles = cycles_to_finish(|| fold.cycle());
        let (reached, goal) = (placement(&target), placement(&fold.last().pose));
        let off = (0..3)
            .map(|i| (reached.0[i] - goal.0[i]).abs())
            .fold(0.0, f64::max);
        println!(
            "{}: finished after {cycles} cycles, {off:.2e} m off",
            tool.name
        );
        assert!(
            off <= Settle::default().tolerance,
            "{}: {off} m off",
            tool.name
        );
    }
}

// Measured, per tool and target: slid 1.73 to 2.02 cm of the 2 along the wall (the diagonal one
// lands 1.7 mm short of where the goal's creep into the wall ends) and finished after 794 to 954
// cycles; the goal's velocity stepped at most 0.028 rad/s where the hold takes it to zero.
#[test]
fn a_stop_right_after_a_target_slanted_into_the_wall_slides_along_it_first() {
    // The default jerk, under which the generator's first cycles from rest look at rest.
    let smooth = |x| crate::otg::OtgLimits {
        max_velocity: 1.0,
        max_acceleration: 8.0,
        max_jerk: x,
    };
    let options = || {
        teleop()
            .with_limits(smooth(20.0))
            .with_rotation_limits(smooth(20.0))
    };
    // 2 cm into the wall (-x needs joint 4 further in) and 2 cm along it, diagonally or along one
    // axis: the stop lands where the goal can no longer slide, not where it was when stopped.
    for slide in [[-0.02, 0.02, 0.0], [-0.02, 0.014, 0.014]] {
        for tool in [FLANGE, HAND, LONG_TOOL] {
            let (_, q) = parked(&tool);
            let mut fold = Fold::with(&folding(&Rig::fer()), q, tool.f_t_ee, options());
            fold.toward(&folded(), 2000);
            parked_at_its_margin(&fold, &tool);
            let from = placement(&fold.pose_of(&q)).0;
            let target = moved(&fold.pose_of(&q), slide, [0.0; 3]);
            let start = fold.records.lock().unwrap().len();
            fold.to(&target, 0);
            fold.shared.stop.store(true, Ordering::SeqCst);
            let cycles = cycles_to_finish(|| fold.cycle());
            let records = fold.records.lock().unwrap();
            let jump = records[start..]
                .windows(2)
                .map(|w| {
                    (0..7)
                        .map(|i| (w[1].dq_goal[i] - w[0].dq_goal[i]).abs())
                        .fold(0.0, f64::max)
                })
                .fold(0.0, f64::max);
            let end = placement(&records[records.len() - 1].pose).0;
            let along = (slide[1] * slide[1] + slide[2] * slide[2]).sqrt();
            let slid = (1..3)
                .map(|i| (end[i] - from[i]) * slide[i] / along)
                .sum::<f64>();
            drop(records);
            let case = format!("{} toward {slide:?}", tool.name);
            println!(
                "{case}: slid {slid:.4} m, {cycles} cycles, largest goal jump {jump:.4} rad/s"
            );
            assert!(slid > 0.017, "{case}: slid {slid} m");
            assert!(cycles < STOP_TIMEOUT_CYCLES, "{case}: timed out");
            assert!(
                jump < 0.05,
                "{case}: the goal's velocity stepped {jump} rad/s"
            );
        }
    }
}

// Measured: the goal's step on the cycle the tool changes within 1e-3 of the one before.
#[test]
fn a_tool_set_mid_motion_keeps_the_goal_moving() {
    // Moving 5 cm along y on the flange, the arm a cycle behind, the tool becomes the hand and the
    // target the hand's pose of it: the goal's step carries on, it does not fall back to the arm.
    let mut fold = Fold::new(&Rig::fer(), READY, FLANGE.f_t_ee);
    let target = moved(&fold.pose_of(&READY), [0.0, 0.05, 0.0], [0.0; 3]);
    fold.to(&target, 100);
    let m = |t: &[f64; 16]| nalgebra::Matrix4::from_column_slice(t);
    let hand = m(&target) * m(&HAND.f_t_ee);
    fold.to(hand.as_slice().try_into().unwrap(), 0);
    let lagging = fold
        .records
        .lock()
        .unwrap()
        .iter()
        .rev()
        .nth(1)
        .unwrap()
        .q_goal;
    fold.arm.state.F_T_EE = HAND.f_t_ee;
    fold.arm.follow(&lagging);
    fold.torque.cycle(&fold.arm.state);
    fold.to(hand.as_slice().try_into().unwrap(), 4);
    let records = fold.records.lock().unwrap();
    let y = |k: usize| placement(&fold.pose_of(&records[k].q_goal)).0[1];
    let steps: Vec<f64> = (98..105).map(|k| y(k + 1) - y(k)).collect();
    println!("goal steps along y around the change: {steps:?}");
    let before = steps[0];
    assert!(before > 3e-4, "moving: {before}");
    for step in &steps[1..] {
        assert!((step - before).abs() < 0.1 * before, "{steps:?}");
    }
}

#[test]
fn a_tool_set_during_the_session_is_taken_from_the_next_cycle_s_state() {
    // At rest on the flange, the tool becomes the hand and the target the hand's pose of the same
    // configuration: nothing moves, and the deviation guard does not see the tool's offset.
    let options = super::super::plant::teleop().with_max_deviation(0.05);
    let mut fold = Fold::with(&Rig::fer(), READY, FLANGE.f_t_ee, options);
    fold.toward(&READY, 100);
    fold.arm.state.F_T_EE = HAND.f_t_ee;
    fold.toward(&READY, 1000);
    let records = fold.records.lock().unwrap();
    let moved = records
        .iter()
        .flat_map(|r| (0..7).map(move |i| (r.q_goal[i] - READY[i]).abs()))
        .fold(0.0, f64::max);
    assert!(moved < 1e-9, "the goal moved {moved} rad");
    drop(records);
    assert!(fold.torque.finish(Ok(())).is_ok());
}
