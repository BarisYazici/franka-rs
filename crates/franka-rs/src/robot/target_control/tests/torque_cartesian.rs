//! The Cartesian torque loop against a mock arm that follows the goal, is held, or is moved.

use std::sync::atomic::Ordering;

use super::super::cartesian::slot_values;
use super::super::rotation::{angle_between, distance, rotation_of, translation_of};
use super::super::*;
use super::torque::{assert_held, bits, cartesian_loop, cycles_to_finish, peak_abs};
use super::{recording, Arm, Records, READY};
use crate::model::Frame;

#[test]
fn the_cartesian_loop_anchors_on_the_models_pose_of_the_measured_q_at_rest() {
    let mut arm = Arm::at(READY);
    // A robot's `O_T_EE` differs from the model's pose by the model's accuracy, franka-sim's
    // by 0.107 m along the tool z; the anchor is the model's, so the IK starts at rest.
    let fk = arm.pose();
    arm.state.O_T_EE[14] -= 0.107;
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default().with_observer(observer);
    let (mut torque, shared, first_cycle) = cartesian_loop(options, ImpedanceOptions::cartesian());
    let output = torque.cycle(&arm.state);
    assert!(
        first_cycle.try_recv().is_ok(),
        "the start must be signalled"
    );
    assert!(!output.motion_finished);
    assert_eq!(
        shared.state.lock().unwrap().q,
        READY,
        "the state is published"
    );
    let mut target = [0.0; 7];
    assert!(shared.slot.load(&mut target));
    let anchor = slot_values(&translation_of(&fk), &rotation_of(&fk));
    for (a, b) in target.iter().zip(&anchor) {
        assert!(
            (a - b).abs() < 1e-9,
            "the slot starts at the model's pose of q: {target:?}"
        );
    }
    let sent = records.lock().unwrap()[0];
    assert!(distance(&translation_of(&sent.pose), &translation_of(&fk)) < 1e-12);
    assert!(angle_between(&rotation_of(&sent.pose), &rotation_of(&fk)) < 1e-9);
    for (a, b) in sent.q_goal.iter().zip(&READY) {
        assert!(
            (a - b).abs() < 1e-9,
            "q_goal starts at q: {:?}",
            sent.q_goal
        );
    }
    assert!(sent.ik_error < 1e-9, "{}", sent.ik_error);
    assert_eq!(sent.tau, output.tau_J);
    assert!(
        sent.tau.iter().all(|t| t.abs() < 1e-9),
        "torque at rest at the goal: {:?}",
        sent.tau
    );
    assert_eq!(
        (sent.backstop_alteration, sent.backstop_angular_alteration),
        (0.0, 0.0)
    );
    assert_eq!(
        (sent.leash_alteration, sent.leash_angular_alteration),
        (0.0, 0.0)
    );
}

/// A 5 cm step in x with the stop requested at once, the arm following the goal one cycle
/// behind: every record and the cycles to the finish.
fn cartesian_step_followed(impedance: ImpedanceOptions) -> (Arm, Records<CartesianSent>, usize) {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default().with_observer(observer);
    let (mut torque, shared, _first) = cartesian_loop(options, impedance);
    torque.cycle(&arm.state);
    let (start, rotation) = (
        translation_of(&arm.state.O_T_EE),
        rotation_of(&arm.state.O_T_EE),
    );
    let target = [start[0] + 0.05, start[1], start[2]];
    shared.slot.publish(slot_values(&target, &rotation));
    shared.stop.store(true, Ordering::SeqCst);
    let mut cycles = 1;
    loop {
        let goal = records.lock().unwrap().last().unwrap().q_goal;
        arm.follow(&goal);
        let output = torque.cycle(&arm.state);
        cycles += 1;
        if output.motion_finished {
            break;
        }
        assert!(cycles < 3000, "the loop never finished");
    }
    assert!(torque.finish(Ok(())).is_ok());
    (arm, records, cycles)
}

#[test]
fn the_cartesian_loop_follows_a_step_lands_a_stop_and_holds_the_desired_pose_then_finishes() {
    let (arm, records, cycles) = cartesian_step_followed(ImpedanceOptions::cartesian());
    let records = records.lock().unwrap();
    assert_eq!(records.len(), cycles);
    // The 5 cm S-curve under the per-axis budget (0.289 m/s^2) takes 0.85 s, then the hold
    // of `settle.cycles` and the finish.
    let settle = Settle::default();
    assert!(
        (1050..=1160).contains(&cycles),
        "finished after {cycles} cycles"
    );
    assert_held(&records, settle, 0.0, |r| r.pose);
    let start = Arm::at(READY).pose();
    let target = [start[12] + 0.05, start[13], start[14]];
    let last = records[records.len() - 1];
    assert!(distance(&translation_of(&last.pose), &target) < 1e-6);
    assert!(angle_between(&rotation_of(&last.pose), &rotation_of(&start)) < 1e-9);
    // The IK followed the pose all the way and the arm followed the IK: a small residual
    // every cycle, the goal's forward kinematics at the target, the arm there too.
    let peak_ik_error = records.iter().map(|r| r.ik_error).fold(0.0, f64::max);
    assert!(peak_ik_error < 1e-4, "peak IK residual {peak_ik_error}");
    let fk = arm.model.pose_q(
        Frame::EndEffector,
        &last.q_goal,
        &arm.state.F_T_EE,
        &arm.state.EE_T_K,
    );
    assert!(distance(&translation_of(&fk), &target) < 1e-4);
    assert!(distance(&translation_of(&arm.pose()), &target) < 1e-4);
    // A following arm never strains the leash, and the torque at rest at the goal is small.
    assert!(records
        .iter()
        .all(|r| r.leash_alteration == 0.0 && r.leash_angular_alteration == 0.0));
    assert!(peak_abs(&last.tau) < 1.0, "{:?}", last.tau);
}

#[test]
fn feedforward_removes_the_damping_torque_of_a_moving_goal() {
    let peak = |impedance: ImpedanceOptions| {
        let (_, records, _) = cartesian_step_followed(impedance);
        let records = records.lock().unwrap();
        records.iter().map(|r| peak_abs(&r.tau)).fold(0.0, f64::max)
    };
    // Following one cycle behind, the position error is one cycle of motion and the velocity
    // error one cycle of acceleration: a fraction of a newton metre. Without feedforward the
    // damping fights the whole velocity, Kd v.
    let with = peak(ImpedanceOptions::cartesian());
    let without = peak(ImpedanceOptions::cartesian().with_velocity_feedforward(false));
    assert!(with < 0.5, "peak torque with feedforward {with} Nm");
    assert!(
        without > 10.0 * with,
        "feedforward on: {with} Nm, off: {without} Nm"
    );
}

#[test]
fn the_cartesian_loop_leashes_the_desired_pose_to_a_held_arm_and_the_stop_times_out() {
    let arm = Arm::at(READY);
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default().with_observer(observer);
    let impedance = ImpedanceOptions::cartesian();
    let (mut torque, shared, _first) = cartesian_loop(options, impedance);
    torque.cycle(&arm.state);
    let (start, rotation) = (
        translation_of(&arm.state.O_T_EE),
        rotation_of(&arm.state.O_T_EE),
    );
    shared.slot.publish(slot_values(
        &[start[0] + 0.05, start[1], start[2]],
        &rotation,
    ));
    shared.stop.store(true, Ordering::SeqCst);
    let cycles = 1 + cycles_to_finish(|| torque.cycle(&arm.state).motion_finished);
    let settle = Settle::default();
    // The arm never follows, so the generator never lands: the stop times out, then holds.
    assert_eq!(cycles, 1 + STOP_TIMEOUT_CYCLES + settle.cycles);
    let records = records.lock().unwrap();
    // The hold is the frozen pose leashed again each cycle: on the leash, identical to rounding.
    assert_held(&records, settle, 1e-12, |r| r.pose);
    // The desired pose stays within the leash and one step of the arm, along x.
    let leash = impedance.leash.translation;
    let ahead = |r: &CartesianSent| distance(&translation_of(&r.pose), &start);
    let furthest = records.iter().map(ahead).fold(0.0, f64::max);
    assert!(
        furthest <= leash + 2e-4 && furthest > leash,
        "the desired ran {furthest} m ahead of the arm against a leash of {leash}"
    );
    let last = records[records.len() - 1];
    assert!(ahead(&last) > leash - 1e-3, "{}", ahead(&last));
    assert!((translation_of(&last.pose)[1] - start[1]).abs() < 1e-12);
    assert!(angle_between(&rotation_of(&last.pose), &rotation) < 1e-12);
    // The leash reports what it took off each cycle -- the step the generator kept trying to
    // make -- and nothing on the orientation.
    let leashed = records.iter().filter(|r| r.leash_alteration > 0.0).count();
    assert!(leashed > records.len() / 2, "leashed in {leashed} cycles");
    let taken = records
        .iter()
        .map(|r| r.leash_alteration)
        .fold(0.0, f64::max);
    assert!(taken < 2e-4, "took {taken} m off in one cycle");
    assert!(records.iter().all(|r| r.leash_angular_alteration < 1e-12));
    // The spring pulls toward the target with a bounded force: 750 N/m x 0.025 m.
    assert!(peak_abs(&last.tau) > 1.0, "{:?}", last.tau);
    assert!(last
        .tau
        .iter()
        .zip(&impedance.torque_limits)
        .all(|(t, l)| t.abs() < *l));
    assert!(torque.finish(Ok(())).is_ok());
}

#[test]
fn the_cartesian_loop_freezes_the_target_when_the_arm_strays() {
    let arm = Arm::at(READY);
    let options = TargetControlOptions::default().with_settle(Settle {
        tolerance: 1e-3,
        cycles: 5,
    });
    let (mut torque, shared, _first) = cartesian_loop(options, ImpedanceOptions::cartesian());
    torque.cycle(&arm.state);
    let start = translation_of(&arm.state.O_T_EE);
    shared.slot.publish(slot_values(
        &[start[0] + 0.10, start[1], start[2]],
        &rotation_of(&arm.state.O_T_EE),
    ));
    for _ in 0..200 {
        torque.cycle(&arm.state);
    }
    let mut strayed = arm.state;
    strayed.O_T_EE[12] += 0.4;
    let mut cycles = 0;
    while !torque.cycle(&strayed).motion_finished {
        cycles += 1;
        assert!(cycles < 2000, "the loop never finished");
    }
    match torque.finish(Ok(())) {
        Err(FrankaError::Control(e)) => assert_eq!(e.message, DEVIATION_MESSAGE),
        other => panic!("expected the deviation error, got {other:?}"),
    }
}

#[test]
fn the_cartesian_stop_waits_for_the_arm_too() {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default().with_observer(observer);
    let (mut torque, shared, _first) = cartesian_loop(options, ImpedanceOptions::cartesian());
    torque.cycle(&arm.state);
    let (start, rotation) = (
        translation_of(&arm.state.O_T_EE),
        rotation_of(&arm.state.O_T_EE),
    );
    shared.slot.publish(slot_values(
        &[start[0] + 0.05, start[1], start[2]],
        &rotation,
    ));
    shared.stop.store(true, Ordering::SeqCst);
    let mut cycle = 1;
    let cycles = 1 + cycles_to_finish(|| {
        cycle += 1;
        let goal = records.lock().unwrap().last().unwrap().q_goal;
        arm.follow(&goal);
        if cycle <= 1400 {
            arm.state.dq[0] = 0.05;
        }
        torque.cycle(&arm.state).motion_finished
    });
    assert_eq!(cycles, 1401, "finished after {cycles} cycles");
    let records = records.lock().unwrap();
    // The runner's hold began at least the settle window before 1160 and the desired pose
    // stayed put until the arm came to rest.
    let last = records[records.len() - 1].pose;
    assert!(records[1160..].iter().all(|r| bits(&r.pose) == bits(&last)));
}

/// An arm dragged 10 cm along x while a stop holds (its velocity keeps the finish waiting):
/// the desired pose follows it, never more than the leash away.
#[test]
fn the_cartesian_hold_leashes_the_desired_pose_to_an_arm_moved_during_it() {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default().with_observer(observer);
    let impedance = ImpedanceOptions::cartesian();
    let (mut torque, shared, _first) = cartesian_loop(options, impedance);
    torque.cycle(&arm.state);
    let (start, rotation) = (
        translation_of(&arm.state.O_T_EE),
        rotation_of(&arm.state.O_T_EE),
    );
    shared.slot.publish(slot_values(
        &[start[0] + 0.05, start[1], start[2]],
        &rotation,
    ));
    shared.stop.store(true, Ordering::SeqCst);
    let mut cycle = 1;
    let dragged_at: u32 = 1300;
    let cycles = 1 + cycles_to_finish(|| {
        cycle += 1;
        if cycle < dragged_at {
            let goal = records.lock().unwrap().last().unwrap().q_goal;
            arm.follow(&goal);
        } else if cycle == dragged_at {
            arm.drag_along_x(0.10);
        }
        arm.state.dq[0] = if cycle <= dragged_at + 200 { 0.05 } else { 0.0 };
        torque.cycle(&arm.state).motion_finished
    });
    assert_eq!(cycles, dragged_at + 201);
    let records = records.lock().unwrap();
    let measured = translation_of(&arm.pose());
    let held = &records[records.len() - 1];
    let leash = impedance.leash.translation;
    let ahead = distance(&translation_of(&held.pose), &measured);
    assert!(
        ahead <= leash + 1e-9 && ahead > leash - 1e-6,
        "the desired is {ahead} m from the dragged arm against a leash of {leash}"
    );
    assert!(
        held.leash_alteration > 0.10 - leash - 1e-3,
        "{}",
        held.leash_alteration
    );
    // Every record since the drag is within the leash, and the spring pulls back toward the
    // hold with the bounded force, well under the clamp.
    assert!(records[dragged_at as usize..]
        .iter()
        .all(|r| distance(&translation_of(&r.pose), &measured) <= leash + 1e-9));
    assert!(peak_abs(&held.tau) > 1.0, "{:?}", held.tau);
    assert!(held
        .tau
        .iter()
        .zip(&impedance.torque_limits)
        .all(|(t, l)| t.abs() < 0.5 * l));
    assert!(torque.finish(Ok(())).is_ok());
}
