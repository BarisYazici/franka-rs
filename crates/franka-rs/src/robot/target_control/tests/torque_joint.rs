//! The joint torque loop against a mock arm that follows the goal, is held, or is moved.

use std::sync::atomic::Ordering;

use super::super::joint::max_abs_difference;
use super::super::*;
use super::torque::{assert_held, bits, cycles_to_finish, joint_loop};
use super::{recording, Arm, Records, READY};

#[test]
fn the_joint_loop_anchors_on_q_follows_a_step_and_holds_the_goal_then_finishes() {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default().with_observer(observer);
    let (mut torque, shared, first_cycle) = joint_loop(options, ImpedanceOptions::joint());
    let output = torque.cycle(&arm.state);
    assert!(first_cycle.try_recv().is_ok());
    let mut target = [0.0; 7];
    assert!(shared.slot.load(&mut target));
    assert_eq!(target, READY, "the slot starts at the measured q");
    {
        let first = records.lock().unwrap()[0];
        assert_eq!((first.q_goal, first.q), (READY, READY));
        assert_eq!(first.tau, output.tau_J);
        assert!(first.tau.iter().all(|t| t.abs() < 1e-9), "{:?}", first.tau);
        assert_eq!(
            (first.backstop_alteration, first.leash_alteration),
            (0.0, 0.0)
        );
    }

    let mut target = READY;
    target[0] += 0.3;
    target[3] -= 0.2;
    shared.slot.publish(target);
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
        assert!(cycles < STOP_TIMEOUT_CYCLES, "the loop never finished");
    }
    let records = records.lock().unwrap();
    assert_eq!(records.len(), cycles as usize);
    assert_held(&records, Settle::default(), 0.0, |r| r.q_goal);
    let last = records[records.len() - 1];
    for (a, b) in last.q_goal.iter().zip(&target) {
        assert!(
            (a - b).abs() < 1e-6,
            "held away from the target: {:?}",
            last.q_goal
        );
    }
    assert_eq!(last.q, last.q_goal);
    assert!(max_abs_difference(&arm.state.q, &target) < 1e-6);
    // A following arm never strains the leash; at rest at the goal the torque is zero.
    assert!(records.iter().all(|r| r.leash_alteration == 0.0));
    assert!(last.tau.iter().all(|t| t.abs() < 1e-12), "{:?}", last.tau);
    assert!(torque.finish(Ok(())).is_ok());
}

/// A 0.3 rad step on joint 1 with the stop requested at once, the arm following the goal one
/// cycle behind and `moving(cycle)` overriding joint 7's measured velocity: the records and
/// the cycles to the finish.
fn joint_step_followed(mut moving: impl FnMut(u32) -> Option<f64>) -> (Records<JointSent>, u32) {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default().with_observer(observer);
    let (mut torque, shared, _first) = joint_loop(options, ImpedanceOptions::joint());
    torque.cycle(&arm.state);
    let mut target = READY;
    target[0] += 0.3;
    shared.slot.publish(target);
    shared.stop.store(true, Ordering::SeqCst);
    let mut cycle = 1;
    let cycles = 1 + cycles_to_finish(|| {
        cycle += 1;
        let goal = records.lock().unwrap().last().unwrap().q_goal;
        arm.follow(&goal);
        if let Some(dq) = moving(cycle) {
            arm.state.dq[6] = dq;
        }
        torque.cycle(&arm.state).motion_finished
    });
    (records, cycles)
}

#[test]
fn a_stop_finishes_only_once_the_arm_is_at_rest_or_the_wait_times_out() {
    let (_, at_rest) = joint_step_followed(|_| None);
    let held = Settle::default().cycles;
    assert!((900..1200).contains(&at_rest), "{at_rest} cycles");
    // Joint 7 creeping at twice the rest velocity until 100 cycles past the runner's finish:
    // the finish waits for it, the law still on the held goal, and follows one cycle later.
    let creep = 2.0 * REST_JOINT_VELOCITY;
    let until = at_rest + 100;
    let (records, waited) = joint_step_followed(|cycle| (cycle <= until).then_some(creep));
    assert_eq!(waited, at_rest + 101);
    let records = records.lock().unwrap();
    let window = (held + 101) as usize;
    let goal = records[records.len() - 1].q_goal;
    assert!(records[records.len() - window..]
        .iter()
        .all(|r| bits(&r.q_goal) == bits(&goal)));
    let gated = records[records.len() - 50];
    let damping = ImpedanceGains::JOINT.joint_damping[6] * creep;
    assert!((gated.tau[6] + damping).abs() < 1e-9, "{:?}", gated.tau);
    // Never at rest: the wait ends at the timeout.
    let (_, timed_out) = joint_step_followed(|_| Some(creep));
    assert_eq!(timed_out, at_rest + STOP_TIMEOUT_CYCLES);
    // Exactly the rest velocity is not rest.
    let (_, edge) = joint_step_followed(|cycle| (cycle <= until).then_some(REST_JOINT_VELOCITY));
    assert_eq!(edge, at_rest + 101);
}

#[test]
fn the_joint_loop_leashes_the_goal_to_a_held_arm_and_the_stop_times_out() {
    let arm = Arm::at(READY);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default().with_observer(observer);
    let impedance = ImpedanceOptions::joint();
    let (mut torque, shared, _first) = joint_loop(options, impedance);
    torque.cycle(&arm.state);
    let mut target = READY;
    target[0] += 0.6;
    shared.slot.publish(target);
    shared.stop.store(true, Ordering::SeqCst);
    let cycles = 1 + cycles_to_finish(|| torque.cycle(&arm.state).motion_finished);
    let settle = Settle::default();
    assert_eq!(cycles, 1 + STOP_TIMEOUT_CYCLES + settle.cycles);
    let records = records.lock().unwrap();
    assert_held(&records, settle, 0.0, |r| r.q_goal);
    // The goal stays within the leash and one step of the arm on joint 1, still on the rest.
    let leash = impedance.leash.joint;
    let ahead = |r: &JointSent| r.q_goal[0] - READY[0];
    let furthest = records.iter().map(ahead).fold(0.0, f64::max);
    assert!(
        furthest <= leash + 1e-3 && furthest > leash,
        "the goal ran {furthest} rad ahead of the arm against a leash of {leash}"
    );
    let last = records[records.len() - 1];
    assert!(ahead(&last) > leash - 1e-3);
    assert!(last.q_goal[1..] == READY[1..]);
    let leashed = records.iter().filter(|r| r.leash_alteration > 0.0).count();
    assert!(leashed > records.len() / 2, "leashed in {leashed} cycles");
    let taken = records
        .iter()
        .map(|r| r.leash_alteration)
        .fold(0.0, f64::max);
    assert!(taken < 1e-3, "took {taken} rad off in one cycle");
    // 0.1 rad on a 600 Nm/rad spring, 60 Nm, under the clamp; nothing else pulls.
    let gains = ImpedanceGains::JOINT;
    assert!(
        (last.tau[0] - gains.joint_stiffness[0] * leash).abs() < 1e-6,
        "{:?}",
        last.tau
    );
    assert!(
        last.tau[1..].iter().all(|t| t.abs() < 1e-9),
        "{:?}",
        last.tau
    );
    // While the leash binds the goal stands still a leash ahead of the arm: its velocity, not
    // the generator's budget velocity (0.43 rad/s here), is fed forward, so the damping share
    // is silent; before the leash binds the goal moves and the damping feeds that forward.
    let damping_share =
        |r: &JointSent| r.tau[0] - gains.joint_stiffness[0] * (r.q_goal[0] - READY[0]);
    // (The cycle the leash first binds still carries the goal's last step.)
    let binding = records.iter().filter(|r| r.leash_alteration > 0.0).skip(1);
    let fed = binding
        .map(damping_share)
        .fold(0.0f64, |m, s| m.max(s.abs()));
    assert!(fed < 1e-6, "damping fed {fed} Nm forward while leashed");
    let moving = &records[200];
    assert!(moving.leash_alteration == 0.0 && moving.velocity[0] > 0.4);
    assert!(damping_share(moving) > 20.0, "{}", damping_share(moving));
    assert!(torque.finish(Ok(())).is_ok());
}

/// The joint form: an arm moved 0.5 rad on joint 1 while a stop holds sees a goal at most
/// the leash ahead of it.
#[test]
fn the_joint_hold_leashes_the_goal_to_an_arm_moved_during_it() {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default().with_observer(observer);
    let impedance = ImpedanceOptions::joint();
    let (mut torque, shared, _first) = joint_loop(options, impedance);
    torque.cycle(&arm.state);
    let mut target = READY;
    target[0] += 0.3;
    shared.slot.publish(target);
    shared.stop.store(true, Ordering::SeqCst);
    let mut cycle = 1;
    let moved_at: u32 = 1300;
    let cycles = 1 + cycles_to_finish(|| {
        cycle += 1;
        if cycle < moved_at {
            let goal = records.lock().unwrap().last().unwrap().q_goal;
            arm.follow(&goal);
        } else if cycle == moved_at {
            let mut q = arm.state.q;
            q[0] -= 0.5;
            arm.follow(&q);
        }
        arm.state.dq = [0.0; 7];
        arm.state.dq[6] = if cycle <= moved_at + 200 { 0.05 } else { 0.0 };
        torque.cycle(&arm.state).motion_finished
    });
    assert_eq!(cycles, moved_at + 201);
    let records = records.lock().unwrap();
    let leash = impedance.leash.joint;
    let held = records[records.len() - 1];
    assert_eq!(held.q_goal[0], arm.state.q[0] + leash);
    assert!(
        held.q_goal[1..]
            .iter()
            .zip(&target[1..])
            .all(|(a, b)| (a - b).abs() < 1e-6),
        "{:?}",
        held.q_goal
    );
    assert!((held.leash_alteration - (0.5 - leash)).abs() < 1e-6);
    assert!(records[moved_at as usize..]
        .iter()
        .all(|r| r.q_goal[0] - arm.state.q[0] <= leash + 1e-12));
    // 60 Nm toward the hold on joint 1, nothing fed forward, nothing on the other joints.
    let gains = ImpedanceGains::JOINT;
    assert!(
        (held.tau[0] - gains.joint_stiffness[0] * leash).abs() < 1e-6,
        "{:?}",
        held.tau
    );
    assert!(
        held.tau[1..6].iter().all(|t| t.abs() < 1e-9),
        "{:?}",
        held.tau
    );
    assert!(torque.finish(Ok(())).is_ok());
}
