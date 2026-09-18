//! The joint velocity cap in both torque loops against a mock arm that follows the goal: the
//! goal never outruns the cap, the generator never runs ahead of what was sent, the stop lands.

use std::sync::atomic::Ordering;

use super::super::cartesian::slot_values;
use super::super::joint::max_abs_difference;
use super::super::rotation::{angle_between, exp, log, rotation_of, translation_of};
use super::super::*;
use super::torque::{assert_held, cartesian_loop, cycles_to_finish, joint_loop};
use super::{recording, Arm, Records, READY};
use crate::model::Frame;
use crate::otg::OtgLimits;
use crate::rate_limiting::DELTA_T;
use crate::wire::robot::codec::FciVersion;

/// Every goal velocity is within `caps`, to rounding.
fn assert_under(caps: &[f64; 7], dq_goals: impl Iterator<Item = [f64; 7]>) {
    for (k, dq) in dq_goals.enumerate() {
        for i in 0..7 {
            assert!(
                dq[i].abs() <= caps[i] * (1.0 + 1e-9),
                "cycle {k}, joint {}: {} rad/s against a cap of {}",
                i + 1,
                dq[i],
                caps[i]
            );
        }
    }
}

/// The index of the last record before the hold: the hold carries neither a generator
/// velocity nor an acceleration, and a generator without either has landed or not started.
fn last_moving(records: &[JointSent]) -> usize {
    records
        .iter()
        .rposition(|r| (r.velocity, r.acceleration) != ([0.0; 7], [0.0; 7]))
        .expect("a record in motion")
}

/// A 0.6 rad move of joint 1 under `limits` and `impedance`, the arm following the goal one cycle
/// behind; the stop requested at once, or after `stop_here_after` records together with a
/// target at the arm's configuration then: every record and that configuration.
fn joint_move_followed(
    limits: [OtgLimits; 7],
    impedance: ImpedanceOptions,
    stop_here_after: Option<usize>,
) -> (Records<JointSent>, [f64; 7]) {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default()
        .with_limits(limits)
        .with_observer(observer);
    let (mut torque, shared, _first) = joint_loop(options, impedance);
    torque.cycle(&arm.state);
    let mut target = READY;
    target[0] += 0.6;
    shared.slot.publish(target);
    if stop_here_after.is_none() {
        shared.stop.store(true, Ordering::SeqCst);
    }
    let mut here = READY;
    cycles_to_finish(|| {
        let (goal, sent) = {
            let records = records.lock().unwrap();
            (records[records.len() - 1].q_goal, records.len())
        };
        arm.follow(&goal);
        if stop_here_after == Some(sent) {
            here = arm.state.q;
            shared.slot.publish(here);
            shared.stop.store(true, Ordering::SeqCst);
        }
        torque.cycle(&arm.state).motion_finished
    });
    assert!(torque.finish(Ok(())).is_ok());
    (records, here)
}

#[test]
fn a_joint_goal_faster_than_the_cap_is_held_to_it_without_windup_and_lands() {
    // The FER's full limits: joint 1's generator would reach 2.13 rad/s, the cap is the default
    // fraction of it, and 0.8.
    let limits = JointTargetControlOptions::scaled_limits(FciVersion::V5, 1.0);
    for impedance in [
        ImpedanceOptions::joint(),
        ImpedanceOptions::joint().with_joint_velocity_fraction(0.8),
    ] {
        joint_move_under_the_cap(limits, impedance);
    }
}

fn joint_move_under_the_cap(limits: [OtgLimits; 7], impedance: ImpedanceOptions) {
    let fraction = impedance.joint_velocity_fraction;
    let caps = max_joint_velocity(FciVersion::V5).map(|v| v * fraction);
    let (records, _) = joint_move_followed(limits, impedance, None);
    let records = records.lock().unwrap();
    let (cycles, target) = (records.len(), READY[0] + 0.6);
    assert_under(&caps, records.iter().map(|r| r.dq_goal));
    let capped = records.iter().filter(|r| r.cap_scale < 1.0).count();
    assert!(capped > 100, "the cap bound in {capped} cycles");
    // No windup: the generator restarts every capped cycle at its end velocity cut to the cap,
    // so it is never more than two cycles of its acceleration above the cap.
    let fastest = records.iter().map(|r| r.velocity[0]).fold(0.0, f64::max);
    let bound = caps[0] + 2.0 * limits[0].max_acceleration * DELTA_T;
    assert!(
        fastest <= bound,
        "the generator ran at {fastest} rad/s, cap {}",
        caps[0]
    );
    // Released before landing, never past the target, landed and held.
    let landing = &records[last_moving(&records)];
    assert_eq!(landing.cap_scale, 1.0);
    let furthest = records.iter().map(|r| r.q_goal[0]).fold(f64::MIN, f64::max);
    assert!(
        furthest <= target + 1e-9,
        "cap {fraction}: overshot to {furthest}"
    );
    assert_held(&records, Settle::default(), 0.0, |r| r.q_goal);
    assert!((records[records.len() - 1].q_goal[0] - target).abs() < 1e-6);
    // At the cap the move takes about 0.47 s at 0.8 and 0.51 s at 0.7, then the settle window.
    assert!(
        cycles < 1000,
        "cap {fraction}: finished after {cycles} cycles"
    );
}

/// The teleop rotation budget: 4 rad/s, 20 rad/s², 500 rad/s³.
const TELEOP_ROTATION: OtgLimits = OtgLimits {
    max_velocity: 4.0,
    max_acceleration: 20.0,
    max_jerk: 500.0,
};

/// A 0.4 rad turn about the base z under the teleop rotation budget, the arm following the goal
/// one cycle behind; the stop requested at once, or after `stop_here_after` records together
/// with a target at the arm's pose then: the arm, every record, the start pose and that pose.
fn turn_followed(
    impedance: ImpedanceOptions,
    stop_here_after: Option<usize>,
) -> (Arm, Records<CartesianSent>, [f64; 16], [f64; 16]) {
    let mut arm = Arm::at(READY);
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default()
        .with_rotation_limits(TELEOP_ROTATION)
        .with_observer(observer);
    let (mut torque, shared, _first) = cartesian_loop(options, impedance);
    torque.cycle(&arm.state);
    let start = arm.pose();
    let turned = exp(&[0.0, 0.0, 0.4]) * rotation_of(&start);
    shared
        .slot
        .publish(slot_values(&translation_of(&start), &turned));
    if stop_here_after.is_none() {
        shared.stop.store(true, Ordering::SeqCst);
    }
    let mut here = start;
    cycles_to_finish(|| {
        let (goal, sent) = {
            let records = records.lock().unwrap();
            (records[records.len() - 1].q_goal, records.len())
        };
        arm.follow(&goal);
        if stop_here_after == Some(sent) {
            here = arm.pose();
            shared
                .slot
                .publish(slot_values(&translation_of(&here), &rotation_of(&here)));
            shared.stop.store(true, Ordering::SeqCst);
        }
        torque.cycle(&arm.state).motion_finished
    });
    assert!(torque.finish(Ok(())).is_ok());
    (arm, records, start, here)
}

#[test]
fn a_turn_faster_than_the_cap_is_held_to_it_without_windup_and_no_further_than_uncapped() {
    // A fifth of the FER's limits, so that a teleop-speed turn's goal reaches the cap.
    let fraction = 0.2;
    let caps = max_joint_velocity(FciVersion::V5).map(|v| v * fraction);
    let impedance = ImpedanceOptions::cartesian().with_joint_velocity_fraction(fraction);
    let (arm, records, start, _) = turn_followed(impedance, None);
    let records = records.lock().unwrap();
    assert_under(&caps, records.iter().map(|r| r.dq_goal));
    let capped = records.iter().filter(|r| r.cap_scale < 1.0).count();
    assert!(capped > 100, "the cap bound in {capped} cycles");
    // No windup: every cycle the generator's angular velocity is within two cycles of its
    // acceleration (and the IK's linearisation) of the twist the previous goal carried.
    let norm = |v: &[f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let slack = 2.0 * 20.0 * DELTA_T + 1e-3;
    for (k, pair) in records.windows(2).enumerate() {
        let (previous, current) = (&pair[0], &pair[1]);
        let j = arm.model.zero_jacobian_q(
            Frame::EndEffector,
            &previous.q_goal,
            &arm.state.F_T_EE,
            &arm.state.EE_T_K,
        );
        let sent: [f64; 3] =
            std::array::from_fn(|r| (0..7).map(|c| j[c * 6 + 3 + r] * previous.dq_goal[c]).sum());
        let (planned, commanded) = (norm(&current.angular_velocity), norm(&sent));
        assert!(
            planned <= commanded + slack,
            "cycle {}: the generator turned at {planned} rad/s after sending {commanded}",
            k + 1
        );
    }
    // No further than the same turn uncapped -- an infinite fraction is the test's own switch,
    // which `validate` refuses -- and both land on the target and hold it.
    let free = ImpedanceOptions::cartesian()
        .with_joint_velocity_fraction(f64::INFINITY)
        .with_velocity_barrier_fraction(f64::INFINITY);
    let (_, free, _, _) = turn_followed(free, None);
    let free = free.lock().unwrap();
    assert!(free.iter().all(|r| r.cap_scale == 1.0));
    let turn = |r: &CartesianSent| angle_between(&rotation_of(&start), &rotation_of(&r.pose));
    let peak = records.iter().map(turn).fold(0.0, f64::max);
    let free_peak = free.iter().map(turn).fold(0.0, f64::max);
    assert!(
        peak <= free_peak + 1e-3,
        "capped {peak} rad, uncapped {free_peak} rad"
    );
    for run in [&records[..], &free[..]] {
        let last = run[run.len() - 1];
        assert!(
            (turn(&last) - 0.4).abs() < 1e-3,
            "landed at {}",
            turn(&last)
        );
        assert_held(run, Settle::default(), 0.0, |r| r.pose);
    }
}

/// Whether the cap bound in the ten records before `index`.
fn capped_before<S>(records: &[S], index: usize, cap_scale: impl Fn(&S) -> f64) -> bool {
    records[index - 10..index]
        .iter()
        .any(|r| cap_scale(r) < 1.0)
}

#[test]
fn a_joint_stopped_where_it_is_while_capped_brakes_from_what_was_sent() {
    // The FER's velocity limits under a softer budget: at its full jerk an acceleration the
    // generator kept would be gone in two cycles, at 500 rad/s^3 it takes 20.
    let limits = JointTargetControlOptions::scaled_limits(FciVersion::V5, 1.0).map(|l| OtgLimits {
        max_acceleration: 10.0,
        max_jerk: 500.0,
        ..l
    });
    for after in [200, 300] {
        let impedance = ImpedanceOptions::joint().with_joint_velocity_fraction(0.8);
        let (records, here) = joint_move_followed(limits, impedance, Some(after));
        let records = records.lock().unwrap();
        assert!(capped_before(&records, after, |r| r.cap_scale));
        // Braking from the cap at 10 rad/s^2 overshoots by 0.162 rad and ends 696 cycles later.
        let overshoot = records[after..]
            .iter()
            .map(|r| max_abs_difference(&r.q_goal, &here))
            .fold(0.0, f64::max);
        let cycles = records.len() - after;
        assert!(
            overshoot < 0.17,
            "stop {after}: overshot by {overshoot} rad"
        );
        assert!(cycles < 710, "stop {after}: finished after {cycles} cycles");
        assert_held(&records, Settle::default(), 0.0, |r| r.q_goal);
        assert!(max_abs_difference(&records[records.len() - 1].q_goal, &here) < 1e-6);
    }
}

#[test]
fn a_turn_stopped_where_it_is_while_capped_brakes_from_what_was_sent() {
    let impedance = ImpedanceOptions::cartesian().with_joint_velocity_fraction(0.2);
    for after in [150, 450] {
        let (arm, records, _, here) = turn_followed(impedance, Some(after));
        let records = records.lock().unwrap();
        assert!(capped_before(&records, after, |r| r.cap_scale));
        // The arm's turn past where the stop was requested: 27.2 mrad and 449 cycles to the
        // finish.
        let turned = |r: &CartesianSent| {
            let pose = arm.model.pose_q(
                Frame::EndEffector,
                &r.q_goal,
                &arm.state.F_T_EE,
                &arm.state.EE_T_K,
            );
            angle_between(&rotation_of(&here), &rotation_of(&pose))
        };
        let overshoot = records[after..].iter().map(turned).fold(0.0, f64::max);
        let cycles = records.len() - after;
        assert!(
            overshoot < 0.03,
            "stop {after}: overshot by {overshoot} rad"
        );
        assert!(cycles < 470, "stop {after}: finished after {cycles} cycles");
        assert_held(&records, Settle::default(), 0.0, |r| r.pose);
        assert!(turned(&records[records.len() - 1]) < 1e-3);
    }
}

#[test]
fn the_loop_adds_the_barrier_to_the_law_before_the_clamp_and_reports_it() {
    let limits = max_joint_velocity(FciVersion::V5);
    let rated = ImpedanceOptions::cartesian().torque_limits;
    // Joint 7 at 1 Nm: the law saturates at -1, the barrier's gain is lowered with the clamp, and
    // the clamp of the sum sends -1.
    let mut tight = rated;
    tight[6] = 1.0;
    for (fraction, engaged, torque_limits) in [
        (0.85, true, rated),
        (0.95, false, rated),
        (0.85, true, tight),
    ] {
        let mut arm = Arm::at(READY);
        arm.state.dq[6] = 0.9 * limits[6];
        let (records, observer) = recording::<CartesianSent>();
        let options = TargetControlOptions::default().with_observer(observer);
        let impedance = ImpedanceOptions::cartesian()
            .with_velocity_barrier_fraction(fraction)
            .with_torque_limits(torque_limits);
        let (mut torque, _shared, _first) = cartesian_loop(options, impedance);
        let output = torque.cycle(&arm.state);
        let sent = records.lock().unwrap()[0];
        let excess = 0.9 * limits[6] - fraction * limits[6];
        let gain = VELOCITY_BARRIER_GAIN * (torque_limits[6] / rated[6]).min(1.0);
        let expected = if engaged {
            (-gain * excess).max(-torque_limits[6])
        } else {
            0.0
        };
        assert!(
            (sent.tau_envelope[6] - expected).abs() < 1e-12,
            "{:?}",
            sent.tau_envelope
        );
        assert!(sent.tau_envelope[..6].iter().all(|t| *t == 0.0));
        let law = impedance_torques(
            &impedance,
            &arm.model.zero_jacobian(Frame::EndEffector, &arm.state),
            &sent.q_goal,
            &sent.dq_goal,
            &arm.state.q,
            &arm.state.dq,
            &arm.model.coriolis(&arm.state),
        );
        if torque_limits == tight {
            assert_eq!((law[6], sent.tau[6]), (-1.0, -1.0));
        }
        for (i, law) in law.iter().enumerate() {
            let limit = impedance.torque_limits[i];
            let expected = (law + sent.tau_envelope[i]).clamp(-limit, limit);
            assert!((sent.tau[i] - expected).abs() < 1e-12, "joint {i}");
        }
        assert_eq!(output.tau_J, sent.tau);
    }
}

#[test]
fn a_joint_past_the_fade_start_keeps_less_law_torque_along_its_motion_and_past_its_onset_none() {
    let limits = max_joint_velocity(FciVersion::V5);
    // Undamped joint springs: joint 7 lagging its goal by 0.09 rad draws 4.5 Nm along its
    // motion, more than the barrier's 2.5 Nm against it at 0.9 of its limit. The onset is 0.85
    // and the fade starts at 0.7: the default cap, and FADE_BAND under the onset for a cap of
    // 0.8 or one at the onset itself.
    let gains = ImpedanceGains {
        joint_damping: [0.0; 7],
        ..ImpedanceGains::JOINT
    };
    for cap in [0.7, 0.8, 0.85] {
        let impedance = ImpedanceOptions::joint()
            .with_gains(gains)
            .with_joint_velocity_fraction(cap);
        for (sign, speed, kept) in [
            (1.0, 0.9, 0.0),
            (-1.0, 0.9, 0.0),
            (-1.0, 0.775, 0.5),
            (1.0, 0.65, 1.0),
        ] {
            let mut arm = Arm::at(READY);
            let (records, observer) = recording::<JointSent>();
            let options = JointTargetControlOptions::default().with_observer(observer);
            let (mut torque, _shared, _first) = joint_loop(options, impedance);
            torque.cycle(&arm.state);
            arm.state.q[6] -= sign * 0.09;
            arm.state.dq[6] = sign * speed * limits[6];
            let output = torque.cycle(&arm.state);
            let sent = records.lock().unwrap()[1];
            let law = impedance_torques(
                &impedance,
                &arm.model.zero_jacobian(Frame::EndEffector, &arm.state),
                &sent.q_goal,
                &sent.dq_goal,
                &arm.state.q,
                &arm.state.dq,
                &arm.model.coriolis(&arm.state),
            );
            assert!(sign * law[6] > 4.0, "the law pushes along: {}", law[6]);
            let barrier = -sign * VELOCITY_BARRIER_GAIN * (speed - 0.85f64).max(0.0) * limits[6];
            let expected = kept * law[6] + barrier;
            assert!(
                (sent.tau[6] - expected).abs() < 1e-9,
                "cap {cap}, sign {sign}, speed {speed}: sent {} against {expected}",
                sent.tau[6]
            );
            assert_eq!(sign * sent.tau[6] < 0.0, kept == 0.0);
            // The record's share is the barrier less the faded law, so `tau` stays the clamped
            // sum of the law and it.
            for (i, law) in law.iter().enumerate() {
                let limit = impedance.torque_limits[i];
                let sum = (law + sent.tau_envelope[i]).clamp(-limit, limit);
                assert!((sent.tau[i] - sum).abs() < 1e-9, "joint {i}");
            }
            assert_eq!(output.tau_J, sent.tau);
        }
    }
}

/// A wrist configuration and a turn from it, base frame, that drives joints 5 and 7 toward
/// alignment: their speed for a given twist grows along the turn.
const WRIST: [f64; 7] = [-0.243, -0.129, -0.199, -2.094, -0.122, 2.102, 1.655];
const TOWARD_ALIGNMENT: [f64; 3] = [-0.45, -0.65, -0.44];

#[test]
fn a_turn_held_at_the_cap_brakes_at_its_jerk_when_the_target_reverses() {
    // Half the limits: joint 7 meets its cap before the reversal, as the solver routes part of the
    // turn around it and the restart cuts the twist to what went out.
    let impedance = ImpedanceOptions::cartesian().with_joint_velocity_fraction(0.5);
    let mut arm = Arm::at(WRIST);
    let (records, observer) = recording::<CartesianSent>();
    let options = TargetControlOptions::default()
        .with_rotation_limits(TELEOP_ROTATION)
        .with_observer(observer);
    let (mut torque, shared, _first) = cartesian_loop(options, impedance);
    torque.cycle(&arm.state);
    let start = arm.pose();
    let (position, orientation) = (translation_of(&start), rotation_of(&start));
    let turned = exp(&TOWARD_ALIGNMENT) * orientation;
    shared.slot.publish(slot_values(&position, &turned));
    let reversal = 200;
    for cycle in 1..reversal + 300 {
        if cycle == reversal {
            shared.slot.publish(slot_values(&position, &orientation));
        }
        let goal = records.lock().unwrap().last().unwrap().q_goal;
        arm.follow(&goal);
        torque.cycle(&arm.state);
    }
    let records = records.lock().unwrap();
    let length = TOWARD_ALIGNMENT.iter().map(|x| x * x).sum::<f64>().sqrt();
    let along = |v: &[f64; 3]| (0..3).map(|i| v[i] * TOWARD_ALIGNMENT[i]).sum::<f64>() / length;
    let commanded = |r: &CartesianSent| {
        let pose = arm.model.pose_q(
            Frame::EndEffector,
            &r.q_goal,
            &arm.state.F_T_EE,
            &arm.state.EE_T_K,
        );
        rotation_of(&pose)
    };
    // Braking at its jerk the generator's turn halves in 50 cycles and the command runs
    // 0.058 rad past where it was.
    let turning = along(&records[reversal - 1].angular_velocity);
    let halved = records[reversal..]
        .iter()
        .position(|r| along(&r.angular_velocity) < 0.5 * turning);
    assert!(
        halved.is_some_and(|k| k <= 60),
        "halved after {halved:?} cycles"
    );
    let reversed_at = commanded(&records[reversal - 1]);
    let past = records[reversal..]
        .iter()
        .map(|r| along(&log(&(commanded(r) * reversed_at.transpose()))))
        .fold(0.0, f64::max);
    assert!(past < 0.075, "ran {past} rad past the reversal");
    // Before the reversal joint 7 meets its bound and never passes it, and the turn goes on.
    let held = &records[reversal - 50..reversal];
    let cap = impedance.joint_velocity_fraction * max_joint_velocity(FciVersion::V5)[6];
    for r in held {
        assert!(
            r.dq_goal[6].abs() <= cap * (1.0 + 1e-9),
            "{} over {cap}",
            r.dq_goal[6]
        );
    }
    let met = held.iter().filter(|r| r.pinned[6] == 2).count();
    assert!(met >= 2, "joint 7 met its bound in {met} cycles");
    // Nor does its goal velocity saw: the restart keeps the IK's lag behind the generator, which,
    // dropped, took 0.13 to 0.32 rad/s off joint 7's goal every 13 cycles.
    let jump = held
        .windows(2)
        .flat_map(|w| (0..7).map(move |i| (w[1].dq_goal[i] - w[0].dq_goal[i]).abs()))
        .fold(0.0, f64::max);
    assert!(
        jump < 0.02,
        "a goal velocity jumped {jump} rad/s in a cycle"
    );
    let turned: Vec<f64> = held
        .iter()
        .map(|r| along(&log(&(commanded(r) * orientation.transpose()))))
        .collect();
    assert!(turned.windows(2).all(|w| w[1] > w[0]), "{turned:?}");
    let rate = (turned[49] - turned[0]) / (49.0 * DELTA_T);
    let fastest = held
        .iter()
        .map(|r| along(&r.angular_velocity))
        .fold(0.0, f64::max);
    assert!(rate >= 0.9 * fastest, "{rate} rad/s against {fastest}");
}
