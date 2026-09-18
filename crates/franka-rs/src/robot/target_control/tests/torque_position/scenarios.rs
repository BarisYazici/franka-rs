//! The torque loops against a joint limit, the arm following the goal one cycle behind: the
//! torque's attribution, both arms' envelopes on both interfaces, a joint target at the margin, a Cartesian target
//! beyond reach held, stopped and reversed.

use std::sync::atomic::Ordering;

use super::super::super::cartesian::slot_values;
use super::super::super::rotation::{rotation_of, translation_of};
use super::super::super::runner::Step;
use super::super::super::torque::{JointTracker, Tracker};
use super::super::plant::{teleop, FLANGE};
use super::super::torque::{cartesian_loop_on, joint_loop_on, Rig};
use super::super::{recording, Arm, READY};
use super::*;
use crate::model::Frame;

/// An arm on `rig`'s model at rest in `q` with `tool` as `F_T_EE`.
pub(super) fn arm_on(rig: &Rig, q: [f64; 7], tool: [f64; 16]) -> Arm {
    let mut state = RobotState {
        q,
        q_d: q,
        F_T_EE: tool,
        ..RobotState::default()
    };
    state.O_T_EE = rig.model.pose(Frame::EndEffector, &state);
    state.O_T_EE_c = state.O_T_EE;
    Arm {
        model: Arc::clone(&rig.model),
        state,
    }
}

#[test]
fn the_loop_sends_the_clamped_sum_of_the_law_and_both_envelopes_and_reports_each() {
    let rig = Rig::fer();
    let impedance = ImpedanceOptions::joint();
    let guard = JointLimits {
        position: rig.limits,
        margin: impedance.joint_position_margin,
        fraction: impedance.joint_velocity_fraction,
        velocity: VelocityLimit::of(rig.version),
    };
    // Joint 4 in the fade band moving up, its damping pushing down, joint 6 inside the barrier's
    // onset, joint 7 fast: every stage at once, through the spring's ramp.
    let mut q = READY;
    (q[3], q[5]) = (rig.limits.0[3] + 0.04, rig.limits.1[5] - 0.02);
    let mut arm = arm_on(&rig, q, FLANGE.f_t_ee);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default().with_observer(observer);
    let budget = JointTargetControlOptions::scaled_limits(rig.version, DEFAULT_LIMIT_FRACTION);
    let (mut torque, shared) = joint_loop_on(&rig, options, budget, impedance);
    torque.cycle(&arm.state);
    shared.slot.publish(READY);
    arm.state.dq = [0.0, 0.0, 0.0, 0.3, 0.0, 0.2, 2.4];
    let mut engaged = [false; 3];
    for cycle in 1..700u32 {
        let output = torque.cycle(&arm.state);
        let sent = *records.lock().unwrap().last().unwrap();
        let law = impedance_torques(
            &impedance,
            &arm.model.zero_jacobian(Frame::EndEffector, &arm.state),
            &sent.q_goal,
            &sent.dq_goal,
            &arm.state.q,
            &arm.state.dq,
            &arm.model.coriolis(&arm.state),
        );
        let (q, dq, clamp) = (&arm.state.q, &arm.state.dq, &impedance.torque_limits);
        let (start, onset) = guard.envelope(q, dq, impedance.velocity_barrier_fraction);
        let kept_v = fade_push(&law, dq, &start, &onset);
        let kept = guard.fade_toward(&kept_v, q);
        let barrier = velocity_barrier(dq, &onset, &barrier_gains(clamp), clamp);
        let spring = guard.spring(q, clamp, spring_ramp(cycle));
        for i in 0..7 {
            let sum =
                (law[i] + sent.tau_envelope[i] + sent.tau_position[i]).clamp(-clamp[i], clamp[i]);
            assert!(
                (sent.tau[i] - sum).abs() < 1e-12,
                "cycle {cycle}, joint {i}"
            );
            let expected = (kept[i] + barrier[i] + spring[i]).clamp(-clamp[i], clamp[i]);
            assert!(
                (sent.tau[i] - expected).abs() < 1e-12,
                "cycle {cycle}, joint {i}"
            );
            let envelope = barrier[i] - (law[i] - kept_v[i]);
            assert!(
                (sent.tau_envelope[i] - envelope).abs() < 1e-12,
                "cycle {cycle}, joint {i}"
            );
            let position = spring[i] - (kept_v[i] - kept[i]);
            assert!(
                (sent.tau_position[i] - position).abs() < 1e-12,
                "cycle {cycle}, joint {i}"
            );
        }
        assert_eq!(output.tau_J, sent.tau);
        let shares = [
            sent.tau_envelope[6],
            sent.tau_position[3],
            sent.tau_position[5],
        ];
        for (engaged, share) in engaged.iter_mut().zip(shares) {
            *engaged |= share != 0.0;
        }
    }
    assert_eq!(engaged, [true; 3]);
}

/// Every joint of `rig` driven to each of its limits less the margin under the full budget,
/// the arm following: `check` sees each cycle's arm and record.
fn to_every_limit(rig: &Rig, start: [f64; 7], mut check: impl FnMut(usize, &Arm, &JointSent)) {
    let budget = JointTargetControlOptions::scaled_limits(rig.version, 1.0);
    let margin = ImpedanceOptions::joint().joint_position_margin;
    for i in 0..7 {
        for side in [0, 1] {
            let mut arm = arm_on(rig, start, FLANGE.f_t_ee);
            let (records, observer) = recording::<JointSent>();
            let options = JointTargetControlOptions::default()
                .with_max_deviation(10.0)
                .with_observer(observer);
            let (mut torque, shared) =
                joint_loop_on(rig, options, budget, ImpedanceOptions::joint());
            torque.cycle(&arm.state);
            let mut target = start;
            target[i] = [rig.limits.0[i] + margin, rig.limits.1[i] - margin][side];
            shared.slot.publish(target);
            for _ in 0..3000 {
                let sent = *records.lock().unwrap().last().unwrap();
                arm.follow(&sent.q_goal);
                torque.cycle(&arm.state);
                check(i, &arm, records.lock().unwrap().last().unwrap());
            }
            let last = records.lock().unwrap().last().unwrap().q_goal[i];
            assert!(
                (last - target[i]).abs() < 1e-3,
                "joint {i} side {side}: {last}"
            );
        }
    }
}

/// Joint `i` of an arm at `start` on `rig` placed 0.15 rad inside the margin of `side` (0 lower),
/// the Cartesian loop sent 0.3 rad beyond, the arm following: `check` sees each cycle's arm and
/// record.
fn to_every_limit_by_pose(rig: &Rig, start: [f64; 7], mut check: impl FnMut(&Arm, &CartesianSent)) {
    let margin = ImpedanceOptions::cartesian().joint_position_margin;
    let ee_t_k = RobotState::default().EE_T_K;
    for i in 0..7 {
        for side in [0, 1] {
            let sign = [-1.0, 1.0][side];
            let mut q = start;
            q[i] = [rig.limits.0[i], rig.limits.1[i]][side] - sign * (margin + 0.15);
            let mut arm = arm_on(rig, q, FLANGE.f_t_ee);
            let (records, observer) = recording::<CartesianSent>();
            let options = teleop().with_observer(observer);
            let (mut torque, shared) =
                cartesian_loop_on(rig, options, ImpedanceOptions::cartesian());
            torque.cycle(&arm.state);
            let mut beyond = q;
            beyond[i] += sign * 0.3;
            let pose = rig
                .model
                .pose_q(Frame::EndEffector, &beyond, &FLANGE.f_t_ee, &ee_t_k);
            shared
                .slot
                .publish(slot_values(&translation_of(&pose), &rotation_of(&pose)));
            let mut reached = f64::INFINITY;
            for _ in 0..700 {
                let sent = *records.lock().unwrap().last().unwrap();
                arm.follow(&sent.q_goal);
                torque.cycle(&arm.state);
                let sent = *records.lock().unwrap().last().unwrap();
                reached =
                    reached.min(sign * ([rig.limits.0[i], rig.limits.1[i]][side] - sent.q_goal[i]));
                check(&arm, &sent);
            }
            // Joints 1 and 3 stop 20 to 27 mrad short: the other takes up the rest of the pose.
            assert!(reached - margin < 0.03, "joint {i} side {side}: {reached}");
            assert!(reached >= margin - 1e-9, "joint {i} side {side}: {reached}");
        }
    }
}

/// Asserts the goal's velocity under `f` of `rig`'s velocity limit and of the braking profile to
/// the margin, both where the step ends, and the loop's fade start and onset under `f` and `vbf`
/// of the limit at the arm and where its velocity takes it in a cycle.
fn under_the_envelope(rig: &Rig) -> impl Fn(&Arm, &[f64; 7], &[f64; 7]) {
    let impedance = ImpedanceOptions::joint();
    let (f, vbf) = (
        impedance.joint_velocity_fraction,
        impedance.velocity_barrier_fraction,
    );
    let guard = JointLimits {
        position: rig.limits,
        margin: impedance.joint_position_margin,
        fraction: f,
        velocity: VelocityLimit::of(rig.version),
    };
    move |arm: &Arm, q_goal: &[f64; 7], dq_goal: &[f64; 7]| {
        let toward = |q: &[f64; 7], dq: f64, i: usize| {
            let (lower, upper) = guard.velocity.at(q);
            if dq < 0.0 {
                lower[i]
            } else {
                upper[i]
            }
        };
        let (q, dq) = (&arm.state.q, &arm.state.dq);
        let next: [f64; 7] = std::array::from_fn(|i| q[i] + dq[i] * DELTA_T);
        let (fade, onset) = guard.envelope(q, dq, vbf);
        let flat = guard.velocity.flat();
        let (lower, upper) = guard.position;
        for i in 0..7 {
            let s = if dq_goal[i] < 0.0 {
                q_goal[i] - lower[i]
            } else {
                upper[i] - q_goal[i]
            };
            let limit = toward(q_goal, dq_goal[i], i).min(guard.velocity.braking(
                i,
                s - guard.margin,
                flat[i],
            ));
            assert!(
                dq_goal[i].abs() <= f * limit * (1.0 + 1e-9) + 1e-12,
                "joint {i}: {} at {q_goal:?}",
                dq_goal[i]
            );
            let limit = toward(q, dq[i], i).min(toward(&next, dq[i], i));
            assert!(
                fade[i] <= f * limit + 1e-12 && onset[i] <= vbf * limit + 1e-12,
                "joint {i}"
            );
        }
    }
}

/// Mid-range on both arms, joint 6 well inside the FR3's lower limit.
fn envelope_start() -> [f64; 7] {
    let mut start = READY;
    start[5] = 2.0;
    start
}

#[test]
fn the_joint_goal_and_the_envelope_never_ask_more_than_the_velocity_limits() {
    for rig in [Rig::fer(), Rig::fr3()] {
        let check = under_the_envelope(&rig);
        to_every_limit(&rig, envelope_start(), |_, arm, sent| {
            check(arm, &sent.q_goal, &sent.dq_goal)
        });
    }
}

#[test]
fn the_cartesian_goal_and_the_envelope_never_ask_more_than_the_velocity_limits() {
    for rig in [Rig::fer(), Rig::fr3()] {
        let check = under_the_envelope(&rig);
        to_every_limit_by_pose(&rig, envelope_start(), |arm, sent| {
            check(arm, &sent.q_goal, &sent.dq_goal)
        });
    }
}

#[test]
fn a_joint_target_at_the_margin_lands_under_the_braking_envelope_and_the_stop_finishes() {
    let fer = Rig::fer();
    let mut limits = fer.limits;
    limits.0[3] = READY[3] - 0.3;
    let rig = fer.within(limits);
    let impedance = ImpedanceOptions::joint();
    let (margin, f) = (
        impedance.joint_position_margin,
        impedance.joint_velocity_fraction,
    );
    let flat = max_joint_velocity(FciVersion::V5);
    let mut arm = arm_on(&rig, READY, FLANGE.f_t_ee);
    let (records, observer) = recording::<JointSent>();
    let options = JointTargetControlOptions::default().with_observer(observer);
    let budget = JointTargetControlOptions::scaled_limits(rig.version, DEFAULT_LIMIT_FRACTION);
    let (mut torque, shared) = joint_loop_on(&rig, options, budget, impedance);
    torque.cycle(&arm.state);
    let mut target = READY;
    target[3] = limits.0[3] + margin;
    shared.slot.publish(target);
    shared.stop.store(true, Ordering::SeqCst);
    let (mut pinned, mut landed) = (None, None);
    let cycles = 1 + super::super::torque::cycles_to_finish(|| {
        let (goal, count) = {
            let records = records.lock().unwrap();
            (records.last().unwrap().q_goal, records.len())
        };
        let x = goal[3] - limits.0[3] - margin;
        arm.follow(&goal);
        if landed.is_none() && (goal[3] - target[3]).abs() < Settle::default().tolerance {
            landed = Some(count);
        }
        if pinned.is_none() && records.lock().unwrap().last().unwrap().pinned[3] == -1 {
            pinned = Some(count);
        }
        let finished = torque.cycle(&arm.state).motion_finished;
        let sent = records.lock().unwrap().last().unwrap().dq_goal[3];
        let braking = f * VelocityLimit::of(rig.version).braking(3, x, flat[3]);
        assert!(
            -sent <= braking * (1.0 + 1e-9) + 1e-12,
            "{sent} over {braking} at {x}"
        );
        finished
    });
    let (pinned, landed) = (pinned.expect("pinned"), landed.expect("landed"));
    println!("joint 4 to the margin: pinned {pinned}, landed {landed}, finished {cycles} cycles");
    // Once the goal brakes for the margin, the tail lands within a second.
    assert!(
        landed - pinned <= 1000,
        "landed {} cycles after the pin",
        landed - pinned
    );
    let last = records.lock().unwrap().last().unwrap().q_goal[3];
    assert!(last >= limits.0[3] + margin - 1e-12 && (last - target[3]).abs() < 1e-3);
    assert!(torque.finish(Ok(())).is_ok());
}

#[test]
fn a_joint_goal_cut_at_its_margin_restarts_with_nothing_winding_into_it() {
    let rig = Rig::fer();
    let impedance = ImpedanceOptions::joint();
    let limits = JointLimits {
        position: rig.limits,
        margin: impedance.joint_position_margin,
        fraction: impedance.joint_velocity_fraction,
        velocity: VelocityLimit::of(rig.version),
    };
    let restarted = |q: [f64; 7], planned: [f64; 7], v: [f64; 7], a: [f64; 7]| {
        let options = JointTargetControlOptions::default();
        let mut tracker = JointTracker::new(&options, limits);
        tracker.anchor(&arm_on(&rig, q, FLANGE.f_t_ee).state, &impedance.leash);
        let step = Step {
            position: std::array::from_fn(|i| q[i] + planned[i]),
            velocity: v,
            acceleration: a,
            target: q,
            hold: false,
            finished: false,
        };
        tracker.goal(&step);
        tracker.restart().expect("cut")
    };
    let (mut planned, mut v, mut a) = ([0.0; 7], [0.0; 7], [0.0; 7]);
    // Joint 4 on its margin, planned into it; joint 1 at rest, accelerating.
    (planned[3], v[3], a[3], a[0]) = (-2e-4, -0.2, -3.0, 1.0);
    let mut q = READY;
    q[3] = limits.position.0[3] + limits.margin;
    assert_eq!(restarted(q, planned, v, a), ([0.0; 7], [0.0; 7]));
    // A velocity cut alone, joint 1 far from its limits, keeps an acceleration where the velocity
    // is zero, as the cap does.
    let (mut planned, mut v, mut a) = ([0.0; 7], [0.0; 7], [0.0; 7]);
    (planned[0], v[0], a[0], a[1]) = (0.01, 10.0, 3.0, 1.0);
    let (v, a) = restarted(READY, planned, v, a);
    assert!(v[0] > 0.0 && a[0] == 0.0 && a[1] == 1.0, "{v:?} {a:?}");
}
