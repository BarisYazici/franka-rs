//! Joint 2 driven into its margin by a Cartesian stream while the wrist is pinned at its own, on
//! the closed-loop plant: the goal stops at the margin, the arm within 0.02 rad past it, and
//! nothing rings.

use super::super::super::cartesian::slot_values;
use super::super::super::rotation::{rotation_of, translation_of};
use super::super::plant::teleop;
use super::super::plant::{Plant, PlantOptions};
use super::super::torque::{cartesian_loop_on, Rig};
use super::super::{recording, READY};
use super::fold::flips;
use super::*;
use crate::model::Frame;

// Measured: the goal enters the last 50 mrad at 0.17 / 0.20 rad/s, the arm peaks 6.4 / 9.6 mrad
// past the margin; the goals settle 2.5 / 2.0 s later, joints 1 and 3 at most 0.035 rad on, then
// the torques spread by at most 0.18 Nm over 0.5 s and no joint reverses.
#[test]
fn joint_2_driven_into_its_margin_under_a_pinned_wrist_stops_there_without_ringing() {
    let fer = Rig::fer();
    let margin = ImpedanceOptions::cartesian().joint_position_margin;
    let mut limits = fer.limits;
    (limits.1[1], limits.1[4], limits.1[6]) = (READY[1] + 0.15, READY[4] + 0.1, READY[6] + 0.1);
    let rig = fer.within(limits);
    let wall = limits.1.map(|u| u - margin);
    for speed in [0.1, 0.25] {
        let case = format!("joint 2 at {speed} rad/s");
        let mut plant = Plant::new(
            READY,
            PlantOptions {
                limits,
                ..PlantOptions::default()
            },
        );
        let (records, observer) = recording::<CartesianSent>();
        let (mut torque, shared) = cartesian_loop_on(
            &rig,
            teleop().with_observer(observer),
            ImpedanceOptions::cartesian(),
        );
        // Joint 2's reference up at `speed`, up to 0.65 rad past its margin, so that the rest of
        // the arm cannot make up for it; the wrist's turned 0.35 rad past joints 5 and 7's over
        // the first second. Held from when joint 2's goal is within 1 mrad of its margin.
        let (mut arm, mut arrived) = (Vec::new(), None);
        for k in 0.. {
            let t = arrived.unwrap_or(k) as f64 * DELTA_T;
            let mut q = READY;
            q[1] += speed * t.min(0.75 / speed);
            let turn = 0.4 * t.min(1.0);
            (q[4], q[6]) = (q[4] + turn, q[6] + turn);
            let state = plant.state();
            let pose = rig
                .model
                .pose_q(Frame::EndEffector, &q, &state.F_T_EE, &state.EE_T_K);
            let target = slot_values(&translation_of(&pose), &rotation_of(&pose));
            shared.slot.publish(target);
            let command = torque.cycle(&state);
            plant.step(&command.tau_J);
            let (q, dq) = plant.joints();
            arm.push((q, dq, plant.applied()));
            let records = records.lock().unwrap();
            if arrived.is_none() && records[k].q_goal[1] > wall[1] - 1e-3 {
                arrived = Some(k);
            }
            // Held until no joint's goal has moved 2 mrad over the last 0.5 s: the turn the wrist
            // cannot make goes through the elbow first.
            let settled = |a: usize| {
                k >= a + 500
                    && (0..7)
                        .all(|i| (records[k].q_goal[i] - records[k - 500].q_goal[i]).abs() < 2e-3)
            };
            if arrived.is_some_and(settled) || k == 15_000 {
                break;
            }
        }
        let arrived = arrived.unwrap_or_else(|| panic!("{case}: never at the margin"));
        let records = records.lock().unwrap();
        assert!(records.len() < 15_001, "{case}: never settled");
        let elbow = [0, 2]
            .map(|i| (records[records.len() - 1].q_goal[i] - records[arrived].q_goal[i]).abs());
        for r in records.iter() {
            for i in [1, 4, 6] {
                assert!(r.q_goal[i] <= wall[i] + 1e-12, "{case}: joint {i} {r:?}");
            }
        }
        // The exact solve may let a wrist pin go for a cycle: both held around the arrival.
        let around = &records[arrived.saturating_sub(50)..arrived + 50];
        let held = |i: usize| around.iter().any(|r| r.pinned[i] == 1);
        assert!(held(4) && held(6), "{case}: {:?}", records[arrived].pinned);
        let near = records.iter().position(|r| r.q_goal[1] > wall[1] - 0.05);
        let entry = records[near.unwrap()..arrived]
            .iter()
            .map(|r| r.dq_goal[1])
            .fold(0.0, f64::max);
        assert_eq!(plant.violation(), None, "{case}");
        let furthest = arm.iter().map(|(q, _, _)| q[1]).fold(f64::MIN, f64::max);
        let tail = &arm[arm.len() - 500..];
        let spread = |i: usize| {
            let t = tail.iter().map(|(_, _, tau)| tau[i]);
            t.clone().fold(f64::MIN, f64::max) - t.fold(f64::MAX, f64::min)
        };
        let spreads = [1, 4, 6].map(spread);
        // Ringing reverses the joints: none moving faster than 1 mrad/s turns around.
        let reversals = tail
            .windows(2)
            .filter(|w| (0..7).any(|i| w[0].1[i] * w[1].1[i] < 0.0 && w[0].1[i].abs() > 1e-3))
            .count();
        let count = flips(&records[50..]);
        println!(
            "{case}: at the margin after {arrived} cycles, entering at {entry:.3} rad/s, arm \
             {:.4} rad past it, settled {} cycles later with joints 1, 3 {elbow:.3?} rad on, tail \
             torque spread {spreads:.3?} Nm, {reversals} reversals, {count} flips",
            furthest - wall[1],
            records.len() - arrived,
        );
        assert!(furthest <= wall[1] + 0.02, "{case}: {furthest}");
        // Without the walls' memory of a pin the solve drops for a cycle, 0.66-0.98 rad.
        assert!(
            elbow.iter().all(|e| *e <= 0.15),
            "{case}: joints 1, 3 {elbow:?}"
        );
        assert_eq!(reversals, 0, "{case}");
        assert!(spreads.iter().all(|s| *s < 0.25), "{case}: {spreads:?}");
        assert_eq!(count, 0, "{case}");
    }
}
