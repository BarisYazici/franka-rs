//! A Cartesian target that folds joint 4 past its margin, the arm following the goal one cycle
//! behind: held without windup or chatter, stopped, and reversed.

use std::sync::atomic::Ordering;
use std::sync::Mutex;

use super::super::super::cartesian::slot_values;
use super::super::super::rotation::{rotation_of, translation_of};
use super::super::plant::{teleop, Tool, FLANGE, HAND, LONG_TOOL};
use super::super::torque::{cartesian_loop_on, cycles_to_finish, PoseLoop, Rig};
use super::super::{recording, Arm, READY};
use super::scenarios::arm_on;
use super::*;

/// The rig with joint 4's lower limit 0.3 rad under the ready pose.
pub(super) fn folding(rig: &Rig) -> Rig {
    let mut limits = rig.limits;
    limits.0[3] = READY[3] - 0.3;
    rig.within(limits)
}

/// A Cartesian loop on `rig` from `start` with `tool`, the arm following it.
pub(super) struct Fold {
    pub(super) arm: Arm,
    pub(super) torque: PoseLoop,
    pub(super) shared: Arc<Shared<7>>,
    pub(super) records: Arc<Mutex<Vec<CartesianSent>>>,
}

impl Fold {
    pub(super) fn new(rig: &Rig, start: [f64; 7], tool: [f64; 16]) -> Fold {
        Fold::with(rig, start, tool, teleop())
    }

    /// [`new`](Self::new) under `options`.
    pub(super) fn with(
        rig: &Rig,
        start: [f64; 7],
        tool: [f64; 16],
        options: TargetControlOptions,
    ) -> Fold {
        let arm = arm_on(rig, start, tool);
        let (records, observer) = recording::<CartesianSent>();
        let options = options.with_observer(observer);
        let (mut torque, shared) = cartesian_loop_on(rig, options, ImpedanceOptions::cartesian());
        torque.cycle(&arm.state);
        Fold {
            arm,
            torque,
            shared,
            records,
        }
    }

    /// Targets the pose of `q` and runs `cycles`; whether the last one finished.
    pub(super) fn toward(&mut self, q: &[f64; 7], cycles: usize) -> bool {
        let pose = self.pose_of(q);
        self.to(&pose, cycles)
    }

    /// Targets `pose` and runs `cycles`; whether the last one finished.
    pub(super) fn to(&mut self, pose: &[f64; 16], cycles: usize) -> bool {
        let target = slot_values(&translation_of(pose), &rotation_of(pose));
        self.shared.slot.publish(target);
        (0..cycles).any(|_| self.cycle())
    }

    /// The end effector's pose at `q` with the arm's tool.
    pub(super) fn pose_of(&self, q: &[f64; 7]) -> [f64; 16] {
        let state = &self.arm.state;
        let frame = crate::model::Frame::EndEffector;
        self.arm
            .model
            .pose_q(frame, q, &state.F_T_EE, &state.EE_T_K)
    }

    pub(super) fn cycle(&mut self) -> bool {
        let goal = self.last().q_goal;
        self.arm.follow(&goal);
        self.torque.cycle(&self.arm.state).motion_finished
    }

    pub(super) fn last(&self) -> CartesianSent {
        *self.records.lock().unwrap().last().unwrap()
    }
}

/// Ready with joint 4 folded 0.4 rad: 0.1 rad past the fold's limit.
pub(super) fn folded() -> [f64; 7] {
    let mut q = READY;
    q[3] -= 0.4;
    q
}

/// The tools every fold runs with: the pins, the stall and the restart are the same whatever it is.
const TOOLS: [Tool; 3] = [FLANGE, HAND, LONG_TOOL];

// Measured, per tool: at most 0.24 mm/s once held, the stop finishes after 251 cycles.
#[test]
fn a_fold_past_the_margin_holds_the_goal_at_it_without_windup_and_a_stop_finishes() {
    let rig = folding(&Rig::fer());
    let bound = rig.limits.0[3] + ImpedanceOptions::cartesian().joint_position_margin;
    let mut failures = Vec::new();
    for tool in TOOLS {
        // Held 4 s: the flange's goal slides along the margin for 3 s before it settles.
        let mut fold = Fold::new(&rig, READY, tool.f_t_ee);
        fold.toward(&folded(), 4000);
        let records = fold.records.lock().unwrap().clone();
        assert!(records.iter().all(|r| r.q_goal[3] >= bound - 1e-12));
        let held = &records[records.len() - 200..];
        let last = held[held.len() - 1];
        let speed = |r: &CartesianSent| {
            let v = r.velocity.iter().chain(&r.angular_velocity);
            v.fold(0.0f64, |m, x| m.max(x.abs()))
        };
        let (fastest, leashed) = held.iter().fold((0.0f64, 0.0f64), |(f, l), r| {
            (f.max(speed(r)), l.max(r.leash_alteration))
        });
        let onset = records.iter().map(|r| r.stall_pressure).fold(0.0, f64::max);
        fold.shared.stop.store(true, Ordering::SeqCst);
        let cycles = cycles_to_finish(|| fold.cycle());
        println!(
            "{}: generator at most {fastest:.2e} once held, leash {leashed:.2e}, ik_error {:.2e}, \
             peak stall pressure {onset:.2e}, stop after {cycles} cycles",
            tool.name, last.ik_error
        );
        // Restarted at rest into the wall, the generator moves by one cycle of its jerk at most.
        let jerk = 500.0 / 3f64.sqrt();
        if fastest > REST_VELOCITY + 0.5 * jerk * DELTA_T * DELTA_T
            || leashed != 0.0
            || cycles > Settle::default().cycles + 100
            || fold.torque.finish(Ok(())).is_err()
        {
            failures.push(tool.name);
        }
    }
    assert!(failures.is_empty(), "{failures:?}");
}

// Measured, per tool: 60 cycles against 60 from rest.
#[test]
fn a_fold_held_at_the_margin_turns_back_as_fast_as_from_rest() {
    let rig = folding(&Rig::fer());
    let band = POSITION_FADE_BAND;
    let out = |fold: &mut Fold| {
        let from = fold.last().q_goal[3];
        fold.toward(&READY, 0);
        (1..=1000).find(|_| {
            fold.cycle();
            fold.last().q_goal[3] > from + band
        })
    };
    for tool in TOOLS {
        let mut held = Fold::new(&rig, READY, tool.f_t_ee);
        held.toward(&folded(), 2000);
        let pinned = held.last().q_goal;
        let reversed = out(&mut held).expect("turned back");
        let mut rest = Fold::new(&rig, pinned, tool.f_t_ee);
        let from_rest = out(&mut rest).expect("left");
        println!(
            "{}: turned back after {reversed} cycles, from rest after {from_rest}",
            tool.name
        );
        assert!(
            reversed <= from_rest + 5,
            "{}: {reversed} against {from_rest}",
            tool.name
        );
    }
}

/// Joint `i`'s goal increments that reverse from one cycle to the next, both above 0.5 mrad.
pub(super) fn flips(records: &[CartesianSent]) -> usize {
    records
        .windows(3)
        .map(|w| {
            (0..7)
                .filter(|&i| {
                    let (a, b) = (
                        w[1].q_goal[i] - w[0].q_goal[i],
                        w[2].q_goal[i] - w[1].q_goal[i],
                    );
                    a * b < 0.0 && a.abs() > 5e-4 && b.abs() > 5e-4
                })
                .count()
        })
        .sum()
}

// Measured: no flip on either arm with either tool, at most 5 passes.
#[test]
fn a_fold_held_far_past_reach_does_not_chatter() {
    for (name, rig) in [("FER", Rig::fer()), ("FR3", Rig::fr3())] {
        for tool in TOOLS {
            let rig = folding(&rig);
            let mut fold = Fold::new(&rig, READY, tool.f_t_ee);
            fold.toward(&folded(), 3000);
            let records = fold.records.lock().unwrap();
            let passes = records.iter().map(|r| r.ik_passes).max().unwrap();
            let count = flips(&records[50..]);
            println!(
                "{name} {}: {count} flips, at most {passes} passes",
                tool.name
            );
            assert_eq!(count, 0, "{name} {}", tool.name);
        }
    }
}
