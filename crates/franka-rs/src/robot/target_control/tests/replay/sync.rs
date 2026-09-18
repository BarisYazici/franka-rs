//! When the loop picked each recorded target up. The node publishes a target and then records
//! it as accepted, so the recorded cycle can fall on or after the loop's pickup, or before it
//! where the write raced the loop's read: the recording does not say which. The recorded goals
//! do. The as-run generator and tracker (no torque law: a commanded goal does not depend on it)
//! run through the session; at each target copies of them try every pickup from two cycles before
//! the recorded one to two after it over a short window, and the run goes on with the best.

use std::sync::{mpsc, Arc};

use super::super::super::cartesian::{self, axis_limits};
use super::super::super::runner::{identity, Chart, Step};
use super::super::super::*;
use super::super::plant::as_run::{joint_tracker, pose_tracker, Restart, Resume};
use super::csv::{load, oracle, recording_path, Arm, Session};
use super::{loop_options, Config};
use crate::otg::OtgLimits;

/// Cycles each candidate runs for before the choice.
const WINDOW: usize = 12;
/// How many cycles before and after the recorded one a pickup is tried on.
const PICKUP: usize = 2;

/// The generator and the tracker of a loop as it ran, without the torque law.
struct Machine<const N: usize, const S: usize, T> {
    runner: Runner<N, S>,
    shared: Arc<Shared<S>>,
    tracker: T,
    axes: [OtgLimits; N],
    chart: Chart<N, S>,
    leash: Leash,
    /// The generator's velocity and acceleration after the last step.
    motion: ([f64; N], [f64; N]),
    /// What the slot holds.
    target: [f64; S],
    anchored: bool,
    resumed: bool,
}

impl<const N: usize, const S: usize, T: Resume<N, S>> Machine<N, S, T> {
    fn new(tracker: T, axes: [OtgLimits; N], chart: Chart<N, S>, leash: Leash) -> Self {
        let shared = Arc::new(Shared::<S>::default());
        let (started, _) = mpsc::sync_channel(1);
        let settle = Settle::default();
        let runner = Runner::new(Arc::clone(&shared), started, axes, settle, chart).unwrap();
        Machine {
            runner,
            shared,
            tracker,
            axes,
            chart,
            leash,
            motion: ([0.0; N], [0.0; N]),
            target: [0.0; S],
            anchored: false,
            resumed: false,
        }
    }

    /// One cycle as `TorqueLoop::cycle` runs it up to the goal.
    fn cycle(&mut self, state: &RobotState) -> [f64; 7] {
        let commanded = self.tracker.anchor(state, &self.leash);
        let strayed = self.tracker.strayed(state);
        let restart = self
            .tracker
            .restart()
            .or(self.resumed.then_some(self.motion));
        if let Some((velocity, acceleration)) = restart {
            self.runner.restart(velocity, acceleration);
        }
        self.resumed = false;
        let step: Step<N, S> = self.runner.cycle(state, commanded, strayed, false);
        if !self.anchored {
            (self.anchored, self.target) = (true, commanded);
        }
        self.motion = (step.velocity, step.acceleration);
        self.tracker.goal(&step).0
    }

    fn publish(&mut self, target: [f64; S]) {
        self.shared.slot.publish(target);
        self.target = target;
    }

    /// A copy whose next cycle is this one's: a fresh generator anchored on the slot's target,
    /// restarted at this one's motion on that cycle.
    fn fork(&self, state: &RobotState) -> Self {
        let mut copy = Machine::new(self.tracker.resume(), self.axes, self.chart, self.leash);
        copy.runner.cycle(state, self.target, false, false);
        copy.publish(self.target);
        (copy.motion, copy.anchored, copy.resumed) = (self.motion, true, true);
        copy
    }
}

/// The targets at the cycles a loop picked them up on, and its goals.
pub(in super::super) type Synchronised<const S: usize> = (Vec<(usize, [f64; S])>, Vec<[f64; 7]>);

/// The goals of `tracker` as it ran through `session` commanded on `state(cycle)`, each of
/// `targets` (recorded cycle, value) picked up on the cycle, from two before the recorded one to
/// two after it and never before the previous pickup, that best reproduces the recorded goals.
/// A pickup is scored over the window from the first cycle it may fall on, the later targets
/// there published at the same offset from their recorded cycles: by the largest joint
/// difference, then its sum. A target picked up on the cycle of the next is overwritten by it.
pub(in super::super) fn synchronised<const N: usize, const S: usize, T: Resume<N, S>>(
    session: &Session,
    targets: &[(usize, [f64; S])],
    tracker: T,
    (axes, chart, leash): ([OtgLimits; N], Chart<N, S>, Leash),
    state: impl Fn(usize) -> RobotState,
) -> Synchronised<S> {
    let n = session.t.len();
    let mut machine = Machine::new(tracker, axes, chart, leash);
    let (mut picked, mut goals) = (Vec::with_capacity(targets.len()), Vec::with_capacity(n));
    // The chosen pickup not yet published.
    let mut pending: Option<(usize, [f64; S])> = None;
    let mut previous = 1;
    for (i, &(row, value)) in targets.iter().enumerate() {
        let first = row.saturating_sub(PICKUP).max(previous);
        if first >= n {
            break;
        }
        advance(&mut machine, &mut goals, &mut pending, first, &state);
        let (last, end) = ((row + PICKUP).max(first), (first + WINDOW).min(n));
        let mut best = (f64::INFINITY, f64::INFINITY, first);
        for pickup in (first < last).then_some(first..=last).into_iter().flatten() {
            let offset = pickup as isize - row as isize;
            let mut candidate = machine.fork(&state(first));
            let (mut worst, mut sum, mut j) = (0.0f64, 0.0, i);
            for cycle in first..end {
                let due = targets.get(j).map(|t| (t.0 as isize + offset) as usize);
                if due.is_some_and(|due| due <= cycle) {
                    candidate.publish(targets[j].1);
                    j += 1;
                }
                let goal = candidate.cycle(&state(cycle));
                let recorded = &session.q_goal[cycle];
                for (g, r) in goal.iter().zip(recorded) {
                    worst = worst.max((g - r).abs());
                    sum += (g - r).abs();
                }
            }
            if worst < best.0 || (worst == best.0 && sum < best.1) {
                best = (worst, sum, pickup);
            }
        }
        pending = Some((best.2, value));
        previous = best.2;
        picked.push((best.2, value));
    }
    advance(&mut machine, &mut goals, &mut pending, n, &state);
    (picked, goals)
}

/// Runs `machine` on to cycle `until`, `pending` published before its cycle.
fn advance<const N: usize, const S: usize, T: Resume<N, S>>(
    machine: &mut Machine<N, S, T>,
    goals: &mut Vec<[f64; 7]>,
    pending: &mut Option<(usize, [f64; S])>,
    until: usize,
    state: &impl Fn(usize) -> RobotState,
) {
    loop {
        let k = goals.len();
        if let Some((_, value)) = pending.filter(|p| p.0 == k) {
            machine.publish(value);
            *pending = None;
        }
        if k >= until {
            return;
        }
        goals.push(machine.cycle(&state(k)));
    }
}

/// The Cartesian loop as it ran at `config` through `session` on `arm`, the posture the first
/// recorded configuration.
pub(in super::super) fn cartesian_as_run(
    session: &Session,
    arm: &Arm,
    config: Config,
    restart: Restart,
) -> Synchronised<7> {
    let (options, impedance) = loop_options(config, |_: &RobotState, _: &CartesianSent| {});
    let impedance = impedance.with_posture(Some(session.q[0]));
    let tracker = pose_tracker(&arm.rig, &options, impedance, restart);
    let axes = axis_limits(options.limits, options.rotation_limits);
    let run = (axes, cartesian::chart as Chart<6, 7>, impedance.leash);
    synchronised(session, &session.targets, tracker, run, |k| {
        arm.recorded(session, k)
    })
}

/// The joint loop as it ran under `budget` with `impedance` through `session` on the FER.
pub(in super::super) fn joint_as_run(
    session: &Session,
    budget: [OtgLimits; 7],
    impedance: ImpedanceOptions,
) -> Synchronised<7> {
    let arm = Arm::fer();
    let options = JointTargetControlOptions::default().with_max_deviation(100.0);
    let tracker = joint_tracker(&arm.rig, &options, impedance, Restart::End);
    let run = (budget, identity as Chart<7, 7>, impedance.leash);
    synchronised(session, &session.joint_targets, tracker, run, |k| {
        arm.recorded(session, k)
    })
}

/// The session `name` on `arm` and the Cartesian loop as it ran through it at `config`: the
/// session, the picked-up targets and the goals; `None` without `REPLAY_DIR`. The oracle's
/// numbers are printed, its failure goes to `failures`.
pub(in super::super) fn reproduced(
    name: &str,
    arm: &Arm,
    config: Config,
    restart: Restart,
    failures: &mut Vec<String>,
) -> Option<(Session, Synchronised<7>)> {
    let Some(path) = recording_path(name) else {
        println!("{name}: REPLAY_DIR is not set, skipped");
        return None;
    };
    let session = load(&path);
    let (targets, goals) = cartesian_as_run(&session, arm, config, restart);
    check_oracle(name, &goals, &session.q_goal, failures);
    Some((session, (targets, goals)))
}

/// Prints the oracle of `goals` against `recorded` and adds its failure to `failures`.
pub(in super::super) fn check_oracle(
    case: &str,
    goals: &[[f64; 7]],
    recorded: &[[f64; 7]],
    failures: &mut Vec<String>,
) {
    let (median, p99, first) = oracle(goals, recorded);
    let numbers = format!(
        "{case}: as run over {} cycles, q_goal median {:.3} µrad, p99 {:.3} µrad, first above \
         20 µrad at {first:?}",
        goals.len(),
        median * 1e6,
        p99 * 1e6
    );
    println!("{numbers}");
    if median > 1e-6 || p99 > 20e-6 {
        failures.push(format!(
            "{numbers} -- the loop as it ran diverges from the recorded goals; open harness item \
             (target pickup, or as_run's numerics against the node's build), not a guard result"
        ));
    }
}
