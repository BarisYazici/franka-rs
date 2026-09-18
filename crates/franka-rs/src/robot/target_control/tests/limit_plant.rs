//! The torque loop on the closed-loop plant against joint limits: the CI regression of the
//! position guard and the tool matrix. Each scenario runs as the loop ran before the guard, which
//! must bring the joint within [`EVIDENCE`] of its limit (else the scenario proves nothing), and
//! guarded: every joint at least [`CLEARANCE`] from its limits, every goal inside the margin, no
//! limit violation, the measured velocity under the arm's envelope, and a reversal off
//! the margin as quick as a start from rest. The FR3's joint-space home is characterised last.

use std::sync::{Arc, Mutex};

use super::super::cartesian::slot_values;
use super::super::position::VelocityLimit;
use super::super::rotation::{exp, rotation_of, translation_of};
use super::super::torque::{TorqueLoop, Tracker};
use super::super::*;
use super::plant::as_run::{self, Restart};
use super::plant::{nearest, teleop, Plant, PlantOptions, Tool, FLANGE, HAND, LONG_TOOL, ROBOTIQ};
use super::torque::{cartesian_loop_on, joint_loop_on, Rig};
use super::{recording, READY};
use crate::model::Frame;
use crate::rate_limiting::DELTA_T;

/// The as-run loop must bring the joint this close to its limit, rad.
const EVIDENCE: f64 = 0.005;
/// The guarded loop keeps every joint this far from its limits, rad.
const CLEARANCE: f64 = 0.02;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Scenario {
    /// Joint 4's lower limit 0.3 rad under ready, the hand sent to where it is 0.1 rad past.
    Fold,
    /// Joints 5 and 7 0.1 rad under their upper limits, joint 2 0.25: the hand moves along x at
    /// 3 to 8 cm/s (joint 2 at 0.1 to 0.25 rad/s) while it turns about its own z.
    Wrist,
    /// The fold held for a second, then the target back at ready.
    Reversal,
}

impl Scenario {
    /// The rig's limits for the scenario, and the joint it drives.
    fn limits(self, rig: &Rig) -> (Rig, usize) {
        let mut limits = rig.limits;
        match self {
            Scenario::Fold | Scenario::Reversal => {
                limits.0[3] = READY[3] - 0.3;
                (rig.within(limits), 3)
            }
            Scenario::Wrist => {
                (limits.1[1], limits.1[4], limits.1[6]) =
                    (READY[1] + 0.25, READY[4] + 0.1, READY[6] + 0.1);
                (rig.within(limits), 1)
            }
        }
    }

    fn cycles(self, tool: &Tool) -> usize {
        match (self, tool.name) {
            (Scenario::Fold, "flange") => 1000,
            (Scenario::Fold, _) => 1500,
            (Scenario::Wrist, _) => 2500,
            (Scenario::Reversal, _) => 1500,
        }
    }

    /// The target at `cycle`, if one is published then, for an arm with `tool` on `rig`.
    fn target(self, rig: &Rig, tool: &Tool, cycle: usize) -> Option<[f64; 7]> {
        let ee_t_k = RobotState::default().EE_T_K;
        let pose = |q: &[f64; 7]| {
            rig.model
                .pose_q(Frame::EndEffector, q, &tool.f_t_ee, &ee_t_k)
        };
        let slot = |pose: &[f64; 16]| slot_values(&translation_of(pose), &rotation_of(pose));
        match self {
            Scenario::Fold | Scenario::Reversal if cycle == 1 => {
                let mut folded = READY;
                folded[3] -= 0.4;
                Some(slot(&pose(&folded)))
            }
            Scenario::Reversal if cycle == 1000 => Some(slot(&pose(&READY))),
            Scenario::Wrist if cycle % 20 == 1 => {
                let t = cycle as f64 * DELTA_T;
                // x = ∫ 0.03 + 0.025 t, the turn 0.3 rad over the first second.
                let (dx, turn) = (0.03 * t + 0.0125 * t * t, 0.3 * t.min(1.0));
                let mut start = pose(&READY);
                start[12] += dx;
                let rotation = rotation_of(&start) * exp(&[0.0, 0.0, turn]);
                Some(slot_values(&translation_of(&start), &rotation))
            }
            _ => None,
        }
    }
}

/// What a run on the plant measured, per cycle: the plant's configuration and velocity, and
/// the loop's records.
struct Outcome {
    q: Vec<[f64; 7]>,
    dq: Vec<[f64; 7]>,
    sent: Vec<CartesianSent>,
    violation: Option<(usize, usize)>,
}

/// `torque` on a plant of `rig` with `tool` from rest at `start` for `cycles`, `target` published
/// before the cycles it returns one for.
fn on_plant<T: Tracker<6, 7>>(
    torque: &mut TorqueLoop<6, 7, T>,
    shared: &Shared<7>,
    records: &Arc<Mutex<Vec<CartesianSent>>>,
    (rig, tool, start): (&Rig, &Tool, [f64; 7]),
    cycles: usize,
    target: impl Fn(usize) -> Option<[f64; 7]>,
) -> Outcome {
    let options = PlantOptions {
        version: rig.version,
        limits: rig.limits,
        tool: *tool,
        ..PlantOptions::default()
    };
    let mut plant = Plant::new(start, options);
    let (mut q, mut dq) = (Vec::with_capacity(cycles), Vec::with_capacity(cycles));
    for cycle in 0..cycles {
        if let Some(t) = target(cycle).filter(|_| cycle > 0) {
            shared.slot.publish(t);
        }
        plant.step(&torque.cycle(&plant.state()).tau_J);
        let (now, velocity) = plant.joints();
        q.push(now);
        dq.push(velocity);
    }
    Outcome {
        q,
        dq,
        sent: std::mem::take(&mut *records.lock().unwrap()),
        violation: plant.violation(),
    }
}

/// `scenario` with `tool` on `rig` as it ran (`guarded` false) or guarded, from `start`.
fn run(scenario: Scenario, rig: &Rig, tool: &Tool, start: [f64; 7], guarded: bool) -> Outcome {
    let cycles = scenario.cycles(tool);
    let (records, observer) = recording::<CartesianSent>();
    let options = teleop().with_observer(observer);
    let impedance = ImpedanceOptions::cartesian();
    let target = |k| scenario.target(rig, tool, k);
    let arm = (rig, tool, start);
    if guarded {
        let (mut torque, shared) = cartesian_loop_on(rig, options, impedance);
        on_plant(&mut torque, &shared, &records, arm, cycles, target)
    } else {
        let (mut torque, shared) = as_run::cartesian_loop(rig, options, impedance, Restart::End);
        on_plant(&mut torque, &shared, &records, arm, cycles, target)
    }
}

/// The cycles until joint `i`'s goal is [`POSITION_FADE_BAND`] off where it was at `from`.
fn cycles_off(sent: &[CartesianSent], from: usize, i: usize) -> Option<usize> {
    let held = sent[from].q_goal[i];
    (from..sent.len()).position(|k| (sent[k].q_goal[i] - held).abs() > POSITION_FADE_BAND)
}

/// `scenario` with `tool` on `arm`: every failed check, with its measured numbers.
fn check(scenario: Scenario, arm: &Rig, tool: &Tool) -> Vec<String> {
    let (rig, joint) = scenario.limits(arm);
    let case = format!("{scenario:?} {} {}", arm_name(arm), tool.name);
    let mut failures = Vec::new();
    let margin = ImpedanceOptions::cartesian().joint_position_margin;
    let as_run = run(scenario, &rig, tool, READY, false);
    let evidence = as_run
        .q
        .iter()
        .map(|q| (q[joint] - rig.limits.0[joint]).min(rig.limits.1[joint] - q[joint]))
        .fold(f64::INFINITY, f64::min);
    let guarded = run(scenario, &rig, tool, READY, true);
    let (clearance, nearest_joint) = nearest(&guarded.q, &rig.limits);
    let goals: Vec<[f64; 7]> = guarded.sent.iter().map(|r| r.q_goal).collect();
    let (goal_clearance, _) = nearest(&goals, &rig.limits);
    println!(
        "{case}: as run joint {} within {:.1} mrad, guarded {:.1} mrad (joint {}), goal {:.2} \
         mrad, violation {:?} / {:?}",
        joint + 1,
        evidence * 1e3,
        clearance * 1e3,
        nearest_joint + 1,
        goal_clearance * 1e3,
        as_run.violation,
        guarded.violation
    );
    if scenario == Scenario::Wrist {
        let bound = rig.limits.1[1] - margin - 0.1;
        let reach = guarded.sent.iter().position(|r| r.q_goal[1] > bound);
        let speed = reach.map(|k| guarded.sent[k].dq_goal[1]);
        let pinned = guarded
            .sent
            .iter()
            .map(|r| r.pinned)
            .fold([0i8; 7], |a, p| {
                std::array::from_fn(|i| if p[i] != 0 { p[i] } else { a[i] })
            });
        println!("{case}: joint 2's goal 0.1 rad before the margin at {speed:?} rad/s, pins seen {pinned:?}");
    }
    if evidence > EVIDENCE {
        failures.push(format!(
            "{case}: no evidence, as run {:.1} mrad",
            evidence * 1e3
        ));
    }
    if clearance < CLEARANCE || guarded.violation.is_some() {
        failures.push(format!("{case}: guarded {:.1} mrad", clearance * 1e3));
    }
    if goal_clearance < margin - 1e-12 {
        failures.push(format!(
            "{case}: goal {:.3} mrad inside",
            goal_clearance * 1e3
        ));
    }
    // The arm's own limit toward each side, and the braking profile to the limit itself: the
    // speed the joint can still come to rest from before it (flat, the FER's limit, does not).
    let velocity = VelocityLimit::of(rig.version);
    let flat = velocity.flat();
    let (lower_q, upper_q) = rig.limits;
    let over = guarded.q.iter().zip(&guarded.dq).position(|(q, dq)| {
        let (lower, upper) = velocity.at(q);
        (0..7).any(|i| {
            let (side, limit) = if dq[i] < 0.0 {
                (q[i] - lower_q[i], lower[i])
            } else {
                (upper_q[i] - q[i], upper[i])
            };
            dq[i].abs() > limit.min(velocity.braking(i, side, flat[i])) + 1e-9
        })
    });
    if let Some(k) = over {
        failures.push(format!("{case}: over the velocity limit at cycle {k}"));
    }
    if scenario == Scenario::Reversal {
        let reversed = cycles_off(&guarded.sent, 1000, joint);
        let held = guarded.q[999];
        let (records, observer) = recording::<CartesianSent>();
        let options = teleop().with_observer(observer);
        let (mut torque, shared) = cartesian_loop_on(&rig, options, ImpedanceOptions::cartesian());
        let back =
            |k: usize| (k == 1).then(|| Scenario::Reversal.target(&rig, tool, 1000).unwrap());
        let rest = on_plant(
            &mut torque,
            &shared,
            &records,
            (&rig, tool, held),
            500,
            back,
        );
        let from_rest = cycles_off(&rest.sent, 1, joint);
        println!("{case}: off the margin after {reversed:?} cycles, from rest after {from_rest:?}");
        match (reversed, from_rest) {
            (Some(r), Some(f)) if r <= f + 5 => {}
            _ => failures.push(format!(
                "{case}: reversal {reversed:?} against {from_rest:?}"
            )),
        }
    }
    failures
}

fn arm_name(rig: &Rig) -> &'static str {
    match rig.version {
        FciVersion::V5 => "FER",
        FciVersion::V10 => "FR3",
    }
}

#[test]
fn a_fold_into_joint_4_on_the_plant_holds_the_margin_with_the_hand() {
    let failures = check(Scenario::Fold, &Rig::fer(), &HAND);
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
#[ignore = "the tool matrix: release, a minute"]
fn the_tool_matrix_on_the_plant() {
    let mut failures = Vec::new();
    for scenario in [Scenario::Fold, Scenario::Wrist, Scenario::Reversal] {
        for tool in [FLANGE, HAND, ROBOTIQ, LONG_TOOL] {
            if (scenario, tool.name) != (Scenario::Fold, "hand") {
                failures.extend(check(scenario, &Rig::fer(), &tool));
            }
        }
        failures.extend(check(scenario, &Rig::fr3(), &ROBOTIQ));
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// The FR3's joint-space home (0.2 of its limits, about 2 s) on the plant with the joint
/// preset: joint 5's torque peak and dominant frequency. The arm showed 35.5 Hz at ±2.7 Nm.
#[test]
#[ignore = "a characterisation, release, prints"]
fn the_fr3_joint_home_on_the_plant_characterises_joint_5() {
    let rig = Rig::fr3();
    let offset = [0.3, -0.3, 0.3, -0.5, 0.6, 0.4, -0.9];
    let start: [f64; 7] = std::array::from_fn(|i| READY[i] + offset[i]);
    let impedance = ImpedanceOptions::joint();
    let budget = JointTargetControlOptions::scaled_limits(rig.version, 0.2);
    let options = JointTargetControlOptions::default().with_max_deviation(10.0);
    let (mut torque, shared) = joint_loop_on(&rig, options, budget, impedance);
    let plant_options = PlantOptions {
        version: rig.version,
        limits: rig.limits,
        torque_limits: impedance.torque_limits,
        tool: ROBOTIQ,
        ..PlantOptions::default()
    };
    let mut plant = Plant::new(start, plant_options);
    let mut tau5 = Vec::new();
    for cycle in 0..3000 {
        if cycle == 1 {
            shared.slot.publish(READY);
        }
        plant.step(&torque.cycle(&plant.state()).tau_J);
        tau5.push(plant.applied()[4]);
    }
    let peak = tau5.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    // The oscillation on top of the move: the signal less its 50 ms moving mean.
    let smooth: Vec<f64> = (0..tau5.len())
        .map(|k| {
            let window = &tau5[k.saturating_sub(25)..(k + 25).min(tau5.len())];
            window.iter().sum::<f64>() / window.len() as f64
        })
        .collect();
    let ripple: Vec<f64> = tau5.iter().zip(&smooth).map(|(a, b)| a - b).collect();
    let spectrum = |hz: f64| {
        let w = 2.0 * std::f64::consts::PI * hz * DELTA_T;
        let (c, s) = ripple
            .iter()
            .enumerate()
            .fold((0.0, 0.0), |(c, s), (k, x)| {
                (c + x * (w * k as f64).cos(), s + x * (w * k as f64).sin())
            });
        c.hypot(s)
    };
    let (dominant, _) = (50..=1000)
        .map(|tenth| (tenth as f64 * 0.1, spectrum(tenth as f64 * 0.1)))
        .fold((0.0, 0.0), |best, x| if x.1 > best.1 { x } else { best });
    let ripple_peak = ripple.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    println!(
        "FR3 home: joint 5 torque peak {peak:.2} Nm, ripple ±{ripple_peak:.2} Nm at {dominant:.1} Hz"
    );
}
