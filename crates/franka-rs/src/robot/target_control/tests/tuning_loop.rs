//! The loop's apply step: that a published target reaches every site the option was copied to,
//! that it reaches them together, and that a session nobody tunes is the session that started.
//!
//! The law is checked against [`impedance_torques`] itself, run on the recorded state and the
//! recorded goal with the values the crossing should have reached by that cycle. That oracle is
//! exact -- the same function on the same inputs -- as long as the joint envelope contributes
//! nothing, which every case here asserts rather than assumes. The arm is held where it started
//! and the goal walks away from it to the leash, which is what leaves a standing position error
//! and a goal velocity for the gains to act on, and keeps every joint far from the limits and at
//! rest, which is what keeps the envelope out of the oracle.
//!
//! What the budget reaches instead of the law -- the generator's limits, and the landing dwell
//! derived from its jerks -- is in [`budget`], on a session whose arm follows the goal.

mod budget;

use std::sync::{Arc, Mutex};

use super::super::*;
use super::bench_ik::allocations;
use super::torque::{cartesian_loop, PoseLoop};
use super::{Arm, READY};
use crate::model::{Frame, Model};
use crate::rate_limiting::DELTA_T;

/// A cycle's state and what the law made of it.
type Record = (RobotState, CartesianSent);

/// One operator's move on a [`LiveTuning`].
type Change = fn(&mut LiveTuning);

fn options() -> TargetControlOptions {
    TargetControlOptions::default()
}

fn seed(impedance: &ImpedanceOptions) -> LiveTuning {
    cartesian::seed(&options(), impedance).expect("the preset's gains are a scaled CARTESIAN")
}

/// [`seed`] with `change` applied: an operator's move, as a whole target.
fn tuned(impedance: &ImpedanceOptions, change: impl FnOnce(&mut LiveTuning)) -> LiveTuning {
    let mut target = seed(impedance);
    change(&mut target);
    target
}

/// A retune, and the cycle it is published before: at 0 it arrives before the session's first
/// cycle, where the tracker has yet to build the anchor that holds its copies.
struct Retune {
    target: LiveTuning,
    at: usize,
}

/// A Cartesian session against a held arm, with a 5 cm step in x from the second cycle and
/// `retune` published into the tuning slot before the cycle it names; `inspect` runs after
/// every cycle.
fn session_with(
    impedance: ImpedanceOptions,
    retune: Option<Retune>,
    cycles: usize,
    mut inspect: impl FnMut(usize, &PoseLoop),
) -> Vec<Record> {
    let records: Arc<Mutex<Vec<Record>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&records);
    let options = options().with_observer(move |state: &RobotState, sent: &CartesianSent| {
        sink.lock().unwrap().push((*state, *sent));
    });
    let (mut torque, shared, _first): (PoseLoop, _, _) = cartesian_loop(options, impedance);
    let publish = |cycle: usize| {
        if let Some(retune) = &retune {
            if retune.at == cycle {
                shared.tuning.publish(retune.target.to_words());
            }
        }
    };
    let arm = Arm::at(READY);
    publish(0);
    torque.cycle(&arm.state);
    inspect(0, &torque);
    let mut step = [0.0; 7];
    assert!(shared.slot.load(&mut step), "the first cycle anchors");
    step[0] += 0.05;
    shared.slot.publish(step);
    for cycle in 1..cycles {
        publish(cycle);
        torque.cycle(&arm.state);
        inspect(cycle, &torque);
    }
    let records = records.lock().unwrap().clone();
    assert_eq!(records.len(), cycles);
    records
}

fn session(impedance: ImpedanceOptions, retune: Option<Retune>, cycles: usize) -> Vec<Record> {
    session_with(impedance, retune, cycles, |_, _| {})
}

/// `target`, published before the session's first cycle.
fn at_once(target: LiveTuning) -> Option<Retune> {
    Some(Retune { target, at: 0 })
}

/// `impedance` with the law's live fields as `tuning` has them: what the loop should be running
/// on the cycle its crossing reached `tuning`.
fn law_of(impedance: &ImpedanceOptions, tuning: &LiveTuning) -> ImpedanceOptions {
    ImpedanceOptions {
        gains: tuning.gains(),
        ik: IkOptions {
            damping: tuning.ik_damping,
            nullspace_gain: tuning.ik_nullspace_gain,
            ..impedance.ik
        },
        velocity_feedforward: true,
        velocity_feedforward_gain: tuning.velocity_feedforward_gain,
        velocity_feedforward_cutoff: tuning.velocity_feedforward_cutoff,
        ..*impedance
    }
}

/// Every cycle's torque against [`impedance_torques`] run with the values the crossing from
/// `impedance` to `target` should have reached by then. Returns how far the recorded torques
/// stood from the ones the session's own options would have made, so a caller can show that the
/// retune did reach the law rather than agreeing with it by accident.
fn assert_law_follows_the_crossing(
    records: &[Record],
    impedance: &ImpedanceOptions,
    retune: &Option<Retune>,
) -> f64 {
    let model = Model::native_fer();
    let (mut current, gains) = (seed(impedance), LiveTuning::slew_gains(DELTA_T));
    let mut apart = 0.0f64;
    for (cycle, (state, sent)) in records.iter().enumerate() {
        if let Some(retune) = retune {
            if cycle >= retune.at {
                current.slew_toward(&retune.target, &gains);
            }
        }
        // The oracle is the law alone, so it holds only where the envelope is out of the way.
        assert_eq!(
            sent.tau_envelope, [0.0; 7],
            "cycle {cycle}: the barrier bit"
        );
        assert_eq!(sent.tau_position, [0.0; 7], "cycle {cycle}: the spring bit");
        let law = |options: &ImpedanceOptions| {
            impedance_torques(
                options,
                &model.zero_jacobian(Frame::EndEffector, state),
                &sent.q_goal,
                &sent.dq_goal,
                &state.q,
                &state.dq,
                &model.coriolis(state),
            )
        };
        // Compared by value, not by bits: a torque of exactly zero comes back as `-0.0` from a
        // law with the feedforward switched off and `0.0` from the same law weighing it at zero.
        assert_eq!(
            sent.tau,
            law(&law_of(impedance, &current)),
            "cycle {cycle}: the law did not run on the crossing's values"
        );
        let untuned = law(impedance);
        apart = (0..7).fold(apart, |a, i| a.max((sent.tau[i] - untuned[i]).abs()));
    }
    apart
}

#[test]
fn each_law_field_reaches_the_law_and_moves_it() {
    let impedance = ImpedanceOptions::cartesian();
    let changes: [(&str, Change); 4] = [
        ("joint_stiffness", |t| t.joint_stiffness = [200.0; 7]),
        ("joint_damping", |t| t.joint_damping = [20.0; 7]),
        ("cartesian_stiffness", |t| t.cartesian_stiffness = 1500.0),
        ("velocity_feedforward_gain", |t| {
            t.velocity_feedforward_gain = 0.0
        }),
    ];
    for (name, change) in changes {
        let target = tuned(&impedance, change);
        let retune = at_once(target);
        let records = session(impedance, at_once(target), 300);
        let apart = assert_law_follows_the_crossing(&records, &impedance, &retune);
        // And it did something: a law that dropped this field would sit on the seed's torques.
        assert!(apart > 1e-3, "{name} moved the torque by only {apart} Nm");
    }
}

#[test]
fn the_feedforward_switch_is_carried_by_the_weight() {
    // Started off, the seed's weight is zero, so the apply's unconditional `true` is the same
    // law -- which is what lets a session started with the switch off be crossed to a weight.
    let off = ImpedanceOptions::cartesian().with_velocity_feedforward(false);
    assert_eq!(seed(&off).velocity_feedforward_gain, 0.0);
    let untouched = session(off, None, 50);
    assert_law_follows_the_crossing(&untouched, &off, &None);
    let target = tuned(&off, |t| t.velocity_feedforward_gain = 1.0);
    let crossed = session(off, at_once(target), 300);
    let apart = assert_law_follows_the_crossing(&crossed, &off, &at_once(target));
    assert!(apart > 1e-3, "the weight did not reach the law: {apart} Nm");
}

#[test]
fn the_shared_options_are_the_values_in_force() {
    // Every live field at once, so the record the loop keeps is checked whole: the law reads it
    // afresh every cycle, and what it does not read -- the feedforward's cutoff, the nullspace
    // gain -- is what a later reader would take for the truth.
    let impedance = ImpedanceOptions::cartesian();
    let target = tuned(&impedance, |t| {
        t.joint_stiffness = [200.0; 7];
        t.joint_damping = [20.0; 7];
        t.cartesian_stiffness = 1500.0;
        t.ik_damping = 0.5;
        t.ik_nullspace_gain = 0.0;
        t.velocity_feedforward_gain = 0.25;
        t.velocity_feedforward_cutoff = 5.0;
    });
    let (mut current, gains) = (seed(&impedance), LiveTuning::slew_gains(DELTA_T));
    let records = session_with(impedance, at_once(target), 300, |cycle, torque| {
        current.slew_toward(&target, &gains);
        assert_eq!(
            torque.tuned().0,
            law_of(&impedance, &current),
            "cycle {cycle}"
        );
    });
    let apart = assert_law_follows_the_crossing(&records, &impedance, &at_once(target));
    assert!(apart > 1e-3, "nothing reached the law: {apart} Nm");
}

#[test]
fn the_damping_the_projector_reads_is_the_damping_the_solver_runs() {
    let impedance = ImpedanceOptions::cartesian().with_project_joint_gains(true);
    let target = tuned(&impedance, |t| {
        t.ik_damping = 0.5;
        t.ik_nullspace_gain = 0.0;
    });
    let (mut current, gains) = (seed(&impedance), LiveTuning::slew_gains(DELTA_T));
    let records = session_with(impedance, at_once(target), 300, |cycle, torque| {
        current.slew_toward(&target, &gains);
        let (law, solver) = torque.tuned();
        let solver = solver.expect("the first cycle anchors the solver");
        assert_eq!(law.ik.damping, current.ik_damping, "cycle {cycle}");
        assert_eq!(solver.damping, law.ik.damping, "cycle {cycle}: two lambdas");
        assert_eq!(solver.nullspace_gain, current.ik_nullspace_gain);
        // The retune wrote those two options of the solver and nothing else it holds.
        assert_eq!(solver.iterations, impedance.ik.iterations);
        assert_eq!(solver.rotation_weight, impedance.ik.rotation_weight);
        assert_eq!(solver.tolerance, impedance.ik.tolerance);
    });
    // The projector is the law's only reader of lambda, so this is the shared copy's own check.
    let apart = assert_law_follows_the_crossing(&records, &impedance, &at_once(target));
    assert!(
        apart > 1e-6,
        "lambda did not reach the projector: {apart} Nm"
    );
}

#[test]
fn a_retune_before_the_first_cycle_reaches_the_anchor_that_cycle_builds() {
    let impedance = ImpedanceOptions::cartesian();
    let target = tuned(&impedance, |t| t.ik_damping = 0.5);
    let mut current = seed(&impedance);
    current.slew_toward(&target, &LiveTuning::slew_gains(DELTA_T));
    session_with(impedance, at_once(target), 1, |_, torque| {
        let (law, solver) = torque.tuned();
        // One cycle of the crossing, in the shared options and in the anchor the same cycle
        // built after them: the write into the tracker's template is what carries it there.
        assert_eq!(law.ik.damping, current.ik_damping);
        assert_eq!(solver.expect("anchored").damping, current.ik_damping);
    });
    assert!(current.ik_damping > impedance.ik.damping);
    assert!(current.ik_damping < target.ik_damping);
}

#[test]
fn the_feedforward_cutoff_is_in_force_on_the_cycle_it_arrives() {
    let impedance = ImpedanceOptions::cartesian();
    let cutoff = 5.0;
    let target = tuned(&impedance, |t| t.velocity_feedforward_cutoff = cutoff);
    // Before the first cycle, where the tracker's template is what carries it into the anchor
    // the same cycle builds, and in the middle of a session, where the anchor's own copy is.
    for at in [0, 40] {
        let records = session(impedance, Some(Retune { target, at }), 200);
        assert_filtered_from(&records, at, cutoff);
    }
    // Untuned, the cutoff is the filter's off switch and `dq_goal` is the raw difference: what
    // the retune changed is real.
    let unfiltered = session(impedance, None, 200);
    assert_filtered_from(&unfiltered, 200, cutoff);
    let filtered = session(impedance, at_once(target), 200);
    let apart = (2..200).fold(0.0f64, |a, cycle| {
        let (l, r) = (&filtered[cycle].1, &unfiltered[cycle].1);
        (0..7).fold(a, |a, i| a.max((l.dq_goal[i] - r.dq_goal[i]).abs()))
    });
    assert!(apart > 1e-3, "the cutoff changed nothing: {apart} rad/s");
}

/// `dq_goal` is the raw finite difference of the goal until cycle `at`, and its low-pass at
/// `cutoff` -- over the previous cycle's own output, the filter's only state -- from then on.
/// A cutoff that crossed instead of stepping would not hold this on the cycle it arrived.
fn assert_filtered_from(records: &[Record], at: usize, cutoff: f64) {
    let gain = crate::lowpass_filter::gain(DELTA_T, cutoff);
    for cycle in 2..records.len() {
        let (previous, sent) = (&records[cycle - 1].1, &records[cycle].1);
        let raw: [f64; 7] =
            std::array::from_fn(|i| (sent.q_goal[i] - previous.q_goal[i]) / DELTA_T);
        let expected: [f64; 7] = std::array::from_fn(|i| {
            if cycle >= at {
                gain * raw[i] + (1.0 - gain) * previous.dq_goal[i]
            } else {
                raw[i]
            }
        });
        for (i, expected) in expected.into_iter().enumerate() {
            assert!(
                (sent.dq_goal[i] - expected).abs() < 1e-12,
                "cycle {cycle}, joint {i}: {} against {expected}",
                sent.dq_goal[i],
            );
        }
    }
}

#[test]
fn a_session_nobody_tunes_is_the_session_that_was_started() {
    let impedance = ImpedanceOptions::cartesian();
    let untouched = session(impedance, None, 300);
    // The same values published: the loop reads them, finds nothing to cross to and applies
    // nothing. Bit for bit, the goal the IK found included.
    let seeded = session(impedance, at_once(seed(&impedance)), 300);
    for cycle in 0..300 {
        assert_eq!(
            seeded[cycle].1.tau.map(f64::to_bits),
            untouched[cycle].1.tau.map(f64::to_bits),
            "cycle {cycle}"
        );
        assert_eq!(
            seeded[cycle].1.q_goal.map(f64::to_bits),
            untouched[cycle].1.q_goal.map(f64::to_bits),
            "cycle {cycle}"
        );
    }
    // Not trivially equal: this session did work, and the law it ran is not all zeros.
    let moved = (0..7).fold(0.0f64, |a, i| {
        a.max((untouched[299].1.q_goal[i] - untouched[0].1.q_goal[i]).abs())
    });
    assert!(moved > 1e-3, "the goal did not move: {moved} rad");
    let torque = untouched[299]
        .1
        .tau
        .iter()
        .fold(0.0f64, |a, t| a.max(t.abs()));
    assert!(torque > 1.0, "the law made no torque: {torque} Nm");
}

#[test]
fn a_cycle_that_retunes_allocates_nothing() {
    let impedance = ImpedanceOptions::cartesian();
    let target = tuned(&impedance, |t| t.joint_stiffness = [200.0; 7]);
    let (mut torque, shared, _first): (PoseLoop, _, _) = cartesian_loop(options(), impedance);
    let arm = Arm::at(READY);
    torque.cycle(&arm.state);
    let mut step = [0.0; 7];
    assert!(shared.slot.load(&mut step));
    step[0] += 0.05;
    shared.slot.publish(step);
    shared.tuning.publish(target.to_words());
    // The solver's own first passes are warmed first; what is measured is the retune on a loop
    // that is otherwise running.
    for _ in 0..50 {
        torque.cycle(&arm.state);
    }
    let before = allocations();
    for _ in 0..200 {
        std::hint::black_box(torque.cycle(&arm.state));
    }
    assert_eq!(allocations(), before, "a retuning cycle allocated");
}

/// The joint interface has no live tuning: its slot is never seeded, so its loop reads nothing
/// per cycle and its handle has nothing to offer.
#[test]
fn a_joint_session_has_no_tuning_slot_to_read() {
    let shared = Arc::new(Shared::<7>::default());
    assert_eq!(shared.tuning.sequence(), 0);
    let mut words = [1.0; LiveTuning::WORDS];
    assert!(!shared.tuning.load(&mut words));
    assert_eq!(words, [1.0; LiveTuning::WORDS], "the load wrote");
}
