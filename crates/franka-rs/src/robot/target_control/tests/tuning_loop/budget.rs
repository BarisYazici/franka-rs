//! The budget's end of a retune: that it reaches the generator at all, that raising it is a
//! step and lowering it a ramp the generator can follow rather than the impulse its clamp
//! would otherwise make of it, and that the one value derived from it -- the landing dwell --
//! is derived again only when it has to be.

use super::super::super::*;
use super::super::torque::cartesian_loop;
use super::super::{recording, Arm, READY};
use super::{at_once, options, seed, session_with, tuned, PoseLoop, Retune};
use crate::otg::OtgLimits;
use crate::rate_limiting::DELTA_T;

/// The per-axis limits `tuning`'s budget stands for: what the loop should be handing the
/// generator, through the same conversion the session was built with.
fn axes_of(tuning: &LiveTuning) -> [OtgLimits; 6] {
    let (budget, rotation) = tuning.budgets();
    cartesian::axis_limits(budget, rotation)
}

/// `limits` by bit pattern, for the claim that a retune left them alone rather than rewriting
/// them with something that compares equal.
fn bits(limits: &[OtgLimits; 6]) -> [[u64; 3]; 6] {
    limits.map(|l| [l.max_velocity, l.max_acceleration, l.max_jerk].map(f64::to_bits))
}

#[test]
fn the_per_axis_conversion_is_one_factor_per_group_and_these_are_the_factors() {
    // Every expectation in this file goes through `axis_limits`, so nothing else here can tell
    // whether `axis_limits` is right. These are its numbers, written out rather than computed
    // from it: a norm divided among three axes, and the backstop's 0.99 on the rotational rows.
    let axes = cartesian::axis_limits(options().limits, options().rotation_limits);
    let expected = [
        [
            0.173_205_080_756_887_76,
            0.288_675_134_594_812_9,
            11.547_005_383_792_516,
        ],
        [
            0.285_788_383_248_864_8,
            0.571_576_766_497_729_6,
            11.431_535_329_954_592,
        ],
    ];
    for axis in 0..6 {
        let want = expected[usize::from(axis >= 3)];
        let got = [
            axes[axis].max_velocity,
            axes[axis].max_acceleration,
            axes[axis].max_jerk,
        ];
        for order in 0..3 {
            assert!(
                (got[order] - want[order]).abs() < 1e-15,
                "axis {axis} order {order}: {} against {}",
                got[order],
                want[order]
            );
        }
    }
    // And the property the whole norm-space gate rests on: within a group the three orders are
    // scaled by *one* factor, so gating the norm and scaling afterwards is the same walk as
    // scaling and then gating. A factor that stopped being shared would break that silently.
    for (norm, axis) in [(options().limits, 0), (options().rotation_limits, 3)] {
        let factor = axes[axis].max_velocity / norm.max_velocity;
        assert!(
            (axes[axis].max_acceleration / norm.max_acceleration - factor).abs() < 1e-15
                && (axes[axis].max_jerk / norm.max_jerk - factor).abs() < 1e-15,
            "axis {axis} scales its three orders by different factors"
        );
    }
}

#[test]
fn a_retune_that_moves_no_budget_leaves_the_generators_limits_bit_for_bit() {
    // The apply rebuilds the generator's limits from the whole budget whenever *any* field
    // moves, the way it rebuilds the twelve Cartesian gains from one stiffness -- so a session
    // whose budget nobody touched has to come through the rebuild untouched.
    let impedance = ImpedanceOptions::cartesian();
    let started = bits(&axes_of(&seed(&impedance)));
    let law = tuned(&impedance, |t| t.joint_stiffness = [200.0; 7]);
    session_with(impedance, at_once(law), 300, |cycle, torque| {
        assert_eq!(bits(&torque.generator().1), started, "cycle {cycle}");
    });
}

#[test]
fn a_raised_budget_reaches_the_generator_on_the_cycle_it_arrives() {
    let impedance = ImpedanceOptions::cartesian();
    let target = tuned(&impedance, |t| t.budget = [0.6, 1.0, 40.0]);
    let (started, raised) = (axes_of(&seed(&impedance)), axes_of(&target));
    assert_ne!(started, raised, "the retune asked for the budget in force");
    session_with(
        impedance,
        Some(Retune { target, at: 20 }),
        40,
        |cycle, torque| {
            let (_, limits) = torque.generator();
            let expected = if cycle >= 20 { raised } else { started };
            assert_eq!(limits, expected, "cycle {cycle}");
        },
    );
}

/// A Cartesian session under `budget` whose arm follows the goal one cycle behind, driven 40 cm
/// along x so the generator gets up to speed and stays there; `retune` is published before the
/// cycle it names, and `watch` sees the loop after every cycle.
fn driven_session(
    budget: OtgLimits,
    retune: Retune,
    cycles: usize,
    mut watch: impl FnMut(usize, &PoseLoop),
) {
    let (records, observer) = recording::<CartesianSent>();
    let (mut torque, shared, _first): (PoseLoop, _, _) = cartesian_loop(
        options().with_limits(budget).with_observer(observer),
        ImpedanceOptions::cartesian(),
    );
    let mut arm = Arm::at(READY);
    torque.cycle(&arm.state);
    let mut step = [0.0; 7];
    assert!(shared.slot.load(&mut step), "the first cycle anchors");
    step[0] += 0.4;
    shared.slot.publish(step);
    for cycle in 1..cycles {
        if retune.at == cycle {
            shared.tuning.publish(retune.target.to_words());
        }
        let goal = {
            let records = records.lock().unwrap();
            records[records.len() - 1].q_goal
        };
        arm.follow(&goal);
        torque.cycle(&arm.state);
        watch(cycle, &torque);
    }
}

/// The largest one-cycle change of the generator's stored velocity on any axis over a session
/// under `budget` retuned to `target` at cycle `at`, and what the generator was doing on axis 0
/// on that cycle. Every cycle is held to the invariant as it goes.
///
/// The invariant is `|dv| <= (a_before + a_after) * DELTA_T`, two terms in the same cycle:
///
/// * the **clamp**. A narrowed budget truncates the stored velocity at the re-anchoring, by at
///   most what the crossing took off the limit -- one cycle of the acceleration in force at the
///   top of the cycle, `a_before`, since that is the rate the gate descends at and the limit the
///   previous cycle's `follow` had already clamped the velocity into.
/// * the **plan's own step**, at most `a_after * DELTA_T`, under the limits this cycle runs.
///
/// The two trade off rather than adding: a cycle already braking at `a_max` has no headroom left
/// for the clamp to take, and a cycle at cruise where the clamp takes its full share has a plan
/// that has barely begun to brake. The sum approaches twice one cycle's acceleration only where
/// the plan can swing from no braking to full braking within a cycle, which needs
/// `j * DELTA_T >= a_max` -- so it is a *high*-jerk budget that reaches it, not a low-jerk one.
/// `a_before` and `a_after` differ whenever the acceleration word itself moved.
fn worst_velocity_step(
    budget: OtgLimits,
    target: LiveTuning,
    at: usize,
    cycles: usize,
) -> (f64, f64) {
    let (mut previous, mut worst, mut at_retune) = (None, 0.0f64, 0.0f64);
    driven_session(budget, Retune { target, at }, cycles, |cycle, torque| {
        let (velocity, limits) = torque.generator();
        if cycle == at {
            at_retune = velocity[0].abs();
        }
        if let Some((was, before)) = previous {
            let (was, before): ([f64; 6], [OtgLimits; 6]) = (was, before);
            for axis in 0..6 {
                let step = (velocity[axis] - was[axis]).abs();
                let allowed =
                    (before[axis].max_acceleration + limits[axis].max_acceleration) * DELTA_T;
                assert!(
                    step <= allowed + 1e-12,
                    "cycle {cycle}, axis {axis}: the velocity stepped {step}, over {allowed}"
                );
                worst = worst.max(step);
            }
        }
        previous = Some((velocity, limits));
    });
    (worst, at_retune)
}

#[test]
fn a_lowered_budget_is_a_ramp_the_generator_follows_and_not_an_impulse() {
    let impedance = ImpedanceOptions::cartesian();
    let budget = options().limits;
    let target = tuned(&impedance, |t| t.budget = [0.08, 0.3, 5.0]);
    let (started, lowered) = (axes_of(&seed(&impedance)), axes_of(&target));
    let mut landed = None;
    driven_session(budget, Retune { target, at: 700 }, 2000, |cycle, torque| {
        let (_, limits) = torque.generator();
        if cycle > 700 && landed.is_none() && limits[0] == lowered[0] {
            landed = Some(cycle);
        }
    });
    // It took a ramp to get there, not a cycle.
    let landed = landed.expect("the lowered budget never came into force");
    assert!(
        landed - 700 > 400,
        "the budget landed {} cycles in",
        landed - 700
    );

    let (worst, at_retune) = worst_velocity_step(budget, target, 700, 2000);
    // The cut is the case that matters: the generator was already running faster than the
    // budget it was asked to come down to, which is what the clamp would truncate.
    assert!(
        at_retune > 1.4 * lowered[0].max_velocity,
        "the generator was doing {at_retune} m/s against a new cap of {}",
        lowered[0].max_velocity
    );
    // At the shipped budget the jerk is far too small to reach the two-term worst case -- one
    // cycle of it is 4 % of the acceleration limit -- so the real number is one cycle of the
    // acceleration and a half-percent, which is what a regression here has to be measured
    // against. The general invariant above is four times looser than this and would not notice
    // the descent rate doubling.
    let one_cycle = started[0].max_acceleration * DELTA_T;
    assert!(
        worst <= 1.05 * one_cycle,
        "the worst cycle stepped {worst} m/s, over {} m/s",
        1.05 * one_cycle
    );
    // Un-gated, the clamp would have taken the whole width of the cut off the stored velocity
    // in one cycle. That is the impulse the ramp exists to spread out.
    let ungated = started[0].max_velocity - lowered[0].max_velocity;
    assert!(
        worst < ungated / 100.0,
        "the worst cycle stepped {worst} m/s against an un-gated {ungated}"
    );
}

#[test]
fn lowering_the_velocity_while_raising_the_acceleration_stays_inside_the_wider_limit() {
    // Two sliders on one panel, and the ordinary way to say "slower but crisper". The
    // acceleration is `StepUpGateDown`, so the raise lands whole on the arriving cycle while
    // the velocity is still descending at the acceleration that was in force before it -- the
    // one term of the invariant is the old limit and the other the new, and they differ by the
    // whole width of the raise. It is not a safety regression: the step stays inside the
    // acceleration the operator has just permitted.
    let impedance = ImpedanceOptions::cartesian();
    let budget = options().limits;
    let target = tuned(&impedance, |t| t.budget = [0.08, 5.0, 20.0]);
    let raised = axes_of(&target)[0].max_acceleration;
    let before = axes_of(&seed(&impedance))[0].max_acceleration;
    assert!(
        raised > 5.0 * before,
        "the raise is not the case under test"
    );
    let (worst, at_retune) = worst_velocity_step(budget, target, 700, 1200);
    assert!(at_retune > 0.0, "the generator never moved");
    // Larger than twice the old limit -- which is what makes this the case the two-term
    // invariant exists for -- and inside the new one.
    assert!(
        worst > 2.0 * before * DELTA_T,
        "the case did not arise: {worst} m/s"
    );
    assert!(
        worst <= (before + raised) * DELTA_T + 1e-12,
        "the velocity stepped {worst} m/s"
    );
}

#[test]
fn the_limits_are_in_force_for_the_step_of_the_cycle_they_arrive_on() {
    // What bounds the clamp is the crossing's rate, not where in the cycle the retune runs --
    // so nothing about the *bound* pins that it runs first. What does is the invariant below:
    // after every cycle the generator's stored state is inside the limits it is holding,
    // because `set_position` clamps into them before the step and `follow` clamps into them
    // after it. Write the limits after the step instead and the arriving cycle leaves a
    // generator running faster than the budget it is holding, for one cycle.
    //
    // A slow budget, so the generator rides its own velocity cap rather than the joint side's:
    // at 0.058 m/s per axis nothing else binds, and only then does one cycle of the descent
    // show up as a clamp at all.
    let budget = OtgLimits {
        max_velocity: 0.1,
        max_acceleration: 0.5,
        max_jerk: 20.0,
    };
    let impedance = ImpedanceOptions::cartesian();
    let mut target = cartesian::seed(&options().with_limits(budget), &impedance)
        .expect("the preset's gains are a scaled CARTESIAN");
    target.budget[0] = 0.05;
    let mut rode_the_cap = false;
    driven_session(budget, Retune { target, at: 400 }, 900, |cycle, torque| {
        let (velocity, limits) = torque.generator();
        for axis in 0..6 {
            assert!(
                velocity[axis].abs() <= limits[axis].max_velocity,
                "cycle {cycle}, axis {axis}: {} m/s against a limit of {}",
                velocity[axis],
                limits[axis].max_velocity
            );
        }
        if cycle == 399 {
            rode_the_cap = velocity[0].abs() == limits[0].max_velocity;
        }
    });
    // Not vacuous: the generator was sitting exactly on the cap when the cut arrived, so the
    // cut had something to truncate.
    assert!(rode_the_cap, "the generator never reached its velocity cap");
}

#[test]
fn the_landing_dwell_is_derived_again_only_on_a_cycle_a_jerk_moved() {
    let impedance = ImpedanceOptions::cartesian();
    let tolerance = options().settle.tolerance;
    let dwell_of = |tuning: &LiveTuning| {
        torque::landing_dwell(&axes_of(tuning).map(|a| a.max_jerk), tolerance)
    };
    let at_start = dwell_of(&seed(&impedance));

    // A retune of the law crosses for hundreds of cycles and moves no jerk, so the dwell is
    // the one the session was built with and its cube roots ran once, at construction.
    let law = tuned(&impedance, |t| t.joint_stiffness = [200.0; 7]);
    session_with(impedance, at_once(law), 300, |cycle, torque| {
        assert_eq!(torque.dwell(), (at_start, 1), "cycle {cycle}");
    });

    // A jerk does move it, on the cycle it arrives -- a jerk steps -- and only then, however
    // long the rest of the retune goes on crossing. The two jerks are left different from each
    // other, so that what the tracker remembers them as has to be the right way round.
    let jerked = tuned(&impedance, |t| {
        t.budget = [0.3, 0.5, 2.0];
        t.joint_stiffness = [200.0; 7];
    });
    assert_ne!(jerked.budget[2], jerked.rotation_budget[2]);
    let after = dwell_of(&jerked);
    assert!(after > at_start, "{after} against {at_start}");
    session_with(
        impedance,
        Some(Retune {
            target: jerked,
            at: 10,
        }),
        300,
        |cycle, torque| {
            let expected = if cycle < 10 {
                (at_start, 1)
            } else {
                (after, 2)
            };
            assert_eq!(torque.dwell(), expected, "cycle {cycle}");
        },
    );
}
