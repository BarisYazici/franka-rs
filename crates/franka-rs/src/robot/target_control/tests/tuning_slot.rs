//! The tuning slot and the setter that writes it: that a whole [`LiveTuning`] crosses the
//! seqlock atomically, that a session without one says so, and that nothing reaches the slot
//! except through [`LiveTuning::apply_update`].

use std::sync::atomic::Ordering;
use std::sync::Arc;

use super::super::*;
use super::tuning::{bits, probe, shipped};
use crate::error::FrankaError;

/// A handle on a running session whose slot holds `seed`, as `cartesian::start` leaves it.
fn running(seed: Option<LiveTuning>) -> Handle<7> {
    let shared = Arc::new(Shared::<7>::default());
    if let Some(seed) = seed {
        shared.tuning.publish(seed.to_words());
    }
    shared.running.store(true, Ordering::SeqCst);
    Handle {
        shared,
        thread: None,
    }
}

/// An update carrying what `carry` sets, as a caller outside the crate has to build one.
fn update(carry: impl FnOnce(&mut TuningUpdate)) -> TuningUpdate {
    let mut update = TuningUpdate::default();
    carry(&mut update);
    update
}

fn is_invalid_operation<T>(result: FrankaResult<T>, needle: &str) -> bool {
    matches!(result, Err(FrankaError::InvalidOperation(m)) if m.contains(needle))
}

#[test]
fn a_whole_tuning_crosses_the_slot_or_none_of_it_does() {
    // Every published set is `k` in word 0 and `k * (1 + word)` in the rest; a torn read would
    // show one word from one publish beside another word from the next.
    let slot = Arc::new(TargetSlot::<{ LiveTuning::WORDS }>::default());
    let writer = {
        let slot = Arc::clone(&slot);
        std::thread::spawn(move || {
            for k in 1..200_000u32 {
                let k = f64::from(k);
                slot.publish(std::array::from_fn(|word| k * (1 + word) as f64));
            }
        })
    };
    let mut words = [0.0; LiveTuning::WORDS];
    let (mut loads, mut torn) = (0u64, 0u64);
    while !writer.is_finished() {
        if slot.load(&mut words) {
            loads += 1;
            let tuning = LiveTuning::from_words(&words);
            let k = tuning.joint_stiffness[0];
            for (word, value) in tuning.to_words().into_iter().enumerate() {
                assert_eq!(value, k * (1 + word) as f64, "torn read at word {word}");
            }
        } else {
            torn += 1;
        }
    }
    writer.join().unwrap();
    assert!(
        loads > 0,
        "the reader never saw a consistent set ({torn} torn)"
    );
    assert!(slot.load(&mut words));
    assert_eq!(LiveTuning::from_words(&words).joint_stiffness[0], 199_999.0);
}

#[test]
fn a_session_that_was_never_seeded_has_no_tuning_to_read_or_write() {
    let handle = running(None);
    assert!(is_invalid_operation(handle.tuning(), NO_TUNING_MESSAGE));
    assert!(is_invalid_operation(
        handle.tune(&TuningUpdate::default()),
        NO_TUNING_MESSAGE
    ));
    assert_eq!(handle.shared.tuning.sequence(), 0, "the refusal wrote");
}

#[test]
fn the_seed_reads_back_and_a_write_advances_the_sequence_by_two() {
    let handle = running(Some(shipped()));
    assert_eq!(handle.shared.tuning.sequence(), 2);
    assert_eq!(bits(&handle.tuning().unwrap()), bits(&shipped()));
    let update = update(|u| u.ik_damping = Some(0.2));
    assert!(handle.tune(&update).unwrap().is_empty());
    assert_eq!(handle.shared.tuning.sequence(), 4);
    let read = handle.tuning().unwrap();
    assert_eq!(read.ik_damping, 0.2);
    assert_eq!(
        bits(&LiveTuning {
            ik_damping: shipped().ik_damping,
            ..read
        }),
        bits(&shipped()),
        "a one-field update moved another field"
    );
}

#[test]
fn the_setter_is_the_gate_and_reports_what_it_clamped() {
    let handle = running(Some(probe()));
    // Past the ceiling: clamped, in force at the ceiling, and the bound comes back.
    let raised = update(|u| u.ik_nullspace_gain = Some(1e9));
    let clamped = handle.tune(&raised).unwrap();
    assert_eq!(clamped, vec![&LiveTuning::BOUNDS[16]]);
    assert_eq!(
        handle.tuning().unwrap().ik_nullspace_gain,
        LiveTuning::BOUNDS[16].max
    );
    // Refused rather than clamped, and nothing is written.
    let published = bits(&handle.tuning().unwrap());
    let sequence = handle.shared.tuning.sequence();
    let bad = update(|u| u.ik_damping = Some(0.0));
    assert!(matches!(
        handle.tune(&bad),
        Err(FrankaError::InvalidArgument(_))
    ));
    assert_eq!(bits(&handle.tuning().unwrap()), published);
    assert_eq!(handle.shared.tuning.sequence(), sequence);
}

#[test]
fn a_setter_on_a_loop_that_has_ended_refuses_and_leaves_the_slot_alone() {
    let handle = running(Some(shipped()));
    handle.shared.running.store(false, Ordering::SeqCst);
    let update = update(|u| u.ik_damping = Some(0.2));
    assert!(is_invalid_operation(handle.tune(&update), ENDED_MESSAGE));
    assert_eq!(bits(&handle.tuning().unwrap()), bits(&shipped()));
    assert_eq!(handle.shared.tuning.sequence(), 2);
}

#[test]
fn a_session_is_seeded_with_the_options_it_was_started_with() {
    let options = TargetControlOptions::default();
    let impedance = ImpedanceOptions::cartesian();
    let seed = cartesian::seed(&options, &impedance).unwrap();
    assert_eq!(bits(&seed), bits(&shipped()));
    // The seed is what the law is already running, so it puts nothing into effect: the gains
    // it stands for are the ones the session was built with, bit for bit.
    assert_eq!(
        seed.gains().cartesian_stiffness.map(f64::to_bits),
        impedance.gains.cartesian_stiffness.map(f64::to_bits)
    );
    assert_eq!(
        seed.gains().cartesian_damping.map(f64::to_bits),
        impedance.gains.cartesian_damping.map(f64::to_bits)
    );
    assert_eq!(seed.gains(), impedance.gains);
    // ... and so does a joint session's, whose Cartesian gains are zero.
    let joint = ImpedanceOptions::joint();
    assert_eq!(
        LiveTuning::from_options(&joint, options.limits, options.rotation_limits).gains(),
        joint.gains
    );
    assert!(cartesian::seed(&options, &joint).is_ok());
}

#[test]
fn every_stiffness_the_one_slider_can_hold_seeds_a_session() {
    // The faithfulness check must not refuse a session someone configured through the rule it
    // checks: `gains()` reads the scalar back off axis 0 and rescales from it, and that has to
    // land on the same twelve words for every stiffness a caller can ask for -- the node's
    // `cartesian_gains` is this function, so a refusal here is a refusal on the arm.
    let options = TargetControlOptions::default();
    let bound = &LiveTuning::BOUNDS[14];
    let mut checked = 0;
    let mut stiffness = bound.min;
    while stiffness <= bound.max {
        let impedance =
            ImpedanceOptions::cartesian().with_gains(ImpedanceGains::scaled_cartesian(stiffness));
        let seed = cartesian::seed(&options, &impedance)
            .unwrap_or_else(|why| panic!("{stiffness} N/m was refused: {why}"));
        assert_eq!(seed.gains(), impedance.gains);
        // The scalar read back off axis 0 is the one asked for to within a rounding -- `750 *
        // (s / 750)` is not always `s` -- and what the check needs is not that it is `s` but
        // that rescaling from it lands on the same twelve words, which the line above is.
        assert!(
            (seed.cartesian_stiffness - stiffness).abs() <= f64::EPSILON * stiffness,
            "{} against {stiffness}",
            seed.cartesian_stiffness
        );
        checked += 1;
        stiffness += 0.37;
    }
    assert!(checked > 7000, "only {checked} stiffnesses were checked");
}

#[test]
fn a_gains_set_the_one_stiffness_cannot_rebuild_is_refused_its_tuning() {
    let options = TargetControlOptions::default();
    // DROID is the crate's parity preset and the one shipped gains set `scaled_cartesian` does
    // not reproduce: its translational damping is 37 where the scaled preset's is 50 and 90.
    let droid = ImpedanceOptions::cartesian().with_gains(ImpedanceGains::DROID);
    let derived = LiveTuning::from_options(&droid, options.limits, options.rotation_limits).gains();
    assert_eq!(derived.cartesian_damping, [50.0, 50.0, 90.0, 2.0, 2.0, 2.0]);
    assert_eq!(
        droid.gains.cartesian_damping,
        [37.0, 37.0, 37.0, 2.0, 2.0, 2.0]
    );
    assert_eq!(derived.cartesian_stiffness, droid.gains.cartesian_stiffness);
    // So the session is left without live tuning rather than having those six words rewritten
    // in one cycle by the first retune of an unrelated field -- a step the crossing cannot
    // soften, because the stiffness it watches never moved.
    assert_eq!(
        cartesian::seed(&options, &droid),
        Err(DERIVED_GAINS_MESSAGE),
        "a gains set the seed cannot represent was seeded"
    );
    // And the handle says so, with the reason, rather than the backend's.
    let shared = Arc::new(Shared::<7>::default());
    shared.running.store(true, Ordering::SeqCst);
    let control = CartesianTargetControl::on(Arc::clone(&shared), Err(DERIVED_GAINS_MESSAGE));
    assert!(is_invalid_operation(
        control.tuning(),
        DERIVED_GAINS_MESSAGE
    ));
    assert!(is_invalid_operation(
        control.tune(&update(|u| u.ik_damping = Some(0.2))),
        DERIVED_GAINS_MESSAGE
    ));
    assert_eq!(shared.tuning.sequence(), 0, "the refusal wrote");
    // A session that is representable answers through the same handle.
    let seeded = Arc::new(Shared::<7>::default());
    seeded.tuning.publish(shipped().to_words());
    seeded.running.store(true, Ordering::SeqCst);
    let control = CartesianTargetControl::on(seeded, Ok(shipped()));
    assert_eq!(bits(&control.tuning().unwrap()), bits(&shipped()));
}

#[test]
fn a_running_session_takes_a_budget_through_the_gate_every_other_field_goes_through() {
    // A budget was refused here while nothing took its limits from the loop, because answering
    // `Ok` to a request to slow an arm down that the arm will not act on is worse than
    // refusing it. The generator takes them now, so it is a field like any other: clamped to
    // `BOUNDS`, echoed back, and what an update does not carry it does not touch.
    let handle = running(Some(shipped()));
    let rotation = handle.tuning().unwrap().rotation_budget;
    assert!(handle
        .tune(&update(|u| u.budget = Some([0.2, 0.4, 10.0])))
        .unwrap()
        .is_empty());
    let tuning = handle.tuning().unwrap();
    assert_eq!(tuning.budget, [0.2, 0.4, 10.0]);
    assert_eq!(
        tuning.rotation_budget, rotation,
        "an uncarried budget moved"
    );
    assert_eq!(handle.shared.tuning.sequence(), 4);

    // Past its ceiling it snaps, and the bound that did it comes back, which is how an
    // operator learns the limit is there.
    let clamped = handle
        .tune(&update(|u| u.budget = Some([5.0, 0.4, 10.0])))
        .unwrap();
    assert_eq!(
        clamped
            .iter()
            .map(|b| (b.name, b.index))
            .collect::<Vec<_>>(),
        vec![("budget", Some(0))]
    );
    assert_eq!(
        handle.tuning().unwrap().budget[0],
        LiveTuning::BOUNDS[19].max
    );

    // And a value that is not finite is refused with nothing written, budget or not.
    let refused = handle.tune(&update(|u| u.rotation_budget = Some([f64::NAN, 1.0, 20.0])));
    assert!(
        matches!(&refused, Err(FrankaError::InvalidArgument(m)) if m.contains("rotation_budget[0]")),
        "{refused:?}"
    );
    assert_eq!(handle.tuning().unwrap().rotation_budget, rotation);
    assert_eq!(handle.shared.tuning.sequence(), 6, "the refusal wrote");
}

#[test]
fn the_feedforward_switch_is_seeded_as_the_weight_that_means_the_same() {
    let options = TargetControlOptions::default();
    let off = ImpedanceOptions::cartesian()
        .with_velocity_feedforward(false)
        .with_velocity_feedforward_gain(1.0);
    let seed = LiveTuning::from_options(&off, options.limits, options.rotation_limits);
    assert_eq!(
        seed.velocity_feedforward_gain, 0.0,
        "the switch was dropped"
    );
    let on = ImpedanceOptions::cartesian()
        .with_velocity_feedforward(true)
        .with_velocity_feedforward_gain(0.4);
    assert_eq!(
        LiveTuning::from_options(&on, options.limits, options.rotation_limits)
            .velocity_feedforward_gain,
        0.4
    );
    // The default is off, so a default session's live weight starts at zero.
    let default = ImpedanceOptions::cartesian();
    assert_eq!(
        LiveTuning::from_options(&default, options.limits, options.rotation_limits)
            .velocity_feedforward_gain,
        0.0
    );
}
