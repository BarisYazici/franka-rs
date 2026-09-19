//! [`LiveTuning::apply_update`]: all or nothing, then clamp, and only what the update carries.
//! The table it clamps against is pinned in [`tuning`](super::tuning).

use super::super::*;
use super::tuning::{bits, floor_of, probe, shipped, whole, with_word};

#[test]
fn the_shipped_values_are_inside_every_bound_and_nothing_clamps_them() {
    let mut tuning = shipped();
    let before = bits(&tuning);
    let carry = whole(&shipped());
    assert!(tuning.apply_update(&carry).unwrap().is_empty());
    assert_eq!(bits(&tuning), before, "an in-range update moved a value");
}

#[test]
fn every_field_clamps_to_its_floor_and_ceiling_exactly() {
    for word in 0..LiveTuning::WORDS {
        let bound = &LiveTuning::BOUNDS[word];
        let low = floor_of(word, &probe().joint_stiffness);
        // Halving stays above zero, which is what a floor above zero needs; a floor of zero is
        // probed from below it instead.
        let below = if low > 0.0 { low / 2.0 } else { low - 1.0 };
        for (outside, expected) in [(below, low), (bound.max * 2.0, bound.max)] {
            let mut tuning = probe();
            let clamped = tuning.apply_update(&with_word(word, outside)).unwrap();
            let words = tuning.to_words();
            assert_eq!(
                words[word].to_bits(),
                expected.to_bits(),
                "{}{:?} did not land on {expected}",
                bound.name,
                bound.index
            );
            assert_eq!(clamped, vec![bound], "{} was not reported", bound.name);
            for other in (0..LiveTuning::WORDS).filter(|o| *o != word) {
                assert_eq!(
                    words[other],
                    probe().to_words()[other],
                    "word {other} moved"
                );
            }
        }
    }
}

#[test]
fn a_value_exactly_on_a_bound_is_kept_and_not_reported() {
    for word in 0..LiveTuning::WORDS {
        let bound = &LiveTuning::BOUNDS[word];
        for edge in [floor_of(word, &probe().joint_stiffness), bound.max] {
            let mut tuning = probe();
            let clamped = tuning.apply_update(&with_word(word, edge)).unwrap();
            assert!(
                clamped.is_empty(),
                "{edge} on the bound was clamped: {clamped:?}"
            );
            assert_eq!(tuning.to_words()[word].to_bits(), edge.to_bits());
        }
    }
}

#[test]
fn a_rejected_update_leaves_every_word_bit_identical() {
    const NON_POSITIVE: [f64; 3] = [0.0, -0.0, -1.0];
    let bad = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
    for word in 0..LiveTuning::WORDS {
        let bound = &LiveTuning::BOUNDS[word];
        let rejected: &[f64] = if bound.reject_non_positive {
            &NON_POSITIVE
        } else {
            &[]
        };
        for value in bad.iter().chain(rejected) {
            let mut tuning = probe();
            let error = tuning.apply_update(&with_word(word, *value)).unwrap_err();
            assert!(
                matches!(&error, FrankaError::InvalidArgument(m) if m.contains(bound.name)),
                "{}{:?} at {value} reported {error:?}",
                bound.name,
                bound.index
            );
            assert_eq!(bits(&tuning), bits(&probe()), "a rejection wrote a word");
        }
    }
}

#[test]
fn a_zero_below_a_clamping_floor_still_clamps() {
    // The counterpart of the test above: only `reject_non_positive` fields refuse a zero.
    let mut tuning = probe();
    let clamped = tuning.apply_update(&with_word(19, 0.0)).unwrap();
    assert_eq!(clamped, vec![&LiveTuning::BOUNDS[19]]);
    assert_eq!(tuning.budget[0], 0.05);
}

#[test]
fn an_absent_field_keeps_its_current_value() {
    let mut tuning = shipped();
    let update = TuningUpdate {
        ik_damping: Some(0.2),
        ..TuningUpdate::default()
    };
    assert!(tuning.apply_update(&update).unwrap().is_empty());
    assert_eq!(tuning.ik_damping, 0.2);
    let mut expected = shipped();
    expected.ik_damping = 0.2;
    assert_eq!(bits(&tuning), bits(&expected));
    assert_eq!(TuningUpdate::default().joint_stiffness, None);
}

/// A permissive configuration can seed a session outside `BOUNDS`. Moving one slider must not
/// retune a field nobody touched: lowering a budget is an acceleration impulse, not a re-plan,
/// and it would arrive on a field the operator did not move.
#[test]
fn an_update_neither_clamps_nor_refuses_a_field_it_does_not_carry() {
    let seeded = LiveTuning {
        budget: [5.0, 50.0, 2000.0],
        ik_nullspace_gain: f64::NAN,
        ..shipped()
    };
    let mut tuning = seeded;
    let update = TuningUpdate {
        ik_damping: Some(0.2),
        ..TuningUpdate::default()
    };
    assert!(tuning.apply_update(&update).unwrap().is_empty());
    assert_eq!(tuning.budget, seeded.budget);
    assert!(tuning.ik_nullspace_gain.is_nan());
    assert_eq!(tuning.ik_damping, 0.2);

    // The same values do clamp once the update carries them.
    let update = TuningUpdate {
        budget: Some(seeded.budget),
        ..TuningUpdate::default()
    };
    let clamped = tuning.apply_update(&update).unwrap();
    assert_eq!(clamped.len(), 3, "{clamped:?}");
    assert_eq!(tuning.budget, [1.2, 20.0, 800.0]);
}

/// The corner `joint_damping_floor` exists for: the stiffest spring the table allows with no
/// damping at all is one slider move away, and the velocity barrier does not reach it.
#[test]
fn the_stiffest_spring_cannot_be_left_undamped() {
    let mut tuning = shipped();
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_stiffness: Some([1200.0; 7]),
            joint_damping: Some([0.0; 7]),
            ..TuningUpdate::default()
        })
        .unwrap();
    let floor = LiveTuning::joint_damping_floor(1200.0);
    assert_eq!(tuning.joint_stiffness, [1200.0; 7]);
    assert_eq!(tuning.joint_damping, [floor; 7]);
    assert_eq!(clamped.len(), 7, "{clamped:?}");
    assert!(clamped.iter().all(|b| b.name == "joint_damping"));
}

/// The one exception to "only what the update carries": a spring cannot be raised out from
/// under its own damping, and the damping that rises is reported.
#[test]
fn raising_a_spring_raises_a_damping_the_update_did_not_carry() {
    let mut tuning = shipped();
    let mut stiffness = tuning.joint_stiffness;
    stiffness[0] = 1200.0;
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_stiffness: Some(stiffness),
            ..TuningUpdate::default()
        })
        .unwrap();
    assert_eq!(clamped, vec![&LiveTuning::BOUNDS[7]], "{clamped:?}");
    assert_eq!(
        tuning.joint_damping[0],
        LiveTuning::joint_damping_floor(1200.0)
    );
    // Only joint 1 was stiffened, so every other damping is the preset's, untouched.
    assert_eq!(
        tuning.joint_damping[1..],
        shipped().joint_damping[1..],
        "an unrelated joint's damping moved"
    );
}

/// Lowering a spring does not drag its damping down with it: the floor is a floor, not a
/// target, and nothing else in the pair rule is symmetric.
#[test]
fn lowering_a_spring_leaves_its_damping_where_it_was() {
    let mut tuning = probe();
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_stiffness: Some([1.0; 7]),
            ..TuningUpdate::default()
        })
        .unwrap();
    assert!(clamped.is_empty(), "{clamped:?}");
    assert_eq!(tuning.joint_damping, probe().joint_damping);
}

#[test]
fn the_words_round_trip_including_negative_zero_and_subnormals() {
    let mut words: [f64; LiveTuning::WORDS] = std::array::from_fn(|i| {
        if i % 2 == 0 {
            -(i as f64)
        } else {
            i as f64 * 1.5
        }
    });
    words[0] = -0.0;
    words[1] = f64::MIN_POSITIVE / 2.0;
    let tuning = LiveTuning::from_words(&words);
    assert_eq!(tuning.to_words().map(f64::to_bits), words.map(f64::to_bits));
    assert_eq!(tuning, LiveTuning::from_words(&tuning.to_words()));
}

/// A stiffness above 57 600 Nm/rad puts `0.25·sqrt(K)` over the damping ceiling of 60, and an
/// uncarried stiffness is never clamped, so reading the floor off the raw seed made
/// `f64::clamp` panic on `min > max`. A configuration is a text file and nothing upstream
/// range-checks it, so this had to be unreachable rather than unlikely.
#[test]
fn a_stiffness_no_bound_would_allow_does_not_panic_through_the_pair_rule() {
    let mut tuning = LiveTuning {
        joint_stiffness: [60_000.0; 7],
        ..shipped()
    };
    assert!(LiveTuning::joint_damping_floor(60_000.0) > LiveTuning::BOUNDS[7].max);
    let carried = tuning.joint_damping;
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_damping: Some(carried),
            ..TuningUpdate::default()
        })
        .unwrap();
    // The floor comes off the stiffness the table allows, which is 1200, not 60 000.
    let floor = LiveTuning::joint_damping_floor(LiveTuning::BOUNDS[0].max);
    assert_eq!(tuning.joint_damping, [floor; 7], "{clamped:?}");
    assert_eq!(
        tuning.joint_stiffness, [60_000.0; 7],
        "an uncarried word moved"
    );
}

/// The pair rule reaches past the carried fields, so it must only ever raise. A damping the
/// operator did not move must not be pulled down to the ceiling — that is the very failure
/// the carried-word mask exists to prevent, and `carried()` is per array, so one joint's
/// slider would otherwise drag all seven.
#[test]
fn a_stiffness_update_never_lowers_a_damping_it_does_not_carry() {
    let seeded = LiveTuning {
        joint_damping: [100.0; 7],
        ..shipped()
    };
    let mut tuning = seeded;
    let mut stiffness = tuning.joint_stiffness;
    stiffness[0] = 60.0;
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_stiffness: Some(stiffness),
            ..TuningUpdate::default()
        })
        .unwrap();
    assert!(clamped.is_empty(), "{clamped:?}");
    assert_eq!(tuning.joint_damping, seeded.joint_damping);
    assert_eq!(tuning.joint_stiffness[0], 60.0);

    // Carrying it is what applies the ceiling.
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_damping: Some(seeded.joint_damping),
            ..TuningUpdate::default()
        })
        .unwrap();
    assert_eq!(clamped.len(), 7, "{clamped:?}");
    let ceilings: [f64; 7] = std::array::from_fn(|j| LiveTuning::BOUNDS[7 + j].max);
    assert_eq!(tuning.joint_damping, ceilings);
}

/// The wrist's damping ceiling is its own: a value the shoulder keeps is clamped there.
#[test]
fn the_wrist_damping_is_clamped_below_the_shoulder_ceiling() {
    let mut tuning = probe();
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_damping: Some([55.0; 7]),
            ..TuningUpdate::default()
        })
        .unwrap();
    assert_eq!(
        tuning.joint_damping,
        [55.0, 55.0, 55.0, 55.0, 40.0, 40.0, 40.0]
    );
    let indices: Vec<_> = clamped.iter().map(|b| (b.name, b.index)).collect();
    assert_eq!(
        indices,
        [4, 5, 6].map(|j| ("joint_damping", Some(j))).to_vec()
    );
}

/// An uncarried word was never checked, so a seeded NaN reaches the clamp. It must come back
/// untouched and, above all, must not be reported as a clamp that did not happen.
#[test]
fn an_uncarried_nan_damping_is_left_alone_and_not_reported() {
    let mut tuning = LiveTuning {
        joint_damping: [f64::NAN; 7],
        ..shipped()
    };
    let clamped = tuning
        .apply_update(&TuningUpdate {
            joint_stiffness: Some([1200.0; 7]),
            ..TuningUpdate::default()
        })
        .unwrap();
    assert!(
        clamped.is_empty(),
        "a clamp was reported that did not happen: {clamped:?}"
    );
    assert!(tuning.joint_damping.iter().all(|d| d.is_nan()));
}
