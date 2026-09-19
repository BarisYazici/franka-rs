//! [`LiveTuning::BOUNDS`] itself: what each word carries, what each bound says, and that the
//! table is no tighter than the library's own validators and no looser than what the crate
//! ships. What [`LiveTuning::apply_update`] does with it is in [`tuning_update`](super::tuning_update).

use super::super::*;
use crate::otg::OtgLimits;

/// What word carries what, pinned. The clamp walks [`LiveTuning::BOUNDS`] against the word
/// array, so a bound on the wrong word would gate the wrong field; this list plus
/// [`each_word_carries_the_field_layout_names_it`] is the only thing that ties the two.
pub(super) const LAYOUT: [(&str, Option<usize>); LiveTuning::WORDS] = [
    ("joint_stiffness", Some(0)),
    ("joint_stiffness", Some(1)),
    ("joint_stiffness", Some(2)),
    ("joint_stiffness", Some(3)),
    ("joint_stiffness", Some(4)),
    ("joint_stiffness", Some(5)),
    ("joint_stiffness", Some(6)),
    ("joint_damping", Some(0)),
    ("joint_damping", Some(1)),
    ("joint_damping", Some(2)),
    ("joint_damping", Some(3)),
    ("joint_damping", Some(4)),
    ("joint_damping", Some(5)),
    ("joint_damping", Some(6)),
    ("cartesian_stiffness", None),
    ("ik_damping", None),
    ("ik_nullspace_gain", None),
    ("velocity_feedforward_gain", None),
    ("velocity_feedforward_cutoff", None),
    ("budget", Some(0)),
    ("budget", Some(1)),
    ("budget", Some(2)),
    ("rotation_budget", Some(0)),
    ("rotation_budget", Some(1)),
    ("rotation_budget", Some(2)),
];

/// What a Cartesian session ships with, taken from the crate's own constants rather than
/// retyped, so that a changed default shows up here.
pub(super) fn shipped() -> LiveTuning {
    let defaults = TargetControlOptions::default();
    LiveTuning::from_options(
        &ImpedanceOptions::cartesian(),
        defaults.limits,
        defaults.rotation_limits,
    )
}

/// [`shipped`] damped well clear of the floor its stiffness ceiling implies, so that a probe
/// of one word moves exactly one word. The pair rule has its own tests.
pub(super) fn probe() -> LiveTuning {
    LiveTuning {
        joint_damping: [20.0; 7],
        ..shipped()
    }
}

/// The floor `apply_update` actually holds `word` above, given `stiffness` in force.
pub(super) fn floor_of(word: usize, stiffness: &[f64; 7]) -> f64 {
    let min = LiveTuning::BOUNDS[word].min;
    match word {
        7..=13 => min.max(LiveTuning::joint_damping_floor(stiffness[word - 7])),
        _ => min,
    }
}

/// Every field of `tuning` as an update.
pub(super) fn whole(tuning: &LiveTuning) -> TuningUpdate {
    TuningUpdate {
        joint_stiffness: Some(tuning.joint_stiffness),
        joint_damping: Some(tuning.joint_damping),
        cartesian_stiffness: Some(tuning.cartesian_stiffness),
        ik_damping: Some(tuning.ik_damping),
        ik_nullspace_gain: Some(tuning.ik_nullspace_gain),
        velocity_feedforward_gain: Some(tuning.velocity_feedforward_gain),
        velocity_feedforward_cutoff: Some(tuning.velocity_feedforward_cutoff),
        budget: Some(tuning.budget),
        rotation_budget: Some(tuning.rotation_budget),
    }
}

/// [`probe`] with one word replaced, as an update carrying every field.
pub(super) fn with_word(word: usize, value: f64) -> TuningUpdate {
    let mut words = probe().to_words();
    words[word] = value;
    whole(&LiveTuning::from_words(&words))
}

pub(super) fn bits(tuning: &LiveTuning) -> [u64; LiveTuning::WORDS] {
    tuning.to_words().map(f64::to_bits)
}

/// Each word carries the field `LAYOUT` names, written through the struct rather than the
/// array: this is the only thing that ties `BOUNDS` to `to_words`, and the clamp depends on it.
#[test]
fn each_word_carries_the_field_layout_names_it() {
    for (word, (name, index)) in LAYOUT.into_iter().enumerate() {
        let mut tuning = probe();
        let sentinel = 12_345.0 + word as f64;
        match (name, index) {
            ("joint_stiffness", Some(i)) => tuning.joint_stiffness[i] = sentinel,
            ("joint_damping", Some(i)) => tuning.joint_damping[i] = sentinel,
            ("cartesian_stiffness", None) => tuning.cartesian_stiffness = sentinel,
            ("ik_damping", None) => tuning.ik_damping = sentinel,
            ("ik_nullspace_gain", None) => tuning.ik_nullspace_gain = sentinel,
            ("velocity_feedforward_gain", None) => tuning.velocity_feedforward_gain = sentinel,
            ("velocity_feedforward_cutoff", None) => tuning.velocity_feedforward_cutoff = sentinel,
            ("budget", Some(i)) => tuning.budget[i] = sentinel,
            ("rotation_budget", Some(i)) => tuning.rotation_budget[i] = sentinel,
            other => panic!("LAYOUT names a field that is not in LiveTuning: {other:?}"),
        }
        let words = tuning.to_words();
        assert_eq!(
            words[word], sentinel,
            "{name}{index:?} is not at word {word}"
        );
        for other in (0..LiveTuning::WORDS).filter(|o| *o != word) {
            assert_eq!(
                words[other],
                probe().to_words()[other],
                "word {other} moved"
            );
        }
        // The other direction, without a second copy of the mapping above.
        assert_eq!(
            LiveTuning::from_words(&words),
            tuning,
            "from_words disagrees with to_words at word {word}"
        );
    }
}

#[test]
fn the_bounds_describe_every_scalar_exactly_once_and_in_word_order() {
    assert_eq!(LiveTuning::BOUNDS.len(), LiveTuning::WORDS);
    for (bound, (name, index)) in LiveTuning::BOUNDS.iter().zip(LAYOUT) {
        assert_eq!((bound.name, bound.index), (name, index));
        assert!(
            bound.min < bound.max && bound.min.is_finite() && bound.max.is_finite(),
            "{name}{index:?} has an empty or infinite range"
        );
    }
}

#[test]
fn the_bounds_are_the_numbers_the_design_settled() {
    let at = |word: usize| (LiveTuning::BOUNDS[word].min, LiveTuning::BOUNDS[word].max);
    for joint in 0..7 {
        assert_eq!(at(joint), (0.0, 1200.0));
        // 60, not reference-stack's 80, because the barrier adds its 20 on top near the velocity limit.
        assert_eq!(at(7 + joint), (0.0, 60.0));
        assert_eq!(
            LiveTuning::BOUNDS[7 + joint].max + VELOCITY_BARRIER_GAIN,
            80.0
        );
    }
    assert_eq!(at(14), (50.0, 3000.0));
    assert_eq!(at(15), (1e-3, 1.0));
    assert_eq!(at(16), (0.0, 20.0));
    assert_eq!(at(17), (0.0, 1.0));
    assert_eq!(at(18), (MIN_FEEDFORWARD_CUTOFF, 1000.0));
    assert_eq!(at(19), (0.05, 1.2));
    assert_eq!(at(20), (0.1, 20.0));
    assert_eq!(at(21), (1.0, 800.0));
    assert_eq!(at(22), (0.05, 2.5));
    assert_eq!(at(23), (0.1, 30.0));
    assert_eq!(at(24), (1.0, 800.0));

    // Only the two where zero means something else than "softer" are rejected rather than
    // clamped, and only the three ranges spanning decades are drawn logarithmically.
    let flagged = |f: fn(&FieldBound) -> bool| {
        (0..LiveTuning::WORDS)
            .filter(|w| f(&LiveTuning::BOUNDS[*w]))
            .collect::<Vec<_>>()
    };
    assert_eq!(flagged(|b| b.reject_non_positive), vec![14, 15]);
    assert_eq!(flagged(|b| b.log_slider), vec![14, 15, 18]);
}

/// Stated by name rather than by position, so that adding a field cannot shift the expectation
/// off the word it was written for.
#[test]
fn every_field_has_the_policy_its_product_needs() {
    assert_eq!(SLEW_TAU, 0.3);
    for (word, (name, index)) in LAYOUT.into_iter().enumerate() {
        let expected = match (name, index) {
            // The filter's state carries across the change, so the output steps by at most the
            // ripple the filter exists to remove.
            ("velocity_feedforward_cutoff", _) => TuningPolicy::Step,
            // The generator stores no jerk, so jerk re-plans; velocity and acceleration are
            // stored and clamped every cycle, so lowering either is an impulse.
            ("budget" | "rotation_budget", Some(2)) => TuningPolicy::Step,
            ("budget" | "rotation_budget", _) => TuningPolicy::StepUpGateDown {
                rate_word: word + 1,
            },
            // Everything else multiplies a generally-nonzero state.
            _ => TuningPolicy::Slew { tau: SLEW_TAU },
        };
        assert_eq!(
            LiveTuning::BOUNDS[word].policy,
            expected,
            "{name}{index:?} has the wrong policy"
        );
    }
}

/// The pair rule of [`LiveTuning::joint_damping_floor`]: it must forbid the undamped corner,
/// it must not reach the damping ceiling, and it must not clamp anything the crate ships.
#[test]
fn the_joint_damping_floor_clears_every_shipped_preset_and_never_meets_the_ceiling() {
    assert_eq!(MIN_JOINT_DAMPING_RATIO, 0.25);
    let at_ceiling = LiveTuning::joint_damping_floor(LiveTuning::BOUNDS[0].max);
    assert!(
        (at_ceiling - 8.660_254_037_844_387).abs() < 1e-12,
        "the floor under the stiffest spring is {at_ceiling}"
    );
    assert!(
        at_ceiling < LiveTuning::BOUNDS[7].max,
        "the floor fights the ceiling"
    );
    assert_eq!(LiveTuning::joint_damping_floor(0.0), 0.0);

    for gains in [
        ImpedanceGains::CARTESIAN,
        ImpedanceGains::JOINT,
        ImpedanceGains::DROID,
    ] {
        for joint in 0..7 {
            let (k, d) = (gains.joint_stiffness[joint], gains.joint_damping[joint]);
            assert!(
                d >= LiveTuning::joint_damping_floor(k),
                "a shipped preset is below the floor at joint {joint}: K {k}, D {d}"
            );
        }
    }
}

/// The class of failure the owner has already paid a day for: a default the panel silently
/// caps at engage. Every preset and default the crate ships must pass the gate untouched.
#[test]
fn every_shipped_default_is_inside_the_bounds_and_is_not_clamped() {
    for gains in [
        ImpedanceGains::CARTESIAN,
        ImpedanceGains::JOINT,
        ImpedanceGains::DROID,
    ] {
        for options in [ImpedanceOptions::cartesian(), ImpedanceOptions::joint()] {
            let mut tuning = LiveTuning {
                joint_stiffness: gains.joint_stiffness,
                joint_damping: gains.joint_damping,
                // The joint presets carry no Cartesian spring, so the scalar is the one a
                // Cartesian session would seed; on a joint session it is inert.
                cartesian_stiffness: ImpedanceGains::CARTESIAN.cartesian_stiffness[0],
                ik_damping: options.ik.damping,
                ik_nullspace_gain: options.ik.nullspace_gain,
                velocity_feedforward_gain: options.velocity_feedforward_gain,
                velocity_feedforward_cutoff: options.velocity_feedforward_cutoff,
                ..shipped()
            };
            let before = bits(&tuning);
            let carry = whole(&tuning);
            let clamped = tuning.apply_update(&carry).unwrap();
            assert!(
                clamped.is_empty(),
                "a shipped default is outside BOUNDS: {clamped:?} of {tuning:?}"
            );
            assert_eq!(bits(&tuning), before);
        }
    }
}

/// The invariant that keeps this gate and the library's honest: everything `BOUNDS` admits,
/// [`ImpedanceOptions::validate`] and [`TargetControlOptions::validate`] admit too. `BOUNDS`
/// may be tighter — that is its purpose — never looser.
#[test]
fn every_bound_is_at_least_as_tight_as_the_library_validators() {
    for word in 0..LiveTuning::WORDS {
        let bound = &LiveTuning::BOUNDS[word];
        for edge in [floor_of(word, &probe().joint_stiffness), bound.max] {
            let mut tuning = probe();
            tuning.apply_update(&with_word(word, edge)).unwrap();
            let name = bound.name;
            assert!(
                impedance_of(&tuning).validate().is_ok(),
                "{name} at {edge} is outside ImpedanceOptions::validate"
            );
            assert!(
                budgets_of(&tuning).validate().is_ok(),
                "{name} at {edge} is outside TargetControlOptions::validate"
            );
        }
    }
}

/// The options a session would run at `tuning`. The one Cartesian scalar becomes twelve gains
/// through the library's own rule, which is the only copy of it.
pub(super) fn impedance_of(tuning: &LiveTuning) -> ImpedanceOptions {
    let mut options = ImpedanceOptions::cartesian();
    options.gains = ImpedanceGains {
        joint_stiffness: tuning.joint_stiffness,
        joint_damping: tuning.joint_damping,
        ..ImpedanceGains::scaled_cartesian(tuning.cartesian_stiffness)
    };
    options.ik.damping = tuning.ik_damping;
    options.ik.nullspace_gain = tuning.ik_nullspace_gain;
    // As the loop applies it: the switch rides in the weight.
    options.velocity_feedforward = true;
    options.velocity_feedforward_gain = tuning.velocity_feedforward_gain;
    options.velocity_feedforward_cutoff = tuning.velocity_feedforward_cutoff;
    options
}

pub(super) fn budgets_of(tuning: &LiveTuning) -> TargetControlOptions {
    let limits = |budget: [f64; 3]| OtgLimits {
        max_velocity: budget[0],
        max_acceleration: budget[1],
        max_jerk: budget[2],
    };
    TargetControlOptions::default()
        .with_limits(limits(tuning.budget))
        .with_rotation_limits(limits(tuning.rotation_budget))
}

/// The one home of the `ratio × CARTESIAN` rule, pinned here rather than only in the node that
/// used to own it: the damping scales with the square root, so a linear rescale would show.
#[test]
fn the_scaled_preset_holds_the_damping_ratio_it_was_tuned_for() {
    let preset = ImpedanceGains::CARTESIAN;
    assert_eq!(
        ImpedanceGains::scaled_cartesian(preset.cartesian_stiffness[0]),
        preset
    );
    let stiff = ImpedanceGains::scaled_cartesian(4.0 * preset.cartesian_stiffness[0]);
    assert_eq!(
        stiff.cartesian_stiffness,
        [3000.0, 3000.0, 3000.0, 60.0, 60.0, 60.0]
    );
    assert_eq!(
        stiff.cartesian_damping,
        [100.0, 100.0, 180.0, 4.0, 4.0, 4.0]
    );
    assert_eq!(stiff.joint_stiffness, preset.joint_stiffness);
}
