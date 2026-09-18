//! [`LiveTuning::slew_toward`] and the fraction [`TuningPolicy::remaining`] reports: that each
//! word moves under the policy [`LiveTuning::BOUNDS`] names for it, that the crossing takes the
//! time constant it says, and that what a caller is told about it counts down.

use super::super::*;
use super::bench_ik::allocations;
use super::tuning::{bits, probe};
use crate::rate_limiting::DELTA_T;

/// The gap every word of [`target_of`] stands from [`probe`], so one gain is one step.
const GAP: f64 = 10.0;

/// [`probe`] with `gap` added to every word: one target that moves all twenty-five.
fn target_of(base: &LiveTuning, gap: f64) -> LiveTuning {
    LiveTuning::from_words(&base.to_words().map(|w| w + gap))
}

fn gains() -> SlewGains {
    LiveTuning::slew_gains(DELTA_T)
}

/// The one gain every slewed word runs at, `1 - e^(-dt/tau)`.
fn slew_gain() -> f64 {
    TuningPolicy::Slew { tau: SLEW_TAU }.slew_gain(DELTA_T)
}

/// A gate descending at the value of word `rate_word`, for the assertions that need a policy
/// rather than a word of the table.
fn gate(rate_word: usize) -> TuningPolicy {
    TuningPolicy::StepUpGateDown { rate_word }
}

#[test]
fn the_cycle_gain_is_the_exponential_of_the_time_constant() {
    let gain = slew_gain();
    assert!((gain - 0.003_327_783_945).abs() < 1e-12, "{gain}");
    // Not the other first-order gain the crate has, `dt / (dt + tau)`, which the analytic
    // fraction `TuningPolicy::remaining` would then disagree with in the sixth decimal.
    let bilinear = DELTA_T / (DELTA_T + SLEW_TAU);
    assert!((gain - bilinear).abs() > 5e-6, "{gain} against {bilinear}");
    // A stepped field takes the whole gap, and so does a degenerate cycle time.
    assert_eq!(TuningPolicy::Step.slew_gain(DELTA_T), 1.0);
    assert_eq!(gate(20).slew_gain(DELTA_T), 1.0);
    assert_eq!(TuningPolicy::Slew { tau: SLEW_TAU }.slew_gain(0.0), 1.0);
}

#[test]
fn each_word_moves_one_cycle_under_the_policy_its_bound_names() {
    // Both ways round. A gated word is the only one whose two directions differ, and a table
    // that lost the asymmetry would pass one of these and fail the other.
    for gap in [GAP, -GAP] {
        assert_one_cycle_of_every_word(gap);
    }
}

/// One cycle of a target `gap` away in one word, for each word in turn: that the word lands
/// where its policy says it should, and that no other word moved with it.
fn assert_one_cycle_of_every_word(gap: f64) {
    let (base, gains) = (probe(), gains());
    for word in 0..LiveTuning::WORDS {
        let mut words = base.to_words();
        words[word] += gap;
        let target = LiveTuning::from_words(&words);
        let mut current = base;
        assert!(current.slew_toward(&target, &gains), "word {word} stood");
        let moved = current.to_words();
        let expected = match LiveTuning::BOUNDS[word].policy {
            TuningPolicy::Slew { .. } => base.to_words()[word] + slew_gain() * gap,
            // Raised whole, and lowered by no more than one cycle of the next order's limit,
            // which `BOUNDS` puts in the next word.
            TuningPolicy::StepUpGateDown { rate_word } if gap < 0.0 => {
                base.to_words()[word] - base.to_words()[rate_word] * DELTA_T
            }
            TuningPolicy::Step | TuningPolicy::StepUpGateDown { .. } => words[word],
        };
        assert!(
            (moved[word] - expected).abs() < 1e-12,
            "{}{:?} landed on {} rather than {expected} for a gap of {gap}",
            LiveTuning::BOUNDS[word].name,
            LiveTuning::BOUNDS[word].index,
            moved[word]
        );
        for other in (0..LiveTuning::WORDS).filter(|o| *o != word) {
            assert_eq!(
                moved[other].to_bits(),
                base.to_words()[other].to_bits(),
                "word {other} moved with word {word}"
            );
        }
    }
}

#[test]
fn a_slewed_word_crosses_63_percent_of_its_gap_in_one_time_constant() {
    let (base, target, gains) = (probe(), target_of(&probe(), GAP), gains());
    let cycles = (SLEW_TAU / DELTA_T).round() as usize;
    let mut current = base;
    for _ in 0..cycles {
        current.slew_toward(&target, &gains);
    }
    let crossed = (current.ik_damping - base.ik_damping) / GAP;
    assert!(
        (crossed - 0.632_120_558).abs() < 1e-3,
        "{crossed} of the gap after {cycles} cycles"
    );
    // One cycle either side is the tolerance; a tenfold slower or faster filter is not.
    assert!(crossed > 0.630 && crossed < 0.634, "{crossed}");
    // A stepped word was there on the first of those cycles.
    assert_eq!(
        current.velocity_feedforward_cutoff.to_bits(),
        target.velocity_feedforward_cutoff.to_bits()
    );
}

#[test]
fn no_cycle_of_a_crossing_moves_a_word_further_than_its_gain_allows() {
    let (base, target, gains) = (probe(), target_of(&probe(), GAP), gains());
    let rate = slew_gain() * GAP;
    let (mut current, mut previous) = (base, base);
    for cycle in 0..3000 {
        current.slew_toward(&target, &gains);
        let (now, was) = (current.to_words(), previous.to_words());
        for word in 0..LiveTuning::WORDS {
            let step = (now[word] - was[word]).abs();
            let allowed = match LiveTuning::BOUNDS[word].policy {
                TuningPolicy::Slew { .. } => rate,
                TuningPolicy::Step | TuningPolicy::StepUpGateDown { .. } => GAP,
            };
            // The slack is the rounding of a difference of two values a thousand times the
            // step, not room for a second cycle's worth.
            assert!(
                step <= allowed + 1e-12,
                "word {word} moved {step} in cycle {cycle}, over {allowed}"
            );
        }
        previous = current;
    }
}

#[test]
fn the_reported_fraction_is_what_is_left_and_counts_down_to_zero() {
    let (base, target, gains) = (probe(), target_of(&probe(), GAP), gains());
    let policy = TuningPolicy::Slew { tau: SLEW_TAU };
    assert_eq!(policy.remaining(0.0), 1.0);
    assert_eq!(policy.remaining(-1.0), 1.0, "nothing has happened yet");
    let mut current = base;
    let mut previous = 1.0;
    for cycle in 1..=900 {
        current.slew_toward(&target, &gains);
        let left = (target.ik_damping - current.ik_damping) / GAP;
        let reported = policy.remaining(cycle as f64 * DELTA_T);
        assert!(
            (reported - left).abs() < 1e-12,
            "cycle {cycle}: {reported} reported, {left} left"
        );
        assert!(
            reported < previous,
            "cycle {cycle}: {reported} did not fall"
        );
        previous = reported;
    }
    // Counted down, not up: three time constants in, a twentieth is left, not nineteen.
    assert!(previous < 0.05, "{previous}");
    // A stepped word is in force and has nothing to report at any time.
    assert_eq!(TuningPolicy::Step.remaining(0.0), 0.0);
    assert_eq!(gate(20).remaining(10.0), 0.0);
}

#[test]
fn a_target_arriving_mid_crossing_moves_the_attractor_and_nothing_else() {
    let (base, gains) = (probe(), gains());
    let first = LiveTuning {
        ik_damping: base.ik_damping + GAP,
        ..base
    };
    // Back where it started, halfway through the crossing to `first`.
    let second = base;
    let mut current = base;
    for _ in 0..100 {
        current.slew_toward(&first, &gains);
    }
    // The cycle the new target lands on is a cycle like any other: one gain of the new gap,
    // from where the old crossing had got to, with no restart and no step.
    let caught = current;
    current.slew_toward(&second, &gains);
    let step = current.ik_damping - caught.ik_damping;
    let expected = slew_gain() * (second.ik_damping - caught.ik_damping);
    assert!((step - expected).abs() < 1e-15, "{step} against {expected}");
    // And it keeps going there, monotonically: no overshoot, no restart from the old gap.
    let mut previous = caught.ik_damping;
    while current.slew_toward(&second, &gains) {
        assert!(
            current.ik_damping < previous && current.ik_damping >= second.ik_damping,
            "{} left {previous} for a value past {}",
            current.ik_damping,
            second.ik_damping
        );
        previous = current.ik_damping;
    }
    assert_eq!(current.ik_damping.to_bits(), second.ik_damping.to_bits());
}

#[test]
fn a_crossing_lands_on_its_target_and_then_reports_nothing_to_do() {
    let (base, target, gains) = (probe(), target_of(&probe(), GAP), gains());
    let mut current = base;
    let mut cycles = 0;
    while current.slew_toward(&target, &gains) {
        cycles += 1;
        assert!(cycles < 100_000, "the crossing did not converge");
    }
    assert_eq!(bits(&current), bits(&target));
    assert!(!current.slew_toward(&target, &gains));
    // Bounded, and long after the arm has stopped feeling it: a few time constants of ulps.
    assert!(cycles < 20_000, "{cycles} cycles to converge");
}

/// The gated words, each beside the word whose value bounds its descent: the budget's velocity
/// at its acceleration, its acceleration at its jerk, translational then rotational. Written
/// out rather than derived, so that a table that renumbered the words has to say so here.
const GATED: [(usize, usize); 4] = [(19, 20), (20, 21), (22, 23), (23, 24)];

#[test]
fn the_gated_words_are_the_budgets_velocity_and_acceleration() {
    for word in 0..LiveTuning::WORDS {
        let gated = LiveTuning::BOUNDS[word].policy.rate_word().is_some();
        assert_eq!(
            gated,
            GATED.iter().any(|(w, _)| *w == word),
            "word {word} is gated in one place and not the other"
        );
    }
    // Each gated word names the word its descent rate is read off, and it must be the next
    // order of its own budget. Nothing else in the crossing says so, and a table that
    // renumbered them would have every gated word descending at a stranger's limit.
    for (word, order) in GATED {
        let (gated, rate) = (&LiveTuning::BOUNDS[word], &LiveTuning::BOUNDS[order]);
        assert_eq!(
            gated.policy.rate_word(),
            Some(order),
            "{} does not name word {order} as its rate",
            gated.name
        );
        assert_eq!(rate.name, gated.name, "word {order} is a different field");
        assert_eq!(
            rate.index,
            gated.index.map(|i| i + 1),
            "word {order} is not the next order of {}",
            gated.name
        );
    }
}

#[test]
fn a_raised_budget_is_in_force_on_the_cycle_it_arrives() {
    let (base, gains) = (probe(), gains());
    for (word, _) in GATED {
        let mut words = base.to_words();
        words[word] *= 2.0;
        let target = LiveTuning::from_words(&words);
        let mut current = base;
        assert!(current.slew_toward(&target, &gains), "word {word} stood");
        assert_eq!(
            current.to_words()[word].to_bits(),
            words[word].to_bits(),
            "word {word} ramped up instead of stepping"
        );
        assert!(!current.slew_toward(&target, &gains), "word {word} kept on");
    }
}

#[test]
fn a_lowered_budget_ramps_down_at_the_next_order_and_never_faster() {
    let (base, gains) = (probe(), gains());
    for (word, order) in GATED {
        let mut words = base.to_words();
        words[word] *= 0.25;
        let (target, was) = (LiveTuning::from_words(&words), base.to_words());
        // Only `word` differs from `base`, so the rate is the one `base` holds throughout.
        let step = was[order] * DELTA_T;
        let (mut current, mut previous, mut cycles) = (base, was[word], 0u32);
        while current.slew_toward(&target, &gains) {
            cycles += 1;
            assert!(cycles < 100_000, "word {word} never landed");
            let now = current.to_words()[word];
            assert!(now < previous, "cycle {cycles}: word {word} did not fall");
            assert!(
                previous - now <= step + 1e-15,
                "cycle {cycles}: word {word} fell {} in one cycle, over {step}",
                previous - now
            );
            assert!(now >= words[word], "word {word} fell past its target");
            previous = now;
        }
        assert_eq!(current.to_words()[word].to_bits(), words[word].to_bits());
        // And it is a ramp, not a step: the same gap the other way takes one cycle.
        let ramp = ((was[word] - words[word]) / step).ceil() as u32;
        assert!(
            cycles.abs_diff(ramp) <= 1 && cycles > 10,
            "word {word} took {cycles} cycles, not the {ramp} its rate implies"
        );
    }
}

#[test]
fn the_budgets_jerks_step_in_both_directions() {
    let (base, gains) = (probe(), gains());
    // Nothing stores a jerk, so lowering one lengthens the next plan's brake and clamps no
    // state: it is the one budget word that may be thrown rather than walked.
    for word in [21, 24] {
        for factor in [0.25, 4.0] {
            let mut words = base.to_words();
            words[word] *= factor;
            let target = LiveTuning::from_words(&words);
            let mut current = base;
            assert!(current.slew_toward(&target, &gains));
            assert_eq!(
                current.to_words()[word].to_bits(),
                words[word].to_bits(),
                "the jerk at word {word} did not take a factor of {factor} whole"
            );
        }
    }
}

#[test]
fn a_budget_descends_at_the_next_order_in_force_at_the_top_of_the_cycle() {
    let (base, gains) = (probe(), gains());
    let was = base.to_words();
    let mut words = was;
    // As far down as the velocity goes, with its rate raised fourfold in the same update.
    words[19] = 0.0;
    words[20] = 4.0 * was[20];
    let target = LiveTuning::from_words(&words);
    let mut current = base;

    // The rates are read before anything moves, so this cycle descends at the acceleration
    // that was in force while the acceleration itself steps up beside it. Read after the
    // walk instead, the first cycle would already take four times as much off the velocity.
    assert!(current.slew_toward(&target, &gains));
    let first = current.to_words();
    assert!(
        (was[19] - first[19] - was[20] * DELTA_T).abs() < 1e-15,
        "the first cycle took {} off the velocity",
        was[19] - first[19]
    );
    assert_eq!(first[20].to_bits(), words[20].to_bits());

    // From the next cycle on it descends at the raised one.
    assert!(current.slew_toward(&target, &gains));
    let second = current.to_words();
    assert!(
        (first[19] - second[19] - words[20] * DELTA_T).abs() < 1e-15,
        "the second cycle took {} off the velocity",
        first[19] - second[19]
    );
}

#[test]
fn a_cycle_time_that_does_not_pass_cannot_walk_a_gated_word_the_wrong_way() {
    // The floor is `current - rate * dt`, so a negative `dt` would put it *above* where the
    // word is and walk one being lowered upwards. `slew_gains` floors the cycle time at zero,
    // and a cycle that does not pass leaves a gated word exactly where it was.
    let base = probe();
    let mut words = base.to_words();
    words[19] *= 0.5;
    let target = LiveTuning::from_words(&words);
    for dt in [0.0, -1.0, f64::NAN] {
        let mut current = base;
        current.slew_toward(&target, &LiveTuning::slew_gains(dt));
        assert_eq!(
            current.budget[0].to_bits(),
            base.budget[0].to_bits(),
            "a cycle of {dt} s moved the budget"
        );
    }
}

#[test]
fn a_crossing_allocates_nothing() {
    let (base, target, gains) = (probe(), target_of(&probe(), GAP), gains());
    let mut current = base;
    let before = allocations();
    for _ in 0..1000 {
        std::hint::black_box(current.slew_toward(std::hint::black_box(&target), &gains));
    }
    assert_eq!(allocations(), before, "the crossing allocated");
}

/// The seconds [`TuningPolicy::descent`] reports are the seconds the crossing takes, measured:
/// a watcher's arithmetic and the loop's ramp are the same ramp, and the rate is the word the
/// table names rather than whichever one the reader guessed.
#[test]
fn the_descent_reported_is_the_descent_the_crossing_runs() {
    let (base, gains) = (probe(), gains());
    for (word, order) in GATED {
        let policy = LiveTuning::BOUNDS[word].policy;
        let mut words = base.to_words();
        words[word] *= 0.25;
        let target = LiveTuning::from_words(&words);
        let (was, rate) = (base.to_words()[word], base.to_words()[order]);
        let seconds = policy.descent(was, words[word], rate);
        let mut current = base;
        let mut cycles = 0u32;
        while current.slew_toward(&target, &gains) {
            cycles += 1;
            assert!(cycles < 100_000, "word {word} never landed");
        }
        let measured = f64::from(cycles) * DELTA_T;
        assert!(
            (measured - seconds).abs() <= 2.0 * DELTA_T,
            "word {word} took {measured} s, reported as {seconds} s"
        );
        // The rate is the next order's value and not the word's own, which is the guess a
        // reader without the table would make: at that rate the answer is a different number.
        assert!(
            (policy.descent(was, words[word], was) - measured).abs() > 2.0 * DELTA_T,
            "word {word}: its own value is as good a rate as the one the table names"
        );
    }
}

#[test]
fn nothing_but_a_gated_word_going_down_has_a_descent() {
    let policy = LiveTuning::BOUNDS[19].policy;
    // A raise is in force on the cycle it arrives, and so is a word that never moved.
    assert_eq!(policy.descent(0.3, 0.9, 0.5), 0.0);
    assert_eq!(policy.descent(0.3, 0.3, 0.5), 0.0);
    // A filtered word's remainder is a fraction of an elapsed time, not a length of one.
    assert_eq!(
        TuningPolicy::Slew { tau: SLEW_TAU }.descent(0.9, 0.3, 0.5),
        0.0
    );
    assert_eq!(TuningPolicy::Step.descent(0.9, 0.3, 0.5), 0.0);
    // A rate of zero is a descent that never arrives, not a division by zero.
    assert!(policy.descent(0.9, 0.3, 0.0).is_infinite());
    assert!(policy.descent(0.9, 0.3, -1.0).is_infinite());
    // And a NaN either side is nobody's ramp.
    assert_eq!(policy.descent(f64::NAN, 0.3, 0.5), 0.0);
    assert_eq!(policy.descent(0.9, f64::NAN, 0.5), 0.0);
}
