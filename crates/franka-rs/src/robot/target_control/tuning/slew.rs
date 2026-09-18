//! How a [`LiveTuning`] target is moved into force: the first-order filter the loop runs, and
//! the same filter read off the clock for a caller that only watches.
//!
//! A parameter multiplies some state. Where that state is generally nonzero -- the joint error
//! a spring acts on, the goal velocity the feedforward weighs -- a step in the parameter is a
//! step in the torque, so those fields cross to their target over
//! [`SLEW_TAU`](super::SLEW_TAU) instead. Which field does which is
//! [`FieldBound::policy`](super::FieldBound::policy), and this module is the only reader of it:
//! [`LiveTuning::slew_gains`] turns the whole table into one gain per word, once, so the cycle
//! itself holds no `exp` and no match on a policy.
//!
//! Both halves are the same filter, `e^(-t/tau)`, on purpose. The loop applies it a cycle at a
//! time ([`LiveTuning::slew_toward`]); a panel is told how much of a change is still to come by
//! [`TuningPolicy::remaining`], which reads the elapsed time rather than the loop, so nothing has
//! to be published back out of the realtime thread. The exact exponential, not the crate's other
//! first-order gain ([`lowpass_filter::gain`](crate::lowpass_filter), `dt / (dt + tau)`, which
//! differs from it in the sixth decimal here): a fraction reported off a filter the loop is not
//! running is a fraction of nothing.
//!
//! The budget's velocity and acceleration are neither slewed nor stepped but *gated*
//! ([`TuningPolicy::StepUpGateDown`]): raising them is a step, and lowering them is a ramp at
//! the next order's limit, because what a narrower budget does to a moving generator is a clamp
//! rather than a re-plan ([`Otg::set_limits`](crate::otg::Otg::set_limits)). That is a floor
//! under the word rather than a filter on it, so it composes with the same
//! [`slewed`] step the other words take and needs no second pass.

use super::{LiveTuning, TuningPolicy};

impl TuningPolicy {
    /// The fraction of the remaining gap one cycle of `dt` seconds closes: `1 - e^(-dt/tau)`
    /// for a slewed field, and 1 -- the whole gap, on the cycle it arrives -- for every other.
    ///
    /// A non-positive `dt` or `tau` has no filter to run and takes the value whole.
    ///
    /// Call it once per session, not per cycle: it holds the only `exp` on this path, and
    /// [`LiveTuning::slew_gains`] is the form the loop wants.
    pub fn slew_gain(self, dt: f64) -> f64 {
        match self {
            TuningPolicy::Slew { tau } if tau > 0.0 && dt > 0.0 => 1.0 - (-dt / tau).exp(),
            _ => 1.0,
        }
    }

    /// The fraction of an accepted change **still to go** `elapsed` seconds after it was
    /// accepted: 1 at acceptance, `e^(-elapsed/tau)` from there, and never negative. It counts
    /// *down*; a caller showing the fraction already reached would draw every slew backwards.
    ///
    /// Zero for [`Step`](Self::Step), which is in force on the first cycle after it is accepted
    /// and so has nothing left to report, and zero for [`StepUpGateDown`](Self::StepUpGateDown),
    /// which is that on the way **up**. A gated field on the way *down* is not a filter at all
    /// but a ramp at the next order's limit, and what is left of one is seconds rather than a
    /// fraction of an elapsed time: [`descent`](Self::descent) is that half.
    ///
    /// This is the loop's filter read off a clock instead of a cycle counter: `elapsed` seconds
    /// of `dt` cycles leave `(1 - slew_gain(dt))^(elapsed/dt)` of the gap. It needs
    /// nothing from the realtime thread, which is why the tuning slot has one writer and no
    /// reply.
    ///
    /// ```
    /// use franka::robot::target_control::{LiveTuning, TuningPolicy, SLEW_TAU};
    ///
    /// let policy = LiveTuning::BOUNDS[0].policy;
    /// assert_eq!(policy, TuningPolicy::Slew { tau: SLEW_TAU });
    /// assert_eq!(policy.remaining(0.0), 1.0);
    /// // One time constant leaves 1/e of the change to come -- not the 1 - 1/e it has made.
    /// assert!((policy.remaining(SLEW_TAU) - 0.367_879_441).abs() < 1e-9);
    /// assert!(policy.remaining(3.0 * SLEW_TAU) < 0.05);
    /// assert_eq!(TuningPolicy::Step.remaining(0.0), 0.0);
    /// ```
    pub fn remaining(self, elapsed: f64) -> f64 {
        match self {
            TuningPolicy::Slew { tau } if tau > 0.0 => (-elapsed.max(0.0) / tau).exp(),
            _ => 0.0,
        }
    }

    /// How long, in seconds, a [gated](Self::StepUpGateDown) word takes to walk from `from`
    /// down to `to` at `rate` units a second: the other half of
    /// [`remaining`](Self::remaining), for the words that ramp rather than filter.
    ///
    /// Zero for a raise, which is in force on the cycle it arrives, and zero for every other
    /// policy, which has no ramp. Infinite where the rate is not a rate: a word whose next
    /// order stands at zero never descends.
    ///
    /// `rate` is the value **in force** of the word [`rate_word`](Self::rate_word) names —
    /// read it off the table rather than from the field beside it, which is the rule this
    /// method exists so that nobody writes down twice. The answer is exact while that word is
    /// itself at rest; a descent whose rate is moving at the same time (both budget sliders
    /// dragged at once) walks at whatever the rate word holds each cycle, and this is then the
    /// estimate its value at the start implies.
    ///
    /// ```
    /// use franka::robot::target_control::{LiveTuning, TuningPolicy};
    ///
    /// // The translational budget's velocity descends at its own acceleration.
    /// let velocity = LiveTuning::BOUNDS[19].policy;
    /// let rate_word = velocity.rate_word().expect("the velocity budget is gated");
    /// assert_eq!(LiveTuning::BOUNDS[rate_word].name, "budget");
    /// assert_eq!(LiveTuning::BOUNDS[rate_word].index, Some(1));
    ///
    /// // From 0.9 m/s to 0.3 m/s at the default 0.5 m/s^2 is 1.2 s of ramp.
    /// assert!((velocity.descent(0.9, 0.3, 0.5) - 1.2).abs() < 1e-12);
    /// // Raising is not a ramp, and neither is a stepped word.
    /// assert_eq!(velocity.descent(0.3, 0.9, 0.5), 0.0);
    /// assert_eq!(TuningPolicy::Step.descent(0.9, 0.3, 0.5), 0.0);
    /// ```
    pub fn descent(self, from: f64, to: f64, rate: f64) -> f64 {
        // Only an ordered pair going down is a ramp, so a NaN either side answers "no ramp"
        // rather than the infinity a caller would then divide by.
        let descending = matches!(from.partial_cmp(&to), Some(std::cmp::Ordering::Greater));
        if !descending || self.rate_word().is_none() {
            return 0.0;
        }
        if rate > 0.0 {
            (from - to) / rate
        } else {
            f64::INFINITY
        }
    }
}

/// Per word: which word's value in force one cycle may take off it going down, which is what
/// [`TuningPolicy::StepUpGateDown`] means, and `None` for a word that is not gated. Flattened
/// out of [`LiveTuning::BOUNDS`]' own [`rate_word`](TuningPolicy::rate_word) at compile time,
/// so the cycle holds no match on a policy and no second opinion about which word the rate is.
const RATE_WORD: [Option<usize>; LiveTuning::WORDS] = {
    let mut rate = [None; LiveTuning::WORDS];
    let mut word = 0;
    while word < LiveTuning::WORDS {
        // `rate_word` is not a `const fn`: a const context cannot call it, and duplicating the
        // match here rather than the *rule* is what keeps the table the only place the rate is
        // named. `bounds.rs` asserts every index is in range.
        if let TuningPolicy::StepUpGateDown { rate_word } = LiveTuning::BOUNDS[word].policy {
            rate[word] = Some(rate_word);
        }
        word += 1;
    }
    rate
};

/// How far one cycle moves each word of a [`LiveTuning`] toward its target: the fraction of the
/// remaining gap a slewed word closes, and the cycle time a gated word's descent is measured in.
///
/// Built once, at the loop's construction, by [`LiveTuning::slew_gains`], out of
/// [`LiveTuning::BOUNDS`]' policies -- so a field whose policy changes there changes here with
/// it, and the cycle itself holds no `exp` and no match on a policy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlewGains {
    /// One [`TuningPolicy::slew_gain`] per word of [`LiveTuning::to_words`].
    gain: [f64; LiveTuning::WORDS],
    /// The cycle time those gains were built for, and what a gated word's descent rate is
    /// multiplied by. Never negative: a cycle that does not pass moves nothing.
    dt: f64,
}

impl LiveTuning {
    /// The [`SlewGains`] of a loop running at `dt` seconds a cycle. Build it once, at the
    /// loop's construction, with the cycle time the loop runs at.
    pub fn slew_gains(dt: f64) -> SlewGains {
        SlewGains {
            gain: std::array::from_fn(|word| Self::BOUNDS[word].policy.slew_gain(dt)),
            dt: dt.max(0.0),
        }
    }

    /// Moves every word one cycle toward `target` under `gains`, and reports whether any of them
    /// moved. `false` means the values in force *are* the targets, bit for bit, so a caller can
    /// skip the work of putting them into effect until the next change arrives.
    ///
    /// A [gated](TuningPolicy::StepUpGateDown) word -- the budget's velocity and acceleration --
    /// is not filtered at all: its target is raised to a floor one cycle of the next order's
    /// limit below where it is, so raising it is a step and lowering it a ramp. The rate is read
    /// off the values in force at the top of the cycle, so the order the words are walked in
    /// cannot change the answer.
    ///
    /// The realtime half of the tuning path: no allocation, no lock, no fallible call, and
    /// `Self::WORDS` multiply-adds whatever the targets are. `gains` comes from
    /// [`slew_gains`](Self::slew_gains) so that no `exp` runs here.
    ///
    /// It walks the word array rather than the named fields, which is what makes it impossible
    /// for a field to be slewed under another field's policy, or left out.
    ///
    /// ```
    /// use franka::robot::target_control::{LiveTuning, TuningPolicy};
    /// # use franka::{ImpedanceOptions, TargetControlOptions};
    /// # let options = TargetControlOptions::default();
    /// # let impedance = ImpedanceOptions::cartesian();
    /// let mut current =
    ///     LiveTuning::from_options(&impedance, options.limits, options.rotation_limits);
    /// let target = LiveTuning {
    ///     ik_damping: current.ik_damping * 2.0,
    ///     ..current
    /// };
    /// let gains = LiveTuning::slew_gains(0.001);
    ///
    /// // One cycle of a 0.3 s time constant is a third of a percent of the gap, not the gap.
    /// assert!(current.slew_toward(&target, &gains));
    /// assert!(current.ik_damping < impedance.ik.damping * 1.01);
    ///
    /// // Converged, it reports that it has nothing left to do.
    /// for _ in 0..20_000 {
    ///     current.slew_toward(&target, &gains);
    /// }
    /// assert_eq!(current, target);
    /// assert!(!current.slew_toward(&target, &gains));
    /// ```
    pub fn slew_toward(&mut self, target: &LiveTuning, gains: &SlewGains) -> bool {
        let (mut words, target) = (self.to_words(), target.to_words());
        // The values in force at the top of the cycle: a gated word descends no faster than the
        // word `BOUNDS` names as its rate, read from this snapshot rather than from `words`, so
        // that the answer cannot depend on the order the words are walked in.
        let before = words;
        let mut moved = false;
        for word in 0..Self::WORDS {
            let goal = match RATE_WORD[word] {
                Some(rate) => target[word].max(words[word] - before[rate] * gains.dt),
                None => target[word],
            };
            let next = slewed(words[word], goal, gains.gain[word]);
            moved |= next.to_bits() != words[word].to_bits();
            words[word] = next;
        }
        if moved {
            *self = Self::from_words(&words);
        }
        moved
    }
}

/// `current` one cycle of `gain` toward `target`.
///
/// The filter approaches its target and never arrives, so the last few ulps are a step too small
/// for the float to carry: landing on the target on the cycle it can no longer move is what lets
/// [`LiveTuning::slew_toward`]'s `false` mean "the target is in force" rather than "near enough".
/// A `gain` of 1 or more is the whole gap and lands in one cycle by the same rule.
///
/// It guards nothing against a non-finite value, because none reaches it: a seed's gains are
/// finite by [`ImpedanceOptions::validate`](super::super::ImpedanceOptions::validate) and its
/// budget by `Otg::new`, both before the loop is built, and a target's every carried word is
/// checked finite by [`LiveTuning::apply_update`]. A NaN here would report a move on every cycle
/// for the session's life.
fn slewed(current: f64, target: f64, gain: f64) -> f64 {
    if gain >= 1.0 {
        return target;
    }
    let next = current + gain * (target - current);
    if next.to_bits() == current.to_bits() {
        target
    } else {
        next
    }
}
