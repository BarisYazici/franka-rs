//! How far each scalar of a [`LiveTuning`] may be moved, and how the loop is to move it.
//!
//! This table is the gate, not a description of one: [`LiveTuning::apply_update`] clamps by
//! walking it, so a caller that serialises it into a schema is serialising the code that runs.

use super::super::MIN_FEEDFORWARD_CUTOFF;
use super::LiveTuning;

/// How the loop is to move an accepted value into effect. Carried here rather than in the
/// loop so that a published schema can explain a slider's behaviour without a second table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TuningPolicy {
    /// First-order slew toward the target over `tau` seconds. The parameter multiplies a
    /// generally-nonzero state, so a step in it is a step in the torque.
    Slew {
        /// The time constant, s.
        tau: f64,
    },
    /// Applied whole on the cycle it arrives: either nothing nonzero is multiplied, or the
    /// product is bounded by the ripple the parameter exists to shape.
    Step,
    /// Raised on the cycle it arrives, lowered no faster than the next order's limit: the
    /// budget's velocity at the acceleration in force, its acceleration at the jerk.
    ///
    /// The generator re-plans continuously into a *wider* budget -- it stores no plan, only
    /// `(p, v, a, target, limits)` -- but it clamps its stored velocity and acceleration into a
    /// *narrower* one, on the next cycle's re-anchoring and again at the end of every step
    /// ([`Otg::set_limits`](crate::otg::Otg::set_limits) says where). A velocity truncated by
    /// `dv` in one cycle is `dv / dt` of acceleration in the command, which is an impulse
    /// rather than a re-plan. Descending no faster than the next order holds each clamp to one
    /// cycle of that order, which is the generator's own bound.
    ///
    /// [`LiveTuning::slew_toward`] runs the gate on the budget as a norm, and whoever hands the
    /// generator its per-axis limits scales the gated value. That is the same walk: the scaling
    /// is one positive factor per axis, so it carries the rate with it.
    StepUpGateDown {
        /// Which word's value in force the descent is measured at: the index into
        /// [`LiveTuning::BOUNDS`] of the next order of the same budget, whose unit is this
        /// word's per second. Named here rather than left to the reader as "the next word",
        /// because a caller that watches a descent -- [`descent`](Self::descent), a published
        /// schema, a panel drawing the ramp -- would otherwise have to know that rule too, and
        /// then there would be two of it.
        rate_word: usize,
    },
}

impl TuningPolicy {
    /// Which word of [`LiveTuning::to_words`] holds this one's descent rate, for the gated
    /// words; `None` for every other policy, which has no ramp.
    pub fn rate_word(self) -> Option<usize> {
        match self {
            TuningPolicy::StepUpGateDown { rate_word } => Some(rate_word),
            _ => None,
        }
    }
}

/// The slew time constant of every slewed field, s.
pub const SLEW_TAU: f64 = 0.3;

/// One scalar of [`LiveTuning`]: what it is called, what it means, and how far it may go.
///
/// Read, never built, outside this crate: `#[non_exhaustive]` so that the fields
/// a later parameter needs can be added without a major version.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct FieldBound {
    /// The field's name, as a caller's schema and an update name it.
    pub name: &'static str,
    /// The element, for the array fields; `None` for the scalars.
    pub index: Option<usize>,
    /// The smallest value left in force. Anything below it is clamped up to it, negatives
    /// included — the values that are refused instead of clamped are the ones
    /// [`reject_non_positive`](Self::reject_non_positive) marks. `joint_damping` has a second,
    /// higher floor that depends on the spring it damps
    /// ([`LiveTuning::joint_damping_floor`]).
    pub min: f64,
    /// The largest value left in force *for an update that carries this field*; anything above
    /// it is then clamped down to it. A field an update does not carry is never lowered, so a
    /// value seeded above this survives until someone moves it; [`LiveTuning::apply_update`] is
    /// the authority on both.
    pub max: f64,
    /// The unit, for a caller's schema; empty where the quantity is dimensionless.
    pub unit: &'static str,
    /// How the loop moves an accepted value into effect.
    pub policy: TuningPolicy,
    /// Whether the range spans enough decades that a caller should draw it logarithmically.
    pub log_slider: bool,
    /// Whether a value of zero or less is *rejected* rather than clamped up to `min`. Set
    /// where zero is not a soft version of the parameter but a different thing entirely, so
    /// that clamping would silently change the meaning of the request.
    pub reject_non_positive: bool,
    /// Which set of controls this one belongs beside. Presentation, and here so that a
    /// publisher of this table needs no field list of its own: the moment one exists, a field
    /// added here is a field that table forgets.
    pub group: &'static str,
    /// How much care moving this one wants, if any.
    pub danger: Option<TuningDanger>,
    /// A value below [`max`](Self::max) that a slider may usefully stop at, where the top of
    /// the range is reachable but rarely wanted; `None` draws the whole range. The bound is
    /// still `max`: this narrows the slider, never the gate.
    pub slider_max: Option<f64>,
    /// The value at which this parameter means "off" rather than "very small" or "very large",
    /// for a caller that wants to say so beside the control: the cutoff that is the filter's
    /// off switch, the gain that is the term's.
    pub off_at: Option<f64>,
    /// Whether this word is one axis of a budget expressed as a *norm* rather than per axis,
    /// so a caller shows the per-axis share beside it. The [OTG's first
    /// rule](crate::otg#in-a-control-loop) is why the difference matters.
    pub norm: bool,
}

/// How much care moving a [`FieldBound`] wants. Advice for the operator, enforced by whoever
/// publishes the table -- this crate clamps, and has no opinion beyond the clamp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TuningDanger {
    /// Worth a second look: the field reaches the torque directly.
    Advise,
    /// Above this value the field widens what the arm may do, so a publisher should ask for a
    /// deliberate confirmation *on the way up* -- the value stored crossing the threshold from
    /// at or below it. Already past it, moving within or down, there is nothing new to confirm.
    ConfirmAbove(f64),
}

impl FieldBound {
    const fn new(
        name: &'static str,
        index: Option<usize>,
        group: &'static str,
        min: f64,
        max: f64,
        unit: &'static str,
        policy: TuningPolicy,
    ) -> FieldBound {
        FieldBound {
            name,
            index,
            min,
            max,
            unit,
            policy,
            log_slider: false,
            reject_non_positive: false,
            group,
            danger: None,
            slider_max: None,
            off_at: None,
            norm: false,
        }
    }

    const fn log(mut self) -> FieldBound {
        self.log_slider = true;
        self
    }

    const fn non_zero(mut self) -> FieldBound {
        self.reject_non_positive = true;
        self
    }

    const fn advise(mut self) -> FieldBound {
        self.danger = Some(TuningDanger::Advise);
        self
    }

    const fn confirm_above(mut self, value: f64) -> FieldBound {
        self.danger = Some(TuningDanger::ConfirmAbove(value));
        self
    }

    const fn slider_max(mut self, value: f64) -> FieldBound {
        self.slider_max = Some(value);
        self
    }

    const fn off_at(mut self, value: f64) -> FieldBound {
        self.off_at = Some(value);
        self
    }

    const fn norm(mut self) -> FieldBound {
        self.norm = true;
        self
    }
}

const SLEW: TuningPolicy = TuningPolicy::Slew { tau: SLEW_TAU };

/// The law's springs and dampers.
const GAINS: &str = "gains";
/// The goal velocity's weight and the filter on it.
const FEEDFORWARD: &str = "feedforward";
/// What the inverse kinematics does with a redundant arm.
const IK: &str = "ik";
/// How fast the plan may move, which is what the operator feels first.
const ENVELOPE: &str = "envelope";

impl LiveTuning {
    /// How far each scalar may be moved, in the order of [`to_words`](Self::to_words).
    ///
    /// Publish this; never copy the numbers, and publish
    /// [`joint_damping_floor`](Self::joint_damping_floor) with it. The joint damping ceilings
    /// are the reference joint-impedance stack's, less the crate's own velocity barrier gain,
    /// which is added on top of the law's damping and is what the stability bound applies to,
    /// and lower still on the wrist; the budget ceilings sit just past where the robot's own
    /// joint-velocity guard can fire.
    pub const BOUNDS: &'static [FieldBound] = &{
        let blank = FieldBound::new("", None, "", 0.0, 0.0, "", TuningPolicy::Step);
        let mut rows = [blank; Self::WORDS];
        let mut i = 0;
        while i < 7 {
            // Above the preset's own ceiling the torque clamp binds before the spring does.
            rows[i] = FieldBound::new(
                "joint_stiffness",
                Some(i),
                GAINS,
                0.0,
                1200.0,
                "Nm/rad",
                SLEW,
            )
            .advise();
            // The ceiling is 60 rather than the reference joint-impedance stack's 80 because
            // near the joint's velocity limit `VELOCITY_BARRIER_GAIN` is added to this, and it
            // is the sum the barrier's stability rule bounds: `K × 1 ms / I < 1` (the book's
            // *The impedance backend*, "The joint velocity envelope"). The wrist (joints 5 to 7)
            // gets 40: on its 0.074 kg m², 60 + 20 gives 1.08 and 40 + 20 gives 0.81.
            // Away from that limit the barrier contributes nothing, so this `min` of zero is
            // not the floor in force: `joint_damping_floor` is.
            let ceiling = if i < 4 { 60.0 } else { 40.0 };
            rows[7 + i] = FieldBound::new(
                "joint_damping",
                Some(i),
                GAINS,
                0.0,
                ceiling,
                "Nm s/rad",
                SLEW,
            )
            .advise();
            i += 1;
        }
        // Four times the preset; past it the joint-side IK acceleration binds, not the spring.
        // Zero is rejected: the derived damping scales with its square root, so a zero or
        // negative stiffness is not a soft spring but a different sign.
        rows[14] = FieldBound::new(
            "cartesian_stiffness",
            None,
            GAINS,
            50.0,
            3000.0,
            "N/m",
            SLEW,
        )
        .log()
        .non_zero()
        .advise();
        // The damped-least-squares gain is bounded by 1/2λ, so the floor is already a 500-fold
        // amplification near a singularity. Zero is rejected rather than clamped: the IK errors
        // on it, and snapping it to the floor would silently answer a different request.
        rows[15] = FieldBound::new("ik_damping", None, IK, 1e-3, 1.0, "", SLEW)
            .log()
            .non_zero();
        // Zero is a valid, and used, setting -- it is the posture term switched off. The pull is
        // capped at `MAX_POSTURE_RATE` whatever the gain, so the ceiling only bounds how fast
        // the cap is reached, and a slider spanning the useful end is worth more than one that
        // reaches it.
        rows[16] = FieldBound::new("ik_nullspace_gain", None, IK, 0.0, 20.0, "1/s", SLEW)
            .slider_max(5.0)
            .off_at(0.0);
        rows[17] = FieldBound::new(
            "velocity_feedforward_gain",
            None,
            FEEDFORWARD,
            0.0,
            1.0,
            "",
            SLEW,
        )
        .confirm_above(0.0)
        .off_at(0.0);
        // The library's own floor and ceiling, taken from it rather than restated: the ceiling
        // is the filter's off switch, and below the floor the filter is a near-integrator.
        rows[18] = FieldBound::new(
            "velocity_feedforward_cutoff",
            None,
            FEEDFORWARD,
            MIN_FEEDFORWARD_CUTOFF,
            crate::lowpass_filter::MAX_CUTOFF_FREQUENCY,
            "Hz",
            TuningPolicy::Step,
        )
        .log()
        .off_at(crate::lowpass_filter::MAX_CUTOFF_FREQUENCY);
        // A gated word descends at the next word's value, which is the next order of the same
        // budget; `rate_word` says so rather than leaving it to be rediscovered.
        const fn gate(word: usize) -> TuningPolicy {
            TuningPolicy::StepUpGateDown {
                rate_word: word + 1,
            }
        }
        // The robot's own joint-velocity guard can fire near a translational budget of
        // [1.0, 15, 600], depending on pose and path, so these ceilings sit just past that edge
        // with the guard behind them. The confirmations sit lower, at `[0.85, 10, 400]`, the
        // fastest set the stack has been validated at: going past it is a decision rather than a
        // drag of a slider. The rotational set is the
        // same fractions of its own ceilings.
        rows[19] = FieldBound::new("budget", Some(0), ENVELOPE, 0.05, 1.2, "m/s", gate(19))
            .confirm_above(0.85)
            .norm();
        rows[20] = FieldBound::new("budget", Some(1), ENVELOPE, 0.1, 20.0, "m/s^2", gate(20))
            .confirm_above(10.0)
            .norm();
        rows[21] = FieldBound::new(
            "budget",
            Some(2),
            ENVELOPE,
            1.0,
            800.0,
            "m/s^3",
            TuningPolicy::Step,
        )
        .confirm_above(400.0)
        .norm();
        rows[22] = FieldBound::new(
            "rotation_budget",
            Some(0),
            ENVELOPE,
            0.05,
            2.5,
            "rad/s",
            gate(22),
        )
        .confirm_above(1.0)
        .norm();
        rows[23] = FieldBound::new(
            "rotation_budget",
            Some(1),
            ENVELOPE,
            0.1,
            30.0,
            "rad/s^2",
            gate(23),
        )
        .confirm_above(15.0)
        .norm();
        rows[24] = FieldBound::new(
            "rotation_budget",
            Some(2),
            ENVELOPE,
            1.0,
            800.0,
            "rad/s^3",
            TuningPolicy::Step,
        )
        .confirm_above(400.0)
        .norm();
        rows
    };

    /// The least damping, Nm s/rad, a joint spring of `stiffness` Nm/rad is allowed to run
    /// with. [`apply_update`](Self::apply_update) raises `joint_damping[i]` to this, and a
    /// caller that publishes [`BOUNDS`](Self::BOUNDS) has to publish this rule beside it: it
    /// is the one limit in this file that a per-field row cannot express.
    ///
    /// Why it exists: the pair is the only one where both halves are exposed and independent,
    /// and `joint_damping`'s own floor is zero, so without it one slider move reaches a spring
    /// of up to 1200 Nm/rad with no damping at all. The
    /// [`VELOCITY_BARRIER_GAIN`](super::super::VELOCITY_BARRIER_GAIN) the damping *ceiling* is derived
    /// against is no help here — it only engages beyond
    /// [`velocity_barrier_fraction`](super::super::ImpedanceOptions::velocity_barrier_fraction) of the
    /// joint's velocity limit, so at working speed it contributes nothing, and what is left is
    /// an undamped spring behind a low-pass that adds lag rather than damping.
    ///
    /// [`MIN_JOINT_DAMPING_RATIO`] is below every shipped preset's own `D / sqrt(K)`, so this
    /// forbids the corner and no configuration anyone runs today; it is not a tuning opinion
    /// and it is far enough under the ceiling never to fight it.
    pub fn joint_damping_floor(stiffness: f64) -> f64 {
        MIN_JOINT_DAMPING_RATIO * stiffness.sqrt()
    }
}

/// The least `joint_damping[i] / sqrt(joint_stiffness[i])` a running session is left at. The
/// shipped presets sit between 0.32 ([`ImpedanceGains::CARTESIAN`](super::super::ImpedanceGains)'s
/// wrist) and 2.12 (`JOINT`'s), so this is under every one of them with room to spare: it
/// forbids the undamped corner and nothing else. Like every number in
/// [`LiveTuning::BOUNDS`](LiveTuning::BOUNDS) it is a derivation on paper until it is swept.
pub const MIN_JOINT_DAMPING_RATIO: f64 = 0.25;

// The clamp walks `BOUNDS` against the word array, so the two must describe the same
// scalars in the same order. This catches a field bounded twice or not at all.
const _: () = assert!(LiveTuning::BOUNDS.len() == LiveTuning::WORDS);

// Every row is a row a publisher can render, and every gated word's rate is a word that
// exists. A row left half-built -- the blank the table starts from -- fails here rather than
// reaching a schema as a field with no group and a descent rate off the end of the array.
const _: () = {
    let mut word = 0;
    while word < LiveTuning::WORDS {
        assert!(
            !LiveTuning::BOUNDS[word].name.is_empty() && !LiveTuning::BOUNDS[word].group.is_empty(),
            "a bound with no name or no group"
        );
        if let TuningPolicy::StepUpGateDown { rate_word } = LiveTuning::BOUNDS[word].policy {
            assert!(
                rate_word < LiveTuning::WORDS,
                "a descent rate off the table"
            );
            // Strictly after, which rules out a word descending at its own rate and buys the
            // crossing a second property: `slew_toward` walks the words in this order, so a
            // rate word sitting after the word it paces has not moved yet when it is read.
            // Its snapshot of the values in force is then provably the values in force, for
            // this table, rather than a defence whose only evidence is the reader's argument.
            // The snapshot stays: it is what keeps that true of a table ordered differently.
            assert!(rate_word > word, "a descent rate at or before its own word");
        }
        word += 1;
    }
};
