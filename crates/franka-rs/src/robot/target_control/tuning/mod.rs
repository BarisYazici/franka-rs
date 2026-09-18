//! What a running [target control](super) session will accept a change to, and how far.
//!
//! [`LiveTuning`] is the set of control-law and plan parameters an operator may move while the
//! arm is engaged; [`LiveTuning::BOUNDS`] is how far each of them may be moved. The bounds are
//! a *second, tighter* gate than [`ImpedanceOptions::validate`](super::ImpedanceOptions::validate)
//! and [`IkOptions::validate`](super::IkOptions::validate), which answer a different question:
//! `validate` asks whether a session can be constructed at all and must stay permissive enough
//! for every caller of the crate, while `BOUNDS` asks whether a *running* session may be moved
//! somewhere by hand. Widening `validate` to carry this policy would impose it on every
//! consumer; narrowing this table to `validate`'s permissiveness would let a slider reach 1e9.
//!
//! `BOUNDS` is the only description of the limits: it is the table the clamp itself walks, so
//! a caller that publishes it — a schema for a panel, a check in another process — cannot
//! drift from what is enforced. There must not be a second copy of these numbers anywhere.
//! One limit does not fit a per-field row, the floor a joint spring puts under its own
//! damping, and it has a function of its own ([`LiveTuning::joint_damping_floor`]); publishing
//! the table means publishing that rule with it.
//!
//! [`LiveTuning::apply_update`] belongs to the arm thread. It allocates — a `Vec` of the
//! bounds that bit, a message on a refusal — and must not be called from the 1 kHz loop, which
//! only ever reads a `LiveTuning` that has already been through it.
//!
//! Every field is an `f64` and the struct is [`Copy`], so a rejected update costs nothing to
//! discard and the whole set packs into a word array ([`LiveTuning::to_words`]) with no
//! rounding and no layout question. The table itself is in [`bounds`].
//!
//! What the loop does with an accepted set is in [`slew`]: it seeds its own copy from the
//! session's options ([`LiveTuning::from_options`]), moves that copy toward the published
//! targets a cycle at a time ([`LiveTuning::slew_toward`]), and writes it into the law through
//! [`LiveTuning::gains`]. The published set is therefore a set of *targets*, and a cycle that
//! misses a read of it loses one cycle of the crossing, not a change.

use super::{ImpedanceGains, ImpedanceOptions};
use crate::error::{FrankaError, FrankaResult};
use crate::otg::OtgLimits;

mod bounds;
mod slew;

pub use bounds::{FieldBound, TuningDanger, TuningPolicy, MIN_JOINT_DAMPING_RATIO, SLEW_TAU};
pub use slew::SlewGains;

/// The targets a running session will accept. These are targets, not the values in force: the
/// loop keeps its own current values and moves them here under each field's
/// [`policy`](FieldBound::policy).
///
/// `cartesian_stiffness` is one number here and twelve in the law. The operator sets a single
/// translational stiffness; whoever owns the preset derives the six stiffnesses and six
/// dampings from it, which is what holds the damping ratio the preset was tuned for. Neither
/// the panel nor this struct should know that rule, and it is not duplicated here.
///
/// Deliberately *not* `#[non_exhaustive]`, unlike [`FieldBound`] and [`TuningUpdate`]: a
/// consumer builds one of these — it is the session's seed, and it is what an update is
/// validated against. Closing the struct expression would leave the positional
/// [`from_words`](Self::from_words) as the only way in, which is the mis-indexing hazard this
/// module's tests exist to catch, promoted to the sole public constructor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiveTuning {
    /// `Kq`, per joint, Nm/rad.
    pub joint_stiffness: [f64; 7],
    /// `Kqd`, per joint, Nm s/rad.
    pub joint_damping: [f64; 7],
    /// The translational Cartesian stiffness the six-axis `Kx` and `Kxd` are derived from, N/m.
    pub cartesian_stiffness: f64,
    /// `λ` of the IK's damped least squares.
    pub ik_damping: f64,
    /// Gain, 1/s, of the IK's nullspace pull toward the posture.
    pub ik_nullspace_gain: f64,
    /// The weight on the goal velocity in the damping term, within [0, 1]; zero is the law
    /// with the feedforward off.
    pub velocity_feedforward_gain: f64,
    /// The low-pass cutoff on that goal velocity, Hz; the maximum is the filter off.
    pub velocity_feedforward_cutoff: f64,
    /// The translational budget as a norm: m/s, m/s^2, m/s^3.
    pub budget: [f64; 3],
    /// The rotational budget as a norm: rad/s, rad/s^2, rad/s^3.
    pub rotation_budget: [f64; 3],
}

// Trivially copyable, which is what makes the scratch copy of an update free and the word
// packing a pure bit-pun. Rust has no `is_trivially_copyable`; this is the equivalent.
const _: fn() = || {
    fn assert<T: Copy + Send + Sync + 'static>() {}
    assert::<LiveTuning>();
};
// One word per scalar: adding a field without extending `WORDS` fails to compile.
const _: () = assert!(std::mem::size_of::<LiveTuning>() == LiveTuning::WORDS * 8);

impl LiveTuning {
    /// The number of scalars, and the length of both [`to_words`](Self::to_words) and
    /// [`BOUNDS`](Self::BOUNDS).
    pub const WORDS: usize = 25;

    /// The scalars in the order of [`BOUNDS`](Self::BOUNDS).
    pub fn to_words(&self) -> [f64; Self::WORDS] {
        let mut words = [0.0; Self::WORDS];
        words[..7].copy_from_slice(&self.joint_stiffness);
        words[7..14].copy_from_slice(&self.joint_damping);
        words[14] = self.cartesian_stiffness;
        words[15] = self.ik_damping;
        words[16] = self.ik_nullspace_gain;
        words[17] = self.velocity_feedforward_gain;
        words[18] = self.velocity_feedforward_cutoff;
        words[19..22].copy_from_slice(&self.budget);
        words[22..25].copy_from_slice(&self.rotation_budget);
        words
    }

    /// The inverse of [`to_words`](Self::to_words).
    pub fn from_words(words: &[f64; Self::WORDS]) -> LiveTuning {
        LiveTuning {
            joint_stiffness: std::array::from_fn(|i| words[i]),
            joint_damping: std::array::from_fn(|i| words[7 + i]),
            cartesian_stiffness: words[14],
            ik_damping: words[15],
            ik_nullspace_gain: words[16],
            velocity_feedforward_gain: words[17],
            velocity_feedforward_cutoff: words[18],
            budget: std::array::from_fn(|i| words[19 + i]),
            rotation_budget: std::array::from_fn(|i| words[22 + i]),
        }
    }

    /// A session's seed: the values `impedance` and the two budgets are already running with,
    /// which is what the loop slews *from*, so that the first change an operator makes starts
    /// where the arm is and not at a default.
    ///
    /// The budgets are the arguments'; of what `impedance` supplies, two words are not one of
    /// its fields read straight off.
    ///
    /// `cartesian_stiffness` is the first axis of
    /// [`ImpedanceGains::cartesian_stiffness`](super::ImpedanceGains::cartesian_stiffness),
    /// because that is the number
    /// [`scaled_cartesian`](super::ImpedanceGains::scaled_cartesian) scales, and
    /// [`gains`](Self::gains) derives all twelve Cartesian words back from it. A gains set that
    /// is *not* a scaled [`CARTESIAN`](super::ImpedanceGains::CARTESIAN) therefore loses
    /// whatever else it changed the moment anything is retuned: seeded from
    /// [`DROID`](super::ImpedanceGains::DROID), the first retune moves the translational
    /// damping from DROID's 37 to the preset's 50 and 90. That is the parity preset's price for
    /// one slider, and the reason `gains` is the only mapping.
    ///
    /// `velocity_feedforward_gain` is zero when
    /// [`velocity_feedforward`](super::ImpedanceOptions::velocity_feedforward) is off, which is
    /// the same law: the weighted goal velocity is all the switch gates. Carrying the switch in
    /// the gain is what lets it be crossed continuously rather than thrown.
    pub fn from_options(
        impedance: &ImpedanceOptions,
        budget: OtgLimits,
        rotation_budget: OtgLimits,
    ) -> LiveTuning {
        let norm = |limits: OtgLimits| {
            [
                limits.max_velocity,
                limits.max_acceleration,
                limits.max_jerk,
            ]
        };
        LiveTuning {
            joint_stiffness: impedance.gains.joint_stiffness,
            joint_damping: impedance.gains.joint_damping,
            cartesian_stiffness: impedance.gains.cartesian_stiffness[0],
            ik_damping: impedance.ik.damping,
            ik_nullspace_gain: impedance.ik.nullspace_gain,
            velocity_feedforward_gain: if impedance.velocity_feedforward {
                impedance.velocity_feedforward_gain
            } else {
                0.0
            },
            velocity_feedforward_cutoff: impedance.velocity_feedforward_cutoff,
            budget: norm(budget),
            rotation_budget: norm(rotation_budget),
        }
    }

    /// The two budgets as [`OtgLimits`], which is the form a generator takes them in: the
    /// other side of [`from_options`](Self::from_options)'s reading of them, and the only
    /// mapping, so that nobody has to remember which of the three words is which.
    ///
    /// Still norms, as the fields are. Whoever hands them to a generator owes them the
    /// per-axis division the [OTG's first rule](crate::otg#in-a-control-loop) asks for.
    pub fn budgets(&self) -> (OtgLimits, OtgLimits) {
        let limits = |[max_velocity, max_acceleration, max_jerk]: [f64; 3]| OtgLimits {
            max_velocity,
            max_acceleration,
            max_jerk,
        };
        (limits(self.budget), limits(self.rotation_budget))
    }

    /// The law's gains these targets stand for: the joint springs as they are, and the six
    /// Cartesian stiffnesses and six dampings
    /// [`scaled_cartesian`](super::ImpedanceGains::scaled_cartesian) derives from the one
    /// translational stiffness, which is where that rule lives and the only place it may.
    ///
    /// The other side of [`from_options`](Self::from_options)'s reading of the gains, and the
    /// mapping the loop applies every cycle it moves.
    pub fn gains(&self) -> ImpedanceGains {
        ImpedanceGains {
            joint_stiffness: self.joint_stiffness,
            joint_damping: self.joint_damping,
            ..ImpedanceGains::scaled_cartesian(self.cartesian_stiffness)
        }
    }

    /// Applies every field `update` carries and returns the bounds that had to clamp one,
    /// leaving `self` at the accepted, clamped values. Runs on the arm thread: it allocates,
    /// and must not be called from the loop.
    ///
    /// All or nothing: the update is built on a scratch copy, and if any value it carries is
    /// rejected — not finite, or zero where zero means something else
    /// ([`reject_non_positive`](FieldBound::reject_non_positive)) — `self` is left bit for bit
    /// as it was and nothing is clamped.
    ///
    /// A field the update does not carry keeps its current value and is not re-checked, so a
    /// partial update is a real partial update and cannot quietly retune a field nobody moved.
    /// A value seeded outside [`BOUNDS`](Self::BOUNDS) therefore survives until someone moves
    /// that field; whoever builds the seed has to check it against `BOUNDS` first, and the way
    /// to do that is this method with an update carrying every field, never a second copy of
    /// the table.
    ///
    /// The pair rule is the one thing that reaches past the carried fields. An update carrying
    /// `joint_stiffness` raises any of the seven `joint_damping` values that sit under
    /// [`joint_damping_floor`](Self::joint_damping_floor) of the spring they damp — the pair,
    /// not either half, is what has to stay damped. A damping the update does not carry is
    /// only ever *raised*: its ceiling is not applied, because lowering a field nobody moved is
    /// the thing this method exists not to do. The floor is read off the stiffness `BOUNDS`
    /// allows rather than the raw seeded one, so a seed the table would never have accepted
    /// cannot push the floor above the ceiling.
    ///
    /// Clamping is deliberate rather than a refusal, and the clamped bounds come back so that
    /// a caller can show the value it will actually get; that echo is also what would expose a
    /// caller whose own idea of the limits has drifted from `BOUNDS`. Read the value out of
    /// `self`, not out of the bound: where the pair rule bit, the value in force is the floor
    /// and `FieldBound::min` is still zero.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] naming the field, with `self` unchanged.
    pub fn apply_update(
        &mut self,
        update: &TuningUpdate,
    ) -> FrankaResult<Vec<&'static FieldBound>> {
        let carried = update.carried();
        let mut words = update.overlay(self).to_words();
        for word in (0..Self::WORDS).filter(|w| carried[*w]) {
            let (value, bound) = (words[word], &Self::BOUNDS[word]);
            if !value.is_finite() {
                return Err(reject(bound, value, "must be finite"));
            }
            if bound.reject_non_positive && value <= 0.0 {
                return Err(reject(bound, value, "must be positive"));
            }
        }
        // An uncarried word was never checked, so anything at all can reach the clamp; `snap`
        // leaves a value it cannot order alone. The stiffnesses go first so the damping floor
        // is read off what will be in force.
        let mut clamped = Vec::new();
        for joint in (0..7).filter(|j| carried[*j]) {
            let bound = &Self::BOUNDS[joint];
            clamped.extend(snap(&mut words[joint], bound, bound.min, bound.max));
        }
        for joint in (0..7).filter(|j| carried[*j] || carried[7 + *j]) {
            let spring = &Self::BOUNDS[joint];
            // Off the bounded stiffness: an uncarried one is unclamped and a seeded 60 000
            // would otherwise put the floor above the damping's own ceiling.
            let floor = Self::joint_damping_floor(words[joint].clamp(spring.min, spring.max));
            let bound = &Self::BOUNDS[7 + joint];
            let ceiling = if carried[7 + joint] {
                bound.max
            } else {
                f64::INFINITY
            };
            clamped.extend(snap(
                &mut words[7 + joint],
                bound,
                floor.max(bound.min),
                ceiling,
            ));
        }
        for word in (14..Self::WORDS).filter(|w| carried[*w]) {
            let bound = &Self::BOUNDS[word];
            clamped.extend(snap(&mut words[word], bound, bound.min, bound.max));
        }
        *self = Self::from_words(&words);
        Ok(clamped)
    }
}

/// `word` into `[low, high]`, naming `bound` if that moved it. Callers keep `low <= high`, and
/// a NaN comes back untouched and unreported: an uncarried word is nobody's request to change.
fn snap(
    word: &mut f64,
    bound: &'static FieldBound,
    low: f64,
    high: f64,
) -> Option<&'static FieldBound> {
    let limited = word.clamp(low, high);
    if limited.to_bits() == word.to_bits() {
        None
    } else {
        *word = limited;
        Some(bound)
    }
}

fn reject(bound: &FieldBound, value: f64, what: &str) -> FrankaError {
    let index = match bound.index {
        Some(i) => format!("[{i}]"),
        None => String::new(),
    };
    FrankaError::InvalidArgument(format!(
        "target control: tuning {}{index} {what}, got {value}",
        bound.name
    ))
}

/// A partial change to a [`LiveTuning`]: every field left `None` keeps the value it has.
///
/// `#[non_exhaustive]`: outside this crate there is no struct expression for it at all —
/// functional update syntax included — so start from [`Default`] and assign what the update
/// carries.
///
/// ```
/// use franka::robot::target_control::TuningUpdate;
///
/// let mut update = TuningUpdate::default();
/// update.ik_damping = Some(0.2);
///
/// assert_eq!(update.ik_damping, Some(0.2));
/// assert_eq!(update.budget, None);
/// ```
///
/// The form that reads as though it should work does not, and this is the gate that says so:
///
/// ```compile_fail,E0639
/// use franka::robot::target_control::TuningUpdate;
///
/// let update = TuningUpdate {
///     ik_damping: Some(0.2),
///     ..TuningUpdate::default()
/// };
/// ```
///
/// The attribute buys the freedom to stop *reading* a field, not the freedom to add one: a new
/// `TuningUpdate` field needs the matching [`LiveTuning`] field, which moves
/// [`WORDS`](LiveTuning::WORDS) and the signature of [`to_words`](LiveTuning::to_words), and
/// that is a major version either way.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[non_exhaustive]
pub struct TuningUpdate {
    /// See [`LiveTuning::joint_stiffness`].
    pub joint_stiffness: Option<[f64; 7]>,
    /// See [`LiveTuning::joint_damping`].
    pub joint_damping: Option<[f64; 7]>,
    /// See [`LiveTuning::cartesian_stiffness`].
    pub cartesian_stiffness: Option<f64>,
    /// See [`LiveTuning::ik_damping`].
    pub ik_damping: Option<f64>,
    /// See [`LiveTuning::ik_nullspace_gain`].
    pub ik_nullspace_gain: Option<f64>,
    /// See [`LiveTuning::velocity_feedforward_gain`].
    pub velocity_feedforward_gain: Option<f64>,
    /// See [`LiveTuning::velocity_feedforward_cutoff`].
    pub velocity_feedforward_cutoff: Option<f64>,
    /// See [`LiveTuning::budget`].
    pub budget: Option<[f64; 3]>,
    /// See [`LiveTuning::rotation_budget`].
    pub rotation_budget: Option<[f64; 3]>,
}

impl TuningUpdate {
    /// Which words of [`LiveTuning::to_words`] this update carries.
    fn carried(&self) -> [bool; LiveTuning::WORDS] {
        let mut carried = [false; LiveTuning::WORDS];
        carried[..7].fill(self.joint_stiffness.is_some());
        carried[7..14].fill(self.joint_damping.is_some());
        carried[14] = self.cartesian_stiffness.is_some();
        carried[15] = self.ik_damping.is_some();
        carried[16] = self.ik_nullspace_gain.is_some();
        carried[17] = self.velocity_feedforward_gain.is_some();
        carried[18] = self.velocity_feedforward_cutoff.is_some();
        carried[19..22].fill(self.budget.is_some());
        carried[22..25].fill(self.rotation_budget.is_some());
        carried
    }

    /// `current` with every field this update carries written over it: the scratch copy.
    fn overlay(&self, current: &LiveTuning) -> LiveTuning {
        LiveTuning {
            joint_stiffness: self.joint_stiffness.unwrap_or(current.joint_stiffness),
            joint_damping: self.joint_damping.unwrap_or(current.joint_damping),
            cartesian_stiffness: self
                .cartesian_stiffness
                .unwrap_or(current.cartesian_stiffness),
            ik_damping: self.ik_damping.unwrap_or(current.ik_damping),
            ik_nullspace_gain: self.ik_nullspace_gain.unwrap_or(current.ik_nullspace_gain),
            velocity_feedforward_gain: self
                .velocity_feedforward_gain
                .unwrap_or(current.velocity_feedforward_gain),
            velocity_feedforward_cutoff: self
                .velocity_feedforward_cutoff
                .unwrap_or(current.velocity_feedforward_cutoff),
            budget: self.budget.unwrap_or(current.budget),
            rotation_budget: self.rotation_budget.unwrap_or(current.rotation_budget),
        }
    }
}
