//! The arm thread's end of `params/*`: what a `set` is allowed to do, what `get` and
//! `current` say the arm is running, and how much of a change is still on its way.
//!
//! The order a `set` runs in is the wire contract's: decode the shape, find the session,
//! check `base_version`, apply to a scratch copy, ask for a confirmation where the change
//! widens what the arm may do, and only then publish -- so a refusal at any step leaves the
//! running session bit for bit as it was. The gate itself is
//! [`LiveTuning::apply_update`](franka::robot::target_control::LiveTuning::apply_update),
//! reached through the session handle, which is the single writer of the tuning slot; this
//! module adds the protocol around it and keeps no second copy of the values.
//!
//! `slewing` is read off a clock, not off the loop. The arm thread knows what the values were
//! when a `set` was accepted, what they became, and what each field's policy does with a
//! change ([`TuningPolicy::remaining`](franka::robot::target_control::TuningPolicy::remaining)
//! for the filtered words, [`descent`](franka::robot::target_control::TuningPolicy::descent)
//! for the ramped ones), so nothing has to be published back out of the realtime thread.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use franka::robot::target_control::{FieldBound, LiveTuning, TuningDanger};
use log::{debug, info};

use super::machine::Machine;
use super::{JsonReply, RobotSide};
use crate::monotonic_ns;
use crate::msg::params::{
    fields, params_of, to_json, update_of, Clamped, Origin, ParamsMsg, ParsedUpdate, Reason,
    Rejection, SchemaMsg, SetReply, SetRequest, OWNER,
};

/// The `params/*` queryables. Not `cmd/*` verbs: a `CmdRequest` is four fixed fields with
/// `deny_unknown_fields` and a `CmdReply` is `{ok, error}`, and neither carries a parameter
/// set -- so tuning has its own keys and its own JSON rather than a smuggled field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamsVerb {
    /// The vocabulary: every tunable field's bounds, default, unit, policy and danger, plus the
    /// read-only `derived` block. Takes no payload.
    Schema,
    /// The values in force, with `version`, `origin`, `slewing` and `dirty`. Takes no payload.
    Get,
    /// A partial, all-or-nothing change.
    Set,
}

impl ParamsVerb {
    /// Every verb, in key order.
    pub const ALL: [ParamsVerb; 3] = [ParamsVerb::Schema, ParamsVerb::Get, ParamsVerb::Set];

    /// The last key segment of the verb's queryable.
    pub fn key(self) -> &'static str {
        match self {
            ParamsVerb::Schema => "schema",
            ParamsVerb::Get => "get",
            ParamsVerb::Set => "set",
        }
    }
}

/// How often `current` is published when nothing changes.
pub const CURRENT_PERIOD: Duration = Duration::from_secs(1);

/// The fraction below which a crossing is over as far as anyone watching can tell: half a
/// percent, under what the panel's whole-percent readout can draw. The filtered words approach
/// their target and never quite arrive, so without a floor `slewing` would never empty.
const SETTLED: f64 = 0.005;

/// The `set` that is still taking effect: the values it left, the values it asked for, and
/// when it was accepted. All three are needed to say what is left of it, and none of them is
/// the loop's own state.
struct Crossing {
    from: LiveTuning,
    to: LiveTuning,
    at: Instant,
}

/// The arm's `params/*` bookkeeping. The values themselves live in the session's slot; this is
/// what the wire needs around them.
#[derive(Default)]
pub(super) struct Params {
    /// Accepted `set`s since boot. Monotonic: a session ending reverts the values to the
    /// config's, which is a change a watcher must see, but never a version a `base_version`
    /// could match twice.
    version: u64,
    origin: Option<Origin>,
    crossing: Option<Crossing>,
    /// When `current` last went out.
    published: Option<Instant>,
}

impl Params {
    /// A `set` by `client` moved the values `from` -> `to`.
    fn accept(&mut self, client: u32, from: LiveTuning, to: LiveTuning) {
        self.version += 1;
        self.origin = Some(Origin {
            version: self.version,
            by: client,
            at_ns: monotonic_ns(),
        });
        self.crossing = Some(Crossing {
            from,
            to,
            at: Instant::now(),
        });
    }

    /// The values are no longer the ones any `set` produced: a session started from the config
    /// or ended back to it. The version stands -- it counts `set`s and nothing else -- but
    /// saying who set values that are no longer in force would be a lie.
    fn forget(&mut self) {
        self.origin = None;
        self.crossing = None;
    }
}

impl<R: RobotSide> Machine<R> {
    /// One `params/*` query: `payload` is the query's bytes, empty for the two that take none.
    pub(super) fn params_query(&mut self, verb: ParamsVerb, payload: &[u8], reply: JsonReply) {
        let json = match verb {
            ParamsVerb::Schema => to_json(&SchemaMsg::new(
                &self.config,
                self.robot.fci_version(),
                crate::boot_id(),
            )),
            ParamsVerb::Get => to_json(&self.params_msg()),
            ParamsVerb::Set => to_json(&self.params_set(payload)),
        };
        reply(json);
    }

    /// What `get` and `current` carry.
    fn params_msg(&self) -> ParamsMsg {
        let tuning = self.live_tuning();
        ParamsMsg {
            owner: OWNER,
            arm: self.config.name.clone(),
            version: self.params.version,
            boot_id: crate::boot_id().to_string(),
            t_node_ns: monotonic_ns(),
            origin: self.params.origin,
            params: params_of(&tuning),
            slewing: self.slewing(),
            dirty: tuning != self.config.live_tuning(),
        }
    }

    /// The tuning targets in force: the running session's, or -- with no session, or one whose
    /// tracking is not the client's to tune -- what the config says the next one will start at.
    pub(super) fn live_tuning(&self) -> LiveTuning {
        self.control
            .as_ref()
            .and_then(|control| control.tuning().ok())
            .unwrap_or_else(|| self.config.live_tuning())
    }

    /// Publishes `current`, and remembers when.
    pub(super) fn publish_params(&mut self) {
        self.params.published = Some(Instant::now());
        let message = self.params_msg();
        (self.publish_params_msg)(&message);
    }

    /// `current` on the tick after [`CURRENT_PERIOD`] without one; a change publishes at once.
    pub(super) fn publish_params_if_due(&mut self) {
        if self
            .params
            .published
            .is_none_or(|at| at.elapsed() >= CURRENT_PERIOD)
        {
            self.publish_params();
        }
    }

    /// The session's values are no longer a client's doing: it started from the config, or
    /// ended and left the config's behind. Publishes, because that is a change.
    pub(super) fn params_reseeded(&mut self) {
        self.params.forget();
        self.publish_params();
    }

    fn params_set(&mut self, payload: &[u8]) -> SetReply {
        match self.apply_set(payload) {
            Ok(reply) => reply,
            Err(rejection) => {
                debug!(
                    "arm {}: params set refused: {} ({})",
                    self.config.name,
                    rejection.error,
                    rejection.reason.as_str()
                );
                SetReply::refused(&rejection, self.params.version)
            }
        }
    }

    /// The wire contract's order, each step leaving nothing behind if it refuses.
    fn apply_set(&mut self, payload: &[u8]) -> Result<SetReply, Rejection> {
        let request: SetRequest = serde_json::from_slice(payload).map_err(|e| {
            Rejection::new(Reason::Type, None, format!("bad params/set request: {e}"))
        })?;
        if request.client_id == 0 {
            return Err(Rejection::new(
                Reason::Type,
                Some("client_id".into()),
                "client_id must be positive",
            ));
        }
        let current = self.session_tuning()?;
        if request
            .base_version
            .is_some_and(|at| at != self.params.version)
        {
            return Err(Rejection::new(
                Reason::Stale,
                None,
                format!(
                    "base_version {} is not the current {}",
                    request.base_version.unwrap_or_default(),
                    self.params.version
                ),
            ));
        }
        let parsed = update_of(&request.params)?;
        // What the gate would store, worked out before anything is stored, because the
        // confirmation is decided on the clamped value and a refusal here must change nothing.
        // The handle then runs the same gate on the same update for real, under its own writer
        // lock; the two cannot disagree, because this thread is the only caller of `tune` and
        // the only thing that can move the slot between these two lines.
        let mut scratch = current;
        scratch
            .apply_update(&parsed.update)
            .map_err(|e| Rejection::new(Reason::Invalid, None, e.to_string()))?;
        if let Some(bound) = unconfirmed(&current, &scratch, &request.confirm) {
            return Err(Rejection::new(
                Reason::NeedsConfirm,
                Some(bound.name.to_string()),
                format!(
                    "{} crosses {} from below; confirm it",
                    bound.name,
                    confirm_above(bound).unwrap_or_default()
                ),
            ));
        }
        let (clamped, stored) = {
            let control = self.control.as_ref().ok_or_else(no_session)?;
            let clamped = control
                .tune(&parsed.update)
                .map_err(|e| Rejection::new(Reason::Invalid, None, e.to_string()))?;
            let stored = control
                .tuning()
                .map_err(|e| Rejection::new(Reason::NotReady, None, e.to_string()))?;
            (clamped, stored)
        };
        self.params.accept(request.client_id, current, stored);
        info!(
            "arm {}: params version {} by {} ({} field{} moved)",
            self.config.name,
            self.params.version,
            request.client_id,
            request.params.len(),
            if request.params.len() == 1 { "" } else { "s" },
        );
        let reply = SetReply::accepted(
            self.params.version,
            params_of(&stored),
            clamped_list(&clamped, &parsed, &current, &stored),
            self.slewing(),
        );
        self.publish_params();
        Ok(reply)
    }

    /// The running session's tuning targets, or why there are none to move.
    fn session_tuning(&self) -> Result<LiveTuning, Rejection> {
        self.control
            .as_ref()
            .ok_or_else(no_session)?
            .tuning()
            .map_err(|e| Rejection::new(Reason::NotReady, None, e.to_string()))
    }

    /// Per field still crossing, the fraction of the last accepted change still to go.
    fn slewing(&self) -> BTreeMap<&'static str, f64> {
        let Some(crossing) = &self.params.crossing else {
            return BTreeMap::new();
        };
        let (from, to) = (crossing.from.to_words(), crossing.to.to_words());
        let elapsed = crossing.at.elapsed().as_secs_f64();
        let mut slewing = BTreeMap::new();
        for field in fields() {
            let left = field
                .words
                .clone()
                .map(|word| remaining(word, &from, &to, elapsed))
                .fold(0.0, f64::max);
            if left >= SETTLED {
                slewing.insert(field.name, left);
            }
        }
        slewing
    }
}

/// What is left of one word's change, as a fraction of it: the filter read off the clock for a
/// slewed word, the ramp's remaining seconds over its whole length for a gated one descending,
/// and nothing at all for a word that stepped or never moved.
fn remaining(
    word: usize,
    from: &[f64; LiveTuning::WORDS],
    to: &[f64; LiveTuning::WORDS],
    elapsed: f64,
) -> f64 {
    let policy = LiveTuning::BOUNDS[word].policy;
    if from[word] == to[word] {
        return 0.0;
    }
    let Some(rate_word) = policy.rate_word() else {
        return policy.remaining(elapsed);
    };
    // The gate raises whole and lowers at the next order's limit, so the rate in force just
    // after the change is the larger of that word's two values: a raise of it was already in
    // force, a lowering of it starts where it was.
    let rate = from[rate_word].max(to[rate_word]);
    let whole = policy.descent(from[word], to[word], rate);
    if whole == 0.0 {
        return 0.0;
    }
    ((whole - elapsed) / whole).clamp(0.0, 1.0)
}

/// The first field whose stored value crosses its confirmation threshold from at or below it
/// and that `confirmed` does not name; `None` when the change needs no confirmation.
///
/// Crossing, not "is above": a value already past the threshold may be moved within or below it
/// freely, and only the step over it is the decision. A name in `confirmed` that no field
/// carries is ignored, as the wire contract says.
fn unconfirmed(
    current: &LiveTuning,
    next: &LiveTuning,
    confirmed: &[String],
) -> Option<&'static FieldBound> {
    let (current, next) = (current.to_words(), next.to_words());
    LiveTuning::BOUNDS
        .iter()
        .enumerate()
        .find(|(word, bound)| {
            confirm_above(bound).is_some_and(|at| next[*word] > at && current[*word] <= at)
                && !confirmed.iter().any(|name| name == bound.name)
        })
        .map(|(_, bound)| bound)
}

fn confirm_above(bound: &FieldBound) -> Option<f64> {
    match bound.danger {
        Some(TuningDanger::ConfirmAbove(at)) => Some(at),
        _ => None,
    }
}

/// What the gate moved, as the wire reports it: the value the request asked for beside the
/// value in force. A word the request did not carry can be here too -- the pair rule raises a
/// damping under a spring somebody else moved -- and then what was "requested" for it is the
/// value it already had.
fn clamped_list(
    clamped: &[&'static FieldBound],
    parsed: &ParsedUpdate,
    current: &LiveTuning,
    stored: &LiveTuning,
) -> Vec<Clamped> {
    let (current, stored) = (current.to_words(), stored.to_words());
    clamped
        .iter()
        .filter_map(|bound| {
            // By name and element, not by address: `BOUNDS` is a `const`, so the rows a
            // caller hands back may be a copy of the ones this crate reads, at a different
            // address entirely. The pair is unique -- the table asserts one row per word.
            let word = LiveTuning::BOUNDS
                .iter()
                .position(|row| row.name == bound.name && row.index == bound.index)?;
            Some(Clamped {
                field: bound.name,
                index: bound.index,
                requested: parsed.requested[word].unwrap_or(current[word]),
                stored: stored[word],
            })
        })
        .collect()
}

fn no_session() -> Rejection {
    Rejection::new(
        Reason::NotReady,
        None,
        "no session is running: enable one before tuning it",
    )
}
