//! The values half: what `get` and `current` carry, and the request and reply of a `set`.
//!
//! The decoding here is the node's own half of the check -- is it a number, is the array the
//! right length, is the field one the table has. What a value is *allowed* to be is the
//! library's, and a refusal of its comes back as `invalid`.

use std::collections::BTreeMap;

use franka::robot::target_control::{LiveTuning, TuningUpdate};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use super::{fields, Field, ParamValue};

/// Who moved the values last: the `version` it produced and the `client_id` of the `set`,
/// echoed unchanged so a panel can tell its own change from another browser's.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Origin {
    pub version: u64,
    pub by: u32,
    pub at_ns: u64,
}

/// `franka/<arm>/params/get` and `franka/<arm>/params/current`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamsMsg {
    pub owner: &'static str,
    pub arm: String,
    pub version: u64,
    pub boot_id: String,
    pub t_node_ns: u64,
    pub origin: Option<Origin>,
    pub params: BTreeMap<&'static str, ParamValue>,
    /// Per field still crossing to its target, the fraction of the change **still to go**.
    pub slewing: BTreeMap<&'static str, f64>,
    /// Whether the values in force differ from the ones the TOML asked for.
    pub dirty: bool,
}

/// Every field's value, as `get` and a `set` reply carry them.
pub fn params_of(tuning: &LiveTuning) -> BTreeMap<&'static str, ParamValue> {
    let words = tuning.to_words();
    fields()
        .iter()
        .map(|field| (field.name, field.value(&words)))
        .collect()
}

/// The JSON payload of a `params/set` query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SetRequest {
    pub client_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base_version: Option<u64>,
    /// Fields whose [`ConfirmAbove`](franka::robot::target_control::TuningDanger::ConfirmAbove)
    /// crossing the operator has confirmed. A name
    /// this owner does not know is ignored rather than refused, as the wire contract says.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub confirm: Vec<String>,
    pub params: Map<String, Value>,
}

/// One value the gate moved, so a caller can show a slider snapping to a bound it did not know.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Clamped {
    pub field: &'static str,
    pub index: Option<usize>,
    pub requested: f64,
    pub stored: f64,
}

/// Why a `set` was refused. The closed list of the wire contract; the bridge adds its own and
/// never produces one of these.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    Type,
    NonFinite,
    Length,
    UnknownField,
    Relation,
    NeedsConfirm,
    Stale,
    NotReady,
    Busy,
    Invalid,
}

impl Reason {
    pub fn as_str(self) -> &'static str {
        match self {
            Reason::Type => "type",
            Reason::NonFinite => "non_finite",
            Reason::Length => "length",
            Reason::UnknownField => "unknown_field",
            Reason::Relation => "relation",
            Reason::NeedsConfirm => "needs_confirm",
            Reason::Stale => "stale",
            Reason::NotReady => "not_ready",
            Reason::Busy => "busy",
            Reason::Invalid => "invalid",
        }
    }
}

/// A refusal: nothing was stored and the version is unchanged.
#[derive(Debug, Clone, PartialEq)]
pub struct Rejection {
    pub reason: Reason,
    pub field: Option<String>,
    pub error: String,
}

impl Rejection {
    pub fn new(reason: Reason, field: Option<String>, error: impl Into<String>) -> Rejection {
        Rejection {
            reason,
            field,
            error: error.into(),
        }
    }
}

/// The JSON reply of a `params/set` query, accepted or refused.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SetReply {
    Accepted {
        ok: bool,
        version: u64,
        params: BTreeMap<&'static str, ParamValue>,
        clamped: Vec<Clamped>,
        slewing: BTreeMap<&'static str, f64>,
    },
    Refused {
        ok: bool,
        reason: &'static str,
        field: Option<String>,
        error: String,
        version: u64,
    },
}

impl SetReply {
    pub fn accepted(
        version: u64,
        params: BTreeMap<&'static str, ParamValue>,
        clamped: Vec<Clamped>,
        slewing: BTreeMap<&'static str, f64>,
    ) -> SetReply {
        SetReply::Accepted {
            ok: true,
            version,
            params,
            clamped,
            slewing,
        }
    }

    pub fn refused(rejection: &Rejection, version: u64) -> SetReply {
        SetReply::Refused {
            ok: false,
            reason: rejection.reason.as_str(),
            field: rejection.field.clone(),
            error: rejection.error.clone(),
            version,
        }
    }
}

/// A `set` request's `params`, decoded: the update to apply, and the raw values it asked for
/// beside it, which is what the `clamped` list reports against what was stored.
#[derive(Debug)]
pub struct ParsedUpdate {
    pub update: TuningUpdate,
    /// The value the request carried for each word, `None` where it carried none.
    pub requested: [Option<f64>; LiveTuning::WORDS],
}

/// The request's `params` as a [`TuningUpdate`], or why its shape is wrong.
///
/// This is the node's half of the check -- is it a number, is the array the right length, is
/// the field one the table has -- and it ends there. Whether the value is *allowed* is
/// [`LiveTuning::apply_update`]'s to say, and the reply names that refusal `invalid`.
pub fn update_of(params: &Map<String, Value>) -> Result<ParsedUpdate, Rejection> {
    let fields = fields();
    let mut words = [None; LiveTuning::WORDS];
    for (name, value) in params {
        let field = fields
            .iter()
            .find(|field| field.name == *name)
            .ok_or_else(|| {
                Rejection::new(
                    Reason::UnknownField,
                    Some(name.clone()),
                    format!("unknown field {name}"),
                )
            })?;
        let carried = numbers(field, name, value)?;
        for (word, value) in field.words.clone().zip(carried) {
            words[word] = Some(value);
        }
    }
    Ok(ParsedUpdate {
        update: update_from_words(&fields, &words)?,
        requested: words,
    })
}

/// `value` as one number per word of `field`.
fn numbers(field: &Field, name: &str, value: &Value) -> Result<Vec<f64>, Rejection> {
    let bad = |reason, what: String| Rejection::new(reason, Some(name.to_string()), what);
    let scalar = |value: &Value, at: Option<usize>| {
        let at = at.map_or(String::new(), |i| format!("[{i}]"));
        match value.as_f64() {
            Some(number) if number.is_finite() => Ok(number),
            Some(number) => Err(bad(
                Reason::NonFinite,
                format!("{name}{at}: {number} is not finite"),
            )),
            None => Err(bad(
                Reason::Type,
                format!("{name}{at}: expected a number, got {value}"),
            )),
        }
    };
    let wanted = field.words.len();
    match (value, field.kind().as_str()) {
        (Value::Array(_), "f64") => Err(bad(
            Reason::Type,
            format!("{name}: expected a number, got an array"),
        )),
        (_, "f64") => Ok(vec![scalar(value, None)?]),
        (Value::Array(values), _) if values.len() == wanted => values
            .iter()
            .enumerate()
            .map(|(i, value)| scalar(value, Some(i)))
            .collect(),
        (Value::Array(values), _) => Err(bad(
            Reason::Length,
            format!("{name}: expected {wanted} elements, got {}", values.len()),
        )),
        _ => Err(bad(
            Reason::Type,
            format!("{name}: expected an array of {wanted}, got {value}"),
        )),
    }
}

/// The carried words as a [`TuningUpdate`]. The one place in the node that spells the field
/// names out; `every_field_of_the_table_is_settable` is what keeps it complete.
fn update_from_words(
    fields: &[Field],
    words: &[Option<f64>; LiveTuning::WORDS],
) -> Result<TuningUpdate, Rejection> {
    let mut update = TuningUpdate::default();
    for field in fields
        .iter()
        .filter(|field| words[field.words.start].is_some())
    {
        let at = |i: usize| words[field.words.start + i].expect("a field is carried whole");
        let seven = || std::array::from_fn::<f64, 7, _>(&at);
        let three = || std::array::from_fn::<f64, 3, _>(&at);
        match field.name {
            "joint_stiffness" => update.joint_stiffness = Some(seven()),
            "joint_damping" => update.joint_damping = Some(seven()),
            "cartesian_stiffness" => update.cartesian_stiffness = Some(at(0)),
            "ik_damping" => update.ik_damping = Some(at(0)),
            "ik_nullspace_gain" => update.ik_nullspace_gain = Some(at(0)),
            "velocity_feedforward_gain" => update.velocity_feedforward_gain = Some(at(0)),
            "velocity_feedforward_cutoff" => update.velocity_feedforward_cutoff = Some(at(0)),
            "budget" => update.budget = Some(three()),
            "rotation_budget" => update.rotation_budget = Some(three()),
            name => {
                return Err(Rejection::new(
                    Reason::UnknownField,
                    Some(name.to_string()),
                    format!("{name} is in the table but this node cannot set it"),
                ))
            }
        }
    }
    Ok(update)
}
