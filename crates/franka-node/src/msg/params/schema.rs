//! The schema half: every published field as its [`FieldBound`] rows describe it, and the
//! read-only `derived` block beside them.
//!
//! Not a number in this file. Each key is read out of the rows it was handed, so a bound moved
//! in the library moves here, and a `FieldBound` field a later parameter needs is one key more
//! rather than a table to maintain.

use std::collections::BTreeMap;

use franka::robot::target_control::{
    FieldBound, ImpedanceGains, LiveTuning, TuningDanger, TuningPolicy, MIN_JOINT_DAMPING_RATIO,
};
use franka::{FciVersion, DELTA_T};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use super::{fields_of, Field, ParamValue, OWNER, SCHEMA_VERSION};
use crate::config::ArmConfig;

/// What one field of the schema says about itself. Serialised straight out of that field's
/// [`FieldBound`] rows; the optional keys are absent where the rows carry nothing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FieldSchema {
    #[serde(rename = "type")]
    pub kind: String,
    pub min: ParamValue,
    pub max: ParamValue,
    pub default: ParamValue,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<Value>,
    pub policy: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slew_tau_s: Option<f64>,
    pub group: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub danger: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confirm_above: Option<ParamValue>,
    pub scale: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slider_max: Option<ParamValue>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub off_at: Option<ParamValue>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub norm: Option<bool>,
}

/// Every field of `table`, keyed by name, with `defaults` the values a session starts at -- one
/// per word of `table`, in its order.
///
/// The whole schema half of this module: nothing here holds a number, every one of them is read
/// out of the rows it was handed.
pub fn field_schemas(
    table: &'static [FieldBound],
    defaults: &[f64],
) -> BTreeMap<&'static str, FieldSchema> {
    fields_of(table)
        .iter()
        .map(|field| (field.name, FieldSchema::of(field, defaults)))
        .collect()
}

impl FieldSchema {
    /// The field as its rows describe it, with `defaults` the values a session of this config
    /// starts at.
    fn of(field: &Field, defaults: &[f64]) -> FieldSchema {
        let rows = field.bounds();
        let policies = rows.iter().map(|bound| bound.policy);
        // One policy per field on the wire, and up to one per word in the table: a budget's
        // jerk steps while its velocity and acceleration are gated. The field is named for the
        // strongest thing that happens to any of its words, because that is what an operator
        // needs told -- dragging this slider down is a ramp.
        let policy = if policies.clone().any(|p| p.rate_word().is_some()) {
            "step_up_gate_down"
        } else if policies
            .clone()
            .any(|p| matches!(p, TuningPolicy::Slew { .. }))
        {
            "slew"
        } else {
            "step"
        };
        let tau = rows.iter().find_map(|bound| match bound.policy {
            TuningPolicy::Slew { tau } => Some(tau),
            _ => None,
        });
        let confirm = |bound: &FieldBound| match bound.danger {
            Some(TuningDanger::ConfirmAbove(at)) => Some(at),
            _ => None,
        };
        let confirm_above = field.every(confirm);
        FieldSchema {
            kind: field.kind(),
            min: field.wire(rows.iter().map(|bound| bound.min).collect()),
            max: field.wire(rows.iter().map(|bound| bound.max).collect()),
            default: field.value(defaults),
            unit: units(field),
            policy,
            slew_tau_s: tau,
            group: rows[0].group,
            danger: match (&confirm_above, rows.iter().any(|b| b.danger.is_some())) {
                (Some(_), _) => Some("confirm_above"),
                (None, true) => Some("advise"),
                (None, false) => None,
            },
            confirm_above,
            scale: if rows.iter().any(|bound| bound.log_slider) {
                "log"
            } else {
                "linear"
            },
            slider_max: field.every(|bound| bound.slider_max),
            off_at: field.every(|bound| bound.off_at),
            norm: rows.iter().all(|bound| bound.norm).then_some(true),
        }
    }
}

/// The field's unit, as a string for a scalar and one per element for an array; absent where
/// the quantity is dimensionless.
fn units(field: &Field) -> Option<Value> {
    let units: Vec<&'static str> = field.bounds().iter().map(|bound| bound.unit).collect();
    if units.iter().all(|unit| unit.is_empty()) {
        return None;
    }
    Some(match field.kind().as_str() {
        "f64" => Value::from(units[0]),
        _ => Value::from(units),
    })
}

/// A rule the owner enforces that no per-field row can express, for a reader to show. The
/// owner enforces it either way; this is documentation, as the wire contract says.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Relation {
    pub rule: String,
}

/// `franka/<arm>/params/schema`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SchemaMsg {
    pub owner: &'static str,
    pub arm: String,
    pub boot_id: String,
    pub schema_version: u32,
    pub params: BTreeMap<&'static str, FieldSchema>,
    pub relations: Vec<Relation>,
    pub derived: Derived,
}

impl SchemaMsg {
    /// The schema of `config`'s arm connected at `version`, as of `boot_id`.
    pub fn new(config: &ArmConfig, version: FciVersion, boot_id: &str) -> SchemaMsg {
        let defaults = config.live_tuning().to_words();
        SchemaMsg {
            owner: OWNER,
            arm: config.name.clone(),
            boot_id: boot_id.to_string(),
            schema_version: SCHEMA_VERSION,
            params: field_schemas(LiveTuning::BOUNDS, &defaults),
            // The pair rule of `LiveTuning::joint_damping_floor`: the one limit the table
            // enforces that no single row holds, so it is published as the rule it is rather
            // than left for a panel to discover by watching a damping move on its own.
            relations: vec![Relation {
                rule: format!(
                    "joint_damping[i] >= {MIN_JOINT_DAMPING_RATIO} * sqrt(joint_stiffness[i])"
                ),
            }],
            derived: Derived::new(config, version),
        }
    }
}

/// The leash, as `derived` carries it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LeashDerived {
    pub translation: f64,
    pub rotation: f64,
}

/// The Cartesian preset the one stiffness slider scales, which is what a reader needs to work
/// out what the law does at a given stiffness: `stiffness` and `damping` hold at `reference`,
/// and the crate scales them by the ratio and its square root.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CartesianPreset {
    pub stiffness: [f64; 6],
    pub damping: [f64; 6],
    pub reference: f64,
}

/// What the node will not let a client change but a client has to know: the gate's limits, the
/// rates, and the preset behind the one stiffness slider. Read-only, and the reason
/// `limits.py` need hold no copy of any of it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Derived {
    pub leash: LeashDerived,
    pub max_lead: f64,
    pub max_lead_rotation: f64,
    pub max_step: f64,
    pub max_step_rotation: f64,
    pub max_step_joint: f64,
    pub rate_hz: f64,
    pub state_hz: u32,
    pub stop_after_ms: u64,
    pub delta_t: f64,
    /// The connected arm's joint velocity limits, rad/s: an FR3's and an FER's differ.
    pub dq_limit: [f64; 7],
    pub cutoff_frequency: f64,
    pub cartesian_preset: CartesianPreset,
}

impl Derived {
    /// The read-only block of `config`'s arm, connected at FCI `version`.
    pub fn new(config: &ArmConfig, version: FciVersion) -> Derived {
        let preset = ImpedanceGains::CARTESIAN;
        Derived {
            leash: LeashDerived {
                translation: config.leash.translation,
                rotation: config.leash.rotation,
            },
            max_lead: config.max_lead,
            max_lead_rotation: config.max_lead_rotation,
            max_step: config.max_step,
            max_step_rotation: config.max_step_rotation,
            max_step_joint: config.max_step_joint,
            rate_hz: config.rate_hz,
            state_hz: config.state_hz,
            stop_after_ms: config.stop_after_ms,
            delta_t: DELTA_T,
            dq_limit: franka::robot::target_control::max_joint_velocity(version),
            cutoff_frequency: config.cutoff_frequency,
            cartesian_preset: CartesianPreset {
                stiffness: preset.cartesian_stiffness,
                damping: preset.cartesian_damping,
                reference: preset.cartesian_stiffness[0],
            },
        }
    }
}
