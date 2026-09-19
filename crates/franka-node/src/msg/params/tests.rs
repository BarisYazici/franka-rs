//! That the schema is the table and nothing else: every number it serves is the row's, the
//! numbers themselves are the ones the design settled on, and the committed dump is what a
//! running node would answer.

use std::collections::BTreeMap;

use franka::robot::target_control::{
    LiveTuning, TuningDanger, TuningPolicy, MIN_JOINT_DAMPING_RATIO,
};
use franka::FciVersion;
use serde_json::{Map, Value};

use super::*;
use crate::config::{ArmConfig, NodeConfig};

fn config(extra: &str) -> ArmConfig {
    let toml = format!("[[arm]]\nname = \"L\"\nhost = \"robot\"\n{extra}");
    toml.parse::<NodeConfig>()
        .expect("a valid arm")
        .arms
        .remove(0)
}

fn schema() -> SchemaMsg {
    SchemaMsg::new(&config(""), FciVersion::V10, DUMP_BOOT_ID)
}

fn at(value: &ParamValue, index: usize) -> f64 {
    match value {
        ParamValue::Scalar(one) if index == 0 => *one,
        ParamValue::Array(many) => many[index],
        other => panic!("{other:?} has no element {index}"),
    }
}

/// The property the whole design rests on: what the schema serves is read out of the rows, word
/// for word. A hand-written copy passes this only until a bound moves, which is exactly the
/// drift that cost this project a day -- and the committed dump below is what notices the move.
#[test]
fn every_word_of_the_schema_is_its_own_row() {
    let schema = schema();
    let defaults = config("").live_tuning().to_words();
    for (word, bound) in LiveTuning::BOUNDS.iter().enumerate() {
        let field = &schema.params[bound.name];
        let element = bound.index.unwrap_or(0);
        let named = |what: &str| format!("{}{:?} {what}", bound.name, bound.index);
        assert_eq!(at(&field.min, element), bound.min, "{}", named("min"));
        assert_eq!(at(&field.max, element), bound.max, "{}", named("max"));
        assert_eq!(
            at(&field.default, element),
            defaults[word],
            "{}",
            named("default")
        );
        assert_eq!(
            field.unit.as_ref().map(|unit| match unit {
                Value::String(one) => one.clone(),
                Value::Array(many) => many[element].as_str().unwrap_or_default().to_string(),
                other => panic!("{other} is not a unit"),
            }),
            (!bound.unit.is_empty()).then(|| bound.unit.to_string()),
            "{}",
            named("unit")
        );
        assert_eq!(field.group, bound.group, "{}", named("group"));
        assert_eq!(
            field.confirm_above.as_ref().map(|at_| at(at_, element)),
            match bound.danger {
                Some(TuningDanger::ConfirmAbove(threshold)) => Some(threshold),
                _ => None,
            },
            "{}",
            named("confirm_above")
        );
        assert_eq!(
            field.slider_max.as_ref().map(|value| at(value, element)),
            bound.slider_max,
            "{}",
            named("slider_max")
        );
        assert_eq!(
            field.off_at.as_ref().map(|value| at(value, element)),
            bound.off_at,
            "{}",
            named("off_at")
        );
        assert_eq!(field.norm, bound.norm.then_some(true), "{}", named("norm"));
        if bound.log_slider {
            assert_eq!(field.scale, "log", "{}", named("scale"));
        }
        if let TuningPolicy::Slew { tau } = bound.policy {
            assert_eq!(field.slew_tau_s, Some(tau), "{}", named("slew_tau_s"));
            assert_eq!(field.policy, "slew", "{}", named("policy"));
        }
    }
}

/// Hand a different table and the schema follows it. [`FieldBound`] cannot be built outside the
/// library, so the different table is a slice of the one: the seven damping rows, read where the
/// seven stiffness rows sit. A builder that answered from a per-name or per-position copy of the
/// numbers rather than from the rows it was handed would still say 1200 here.
#[test]
fn the_schema_is_of_the_table_it_is_given_and_not_of_a_copy() {
    let damping = &LiveTuning::BOUNDS[7..14];
    let served = field_schemas(damping, &[4.0; 7]);
    assert_eq!(served.len(), 1, "{served:?}");
    let field = &served["joint_damping"];
    assert_eq!(at(&field.max, 0), 60.0);
    assert_eq!(at(&field.default, 6), 4.0);
    assert_eq!(field.kind, "f64[7]");
    // The whole table, by contrast, has a ceiling of 1200 at those same word offsets.
    assert_eq!(at(&schema().params["joint_stiffness"].max, 0), 1200.0);
}

/// The numbers themselves, by value, with the value each of them is *not*: the design record's
/// disagreements with `DESIGN-ui.md` are the ones a silent revert would land on.
#[test]
fn the_bounds_served_are_the_ones_the_design_settled_on() {
    let schema = schema();
    let field = |name: &str| schema.params[name].clone();
    assert_eq!(at(&field("joint_stiffness").max, 0), 1200.0);
    // 60, not the reference joint-impedance stack's 80: the velocity barrier's gain is added
    // on top of this one.
    assert_eq!(at(&field("joint_damping").max, 0), 60.0);
    assert_ne!(at(&field("joint_damping").max, 0), 80.0);
    assert_eq!(at(&field("joint_damping").min, 0), 0.0);
    assert_eq!(at(&field("cartesian_stiffness").min, 0), 50.0);
    assert_eq!(at(&field("cartesian_stiffness").max, 0), 3000.0);
    // 1e-3, not DESIGN-ui's 1e-4.
    assert_eq!(at(&field("ik_damping").min, 0), 1e-3);
    assert_ne!(at(&field("ik_damping").min, 0), 1e-4);
    assert_eq!(at(&field("ik_nullspace_gain").max, 0), 20.0);
    assert_eq!(at(&field("velocity_feedforward_gain").max, 0), 1.0);
    assert_eq!(at(&field("velocity_feedforward_cutoff").min, 0), 1.0);
    assert_eq!(at(&field("velocity_feedforward_cutoff").max, 0), 1000.0);
    // 1.2 m/s, not DESIGN-ui's 1.5: the client's dq guard was measured firing at 1.0.
    assert_eq!(at(&field("budget").max, 0), 1.2);
    assert_ne!(at(&field("budget").max, 0), 1.5);
    assert_eq!(at(&field("budget").max, 1), 20.0);
    assert_eq!(at(&field("budget").max, 2), 800.0);
    assert_eq!(at(&field("rotation_budget").max, 0), 2.5);
    // The confirmation sits below the ceiling, at the set an arm was driven at all afternoon.
    assert_eq!(at(&field("budget").confirm_above.unwrap(), 0), 0.85);
    assert_eq!(field("budget").danger, Some("confirm_above"));
    assert_eq!(field("joint_stiffness").danger, Some("advise"));
    assert_eq!(field("ik_damping").danger, None);
    // The three ranges spanning decades, and no others.
    let log: Vec<&str> = schema
        .params
        .iter()
        .filter(|(_, field)| field.scale == "log")
        .map(|(name, _)| *name)
        .collect();
    assert_eq!(
        log,
        [
            "cartesian_stiffness",
            "ik_damping",
            "velocity_feedforward_cutoff"
        ]
    );
}

/// A budget's velocity and acceleration are gated and its jerk steps, and the wire has one
/// policy per field: the field is named for the ramp, which is what an operator has to know.
#[test]
fn a_field_is_named_for_the_strongest_policy_any_of_its_words_has() {
    let schema = schema();
    assert_eq!(schema.params["budget"].policy, "step_up_gate_down");
    assert_eq!(schema.params["budget"].slew_tau_s, None);
    assert_eq!(schema.params["velocity_feedforward_cutoff"].policy, "step");
    assert_eq!(schema.params["joint_stiffness"].policy, "slew");
    assert_eq!(schema.params["joint_stiffness"].slew_tau_s, Some(0.3));
    // And the table really does hold both policies inside the one field.
    let budget: Vec<_> = fields()
        .into_iter()
        .find(|field| field.name == "budget")
        .expect("the budget is published")
        .words
        .map(|word| LiveTuning::BOUNDS[word].policy)
        .collect();
    assert!(budget[0].rate_word().is_some() && budget[1].rate_word().is_some());
    assert_eq!(budget[2], TuningPolicy::Step);
}

/// The pair rule no per-field row can express, published as the rule it is.
#[test]
fn the_joint_damping_floor_is_published_as_a_relation() {
    let relations = schema().relations;
    assert_eq!(relations.len(), 1);
    assert_eq!(
        relations[0].rule,
        "joint_damping[i] >= 0.25 * sqrt(joint_stiffness[i])"
    );
    assert!(relations[0]
        .rule
        .contains(&MIN_JOINT_DAMPING_RATIO.to_string()));
}

/// `derived` is the config's, key by key, and the arm's own joint velocity limits.
#[test]
fn derived_is_the_configs_and_the_connected_arms() {
    let config = config("max_lead = 0.12\nrate_hz = 200\nstate_hz = 50\ncutoff_frequency = 80\n");
    let derived = Derived::new(&config, FciVersion::V10);
    assert_eq!(derived.max_lead, 0.12);
    assert_eq!(derived.rate_hz, 200.0);
    assert_eq!(derived.state_hz, 50);
    assert_eq!(derived.cutoff_frequency, 80.0);
    assert_eq!(derived.leash.translation, 0.025);
    assert_eq!(derived.delta_t, 0.001);
    // The FR3's limits, not the FER's -- the panel's dq headroom bar is drawn from these.
    assert_eq!(derived.dq_limit[0], 2.62);
    // The FER's, which the library already takes libfranka's epsilon and packet allowance off
    // -- the usable limit rather than the datasheet's 2.175.
    assert_eq!(Derived::new(&config, FciVersion::V5).dq_limit[0], 2.129003);
    // The preset the one stiffness slider scales, which is what the feedforward speed-cap
    // warning needs: without it the panel omits the warning altogether.
    assert_eq!(derived.cartesian_preset.reference, 750.0);
    assert_eq!(
        derived.cartesian_preset.stiffness,
        [750.0, 750.0, 750.0, 15.0, 15.0, 15.0]
    );
    assert_eq!(
        derived.cartesian_preset.damping,
        [50.0, 50.0, 90.0, 2.0, 2.0, 2.0]
    );
}

/// The defaults follow the TOML, so a panel opens on the values the arm will actually start at.
#[test]
fn the_defaults_are_the_configs_not_the_presets() {
    let config = config(
        "cartesian_stiffness = 1500\nbudget = [0.4, 0.6, 30]\njoint_damping = [9,9,9,9,9,9,9]\n",
    );
    let schema = SchemaMsg::new(&config, FciVersion::V10, DUMP_BOOT_ID);
    assert_eq!(at(&schema.params["cartesian_stiffness"].default, 0), 1500.0);
    assert_eq!(at(&schema.params["budget"].default, 1), 0.6);
    assert_eq!(at(&schema.params["joint_damping"].default, 3), 9.0);
    // Still the table's bounds: a config cannot widen or narrow the gate.
    assert_eq!(at(&schema.params["budget"].max, 0), 1.2);
}

/// Every field the table has can be set. The one name-by-name match in the node is here; a
/// field added to the library that this forgot would be published and then unsettable.
#[test]
fn every_field_of_the_table_is_settable() {
    for field in fields() {
        let value = match field.words.len() {
            1 => Value::from(0.5),
            n => Value::from(vec![0.5; n]),
        };
        let mut params = Map::new();
        params.insert(field.name.to_string(), value);
        let parsed = update_of(&params).expect(field.name);
        let carried: Vec<usize> = (0..LiveTuning::WORDS)
            .filter(|word| parsed.requested[*word].is_some())
            .collect();
        assert_eq!(
            carried,
            field.words.clone().collect::<Vec<_>>(),
            "{} carried the wrong words",
            field.name
        );
        // And it reached the update, not just the word array.
        let mut tuning = LiveTuning::from_words(&[0.5; LiveTuning::WORDS]);
        tuning
            .apply_update(&parsed.update)
            .unwrap_or_else(|e| panic!("{}: {e}", field.name));
    }
}

/// The shape checks are the node's, and each names the field and the reason the wire contract
/// closes on.
#[test]
fn a_request_of_the_wrong_shape_is_refused_by_shape() {
    let refuse = |json: &str| {
        let params: Map<String, Value> = serde_json::from_str(json).expect("valid json");
        update_of(&params).expect_err(json)
    };
    let bad = refuse(r#"{"nope": 1}"#);
    assert_eq!(
        (bad.reason, bad.field.as_deref()),
        (Reason::UnknownField, Some("nope"))
    );
    let bad = refuse(r#"{"ik_damping": "x"}"#);
    assert_eq!(
        (bad.reason, bad.field.as_deref()),
        (Reason::Type, Some("ik_damping"))
    );
    let bad = refuse(r#"{"ik_damping": [0.1]}"#);
    assert_eq!(bad.reason, Reason::Type);
    let bad = refuse(r#"{"budget": [1, 2]}"#);
    assert_eq!(
        (bad.reason, bad.field.as_deref()),
        (Reason::Length, Some("budget"))
    );
    let bad = refuse(r#"{"joint_damping": 4}"#);
    assert_eq!(bad.reason, Reason::Type);
    let bad = refuse(r#"{"budget": [0.3, 0.5, null]}"#);
    assert_eq!(bad.reason, Reason::Type);
    // What passes: a scalar, a whole array, and an integer where a float is wanted.
    let params: Map<String, Value> =
        serde_json::from_str(r#"{"ik_damping": 0.1, "budget": [1, 2, 3]}"#).expect("valid json");
    let parsed = update_of(&params).expect("a well-shaped request");
    assert_eq!(parsed.update.ik_damping, Some(0.1));
    assert_eq!(parsed.update.budget, Some([1.0, 2.0, 3.0]));
}

/// `params` is every field, as scalars and arrays, and reads back as the tuning it came from.
#[test]
fn the_values_carry_every_field_in_its_own_shape() {
    let tuning = config("").live_tuning();
    let values = params_of(&tuning);
    assert_eq!(values.len(), fields().len());
    assert_eq!(values["cartesian_stiffness"], ParamValue::Scalar(750.0));
    assert_eq!(values["budget"], ParamValue::Array(vec![0.3, 0.5, 20.0]));
    assert_eq!(
        values["joint_damping"],
        ParamValue::Array(tuning.joint_damping.to_vec())
    );
}

/// A refusal and an acceptance serialise to the envelope the contract names, `ok` included.
#[test]
fn the_replies_serialise_to_the_contracts_envelope() {
    let refused = SetReply::refused(
        &Rejection::new(Reason::Stale, Some("budget".into()), "no"),
        7,
    );
    let json: Value = serde_json::from_str(&to_json(&refused)).expect("valid json");
    assert_eq!(json["ok"], Value::Bool(false));
    assert_eq!(json["reason"], "stale");
    assert_eq!(json["field"], "budget");
    assert_eq!(json["version"], 7);
    let accepted = SetReply::accepted(
        8,
        params_of(&config("").live_tuning()),
        vec![Clamped {
            field: "joint_damping",
            index: Some(4),
            requested: 95.0,
            stored: 60.0,
        }],
        BTreeMap::from([("joint_damping", 1.0)]),
    );
    let json: Value = serde_json::from_str(&to_json(&accepted)).expect("valid json");
    assert_eq!(json["ok"], Value::Bool(true));
    assert_eq!(json["version"], 8);
    assert_eq!(json["clamped"][0]["stored"], 60.0);
    assert_eq!(json["clamped"][0]["index"], 4);
    assert_eq!(json["slewing"]["joint_damping"], 1.0);
    assert_eq!(json["params"]["cartesian_stiffness"], 750.0);
}

/// A `set` request with a key the envelope does not have is refused rather than half-read: the
/// same `deny_unknown_fields` discipline `CmdRequest` holds.
#[test]
fn the_request_envelope_is_closed() {
    let ok: Result<SetRequest, _> =
        serde_json::from_str(r#"{"client_id": 1, "params": {}, "confirm": ["budget"]}"#);
    assert_eq!(ok.expect("a valid request").confirm, vec!["budget"]);
    let bad: Result<SetRequest, _> =
        serde_json::from_str(r#"{"client_id": 1, "params": {}, "speed": 0.5}"#);
    assert!(bad.is_err(), "an unknown envelope key was accepted");
}

/// The committed dump is what the node would serve. Regenerate it with
/// `cargo run -p franka-node --example params_schema > crates/franka-node/schema/params-schema.json`
/// -- and read the diff, because everything downstream of it (the panel's mock owner) is
/// generated from it and a number moving here is a number moving on an arm.
#[test]
fn the_committed_dump_is_the_schema_this_node_serves() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/schema/params-schema.json");
    let committed = std::fs::read_to_string(path).expect("the dump is committed");
    let config = format!("[[arm]]\nname = \"{DUMP_ARM}\"\nhost = \"robot\"\n")
        .parse::<NodeConfig>()
        .expect("a valid arm")
        .arms
        .remove(0);
    let schema = SchemaMsg::new(&config, FciVersion::V10, DUMP_BOOT_ID);
    let served = serde_json::to_string_pretty(&schema).expect("the schema serialises") + "\n";
    assert_eq!(
        served, committed,
        "schema/params-schema.json is stale; regenerate it (see this test)"
    );
}
