//! The wire contract of a `params/set`: the clamp is stored and reported, and the confirmation
//! gate holds. SIM-PLAN assertions 4 and 5.

use franka::robot::target_control::LiveTuning;
use serde_json::{json, Value};

use super::bounds::must_confirm_above;
use super::law::gains_of;
use super::params::{get, schema, set, set_field, set_ok, set_raw, word};
use super::rig::Rig;
use crate::CLIENT;

/// Assertion 4. A `set` past a bound stores the clamp and reports it with what was asked for;
/// every shape and domain refusal comes back with the contract's reason and changes nothing.
#[test]
fn a4_the_clamp_is_stored_and_reported() {
    let rig = Rig::wire();
    let (_token, _home) = rig.engage();
    let served = schema(rig.panel());
    let ceiling = word(&served["params"]["joint_stiffness"], "max", Some(0));
    let asked = ceiling * 2.0;

    let reply = set_ok(rig.panel(), "joint_stiffness", json!(vec![asked; 7]), &[]);
    let stored = gains_of(&reply["params"]);
    let floor = LiveTuning::joint_damping_floor(ceiling);
    for i in 0..7 {
        assert_eq!(
            stored.joint_stiffness[i], ceiling,
            "joint_stiffness[{i}] was not stored at the bound"
        );
        assert_eq!(
            stored.joint_damping[i], floor,
            "joint_damping[{i}] was not raised to the floor the spring puts under it"
        );
    }

    let clamped = reply["clamped"].as_array().expect("clamped").clone();
    for i in 0..7 {
        let row = clamped
            .iter()
            .find(|c| c["field"] == "joint_stiffness" && c["index"] == i)
            .unwrap_or_else(|| panic!("joint_stiffness[{i}] was clamped and not reported"));
        assert_eq!(row["requested"].as_f64(), Some(asked));
        assert_eq!(row["stored"].as_f64(), Some(ceiling));
        let pair = clamped
            .iter()
            .find(|c| c["field"] == "joint_damping" && c["index"] == i)
            .unwrap_or_else(|| panic!("joint_damping[{i}] was raised by the pair rule silently"));
        assert_eq!(pair["stored"].as_f64(), Some(floor));
        // A word nobody moved reports what it already had as the request.
        assert_eq!(
            pair["requested"].as_f64(),
            Some(rig.defaults.joint_damping[i]),
            "the pair rule's report does not say what the damping was"
        );
    }
    println!(
        "{} clamp rows, all named with requested and stored",
        clamped.len()
    );

    // A floor, reported the same way.
    let lower = word(&served["params"]["ik_damping"], "min", None);
    let below = set_ok(rig.panel(), "ik_damping", json!(lower / 1000.0), &[]);
    let row = &below["clamped"][0];
    assert_eq!(row["field"], "ik_damping");
    assert!(row["index"].is_null(), "a scalar field reported an index");
    assert_eq!(row["stored"].as_f64(), Some(lower));

    // Refusals, each by its contract reason, none of them changing anything.
    let version = get(rig.panel())["version"].as_u64().expect("version");
    let cases: Vec<(&str, Value, &str, Option<&str>)> = vec![
        (
            "zero where zero means something else",
            json!({"client_id": CLIENT, "params": {"ik_damping": 0.0}}),
            "invalid",
            None,
        ),
        (
            "not a number",
            json!({"client_id": CLIENT, "params": {"ik_damping": "x"}}),
            "type",
            Some("ik_damping"),
        ),
        (
            "wrong length",
            json!({"client_id": CLIENT, "params": {"joint_stiffness": [1.0, 2.0]}}),
            "length",
            Some("joint_stiffness"),
        ),
        (
            "a field the table does not have",
            json!({"client_id": CLIENT, "params": {"nope": 1.0}}),
            "unknown_field",
            Some("nope"),
        ),
        (
            "no client id",
            json!({"client_id": 0, "params": {"ik_nullspace_gain": 2.0}}),
            "type",
            Some("client_id"),
        ),
        (
            "a base version from before",
            json!({"client_id": CLIENT, "base_version": 0, "params": {"ik_nullspace_gain": 2.0}}),
            "stale",
            None,
        ),
        (
            "a scalar sent as an array",
            json!({"client_id": CLIENT, "params": {"ik_damping": [0.1]}}),
            "type",
            Some("ik_damping"),
        ),
        (
            "an array sent as a scalar",
            json!({"client_id": CLIENT, "params": {"joint_stiffness": 40.0}}),
            "type",
            Some("joint_stiffness"),
        ),
    ];
    for (what, body, reason, field) in cases {
        let reply = set(rig.panel(), body);
        assert_eq!(reply["ok"], false, "{what} was accepted: {reply}");
        assert_eq!(reply["reason"], reason, "{what}: wrong reason in {reply}");
        if let Some(name) = field {
            assert_eq!(reply["field"], name, "{what}: wrong field in {reply}");
        }
        assert_eq!(
            reply["version"].as_u64(),
            Some(version),
            "{what} moved the version"
        );
        assert!(
            !reply["error"].as_str().unwrap_or_default().is_empty(),
            "{what} refused with no message"
        );
    }

    // A number too large for an f64 never reaches the domain check: serde refuses it first, so
    // the contract's `non_finite` is unreachable over JSON and this comes back `type`. Asserted
    // rather than assumed, because the reason a panel shows depends on it.
    let overflow = set_raw(
        rig.panel(),
        &format!(r#"{{"client_id":{CLIENT},"params":{{"ik_damping":1e400}}}}"#),
    );
    assert_eq!(overflow["ok"], false, "1e400 was accepted: {overflow}");
    assert_eq!(overflow["reason"], "type", "1e400 came back as {overflow}");
    assert_eq!(overflow["version"].as_u64(), Some(version));

    // All or nothing: one good field beside one bad one leaves the good one alone.
    let before = get(rig.panel());
    let mixed = set(
        rig.panel(),
        json!({"client_id": CLIENT, "params": {"ik_nullspace_gain": 3.0, "ik_damping": 0.0}}),
    );
    assert_eq!(mixed["ok"], false, "a mixed request was accepted: {mixed}");
    let after = get(rig.panel());
    assert_eq!(
        after["params"]["ik_nullspace_gain"], before["params"]["ik_nullspace_gain"],
        "a refused request moved the field beside the bad one"
    );
    assert_eq!(after["version"], before["version"]);

    // A partial set leaves every other field exactly as it was.
    let before = get(rig.panel());
    let partial = set_ok(rig.panel(), "ik_nullspace_gain", json!(2.5), &[]);
    for (name, value) in before["params"].as_object().expect("params") {
        if name == "ik_nullspace_gain" {
            continue;
        }
        assert_eq!(
            &partial["params"][name], value,
            "a partial set moved {name}, which it did not carry"
        );
    }
    rig.assert_healthy();
}

/// Assertion 5. An envelope-widening field does not widen without a confirmation, the gate is
/// `>` and not `>=`, and it re-arms on every crossing from at or below the threshold.
#[test]
fn a5_the_confirmation_gate_holds() {
    let rig = Rig::wire();
    let (_token, _home) = rig.engage();
    let served = schema(rig.panel());
    let at = must_confirm_above("budget", Some(0));
    assert_eq!(
        word(&served["params"]["budget"], "confirm_above", Some(0)),
        at,
        "the schema's confirm_above is not the table's"
    );
    assert_eq!(served["params"]["budget"]["danger"], "confirm_above");
    let ceiling = word(&served["params"]["budget"], "max", Some(0));
    let above = at + (ceiling - at) * 0.3;
    let higher = at + (ceiling - at) * 0.5;
    let below = word(&served["params"]["budget"], "default", Some(0));
    assert!(
        below < at,
        "this config starts above the gate; nothing to cross"
    );
    let budget = |v: f64| json!([v, rig.defaults.budget[1], rig.defaults.budget[2]]);

    // Crossing it needs a confirmation, and refusing changes nothing.
    let version = get(rig.panel())["version"].as_u64().expect("version");
    let refused = set_field(rig.panel(), "budget", budget(above), &[]);
    assert_eq!(
        refused["ok"], false,
        "the gate let a crossing through: {refused}"
    );
    assert_eq!(refused["reason"], "needs_confirm");
    assert_eq!(
        refused["field"], "budget",
        "the refusal does not name the field"
    );
    assert_eq!(get(rig.panel())["version"].as_u64(), Some(version));
    assert_eq!(
        word(&get(rig.panel())["params"], "budget", Some(0)),
        below,
        "a refused crossing moved the value"
    );

    // A confirmation naming another field is not this field's.
    let wrong = set_field(rig.panel(), "budget", budget(above), &["ik_damping"]);
    assert_eq!(
        wrong["ok"], false,
        "a confirmation for another field was taken"
    );
    assert_eq!(wrong["reason"], "needs_confirm");

    // Confirmed, it goes through.
    let ok = set_ok(rig.panel(), "budget", budget(above), &["budget"]);
    assert_eq!(word(&ok["params"], "budget", Some(0)), above);

    // Already above: moving within, and further up, is free. Only the step over it is a decision.
    let within = set_ok(rig.panel(), "budget", budget(higher), &[]);
    assert_eq!(word(&within["params"], "budget", Some(0)), higher);

    // Exactly at the threshold is not above it: `>` and not `>=`.
    let exact = set_ok(rig.panel(), "budget", budget(at), &[]);
    assert_eq!(word(&exact["params"], "budget", Some(0)), at);
    let nudged = set_field(rig.panel(), "budget", budget(at * 1.000_001), &[]);
    assert_eq!(
        nudged["ok"], false,
        "standing exactly at the threshold, a step above it needed no confirmation: {nudged}"
    );
    assert_eq!(nudged["reason"], "needs_confirm");

    // And back below, the gate re-arms: one confirmation is not a licence.
    set_ok(rig.panel(), "budget", budget(below), &[]);
    let rearmed = set_field(rig.panel(), "budget", budget(above), &[]);
    assert_eq!(
        rearmed["ok"], false,
        "the gate stayed open after the value came back down: {rearmed}"
    );
    assert_eq!(rearmed["reason"], "needs_confirm");
    println!("the gate holds at {at}, is `>` not `>=`, and re-arms");
    rig.assert_healthy();
}
