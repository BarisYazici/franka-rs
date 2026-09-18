//! The `params/*` surface against the fake arm: what a `set` moves, what it refuses, and what
//! `get` and `current` say while it does.

use std::sync::mpsc;

use franka::robot::target_control::LiveTuning;
use serde_json::{json, Value};

use super::{arm_config, rig, Rig, CLIENT};
use crate::arm::{Event, ParamsVerb};
use crate::msg::params::{ParamValue, SetRequest};

impl Rig {
    /// One `params/*` query, answered synchronously by the machine.
    fn params(&mut self, verb: ParamsVerb, payload: &str) -> Value {
        let (tx, rx) = mpsc::channel();
        self.machine.handle(Event::Params(
            verb,
            payload.as_bytes().to_vec(),
            Box::new(move |json| tx.send(json).unwrap()),
        ));
        serde_json::from_str(&rx.recv().unwrap()).expect("the node replies with JSON")
    }

    fn get(&mut self) -> Value {
        self.params(ParamsVerb::Get, "")
    }

    /// A `set` of `params` by [`CLIENT`], confirming `confirm`.
    fn set(&mut self, params: Value, confirm: &[&str]) -> Value {
        let request = json!({
            "client_id": CLIENT,
            "confirm": confirm,
            "params": params,
        });
        self.params(ParamsVerb::Set, &request.to_string())
    }

    /// The session's tuning targets as the fake control holds them.
    fn tuning(&self) -> LiveTuning {
        self.fake
            .tuning
            .lock()
            .unwrap()
            .expect("a Cartesian session is running")
    }
}

fn number(value: &Value) -> f64 {
    value.as_f64().expect("a number")
}

#[test]
fn the_schema_is_served_without_a_session_and_names_the_arm_and_the_boot() {
    let mut rig = rig();
    let schema = rig.params(ParamsVerb::Schema, "");
    assert_eq!(schema["arm"], "t");
    assert_eq!(schema["owner"], "node");
    assert_eq!(schema["schema_version"], 1);
    assert_eq!(schema["boot_id"], crate::boot_id());
    assert_eq!(schema["params"].as_object().unwrap().len(), 9);
    // The block the panel's feedforward speed-cap warning cannot be computed without.
    assert_eq!(schema["derived"]["cartesian_preset"]["reference"], 750.0);
    assert_eq!(schema["derived"]["leash"]["translation"], 0.025);
}

#[test]
fn get_before_any_set_is_the_config_at_version_zero() {
    let mut rig = rig();
    let get = rig.get();
    assert_eq!(get["version"], 0);
    assert_eq!(get["origin"], Value::Null);
    assert_eq!(get["dirty"], false);
    assert_eq!(get["slewing"], json!({}));
    assert_eq!(get["owner"], "node");
    let config = arm_config().live_tuning();
    assert_eq!(number(&get["params"]["cartesian_stiffness"]), 750.0);
    assert_eq!(
        number(&get["params"]["joint_damping"][1]),
        config.joint_damping[1]
    );
    assert_eq!(number(&get["params"]["budget"][0]), config.budget[0]);
}

#[test]
fn a_set_without_a_session_is_not_ready_and_changes_nothing() {
    let mut rig = rig();
    let reply = rig.set(json!({"ik_damping": 0.2}), &[]);
    assert_eq!(reply["ok"], false);
    assert_eq!(reply["reason"], "not_ready");
    assert_eq!(reply["version"], 0);
    assert_eq!(rig.get()["version"], 0);
    assert_eq!(number(&rig.get()["params"]["ik_damping"]), 0.05);
}

#[test]
fn a_joints_session_has_no_live_tuning_to_move() {
    let mut rig = rig();
    let mut request = crate::msg::CmdRequest::new(CLIENT);
    request.mode = Some(crate::msg::Kind::Joints);
    rig.activate_with(request);
    let reply = rig.set(json!({"ik_damping": 0.2}), &[]);
    assert_eq!(reply["reason"], "not_ready");
    // And `get` falls back to what the next Cartesian session would start at.
    assert_eq!(number(&rig.get()["params"]["ik_damping"]), 0.05);
}

#[test]
fn a_set_moves_the_session_and_leaves_every_field_it_does_not_carry_alone() {
    let mut rig = rig();
    rig.activate();
    let before = rig.tuning();
    // 16 Nm/rad puts the damping floor at 1, which is where the softest joint's damping
    // already is, so the pair rule has nothing to say and nothing else may move either.
    let reply = rig.set(
        json!({"ik_damping": 0.2, "joint_stiffness": vec![16.0; 7]}),
        &[],
    );
    assert_eq!(reply["ok"], true, "{reply}");
    assert_eq!(reply["version"], 1);
    assert_eq!(reply["clamped"], json!([]));
    let after = rig.tuning();
    assert_eq!(after.ik_damping, 0.2);
    assert_eq!(after.joint_stiffness, [16.0; 7]);
    assert_eq!(after.joint_damping, before.joint_damping);
    // Everything else, bit for bit.
    assert_eq!(after.budget, before.budget);
    assert_eq!(after.cartesian_stiffness, before.cartesian_stiffness);
    assert_eq!(after.ik_nullspace_gain, before.ik_nullspace_gain);
    assert_eq!(
        after.velocity_feedforward_cutoff.to_bits(),
        before.velocity_feedforward_cutoff.to_bits()
    );
    // And the reply carries the whole effective set, not just what moved.
    assert_eq!(number(&reply["params"]["ik_damping"]), 0.2);
    assert_eq!(number(&reply["params"]["cartesian_stiffness"]), 750.0);
    assert_eq!(rig.get()["dirty"], true);
}

#[test]
fn a_refused_set_leaves_the_session_and_the_version_where_they_were() {
    let mut rig = rig();
    rig.activate();
    let before = rig.tuning();
    let refusals = [
        (json!({"nope": 1.0}), "unknown_field"),
        (json!({"ik_damping": "x"}), "type"),
        (json!({"budget": [1.0, 2.0]}), "length"),
        // Rejected outright rather than clamped to the floor: zero damping is a different
        // request, not a softer one.
        (json!({"ik_damping": 0.0}), "invalid"),
        (json!({"cartesian_stiffness": -5.0}), "invalid"),
    ];
    for (params, reason) in refusals {
        let reply = rig.set(params.clone(), &[]);
        assert_eq!(reply["ok"], false, "{params} was accepted");
        assert_eq!(reply["reason"], reason, "{params}");
        assert_eq!(reply["version"], 0, "{params}");
        assert_eq!(rig.tuning(), before, "{params} moved the session");
    }
}

#[test]
fn a_stale_base_version_is_refused_and_the_current_one_comes_back() {
    let mut rig = rig();
    rig.activate();
    let request = json!({"client_id": CLIENT, "base_version": 3, "params": {"ik_damping": 0.2}});
    let reply = rig.params(ParamsVerb::Set, &request.to_string());
    assert_eq!(reply["reason"], "stale");
    assert_eq!(reply["version"], 0);
    // The matching one goes through, and the next one with the same base does not.
    let request = json!({"client_id": CLIENT, "base_version": 0, "params": {"ik_damping": 0.2}});
    let reply = rig.params(ParamsVerb::Set, &request.to_string());
    assert_eq!(reply["ok"], true, "{reply}");
    assert_eq!(reply["version"], 1);
    let request = json!({"client_id": CLIENT, "base_version": 0, "params": {"ik_damping": 0.3}});
    assert_eq!(
        rig.params(ParamsVerb::Set, &request.to_string())["reason"],
        "stale"
    );
    assert_eq!(rig.tuning().ik_damping, 0.2);
}

#[test]
fn a_set_with_no_client_id_is_refused() {
    let mut rig = rig();
    rig.activate();
    let request = json!({"client_id": 0, "params": {"ik_damping": 0.2}});
    let reply = rig.params(ParamsVerb::Set, &request.to_string());
    assert_eq!(
        (reply["reason"].as_str(), reply["field"].as_str()),
        (Some("type"), Some("client_id"))
    );
    assert_eq!(rig.tuning().ik_damping, 0.05);
}

#[test]
fn a_value_past_a_bound_is_stored_at_the_bound_and_reported() {
    let mut rig = rig();
    rig.activate();
    let reply = rig.set(
        json!({"joint_damping": [95.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]}),
        &[],
    );
    assert_eq!(reply["ok"], true, "{reply}");
    assert_eq!(rig.tuning().joint_damping[0], 60.0);
    assert_eq!(
        reply["clamped"],
        json!([{"field": "joint_damping", "index": 0, "requested": 95.0, "stored": 60.0}])
    );
    assert_eq!(number(&reply["params"]["joint_damping"][0]), 60.0);
}

/// The pair rule reaches a field the request did not carry, and what it did is reported with
/// the value that field had as the "request".
#[test]
fn raising_a_spring_raises_the_damping_under_it_and_says_so() {
    let mut rig = rig();
    rig.activate();
    let damping = rig.tuning().joint_damping[0];
    let reply = rig.set(json!({"joint_stiffness": vec![1200.0; 7]}), &[]);
    assert_eq!(reply["ok"], true, "{reply}");
    let floor = LiveTuning::joint_damping_floor(1200.0);
    assert_eq!(rig.tuning().joint_damping[0], floor);
    let clamped = &reply["clamped"].as_array().expect("a list")[0];
    assert_eq!(clamped["field"], "joint_damping");
    assert_eq!(number(&clamped["requested"]), damping);
    assert_eq!(number(&clamped["stored"]), floor);
}

#[test]
fn crossing_a_confirmation_threshold_needs_the_confirmation_once() {
    let mut rig = rig();
    rig.activate();
    // 0.9 m/s is over the 0.85 the schema publishes, from 0.3 below it.
    let reply = rig.set(json!({"budget": [0.9, 0.5, 20.0]}), &[]);
    assert_eq!(reply["reason"], "needs_confirm");
    assert_eq!(reply["field"], "budget");
    assert_eq!(rig.tuning().budget[0], 0.3, "the refusal stored something");
    let reply = rig.set(json!({"budget": [0.9, 0.5, 20.0]}), &["budget"]);
    assert_eq!(reply["ok"], true, "{reply}");
    assert_eq!(rig.tuning().budget[0], 0.9);
    // Already above it, moving within the band: nothing new to confirm.
    let reply = rig.set(json!({"budget": [1.1, 0.5, 20.0]}), &[]);
    assert_eq!(reply["ok"], true, "{reply}");
    assert_eq!(rig.tuning().budget[0], 1.1);
    // And coming back down needs none either.
    let reply = rig.set(json!({"budget": [0.3, 0.5, 20.0]}), &[]);
    assert_eq!(reply["ok"], true, "{reply}");
}

#[test]
fn a_confirm_naming_a_field_the_node_does_not_know_is_ignored() {
    let mut rig = rig();
    rig.activate();
    let reply = rig.set(json!({"ik_damping": 0.2}), &["spatial_scale", "budget"]);
    assert_eq!(reply["ok"], true, "{reply}");
    assert_eq!(rig.tuning().ik_damping, 0.2);
}

/// What is still to come, per field: a filtered field counts down from 1, a stepped one is in
/// force at once and says nothing, and a gated one on its way down reports its ramp.
#[test]
fn slewing_reports_what_is_left_of_the_last_change() {
    let mut rig = rig();
    rig.activate();
    let reply = rig.set(
        json!({"joint_stiffness": vec![100.0; 7], "velocity_feedforward_cutoff": 50.0}),
        &[],
    );
    let slewing = reply["slewing"].as_object().expect("a map");
    assert!(
        number(&slewing["joint_stiffness"]) > 0.99,
        "a change just accepted has all of itself to go: {slewing:?}"
    );
    assert!(
        !slewing.contains_key("velocity_feedforward_cutoff"),
        "a stepped field is in force and has nothing to report: {slewing:?}"
    );
    assert!(!slewing.contains_key("budget"), "{slewing:?}");
    // A lowered budget is a ramp, and the ramp is what is reported.
    let reply = rig.set(json!({"budget": [0.1, 0.5, 20.0]}), &[]);
    let slewing = reply["slewing"].as_object().expect("a map");
    assert!(number(&slewing["budget"]) > 0.99, "{slewing:?}");
    // Raising it is a step, so the next one has nothing left.
    let reply = rig.set(json!({"budget": [0.3, 0.5, 20.0]}), &[]);
    assert_eq!(reply["slewing"], json!({}), "a raise is in force at once");
}

#[test]
fn the_origin_echoes_the_client_that_set_it() {
    let mut rig = rig();
    rig.activate();
    rig.set(json!({"ik_damping": 0.2}), &[]);
    let get = rig.get();
    assert_eq!(get["origin"]["by"], CLIENT);
    assert_eq!(get["origin"]["version"], 1);
    assert_eq!(get["version"], 1);
    assert!(get["origin"]["at_ns"].as_u64().unwrap() > 0);
}

/// A new session starts from the config, deliberately (DESIGN-rt 4.5), so the values a client
/// set during the last one are gone and nobody is named as their author.
#[test]
fn a_session_ending_reverts_the_values_and_forgets_who_set_them() {
    let mut rig = rig();
    rig.activate();
    rig.set(json!({"ik_damping": 0.2}), &[]);
    assert_eq!(rig.get()["dirty"], true);
    rig.ok(crate::arm::Verb::Stop, CLIENT);
    let get = rig.get();
    assert_eq!(number(&get["params"]["ik_damping"]), 0.05);
    assert_eq!(get["origin"], Value::Null);
    assert_eq!(get["dirty"], false);
    assert_eq!(get["slewing"], json!({}));
    // The version counts accepted sets and never goes backwards, so a `base_version` from
    // before the session ended is still stale.
    assert_eq!(get["version"], 1);
}

#[test]
fn current_goes_out_on_every_change_and_once_a_second_without_one() {
    let mut rig = rig();
    rig.activate();
    let before = rig.params.lock().unwrap().len();
    rig.set(json!({"ik_damping": 0.2}), &[]);
    assert_eq!(
        rig.params.lock().unwrap().len(),
        before + 1,
        "a change did not publish"
    );
    let last = rig
        .params
        .lock()
        .unwrap()
        .last()
        .cloned()
        .expect("a sample");
    assert_eq!(last.version, 1);
    assert_eq!(last.params["ik_damping"], ParamValue::Scalar(0.2));
    assert_eq!(last.origin.expect("an origin").by, CLIENT);
    // A tick inside the period adds nothing; the period itself is `CURRENT_PERIOD`, read from
    // one place, and a test that waited a second for it would pay a second for nothing.
    let after = rig.params.lock().unwrap().len();
    rig.machine.tick();
    assert_eq!(
        rig.params.lock().unwrap().len(),
        after,
        "a tick republished inside the period"
    );
}

/// The request envelope is the contract's and nothing else: a `set` cannot be smuggled through
/// with a key the node does not know.
#[test]
fn an_unknown_envelope_key_is_refused() {
    let mut rig = rig();
    rig.activate();
    let request = json!({"client_id": CLIENT, "params": {"ik_damping": 0.2}, "mode": "joints"});
    let reply = rig.params(ParamsVerb::Set, &request.to_string());
    assert_eq!(reply["reason"], "type");
    assert_eq!(rig.tuning().ik_damping, 0.05);
    // And the bytes a `set` does take are a `SetRequest`, not a `CmdRequest`.
    let request: SetRequest =
        serde_json::from_str(r#"{"client_id": 7, "params": {"ik_damping": 0.2}}"#).expect("valid");
    assert_eq!(request.client_id, 7);
}
