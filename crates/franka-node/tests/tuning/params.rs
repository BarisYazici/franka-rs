//! The `franka/<arm>/params/*` wire, driven straight at the node. No bridge, no panel.

// Shared by two test binaries: the asserting `sim_tuning` suite and the `sim_sweep` campaign
// script, which each use a different part of this module. Rust's dead-code analysis is per
// binary and has no view of the other one, so the parts one of them does not reach are not
// dead -- they are the other's. It is a blanket allow: something that went dead in *both*
// would be silent here, so a reader deleting from this module should check both binaries.
#![allow(dead_code)]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use zenoh::Wait;

use crate::{ARM, CLIENT};

/// The subscriber's samples, and the subscriber that must outlive them.
pub type Currents = (
    Arc<Mutex<Vec<(Instant, Value)>>>,
    zenoh::pubsub::Subscriber<()>,
);

fn query(session: &zenoh::Session, verb: &str, payload: Option<String>) -> Value {
    let mut get = session
        .get(format!("franka/{ARM}/params/{verb}"))
        .timeout(Duration::from_secs(5));
    if let Some(body) = payload {
        get = get.payload(body);
    }
    let replies = get.wait().expect("params query");
    let reply = replies.recv().expect("params reply");
    let sample = reply.result().expect("params reply body");
    serde_json::from_slice(&sample.payload().to_bytes()).expect("params reply json")
}

pub fn schema(session: &zenoh::Session) -> Value {
    query(session, "schema", None)
}

pub fn get(session: &zenoh::Session) -> Value {
    query(session, "get", None)
}

pub fn set(session: &zenoh::Session, body: Value) -> Value {
    query(session, "set", Some(body.to_string()))
}

/// One field, with the confirmations the request carries.
pub fn set_field(session: &zenoh::Session, field: &str, value: Value, confirm: &[&str]) -> Value {
    let mut body = json!({"client_id": CLIENT, "params": {field: value}});
    if !confirm.is_empty() {
        body["confirm"] = json!(confirm);
    }
    set(session, body)
}

/// `set_field` that must be accepted, returning the reply.
pub fn set_ok(session: &zenoh::Session, field: &str, value: Value, confirm: &[&str]) -> Value {
    let reply = set_field(session, field, value, confirm);
    assert_eq!(reply["ok"], true, "set {field} refused: {reply}");
    reply
}

/// A `set` whose body is sent verbatim, for payloads a `Value` cannot hold.
pub fn set_raw(session: &zenoh::Session, body: &str) -> Value {
    query(session, "set", Some(body.to_string()))
}

/// Every `params/current` sample with the instant it arrived.
pub fn watch_current(session: &zenoh::Session) -> Currents {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&seen);
    let subscriber = session
        .declare_subscriber(format!("franka/{ARM}/params/current"))
        .callback(move |sample: zenoh::sample::Sample| {
            if let Ok(json) = serde_json::from_slice::<Value>(&sample.payload().to_bytes()) {
                sink.lock().unwrap().push((Instant::now(), json));
            }
        })
        .wait()
        .expect("current subscriber");
    (seen, subscriber)
}

/// A scalar out of a `params` map, or an element of an array field.
pub fn word(params: &Value, field: &str, index: Option<usize>) -> f64 {
    match (&params[field], index) {
        (Value::Array(a), Some(i)) => a[i].as_f64().expect("array word"),
        (Value::Array(a), None) => a[0].as_f64().expect("array word"),
        (v, _) => v
            .as_f64()
            .unwrap_or_else(|| panic!("{field} is not a number: {v}")),
    }
}

/// `slewing[field]`, or zero where the field is not crossing.
pub fn slewing(message: &Value, field: &str) -> f64 {
    message["slewing"][field].as_f64().unwrap_or(0.0)
}
