//! One bound in `LiveTuning::BOUNDS` is the only place a limit exists, and every consumer reads
//! it there: the schema a panel renders, the clamp the node applies, and the value the loop
//! enforces. SIM-PLAN assertion 6.

use std::time::Duration;

use franka::robot::target_control::{ImpedanceGains, LiveTuning, SLEW_TAU};
use franka_node::msg::params::field_schemas;
use serde_json::json;

use super::bounds::bound;
use super::law::{assert_discriminating, gains_of, residual};
use super::metrics::dq_limits;
use super::params::{schema, set_ok, word};
use super::rig::{Driver, Rig};

/// Assertion 6, first leg. The schema served over the wire **is** the table, byte for byte: the
/// node's own serialisation of `LiveTuning::BOUNDS` at this config's defaults. This is both
/// directions at once — a dropped `danger`, an extra field or a wrong `slew_tau_s` all fail.
#[test]
fn the_served_schema_is_the_wire_form_of_the_table() {
    let rig = Rig::wire();
    let served = schema(rig.panel());
    assert_eq!(served["owner"], "node");
    assert_eq!(served["arm"], crate::ARM);
    assert_eq!(
        served["schema_version"].as_u64(),
        Some(u64::from(franka_node::msg::params::SCHEMA_VERSION))
    );

    let expected =
        serde_json::to_value(field_schemas(LiveTuning::BOUNDS, &rig.defaults.to_words()))
            .expect("the table serialises");
    assert_eq!(
        served["params"], expected,
        "the schema on the wire is not the table's wire form"
    );

    // And, independently of that serialisation, every row's numbers come from `BOUNDS`.
    for b in LiveTuning::BOUNDS {
        let row = &served["params"][b.name];
        assert!(!row.is_null(), "{} is not served at all", b.name);
        assert_eq!(word(row, "min", b.index), b.min, "{} min", b.name);
        assert_eq!(word(row, "max", b.index), b.max, "{} max", b.name);
        assert_eq!(row["group"], b.group, "{} group", b.name);
    }
    // No field on the wire that the table does not have.
    for name in served["params"].as_object().expect("params").keys() {
        assert!(
            LiveTuning::BOUNDS.iter().any(|b| b.name == name),
            "the schema serves {name}, which is not in BOUNDS"
        );
    }
    // The damping floor no per-field row can express is published beside them.
    assert_eq!(
        served["relations"][0]["rule"],
        format!(
            "joint_damping[i] >= {} * sqrt(joint_stiffness[i])",
            franka::robot::target_control::MIN_JOINT_DAMPING_RATIO
        ),
        "the pair rule is not published as the table's ratio"
    );
    // `derived.dq_limit` is the connected arm's, from the library.
    dq_limits(rig.fci_version(), &served);
    println!(
        "the schema is the table: {} fields",
        LiveTuning::BOUNDS.len()
    );
}

/// Assertion 6, the claim itself. The bound exists in one place, and all three consumers read
/// it: the schema the panel renders, the clamp the node applies, and the value the loop
/// enforces. Fails if any one of the three disagrees with the other two.
#[test]
fn a6_one_bound_three_consumers() {
    let rig = Rig::new(0.20, 0.30, 40_000);
    let (_token, home) = rig.engage();
    let served = schema(rig.panel());

    // Leg 1: what the panel would draw.
    let ceiling = word(&served["params"]["joint_stiffness"], "max", Some(0));
    assert_eq!(
        ceiling,
        bound("joint_stiffness", Some(0)).max,
        "the schema's ceiling is not the table's"
    );

    let _driver = Driver::start(&rig, home, 0.04, 0.25);
    std::thread::sleep(Duration::from_secs(3));

    // Leg 2: what the clamp stores when asked for more. Deliberately above the bound, so the
    // clamp actually fires -- a value inside it would exercise nothing.
    let t_set = rig.now_ns();
    let reply = set_ok(
        rig.panel(),
        "joint_stiffness",
        json!(vec![ceiling * 1.5; 7]),
        &[],
    );
    let stored = gains_of(&reply["params"]);
    assert_eq!(
        stored.joint_stiffness[0], ceiling,
        "the clamp did not store the bound the schema published"
    );
    assert!(
        reply["clamped"]
            .as_array()
            .expect("clamped")
            .iter()
            .any(|c| c["field"] == "joint_stiffness"),
        "the clamp fired and was not reported"
    );

    std::thread::sleep(Duration::from_secs_f64(10.0 * SLEW_TAU));
    rig.assert_healthy();
    let window = rig
        .trace
        .between(t_set + 2_500_000_000, t_set + 3_000_000_000);
    assert_discriminating(&window, &rig.torque_limits());

    // Leg 3: the gain the torque the loop sent implies. A ladder of candidates around the
    // bound; the best fit must be the bound itself, and unambiguously.
    let candidates: Vec<f64> = [0.5, 0.75, 0.9, 1.0, 1.1, 1.25]
        .iter()
        .map(|f| ceiling * f)
        .collect();
    let mut fits: Vec<(f64, f64)> = candidates
        .iter()
        .map(|k| {
            let gains = ImpedanceGains {
                joint_stiffness: [*k; 7],
                ..stored
            };
            let (mean, _) = residual(
                &window,
                &rig.base_impedance,
                gains,
                &rig.model,
                &rig.template,
            );
            (*k, mean)
        })
        .collect();
    for (k, mean) in &fits {
        println!("  law(K = {k:>8.2}) misfit mean {mean:.6} Nm");
    }
    fits.sort_by(|a, b| a.1.total_cmp(&b.1));
    let (best, best_misfit) = fits[0];
    let (_, second) = fits[1];
    assert_eq!(
        best, ceiling,
        "the torque the loop sent implies K = {best}, not the bound {ceiling} the schema \
         published and the clamp stored"
    );
    assert!(
        best_misfit < second / 10.0,
        "the bound is not the unambiguous best fit: {best_misfit:.6} against {second:.6} Nm"
    );
    println!(
        "schema {ceiling}, clamp {}, torque {best} -- one number, three consumers",
        stored.joint_stiffness[0]
    );
}
