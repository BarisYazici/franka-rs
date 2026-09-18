//! Does a value moved through `params/set` reach the control law, cross to it smoothly, and get
//! reported honestly? SIM-PLAN assertions 1, 2 and 3, one `#[test]` each.

use std::time::{Duration, Instant};

use franka::robot::target_control::{ImpedanceGains, LiveTuning, TuningPolicy, SLEW_TAU};
use franka::DELTA_T;
use serde_json::{json, Value};

use super::law::{assert_discriminating, crossing, gains_of, residual};
use super::metrics::{dq_limits, stats};
use super::params::{get, set_ok, slewing, watch_current};
use super::rig::{Driver, Rig};
use super::tap::Sample;

/// The stiffness a `set` moves the joint springs to: ten times the preset, well inside the
/// bound, so the change is unmistakable in the torque without going near the clamp.
const RAISED: f64 = 400.0;

fn raised() -> Value {
    json!(vec![RAISED; 7])
}

/// The `BOUNDS` row of `joint_stiffness[0]`, which is where this field's policy and time
/// constant live. Nothing here writes either down.
fn stiffness_policy() -> TuningPolicy {
    let word = LiveTuning::BOUNDS
        .iter()
        .position(|b| b.name == "joint_stiffness" && b.index == Some(0))
        .expect("joint_stiffness[0] has a bounds row");
    LiveTuning::BOUNDS[word].policy
}

fn slew_tau() -> f64 {
    match stiffness_policy() {
        TuningPolicy::Slew { tau } => tau,
        other => panic!("joint_stiffness is not slewed: {other:?}"),
    }
}

/// Everything a crossing test needs: the trace either side of one accepted `set`, with the
/// windows anchored on *observed* cycles rather than on a sleep.
struct Crossing {
    old: ImpedanceGains,
    new: ImpedanceGains,
    /// Cycles before the change had any effect.
    baseline: Vec<Sample>,
    /// Cycles after it had all of it.
    settled: Vec<Sample>,
    /// (seconds since the `set`, fraction of the change the torque has travelled).
    alphas: Vec<(f64, f64)>,
}

/// Raises `joint_stiffness` on a tracking arm and slices the trace around it.
fn run_crossing(rig: &Rig) -> Crossing {
    let before = get(rig.panel());
    let old = gains_of(&before["params"]);
    let t_set = rig.now_ns();
    let reply = set_ok(rig.panel(), "joint_stiffness", raised(), &[]);
    let new = gains_of(&reply["params"]);
    assert_eq!(
        new.joint_stiffness, [RAISED; 7],
        "the node stored something other than what it was asked for and did not clamp"
    );
    assert_ne!(
        old.joint_stiffness, new.joint_stiffness,
        "the set did not change the stiffness, so there is nothing to detect"
    );

    // Long enough for the filter to be over: ten time constants.
    std::thread::sleep(Duration::from_secs_f64(10.0 * slew_tau()));
    rig.assert_healthy();

    let post = rig.trace.between(t_set, rig.now_ns());
    let alphas = crossing(
        &post,
        t_set,
        &rig.base_impedance,
        old,
        new,
        &rig.model,
        &rig.template,
    );
    assert!(
        alphas.len() > 1500,
        "only {} cycles of crossing recovered; the loop was not running",
        alphas.len()
    );

    // Anchor on observations, never on a sleep. `moved` is the first cycle the change shows in
    // the torque at all -- the threshold is half of one cycle of the filter, so the baseline
    // cannot contain a cycle that has already begun to cross -- and `arrived` is the first cycle
    // after which it never again falls below 0.99.
    let resolution = 0.5 * stiffness_policy().slew_gain(DELTA_T);
    let moved = alphas
        .iter()
        .position(|(_, a)| *a > resolution)
        .expect("the torque never moved toward the new gains");
    let arrived = alphas
        .iter()
        .rposition(|(_, a)| *a < 0.99)
        .expect("the torque was already at the new gains before the set")
        + 1;
    assert!(
        arrived < alphas.len() - 300,
        "the crossing had not settled with {} cycles left",
        alphas.len() - arrived
    );
    let before_ns = t_set + (alphas[moved].0 * 1e9) as u64;
    let settled_ns = t_set + (alphas[arrived].0 * 1e9) as u64;

    let baseline = rig
        .trace
        .between(before_ns.saturating_sub(400_000_000), before_ns - 1);
    let settled = rig.trace.between(settled_ns, settled_ns + 500_000_000);
    Crossing {
        old,
        new,
        baseline,
        settled,
        alphas,
    }
}

/// Assertion 1. A `set` moves the live value and the control law's next cycles use it — asserted
/// against the torque the loop produced, not against the stored struct.
#[test]
fn a1_the_set_reaches_the_torque() {
    let rig = Rig::new(0.20, 0.30, 40_000);
    let (_token, home) = rig.engage();
    let _driver = Driver::start(&rig, home, 0.04, 0.25);
    std::thread::sleep(Duration::from_secs(3));

    let c = run_crossing(&rig);
    let limits = rig.torque_limits();
    assert_discriminating(&c.baseline, &limits);
    assert_discriminating(&c.settled, &limits);

    let fit = |window: &[Sample], gains| {
        residual(
            window,
            &rig.base_impedance,
            gains,
            &rig.model,
            &rig.template,
        )
    };
    let (before_old, _) = fit(&c.baseline, c.old);
    let (before_new, _) = fit(&c.baseline, c.new);
    let (after_old, _) = fit(&c.settled, c.old);
    let (after_new, after_new_max) = fit(&c.settled, c.new);
    let dq = dq_limits(rig.fci_version(), &super::params::schema(rig.panel()));
    println!("baseline: {}", stats(&c.baseline, &limits, &dq));
    println!("settled : {}", stats(&c.settled, &limits, &dq));
    println!(
        "before: |tau-law(old)| {before_old:.6}  |tau-law(new)| {before_new:.6} Nm\n\
         after : |tau-law(old)| {after_old:.6}  |tau-law(new)| {after_new:.6} \
         (max {after_new_max:.6}) Nm"
    );

    // The right answer at both ends of the change, and the wrong one failing at both ends. The
    // claims are ratios, so they say "this discriminates" rather than resting on a window
    // boundary landing on an exact cycle.
    assert!(
        before_old < before_new / 1e4,
        "before the set the loop was not unambiguously running the old gains: they miss by \
         {before_old:.9} Nm against the new gains' {before_new:.6} Nm"
    );
    assert!(
        before_old < 1e-6,
        "before the set the loop was not running the old gains: {before_old:.9} Nm"
    );
    assert!(
        before_new > 0.1,
        "the new gains already fit before the set ({before_new:.6} Nm), so nothing is being \
         discriminated"
    );
    assert!(
        after_new < 1e-2,
        "after the crossing the loop is not running the gains the node said it stored: \
         {after_new:.6} Nm"
    );
    assert!(
        after_old > 100.0 * after_new,
        "the old gains fit the settled torque nearly as well as the stored ones \
         (old {after_old:.6}, new {after_new:.6} Nm); this does not discriminate"
    );
    assert!(
        after_old > 0.05,
        "the change in torque is only {after_old:.6} Nm; too small to conclude from"
    );
}

/// Assertion 2. The change ramps over `SLEW_TAU` and never steps — the per-cycle delta stays
/// under the slew rate for the whole ramp, and the ramp is the filter the table declares.
#[test]
fn a2_the_crossing_is_a_slew_and_never_a_step() {
    let rig = Rig::new(0.20, 0.30, 40_000);
    let (_token, home) = rig.engage();
    let _driver = Driver::start(&rig, home, 0.04, 0.25);
    std::thread::sleep(Duration::from_secs(3));

    let c = run_crossing(&rig);
    let tau = slew_tau();
    assert_eq!(tau, SLEW_TAU, "this field's tau is not the shared SLEW_TAU");
    // The gap one cycle closes, straight off the policy the loop reads.
    let per_cycle = stiffness_policy().slew_gain(DELTA_T);

    let mut worst_step = 0.0f64;
    let mut worst_step_at = 0.0;
    let mut worst_error = 0.0f64;
    let mut worst_error_at = 0.0;
    let mut backwards = 0usize;
    for w in c.alphas.windows(2) {
        let step = w[1].1 - w[0].1;
        if step.abs() > worst_step {
            worst_step = step.abs();
            worst_step_at = w[1].0;
        }
        if step < -2.0 * per_cycle {
            backwards += 1;
        }
    }
    for (t, a) in &c.alphas {
        if *t <= 0.0 {
            continue;
        }
        let ideal = 1.0 - (-t / tau).exp();
        if (a - ideal).abs() > worst_error {
            worst_error = (a - ideal).abs();
            worst_error_at = *t;
        }
    }
    println!(
        "{} cycles: largest per-cycle step {worst_step:.6} at t={worst_step_at:.3}s \
         (one cycle of the filter is {per_cycle:.6}); worst |alpha - (1-e^(-t/{tau}))| \
         {worst_error:.5} at t={worst_error_at:.3}s; steps backwards {backwards}",
        c.alphas.len()
    );

    assert!(
        worst_step <= 1.05 * per_cycle,
        "a cycle moved {worst_step:.6} of the change at t={worst_step_at:.3}s, more than the \
         filter's {per_cycle:.6}: that is a step, not a slew"
    );
    assert!(
        worst_error < 0.02,
        "the crossing is not the declared filter: worst deviation {worst_error:.5} from \
         1-e^(-t/{tau}) at t={worst_error_at:.3}s"
    );
    assert_eq!(
        backwards, 0,
        "the crossing went backwards on {backwards} cycles"
    );
}

/// Assertion 3. `current` reports the change, and `slewing` is the fraction **still to go** — it
/// counts down, by value, on the filter the schema declares. A node publishing `1 - remaining`
/// fails here rather than inverting the ghost slider silently.
#[test]
fn a3_slewing_is_the_fraction_still_to_go() {
    let rig = Rig::new(0.20, 0.30, 40_000);
    let (currents, _subscriber) = watch_current(rig.panel());
    let (_token, home) = rig.engage();
    let _driver = Driver::start(&rig, home, 0.04, 0.25);
    std::thread::sleep(Duration::from_secs(2));

    let before = get(rig.panel());
    assert_eq!(
        before["version"], 0,
        "a fresh session has taken a set already"
    );
    assert_eq!(before["dirty"], false, "a fresh session is already dirty");
    assert!(before["origin"].is_null(), "a fresh session has an origin");

    let accepted_at = Instant::now();
    let reply = set_ok(rig.panel(), "joint_stiffness", raised(), &[]);
    assert_eq!(
        reply["version"], 1,
        "version did not advance on an accepted set"
    );
    assert!(
        slewing(&reply, "joint_stiffness") > 0.99,
        "the reply says the crossing is nearly over before it began: {}",
        reply["slewing"]
    );

    // `current` is published at 1 Hz, which cannot resolve a 0.3 s ramp, so the ramp is read by
    // polling `get` — the same value off the same arm thread.
    let tau = slew_tau();
    let mut series: Vec<(f64, f64)> = Vec::new();
    while accepted_at.elapsed() < Duration::from_secs_f64(8.0 * tau) {
        let message = get(rig.panel());
        series.push((
            accepted_at.elapsed().as_secs_f64(),
            slewing(&message, "joint_stiffness"),
        ));
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        series.len() > 40,
        "only {} samples of the ramp; too few to pin its direction",
        series.len()
    );

    let first = series[0].1;
    let last = series[series.len() - 1].1;
    println!(
        "{} samples over {:.2}s: slewing {first:.4} -> {last:.4}",
        series.len(),
        series[series.len() - 1].0
    );
    assert!(
        first > 0.9,
        "slewing starts at {first:.4}; it must start near 1 (the whole change still to go)"
    );
    assert_eq!(last, 0.0, "slewing never reached zero: {last:.4}");
    for w in series.windows(2) {
        assert!(
            w[1].1 <= w[0].1 + 1e-9,
            "slewing rose from {:.6} to {:.6} between t={:.3}s and t={:.3}s: it is counting up",
            w[0].1,
            w[1].1,
            w[0].0,
            w[1].0
        );
    }

    // By value, not merely by direction: the time constant the fraction implies is the one the
    // schema published. `1 - remaining` would imply a negative tau and fail outright.
    let mut checked = 0;
    for (t, remaining) in &series {
        if !(0.1..=0.9).contains(remaining) {
            continue;
        }
        let implied = -t / remaining.ln();
        assert!(
            (implied - tau).abs() / tau < 0.2,
            "slewing {remaining:.4} at t={t:.3}s implies tau {implied:.4}s, not the published \
             {tau}s"
        );
        checked += 1;
    }
    assert!(
        checked >= 5,
        "only {checked} samples landed in the middle of the ramp; the decay was not pinned"
    );
    println!("{checked} samples of the ramp imply tau within 20% of the published {tau}s");

    // What `current` carries, and what it means: the target, with the fraction still to come.
    let seen = currents.lock().unwrap().clone();
    let at_v1: Vec<&(Instant, Value)> = seen.iter().filter(|(_, v)| v["version"] == 1).collect();
    assert!(
        !at_v1.is_empty(),
        "no `current` was published for the accepted set"
    );
    let (_, first_v1) = at_v1[0];
    assert!(
        (first_v1["t_node_ns"].as_u64().is_some()),
        "current carries no node timestamp"
    );
    for i in 0..7 {
        assert_eq!(
            first_v1["params"]["joint_stiffness"][i].as_f64(),
            Some(RAISED),
            "current does not carry the target the set asked for"
        );
    }
    assert!(
        slewing(first_v1, "joint_stiffness") > 0.9,
        "the immediate `current` does not say the change is still arriving"
    );

    let after = get(rig.panel());
    assert_eq!(after["version"], 1);
    assert_eq!(
        after["dirty"], true,
        "the values differ from the TOML's but dirty is false"
    );
    assert_eq!(
        after["origin"]["by"].as_u64(),
        Some(u64::from(crate::CLIENT)),
        "origin does not echo the client that set it"
    );
    assert_eq!(after["origin"]["version"], 1);
    assert_eq!(
        after["slewing"].as_object().map(|o| o.len()),
        Some(0),
        "slewing is not empty after the crossing: {}",
        after["slewing"]
    );
    rig.assert_healthy();
}
