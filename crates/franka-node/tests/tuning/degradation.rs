//! What happens to the loop while a `set` is in flight, and what happens when the client that
//! sent it goes away. SIM-PLAN assertions 7 and 8, plus the lease finding that came out of 8.

use std::time::Duration;

use franka::robot::target_control::SLEW_TAU;
use franka_node::Phase;
use serde_json::json;

use super::law::{assert_discriminating, gains_of, residual};
use super::metrics::{dq_limits, jitter, realtime_kernel, stats};
use super::params::{get, schema, set_ok};
use super::rig::{Driver, Rig};

/// Assertion 7. The 1 kHz thread keeps its period while a `set` crosses.
///
/// **What is asserted depends on the kernel, deliberately.** A single overrun in four thousand
/// cycles on an ordinary kernel is the scheduler, not the tuning path: asserting on it makes a
/// safety test flaky, and a flaky safety test is worse than none because people learn to re-run
/// it. So the exact counts — no cycle over 2 ms, a perfect command success rate — are asserted
/// only on a `PREEMPT_RT` host, where an overrun *is* the code's fault. Everywhere else the test
/// still runs and still fails on a real regression, but through **quantiles**, which measure the
/// loop's work rather than the host's luck: work added to the cycle moves p50 and p99, while a
/// host hiccup moves only the maximum. The hard counts are printed either way.
///
/// This is a *timing* measurement. It does not prove there is no allocation on the path — that
/// is argued from the code (`TorqueLoop::retune` is one atomic load in the common case, and
/// `SlewGains` holds the only `exp`) and is **not instrumented here**. Note also that the tap
/// itself adds a `try_send` to every cycle, so these numbers are an upper bound on the loop's
/// own jitter.
#[test]
fn a7_the_realtime_path_keeps_its_period() {
    let rig = Rig::new(0.20, 0.30, 40_000);
    let (_token, home) = rig.engage();
    let _driver = Driver::start(&rig, home, 0.04, 0.25);

    // Three seconds of baseline, so the percentile is a percentile.
    std::thread::sleep(Duration::from_secs(1));
    let quiet_from = rig.now_ns();
    std::thread::sleep(Duration::from_secs(3));
    let t_set = rig.now_ns();
    set_ok(
        rig.panel(),
        "joint_stiffness",
        json!([400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0]),
        &[],
    );
    // The whole crossing, and a little either side of it.
    std::thread::sleep(Duration::from_secs_f64(4.0 * SLEW_TAU));
    let crossing_to = rig.now_ns();
    std::thread::sleep(Duration::from_secs(1));

    let quiet = rig.trace.between(quiet_from, t_set - 1);
    let crossing = rig.trace.between(t_set, crossing_to);
    let quiet_jitter = jitter(&quiet);
    let crossing_jitter = jitter(&crossing);
    println!("quiet    : {quiet_jitter}");
    println!("crossing : {crossing_jitter}");

    assert!(quiet.len() > 2500, "only {} baseline cycles", quiet.len());
    assert!(
        crossing.len() > 1000,
        "only {} crossing cycles",
        crossing.len()
    );

    // Everywhere: the quantiles. A cycle that got slower does so on every cycle, not on one.
    for (what, j) in [("quiet", &quiet_jitter), ("crossing", &crossing_jitter)] {
        assert!(
            j.p50 < 1.05,
            "{what}: the median cycle is {:.3} ms, not the 1 ms the loop is paced at",
            j.p50
        );
        assert!(
            j.p99 < 1.30,
            "{what}: p99 is {:.3} ms of a 1 ms budget; the loop is doing more work per cycle",
            j.p99
        );
    }
    assert!(
        crossing_jitter.p99 < quiet_jitter.p99 + 0.10,
        "crossing p99 {:.3} ms against quiet p99 {:.3} ms: the crossing costs the loop \
         measurable time",
        crossing_jitter.p99,
        quiet_jitter.p99
    );

    // On a realtime host, and only there, an overrun is the code's fault.
    if realtime_kernel() {
        assert_eq!(
            quiet_jitter.over_two_ms, 0,
            "the loop overran a cycle while nothing was in flight"
        );
        assert_eq!(
            crossing_jitter.over_two_ms, 0,
            "the loop overran a cycle while a set was crossing"
        );
        assert!(
            crossing_jitter.worst < 1.5,
            "worst cycle during the crossing was {:.3} ms",
            crossing_jitter.worst
        );
    } else {
        println!(
            "not a PREEMPT_RT kernel: {} quiet and {} crossing cycles over 2 ms are reported, \
             not asserted",
            quiet_jitter.over_two_ms, crossing_jitter.over_two_ms
        );
    }
    // The robot's own view of the same cycles, and the loop's.
    let dq = dq_limits(rig.fci_version(), &schema(rig.panel()));
    let window = stats(&crossing, &rig.torque_limits(), &dq);
    println!("crossing window: {window}");
    // The robot's own rolling fraction. Exact on a realtime host; on an ordinary one a single
    // missed command shows here as 0.99 and is the same host event the quantiles ignore.
    let floor = if realtime_kernel() { 1.0 } else { 0.95 };
    assert!(
        window.min_success_rate >= floor,
        "the robot reports missed commands while a set crossed: {} (floor {floor})",
        window.min_success_rate
    );
    assert_eq!(
        window.capped, 0,
        "the joint velocity cap cut the goal on {} cycles of a gentle sinusoid",
        window.capped
    );
    assert!(
        !window.envelope_engaged,
        "an envelope engaged during the crossing window"
    );
    assert_eq!(rig.trace.dropped(), 0, "the tap dropped samples");
    rig.assert_healthy();
}

/// Assertion 8, the half a simulator can answer. The panel dies mid-crossing; the loop holds the
/// value it was given and finishes the ramp.
///
/// The other half — "kill the node, the panel says stale rather than showing a frozen sparkline
/// as live" — is panel-side and is **not** tested here.
#[test]
fn a8_the_panel_can_die_mid_crossing() {
    let mut rig = Rig::new(0.20, 0.30, 40_000);
    let (_token, home) = rig.engage();
    let _driver = Driver::start(&rig, home, 0.04, 0.25);
    std::thread::sleep(Duration::from_secs(3));

    let before = gains_of(&get(rig.panel())["params"]);
    let t_set = rig.now_ns();
    let reply = set_ok(
        rig.panel(),
        "joint_stiffness",
        json!([120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0]),
        &[],
    );
    let target = gains_of(&reply["params"]);

    // A fifth of a time constant in, the panel process dies outright.
    std::thread::sleep(Duration::from_secs_f64(0.2 * SLEW_TAU));
    rig.kill_panel();
    std::thread::sleep(Duration::from_secs_f64(10.0 * SLEW_TAU));

    let window = rig
        .trace
        .between(t_set + 2_500_000_000, t_set + 3_000_000_000);
    assert_discriminating(&window, &rig.torque_limits());
    let (to_target, worst) = residual(
        &window,
        &rig.base_impedance,
        target,
        &rig.model,
        &rig.template,
    );
    let (to_before, _) = residual(
        &window,
        &rig.base_impedance,
        before,
        &rig.model,
        &rig.template,
    );
    println!(
        "with the panel gone, over {} cycles: |tau-law(target)| {to_target:.6} (max \
         {worst:.6}) Nm, |tau-law(before)| {to_before:.6} Nm",
        window.len()
    );
    assert!(
        to_target < 1e-2,
        "the loop did not finish the ramp its dead client asked for: {to_target:.6} Nm"
    );
    assert!(
        to_before > 100.0 * to_target,
        "the loop is nearer where it started than where it was sent (before {to_before:.6}, \
         target {to_target:.6} Nm)"
    );
    rig.assert_healthy();
}

/// The lease and the panel are not the same thing, and this is why the panel must keep its own
/// Zenoh session: the session holding the liveliness token *is* the lease, so closing it ends
/// the arm's session. A panel that shared a session with the teleop client would stop the arm
/// by exiting.
#[test]
fn closing_the_leaseholders_session_ends_the_session() {
    let rig = Rig::wire();
    let (token, home) = rig.engage();
    let mut sender = super::rig::Sender::new(&rig);
    sender.hold(home, Duration::from_millis(600));
    assert_eq!(
        Phase::from_u8(rig.state().phase),
        Some(Phase::Active),
        "the session did not reach Active"
    );

    drop(token);
    let ended = rig
        .watch
        .wait_for(Duration::from_secs(5), |s| {
            Phase::from_u8(s.phase) != Some(Phase::Active)
        })
        .expect("the session outlived the lease");
    println!(
        "the lease went and the arm left Active for phase {:?}",
        Phase::from_u8(ended.phase)
    );
    assert_ne!(Phase::from_u8(ended.phase), Some(Phase::Active));
}
