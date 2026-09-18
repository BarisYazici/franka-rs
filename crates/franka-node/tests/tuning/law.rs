//! What the torque should have been, for a given set of gains.
//!
//! The oracle is `impedance_torques` — the very function `TorqueLoop::cycle` calls. That makes
//! the *law* untested here by construction: a bug in it is invisible, and a small residual
//! means "both sides used the same law with the same gains", never "the law is right". What is
//! independent, and what these tests are about, is the **gain path**: the gains fed to the
//! recomputation are parsed out of the node's reply JSON off the wire, while the torque comes
//! from the loop's own live `LiveTuning`.
//!
//! **How fine a difference this can see, measured rather than assumed.** A mutation that made
//! the loop apply a joint stiffness 10% away from what the node reported was caught, but with
//! only 22% of margin on [`residual`]'s binding threshold; a 5% error would very likely have
//! passed. The discrimination floor of every torque-side claim here is therefore roughly
//! **±8-10% of a gain**. "The set reaches the control law" is true to that precision and no
//! finer, and a pass must not be read as proof of exactness.
//!
//! [`expected`] reconstructs what the loop actually sends —
//! `clamp(law + tau_envelope + tau_position)`, `torque/mod.rs:174-190` — rather than the bare
//! law. [`assert_discriminating`] is the guard that makes a residual mean anything at all: on a
//! cycle where the envelope contributes, the two barrier shares are themselves functions of the
//! gains, and on a cycle where the torque sits on `torque_limits` every candidate gain set fits
//! equally well. Both cases are refused rather than reported.

// Shared by two test binaries: the asserting `sim_tuning` suite and the `sim_sweep` campaign
// script, which each use a different part of this module. Rust's dead-code analysis is per
// binary and has no view of the other one, so the parts one of them does not reach are not
// dead -- they are the other's. It is a blanket allow: something that went dead in *both*
// would be silent here, so a reader deleting from this module should check both binaries.
#![allow(dead_code)]

use franka::model::{Frame, Model};
use franka::robot::target_control::{impedance_torques, ImpedanceGains, ImpedanceOptions};
use franka::RobotState;
use serde_json::Value;

use super::tap::Sample;

/// The gains a `params` map (from `get`, `current` or a `set` reply) stands for. The twelve
/// Cartesian words are derived from the one translational stiffness by the library's own rule,
/// which is the only place that rule may live.
pub fn gains_of(params: &Value) -> ImpedanceGains {
    let seven = |name: &str| -> [f64; 7] {
        let v = params[name].as_array().unwrap_or_else(|| panic!("{name}"));
        std::array::from_fn(|i| v[i].as_f64().unwrap_or_else(|| panic!("{name}[{i}]")))
    };
    let scaled = ImpedanceGains::scaled_cartesian(
        params["cartesian_stiffness"]
            .as_f64()
            .expect("cartesian_stiffness"),
    );
    ImpedanceGains {
        joint_stiffness: seven("joint_stiffness"),
        joint_damping: seven("joint_damping"),
        ..scaled
    }
}

/// The torque the loop would have sent at `sample` with `gains`.
pub fn expected(
    base: &ImpedanceOptions,
    gains: ImpedanceGains,
    model: &Model,
    template: &RobotState,
    sample: &Sample,
) -> [f64; 7] {
    let options = base.with_gains(gains);
    let mut state = *template;
    state.q = sample.q;
    state.dq = sample.dq;
    let law = impedance_torques(
        &options,
        &model.zero_jacobian(Frame::EndEffector, &state),
        &sample.q_goal,
        &sample.dq_goal,
        &sample.q,
        &sample.dq,
        &model.coriolis(&state),
    );
    let limits = &options.torque_limits;
    std::array::from_fn(|i| {
        (law[i] + sample.tau_envelope[i] + sample.tau_position[i]).clamp(-limits[i], limits[i])
    })
}

/// Mean and worst `max_i |tau_i - expected_i|` over a window, Nm.
pub fn residual(
    window: &[Sample],
    base: &ImpedanceOptions,
    gains: ImpedanceGains,
    model: &Model,
    template: &RobotState,
) -> (f64, f64) {
    assert!(!window.is_empty(), "an empty window has no residual");
    let mut worst = 0.0f64;
    let mut sum = 0.0;
    for s in window {
        let d = max_abs_diff(&expected(base, gains, model, template, s), &s.tau);
        worst = worst.max(d);
        sum += d;
    }
    (sum / window.len() as f64, worst)
}

/// Refuses a window on which a residual would not discriminate between gain sets: one where
/// either envelope contributed (its share is itself a function of the gains) or the torque sat
/// near the clamp (where every candidate fits).
pub fn assert_discriminating(window: &[Sample], torque_limits: &[f64; 7]) {
    assert!(
        window.len() > 100,
        "a window of {} cycles is too short to conclude anything",
        window.len()
    );
    let headroom = torque_limits.iter().copied().fold(f64::MAX, f64::min) * 0.9;
    for s in window {
        assert_eq!(
            max_abs(&s.tau_envelope),
            0.0,
            "the velocity envelope contributed at t={} ns; the fit is not a fit there",
            s.t_ns
        );
        assert_eq!(
            max_abs(&s.tau_position),
            0.0,
            "the position envelope contributed at t={} ns; the fit is not a fit there",
            s.t_ns
        );
        assert!(
            max_abs(&s.tau) < headroom,
            "|tau| = {:.3} Nm is within 10% of the torque clamp at t={} ns; every candidate \
             gain set fits a saturated torque",
            max_abs(&s.tau),
            s.t_ns
        );
    }
}

/// Per cycle, how much of the way from `from` to `to` the torque has travelled: the
/// least-squares projection of `tau - expected(from)` onto `expected(to) - expected(from)`.
/// The pairs are (seconds since `origin_ns`, fraction).
pub fn crossing(
    window: &[Sample],
    origin_ns: u64,
    base: &ImpedanceOptions,
    from: ImpedanceGains,
    to: ImpedanceGains,
    model: &Model,
    template: &RobotState,
) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(window.len());
    for s in window {
        let before = expected(base, from, model, template, s);
        let after = expected(base, to, model, template, s);
        let direction: [f64; 7] = std::array::from_fn(|i| after[i] - before[i]);
        let norm2: f64 = direction.iter().map(|v| v * v).sum();
        if norm2 < 1e-12 {
            continue;
        }
        let dot: f64 = (0..7).map(|i| (s.tau[i] - before[i]) * direction[i]).sum();
        out.push(((s.t_ns as f64 - origin_ns as f64) / 1e9, dot / norm2));
    }
    out
}

pub fn max_abs(a: &[f64; 7]) -> f64 {
    a.iter().map(|v| v.abs()).fold(0.0, f64::max)
}

pub fn max_abs_diff(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    (0..7).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}
