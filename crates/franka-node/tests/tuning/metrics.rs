//! What a window of cycles amounts to. Every limit is read from the library or from the served
//! schema; none is written down here.

use std::fmt;

use franka::robot::target_control::max_joint_velocity;
use franka::FciVersion;
use serde_json::Value;

use super::tap::Sample;

/// The connected arm's joint velocity limits, from the library, and asserted to be what the
/// node serves as `derived.dq_limit` — which is the whole point of serving it.
pub fn dq_limits(version: FciVersion, served: &Value) -> [f64; 7] {
    let limits = max_joint_velocity(version);
    let derived = served["derived"]["dq_limit"]
        .as_array()
        .expect("derived.dq_limit");
    for (i, limit) in limits.iter().enumerate() {
        assert_eq!(
            derived[i].as_f64().expect("dq_limit word"),
            *limit,
            "derived.dq_limit[{i}] is not the library's"
        );
    }
    limits
}

/// Everything a window says, each quantity against the limit it belongs to.
pub struct WindowStats {
    pub cycles: usize,
    pub peak_tau: f64,
    /// The worst `|tau_i| / torque_limits_i`.
    pub tau_fraction: f64,
    /// The worst `|dq_i| / dq_limit_i`.
    pub dq_fraction: f64,
    /// RMS of the second difference of the commanded torque, Nm: the chatter a gain feeds from
    /// measured velocity, with the tracked trend removed.
    pub chatter: f64,
    /// The generator's largest translational speed.
    pub peak_speed: f64,
    /// Cycles on which the joint velocity cap cut the goal.
    pub capped: usize,
    /// The largest `|q_goal - q|`, rad.
    pub joint_lead: f64,
    /// The largest `|commanded - measured|` EE position, m. Against franka-sim this carries a
    /// constant ~0.107 m frame offset (`O_T_EE` is the joint-7 frame), so only its *change*
    /// across a sweep means anything.
    pub cart_lead: f64,
    pub min_success_rate: f64,
    /// Whether either envelope contributed anywhere in the window.
    pub envelope_engaged: bool,
}

/// The generator's translational speed at one cycle, m/s.
pub fn speed(s: &Sample) -> f64 {
    (s.velocity[0].powi(2) + s.velocity[1].powi(2) + s.velocity[2].powi(2)).sqrt()
}

pub fn stats(window: &[Sample], torque_limits: &[f64; 7], dq_limit: &[f64; 7]) -> WindowStats {
    assert!(!window.is_empty(), "an empty window has no statistics");
    let worst = |f: &dyn Fn(&Sample) -> f64| window.iter().map(f).fold(0.0f64, f64::max);
    let mut chatter_sum = 0.0;
    for w in window.windows(3) {
        for i in 0..7 {
            let d = w[2].tau[i] - 2.0 * w[1].tau[i] + w[0].tau[i];
            chatter_sum += d * d;
        }
    }
    WindowStats {
        cycles: window.len(),
        peak_tau: worst(&|s| super::law::max_abs(&s.tau)),
        tau_fraction: worst(&|s| {
            (0..7)
                .map(|i| s.tau[i].abs() / torque_limits[i])
                .fold(0.0, f64::max)
        }),
        dq_fraction: worst(&|s| {
            (0..7)
                .map(|i| s.dq[i].abs() / dq_limit[i])
                .fold(0.0, f64::max)
        }),
        chatter: (chatter_sum / (window.len().max(3) - 2) as f64).sqrt(),
        peak_speed: worst(&speed),
        capped: window.iter().filter(|s| s.cap_scale < 0.999).count(),
        joint_lead: worst(&|s| {
            (0..7)
                .map(|i| (s.q_goal[i] - s.q[i]).abs())
                .fold(0.0, f64::max)
        }),
        cart_lead: worst(&|s| {
            ((s.cmd[0] - s.measured[0]).powi(2)
                + (s.cmd[1] - s.measured[1]).powi(2)
                + (s.cmd[2] - s.measured[2]).powi(2))
            .sqrt()
        }),
        min_success_rate: window
            .iter()
            .map(|s| s.success_rate)
            .fold(f64::INFINITY, f64::min),
        envelope_engaged: window.iter().any(|s| {
            super::law::max_abs(&s.tau_envelope) != 0.0
                || super::law::max_abs(&s.tau_position) != 0.0
        }),
    }
}

impl fmt::Display for WindowStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} cycles | peak|tau| {:.3} Nm ({:.1}% of limit) | peak|dq|/limit {:.1}% | \
             d2(tau) rms {:.5} Nm | peak speed {:.4} m/s | capped {} | lead {:.5} rad / \
             {:.4} m | min success {:.4}{}",
            self.cycles,
            self.peak_tau,
            self.tau_fraction * 100.0,
            self.dq_fraction * 100.0,
            self.chatter,
            self.peak_speed,
            self.capped,
            self.joint_lead,
            self.cart_lead,
            self.min_success_rate,
            if self.envelope_engaged {
                " | ENVELOPE ENGAGED"
            } else {
                ""
            }
        )
    }
}

/// Whether this host runs a `PREEMPT_RT` kernel. On one, a cycle that overran is the code's
/// fault and must fail a test; on an ordinary kernel it is the scheduler's, and asserting on it
/// makes a safety test flaky — which is worse than not having it, because people learn to
/// re-run a flaky test.
pub fn realtime_kernel() -> bool {
    std::fs::read_to_string("/sys/kernel/realtime")
        .map(|s| s.trim() == "1")
        .unwrap_or(false)
        || std::fs::read_to_string("/proc/version")
            .map(|s| s.contains("PREEMPT_RT"))
            .unwrap_or(false)
}

/// Per-cycle interval statistics, ms. `p99` over fewer than a few thousand samples is the
/// n-th worst point and not a percentile, so `worst` is what a short window can claim.
pub struct Jitter {
    pub cycles: usize,
    pub p50: f64,
    pub p99: f64,
    pub worst: f64,
    pub over_two_ms: usize,
}

pub fn jitter(window: &[Sample]) -> Jitter {
    assert!(window.len() > 1, "jitter needs at least two cycles");
    let mut deltas: Vec<f64> = window
        .windows(2)
        .map(|w| (w[1].t_ns - w[0].t_ns) as f64 / 1e6)
        .collect();
    deltas.sort_by(f64::total_cmp);
    let at = |q: f64| deltas[((deltas.len() - 1) as f64 * q) as usize];
    Jitter {
        cycles: window.len(),
        p50: at(0.5),
        p99: at(0.99),
        worst: deltas[deltas.len() - 1],
        over_two_ms: deltas.iter().filter(|d| **d > 2.0).count(),
    }
}

impl fmt::Display for Jitter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} cycles, dt ms p50 {:.3} p99 {:.3} worst {:.3}, over 2 ms: {}",
            self.cycles, self.p50, self.p99, self.worst, self.over_two_ms
        )
    }
}
