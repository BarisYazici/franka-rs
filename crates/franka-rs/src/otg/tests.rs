use super::*;

const LIMITS: OtgLimits = OtgLimits {
    max_velocity: 0.3,
    max_acceleration: 0.5,
    max_jerk: 20.0,
};
const DT: f64 = 1e-3;

/// Checks the limits, the continuity of the acceleration and the consistency of the
/// derivatives across consecutive steps.
struct Monitor {
    limits: OtgLimits,
    last: Option<(f64, f64, f64)>,
    peak_speed: f64,
    /// Whether the position is the generator's own integral (false once it is re-anchored).
    consistent: bool,
}

impl Monitor {
    fn new(limits: OtgLimits) -> Self {
        Self {
            limits,
            last: None,
            peak_speed: 0.0,
            consistent: true,
        }
    }

    fn check(&mut self, otg: &Otg, dt: f64, what: &str) {
        let OtgLimits {
            max_velocity,
            max_acceleration,
            max_jerk,
        } = self.limits;
        let (p, v, a) = (otg.position(), otg.velocity(), otg.acceleration());
        assert!(
            p.is_finite() && v.is_finite() && a.is_finite(),
            "{what}: non-finite state"
        );
        assert!(
            v.abs() <= max_velocity + 1e-9,
            "{what}: |v| = {v} > {max_velocity}"
        );
        assert!(
            a.abs() <= max_acceleration + 1e-9,
            "{what}: |a| = {a} > {max_acceleration}"
        );
        if let Some((lp, lv, la)) = self.last {
            let jerk = (a - la) / dt;
            assert!(
                jerk.abs() <= max_jerk + 1e-6,
                "{what}: jerk {jerk} > {max_jerk}"
            );
            let mean_v = (p - lp) / dt;
            if self.consistent {
                assert!(
                    mean_v.abs() <= max_velocity + 1e-9,
                    "{what}: mean speed {mean_v}"
                );
                // The mean velocity over the cycle must agree with the endpoint velocities.
                let expected = 0.5 * (lv + v);
                assert!(
                    (mean_v - expected).abs() <= max_jerk * dt * dt + 1e-9,
                    "{what}: mean speed {mean_v} vs velocities {lv}, {v}"
                );
            }
            self.peak_speed = self.peak_speed.max(mean_v.abs());
        }
        self.last = Some((p, v, a));
    }
}

/// The closed-form duration of a rest-to-rest S-curve over distance `d`.
fn rest_to_rest_duration(d: f64, limits: OtgLimits) -> f64 {
    let OtgLimits {
        max_velocity: vm,
        max_acceleration: am,
        max_jerk: jm,
    } = limits;
    let d = d.abs();
    // Time and distance to accelerate from rest to `v` (triangular or trapezoidal profile).
    let accelerate = |v: f64| {
        let t = if v <= am * am / jm {
            2.0 * (v / jm).sqrt()
        } else {
            v / am + am / jm
        };
        (t, v * t / 2.0)
    };
    let (t_max, d_max) = accelerate(vm);
    if d >= 2.0 * d_max {
        return 2.0 * t_max + (d - 2.0 * d_max) / vm;
    }
    let peak = if d / 2.0 <= (am * am / jm) * am / jm {
        (d / 2.0).powf(2.0 / 3.0) * jm.powf(1.0 / 3.0)
    } else {
        // d = v (v / am + am / jm)  <=>  v^2 / am + (am / jm) v - d = 0
        let (b, c) = (am / jm, -d);
        (-b + (b * b - 4.0 * c / am).sqrt()) / (2.0 / am)
    };
    2.0 * accelerate(peak).0
}

/// Where the axis would come to rest if it braked now, and how long that takes.
fn stop(otg: &Otg) -> (f64, f64) {
    let l = otg.limits;
    let mut state = (otg.position(), otg.velocity(), otg.acceleration());
    let segments = transfer(state.1, state.2, 0.0, l.max_acceleration, l.max_jerk);
    for s in &segments {
        integrate(&mut state, s.jerk, s.duration);
    }
    (state.0, segments.iter().map(|s| s.duration).sum())
}

/// Runs `otg` until it is exactly at rest at its target; returns the time that took.
fn run_to_rest(otg: &mut Otg, monitor: &mut Monitor, what: &str) -> f64 {
    let mut t = 0.0;
    while !(otg.position() == otg.target() && otg.velocity() == 0.0 && otg.acceleration() == 0.0) {
        otg.step(DT);
        t += DT;
        monitor.check(otg, DT, what);
        assert!(t < 60.0, "{what}: never came to rest");
    }
    t
}

#[test]
fn rejects_bad_limits_positions_and_targets() {
    let bad = |limits: OtgLimits| {
        assert!(matches!(
            Otg::new(0.0, limits),
            Err(FrankaError::InvalidArgument(_))
        ));
        assert!(matches!(
            CartesianOtg::new([0.0; 3], limits, true),
            Err(FrankaError::InvalidArgument(_))
        ));
    };
    for (v, a, j) in [
        (f64::NAN, 0.5, 20.0),
        (0.3, -0.5, 20.0),
        (0.3, 0.5, 0.0),
        (0.3, f64::INFINITY, 20.0),
        (-0.3, 0.5, 20.0),
    ] {
        bad(OtgLimits {
            max_velocity: v,
            max_acceleration: a,
            max_jerk: j,
        });
    }
    assert!(Otg::new(f64::NAN, LIMITS).is_err());
    let mut otg = Otg::new(1.0, LIMITS).unwrap();
    assert!(otg.set_target(f64::NAN).is_err());
    assert!(otg.set_target(f64::NEG_INFINITY).is_err());
    assert_eq!(otg.target(), 1.0);
    let mut multi = CartesianOtg::new([1.0; 3], LIMITS, false).unwrap();
    assert!(multi.set_target([0.0, f64::NAN, 0.0]).is_err());
    assert_eq!(multi.axes().map(|a| a.target()), [1.0; 3]);
}

#[test]
fn five_centimetre_step_from_rest_arrives_in_the_s_curve_time() {
    let mut otg = Otg::new(0.0, LIMITS).unwrap();
    otg.set_target(0.05).unwrap();
    let expected = rest_to_rest_duration(0.05, LIMITS);
    assert!((expected - 0.658).abs() < 1e-3, "{expected}");
    assert!(
        (otg.duration() - expected).abs() < 0.01 * expected,
        "{}",
        otg.duration()
    );
    let mut monitor = Monitor::new(LIMITS);
    let t = run_to_rest(&mut otg, &mut monitor, "5 cm");
    assert!(
        (t - expected).abs() < 0.05 * expected,
        "arrived after {t} s, expected {expected}"
    );
    assert!(
        (monitor.peak_speed - 0.152).abs() < 0.005,
        "{}",
        monitor.peak_speed
    );
    for _ in 0..1000 {
        assert_eq!(otg.step(DT), 0.05);
        assert_eq!((otg.velocity(), otg.acceleration()), (0.0, 0.0));
    }
}

#[test]
fn long_move_cruises_at_max_velocity_and_a_step_with_dt_zero_is_a_no_op() {
    let mut otg = Otg::new(0.0, LIMITS).unwrap();
    otg.set_target(1.0).unwrap();
    let mut monitor = Monitor::new(LIMITS);
    let t = run_to_rest(&mut otg, &mut monitor, "1 m");
    let expected = rest_to_rest_duration(1.0, LIMITS);
    assert!((t - expected).abs() < 0.01 * expected, "{t} vs {expected}");
    assert!(monitor.peak_speed > 0.3 - 1e-6);
    otg.set_target(2.0).unwrap();
    assert_eq!(otg.step(0.0), 1.0);
    assert_eq!(otg.step(f64::NAN), 1.0);
    assert_eq!(otg.step(-1.0), 1.0);
}

#[test]
fn new_targets_mid_motion_in_both_directions_and_closer_than_the_braking_distance() {
    for (second, what) in [
        (0.20, "same direction"),
        (-0.10, "opposite direction"),
        (0.0, "back to the start"),
    ] {
        let mut otg = Otg::new(0.0, LIMITS).unwrap();
        let mut monitor = Monitor::new(LIMITS);
        otg.set_target(0.10).unwrap();
        for _ in 0..300 {
            otg.step(DT);
            monitor.check(&otg, DT, what);
        }
        assert!(otg.velocity() > 0.1, "{}", otg.velocity());
        otg.set_target(second).unwrap();
        let t = run_to_rest(&mut otg, &mut monitor, what);
        assert!(t < 3.0, "{what}: {t} s");
    }

    // At full speed, a target just ahead: brake past it and come back, no jerk spike.
    let mut otg = Otg::new(0.0, LIMITS).unwrap();
    let mut monitor = Monitor::new(LIMITS);
    otg.set_target(1.0).unwrap();
    while otg.velocity() < LIMITS.max_velocity {
        otg.step(DT);
        monitor.check(&otg, DT, "run-up");
    }
    let (stop_position, _) = stop(&otg);
    let target = otg.position() + 0.001;
    assert!(target < stop_position);
    otg.set_target(target).unwrap();
    let mut furthest = otg.position();
    let mut t = 0.0;
    while otg.position() != target || otg.velocity() != 0.0 {
        otg.step(DT);
        t += DT;
        monitor.check(&otg, DT, "braking");
        furthest = furthest.max(otg.position());
        assert!(t < 5.0);
    }
    assert!(
        furthest > target + 0.01,
        "passed the target only by {}",
        furthest - target
    );
    assert!(
        furthest < stop_position + 1e-6,
        "{furthest} vs {stop_position}"
    );
}

#[test]
fn a_two_millisecond_cycle_after_a_lost_packet_keeps_the_limits() {
    let mut otg = Otg::new(0.0, LIMITS).unwrap();
    let mut monitor = Monitor::new(LIMITS);
    otg.set_target(0.05).unwrap();
    let mut t = 0.0;
    let mut i = 0;
    while otg.position() != 0.05 {
        let dt = if i % 7 == 3 { 2.0 * DT } else { DT };
        otg.step(dt);
        t += dt;
        monitor.check(&otg, dt, "varying dt");
        i += 1;
    }
    let expected = rest_to_rest_duration(0.05, LIMITS);
    assert!(
        (t - expected).abs() < 0.05 * expected + 2.0 * DT,
        "{t} vs {expected}"
    );
}

#[test]
fn synchronised_axes_arrive_together() {
    for synchronize in [false, true] {
        let mut otg = CartesianOtg::new([0.0; 3], LIMITS, synchronize).unwrap();
        otg.set_target([0.05, -0.01, 0.002]).unwrap();
        let mut monitors = [
            Monitor::new(LIMITS),
            Monitor::new(LIMITS),
            Monitor::new(LIMITS),
        ];
        let mut arrival = [None; 3];
        let mut t = 0.0;
        while arrival.iter().any(Option::is_none) {
            let position = otg.step(DT);
            t += DT;
            for i in 0..3 {
                monitors[i].check(&otg.axes()[i], DT, "sync");
                if arrival[i].is_none() && position[i] == otg.axes()[i].target() {
                    arrival[i] = Some(t);
                }
            }
            assert!(t < 5.0, "{arrival:?}");
        }
        let arrival = arrival.map(Option::unwrap);
        let expected = rest_to_rest_duration(0.05, LIMITS);
        assert!(
            (arrival[0] - expected).abs() < 0.05 * expected,
            "{arrival:?}"
        );
        if synchronize {
            assert!(
                arrival.iter().all(|a| (a - arrival[0]).abs() <= DT + 1e-9),
                "{arrival:?}"
            );
            assert_eq!(otg.position(), [0.05, -0.01, 0.002]);
        } else {
            assert!(
                arrival[2] < arrival[1] && arrival[1] < 0.6 * arrival[0],
                "{arrival:?}"
            );
        }
    }
}

/// The target sequence of the commander example's first run with this generator on a real FER
/// (2026-09-08), relative to the start position in metres at the logged times, and the cycles
/// of that run that were longer than 1 ms (cycle index, seconds). The generator itself stayed
/// on its targets; the rate limiter behind it did not (see the module documentation).
#[rustfmt::skip]
const FLIGHT_EVENTS: &[(f64, [f64; 3])] = &[
    (0.498, [0.05, 0.0, 0.0]), (1.298, [0.05, 0.05, 0.0]), (1.599, [0.05, 0.05, 0.05]),
    (2.799, [0.0, 0.05, 0.05]), (3.299, [-0.05, 0.05, 0.05]), (4.799, [-0.05, 0.0, 0.05]),
    (4.999, [-0.05, -0.05, 0.05]), (5.899, [-0.05, -0.05, 0.0]), (6.299, [-0.05, -0.05, -0.05]),
    (7.399, [0.0, -0.05, -0.05]), (7.999, [0.0, -0.05, 0.0]), (11.299, [0.05, -0.05, 0.0]),
    (11.999, [0.05, 0.0, 0.0]), (12.249, [0.05, 0.05, 0.0]), (13.649, [0.05, 0.05, 0.05]),
    (13.999, [0.1, 0.05, 0.05]), (14.004, [0.05, 0.05, 0.05]), (14.009, [0.1, 0.05, 0.05]),
    (14.014, [0.05, 0.05, 0.05]), (14.019, [0.1, 0.05, 0.05]), (14.024, [0.05, 0.05, 0.05]),
    (14.029, [0.1, 0.05, 0.05]), (14.034, [0.05, 0.05, 0.05]), (14.039, [0.1, 0.05, 0.05]),
    (14.044, [0.05, 0.05, 0.05]), (14.049, [0.1, 0.05, 0.05]), (14.054, [0.05, 0.05, 0.05]),
    (14.059, [0.1, 0.05, 0.05]), (14.064, [0.05, 0.05, 0.05]), (14.069, [0.1, 0.05, 0.05]),
    (14.074, [0.05, 0.05, 0.05]), (14.079, [0.1, 0.05, 0.05]), (14.084, [0.05, 0.05, 0.05]),
    (14.089, [0.1, 0.05, 0.05]), (14.094, [0.05, 0.05, 0.05]), (15.099, [0.0, 0.05, 0.05]),
    (15.999, [0.05, 0.05, 0.05]), (16.549, [0.0, 0.05, 0.05]), (17.149, [0.0, 0.0, 0.05]),
    (17.599, [0.0, 0.0, 0.0]),
];
#[rustfmt::skip]
const FLIGHT_LONG_CYCLES: &[(usize, f64)] = &[
    (168, 0.002), (364, 0.003), (781, 0.002), (1165, 0.002), (1465, 0.002), (1674, 0.002),
    (1867, 0.002), (2026, 0.002), (2584, 0.003), (2873, 0.002), (3388, 0.002), (3692, 0.002),
    (5311, 0.002), (6005, 0.002), (6529, 0.002), (6727, 0.002), (6815, 0.003), (6918, 0.002),
    (8532, 0.003), (9143, 0.002), (9435, 0.002), (10241, 0.002), (10347, 0.003), (10646, 0.002),
    (10841, 0.002), (11248, 0.002), (11853, 0.002), (12157, 0.002), (12265, 0.002), (12671, 0.003),
    (13268, 0.002), (13671, 0.002), (15188, 0.002), (15390, 0.002), (15566, 0.002), (15581, 0.002),
    (15882, 0.002), (17601, 0.004), (17911, 0.003), (18102, 0.002), (18304, 0.002), (18607, 0.003),
    (19111, 0.003), (19308, 0.002), (19414, 0.002), (19512, 0.003), (19911, 0.002), (20011, 0.002),
    (20213, 0.002), (20413, 0.002), (20816, 0.002), (21324, 0.004),
];
const FLIGHT_START: [f64; 3] = [0.307012006, -4.387e-5, 0.486808997];
const FLIGHT_CYCLES: usize = 21705;

/// The example's bridge behind the generator: `limit_rate_cartesian_pose` under `budget`
/// against an echo of its own last command, as the robot would return it.
struct Backstop {
    budget: [f64; 6],
    pose: [f64; 16],
    twist: [f64; 6],
    acceleration: [f64; 6],
    worst_bind: f64,
}

impl Backstop {
    fn new(budget: [f64; 6], start: [f64; 3]) -> Self {
        let mut pose = [0.0; 16];
        (pose[0], pose[5], pose[10], pose[15]) = (1.0, -1.0, -1.0, 1.0);
        pose[12..15].copy_from_slice(&start);
        Self {
            budget,
            pose,
            twist: [0.0; 6],
            acceleration: [0.0; 6],
            worst_bind: 0.0,
        }
    }

    fn echo(&self) -> [f64; 3] {
        [self.pose[12], self.pose[13], self.pose[14]]
    }

    /// Sends `wanted`; returns what the robot would have received.
    fn send(&mut self, wanted: [f64; 3]) -> [f64; 3] {
        use crate::rate_limiting::{limit_rate_cartesian_pose, DELTA_T};
        let [v, a, j, rv, ra, rj] = self.budget;
        let mut pose = self.pose;
        pose[12..15].copy_from_slice(&wanted);
        let limited = limit_rate_cartesian_pose(
            v,
            a,
            j,
            rv,
            ra,
            rj,
            &pose,
            &self.pose,
            &self.twist,
            &self.acceleration,
        )
        .unwrap();
        let sent = [limited[12], limited[13], limited[14]];
        let bind = (0..3)
            .map(|k| (sent[k] - wanted[k]).abs())
            .fold(0.0, f64::max);
        self.worst_bind = self.worst_bind.max(bind);
        let twist: [f64; 6] = std::array::from_fn(|k| match k < 3 {
            true => (sent[k] - self.pose[12 + k]) / DELTA_T,
            false => 0.0,
        });
        self.acceleration = std::array::from_fn(|k| (twist[k] - self.twist[k]) / DELTA_T);
        self.twist = twist;
        self.pose = limited;
        sent
    }
}

/// Replays the flight through a synchronised or free `CartesianOtg`, alone (stepping the
/// logged cycle lengths) or behind a [`Backstop`] with re-anchoring on its echo (stepping one
/// nominal cycle per command, as the example does). Asserts the per-axis limits every cycle,
/// that an axis at rest whose target did not change stays bit-exact, that a stoppable target
/// is never passed, and that every target is held within 1e-6 once the profile has had 10 %
/// more than its planned duration. Returns the worst amount the backstop altered a command.
fn replay_flight(limits: OtgLimits, synchronize: bool, backstop: Option<[f64; 6]>) -> f64 {
    let mut otg = CartesianOtg::new(FLIGHT_START, limits, synchronize).unwrap();
    let mut backstop = backstop.map(|budget| Backstop::new(budget, FLIGHT_START));
    let mut monitors = [
        Monitor::new(limits),
        Monitor::new(limits),
        Monitor::new(limits),
    ];
    monitors
        .iter_mut()
        .for_each(|m| m.consistent = backstop.is_none());
    let mut promises = otg.axes().map(|axis| make_promise(&axis, 0.0));
    let mut pinned: [Option<f64>; 3] = [None; 3];
    let mut target = FLIGHT_START;
    let mut command = FLIGHT_START;
    let (mut t, mut next_event) = (0.0, 0);
    for cycle in 1..=FLIGHT_CYCLES {
        let what = format!(
            "flight cycle {cycle} (sync {synchronize}, backstop {})",
            backstop.is_some()
        );
        if let Some(&(when, offset)) = FLIGHT_EVENTS.get(next_event) {
            if when <= t + 1e-9 {
                let new_target = std::array::from_fn(|k| FLIGHT_START[k] + offset[k]);
                otg.set_target(new_target).unwrap();
                for (k, axis) in otg.axes().iter().enumerate() {
                    let at_rest = axis.velocity() == 0.0
                        && axis.acceleration() == 0.0
                        && axis.position() == target[k];
                    pinned[k] = (new_target[k] == target[k] && at_rest).then_some(target[k]);
                }
                target = new_target;
                promises = otg.axes().map(|axis| make_promise(&axis, t));
                let deadline = promises.iter().map(|p| p.deadline).fold(0.0, f64::max);
                promises.iter_mut().for_each(|p| p.deadline = deadline);
                next_event += 1;
            }
        }
        let long = FLIGHT_LONG_CYCLES.iter().find(|(i, _)| *i == cycle);
        let dt = long.map_or(DT, |&(_, dt)| dt);
        let out = match backstop.as_mut() {
            Some(backstop) => {
                otg.set_position(backstop.echo()).unwrap();
                let wanted = otg.step(DT);
                backstop.send(wanted)
            }
            None => otg.step(dt),
        };
        t += dt;
        if let Some(b) = backstop.as_ref() {
            // What the robot sees: the command's speed within the budget's norm.
            let speed = (0..3)
                .map(|k| (out[k] - command[k]).powi(2))
                .sum::<f64>()
                .sqrt()
                / DT;
            assert!(
                speed <= b.budget[0] + 1e-9,
                "{what}: commanded speed {speed}"
            );
        }
        for k in 0..3 {
            let stepped = if backstop.is_some() { DT } else { dt };
            monitors[k].check(&otg.axes()[k], stepped, &what);
            if let Some(p) = pinned[k] {
                assert_eq!(
                    out[k], p,
                    "{what}: axis {k} at rest moved away from its target"
                );
            }
            let error = out[k] - promises[k].target;
            if promises[k].stoppable && backstop.is_none() {
                assert!(
                    error * promises[k].direction <= 2e-9,
                    "{what}: axis {k} overshoot {error}"
                );
            }
            if t >= promises[k].deadline {
                // A binding backstop shapes a slower path than the plan, and its vector-norm
                // clamp distorts one axis's corrections while another saturates, so the loop
                // does not land exactly; what re-anchoring guarantees is that the lag stays
                // bounded -- millimetres, where the open-loop bridge orbited by a decimetre.
                let tolerance = match backstop.as_ref().map(|b| b.worst_bind > 1e-9) {
                    Some(true) => 2e-2,
                    _ => 1e-6,
                };
                assert!(
                    error.abs() <= tolerance,
                    "{what}: axis {k} is {error} from its target at {t:.3} s"
                );
            }
        }
        command = out;
    }
    match backstop.as_ref() {
        None => assert_eq!(command, target, "did not finish exactly on the last target"),
        Some(b) => {
            let error = (0..3)
                .map(|k| (command[k] - target[k]).abs())
                .fold(0.0, f64::max);
            let tolerance = if b.worst_bind > 1e-9 { 5e-3 } else { 1e-9 };
            assert!(
                error <= tolerance,
                "finished {error} m from the last target"
            );
            return b.worst_bind;
        }
    }
    0.0
}

#[test]
fn flight_replay_stays_on_every_target() {
    replay_flight(LIMITS, true, None);
    replay_flight(LIMITS, false, None);
}

#[test]
fn flight_replay_behind_the_backstop_never_binds_with_norm_limits_and_survives_binding() {
    let budget = [0.3, 0.5, 20.0, 1.0, 5.0, 500.0];
    let bind = replay_flight(LIMITS.per_axis_for_norm(3), true, Some(budget));
    assert!(bind <= 1e-9, "the backstop altered a command by {bind} m");
    // With the budget as per-axis limits the backstop does bind on two-axis motion; with the
    // generator re-anchored on the echo every cycle that must stay bounded (no orbit).
    let bind = replay_flight(LIMITS, true, Some(budget));
    assert!(
        bind > 1e-6,
        "expected the backstop to bind, it altered commands by at most {bind}"
    );
}

/// xorshift64*: deterministic, no dependencies.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// What the generator promised when the target last changed.
struct Promise {
    target: f64,
    direction: f64,
    /// The target lies beyond where braking now would end, so it must not be passed.
    stoppable: bool,
    /// Braking to rest and then moving rest-to-rest is one admissible profile, so the
    /// optimum is no slower; the axis must be there by then.
    deadline: f64,
}

fn make_promise(otg: &Otg, t: f64) -> Promise {
    let (stop_position, stop_duration) = stop(otg);
    let e = otg.target() - otg.position();
    let bound = stop_duration + rest_to_rest_duration(otg.target() - stop_position, otg.limits);
    assert!(
        otg.duration() <= bound + 1e-5,
        "{} > {bound} from p={} v={} a={} target={} (stop at {stop_position} in {stop_duration})",
        otg.duration(),
        otg.position(),
        otg.velocity(),
        otg.acceleration(),
        otg.target()
    );
    Promise {
        target: otg.target(),
        direction: e.signum(),
        stoppable: e != 0.0 && (otg.target() - stop_position) * e.signum() >= 0.0,
        deadline: t + 1.1 * bound + 2.0 * DT,
    }
}

#[test]
fn random_target_sequences_respect_every_promise() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for sequence in 0..2000 {
        let limits = if sequence % 3 == 0 {
            OtgLimits {
                max_velocity: rng.uniform(0.05, 2.0),
                max_acceleration: rng.uniform(0.1, 10.0),
                max_jerk: rng.uniform(1.0, 500.0),
            }
        } else {
            LIMITS
        };
        let what = format!("sequence {sequence} with {limits:?}");
        let mut otg = Otg::new(rng.uniform(-0.1, 0.1), limits).unwrap();
        let mut monitor = Monitor::new(limits);
        let mut t = 0.0;
        let mut promise = make_promise(&otg, t);
        // A schedule of (time, target) changes: steps with holds, bursts, gaps.
        let mut changes = [(f64::INFINITY, 0.0); 64];
        let mut count = 0;
        let mut when = rng.uniform(0.0, 0.5);
        while count < changes.len() && when < 2.5 {
            match rng.next() % 10 {
                0 | 1 => {
                    for _ in 0..20.min(changes.len() - count) {
                        changes[count] = (when, rng.uniform(-0.3, 0.3));
                        count += 1;
                        when += 5.0 * DT;
                    }
                    when += rng.uniform(0.2, 3.0);
                }
                2 => when += rng.uniform(0.5, 3.0),
                _ => {
                    changes[count] = (when, rng.uniform(-0.3, 0.3));
                    count += 1;
                    when += rng.uniform(0.01, 3.0);
                }
            }
        }
        let mut next = 0;
        while t < when + 0.5 || t < promise.deadline + 0.05 {
            while next < count && changes[next].0 <= t {
                otg.set_target(changes[next].1).unwrap();
                next += 1;
                promise = make_promise(&otg, t);
            }
            otg.step(DT);
            t += DT;
            monitor.check(&otg, DT, &what);
            let error = otg.position() - promise.target;
            if promise.stoppable {
                assert!(
                    error * promise.direction <= 2e-9,
                    "{what}: overshoot {error} at {t}"
                );
            }
            if t >= promise.deadline {
                assert!(
                    error.abs() <= 1e-4,
                    "{what}: {error} from the target at {t}"
                );
            }
        }
        if sequence % 8 == 1 {
            synchronised_axes_keep_every_promise(&mut rng, limits, &changes[..count], &what);
        }
    }
}

/// The same schedule on a synchronised `MultiOtg<3>`, each change landing on one random axis
/// while the other two hold (so changes arrive while an axis is braking or synchronised to a
/// slower one): per-axis limits every cycle, an axis at rest whose target did not change stays
/// bit-exact, no stoppable target is passed, and every axis is on target within 10 % of the
/// slowest axis's brake-then-move bound.
fn synchronised_axes_keep_every_promise(
    rng: &mut Rng,
    limits: OtgLimits,
    changes: &[(f64, f64)],
    what: &str,
) {
    let what = format!("{what}, synchronised");
    let start = [
        rng.uniform(-0.1, 0.1),
        rng.uniform(-0.1, 0.1),
        rng.uniform(-0.1, 0.1),
    ];
    let mut otg = CartesianOtg::new(start, limits, true).unwrap();
    let mut monitors = [
        Monitor::new(limits),
        Monitor::new(limits),
        Monitor::new(limits),
    ];
    let mut promises = otg.axes().map(|axis| make_promise(&axis, 0.0));
    let mut pinned: [Option<f64>; 3] = [None; 3];
    let mut target = start;
    let (mut t, mut next) = (0.0, 0);
    let last_change = changes.last().map_or(0.0, |c| c.0);
    while t < last_change + 0.5 || promises.iter().any(|p| t < p.deadline + 0.05) {
        while next < changes.len() && changes[next].0 <= t {
            let axis = (rng.next() % 3) as usize;
            for (k, axis_otg) in otg.axes().iter().enumerate() {
                let at_rest = axis_otg.velocity() == 0.0
                    && axis_otg.acceleration() == 0.0
                    && axis_otg.position() == target[k];
                pinned[k] = (k != axis && at_rest).then_some(target[k]);
            }
            target[axis] = changes[next].1;
            otg.set_target(target).unwrap();
            next += 1;
            promises = otg.axes().map(|a| make_promise(&a, t));
            let deadline = promises.iter().map(|p| p.deadline).fold(0.0, f64::max);
            promises.iter_mut().for_each(|p| p.deadline = deadline);
        }
        let out = otg.step(DT);
        t += DT;
        for k in 0..3 {
            monitors[k].check(&otg.axes()[k], DT, &what);
            if let Some(p) = pinned[k] {
                assert_eq!(
                    out[k], p,
                    "{what}: axis {k} at rest moved off its target at {t}"
                );
            }
            let error = out[k] - promises[k].target;
            if promises[k].stoppable {
                assert!(
                    error * promises[k].direction <= 2e-9,
                    "{what}: axis {k} overshoot {error} at {t}"
                );
            }
            if t >= promises[k].deadline {
                assert!(
                    error.abs() <= 1e-4,
                    "{what}: axis {k} is {error} from its target at {t}"
                );
            }
        }
        assert!(t < 30.0, "{what}: never settled");
    }
}
