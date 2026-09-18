//! [`Otg::set_limits`] and [`MultiOtg::set_limits`]: that new limits reach the next plan
//! without moving the state, that lowering one under a moving generator clamps nothing here
//! but bites at the next re-anchor, and that a refused set leaves every axis as it was.

use super::*;

/// The generator's whole state, by bit pattern: position, velocity, acceleration, target.
fn state(otg: &Otg) -> [u64; 4] {
    [
        otg.position(),
        otg.velocity(),
        otg.acceleration(),
        otg.target(),
    ]
    .map(f64::to_bits)
}

/// A generator cruising at its velocity limit toward a target two metres away.
fn cruising(limits: OtgLimits) -> Otg {
    let mut otg = Otg::new(0.0, limits).unwrap();
    otg.set_target(2.0).unwrap();
    for _ in 0..10_000 {
        otg.step(DT);
        if otg.velocity() >= limits.max_velocity {
            return otg;
        }
    }
    panic!("the generator never reached its velocity limit");
}

#[test]
fn raising_a_limit_leaves_the_state_bit_for_bit_and_the_next_plan_takes_it() {
    let mut otg = cruising(LIMITS);
    let before = state(&otg);
    let raised = LIMITS.scaled(2.0);
    otg.set_limits(raised).unwrap();
    assert_eq!(state(&otg), before, "set_limits moved the generator");
    assert_eq!(otg.limits(), raised);

    // The next plan is drawn under the new limits: the generator accelerates past a cap the
    // old ones held it at, and every cycle of the rest of the move stays inside the new ones.
    let mut monitor = Monitor::new(raised);
    for _ in 0..500 {
        otg.step(DT);
        monitor.check(&otg, DT, "raised");
    }
    assert!(
        otg.velocity() > LIMITS.max_velocity + 1e-6,
        "still capped at {}",
        otg.velocity()
    );
    run_to_rest(&mut otg, &mut monitor, "raised");
    assert_eq!(otg.position(), 2.0);
}

#[test]
fn lowering_a_limit_clamps_nothing_until_the_next_re_anchor_does() {
    let mut otg = cruising(LIMITS);
    let before = state(&otg);
    let lowered = OtgLimits {
        max_velocity: 0.1,
        ..LIMITS
    };
    otg.set_limits(lowered).unwrap();
    // Not this call: the stored state is still that of a generator doing 0.3 m/s.
    assert_eq!(state(&otg), before, "set_limits clamped the state");
    assert_eq!(otg.velocity(), LIMITS.max_velocity);
    assert_eq!(otg.limits(), lowered);

    // The third OTG rule is what bites. Re-anchoring on the echo clamps the stored velocity
    // into the limits now in force, and the command's velocity drops the whole 0.2 m/s in one
    // cycle -- 200 m/s^2, four hundred times the generator's own acceleration limit. This is
    // the impulse a caller lowering a limit under a moving generator has to walk away from.
    otg.set_position(otg.position()).unwrap();
    assert_eq!(otg.velocity(), 0.1);
    let impulse = (LIMITS.max_velocity - 0.1) / DT;
    assert!(
        impulse > 100.0 * LIMITS.max_acceleration,
        "{impulse} m/s^2 against a limit of {}",
        LIMITS.max_acceleration
    );
}

#[test]
fn an_invalid_limit_is_refused_with_the_limits_and_the_state_untouched() {
    let mut otg = cruising(LIMITS);
    let before = state(&otg);
    for bad in REFUSED {
        assert!(
            matches!(otg.set_limits(bad), Err(FrankaError::InvalidArgument(_))),
            "{bad:?} was accepted"
        );
        assert_eq!(otg.limits(), LIMITS, "{bad:?} reached the limits");
        assert_eq!(state(&otg), before, "{bad:?} reached the state");
    }
    // And the generator still plans under the limits it was refused out of.
    let mut monitor = Monitor::new(LIMITS);
    run_to_rest(&mut otg, &mut monitor, "refused");
    assert_eq!(otg.position(), 2.0);
}

#[test]
fn a_lowered_jerk_lengthens_the_brake_and_breaks_no_continuity() {
    // Braking to a standstill where it is, from the cruise, under each jerk.
    let brake = |jerk: f64| {
        let limits = OtgLimits {
            max_jerk: jerk,
            ..LIMITS
        };
        let mut otg = cruising(LIMITS);
        let mut monitor = Monitor::new(LIMITS);
        monitor.check(&otg, DT, "cruise");
        let velocity = otg.velocity();
        otg.set_limits(limits).unwrap();
        // No discontinuity of its own: the state the brake starts from is bit for bit the
        // state the cruise ended on, and the monitor carries that cycle across the change.
        assert_eq!(otg.velocity().to_bits(), velocity.to_bits());
        monitor.limits = limits;
        otg.set_target(otg.position()).unwrap();
        run_to_rest(&mut otg, &mut monitor, "brake")
    };
    let (at_full, at_a_twentieth) = (brake(LIMITS.max_jerk), brake(LIMITS.max_jerk / 20.0));
    assert!(
        at_a_twentieth > at_full + 0.05,
        "{at_a_twentieth} s against {at_full} s"
    );
}

#[test]
fn a_multi_axis_set_is_per_axis_and_all_or_nothing() {
    let mut multi = CartesianOtg::new([0.0; 3], LIMITS, true).unwrap();
    multi.set_target([2.0, 2.0, 2.0]).unwrap();
    for _ in 0..500 {
        multi.step(DT);
    }
    let moving = multi.axes().map(|a| a.velocity());
    assert!(moving[0] > 0.0, "the axes never started");

    let per_axis = [LIMITS, LIMITS.scaled(2.0), LIMITS.scaled(0.5)];
    multi.set_limits(per_axis).unwrap();
    assert_eq!(multi.axes().map(|a| a.limits()), per_axis);
    assert_eq!(multi.axes().map(|a| a.velocity()), moving, "a state moved");

    // One bad axis leaves all three as they were, the two good ones included -- and the good
    // ones are all different from what is in force, so an axis written before the refusal
    // shows up rather than landing on the value it already had.
    for (attempt, bad) in REFUSED.into_iter().enumerate() {
        let axis = attempt % 3;
        let mut limits = per_axis.map(|l| l.scaled(3.0));
        limits[axis] = bad;
        assert!(
            matches!(
                multi.set_limits(limits),
                Err(FrankaError::InvalidArgument(_))
            ),
            "{bad:?} on axis {axis} was accepted"
        );
        assert_eq!(
            multi.axes().map(|a| a.limits()),
            per_axis,
            "{bad:?} on axis {axis} reached an axis"
        );
    }
}
