//! Position first where pins make a pose unreachable: the position is reached, the orientation
//! gets as close as the box allows and never recedes, and the blend into it leaves no step.

use nalgebra::{DMatrix, DVector, SMatrix};

use super::*;

/// Two joints at their upper margin at the ready pose.
const PINS: [(&str, [usize; 2]); 2] = [("j5+j7", [4, 6]), ("j4+j7", [3, 6])];

/// The arm's limits with `pins`' upper ones `margin` above the ready pose.
fn pinned_at(arm: &Arm, pins: [usize; 2], margin: f64) -> ([f64; 7], [f64; 7]) {
    let mut limits = arm.limits;
    pins.iter().for_each(|&i| limits.1[i] = READY[i] + margin);
    limits
}

/// Cycles the target takes to its end, and holds it.
const RAMP: usize = 1000;
const HOLD: usize = 2000;

/// `k` of `cycles` along a smooth start and stop, 0 to 1.
fn smooth(k: usize, cycles: usize) -> f64 {
    0.5 - 0.5 * (std::f64::consts::PI * k.min(cycles) as f64 / cycles as f64).cos()
}

/// The ready pose moved `s` of 2 cm along x and y and of 0.4 rad about the tool's z.
fn beyond(arm: &Arm, tool: &[f64; 16], s: f64) -> [f64; 16] {
    let start = arm.fk(&READY, tool);
    let p = translation_of(&start);
    pose_from(
        &(rotation_of(&start) * exp(&[0.0, 0.0, 0.4 * s])),
        &[p[0] + 0.02 * s, p[1] + 0.02 * s, p[2]],
    )
}

/// How much faster than not at all the orientation error could fall with the translation held,
/// relative to its gradient: over the free joints in the translation's nullspace, and for each
/// pinned joint moved off its bound with the free joints making up its translation. 0 where no
/// such move turns the tool closer.
fn orientation_descent(
    arm: &Arm,
    q: &[f64; 7],
    pose: &[f64; 16],
    tool: &[f64; 16],
    pinned: &[i8; 7],
) -> f64 {
    let j = SMatrix::<f64, 6, 7>::from_column_slice(&arm.model.zero_jacobian_q(
        Frame::EndEffector,
        q,
        tool,
        &IDENTITY,
    ));
    let e = log(&(rotation_of(pose) * rotation_of(&arm.fk(q, tool)).transpose()));
    let e = DVector::from_row_slice(&e);
    let free: Vec<usize> = (0..7).filter(|&i| pinned[i] == 0).collect();
    let column =
        |rows: std::ops::Range<usize>, i: usize| DVector::from_iterator(3, rows.map(|r| j[(r, i)]));
    let block = |rows: std::ops::Range<usize>| {
        DMatrix::from_columns(
            &free
                .iter()
                .map(|&i| column(rows.clone(), i))
                .collect::<Vec<_>>(),
        )
    };
    let (jp, jr) = (block(0..3), block(3..6));
    let jp_pinv = jp.clone().pseudo_inverse(1e-12).expect("a pseudo-inverse");
    let nullspace = DMatrix::identity(free.len(), free.len()) - &jp_pinv * &jp;
    let gradient = jr.transpose() * &e;
    let scale = gradient.norm().max(1e-12);
    let mut descent = (&nullspace * &gradient).norm() / scale;
    for i in (0..7).filter(|&i| pinned[i] != 0) {
        // Off the bound by one unit, the free joints keeping the translation.
        let off = -f64::from(pinned[i].signum());
        let make_up = -(&jp_pinv * column(0..3, i)) * off;
        let turn = column(3..6, i) * off + &jr * make_up;
        descent = descent.max(e.dot(&turn) / scale);
    }
    descent
}

#[test]
fn position_comes_first_and_the_orientation_gets_as_close_as_the_pins_allow() {
    for arm in arms() {
        for (tool_name, tool) in TOOLS {
            for (pins_name, pins) in PINS {
                let case = format!("{} {tool_name} {pins_name}", arm.name);
                let limits = pinned_at(&arm, pins, MARGIN);
                let pose = beyond(&arm, &tool, 1.0);
                let run = |ramp: u32| {
                    let mut ik = arm.ik(IkOptions::default(), arm.real(limits), READY, tool);
                    ik.set_priority_ramp(ramp);
                    let errors: Vec<(f64, f64)> = (0..RAMP + HOLD)
                        .map(|k| {
                            let to = beyond(&arm, &tool, smooth(k, RAMP));
                            pose_distance(&arm.fk(&ik.step(&to, &READY, DELTA_T).0, &tool), &pose)
                        })
                        .collect();
                    (ik, errors)
                };
                let (ik, errors) = run(PRIORITY_RAMP_CYCLES);
                // The baseline: a blend held at 0, which is the weighted solve itself.
                let (weighted, weighted_errors) = run(0);
                assert_eq!(weighted.priority(), 0.0, "{case}: not the weighted solve");
                let (dp, da) = errors[errors.len() - 1];
                let (wp, wa) = weighted_errors[weighted_errors.len() - 1];
                let q = ik.q();
                let descent = orientation_descent(&arm, &q, &pose, &tool, &ik.pinned());
                let receded = errors[RAMP..]
                    .windows(2)
                    .map(|w| w[1].1 - w[0].1)
                    .fold(f64::NEG_INFINITY, f64::max);
                println!(
                    "{case}: position first {:.4} mm, {da:.4} rad (descent {descent:.1e}, most \
                     receded {receded:.1e}); weighted {:.3} mm, {wa:.4} rad",
                    dp * 1e3,
                    wp * 1e3
                );
                // Evidence: the pose is out of reach, and the weighted solve trades position.
                assert!(
                    pins.iter().all(|&i| ik.pinned()[i] == 1),
                    "{case}: {:?}",
                    ik.pinned()
                );
                assert!(wp > 2e-3 && wa < da, "{case}: no evidence");
                assert!(dp < 2e-5 && descent < 1e-9 && receded < 1e-9, "{case}");
                assert_eq!(ik.fallbacks(), 0, "{case}");
            }
        }
    }
}

/// The largest change of any joint's goal velocity from one cycle to the next, rad/s, while `ik`
/// follows `target` of the cycle over `cycles` at `ramp`, and the blend after.
fn largest_step(
    ik: &mut Ik,
    target: impl Fn(usize) -> [f64; 16],
    ramp: u32,
    cycles: usize,
) -> (f64, f64) {
    ik.set_priority_ramp(ramp);
    let (mut last, mut dq) = (ik.q(), [0.0; 7]);
    let mut largest = 0.0f64;
    for k in 0..cycles {
        let (q, _) = ik.step(&target(k), &READY, DELTA_T);
        let next: [f64; 7] = std::array::from_fn(|i| (q[i] - last[i]) / DELTA_T);
        if k > 0 {
            largest = (0..7).fold(largest, |m, i| m.max((next[i] - dq[i]).abs()));
        }
        (last, dq) = (q, next);
    }
    (largest, ik.priority())
}

#[test]
fn the_blend_into_position_first_leaves_no_step_in_the_goal_velocity() {
    for arm in arms() {
        for (tool_name, tool) in TOOLS {
            for (pins_name, pins) in PINS {
                let case = format!("{} {tool_name} {pins_name}", arm.name);
                let limits = pinned_at(&arm, pins, MARGIN);
                let pose = beyond(&arm, &tool, 1.0);
                // The weighted solve along the move and settled on its trade, then position first
                // eased in or at once: the worst place to switch, the two solutions far apart. What
                // is left eased in is the position stage's pins changing on the way, as any pin
                // does.
                let switch = |ramp| {
                    let mut ik = arm.ik(IkOptions::default(), arm.real(limits), READY, tool);
                    let to = |k| beyond(&arm, &tool, smooth(k, RAMP));
                    let (moving, _) = largest_step(&mut ik, to, 0, RAMP + HOLD);
                    let (step, blend) = largest_step(&mut ik, |_| pose, ramp, 2 * RAMP);
                    assert_eq!(blend, 1.0, "{case}");
                    (moving, step)
                };
                let ((moving, eased), (_, instant)) = (switch(PRIORITY_RAMP_CYCLES), switch(1));
                println!(
                    "{case}: goal velocity step {eased:.2e} rad/s eased in, {instant:.2e} at \
                     once, {moving:.2e} along the move"
                );
                assert!(instant > 1.0, "{case}: no evidence");
                assert!(eased < 0.02 * instant, "{case}");
            }
        }
    }
}

// Measured: the flag settles within 40 cycles of the reach and never toggles again; the blend
// stands at 1.
#[test]
fn the_stall_flag_does_not_toggle_while_position_first_holds_a_reach_past_a_pin() {
    let (arm, tool) = (&arms()[0], TOOLS[1].1);
    let limits = pinned_at(arm, [3, 3], MARGIN);
    let mut ik = arm.ik(IkOptions::default(), arm.real(limits), READY, tool);
    // Translation only, 5 cm along x and y: joint 4 pins and the rest cannot make it up.
    let start = arm.fk(&READY, &tool);
    let p = translation_of(&start);
    let target = pose_from(&rotation_of(&start), &[p[0] + 0.05, p[1] + 0.05, p[2]]);
    let mut toggles = 0;
    let (mut was, mut settled) = (false, 0);
    for k in 0..3000 {
        ik.step(&target, &READY, DELTA_T);
        let stalled = ik.stall().stalled;
        // The first second reaches and blends in; the two after it must not toggle.
        if k >= 1000 && stalled != was {
            toggles += 1;
        }
        if stalled != was {
            (was, settled) = (stalled, k);
        }
    }
    println!(
        "{}: stalled {was} from cycle {settled}, blend {:.3}, joint 4 pinned {}",
        arm.name,
        ik.priority(),
        ik.pinned()[3]
    );
    assert_eq!(ik.pinned()[3], 1, "no evidence: joint 4 is not pinned");
    assert!(was && ik.priority() > 0.999, "position first is not in");
    assert_eq!(toggles, 0, "the stall flag toggled {toggles} times");
}
