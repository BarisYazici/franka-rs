//! The joint limit guard of the IK: the margin, the braking and velocity box, the start, the
//! release, the posture clamp and fade, and the solver's edge cases.

use super::*;

/// The arm's limits with joint 4's lower one 0.3 rad under the ready pose, and a pose that
/// folds it 0.4 rad.
fn folding(arm: &Arm, tool: &[f64; 16]) -> (([f64; 7], [f64; 7]), [f64; 16]) {
    let mut limits = arm.limits;
    limits.0[3] = READY[3] - 0.3;
    let mut folded = READY;
    folded[3] -= 0.4;
    (limits, arm.fk(&folded, tool))
}

/// `‖(dp, w_r·da)‖` of `q` against `pose`.
fn weighted_error(arm: &Arm, q: &[f64; 7], pose: &[f64; 16], tool: &[f64; 16]) -> f64 {
    let (dp, da) = pose_distance(&arm.fk(q, tool), pose);
    dp.hypot(IkOptions::default().rotation_weight * da)
}

#[test]
fn the_margin_holds_a_folding_joint_4_under_the_braking_envelope() {
    for arm in arms() {
        for (tool_name, tool) in TOOLS {
            let case = format!("{} {tool_name}", arm.name);
            let (limits, pose) = folding(&arm, &tool);
            let guard = arm.real(limits);
            let flat = guard.velocity.flat();
            let bound = limits.0[3] + MARGIN;
            let mut ik = arm.ik(IkOptions::default(), guard, READY, tool);
            let mut from = READY;
            for k in 0..1000 {
                let (q, _) = ik.step(&pose, &READY, DELTA_T);
                let x = from[3] - limits.0[3] - MARGIN;
                let braking = 0.7 * guard.velocity.braking(3, x, flat[3]) * DELTA_T;
                assert!(q[3] >= bound - 1e-12, "{case}: call {k} at {}", q[3]);
                assert!(from[3] - q[3] <= braking + 1e-15, "{case}: call {k}");
                from = q;
            }
            assert!(from[3] - bound < 1e-3, "{case}: joint 4 at {}", from[3]);
            assert_eq!(ik.pinned()[3], -1, "{case}");
            assert!(ik.stall().stalled, "{case}: {:?}", ik.stall());
            // The clamp at the same inset stops joint 4 there too, but further from the pose.
            let shifted = (limits.0.map(|l| l + MARGIN - 0.02), limits.1);
            let mut clamp = Ik::as_run(
                Arc::clone(&arm.model),
                IkOptions::default(),
                shifted,
                CAP,
                READY,
                tool,
                IDENTITY,
            );
            let clamped = (0..1000).fold(READY, |_, _| clamp.step(&pose, &READY, DELTA_T).0);
            let (guarded, clamped) = (
                weighted_error(&arm, &from, &pose, &tool),
                weighted_error(&arm, &clamped, &pose, &tool),
            );
            println!("{case}: residual {guarded:.3e} guarded, {clamped:.3e} clamped");
            assert!(guarded <= clamped + 1e-9, "{case}: {guarded} > {clamped}");
            // As it ran, the clamp at 0.02 takes joint 4 past the margin.
            let mut before = Ik::as_run(
                Arc::clone(&arm.model),
                IkOptions::default(),
                limits,
                CAP,
                READY,
                tool,
                IDENTITY,
            );
            let lowest = (0..300).fold(0.0f64, |lowest, _| {
                lowest.min(before.step(&pose, &READY, DELTA_T).0[3])
            });
            assert!(lowest < bound - 0.02, "{case}: {lowest}");
        }
    }
}

#[test]
fn a_start_inside_the_margin_or_past_a_limit_does_not_jump() {
    for arm in arms() {
        let mut start = READY;
        start[0] = arm.limits.1[0] - 0.01;
        start[3] = arm.limits.0[3] - 0.01;
        let guard = arm.real(arm.limits);
        let mut ik = arm.ik(IkOptions::default(), guard, start, HAND);
        assert_eq!(ik.q(), start, "{}: not clamped", arm.name);
        // A pose that asks joint 1 further up and joint 4 further down.
        let mut beyond = start;
        beyond[0] += 0.2;
        beyond[3] -= 0.2;
        let pose = arm.fk(&beyond, &HAND);
        let mut from = start;
        for k in 0..300 {
            let (q, _) = ik.step(&pose, &start, DELTA_T);
            let b = guard.cycle_box(&from, DELTA_T);
            for i in 0..7 {
                let step = q[i] - from[i];
                assert!(
                    step >= b.lower[i] - 1e-15 && step <= b.upper[i] + 1e-15,
                    "{}: call {k}",
                    arm.name
                );
            }
            assert!(
                q[0] <= start[0] && q[3] >= start[3],
                "{}: call {k}: {q:?}",
                arm.name
            );
            from = q;
        }
    }
}

#[test]
fn the_velocity_box_bounds_each_joint_and_leaves_the_backstop_idle() {
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        // A pose a metre away: every call moves some joint at its velocity bound, none past it.
        let pose = pose_from(&r, &[p[0] + 1.0, p[1], p[2]]);
        let guard = arm.real(arm.limits);
        let mut ik = arm.ik(IkOptions::default(), guard, READY, HAND);
        let (mut q, mut last) = (READY, f64::INFINITY);
        for k in 0..50 {
            let (next, residual) = ik.step(&pose, &READY, DELTA_T);
            let b = guard.cycle_box(&q, DELTA_T);
            for i in 0..7 {
                let step = next[i] - q[i];
                assert!(step <= b.velocity_upper[i] + 1e-15 && step >= b.velocity_lower[i] - 1e-15);
            }
            assert_eq!(ik.cap_scale(), 1.0, "{}: call {k}", arm.name);
            assert!(
                ik.pinned().iter().any(|p| p.abs() == 2),
                "{}: call {k}",
                arm.name
            );
            assert!(
                !ik.stall().stalled,
                "{}: velocity pins never stall",
                arm.name
            );
            assert!(residual < last, "{}: call {k}", arm.name);
            (q, last) = (next, residual);
        }
    }
}

#[test]
fn a_joint_pinned_at_its_margin_is_released_in_the_cycle_the_target_turns_back() {
    for arm in arms() {
        let (limits, pose) = folding(&arm, &HAND);
        let mut ik = arm.ik(IkOptions::default(), arm.real(limits), READY, HAND);
        for _ in 0..1000 {
            ik.step(&pose, &READY, DELTA_T);
        }
        assert_eq!(ik.pinned()[3], -1, "{}", arm.name);
        let held = ik.q()[3];
        let back = arm.fk(&READY, &HAND);
        let (q, _) = ik.step(&back, &READY, DELTA_T);
        assert_ne!(ik.pinned()[3], -1, "{}", arm.name);
        assert!(q[3] > held + 1e-4, "{}: {} from {held}", arm.name, q[3]);
    }
}

#[test]
fn the_posture_is_clamped_inside_the_limits_and_fades_near_them() {
    for arm in arms() {
        let upper = arm.limits.1;
        let pose = arm.fk(&READY, &HAND);
        // One iteration: the bias alone, linear in its rate.
        let options = IkOptions {
            iterations: 1,
            ..IkOptions::default()
        };
        let step = |limits, posture: &[f64; 7]| {
            let mut ik = arm.ik(options, arm.loose(limits), READY, HAND);
            let (q, _) = ik.step(&pose, posture, DELTA_T);
            SVector::<f64, 7>::from(q) - SVector::from(READY)
        };
        // A posture past joint 1's limit pulls as one clamped 0.35 rad inside it.
        let (mut past, mut clamped) = (READY, READY);
        past[0] = upper[0] + 1.0;
        clamped[0] = upper[0] - (MARGIN + 0.3);
        let near = step(arm.limits, &past);
        assert!(near.amax() > 1e-5, "{}: {near}", arm.name);
        assert_eq!(near, step(arm.limits, &clamped), "{}", arm.name);
        // Joint 7 0.2 from a limit, half-way into the fade: half the pull of the same posture
        // far from every limit; at the margin, none.
        let mut close = arm.limits;
        close.1[6] = READY[6] + 0.2;
        let mut posture = READY;
        posture[0] += 0.1;
        posture[6] = close.1[6] - MARGIN - 0.3;
        let (half, full) = (step(close, &posture), step(arm.limits, &posture));
        assert!(full.amax() > 1e-5, "{}: {full}", arm.name);
        assert!(
            (half - 0.5 * full).amax() < 1e-12,
            "{}: {half} against {full}",
            arm.name
        );
        close.1[6] = READY[6] + MARGIN;
        let none = step(close, &posture);
        assert!(none.amax() < 1e-12, "{}: {none}", arm.name);
    }
}

#[test]
fn a_non_finite_pose_keeps_the_goal_and_counts() {
    for arm in arms() {
        let mut ik = arm.ready();
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let moved = pose_from(&r, &[p[0] + 1e-3, p[1], p[2]]);
        let (q, _) = ik.step(&moved, &READY, DELTA_T);
        let mut nan = moved;
        nan[12] = f64::NAN;
        let (held, residual) = ik.step(&nan, &READY, DELTA_T);
        assert_eq!(held, q, "{}", arm.name);
        assert!(residual.is_nan() && ik.fallbacks() == 1, "{}", arm.name);
        let (after, residual) = ik.step(&moved, &READY, DELTA_T);
        assert!(
            after.iter().all(|x| x.is_finite()) && residual < TRACKING,
            "{}",
            arm.name
        );
        assert_eq!(ik.fallbacks(), 1, "{}", arm.name);
    }
}

#[test]
fn the_first_jacobian_of_a_cycle_is_the_last_goals_bit_for_bit() {
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let at = |k: u32| pose_from(&r, &[p[0] + f64::from(k) * 2e-4, p[1], p[2]]);
        let mut ik = arm.ready();
        for k in 1..=20 {
            ik.step(&at(k), &READY, DELTA_T);
        }
        let options = IkOptions::default();
        let mut fresh = arm.ik(options, arm.loose(arm.limits), ik.q(), HAND);
        let (reused, fresh) = (
            ik.step(&at(21), &READY, DELTA_T),
            fresh.step(&at(21), &READY, DELTA_T),
        );
        assert_eq!(
            reused.0.map(f64::to_bits),
            fresh.0.map(f64::to_bits),
            "{}",
            arm.name
        );
        assert_eq!(reused.1.to_bits(), fresh.1.to_bits(), "{}", arm.name);
    }
}

#[test]
fn the_elbow_stops_under_a_pinned_wrist_where_the_clamp_walked_it() {
    // Joint 5 at its upper margin (the clamp's inset as it ran), a tool yaw 0.1 rad ahead held
    // still, the posture off the start as a session's first configuration was.
    let mut posture = READY;
    posture[0] += 0.5;
    posture[2] -= 0.5;
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let pose = pose_from(
            &(rotation_of(&start) * exp(&[0.0, 0.0, 0.1])),
            &translation_of(&start),
        );
        let (mut clamped, mut guarded) = (arm.limits, arm.limits);
        (clamped.1[4], guarded.1[4]) = (READY[4] + 0.02, READY[4] + MARGIN);
        let guard = arm.real(guarded);
        let cap = guard.velocity.flat().map(|v| 0.7 * v);
        let model = Arc::clone(&arm.model);
        let mut before = Ik::as_run(
            model,
            IkOptions::default(),
            clamped,
            cap,
            READY,
            HAND,
            IDENTITY,
        );
        let mut ik = arm.ik(IkOptions::default(), guard, READY, HAND);
        let run = |ik: &mut Ik| -> Vec<[f64; 7]> {
            (0..1500)
                .map(|_| ik.step(&pose, &posture, DELTA_T).0)
                .collect()
        };
        let (walked, held) = (run(&mut before), run(&mut ik));
        // Joints 1 and 3's largest move in each 100 ms after the first 0.5 s, rad.
        let elbow = |q: &[[f64; 7]]| -> Vec<f64> {
            let moved = |k: usize, j: usize| (q[k + 99][j] - q[k][j]).abs();
            (500..1500)
                .step_by(100)
                .map(|k| moved(k, 0).max(moved(k, 2)))
                .collect()
        };
        let slowest = elbow(&walked).into_iter().fold(f64::INFINITY, f64::min);
        let drift = elbow(&held).into_iter().fold(0.0, f64::max);
        let tool =
            |q: &[f64; 7]| angle_between(&rotation_of(&arm.fk(q, &HAND)), &rotation_of(&pose));
        println!(
            "{}: as run the elbow walks at least {:.1} mrad per 100 ms, guarded at most {:.3}",
            arm.name,
            slowest * 1e3,
            drift * 1e3
        );
        // Evidence: the clamp's elbow still walks over 50 mrad/s with the yaw on target.
        assert!(
            slowest > 5e-3 && tool(&walked[1499]) < 1e-3,
            "{}: no evidence",
            arm.name
        );
        assert!(
            drift <= 1e-3 && tool(&held[1499]) < 1e-3,
            "{}: {drift}",
            arm.name
        );
        assert_eq!(ik.pinned()[4], 1, "{}", arm.name);
    }
}
