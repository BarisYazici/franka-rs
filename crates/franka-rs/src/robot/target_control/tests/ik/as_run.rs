//! The as-run constructor against the solver before the guard, kept here verbatim: the
//! replays' oracle rests on the two agreeing.

use nalgebra::{Matrix3, SMatrix, Vector3};

use super::super::super::rotation::log;
use super::super::super::velocity::cap_step;
use super::*;

/// The solver before the guard, as it was: its state and one step of it.
struct Before<'a> {
    arm: &'a Arm,
    options: IkOptions,
    lower: [f64; 7],
    upper: [f64; 7],
    max_velocity: [f64; 7],
    tool: [f64; 16],
    q: [f64; 7],
}

impl Before<'_> {
    fn error(&self, p_des: &Vector3<f64>, r_des: &Matrix3<f64>) -> SVector<f64, 6> {
        let fk = self.arm.fk(&self.q, &self.tool);
        let p = p_des - Vector3::from(translation_of(&fk));
        let turn = log(&(r_des * rotation_of(&fk).transpose()));
        SVector::<f64, 6>::new(p[0], p[1], p[2], turn[0], turn[1], turn[2])
    }

    fn step(&mut self, pose: &[f64; 16], posture: &[f64; 7], dt: f64) -> ([f64; 7], f64) {
        let p_des = Vector3::from(translation_of(pose));
        let r_des = rotation_of(pose);
        let posture = SVector::<f64, 7>::from(*posture);
        let damping =
            SMatrix::<f64, 6, 6>::identity() * (self.options.damping * self.options.damping);
        let from = self.q;
        let mut residual = 0.0;
        for iteration in 0..=self.options.iterations {
            let error = self.error(&p_des, &r_des);
            residual = error.norm();
            if iteration == self.options.iterations
                || (iteration > 0 && residual < self.options.tolerance)
            {
                break;
            }
            let j = SMatrix::<f64, 6, 7>::from_column_slice(&self.arm.model.zero_jacobian_q(
                Frame::EndEffector,
                &self.q,
                &self.tool,
                &IDENTITY,
            ));
            let Some(inverse) = (j * j.transpose() + damping).try_inverse() else {
                break;
            };
            let pseudo_inverse = j.transpose() * inverse;
            let q = SVector::<f64, 7>::from(self.q);
            let mut next = q + pseudo_inverse * error;
            if iteration == 0 {
                let nullspace = SMatrix::<f64, 7, 7>::identity() - pseudo_inverse * j;
                let mut bias = nullspace * (posture - q) * (self.options.nullspace_gain * dt);
                let (largest, cap) = (bias.amax(), MAX_POSTURE_RATE * dt);
                if largest > cap {
                    bias *= cap / largest;
                }
                next += bias;
            }
            for i in 0..7 {
                self.q[i] = next[i].clamp(self.lower[i], self.upper[i]);
            }
        }
        let max_step = self.max_velocity.map(|v| v * dt);
        if cap_step(&from, &mut self.q, &max_step) < 1.0 {
            residual = self.error(&p_des, &r_des).norm();
        }
        (self.q, residual)
    }
}

/// Runs the as-run IK and the solver before it side by side from `q0` through `poses`, and
/// returns the largest joint and residual differences.
fn side_by_side(
    arm: &Arm,
    limits: ([f64; 7], [f64; 7]),
    max_velocity: [f64; 7],
    q0: [f64; 7],
    posture: [f64; 7],
    poses: impl Iterator<Item = [f64; 16]>,
) -> (f64, f64) {
    let model = Arc::clone(&arm.model);
    let options = IkOptions::default();
    let mut ik = Ik::as_run(model, options, limits, max_velocity, q0, HAND, IDENTITY);
    let mut before = Before {
        arm,
        options,
        lower: limits.0.map(|l| l + 0.02),
        upper: limits.1.map(|u| u - 0.02),
        max_velocity,
        tool: HAND,
        q: q0,
    };
    before.q = std::array::from_fn(|i| q0[i].clamp(before.lower[i], before.upper[i]));
    assert_eq!(ik.q(), before.q);
    poses.fold((0.0f64, 0.0f64), |(dq, dr), pose| {
        let (a, ra) = ik.step(&pose, &posture, DELTA_T);
        let (b, rb) = before.step(&pose, &posture, DELTA_T);
        let joints = (0..7).fold(0.0f64, |m, i| m.max((a[i] - b[i]).abs()));
        (dq.max(joints), dr.max((ra - rb).abs()))
    })
}

#[test]
fn the_as_run_solver_is_the_solver_before_the_guard() {
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let line = (1..=300).map(|k| pose_from(&r, &[p[0] + f64::from(k) * 1e-3, p[1], p[2]]));
        let turn = (1..=300).map(|k| pose_from(&(exp(&[0.0, 0.0, 2e-3 * f64::from(k)]) * r), &p));
        let far = pose_from(&r, &[p[0] + 1.0, p[1], p[2]]);
        let (mut folding, mut posture) = (arm.limits, READY);
        folding.0[3] = READY[3] - 0.3;
        posture[0] += 1.0;
        let mut folded = READY;
        folded[3] -= 0.4;
        let fold = arm.fk(&folded, &HAND);
        let fer = rate_limiting::fer::MAX_JOINT_VELOCITY.map(|v| 0.7 * v);
        let runs = [
            (
                "line",
                side_by_side(&arm, arm.limits, CAP, READY, READY, line),
            ),
            (
                "turn",
                side_by_side(&arm, arm.limits, CAP, READY, posture, turn),
            ),
            (
                "capped",
                side_by_side(
                    &arm,
                    arm.limits,
                    fer,
                    READY,
                    READY,
                    std::iter::repeat_n(far, 300),
                ),
            ),
            (
                "clamped",
                side_by_side(
                    &arm,
                    folding,
                    CAP,
                    READY,
                    posture,
                    std::iter::repeat_n(fold, 300),
                ),
            ),
        ];
        for (name, (dq, dr)) in runs {
            println!(
                "{} {name}: as-run within {dq:.1e} rad, residual {dr:.1e}",
                arm.name
            );
            assert!(dq <= 1e-6 && dr <= 1e-6, "{} {name}: {dq}, {dr}", arm.name);
        }
    }
}

#[test]
fn the_as_run_start_is_clamped_inside_the_inset_limits() {
    for arm in arms() {
        let mut outside = READY;
        outside[0] = arm.limits.1[0] + 0.1;
        outside[3] = arm.limits.0[3] - 0.1;
        let model = Arc::clone(&arm.model);
        let ik = Ik::as_run(
            model,
            IkOptions::default(),
            arm.limits,
            CAP,
            outside,
            HAND,
            IDENTITY,
        );
        let q = ik.q();
        assert_eq!(q[0], arm.limits.1[0] - AS_RUN_LIMIT_MARGIN, "{}", arm.name);
        assert_eq!(q[3], arm.limits.0[3] + AS_RUN_LIMIT_MARGIN, "{}", arm.name);
        assert_eq!(q[1], READY[1]);
    }
}

#[test]
fn the_as_run_step_is_capped_per_joint_as_a_whole() {
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let as_run = |q, cap| {
            let model = Arc::clone(&arm.model);
            Ik::as_run(
                model,
                IkOptions::default(),
                arm.limits,
                cap,
                q,
                HAND,
                IDENTITY,
            )
        };
        // A pose a metre away asks for a jump: each call moves every joint at most its cap for
        // one cycle, the binding joint exactly that, along the uncapped step's direction.
        let pose = pose_from(&r, &[p[0] + 1.0, p[1], p[2]]);
        for cap in [CAP, [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]] {
            let mut ik = as_run(READY, cap);
            let mut q = READY;
            for k in 0..50 {
                let free = as_run(q, [f64::INFINITY; 7]).step(&pose, &READY, DELTA_T).0;
                let (next, residual) = ik.step(&pose, &READY, DELTA_T);
                let scale = ik.cap_scale();
                assert!(scale < 1.0, "{}: call {k} was not capped", arm.name);
                let binding = (0..7)
                    .map(|i| (next[i] - q[i]).abs() / (cap[i] * DELTA_T))
                    .fold(0.0, f64::max);
                assert!(
                    (binding - 1.0).abs() < 1e-9,
                    "{}: call {k} at {binding}",
                    arm.name
                );
                for i in 0..7 {
                    let off = next[i] - q[i] - scale * (free[i] - q[i]);
                    assert!(
                        off.abs() < 1e-12,
                        "{}: call {k}, joint {i}: {off}",
                        arm.name
                    );
                }
                let (dp, da) = pose_distance(&arm.fk(&next, &HAND), &pose);
                assert!((residual - dp.hypot(da)).abs() < 1e-9, "{}", arm.name);
                q = next;
            }
        }
        // A reachable pose 1 mm away is not capped.
        let near = pose_from(&r, &[p[0] + 1e-3, p[1], p[2]]);
        let mut ik = as_run(READY, CAP);
        let (q, residual) = ik.step(&near, &READY, DELTA_T);
        assert_eq!(ik.cap_scale(), 1.0, "{}", arm.name);
        assert!(residual < TRACKING, "{}: {residual}", arm.name);
        assert!(joint_distance(&q, &READY) < 0.01, "{}", arm.name);
    }
}
