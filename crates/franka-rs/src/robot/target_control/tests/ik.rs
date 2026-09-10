//! The differential inverse kinematics, offline, on both robot models.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4};
use std::sync::Arc;

use nalgebra::SVector;

use super::super::ik::*;
use super::super::rotation::{
    angle_between, distance, exp, pose_from, rotation_of, translation_of,
};
use crate::model::{Frame, Model};
use crate::rate_limiting::{self, DELTA_T};

#[rustfmt::skip]
const READY: [f64; 7] = [0.0, -FRAC_PI_4, 0.0, -3.0 * FRAC_PI_4, 0.0, FRAC_PI_2, FRAC_PI_4];
/// The Franka Hand's `F_T_EE`: turned -45° about z, 0.1034 m out along it.
#[rustfmt::skip]
const HAND: [f64; 16] = [
    FRAC_1_SQRT_2, -FRAC_1_SQRT_2, 0.0, 0.0,
    FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.1034, 1.0,
];
#[rustfmt::skip]
const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
/// The FK of the solution may be this far from the pose it was asked for, m and rad.
const TRACKING: f64 = 1e-4;
const MARGIN: f64 = 0.02;

struct Arm {
    name: &'static str,
    model: Arc<Model>,
    limits: ([f64; 7], [f64; 7]),
}

fn arms() -> [Arm; 2] {
    let fr3 = include_str!("../../../../tests/data/fr3.urdf");
    [
        Arm {
            name: "FER",
            model: Arc::new(Model::native_fer()),
            limits: rate_limiting::fer::JOINT_POSITION_LIMITS,
        },
        Arm {
            name: "FR3",
            model: Arc::new(Model::from_urdf(fr3).expect("fr3.urdf loads")),
            limits: rate_limiting::JOINT_POSITION_LIMITS,
        },
    ]
}

impl Arm {
    fn ik(&self, options: IkOptions, limits: ([f64; 7], [f64; 7])) -> Ik {
        let model = Arc::clone(&self.model);
        Ik::new(model, options, limits, READY, HAND, IDENTITY)
    }

    fn fk(&self, q: &[f64; 7]) -> [f64; 16] {
        self.model.pose_q(Frame::EndEffector, q, &HAND, &IDENTITY)
    }

    fn within_limits(&self, q: &[f64; 7]) -> bool {
        let (lower, upper) = self.limits;
        (0..7).all(|i| q[i] >= lower[i] + MARGIN - 1e-12 && q[i] <= upper[i] - MARGIN + 1e-12)
    }

    /// Steps the IK through `poses` and checks every solution's FK against its pose.
    fn follow(&self, poses: impl Iterator<Item = [f64; 16]>) -> [f64; 7] {
        let mut ik = self.ik(IkOptions::default(), self.limits);
        let mut q = READY;
        for (k, pose) in poses.enumerate() {
            let (solution, residual) = ik.step(&pose, &READY, DELTA_T);
            q = solution;
            let (dp, da) = pose_distance(&self.fk(&q), &pose);
            let tracks = dp < TRACKING && da < TRACKING && residual < 2.0 * TRACKING;
            assert!(tracks, "{}: step {k} is {dp} m, {da} rad off", self.name);
            assert!(self.within_limits(&q), "{}: {q:?}", self.name);
        }
        q
    }
}

fn pose_distance(a: &[f64; 16], b: &[f64; 16]) -> (f64, f64) {
    (
        distance(&translation_of(a), &translation_of(b)),
        angle_between(&rotation_of(a), &rotation_of(b)),
    )
}

fn joint_distance(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    (SVector::<f64, 7>::from(*a) - SVector::<f64, 7>::from(*b)).norm()
}

#[test]
fn at_the_pose_nothing_moves() {
    for arm in arms() {
        let pose = arm.fk(&READY);
        let mut ik = arm.ik(IkOptions::default(), arm.limits);
        let (q, residual) = ik.step(&pose, &READY, DELTA_T);
        assert!(joint_distance(&q, &READY) < 1e-12, "{}: {q:?}", arm.name);
        assert!(residual < 1e-12, "{}: {residual}", arm.name);
    }
}

#[test]
fn follows_a_straight_line() {
    for arm in arms() {
        let start = arm.fk(&READY);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let line = (1..=50).map(|k| pose_from(&r, &[p[0] + f64::from(k) * 1e-3, p[1], p[2]]));
        let q = arm.follow(line);
        assert!(joint_distance(&q, &READY) > 0.01, "{}", arm.name);
    }
}

#[test]
fn follows_a_rotation_about_the_base_z() {
    let steps = 18;
    let turn = 10f64.to_radians() / f64::from(steps);
    for arm in arms() {
        let start = arm.fk(&READY);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let rotated = |k: u8| pose_from(&(exp(&[0.0, 0.0, f64::from(k) * turn]) * r), &p);
        let q = arm.follow((1..=steps).map(rotated));
        assert!(joint_distance(&q, &READY) > 0.01, "{}", arm.name);
    }
}

#[test]
fn an_unreachable_pose_lags_and_reports_the_residual() {
    for arm in arms() {
        let start = arm.fk(&READY);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let pose = pose_from(&r, &[p[0] + 1.0, p[1], p[2]]);
        let mut ik = arm.ik(IkOptions::default(), arm.limits);
        let mut residual = 0.0;
        for _ in 0..500 {
            let (q, r) = ik.step(&pose, &READY, DELTA_T);
            assert!(q.iter().all(|v| v.is_finite()), "{}: {q:?}", arm.name);
            assert!(arm.within_limits(&q), "{}: {q:?}", arm.name);
            residual = r;
        }
        let (dp, da) = pose_distance(&arm.fk(&ik.q()), &pose);
        assert!(residual.is_finite() && dp > 0.1, "{}: {residual}", arm.name);
        assert!(
            (residual - dp.hypot(da)).abs() < 1e-9,
            "{}: {residual}",
            arm.name
        );
    }
}

#[test]
fn the_nullspace_bias_moves_toward_the_posture_at_a_fixed_pose() {
    let mut posture = READY;
    posture[0] += 0.5;
    posture[2] -= 0.5;
    for arm in arms() {
        let pose = arm.fk(&READY);
        let mut ik = arm.ik(IkOptions::default(), arm.limits);
        let mut q = READY;
        for _ in 0..2000 {
            q = ik.step(&pose, &posture, DELTA_T).0;
            let (dp, da) = pose_distance(&arm.fk(&q), &pose);
            assert!(
                dp < TRACKING && da < TRACKING,
                "{}: {dp} m, {da} rad",
                arm.name
            );
        }
        let (before, after) = (
            joint_distance(&READY, &posture),
            joint_distance(&q, &posture),
        );
        assert!(after < before - 0.05, "{}: {before} -> {after}", arm.name);
        assert!(
            q[0] > READY[0] + 0.05 && q[2] < READY[2] - 0.05,
            "{}: {q:?}",
            arm.name
        );
    }
}

#[test]
fn the_posture_pull_is_capped_at_max_posture_rate() {
    for arm in arms() {
        let pose = arm.fk(&READY);
        // A posture 2 rad away on joint 1 and 3, with a gain of 10: uncapped, the pull
        // would be 20 rad/s; it is 0.5 rad/s on its largest joint, the pose kept.
        let mut far = READY;
        far[0] += 2.0;
        far[2] -= 2.0;
        let options = IkOptions {
            nullspace_gain: 10.0,
            ..IkOptions::default()
        };
        let mut ik = arm.ik(options, arm.limits);
        let mut q = READY;
        let cap = MAX_POSTURE_RATE * DELTA_T;
        for k in 0..200 {
            let (next, _) = ik.step(&pose, &far, DELTA_T);
            let largest = (0..7).fold(0.0f64, |m, i| m.max((next[i] - q[i]).abs()));
            // The pose correction rides on top of the bias, within the tracking error.
            assert!(
                largest <= cap + TRACKING,
                "{}: call {k} moved {largest} rad against {cap}",
                arm.name
            );
            assert!(largest > 0.5 * cap, "{}: the pull is silent", arm.name);
            let (dp, da) = pose_distance(&arm.fk(&next), &pose);
            assert!(dp < TRACKING && da < TRACKING, "{}", arm.name);
            q = next;
        }
        assert!(q[0] > READY[0] + 0.05, "{}: {q:?}", arm.name);
    }
}

#[test]
fn the_start_is_clamped_inside_the_inset_limits() {
    for arm in arms() {
        let mut outside = READY;
        outside[0] = arm.limits.1[0] + 0.1;
        outside[3] = arm.limits.0[3] - 0.1;
        let ik = Ik::new(
            Arc::clone(&arm.model),
            IkOptions::default(),
            arm.limits,
            outside,
            HAND,
            IDENTITY,
        );
        let q = ik.q();
        assert_eq!(q[0], arm.limits.1[0] - MARGIN, "{}", arm.name);
        assert_eq!(q[3], arm.limits.0[3] + MARGIN, "{}", arm.name);
        assert_eq!(q[1], READY[1]);
    }
}

#[test]
fn the_limit_margin_holds_a_folding_joint_4() {
    for arm in arms() {
        let mut limits = arm.limits;
        limits.0[3] = READY[3] - 0.3;
        let mut folded = READY;
        folded[3] -= 0.4;
        let pose = arm.fk(&folded);
        let lowest_joint_4 = |limit_margin: f64| {
            let options = IkOptions {
                limit_margin,
                ..IkOptions::default()
            };
            let mut ik = arm.ik(options, limits);
            (0..200).fold(f64::INFINITY, |lowest, _| {
                lowest.min(ik.step(&pose, &READY, DELTA_T).0[3])
            })
        };
        let bound = limits.0[3] + 0.1;
        let with_margin = lowest_joint_4(0.1);
        assert!(
            with_margin >= bound - 1e-12,
            "{}: {with_margin} < {bound}",
            arm.name
        );
        assert!(
            (with_margin - bound).abs() < 1e-9,
            "{}: the margin did not bind",
            arm.name
        );
        assert!(
            lowest_joint_4(0.0) < bound - 0.05,
            "{}: joint 4 stays above",
            arm.name
        );
    }
}

#[test]
fn the_step_is_capped_at_max_step_per_call() {
    for arm in arms() {
        let start = arm.fk(&READY);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        // A pose a metre away asks for a jump; the solution moves at most `max_step` on
        // its largest joint per call, all joints scaled alike.
        let pose = pose_from(&r, &[p[0] + 1.0, p[1], p[2]]);
        for max_step in [0.01, 0.002] {
            let options = IkOptions {
                max_step,
                ..IkOptions::default()
            };
            let mut ik = arm.ik(options, arm.limits);
            let mut q = READY;
            for k in 0..50 {
                let (next, residual) = ik.step(&pose, &READY, DELTA_T);
                let largest = (0..7).fold(0.0f64, |m, i| m.max((next[i] - q[i]).abs()));
                assert!(
                    largest <= max_step + 1e-12,
                    "{}: call {k} moved {largest} rad against {max_step}",
                    arm.name
                );
                assert!(
                    largest > 0.99 * max_step,
                    "{}: the cap did not bind",
                    arm.name
                );
                let (dp, da) = pose_distance(&arm.fk(&next), &pose);
                assert!((residual - dp.hypot(da)).abs() < 1e-9, "{}", arm.name);
                q = next;
            }
        }
        // A reachable pose 1 mm away is not capped.
        let near = pose_from(&r, &[p[0] + 1e-3, p[1], p[2]]);
        let mut ik = arm.ik(IkOptions::default(), arm.limits);
        let (q, residual) = ik.step(&near, &READY, DELTA_T);
        assert!(residual < TRACKING, "{}: {residual}", arm.name);
        assert!(joint_distance(&q, &READY) < 0.01, "{}", arm.name);
    }
}

#[test]
fn options_validate() {
    IkOptions::default().validate().unwrap();
    let rejects = |change: fn(&mut IkOptions)| {
        let mut options = IkOptions::default();
        change(&mut options);
        options.validate().is_err()
    };
    assert!(rejects(|o| o.damping = 0.0));
    assert!(rejects(|o| o.nullspace_gain = -1.0));
    assert!(rejects(|o| o.iterations = 0));
    assert!(rejects(|o| o.tolerance = f64::NAN));
    assert!(rejects(|o| o.limit_margin = f64::INFINITY));
    assert!(rejects(|o| o.max_step = 0.0));
    assert!(rejects(|o| o.max_step = f64::NAN));
    assert_eq!(IkOptions::default().max_step, 0.01);
}
