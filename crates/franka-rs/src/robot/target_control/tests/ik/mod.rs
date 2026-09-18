//! The differential inverse kinematics, offline, on both robot models: tracking and the
//! posture here, the joint limit guard in [`limits`], the solver as it ran in [`as_run`].

mod as_run;
mod limits;
mod priority;

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4};
use std::sync::Arc;

use nalgebra::SVector;

use super::super::ik::*;
use super::super::position::{JointLimits, VelocityLimit};
use super::super::rotation::{
    angle_between, distance, exp, log, pose_from, rotation_of, translation_of,
};
use crate::model::{Frame, Model};
use crate::rate_limiting::{self, DELTA_T};
use crate::wire::robot::codec::FciVersion;

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

/// `F_T_EE` a distance out along the flange's z, unturned.
const fn along_z(z: f64) -> [f64; 16] {
    let mut t = IDENTITY;
    t[14] = z;
    t
}

/// The flange, the Franka Hand, a Robotiq gripper's TCP and a long tool's tip.
const TOOLS: [(&str, [f64; 16]); 4] = [
    ("flange", IDENTITY),
    ("hand", HAND),
    ("robotiq", along_z(0.174)),
    ("long tool", along_z(0.25)),
];

/// The FK of the solution may be this far from the pose it was asked for, m and rad.
const TRACKING: f64 = 1e-4;
const MARGIN: f64 = 0.05;
/// The velocity limit of the tests that are not about it, rad/s: 10 on every joint.
const CAP: [f64; 7] = [10.0; 7];

struct Arm {
    name: &'static str,
    model: Arc<Model>,
    version: FciVersion,
    limits: ([f64; 7], [f64; 7]),
}

fn arms() -> [Arm; 2] {
    let fr3 = include_str!("../../../../../tests/data/fr3.urdf");
    [
        Arm {
            name: "FER",
            model: Arc::new(Model::native_fer()),
            version: FciVersion::V5,
            limits: rate_limiting::fer::JOINT_POSITION_LIMITS,
        },
        Arm {
            name: "FR3",
            model: Arc::new(Model::from_urdf(fr3).expect("fr3.urdf loads")),
            version: FciVersion::V10,
            limits: rate_limiting::JOINT_POSITION_LIMITS,
        },
    ]
}

impl Arm {
    /// The guard on `limits` at the margin, the velocity limit out of the way.
    fn loose(&self, limits: ([f64; 7], [f64; 7])) -> JointLimits {
        JointLimits {
            position: limits,
            margin: MARGIN,
            fraction: 1.0,
            velocity: CAP.into(),
        }
    }

    /// The guard as the loop runs it: the arm's velocity limit, a fraction of 0.7.
    fn real(&self, limits: ([f64; 7], [f64; 7])) -> JointLimits {
        JointLimits {
            fraction: 0.7,
            velocity: VelocityLimit::of(self.version),
            ..self.loose(limits)
        }
    }

    fn ik(&self, options: IkOptions, limits: JointLimits, q0: [f64; 7], tool: [f64; 16]) -> Ik {
        Ik::new(Arc::clone(&self.model), options, limits, q0, tool, IDENTITY)
    }

    fn ready(&self) -> Ik {
        self.ik(IkOptions::default(), self.loose(self.limits), READY, HAND)
    }

    fn fk(&self, q: &[f64; 7], tool: &[f64; 16]) -> [f64; 16] {
        self.model.pose_q(Frame::EndEffector, q, tool, &IDENTITY)
    }

    fn within_margin(&self, q: &[f64; 7], limits: &([f64; 7], [f64; 7])) -> bool {
        let (lower, upper) = limits;
        (0..7).all(|i| q[i] >= lower[i] + MARGIN - 1e-12 && q[i] <= upper[i] - MARGIN + 1e-12)
    }

    /// Steps the IK through `poses` and checks every solution's FK against its pose; the
    /// solution and the worst position and orientation errors.
    fn follow(&self, poses: impl Iterator<Item = [f64; 16]>) -> ([f64; 7], f64, f64) {
        let mut ik = self.ready();
        let (mut q, mut worst) = (READY, (0.0f64, 0.0f64));
        for (k, pose) in poses.enumerate() {
            let (solution, residual) = ik.step(&pose, &READY, DELTA_T);
            q = solution;
            let (dp, da) = pose_distance(&self.fk(&q, &HAND), &pose);
            worst = (worst.0.max(dp), worst.1.max(da));
            let tracks = dp < TRACKING && da < TRACKING && residual < 2.0 * TRACKING;
            assert!(tracks, "{}: step {k} is {dp} m, {da} rad off", self.name);
            assert!(self.within_margin(&q, &self.limits), "{}: {q:?}", self.name);
        }
        (q, worst.0, worst.1)
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
        for (tool_name, tool) in TOOLS {
            let pose = arm.fk(&READY, &tool);
            let mut ik = arm.ik(IkOptions::default(), arm.real(arm.limits), READY, tool);
            let (q, residual) = ik.step(&pose, &READY, DELTA_T);
            let case = format!("{} {tool_name}", arm.name);
            assert!(joint_distance(&q, &READY) < 1e-12, "{case}: {q:?}");
            assert!(residual < 1e-12, "{case}: {residual}");
            assert_eq!((ik.pinned(), ik.passes(), ik.cap_scale()), ([0; 7], 1, 1.0));
        }
    }
}

#[test]
fn follows_a_straight_line() {
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let line = (1..=50).map(|k| pose_from(&r, &[p[0] + f64::from(k) * 1e-3, p[1], p[2]]));
        let (q, dp, da) = arm.follow(line);
        println!(
            "{}: a straight line tracked within {dp:.2e} m, {da:.2e} rad",
            arm.name
        );
        assert!(joint_distance(&q, &READY) > 0.01, "{}", arm.name);
    }
}

#[test]
fn follows_a_rotation_about_the_base_z() {
    let steps = 18;
    let turn = 10f64.to_radians() / f64::from(steps);
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let rotated = |k: u8| pose_from(&(exp(&[0.0, 0.0, f64::from(k) * turn]) * r), &p);
        let (q, dp, da) = arm.follow((1..=steps).map(rotated));
        println!(
            "{}: a base-z turn tracked within {dp:.2e} m, {da:.2e} rad",
            arm.name
        );
        assert!(joint_distance(&q, &READY) > 0.01, "{}", arm.name);
    }
}

#[test]
fn an_unreachable_pose_lags_and_reports_the_weighted_residual() {
    for arm in arms() {
        let start = arm.fk(&READY, &HAND);
        let (p, r) = (translation_of(&start), rotation_of(&start));
        let pose = pose_from(&r, &[p[0] + 1.0, p[1], p[2]]);
        let mut ik = arm.ready();
        let mut residual = 0.0;
        for _ in 0..500 {
            let (q, r) = ik.step(&pose, &READY, DELTA_T);
            assert!(q.iter().all(|v| v.is_finite()), "{}: {q:?}", arm.name);
            assert!(arm.within_margin(&q, &arm.limits), "{}: {q:?}", arm.name);
            residual = r;
        }
        let (dp, da) = pose_distance(&arm.fk(&ik.q(), &HAND), &pose);
        let weight = IkOptions::default().rotation_weight;
        assert!(residual.is_finite() && dp > 0.1, "{}: {residual}", arm.name);
        let weighted = dp.hypot(weight * da);
        assert!(
            (residual - weighted).abs() < 1e-9,
            "{}: {residual}",
            arm.name
        );
        assert_eq!(ik.fallbacks(), 0, "{}", arm.name);
    }
}

#[test]
fn the_nullspace_bias_moves_toward_the_posture_at_a_fixed_pose() {
    let mut posture = READY;
    posture[0] += 0.5;
    posture[2] -= 0.5;
    for arm in arms() {
        let pose = arm.fk(&READY, &HAND);
        let mut ik = arm.ready();
        let mut q = READY;
        for _ in 0..2000 {
            q = ik.step(&pose, &posture, DELTA_T).0;
            let (dp, da) = pose_distance(&arm.fk(&q, &HAND), &pose);
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
        let pose = arm.fk(&READY, &HAND);
        // A posture 2 rad away on joint 1 and 3, with a gain of 10: uncapped, the pull
        // would be 20 rad/s; it is 0.5 rad/s on its largest joint, the pose kept.
        let mut far = READY;
        far[0] += 2.0;
        far[2] -= 2.0;
        let options = IkOptions {
            nullspace_gain: 10.0,
            ..IkOptions::default()
        };
        let mut ik = arm.ik(options, arm.loose(arm.limits), READY, HAND);
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
            let (dp, da) = pose_distance(&arm.fk(&next, &HAND), &pose);
            assert!(dp < TRACKING && da < TRACKING, "{}", arm.name);
            q = next;
        }
        assert!(q[0] > READY[0] + 0.05, "{}: {q:?}", arm.name);
    }
}

/// The largest orientation and position error while `ik` follows a turn of the tool about its
/// own z at 1 rad/s for 0.3 s from `q0`.
fn tool_turn_lag(arm: &Arm, ik: &mut Ik, q0: &[f64; 7], tool: &[f64; 16]) -> (f64, f64) {
    let start = arm.fk(q0, tool);
    let (p, r) = (translation_of(&start), rotation_of(&start));
    (1..=300).fold((0.0f64, 0.0f64), |(da_max, dp_max), k| {
        let pose = pose_from(&(r * exp(&[0.0, 0.0, 1e-3 * f64::from(k)])), &p);
        let (q, _) = ik.step(&pose, q0, DELTA_T);
        let (dp, da) = pose_distance(&arm.fk(&q, tool), &pose);
        (da_max.max(da), dp_max.max(dp))
    })
}

#[test]
fn a_tool_turn_lags_under_the_rotation_weight_by_at_most_a_milliradian() {
    // At the ready pose and at the wrist configuration of a recorded yaw stall, σ_min 0.07;
    // free: the limits pushed out of reach.
    #[rustfmt::skip]
    let stall = [0.892333080197721, 1.4752562981118584, -1.2414016896868978, -0.9920924393023431,
                 2.826896993360278, 3.716821643373931, 2.870729721149785];
    let free = ([-10.0; 7], [10.0; 7]);
    for arm in arms() {
        for (start_name, q0) in [("ready", READY), ("stall", stall)] {
            for (tool_name, tool) in TOOLS {
                let lag = |rotation_weight| {
                    let options = IkOptions {
                        rotation_weight,
                        ..IkOptions::default()
                    };
                    let mut ik = arm.ik(options, arm.loose(free), q0, tool);
                    tool_turn_lag(&arm, &mut ik, &q0, &tool)
                };
                let (weighted, unweighted) = (lag(0.1), lag(1.0));
                let case = format!("{} {start_name} {tool_name}", arm.name);
                println!(
                    "{case}: lag {:.2e} rad / {:.2e} m at 0.1, {:.2e} rad / {:.2e} m at 1",
                    weighted.0, weighted.1, unweighted.0, unweighted.1
                );
                assert!(weighted.0 <= unweighted.0 + 1e-3, "{case}");
            }
        }
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
    assert!(rejects(|o| o.rotation_weight = 0.0));
    assert!(rejects(|o| o.rotation_weight = 1.0 + 1e-12));
    assert!(rejects(|o| o.rotation_weight = f64::NAN));
    let unweighted = IkOptions {
        rotation_weight: 1.0,
        ..IkOptions::default()
    };
    unweighted.validate().unwrap();
}
