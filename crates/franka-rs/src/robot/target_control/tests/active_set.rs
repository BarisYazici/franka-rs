//! The box-constrained solve against a brute-force oracle, and the stall flag, offline.

use nalgebra::{SMatrix, SVector};

use super::super::ik::active_set::*;
use super::super::ik::stall::{Stall, Wall};
use super::super::position::{JointLimits, VelocityLimit};
use super::super::rotation::{log, rotation_of, translation_of};
use super::super::*;
use super::plant::HAND;
use crate::model::{Frame, Model};
use crate::rate_limiting::{fer::JOINT_POSITION_LIMITS, DELTA_T};

const DAMPING: f64 = 0.05;
const WEIGHT: f64 = 0.1;

struct Problem {
    j: Jacobian,
    e: Task,
    lo: Joints,
    hi: Joints,
}

/// `½‖Jd − e‖² + ½λ²‖d‖²` minimised on every one of the 3⁷ active sets by the primal normal
/// equations, the feasible minimum kept.
fn oracle(p: &Problem) -> Joints {
    let mut best = (f64::INFINITY, Joints::zeros());
    for code in 0..3usize.pow(7) {
        let act: [i8; 7] = std::array::from_fn(|i| (code / 3usize.pow(i as u32) % 3) as i8 - 1);
        let fixed = Joints::from_fn(|i, _| match act[i] {
            -1 => p.lo[i],
            1 => p.hi[i],
            _ => 0.0,
        });
        let mut h = p.j.transpose() * p.j + SMatrix::<f64, 7, 7>::identity() * DAMPING.powi(2);
        let mut b = p.j.transpose() * (p.e - p.j * fixed);
        for i in (0..7).filter(|&i| act[i] != 0) {
            h.row_mut(i).fill(0.0);
            h.column_mut(i).fill(0.0);
            h[(i, i)] = 1.0;
            b[i] = 0.0;
        }
        let d = h.lu().solve(&b).expect("regular") + fixed;
        let feasible = (0..7).all(|i| d[i] >= p.lo[i] - 1e-12 && d[i] <= p.hi[i] + 1e-12);
        let cost = 0.5 * (p.j * d - p.e).norm_squared() + 0.5 * DAMPING.powi(2) * d.norm_squared();
        if feasible && cost < best.0 {
            best = (cost, d);
        }
    }
    best.1
}

/// Solves `p` warm-started on its own active set until it converges, as the cycles of a
/// standing problem do.
fn solved(p: &Problem, act: &mut [i8; 7]) -> Joints {
    for _ in 0..4 {
        let s = solve(
            &p.j,
            &p.e,
            (&p.lo, &p.hi),
            None,
            act,
            DAMPING,
            MAX_ACTIVE_SET_PASSES,
        )
        .expect("finite");
        if s.converged {
            return s.d;
        }
    }
    panic!("no convergence on {act:?}");
}

/// The weighted Jacobian and error of the hand at `q` toward `target` (x, y, z, qx, qy, qz, qw),
/// and the cycle box of the FER from `q`.
fn problem_at(model: &Model, q: &[f64; 7], target: Option<[f64; 7]>, e: Task) -> Problem {
    let (f_t_ee, identity) = (HAND.f_t_ee, RobotState::default().EE_T_K);
    let mut j = Jacobian::from_column_slice(&model.zero_jacobian_q(
        Frame::EndEffector,
        q,
        &f_t_ee,
        &identity,
    ));
    j.fixed_rows_mut::<3>(3).scale_mut(WEIGHT);
    let e = target.map_or(e, |t| {
        let fk = model.pose_q(Frame::EndEffector, q, &f_t_ee, &identity);
        let p = translation_of(&fk);
        let r = super::super::rotation::from_quaternion(&[t[3], t[4], t[5], t[6]]);
        let turn = log(&(r * rotation_of(&fk).transpose()));
        Task::new(
            t[0] - p[0],
            t[1] - p[1],
            t[2] - p[2],
            WEIGHT * turn[0],
            WEIGHT * turn[1],
            WEIGHT * turn[2],
        )
    });
    let limits = JointLimits {
        position: JOINT_POSITION_LIMITS,
        margin: 0.05,
        fraction: 0.7,
        velocity: VelocityLimit::of(FciVersion::V5),
    };
    let b = limits.cycle_box(q, DELTA_T);
    Problem {
        j,
        e,
        lo: b.lower.into(),
        hi: b.upper.into(),
    }
}

fn random() -> impl FnMut() -> f64 {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn assert_matches(p: &Problem, d: &Joints, case: &str) {
    let expected = oracle(p);
    let off = (d - expected).amax();
    assert!(
        off <= 1e-9,
        "{case}: {off:e} off the oracle, {d} against {expected}"
    );
}

#[test]
fn the_solve_is_the_box_qp_on_random_problems_near_the_limits() {
    let model = Model::native_fer();
    let (lower, upper) = JOINT_POSITION_LIMITS;
    let mut random = random();
    let mut pinned = 0;
    for k in 0..100 {
        // Every joint within 0.15 rad of one of its limits or mid-range, pushed 1–15 mm / mrad.
        let q: [f64; 7] = std::array::from_fn(|i| match (random() * 3.0) as u32 {
            0 => lower[i] + 0.15 * random(),
            1 => upper[i] - 0.15 * random(),
            _ => lower[i] + (upper[i] - lower[i]) * random(),
        });
        let scale = 1e-3 + 1.4e-2 * random();
        let e = Task::from_fn(|_, _| scale * (2.0 * random() - 1.0));
        let p = problem_at(&model, &q, None, e);
        let mut act = [0; 7];
        let d = solved(&p, &mut act);
        pinned += act.iter().filter(|&&a| a != 0).count();
        assert_matches(&p, &d, &format!("problem {k}"));
    }
    assert!(pinned > 100, "only {pinned} pins over the problems");
}

#[test]
fn the_solve_is_the_box_qp_at_the_recorded_joint_4_faults() {
    let model = Model::native_fer();
    #[rustfmt::skip]
    let faults = [
        ([-0.29367650789403094, -1.4156815719309788, 0.5620948132880287, -3.0518,
          0.7504959420498536, 2.2291428222428564, 1.0051317696287403],
         [0.29897548310328087, 0.031920766435563394, 0.3640051323181709, 0.9094673421348625,
          -0.30483939511080815, 0.271834494943897, -0.07776955790810533]),
        ([0.9037485351484535, -1.1901237046432156, 0.13636988750490617, -3.0518,
          1.4051091743395523, 3.318546903805811, -0.10951877745485623],
         [0.26757500670033996, 0.20387332328592556, 0.4549067475834376, 0.7949792476381141,
          0.20191219945899444, 0.4869257658905, -0.30023783580075214]),
    ];
    for (k, (q, target)) in faults.iter().enumerate() {
        let p = problem_at(&model, q, Some(*target), Task::zeros());
        let mut act = [0; 7];
        let d = solved(&p, &mut act);
        assert_eq!(p.lo[3], 0.0, "joint 4 is inside its margin");
        assert_matches(&p, &d, &format!("fault {k}"));
    }
}

/// Six joints on the task's axes, the seventh on half of each, within `±bound`.
fn synthetic(e: Task, bound: [f64; 7]) -> Problem {
    let mut j = Jacobian::zeros();
    for i in 0..6 {
        j[(i, i)] = 1.0;
        j[(i, 6)] = 0.5;
    }
    Problem {
        j,
        e,
        lo: -Joints::from(bound),
        hi: Joints::from(bound),
    }
}

#[test]
fn a_new_pin_takes_the_side_its_joint_violates() {
    let p = synthetic(Task::new(0.1, -0.1, 0.0, 0.0, 0.0, 0.0), [0.01; 7]);
    let mut act = [0; 7];
    let d = solved(&p, &mut act);
    assert_eq!(&act[..2], &[1, -1]);
    assert_eq!((d[0], d[1]), (0.01, -0.01));
    assert_matches(&p, &d, "synthetic");
}

#[test]
fn every_joint_pinned_leaves_an_empty_free_set_and_a_finite_step() {
    let p = synthetic(Task::repeat(1.0), [1e-4; 7]);
    let mut act = [0; 7];
    let d = solved(&p, &mut act);
    assert_eq!(act, [1; 7]);
    assert_eq!(d, Joints::repeat(1e-4));
    assert_matches(&p, &d, "all pinned");
    // Warm-started there, one pass confirms it.
    let s = solve(&p.j, &p.e, (&p.lo, &p.hi), None, &mut act, DAMPING, 8).unwrap();
    assert!(s.converged && s.passes == 1 && s.d == d, "{s:?}");
}

#[test]
fn a_pin_whose_multiplier_points_inward_is_released_in_one_pass() {
    // Pinned upward on joint 1 while the task pulls it down: released and solved free.
    let p = synthetic(
        Task::new(-1e-4, 0.0, 0.0, 0.0, 0.0, 0.0),
        [1e-3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    );
    let mut act = [1, 0, 0, 0, 0, 0, 0];
    let s = solve(&p.j, &p.e, (&p.lo, &p.hi), None, &mut act, DAMPING, 8).unwrap();
    assert!(s.converged && s.passes == 2, "{s:?}");
    assert_eq!(act, [0; 7]);
    assert!(s.d[0] < 0.0);
    assert_matches(&p, &s.d, "released");
}

#[test]
fn no_passes_is_the_plain_damped_step_clipped_and_non_finite_is_refused() {
    let p = synthetic(Task::new(0.1, 1e-3, 0.0, 0.0, 0.0, 0.0), [0.01; 7]);
    let mut act = [0; 7];
    let s = solve(&p.j, &p.e, (&p.lo, &p.hi), None, &mut act, DAMPING, 0).unwrap();
    let a = p.j * p.j.transpose() + SMatrix::<f64, 6, 6>::identity() * DAMPING.powi(2);
    let plain = p.j.transpose() * a.try_inverse().unwrap() * p.e;
    let clipped = Joints::from_fn(|i, _| plain[i].clamp(-0.01, 0.01));
    assert!((s.d - clipped).amax() < 1e-15 && s.passes == 0 && s.converged);
    let nan = Task::repeat(f64::NAN);
    let mut act = [1, 0, 0, 0, 0, 0, -1];
    assert!(solve(&p.j, &nan, (&p.lo, &p.hi), None, &mut act, DAMPING, 8).is_none());
    assert_eq!(act, [1, 0, 0, 0, 0, 0, -1]);
}

#[test]
fn out_of_passes_the_pins_are_those_the_step_was_solved_on() {
    // One pass pins joint 1 but has none left to solve with it: the step is the free one clipped,
    // and the pins stay as they were, the next cycle's warm start agreeing with this goal.
    let p = synthetic(Task::new(0.1, 1e-3, 0.0, 0.0, 0.0, 0.0), [0.01; 7]);
    let mut act = [0; 7];
    let s = solve(&p.j, &p.e, (&p.lo, &p.hi), None, &mut act, DAMPING, 1).unwrap();
    let mut free = [0; 7];
    let plain = solve(&p.j, &p.e, (&p.lo, &p.hi), None, &mut free, DAMPING, 0).unwrap();
    assert!(!s.converged && s.passes == 1 && s.d == plain.d, "{s:?}");
    assert_eq!(act, [0; 7]);
}

#[test]
fn the_bias_rides_on_the_free_joints_under_its_cap_after_the_pins_are_settled() {
    let p = synthetic(Task::new(0.1, 0.0, 0.0, 0.0, 0.0, 0.0), [0.01; 7]);
    let bias = Bias {
        rate: SVector::from([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        cap: 5e-4,
    };
    let (mut plain, mut biased) = ([0; 7], [0; 7]);
    let d = solved(&p, &mut plain);
    let s = solve(
        &p.j,
        &p.e,
        (&p.lo, &p.hi),
        Some(bias),
        &mut biased,
        DAMPING,
        8,
    )
    .unwrap();
    assert_eq!(plain, biased, "the bias changes no pin");
    assert_eq!(s.d[0], 0.01, "the pinned joint stays on its bound");
    let pull = s.d - d;
    assert!(pull.amax() <= 5e-4 + 1e-15 && pull.amax() > 1e-4, "{pull}");
}

#[test]
fn the_stall_flag_sets_above_on_and_clears_after_twenty_quiet_cycles() {
    let mut stall = Stall::default();
    stall.record(2e-5, 1.0);
    assert!(
        !stall.stalled,
        "at 2e-5 exactly, and velocity pressure never sets it"
    );
    stall.record(2.01e-5, 0.0);
    assert!(stall.stalled);
    // Any push at all holds the flag and restarts the quiet count: position first lowers the
    // pressure that set it, so only a pin that stops pushing clears it.
    for _ in 0..19 {
        stall.record(1e-9, 0.0);
    }
    stall.record(1e-8, 0.0);
    assert!(stall.stalled);
    for k in 1..=20 {
        stall.record(0.0, 0.0);
        assert_eq!(stall.stalled, k < 20, "quiet cycle {k}");
    }
    // Pressure along each unit column, into its bound only, a velocity pin's kept apart: the
    // same whatever the column's length.
    let mut j = Jacobian::zeros();
    j[(0, 0)] = 20.0;
    j[(0, 1)] = 1.0;
    j[(1, 2)] = 1.0;
    let r = Task::new(1e-4, 0.0, 0.0, 0.0, 0.0, 0.0);
    stall.update(&j, &r, &[2, 1, 0, 0, 0, 0, 0], 0.05);
    assert!((stall.velocity - 1e-4).abs() < 1e-18 && (stall.position - 1e-4).abs() < 1e-18);
    assert!(stall.stalled);
    // What joint 2 withholds, joint 3 cannot make up: the wall is along x, none in rotation.
    let joint_2 = [0, 1, 0, 0, 0, 0, 0];
    let x = Wall {
        normal: nalgebra::Vector3::x(),
        joints: joint_2,
    };
    assert_eq!(stall.withheld, [Some(x), None]);
    // Joint 1 free makes up all but λ²/(400 + λ²) of it: pushed, but no wall. However little
    // joint 2 pushes, what joint 3 cannot make up is its wall.
    stall.update(&j, &r, &[0, 1, 0, 0, 0, 0, 0], 0.05);
    assert!(stall.position > 0.0 && stall.withheld == [None; 2]);
    stall.update(&j, &(r * 1e-6), &[2, 1, 0, 0, 0, 0, 0], 0.05);
    assert_eq!(stall.withheld, [Some(x), None]);
    stall.update(&j, &r, &[2, -1, 0, 0, 0, 0, 0], 0.05);
    assert_eq!((stall.position, stall.withheld), (0.0, [None; 2]));
}
