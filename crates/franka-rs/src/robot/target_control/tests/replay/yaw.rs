//! The eleven yaw stalls of the three `wrist-*` recordings (`#[ignore]`, `REPLAY_DIR`): (i) the loop as it
//! ran reproduces the recorded goals over each window; (ii) the guarded solver, open loop from the
//! recorded goal at the window's start on the commander's targets (the Franka Hand, the session's
//! first configuration as the posture), settled on the window's last target, holds the position a
//! lexicographic solve reaches and gives up no more *weighted orientation error* than it -- what
//! the guard minimises once the position is held, the tool yaw and the roll and pitch it is paid
//! for being reported beside it; (iii) with the target held and the blend steady, the solve never
//! gives up the quantity it is minimising, and the goal does not walk away from the posture while
//! the task is stalled; (iv) over the whole session, commanded, the guarded loop's joint goals do
//! not chatter near the limits.

use std::sync::Arc;

use super::super::super::ik::{Ik, IkOptions, POSTURE_LIMIT_DISTANCE};
use super::super::super::position::{JointLimits, VelocityLimit};
use super::super::super::rotation::{angle_between, from_quaternion, log, pose_from};
use super::super::super::rotation::{rotation_of, translation_of};
use super::super::super::*;
use super::super::plant::as_run::Restart;
use super::super::plant::HAND;
use super::chatter;
use super::csv::{Arm, Session};
use super::envelope::recorded_on_the_fer;
use super::sync::{check_oracle, reproduced};
use super::Config;
use crate::model::Model;
use crate::rate_limiting::fer::JOINT_POSITION_LIMITS;
use crate::rate_limiting::DELTA_T;

const MARGIN: f64 = 0.05;

/// A stall: the window, s; the spec's bound on the yaw shortfall, rad, and on the position error,
/// mm, at its end, from a weighted least-squares replay at a 0.02 margin; and a reference
/// weighted solve's own values there. Both are printed for continuity, neither is judged: the
/// bound is [`feasible`]'s lexicographic optimum for the window's own target, widened by
/// [`WEIGHTED_SLACK_MM`] and [`POSITION_SLACK`]. An optimum the run beats is reported as a bound
/// that says nothing.
struct Window {
    t: (f64, f64),
    spec: (f64, f64),
    reference: (f64, f64),
}

const fn window(t: (f64, f64), spec: (f64, f64), reference: (f64, f64)) -> Window {
    Window { t, spec, reference }
}

/// How far past the position-first optimum a window may end: mm of position, and mm of *weighted*
/// orientation error (`rotation_weight` × rad, the guard's own units, so 0.2 mm is 2 mrad).
const POSITION_SLACK: f64 = 2.0;
const WEIGHTED_SLACK_MM: f64 = 0.2;
/// Cycles the window's last target is held for after it, so the run is compared to a static
/// optimum from rest and not in the middle of tracking a target that was still moving.
const SETTLE_CYCLES: usize = 2000;
/// How far a free joint may travel from a seed when the optimum is computed: a
/// tenth of a radian is what a joint covers in 0.1 s at the speeds these windows run at, so the
/// optimum is one the run could still have reached, not one across the workspace.
const LOCAL: f64 = 0.1;
/// How much position error the reference's orientation stage may carry before its step is
/// rejected, m; it is closed again at the end. Two orders under the position slack (ii) allows.
const POSITION_BAND: f64 = 1e-5;

const RECORDINGS: [(&str, &[Window]); 3] = [
    (
        "wrist-a",
        &[
            window((23.66, 24.13), (0.01, 2.0), (0.000, 0.5)),
            window((39.46, 40.66), (0.09, 3.0), (0.076, 1.3)),
            window((41.33, 41.67), (0.08, 3.0), (0.062, 1.1)),
            window((55.90, 56.39), (0.01, 2.0), (0.000, 0.8)),
        ],
    ),
    (
        "wrist-b",
        &[
            window((57.50, 58.59), (0.01, 2.0), (0.001, 0.7)),
            window((65.00, 66.02), (0.01, 2.0), (0.000, 1.0)),
            window((69.46, 69.82), (0.01, 2.0), (0.000, 0.3)),
            window((90.63, 92.51), (0.05, 2.0), (0.035, 0.8)),
            window((92.70, 93.53), (0.11, 4.0), (0.094, 2.3)),
        ],
    ),
    (
        "wrist-b-2",
        &[
            window((78.35, 79.12), (0.05, 3.0), (0.038, 1.2)),
            window((80.62, 81.47), (0.15, 3.0), (0.137, 1.2)),
        ],
    ),
];

/// Which recorded target is in force at `cycle`: the commander stamps a new one every few
/// cycles, so a block of cycles sharing an index is a target that literally holds still.
fn target_index(session: &Session, cycle: usize) -> usize {
    session.targets.partition_point(|t| t.0 <= cycle).max(1) - 1
}

/// The recorded target in force at `cycle` as a column-major pose.
fn target_at(session: &Session, cycle: usize) -> [f64; 16] {
    let i = target_index(session, cycle);
    let t = session.targets[i].1;
    pose_from(
        &from_quaternion(&[t[3], t[4], t[5], t[6]]),
        &[t[0], t[1], t[2]],
    )
}

/// A pose against a target, as (ii) judges it: the position error, m; the whole orientation
/// error, rad -- what the guard's weighted solve minimises once the position is held; and, for
/// information, how that orientation error splits into the tool yaw the operator asked for and
/// the roll and pitch left over.
#[derive(Clone, Copy)]
struct Measured {
    position: f64,
    orientation: f64,
    yaw: f64,
}

impl Measured {
    fn roll_pitch(&self) -> f64 {
        (self.orientation.powi(2) - self.yaw.powi(2))
            .max(0.0)
            .sqrt()
    }

    fn show(&self) -> String {
        format!(
            "{:.2} mm / {:.1} mrad ({:.4} rad yaw, {:.1} mrad roll-pitch)",
            self.position * 1e3,
            self.orientation * 1e3,
            self.yaw,
            self.roll_pitch() * 1e3
        )
    }
}

/// Which joints of `q` sit on the guard's box, -1 lower, 1 upper, 0 free: the active set of a
/// configuration, which is what tells two nearby stationary points apart.
fn contacts(q: &[f64; 7]) -> [i8; 7] {
    let (lower, upper) = JOINT_POSITION_LIMITS;
    std::array::from_fn(|i| {
        let on = |bound: f64| (q[i] - bound).abs() < 1e-6;
        i8::from(on(upper[i] - MARGIN)) - i8::from(on(lower[i] + MARGIN))
    })
}

/// `pose` against `target`.
fn measure(pose: &[f64; 16], target: &[f64; 16]) -> Measured {
    let (yaw, position) = shortfall(pose, target);
    Measured {
        position,
        orientation: angle_between(&rotation_of(pose), &rotation_of(target)),
        yaw,
    }
}

/// The tool-yaw shortfall, rad (the body-z part of `log(R(q)ᵀ R_target)`), and the position
/// error, m, of `pose` against `target`.
fn shortfall(pose: &[f64; 16], target: &[f64; 16]) -> (f64, f64) {
    let yaw = log(&(rotation_of(pose).transpose() * rotation_of(target)))[2].abs();
    let (p, t) = (translation_of(pose), translation_of(target));
    (
        yaw,
        (0..3).map(|i| (p[i] - t[i]).powi(2)).sum::<f64>().sqrt(),
    )
}

/// The lexicographic optimum of `target` over three seeds -- where the run settled, where the
/// window started and where it ended -- the best of the three, so that a run which settles in a
/// poor basin is not bounded by its own basin, and one that leaves both of the window's
/// configurations behind is still held to a local optimum. Better is position first: closer to
/// the target, and only where two are as close, the smaller orientation error, which is what the
/// guard minimises there. A seed whose box cannot reach the target is dropped that way, however
/// little it leaves. Returns the best and the descent from where the run settled, its own basin.
fn feasible(
    case: &str,
    model: &Model,
    seeds: [[f64; 7]; 3],
    target: &[f64; 16],
) -> (Reference, Reference) {
    let each = seeds.map(|seed| from_seed(model, &seed, target));
    println!(
        "{case}: optimum from where the run settled {}, from the window's start {}, from its \
         end {}",
        each[0].best.show(),
        each[1].best.show(),
        each[2].best.show()
    );
    let nearer = |x: &Reference, y: &Reference| x.best.position + 1e-4 < y.best.position;
    let better = |x: &Reference, y: &Reference| {
        nearer(x, y) || (!nearer(y, x) && x.best.orientation < y.best.orientation)
    };
    let mut best = each[0];
    for candidate in &each[1..] {
        if better(candidate, &best) {
            best = *candidate;
        }
    }
    (best, each[0])
}

/// What a lexicographic solve reaches from a seed: `best` minimises the whole orientation error
/// with the position held, which is the guard's own objective and what (ii) judges; `least_yaw`
/// minimises the tool yaw instead, reported to show what a yaw minimised on its own pays for in
/// roll and pitch.
#[derive(Clone, Copy)]
struct Reference {
    best: Measured,
    /// Where `best` is, so a window that misses it can say how far away it lies.
    at: [f64; 7],
    least_yaw: Measured,
}

/// The lexicographic optimum of `target` reachable from `seed`, inside the joint limits less the
/// margin. A projected Gauss-Newton on the translation alone, then a projected gradient in its
/// nullspace with the translation restored after every step, so the second stage never pays in
/// position: once down the whole orientation error, as the guard's weighted solve trades it, and
/// once down the yaw alone. Independent of the solver under test, which runs one cycle at a time
/// inside a velocity box.
fn from_seed(model: &Model, seed: &[f64; 7], target: &[f64; 16]) -> Reference {
    let ee_t_k = RobotState::default().EE_T_K;
    let (lower, upper) = JOINT_POSITION_LIMITS;
    // The guard's own box -- the limits less the margin -- and no joint further than LOCAL from
    // the seed: an optimum a window's worth of cycles could reach, not one an elbow may walk half
    // the workspace for. The seed itself is always in it: these recordings ran unguarded, so a
    // window can start with a joint already inside its margin, and the guard lets such a joint
    // stay where it is rather than pushing it out.
    let clamp = |q: [f64; 7]| -> [f64; 7] {
        std::array::from_fn(|i| {
            let (lo, hi) = (
                (seed[i] - LOCAL).max(lower[i] + MARGIN).min(seed[i]),
                (seed[i] + LOCAL).min(upper[i] - MARGIN).max(seed[i]),
            );
            q[i].clamp(lo, hi)
        })
    };
    let pose =
        |q: &[f64; 7]| model.pose_q(crate::model::Frame::EndEffector, q, &HAND.f_t_ee, &ee_t_k);
    let jacobian = |q: &[f64; 7]| {
        nalgebra::SMatrix::<f64, 6, 7>::from_column_slice(&model.zero_jacobian_q(
            crate::model::Frame::EndEffector,
            q,
            &HAND.f_t_ee,
            &ee_t_k,
        ))
    };
    let error = |q: &[f64; 7]| {
        let p = translation_of(&pose(q));
        nalgebra::Vector3::from_fn(|i, _| translation_of(target)[i] - p[i])
    };
    // The translation, damped least squares with a tiny damping, clipped into the box.
    let mut q = clamp(*seed);
    let translate = |q: &mut [f64; 7], steps: usize| {
        for _ in 0..steps {
            let j = jacobian(q);
            let jp = j.fixed_rows::<3>(0);
            let a = jp * jp.transpose() + nalgebra::Matrix3::identity() * 1e-12;
            let Some(solved) = a.cholesky().map(|c| c.solve(&error(q))) else {
                return;
            };
            let step = jp.transpose() * solved;
            *q = clamp(std::array::from_fn(|i| q[i] + step[i]));
        }
    };
    translate(&mut q, 200);
    let start = q;
    let yaw = |q: &[f64; 7]| log(&(rotation_of(&pose(q)).transpose() * rotation_of(target)))[2];
    let turn = |q: &[f64; 7]| {
        nalgebra::Vector3::from(log(
            &(rotation_of(target) * rotation_of(&pose(q)).transpose())
        ))
    };
    // The tool's z in the base frame: the axis the yaw is read along, so `yaw = ⟨turn, axis⟩`.
    let axis = |q: &[f64; 7]| rotation_of(&pose(q)).column(2).into_owned();
    let at = |q: &[f64; 7]| measure(&pose(q), target);
    let (mut best, mut least_yaw, mut best_q) = (at(&start), at(&start), start);
    // Lexicographic, with a band: a candidate counts while it holds the position the first stage
    // reached to within POSITION_BAND. Held to the last micrometre instead, a descent seeded at a
    // configuration the guard had already settled on could not take its first step.
    let floor = best.position + POSITION_BAND;
    for down_the_yaw in [false, true] {
        let mut q = start;
        for _ in 0..2000 {
            let j = jacobian(&q);
            let jr = j.fixed_rows::<3>(3).transpose();
            let gradient = if down_the_yaw {
                jr * axis(&q) * yaw(&q).signum()
            } else {
                jr * turn(&q)
            };
            let jp = j.fixed_rows::<3>(0);
            let a = jp * jp.transpose() + nalgebra::Matrix3::identity() * 1e-12;
            let Some(pinv) = a.cholesky().map(|c| jp.transpose() * c.inverse()) else {
                break;
            };
            let null = nalgebra::SMatrix::<f64, 7, 7>::identity() - pinv * jp;
            let step = null * gradient * 1e-2;
            q = clamp(std::array::from_fn(|i| q[i] + step[i]));
            translate(&mut q, 5);
            let here = at(&q);
            if here.position > floor {
                continue;
            }
            if here.orientation < best.orientation {
                (best, best_q) = (here, q);
            }
            if here.yaw < least_yaw.yaw {
                least_yaw = here;
            }
        }
    }
    // The band the descent was allowed, given back: the point it chose, with its position closed
    // again, is what the window is judged against.
    translate(&mut best_q, 200);
    Reference {
        best: at(&best_q),
        at: best_q,
        least_yaw,
    }
}

/// One open-loop cycle of the solver: the goal, its weighted residual, the pins, the backstop's
/// cut and the position-first blend. The last three say why a residual moved.
struct Cycle {
    q: [f64; 7],
    residual: f64,
    pinned: [i8; 7],
    cap: f64,
    blend: f64,
}

/// The solver `ik` open loop over `cycles`.
fn open_loop(ik: &mut Ik, session: &Session, cycles: std::ops::Range<usize>) -> Vec<Cycle> {
    cycles
        .map(|k| {
            let (q, residual) = ik.step(&target_at(session, k), &session.q[0], DELTA_T);
            Cycle {
                q,
                residual,
                pinned: ik.pinned(),
                cap: ik.cap_scale(),
                blend: ik.priority(),
            }
        })
        .collect()
}

/// The growth (iii) tolerates from one cycle to the next before it is a convergence defect,
/// metres. Numerical, not physical: a cycle is up to three damped solves whose active set may
/// change, the posture bias is only to first order in the free joints' nullspace, and the run
/// compares two floating-point forward kinematics. A tenth of a micrometre is four orders under
/// the per-cycle progress these windows make and under the solver's own `tolerance`.
const GROWTH_FLOOR: f64 = 1e-7;
/// Over 100 ms in which the task closed less than [`STALLED`] metres of its weighted error, the
/// goal may leave the posture by this much, rad: the 1 mrad per 100 ms the elbow-drift unit test
/// holds the guard to. Drift is a walk, so it is measured over the 100 ms the operator would see
/// it in, not per cycle, where a solve converging along a near-singular direction moves the arm
/// a milliradian for microradians of task -- slow, but not drift.
const DRIFT: f64 = 1e-3;
/// What counts as the task having closed nothing over those 100 ms, metres of weighted error: a
/// hundredth of what a solve that is converging at all closes in that time.
const STALLED: f64 = 1e-5;

/// (iii) on a guarded open-loop run from `start`, from one cycle to the next *with the commanded
/// target unchanged* -- like for like, so nothing here is the target moving under the goal:
/// - whatever the blend is trading, no cycle may give up the weighted residual and the position
///   error at once;
/// - with the blend equal on both cycles, so the position-first ramp is not itself moving the
///   objective, the quantity in force must not grow: the weighted residual under the weighted
///   solve (blend 0), the position error once position comes first (blend 1), and there the
///   orientation too once the position error has settled. The residual may grow at blend 1:
///   that is what buying position with orientation looks like.
///
/// Drift is a walk, so it is judged over 100 ms: where the target holds nearly still and the task
/// closed less than [`STALLED`], the goal may leave the posture by at most [`DRIFT`], and joints 1
/// and 3 may move once the yaw is under 0.01 rad only while the orientation error falls with them
/// -- the elbow closing roll and pitch is the task, not drift. Every failure, and a summary.
fn stationary(
    session: &Session,
    start: usize,
    run: &[Cycle],
    model: &Model,
) -> (Vec<String>, String) {
    let (lower, upper) = JOINT_POSITION_LIMITS;
    let inset = MARGIN + POSTURE_LIMIT_DISTANCE;
    let post: [f64; 7] =
        std::array::from_fn(|i| session.q[0][i].clamp(lower[i] + inset, upper[i] - inset));
    let distance = |q: &[f64; 7]| (0..7).map(|i| (q[i] - post[i]).powi(2)).sum::<f64>().sqrt();
    let ee_t_k = RobotState::default().EE_T_K;
    let pose =
        |q: &[f64; 7]| model.pose_q(crate::model::Frame::EndEffector, q, &HAND.f_t_ee, &ee_t_k);
    // Per cycle, against the target in force on it.
    let errors: Vec<Measured> = (0..run.len())
        .map(|k| measure(&pose(&run[k].q), &target_at(session, start + k)))
        .collect();
    let mut failures = Vec::new();
    // The worst growth of each kind over the window, and where; the posture over a whole run of
    // cycles without progress, since drift is a walk and one cycle is noise.
    let (mut grew_r, mut grew_p, mut grew_o, mut gave_up) =
        ((0.0, 0), (0.0, 0), (0.0, 0), (0.0, 0));
    let (mut judged, mut steady) = (0, 0);
    for k in 1..run.len() {
        if target_index(session, start + k) != target_index(session, start + k - 1) {
            continue;
        }
        judged += 1;
        let worse = |(most, at): (f64, usize), by: f64| {
            if by > most {
                (by, start + k)
            } else {
                (most, at)
            }
        };
        let closed = run[k - 1].residual - run[k].residual;
        let closed_position = errors[k - 1].position - errors[k].position;
        // Whatever the blend is trading, a cycle may not give up both at once.
        gave_up = worse(gave_up, (-closed).min(-closed_position));
        // The two ends of the blend, where the objective is exactly one thing and the ramp is
        // not moving it: only there is a single quantity the solve is answerable for.
        if run[k].blend != run[k - 1].blend {
            continue;
        }
        steady += 1;
        if run[k].blend == 0.0 {
            grew_r = worse(grew_r, -closed);
        } else if run[k].blend == 1.0 {
            grew_p = worse(grew_p, -closed_position);
            if closed_position <= GROWTH_FLOOR {
                grew_o = worse(grew_o, errors[k].orientation - errors[k - 1].orientation);
            }
        }
    }
    let context = |at: usize| {
        let k = at - start;
        format!(
            "[blend {:.2}, cap {:.4}, pins {:?}, residual {:.9e} -> {:.9e}]",
            run[k].blend,
            run[k].cap,
            run[k].pinned,
            run[k - 1].residual,
            run[k].residual,
        )
    };
    for (what, (by, at), unit) in [
        (
            "the weighted residual and the position error both grew",
            gave_up,
            "mm",
        ),
        (
            "the weighted residual grew under the weighted solve",
            grew_r,
            "mm",
        ),
        ("the position error grew with position first", grew_p, "mm"),
        (
            "the orientation error grew with position first and the position error settled",
            grew_o,
            "mrad",
        ),
    ] {
        if by > GROWTH_FLOOR {
            failures.push(format!(
                "cycle {at}: {what} by {:.4} {unit} in a cycle {}",
                by * 1e3,
                context(at)
            ));
        }
    }
    let (mut walk, mut stalled, mut worst) = ((0.0, 0, 0.0), 0, (0.0, 0.0));
    for block in (0..run.len()).step_by(100).filter(|b| b + 100 <= run.len()) {
        let first = target_at(session, start + block);
        let still = (block..block + 100).all(|k| {
            let t = target_at(session, start + k);
            let moved = shortfall(&t, &first).1;
            moved < 5e-4 && angle_between(&rotation_of(&t), &rotation_of(&first)) < 5e-3
        });
        // Drift: 100 ms in which the task closed nothing and the goal still left the posture.
        let closed = run[block].residual - run[block + 99].residual;
        let left = distance(&run[block + 99].q) - distance(&run[block].q);
        if still && left > worst.0 {
            worst = (left, closed);
        }
        if still && closed < STALLED {
            stalled += 1;
            if left > walk.0 {
                walk = (left, start + block, closed);
            }
        }
        // Joints 1 and 3 once the yaw is done: moving with the orientation error is the elbow
        // closing what is left of roll and pitch, at its velocity bound; moving without it is
        // drift.
        if still && errors[block].yaw < 0.01 {
            let (a, b) = (run[block].q, run[block + 99].q);
            let drift = (a[0] - b[0]).abs().max((a[2] - b[2]).abs());
            let closed = errors[block].orientation - errors[block + 99].orientation;
            if drift > 5e-3 && closed < 1e-3 {
                failures.push(format!(
                    "cycle {}: joints 1 and 3 move {drift:.4} rad while the orientation error \
                     closes {closed:.4} rad",
                    start + block
                ));
            }
        }
    }
    if walk.0 > DRIFT {
        let k = walk.1 - start;
        let moved: [f64; 7] = std::array::from_fn(|i| (run[k + 99].q[i] - run[k].q[i]) * 1e3);
        failures.push(format!(
            "cycle {}: over 100 ms that closed {:.4} mm of the weighted error the goal left the \
             posture by {:.4} mrad of the {DRIFT} rad allowed; joints moved {moved:.4?} mrad {}",
            walk.1,
            walk.2 * 1e3,
            walk.0 * 1e3,
            context(walk.1 + 99)
        ));
    }
    let summary = format!(
        "(iii) judged {judged} held-target cycles of {} ({steady} with the blend steady); worst \
         100 ms walk from the posture {:.4} mrad while the task closed {:.4} mm, and over the \
         {stalled} stalled blocks {:.4} mrad at cycle {}",
        run.len(),
        worst.0 * 1e3,
        worst.1 * 1e3,
        walk.0 * 1e3,
        walk.1
    );
    (failures, summary)
}

#[test]
#[ignore = "REPLAY_DIR, release"]
fn the_yaw_stalls_of_the_wrist_recordings() {
    let model = Arc::new(Model::native_fer());
    let ee_t_k = RobotState::default().EE_T_K;
    let limits = JointLimits {
        position: JOINT_POSITION_LIMITS,
        margin: MARGIN,
        fraction: 0.7,
        velocity: VelocityLimit::of(FciVersion::V5),
    };
    let mut failures = Vec::new();
    for (name, windows) in RECORDINGS {
        let config = Config::wrist_limit();
        let Some((session, (targets, goals))) =
            reproduced(name, &Arm::fer(), config, Restart::End, &mut failures)
        else {
            return;
        };
        chatter::check(
            name,
            &session,
            &targets,
            (config, Restart::End),
            &mut failures,
        );
        let spans: Vec<(f64, f64)> = windows.iter().map(|w| w.t).collect();
        recorded_on_the_fer(name, &session, &spans, &mut failures);
        for w in windows {
            let at = |t: f64| session.t.partition_point(|x| *x < t);
            let (i0, i1) = (at(w.t.0), at(w.t.1));
            let case = format!("{name} {:.2}-{:.2}", w.t.0, w.t.1);
            // (i) the loop as it ran over the window.
            let (ours, recorded) = (&goals[i0..i1], &session.q_goal[i0..i1]);
            check_oracle(&case, ours, recorded, &mut failures);
            // (ii) the guarded solver, and the solver as it ran, open loop.
            let q0 = session.q_goal[i0];
            let mut guarded = Ik::new(
                Arc::clone(&model),
                IkOptions::default(),
                limits,
                q0,
                HAND.f_t_ee,
                ee_t_k,
            );
            let cap = max_joint_velocity(FciVersion::V5).map(|v| 0.7 * v);
            let mut before = Ik::as_run(
                Arc::clone(&model),
                IkOptions::default(),
                JOINT_POSITION_LIMITS,
                cap,
                q0,
                HAND.f_t_ee,
                ee_t_k,
            );
            let run = open_loop(&mut guarded, &session, i0..i1);
            let clamp = open_loop(&mut before, &session, i0..i1);
            let target = target_at(&session, i1 - 1);
            let last = run[run.len() - 1].q;
            let ended = measure(&guarded.pose_of(&last), &target);
            let as_run = measure(&before.pose_of(&clamp[clamp.len() - 1].q), &target);
            let elbow = |r: &[Cycle], j: usize| {
                r.iter().map(|x| x.q[j]).fold(f64::NEG_INFINITY, f64::max)
                    - r.iter().map(|x| x.q[j]).fold(f64::INFINITY, f64::min)
            };
            // What the guard converges to when the window's last target is held: the window's own
            // end still carries the lag of tracking a target that moved through it.
            let mut held = last;
            for _ in 0..SETTLE_CYCLES {
                held = guarded.step(&target, &session.q[0], DELTA_T).0;
            }
            let settled = measure(&guarded.pose_of(&held), &target);
            let (reference, own) = feasible(&case, &model, [held, q0, last], &target);
            let weight = IkOptions::default().rotation_weight;
            let over = weight * (settled.orientation - reference.best.orientation);
            println!(
                "{case}: guarded {}, joints 1/3 range {:.3}/{:.3} rad; as run {}, range \
                 {:.3}/{:.3} rad; lexicographic optimum {}, least yaw there {:.4} rad ({} \
                 whole); (ii) on the run settled at that target {}: weighted orientation over \
                 the optimum by {:.3} mm of {WEIGHTED_SLACK_MM} allowed, position over by \
                 {:.2} mm of {POSITION_SLACK}; spec {:?}, weighted reference {:?}",
                ended.show(),
                elbow(&run, 0),
                elbow(&run, 2),
                as_run.show(),
                elbow(&clamp, 0),
                elbow(&clamp, 2),
                reference.best.show(),
                reference.least_yaw.yaw,
                reference.least_yaw.show(),
                settled.show(),
                over * 1e3,
                (settled.position - reference.best.position) * 1e3,
                w.spec,
                w.reference
            );
            if over * 1e3 > WEIGHTED_SLACK_MM
                || (settled.position - reference.best.position) * 1e3 > POSITION_SLACK
            {
                // A run that sits on the optimum of the basin it settled in, with no joint on
                // the box at either point, has converged: the better optimum is another
                // stationary point of a trade that has several, reached from another seed.
                let own_basin = own.best.orientation >= settled.orientation - 1e-6;
                failures.push(format!(
                    "{case}: settled {} is short of the lexicographic optimum {} by {:.3} mm \
                     weighted orientation ({WEIGHTED_SLACK_MM} allowed) / {:.2} mm position \
                     ({POSITION_SLACK} allowed); the optimum of the basin it settled in is {}, \
                     and the better optimum lies {:.3?} rad away, on the box at {:?} where the \
                     run settled on it at {:?}{}",
                    settled.show(),
                    reference.best.show(),
                    over * 1e3,
                    (settled.position - reference.best.position) * 1e3,
                    own.best.show(),
                    std::array::from_fn::<f64, 7, _>(|i| reference.at[i] - held[i]),
                    contacts(&reference.at),
                    contacts(&held),
                    if own_basin {
                        " -- the run is at the optimum of the basin it settled in; open item (a \
                         trade with more than one stationary point), not a convergence defect"
                    } else {
                        ""
                    }
                ));
            }
            // An optimum the run beats is no bound: (ii) could not fail however the solver
            // traded. A milliradian of it is the projected gradient's own convergence.
            if reference.best.orientation > settled.orientation + 1e-3 {
                failures.push(format!(
                    "{case}: optimum not informative, the settled run's orientation error {:.1} \
                     mrad is under the computed optimum's {:.1} mrad",
                    settled.orientation * 1e3,
                    reference.best.orientation * 1e3
                ));
            }
            // (iii) no drift while the target holds still.
            let (drift, summary) = stationary(&session, i0, &run, &model);
            println!("{case}: {summary}");
            failures.extend(drift.into_iter().map(|f| format!("{case}: {f}")));
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
