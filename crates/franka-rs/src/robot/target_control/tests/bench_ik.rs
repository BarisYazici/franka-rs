//! The IK solver's cycle cost, guarded against as run (`#[ignore]`, release, a pinned core):
//! one [`Ik::step`] timed per cycle on the FER with the Franka Hand, with its active-set passes,
//! pins and fallbacks, and the allocations it makes, counted by this test binary's allocator.
//!
//! ```text
//! taskset -c 3 cargo test --release -p franka-rs --lib bench_ik_box -- --ignored --nocapture
//! ```
//!
//! Regimes: free space (a 5 cm, 0.1 rad sway at the ready pose), a fold past joint 4's margin
//! (held on joints 2 and 4: the redundancy walks the arm onto joint 2's limit), the wrist's yaw
//! stall (joints 5 and 7 on their upper limits, a recorded configuration and target), random
//! configurations with 1 to 4 joints within 0.1 rad of a limit, timed per number of pins too.
//! `BENCH_IK_WINDOWS` adds windows of recorded CSV sessions ([`csv::load`]), `;`-separated
//! `path,t0,t1` in s, or `path` alone for its last 2 s: from the logged goal at `t0`, each
//! recorded target fed as the pose.
//! `BENCH_IK_CYCLES` (100 000) timed cycles per regime, `BENCH_IK_RUNS` (5) runs.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::f64::consts::TAU;
use std::hint::black_box;
use std::sync::Arc;
use std::time::Instant;

use super::super::ik::{Ik, IkOptions};
use super::super::position::{JointLimits, VelocityLimit, DEFAULT_JOINT_POSITION_MARGIN};
use super::super::rotation::{exp, from_quaternion, pose_from, rotation_of, translation_of};
use super::plant::HAND;
use super::replay::csv;
use super::READY;
use crate::model::{Frame, Model};
use crate::rate_limiting::{fer, DELTA_T};
use crate::wire::robot::codec::FciVersion;

struct CountingAllocator;

thread_local! {
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
}

fn count() {
    let _ = ALLOCATIONS.try_with(|c| c.set(c.get() + 1));
}

fn allocations() -> u64 {
    ALLOCATIONS.with(Cell::get)
}

// SAFETY: every method defers to `System` after bumping a thread-local counter that neither
// allocates nor panics (`try_with` on a `const` `Cell` with no destructor).
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

type Limits = ([f64; 7], [f64; 7]);

#[rustfmt::skip]
const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
const FRACTION: f64 = 0.7;
const WARMUP: usize = 1000;
/// Passes a cycle can take: `iterations × MAX_ACTIVE_SET_PASSES`, twice under position first.
const PASS_BINS: usize = 73;

/// The yaw stall at its start: the logged joint goal, joints 5 and 7 at the as-run clamp 0.02
/// under their upper limits, and the commander's target 0.136 rad of tool yaw ahead.
const YAW_STALL_Q: [f64; 7] = [
    -1.271891, 1.256394, 1.424762, -1.517014, 2.8773, 2.683489, 2.8773,
];
const YAW_STALL_TARGET: [f64; 7] = [
    0.792527, -0.242192, 0.387927, 0.678433, -0.099632, 0.725248, 0.061782,
];

/// One solver run: from `q0` toward `poses` in turn, one per cycle.
struct Episode {
    q0: [f64; 7],
    posture: [f64; 7],
    limits: Limits,
    poses: Vec<[f64; 16]>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Solver {
    Guarded,
    AsRun,
}

fn build(solver: Solver, model: &Arc<Model>, e: &Episode) -> Ik {
    let v = fer::MAX_JOINT_VELOCITY;
    let options = IkOptions::default();
    match solver {
        Solver::Guarded => {
            let limits = JointLimits {
                position: e.limits,
                margin: DEFAULT_JOINT_POSITION_MARGIN,
                fraction: FRACTION,
                velocity: VelocityLimit::of(FciVersion::V5),
            };
            Ik::new(
                Arc::clone(model),
                options,
                limits,
                e.q0,
                HAND.f_t_ee,
                IDENTITY,
            )
        }
        Solver::AsRun => {
            let cap = v.map(|v| v * FRACTION);
            Ik::as_run(
                Arc::clone(model),
                options,
                e.limits,
                cap,
                e.q0,
                HAND.f_t_ee,
                IDENTITY,
            )
        }
    }
}

fn fk(model: &Model, q: &[f64; 7]) -> [f64; 16] {
    model.pose_q(Frame::EndEffector, q, &HAND.f_t_ee, &IDENTITY)
}

/// `pose` moved by `dp` and turned by the base-frame rotation vector `dr`.
fn moved(pose: &[f64; 16], dp: [f64; 3], dr: [f64; 3]) -> [f64; 16] {
    let p = translation_of(pose);
    let p = std::array::from_fn(|i| p[i] + dp[i]);
    pose_from(&(exp(&dr) * rotation_of(pose)), &p)
}

/// `[x, y, z, qx, qy, qz, qw]` as a pose.
fn target_pose(t: &[f64; 7]) -> [f64; 16] {
    pose_from(
        &from_quaternion(&[t[3], t[4], t[5], t[6]]),
        &[t[0], t[1], t[2]],
    )
}

/// `pose` swayed by `amplitude` m along x at `hz` over `cycles`.
fn swaying(pose: &[f64; 16], amplitude: f64, hz: f64, cycles: usize) -> Vec<[f64; 16]> {
    (0..cycles)
        .map(|k| {
            let s = (TAU * hz * k as f64 * DELTA_T).sin();
            moved(pose, [amplitude * s, 0.0, 0.0], [0.0; 3])
        })
        .collect()
}

fn free(model: &Model) -> Vec<Episode> {
    let ready = fk(model, &READY);
    let poses = (0..2000)
        .map(|k| {
            let s = (TAU * 0.5 * k as f64 * DELTA_T).sin();
            moved(&ready, [0.05 * s, 0.0, 0.0], [0.0, 0.0, 0.1 * s])
        })
        .collect();
    let limits = fer::JOINT_POSITION_LIMITS;
    vec![Episode {
        q0: READY,
        posture: READY,
        limits,
        poses,
    }]
}

/// Joint 4's lower limit 0.3 rad under the ready pose, a pose folding it 0.4 rad swayed by 5 mm,
/// from where the guard settles: joints 2 and 4 on their lower margins.
fn fold(model: &Arc<Model>) -> Vec<Episode> {
    let mut limits = fer::JOINT_POSITION_LIMITS;
    limits.0[3] = READY[3] - 0.3;
    let mut folded = READY;
    folded[3] -= 0.4;
    let pose = fk(model, &folded);
    let mut settle = Episode {
        q0: READY,
        posture: READY,
        limits,
        poses: vec![pose],
    };
    let mut ik = build(Solver::Guarded, model, &settle);
    settle.q0 = (0..1000).fold(READY, |_, _| ik.step(&pose, &READY, DELTA_T).0);
    settle.poses = swaying(&pose, 0.005, 1.0, 2000);
    vec![settle]
}

fn yaw_stall() -> Vec<Episode> {
    let pose = target_pose(&YAW_STALL_TARGET);
    vec![Episode {
        q0: YAW_STALL_Q,
        posture: YAW_STALL_Q,
        limits: fer::JOINT_POSITION_LIMITS,
        poses: swaying(&pose, 0.001, 1.0, 2000),
    }]
}

/// 200 configurations, each with 1 to 4 joints within 0.1 rad of a limit and a target 0.1 rad
/// past it on those joints, held 500 cycles.
fn random_pins(model: &Model) -> Vec<Episode> {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut random = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let limits = fer::JOINT_POSITION_LIMITS;
    (0..200)
        .map(|_| {
            let (lower, upper) = limits;
            let mut q: [f64; 7] =
                std::array::from_fn(|i| lower[i] + 0.5 + random() * (upper[i] - lower[i] - 1.0));
            let mut goal: [f64; 7] = std::array::from_fn(|i| q[i] + 0.1 * (random() - 0.5));
            let mut near = [false; 7];
            for _ in 0..1 + (random() * 4.0) as usize {
                let i = (0..7)
                    .cycle()
                    .skip((random() * 7.0) as usize)
                    .find(|&i| !near[i]);
                let i = i.expect("a free joint");
                near[i] = true;
                let side = if random() < 0.5 { -1.0 } else { 1.0 };
                let limit = if side < 0.0 { lower[i] } else { upper[i] };
                q[i] = limit - side * 0.1 * random();
                goal[i] = q[i] + side * 0.1;
            }
            Episode {
                q0: q,
                posture: q,
                limits,
                poses: vec![fk(model, &goal); 500],
            }
        })
        .collect()
}

/// A window, `path,t0,t1` or `path` for the last 2 s, of a recorded session: its name and one
/// episode from the logged goal, the targets fed as poses.
fn recorded(window: &str) -> (String, Vec<Episode>) {
    let fields: Vec<&str> = window.split(',').map(str::trim).collect();
    let session = csv::load(fields[0]);
    let end = *session.t.last().expect("a cycle");
    let (t0, t1) = match fields[1..] {
        [t0, t1] => (t0.parse().expect("t0"), t1.parse().expect("t1")),
        _ => (end - 2.0, end),
    };
    let stem = std::path::Path::new(fields[0])
        .file_stem()
        .expect("a file name");
    let name = format!("{} {t0:.2}", stem.to_string_lossy());
    let cycles: Vec<usize> = (0..session.t.len())
        .filter(|&k| session.t[k] >= t0 && session.t[k] <= t1)
        .collect();
    let target = |k: usize| {
        let at = session.targets.partition_point(|(cycle, _)| *cycle <= k);
        target_pose(&session.targets[at.max(1) - 1].1)
    };
    let q0 = session.q_goal[cycles[0]];
    let episode = Episode {
        q0,
        posture: q0,
        limits: fer::JOINT_POSITION_LIMITS,
        poses: cycles.iter().map(|&k| target(k)).collect(),
    };
    (name, vec![episode])
}

/// One regime's run under one solver.
struct Stats {
    micros: Vec<f64>,
    /// The same per number of pinned joints.
    by_pins: [Vec<f64>; 8],
    passes: [u64; PASS_BINS],
    pins: [u64; 8],
    /// Cycles each joint ended pinned.
    joints: [u64; 7],
    fallback_cycles: u64,
    fallbacks: u64,
    allocations: u64,
}

fn measure(solver: Solver, model: &Arc<Model>, episodes: &[Episode], cycles: usize) -> Stats {
    let mut stats = Stats {
        micros: Vec::with_capacity(cycles),
        by_pins: std::array::from_fn(|_| Vec::with_capacity(cycles)),
        passes: [0; PASS_BINS],
        pins: [0; 8],
        joints: [0; 7],
        fallback_cycles: 0,
        fallbacks: 0,
        allocations: 0,
    };
    let mut done = 0;
    for e in episodes.iter().cycle() {
        let mut ik = build(solver, model, e);
        for pose in &e.poses {
            let (fallbacks, allocated) = (ik.fallbacks(), allocations());
            let start = Instant::now();
            let solution = ik.step(black_box(pose), &e.posture, DELTA_T);
            let elapsed = start.elapsed();
            let allocated = allocations() - allocated;
            black_box(solution);
            if done >= WARMUP {
                let (micros, pinned) = (elapsed.as_secs_f64() * 1e6, ik.pinned());
                let pinned = pinned.map(|p| u64::from(p != 0));
                let count = pinned.iter().sum::<u64>() as usize;
                stats.micros.push(micros);
                stats.by_pins[count].push(micros);
                stats.passes[(ik.passes() as usize).min(PASS_BINS - 1)] += 1;
                stats.pins[count] += 1;
                (0..7).for_each(|i| stats.joints[i] += pinned[i]);
                let fallen = ik.fallbacks() - fallbacks;
                stats.fallbacks += fallen;
                stats.fallback_cycles += u64::from(fallen > 0);
                stats.allocations += allocated;
            }
            done += 1;
            if done == WARMUP + cycles {
                return stats;
            }
        }
    }
    unreachable!("episodes cycle forever")
}

/// min, p50, p99, p99.9, max, µs.
fn percentiles(micros: &[f64]) -> [f64; 5] {
    let mut sorted = micros.to_vec();
    sorted.sort_by(f64::total_cmp);
    let at = |p: f64| sorted[((sorted.len() - 1) as f64 * p).round() as usize];
    [at(0.0), at(0.5), at(0.99), at(0.999), at(1.0)]
}

/// The non-empty bins of `bins` as `label bin:share%` of `total`.
fn histogram(bins: &[u64], label: &str, first: usize, total: usize) -> String {
    let shares: Vec<String> = (0..bins.len())
        .filter(|&k| bins[k] > 0)
        .map(|k| {
            let share = 100.0 * bins[k] as f64 / total as f64;
            format!("{label}{}:{share:.1}%", k + first)
        })
        .collect();
    shares.join(" ")
}

fn env_usize(var: &str, default: usize) -> usize {
    std::env::var(var).map_or(default, |v| v.parse().expect(var))
}

#[test]
#[ignore]
fn bench_ik_box() {
    let cycles = env_usize("BENCH_IK_CYCLES", 100_000);
    let runs = env_usize("BENCH_IK_RUNS", 5);
    let before = allocations();
    black_box(Box::new(0u64));
    assert!(allocations() > before, "the allocator counts");
    let model = Arc::new(Model::native_fer());
    let mut regimes = vec![
        ("free space".to_string(), free(&model)),
        ("j4 fold, j2+j4".to_string(), fold(&model)),
        ("yaw stall j5+j7".to_string(), yaw_stall()),
        ("random 1-4 pins".to_string(), random_pins(&model)),
    ];
    if let Ok(windows) = std::env::var("BENCH_IK_WINDOWS") {
        regimes.extend(windows.split(';').filter(|w| !w.is_empty()).map(recorded));
    }
    println!("bench_ik_box: {cycles} cycles after {WARMUP} warm-up, {runs} runs, µs");
    let mut summary = Vec::new();
    for (name, episodes) in &regimes {
        for solver in [Solver::Guarded, Solver::AsRun] {
            let mut per_run = Vec::with_capacity(runs);
            for run in 0..runs {
                let stats = measure(solver, &model, episodes, cycles);
                let [min, p50, p99, p999, max] = percentiles(&stats.micros);
                println!(
                    "{name:<16} {solver:<7?} run {run}: min {min:.2} p50 {p50:.2} p99 {p99:.2} \
                     p99.9 {p999:.2} max {max:.2} | passes {} | pins {} | {} | fallbacks {} in \
                     {} cycles | allocations {}",
                    histogram(&stats.passes, "", 0, cycles),
                    histogram(&stats.pins, "", 0, cycles),
                    histogram(&stats.joints, "j", 1, cycles),
                    stats.fallbacks,
                    stats.fallback_cycles,
                    stats.allocations
                );
                let by_pins: Vec<String> = (0..8)
                    .filter(|&n| stats.by_pins[n].len() >= 1000)
                    .map(|n| {
                        let [_, p50, p99, p999, _] = percentiles(&stats.by_pins[n]);
                        format!("{n}: {p50:.2}/{p99:.2}/{p999:.2}")
                    })
                    .collect();
                println!(
                    "  by pins, p50/p99/p99.9 (>= 1000 cycles): {}",
                    by_pins.join(", ")
                );
                assert_eq!(stats.allocations, 0, "{name} {solver:?} allocates");
                per_run.push([min, p50, p99, p999, max]);
            }
            let median = |k: usize| {
                let mut v: Vec<f64> = per_run.iter().map(|r| r[k]).collect();
                v.sort_by(f64::total_cmp);
                v[v.len() / 2]
            };
            let max = per_run.iter().map(|r| r[4]).fold(0.0, f64::max);
            summary.push(format!(
                "| {name} | {solver:?} | {:.2} | {:.2} | {:.2} | {:.2} | {max:.2} |",
                median(0),
                median(1),
                median(2),
                median(3)
            ));
        }
    }
    println!("| regime | solver | min | p50 | p99 | p99.9 | max |\n|---|---|---|---|---|---|---|");
    summary.iter().for_each(|row| println!("{row}"));
}
