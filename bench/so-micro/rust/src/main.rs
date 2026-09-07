//! Per-call microbenchmark of the FCI v5 (Franka Emika Robot, FER) `libfcimodels` model path, Rust side.
//!
//! Prices the five `franka::Model` calls a model-based controller makes —
//! `gravity`, `coriolis`, `mass`, `pose(EndEffector)`, `zeroJacobian(EndEffector)` —
//! individually and as the five-call sequence, driving the crate's
//! [`franka::model::so_backend::SoModelBackend`] over a captured
//! `libfcimodels_x64.so`. The C++ counterpart is `../cpp/main.cpp`, which drives the
//! same shared object through libfranka 0.9.2's own `ModelLibrary` + `franka::Model`
//! call path.
//!
//! No robot, no simulator, no network: this loads a file and calls functions out of it.
//!
//! # Why the results are reported three ways
//!
//! `steady` runs the calls back to back, so the core is at its top frequency and every
//! cache is warm: that is the cost of the *code*, and it is what a Rust-versus-C++
//! comparison has to be made on.
//!
//! `duty-1kHz` inserts a `nanosleep` before every cycle to imitate a 1 kHz control loop
//! waiting on the robot's state datagram. On a laptop-class part the core drops into a
//! low-power state during that millisecond and needs tens of microseconds of work to
//! climb back to its top frequency, so the *same* five calls read several times slower —
//! and how much slower depends on how much unrelated work the caller happened to do
//! first. `ramp` makes that explicit by timing the same sequence after increasing amounts
//! of untimed filler work.
//!
//! This matters because it is the trap the full FER benchmark write-up (kept privately)
//! fell into: an in-the-loop measurement of this model path prices the CPU's wake-up
//! behaviour at least as much as it prices the model.
//!
//! Only the `steady state` block is a cross-language comparison. [`filler`] is not the same
//! machine code here and in `../cpp/main.cpp` and does not drive the core to the same
//! P-state, so the two ramp blocks plateau at different values; what they are for is the
//! *shape* of each binary's own curve.
//!
//! # Usage
//!
//! ```text
//! so_micro_rust <path-to-libfcimodels_x64.so> [iterations]
//! ```

use std::hint::black_box;
use std::path::PathBuf;

use franka::model::{Frame, Model};
use franka::robot_state::RobotState;

/// `CLOCK_MONOTONIC`, the clock both sides of the benchmark read.
fn monotonic_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, initialised `timespec` and `CLOCK_MONOTONIC` always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec * 1_000_000_000 + ts.tv_nsec
}

/// Pins the process to one CPU so the frequency behaviour is at least reproducible.
fn pin_cpu(cpu: usize) {
    // SAFETY: `set` is zeroed before use and `sched_setaffinity` is passed its real size.
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(cpu, &mut set);
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set);
    }
}

/// Sleeps for `nanos`, imitating the wait on a robot state datagram.
fn sleep_ns(nanos: i64) {
    let ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: nanos,
    };
    // SAFETY: `ts` is a valid `timespec` and a null remainder pointer is allowed.
    unsafe { libc::nanosleep(&ts, std::ptr::null_mut()) };
}

/// Nearest-rank percentile over an already-sorted slice.
fn percentile(sorted: &[f64], q: f64) -> f64 {
    let rank = (q * sorted.len() as f64).ceil() as usize;
    sorted[rank.clamp(1, sorted.len()) - 1]
}

fn report(name: &str, values: &mut [f64]) {
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!(
        "{name:<16} p50 {:>8.3}  p90 {:>8.3}  p99 {:>8.3}  min {:>8.3}  mean {:>8.3}",
        percentile(values, 0.50),
        percentile(values, 0.90),
        percentile(values, 0.99),
        values[0],
        values.iter().sum::<f64>() / values.len() as f64,
    );
}

/// Untimed filler work, used only to move the core up its frequency ramp.
fn filler(mut a: f64, iterations: u64) -> f64 {
    for _ in 0..iterations {
        a = a * 1.000_000_1 + 1e-7;
        if a > 1e30 {
            a *= 1e-30;
        }
    }
    a
}

/// The state the C++ side uses too: robot L's read-only probe of 2026-09-05
/// (`bench/results/20260905-fer-hw/L/model_probe_cpp_0.9.2.txt`), with a non-zero `dq`
/// so `coriolis` is not evaluated at rest.
#[allow(non_snake_case)]
fn probe_state() -> RobotState {
    RobotState {
        q: [
            -0.000_230_663_600_055_855,
            -0.785_250_788_805_778,
            0.000_051_590_539_422_385,
            -2.356_921_386_531_59,
            0.000_811_206_067_415_174,
            1.570_338_876_909_68,
            0.785_072_525_387_837,
        ],
        dq: [0.11, -0.22, 0.33, -0.44, 0.55, -0.66, 0.77],
        F_T_EE: [
            0.707_099_974_155_426,
            -0.707_099_974_155_426,
            0.0,
            0.0,
            0.707_099_974_155_426,
            0.707_099_974_155_426,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.103_399_999_439_716,
            1.0,
        ],
        EE_T_K: [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        m_total: 0.730_000_019_073_486,
        I_total: [
            0.001_000_000_047_497_45,
            0.0,
            0.0,
            0.0,
            0.002_499_999_944_120_65,
            0.0,
            0.0,
            0.0,
            0.001_700_000_022_538_01,
        ],
        F_x_Ctotal: [-0.009_999_999_776_482_58, 0.0, 0.029_999_999_329_447_7],
        O_ddP_O: [0.0, 0.0, -9.81],
        ..RobotState::default()
    }
}

fn main() {
    pin_cpu(2);

    let mut args = std::env::args().skip(1);
    let path = match args.next() {
        Some(value) => PathBuf::from(value),
        None => {
            eprintln!("usage: so_micro_rust <libfcimodels_x64.so> [iterations]");
            std::process::exit(2);
        }
    };
    let n: usize = args
        .next()
        .map(|s| s.parse().expect("iterations"))
        .unwrap_or(100_000);

    // The model library imports `sin`, `cos` and `sincos` and carries no `DT_NEEDED` of its
    // own; `franka::model::so_backend::SoModelBackend::load` now opens `libm.so.6` into the
    // process's global scope itself before `dlopen`ing the model library, so this binary no
    // longer needs its own libm priming call — see that module's `open_libm_global`.
    let mut checksum = 0.0;

    // SAFETY: the file the operator named is a captured `libfcimodels` build for this
    // platform; running its code is the entire point of this benchmark.
    let model = unsafe { Model::from_model_library_path(&path) }
        .unwrap_or_else(|e| panic!("cannot load {}: {e}", path.display()));
    let state = probe_state();

    println!("# steady state (back to back, warm core)");
    macro_rules! bench_one {
        ($name:literal, $call:expr) => {{
            for _ in 0..(n / 10) {
                checksum += black_box($call)[0];
            }
            let mut samples = Vec::with_capacity(n);
            for _ in 0..n {
                let t0 = monotonic_ns();
                let out = black_box($call);
                let t1 = monotonic_ns();
                checksum += out[0];
                samples.push((t1 - t0) as f64 * 1e-3);
            }
            report($name, &mut samples);
        }};
    }

    bench_one!("gravity", model.gravity(black_box(&state)));
    bench_one!("coriolis", model.coriolis(black_box(&state)));
    bench_one!("mass", model.mass(black_box(&state)));
    bench_one!("pose_ee", model.pose(Frame::EndEffector, black_box(&state)));
    bench_one!(
        "zero_jacobian_ee",
        model.zero_jacobian(Frame::EndEffector, black_box(&state))
    );

    // The five calls in the order `bench/rust/src/bin/bench_model_control.rs` makes them.
    macro_rules! five_call {
        ($s:expr) => {{
            let s = $s;
            let mass = model.mass(s);
            let coriolis = model.coriolis(s);
            let gravity = model.gravity(s);
            let jacobian = model.zero_jacobian(Frame::EndEffector, s);
            let pose = model.pose(Frame::EndEffector, s);
            black_box(mass)[0]
                + black_box(coriolis)[0]
                + black_box(gravity)[0]
                + black_box(jacobian)[0]
                + black_box(pose)[0]
        }};
    }

    {
        for _ in 0..(n / 10) {
            checksum += five_call!(black_box(&state));
        }
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            let s = black_box(&state);
            let t0 = monotonic_ns();
            let sum = five_call!(s);
            let t1 = monotonic_ns();
            checksum += sum;
            samples.push((t1 - t0) as f64 * 1e-3);
        }
        report("five-call", &mut samples);
    }

    // Duty-cycled: the same five calls, but first in the cycle after a ~1 ms idle wait.
    println!();
    println!("# duty cycled (~1 ms idle before each cycle, as in a 1 kHz control loop)");
    let duty_cycles = (n / 5).clamp(2_000, 20_000);
    {
        let mut samples = Vec::with_capacity(duty_cycles);
        for _ in 0..duty_cycles {
            sleep_ns(900_000);
            let s = black_box(&state);
            let t0 = monotonic_ns();
            let sum = five_call!(s);
            let t1 = monotonic_ns();
            checksum += sum;
            samples.push((t1 - t0) as f64 * 1e-3);
        }
        report("five-call", &mut samples);
    }

    // The same sequence again, after increasing amounts of untimed filler work: this is
    // the frequency ramp, isolated.
    println!();
    println!("# duty cycled, after N iterations of untimed filler work");
    for &warm in &[0u64, 500, 1_500, 4_000, 12_000, 40_000] {
        let mut samples = Vec::with_capacity(duty_cycles);
        for _ in 0..duty_cycles {
            sleep_ns(900_000);
            checksum += filler(1.0, warm);
            let s = black_box(&state);
            let t0 = monotonic_ns();
            let sum = five_call!(s);
            let t1 = monotonic_ns();
            checksum += sum;
            samples.push((t1 - t0) as f64 * 1e-3);
        }
        report(&format!("filler {warm}"), &mut samples);
    }

    println!();
    println!("checksum {checksum:.6}");
}
