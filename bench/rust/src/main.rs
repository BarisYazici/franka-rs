//! Instrumented joint-velocity benchmark for `franka-rs`.
//!
//! Runs the `generate_joint_velocity_motion` profile (same amplitude and period as the
//! libfranka example) for a configurable duration and records, per control cycle, a
//! `CLOCK_MONOTONIC` timestamp, `RobotState::time` and `control_command_success_rate`.
//!
//! Two variants:
//!
//! * `--variant control` — [`Robot::control_joint_velocities`] with `limit_rate = true` and the
//!   default cutoff frequency.
//! * `--variant active` — [`Robot::start_joint_velocity_control`] and then
//!   `read_once()` / `write_once()`.
//!
//! Nothing is allocated or printed inside the loop: the sample array is sized and zeroed up
//! front, and all statistics are computed after the motion has finished.
//!
//! The C++ counterpart is `../cpp/bench_joint_velocity.cpp`; both write the same JSON schema.

// The homing motion generator is shared verbatim with the ported examples rather than
// duplicated here, exactly as the C++ side links `libexamples_common.a`.
#[path = "../../../crates/franka-rs/examples/common/mod.rs"]
mod common;

use franka::{
    motion_finished, ControllerMode, FrankaResult, JointVelocities, MoveControllerMode,
    RealtimeConfig, Robot, DEFAULT_CUTOFF_FREQUENCY,
};
use franka_bench::sched::{apply_mlockall, getrusage_self, sched_json, timeval_delta};
use franka_bench::stats::{stats_json, summarize};
use franka_bench::time::monotonic_ns;

/// The libfranka example's `time_max`, in seconds.
const TIME_MAX: f64 = 1.0;
/// The libfranka example's `omega_max`, in rad/s.
const OMEGA_MAX: f64 = 1.0;

#[derive(Clone, Copy, Default)]
struct Sample {
    /// `CLOCK_MONOTONIC` at callback entry / `read_once()` return.
    t_enter_ns: i64,
    /// `CLOCK_MONOTONIC` after `write_once()` returned (active variant only).
    t_sent_ns: i64,
    state_time_ms: u64,
    success_rate: f64,
}

/// The `generate_joint_velocity_motion` profile, verbatim from the libfranka example.
fn omega_at(time: f64) -> f64 {
    let cycle = (-1.0f64)
        .powf(((time - time % TIME_MAX) / TIME_MAX).floor())
        .floor();
    cycle * OMEGA_MAX / 2.0 * (1.0 - (2.0 * std::f64::consts::PI / TIME_MAX * time).cos())
}

struct Args {
    host: String,
    variant: String,
    condition: String,
    cell_first: String,
    out: Option<String>,
    duration_s: f64,
    rep: i32,
    order_in_cell: i32,
    mlock: bool,
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().collect();
    let mut args = Args {
        host: String::new(),
        variant: "control".to_owned(),
        condition: "unspecified".to_owned(),
        cell_first: "unspecified".to_owned(),
        out: None,
        duration_s: 30.0,
        rep: 1,
        order_in_cell: 0,
        mlock: false,
    };
    let mut i = 1;
    while i < argv.len() {
        let flag = argv[i].clone();
        let takes_value = matches!(
            flag.as_str(),
            "--variant"
                | "--duration"
                | "--out"
                | "--condition"
                | "--rep"
                | "--order"
                | "--cell-first"
        );
        let value = if takes_value {
            i += 1;
            match argv.get(i) {
                Some(value) => value.clone(),
                None => usage_exit(&argv[0], &format!("missing value for {flag}")),
            }
        } else {
            String::new()
        };
        match flag.as_str() {
            "--variant" => args.variant = value,
            "--duration" => args.duration_s = value.parse().unwrap_or(30.0),
            "--out" => args.out = Some(value),
            "--condition" => args.condition = value,
            "--rep" => args.rep = value.parse().unwrap_or(1),
            "--order" => args.order_in_cell = value.parse().unwrap_or(0),
            "--cell-first" => args.cell_first = value,
            "--mlock" => args.mlock = true,
            other if other.starts_with('-') => {
                usage_exit(&argv[0], &format!("unknown flag {other}"));
            }
            other => args.host = other.to_owned(),
        }
        i += 1;
    }
    if args.host.is_empty() || (args.variant != "control" && args.variant != "active") {
        usage_exit(
            &argv[0],
            "a robot hostname and a valid --variant are required",
        );
    }
    args
}

fn usage_exit(program: &str, message: &str) -> ! {
    eprintln!("error: {message}");
    eprintln!(
        "usage: {program} <robot-hostname> [--variant control|active] [--duration 30] \
         [--mlock] [--condition NAME] [--rep N] [--order N] [--cell-first cpp|rust] \
         [--out FILE]"
    );
    std::process::exit(2);
}

fn main() {
    let args = parse_args();
    if let Err(e) = run(&args) {
        eprintln!("franka error: {e}");
        std::process::exit(1);
    }
}

fn run(args: &Args) -> FrankaResult<()> {
    let mlock = apply_mlockall(args.mlock);
    // Both libraries raise the calling thread to the highest SCHED_FIFO priority in the
    // `Robot` constructor even under `RealtimeConfig::Ignore`, so record the policy the
    // process was *launched* with as well as the one it ends up running the loop with.
    let sched_at_start = sched_json();

    // Preallocate and zero the sample array: 1 kHz plus generous headroom, never resized in
    // the loop.
    let capacity = (args.duration_s * 1000.0 * 1.5) as usize + 4096;
    let mut samples = vec![Sample::default(); capacity];
    let mut count = 0usize;

    // `RealtimeConfig::Ignore`: this box is not PREEMPT_RT (no `/sys/kernel/realtime`), so
    // realtime priority is applied externally with `chrt -f 80`.
    let robot = Robot::new(&args.host, RealtimeConfig::Ignore)?;
    common::set_default_behavior(&robot)?;

    // Home the arm before measuring, exactly like the ported examples.
    let mut motion_generator =
        common::MotionGenerator::new(robot.fci_version(), 0.5, common::READY_POSE);
    robot.control_joint_positions(
        |state, period| motion_generator.step(state, period),
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;

    robot.set_collision_behavior(
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    )?;

    let usage_before = getrusage_self();
    let wall_start = monotonic_ns();

    if args.variant == "control" {
        let mut time = 0.0f64;
        let duration_s = args.duration_s;
        robot.control_joint_velocities(
            |state, period| {
                let now = monotonic_ns();
                if count < capacity {
                    samples[count] = Sample {
                        t_enter_ns: now,
                        t_sent_ns: 0,
                        state_time_ms: state.time.as_millis(),
                        success_rate: state.control_command_success_rate,
                    };
                    count += 1;
                }
                time += period.as_secs_f64();
                let omega = omega_at(time);
                let velocities = JointVelocities::new([0.0, 0.0, 0.0, omega, omega, omega, omega]);
                if time >= duration_s {
                    motion_finished(velocities)
                } else {
                    velocities
                }
            },
            ControllerMode::JointImpedance,
            true,
            DEFAULT_CUTOFF_FREQUENCY,
        )?;
    } else {
        let mut time = 0.0f64;
        let mut finished = false;
        let mut control = robot.start_joint_velocity_control(MoveControllerMode::JointImpedance)?;
        while !finished {
            let (state, period) = control.read_once()?;
            let t_read = monotonic_ns();
            time += period.as_secs_f64();
            let omega = omega_at(time);
            let mut velocities = JointVelocities::new([0.0, 0.0, 0.0, omega, omega, omega, omega]);
            if time >= args.duration_s {
                velocities = motion_finished(velocities);
                finished = true;
            }
            let state_time_ms = state.time.as_millis();
            let success_rate = state.control_command_success_rate;
            control.write_once(&velocities, None)?;
            let t_sent = monotonic_ns();
            if count < capacity {
                samples[count] = Sample {
                    t_enter_ns: t_read,
                    t_sent_ns: t_sent,
                    state_time_ms,
                    success_rate,
                };
                count += 1;
            }
        }
    }

    let wall_end = monotonic_ns();
    let usage_after = getrusage_self();

    // --- statistics, all computed after the loop ---
    let wall_s = (wall_end - wall_start) as f64 * 1e-9;
    let samples = &samples[..count];

    let mut intervals: Vec<f64> = samples
        .windows(2)
        .map(|w| (w[1].t_enter_ns - w[0].t_enter_ns) as f64 * 1e-3)
        .collect();
    let mut latencies: Vec<f64> = if args.variant == "active" {
        samples
            .iter()
            .map(|s| (s.t_sent_ns - s.t_enter_ns) as f64 * 1e-3)
            .collect()
    } else {
        Vec::new()
    };

    // Saturating: a backwards `state.time` step counts as dt = 0 (no loss) and is reported
    // separately, so C++ and Rust score such an event identically.
    let (mut lost_cycles, mut lost_states, mut max_consecutive, mut consecutive) =
        (0u64, 0u64, 0u64, 0u64);
    let mut backwards_steps = 0u64;
    for w in samples.windows(2) {
        if w[1].state_time_ms < w[0].state_time_ms {
            backwards_steps += 1;
            consecutive = 0;
            continue;
        }
        let dt = w[1].state_time_ms - w[0].state_time_ms;
        if dt > 1 {
            lost_cycles += 1;
            lost_states += dt - 1;
            consecutive += 1;
            max_consecutive = max_consecutive.max(consecutive);
        } else {
            consecutive = 0;
        }
    }

    // Skip cycle 0: no command has been acknowledged yet, so its success rate is always 0.
    let mut sr_min = 1.0f64;
    let mut sr_max = 0.0f64;
    let mut sr_sum = 0.0f64;
    let scored = samples.get(1..).unwrap_or(&[]);
    for sample in scored {
        sr_min = sr_min.min(sample.success_rate);
        sr_max = sr_max.max(sample.success_rate);
        sr_sum += sample.success_rate;
    }
    let sr_n = scored.len();
    let sr_avg = if sr_n > 0 { sr_sum / sr_n as f64 } else { 0.0 };
    let sr_final = samples.last().map(|s| s.success_rate).unwrap_or(0.0);

    let user_s = timeval_delta(usage_after.ru_utime, usage_before.ru_utime);
    let sys_s = timeval_delta(usage_after.ru_stime, usage_before.ru_stime);

    let interval_stats = summarize(&mut intervals);
    let latency_stats = summarize(&mut latencies);

    let json = format!(
        "{{\n  \"lang\": \"rust\",\n  \"library\": \"franka-rs 0.1.0\",\n  \"variant\": \"{variant}\",\n  \
         \"condition\": \"{condition}\",\n  \"rep\": {rep},\n  \
         \"order_in_cell\": {order},\n  \"cell_first_client\": \"{cell_first}\",\n  \
         \"host\": \"{host}\",\n  \
         \"duration_s\": {duration:.6},\n  \"limit_rate\": {limit_rate},\n  \"cycles\": {cycles},\n  \
         \"wall_s\": {wall:.6},\n  \"sched\": {sched},\n  \"sched_at_start\": {sched_start},\n  \
         \"mlockall\": {{\"requested\": {mlock_req}, \"ok\": {mlock_ok}, \"error\": {mlock_err}, \
         \"rlimit_memlock\": \"{mlock_rlimit}\"}},\n  \
         \"interval_us\": {interval},\n  \"latency_us\": {latency},\n  \
         \"lost\": {{\"cycles\": {lost_cycles}, \"states\": {lost_states}, \
         \"max_consecutive\": {max_consecutive}, \
         \"backwards_time_steps\": {backwards_steps}}},\n  \
         \"success_rate\": {{\"min\": {sr_min:.6}, \"avg\": {sr_avg:.6}, \"max\": {sr_max:.6}, \
         \"final\": {sr_final:.6}, \"n\": {sr_n}}},\n  \
         \"cpu\": {{\"user_s\": {user:.6}, \"sys_s\": {sys:.6}, \"total_s\": {total:.6}, \
         \"percent\": {percent:.6}, \"minor_faults\": {minflt}, \"major_faults\": {majflt}, \
         \"vol_ctx_switches\": {nvcsw}, \"invol_ctx_switches\": {nivcsw}}}\n}}\n",
        variant = args.variant,
        condition = args.condition,
        rep = args.rep,
        order = args.order_in_cell,
        cell_first = args.cell_first,
        host = args.host,
        duration = args.duration_s,
        limit_rate = if args.variant == "control" {
            "true"
        } else {
            "null"
        },
        cycles = count,
        wall = wall_s,
        sched = sched_json(),
        sched_start = sched_at_start,
        mlock_req = mlock.requested,
        mlock_ok = mlock.ok,
        mlock_err = mlock
            .error
            .as_ref()
            .map(|e| format!("\"{}\"", e.replace('\\', "\\\\").replace('"', "\\\"")))
            .unwrap_or_else(|| "null".to_owned()),
        mlock_rlimit = mlock.rlimit,
        interval = stats_json(&interval_stats),
        latency = if args.variant == "active" {
            stats_json(&latency_stats)
        } else {
            "null".to_owned()
        },
        lost_cycles = lost_cycles,
        lost_states = lost_states,
        max_consecutive = max_consecutive,
        backwards_steps = backwards_steps,
        sr_min = sr_min,
        sr_avg = sr_avg,
        sr_max = sr_max,
        sr_final = sr_final,
        sr_n = sr_n,
        user = user_s,
        sys = sys_s,
        total = user_s + sys_s,
        percent = if wall_s > 0.0 {
            (user_s + sys_s) / wall_s * 100.0
        } else {
            0.0
        },
        minflt = usage_after.ru_minflt - usage_before.ru_minflt,
        majflt = usage_after.ru_majflt - usage_before.ru_majflt,
        nvcsw = usage_after.ru_nvcsw - usage_before.ru_nvcsw,
        nivcsw = usage_after.ru_nivcsw - usage_before.ru_nivcsw,
    );

    print!("{json}");
    if let Some(path) = &args.out {
        if let Err(e) = std::fs::write(path, &json) {
            eprintln!("failed to write {path}: {e}");
            std::process::exit(1);
        }
    }

    Ok(())
}
