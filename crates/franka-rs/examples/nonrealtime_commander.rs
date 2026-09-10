//! A non-realtime commander moving the end effector through Cartesian *targets* -- stepped,
//! bursty, sometimes silent for seconds -- and what the 1 kHz side does with them.
//!
//! In the default `--bridged` mode the commander is three calls: `Robot::
//! start_cartesian_target_control`, `set_position` whenever a target comes, `stop()` at the
//! end. The crate's loop on its own thread does the rest (see `franka::robot::target_control`):
//! the online trajectory generator re-plans a jerk-limited profile every cycle under a small
//! budget (0.3 m/s, 0.5 m/s^2, 20 m/s^3 by default; `--budget V,A,J`), re-anchored on the
//! robot's echo, with the rate limiter under the same budget as the backstop, so a 5 cm step
//! becomes an S-curve peaking at about 0.12 m/s that lands after about 0.85 s (the budget is a
//! norm; each axis gets 1/sqrt(3) of it). In `--raw` mode the same targets go to
//! `control_cartesian_pose` as they are, with no generator and no limiting, so the first 5 cm
//! target reaches the robot as a 50 m/s jump and its motion generator refuses it with a
//! reflex; the example prints the error text, runs `automatic_error_recovery()` and exits 0.
//! That is the contrast.
//!
//! The scripted commander steps the target by +-5 cm in x, y or z inside a +-12 cm box around
//! the start pose (never more than 5 cm below it), with irregular holds, one 2 s stall and one
//! burst of 20 targets inside 100 ms, about 20 s in all; `--stdin` reads
//! `x y z` lines (metres, relative to the start) instead. `--log PATH` writes one CSV row per
//! cycle (raw target, echoed `O_T_EE_c`, measured `O_T_EE`, joint angles, external wrench, the
//! generator's velocity and acceleration of the command -- zero in `--raw` mode) from the
//! loop's observer into a `Vec` sized before the loop. `--rotate` (bridged only) adds a slow
//! yaw sweep of +-15 degrees about the base z, a sine with a 12 s period set at 20 Hz from a
//! thread of its own through `set_orientation`. `FRANKA_REALTIME=ignore` runs it against
//! franka-sim.
//!
//! # Warning
//! The end effector moves inside a 24 cm cube around wherever it is when the example starts;
//! make sure that cube is free and have the user stop button at hand. `--raw` provokes a reflex.

#[path = "common/mod.rs"]
mod common;

use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use common::commander::{run_commander, BOX};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use franka::robot::target_control::TargetSlot;
use franka::{
    CartesianPose, CartesianSent, CartesianTargetControl, ControllerMode, Duration, FrankaError,
    FrankaResult, OtgLimits, Robot, RobotState, TargetControlOptions, MAX_CUTOFF_FREQUENCY,
};

/// Measured deviation from the start at which the loop gives up, m.
const MAX_DEVIATION: f64 = 0.30;
/// Rows the CSV log can hold: two minutes at 1 kHz, allocated before the loop starts.
const LOG_CAPACITY: usize = 120_000;
/// The `--rotate` sweep: +-15 degrees of yaw about the base z, a sine with this period.
const YAW_SWEEP: f64 = 15.0 * std::f64::consts::PI / 180.0;
const YAW_PERIOD: f64 = 12.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Bridged,
    Raw,
}

fn usage(program: &str) -> ! {
    eprintln!("Usage: {program} <hostname> [--bridged | --raw] [--stdin] [--log PATH] [--yes] [--budget V,A,J] [--rotate]");
    std::process::exit(1);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (mut hostname, mut mode, mut stdin, mut log, mut yes, mut rotate) =
        (None, Mode::Bridged, false, None, false, false);
    let mut limits = TargetControlOptions::default().limits;
    let mut arg = 1;
    while arg < args.len() {
        match args[arg].as_str() {
            "--bridged" => mode = Mode::Bridged,
            "--raw" => mode = Mode::Raw,
            "--stdin" => stdin = true,
            "--yes" => yes = true,
            "--rotate" => rotate = true,
            "--log" => {
                arg += 1;
                log = Some(args.get(arg).cloned().unwrap_or_else(|| usage(&args[0])));
            }
            "--budget" => {
                arg += 1;
                let parsed: Vec<f64> = args.get(arg).map_or(Vec::new(), |s| {
                    s.split(',').filter_map(|v| v.parse().ok()).collect()
                });
                let [max_velocity, max_acceleration, max_jerk] = parsed[..] else {
                    usage(&args[0])
                };
                limits = OtgLimits {
                    max_velocity,
                    max_acceleration,
                    max_jerk,
                };
            }
            other if hostname.is_none() && !other.starts_with("--") => hostname = Some(other),
            _ => usage(&args[0]),
        }
        arg += 1;
    }
    let hostname = hostname.unwrap_or_else(|| usage(&args[0]));
    if rotate && mode == Mode::Raw {
        eprintln!("--rotate is for --bridged mode only");
        usage(&args[0]);
    }
    match run(hostname, mode, stdin, log.as_deref(), yes, limits, rotate) {
        // In `--raw` mode the reflex is the demonstration; in `--bridged` mode it is a failure.
        Ok(reflex) => std::process::exit(i32::from(reflex && mode != Mode::Raw)),
        Err(e) => {
            println!("{e}");
            std::process::exit(1);
        }
    }
}

fn translation(pose: &[f64; 16]) -> [f64; 3] {
    [pose[12], pose[13], pose[14]]
}

fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The CSV rows -- `t`, target, commanded and measured positions, joint angles, external
/// wrench, the generator's velocity and acceleration -- pushed from the realtime side under
/// `try_lock` (uncontended until the loop has ended) into capacity reserved up front.
#[derive(Default)]
struct Log {
    rows: Mutex<Vec<[f64; 29]>>,
    cycles: AtomicU64,
    backstop_bound: AtomicU64,
}

impl Log {
    fn record(&self, state: &RobotState, target: &[f64; 3], v: &[f64; 3], a: &[f64; 3]) {
        let t = self.cycles.fetch_add(1, Ordering::Relaxed) as f64 * 1e-3;
        let Ok(mut rows) = self.rows.try_lock() else {
            return;
        };
        if rows.len() < rows.capacity() {
            let mut out = [t; 29];
            out[1..4].copy_from_slice(target);
            out[4..7].copy_from_slice(&translation(&state.O_T_EE_c));
            out[7..10].copy_from_slice(&translation(&state.O_T_EE));
            out[10..17].copy_from_slice(&state.q);
            out[17..23].copy_from_slice(&state.O_F_ext_hat_K);
            out[23..26].copy_from_slice(v);
            out[26..29].copy_from_slice(a);
            rows.push(out);
        }
    }
}

/// Returns whether the motion was aborted by the robot (and recovered from).
fn run(
    hostname: &str,
    mode: Mode,
    from_stdin: bool,
    log_path: Option<&str>,
    yes: bool,
    limits: OtgLimits,
    rotate: bool,
) -> FrankaResult<bool> {
    let robot = Arc::new(Robot::new(hostname, common::realtime_config_from_env())?);
    common::set_default_behavior(&robot)?;
    // libfranka's example thresholds: the shared 10 N default is crossed at 0.25 m/s.
    let torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0];
    let force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0];
    robot.set_collision_behavior(torque, torque, torque, torque, force, force, force, force)?;
    match mode {
        Mode::Bridged => println!("Mode: bridged, budget {limits:?} (m/s, m/s^2, m/s^3)"),
        Mode::Raw => println!("Mode: raw, no generator, no rate limiting, expect a reflex"),
    }
    if rotate {
        println!(
            "Yaw sweep: +-{:.0} degrees about the base z, {YAW_PERIOD} s period",
            YAW_SWEEP.to_degrees()
        );
    }
    if from_stdin {
        println!("Reading targets `x y z` (metres, relative to the start pose) from stdin.");
    } else if !yes {
        common::wait_for_enter();
    }
    let log = Arc::new(Log::default());
    *log.rows.lock().unwrap() = Vec::with_capacity(LOG_CAPACITY * usize::from(log_path.is_some()));

    let started = Instant::now();
    let result = match mode {
        Mode::Bridged => run_bridged(&robot, from_stdin, limits, rotate, Arc::clone(&log)),
        Mode::Raw => run_raw(&robot, from_stdin, &log),
    };
    let elapsed = started.elapsed().as_secs_f64();
    let reflex = match result {
        Ok(()) => {
            println!("\nMotion finished after {elapsed:.3} s.");
            false
        }
        Err(FrankaError::Control(exception)) => {
            println!("\nThe robot aborted the motion after {elapsed:.3} s. Its error text:\n");
            println!("{}\n", exception.message.trim_end());
            println!("reflex reasons: {}", exception.last_motion_errors);
            robot.automatic_error_recovery()?;
            println!("automatic_error_recovery() succeeded; the robot is ready again.");
            true
        }
        Err(other) => return Err(other),
    };
    println!(
        "{} cycles, backstop bound (by more than 10 um) in {} of them",
        log.cycles.load(Ordering::Relaxed),
        log.backstop_bound.load(Ordering::Relaxed)
    );
    if let Some(path) = log_path {
        let rows = log.rows.lock().unwrap();
        let io_error =
            |e: std::io::Error| FrankaError::InvalidArgument(format!("--log {path}: {e}"));
        let mut out = std::io::BufWriter::new(std::fs::File::create(path).map_err(io_error)?);
        writeln!(out, "t,target_x,target_y,target_z,cmd_x,cmd_y,cmd_z,meas_x,meas_y,meas_z,q0,q1,q2,q3,q4,q5,q6,fx,fy,fz,tx,ty,tz,cmd_vx,cmd_vy,cmd_vz,cmd_ax,cmd_ay,cmd_az").map_err(io_error)?;
        for row in rows.iter() {
            write!(out, "{:.4}", row[0]).map_err(io_error)?;
            for value in &row[1..] {
                write!(out, ",{value:.9}").map_err(io_error)?;
            }
            writeln!(out).map_err(io_error)?;
        }
        out.flush().map_err(io_error)?;
        println!("wrote {} rows to {path}", rows.len());
    }
    Ok(reflex)
}

/// The whole bridge: start, `set_position` from the commander (and `set_orientation` from
/// the yaw sweep), `stop`. The observer keeps the CSV and the counters; it runs on the
/// realtime thread and must not allocate.
fn run_bridged(
    robot: &Arc<Robot>,
    from_stdin: bool,
    limits: OtgLimits,
    rotate: bool,
    log: Arc<Log>,
) -> FrankaResult<()> {
    let observer = {
        let log = Arc::clone(&log);
        move |state: &RobotState, sent: &CartesianSent| {
            // Micrometres are the float32 echo's noise in the limiter's reference; count
            // the cycles in which the backstop really shaped the command.
            let bound = sent.backstop_alteration > 1e-5;
            log.backstop_bound
                .fetch_add(u64::from(bound), Ordering::Relaxed);
            log.record(state, &sent.target, &sent.velocity, &sent.acceleration);
        }
    };
    let options = TargetControlOptions::default()
        .with_limits(limits)
        .with_max_deviation(MAX_DEVIATION)
        .with_observer(observer);
    let control = robot.start_cartesian_target_control(options)?;
    let start = control.target();
    println!("Start position: {start:.3?}; targets stay within +-{BOX} m of it.");
    let done = AtomicBool::new(false);
    std::thread::scope(|scope| {
        if rotate {
            scope.spawn(|| yaw_sweep(&control, &done));
        }
        run_commander(from_stdin, &mut |relative| {
            let absolute = std::array::from_fn(|i| start[i] + relative[i]);
            control
                .set_position(absolute)
                .map_err(|e| eprintln!("commander: {e}"))
                .is_ok()
        });
        done.store(true, Ordering::SeqCst);
    });
    control.stop()
}

/// Sets the orientation to `Rz(yaw) * R_start` at 20 Hz until `done`, a quaternion in
/// `[x, y, z, w]` order as `set_orientation` takes it.
fn yaw_sweep(control: &CartesianTargetControl, done: &AtomicBool) {
    let [x, y, z, w] = control.target_orientation();
    let start = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    let began = Instant::now();
    while !done.load(Ordering::SeqCst) {
        let t = began.elapsed().as_secs_f64();
        let yaw = YAW_SWEEP * (std::f64::consts::TAU * t / YAW_PERIOD).sin();
        let q = (UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw) * start).coords;
        if control.set_orientation([q[0], q[1], q[2], q[3]]).is_err() {
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}

/// The contrast: the commander's slot read by a plain `control_cartesian_pose` callback
/// that sends `start pose + target` as is, anchored on the first echo like the bridge.
fn run_raw(robot: &Robot, from_stdin: bool, log: &Log) -> FrankaResult<()> {
    let slot = Arc::new(TargetSlot::<3>::new([0.0; 3]));
    let finished = Arc::new(AtomicBool::new(false));
    let (commander_slot, commander_finished) = (Arc::clone(&slot), Arc::clone(&finished));
    std::thread::spawn(move || {
        run_commander(from_stdin, &mut |target| {
            commander_slot.publish(target);
            true
        });
        commander_finished.store(true, Ordering::SeqCst);
    });
    let (mut start_pose, mut target, mut settled, mut deviated) = (None, [0.0f64; 3], 0u32, false);
    robot.control_cartesian_pose(
        |state: &RobotState, _period: Duration| {
            let start_pose = *start_pose.get_or_insert(state.O_T_EE_c);
            let (start, commanded) = (translation(&start_pose), translation(&state.O_T_EE_c));
            if !deviated && distance(&translation(&state.O_T_EE), &start) > MAX_DEVIATION {
                deviated = true;
                target = std::array::from_fn(|i| commanded[i] - start[i]);
            } else if !deviated {
                slot.load(&mut target);
            }
            let goal: [f64; 3] = std::array::from_fn(|i| start[i] + target[i]);
            let mut pose = start_pose;
            pose[12..15].copy_from_slice(&goal);
            log.record(state, &goal, &[0.0; 3], &[0.0; 3]);
            let done = deviated || finished.load(Ordering::SeqCst);
            settled = if done && distance(&commanded, &goal) < 1e-3 {
                settled + 1
            } else {
                0
            };
            let mut output = CartesianPose::new(pose);
            output.motion_finished = settled >= 250;
            output
        },
        ControllerMode::CartesianImpedance,
        false,
        MAX_CUTOFF_FREQUENCY,
    )
}
