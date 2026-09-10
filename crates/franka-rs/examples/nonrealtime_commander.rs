//! A non-realtime commander moving the end effector through Cartesian *targets* -- stepped,
//! bursty, sometimes silent for seconds -- and what the 1 kHz side does with them.
//!
//! In the default `--bridged` mode the commander is three calls: `Robot::
//! start_cartesian_target_control`, `set_position` whenever a target comes, `stop()` at the
//! end. The crate's loop on its own thread does the rest (see `franka::robot::target_control`):
//! the online trajectory generator re-plans a jerk-limited profile every cycle under a small
//! budget (0.3 m/s, 0.5 m/s^2, 20 m/s^3 by default; `--budget V,A,J`), anchored every cycle
//! on the robot's echo with the rate limiter as the backstop under `--backend robot`, on the
//! measured pose within the leash under `--backend impedance`, so a 5 cm step becomes an
//! S-curve peaking at about 0.12 m/s that lands after about 0.85 s (the budget is a norm; each
//! axis gets 1/sqrt(3) of it). In `--raw` mode the same targets go to
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
//! `--backend impedance` (the default, the crate's) follows the generator's pose with the
//! crate's own impedance torques; `--stiffness KX` and `--damping KXD` set its three
//! translational Cartesian gains (N/m, N s/m), `--no-feedforward` damps the absolute velocity
//! instead of the velocity error (DROID parity), `--project-joint-gains` confines the joint
//! gains to the Jacobian's nullspace, `--leash M` sets the translational leash (m); the CSV
//! gains the joint goal, the torques, the IK residual and what the leash took off. `--backend
//! robot` streams the pose to the robot's own controller. `--thresholds N` sets all eight
//! collision thresholds (contact and collision, joints in Nm and Cartesian in N / Nm) to `N`;
//! the default is libfranka's example values.
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
    Backend, CartesianPose, CartesianSent, CartesianTargetControl, ControllerMode, Duration,
    FrankaError, FrankaResult, ImpedanceGains, ImpedanceOptions, Leash, OtgLimits, Robot,
    RobotState, TargetControlOptions, MAX_CUTOFF_FREQUENCY,
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

// -- Arguments -------------------------------------------------------------------------------

struct Args {
    hostname: String,
    mode: Mode,
    stdin: bool,
    log: Option<String>,
    yes: bool,
    rotate: bool,
    limits: OtgLimits,
    backend: Backend,
    /// One value for all eight collision thresholds, or libfranka's example values.
    thresholds: Option<f64>,
}

fn usage(program: &str) -> ! {
    eprintln!(
        "Usage: {program} <hostname> [--bridged | --raw] [--stdin] [--log PATH] [--yes] \
         [--budget V,A,J] [--rotate] [--backend robot|impedance] [--stiffness KX] [--damping KXD] \
         [--no-feedforward] [--project-joint-gains] [--leash M] [--thresholds N]"
    );
    std::process::exit(1);
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    let (mut hostname, mut mode, mut stdin, mut log, mut yes, mut rotate) =
        (None, Mode::Bridged, false, None, false, false);
    let mut limits = TargetControlOptions::default().limits;
    let (mut robot_backend, mut stiffness, mut damping, mut leash) = (false, None, None, None);
    let (mut feedforward, mut project, mut thresholds) = (true, false, None);
    let mut arg = 1;
    let value = |arg: &mut usize| {
        *arg += 1;
        args.get(*arg).cloned().unwrap_or_else(|| usage(&args[0]))
    };
    while arg < args.len() {
        match args[arg].as_str() {
            "--bridged" => mode = Mode::Bridged,
            "--raw" => mode = Mode::Raw,
            "--stdin" => stdin = true,
            "--yes" => yes = true,
            "--rotate" => rotate = true,
            "--log" => log = Some(value(&mut arg)),
            "--budget" => {
                let parsed: Vec<f64> = value(&mut arg)
                    .split(',')
                    .filter_map(|v| v.parse().ok())
                    .collect();
                let [max_velocity, max_acceleration, max_jerk] = parsed[..] else {
                    usage(&args[0])
                };
                limits = OtgLimits {
                    max_velocity,
                    max_acceleration,
                    max_jerk,
                };
            }
            "--backend" => match value(&mut arg).as_str() {
                "robot" => robot_backend = true,
                "impedance" => robot_backend = false,
                _ => usage(&args[0]),
            },
            "--stiffness" => stiffness = value(&mut arg).parse().ok().or_else(|| usage(&args[0])),
            "--damping" => damping = value(&mut arg).parse().ok().or_else(|| usage(&args[0])),
            "--leash" => leash = value(&mut arg).parse().ok().or_else(|| usage(&args[0])),
            "--thresholds" => thresholds = value(&mut arg).parse().ok().or_else(|| usage(&args[0])),
            "--no-feedforward" => feedforward = false,
            "--project-joint-gains" => project = true,
            other if hostname.is_none() && !other.starts_with("--") => {
                hostname = Some(other.to_string())
            }
            _ => usage(&args[0]),
        }
        arg += 1;
    }
    let hostname = hostname.unwrap_or_else(|| usage(&args[0]));
    let impedance_flags =
        stiffness.is_some() || damping.is_some() || leash.is_some() || !feedforward || project;
    if mode == Mode::Raw && (rotate || robot_backend || impedance_flags) {
        eprintln!("--rotate, --backend and the impedance flags are for --bridged mode only");
        usage(&args[0]);
    }
    if robot_backend && impedance_flags {
        eprintln!("the impedance flags are for --backend impedance only");
        usage(&args[0]);
    }
    let backend = if robot_backend {
        Backend::RobotController
    } else {
        let mut gains = ImpedanceGains::CARTESIAN;
        if let Some(stiffness) = stiffness {
            gains.cartesian_stiffness[..3].fill(stiffness);
        }
        if let Some(damping) = damping {
            gains.cartesian_damping[..3].fill(damping);
        }
        let leash = Leash {
            translation: leash.unwrap_or(Leash::default().translation),
            ..Leash::default()
        };
        let impedance = ImpedanceOptions::cartesian()
            .with_gains(gains)
            .with_leash(leash)
            .with_velocity_feedforward(feedforward)
            .with_project_joint_gains(project);
        Backend::Impedance(impedance)
    };
    Args {
        hostname,
        mode,
        stdin,
        log,
        yes,
        rotate,
        limits,
        backend,
        thresholds,
    }
}

fn main() {
    let args = parse_args();
    match run(&args) {
        // In `--raw` mode the reflex is the demonstration; in `--bridged` mode it is a failure.
        Ok(reflex) => std::process::exit(i32::from(reflex && args.mode != Mode::Raw)),
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

/// The CSV columns every mode writes, and those the impedance backend adds.
const COLUMNS: &str = "t,target_x,target_y,target_z,cmd_x,cmd_y,cmd_z,meas_x,meas_y,meas_z,\
    q0,q1,q2,q3,q4,q5,q6,fx,fy,fz,tx,ty,tz,cmd_vx,cmd_vy,cmd_vz,cmd_ax,cmd_ay,cmd_az";
const IMPEDANCE_COLUMNS: &str = ",q_goal_0,q_goal_1,q_goal_2,q_goal_3,q_goal_4,q_goal_5,\
    q_goal_6,tau_0,tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,ik_error,leash_alteration";
const COMMON_WIDTH: usize = 29;
const ROW_WIDTH: usize = 45;

/// The CSV rows -- `t`, target, commanded and measured positions, joint angles, external
/// wrench, the generator's velocity and acceleration, and the impedance backend's joint goal,
/// torques, IK residual and leash alteration -- pushed from the realtime side under `try_lock`
/// (uncontended until the loop has ended) into capacity reserved up front.
#[derive(Default)]
struct Log {
    rows: Mutex<Vec<[f64; ROW_WIDTH]>>,
    cycles: AtomicU64,
    backstop_bound: AtomicU64,
    /// The peak IK residual, as `f64` bits.
    peak_ik_error: AtomicU64,
}

impl Log {
    /// One row; `sent` is `None` in `--raw` mode, where the generator's columns are zero.
    fn record(&self, state: &RobotState, target: &[f64; 3], sent: Option<&CartesianSent>) {
        let t = self.cycles.fetch_add(1, Ordering::Relaxed) as f64 * 1e-3;
        let Ok(mut rows) = self.rows.try_lock() else {
            return;
        };
        if rows.len() < rows.capacity() {
            let mut out = [0.0; ROW_WIDTH];
            out[0] = t;
            out[1..4].copy_from_slice(target);
            out[4..7].copy_from_slice(&translation(&state.O_T_EE_c));
            out[7..10].copy_from_slice(&translation(&state.O_T_EE));
            out[10..17].copy_from_slice(&state.q);
            out[17..23].copy_from_slice(&state.O_F_ext_hat_K);
            if let Some(sent) = sent {
                out[23..26].copy_from_slice(&sent.velocity);
                out[26..29].copy_from_slice(&sent.acceleration);
                out[29..36].copy_from_slice(&sent.q_goal);
                out[36..43].copy_from_slice(&sent.tau);
                out[43] = sent.ik_error;
                out[44] = sent.leash_alteration;
            }
            rows.push(out);
        }
    }

    fn write(&self, path: &str, impedance: bool) -> FrankaResult<()> {
        let rows = self.rows.lock().unwrap();
        let io_error =
            |e: std::io::Error| FrankaError::InvalidArgument(format!("--log {path}: {e}"));
        let mut out = std::io::BufWriter::new(std::fs::File::create(path).map_err(io_error)?);
        let width = if impedance { ROW_WIDTH } else { COMMON_WIDTH };
        let extra = if impedance { IMPEDANCE_COLUMNS } else { "" };
        writeln!(out, "{COLUMNS}{extra}").map_err(io_error)?;
        for row in rows.iter() {
            write!(out, "{:.4}", row[0]).map_err(io_error)?;
            for value in &row[1..width] {
                write!(out, ",{value:.9}").map_err(io_error)?;
            }
            writeln!(out).map_err(io_error)?;
        }
        out.flush().map_err(io_error)?;
        println!("wrote {} rows to {path}", rows.len());
        Ok(())
    }
}

/// Returns whether the motion was aborted by the robot (and recovered from).
fn run(args: &Args) -> FrankaResult<bool> {
    let robot = Arc::new(Robot::new(
        &args.hostname,
        common::realtime_config_from_env(),
    )?);
    common::set_default_behavior(&robot)?;
    // libfranka's example thresholds unless `--thresholds`: the shared 10 N default is
    // crossed at 0.25 m/s.
    let (torque, force) = match args.thresholds {
        Some(n) => ([n; 7], [n; 6]),
        None => (
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        ),
    };
    robot.set_collision_behavior(torque, torque, torque, torque, force, force, force, force)?;
    println!("Collision thresholds: joints {torque:?} Nm, Cartesian {force:?} N / Nm");
    let impedance = matches!(args.backend, Backend::Impedance(_));
    match args.mode {
        Mode::Bridged => {
            println!(
                "Mode: bridged, budget {:?} (m/s, m/s^2, m/s^3)",
                args.limits
            );
            match args.backend {
                Backend::Impedance(impedance) => println!(
                    "Backend: impedance, Cartesian stiffness {:?} N/m, damping {:?} N s/m, \
                     joint gains DROID's, feedforward {}, joint gains {}, leash {} m",
                    &impedance.gains.cartesian_stiffness[..3],
                    &impedance.gains.cartesian_damping[..3],
                    if impedance.velocity_feedforward {
                        "on"
                    } else {
                        "off"
                    },
                    if impedance.project_joint_gains {
                        "projected"
                    } else {
                        "unprojected"
                    },
                    impedance.leash.translation
                ),
                Backend::RobotController => {
                    println!("Backend: the robot's Cartesian impedance controller")
                }
            }
        }
        Mode::Raw => println!("Mode: raw, no generator, no rate limiting, expect a reflex"),
    }
    if args.rotate {
        println!(
            "Yaw sweep: +-{:.0} degrees about the base z, {YAW_PERIOD} s period",
            YAW_SWEEP.to_degrees()
        );
    }
    if args.stdin {
        println!("Reading targets `x y z` (metres, relative to the start pose) from stdin.");
    } else if !args.yes {
        common::wait_for_enter();
    }
    let log = Arc::new(Log::default());
    *log.rows.lock().unwrap() = Vec::with_capacity(LOG_CAPACITY * usize::from(args.log.is_some()));

    let started = Instant::now();
    let result = match args.mode {
        Mode::Bridged => run_bridged(&robot, args, Arc::clone(&log)),
        Mode::Raw => run_raw(&robot, args.stdin, &log),
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
    let cycles = log.cycles.load(Ordering::Relaxed);
    if impedance && args.mode == Mode::Bridged {
        println!(
            "{cycles} cycles, peak IK residual {:.2e} (m plus rad)",
            f64::from_bits(log.peak_ik_error.load(Ordering::Relaxed))
        );
    } else {
        println!(
            "{cycles} cycles, backstop bound (by more than 10 um) in {} of them",
            log.backstop_bound.load(Ordering::Relaxed)
        );
    }
    if let Some(path) = &args.log {
        log.write(path, impedance && args.mode == Mode::Bridged)?;
    }
    Ok(reflex)
}

/// The whole bridge: start, `set_position` from the commander (and `set_orientation` from
/// the yaw sweep), `stop`. The observer keeps the CSV and the counters; it runs on the
/// realtime thread and must not allocate.
fn run_bridged(robot: &Arc<Robot>, args: &Args, log: Arc<Log>) -> FrankaResult<()> {
    let observer = {
        let log = Arc::clone(&log);
        move |state: &RobotState, sent: &CartesianSent| {
            // Micrometres are the float32 echo's noise in the limiter's reference; count
            // the cycles in which the backstop really shaped the command.
            let bound = sent.backstop_alteration > 1e-5;
            log.backstop_bound
                .fetch_add(u64::from(bound), Ordering::Relaxed);
            if sent.ik_error > f64::from_bits(log.peak_ik_error.load(Ordering::Relaxed)) {
                log.peak_ik_error
                    .store(sent.ik_error.to_bits(), Ordering::Relaxed);
            }
            log.record(state, &sent.target, Some(sent));
        }
    };
    let options = TargetControlOptions::default()
        .with_limits(args.limits)
        .with_backend(args.backend)
        .with_max_deviation(MAX_DEVIATION)
        .with_observer(observer);
    let control = robot.start_cartesian_target_control(options)?;
    let start = control.target();
    println!("Start position: {start:.3?}; targets stay within +-{BOX} m of it.");
    let done = AtomicBool::new(false);
    std::thread::scope(|scope| {
        if args.rotate {
            scope.spawn(|| yaw_sweep(&control, &done));
        }
        run_commander(args.stdin, &mut |relative| {
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
            log.record(state, &goal, None);
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
