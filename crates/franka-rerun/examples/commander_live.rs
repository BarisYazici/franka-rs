//! The non-realtime commander of franka-rs's `examples/nonrealtime_commander.rs`, streamed
//! live into a Rerun viewer: the raw staircase target arriving, the robot refusing it
//! (`--raw`) or the trajectory generator's command following it (`--bridged`), the arm
//! moving, the derivatives of the sent position against the robot's limits, the events as
//! they happen.
//!
//! ```text
//! commander_live <robot-hostname> (--live ADDR | --out FILE) [--bridged | --raw] [--stdin]
//!                [--budget V,A,J] [--controller joint|cartesian] [--meshes DIR] [--yes]
//! ```
//!
//! The control side is the original's: a scripted commander thread publishes targets (steps
//! of 5 cm with irregular holds, a 2 s stall, a burst of 20 targets in 100 ms) into a seqlock
//! slot; the 1 kHz callback reads it and hands `start pose + target` to
//! `control_cartesian_pose`. In `--bridged` mode (the default) the crate's online trajectory
//! generator (`CartesianOtg`) re-plans, every cycle, a time-optimal jerk-limited profile from
//! the commanded state to rest at the latest target and follows it for one cycle (a 5 cm step
//! becomes an S-curve peaking at 0.15 m/s that lands after 0.66 s), and the Cartesian rate
//! limiter runs after it as the backstop, both under a small budget (0.3 m/s, 0.5 m/s^2,
//! 20 m/s^3; `--budget V,A,J`). In `--raw` mode the pose goes out as is, so the first 5 cm
//! step reaches the robot as a 50 m/s jump and its motion generator refuses it with a reflex;
//! the example prints the robot's error text and runs `automatic_error_recovery()`.
//! `--controller joint|cartesian` picks the robot's internal controller (Cartesian impedance
//! by default). The start pose is anchored on the first control cycle, the motion ends when
//! the commander is done and the command has settled, or when the measured end effector
//! strays 30 cm from the start. `--stdin` replaces the script with lines of `x y z` (metres,
//! relative to the start).
//!
//! What is new is where it all goes. Every cycle the callback pushes the state and the pose
//! it sent to a [`franka_rerun::Recorder`] (`--live ADDR` streams to a viewer already
//! listening there, `rerun --port 9876` say; `--out FILE` writes an `.rrd`; both together
//! do both) -- one `push`, no Rerun call and no allocation on the realtime thread -- and
//! its background thread draws the flight recording: the arm (with `--meshes DIR`, Franka's
//! link meshes), `ee/position/{x,y,z}` sent against measured, `ee/derivatives/{speed,
//! acceleration,jerk}` against the limits. The commander thread logs its own side at the
//! moment it publishes, stamped with the robot time the callback keeps in an atomic:
//! `commander/target/{x,y,z}` (the staircase, absolute), `commander/target_speed` (the
//! step over one 1 ms cycle -- 50 m/s -- and over the time since the previous target),
//! `world/target`, and an `events` line per step, stall and burst. The original's CSV
//! `--log` is not here; the recording carries all of it.
//!
//! `FRANKA_REALTIME=ignore` runs it against franka-sim on an ordinary kernel.
//!
//! # Warning
//! The end effector moves inside a 24 cm cube around wherever it is when the example starts;
//! make sure that cube is free and have the user stop button at hand. `--raw` provokes a
//! reflex on purpose.

#[path = "commander_live/script.rs"]
mod script;
#[path = "commander_live/sink.rs"]
mod sink;

use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use franka::{
    limit_rate_cartesian_pose, CartesianOtg, CartesianPose, ControllerMode, Duration, FrankaError,
    FrankaResult, OtgLimits, RealtimeConfig, Robot, RobotCommandLog, RobotState, DELTA_T,
    MAX_CUTOFF_FREQUENCY,
};
use franka_rerun::{distance, flight, Recorder, RecorderOptions, RobotKind, TIMELINE};
use rerun::{TextLog, TextLogLevel};
use script::{TargetSlot, BOX};
use sink::{log_target, Clock, LiveCommander};

/// Measured deviation from the start position at which the loop gives up, m.
const MAX_DEVIATION: f64 = 0.30;
/// The default `--bridged` budget, shared by the trajectory generator and the rate limiter:
/// translational m/s, m/s^2, m/s^3, then rotational rad/s, rad/s^2, rad/s^3 (the orientation
/// never changes here); see the original example for how it was chosen on a real FER.
const BRIDGE_LIMITS: [f64; 6] = [0.3, 0.5, 20.0, 1.0, 5.0, 500.0];
/// The commanded pose counts as settled when it stays within `SETTLE_TOLERANCE` metres of the
/// target for `SETTLE_CYCLES` consecutive cycles.
const SETTLE_TOLERANCE: f64 = 1e-3;
const SETTLE_CYCLES: u32 = 250;
/// One control cycle, s: the raw target's step is a jump over this.
pub const CYCLE: f64 = 1e-3;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Bridged,
    Raw,
}

struct Args {
    hostname: String,
    mode: Mode,
    from_stdin: bool,
    yes: bool,
    limits: [f64; 6],
    controller: ControllerMode,
    live: Option<String>,
    out: Option<PathBuf>,
    meshes: Option<PathBuf>,
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut parsed = Args {
        hostname: String::new(),
        mode: Mode::Bridged,
        from_stdin: false,
        yes: false,
        limits: BRIDGE_LIMITS,
        controller: ControllerMode::CartesianImpedance,
        live: None,
        out: None,
        meshes: None,
    };
    let mut i = 0;
    let value = |i: &mut usize, flag: &str| {
        *i += 1;
        args.get(*i)
            .cloned()
            .ok_or_else(|| format!("{flag} needs a value"))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--bridged" => parsed.mode = Mode::Bridged,
            "--raw" => parsed.mode = Mode::Raw,
            "--stdin" => parsed.from_stdin = true,
            "--yes" => parsed.yes = true,
            "--live" => parsed.live = Some(value(&mut i, "--live")?),
            "--out" => parsed.out = Some(PathBuf::from(value(&mut i, "--out")?)),
            "--meshes" => parsed.meshes = Some(PathBuf::from(value(&mut i, "--meshes")?)),
            "--budget" => {
                let text = value(&mut i, "--budget")?;
                let parsed_budget: Vec<f64> =
                    text.split(',').filter_map(|v| v.parse().ok()).collect();
                match parsed_budget[..] {
                    [v, a, j] => parsed.limits[..3].copy_from_slice(&[v, a, j]),
                    _ => return Err(format!("--budget {text:?}: want V,A,J")),
                }
            }
            "--controller" => {
                parsed.controller = match value(&mut i, "--controller")?.as_str() {
                    "joint" => ControllerMode::JointImpedance,
                    "cartesian" => ControllerMode::CartesianImpedance,
                    other => return Err(format!("--controller {other:?}: want joint|cartesian")),
                }
            }
            other if parsed.hostname.is_empty() && !other.starts_with('-') => {
                parsed.hostname = other.to_string()
            }
            other => return Err(format!("unexpected argument {other:?}")),
        }
        i += 1;
    }
    if parsed.hostname.is_empty() {
        return Err("no robot hostname given".into());
    }
    if parsed.live.is_none() && parsed.out.is_none() {
        return Err("give --live ADDR (a listening viewer) and/or --out FILE (an .rrd)".into());
    }
    Ok(parsed)
}

/// `RealtimeConfig` from `FRANKA_REALTIME`, like the franka-rs examples.
fn realtime_config_from_env() -> RealtimeConfig {
    match std::env::var("FRANKA_REALTIME").as_deref() {
        Ok("ignore") => RealtimeConfig::Ignore,
        Ok("enforce") | Err(_) => RealtimeConfig::Enforce,
        Ok(other) => {
            eprintln!(
                "FRANKA_REALTIME={other:?} is neither \"ignore\" nor \"enforce\"; enforcing."
            );
            RealtimeConfig::Enforce
        }
    }
}

fn main() {
    let args = match parse_args() {
        Ok(args) => args,
        Err(e) => {
            eprintln!(
                "{e}\nUsage: commander_live <robot-hostname> (--live ADDR | --out FILE) \
                 [--bridged | --raw] [--stdin] [--budget V,A,J] [--controller joint|cartesian] \
                 [--meshes DIR] [--yes]"
            );
            std::process::exit(2);
        }
    };
    match run(&args) {
        // In `--raw` mode the reflex is the demonstration; in `--bridged` mode it is a failure.
        Ok(reflex) => std::process::exit(i32::from(reflex && args.mode != Mode::Raw)),
        Err(e) => {
            eprintln!("commander_live: {e}");
            std::process::exit(1);
        }
    }
}

fn translation(pose: &[f64; 16]) -> [f64; 3] {
    [pose[12], pose[13], pose[14]]
}

/// `limit_rate_cartesian_pose` under the budget, against the robot's echo of its last command.
#[rustfmt::skip]
fn backstop(l: &[f64; 6], pose: &[f64; 16], s: &RobotState) -> FrankaResult<[f64; 16]> {
    limit_rate_cartesian_pose(l[0], l[1], l[2], l[3], l[4], l[5], pose,
                              &s.O_T_EE_c, &s.O_dP_EE_c, &s.O_ddP_EE_c)
}

/// Returns whether the motion was aborted by the robot (and recovered from).
fn run(args: &Args) -> Result<bool, Box<dyn std::error::Error>> {
    let robot = Robot::new(&args.hostname, realtime_config_from_env())?;
    let kind = RobotKind::from(robot.fci_version());
    println!("Connected: FCI {:?} ({kind:?})", robot.fci_version());
    // libfranka's example impedances and its current example collision thresholds.
    robot.set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])?;
    robot.set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])?;
    let torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0];
    let force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0];
    robot.set_collision_behavior(torque, torque, torque, torque, force, force, force, force)?;

    let model = robot.load_model()?;
    let options = RecorderOptions {
        flight: flight::FlightOptions {
            every: 10,
            meshes: args.meshes.clone(),
            ..flight::FlightOptions::default()
        },
        ..RecorderOptions::default()
    };
    let recorder = match (&args.live, &args.out) {
        (Some(addr), Some(path)) => Recorder::to_viewer_and_file(addr, path, model, kind, options)?,
        (Some(addr), None) => Recorder::to_viewer(addr, model, kind, options)?,
        (None, Some(path)) => Recorder::to_file(path, model, kind, options)?,
        (None, None) => unreachable!("parse_args requires one"),
    };
    let rec = recorder.stream();
    flight::send_commander_blueprint(&rec)?;
    flight::log_target_styles(&rec)?;

    let (limit_rate, name) = match args.mode {
        Mode::Bridged => (
            true,
            "bridged: generator, then rate limiting within the budget",
        ),
        Mode::Raw => (
            false,
            "raw: no generator, no rate limiting, expect a reflex",
        ),
    };
    println!("Mode: {name}\nController: {:?}", args.controller);
    if args.mode == Mode::Bridged {
        println!("Budget: {:?} m/s, m/s^2, m/s^3", &args.limits[..3]);
    }
    if let Some(addr) = &args.live {
        println!("Streaming to the viewer at {addr}.");
    }
    if let Some(path) = &args.out {
        println!("Recording to {}.", path.display());
    }
    if args.from_stdin {
        println!("Reading targets `x y z` (metres, relative to the start pose) from stdin.");
    } else if !args.yes {
        println!("WARNING: This example will move the robot! Have the user stop button at hand!");
        println!("Press Enter to continue...");
        let mut discard = String::new();
        let _ = std::io::stdin().read_line(&mut discard);
    }

    // Everything is relative to the pose the robot is *commanded* to be in right now, which
    // is also what the first setpoint must equal on an FER (see `control_cartesian_pose`).
    // Re-anchored on the first control cycle below: on a real robot `O_T_EE_c` drifts by
    // micrometres between this read and the first cycle, which `--raw` would send as a jump.
    let initial = robot.read_once()?;
    let mut start_pose = initial.O_T_EE_c;
    let mut start = translation(&start_pose);
    println!("Start position: {start:.3?}; targets stay within +-{BOX} m of it.");
    let clock = Arc::new(Clock::default());
    clock
        .time_ms
        .store(initial.time.as_millis(), Ordering::Relaxed);
    clock.set_start(&start);
    log_target(&rec, clock.now(), Some(&start), [0.0; 2]);
    let [max_velocity, max_acceleration, max_jerk, ..] = args.limits;
    #[rustfmt::skip]
    let otg_limits = OtgLimits { max_velocity, max_acceleration, max_jerk }.per_axis_for_norm(3);
    let mut otg = CartesianOtg::new(start, otg_limits, true)?; // synchronised axes

    let slot = Arc::new(TargetSlot::default());
    let mut commander = LiveCommander {
        rec: rec.clone(),
        clock: Arc::clone(&clock),
        last: start,
        last_time: clock.now(),
    };
    let (commander_slot, from_stdin) = (Arc::clone(&slot), args.from_stdin);
    std::thread::spawn(move || {
        franka_rerun::recorder::drop_realtime_priority();
        match from_stdin {
            true => script::run_stdin(&commander_slot, &mut commander),
            false => script::run_script(&commander_slot, &mut commander),
        }
    });

    let mut time = 0.0;
    let mut target = [0.0f64; 3];
    let mut last_commanded = start;
    let mut peak_speed = 0.0f64;
    let mut torn_reads = 0u64;
    let mut settled_cycles = 0u32;
    let mut deviated = false;
    let (mode, limits) = (args.mode, args.limits);

    // Only the control thread runs realtime; the recorder, the Rerun SDK's own threads and
    // the commander were spawned above and stay on the normal scheduler.
    if !franka_rerun::recorder::raise_realtime_priority(80) {
        eprintln!("warning: could not raise the control thread to SCHED_FIFO 80");
    }
    let result = robot.control_cartesian_pose(
        |state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            clock
                .time_ms
                .store(state.time.as_millis(), Ordering::Relaxed);
            let commanded = translation(&state.O_T_EE_c);
            let measured = translation(&state.O_T_EE);
            let dt = period.as_secs_f64().max(CYCLE);
            peak_speed = peak_speed.max(distance(&commanded, &last_commanded) / dt);
            last_commanded = commanded;

            // The first setpoint is always the start pose: on an FR3 the first command is its
            // own filter reference, so a target that arrived before this cycle would go out
            // unfiltered and unlimited -- exactly the jump this example exists to prevent.
            if time == 0.0 {
                start_pose = state.O_T_EE_c;
                start = translation(&start_pose);
                clock.set_start(&start);
                last_commanded = start;
                otg.reset(start);
                target = [0.0; 3];
            } else if !deviated && distance(&measured, &start) > MAX_DEVIATION {
                // Freeze the target where the command is; the generator brings it to rest.
                deviated = true;
                target = std::array::from_fn(|i| commanded[i] - start[i]);
            } else if !deviated && !slot.load(&mut target) {
                torn_reads += 1;
            }
            let goal: [f64; 3] = std::array::from_fn(|i| start[i] + target[i]);
            let mut pose = start_pose;
            pose[12..15].copy_from_slice(&goal);
            if mode == Mode::Bridged {
                // The three rules from `franka::otg`: limits that stay inside the norm budget
                // (above), re-anchor on the robot's echo of the last command, and advance by
                // one nominal cycle. Then rate-limit against the echo as the backstop; the
                // calls only fail on non-finite input, and holding is then safe.
                let _ = otg.set_position(commanded);
                let _ = otg.set_target(goal);
                pose[12..15].copy_from_slice(&otg.step(DELTA_T));
                pose = backstop(&limits, &pose, state).unwrap_or(state.O_T_EE_c);
            }
            // The one thing the realtime thread does for the recording: a copy into a ring.
            let command = RobotCommandLog {
                O_T_EE_c: pose,
                ..RobotCommandLog::default()
            };
            recorder.push(state, Some(command));

            // Under the generator the command lands on the target and stays; raw, at once.
            let commander_done = deviated || slot.finished.load(Ordering::SeqCst);
            if commander_done && distance(&commanded, &goal) < SETTLE_TOLERANCE {
                settled_cycles += 1;
            } else {
                settled_cycles = 0;
            }
            let mut output = CartesianPose::new(pose);
            output.motion_finished = settled_cycles >= SETTLE_CYCLES;
            output
        },
        args.controller,
        limit_rate,
        MAX_CUTOFF_FREQUENCY,
    );

    let reflex = match result {
        Ok(()) => {
            println!("\nMotion finished after {time:.3} s.");
            false
        }
        Err(FrankaError::Control(exception)) => {
            println!("\nThe robot aborted the motion after {time:.3} s. Its error text:\n");
            println!("{}\n", exception.message.trim_end());
            println!("reflex reasons: {}", exception.last_motion_errors);
            let text = format!("motion aborted: {}", exception.last_motion_errors);
            rec.set_duration_secs(TIMELINE, clock.now());
            rec.log(
                "events",
                &TextLog::new(text).with_level(TextLogLevel::ERROR),
            )?;
            robot.automatic_error_recovery()?;
            println!("automatic_error_recovery() succeeded; the robot is ready again.");
            true
        }
        Err(other) => return Err(other.into()),
    };
    // Extend the staircase to the end of the recording.
    let start = clock.start();
    let _ = slot.load(&mut target);
    let last: [f64; 3] = std::array::from_fn(|k| start[k] + target[k]);
    log_target(&rec, clock.now(), Some(&last), [0.0; 2]);

    println!("peak commanded speed {peak_speed:.3} m/s, torn slot reads {torn_reads}");
    if deviated {
        println!("Stopped early: the end effector strayed {MAX_DEVIATION} m from the start.");
    }
    let stats = recorder.finish()?;
    println!(
        "Recorder: {} pushed, {} dropped\n{}",
        stats.pushed, stats.dropped, stats.summary
    );
    if let Some(path) = &args.out {
        println!("{}: written", path.display());
    }
    Ok(reflex)
}
