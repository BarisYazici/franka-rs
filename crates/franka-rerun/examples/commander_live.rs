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
//! In `--bridged` mode (the default) the control side is the crate's target control:
//! `Robot::start_cartesian_target_control` runs the loop on its own thread under a small
//! budget (0.3 m/s, 0.5 m/s^2, 20 m/s^3; `--budget V,A,J`), and the scripted commander (steps
//! of 5 cm with irregular holds, a 2 s stall, a burst of 20 targets in 100 ms; `--stdin` for
//! lines of `x y z`, metres relative to the start) calls `set_position` from the main thread.
//! In `--raw` mode the pose goes to `control_cartesian_pose` as is, so the first 5 cm step
//! reaches the robot as a 50 m/s jump and its motion generator refuses it with a reflex; the
//! example prints the robot's error text and runs `automatic_error_recovery()`.
//! `--controller joint|cartesian` picks the robot's internal controller.
//!
//! Every cycle the loop's observer (or the raw callback) pushes the state and the pose it
//! sent to a [`franka_rerun::Recorder`] (`--live ADDR` streams to a viewer already listening
//! there, `rerun --port 9876` say; `--out FILE` writes an `.rrd`; both together do both) --
//! one `push`, no Rerun call and no allocation on the realtime thread -- and its background
//! thread draws the flight recording: the arm (with `--meshes DIR`, Franka's link meshes),
//! `ee/position/{x,y,z}` sent against measured, `ee/derivatives/{speed,acceleration,jerk}`
//! against the limits. The commander logs its own side at the moment it publishes, stamped
//! with the robot time the observer keeps in an atomic: `commander/target/{x,y,z}`,
//! `commander/target_speed`, `world/target`, and an `events` line per step, stall and burst.
//!
//! `FRANKA_REALTIME=ignore` runs it against franka-sim on an ordinary kernel; the loop thread
//! is raised to `SCHED_FIFO` 80 either way (`TargetControlOptions::with_realtime_priority`),
//! fatal only under `enforce`.
//!
//! # Warning
//! The end effector moves inside a 24 cm cube around wherever it is when the example starts;
//! make sure that cube is free and have the user stop button at hand. `--raw` provokes a
//! reflex on purpose.

#[path = "commander_live/args.rs"]
mod args;
#[path = "commander_live/raw.rs"]
mod raw;
#[path = "commander_live/script.rs"]
mod script;
#[path = "commander_live/sink.rs"]
mod sink;

use std::sync::atomic::Ordering;
use std::sync::Arc;

use args::{Args, Mode};
use franka::{
    CartesianSent, FrankaError, FrankaResult, Robot, RobotCommandLog, RobotState,
    TargetControlOptions,
};
use franka_rerun::{flight, Recorder, RecorderOptions, RobotKind, TIMELINE};
use rerun::{TextLog, TextLogLevel};
use script::BOX;
use sink::{log_target, Clock, LiveCommander};

/// Measured deviation from the start position at which the loop gives up, m.
const MAX_DEVIATION: f64 = 0.30;
/// One control cycle, s: the raw target's step is a jump over this.
pub const CYCLE: f64 = 1e-3;

fn main() {
    let args = match args::parse() {
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

/// Returns whether the motion was aborted by the robot (and recovered from).
fn run(args: &Args) -> Result<bool, Box<dyn std::error::Error>> {
    let robot = Arc::new(Robot::new(
        &args.hostname,
        args::realtime_config_from_env(),
    )?);
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
    let recorder = Arc::new(match (&args.live, &args.out) {
        (Some(addr), Some(path)) => Recorder::to_viewer_and_file(addr, path, model, kind, options)?,
        (Some(addr), None) => Recorder::to_viewer(addr, model, kind, options)?,
        (None, Some(path)) => Recorder::to_file(path, model, kind, options)?,
        (None, None) => unreachable!("parse_args requires one"),
    });
    let rec = recorder.stream();
    flight::send_commander_blueprint(&rec)?;
    flight::log_target_styles(&rec)?;
    match args.mode {
        Mode::Bridged => println!("Mode: bridged, budget {:?}", args.limits),
        Mode::Raw => println!("Mode: raw, no generator, no rate limiting, expect a reflex"),
    }
    println!("Controller: {:?}", args.controller);
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
        let _ = std::io::stdin().read_line(&mut String::new());
    }

    let clock = Arc::new(Clock::default());
    let result = match args.mode {
        Mode::Bridged => run_bridged(args, &robot, &recorder, &clock),
        Mode::Raw => raw::run(args, &robot, &recorder, &clock),
    };
    let reflex = match result {
        Ok(()) => {
            println!("\nMotion finished at robot time {:.3} s.", clock.now());
            false
        }
        Err(FrankaError::Control(exception)) => {
            println!("\nThe robot aborted the motion. Its error text:\n");
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
    // The loop thread has ended, so the observer's clone of the recorder is gone.
    let recorder = Arc::try_unwrap(recorder).map_err(|_| "the recorder is still shared")?;
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

/// The commander's view of the recording, with the staircase started and ended at `start`
/// plus the last target so that it spans the whole run.
fn commander(recorder: &Recorder, clock: &Arc<Clock>, start: [f64; 3]) -> LiveCommander {
    clock.set_start(&start);
    log_target(&recorder.stream(), clock.now(), Some(&start), [0.0; 2]);
    LiveCommander {
        rec: recorder.stream(),
        clock: Arc::clone(clock),
        last: start,
        last_time: clock.now(),
    }
}

/// The whole bridge: the crate's target control with the recorder in its observer, and the
/// commander on this thread calling `set_position`.
fn run_bridged(
    args: &Args,
    robot: &Arc<Robot>,
    recorder: &Arc<Recorder>,
    clock: &Arc<Clock>,
) -> FrankaResult<()> {
    let (observer_recorder, observer_clock) = (Arc::clone(recorder), Arc::clone(clock));
    let options = TargetControlOptions::default()
        .with_limits(args.limits)
        .with_controller_mode(args.controller)
        .with_max_deviation(MAX_DEVIATION)
        .with_realtime_priority(Some(80))
        .with_observer(move |state: &RobotState, sent: &CartesianSent| {
            // The one thing the realtime thread does for the recording: a copy into a ring.
            observer_clock
                .time_ms
                .store(state.time.as_millis(), Ordering::Relaxed);
            let command = RobotCommandLog {
                O_T_EE_c: sent.pose,
                ..RobotCommandLog::default()
            };
            observer_recorder.push(state, Some(command));
        });
    let control = robot.start_cartesian_target_control(options)?;
    let start = control.target();
    println!("Start position: {start:.3?}; targets stay within +-{BOX} m of it.");
    let mut sink = commander(recorder, clock, start);
    let mut publish = |relative: [f64; 3]| {
        let absolute = std::array::from_fn(|i| start[i] + relative[i]);
        control
            .set_position(absolute)
            .map_err(|e| eprintln!("commander: {e}"))
            .is_ok()
    };
    match args.from_stdin {
        true => script::run_stdin(&mut publish, &mut sink),
        false => script::run_script(&mut publish, &mut sink),
    }
    let last = sink.last;
    let result = control.stop();
    log_target(&recorder.stream(), clock.now(), Some(&last), [0.0; 2]);
    result
}
