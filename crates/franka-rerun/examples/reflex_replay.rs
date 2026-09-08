//! Flight recorder demonstration: a slow joint motion with low collision thresholds, recorded
//! live, and the control log of the reflex that stops it replayed as a Rerun recording.
//!
//! ```text
//! reflex_replay <robot-hostname> [--yes] [--force N] [--torque NM] [--seconds S]
//!               [--live ADDR] [--out DIR]
//! ```
//!
//! What it does, in order:
//!
//! 1. Connects with a control log of 3000 cycles (`Robot::new_with_log_size`), so that a
//!    `ControlException` carries the last three seconds before a reflex.
//! 2. Sets the collision behaviour: `--torque` (default 10 Nm) on every joint and `--force`
//!    (default 10 N / Nm) on every Cartesian axis as the *upper* (collision) thresholds,
//!    acceleration and nominal, with the *lower* (contact) thresholds at half of that, so a
//!    push raises the contact flag first and the collision flag, which stops the motion, once
//!    it reaches the full threshold.
//! 3. Reads the state once and refuses to run if joints 4 or 6 have less than 0.45 rad of
//!    margin to their upper limit (the motion adds up to 0.4 rad to both).
//! 4. Starts a [`franka_rerun::Recorder`]: to `<out>/run.rrd`, or with `--live ADDR` to a
//!    viewer already listening there (`rerun` listens on 9876).
//! 5. Prints all of the above and waits for Enter, unless `--yes`.
//! 6. Runs a joint-position motion: joints 4 and 6 follow `0.2 (1 - cos(2 pi t / 12))`, i.e.
//!    they swing from the start position to +0.4 rad and back with a 12 s period, seeded from
//!    `q_d` (what an FER needs; correct on an FR3 too). It runs until the first multiple of
//!    12 s at or after `--seconds` (default 60), so that it ends at rest. Every cycle pushes
//!    the state and the sent command to the recorder from inside the callback.
//! 7. Pushing the arm hard enough to cross a threshold trips a reflex. The example prints the
//!    exception's message and its reflex names, writes `<out>/reflex.rrd`
//!    (`franka_rerun::replay_exception`) and `<out>/reflex.json` (`save_records`), and calls
//!    `automatic_error_recovery()`.
//! 8. Finishes the recorder (which flushes `run.rrd` when recording to a file) and prints
//!    what it counted, including dropped records.
//!
//! `FRANKA_REALTIME=ignore|enforce` picks `RealtimeConfig` like the franka-rs examples.
//!
//! # Warning
//! The arm moves, and it is meant to be pushed. Have the user stop at hand.

use std::path::PathBuf;

use franka::{
    motion_finished, ControllerMode, FrankaError, JointPositions, RealtimeConfig, Robot,
    RobotCommandLog, DEFAULT_CUTOFF_FREQUENCY,
};
use franka_rerun::{flight, Recorder, RecorderOptions, RobotKind};

/// Period of the swing, s.
const PERIOD: f64 = 12.0;
/// Peak deflection of joints 4 and 6 from the start position, rad.
const SWING: f64 = 0.4;
/// Upper joint limits of joints 4 and 6 on both robots (libfranka's `q_max`), rad.
const Q4_MAX: f64 = -0.0698;
const Q6_MAX: f64 = 3.7525;
/// Margin the start position must leave below those limits after the swing, rad.
const LIMIT_MARGIN: f64 = 0.05;

struct Args {
    hostname: String,
    yes: bool,
    force: f64,
    torque: f64,
    seconds: f64,
    live: Option<String>,
    out: PathBuf,
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut parsed = Args {
        hostname: String::new(),
        yes: false,
        force: 10.0,
        torque: 10.0,
        seconds: 60.0,
        live: None,
        out: PathBuf::from("."),
    };
    let mut i = 0;
    let value = |i: &mut usize, flag: &str| {
        *i += 1;
        args.get(*i)
            .cloned()
            .ok_or_else(|| format!("{flag} needs a value"))
    };
    let number = |text: String, flag: &str| {
        text.parse::<f64>()
            .ok()
            .filter(|v| *v > 0.0 && v.is_finite())
            .ok_or_else(|| format!("{flag} {text:?}: want a positive number"))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--yes" => parsed.yes = true,
            "--force" => parsed.force = number(value(&mut i, "--force")?, "--force")?,
            "--torque" => parsed.torque = number(value(&mut i, "--torque")?, "--torque")?,
            "--seconds" => parsed.seconds = number(value(&mut i, "--seconds")?, "--seconds")?,
            "--live" => parsed.live = Some(value(&mut i, "--live")?),
            "--out" => parsed.out = PathBuf::from(value(&mut i, "--out")?),
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
    Ok(parsed)
}

/// `RealtimeConfig` from `FRANKA_REALTIME`, like `examples/common/mod.rs` in franka-rs.
fn realtime_config_from_env() -> RealtimeConfig {
    let config = match std::env::var("FRANKA_REALTIME").as_deref() {
        Ok("ignore") => RealtimeConfig::Ignore,
        Ok("enforce") | Err(_) => RealtimeConfig::Enforce,
        Ok(other) => {
            eprintln!(
                "FRANKA_REALTIME={other:?} is neither \"ignore\" nor \"enforce\"; enforcing."
            );
            RealtimeConfig::Enforce
        }
    };
    println!("RealtimeConfig: {config:?}");
    config
}

fn main() {
    let args = match parse_args() {
        Ok(args) => args,
        Err(e) => {
            eprintln!(
                "{e}\nUsage: reflex_replay <robot-hostname> [--yes] [--force N] [--torque NM] \
                 [--seconds S] [--live ADDR] [--out DIR]"
            );
            std::process::exit(2);
        }
    };
    if let Err(e) = run(&args) {
        eprintln!("reflex_replay: {e}");
        std::process::exit(1);
    }
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let robot = Robot::new_with_log_size(&args.hostname, realtime_config_from_env(), 3000)?;
    let kind = RobotKind::from(robot.fci_version());
    println!("Connected: FCI {:?} ({kind:?})", robot.fci_version());

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Contact (the lower thresholds) at half the collision (upper) thresholds, so the replay
    // shows the amber contact flags before the red collision that stops the motion.
    let (torque, force) = ([args.torque; 7], [args.force; 6]);
    let (contact_torque, contact_force) = ([args.torque / 2.0; 7], [args.force / 2.0; 6]);
    robot.set_collision_behavior(
        contact_torque,
        torque,
        contact_torque,
        torque,
        contact_force,
        force,
        contact_force,
        force,
    )?;

    let start = robot.read_once()?;
    for (joint, limit) in [(3, Q4_MAX), (5, Q6_MAX)] {
        let margin = limit - (start.q_d[joint] + SWING);
        if margin < LIMIT_MARGIN {
            return Err(format!(
                "joint {} is at {:.3} rad; the swing to {:.3} rad would come within {:.3} rad \
                 of its limit {limit:.3}. Move the arm to the ready pose first.",
                joint + 1,
                start.q_d[joint],
                start.q_d[joint] + SWING,
                margin.max(0.0)
            )
            .into());
        }
    }
    let duration = (args.seconds / PERIOD).ceil().max(1.0) * PERIOD;

    let model = robot.load_model()?;
    let options = RecorderOptions::default();
    let run_path = args.out.join("run.rrd");
    let recorder = match &args.live {
        Some(addr) => Recorder::to_viewer(addr, model, kind, options.clone())?,
        None => Recorder::to_file(&run_path, model, kind, options.clone())?,
    };
    // A second model for the replay: the recorder owns the first on its thread.
    let model = robot.load_model()?;

    println!(
        "Collision thresholds: {} Nm on every joint, {} N/Nm on every Cartesian axis,",
        args.torque, args.force
    );
    println!("  as collision thresholds; contact thresholds at half of that.");
    println!(
        "Motion: joints 4 and 6 swing +{SWING} rad and back, {PERIOD} s period, for {duration} s."
    );
    match &args.live {
        Some(addr) => println!("Recording live to the viewer at {addr}."),
        None => println!("Recording live to {}.", run_path.display()),
    }
    println!(
        "On a reflex: {} and {} are written, then automatic error recovery.",
        args.out.join("reflex.rrd").display(),
        args.out.join("reflex.json").display()
    );
    println!("WARNING: This example will move the robot, and you are meant to push it. Have the user stop at hand!");
    if args.yes {
        println!("--yes given, starting.");
    } else {
        println!("Press Enter to continue...");
        let mut discard = String::new();
        let _ = std::io::stdin().read_line(&mut discard);
    }

    let mut initial = [0.0; 7];
    let mut time = 0.0;
    let result = robot.control_joint_positions(
        |state, period| {
            time += period.as_secs_f64();
            if time == 0.0 {
                // From `q_d`: on FCI v5 the first setpoint is rate limited against the robot's
                // own `q_d`, so a `q` seed implies a commanded velocity (see the book, "The
                // q_d seed rule"). Harmless on an FR3.
                initial = state.q_d;
            }
            let delta = 0.5 * SWING * (1.0 - (2.0 * std::f64::consts::PI * time / PERIOD).cos());
            let mut q = initial;
            q[3] += delta;
            q[5] += delta;
            let command = RobotCommandLog {
                q_c: q,
                ..RobotCommandLog::default()
            };
            recorder.push(state, Some(command));
            let output = JointPositions::new(q);
            if time >= duration {
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    );

    match result {
        Ok(()) => println!("\nMotion finished after {time:.1} s without a reflex."),
        Err(FrankaError::Control(e)) => {
            println!("\nControlException after {time:.1} s: {}", e.message);
            println!("reflex: {:?}", e.last_motion_errors.names());
            println!("move status: {:?}", e.move_status);
            let reflex_rrd = args.out.join("reflex.rrd");
            let summary = flight::replay_exception(&reflex_rrd, &e, &model, kind, &options.flight)?;
            println!(
                "{} ({} records):\n{summary}",
                reflex_rrd.display(),
                e.log.len()
            );
            let reflex_json = args.out.join("reflex.json");
            flight::save_records(&reflex_json, &e.log)?;
            println!("{}: written", reflex_json.display());
            robot.automatic_error_recovery()?;
            println!("Automatic error recovery done.");
        }
        Err(e) => return Err(e.into()),
    }

    let stats = recorder.finish()?;
    println!(
        "Recorder: {} pushed, {} dropped\n{}",
        stats.pushed, stats.dropped, stats.summary
    );
    if args.live.is_none() {
        println!("{}: written", run_path.display());
    }
    Ok(())
}
