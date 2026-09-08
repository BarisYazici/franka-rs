//! `franka-rerun`: replays franka-rs logs as Rerun recordings.
//!
//! * `franka-rerun csv <log.csv> --robot fr3|fer` turns a `nonrealtime_commander --log` CSV
//!   into an `.rrd` with the positions, the derivatives of the commanded position against the
//!   robot's limits, the commander's events and a 3D replay of the arm.
//! * `franka-rerun log <records.json> --robot fr3|fer` turns a control log saved with
//!   `franka_rerun::save_records` (the `Vec<franka::Record>` a `ControlException` carries)
//!   into a flight recording: contact and collision flags, external wrench, commanded versus
//!   measured, errors, the arm.
//!
//! Open the result with `rerun out.rrd`.

use std::path::PathBuf;
use std::process::ExitCode;

use franka::Model;
use franka_rerun::{commander, flight, CommanderLog, FlightOptions, Meshes, RobotKind};

/// The FR3 URDF the crate's tests use, found relative to this crate inside the repository.
const FR3_URDF: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../franka-rs/tests/data/fr3.urdf"
);

const USAGE: &str = "Usage: franka-rerun csv <log.csv> --robot fr3|fer [--urdf PATH] [-o out.rrd] \
                     [--every N] [--meshes DIR]\n       franka-rerun log <records.json> --robot \
                     fr3|fer [--urdf PATH] [-o out.rrd] [--every N] [--meshes DIR] \
                     [--force-scale M] [--noise-floor NM]\n\n  csv           replay a \
                     nonrealtime_commander --log CSV\n  log           replay a control log \
                     saved with franka_rerun::save_records (JSON)\n  --robot fr3   FR3 limits, \
                     model from --urdf (default: the repository's tests/data/fr3.urdf)\n  \
                     --robot fer   FER limits, the crate's built-in FER model\n  -o PATH       \
                     output recording (default: the input with .rrd)\n  --every N     log every \
                     N-th row to the 3D scene (default 1)\n  --meshes DIR  draw the arm with \
                     the link meshes in DIR (link0..7, hand, finger as .glb; see \
                     tools/franka-meshes)\n  --force-scale M  log only: metres of arrow per \
                     newton of external force (default 0.01)\n  --noise-floor NM  log only: \
                     external joint torques below NM count as zero for the contact estimate \
                     (default 1)";

struct Args {
    input: PathBuf,
    robot: RobotKind,
    urdf: Option<PathBuf>,
    out: PathBuf,
    every: usize,
    meshes: Option<PathBuf>,
    force_scale: f64,
    noise_floor: f64,
}

/// A positive, finite number after `flag`.
fn positive(flag: &str, text: &str) -> Result<f64, String> {
    text.parse::<f64>()
        .ok()
        .filter(|&s| s > 0.0 && s.is_finite())
        .ok_or_else(|| format!("{flag} {text:?}: want a positive number"))
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let (mut input, mut robot, mut urdf, mut out, mut meshes) = (None, None, None, None, None);
    let defaults = FlightOptions::default();
    let (mut every, mut force_scale) = (1, defaults.force_scale);
    let mut noise_floor = defaults.contact.noise_floor;
    let mut i = 0;
    let value = |i: &mut usize, flag: &str| {
        *i += 1;
        args.get(*i)
            .cloned()
            .ok_or_else(|| format!("{flag} needs a value\n{USAGE}"))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--robot" => {
                robot = Some(match value(&mut i, "--robot")?.as_str() {
                    "fr3" => RobotKind::Fr3,
                    "fer" => RobotKind::Fer,
                    other => return Err(format!("--robot {other:?}: want fr3 or fer\n{USAGE}")),
                })
            }
            "--urdf" => urdf = Some(PathBuf::from(value(&mut i, "--urdf")?)),
            "-o" | "--out" => out = Some(PathBuf::from(value(&mut i, "-o")?)),
            "--meshes" => meshes = Some(PathBuf::from(value(&mut i, "--meshes")?)),
            "--every" => {
                let text = value(&mut i, "--every")?;
                every = text
                    .parse::<usize>()
                    .ok()
                    .filter(|&n| n > 0)
                    .ok_or_else(|| format!("--every {text:?}: want a positive integer"))?;
            }
            "--force-scale" => {
                force_scale = positive("--force-scale", &value(&mut i, "--force-scale")?)?;
            }
            "--noise-floor" => {
                noise_floor = positive("--noise-floor", &value(&mut i, "--noise-floor")?)?;
            }
            "-h" | "--help" => return Err(USAGE.to_string()),
            other if input.is_none() && !other.starts_with('-') => {
                input = Some(PathBuf::from(other))
            }
            other => return Err(format!("unexpected argument {other:?}\n{USAGE}")),
        }
        i += 1;
    }
    let input: PathBuf = input.ok_or_else(|| format!("no input file given\n{USAGE}"))?;
    let robot = robot.ok_or_else(|| format!("--robot fr3|fer is required\n{USAGE}"))?;
    if robot == RobotKind::Fer && urdf.is_some() {
        return Err("--urdf applies to --robot fr3 only; the FER model is built in".into());
    }
    let out = out.unwrap_or_else(|| input.with_extension("rrd"));
    Ok(Args {
        input,
        robot,
        urdf,
        out,
        every,
        meshes,
        force_scale,
        noise_floor,
    })
}

/// The model the 3D arm is drawn with: the URDF for an FR3, the built-in one for an FER.
fn model(args: &Args) -> Result<Model, Box<dyn std::error::Error>> {
    Ok(match args.robot {
        RobotKind::Fr3 => {
            let path = args.urdf.clone().unwrap_or_else(|| PathBuf::from(FR3_URDF));
            let urdf = std::fs::read_to_string(&path)
                .map_err(|e| format!("--urdf {}: {e}", path.display()))?;
            Model::from_urdf(&urdf)?
        }
        RobotKind::Fer => Model::native_fer(),
    })
}

fn replay_csv(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args(args)?;
    let log = CommanderLog::read(&args.input)?;
    let model = model(&args)?;
    let limits = args.robot.limits();
    let meshes = args.meshes.as_deref().map(Meshes::find).transpose()?;
    let rec = rerun::RecordingStreamBuilder::new("franka_rs").save(&args.out)?;
    let summary = log.record(&rec, &model, &limits, args.every, meshes.as_ref())?;
    commander::send_blueprint(&rec)?;
    rec.flush_blocking()?;

    let span = log.t[log.rows() - 1] - log.t[0];
    println!(
        "{}: {} rows over {span:.3} s, {} target changes -> {}",
        args.input.display(),
        log.rows(),
        summary.changes,
        args.out.display()
    );
    let peaks = summary.commanded;
    println!(
        "commanded position: peak speed {:.3} m/s (limit {:.3}), acceleration {:.2} m/s^2 \
         (limit {:.2}), jerk {:.0} m/s^3 (limit {:.0})",
        peaks.speed, limits.speed, peaks.acceleration, limits.acceleration, peaks.jerk, limits.jerk
    );
    println!(
        "raw target: peak implied speed {:.1} m/s",
        summary.target_speed
    );
    match (summary.tool_offset, summary.fk_gap) {
        (Some(offset), Some(gap)) => println!(
            "3D scene: every {} row(s); tool offset F_T_EE translation {:.4?} m from the first \
             row, model end effector within {:.2} mm of the measured O_T_EE",
            args.every,
            offset,
            gap * 1e3
        ),
        _ => println!("3D scene: no q0..q6 columns, the arm is not drawn"),
    }
    if let Some(meshes) = &meshes {
        println!("meshes: {}", meshes.describe());
    }
    Ok(())
}

fn replay_log(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args(args)?;
    let records = flight::load_records(&args.input)?;
    let model = model(&args)?;
    let options = FlightOptions {
        force_scale: args.force_scale,
        every: args.every,
        meshes: args.meshes.clone(),
        contact: flight::ContactOptions {
            noise_floor: args.noise_floor,
            ..flight::ContactOptions::default()
        },
    };
    // The log's own `last_motion_errors` of its last state is the reflex reason when the log
    // came from a `ControlException`; a log saved from a healthy run has none set.
    let last = records[records.len() - 1].state.last_motion_errors;
    let rec = rerun::RecordingStreamBuilder::new("franka_rs").save(&args.out)?;
    let summary = flight::log_records(&rec, &records, &model, args.robot, &options, Some(&last))?;
    flight::send_blueprint(&rec)?;
    rec.flush_blocking()?;
    println!("{} -> {}", args.input.display(), args.out.display());
    if let Some(dir) = &args.meshes {
        println!(
            "meshes: {} from {}",
            Meshes::find(dir)?.describe(),
            dir.display()
        );
    }
    println!("{summary}");
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("csv") => replay_csv(&args[1..]),
        Some("log") => replay_log(&args[1..]),
        Some("-h" | "--help") => Err(USAGE.into()),
        Some(other) => Err(format!("unknown subcommand {other:?}\n{USAGE}").into()),
        None => Err(USAGE.into()),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("franka-rerun: {e}");
            ExitCode::FAILURE
        }
    }
}
