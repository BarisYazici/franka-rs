//! `franka-rerun`: replays franka-rs logs as Rerun recordings.
//!
//! * `franka-rerun csv <log.csv> --robot fr3|fer` turns a `nonrealtime_commander --log` CSV
//!   into an `.rrd` with the positions, the velocities, the derivatives of the commanded
//!   position against the robot's limits, the commander's events and a 3D replay of the arm
//!   (`--layout demo` for the screen-capture layout).
//! * `franka-rerun log <records.json> --robot fr3|fer` turns a control log saved with
//!   `franka_rerun::save_records` (the `Vec<franka::Record>` a `ControlException` carries)
//!   into a flight recording: contact and collision flags, external wrench, commanded versus
//!   measured, errors, the arm.
//!
//! Open the result with `rerun out.rrd`.

use std::path::PathBuf;
use std::process::ExitCode;

use franka::Model;
use franka_rerun::{
    commander, demo, flight, CommanderLog, FlightOptions, Layout, Limits, MeshChoice, Prefix,
    RobotKind,
};

const USAGE: &str = "Usage: franka-rerun csv <log.csv> --robot fr3|fer [--urdf PATH] [-o out.rrd] \
                     [--every N] [--meshes DIR | --no-meshes] [--layout default|demo] \
                     [--budget V,A,J]\n       \
                     franka-rerun log \
                     <records.json> --robot \
                     fr3|fer [--urdf PATH] [-o out.rrd] [--every N] [--meshes DIR | --no-meshes] \
                     [--force-scale M] [--noise-floor NM] [--prefix NAME]\n\n  csv           \
                     replay a \
                     nonrealtime_commander --log CSV\n  log           replay a control log \
                     saved with franka_rerun::save_records (JSON)\n  --robot fr3   FR3 limits, \
                     model from --urdf (default: the repository's tests/data/fr3.urdf)\n  \
                     --robot fer   FER limits, the crate's built-in FER model\n  -o PATH       \
                     output recording (default: the input with .rrd)\n  --every N     log every \
                     N-th row to the 3D scene (default 1)\n  --meshes DIR  draw the arm with \
                     the link meshes in DIR (link0..7, hand, finger as .glb; see \
                     tools/franka-meshes) instead of the built-in ones\n  --no-meshes   the \
                     skeleton alone\n  --layout demo  csv only: the screen-capture \
                     layout (the arm at full height, the velocity per axis and the speed on \
                     the right, no event log)\n  --budget V,A,J  csv only: the commander's \
                     own velocity, acceleration and jerk budget (m/s, m/s^2, m/s^3), the \
                     limit lines of sent/* (default: the robot's limits)\n  --force-scale M  \
                     log only: metres of arrow per \
                     newton of external force (default 0.01)\n  --noise-floor NM  log only: \
                     external joint torques below NM count as zero for the contact estimate \
                     (default 1)\n  --prefix NAME  log only: put every entity under NAME, \
                     as the node does with an arm's name, so that two replays can be merged \
                     into one recording";

struct Args {
    input: PathBuf,
    robot: RobotKind,
    urdf: Option<PathBuf>,
    out: PathBuf,
    every: usize,
    meshes: MeshChoice,
    force_scale: f64,
    noise_floor: f64,
    demo: bool,
    budget: Option<Limits>,
    prefix: Prefix,
}

/// A name that can be one entity path part, as an arm's name is: anything else would split the
/// path or escape it.
fn key_safe(flag: &str, name: &str) -> Result<String, String> {
    let ok = |c: char| c.is_ascii_alphanumeric() || c == '_' || c == '-';
    if name.is_empty() || !name.chars().all(ok) {
        return Err(format!("{flag} {name:?}: want [A-Za-z0-9_-]+"));
    }
    Ok(name.to_string())
}

/// A positive, finite number after `flag`.
fn positive(flag: &str, text: &str) -> Result<f64, String> {
    text.parse::<f64>()
        .ok()
        .filter(|&s| s > 0.0 && s.is_finite())
        .ok_or_else(|| format!("{flag} {text:?}: want a positive number"))
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let (mut input, mut robot, mut urdf, mut out) = (None, None, None, None);
    let mut meshes = MeshChoice::default();
    let defaults = FlightOptions::default();
    let (mut every, mut force_scale) = (1, defaults.force_scale);
    let mut noise_floor = defaults.contact.noise_floor;
    let mut demo = false;
    let mut budget = None;
    let mut prefix = Prefix::none();
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
            "--meshes" => meshes = MeshChoice::Dir(PathBuf::from(value(&mut i, "--meshes")?)),
            "--no-meshes" => meshes = MeshChoice::Off,
            "--prefix" => prefix = Prefix::new(key_safe("--prefix", &value(&mut i, "--prefix")?)?),
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
            "--budget" => {
                let text = value(&mut i, "--budget")?;
                let parts = text
                    .split(',')
                    .map(|part| positive("--budget", part.trim()))
                    .collect::<Result<Vec<f64>, String>>()?;
                let [speed, acceleration, jerk] = parts
                    .try_into()
                    .map_err(|_| format!("--budget {text:?}: want three numbers V,A,J"))?;
                budget = Some(Limits {
                    speed,
                    acceleration,
                    jerk,
                });
            }
            "--layout" => {
                demo = match value(&mut i, "--layout")?.as_str() {
                    "default" => false,
                    "demo" => true,
                    other => {
                        return Err(format!("--layout {other:?}: want default or demo\n{USAGE}"))
                    }
                }
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
        demo,
        budget,
        prefix,
    })
}

/// The model the 3D arm is drawn with: the URDF for an FR3, the built-in one for an FER.
fn model(args: &Args) -> Result<Model, Box<dyn std::error::Error>> {
    Ok(match args.robot {
        RobotKind::Fr3 => {
            let urdf = match &args.urdf {
                Some(path) => std::fs::read_to_string(path)
                    .map_err(|e| format!("--urdf {}: {e}", path.display()))?,
                None => franka::model::FR3_URDF.to_owned(),
            };
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
    let budget = args.budget.unwrap_or(limits);
    let meshes = args.meshes.resolve(args.robot)?;
    let rec = rerun::RecordingStreamBuilder::new(franka_rerun::APPLICATION_ID).save(&args.out)?;
    let summary = log.record(&rec, &model, &limits, &budget, args.every, meshes.as_ref())?;
    if args.demo {
        demo::send_blueprint(&rec, &budget)?;
    } else {
        commander::send_blueprint(&rec)?;
    }
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
        "sent command: derivatives {}; limit lines at {} (speed {:.3} m/s, acceleration \
         {:.2} m/s^2, jerk {:.0} m/s^3)",
        if summary.exact_derivatives {
            "from the generator's cmd_v*, cmd_a* columns"
        } else {
            "as finite differences of cmd_* (no cmd_v*, cmd_a* columns)"
        },
        if args.budget.is_some() {
            "the --budget"
        } else {
            "the robot's limits"
        },
        budget.speed,
        budget.acceleration,
        budget.jerk
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
        prefix: args.prefix.clone(),
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
    let rec = rerun::RecordingStreamBuilder::new(franka_rerun::APPLICATION_ID).save(&args.out)?;
    let summary = flight::log_records(&rec, &records, &model, args.robot, &options, Some(&last))?;
    flight::send_blueprint(&rec, &Layout::single(args.prefix.clone()))?;
    rec.flush_blocking()?;
    println!("{} -> {}", args.input.display(), args.out.display());
    if let Some(meshes) = args.meshes.resolve(args.robot)? {
        println!("meshes: {}", meshes.describe());
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
