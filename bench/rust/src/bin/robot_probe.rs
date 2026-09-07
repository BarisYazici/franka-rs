//! Read-only precondition probe for the hardware stages, plus the return-to-ready move.
//!
//! Three things the hardware harness needs and neither benchmark client should be doing
//! itself:
//!
//! * **probe** (the default, and read-only): one `read_once()`, printing the robot mode, the
//!   set error flags and the joint angles as JSON. Exits 0 when the robot is `Idle` with no
//!   errors, 4 when it is not, 1 when it could not be reached. The harness reads the JSON.
//! * `--recover`: run `automatic_error_recovery()` **once** and probe again. The harness only
//!   passes this when the first probe reported `Reflex`, and counts the event.
//! * `--home`: drive the arm back to the ready pose with the shared `MotionGenerator` — the
//!   same generator the ported examples use, itself a port of libfranka's
//!   `examples/examples_common.cpp`. Default speed factor 0.2. The C++ equivalent is
//!   `bench/cpp/robot_home.cpp`, which is what `bench/run.sh --hardware` actually calls; this
//!   flag exists so the move can also be made by hand from the Rust side.
//!
//! Nothing here commands a torque, and the default invocation writes nothing to the robot.

// The homing motion generator is shared verbatim with the ported examples rather than
// duplicated here, exactly as the C++ side links `libexamples_common.a`.
#[path = "../../../../crates/franka-rs/examples/common/mod.rs"]
mod common;

use franka::{
    ControllerMode, FrankaResult, RealtimeConfig, Robot, RobotMode, DEFAULT_CUTOFF_FREQUENCY,
};

/// Exit code for "reached the robot, but it is not `Idle` / has errors".
const EXIT_NOT_READY: i32 = 4;

struct Args {
    host: String,
    out: Option<String>,
    speed: f64,
    recover: bool,
    home: bool,
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().collect();
    let mut args = Args {
        host: String::new(),
        out: None,
        speed: 0.2,
        recover: false,
        home: false,
    };
    let mut i = 1;
    while i < argv.len() {
        let flag = argv[i].clone();
        let takes_value = matches!(flag.as_str(), "--out" | "--speed");
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
            "--out" => args.out = Some(value),
            "--speed" => args.speed = value.parse().unwrap_or(0.2),
            "--recover" => args.recover = true,
            "--home" => args.home = true,
            other if other.starts_with('-') => {
                usage_exit(&argv[0], &format!("unknown flag {other}"))
            }
            other => args.host = other.to_owned(),
        }
        i += 1;
    }
    if args.host.is_empty() {
        usage_exit(&argv[0], "a robot hostname is required");
    }
    args
}

fn usage_exit(program: &str, message: &str) -> ! {
    eprintln!("error: {message}");
    eprintln!(
        "usage: {program} <robot-hostname> [--recover] [--home] [--speed 0.2] [--out FILE]\n  \
         default: a read-only probe; exit 0 if Idle with no errors, {EXIT_NOT_READY} if not"
    );
    std::process::exit(2);
}

fn mode_name(mode: RobotMode) -> &'static str {
    match mode {
        RobotMode::Other => "Other",
        RobotMode::Idle => "Idle",
        RobotMode::Move => "Move",
        RobotMode::Guiding => "Guiding",
        RobotMode::Reflex => "Reflex",
        RobotMode::UserStopped => "UserStopped",
        RobotMode::AutomaticErrorRecovery => "AutomaticErrorRecovery",
    }
}

fn json_string_array(values: &[&str]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("\"{v}\"")).collect();
    format!("[{}]", items.join(", "))
}

fn json_f64_array(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("{v:.6}")).collect();
    format!("[{}]", items.join(", "))
}

fn main() {
    let args = parse_args();
    match run(&args) {
        Ok(ready) => std::process::exit(if ready { 0 } else { EXIT_NOT_READY }),
        Err(e) => {
            eprintln!("franka error: {e}");
            std::process::exit(1);
        }
    }
}

/// Returns whether the robot ended up `Idle` with no errors.
fn run(args: &Args) -> FrankaResult<bool> {
    // `RealtimeConfig::Ignore`: this probe does no realtime work of its own.
    let robot = Robot::new(&args.host, RealtimeConfig::Ignore)?;

    let before = robot.read_once()?;
    let mode_before = mode_name(before.robot_mode);
    let errors_before = before.current_errors.names();

    let mut recovered = false;
    let mut recovery_error: Option<String> = None;
    if args.recover {
        match robot.automatic_error_recovery() {
            Ok(()) => recovered = true,
            Err(e) => recovery_error = Some(e.to_string()),
        }
    }

    let mut homed = false;
    let mut home_error: Option<String> = None;
    if args.home {
        // Only move an arm that is actually idle and error-free.
        let state = robot.read_once()?;
        if state.robot_mode == RobotMode::Idle && !state.current_errors.any() {
            common::set_default_behavior(&robot)?;
            let mut motion_generator =
                common::MotionGenerator::new(robot.fci_version(), args.speed, common::READY_POSE);
            match robot.control_joint_positions(
                |state, period| motion_generator.step(state, period),
                ControllerMode::JointImpedance,
                true,
                DEFAULT_CUTOFF_FREQUENCY,
            ) {
                Ok(()) => homed = true,
                Err(e) => home_error = Some(e.to_string()),
            }
        } else {
            home_error = Some(format!(
                "refusing to home: mode {} with errors {:?}",
                mode_name(state.robot_mode),
                state.current_errors.names()
            ));
        }
    }

    let after = robot.read_once()?;
    let ready = after.robot_mode == RobotMode::Idle && !after.current_errors.any();

    let json = format!(
        "{{\n  \"host\": \"{host}\",\n  \"mode_before\": \"{mode_before}\",\n  \
         \"errors_before\": {errors_before},\n  \"recover_requested\": {recover},\n  \
         \"recovered\": {recovered},\n  \"recovery_error\": {recovery_error},\n  \
         \"home_requested\": {home_requested},\n  \"home_speed\": {speed},\n  \
         \"homed\": {homed},\n  \"home_error\": {home_error},\n  \
         \"mode\": \"{mode}\",\n  \"errors\": {errors},\n  \"has_errors\": {has_errors},\n  \
         \"ready\": {ready},\n  \"q\": {q},\n  \"tau_J\": {tau}\n}}\n",
        host = args.host,
        mode_before = mode_before,
        errors_before = json_string_array(&errors_before),
        recover = args.recover,
        recovered = recovered,
        recovery_error = recovery_error
            .as_ref()
            .map(|e| format!("\"{}\"", e.replace('\\', "\\\\").replace('"', "\\\"")))
            .unwrap_or_else(|| "null".to_owned()),
        home_requested = args.home,
        speed = args.speed,
        homed = homed,
        home_error = home_error
            .as_ref()
            .map(|e| format!("\"{}\"", e.replace('\\', "\\\\").replace('"', "\\\"")))
            .unwrap_or_else(|| "null".to_owned()),
        mode = mode_name(after.robot_mode),
        errors = json_string_array(&after.current_errors.names()),
        has_errors = after.current_errors.any(),
        ready = ready,
        q = json_f64_array(&after.q),
        tau = json_f64_array(&after.tau_J),
    );

    print!("{json}");
    if let Some(path) = &args.out {
        if let Err(e) = std::fs::write(path, &json) {
            eprintln!("failed to write {path}: {e}");
            std::process::exit(1);
        }
    }
    Ok(ready)
}
