//! A commander for `franka-node`: takes the lease, enables the arm, moves the end effector
//! along a sine in z of amplitude `--dz` around where it is (or, with `--mode joints`, joint
//! 7 by 0.1 rad), prints what the node reports once a second, then stops and releases.
//! Given a verb instead it sends that one command and reports how the node came out; `home`
//! takes the lease, drives the arm to the ready pose and releases; `gripper ...` commands the
//! arm's gripper (`gripper.rs`). `--episode NAME` names the session, so that two arms driven
//! with the same name record into one Rerun recording. `--help` prints the usage.
//!
//! # Warning
//! Without a verb the end effector moves `dz` up and down from its current pose, and `home`
//! moves every joint to the ready pose; keep that space free and the user stop button at hand.

mod args;
mod common;
mod gripper;

use std::f64::consts::TAU;
use std::time::{Duration, Instant};

use args::{parse_args, Args, Mode, Sine, Verb, USAGE};
use common::{describe, lease, verdict};
use franka_node::{CmdRequest, Kind, Phase, StateMsg};
use zenoh::{Session, Wait};

/// Period of the sine, s: with the default `--dz 0.05` the peak speed is 0.08 m/s.
const PERIOD: f64 = 4.0;
/// Amplitude of the joint 7 sine with `--mode joints`, rad.
const JOINT_AMPLITUDE: f64 = 0.1;

fn main() {
    if std::env::args().any(|a| a == "--help" || a == "-h") {
        println!("{USAGE}");
        return;
    }
    let args = match parse_args() {
        Ok(args) => args,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let outcome = match &args.mode {
        Mode::Sine(sine) => run_sine(&args, sine),
        Mode::Verb(verb) => run_verb(&args, verb),
        Mode::Home(speed) => run_home(&args, *speed),
        Mode::Gripper(cmd) => gripper::run(&args, cmd),
    };
    if let Err(e) = outcome {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

/// A session connected to the node with the arm's state watched and its first state printed.
pub fn connect(args: &Args) -> Result<(Session, common::StateWatch), String> {
    let session = common::open(args.connect.as_deref(), None).map_err(|e| e.to_string())?;
    let watch = common::StateWatch::subscribe(&session, &args.arm).map_err(|e| e.to_string())?;
    let state = watch
        .wait_for(Duration::from_secs(5), |_| true)
        .ok_or("no state from the node within 5 s")?;
    println!("node: {}", describe(&state));
    Ok((session, watch))
}

/// Sends the one command and waits up to 3 s for the state to show its effect.
fn run_verb(args: &Args, verb: &Verb) -> Result<(), String> {
    let arm = args.arm.as_str();
    let (session, watch) = connect(args)?;
    let client_id = verb.client_id.unwrap_or_else(std::process::id);
    let reply = common::cmd(&session, arm, &verb.name, client_id)?;
    println!("{}: {}", verb.name, verdict(&reply));
    let settled: fn(&StateMsg) -> bool = match verb.name.as_str() {
        "recover" => |s| s.phase != Phase::Faulted as u8,
        "stop" => |s| s.phase != Phase::Active as u8 && s.phase != Phase::Stopping as u8,
        _ => |s| s.client_id.get() == 0,
    };
    let settled = watch.wait_for(Duration::from_secs(3), settled);
    let last = settled
        .or_else(|| watch.latest())
        .ok_or("the state stream stopped")?;
    println!("final: {}", describe(&last));
    session.close().wait().map_err(|e| e.to_string())?;
    match (reply.ok, settled.is_some()) {
        (true, true) => Ok(()),
        (false, _) => Err(format!("{} refused", verb.name)),
        (true, false) => Err(format!(
            "{}: the state did not follow within 3 s",
            verb.name
        )),
    }
}

/// Takes the lease, homes the arm (the reply comes when it has arrived), releases.
fn run_home(args: &Args, speed: Option<f64>) -> Result<(), String> {
    let arm = args.arm.as_str();
    let client_id = std::process::id();
    let (session, watch) = connect(args)?;
    let token = lease(&session, arm, client_id)?;
    let request = CmdRequest {
        speed,
        episode: args.episode.clone(),
        ..CmdRequest::new(client_id)
    };
    let began = Instant::now();
    let reply = common::request(&session, arm, "home", &request)?;
    println!(
        "home: {} after {:.1} s",
        verdict(&reply),
        began.elapsed().as_secs_f64()
    );
    let state = watch.latest().ok_or("the state stream stopped")?;
    println!(
        "final: {} q {:.3?}",
        describe(&state),
        state.q.map(|v| v.get())
    );
    if state.phase == Phase::Acquired as u8 {
        common::cmd_ok(&session, arm, "release", client_id)?;
        println!("released");
    }
    token.undeclare().wait().map_err(|e| e.to_string())?;
    session.close().wait().map_err(|e| e.to_string())?;
    if reply.ok {
        Ok(())
    } else {
        Err("home refused".into())
    }
}

fn run_sine(args: &Args, sine: &Sine) -> Result<(), String> {
    let arm = args.arm.as_str();
    let client_id = std::process::id();
    let (session, watch) = connect(args)?;
    let publisher = session
        .declare_publisher(format!("franka/{arm}/target"))
        .wait()
        .map_err(|e| e.to_string())?;
    let token = lease(&session, arm, client_id)?;
    let (kind, axis, amplitude) = if sine.joints {
        (Kind::Joints, 6, JOINT_AMPLITUDE)
    } else {
        (Kind::Cartesian, 2, sine.dz)
    };
    let enable = CmdRequest {
        mode: Some(kind),
        episode: args.episode.clone(),
        ..CmdRequest::new(client_id)
    };
    let reply = common::request(&session, arm, "enable", &enable)?;
    if !reply.ok {
        return Err(format!("enable refused: {}", verdict(&reply)));
    }
    let state = watch
        .wait_for(Duration::from_secs(2), |s| s.phase == Phase::Active as u8)
        .ok_or("the arm did not become active")?;
    let start = state.target.map(|v| v.get());
    println!(
        "enabled ({kind}); start target {start:.4?}, moving [{axis}] by +-{amplitude} at {} Hz \
         for {} s",
        sine.hz, sine.seconds
    );

    let began = Instant::now();
    let step = Duration::from_secs_f64(1.0 / sine.hz);
    let (mut seq, mut next, mut next_report) = (0u64, began, began + Duration::from_secs(1));
    let outcome = loop {
        let t = began.elapsed().as_secs_f64();
        if t >= sine.seconds {
            break Ok(());
        }
        let mut target = start;
        target[axis] = start[axis] + amplitude * (TAU * t / PERIOD).sin();
        seq += 1;
        common::publish_target(&publisher, kind, client_id, seq, target)
            .map_err(|e| e.to_string())?;
        if Instant::now() >= next_report {
            next_report += Duration::from_secs(1);
            let state = watch.latest().ok_or("the state stream stopped")?;
            let rtts = watch.take_rtts();
            let rtt = match rtts.iter().min() {
                Some(min) => format!(
                    "rtt min {:.2} ms mean {:.2} ms",
                    *min as f64 * 1e-6,
                    rtts.iter().sum::<u64>() as f64 * 1e-6 / rtts.len() as f64
                ),
                None => "rtt -".to_string(),
            };
            let peaks = watch.take_peaks();
            println!(
                "t {t:4.1} s: {} {rtt} peak |F_ext| {:.1} N deviation {:.1} mm",
                describe(&state),
                peaks.force,
                peaks.deviation * 1e3
            );
            if state.phase != Phase::Active as u8 {
                break Err(format!("the arm left Active: {}", describe(&state)));
            }
        }
        next += step;
        if let Some(pause) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(pause);
        }
    };

    let stop = common::cmd(&session, arm, "stop", client_id)?;
    println!("stop: {}", verdict(&stop));
    let state = watch
        .wait_for(Duration::from_secs(3), |s| s.phase != Phase::Stopping as u8)
        .ok_or("the arm did not leave Stopping")?;
    println!("final: {}", describe(&state));
    if state.phase == Phase::Idle as u8 {
        common::cmd_ok(&session, arm, "release", client_id)?;
        println!("released");
    }
    token.undeclare().wait().map_err(|e| e.to_string())?;
    session.close().wait().map_err(|e| e.to_string())?;
    outcome
}
