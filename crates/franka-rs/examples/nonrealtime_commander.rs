//! A non-realtime commander feeding the Cartesian pose interface through a lock-free slot,
//! while the realtime loop keeps every command the robot sees smooth.
//!
//! Most programs that want to move a Franka are not 1 kHz programs: a planner, a vision loop,
//! a script over a socket, a human at a keyboard. They produce *targets* -- at 10 Hz, in
//! bursts, with pauses -- and each of those is a step the robot must never see as a step. This
//! example puts such a commander on its own (non-realtime) thread and shows what the realtime
//! loop has to do about it:
//!
//! * The commander publishes the latest target position into a **seqlock of four
//!   `AtomicU64`** (three `f64`s as bits plus a sequence number). The control callback reads
//!   it every cycle without blocking or allocating; if it catches the writer mid-update it
//!   keeps the previous target for that one cycle instead of spinning.
//! * The callback hands `start pose + target` to `control_cartesian_pose`. In the default
//!   `--bridged` mode it first runs the crate's first-order low-pass filter at a **1 Hz**
//!   cutoff (gain per 1 ms cycle `dt / (dt + 1 / (2 pi f_c))` = 0.0062, so a 5 cm step asks
//!   for 0.31 m/s at first, decaying with a 0.16 s time constant) and then the crate's
//!   Cartesian rate limiter with a deliberately small budget, 0.5 m/s^2 and 20 m/s^3 by
//!   default (`--budget V,A,J` overrides it). The library's own limits (13 m/s^2 on an FER)
//!   are what the robot accepts *in Cartesian space*; it also runs inverse kinematics on every
//!   pose and checks the joint-space continuity, and a real FER refuses a 2.5 m/s^2 ramp near
//!   the ready pose as `cartesian_motion_generator_joint_velocity_discontinuity`. The control
//!   loop's limiter stays on as the backstop and never binds.
//! * In `--raw` mode both are off (`limit_rate = false`, `MAX_CUTOFF_FREQUENCY`), so the first
//!   5 cm target reaches the robot as a 50 m/s jump and its motion generator refuses it with a
//!   `cartesian_motion_generator_velocity_discontinuity` reflex. The example prints the
//!   robot's error text, runs `automatic_error_recovery()` and exits 0.
//!
//! The scripted commander steps the target by +-5 cm in x, y or z inside a +-12 cm box around
//! the start pose (never more than 5 cm below it), with irregular holds between 0.2 s and
//! 1.5 s, one 2 s stall in which it sends nothing, and one burst of 20 targets inside 100 ms;
//! about 20 s in all, orientation fixed. With `--stdin` the commander is whoever writes lines
//! of `x y z` (metres, relative to the start pose) to standard input, same box clamp. The
//! motion ends when the commander is done and the commanded pose has settled, or earlier if
//! the measured end effector strays more than 30 cm from where it started. `--log PATH`
//! writes one CSV row per cycle (`t,target_*,cmd_*,meas_*,q0..q6`: the raw target, the echoed
//! `O_T_EE_c`, the measured `O_T_EE` and joint angles) into a `Vec` sized before the loop, so
//! the loop never allocates; `bench/commander/plot.py` and `tools/rerun-replay` draw it.
//!
//! Usage: `nonrealtime_commander <robot-hostname> [--bridged | --raw] [--stdin] [--log PATH]
//! [--yes]`. Set `FRANKA_REALTIME=ignore` to run against franka-sim on an ordinary kernel;
//! `FRANKA_REALTIME=enforce` (the default) is what a `PREEMPT_RT` box with a real robot wants.
//!
//! # Warning
//! The end effector moves inside a 24 cm cube around wherever it is when the example starts;
//! make sure that cube is free and have the user stop button at hand. `--raw` provokes a
//! reflex on purpose.

#[path = "common/mod.rs"]
mod common;

use std::io::{BufRead, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration as StdDuration, Instant};

use franka::{
    cartesian_low_pass_filter, limit_rate_cartesian_pose, CartesianPose, ControllerMode, Duration,
    FrankaError, FrankaResult, Robot, RobotState, MAX_CUTOFF_FREQUENCY,
};

/// Size of one scripted step, m.
const STEP: f64 = 0.05;
/// Targets are clamped into +-`BOX` m around the start position, never more than `MAX_DROP` m
/// below it.
const BOX: f64 = 0.12;
const MAX_DROP: f64 = 0.05;
/// Measured deviation from the start position at which the loop gives up, m.
const MAX_DEVIATION: f64 = 0.30;
/// Cutoff of the low-pass filter in `--bridged` mode, Hz.
const BRIDGE_CUTOFF_FREQUENCY: f64 = 1.0;
/// The default `--bridged` rate-limit budget: translational m/s, m/s^2, m/s^3, then rotational
/// rad/s, rad/s^2, rad/s^3 (the orientation never changes here). Measured on a real FER near
/// the ready pose: 2.5 m/s^2 with 500 m/s^3 is refused as a joint velocity discontinuity,
/// 1.5 m/s^2 with 200 m/s^3 passes that check, and above about 1 m/s^2 the robot's
/// external-force estimate starts to cross the 20 N collision threshold, so the default sits
/// well below both.
const BRIDGE_LIMITS: [f64; 6] = [0.3, 0.5, 20.0, 1.0, 5.0, 500.0];
/// The commanded pose counts as settled when it stays within `SETTLE_TOLERANCE` metres of the
/// target for `SETTLE_CYCLES` consecutive cycles.
const SETTLE_TOLERANCE: f64 = 1e-3;
const SETTLE_CYCLES: u32 = 250;
/// Rows the CSV log can hold: two minutes at 1 kHz, allocated before the loop starts.
const LOG_CAPACITY: usize = 120_000;

/// A step of `(dx, dy, dz)` metres held for some seconds; a stall of some seconds sending
/// nothing; a burst toggling x by [`STEP`] `count` times `spacing` seconds apart, then holding.
enum Event {
    Step(f64, f64, f64, f64),
    Stall(f64),
    Burst(usize, f64, f64),
}
use Event::{Burst, Stall, Step};

/// About 21 s of steps that never leave the box: `x` and `y` stay within +-0.10 m (the burst
/// reaches 0.10), `z` within `[-0.05, +0.05]`, and the target ends back at the start.
#[rustfmt::skip]
const SCRIPT: &[Event] = &[
    Stall(0.5),
    Step(STEP, 0.0, 0.0, 0.8), Step(0.0, STEP, 0.0, 0.3), Step(0.0, 0.0, STEP, 1.2),
    Step(-STEP, 0.0, 0.0, 0.5), Step(-STEP, 0.0, 0.0, 1.5), Step(0.0, -STEP, 0.0, 0.2),
    Step(0.0, -STEP, 0.0, 0.9), Step(0.0, 0.0, -STEP, 0.4), Step(0.0, 0.0, -STEP, 1.1),
    Step(STEP, 0.0, 0.0, 0.6), Step(0.0, 0.0, STEP, 1.3),
    Stall(2.0),
    Step(STEP, 0.0, 0.0, 0.7), Step(0.0, STEP, 0.0, 0.25), Step(0.0, STEP, 0.0, 1.4),
    Step(0.0, 0.0, STEP, 0.35),
    Burst(20, 0.005, 1.0),
    Step(-STEP, 0.0, 0.0, 0.9), Step(STEP, 0.0, 0.0, 0.55), Step(-STEP, 0.0, 0.0, 0.6),
    Step(0.0, -STEP, 0.0, 0.45), Step(0.0, 0.0, -STEP, 1.0),
];

/// The latest target position, relative to the start pose, as a single-writer seqlock: the
/// sequence number is odd while a write is in progress and changes with every write, so a
/// reader seeing the same even number before and after loading the coordinates has a
/// consistent triple. The zero bit pattern is `0.0`, so `Default` is the start pose.
#[derive(Default)]
struct TargetSlot {
    sequence: AtomicU64,
    coordinates: [AtomicU64; 3],
    /// Set once the commander has nothing more to send.
    finished: AtomicBool,
}

impl TargetSlot {
    /// Clamps `target` into the box and publishes it. The single writer.
    fn publish(&self, target: [f64; 3]) -> [f64; 3] {
        let clamped = [
            target[0].clamp(-BOX, BOX),
            target[1].clamp(-BOX, BOX),
            target[2].clamp(-MAX_DROP, BOX),
        ];
        self.sequence.fetch_add(1, Ordering::SeqCst);
        for (slot, value) in self.coordinates.iter().zip(clamped) {
            slot.store(value.to_bits(), Ordering::SeqCst);
        }
        self.sequence.fetch_add(1, Ordering::SeqCst);
        clamped
    }

    /// Copies the latest consistent target into `into`; `false`, with `into` untouched, if the
    /// writer was mid-update on every try. Never blocks, never spins unboundedly.
    fn load(&self, into: &mut [f64; 3]) -> bool {
        for _ in 0..3 {
            let before = self.sequence.load(Ordering::SeqCst);
            if before & 1 == 1 {
                continue;
            }
            let candidate = self
                .coordinates
                .each_ref()
                .map(|slot| f64::from_bits(slot.load(Ordering::SeqCst)));
            if self.sequence.load(Ordering::SeqCst) == before {
                *into = candidate;
                return true;
            }
        }
        false
    }
}

/// The scripted commander: runs [`SCRIPT`] against the clock, then marks the slot finished.
/// Never joined: if the motion ends early it dies with the process, targets unsent.
fn run_script(slot: &TargetSlot) {
    let started = Instant::now();
    let mut target = [0.0f64; 3];
    for event in SCRIPT {
        let elapsed = started.elapsed().as_secs_f64();
        match *event {
            Step(dx, dy, dz, hold) => {
                target = slot.publish([target[0] + dx, target[1] + dy, target[2] + dz]);
                eprintln!("{elapsed:7.3}s  commander: step to {target:.3?}, hold {hold} s");
                pause(hold);
            }
            Stall(seconds) => {
                eprintln!("{elapsed:7.3}s  commander: stall, nothing for {seconds} s");
                pause(seconds);
            }
            Burst(count, spacing, hold) => {
                eprintln!("{elapsed:7.3}s  commander: burst of {count} targets {spacing} s apart");
                for i in 0..count {
                    let mut toggled = target;
                    toggled[0] += STEP * f64::from(i % 2 == 0);
                    slot.publish(toggled);
                    pause(spacing);
                }
                pause(hold);
            }
        }
    }
    slot.finished.store(true, Ordering::SeqCst);
}

fn pause(seconds: f64) {
    std::thread::sleep(StdDuration::from_secs_f64(seconds));
}

/// The interactive commander: one `x y z` line per target until stdin closes.
fn run_stdin(slot: &TargetSlot) {
    for line in std::io::stdin().lock().lines().map_while(Result::ok) {
        let mut fields = line.split_whitespace().map(str::parse::<f64>);
        match (fields.next(), fields.next(), fields.next()) {
            (Some(Ok(x)), Some(Ok(y)), Some(Ok(z))) => {
                eprintln!("commander: stdin -> {:.3?}", slot.publish([x, y, z]));
            }
            _ => eprintln!("commander: ignoring {line:?}, want `x y z` in metres"),
        }
    }
    slot.finished.store(true, Ordering::SeqCst);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Bridged,
    Raw,
}

fn usage(program: &str) -> ! {
    eprintln!(
        "Usage: {program} <robot-hostname> [--bridged | --raw] [--stdin] [--log PATH] [--yes] \
         [--budget V,A,J]"
    );
    std::process::exit(1);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (mut hostname, mut mode, mut from_stdin, mut log, mut assume_yes) =
        (None, Mode::Bridged, false, None, false);
    let mut limits = BRIDGE_LIMITS;
    let mut arg = 1;
    while arg < args.len() {
        match args[arg].as_str() {
            "--bridged" => mode = Mode::Bridged,
            "--raw" => mode = Mode::Raw,
            "--stdin" => from_stdin = true,
            "--yes" => assume_yes = true,
            "--log" => {
                arg += 1;
                log = Some(args.get(arg).cloned().unwrap_or_else(|| usage(&args[0])));
            }
            "--budget" => {
                arg += 1;
                let parsed: Vec<f64> = args
                    .get(arg)
                    .map(|s| s.split(',').filter_map(|v| v.parse().ok()).collect())
                    .unwrap_or_default();
                match parsed[..] {
                    [v, a, j] => limits[..3].copy_from_slice(&[v, a, j]),
                    _ => usage(&args[0]),
                }
            }
            other if hostname.is_none() && !other.starts_with("--") => hostname = Some(other),
            other => {
                eprintln!("Unexpected argument {other:?}");
                usage(&args[0]);
            }
        }
        arg += 1;
    }
    let hostname = hostname.unwrap_or_else(|| usage(&args[0]));

    match run(
        hostname,
        mode,
        from_stdin,
        log.as_deref(),
        assume_yes,
        limits,
    ) {
        // In `--raw` mode the reflex is the demonstration; in `--bridged` mode it is a failure.
        Ok(reflex) => std::process::exit(if reflex && mode != Mode::Raw { 1 } else { 0 }),
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

/// One log row: `t`, the target, the commanded and measured positions, the joint angles.
type Row = [f64; 23];

fn row(
    t: f64,
    target: &[f64; 3],
    commanded: &[f64; 3],
    measured: &[f64; 3],
    q: &[f64; 7],
    f_ext: &[f64; 6],
) -> Row {
    let mut out = [t; 23];
    out[1..4].copy_from_slice(target);
    out[4..7].copy_from_slice(commanded);
    out[7..10].copy_from_slice(measured);
    out[10..17].copy_from_slice(q);
    out[17..23].copy_from_slice(f_ext);
    out
}

/// Returns whether the motion was aborted by the robot (and recovered from).
fn run(
    hostname: &str,
    mode: Mode,
    from_stdin: bool,
    log: Option<&str>,
    assume_yes: bool,
    limits: [f64; 6],
) -> FrankaResult<bool> {
    let realtime_config = common::realtime_config_from_env();
    let robot = Robot::new(hostname, realtime_config)?;
    common::set_default_behavior(&robot)?;
    // libfranka's current example thresholds. The shared defaults above keep the nominal force
    // threshold at 10 N, which the robot's external-force estimate crosses at 0.25 m/s.
    let torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0];
    let force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0];
    robot.set_collision_behavior(torque, torque, torque, torque, force, force, force, force)?;

    let (limit_rate, name) = match mode {
        Mode::Bridged => (
            true,
            "bridged: 1 Hz filter, then rate limiting within the budget",
        ),
        Mode::Raw => (false, "raw: no filter, no rate limiting, expect a reflex"),
    };
    println!("Mode: {name}");
    if mode == Mode::Bridged {
        println!("Budget: {:?} m/s, m/s^2, m/s^3", &limits[..3]);
    }
    if from_stdin {
        println!("Reading targets `x y z` (metres, relative to the start pose) from stdin.");
    } else if !assume_yes {
        common::wait_for_enter();
    }

    // Everything is relative to the pose the robot is *commanded* to be in right now, which
    // is also what the first setpoint must equal on an FER (see `control_cartesian_pose`).
    // Re-anchored on the first control cycle below: on a real robot `O_T_EE_c` drifts by
    // micrometres between this read and the first cycle, which `--raw` would send as a jump.
    let mut start_pose = robot.read_once()?.O_T_EE_c;
    let mut start = translation(&start_pose);
    println!("Start position: {start:.3?}; targets stay within +-{BOX} m of it.");

    let slot = Arc::new(TargetSlot::default());
    let commander_slot = Arc::clone(&slot);
    std::thread::spawn(move || match from_stdin {
        true => run_stdin(&commander_slot),
        false => run_script(&commander_slot),
    });

    // Preallocated: `push` below never reallocates while `len < capacity`.
    let mut rows: Vec<Row> = Vec::with_capacity(LOG_CAPACITY * usize::from(log.is_some()));
    let mut time = 0.0;
    let mut target = [0.0f64; 3];
    let mut last_commanded = start;
    let mut peak_speed = 0.0f64;
    let mut torn_reads = 0u64;
    let mut settled_cycles = 0u32;
    let mut deviated = false;

    let result = robot.control_cartesian_pose(
        |state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            let commanded = translation(&state.O_T_EE_c);
            let measured = translation(&state.O_T_EE);
            let dt = period.as_secs_f64().max(1e-3);
            peak_speed = peak_speed.max(distance(&commanded, &last_commanded) / dt);
            last_commanded = commanded;

            // The first setpoint is always the start pose: on an FR3 the first command is its
            // own filter reference, so a target that arrived before this cycle would go out
            // unfiltered and unlimited -- exactly the jump this example exists to prevent.
            if time == 0.0 {
                start_pose = state.O_T_EE_c;
                start = translation(&start_pose);
                last_commanded = start;
                target = [0.0; 3];
            } else if !deviated && distance(&measured, &start) > MAX_DEVIATION {
                // Freeze the target where the command is; the bridge brings it to rest.
                deviated = true;
                target = std::array::from_fn(|i| commanded[i] - start[i]);
            } else if !deviated && !slot.load(&mut target) {
                torn_reads += 1;
            }
            let goal: [f64; 3] = std::array::from_fn(|i| start[i] + target[i]);
            let mut pose = start_pose;
            pose[12..15].copy_from_slice(&goal);
            if mode == Mode::Bridged {
                // Filter towards the target from the last commanded pose, then limit the
                // rate against the robot's echo of its last commanded twist and acceleration.
                // Both only fail on non-finite input; holding the last command is the safe
                // answer if they ever do.
                let [v, a, j, rv, ra, rj] = limits;
                pose =
                    cartesian_low_pass_filter(dt, &pose, &state.O_T_EE_c, BRIDGE_CUTOFF_FREQUENCY)
                        .and_then(|filtered| {
                            limit_rate_cartesian_pose(
                                v,
                                a,
                                j,
                                rv,
                                ra,
                                rj,
                                &filtered,
                                &state.O_T_EE_c,
                                &state.O_dP_EE_c,
                                &state.O_ddP_EE_c,
                            )
                        })
                        .unwrap_or(state.O_T_EE_c);
            }

            if rows.len() < rows.capacity() {
                rows.push(row(
                    time,
                    &goal,
                    &commanded,
                    &measured,
                    &state.q,
                    &state.O_F_ext_hat_K,
                ));
            }

            // Under the filter the command approaches the target exponentially; raw, at once.
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
        ControllerMode::CartesianImpedance,
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
            robot.automatic_error_recovery()?;
            println!("automatic_error_recovery() succeeded; the robot is ready again.");
            true
        }
        Err(other) => return Err(other),
    };

    println!("peak commanded speed {peak_speed:.3} m/s, torn slot reads {torn_reads}");
    if deviated {
        println!("Stopped early: the end effector strayed {MAX_DEVIATION} m from the start.");
    }
    if let Some(path) = log {
        write_log(path, &rows)?;
        println!("wrote {} rows to {path}", rows.len());
    }
    Ok(reflex)
}

fn write_log(path: &str, rows: &[Row]) -> FrankaResult<()> {
    let io_error = |e: std::io::Error| FrankaError::InvalidArgument(format!("--log {path}: {e}"));
    let mut out = std::io::BufWriter::new(std::fs::File::create(path).map_err(io_error)?);
    let header = "t,target_x,target_y,target_z,cmd_x,cmd_y,cmd_z,meas_x,meas_y,meas_z";
    writeln!(out, "{header},q0,q1,q2,q3,q4,q5,q6,fx,fy,fz,tx,ty,tz").map_err(io_error)?;
    for row in rows {
        write!(out, "{:.4}", row[0]).map_err(io_error)?;
        for value in &row[1..] {
            write!(out, ",{value:.9}").map_err(io_error)?;
        }
        writeln!(out).map_err(io_error)?;
    }
    out.flush().map_err(io_error)
}
