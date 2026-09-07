//! A Cartesian impedance controller whose equilibrium traces a figure eight, driven through
//! the `ActiveControl` API. The demo to run when someone is standing next to the robot.
//!
//! `cartesian_impedance_active_control` holds one pose and lets you push against it. This one
//! keeps the same spring-damper law but moves the equilibrium along a Lissajous figure eight
//! in the base frame's y-z plane, so the arm is already in motion when a visitor grabs it:
//!
//! ```text
//! p_d(t) = p_0 + [ 0 ; A r(t) sin(w t) ; (A/2) r(t) sin(2 w t) ]     w = 2 pi / period
//! tau    = J^T ( -r(t) K e - D J dq  +  r(t) f_floor )  +  N tau_null  +  coriolis
//! e      = [ p - p_d ; -R vec(q_ee^-1 q_d) ]
//! K      = diag(200, 200, 200, 15, 15, 15)     N/m, Nm/rad,   D = 2 sqrt(K)
//! N      = I - J^T (J J^T + lambda^2 I)^-1 J
//! ```
//!
//! Three things it adds over the base example:
//!
//! * **Ramps.** `r(t)` is a raised cosine that takes both the path amplitude and the stiffness
//!   from zero to nominal over the first 3 s, and back to zero over the last 2 s of
//!   `--duration` (or over 1 s after Ctrl-C, or after the arm is pulled too far off the path).
//!   Nothing steps at the start or at the end. The damping is *not* ramped: the loop is
//!   over-damped while `r < 1` and settles into a pure damper before it lets go, which is the
//!   quiet way to end a torque loop.
//! * **A nullspace joint spring** toward the configuration the arm started in, projected into
//!   the Jacobian's nullspace so it never fights the Cartesian task. Without it the elbow
//!   wanders off while you push the hand around and eventually parks against a joint limit.
//! * **A virtual floor** 10 cm below the start pose (`--floor`): a one-sided 1500 N/m spring
//!   that only pushes up. Press the end effector down and there is a surface there.
//!
//! Safety: every commanded joint torque is clamped to +-25 Nm before the Coriolis feedforward
//! is added, and if the end effector is more than 30 cm from its current setpoint -- someone
//! has taken hold of it and walked away with it -- the loop ramps down and finishes.
//!
//! Usage:
//! `cartesian_impedance_figure_eight <robot-hostname> [--duration SEC] [--period SEC]
//! [--amplitude M] [--floor M] [--yes]`
//!
//! Defaults: 60 s, a 10 s lap, 0.08 m amplitude, a floor 0.10 m down. A status line goes to
//! stderr once a second; the summary goes to stdout. Like the base example, `SIGINT` only
//! raises a flag, so Ctrl-C leaves through the same final `motion_finished` write as a
//! timeout instead of abandoning the loop to the robot's communication watchdog.
//!
//! Runs on both generations, unchanged: on an FR3 (FCI v10) `start_torque_control` opens a
//! torque-only external controller, and on an FER (FCI v5), which has no such mode, the crate
//! opens a joint-velocity motion generator commanding zeros alongside it.
//!
//! Set `FRANKA_REALTIME=ignore` to run against franka-sim on an ordinary kernel;
//! `FRANKA_REALTIME=enforce` (the default) is what a `PREEMPT_RT` box with a real robot wants.
//!
//! # Warning
//! The collision thresholds this example sets are the C++ Cartesian impedance example's, and
//! they are high (100 Nm / 100 N on every axis). Have the user stop button at hand.

#[path = "common/mod.rs"]
mod common;

use std::sync::atomic::{AtomicBool, Ordering};

use nalgebra::{Rotation3, SMatrix, SVector, UnitQuaternion, Vector3};

use franka::{motion_finished, Frame, FrankaResult, Robot, RobotState, Torques};

use common::cartesian::{compliance, decompose, orientation_error};

/// Translational stiffness, N/m.
const TRANSLATIONAL_STIFFNESS: f64 = 200.0;
/// Rotational stiffness, Nm/rad.
const ROTATIONAL_STIFFNESS: f64 = 15.0;
/// Joint-space stiffness of the nullspace spring toward the initial configuration, Nm/rad.
const NULLSPACE_STIFFNESS: f64 = 5.0;
/// Stiffness of the one-sided virtual floor, N/m.
const FLOOR_STIFFNESS: f64 = 1500.0;
/// Damping term of the 6x6 damped pseudo-inverse used for the nullspace projector.
const DAMPED_INVERSE_LAMBDA_SQUARED: f64 = 1e-4;

/// Per-joint clamp on the commanded torque, Nm. Applied before the Coriolis feedforward.
const TORQUE_LIMIT: f64 = 25.0;
/// How far the end effector may be from its setpoint before the loop gives up, m.
const MAX_DEVIATION: f64 = 0.30;

/// Seconds the amplitude and the stiffness take to reach nominal.
const RAMP_IN: f64 = 3.0;
/// Seconds they take to come back to zero at the end of `--duration`.
const RAMP_OUT: f64 = 2.0;
/// Seconds they take to come back to zero after Ctrl-C or a too-large deviation.
const ABORT_RAMP_OUT: f64 = 1.0;

/// Defaults for the four numeric options.
const DEFAULT_DURATION: f64 = 60.0;
const DEFAULT_PERIOD: f64 = 10.0;
const DEFAULT_AMPLITUDE: f64 = 0.08;
const DEFAULT_FLOOR: f64 = 0.10;

/// One bin per millimetre of position error, the last one catching everything at or above
/// [`MAX_DEVIATION`]. A fixed-size histogram rather than a `Vec` of samples, so that recording
/// the tracking error does not allocate inside the loop.
const HISTOGRAM_BINS: usize = 301;

/// Raised by the `SIGINT` handler; polled once per control cycle.
static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigint(_signal: libc::c_int) {
    // The only async-signal-safe thing done here: a relaxed store on a lock-free `AtomicBool`.
    STOP.store(true, Ordering::Relaxed);
}

/// Why the loop is ramping down.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Reason {
    Duration,
    Interrupt,
    Deviation,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut hostname: Option<&str> = None;
    let mut duration = DEFAULT_DURATION;
    let mut path_period = DEFAULT_PERIOD;
    let mut amplitude = DEFAULT_AMPLITUDE;
    let mut floor_depth = DEFAULT_FLOOR;
    let mut assume_yes = false;

    let mut arg = 1;
    while arg < args.len() {
        match args[arg].as_str() {
            "--yes" => assume_yes = true,
            "--duration" => duration = positive_argument(&args, &mut arg, "--duration"),
            "--period" => path_period = positive_argument(&args, &mut arg, "--period"),
            "--amplitude" => amplitude = positive_argument(&args, &mut arg, "--amplitude"),
            "--floor" => floor_depth = positive_argument(&args, &mut arg, "--floor"),
            other if hostname.is_none() && !other.starts_with("--") => hostname = Some(other),
            other => {
                eprintln!("Unexpected argument {other:?}");
                hostname = None;
                break;
            }
        }
        arg += 1;
    }

    let Some(hostname) = hostname else {
        eprintln!(
            "Usage: {} <robot-hostname> [--duration SEC] [--period SEC] [--amplitude M] \
             [--floor M] [--yes]",
            args[0]
        );
        std::process::exit(1);
    };

    let options = Options {
        duration,
        path_period,
        amplitude,
        floor_depth,
        assume_yes,
    };
    if let Err(e) = run(hostname, &options) {
        println!("{e}");
        std::process::exit(1);
    }
}

/// Consumes the value after `flag` as a positive number of seconds/metres, or exits.
fn positive_argument(args: &[String], arg: &mut usize, flag: &str) -> f64 {
    *arg += 1;
    match args.get(*arg).and_then(|value| value.parse::<f64>().ok()) {
        Some(value) if value > 0.0 => value,
        _ => {
            eprintln!("{flag} wants a positive number");
            std::process::exit(1);
        }
    }
}

/// The four numbers and the flag from the command line.
struct Options {
    duration: f64,
    path_period: f64,
    amplitude: f64,
    floor_depth: f64,
    assume_yes: bool,
}

/// A raised cosine rising from 0 at `x <= 0` to 1 at `x >= 1`, with zero slope at both ends.
fn raised_cosine(x: f64) -> f64 {
    if x <= 0.0 {
        0.0
    } else if x >= 1.0 {
        1.0
    } else {
        0.5 * (1.0 - (std::f64::consts::PI * x).cos())
    }
}

/// The smallest bin index at which the cumulative count reaches half of `total`, i.e. the
/// median position error in millimetres. `f64::NAN` if nothing was recorded.
fn median_mm(histogram: &[u32; HISTOGRAM_BINS], total: u64) -> f64 {
    if total == 0 {
        return f64::NAN;
    }
    let mut seen = 0u64;
    for (bin, count) in histogram.iter().enumerate() {
        seen += u64::from(*count);
        if seen * 2 >= total {
            return bin as f64;
        }
    }
    (HISTOGRAM_BINS - 1) as f64
}

fn run(hostname: &str, options: &Options) -> FrankaResult<()> {
    let realtime_config = common::realtime_config_from_env();
    let robot = Robot::new(hostname, realtime_config)?;
    println!("Negotiated FCI version: {:?}", robot.fci_version());
    common::set_default_behavior(&robot)?;

    // Load the kinematics and dynamics model. Native on both generations: the FR3's own
    // `GetRobotModel` URDF, and the compiled-in `FER_URDF` on an FER.
    let model = robot.load_model()?;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // The C++ Cartesian impedance example's thresholds, deliberately high so that pushing the
    // arm around does not trip a reflex; `set_default_behavior` above set the usual 20/10 Nm
    // ones, and this call replaces them.
    robot.set_collision_behavior_simple([100.0; 7], [100.0; 7], [100.0; 6], [100.0; 6])?;

    println!(
        "WARNING: Collision thresholds are set to high values. Make sure you have the user \
         stop at hand!"
    );
    println!(
        "The end effector will trace a figure eight of {:.0} mm around where it is now, one \
         lap every {} s.",
        options.amplitude * 1000.0,
        options.path_period
    );
    println!(
        "Push it off the path and let go; press it down to find the virtual floor {:.0} mm \
         below the start pose.",
        options.floor_depth * 1000.0
    );
    println!(
        "Amplitude and stiffness ramp up over {RAMP_IN} s. The loop runs for {} s, or until \
         Ctrl-C.",
        options.duration
    );
    if options.assume_yes {
        println!("--yes given, starting without waiting for Enter.");
    } else {
        println!("Press Enter to continue...");
        let mut discard = String::new();
        let _ = std::io::stdin().read_line(&mut discard);
    }

    // SAFETY: `on_sigint` is an `extern "C"` function that only does a relaxed store on a
    // lock-free static `AtomicBool`, which is async-signal-safe. Installed after the prompt so
    // that Ctrl-C at the prompt still terminates the process the usual way.
    unsafe {
        libc::signal(libc::SIGINT, on_sigint as *const () as libc::sighandler_t);
    }

    // The path is anchored -- and the nullspace spring seeded -- *after* the prompt, like
    // every other example in this crate seeds its trajectory: whatever the arm does while it
    // waits would otherwise become the error the loop wakes up with.
    let initial_state: RobotState = robot.read_once()?;
    let (position_start, rotation_start) = decompose(&initial_state.O_T_EE);
    let orientation_d =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation_start));
    let q_initial = SVector::<f64, 7>::from_column_slice(&initial_state.q);
    let z_floor = position_start.z - options.floor_depth;

    let (stiffness, damping) = compliance(TRANSLATIONAL_STIFFNESS, ROTATIONAL_STIFFNESS);
    let nullspace_damping = 2.0 * NULLSPACE_STIFFNESS.sqrt();
    let angular_velocity = std::f64::consts::TAU / options.path_period;
    let identity_6 = SMatrix::<f64, 6, 6>::identity();
    let identity_7 = SMatrix::<f64, 7, 7>::identity();

    // Everything the loop touches is either a fixed-size (stack) nalgebra type or a plain
    // array, and the model methods return fixed arrays, so no control cycle allocates. The
    // 1 Hz status line writes through `eprintln!`, whose `LineWriter` buffer is allocated
    // once -- by this header line, before the loop, rather than inside it.
    eprintln!("    time   |e_pos|   ramp   floor   clamped");
    let mut error = SVector::<f64, 6>::zeros();
    let mut wrench = SVector::<f64, 6>::zeros();
    let mut tau_d = [0.0f64; 7];
    let mut histogram = [0u32; HISTOGRAM_BINS];

    let mut time = 0.0;
    let mut cycles = 0usize;
    let mut next_status = 1.0;
    let mut samples = 0u64;
    let mut worst_position_error = 0.0f64;
    let mut worst_torque = 0.0f64;
    let mut clamped_cycles = 0usize;
    let mut clamp_reported = false;
    let mut floor_cycles = 0usize;
    // `Some((started_at, seconds, why))` once the loop is on its way out.
    let mut ramp_down: Option<(f64, f64, Reason)> = None;
    // Assigned on every cycle before the loop can break, so no initialiser is needed.
    let mut success_rate;

    let mut control = robot.start_torque_control()?;
    loop {
        let (state, period) = control.read_once()?;
        time += period.as_secs_f64();
        cycles += 1;
        success_rate = state.control_command_success_rate;

        // Decide whether this is the cycle the ramp-down starts on. A deviation abort is
        // raised at the end of the previous cycle, so only the timeout and Ctrl-C land here.
        if ramp_down.is_none() {
            if STOP.load(Ordering::Relaxed) {
                ramp_down = Some((time, ABORT_RAMP_OUT, Reason::Interrupt));
            } else if time >= options.duration - RAMP_OUT {
                ramp_down = Some((time, RAMP_OUT, Reason::Duration));
            }
        }
        // One factor for the path amplitude, the Cartesian stiffness, the nullspace stiffness
        // and the floor, so that all four reach nominal together and leave together.
        let ramp = match ramp_down {
            Some((started_at, seconds, _)) => raised_cosine(time / RAMP_IN)
                .min(1.0 - raised_cosine((time - started_at) / seconds)),
            None => raised_cosine(time / RAMP_IN),
        };

        // Model terms for this state.
        let coriolis = SVector::<f64, 7>::from_column_slice(&model.coriolis(&state));
        let jacobian = SMatrix::<f64, 6, 7>::from_column_slice(
            &model.zero_jacobian(Frame::EndEffector, &state),
        );
        let q = SVector::<f64, 7>::from_column_slice(&state.q);
        let dq = SVector::<f64, 7>::from_column_slice(&state.dq);

        // The moving equilibrium: a Lissajous figure eight in the base frame's y-z plane,
        // 1:2 in frequency, centred on the pose the arm started in. The orientation setpoint
        // stays put.
        let scaled_amplitude = options.amplitude * ramp;
        let position_d = position_start
            + Vector3::new(
                0.0,
                scaled_amplitude * (angular_velocity * time).sin(),
                0.5 * scaled_amplitude * (2.0 * angular_velocity * time).sin(),
            );

        // Current end-effector pose, as the robot reports it.
        let (position, rotation) = decompose(&state.O_T_EE);

        let position_error = position - position_d;
        error
            .fixed_view_mut::<3, 1>(0, 0)
            .copy_from(&position_error);
        // Orientation error, `-R * vec(q_ee^-1 q_d)`, hemisphere fix included.
        let rotation_error = orientation_error(&rotation, &orientation_d);
        error
            .fixed_view_mut::<3, 1>(3, 0)
            .copy_from(&rotation_error);

        let deviation = position_error.norm();
        worst_position_error = worst_position_error.max(deviation);
        // Tracking is only meaningful once the path is at full amplitude and before it
        // starts folding back up.
        if time >= RAMP_IN && ramp_down.is_none() {
            let bin = ((deviation * 1000.0).round() as usize).min(HISTOGRAM_BINS - 1);
            histogram[bin] += 1;
            samples += 1;
        }

        // The Cartesian spring-damper. Only the stiffness is ramped: keeping the damping at
        // its nominal `2 sqrt(K)` throughout makes the loop over-damped while `ramp < 1`,
        // never under-damped, and leaves a pure damper behind at the very end.
        let mut tau =
            jacobian.transpose() * (-(stiffness * error) * ramp - damping * (jacobian * dq));

        // The virtual floor: a one-sided spring along +z that exists only below `z_floor`.
        let penetration = z_floor - position.z;
        let floor_active = penetration > 0.0;
        if floor_active {
            floor_cycles += 1;
            wrench[2] = FLOOR_STIFFNESS * penetration * ramp;
        } else {
            wrench[2] = 0.0;
        }
        tau += jacobian.transpose() * wrench;

        // A joint spring toward the initial configuration, projected onto the Jacobian's
        // nullspace so it cannot disturb the Cartesian task:
        // `N = I - J^T (J J^T + lambda^2 I)^-1 J`, the damped inverse so that the projector
        // stays defined through a singularity instead of blowing up next to one.
        let jjt = jacobian * jacobian.transpose() + identity_6 * DAMPED_INVERSE_LAMBDA_SQUARED;
        if let Some(jjt_inverse) = jjt.try_inverse() {
            let nullspace = identity_7 - jacobian.transpose() * jjt_inverse * jacobian;
            tau += nullspace
                * ((q_initial - q) * (NULLSPACE_STIFFNESS * ramp) - dq * nullspace_damping);
        }

        // Clamp the task torques before the Coriolis feedforward, which is the robot's own
        // dynamics and not something this controller should be trimming.
        let mut clamped_here = false;
        for joint in 0..7 {
            let limited = tau[joint].clamp(-TORQUE_LIMIT, TORQUE_LIMIT);
            clamped_here |= limited != tau[joint];
            tau_d[joint] = limited + coriolis[joint];
            worst_torque = worst_torque.max(tau_d[joint].abs());
        }
        if clamped_here {
            clamped_cycles += 1;
            if !clamp_reported {
                clamp_reported = true;
                eprintln!("note: commanded torque hit the +-{TORQUE_LIMIT} Nm clamp");
            }
        }

        if time >= next_status {
            next_status += 1.0;
            eprintln!(
                "  {time:6.2}s  {:6.1}mm   {ramp:4.2}   {:5}   {clamped_cycles:7}",
                deviation * 1000.0,
                if floor_active { "on" } else { "off" }
            );
        }

        // An operator has grabbed the hand and walked it away from the path: stop driving.
        // Raised here, so it takes effect from the next cycle's `ramp`.
        if ramp_down.is_none() && deviation > MAX_DEVIATION {
            ramp_down = Some((time, ABORT_RAMP_OUT, Reason::Deviation));
        }

        let finished = matches!(ramp_down, Some((started_at, seconds, _))
            if time >= started_at + seconds);
        if finished {
            // The crate's convention for leaving a torque loop: a final command with
            // `motion_finished` set. Zero torques, as in the `ActiveControl` documentation --
            // the robot's own controller takes the arm from here. After the ramp there is
            // nothing left to step down from.
            control.write_once(&motion_finished(Torques::new([0.0; 7])))?;
            break;
        }
        control.write_once(&Torques::new(tau_d))?;
    }

    println!(
        "\nFinished after {time:.3} s ({cycles} cycles), control_command_success_rate \
         {success_rate}"
    );
    println!(
        "|p - p_d| after the ramp-in: p50 {:.0} mm, max {:.1} mm over {samples} cycles",
        median_mm(&histogram, samples),
        worst_position_error * 1000.0
    );
    println!("max |tau_d| = {worst_torque:.2} Nm, clamped on {clamped_cycles} cycles");
    println!("virtual floor active on {floor_cycles} cycles");
    match ramp_down {
        Some((_, _, Reason::Interrupt)) => println!("Stopped by SIGINT."),
        Some((_, _, Reason::Deviation)) => println!(
            "Stopped: the end effector was more than {:.0} cm off the path -- someone had \
             hold of it.",
            MAX_DEVIATION * 100.0
        ),
        _ => {}
    }
    Ok(())
}
