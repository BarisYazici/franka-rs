//! A Cartesian impedance controller, driven through the `ActiveControl` API.
//!
//! Port of libfranka's `examples/cartesian_impedance_control.cpp`, with the callback replaced
//! by [`franka::Robot::start_torque_control`] and a hand-written `read_once` / `write_once`
//! loop. The control law is unchanged: a spring-damper system without inertia shaping whose
//! equilibrium is the pose the end effector had when the loop started. After starting it, try
//! to push the robot around.
//!
//! ```text
//! tau = J(q)^T (-K e - D J(q) dq) + coriolis(q, dq)
//! e   = [ p - p_d ; -R * vec(q_ee^-1 q_d) ]
//! K   = diag(150, 150, 150, 10, 10, 10)          N/m, Nm/rad
//! D   = 2 sqrt(K)
//! ```
//!
//! Usage: `cartesian_impedance_active_control <robot-hostname> [--duration SEC] [--yes]`
//!
//! `--duration` defaults to 30 s. The loop also stops on Ctrl-C: unlike
//! `fer_joint_impedance`, this example installs a `SIGINT` handler that only raises a flag, so
//! the loop leaves through the same path as a normal timeout — a final `write_once` of
//! [`franka::motion_finished`] torques — instead of dying mid-motion and leaving the arm to the
//! robot's `communication_constraints_violation` watchdog. Ctrl-C twice is not handled; the
//! second one hits the default disposition only if the handler is reset, which it is not, so a
//! wedged loop still needs the user stop.
//!
//! Runs on both generations. On an FR3 (FCI v10) `start_torque_control` opens a torque-only
//! external controller; on an FER (FCI v5) there is no such mode, so the crate opens a
//! joint-velocity motion generator commanding zeros alongside the external controller, the way
//! libfranka 0.9.2's `Robot::control(ControlCallback)` does. Nothing about that is visible
//! here, and `Robot::load_model` returns a native model on both.
//!
//! Set `FRANKA_REALTIME=ignore` to run against franka-sim on an ordinary kernel;
//! `FRANKA_REALTIME=enforce` (the default) is what a `PREEMPT_RT` box with a real robot wants.
//!
//! # Warning
//! The collision thresholds this example sets are the C++ example's, and they are high (100 Nm
//! / 100 N on every axis). Have the user stop button at hand.

#[path = "common/mod.rs"]
mod common;

use std::sync::atomic::{AtomicBool, Ordering};

use nalgebra::{Rotation3, SMatrix, SVector, UnitQuaternion};

use franka::{motion_finished, Frame, FrankaResult, Robot, RobotState, Torques};

use common::cartesian::{compliance, decompose, orientation_error};

/// Translational stiffness, N/m.
const TRANSLATIONAL_STIFFNESS: f64 = 150.0;
/// Rotational stiffness, Nm/rad.
const ROTATIONAL_STIFFNESS: f64 = 10.0;

/// Seconds the loop runs for when `--duration` is not given.
const DEFAULT_DURATION: f64 = 30.0;

/// Raised by the `SIGINT` handler; polled once per control cycle.
static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigint(_signal: libc::c_int) {
    // The only async-signal-safe thing done here: a relaxed store on a lock-free `AtomicBool`.
    STOP.store(true, Ordering::Relaxed);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut hostname: Option<&str> = None;
    let mut duration = DEFAULT_DURATION;
    let mut assume_yes = false;

    let mut arg = 1;
    while arg < args.len() {
        match args[arg].as_str() {
            "--yes" => assume_yes = true,
            "--duration" => {
                arg += 1;
                match args.get(arg).and_then(|value| value.parse::<f64>().ok()) {
                    Some(seconds) if seconds > 0.0 => duration = seconds,
                    _ => {
                        eprintln!("--duration wants a positive number of seconds");
                        std::process::exit(1);
                    }
                }
            }
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
            "Usage: {} <robot-hostname> [--duration SEC] [--yes]",
            args[0]
        );
        std::process::exit(1);
    };

    if let Err(e) = run(hostname, duration, assume_yes) {
        println!("{e}");
        std::process::exit(1);
    }
}

fn run(hostname: &str, duration: f64, assume_yes: bool) -> FrankaResult<()> {
    let realtime_config = common::realtime_config_from_env();
    let robot = Robot::new(hostname, realtime_config)?;
    println!("Negotiated FCI version: {:?}", robot.fci_version());
    common::set_default_behavior(&robot)?;

    // Load the kinematics and dynamics model. Native on both generations: the FR3's own
    // `GetRobotModel` URDF, and the compiled-in `FER_URDF` on an FER.
    let model = robot.load_model()?;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior. These are the C++ example's thresholds, which are deliberately
    // high so that pushing the arm around does not trip a reflex; `set_default_behavior`
    // above set the usual 20/10 Nm ones, and this call replaces them.
    robot.set_collision_behavior_simple([100.0; 7], [100.0; 7], [100.0; 6], [100.0; 6])?;

    println!(
        "WARNING: Collision thresholds are set to high values. Make sure you have the user \
         stop at hand!"
    );
    println!("After starting try to push the robot and see how it reacts.");
    println!("The loop runs for {duration} s, or until Ctrl-C.");
    if assume_yes {
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

    // The equilibrium pose is read *after* the prompt, like every other example in this crate
    // seeds its trajectory: whatever the arm does while it waits -- sag, a nudge, a
    // hand-guided move -- would otherwise become the error the loop wakes up with.
    let initial_state: RobotState = robot.read_once()?;
    let (position_d, rotation_d) = decompose(&initial_state.O_T_EE);
    let orientation_d =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation_d));

    let (stiffness, damping) = compliance(TRANSLATIONAL_STIFFNESS, ROTATIONAL_STIFFNESS);

    // Everything the loop touches is either a fixed-size (stack) nalgebra type or a plain
    // array, and the model methods return fixed arrays, so no control cycle allocates.
    let mut error = SVector::<f64, 6>::zeros();
    let mut tau_d = [0.0f64; 7];

    let mut time = 0.0;
    let mut cycles = 0usize;
    let mut worst_position_error = 0.0f64;
    let mut worst_orientation_error = 0.0f64;
    // Assigned on every cycle before the loop can break, so no initialiser is needed.
    let mut success_rate;

    let mut control = robot.start_torque_control()?;
    loop {
        let (state, period) = control.read_once()?;
        time += period.as_secs_f64();
        cycles += 1;
        success_rate = state.control_command_success_rate;

        // Model terms for this state.
        let coriolis = SVector::<f64, 7>::from_column_slice(&model.coriolis(&state));
        let jacobian = SMatrix::<f64, 6, 7>::from_column_slice(
            &model.zero_jacobian(Frame::EndEffector, &state),
        );
        let dq = SVector::<f64, 7>::from_column_slice(&state.dq);

        // Current end-effector pose, as the robot reports it.
        let (position, rotation) = decompose(&state.O_T_EE);

        // Position error.
        let position_error = position - position_d;
        error
            .fixed_view_mut::<3, 1>(0, 0)
            .copy_from(&position_error);

        // Orientation error, `-R * vec(q_ee^-1 q_d)`, hemisphere fix included.
        let rotation_error = orientation_error(&rotation, &orientation_d);
        error
            .fixed_view_mut::<3, 1>(3, 0)
            .copy_from(&rotation_error);

        worst_position_error = worst_position_error.max(position_error.norm());
        worst_orientation_error = worst_orientation_error.max(rotation_error.norm());

        // Spring-damper system with damping ratio = 1, plus the Coriolis feedforward.
        let tau_task = jacobian.transpose() * (-(stiffness * error) - damping * (jacobian * dq));
        for joint in 0..7 {
            tau_d[joint] = tau_task[joint] + coriolis[joint];
        }

        let finished = time >= duration || STOP.load(Ordering::Relaxed);
        if finished {
            // The crate's convention for leaving a torque loop: a final command with
            // `motion_finished` set. Zero torques, as in the `ActiveControl` documentation --
            // the robot's own controller takes the arm from here.
            control.write_once(&motion_finished(Torques::new([0.0; 7])))?;
            break;
        }
        control.write_once(&Torques::new(tau_d))?;
    }

    println!(
        "\nFinished after {time:.3} s ({cycles} cycles), control_command_success_rate \
         {success_rate}"
    );
    println!("max |p - p_d| = {worst_position_error:e} m");
    println!("max orientation error = {worst_orientation_error:e} (quaternion vector norm)");
    if STOP.load(Ordering::Relaxed) {
        println!("Stopped by SIGINT.");
    }
    Ok(())
}
