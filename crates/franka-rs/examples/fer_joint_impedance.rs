//! A 1 kHz joint-impedance torque controller for a Franka Emika Robot (FER, FCI v5).
//!
//! Usage: `fer_joint_impedance <robot-hostname>`
//!
//! The control law is the one a typical libfranka 0.9.2 joint-impedance setup runs on an
//! FER, ported to franka-rs:
//!
//! ```text
//! tau = K (q_d - q) - D dq + coriolis(state)
//! K = [600, 600, 600, 600, 250, 150,  50]  Nm/rad
//! D = [ 50,  50,  50,  50,  30,  25,  15]  Nm s/rad
//! ```
//!
//! with the commanded torque rate-limited by hand against the robot's own `tau_J_d`, using
//! the FER's `kMaxTorqueRate` (1000 - 1e-3 Nm/s per joint, `rate_limiting::fer`). The
//! control loop is therefore started with `limit_rate = false` and
//! [`franka::MAX_CUTOFF_FREQUENCY`]: this controller does its own rate limiting and wants no
//! low-pass filter between it and the robot.
//!
//! On FCI v5 there is no `MotionGeneratorMode::kNone`, so `control_torques` starts a
//! joint-velocity motion generator commanding zero velocity alongside the external
//! controller, the way libfranka 0.9.2's `Robot::control(ControlCallback)` does. Nothing about
//! that is visible here.
//!
//! The trajectory is a 1 s wake-up, a 5 s hold at the pose measured on the first control
//! cycle, a 2 s cosine excursion of +0.2 rad on joint 1, a 1 s dwell and a 2 s cosine return,
//! i.e. 11 s in total. At the end the worst tracking error and the loop's cycle statistics are
//! printed.
//!
//! Three things keep the stiffness safe to point at hardware:
//!
//! * The example **refuses to run on anything but an FER.** The gains, the torque limits and
//!   the torque rate limit below are the FER's, and on FCI v10 `tau_J_d` reaches the client
//!   quantised to `f32`, so the crate shrinks the v10 rate budget by that quantisation --
//!   which the hand-rolled limiter here does not. Use the `generate_*` examples on an FR3.
//! * The held pose is read inside the loop, *after* the operator prompt, so nothing the arm
//!   does while it waits -- sag, a nudge, a hand-guided move -- becomes a wake-up error.
//!   K and D are then ramped from 0 to their full values over [`WAKE_UP`] seconds with a
//!   cosine, so however large that first error turns out to be, the torque it produces starts
//!   at zero and rises with zero slope: the first cycles cannot be a torque step.
//! * The commanded torque is clamped to the FER's joint torque limits (87/87/87/87/12/12/12
//!   Nm) before the rate limiter, because the impedance law itself is unbounded -- 0.1 rad of
//!   error on joint 5 is 25 Nm against a 12 Nm limit.
//!
//! Set `FRANKA_REALTIME=ignore` to run against franka-sim on an ordinary kernel;
//! `FRANKA_REALTIME=enforce` (the default) is what a `PREEMPT_RT` box with a real FER wants.
//!
//! # Warning
//! Before running this on hardware, make sure there is enough space around the robot and have
//! the emergency stop within reach: this is a stiff torque controller, and joint 1 moves.
//! There is no SIGINT handler: Ctrl-C kills the loop mid-motion and leaves the arm to the
//! robot's own `communication_constraints_violation` watchdog, as it does in libfranka.

#[path = "common/mod.rs"]
mod common;

use franka::rate_limiting::fer::MAX_TORQUE_RATE;
use franka::{
    limit_rate_torques, motion_finished, Duration, FciVersion, FrankaError, FrankaResult, Robot,
    RobotState, Torques, MAX_CUTOFF_FREQUENCY,
};

/// Joint stiffness, Nm/rad.
const K: [f64; 7] = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0];
/// Joint damping, Nm s/rad.
const D: [f64; 7] = [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0];

/// Seconds spent holding the start pose before the excursion.
const HOLD: f64 = 5.0;
/// Seconds spent ramping joint 1 out, and again ramping it back.
const RAMP: f64 = 2.0;
/// Seconds spent at the far end of the excursion.
const DWELL: f64 = 1.0;
/// Excursion of joint 1, rad.
const EXCURSION: f64 = 0.2;
/// Seconds spent ramping the impedance gains from zero to `K`/`D`. See [`WAKE_UP`]'s use in
/// `run`.
const WAKE_UP: f64 = 1.0;

/// Maximum joint torque, Nm (`franka::Robot`'s FER datasheet limits: 87 Nm on joints 1-4,
/// 12 Nm on joints 5-7).
///
/// The impedance law above is not bounded by anything else. `limit_rate_torques` bounds the
/// *rate* of change, not the magnitude, so a large setpoint error simply takes ~0.1 s to become
/// a large torque: 0.1 rad on joint 5 is `250 * 0.1 = 25` Nm against a 12 Nm limit, which on a
/// real FER is an immediate torque-limit reflex. Clamping before the rate limiter keeps the
/// commanded value inside the robot's envelope and keeps the rate limiting exact (the clamp is
/// what is fed back as `last`, via the robot's own `tau_J_d`).
const MAX_TORQUE: [f64; 7] = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0];

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        eprintln!("Usage: {} <robot-hostname>", args[0]);
        std::process::exit(1);
    }

    if let Err(e) = run(&args[1]) {
        println!("{e}");
        std::process::exit(1);
    }
}

/// The commanded joint-1 offset at `time` seconds into the motion: nothing until the wake-up
/// ramp has finished, then a hold, a cosine ramp out, a dwell, a cosine ramp back, and zero.
/// Smooth in position and velocity at every junction, so the stiff controller never sees a
/// step.
fn joint1_offset(time: f64) -> f64 {
    let smoothstep = |progress: f64| 0.5 * (1.0 - (std::f64::consts::PI * progress).cos());
    let time = time - WAKE_UP;
    if time < HOLD {
        0.0
    } else if time < HOLD + RAMP {
        EXCURSION * smoothstep((time - HOLD) / RAMP)
    } else if time < HOLD + RAMP + DWELL {
        EXCURSION
    } else if time < HOLD + 2.0 * RAMP + DWELL {
        EXCURSION * (1.0 - smoothstep((time - HOLD - RAMP - DWELL) / RAMP))
    } else {
        0.0
    }
}

fn run(hostname: &str) -> FrankaResult<()> {
    let realtime_config = common::realtime_config_from_env();
    let robot = Robot::new(hostname, realtime_config)?;
    println!("Negotiated FCI version: {:?}", robot.fci_version());
    if robot.fci_version() != FciVersion::V5 {
        // Not a warning: on FCI v10 `tau_J_d` arrives f32-quantised, so `MAX_TORQUE_RATE`
        // below is over the nominal rate this loop is allowed to command and a real FR3
        // answers with a torque-rate reflex. The gains and the 87/12 Nm limits are the
        // FER's too.
        return Err(FrankaError::InvalidOperation(format!(
            "libfranka: {hostname} negotiated {:?}, but this example is FER-only \
             (FCI v5): its gains, torque limits and hand-rolled torque rate limit are the \
             FER's, and on FCI v10 the rate budget has to be shrunk by the f32 quantisation \
             of tau_J_d. On an FR3 run one of the generate_joint_position_motion, \
             generate_joint_velocity_motion, generate_cartesian_pose_motion or \
             generate_cartesian_velocity_motion examples instead.",
            robot.fci_version()
        )));
    }
    common::set_default_behavior(&robot)?;

    // The Coriolis/centrifugal term comes from the robot's own model library, downloaded with
    // `LoadModelLibrary` on FCI v5.
    let model = robot.load_model()?;

    println!(
        "Starting the torque loop: {WAKE_UP} s wake-up, {HOLD} s hold, {RAMP} s out to \
         {EXCURSION} rad on joint 1, {DWELL} s dwell, {RAMP} s back."
    );
    common::wait_for_enter();

    // The held pose is read on the *first control cycle*, not before the prompt: whatever the
    // arm does while it waits for Enter -- sag, a nudge, a hand-guided move -- would otherwise
    // become the error this stiff loop wakes up with. Every other example in the crate seeds
    // its trajectory the same way.
    //
    // Seeding the setpoint from the measured `q` makes the first cycle's error zero, but only
    // for the cycle it was measured on: the arm keeps moving, and gravity sag alone can put a
    // few milliradians of error under a 600 Nm/rad stiffness within a cycle or two. So the
    // gains themselves are ramped in over [`WAKE_UP`] seconds -- see `wake` below -- which
    // bounds the torque of those first cycles by construction rather than by assumption.
    let mut q_hold: Option<[f64; 7]> = None;

    let total = WAKE_UP + HOLD + 2.0 * RAMP + DWELL;
    let mut time = 0.0;
    let mut worst_error = 0.0f64;
    let mut worst_error_joint = 0usize;
    let mut cycles = 0usize;
    let mut exactly_one_millisecond = 0usize;
    let mut worst_period = 0u64;
    let mut success_rate = 0.0;

    robot.control_torques(
        |state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            cycles += 1;
            if cycles > 1 {
                // The first cycle always reports a zero period.
                worst_period = worst_period.max(period.as_millis());
                if period.as_millis() == 1 {
                    exactly_one_millisecond += 1;
                }
            }
            success_rate = state.control_command_success_rate;

            let q_start = *q_hold.get_or_insert(state.q);
            let mut q_d = q_start;
            q_d[0] += joint1_offset(time);

            // Gain ramp: `wake` is 0 at t = 0 and 1 from `WAKE_UP` on, with zero slope at both
            // ends, so the impedance torque starts at exactly zero and rises smoothly. The
            // setpoint is the held pose from the first cycle, so the *error* starts near zero
            // too, but the ramp is what makes "no torque step on the first cycles" a property
            // of the loop rather than an assumption about the arm.
            let wake = 0.5 * (1.0 - (std::f64::consts::PI * (time / WAKE_UP).min(1.0)).cos());

            // The Coriolis feedforward is not ramped: it is a model term that cancels a real
            // force, it is zero at rest, and scaling it down would make the wake-up worse.
            let coriolis = model.coriolis(state);
            let mut tau = [0.0; 7];
            for joint in 0..7 {
                let error = q_d[joint] - state.q[joint];
                if error.abs() > worst_error {
                    worst_error = error.abs();
                    worst_error_joint = joint;
                }
                tau[joint] = (wake * (K[joint] * error - D[joint] * state.dq[joint])
                    + coriolis[joint])
                    .clamp(-MAX_TORQUE[joint], MAX_TORQUE[joint]);
            }

            // Rate limit by hand against the torque the robot last accepted.
            // `limit_rate_torques` only fails on a non-finite command, which the control law
            // above cannot produce from a finite state; falling back to the last accepted
            // torque keeps the closure infallible without hiding a real fault.
            let limited =
                limit_rate_torques(&MAX_TORQUE_RATE, &tau, &state.tau_J_d).unwrap_or(state.tau_J_d);

            let output = Torques::new(limited);
            if time >= total {
                motion_finished(output)
            } else {
                output
            }
        },
        // No client-side rate limiting and no low-pass filter: this controller does its own.
        false,
        MAX_CUTOFF_FREQUENCY,
    )?;

    println!("\nFinished.");
    if let Some(q_hold) = q_hold {
        println!("Held q = {q_hold:?}");
    }
    println!(
        "max |q_d - q| = {:.6} rad (joint {})",
        worst_error,
        worst_error_joint + 1
    );
    let measured = cycles.saturating_sub(1).max(1);
    println!(
        "cycles: {cycles}, {:.2} % exactly 1 ms, worst period {worst_period} ms, \
         control_command_success_rate {success_rate}",
        100.0 * exactly_one_millisecond as f64 / measured as f64
    );

    Ok(())
}
