//! An example showing how to generate a joint velocity motion.
//!
//! Port of libfranka's `examples/generate_joint_velocity_motion.cpp`.
//!
//! Usage: `generate_joint_velocity_motion <robot-hostname>`
//!
//! # Warning
//! Before running this example, make sure there is enough space in front of the robot.

#[path = "common/mod.rs"]
mod common;

use franka::{
    motion_finished, ControllerMode, FrankaResult, JointVelocities, Robot, DEFAULT_CUTOFF_FREQUENCY,
};

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

fn run(hostname: &str) -> FrankaResult<()> {
    let realtime_config = common::realtime_config_from_env();
    let robot = Robot::new(hostname, realtime_config)?;
    common::set_default_behavior(&robot)?;

    // First move the robot to a suitable joint configuration.
    let mut motion_generator =
        common::MotionGenerator::new(robot.fci_version(), 0.5, common::READY_POSE);
    common::wait_for_enter();
    robot.control_joint_positions(
        |state, period| motion_generator.step(state, period),
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;
    println!("Finished moving to initial joint configuration.");

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.set_collision_behavior(
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    )?;

    let time_max = 1.0;
    let omega_max = 1.0;
    let mut time = 0.0;
    robot.control_joint_velocities(
        |_state, period| {
            time += period.as_secs_f64();

            let cycle = (-1.0f64)
                .powf(((time - time % time_max) / time_max).floor())
                .floor();
            let omega = cycle * omega_max / 2.0
                * (1.0 - (2.0 * std::f64::consts::PI / time_max * time).cos());

            let velocities = JointVelocities::new([0.0, 0.0, 0.0, omega, omega, omega, omega]);

            if time >= 2.0 * time_max {
                println!("\nFinished motion, shutting down example");
                motion_finished(velocities)
            } else {
                velocities
            }
        },
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;

    Ok(())
}
