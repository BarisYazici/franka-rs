//! Moves the arm to libfranka's "ready" joint configuration with the examples' motion
//! generator, at a fraction of full speed.
//!
//! Usage: `move_to_ready <robot-hostname> [speed-factor, default 0.2] [--yes]`.
//!
//! # Warning
//! This example moves the robot. Make sure the path to the ready pose is free and have the
//! user stop button at hand.

#[path = "common/mod.rs"]
mod common;

use franka::{ControllerMode, FrankaResult, Robot, DEFAULT_CUTOFF_FREQUENCY};

fn main() -> FrankaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let hostname = args
        .get(1)
        .expect("usage: <hostname> [speed-factor] [--yes]");
    let speed_factor = args
        .iter()
        .skip(2)
        .find_map(|a| a.parse::<f64>().ok())
        .unwrap_or(0.2)
        .clamp(0.05, 1.0);
    let robot = Robot::new(hostname, common::realtime_config_from_env())?;
    common::set_default_behavior(&robot)?;
    let mut motion_generator =
        common::MotionGenerator::new(robot.fci_version(), speed_factor, common::READY_POSE);
    println!("Moving to the ready pose at speed factor {speed_factor}.");
    if !args.iter().any(|a| a == "--yes") {
        common::wait_for_enter();
    }
    robot.control_joint_positions(
        |state, period| motion_generator.step(state, period),
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;
    println!("Finished.");
    Ok(())
}
