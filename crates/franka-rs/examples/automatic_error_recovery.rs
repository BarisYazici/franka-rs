//! Clears a reflex or error state and reports the robot mode before and after, like a
//! command-line `automatic_error_recovery()`.
//!
//! Usage: `automatic_error_recovery <robot-hostname>`.

#[path = "common/mod.rs"]
mod common;

use franka::{FrankaResult, Robot};

fn main() -> FrankaResult<()> {
    let hostname = std::env::args().nth(1).expect("usage: <hostname>");
    let robot = Robot::new(&hostname, common::realtime_config_from_env())?;
    let before = robot.read_once()?;
    println!(
        "robot mode before: {:?}, errors: {}",
        before.robot_mode, before.current_errors
    );
    robot.automatic_error_recovery()?;
    let after = robot.read_once()?;
    println!(
        "robot mode after: {:?}, errors: {}",
        after.robot_mode, after.current_errors
    );
    Ok(())
}
