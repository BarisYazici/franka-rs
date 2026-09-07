//! An example showing how to continuously read the robot state.
//!
//! Port of libfranka's `examples/echo_robot_state.cpp`.
//!
//! Usage: `echo_robot_state <robot-hostname>`

#[path = "common/mod.rs"]
mod common;

use franka::{FrankaResult, Robot};

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

    // Deviation from the upstream C++: `franka::RobotState` has an `operator<<` that prints a
    // JSON-like object; this crate's `RobotState` does not implement `Display` (only `Debug`),
    // so this prints the derived `Debug` form instead.
    let mut count: usize = 0;
    robot.read(|robot_state| {
        // Printing to stdout adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        println!("{robot_state:?}");
        let current = count;
        count += 1;
        current < 100
    })?;

    println!("Done.");
    Ok(())
}
