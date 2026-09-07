//! An example showing how to generate a joint position motion.
//!
//! Port of libfranka's `examples/generate_joint_position_motion.cpp`.
//!
//! Usage: `generate_joint_position_motion <robot-hostname>`
//!
//! # Warning
//! Before running this example, make sure there is enough space in front of the robot.

#[path = "common/mod.rs"]
mod common;

use franka::{
    motion_finished, ControllerMode, Duration, FciVersion, FrankaResult, JointPositions, Robot,
    RobotState, DEFAULT_CUTOFF_FREQUENCY,
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

    let mut initial_position = [0.0; 7];
    let version = robot.fci_version();
    let mut time = 0.0;
    robot.control_joint_positions(
        |robot_state: &RobotState, period: Duration| {
            time += period.as_secs_f64();

            if time == 0.0 {
                // libfranka 0.9.2 `generate_joint_position_motion.cpp:52` seeds from the
                // commanded `q_d`, 0.21.2 `:52` from the measured `q`. The difference matters
                // on FCI v5, whose `convertMotion` always rate-limits against the robot's own
                // `q_d`: a `q` seed makes the very first command imply a non-zero commanded
                // joint velocity. See `examples/common/mod.rs`'s `MotionGenerator::step`.
                initial_position = match version {
                    FciVersion::V5 => robot_state.q_d,
                    FciVersion::V10 => robot_state.q,
                };
            }

            let delta_angle =
                std::f64::consts::PI / 8.0 * (1.0 - (std::f64::consts::PI / 2.5 * time).cos());

            let output = JointPositions::new([
                initial_position[0],
                initial_position[1],
                initial_position[2],
                initial_position[3] + delta_angle,
                initial_position[4] + delta_angle,
                initial_position[5],
                initial_position[6] + delta_angle,
            ]);

            if time >= 5.0 {
                println!("\nFinished motion, shutting down example");
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;

    Ok(())
}
