//! An example showing how to generate a Cartesian motion.
//!
//! Port of libfranka's `examples/generate_cartesian_pose_motion.cpp`.
//!
//! Usage: `generate_cartesian_pose_motion <robot-hostname>`
//!
//! # Warning
//! Before running this example, make sure there is enough space in front of the robot.

#[path = "common/mod.rs"]
mod common;

use franka::{
    motion_finished, CartesianPose, ControllerMode, Duration, FciVersion, FrankaResult, Robot,
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

    let mut initial_pose = [0.0; 16];
    let version = robot.fci_version();
    let mut time = 0.0;
    robot.control_cartesian_pose(
        |robot_state: &RobotState, period: Duration| {
            time += period.as_secs_f64();

            if time == 0.0 {
                // As in `generate_joint_position_motion`: libfranka 0.9.2
                // `generate_cartesian_pose_motion.cpp:52` seeds from the commanded `O_T_EE_c`,
                // 0.21.2 `:52` from the measured `O_T_EE`.
                initial_pose = match version {
                    FciVersion::V5 => robot_state.O_T_EE_c,
                    FciVersion::V10 => robot_state.O_T_EE,
                };
            }

            const RADIUS: f64 = 0.3;
            let angle =
                std::f64::consts::PI / 4.0 * (1.0 - (std::f64::consts::PI / 5.0 * time).cos());
            let delta_x = RADIUS * angle.sin();
            let delta_z = RADIUS * (angle.cos() - 1.0);

            let mut new_pose = initial_pose;
            new_pose[12] += delta_x;
            new_pose[14] += delta_z;

            let output = CartesianPose::new(new_pose);
            if time >= 10.0 {
                println!("\nFinished motion, shutting down example");
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::CartesianImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;

    Ok(())
}
