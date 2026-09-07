//! An example indicating the network performance.
//!
//! Port of libfranka's `examples/communication_test.cpp`. Prints the same statistics as the
//! C++ version -- how many robot states the control loop missed, and the
//! min/avg/max `control_command_success_rate` over the run -- and, as a deliberate addition
//! over the upstream example (which only prints a warning), exits with a non-zero status when
//! the average success rate is below 0.9.
//!
//! Usage: `communication_test <robot-hostname>`
//!
//! # Warning
//! Before running this example, make sure there is enough space in front of the robot.

#[path = "common/mod.rs"]
mod common;

use franka::{ControllerMode, FrankaResult, Robot, Torques, DEFAULT_CUTOFF_FREQUENCY};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        eprintln!("Usage: {} <robot-hostname>", args[0]);
        std::process::exit(1);
    }

    let low_success_rate = match run(&args[1]) {
        Ok(low_success_rate) => low_success_rate,
        Err(e) => {
            println!("{e}");
            std::process::exit(1);
        }
    };

    if low_success_rate {
        std::process::exit(1);
    }
}

/// Returns whether the average `control_command_success_rate` was below 0.9.
fn run(hostname: &str) -> FrankaResult<bool> {
    let realtime_config = common::realtime_config_from_env();

    let mut counter: u64 = 0;
    let mut avg_success_rate = 0.0f64;
    let mut min_success_rate = 1.0f64;
    let mut max_success_rate = 0.0f64;
    let mut time: u64 = 0;

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
    println!("Finished moving to initial joint configuration.\n");
    println!("Starting communication test.");

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

    let mut zero_torques = Torques::new([0.0; 7]);
    let mut control = robot.start_torque_control()?;

    while !zero_torques.motion_finished {
        let (state, period) = control.read_once()?;

        time += period.as_millis();
        if time == 0 {
            control.write_once(&zero_torques)?;
            continue;
        }
        counter += 1;

        if counter % 100 == 0 {
            println!(
                "#{counter} Current success rate: {:.2}",
                state.control_command_success_rate
            );
        }
        std::thread::sleep(std::time::Duration::from_micros(100));

        avg_success_rate += state.control_command_success_rate;
        if state.control_command_success_rate > max_success_rate {
            max_success_rate = state.control_command_success_rate;
        }
        if state.control_command_success_rate < min_success_rate {
            min_success_rate = state.control_command_success_rate;
        }

        if time >= 10_000 {
            println!("\nFinished test, shutting down example");
            zero_torques.motion_finished = true;
        }
        // Sending zero torques - if EE is configured correctly, robot should not move.
        control.write_once(&zero_torques)?;
    }

    avg_success_rate /= counter as f64;

    println!();
    println!();
    println!("#######################################################");
    let lost_robot_states = time.saturating_sub(counter);
    if lost_robot_states > 0 {
        println!(
            "The control loop did not get executed {lost_robot_states} times in the\nlast \
             {time} milliseconds! (lost {lost_robot_states} robot states)\n"
        );
    }

    println!("Control command success rate of {counter} samples: ");
    println!("Max: {max_success_rate:.2}");
    println!("Avg: {avg_success_rate:.2}");
    println!("Min: {min_success_rate:.2}");

    if avg_success_rate < 0.90 {
        println!();
        println!("WARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!");
        println!("PLEASE TRY OUT A DIFFERENT PC / NIC");
    } else if avg_success_rate < 0.95 {
        println!();
        println!("WARNING: MANY PACKETS GOT LOST!");
        println!("PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON");
        println!("https://frankarobotics.github.io/docs/troubleshooting.html");
    }
    println!("#######################################################");
    println!();

    Ok(avg_success_rate < 0.90)
}
