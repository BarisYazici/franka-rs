//! An example showing how to control the Franka Hand gripper.
//!
//! Port of libfranka's `examples/grasp_object.cpp`.
//!
//! # Deviation from libfranka
//! Upstream's CLI is `grasp_object <gripper-hostname> <homing> <object-width>`, with `<homing>`
//! a `0`/`1` flag for whether to home the gripper first. This port's CLI is
//! `grasp_object <hostname> <object-width>` (no `<homing>` argument) and always homes -- i.e.
//! it behaves as upstream's `homing=1` -- since a caller has no way to opt out.
//!
//! Usage: `grasp_object <robot-hostname> <object-width>`

use franka::gripper::Gripper;
use franka::FrankaResult;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("Usage: {} <robot-hostname> <object-width>", args[0]);
        std::process::exit(1);
    }

    let Ok(grasping_width) = args[2].parse::<f64>() else {
        eprintln!("<object-width> must be a number, got {:?}.", args[2]);
        std::process::exit(1);
    };

    match run(&args[1], grasping_width) {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(e) => {
            println!("{e}");
            std::process::exit(1);
        }
    }
}

/// Returns `Ok(true)` on success, `Ok(false)` for the (non-exceptional) failure paths the C++
/// version reports by printing a message and returning `-1`.
fn run(hostname: &str, grasping_width: f64) -> FrankaResult<bool> {
    let gripper = Gripper::new(hostname)?;

    // Homing estimates the maximum grasping width with the current fingers; always done here,
    // see the module doc comment for why this differs from the upstream CLI.
    gripper.homing()?;

    // Check for the maximum grasping width.
    let gripper_state = gripper.read_once()?;
    if gripper_state.max_width < grasping_width {
        println!("Object is too large for the current fingers on the gripper.");
        return Ok(false);
    }

    // Grasp the object. `0.005`/`0.005` are libfranka's default epsilon_inner/epsilon_outer.
    if !gripper.grasp(grasping_width, 0.1, 60.0, 0.005, 0.005)? {
        println!("Failed to grasp object.");
        return Ok(false);
    }

    // Wait 3s and check afterwards whether the object is still grasped.
    std::thread::sleep(std::time::Duration::from_millis(3000));

    let gripper_state = gripper.read_once()?;
    if !gripper_state.is_grasped {
        println!("Object lost.");
        return Ok(false);
    }

    println!("Grasped object, will release it now.");
    gripper.stop()?;

    Ok(true)
}
