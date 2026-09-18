//! `franka-node <config.toml>`: [`franka_node::run`] with a gripper factory that knows the
//! Franka Hand (`gripper = "hand"`) and refuses every other driver name.

use std::sync::Arc;

use franka::Robot;
use franka_node::{ArmConfig, FrankaHand, Gripper, NodeConfig};
use log::error;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: franka-node <config.toml>");
        std::process::exit(2);
    };
    let config = match NodeConfig::from_path(&path) {
        Ok(config) => config,
        Err(e) => {
            error!("{path}: {e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = franka_node::run(config, hand_factory) {
        error!("{e}");
        std::process::exit(1);
    }
}

/// `"hand"` is a [`FrankaHand`] at the arm's host; nothing else is known here.
fn hand_factory(arm: &ArmConfig, _robot: &Arc<Robot>) -> Result<Option<Box<dyn Gripper>>, String> {
    match arm.gripper.as_deref() {
        None => Ok(None),
        Some("hand") => FrankaHand::connect(&arm.host, arm.gripper_speed)
            .map(|hand| Some(Box::new(hand) as Box<dyn Gripper>))
            .map_err(|e| format!("gripper: {e}")),
        Some(other) => Err(format!("unknown gripper driver {other:?}")),
    }
}
