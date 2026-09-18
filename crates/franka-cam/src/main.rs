//! `franka-cam <config.toml>`: [`franka_cam::run`] with the log filter from `RUST_LOG`.

use franka_cam::CamConfig;
use log::error;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: franka-cam <config.toml>");
        std::process::exit(2);
    };
    let config = match CamConfig::from_path(&path) {
        Ok(config) => config,
        Err(e) => {
            error!("{path}: {e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = franka_cam::run(config) {
        error!("{e}");
        std::process::exit(1);
    }
}
