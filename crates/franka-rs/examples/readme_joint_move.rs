//! The joint-move example from the README, kept here so it is compiled by
//! `cargo build --examples` / `cargo test --workspace`, run by CI's `sim-nominal` job against
//! the franka-sim server, and never drifts from what the README shows.
//!
//! The body of `main` below -- the 20 lines between `fn main() ... {` and `Ok(())` -- is
//! byte-identical to the fenced code block in README.md's "Quick example" section, including
//! its indentation. `crates/franka-rs/src/readme_sync.rs` asserts exactly that, and that the
//! block is at most 20 lines of code, as unit tests -- so they run in CI's Docker-free `check`
//! job.
//!
//! Usage: `readme_joint_move <robot-hostname>`, with `FRANKA_REALTIME=ignore|enforce`
//! (default `enforce`, as in libfranka) like every other example here.
use franka::{ControllerMode, JointPositions, RealtimeConfig, Robot, DEFAULT_CUTOFF_FREQUENCY};
use std::{env, f64::consts::PI};

fn main() -> franka::FrankaResult<()> {
    let realtime = match env::var("FRANKA_REALTIME").as_deref() {
        Ok("ignore") => RealtimeConfig::Ignore,
        _ => RealtimeConfig::Enforce,
    };
    let robot = Robot::new(&env::args().nth(1).expect("usage: <hostname>"), realtime)?;
    // `q_d`, the commanded position: FCI v5 rate-limits the first command against it.
    let initial = robot.read_once()?.q_d;
    let mut time = 0.0;
    robot.control_joint_positions(
        |_state, period| {
            time += period.as_secs_f64();
            let mut q = initial;
            q[3] += PI / 8.0 * (1.0 - (PI / 2.5 * time).cos());
            let mut output = JointPositions::new(q);
            output.motion_finished = time >= 5.0;
            output
        },
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;
    Ok(())
}
