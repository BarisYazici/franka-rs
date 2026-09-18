//! `client <arm> gripper ...`: takes the lease, sends one gripper command, waits for the
//! gripper state to show it, prints that state, releases.

use std::time::{Duration, Instant};

use franka_node::{GripperKind, GripperStateMsg, GRIPPER_MOVING};
use zenoh::Wait;

use crate::args::{Args, GripperCmd};
use crate::common::{self, describe_gripper, verdict, GripperWatch};

/// How long a width or grasp command is given to show in the state.
const SETTLE: Duration = Duration::from_secs(10);
/// A width within this of the target counts as arrived, m.
const WIDTH_TOLERANCE: f64 = 0.003;

pub fn run(args: &Args, cmd: &GripperCmd) -> Result<(), String> {
    let arm = args.arm.as_str();
    let client_id = std::process::id();
    let (session, _watch) = crate::connect(args)?;
    let gripper = GripperWatch::subscribe(&session, arm).map_err(|e| e.to_string())?;
    let state = gripper
        .wait_for(Duration::from_secs(2), |_| true)
        .ok_or("no gripper state from the node within 2 s (no gripper configured?)")?;
    println!("gripper: {}", describe_gripper(&state));
    let token = common::lease(&session, arm, client_id)?;
    let began = Instant::now();
    let outcome = match cmd {
        GripperCmd::Home => {
            let reply = common::cmd(&session, arm, "gripper_home", client_id)?;
            println!(
                "gripper_home: {} after {:.1} s",
                verdict(&reply),
                began.elapsed().as_secs_f64()
            );
            reply.ok.then_some(()).ok_or("gripper_home refused")
        }
        GripperCmd::Width(width) => {
            publish(&session, arm, client_id, GripperKind::Width, *width, 0.0)?;
            let arrived = |s: &GripperStateMsg| (s.width.get() - width).abs() < WIDTH_TOLERANCE;
            report(&gripper, arrived, began, "width")
        }
        GripperCmd::Grasp(width, force) => {
            publish(&session, arm, client_id, GripperKind::Grasp, *width, *force)?;
            // Done once the driver reports the move over (moving set, then cleared).
            gripper.wait_for(Duration::from_secs(2), |s| s.flags & GRIPPER_MOVING != 0);
            let done = |s: &GripperStateMsg| s.flags & GRIPPER_MOVING == 0;
            report(&gripper, done, began, "grasp")
        }
    };
    let state = gripper.latest().ok_or("the gripper state stream stopped")?;
    println!("final: {}", describe_gripper(&state));
    common::cmd_ok(&session, arm, "release", client_id)?;
    println!("released");
    token.undeclare().wait().map_err(|e| e.to_string())?;
    session.close().wait().map_err(|e| e.to_string())?;
    outcome.map_err(String::from)
}

fn publish(
    session: &zenoh::Session,
    arm: &str,
    client_id: u32,
    kind: GripperKind,
    width: f64,
    force: f64,
) -> Result<(), String> {
    let publisher = session
        .declare_publisher(format!("franka/{arm}/gripper/target"))
        .wait()
        .map_err(|e| e.to_string())?;
    common::publish_gripper(&publisher, kind, client_id, 1, width, force).map_err(|e| e.to_string())
}

/// Waits up to [`SETTLE`] for `done` and prints how long it took.
fn report(
    gripper: &GripperWatch,
    done: impl Fn(&GripperStateMsg) -> bool,
    began: Instant,
    what: &str,
) -> Result<(), &'static str> {
    match gripper.wait_for(SETTLE, done) {
        Some(_) => {
            println!(
                "{what}: reached after {:.1} s",
                began.elapsed().as_secs_f64()
            );
            Ok(())
        }
        None => Err("the gripper state did not show the command within 10 s"),
    }
}
