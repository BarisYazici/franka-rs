//! A Zenoh node in front of [`franka`]'s target control, Cartesian or joint: wire messages,
//! the target gate, the TOML configuration, the per-arm thread, the Zenoh transport, the
//! status topic and the arm's gripper. [`run`] is the node; the binary is a `main` around it
//! with a [`GripperFactory`] that knows the Franka Hand.

pub mod arm;
pub mod config;
pub mod gripper;
pub mod guard;
pub mod msg;
pub mod status;
pub mod transport;

pub use arm::{
    ArmHandle, ArmSender, Control, Event, GripperSide, Reply, RobotSide, Verb, CHANNEL_DEPTH,
    GRIPPER_STATE_HZ, HOME_SETTLE_TICKS, HOME_SPEED, HOME_TIMEOUT, HOME_TOLERANCE, READY,
};
pub use config::{
    cartesian_gains, ArmConfig, ConfigError, LeashConfig, NodeConfig, Realtime, ZenohConfig,
    ZenohMode, RESERVED_ARM_NAMES,
};
/// The library this node drives, so a [`GripperFactory`] outside this crate names
/// [`franka::Robot`] at the same version.
pub use franka;
pub use gripper::{FrankaHand, Gripper, GripperState};
pub use guard::{
    Axis, Guard, GuardOptions, JointLimits, Reason, Verdict, Workspace, UNIT_QUATERNION_TOLERANCE,
};
pub use msg::{
    robot_mode_code, CmdReply, CmdRequest, DecodeError, EpisodeMsg, EpisodePhase, GripperKind,
    GripperMsg, GripperStateMsg, Kind, Phase, StateMsg, TargetMsg, FLAG_HOLDING, FLAG_JOINTS,
    GRIPPER_CALIBRATED, GRIPPER_FAULT, GRIPPER_GRASPED, GRIPPER_MOVING, GRIPPER_SIZE, STATE_SIZE,
    TARGET_ANCHOR, TARGET_SIZE, VERSION,
};
pub use status::{ArmStats, ArmStatus, GripperStatus, Status};
pub use transport::{Attached, StatusPublisher};

use std::sync::mpsc;
use std::sync::Arc;

use franka::Robot;
use log::info;
use zenoh::Wait;

/// Nanoseconds on the host's `CLOCK_MONOTONIC`: the node's `t_node_ns` and a client's
/// `t_send_ns`.
///
/// The clock is the host's, not the process's, so a second process on the same host, a camera
/// node for instance, timestamps its own samples on the same scale and the two line up without
/// exchanging anything. Across hosts it means nothing until the clocks are aligned.
pub fn monotonic_ns() -> u64 {
    franka::realtime::monotonic_ns()
}

/// This process's boot id: a hex string, fresh per start, the same for every arm of the node.
///
/// It is how a panel learns that the node restarted under it. `version` alone cannot say so --
/// a restart puts it back to 0, which is also where it sits before anyone has tuned anything --
/// so the pair is what identifies a set of values. Built from the wall clock and the pid rather
/// than from a uuid dependency; two nodes started in the same nanosecond on one host would need
/// the same pid to collide.
pub fn boot_id() -> &'static str {
    static BOOT_ID: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    BOOT_ID.get_or_init(|| {
        let since_epoch = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |elapsed| elapsed.as_nanos());
        format!("{since_epoch:x}-{:x}", std::process::id())
    })
}

/// Builds an arm's gripper from its config (`ArmConfig::gripper`, `gripper_speed`) and its
/// connected robot: `Ok(None)` for an arm without one, `Err` for a name it does not know.
pub type GripperFactory = fn(&ArmConfig, &Arc<Robot>) -> Result<Option<Box<dyn Gripper>>, String>;

/// The node: connects every `[[arm]]` of `config` and its gripper through `factory`, serves
/// them over Zenoh, publishes the status once a second, and on SIGINT or SIGTERM stops each
/// arm and returns.
///
/// Each `Robot` is constructed on a scratch thread: under `RealtimeConfig::Enforce`
/// `Robot::new` puts its caller on `SCHED_FIFO`, and every thread spawned afterwards would
/// inherit that. The node's own threads (the caller's, the arm threads, the signal handler)
/// run at normal priority; only the library's loop thread is realtime.
pub fn run(config: NodeConfig, factory: GripperFactory) -> Result<(), String> {
    let text = |e: zenoh::Error| e.to_string();
    let session = transport::open(&config.zenoh).map_err(text)?;
    info!(
        "node {}: {} mode, listening on {:?}, connecting to {:?}, multicast scouting {}",
        config.name,
        config.zenoh.mode.as_str(),
        config.zenoh.listen_endpoints(),
        config.zenoh.connect,
        if config.zenoh.multicast_enabled() {
            "on"
        } else {
            "off"
        }
    );
    let mut arms = Vec::with_capacity(config.arms.len());
    let mut stats = Vec::with_capacity(config.arms.len());
    // Every arm's name, for the layout of a recording the arms share.
    let names: Vec<String> = config.arms.iter().map(|arm| arm.name.clone()).collect();
    for arm in &config.arms {
        info!("arm {}: connecting to {}", arm.name, arm.host);
        let (host, realtime) = (arm.host.clone(), arm.realtime_config());
        let robot = std::thread::spawn(move || Robot::new(&host, realtime))
            .join()
            .map_err(|_| format!("arm {}: connect thread panicked", arm.name))?
            .map_err(|e| format!("arm {}: {e}", arm.name))?;
        let robot = Arc::new(robot);
        let gripper = factory(arm, &robot).map_err(|e| format!("arm {}: {e}", arm.name))?;
        let attached =
            transport::attach(&session, arm.clone(), &names, robot, gripper).map_err(text)?;
        stats.push((arm.name.clone(), Arc::clone(attached.stats())));
        arms.push(attached);
    }
    let status = transport::status_publisher(&session, &config.name, stats).map_err(text)?;
    let (signalled, signal) = mpsc::channel();
    ctrlc::set_handler(move || {
        let _ = signalled.send(());
    })
    .map_err(|e| e.to_string())?;
    let _ = signal.recv();
    info!("signal received, stopping {} arm(s)", arms.len());
    status.shutdown();
    for arm in arms {
        arm.shutdown();
    }
    session.close().wait().map_err(text)
}
