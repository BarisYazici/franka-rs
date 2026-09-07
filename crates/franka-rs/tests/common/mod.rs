//! Helpers shared by the simulator integration tests.
//!
//! The simulator itself is managed by the `franka-sim-test` crate; set `FRANKA_SIM_IMAGE` to
//! pick the image (e.g. `franka-sim:dev`) and `FRANKA_SIM_ADDR` to reuse an already running
//! server instead of starting a container.
//!
//! Only one FCI client may be connected at a time, so every helper here returns a value whose
//! `Drop` closes the connection; a test must drop it before connecting again.

#![allow(dead_code)]

/// libfranka's `examples/examples_common.{h,cpp}` port, shared verbatim with the examples so
/// that the tests home the arm with the same point-to-point motion generator they do. Cargo
/// does not link `examples/` into integration tests, so the file is included by path.
#[path = "../../examples/common/mod.rs"]
pub mod examples_common;

use std::time::Duration;

use franka::error::{FrankaError, FrankaResult};
use franka::network::{connect_handshake, Network};
use franka::wire::{HeaderLayout, ROBOT_COMMAND_PORT, ROBOT_VERSION};
use franka::{RealtimeConfig, Robot};
use franka_sim_test::{SimConfig, SimServer};

/// Starts a simulator with `config` (and removes it when the returned value is dropped).
pub fn sim(config: SimConfig) -> SimServer {
    SimServer::start(config)
}

/// Opens an FCI session to `sim` and performs the `Connect` handshake with this crate's
/// library version, returning the session and the server's version.
pub fn connect_network(sim: &SimServer) -> FrankaResult<(Network, u16)> {
    connect_network_with_version(sim, ROBOT_VERSION)
}

/// Like [`connect_network`], but announces `library_version` in the `Connect` request.
pub fn connect_network_with_version(
    sim: &SimServer,
    library_version: u16,
) -> FrankaResult<(Network, u16)> {
    let network = Network::connect(sim.host(), ROBOT_COMMAND_PORT, HeaderLayout::Robot)?;
    let version = connect_handshake(&network, library_version)?;
    Ok((network, version))
}

/// Reads one robot state, retrying while the socket times out, up to `timeout`.
///
/// Uses the typed helper, so a datagram whose length is not exactly
/// `size_of::<wire::robot::RobotState>()` fails with libfranka's
/// `Protocol("libfranka: incorrect object size")` instead of being parsed against stale bytes.
///
/// Note that a UDP timeout shuts the command socket down (like libfranka), so this helper reads
/// straight from the [`franka::network::UdpChannel`] while it is still retrying and only
/// reports the last error.
pub fn read_state(
    network: &Network,
    timeout: Duration,
) -> FrankaResult<franka::wire::robot::RobotState> {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        match network.udp.blocking_receive_struct() {
            Ok(state) => return Ok(state),
            Err(e) => {
                let retryable = matches!(&e, FrankaError::Network(m) if m.ends_with("Timeout"));
                if !retryable || std::time::Instant::now() >= deadline {
                    return Err(e);
                }
            }
        }
    }
}

/// libfranka's "ready" pose, used by every `examples/generate_*_motion.cpp`.
pub const READY_POSE: [f64; 7] = [
    0.0,
    -std::f64::consts::FRAC_PI_4,
    0.0,
    -3.0 * std::f64::consts::FRAC_PI_4,
    0.0,
    std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_4,
];

/// The joint configuration franka-sim actually starts in.
///
/// It is *not* libfranka's [`READY_POSE`]: joints 2 and 4 differ by about 0.79 rad, and the
/// values are the MuJoCo model's `home` keyframe rounded to two decimals.
pub const SIM_HOME_POSE: [f64; 7] = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785];

/// Connects a [`Robot`] to `sim`.
///
/// Realtime checks are disabled ([`RealtimeConfig::Ignore`]) because the test machines do not
/// run a `PREEMPT_RT` kernel. Only one FCI client may be connected at a time, so the returned
/// robot must be dropped before the next one is created.
pub fn connect(sim: &SimServer) -> Robot {
    Robot::new(sim.host(), RealtimeConfig::Ignore).expect("failed to connect to franka-sim")
}

/// Applies libfranka's `setDefaultBehavior` (`examples/examples_common.cpp`) to `robot`.
pub fn set_default_behavior(robot: &Robot) {
    robot
        .set_collision_behavior(
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        )
        .expect("set_collision_behavior failed");
    robot
        .set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])
        .expect("set_joint_impedance failed");
    robot
        .set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])
        .expect("set_cartesian_impedance failed");
}

/// How close to [`SIM_HOME_POSE`] [`move_to_start`] must get before a test may rely on it.
pub const HOME_POSE_TOLERANCE: f64 = 0.02;

/// Homes the arm at [`SIM_HOME_POSE`] before a test looks at, or moves from, its pose.
///
/// Every `sim_*` test that moves the robot calls this first. The simulator resets its state
/// when the FCI client disconnects, so a test that gets a fresh container starts at
/// [`SIM_HOME_POSE`] anyway and this returns after one pass -- but against a *persistent*
/// server (`FRANKA_SIM_ADDR`, which is what CI's simulator jobs use) the arm is wherever the
/// previous test left it, and without this the tests would only pass in one hand-picked order.
/// libfranka's own examples do the same thing with the same helper: every
/// `examples/generate_*_motion.cpp` runs a `MotionGenerator` to `READY_POSE` first.
///
/// The move is rate limited, so near a joint limit the client-side limiter clips the commanded
/// ramp and one pass of `MotionGenerator` can end short of the goal; the loop then runs another
/// pass from wherever it ended up.
pub fn move_to_start(robot: &Robot) {
    // A test that tripped a reflex leaves the robot unable to start a motion; clear it first.
    if robot.read_once().expect("read_once failed").robot_mode == franka::RobotMode::Reflex {
        robot
            .automatic_error_recovery()
            .expect("automatic_error_recovery before homing failed");
    }

    for attempt in 0..4 {
        let mut generator =
            examples_common::MotionGenerator::new(robot.fci_version(), 0.2, SIM_HOME_POSE);
        robot
            .control_joint_positions(
                |state: &franka::RobotState, period: franka::Duration| {
                    generator.step(state, period)
                },
                franka::ControllerMode::JointImpedance,
                true,
                franka::DEFAULT_CUTOFF_FREQUENCY,
            )
            .unwrap_or_else(|e| panic!("homing pass {attempt} failed: {e}"));

        let q = settled_state(robot, 5).q;
        let worst = q
            .iter()
            .zip(SIM_HOME_POSE.iter())
            .map(|(actual, expected)| (actual - expected).abs())
            .fold(0.0_f64, f64::max);
        if worst < HOME_POSE_TOLERANCE {
            return;
        }
        assert!(
            attempt < 3,
            "the arm is still {worst} rad from SIM_HOME_POSE after {} homing passes: {q:?}",
            attempt + 1
        );
    }
}

/// Reads `count` states and returns the last one, giving the simulator time to apply a command
/// that only shows up in the state stream a few cycles later.
pub fn settled_state(robot: &Robot, count: usize) -> franka::RobotState {
    let mut state = robot.read_once().expect("read_once failed");
    for _ in 1..count {
        state = robot.read_once().expect("read_once failed");
    }
    state
}

/// The translation part of a column-major 4x4 transform.
pub fn translation(transform: &[f64; 16]) -> [f64; 3] {
    [transform[12], transform[13], transform[14]]
}

/// Column-major 4x4 matrix product `a * b`.
pub fn matrix_multiply(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for column in 0..4 {
        for row in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[k * 4 + row] * b[column * 4 + k];
            }
            out[column * 4 + row] = sum;
        }
    }
    out
}

/// Reads one raw state datagram into `buf`, retrying while the socket times out, up to
/// `timeout`, and returns how many bytes it held.
///
/// The byte-level counterpart of [`read_state`], for the FCI v5 tests: an FER's `RobotState`
/// is 2373 bytes and must not be parsed against the v10 struct, and the length itself is what
/// those tests assert.
pub fn read_state_bytes(
    network: &Network,
    timeout: Duration,
    buf: &mut [u8],
) -> FrankaResult<usize> {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        match network.udp.blocking_receive(buf) {
            Ok(received) => return Ok(received),
            Err(e) => {
                let retryable = matches!(&e, FrankaError::Network(m) if m.ends_with("Timeout"));
                if !retryable || std::time::Instant::now() >= deadline {
                    return Err(e);
                }
            }
        }
    }
}

/// Connects a [`Robot`] to an FER (FCI v5) simulator with an explicit [`VersionPolicy`].
pub fn connect_with_version(
    sim: &SimServer,
    version: franka::VersionPolicy,
) -> FrankaResult<Robot> {
    Robot::with_options(
        sim.host(),
        franka::RobotOptions::new(RealtimeConfig::Ignore).with_version(version),
    )
}

/// The joint configuration the FER (FCI v5) simulator starts in.
///
/// Measured, not assumed. The first state of `franka-sim:panda-v5` started with
/// `--protocol v5 --robot panda` and `PANDA_MJCF=.../franka_emika_panda/panda_nohand.xml`
/// reports
///
/// ```text
/// q = [-0.00089105, -0.00000051, 0.00022151, -1.56999978, -0.00000376, 1.56999964, 0.78500486]
/// ```
///
/// i.e. the Menagerie `home` keyframe, the same one the FR3 model uses, so the FER's home
/// pose rounds to exactly [`SIM_HOME_POSE`] and the FER tests can home to the same target.
/// Every value is well inside the FER's own joint limits (joint 1 +-2.8973, joint 2
/// +-1.7628, joint 4 in [-3.0718, -0.0698], joint 6 in [-0.0175, 3.7525]).
pub const FER_SIM_HOME_POSE: [f64; 7] = SIM_HOME_POSE;

/// Homes an FER (FCI v5) arm at [`FER_SIM_HOME_POSE`] before a test moves it.
///
/// A separate entry point from [`move_to_start`] so that the FCI v5 tests name the home pose
/// they rely on; because the two simulators start from the same keyframe (see
/// [`FER_SIM_HOME_POSE`]) the implementation is shared.
pub fn move_to_start_fer(robot: &Robot) {
    move_to_start(robot);
}
