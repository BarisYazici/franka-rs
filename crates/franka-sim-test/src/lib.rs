//! Test harness that boots franka-sim in Docker. See
//! `docs/book/src/howto/simulator-tests.md`.
//!
//! [`SimServer::start`] launches a `franka-sim` (libfranka-sim) container on
//! Docker's host network, waits for it to serve the Franka Control Interface
//! (FCI), and removes the container when the returned [`SimServer`] is
//! dropped. This lets integration tests connect a real client to a real
//! simulated robot without any manual setup.
//!
//! Only one simulator may run at a time on a given host, because the FCI
//! ports (1337 robot, 1338 gripper) are fixed by the protocol. A process-wide
//! lock serialises tests within one test binary; running multiple test
//! binaries (or processes) concurrently against the same host is not
//! supported and [`SimServer::start`] will refuse to start a second
//! container.
//!
//! # Example
//!
//! ```rust,no_run
//! use franka_sim_test::{SimConfig, SimServer};
//!
//! let sim = SimServer::start(SimConfig::nominal());
//! // connect a client to sim.host() on port 1337 (robot) / 1338 (gripper) ...
//! drop(sim); // container is removed here
//! ```

mod docker;
mod server;

pub use server::SimServer;

use std::env;
use std::sync::atomic::AtomicU64;
use std::sync::Mutex;
use std::time::Duration;

/// Serialises access to the simulator across tests in one test binary: only
/// one franka-sim container may run at a time (the FCI ports are fixed).
static LOCK: Mutex<()> = Mutex::new(());

/// Disambiguates container names across [`SimServer`]s started by this
/// process (tests are serialised by [`LOCK`], but names must still not
/// collide with a just-removed container while Docker finishes tearing it
/// down).
static COUNTER: AtomicU64 = AtomicU64::new(0);

/// Default FR3 / FCI v10 image, used when `FRANKA_SIM_IMAGE` is not set.
const DEFAULT_IMAGE: &str = "ghcr.io/barisyazici/franka-sim:latest";

/// Default FER / FCI v5 image, used by [`SimConfig::fer_v5`] when
/// `FRANKA_SIM_FER_IMAGE` is not set.
///
/// There is no published FER image; this is a local build (its recipe is kept
/// privately, outside this repository), so a machine without it fails the v5
/// simulator tests loudly rather than skipping them.
pub const DEFAULT_FER_IMAGE: &str = "franka-sim:panda-v5";

/// Host the simulator's FCI ports are bound on (Docker's host network).
const ROBOT_HOST: &str = "127.0.0.1";
/// Robot FCI TCP port (fixed by the libfranka protocol).
const ROBOT_PORT: u16 = 1337;
/// Gripper FCI TCP port (fixed by the libfranka protocol).
const GRIPPER_PORT: u16 = 1338;

/// How long [`SimServer::start`] waits for a freshly started container to
/// report `Running` before giving up and attempting the readiness probe
/// anyway.
const CONTAINER_START_TIMEOUT: Duration = Duration::from_secs(30);

/// How long the harness's own readiness probe (used when `franka-sim-check`
/// is unavailable in the image, notably `franka-sim:panda-v5`) retries
/// connecting to the robot TCP port before giving up.
const PROBE_CONNECT_TIMEOUT: Duration = Duration::from_secs(60);

/// How long the harness's own readiness probe waits for the first UDP
/// robot-state datagram after a successful Connect handshake.
const PROBE_UDP_TIMEOUT: Duration = Duration::from_secs(5);

/// Expected size, in bytes, of a v10 `RobotState` UDP datagram (see
/// `crates/franka-rs/src/wire/robot/v10/rbk_types.rs`).
const V10_ROBOT_STATE_LEN: usize = 1377;

/// Expected size, in bytes, of a v5 `RobotState` UDP datagram (see
/// `context-shared.md`, "FCI v5 facts").
const V5_ROBOT_STATE_LEN: usize = 2373;

/// FCI protocol version the simulator speaks. Selects `--protocol v5` on the
/// server command line, and which wire version and `RobotState` size
/// `readiness_probe` expects when it is used as [`SimServer::start`]'s
/// readiness check (see `wait_ready`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Protocol {
    /// FCI v10 (FR3), milestone 1's target. Readiness is normally checked
    /// with the image's `franka-sim-check` binary; `readiness_probe` is
    /// used only as a fallback when that binary is missing from the image.
    #[default]
    V10,
    /// FCI v5 (FER). Readiness is always checked with `readiness_probe`,
    /// because the `franka-sim:panda-v5` image has no `franka-sim-check`
    /// binary.
    V5,
}

impl Protocol {
    /// The `u16 version` field value this protocol sends in its Connect
    /// request and expects in the Connect reply.
    fn wire_version(self) -> u16 {
        match self {
            Protocol::V10 => 10,
            Protocol::V5 => 5,
        }
    }

    /// The expected size, in bytes, of a `RobotState` UDP datagram on this
    /// protocol.
    fn expected_robot_state_len(self) -> usize {
        match self {
            Protocol::V10 => V10_ROBOT_STATE_LEN,
            Protocol::V5 => V5_ROBOT_STATE_LEN,
        }
    }
}

/// Robot model the simulator emulates. Selects `--robot panda` on the server
/// command line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RobotKind {
    /// Franka Research 3, milestone 1's target.
    #[default]
    Fr3,
    /// Franka Emika Robot (FER).
    Fer,
}

/// Configuration for a franka-sim server started by [`SimServer::start`].
///
/// Build one with [`SimConfig::nominal`] and the `with_*` builder methods,
/// e.g. `SimConfig::nominal().with_motion_limits()`, or use
/// [`SimConfig::fer_v5`] for the FER / FCI v5 simulator.
#[derive(Debug, Clone)]
pub struct SimConfig {
    /// Passes `--enforce-motion-limits` to the server.
    pub enforce_motion_limits: bool,
    /// Passes `--enforce-comm-constraints` to the server.
    pub enforce_comm_constraints: bool,
    /// Whether the gripper server (port 1338) is enabled. When `false`,
    /// `--no-gripper` is passed.
    pub gripper: bool,
    /// When set, passes `--gripper-object-width <w>` so a graspable object
    /// is placed in the gripper's jaws at that width.
    pub gripper_object_width: Option<f64>,
    /// Additional raw arguments appended after the flags derived from the
    /// other fields.
    pub extra_args: Vec<String>,
    /// Docker image to run. When `None`, [`SimServer::start`] falls back to
    /// the `FRANKA_SIM_IMAGE` environment variable, then `DEFAULT_IMAGE` —
    /// both of which name the **FR3 / FCI v10** image.
    /// [`SimConfig::fer_v5`] always fills this in, from
    /// `FRANKA_SIM_FER_IMAGE` or [`DEFAULT_FER_IMAGE`], so the FER tests
    /// never inherit the FR3 image.
    pub image: Option<String>,
    /// FCI protocol version the server speaks. Defaults to
    /// [`Protocol::V10`].
    pub protocol: Protocol,
    /// Robot model the server emulates. Defaults to [`RobotKind::Fr3`].
    pub robot: RobotKind,
    /// Extra `KEY=VALUE` pairs passed to `docker run` as `-e KEY=VALUE`.
    pub env: Vec<(String, String)>,
}

/// The same configuration as [`SimConfig::nominal`], so that
/// `SimConfig { enforce_motion_limits: true, ..Default::default() }` behaves
/// like the builder methods rather than silently disabling the gripper.
///
/// ```
/// # use franka_sim_test::SimConfig;
/// let config = SimConfig::default();
/// assert!(config.gripper);
/// assert!(!config.enforce_motion_limits);
/// ```
impl Default for SimConfig {
    fn default() -> Self {
        SimConfig {
            enforce_motion_limits: false,
            enforce_comm_constraints: false,
            gripper: true,
            gripper_object_width: None,
            extra_args: Vec::new(),
            image: None,
            protocol: Protocol::V10,
            robot: RobotKind::Fr3,
            env: Vec::new(),
        }
    }
}

impl SimConfig {
    /// The standard test configuration: motion limits and communication
    /// constraints disabled (permissive, deterministic for fast unit tests),
    /// gripper enabled. Identical to [`SimConfig::default`].
    ///
    /// ```
    /// # use franka_sim_test::SimConfig;
    /// let config = SimConfig::nominal();
    /// assert!(config.gripper);
    /// assert!(!config.enforce_motion_limits);
    /// ```
    pub fn nominal() -> Self {
        SimConfig::default()
    }

    /// Enables `--enforce-motion-limits`.
    pub fn with_motion_limits(mut self) -> Self {
        self.enforce_motion_limits = true;
        self
    }

    /// Enables `--enforce-comm-constraints`.
    pub fn with_comm_constraints(mut self) -> Self {
        self.enforce_comm_constraints = true;
        self
    }

    /// Places a graspable object of the given width (metres) in the
    /// gripper's jaws.
    pub fn with_gripper_object(mut self, width: f64) -> Self {
        self.gripper_object_width = Some(width);
        self
    }

    /// Configuration for the FER / FCI v5 simulator: `--protocol v5 --robot
    /// panda`, with the MuJoCo model selected via the `PANDA_MJCF` container
    /// environment variable. Gripper enabled (the verified invocation of this
    /// image passes no gripper flags).
    ///
    /// The image is `$FRANKA_SIM_FER_IMAGE`, defaulting to
    /// [`DEFAULT_FER_IMAGE`]. `FRANKA_SIM_IMAGE` is **not** consulted: that
    /// one names the FR3 image, the two are different builds, and every v5
    /// test would then talk the v5 protocol to a v10 server. Setting `image`
    /// on the returned value still overrides both.
    ///
    /// ```
    /// # use franka_sim_test::{Protocol, RobotKind, SimConfig, DEFAULT_FER_IMAGE};
    /// let expected = std::env::var("FRANKA_SIM_FER_IMAGE")
    ///     .unwrap_or_else(|_| DEFAULT_FER_IMAGE.to_string());
    /// let config = SimConfig::fer_v5();
    /// assert_eq!(config.image.as_deref(), Some(expected.as_str()));
    /// assert_eq!(config.protocol, Protocol::V5);
    /// assert_eq!(config.robot, RobotKind::Fer);
    /// ```
    pub fn fer_v5() -> Self {
        SimConfig {
            gripper: true,
            image: Some(
                env::var("FRANKA_SIM_FER_IMAGE").unwrap_or_else(|_| DEFAULT_FER_IMAGE.to_string()),
            ),
            protocol: Protocol::V5,
            robot: RobotKind::Fer,
            env: vec![(
                "PANDA_MJCF".to_string(),
                "/opt/mujoco_menagerie/franka_emika_panda/panda_nohand.xml".to_string(),
            )],
            ..Default::default()
        }
    }

    /// Renders this configuration as the argument list passed to the
    /// franka-sim server binary.
    fn args(&self) -> Vec<String> {
        let mut args = Vec::new();
        if self.enforce_motion_limits {
            args.push("--enforce-motion-limits".to_string());
        }
        if self.enforce_comm_constraints {
            args.push("--enforce-comm-constraints".to_string());
        }
        if !self.gripper {
            args.push("--no-gripper".to_string());
        }
        if let Some(width) = self.gripper_object_width {
            args.push("--gripper-object-width".to_string());
            args.push(width.to_string());
        }
        if self.protocol == Protocol::V5 {
            args.push("--protocol".to_string());
            args.push("v5".to_string());
        }
        if self.robot == RobotKind::Fer {
            args.push("--robot".to_string());
            args.push("panda".to_string());
        }
        args.extend(self.extra_args.iter().cloned());
        args
    }
}

#[cfg(test)]
mod tests;
