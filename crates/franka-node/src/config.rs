//! The node's TOML configuration: one `[zenoh]` table and one `[[arm]]` table per arm.

use std::path::{Path, PathBuf};
use std::str::FromStr;

use franka::robot::target_control::LiveTuning;
use franka::robot::target_control::{DEFAULT_LIMIT_FRACTION, JOINT_LIMIT_INSET};
use franka::{
    Backend, IkOptions, ImpedanceGains, ImpedanceOptions, JointTargetControlOptions, Leash,
    OtgLimits, RealtimeConfig, TargetControlOptions,
};
use serde::Deserialize;

use crate::guard::{GuardOptions, Workspace};

/// The whole file; [`FromStr`] parses and validates.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeConfig {
    /// The node's name in `franka/node/<name>/*`.
    #[serde(default = "default_name")]
    pub name: String,
    #[serde(default)]
    pub zenoh: ZenohConfig,
    #[serde(default, rename = "arm")]
    pub arms: Vec<ArmConfig>,
}

fn default_name() -> String {
    "franka-node".into()
}

/// How the node joins the network: one of Zenoh's session modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ZenohMode {
    /// A peer among peers, the default: it listens, and clients and other peers reach it
    /// directly. What a lab segment wants.
    #[default]
    Peer,
    /// A client of a router it dials: nothing listens here and nothing needs an inbound rule,
    /// which is how a node behind NAT reaches a commander that is not on its network. `connect`
    /// must then name the router, `listen` is ignored, and an endpoint off the local segment
    /// should be `tls/` or `quic/` rather than `tcp/`.
    Client,
}

impl ZenohMode {
    /// The string Zenoh's own config takes.
    pub const fn as_str(self) -> &'static str {
        match self {
            ZenohMode::Peer => "peer",
            ZenohMode::Client => "client",
        }
    }
}

/// The `[zenoh]` table.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ZenohConfig {
    /// `peer`, the default, or `client` to dial a router instead of listening.
    pub mode: ZenohMode,
    /// Endpoints to listen on. Default `tcp/0.0.0.0:7447`; ignored by a client.
    pub listen: Vec<String>,
    /// Endpoints to connect to. Default none.
    pub connect: Vec<String>,
    /// Multicast scouting interface; `None` is `auto`.
    pub scouting_interface: Option<String>,
    /// Whether peers are found by UDP multicast. `None`, the default, means on unless the node
    /// only connects (`connect` set, `listen` empty), where the peer is already named.
    pub multicast_scouting: Option<bool>,
    /// `transport.link.tx.lease`, ms; the keepalive is a quarter of it. Default 1000.
    pub lease_ms: u64,
    /// A Zenoh configuration file (JSON5) to start from, for everything this table does not
    /// name: the TLS material of a `tls/` endpoint, authentication, access control. The keys
    /// above are applied on top of it, so `mode`, `listen`, `connect`, the scouting switch and
    /// the lease are this table's whatever the file says.
    pub zenoh_config: Option<PathBuf>,
}

impl Default for ZenohConfig {
    fn default() -> Self {
        ZenohConfig {
            mode: ZenohMode::Peer,
            listen: vec!["tcp/0.0.0.0:7447".into()],
            connect: Vec::new(),
            scouting_interface: None,
            multicast_scouting: None,
            lease_ms: 1000,
            zenoh_config: None,
        }
    }
}

impl ZenohConfig {
    /// The resolved multicast scouting switch; see [`ZenohConfig::multicast_scouting`].
    pub fn multicast_enabled(&self) -> bool {
        self.multicast_scouting
            .unwrap_or(!self.listen_endpoints().is_empty() || self.connect.is_empty())
    }

    /// What the session actually listens on: nothing at all for a client.
    pub fn listen_endpoints(&self) -> &[String] {
        match self.mode {
            ZenohMode::Peer => &self.listen,
            ZenohMode::Client => &[],
        }
    }
}

/// `realtime` of an arm, [`RealtimeConfig`] spelt in TOML.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Realtime {
    #[default]
    Enforce,
    Ignore,
}

/// `leash` of an arm: how far the desired pose may run ahead of the measured one, m and rad;
/// [`Leash`] of the impedance backend. Defaults 0.025 and 0.15.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct LeashConfig {
    pub translation: f64,
    pub rotation: f64,
}

impl Default for LeashConfig {
    fn default() -> Self {
        let leash = Leash::default();
        LeashConfig {
            translation: leash.translation,
            rotation: leash.rotation,
        }
    }
}

/// One `[[arm]]` table; `name` and `host` are required, the rest has defaults.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArmConfig {
    /// The `<arm>` in `franka/<arm>/*`, `[A-Za-z0-9_-]+`.
    pub name: String,
    /// The robot's hostname or address.
    pub host: String,
    #[serde(default)]
    pub realtime: Realtime,
    /// State publishing rate, 1..=1000. Default 100.
    #[serde(default = "default_state_hz")]
    pub state_hz: u32,
    /// Silence in Active before `holding` is flagged. Default 200.
    #[serde(default = "default_hold_after_ms")]
    pub hold_after_ms: u64,
    /// Silence in Active before the loop is stopped. Default 2000.
    #[serde(default = "default_stop_after_ms")]
    pub stop_after_ms: u64,
    /// Collision force threshold, N. Default 40.
    #[serde(default = "default_collision")]
    pub collision_force: f64,
    /// Collision torque threshold, Nm. Default 40.
    #[serde(default = "default_collision")]
    pub collision_torque: f64,
    /// Translation budget as a norm: m/s, m/s^2, m/s^3. Default 0.3, 0.5, 20.
    #[serde(default = "default_budget")]
    pub budget: [f64; 3],
    /// Rotation budget as a norm: rad/s, rad/s^2, rad/s^3. Default 0.5, 1.0, 20.
    #[serde(default = "default_rotation_budget")]
    pub rotation_budget: [f64; 3],
    /// Translational stiffness, N/m; rotational is a fiftieth. Default 750.
    #[serde(default = "default_stiffness")]
    pub cartesian_stiffness: f64,
    /// [`TargetControlOptions::max_deviation`], m. Default 0.3.
    #[serde(default = "default_max_deviation")]
    pub max_deviation: f64,
    /// [`TargetControlOptions::max_angular_deviation`], rad. Default 0.5.
    #[serde(default = "default_max_angular_deviation")]
    pub max_angular_deviation: f64,
    #[serde(default)]
    pub leash: LeashConfig,
    /// [`GuardOptions::max_step`].
    #[serde(default = "default_max_step")]
    pub max_step: f64,
    /// [`GuardOptions::max_step_rotation`].
    #[serde(default = "default_max_step_rotation")]
    pub max_step_rotation: f64,
    /// [`GuardOptions::max_step_joint`]. Default 0.2.
    #[serde(default = "default_max_step_joint")]
    pub max_step_joint: f64,
    /// [`GuardOptions::max_lead`], m; 0 disables the check. Default 0.05.
    #[serde(default = "default_max_lead")]
    pub max_lead: f64,
    /// [`GuardOptions::max_lead_rotation`], rad; 0 disables the check. Default 0.26.
    #[serde(default = "default_max_lead_rotation")]
    pub max_lead_rotation: f64,
    /// The fraction of the arm's joint limits a joints session budgets, in (0, 1]. Default
    /// [`DEFAULT_LIMIT_FRACTION`].
    #[serde(default = "default_joint_budget_fraction")]
    pub joint_budget_fraction: f64,
    /// [`JointTargetControlOptions::max_deviation`], rad, of a joints session (`home` widens
    /// it to its travel). Default 1.0.
    #[serde(default = "default_joint_max_deviation")]
    pub joint_max_deviation: f64,
    /// [`ImpedanceOptions::joint_velocity_fraction`] of both session kinds: the fraction of the
    /// arm's joint velocity limits the goal may move at, (0, 1]. Default 0.7.
    #[serde(default = "default_joint_velocity_fraction")]
    pub joint_velocity_fraction: f64,
    /// [`ImpedanceOptions::velocity_barrier_fraction`] of both session kinds, in
    /// [`joint_velocity_fraction`, 1]. Default 0.85.
    #[serde(default = "default_velocity_barrier_fraction")]
    pub velocity_barrier_fraction: f64,
    /// [`ImpedanceOptions::joint_position_margin`] of both session kinds, rad, in [0.035, 0.5]:
    /// the distance the joint goal keeps from the joint position limits; the joint gate refuses
    /// targets inside it too. Default 0.05.
    #[serde(default = "default_joint_position_margin")]
    pub joint_position_margin: f64,
    /// [`IkOptions::damping`], the `λ` of the Cartesian backend's damped least squares, of both
    /// session kinds. The damped pseudo-inverse's gain is bounded by `1 / 2λ`, so near a singular
    /// pose a smaller `λ` buys tracking accuracy with a larger amplification of the pose error
    /// into joint motion. Default 0.05.
    #[serde(default = "default_ik_damping")]
    pub ik_damping: f64,
    /// [`IkOptions::nullspace_gain`], 1/s, of both session kinds: the pull of the redundant
    /// elbow toward the posture, which is the arm's configuration when the session started. The
    /// further a session teleoperates from that snapshot, the more elbow motion the bias adds
    /// that the commander never asked for; 0 switches it off. Default 1.0.
    #[serde(default = "default_ik_nullspace_gain")]
    pub ik_nullspace_gain: f64,
    /// [`ImpedanceOptions::velocity_feedforward`] of both session kinds: with it off, `dq_goal`
    /// is zero and the joint damping acts on the measured velocity alone, dissipatively. Default
    /// true.
    #[serde(default = "default_velocity_feedforward")]
    pub velocity_feedforward: bool,
    /// [`ImpedanceOptions::velocity_feedforward_gain`] of both session kinds, in [0, 1]: the
    /// weight of the goal velocity in the damping term. 0 is `velocity_feedforward = false`.
    /// Default 1.
    #[serde(default = "default_velocity_feedforward_gain")]
    pub velocity_feedforward_gain: f64,
    /// [`ImpedanceOptions::velocity_feedforward_cutoff`], Hz, of a Cartesian session; `1000`
    /// switches the filter off, and it is floored at
    /// [`MIN_FEEDFORWARD_CUTOFF`](franka::robot::target_control::MIN_FEEDFORWARD_CUTOFF).
    /// Bounding it keeps the feedforward's lead while leaving the joint reference's ripple
    /// behind. A joints session forms its goal velocity directly and is unaffected.
    #[serde(default = "default_velocity_feedforward_cutoff")]
    pub velocity_feedforward_cutoff: f64,
    /// [`ImpedanceOptions::cutoff_frequency`], Hz, of the crate's first-order low-pass on the
    /// commanded torque. `1000` ([`MAX_CUTOFF_FREQUENCY`](franka::lowpass_filter::MAX_CUTOFF_FREQUENCY))
    /// switches it off. Default 100.
    #[serde(default = "default_cutoff_frequency")]
    pub cutoff_frequency: f64,
    /// [`ImpedanceGains::joint_stiffness`], Nm/rad, of both session kinds; `None` keeps the
    /// preset's. A Cartesian session's preset is soft on purpose — the Cartesian spring does the
    /// work and the joint terms only steady the posture — so an arm driven mostly by its joint
    /// law (no `velocity_feedforward`, say) wants these raised toward the joint preset's
    /// `[600, 600, 600, 600, 250, 150, 50]`.
    #[serde(default)]
    pub joint_stiffness: Option<[f64; 7]>,
    /// [`ImpedanceGains::joint_damping`], Nm·s/rad, of both session kinds; `None` keeps the
    /// preset's. Raise with the square root of the stiffness to hold the damping ratio.
    #[serde(default)]
    pub joint_damping: Option<[f64; 7]>,
    /// The box a target must lie in, m. Absent, the default, is no box: see
    /// [`GuardOptions::workspace`].
    #[serde(default)]
    pub workspace: Option<Workspace>,
    /// [`GuardOptions::rate_hz`]. Default 250.
    #[serde(default = "default_rate_hz")]
    pub rate_hz: f64,
    /// `SCHED_FIFO` priority of the loop; `None` is the highest.
    #[serde(default)]
    pub realtime_priority: Option<i32>,
    /// The CPU the loop thread is pinned to ([`TargetControlOptions::cpu`]); `None` leaves it
    /// to the scheduler.
    #[serde(default)]
    pub cpu: Option<usize>,
    /// Where every session is recorded as one `.rrd` (the `record` feature); `None` records
    /// nothing.
    #[serde(default)]
    pub record_dir: Option<PathBuf>,
    /// A directory of link meshes the recording draws the arm with instead of the built-in ones
    /// (`franka_rerun::Meshes`); `"none"` draws the skeleton alone.
    #[serde(default)]
    pub record_meshes: Option<PathBuf>,
    /// The arm's gripper driver, `"hand"` for the Franka Hand at `host`; the binary's factory
    /// decides which names it knows. `None` is no gripper.
    #[serde(default)]
    pub gripper: Option<String>,
    /// The speed of every gripper move and grasp, m/s. Default 0.1.
    #[serde(default = "default_gripper_speed")]
    pub gripper_speed: f64,
}

fn default_gripper_speed() -> f64 {
    0.1
}

fn default_state_hz() -> u32 {
    100
}
fn default_hold_after_ms() -> u64 {
    200
}
fn default_stop_after_ms() -> u64 {
    2000
}
fn default_collision() -> f64 {
    40.0
}
fn default_budget() -> [f64; 3] {
    [0.3, 0.5, 20.0]
}
fn default_rotation_budget() -> [f64; 3] {
    [0.5, 1.0, 20.0]
}
fn default_stiffness() -> f64 {
    ImpedanceGains::CARTESIAN.cartesian_stiffness[0]
}
fn default_max_deviation() -> f64 {
    TargetControlOptions::default().max_deviation
}
fn default_max_angular_deviation() -> f64 {
    TargetControlOptions::default().max_angular_deviation
}
fn default_max_step() -> f64 {
    GuardOptions::default().max_step
}
fn default_max_step_rotation() -> f64 {
    GuardOptions::default().max_step_rotation
}
fn default_max_step_joint() -> f64 {
    GuardOptions::default().max_step_joint
}
fn default_max_lead() -> f64 {
    GuardOptions::default().max_lead
}
fn default_max_lead_rotation() -> f64 {
    GuardOptions::default().max_lead_rotation
}
fn default_joint_budget_fraction() -> f64 {
    DEFAULT_LIMIT_FRACTION
}
fn default_joint_max_deviation() -> f64 {
    JointTargetControlOptions::default().max_deviation
}
fn default_joint_velocity_fraction() -> f64 {
    ImpedanceOptions::cartesian().joint_velocity_fraction
}
fn default_velocity_barrier_fraction() -> f64 {
    ImpedanceOptions::cartesian().velocity_barrier_fraction
}
fn default_joint_position_margin() -> f64 {
    ImpedanceOptions::cartesian().joint_position_margin
}
fn default_ik_damping() -> f64 {
    IkOptions::default().damping
}
fn default_ik_nullspace_gain() -> f64 {
    IkOptions::default().nullspace_gain
}
fn default_velocity_feedforward() -> bool {
    ImpedanceOptions::cartesian().velocity_feedforward
}
fn default_cutoff_frequency() -> f64 {
    ImpedanceOptions::cartesian().cutoff_frequency
}
fn default_velocity_feedforward_gain() -> f64 {
    ImpedanceOptions::cartesian().velocity_feedforward_gain
}
fn default_velocity_feedforward_cutoff() -> f64 {
    ImpedanceOptions::cartesian().velocity_feedforward_cutoff
}
fn default_rate_hz() -> f64 {
    GuardOptions::default().rate_hz
}

/// Why a file is not a [`NodeConfig`].
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("config: {0}")]
    Io(#[from] std::io::Error),
    #[error("config: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("config: {0}")]
    Invalid(String),
}

impl FromStr for NodeConfig {
    type Err = ConfigError;

    fn from_str(text: &str) -> Result<Self, ConfigError> {
        let config: NodeConfig = toml::from_str(text)?;
        config.validate()?;
        Ok(config)
    }
}

impl NodeConfig {
    /// Reads, parses and validates `path`.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        std::fs::read_to_string(path)?.parse()
    }

    /// Checks what the types cannot: the lease, unique key-safe arm names and each arm's
    /// numbers.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.zenoh.lease_ms == 0 {
            return Err(ConfigError::Invalid(
                "zenoh: lease_ms must be positive".into(),
            ));
        }
        if self.zenoh.mode == ZenohMode::Client && self.zenoh.connect.is_empty() {
            return Err(ConfigError::Invalid(
                "zenoh: a client must connect to a router, so connect must name one".into(),
            ));
        }
        for (i, arm) in self.arms.iter().enumerate() {
            arm.validate()?;
            if self.arms[..i].iter().any(|other| other.name == arm.name) {
                return Err(ConfigError::Invalid(format!(
                    "arm name {:?} is not unique",
                    arm.name
                )));
            }
        }
        Ok(())
    }
}

fn key_safe(name: &str) -> bool {
    !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// Names an arm cannot take, because something else already owns that branch of the key
/// space: the node's own status (`franka/node/<name>/status`) and the camera node's keys
/// (`franka/cam/<name>/frame`).
pub const RESERVED_ARM_NAMES: [&str; 2] = ["node", "cam"];

impl ArmConfig {
    fn validate(&self) -> Result<(), ConfigError> {
        let arm = &self.name;
        let invalid = |why: String| ConfigError::Invalid(format!("arm {arm}: {why}"));
        if !key_safe(arm) {
            return Err(ConfigError::Invalid(format!(
                "arm name {arm:?} is not [A-Za-z0-9_-]+"
            )));
        }
        if RESERVED_ARM_NAMES.contains(&arm.as_str()) {
            return Err(ConfigError::Invalid(format!(
                "arm name {arm:?} is reserved: franka/{arm}/* is not an arm's"
            )));
        }
        if self.host.is_empty() {
            return Err(invalid("host must not be empty".into()));
        }
        if !(1..=1000).contains(&self.state_hz) {
            return Err(invalid("state_hz must be in 1..=1000".into()));
        }
        if self.stop_after_ms <= self.hold_after_ms {
            return Err(invalid("stop_after_ms must exceed hold_after_ms".into()));
        }
        let positive = |x: f64| x.is_finite() && x > 0.0;
        let scalars = [
            ("collision_force", self.collision_force),
            ("collision_torque", self.collision_torque),
            ("cartesian_stiffness", self.cartesian_stiffness),
            ("max_deviation", self.max_deviation),
            ("max_angular_deviation", self.max_angular_deviation),
            ("leash.translation", self.leash.translation),
            ("leash.rotation", self.leash.rotation),
            ("max_step", self.max_step),
            ("max_step_rotation", self.max_step_rotation),
            ("max_step_joint", self.max_step_joint),
            ("joint_max_deviation", self.joint_max_deviation),
            ("rate_hz", self.rate_hz),
            ("gripper_speed", self.gripper_speed),
            ("ik_damping", self.ik_damping),
            ("cutoff_frequency", self.cutoff_frequency),
            (
                "velocity_feedforward_cutoff",
                self.velocity_feedforward_cutoff,
            ),
        ];
        for (field, value) in scalars {
            if !positive(value) {
                return Err(invalid(format!("{field} must be positive")));
            }
        }
        for (field, gains) in [
            ("joint_stiffness", self.joint_stiffness),
            ("joint_damping", self.joint_damping),
        ] {
            if let Some(gains) = gains {
                if gains.iter().any(|g| !g.is_finite() || *g < 0.0) {
                    return Err(invalid(format!(
                        "{field} must be finite and non-negative, got {gains:?}"
                    )));
                }
            }
        }
        if !self.velocity_feedforward_gain.is_finite()
            || !(0.0..=1.0).contains(&self.velocity_feedforward_gain)
        {
            return Err(invalid(
                "velocity_feedforward_gain must be within [0, 1]".into(),
            ));
        }
        if self.velocity_feedforward_cutoff < franka::robot::target_control::MIN_FEEDFORWARD_CUTOFF
        {
            return Err(invalid(format!(
                "velocity_feedforward_cutoff must be at least {} Hz",
                franka::robot::target_control::MIN_FEEDFORWARD_CUTOFF
            )));
        }
        // 0 switches the posture bias off, so this one is non-negative rather than positive.
        if !self.ik_nullspace_gain.is_finite() || self.ik_nullspace_gain < 0.0 {
            return Err(invalid(
                "ik_nullspace_gain must be finite and non-negative".into(),
            ));
        }
        // The lead limits take 0, which disables the check, and must otherwise stay above the
        // backend's own leash: at or below it the gate would refuse a healthy commander's
        // ordinary tracking error, which the watchdog then reads as a commander in trouble.
        for (field, value, leash, leash_field) in [
            (
                "max_lead",
                self.max_lead,
                self.leash.translation,
                "leash.translation",
            ),
            (
                "max_lead_rotation",
                self.max_lead_rotation,
                self.leash.rotation,
                "leash.rotation",
            ),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(invalid(format!("{field} must be zero or positive")));
            }
            if value > 0.0 && value <= leash {
                return Err(invalid(format!(
                    "{field} {value} must exceed {leash_field} {leash}, or be 0 to disable"
                )));
            }
        }
        if !(positive(self.joint_budget_fraction) && self.joint_budget_fraction <= 1.0) {
            return Err(invalid("joint_budget_fraction must be in (0, 1]".into()));
        }
        for (field, budget) in [
            ("budget", self.budget),
            ("rotation_budget", self.rotation_budget),
        ] {
            if !budget.iter().all(|x| positive(*x)) {
                return Err(invalid(format!("{field} must be positive")));
            }
        }
        if let Some(Workspace { min, max }) = self.workspace {
            if !(0..3).all(|i| min[i].is_finite() && max[i].is_finite() && min[i] < max[i]) {
                return Err(invalid("workspace min must be below max".into()));
            }
        }
        #[cfg(not(feature = "record"))]
        for (key, set) in [
            ("record_dir", self.record_dir.is_some()),
            ("record_meshes", self.record_meshes.is_some()),
        ] {
            if set {
                return Err(invalid(format!("{key}: built without the record feature")));
            }
        }
        if self.record_dir.is_none() && self.record_meshes.is_some() {
            return Err(invalid("record_meshes needs record_dir".into()));
        }
        if self.gripper.as_deref() == Some("") {
            return Err(invalid("gripper must not be empty".into()));
        }
        // The library's own check, for what the above does not name (the priority range, the
        // cpu bound).
        self.target_control_options()
            .validate()
            .map_err(|e| invalid(e.to_string()))
    }

    /// The `RealtimeConfig` for `Robot::new`.
    pub fn realtime_config(&self) -> RealtimeConfig {
        match self.realtime {
            Realtime::Enforce => RealtimeConfig::Enforce,
            Realtime::Ignore => RealtimeConfig::Ignore,
        }
    }

    /// The guard's limits.
    pub fn guard_options(&self) -> GuardOptions {
        GuardOptions {
            max_step: self.max_step,
            max_step_rotation: self.max_step_rotation,
            max_step_joint: self.max_step_joint,
            max_lead: self.max_lead,
            max_lead_rotation: self.max_lead_rotation,
            workspace: self.workspace,
            rate_hz: self.rate_hz,
            joint_limit_inset: JOINT_LIMIT_INSET.max(self.joint_position_margin),
        }
    }

    /// The options of `start_cartesian_target_control`: the budgets, the impedance backend
    /// at `cartesian_stiffness` with the leash and the joint envelopes, the
    /// deviation guards, the priority, the cpu.
    pub fn target_control_options(&self) -> TargetControlOptions {
        let impedance = self.cartesian_impedance();
        TargetControlOptions::default()
            .with_limits(limits(self.budget))
            .with_rotation_limits(limits(self.rotation_budget))
            .with_backend(Backend::Impedance(impedance))
            .with_max_deviation(self.max_deviation)
            .with_max_angular_deviation(self.max_angular_deviation)
            .with_realtime_priority(self.realtime_priority)
            .with_cpu(self.cpu)
    }

    /// The options of `start_joint_target_control` under `limits`: the library's impedance
    /// backend with the joint envelopes, and settle, `joint_max_deviation`, the
    /// priority, the cpu.
    pub fn joint_control_options(&self, limits: [OtgLimits; 7]) -> JointTargetControlOptions {
        let impedance = self.envelopes(ImpedanceOptions::joint());
        JointTargetControlOptions::default()
            .with_limits(limits)
            .with_backend(Backend::Impedance(impedance))
            .with_max_deviation(self.joint_max_deviation)
            .with_realtime_priority(self.realtime_priority)
            .with_cpu(self.cpu)
    }

    /// The impedance backend of a Cartesian session: the preset scaled to `cartesian_stiffness`,
    /// the leash, and the envelopes every session shares.
    fn cartesian_impedance(&self) -> ImpedanceOptions {
        let leash = Leash {
            translation: self.leash.translation,
            rotation: self.leash.rotation,
            ..Leash::default()
        };
        self.envelopes(
            ImpedanceOptions::cartesian()
                .with_gains(cartesian_gains(self.cartesian_stiffness))
                .with_leash(leash),
        )
    }

    /// What a Cartesian session of this config starts tunable at: the same reading of the same
    /// options the session itself seeds its slot from, so the node can answer `params/get`
    /// before any session runs and say what a new one would start with.
    ///
    /// Deriving it here rather than storing a second copy is the point: move a key in the TOML
    /// and this moves with it.
    pub fn live_tuning(&self) -> LiveTuning {
        LiveTuning::from_options(
            &self.cartesian_impedance(),
            limits(self.budget),
            limits(self.rotation_budget),
        )
    }

    /// `impedance` with the arm's joint velocity cap, barrier, position margin and IK damping.
    fn envelopes(&self, impedance: ImpedanceOptions) -> ImpedanceOptions {
        let ik = IkOptions {
            damping: self.ik_damping,
            nullspace_gain: self.ik_nullspace_gain,
            ..impedance.ik
        };
        let gains = ImpedanceGains {
            joint_stiffness: self
                .joint_stiffness
                .unwrap_or(impedance.gains.joint_stiffness),
            joint_damping: self.joint_damping.unwrap_or(impedance.gains.joint_damping),
            ..impedance.gains
        };
        impedance
            .with_gains(gains)
            .with_joint_velocity_fraction(self.joint_velocity_fraction)
            .with_velocity_barrier_fraction(self.velocity_barrier_fraction)
            .with_joint_position_margin(self.joint_position_margin)
            .with_velocity_feedforward(self.velocity_feedforward)
            .with_velocity_feedforward_gain(self.velocity_feedforward_gain)
            .with_velocity_feedforward_cutoff(self.velocity_feedforward_cutoff)
            .with_cutoff_frequency(self.cutoff_frequency)
            .with_ik(ik)
    }
}

fn limits([max_velocity, max_acceleration, max_jerk]: [f64; 3]) -> OtgLimits {
    OtgLimits {
        max_velocity,
        max_acceleration,
        max_jerk,
    }
}

/// [`ImpedanceGains::CARTESIAN`] rescaled to `stiffness` N/m. The rule lives in the library,
/// with the preset it scales, because the live retune path needs it too; this is the config's
/// name for it and must stay a delegation.
pub fn cartesian_gains(stiffness: f64) -> ImpedanceGains {
    ImpedanceGains::scaled_cartesian(stiffness)
}

#[cfg(test)]
mod tests;
