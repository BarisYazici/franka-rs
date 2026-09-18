//! The camera node's TOML configuration: one `[zenoh]` table and one `[[camera]]` table per
//! camera.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::str::FromStr;

use serde::Deserialize;

use crate::msg::Format;

/// The whole file; [`FromStr`] parses and validates.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CamConfig {
    /// The node's name in `franka/node/<name>/status`, where the arm node publishes its own.
    #[serde(default = "default_name")]
    pub name: String,
    #[serde(default)]
    pub zenoh: ZenohConfig,
    /// Where the `.rrd` of a followed arm's episode is written (the `record` feature): one
    /// `<recording_id>-<arm>-cam.rrd` per episode, beside the arm node's own file. `None`, the
    /// default, records nothing.
    #[serde(default)]
    pub record_dir: Option<PathBuf>,
    #[serde(default, rename = "camera")]
    pub cameras: Vec<CameraConfig>,
}

fn default_name() -> String {
    "franka-cam".into()
}

/// How the node joins the network: one of Zenoh's session modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ZenohMode {
    /// A peer among peers, the default: it listens, and consumers on the same network reach it
    /// directly.
    #[default]
    Peer,
    /// A client of a router it dials: nothing listens here and nothing needs an inbound rule,
    /// which is how a camera behind NAT feeds a consumer in the cloud. `connect` must then name
    /// the router, `listen` is ignored, and an endpoint off the local segment should be `tls/`
    /// or `quic/` rather than `tcp/`.
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

/// The `[zenoh]` table, the same shape as the arm node's.
///
/// Deliberately a copy of `franka_node::ZenohConfig` rather than a dependency: a camera host
/// need not carry the robot client. If a third node ever wants it, extract a crate.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ZenohConfig {
    /// `peer`, the default, or `client` to dial a router instead of listening.
    pub mode: ZenohMode,
    /// Endpoints to listen on. Default `tcp/0.0.0.0:7448`, so a camera node and an arm node
    /// on one host do not fight over a port. Ignored by a client.
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
            listen: vec!["tcp/0.0.0.0:7448".into()],
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

/// One `[[camera]]` table; `name` and `device` are required, the rest has defaults.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraConfig {
    /// The `<name>` in `franka/cam/<name>/*`, `[A-Za-z0-9_-]+`.
    pub name: String,
    /// The V4L2 device. A path under `/dev/v4l/by-id/` survives a replug and a reboot;
    /// `/dev/videoN` does not.
    pub device: PathBuf,
    /// Requested frame size, pixels. The driver may answer with another one, which is what
    /// goes on the wire.
    #[serde(default = "default_width")]
    pub width: u16,
    #[serde(default = "default_height")]
    pub height: u16,
    /// Requested frames per second.
    #[serde(default = "default_fps")]
    pub fps: u32,
    /// The pixel format to ask the camera for. Default `mjpeg`.
    #[serde(default)]
    pub format: Format,
    /// How many mmapped buffers the queue holds.
    #[serde(default = "default_buffers")]
    pub buffers: u32,
    /// The cores the capture thread may run on; empty is every core. Leave the cores the
    /// realtime loops are pinned to out of it.
    #[serde(default)]
    pub cpu: Vec<usize>,
    /// UVC controls applied once at start, by name: `controls = { exposure_auto = 1 }`.
    #[serde(default)]
    pub controls: BTreeMap<String, i32>,
    /// Frames per second on `franka/cam/<name>/preview`, a second key carrying every nth frame
    /// for a consumer that should not pull the full rate: a policy off this network, or an
    /// operator watching over a shared uplink. `None`, the default, publishes no preview key.
    ///
    /// The frames are the same bytes, so this costs one more `put` and no encoding. The divider
    /// comes from `fps`, what was asked for rather than what the driver granted.
    #[serde(default)]
    pub preview_fps: Option<u32>,
    /// The arm whose `franka/<arm>/episode` opens and closes this camera's recording, and whose
    /// `franka/<arm>/state` puts a frame on the robot's clock; `None` records nothing for this
    /// camera. Cameras naming one arm share that episode's file.
    #[serde(default)]
    pub record_with: Option<String>,
}

fn default_width() -> u16 {
    640
}

fn default_height() -> u16 {
    480
}

fn default_fps() -> u32 {
    30
}

fn default_buffers() -> u32 {
    4
}

/// What a configuration file was not.
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("config: {0}")]
    Io(#[from] std::io::Error),
    #[error("config: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("config: {0}")]
    Invalid(String),
}

impl FromStr for CamConfig {
    type Err = ConfigError;

    fn from_str(text: &str) -> Result<Self, ConfigError> {
        let config: CamConfig = toml::from_str(text)?;
        config.validate()?;
        Ok(config)
    }
}

impl CamConfig {
    /// Reads, parses and validates `path`.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        std::fs::read_to_string(path)?.parse()
    }

    /// Checks what the types cannot: the node's own name, the lease, unique key-safe camera
    /// names and each camera's numbers.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if !key_safe(&self.name) {
            return Err(ConfigError::Invalid(format!(
                "node name {:?} is not [A-Za-z0-9_-]+",
                self.name
            )));
        }
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
        #[cfg(not(feature = "record"))]
        if self.record_dir.is_some() {
            return Err(ConfigError::Invalid(
                "record_dir: built without the record feature".into(),
            ));
        }
        for (i, camera) in self.cameras.iter().enumerate() {
            camera.validate()?;
            if camera.name == self.name {
                return Err(ConfigError::Invalid(format!(
                    "camera name {:?} is the node's own name, which reads as a mistake in \
                     a key or a log",
                    camera.name
                )));
            }
            let before = &self.cameras[..i];
            if before.iter().any(|other| other.name == camera.name) {
                return Err(ConfigError::Invalid(format!(
                    "camera name {:?} is not unique",
                    camera.name
                )));
            }
            if before.iter().any(|other| other.device == camera.device) {
                return Err(ConfigError::Invalid(format!(
                    "camera {}: device {} is already another camera's",
                    camera.name,
                    camera.device.display()
                )));
            }
            if camera.record_with.is_some() && self.record_dir.is_none() {
                return Err(ConfigError::Invalid(format!(
                    "camera {}: record_with needs record_dir",
                    camera.name
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

impl CameraConfig {
    /// Publish every nth frame on the preview key, or `None` when there is no preview.
    ///
    /// Rounded up, so the preview never exceeds the rate asked for: rounding to nearest turns
    /// 13 of 30 into every second frame, which is 15 fps, and anything above two thirds of the
    /// camera's rate into every frame, which is the full rate on both keys.
    pub fn preview_every(&self) -> Option<u32> {
        let preview = self.preview_fps?;
        Some((self.fps as f64 / preview as f64).ceil().max(2.0) as u32)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        let camera = &self.name;
        let invalid = |why: String| ConfigError::Invalid(format!("camera {camera}: {why}"));
        if !key_safe(camera) {
            return Err(ConfigError::Invalid(format!(
                "camera name {camera:?} is not [A-Za-z0-9_-]+"
            )));
        }
        if self.device.as_os_str().is_empty() {
            return Err(invalid("device must not be empty".into()));
        }
        if self.width == 0 || self.height == 0 {
            return Err(invalid("width and height must be positive".into()));
        }
        if !(1..=240).contains(&self.fps) {
            return Err(invalid("fps must be in 1..=240".into()));
        }
        if !(2..=32).contains(&self.buffers) {
            return Err(invalid("buffers must be in 2..=32".into()));
        }
        if let Some(preview) = self.preview_fps {
            // At the camera's own rate the preview is the frame key with a second name, and it
            // doubles the bandwidth it exists to save.
            if preview == 0 || preview >= self.fps {
                return Err(invalid(format!(
                    "preview_fps must be in 1..{}, below this camera's fps",
                    self.fps
                )));
            }
        }
        if let Some(arm) = &self.record_with {
            if !key_safe(arm) {
                return Err(invalid(format!(
                    "record_with {arm:?} is not [A-Za-z0-9_-]+"
                )));
            }
        }
        // An affinity naming a core the host does not have would only fail at
        // `sched_setaffinity`, with the camera thread already running.
        if let Some(cores) = crate::sys::cpu_count() {
            if let Some(cpu) = self.cpu.iter().find(|cpu| **cpu >= cores) {
                return Err(invalid(format!(
                    "cpu {cpu} is not a core of this host, which has {cores}"
                )));
            }
        }
        // The one rule the affinity exists to keep: an isolated core belongs to a realtime loop,
        // and a camera thread on it could hold that loop off its cycle.
        let isolated = crate::sys::isolated_cpus();
        if let Some(cpu) = self.cpu.iter().find(|cpu| isolated.contains(cpu)) {
            return Err(invalid(format!(
                "cpu {cpu} is isolated ({isolated:?}) and belongs to a realtime loop"
            )));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
