//! Connect-time options and the value types the FCI v5 getters return.

use crate::realtime::RealtimeConfig;
use crate::wire::robot::codec::FciVersion;

use super::DEFAULT_LOG_SIZE;

/// How [`crate::Robot`] decides which FCI protocol version to speak.
///
/// The two supported versions are not distinguishable before the `Connect` handshake, so the
/// default probes: FCI v10 first, then FCI v5 if the server rejects it *and* reports version 5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VersionPolicy {
    /// Connect as FCI v10 (FR3). If the server answers `kIncompatibleLibraryVersion` reporting
    /// version 5, close both sockets and reconnect once as FCI v5 (FER). Any other server
    /// version is reported as [`crate::error::FrankaError::IncompatibleVersion`].
    #[default]
    Auto,
    /// Connect as exactly this version and never retry; a mismatch is
    /// [`crate::error::FrankaError::IncompatibleVersion`].
    Exact(FciVersion),
}

/// Everything [`crate::Robot::with_options`] can be told at connect time.
///
/// The defaults are [`crate::Robot::new`]'s: [`RealtimeConfig::Enforce`] is *not* assumed — the field
/// has no libfranka default and must be chosen — so [`RobotOptions::new`] takes it and the
/// [`Default`] impl uses [`RealtimeConfig::Enforce`], matching `franka::Robot`'s default
/// argument.
#[derive(Debug, Clone)]
pub struct RobotOptions {
    /// Realtime priority policy, as [`crate::Robot::new`]'s second argument.
    pub realtime_config: RealtimeConfig,
    /// Which FCI version to speak.
    pub version: VersionPolicy,
    /// Size of the control log attached to a [`crate::error::ControlException`]; `0` disables
    /// logging.
    pub log_size: usize,
}

impl Default for RobotOptions {
    fn default() -> Self {
        RobotOptions {
            realtime_config: RealtimeConfig::Enforce,
            version: VersionPolicy::Auto,
            log_size: DEFAULT_LOG_SIZE,
        }
    }
}

impl RobotOptions {
    /// Options with `realtime_config`, [`VersionPolicy::Auto`] and [`DEFAULT_LOG_SIZE`].
    pub fn new(realtime_config: RealtimeConfig) -> RobotOptions {
        RobotOptions {
            realtime_config,
            ..RobotOptions::default()
        }
    }

    /// Sets the version policy.
    pub fn with_version(mut self, version: VersionPolicy) -> RobotOptions {
        self.version = version;
        self
    }

    /// Sets the control log size.
    pub fn with_log_size(mut self, log_size: usize) -> RobotOptions {
        self.log_size = log_size;
        self
    }
}

/// Parameters of a cuboid used as a virtual wall (`franka::VirtualWallCuboid`,
/// libfranka 0.9.2 `include/franka/command_types.h`).
///
/// Returned by [`crate::Robot::virtual_wall`], which is FCI v5 only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VirtualWallCuboid {
    /// ID of the virtual wall.
    pub id: i32,
    /// Corner point of the cuboid in the world frame, in \[m\].
    pub object_world_size: [f64; 3],
    /// 4x4 transformation matrix, column-major.
    pub p_frame: [f64; 16],
    /// `true` if this Cartesian limit is active.
    pub active: bool,
}
