//! Connection, state stream and motion lifecycle.
//!
//! Port of `franka::Robot::Impl` (libfranka 0.21.2 `src/robot_impl.{h,cpp}`). Everything that
//! talks to the robot lives here; [`crate::robot::Robot`] is a thin, locked facade on top.
//!
//! Unlike the C++ class, `RobotImpl` is `Sync` and every method takes `&self`: the mutable
//! bookkeeping (message id, modes) lives behind one small mutex — the equivalent of libfranka's
//! `message_id_mutex_`, widened to cover the mode fields it also reads and writes — so that
//! `Robot::stop()` can run on another thread while a control loop is running.
//!
//! This module holds the session itself — the handshake, the shared bookkeeping and the state
//! stream. The motion lifecycle (`Move`, the per-cycle command, `StopMove` and the
//! `ActiveControl` write paths) is in the `motion` submodule, and libfranka's `createControlException`
//! text formatting is in the `exception` submodule.

mod exception;
mod motion;

pub(crate) use exception::create_control_exception;

use std::sync::Mutex;

use crate::duration::Duration;
use crate::error::{
    ControlException, FrankaError, FrankaResult, MoveStatus, Record, RobotCommandLog,
};
use crate::errors::Errors;
use crate::joint_velocity_limits::JointVelocityLimitsConfig;
use crate::model::Model;
use crate::network::{connect_handshake, Network};
use crate::realtime::{
    has_realtime_kernel, set_current_thread_to_highest_scheduler_priority, RealtimeConfig,
    NO_REALTIME_KERNEL_MESSAGE,
};
use crate::robot::commands::{
    handle_move_response, handle_stop_move_response, handle_terminal_move_response, CommandName,
};
use crate::robot::logger::RobotStateLogger;
use crate::robot::VersionPolicy;
use crate::robot_state::{RobotMode, RobotState};
use crate::wire::robot::codec::{self, CommandKind, FciVersion, RobotCommandData, StateModes};
use crate::wire::robot::{
    ControllerCommand, ControllerMode as StateControllerMode, Deviation, MotionGeneratorCommand,
    MotionGeneratorMode as StateMotionGeneratorMode, MoveControllerMode, MoveMotionGeneratorMode,
    RobotMode as WireRobotMode,
};
use crate::wire::{
    f64s_to_f64, incorrect_object_size, message_payload, HeaderLayout, ROBOT_COMMAND_PORT,
};

/// Deviations libfranka's control loops and `ActiveControl` pass to every `Move`
/// (`franka::ControlLoop<T>::kDefaultDeviation`).
pub const DEFAULT_DEVIATION: (f64, f64, f64) = (10.0, 3.12, std::f64::consts::TAU);

/// Number of joints, as `franka::RobotControl::kNumJoints`.
pub const NUM_JOINTS: usize = 7;

/// Mutable bookkeeping shared between the control thread and command threads.
///
/// libfranka guards only `message_id_` with a mutex and leaves the mode fields unsynchronised;
/// the Rust port puts them in the same lock because the same fields are read by `stop()` on a
/// second thread.
#[derive(Debug)]
struct State {
    /// `message_id` of the newest accepted state.
    message_id: u64,
    /// Robot mode of the newest accepted state.
    robot_mode: WireRobotMode,
    /// Motion generator mode of the newest accepted state.
    motion_generator_mode: StateMotionGeneratorMode,
    /// Controller mode of the newest accepted state.
    controller_mode: StateControllerMode,
    /// Motion generator mode requested by the running `Move`.
    current_move_motion_generator_mode: StateMotionGeneratorMode,
    /// Controller mode requested by the running `Move`.
    current_move_controller_mode: StateControllerMode,
}

impl State {
    /// `Robot::Impl::motionGeneratorRunning`.
    fn motion_generator_running(&self) -> bool {
        self.motion_generator_mode != StateMotionGeneratorMode::Idle
            && self.motion_generator_mode != StateMotionGeneratorMode::None
    }

    /// `Robot::Impl::controllerRunning`.
    fn controller_running(&self) -> bool {
        self.controller_mode == StateControllerMode::ExternalController
    }

    /// The `Move` we started is fully active, i.e. the state reports both requested modes.
    fn move_active(&self) -> bool {
        self.motion_generator_mode == self.current_move_motion_generator_mode
            && self.controller_mode == self.current_move_controller_mode
    }
}

/// The FCI session plus everything `franka::Robot::Impl` keeps.
#[derive(Debug)]
pub struct RobotImpl {
    network: Network,
    logger: Mutex<RobotStateLogger>,
    realtime_config: RealtimeConfig,
    /// FCI protocol version negotiated at connect time. Every wire touch below goes through
    /// [`crate::wire::robot::codec`] with this value.
    version: FciVersion,
    ri_version: u16,
    state: Mutex<State>,
    /// Position-dependent joint velocity envelope from the URDF. FCI v5 has no `GetRobotModel`
    /// and no such envelope, so on an FER this stays at its default and
    /// [`RobotImpl::upper_joint_velocity_limits`] answers from `rate_limiting::fer` instead.
    joint_velocity_limits: JointVelocityLimitsConfig,
    /// The URDF `GetRobotModel` returned; empty on FCI v5, which has no such command.
    robot_model_urdf: String,
}

impl RobotImpl {
    /// Connects to `franka_address` and performs libfranka's full startup sequence with the
    /// default [`VersionPolicy::Auto`].
    pub fn new(
        franka_address: &str,
        realtime_config: RealtimeConfig,
        log_size: usize,
    ) -> FrankaResult<RobotImpl> {
        RobotImpl::new_with_policy(
            franka_address,
            realtime_config,
            log_size,
            VersionPolicy::default(),
        )
    }

    /// [`RobotImpl::new`] with an explicit [`VersionPolicy`].
    ///
    /// Port of `Robot::Impl::Impl` for both supported protocol versions: raise the calling
    /// thread to the highest realtime priority (fatal only with [`RealtimeConfig::Enforce`]),
    /// check the kernel, run the `Connect` handshake and wait for the first robot state.
    ///
    /// On FCI v10 the URDF is then fetched with `GetRobotModel` and the position-dependent
    /// joint velocity limits are derived from it, exactly as libfranka 0.21.2 does. FCI v5 has
    /// neither command nor envelope — libfranka 0.9.2's constructor stops after the first state
    /// (`src/robot_impl.cpp:19-27`) — so the FER path skips both.
    ///
    /// [`VersionPolicy::Auto`] tries FCI v10 first and, if the server answers
    /// `kIncompatibleLibraryVersion` reporting version 5, closes both sockets and reconnects
    /// once as FCI v5. Any other server version is returned as the
    /// [`FrankaError::IncompatibleVersion`] it is.
    pub fn new_with_policy(
        franka_address: &str,
        realtime_config: RealtimeConfig,
        log_size: usize,
        policy: VersionPolicy,
    ) -> FrankaResult<RobotImpl> {
        let throw_on_error = realtime_config == RealtimeConfig::Enforce;
        if let Err(message) = set_current_thread_to_highest_scheduler_priority() {
            if throw_on_error {
                log_error(&message);
                return Err(FrankaError::Realtime(message));
            }
        }
        if throw_on_error && !has_realtime_kernel() {
            log_error(NO_REALTIME_KERNEL_MESSAGE);
            return Err(FrankaError::Realtime(
                NO_REALTIME_KERNEL_MESSAGE.to_string(),
            ));
        }

        let (network, ri_version, version) = match policy {
            VersionPolicy::Exact(version) => {
                let (network, ri_version) = connect_as(franka_address, version)?;
                (network, ri_version, version)
            }
            VersionPolicy::Auto => match connect_as(franka_address, FciVersion::V10) {
                Ok((network, ri_version)) => (network, ri_version, FciVersion::V10),
                // `connect_as` owns the failed session and has already dropped it, so both
                // sockets are closed before the second connection is opened.
                Err(FrankaError::IncompatibleVersion {
                    server_version: 5, ..
                }) => {
                    let (network, ri_version) = connect_as(franka_address, FciVersion::V5)?;
                    (network, ri_version, FciVersion::V5)
                }
                Err(other) => return Err(other),
            },
        };

        let robot = RobotImpl {
            network,
            logger: Mutex::new(RobotStateLogger::new(log_size)),
            realtime_config,
            version,
            ri_version,
            state: Mutex::new(State {
                message_id: 0,
                robot_mode: WireRobotMode::Other,
                motion_generator_mode: StateMotionGeneratorMode::Idle,
                controller_mode: StateControllerMode::Other,
                current_move_motion_generator_mode: StateMotionGeneratorMode::Idle,
                current_move_controller_mode: StateControllerMode::Other,
            }),
            joint_velocity_limits: JointVelocityLimitsConfig::default(),
            robot_model_urdf: String::new(),
        };

        // `updateState(network_->udpBlockingReceive<RobotState>())`
        let mut buffer = [0u8; codec::ROBOT_STATE_MAX_LEN];
        let size = codec::state_size(version);
        let received = robot.network.blocking_receive_bytes(&mut buffer)?;
        if received != size {
            return Err(incorrect_object_size());
        }
        robot.update_state(&codec::parse_state_modes(version, &buffer[..size])?);

        if version == FciVersion::V5 {
            return Ok(robot);
        }

        let urdf = robot.get_robot_model()?;
        let joint_velocity_limits = JointVelocityLimitsConfig::from_urdf(&urdf)?;

        Ok(RobotImpl {
            robot_model_urdf: urdf,
            joint_velocity_limits,
            ..robot
        })
    }

    /// The FCI protocol version this session speaks (`Robot::fci_version`).
    pub fn version(&self) -> FciVersion {
        self.version
    }

    /// The FCI version reported by the server (`Robot::Impl::serverVersion`).
    pub fn server_version(&self) -> u16 {
        self.ri_version
    }

    /// The realtime configuration this instance was created with
    /// (`Robot::Impl::realtimeConfig`).
    pub fn realtime_config(&self) -> RealtimeConfig {
        self.realtime_config
    }

    /// The URDF fetched at connection time (`Robot::Impl::robotModelUrdf`).
    pub fn robot_model_urdf(&self) -> &str {
        &self.robot_model_urdf
    }

    /// The TCP/UDP session, for the command implementations in
    /// [`crate::robot::commands`].
    pub(crate) fn network(&self) -> &Network {
        &self.network
    }

    /// Upper joint velocity limits at `q` (`Robot::Impl::getUpperJointVelocityLimits`).
    ///
    /// FCI v5 has no position-dependent envelope: libfranka 0.9.2 rate limits against the flat
    /// `kMaxJointVelocity` of `include/franka/rate_limiting.h`, so `q` is ignored there.
    pub fn upper_joint_velocity_limits(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        match self.version {
            FciVersion::V5 => crate::rate_limiting::fer::MAX_JOINT_VELOCITY,
            FciVersion::V10 => self.joint_velocity_limits.upper_limits(q),
        }
    }

    /// Lower joint velocity limits at `q` (`Robot::Impl::getLowerJointVelocityLimits`).
    ///
    /// See [`RobotImpl::upper_joint_velocity_limits`] for the FCI v5 case.
    pub fn lower_joint_velocity_limits(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        match self.version {
            FciVersion::V5 => crate::rate_limiting::fer::MIN_JOINT_VELOCITY,
            FciVersion::V10 => self.joint_velocity_limits.lower_limits(q),
        }
    }

    /// Loads the FER's model library over `LoadModelLibrary` (FCI v5's `Robot::loadModel`).
    ///
    /// Downloads `libfcimodels.so` from the robot and binds its thirty symbols
    /// ([`crate::model::load_from_robot`]).
    pub fn load_model_v5(&self) -> FrankaResult<Model> {
        crate::model::load_from_robot(&self.network, self.version)
    }

    /// The wire command id of `name` under the negotiated version.
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] when the command does not exist in this FCI version —
    /// `Get Robot Model` on an FER, `Set Filters` and `Get Cartesian Limit` on an FR3.
    pub(crate) fn command_id(&self, name: CommandName) -> FrankaResult<u32> {
        codec::command_id(self.version, name.kind()).ok_or_else(|| {
            FrankaError::InvalidOperation(format!(
                "libfranka: {} is not available on FCI version {}.",
                name.as_str(),
                self.version.number()
            ))
        })
    }

    /// The current robot mode, used by `commandNotPossibleMsg`.
    pub(crate) fn robot_mode(&self) -> RobotMode {
        let mode = self.lock().robot_mode;
        RobotMode::from_u8(mode.to_u8()).unwrap_or(RobotMode::Other)
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn logger(&self) -> std::sync::MutexGuard<'_, RobotStateLogger> {
        self.logger.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `Robot::Impl::updateState`.
    fn update_state(&self, modes: &StateModes) {
        let mut guard = self.lock();
        guard.robot_mode = WireRobotMode::from_u8(modes.robot_mode).unwrap_or(WireRobotMode::Other);
        guard.motion_generator_mode =
            StateMotionGeneratorMode::from_u8(modes.motion_generator_mode)
                .unwrap_or(StateMotionGeneratorMode::Idle);
        guard.controller_mode = StateControllerMode::from_u8(modes.controller_mode)
            .unwrap_or(StateControllerMode::Other);
        guard.message_id = modes.message_id;
    }

    /// Receives the newest robot state (`Robot::Impl::receiveRobotState`).
    ///
    /// Drains everything already queued on the socket keeping the highest `message_id`, then
    /// blocks until a state newer than the last accepted one arrives.
    ///
    /// Both buffers are sized for the largest supported `RobotState` and every datagram's
    /// length is compared against the negotiated version's, so a state of the *other* version
    /// fails with libfranka's `Protocol("libfranka: incorrect object size")`
    /// (`src/network.h:140-142`) instead of being truncated into a plausible-looking state.
    /// Nothing here allocates: the two buffers live on the caller's stack.
    fn receive_robot_state(&self) -> FrankaResult<RobotState> {
        let size = codec::state_size(self.version);
        let last_message_id = self.lock().message_id;
        let mut buffer = [0u8; codec::ROBOT_STATE_MAX_LEN];
        let mut latest = [0u8; codec::ROBOT_STATE_MAX_LEN];
        let mut latest_message_id = last_message_id;

        while let Some(received) = self.network.try_receive_bytes(&mut buffer)? {
            if received != size {
                return Err(incorrect_object_size());
            }
            let modes = codec::parse_state_modes(self.version, &buffer[..size])?;
            if modes.message_id > latest_message_id {
                latest_message_id = modes.message_id;
                latest[..size].copy_from_slice(&buffer[..size]);
            }
        }

        while latest_message_id == last_message_id {
            let received = self.network.blocking_receive_bytes(&mut buffer)?;
            if received != size {
                return Err(incorrect_object_size());
            }
            let modes = codec::parse_state_modes(self.version, &buffer[..size])?;
            if modes.message_id > latest_message_id {
                latest_message_id = modes.message_id;
                latest[..size].copy_from_slice(&buffer[..size]);
            }
        }

        self.update_state(&codec::parse_state_modes(self.version, &latest[..size])?);
        codec::parse_robot_state(self.version, &latest[..size])
    }
}

/// Opens a fresh FCI session and runs the `Connect` handshake for `version`.
///
/// The [`Network`] is owned by this function, so a failing handshake — an
/// [`FrankaError::IncompatibleVersion`] in particular — drops both sockets before the error
/// reaches the caller. That is what makes [`VersionPolicy::Auto`]'s retry a *new* connection
/// rather than a second handshake on the socket the server just rejected.
///
/// **Deviation from libfranka**, which only inspects the `Connect` *status*: a handshake that
/// succeeded but whose response reports a different protocol version is turned into the same
/// [`FrankaError::IncompatibleVersion`] as an outright rejection. Everything after the
/// handshake is decoded against `version`, so a server that answers `kSuccess` while speaking
/// another version would otherwise be met with 2373-byte states on a 1377-byte session — which
/// surfaces as `Protocol("libfranka: incorrect object size")` on the very first state instead
/// of naming the real problem. franka-sim's FER build does exactly this (it answers
/// `kSuccess` with version 5 whatever the client announced), and it is what lets
/// [`VersionPolicy::Auto`] recognise an FER there.
fn connect_as(franka_address: &str, version: FciVersion) -> FrankaResult<(Network, u16)> {
    // `connect_handshake` sends the shared `Connect` id rather than asking the codec for it,
    // which is only correct because both versions number `Connect` as 0.
    const _: () = assert!(matches!(
        codec::command_id(FciVersion::V5, CommandKind::Connect),
        Some(0)
    ));
    const _: () = assert!(matches!(
        codec::command_id(FciVersion::V10, CommandKind::Connect),
        Some(0)
    ));

    let library_version = codec::connect_version(version);
    let network = Network::connect(franka_address, ROBOT_COMMAND_PORT, HeaderLayout::Robot)?;
    let ri_version = connect_handshake(&network, library_version)?;
    if ri_version != library_version {
        return Err(FrankaError::IncompatibleVersion {
            server_version: ri_version,
            library_version,
        });
    }
    Ok((network, ri_version))
}

/// The version-agnostic log record of one sent command.
fn command_log(command: &RobotCommandData) -> RobotCommandLog {
    RobotCommandLog {
        q_c: command.q_c,
        dq_c: command.dq_c,
        O_T_EE_c: command.O_T_EE_c,
        O_dP_EE_c: command.O_dP_EE_c,
        elbow_c: command.elbow_c,
        tau_J_d: command.tau_J_d,
    }
}

/// Builds a [`FrankaError::Control`] without a log, as the C++ `ControlException(msg)` does.
pub(crate) fn control_error(message: &str) -> FrankaError {
    FrankaError::Control(ControlException::new(message))
}

/// First payload byte of a command response.
pub(crate) fn status_byte(message: &[u8]) -> FrankaResult<u8> {
    let payload = message_payload(HeaderLayout::Robot, message);
    payload
        .first()
        .copied()
        .ok_or_else(|| FrankaError::Protocol("libfranka: Incorrect TCP message size.".to_string()))
}

/// libfranka's `logging::logError`, which writes to `std::cerr` through the default sink.
pub(crate) fn log_error(message: &str) {
    eprintln!("{message}");
}

/// libfranka's `logging::logWarn`.
pub(crate) fn log_warn(message: &str) {
    eprintln!("{message}");
}

/// The `franka::Duration` between two states, used by `ActiveControl::readOnce`.
pub(crate) fn time_since(previous: Option<Duration>, now: Duration) -> Duration {
    match previous {
        Some(previous) => now - previous,
        None => Duration::default(),
    }
}

#[cfg(test)]
mod tests;
