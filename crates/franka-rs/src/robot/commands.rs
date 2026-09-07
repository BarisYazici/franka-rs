//! The TCP commands of the robot server and libfranka's response handling.
//!
//! Port of `Robot::Impl::executeCommand` and the `handleCommandResponse` overloads in
//! `src/robot_impl.h` (libfranka 0.21.2). Every rejection text — including
//! `commandNotPossibleMsg`, which names the current robot mode and adds
//! `" Did you open the brakes?"` when it is `Other` — is reproduced verbatim.

use zerocopy::{FromBytes, IntoBytes};

use crate::error::{FrankaError, FrankaResult, MoveStatus};
use crate::robot::robot_impl::{status_byte, RobotImpl};
use crate::robot::VirtualWallCuboid;
use crate::wire::robot::codec::{self, CommandKind, FciVersion};
use crate::wire::robot::v5::{
    GetCartesianLimitRequest, GetCartesianLimitResponse, SetFiltersRequest,
};
use crate::wire::robot::{
    AutomaticErrorRecoveryStatus, CommandStatus, GetterSetterStatus, SetCartesianImpedanceRequest,
    SetCollisionBehaviorRequest, SetEEToKRequest, SetGuidingModeRequest, SetJointImpedanceRequest,
    SetLoadRequest, SetNEToEERequest, StopMoveStatus,
};
use crate::wire::{f64s_to_f64, message_payload, HeaderLayout};

/// The command names libfranka puts into its error texts
/// (`research_interface::robot::CommandTraits<T>::kName`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandName {
    /// Fetches the robot's URDF (`"Get Robot Model"`). FCI v10 only.
    GetRobotModel,
    /// Starts a motion (`"Move"`).
    Move,
    /// Stops the running motion (`"Stop Move"`), the command behind
    /// [`crate::Robot::stop`].
    StopMove,
    /// Reads a virtual wall cuboid (`"Get Cartesian Limit"`). FCI v5 only
    /// (`Robot::getVirtualWall`).
    GetCartesianLimit,
    /// Port of `franka::Robot::setCollisionBehavior` (`"Set Collision Behavior"`).
    SetCollisionBehavior,
    /// Port of `franka::Robot::setJointImpedance` (`"Set Joint Impedance"`).
    SetJointImpedance,
    /// Port of `franka::Robot::setCartesianImpedance` (`"Set Cartesian Impedance"`).
    SetCartesianImpedance,
    /// Port of `franka::Robot::setGuidingMode` (`"Set Guiding Mode"`).
    SetGuidingMode,
    /// Port of `franka::Robot::setK` (`"Set EE To K"`).
    SetEEToK,
    /// Port of `franka::Robot::setEE` (`"Set NE To EE"`).
    SetNEToEE,
    /// Port of `franka::Robot::setLoad` (`"Set Load"`).
    SetLoad,
    /// Port of `franka::Robot::setFilters` (`"Set Filters"`). FCI v5 only.
    SetFilters,
    /// Clears a reflex and re-enables motion (`"Automatic Error Recovery"`), the command behind
    /// [`crate::Robot::automatic_error_recovery`].
    AutomaticErrorRecovery,
}

impl CommandName {
    /// The codec's version-agnostic command identity, which carries the wire numbering of both
    /// versions ([`codec::command_id`]) and libfranka's `CommandTraits<T>::kName`.
    pub(crate) const fn kind(self) -> CommandKind {
        match self {
            CommandName::GetRobotModel => CommandKind::GetRobotModel,
            CommandName::Move => CommandKind::Move,
            CommandName::StopMove => CommandKind::StopMove,
            CommandName::GetCartesianLimit => CommandKind::GetCartesianLimit,
            CommandName::SetCollisionBehavior => CommandKind::SetCollisionBehavior,
            CommandName::SetJointImpedance => CommandKind::SetJointImpedance,
            CommandName::SetCartesianImpedance => CommandKind::SetCartesianImpedance,
            CommandName::SetGuidingMode => CommandKind::SetGuidingMode,
            CommandName::SetEEToK => CommandKind::SetEEToK,
            CommandName::SetNEToEE => CommandKind::SetNEToEE,
            CommandName::SetLoad => CommandKind::SetLoad,
            CommandName::SetFilters => CommandKind::SetFilters,
            CommandName::AutomaticErrorRecovery => CommandKind::AutomaticErrorRecovery,
        }
    }

    /// The name used in error messages.
    pub const fn as_str(self) -> &'static str {
        self.kind().name()
    }

    /// The wire command id of this command under `version`, or `None` when that FCI version
    /// does not have the command (`Get Robot Model` on an FER, `Set Filters` and
    /// `Get Cartesian Limit` on an FR3).
    ///
    /// This replaces the version-less `command()` of the FR3-only releases: the two protocol
    /// versions number their commands differently from `SetCollisionBehavior` onwards, so a
    /// wire id is only meaningful together with a version. `RobotImpl::command_id` is the
    /// same lookup against a live connection's negotiated version, with the
    /// [`FrankaError::InvalidOperation`] text libfranka would print.
    pub const fn command(self, version: FciVersion) -> Option<u32> {
        codec::command_id(version, self.kind())
    }
}

/// Port of `Robot::Impl::commandNotPossibleMsg`.
fn command_not_possible_message(robot: &RobotImpl) -> String {
    let mode = robot.robot_mode();
    let mut message =
        format!(" command rejected: command not possible in the current mode (\"{mode}\")!");
    if mode == crate::robot_state::RobotMode::Other {
        message.push_str(" Did you open the brakes?");
    }
    message
}

/// A `CommandException` with libfranka's `"libfranka: " + kName + <detail>` layout.
fn command_error(name: CommandName, detail: &str) -> FrankaError {
    FrankaError::Command(format!("libfranka: {}{detail}", name.as_str()))
}

/// Port of the `CommandBase` `handleCommandResponse` overload, used by `GetRobotModel`.
///
/// `CommandBase::Status` and `GetterSetterCommandBase::Status` agree on `0` and `1` only, so
/// the two families need separate handlers.
pub(crate) fn handle_command_response(
    robot: &RobotImpl,
    name: CommandName,
    status: CommandStatus,
) -> FrankaResult<()> {
    match status {
        CommandStatus::Success => Ok(()),
        CommandStatus::CommandNotPossibleRejected => {
            Err(command_error(name, &command_not_possible_message(robot)))
        }
        CommandStatus::CommandRejectedDueToActivatedSafetyFunctions => Err(command_error(
            name,
            " command rejected due to activated safety function! Please disable all safety \
             functions.",
        )),
    }
}

/// Port of the `IsBaseOfGetterSetter` `handleCommandResponse` overload, shared by every
/// setter command.
pub(crate) fn handle_getter_setter_response(
    robot: &RobotImpl,
    name: CommandName,
    status: GetterSetterStatus,
) -> FrankaResult<()> {
    match status {
        GetterSetterStatus::Success => Ok(()),
        GetterSetterStatus::CommandNotPossibleRejected => {
            Err(command_error(name, &command_not_possible_message(robot)))
        }
        GetterSetterStatus::InvalidArgumentRejected => {
            Err(command_error(name, " command rejected: invalid argument!"))
        }
        GetterSetterStatus::CommandRejectedDueToActivatedSafetyFunctions => Err(command_error(
            name,
            " command rejected due to activated safety function! Please disable all safety \
             functions. ",
        )),
    }
}

/// Port of `handleCommandResponse<research_interface::robot::Move>`.
///
/// `kMotionStarted` is accepted only while no motion generator is running, exactly like the
/// C++ overload (`robot_impl.h:388-395`).
pub(crate) fn handle_move_response(robot: &RobotImpl, status: MoveStatus) -> FrankaResult<()> {
    handle_move_status(status, robot.motion_generator_running(), robot)
}

/// [`handle_move_response`] for the *terminal* `Move` reply of a motion this client started,
/// i.e. the one claimed by `Robot::Impl::finishMotion` and `Robot::Impl::throwOnMotionError`.
///
/// At that point a motion **is** running as far as the client is concerned -- the `motion_id`
/// belongs to a `Move` that was started and has not been answered yet -- so libfranka's
/// `motionGeneratorRunning()` guard on `kMotionStarted` is evaluated as `true` here. The C++
/// code reads the same guard off the *last robot state*, which has already dropped back to
/// `kIdle` by the time `finishMotion` claims the reply, and therefore silently accepts a second
/// `kMotionStarted` as if it were `kSuccess`. Diverging here is deliberate and is what makes
/// `Ok(())` out of `Robot::control_*` mean "the terminal status was `kSuccess`".
pub(crate) fn handle_terminal_move_response(
    robot: &RobotImpl,
    status: MoveStatus,
) -> FrankaResult<()> {
    handle_move_status(status, true, robot)
}

fn handle_move_status(
    status: MoveStatus,
    motion_running: bool,
    robot: &RobotImpl,
) -> FrankaResult<()> {
    const NAME: CommandName = CommandName::Move;
    match status {
        MoveStatus::Success => Ok(()),
        MoveStatus::MotionStarted => {
            if motion_running {
                return Err(FrankaError::Protocol(
                    "libfranka: Move received unexpected motion started message.".to_string(),
                ));
            }
            Ok(())
        }
        MoveStatus::EmergencyAborted => {
            Err(command_error(NAME, " command aborted: User Stop pressed!"))
        }
        MoveStatus::ReflexAborted => Err(command_error(
            NAME,
            " command aborted: motion aborted by reflex!",
        )),
        MoveStatus::InputErrorAborted => Err(command_error(
            NAME,
            " command aborted: invalid input provided!",
        )),
        MoveStatus::CommandNotPossibleRejected => {
            Err(command_error(NAME, &command_not_possible_message(robot)))
        }
        MoveStatus::StartAtSingularPoseRejected => Err(command_error(
            NAME,
            " command rejected: cannot start at singular pose!",
        )),
        MoveStatus::InvalidArgumentRejected => Err(command_error(
            NAME,
            " command rejected: maximum path deviation out of range!",
        )),
        MoveStatus::Preempted => Err(command_error(NAME, " command preempted!")),
        MoveStatus::Aborted => Err(command_error(NAME, " command aborted!")),
        MoveStatus::PreemptedDueToActivatedSafetyFunctions => Err(command_error(
            NAME,
            " command preempted due to activated safety function! Please disable all safety \
             functions.",
        )),
        MoveStatus::CommandRejectedDueToActivatedSafetyFunctions => Err(command_error(
            NAME,
            " command rejected due to activated safety function! Please disable all safety \
             functions.",
        )),
    }
}

/// Port of `handleCommandResponse<research_interface::robot::StopMove>`.
///
/// Note that libfranka reports `kAborted` with the "command not possible" text and names the
/// *`Move`* command in the safety-function case; both quirks are reproduced.
pub(crate) fn handle_stop_move_response(
    robot: &RobotImpl,
    status: StopMoveStatus,
) -> FrankaResult<()> {
    const NAME: CommandName = CommandName::StopMove;
    match status {
        StopMoveStatus::Success => Ok(()),
        StopMoveStatus::CommandNotPossibleRejected | StopMoveStatus::Aborted => {
            Err(command_error(NAME, &command_not_possible_message(robot)))
        }
        StopMoveStatus::EmergencyAborted => {
            Err(command_error(NAME, " command aborted: User Stop pressed!"))
        }
        StopMoveStatus::ReflexAborted => Err(command_error(
            NAME,
            " command aborted: motion aborted by reflex!",
        )),
        StopMoveStatus::CommandRejectedDueToActivatedSafetyFunctions => Err(command_error(
            CommandName::Move,
            " command rejected due to activated safety function! Please disable all safety \
                 functions.",
        )),
    }
}

/// Port of `handleCommandResponse<research_interface::robot::AutomaticErrorRecovery>`.
pub(crate) fn handle_automatic_error_recovery_response(
    robot: &RobotImpl,
    status: AutomaticErrorRecoveryStatus,
) -> FrankaResult<()> {
    const NAME: CommandName = CommandName::AutomaticErrorRecovery;
    match status {
        AutomaticErrorRecoveryStatus::Success => Ok(()),
        AutomaticErrorRecoveryStatus::EmergencyAborted => {
            Err(command_error(NAME, " command aborted: User Stop pressed!"))
        }
        AutomaticErrorRecoveryStatus::ReflexAborted => Err(command_error(
            NAME,
            " command aborted: motion aborted by reflex!",
        )),
        AutomaticErrorRecoveryStatus::CommandNotPossibleRejected => {
            Err(command_error(NAME, &command_not_possible_message(robot)))
        }
        AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected => Err(command_error(
            NAME,
            " command rejected: manual error recovery required!",
        )),
        AutomaticErrorRecoveryStatus::Aborted => Err(command_error(NAME, " command aborted!")),
        AutomaticErrorRecoveryStatus::CommandRejectedDueToActivatedSafetyFunctions => {
            Err(command_error(
                CommandName::Move,
                " command rejected due to activated safety function! Please disable all safety \
                 functions.",
            ))
        }
    }
}

impl RobotImpl {
    /// Sends a request, blocks for its response and returns the whole message.
    ///
    /// The wire command id comes from the negotiated version, so a command that does not exist
    /// there fails with [`FrankaError::InvalidOperation`] before anything is sent
    /// ([`RobotImpl::command_id`]).
    fn execute(&self, name: CommandName, payload: &[u8]) -> FrankaResult<Vec<u8>> {
        let command_id = self
            .network()
            .tcp
            .send_request(self.command_id(name)?, payload)?;
        self.network().tcp.blocking_receive_response(command_id)
    }

    /// `executeCommand<T>` for the getter/setter family.
    fn execute_setter(&self, name: CommandName, payload: &[u8]) -> FrankaResult<()> {
        let message = self.execute(name, payload)?;
        let status = codec::parse_getter_setter_status(
            self.version(),
            status_byte(&message)?,
            name.as_str(),
        )?;
        handle_getter_setter_response(self, name, status)
    }

    /// `GetRobotModel`: returns the robot's URDF.
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] on FCI v5, which has no such command — an FER serves
    /// its model as a shared object through `LoadModelLibrary` instead.
    pub fn get_robot_model(&self) -> FrankaResult<String> {
        const NAME: CommandName = CommandName::GetRobotModel;
        let message = self.execute(NAME, &[])?;
        let status =
            codec::parse_command_status(self.version(), status_byte(&message)?, NAME.as_str())?;
        handle_command_response(self, NAME, status)?;
        let payload = message_payload(HeaderLayout::Robot, &message);
        Ok(String::from_utf8_lossy(&payload[1..]).into_owned())
    }

    /// `Robot::setFilters` (libfranka 0.9.2 `src/robot.cpp:216-225`), FCI v5 only.
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] on FCI v10, which dropped the command.
    pub fn set_filters(
        &self,
        joint_position_filter_frequency: f64,
        joint_velocity_filter_frequency: f64,
        cartesian_position_filter_frequency: f64,
        cartesian_velocity_filter_frequency: f64,
        controller_filter_frequency: f64,
    ) -> FrankaResult<()> {
        let request = SetFiltersRequest::new(
            joint_position_filter_frequency,
            joint_velocity_filter_frequency,
            cartesian_position_filter_frequency,
            cartesian_velocity_filter_frequency,
            controller_filter_frequency,
        );
        self.execute_setter(CommandName::SetFilters, request.as_bytes())
    }

    /// `Robot::getVirtualWall` (libfranka 0.9.2 `src/robot.cpp:227-231`), FCI v5 only.
    ///
    /// The 154-byte `GetCartesianLimit::Response` is mapped exactly as the C++
    /// `executeCommand<GetCartesianLimit>` specialisation does (`src/robot_impl.h:284-300`):
    /// `p_frame` is the response's `object_frame`, `active` its `object_activation`, and `id`
    /// is echoed from the request rather than read off the wire.
    ///
    /// # Errors
    /// [`FrankaError::InvalidOperation`] on FCI v10, which dropped the command.
    pub fn virtual_wall(&self, id: i32) -> FrankaResult<VirtualWallCuboid> {
        const NAME: CommandName = CommandName::GetCartesianLimit;
        let request = GetCartesianLimitRequest::new(id);
        let message = self.execute(NAME, request.as_bytes())?;
        let payload = message_payload(HeaderLayout::Robot, &message);
        let response = GetCartesianLimitResponse::read_from_bytes(payload).map_err(|_| {
            FrankaError::Protocol("libfranka: Incorrect TCP message size.".to_string())
        })?;

        let wall = VirtualWallCuboid {
            id,
            object_world_size: f64s_to_f64(&response.object_world_size),
            p_frame: f64s_to_f64(&response.object_frame),
            active: response.object_activation != 0,
        };

        let status =
            codec::parse_getter_setter_status(self.version(), response.status, NAME.as_str())?;
        handle_getter_setter_response(self, NAME, status)?;
        Ok(wall)
    }

    /// `Robot::setCollisionBehavior` with all eight threshold arrays.
    #[allow(clippy::too_many_arguments)]
    pub fn set_collision_behavior(
        &self,
        lower_torque_thresholds_acceleration: &[f64; 7],
        upper_torque_thresholds_acceleration: &[f64; 7],
        lower_torque_thresholds_nominal: &[f64; 7],
        upper_torque_thresholds_nominal: &[f64; 7],
        lower_force_thresholds_acceleration: &[f64; 6],
        upper_force_thresholds_acceleration: &[f64; 6],
        lower_force_thresholds_nominal: &[f64; 6],
        upper_force_thresholds_nominal: &[f64; 6],
    ) -> FrankaResult<()> {
        let request = SetCollisionBehaviorRequest::new(
            lower_torque_thresholds_acceleration,
            upper_torque_thresholds_acceleration,
            lower_torque_thresholds_nominal,
            upper_torque_thresholds_nominal,
            lower_force_thresholds_acceleration,
            upper_force_thresholds_acceleration,
            lower_force_thresholds_nominal,
            upper_force_thresholds_nominal,
        );
        self.execute_setter(CommandName::SetCollisionBehavior, request.as_bytes())
    }

    /// `Robot::setJointImpedance`.
    pub fn set_joint_impedance(&self, K_theta: &[f64; 7]) -> FrankaResult<()> {
        let request = SetJointImpedanceRequest::new(K_theta);
        self.execute_setter(CommandName::SetJointImpedance, request.as_bytes())
    }

    /// `Robot::setCartesianImpedance`.
    pub fn set_cartesian_impedance(&self, K_x: &[f64; 6]) -> FrankaResult<()> {
        let request = SetCartesianImpedanceRequest::new(K_x);
        self.execute_setter(CommandName::SetCartesianImpedance, request.as_bytes())
    }

    /// `Robot::setGuidingMode`.
    pub fn set_guiding_mode(&self, guiding_mode: &[bool; 6], elbow: bool) -> FrankaResult<()> {
        let request = SetGuidingModeRequest::new(guiding_mode, elbow);
        self.execute_setter(CommandName::SetGuidingMode, request.as_bytes())
    }

    /// `Robot::setK`.
    pub fn set_k(&self, EE_T_K: &[f64; 16]) -> FrankaResult<()> {
        let request = SetEEToKRequest::new(EE_T_K);
        self.execute_setter(CommandName::SetEEToK, request.as_bytes())
    }

    /// `Robot::setEE`.
    pub fn set_ee(&self, NE_T_EE: &[f64; 16]) -> FrankaResult<()> {
        let request = SetNEToEERequest::new(NE_T_EE);
        self.execute_setter(CommandName::SetNEToEE, request.as_bytes())
    }

    /// `Robot::setLoad`.
    pub fn set_load(
        &self,
        load_mass: f64,
        F_x_Cload: &[f64; 3],
        load_inertia: &[f64; 9],
    ) -> FrankaResult<()> {
        let request = SetLoadRequest::new(load_mass, F_x_Cload, load_inertia);
        self.execute_setter(CommandName::SetLoad, request.as_bytes())
    }

    /// `Robot::automaticErrorRecovery`.
    pub fn automatic_error_recovery(&self) -> FrankaResult<()> {
        const NAME: CommandName = CommandName::AutomaticErrorRecovery;
        let message = self.execute(NAME, &[])?;
        let status = codec::parse_automatic_error_recovery_status(
            self.version(),
            status_byte(&message)?,
            NAME.as_str(),
        )?;
        handle_automatic_error_recovery_response(self, status)
    }

    /// `Robot::stop`: a bare `StopMove`, callable while a control loop is running.
    pub fn stop(&self) -> FrankaResult<()> {
        let message = self.execute(CommandName::StopMove, &[])?;
        handle_stop_move_response(self, self.stop_move_status(&message)?)
    }
}
