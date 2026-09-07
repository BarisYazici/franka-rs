//! FCI v5 (FER) cases: the handshake and version policies, the v5 command numbering, the
//! FER-only commands and the v5 rate-limiting difference.

use super::*;

/// The `Connect` status byte for `kIncompatibleLibraryVersion`.
const INCOMPATIBLE: u8 = 1;

/// An FER server answers the handshake with version 5, streams 2373-byte states, and has no
/// `GetRobotModel`.
#[test]
fn v5_handshake_reports_version_5_and_parses_2373_byte_states() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    assert_eq!(robot.version(), FciVersion::V5);
    assert_eq!(robot.server_version(), 5);
    assert_eq!(codec::state_size(FciVersion::V5), 2373);

    // The constructor consumed one state; a second one must parse just as well.
    let state = v5::RobotState {
        robot_mode: WireRobotMode::Idle.to_u8(),
        q: crate::wire::f64s_to_wire(&[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]),
        ..idle_state_v5()
    };
    server.schedule(20, move |shared| shared.send_state_bytes(state.as_bytes()));
    let read = robot.read_once().expect("read_once failed");
    assert_eq!(read.robot_mode, crate::robot_state::RobotMode::Idle);
    assert_eq!(read.q, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]);

    // FCI v5 has no `GetRobotModel`.
    match robot.get_robot_model() {
        Err(FrankaError::InvalidOperation(message)) => assert_eq!(
            message,
            "libfranka: Get Robot Model is not available on FCI version 5."
        ),
        other => panic!("expected InvalidOperation, got {other:?}"),
    }
}

/// `VersionPolicy::Auto` connects as v10, and when the server rejects that reporting version 5
/// it closes both sockets and reconnects once as v5.
#[test]
fn auto_policy_reconnects_as_v5_after_an_incompatible_v10_handshake() {
    let server = MockServer::start_with(FciVersion::V5, vec![(INCOMPATIBLE, 5)]);
    let robot = server
        .connect_with_policy(0, VersionPolicy::Auto)
        .expect("the Auto policy must fall back to FCI v5");

    assert_eq!(robot.version(), FciVersion::V5);
    assert_eq!(robot.server_version(), 5);
    assert_eq!(server.request_count(Command::Connect), 2);

    // The first TCP connection was closed before the second was opened.
    assert_eq!(
        *server.shared.connection_events.lock().unwrap(),
        vec!["accept", "close", "accept"]
    );

    // The rejected handshake announced 10, the accepted one 5.
    let versions: Vec<u16> = server
        .payloads_for(CommandKind::Connect)
        .iter()
        .map(|payload| u16::from_le_bytes([payload[0], payload[1]]))
        .collect();
    assert_eq!(versions, vec![10, 5]);
}

/// franka-sim's FER build answers `Connect` with `kSuccess` *and* version 5 no matter what
/// the client announced. The reported version, not just the status, decides: `Auto` still
/// recognises the FER and reconnects, and `Exact(V10)` still refuses.
#[test]
fn a_success_handshake_reporting_version_5_is_treated_as_an_fer() {
    {
        let server = MockServer::start_with(
            FciVersion::V5,
            vec![
                (ConnectStatus::Success.to_u8(), 5),
                (ConnectStatus::Success.to_u8(), 5),
            ],
        );
        let robot = server
            .connect_with_policy(0, VersionPolicy::Auto)
            .expect("Auto must fall back on a success handshake reporting version 5");
        assert_eq!(robot.version(), FciVersion::V5);
        assert_eq!(server.request_count(Command::Connect), 2);
        assert_eq!(
            *server.shared.connection_events.lock().unwrap(),
            vec!["accept", "close", "accept"]
        );
    }

    let server = MockServer::start_with(
        FciVersion::V5,
        vec![
            (ConnectStatus::Success.to_u8(), 5),
            (ConnectStatus::Success.to_u8(), 5),
        ],
    );
    match server.connect_with_policy(0, VersionPolicy::Exact(FciVersion::V10)) {
        Err(FrankaError::IncompatibleVersion {
            server_version: 5,
            library_version: 10,
        }) => {}
        other => panic!("expected IncompatibleVersion {{5, 10}}, got {other:?}"),
    }
}

/// `VersionPolicy::Exact` never retries: a v10 client on an FER gets libfranka's
/// `IncompatibleVersionException`.
#[test]
fn exact_v10_against_a_v5_server_is_an_incompatible_version_error() {
    let server = MockServer::start_with(FciVersion::V5, vec![(INCOMPATIBLE, 5), (INCOMPATIBLE, 5)]);
    match server.connect_with_policy(0, VersionPolicy::Exact(FciVersion::V10)) {
        Err(FrankaError::IncompatibleVersion {
            server_version: 5,
            library_version: 10,
        }) => {}
        other => panic!("expected IncompatibleVersion {{5, 10}}, got {other:?}"),
    }
    assert_eq!(
        server.request_count(Command::Connect),
        1,
        "Exact must not retry"
    );
}

/// An FER's `Move` request is 56 bytes — no `use_async_motion_generator`, no maximum
/// velocities — and rides on the v5 command id.
#[test]
fn move_request_on_v5_is_56_bytes() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    server.queue_response_for(CommandKind::Move, &[v5::MoveStatus::MotionStarted.to_u8()]);
    let running = server.moving_bytes(
        StateMotionGeneratorMode::JointVelocity,
        StateControllerMode::JointImpedance,
    );
    server.schedule(20, move |shared| shared.send_state_bytes(&running));

    start_joint_velocity_motion(&robot).expect("motion should start");

    let payloads = server.payloads_for(CommandKind::Move);
    assert_eq!(payloads.len(), 1);
    assert_eq!(payloads[0].len(), 56, "v5 Move::Request is 56 bytes");
    assert_eq!(&payloads[0][0..4], &0u32.to_le_bytes()); // JointImpedance
    assert_eq!(&payloads[0][4..8], &1u32.to_le_bytes()); // JointVelocity
}

/// Torque-only control on an FER: libfranka 0.9.2 runs a joint-velocity motion generator with
/// all-zero velocities next to the external controller (`src/robot.cpp:41-57`) and ends the
/// motion with `motion_generation_finished`, because FCI v5's `ControllerCommand` has no
/// `torque_command_finished` flag.
#[test]
fn torque_only_control_on_v5_sends_zero_joint_velocities_and_finishes_the_motion() {
    use crate::control_types::{motion_finished, Torques};
    use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;

    let server = MockServer::start_v5();
    let robot = server.connect(0);

    server.queue_response_for(CommandKind::Move, &[v5::MoveStatus::MotionStarted.to_u8()]);
    let running = server.moving_bytes(
        StateMotionGeneratorMode::JointVelocity,
        StateControllerMode::ExternalController,
    );
    server.stream_states_bytes(running, server.idle_bytes(), 100, 3000);
    server.schedule(300, |shared| {
        shared.respond_last(CommandKind::Move, &[v5::MoveStatus::Success.to_u8()])
    });

    let mut cycles = 0;
    crate::robot::control_loop::control_torques(
        &robot,
        &mut |_state: &crate::robot_state::RobotState, _period: crate::duration::Duration| {
            cycles += 1;
            let torques = Torques::new([0.25; 7]);
            if cycles >= 3 {
                motion_finished(torques)
            } else {
                torques
            }
        },
        false,
        MAX_CUTOFF_FREQUENCY,
    )
    .expect("torque control should finish regularly");

    // The motion was started as JointVelocity + ExternalController, not `kNone`.
    let payloads = server.payloads_for(CommandKind::Move);
    assert_eq!(payloads[0].len(), 56);
    assert_eq!(&payloads[0][0..4], &2u32.to_le_bytes()); // ExternalController
    assert_eq!(&payloads[0][4..8], &1u32.to_le_bytes()); // JointVelocity

    for datagram in server.command_datagrams() {
        assert_eq!(datagram.len(), 370, "a v5 RobotCommand is 370 bytes");
    }

    let commands = server.commands_v5();
    let commanded = commands
        .iter()
        .filter(|command| crate::wire::f64s_to_f64(&command.control.tau_J_d)[0] != 0.0)
        .collect::<Vec<_>>();
    assert!(!commanded.is_empty(), "no torque command reached the mock");
    for command in &commanded {
        assert_eq!(
            crate::wire::f64s_to_f64(&command.control.tau_J_d),
            [0.25; 7]
        );
        assert_eq!(
            crate::wire::f64s_to_f64(&command.motion.dq_c),
            [0.0; 7],
            "the joint velocity generator must command zero velocities"
        );
    }

    let last = commands.last().expect("at least one command");
    assert_eq!(
        last.motion.motion_generation_finished, 1,
        "a v5 motion ends with motion_generation_finished"
    );
}

/// Every setter goes out under the v5 numbering, where `SetCollisionBehavior` is 4 (it is 3 on
/// FCI v10).
#[test]
fn setters_use_the_v5_command_numbering() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    server.queue_response_id(4, &[v5::GetterSetterStatus::Success.to_u8()]);
    robot
        .set_collision_behavior(
            &[1.0; 7], &[2.0; 7], &[1.0; 7], &[2.0; 7], &[1.0; 6], &[2.0; 6], &[1.0; 6], &[2.0; 6],
        )
        .expect("set_collision_behavior should succeed");

    assert_eq!(
        server.request_count_id(4),
        1,
        "v5 SetCollisionBehavior is 4"
    );
    assert_eq!(
        server.request_count_id(3),
        0,
        "3 is GetCartesianLimit on v5"
    );
    assert_eq!(
        server.payloads_for(CommandKind::SetCollisionBehavior)[0].len(),
        416
    );
}

/// `Robot::setFilters` exists only on an FER.
#[test]
fn set_filters_is_a_v5_command() {
    // On FCI v10 it is rejected before anything is sent.
    {
        let server = MockServer::start();
        let robot = server.connect(0);
        match robot.set_filters(10.0, 10.0, 10.0, 10.0, 10.0) {
            Err(FrankaError::InvalidOperation(message)) => assert_eq!(
                message,
                "libfranka: Set Filters is not available on FCI version 10."
            ),
            other => panic!("expected InvalidOperation, got {other:?}"),
        }
    }

    // On FCI v5 it goes out as command 11 with a 40-byte payload.
    let server = MockServer::start_v5();
    let robot = server.connect(0);
    server.queue_response_id(11, &[v5::GetterSetterStatus::Success.to_u8()]);
    robot
        .set_filters(100.0, 100.0, 100.0, 100.0, 100.0)
        .expect("set_filters should succeed");
    assert_eq!(server.request_count_id(11), 1);
    let payload = &server.payloads_for(CommandKind::SetFilters)[0];
    assert_eq!(payload.len(), 40);
    assert_eq!(&payload[0..8], &100.0f64.to_le_bytes());
}

/// `Robot::getVirtualWall` parses the 154-byte `GetCartesianLimit::Response`, and exists only
/// on an FER.
#[test]
fn virtual_wall_parses_the_154_byte_response() {
    use zerocopy::little_endian::F64;

    {
        let server = MockServer::start();
        let robot = server.connect(0);
        match robot.virtual_wall(1) {
            Err(FrankaError::InvalidOperation(message)) => assert_eq!(
                message,
                "libfranka: Get Cartesian Limit is not available on FCI version 10."
            ),
            other => panic!("expected InvalidOperation, got {other:?}"),
        }
    }

    let server = MockServer::start_v5();
    let robot = server.connect(0);

    let mut object_frame = [F64::new(0.0); 16];
    for (index, value) in object_frame.iter_mut().enumerate() {
        *value = F64::new(index as f64);
    }
    let response = v5::GetCartesianLimitResponse {
        status: v5::GetterSetterStatus::Success.to_u8(),
        object_world_size: [F64::new(1.0), F64::new(2.0), F64::new(3.0)],
        object_frame,
        object_activation: 1,
    };
    assert_eq!(response.as_bytes().len(), 154);
    server.queue_response_id(3, response.as_bytes());

    let wall = robot.virtual_wall(7).expect("virtual_wall should succeed");
    assert_eq!(wall.id, 7);
    assert_eq!(wall.object_world_size, [1.0, 2.0, 3.0]);
    assert_eq!(wall.p_frame, std::array::from_fn(|i| i as f64));
    assert!(wall.active);

    // The request carries the id as a little-endian i32.
    let payload = &server.payloads_for(CommandKind::GetCartesianLimit)[0];
    assert_eq!(payload.len(), 4);
    assert_eq!(payload, &7i32.to_le_bytes());
}

/// A status byte that is not defined for the negotiated version is libfranka's
/// `ProtocolException`, naming the command exactly as `handleCommandResponse` does. Byte 3 is a
/// valid getter/setter status on FCI v10 (`kCommandRejectedDueToActivatedSafetyFunctions`) but
/// not on FCI v5.
#[test]
fn an_unknown_status_byte_is_a_protocol_error_naming_the_command() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    server.queue_response_id(5, &[3]);
    match robot.set_joint_impedance(&[3000.0; 7]) {
        Err(FrankaError::Protocol(message)) => assert_eq!(
            message,
            "libfranka: Unexpected response while handling Set Joint Impedance command!"
        ),
        other => panic!("expected a ProtocolException, got {other:?}"),
    }
}

/// An FER has no position-dependent joint velocity envelope: the limits are the flat
/// `kMaxJointVelocity` of libfranka 0.9.2 at every configuration.
#[test]
fn v5_joint_velocity_limits_are_the_flat_fer_constants() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    for q in [[0.0; 7], [1.0, -0.5, 0.3, -2.0, 0.1, 1.5, 0.7]] {
        assert_eq!(
            robot.upper_joint_velocity_limits(&q),
            crate::rate_limiting::fer::MAX_JOINT_VELOCITY
        );
        assert_eq!(
            robot.lower_joint_velocity_limits(&q),
            crate::rate_limiting::fer::MIN_JOINT_VELOCITY
        );
    }
}

/// A state of the *other* version is rejected with libfranka's size check rather than being
/// truncated into a plausible-looking state.
#[test]
fn a_state_of_the_wrong_version_is_an_incorrect_object_size_error() {
    let server = MockServer::start();
    let robot = server.connect(0);

    server.schedule(20, |shared| {
        shared.send_state_bytes(idle_state_v5().as_bytes())
    });
    match robot.read_once() {
        Err(FrankaError::Protocol(message)) => {
            assert_eq!(message, "libfranka: incorrect object size")
        }
        other => panic!("expected a ProtocolException, got {other:?}"),
    }
}

/// FCI v5 has no `initialized_filter_` (libfranka 0.9.2 `src/control_loop.cpp:188-205` always
/// references `robot_state.q_d`), so the *first* joint-position setpoint of a motion is rate
/// limited like every other one. The mock's states carry `q_d = 0`, so a command of 1 rad must
/// reach the wire as one integration step from zero — not as the raw jump an FR3 would accept.
#[test]
fn the_first_v5_joint_position_command_is_rate_limited_against_q_d() {
    use crate::control_types::{motion_finished, ControllerMode, JointPositions};
    use crate::duration::Duration;
    use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
    use crate::robot::control_loop::ControlLoop;
    use crate::robot_state::RobotState;

    const TARGET: [f64; 7] = [1.0; 7];

    let server = MockServer::start_v5();
    let robot = server.connect(0);

    server.queue_response_for(CommandKind::Move, &[v5::MoveStatus::MotionStarted.to_u8()]);
    let running = server.moving_bytes(
        StateMotionGeneratorMode::JointPosition,
        StateControllerMode::JointImpedance,
    );
    server.stream_states_bytes(running, server.idle_bytes(), 100, 3000);
    server.schedule(300, |shared| {
        shared.respond_last(CommandKind::Move, &[v5::MoveStatus::Success.to_u8()])
    });

    let mut cycles = 0;
    let mut motion_callback = |_state: &RobotState, _period: Duration| {
        cycles += 1;
        let output = JointPositions::new(TARGET);
        if cycles >= 3 {
            motion_finished(output)
        } else {
            output
        }
    };
    let control_loop = ControlLoop::<JointPositions>::new_with_motion(
        &robot,
        ControllerMode::JointImpedance,
        &mut motion_callback,
        true,
        MAX_CUTOFF_FREQUENCY,
    )
    .expect("motion should start");
    control_loop.run().expect("motion should finish");

    let commands = server.commands_v5();
    let first = commands.first().expect("no command reached the mock");
    let sent = crate::wire::f64s_to_f64(&first.motion.q_c);

    // Exactly what `franka::limitRate` yields for a step from rest, with the FER envelope.
    let expected = crate::rate_limiting::limit_rate_joint_positions(
        &crate::rate_limiting::fer::MAX_JOINT_VELOCITY,
        &crate::rate_limiting::fer::MIN_JOINT_VELOCITY,
        &crate::rate_limiting::fer::MAX_JOINT_ACCELERATION,
        &crate::rate_limiting::fer::MAX_JOINT_JERK,
        &TARGET,
        &[0.0; 7],
        &[0.0; 7],
        &[0.0; 7],
    )
    .unwrap();
    assert_eq!(sent, expected);
    assert!(
        sent[0] > 0.0 && sent[0] < 1e-3,
        "the first v5 setpoint must be one step from q_d, got {}",
        sent[0]
    );
    assert_ne!(sent, TARGET, "the raw jump must not reach the wire");
}
