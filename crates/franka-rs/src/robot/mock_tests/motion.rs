//! Ports of the motion-lifecycle cases of `test/robot_impl_tests.cpp`: starting, stopping,
//! cancelling and failing a motion under FCI v10.

use super::*;

// Port of RobotImplTests.CanReceiveReorderedRobotStatesCorrectly.
#[test]
fn can_receive_reordered_robot_states_correctly() {
    let server = MockServer::start();
    let robot = server.connect(0);

    server.shared.send_state_with_id(&idle_state(), 2);
    let state = robot.update_motion(None, None).unwrap();
    assert_eq!(state.time.as_millis(), 2);

    for id in [1u64, 4, 2, 3] {
        server.shared.send_state_with_id(&idle_state(), id);
    }
    std::thread::sleep(StdDuration::from_millis(50));
    let state = robot.update_motion(None, None).unwrap();
    assert_eq!(state.time.as_millis(), 4);
}

// Port of RobotImplTests.ThrowsTimeoutIfNoRobotStateArrives.
#[test]
fn throws_timeout_if_no_robot_state_arrives() {
    let server = MockServer::start();
    let robot = server.connect(0);

    match robot.update_motion(None, None) {
        Err(FrankaError::Network(message)) => {
            assert!(message.contains("Timeout"), "unexpected message {message}")
        }
        other => panic!("expected a network timeout, got {other:?}"),
    }
}

// Port of RobotImplTests.CanNotStartMultipleMotions.
#[test]
fn can_not_start_multiple_motions() {
    let server = MockServer::start();
    let robot = server.connect(0);

    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, |shared| {
        shared.send_state(&moving_state(
            StateMotionGeneratorMode::JointVelocity,
            StateControllerMode::JointImpedance,
        ))
    });

    start_joint_velocity_motion(&robot).expect("first motion should start");

    for (controller, motion) in [
        (
            MoveControllerMode::CartesianImpedance,
            MoveMotionGeneratorMode::JointPosition,
        ),
        (
            MoveControllerMode::ExternalController,
            MoveMotionGeneratorMode::JointVelocity,
        ),
    ] {
        match robot.start_motion(controller, motion, deviation(), deviation()) {
            Err(FrankaError::Control(exception)) => assert_eq!(
                exception.message,
                "libfranka robot: Attempted to start multiple motions!"
            ),
            other => panic!("expected a ControlException, got {other:?}"),
        }
    }
}

// Port of RobotImplTest.CanReceiveMotionRejected.
#[test]
fn can_receive_motion_rejected() {
    let server = MockServer::start();
    let robot = server.connect(0);

    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    // The rejection arrives while the state still reports an idle robot.
    server.schedule(30, |shared| {
        shared.respond_last(
            CommandKind::Move,
            &[MoveStatus::CommandNotPossibleRejected.to_u8()],
        )
    });
    // Keep the state stream alive so the start loop does not time out.
    server.schedule(10, |shared| {
        for _ in 0..200 {
            shared.send_state(&idle_state());
            std::thread::sleep(StdDuration::from_millis(2));
        }
    });

    match robot.start_motion(
        MoveControllerMode::CartesianImpedance,
        MoveMotionGeneratorMode::CartesianVelocity,
        deviation(),
        deviation(),
    ) {
        Err(FrankaError::Control(exception)) => assert_eq!(
            exception.message,
            "libfranka: Move command rejected: command not possible in the current mode \
             (\"Idle\")!"
        ),
        other => panic!("expected a ControlException, got {other:?}"),
    }
    assert!(!robot.motion_generator_running());
}

// Port of RobotImplTests.CanStopMotion.
#[test]
fn can_stop_motion() {
    let server = MockServer::start();
    let robot = server.connect(0);

    let running = moving_state(
        StateMotionGeneratorMode::CartesianVelocity,
        StateControllerMode::CartesianImpedance,
    );

    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    let motion_id = robot
        .start_motion(
            MoveControllerMode::CartesianImpedance,
            MoveMotionGeneratorMode::CartesianVelocity,
            deviation(),
            deviation(),
        )
        .expect("motion should start");
    assert!(robot.motion_generator_running());

    let motion_command = MotionGeneratorCommand::default();
    server.schedule(20, move |shared| shared.send_state(&running));
    robot.update_motion(Some(&motion_command), None).unwrap();

    // The state goes idle, then the Move reply arrives.
    server.schedule(20, |shared| {
        let mut idle = idle_state();
        idle.controller_mode = StateControllerMode::CartesianImpedance.to_u8();
        shared.send_state(&idle);
    });
    server.schedule(60, |shared| {
        shared.respond_last(CommandKind::Move, &[MoveStatus::Success.to_u8()])
    });

    robot
        .finish_motion(motion_id, Some(&motion_command), None)
        .expect("motion should finish");
    assert!(!robot.motion_generator_running());

    let finished = server
        .commands()
        .iter()
        .any(|command| command.motion.motion_generation_finished == 1);
    assert!(finished, "the last command must carry the finished flag");
}

/// The *terminal* `Move` reply must be `kSuccess`: a second `kMotionStarted` is
/// `ProtocolException("libfranka: Move received unexpected motion started message.")`, the
/// message `handleCommandResponse<Move>` (`robot_impl.h:388-395`) uses for that status while a
/// motion is running. This is what makes `Ok(())` out of `finish_motion` -- and therefore out
/// of every `Robot::control_*` -- mean "the motion ended with `kSuccess`".
#[test]
fn a_second_motion_started_reply_finishing_a_motion_is_a_protocol_error() {
    let server = MockServer::start();
    let robot = server.connect(0);

    let running = moving_state(
        StateMotionGeneratorMode::CartesianVelocity,
        StateControllerMode::CartesianImpedance,
    );

    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    let motion_id = robot
        .start_motion(
            MoveControllerMode::CartesianImpedance,
            MoveMotionGeneratorMode::CartesianVelocity,
            deviation(),
            deviation(),
        )
        .expect("motion should start");

    let motion_command = MotionGeneratorCommand::default();
    // The state goes idle, then a *second* MotionStarted arrives as the terminal reply.
    server.schedule(20, |shared| {
        let mut idle = idle_state();
        idle.controller_mode = StateControllerMode::CartesianImpedance.to_u8();
        shared.send_state(&idle);
    });
    server.schedule(60, |shared| {
        shared.respond_last(CommandKind::Move, &[MoveStatus::MotionStarted.to_u8()])
    });

    match robot.finish_motion(motion_id, Some(&motion_command), None) {
        Err(FrankaError::Protocol(message)) => assert_eq!(
            message,
            "libfranka: Move received unexpected motion started message."
        ),
        other => panic!("expected a ProtocolException, got {other:?}"),
    }
}

// Port of RobotImplTests.StopMotionErrorThrowsControlException.
#[test]
fn stop_motion_error_throws_control_exception() {
    let server = MockServer::start();
    let robot = server.connect(0);

    let running = moving_state(
        StateMotionGeneratorMode::CartesianVelocity,
        StateControllerMode::CartesianImpedance,
    );
    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    let motion_id = robot
        .start_motion(
            MoveControllerMode::CartesianImpedance,
            MoveMotionGeneratorMode::CartesianVelocity,
            deviation(),
            deviation(),
        )
        .unwrap();

    server.schedule(20, |shared| {
        let mut idle = idle_state();
        idle.controller_mode = StateControllerMode::CartesianImpedance.to_u8();
        shared.send_state(&idle);
    });
    server.schedule(60, |shared| {
        shared.respond_last(CommandKind::Move, &[MoveStatus::EmergencyAborted.to_u8()])
    });

    let motion_command = MotionGeneratorCommand::default();
    match robot.finish_motion(motion_id, Some(&motion_command), None) {
        Err(FrankaError::Control(exception)) => assert_eq!(
            exception.message,
            "libfranka: Move command aborted: User Stop pressed!"
        ),
        other => panic!("expected a ControlException, got {other:?}"),
    }
    assert!(!robot.motion_generator_running());
}

// Port of RobotImplTests.CanCancelMotion.
#[test]
fn can_cancel_motion() {
    let server = MockServer::start();
    let robot = server.connect(0);

    let running = moving_state(
        StateMotionGeneratorMode::CartesianVelocity,
        StateControllerMode::CartesianImpedance,
    );
    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    let motion_id = robot
        .start_motion(
            MoveControllerMode::CartesianImpedance,
            MoveMotionGeneratorMode::CartesianVelocity,
            deviation(),
            deviation(),
        )
        .unwrap();
    assert!(robot.motion_generator_running());

    server.queue_response(Command::StopMove, &[StopMoveStatus::Success.to_u8()]);
    server.schedule(30, |shared| {
        let mut idle = idle_state();
        idle.controller_mode = StateControllerMode::CartesianImpedance.to_u8();
        shared.send_state(&idle);
        shared.respond_last(CommandKind::Move, &[MoveStatus::Preempted.to_u8()]);
    });

    robot
        .cancel_motion(motion_id)
        .expect("cancel should succeed");
    assert!(!robot.motion_generator_running());
    assert!(!robot.controller_running());
}

// Port of RobotImplTest.CancelMotionErrorThrowsControlException.
#[test]
fn cancel_motion_error_throws_control_exception() {
    let server = MockServer::start();
    let robot = server.connect(0);

    let running = moving_state(
        StateMotionGeneratorMode::CartesianVelocity,
        StateControllerMode::CartesianImpedance,
    );
    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    let motion_id = robot
        .start_motion(
            MoveControllerMode::CartesianImpedance,
            MoveMotionGeneratorMode::CartesianVelocity,
            deviation(),
            deviation(),
        )
        .unwrap();

    server.queue_response(
        Command::StopMove,
        &[StopMoveStatus::CommandNotPossibleRejected.to_u8()],
    );
    match robot.cancel_motion(motion_id) {
        Err(FrankaError::Control(exception)) => assert_eq!(
            exception.message,
            "libfranka: Stop Move command rejected: command not possible in the current mode \
             (\"Move\")!"
        ),
        other => panic!("expected a ControlException, got {other:?}"),
    }
}

// Port of RobotImplTests.ThrowsDuringMotionIfErrorReceived.
#[test]
fn throws_during_motion_if_error_received() {
    let server = MockServer::start();
    let robot = server.connect(50);

    let running = moving_state(
        StateMotionGeneratorMode::JointPosition,
        StateControllerMode::JointImpedance,
    );
    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    let motion_id = robot
        .start_motion(
            MoveControllerMode::JointImpedance,
            MoveMotionGeneratorMode::JointPosition,
            deviation(),
            deviation(),
        )
        .unwrap();
    assert!(robot.motion_generator_running());

    server.schedule(20, |shared| {
        let mut reflex = idle_state();
        reflex.reflex_reason[0] = 1;
        reflex.robot_mode = WireRobotMode::Reflex.to_u8();
        shared.send_state(&reflex);
    });
    server.schedule(60, |shared| {
        shared.respond_last(CommandKind::Move, &[MoveStatus::ReflexAborted.to_u8()])
    });

    let motion_command = MotionGeneratorCommand::default();
    let robot_state = robot.update_motion(Some(&motion_command), None).unwrap();
    match robot.throw_on_motion_error(&robot_state, motion_id) {
        Err(FrankaError::Control(exception)) => {
            assert!(
                exception
                    .message
                    .starts_with("libfranka: Move command aborted: motion aborted by reflex!"),
                "unexpected message {}",
                exception.message
            );
            assert!(exception
                .message
                .contains("joint_position_limits_violation"));
            assert_eq!(exception.move_status, Some(MoveStatus::ReflexAborted));
            assert!(exception
                .last_motion_errors
                .get("joint_position_limits_violation"));
            assert!(!exception.log.is_empty());
        }
        other => panic!("expected a ControlException, got {other:?}"),
    }
    assert!(!robot.motion_generator_running());
}

// The three preconditions of `Robot::Impl::sendRobotCommand`.
#[test]
fn send_robot_command_preconditions() {
    let server = MockServer::start();
    let robot = server.connect(0);

    let motion_command = MotionGeneratorCommand::default();
    let control_command = crate::wire::robot::ControllerCommand::default();

    // No motion generator running.
    match robot.write_once_motion(&motion_command) {
        Err(FrankaError::Control(exception)) => assert_eq!(
            exception.message,
            "libfranka robot: Trying to send motion command, but no motion generator running!"
        ),
        other => panic!("expected a ControlException, got {other:?}"),
    }
    // No controller running.
    match robot.write_once_control(&control_command) {
        Err(FrankaError::Control(exception)) => assert_eq!(
            exception.message,
            "libfranka robot: Trying to send control command, but no controller running!"
        ),
        other => panic!("expected a ControlException, got {other:?}"),
    }

    // Partial command: an external controller motion needs both halves.
    let running = moving_state(
        StateMotionGeneratorMode::JointVelocity,
        StateControllerMode::ExternalController,
    );
    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.schedule(20, move |shared| shared.send_state(&running));
    robot
        .start_motion(
            MoveControllerMode::ExternalController,
            MoveMotionGeneratorMode::JointVelocity,
            deviation(),
            deviation(),
        )
        .unwrap();

    match robot.write_once_motion(&motion_command) {
        Err(FrankaError::Control(exception)) => assert_eq!(
            exception.message,
            "libfranka robot: Trying to send partial robot command!"
        ),
        other => panic!("expected a ControlException, got {other:?}"),
    }
}
