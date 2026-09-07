//! Torque-only control and the paths that must still cancel a running motion: a failed
//! `finish_motion` and a panicking control callback.

use super::*;

/// The state a torque-only motion produces: no motion generator, external controller.
fn external_controller_state() -> WireRobotState {
    moving_state(
        StateMotionGeneratorMode::None,
        StateControllerMode::ExternalController,
    )
}

/// Starts a torque-only motion (`Move` with `MotionGeneratorMode::None`).
fn start_torque_motion(server: &MockServer, robot: &RobotImpl) -> u32 {
    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.stream_states(external_controller_state(), 200, 3000);
    robot
        .start_motion(
            MoveControllerMode::ExternalController,
            MoveMotionGeneratorMode::None,
            deviation(),
            deviation(),
        )
        .expect("torque motion should start")
}

/// A failing `finish_motion` must leave the active control unfinished, so that dropping it
/// still sends exactly one `StopMove`.
///
/// `active_torque_control.cpp:61-64` sets `control_finished` only after `finishMotion` returns,
/// which is what makes `~ActiveControl` (`active_control.cpp:23-27`) cancel in this case.
#[test]
fn a_failed_finish_still_cancels_the_motion_on_drop() {
    use crate::control_types::{motion_finished, Torques};
    use crate::robot::active_control::ActiveTorqueControl;

    let server = MockServer::start();
    let robot = server.connect(0);
    let motion_id = start_torque_motion(&server, &robot);

    // The terminal Move reply is an abort, so finish_motion fails.
    server.schedule(400, |shared| {
        shared.respond_last(CommandKind::Move, &[MoveStatus::Aborted.to_u8()])
    });
    server.queue_response(Command::StopMove, &[StopMoveStatus::Success.to_u8()]);

    let control_lock = Mutex::new(());
    let guard = control_lock.lock().unwrap();
    let mut control = ActiveTorqueControl::new(&robot, motion_id, guard);
    for _ in 0..5 {
        control.read_once().expect("read_once failed");
        control
            .write_once(&Torques::new([0.0; 7]))
            .expect("write_once failed");
    }

    match control.write_once(&motion_finished(Torques::new([0.0; 7]))) {
        Err(FrankaError::Control(exception)) => {
            assert_eq!(exception.message, "libfranka: Move command aborted!")
        }
        other => panic!("expected the finish to fail, got {other:?}"),
    }
    assert_eq!(
        server.request_count(Command::StopMove),
        0,
        "write_once must not cancel by itself"
    );

    drop(control);
    assert_eq!(
        server.request_count(Command::StopMove),
        1,
        "dropping an unfinished active control must send exactly one StopMove"
    );
    assert!(!robot.controller_running());
}

/// A panicking user callback must cancel the motion, like the
/// `catch (...) { cancelMotion; throw; }` of `control_loop.cpp:143-149`.
#[test]
fn a_panicking_control_callback_cancels_the_motion() {
    use crate::control_types::Torques;
    use crate::lowpass_filter::MAX_CUTOFF_FREQUENCY;
    use std::panic::AssertUnwindSafe;

    let server = MockServer::start();
    let robot = server.connect(0);

    server.queue_response(Command::Move, &[MoveStatus::MotionStarted.to_u8()]);
    server.queue_response(Command::StopMove, &[StopMoveStatus::Success.to_u8()]);
    server.stream_states(external_controller_state(), 200, 3000);

    let mut cycles = 0;
    let previous_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let result = std::panic::catch_unwind(AssertUnwindSafe(|| {
        crate::robot::control_loop::control_torques(
            &robot,
            &mut |_state: &crate::robot_state::RobotState, _period: crate::duration::Duration| {
                cycles += 1;
                assert!(cycles < 3, "callback panic under test");
                Torques::new([0.0; 7])
            },
            true,
            MAX_CUTOFF_FREQUENCY,
        )
    }));
    std::panic::set_hook(previous_hook);

    assert!(result.is_err(), "the panic must propagate to the caller");
    assert_eq!(
        server.request_count(Command::StopMove),
        1,
        "a panicking callback must cancel the motion exactly once"
    );
    assert!(!robot.controller_running());
}
