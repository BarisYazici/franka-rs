//! `Robot::stop()` from a second thread, and the reflex/recovery path, against franka-sim.
//!
//! Run with
//! `FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-rs --test sim_stop_and_reflex`.

mod common;

use std::f64::consts::PI;
use std::sync::Arc;
use std::time::Instant;

use franka::{
    motion_finished, ControllerMode, Duration, FrankaError, JointPositions, MoveStatus, RobotMode,
    RobotState, Torques, DEFAULT_CUTOFF_FREQUENCY, MAX_CUTOFF_FREQUENCY,
};
use franka_sim_test::SimConfig;

/// Runs the cosine joint move of `sim_motions`' first test, from wherever the robot is.
fn cosine_joint_move(robot: &franka::Robot) -> franka::FrankaResult<()> {
    let start = robot.read_once()?.q;
    let mut time = 0.0;
    robot.control_joint_positions(
        |_state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            let delta = 0.1 * (1.0 - (PI / 3.0 * time).cos());
            let mut q = start;
            q[0] += delta;
            q[6] += delta;
            let output = JointPositions::new(q);
            if time >= 3.0 {
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )
}

/// 1. `stop()` on another thread preempts a running torque control loop.
#[test]
fn stop_from_another_thread_preempts_the_motion() {
    let sim = common::sim(SimConfig::nominal());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let control_robot = Arc::clone(&robot);
    let control = std::thread::spawn(move || {
        control_robot.control_torques(
            |_state: &RobotState, _period: Duration| Torques::new([0.0; 7]),
            true,
            DEFAULT_CUTOFF_FREQUENCY,
        )
    });

    std::thread::sleep(std::time::Duration::from_millis(500));
    let stopped_at = Instant::now();
    robot.stop().expect("stop() failed");

    let result = control.join().expect("the control thread panicked");
    let elapsed = stopped_at.elapsed();
    assert!(
        elapsed < std::time::Duration::from_secs(2),
        "the control loop took {elapsed:?} to return after stop()"
    );
    match result {
        Err(FrankaError::Control(exception)) => {
            assert_eq!(exception.message, "libfranka: Move command preempted!");
            assert_eq!(exception.move_status, Some(MoveStatus::Preempted));
        }
        other => panic!("expected a preempted ControlException, got {other:?}"),
    }

    assert_eq!(common::settled_state(&robot, 5).robot_mode, RobotMode::Idle);
    cosine_joint_move(&robot).expect("a new motion must start after stop()");
}

/// 2. A `stop()` with no motion running is accepted and leaves no stray reply behind.
#[test]
fn stop_without_a_motion_is_harmless() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);

    robot.stop().expect("first stop() failed");
    robot.stop().expect("second stop() failed");

    robot
        .set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])
        .expect("the next command must not see a leftover Move reply");
    assert_eq!(
        robot.read_once().expect("read_once").robot_mode,
        RobotMode::Idle
    );
}

/// 3. Driving joint 1 past its position-dependent velocity envelope trips a reflex, and
///    `automatic_error_recovery` clears it.
#[test]
fn velocity_violation_trips_a_reflex_and_recovery_clears_it() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;
    let mut time = 0.0;
    let mut commanded = start[0];

    // No rate limiting and no filtering: the client sends the raw ramp, which runs into the
    // robot's envelope near the joint limit.
    let result = robot.control_joint_positions(
        |_state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            let velocity = 2.0 * (time / 0.5).min(1.0);
            // Period-proportional, not a hardcoded 1 ms step: a lost cycle (`period` > 1 ms)
            // must still advance the ramp by the distance that velocity covers over the real
            // elapsed time, or the sim's backward-Euler acceleration check sees a step and
            // trips `joint_motion_generator_velocity_discontinuity` before the velocity
            // envelope this test means to provoke.
            commanded = (commanded + velocity * period.as_secs_f64()).min(3.0);
            let mut q = start;
            q[0] = commanded;
            let output = JointPositions::new(q);
            if time >= 5.0 {
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::JointImpedance,
        false,
        MAX_CUTOFF_FREQUENCY,
    );

    let exception = match result {
        Err(FrankaError::Control(exception)) => exception,
        other => panic!("expected a reflex ControlException, got {other:?}"),
    };
    assert_eq!(exception.move_status, Some(MoveStatus::ReflexAborted));
    assert!(
        exception.message.contains("motion aborted by reflex"),
        "unexpected message: {}",
        exception.message
    );
    let errors = exception.last_motion_errors;
    assert!(
        errors.get("joint_velocity_violation")
            || errors.get("joint_motion_generator_velocity_limits_violation"),
        "unexpected reflex reasons: {errors}"
    );
    for name in errors.names() {
        assert!(
            exception.message.contains(name),
            "the message does not name the error {name}: {}",
            exception.message
        );
    }

    let state = common::settled_state(&robot, 5);
    assert!(
        state.q[0] < 2.7437,
        "joint 1 went past its limit: {}",
        state.q[0]
    );
    assert_eq!(state.robot_mode, RobotMode::Reflex);

    // A new motion is rejected until the errors are cleared.
    match cosine_joint_move(&robot) {
        Err(FrankaError::Command(message)) => assert!(
            message.contains("command not possible in the current mode"),
            "unexpected rejection: {message}"
        ),
        other => panic!("expected a rejected Move, got {other:?}"),
    }

    robot
        .automatic_error_recovery()
        .expect("automatic_error_recovery failed");
    cosine_joint_move(&robot).expect("a motion must run after error recovery");
}

/// 4. With the rate limiter on, the same joint-limit approach stays inside the envelope.
#[test]
fn rate_limiting_keeps_the_motion_inside_the_envelope() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;
    let target = 2.7;
    let mut time = 0.0;

    let started = Instant::now();
    robot
        .control_joint_positions(
            |_state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                const HORIZON: f64 = 5.0;
                let progress = 0.5 * (1.0 - (PI * (time / HORIZON).min(1.0)).cos());
                let mut q = start;
                q[0] = start[0] + (target - start[0]) * progress;
                let output = JointPositions::new(q);
                if time >= HORIZON {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::JointImpedance,
            true,
            DEFAULT_CUTOFF_FREQUENCY,
        )
        .expect("the rate-limited motion must not trip a reflex");
    assert!(
        started.elapsed() < std::time::Duration::from_secs(6),
        "the motion took {:?}",
        started.elapsed()
    );

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    assert!(
        (state.q[0] - target).abs() < 0.05,
        "joint 1 ended at {} rad, expected {target}",
        state.q[0]
    );
}
