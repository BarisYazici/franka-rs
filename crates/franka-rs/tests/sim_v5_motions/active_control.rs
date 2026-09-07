//! The `ActiveControl` half of the FCI v5 motion suite: an externally driven motion
//! generator, external torque control, and the cancel-on-drop guarantees of both handles.

use std::f64::consts::PI;

use franka::{
    motion_finished, ControllerMode, Duration, JointPositions, JointVelocities, MoveControllerMode,
    RobotMode, RobotState, Torques,
};
use franka_sim_test::SimConfig;

use crate::common;
use crate::{fer, CUTOFF, DRIVEN_JOINT, LIMIT_RATE};

/// 7. `ActiveControl`: an externally driven joint velocity motion, then torque control.
///
/// On v5 `start_torque_control` starts a joint-velocity motion generator (see test 5), so
/// `ActiveTorqueControl::write_once` adds the zero `dq_c` itself; the public API is the same
/// as on v10.
///
/// The displacement is measured on **joint 4**, where the FR3 version of this test
/// (`sim_motions.rs`) uses joint 1. Joint 1 of the simulator's FER model does not follow a
/// joint-velocity command at all: it limit-cycles at the 500 Hz Nyquist frequency with a
/// +-0.36 rad/s amplitude and nets about -0.002 rad instead of the commanded +0.095 rad. That
/// is the *simulator's* velocity servo (`tau = KV * (dq_c - dq)`, clipped to the joint force
/// limits, which averages to zero once it limit-cycles), not the client: the same 0.1 rad/s
/// for 1 s through the ordinary `control_joint_velocities` control loop, which sends the very
/// same `dq_c`, was measured as
///
/// ```text
/// joint 1: -0.00189 rad     joint 2: 0.09880 rad
/// joint 4:  0.09909 rad     joint 7: 0.09924 rad
/// ```
///
/// Joint 1 does track *position* commands (test 1 moves it 0.2 rad), because the simulator's
/// position law has a stiffness term that survives the clipping. See `docs/book/src/fer.md`.
#[test]
fn active_control_joint_velocity_then_torques() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

    let start = robot.read_once().expect("read_once").q;

    {
        let mut control = robot
            .start_joint_velocity_control(MoveControllerMode::JointImpedance)
            .expect("start_joint_velocity_control failed");
        let mut velocities = [0.0; 7];
        for cycle in 0..1000 {
            control.read_once().expect("read_once failed");
            // Ramp up over the first 100 cycles: ActiveControl applies no rate limiting.
            velocities[DRIVEN_JOINT] = 0.1 * (cycle as f64 / 100.0).min(1.0);
            control
                .write_once(&JointVelocities::new(velocities), None)
                .expect("write_once failed");
        }
        control
            .write_once(&motion_finished(JointVelocities::new([0.0; 7])), None)
            .expect("final write_once failed");
    }

    let after_velocity = common::settled_state(&robot, 5);
    let moved = after_velocity.q[DRIVEN_JOINT] - start[DRIVEN_JOINT];
    assert!(
        (moved - 0.095).abs() < 0.03,
        "joint {} moved {moved} rad, expected about 0.095 rad",
        DRIVEN_JOINT + 1
    );

    // Pin the simulator gap that forced `DRIVEN_JOINT` off joint 1, so that a future image
    // where joint 1 tracks announces itself instead of leaving this test quietly measuring the
    // wrong joint for ever. Same command, same code path, one second.
    let before_joint1 = common::settled_state(&robot, 5).q;
    {
        let mut control = robot
            .start_joint_velocity_control(MoveControllerMode::JointImpedance)
            .expect("start_joint_velocity_control failed");
        let mut velocities = [0.0; 7];
        for cycle in 0..1000 {
            control.read_once().expect("read_once failed");
            velocities[0] = 0.1 * (cycle as f64 / 100.0).min(1.0);
            control
                .write_once(&JointVelocities::new(velocities), None)
                .expect("write_once failed");
        }
        control
            .write_once(&motion_finished(JointVelocities::new([0.0; 7])), None)
            .expect("final write_once failed");
    }
    let joint1_moved = common::settled_state(&robot, 5).q[0] - before_joint1[0];
    eprintln!("fer sim: joint 1 moved {joint1_moved} rad under the same 0.1 rad/s command");
    assert!(
        joint1_moved.abs() < 0.05,
        "joint 1 moved {joint1_moved} rad, so franka-sim's FER model now follows a \
         joint-velocity command on it: set DRIVEN_JOINT = 0 and delete this check, which \
         exists only to pin the simulator gap described above"
    );

    {
        let mut control = robot
            .start_torque_control()
            .expect("start_torque_control failed");
        for _ in 0..500 {
            control.read_once().expect("read_once failed");
            control
                .write_once(&Torques::new([0.0; 7]))
                .expect("write_once failed");
        }
        control
            .write_once(&motion_finished(Torques::new([0.0; 7])))
            .expect("final write_once failed");
    }

    assert_eq!(common::settled_state(&robot, 5).robot_mode, RobotMode::Idle);
}

/// 9. Dropping an unfinished `ActiveMotionGenerator` cancels the motion.
#[test]
fn dropping_an_active_motion_generator_cancels_it() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

    {
        let mut control = robot
            .start_joint_velocity_control(MoveControllerMode::JointImpedance)
            .expect("start_joint_velocity_control failed");
        for _ in 0..100 {
            control.read_once().expect("read_once failed");
            control
                .write_once(
                    &JointVelocities::new([0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0]),
                    None,
                )
                .expect("write_once failed");
        }
        // Dropped without `motion_finished`.
    }

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);

    // A new motion starts right away.
    let start = state.q;
    let mut time = 0.0;
    robot
        .control_joint_positions(
            |_state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                let mut q = start;
                q[0] += 0.05 * (1.0 - (PI * time).cos()) / 2.0;
                let output = JointPositions::new(q);
                if time >= 1.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::JointImpedance,
            LIMIT_RATE,
            CUTOFF,
        )
        .expect("a new motion must start after the cancelled one");
}

/// 10. Dropping an unfinished `ActiveTorqueControl` cancels the v5 joint-velocity generator
///     it started, and leaves the robot able to move again.
#[test]
fn dropping_an_active_torque_control_cancels_it() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

    {
        let mut control = robot
            .start_torque_control()
            .expect("start_torque_control failed");
        for _ in 0..100 {
            control.read_once().expect("read_once failed");
            control
                .write_once(&Torques::new([0.0; 7]))
                .expect("write_once failed");
        }
        // Dropped without `motion_finished`: on v5 this must cancel the joint-velocity motion
        // generator `start_torque_control` opened under the hood.
    }

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);

    let start = state.q;
    let mut time = 0.0;
    robot
        .control_joint_positions(
            |_state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                let mut q = start;
                q[0] += 0.05 * (1.0 - (PI * time).cos()) / 2.0;
                let output = JointPositions::new(q);
                if time >= 1.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::JointImpedance,
            LIMIT_RATE,
            CUTOFF,
        )
        .expect("a new motion must start after the cancelled torque control");
}
