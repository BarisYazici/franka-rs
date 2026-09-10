//! Motion generators, torque control and `ActiveControl` against the franka-sim simulator.
//!
//! Run with `FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-rs --test sim_motions`.
//!
//! The motion profiles are ports of libfranka's `examples/generate_*_motion.cpp`, shortened so
//! that a test finishes in a few seconds. Every test starts its own container, because a motion
//! leaves the simulated robot somewhere else than it found it.

mod common;

use std::f64::consts::PI;

use franka::{
    motion_finished, CartesianPose, CartesianVelocities, ControllerMode, Duration, JointPositions,
    JointVelocities, MoveControllerMode, RobotMode, RobotState, Torques, DEFAULT_CUTOFF_FREQUENCY,
};
use franka_sim_test::SimConfig;

/// The rate limiter is enabled for every motion here. libfranka 0.21 defaults `limit_rate` to
/// `false` (0.9 defaulted to `true`); these tests pass `true` deliberately, so that the
/// client-side limiter is the one exercised against the simulator.
const LIMIT_RATE: bool = true;
const CUTOFF: f64 = DEFAULT_CUTOFF_FREQUENCY;

/// 1. Joint position: a cosine ramp of +0.2 rad on joints 1 and 7 over 3 s.
///
/// Port of `examples/generate_joint_position_motion.cpp` with a shorter horizon.
#[test]
fn joint_position_motion_reaches_the_target() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;
    let mut target = start;
    target[0] += 0.2;
    target[6] += 0.2;

    let mut time = 0.0;
    robot
        .control_joint_positions(
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
            LIMIT_RATE,
            CUTOFF,
        )
        // This `Ok(())` *is* the terminal Move status assertion. `finish_motion`
        // claims the terminal `Move` reply and passes it through `handle_terminal_move_response`
        // (`robot/commands.rs`), which maps every status other than `MoveStatus::Success` to an
        // error -- including a second `MotionStarted`, unlike libfranka. So `Ok(())` from a
        // joint position motion means it ended with `MoveStatus::Success`, by construction.
        .expect("joint position motion failed");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    for (index, (actual, expected)) in state.q.iter().zip(target.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 0.03,
            "q[{index}] = {actual}, expected {expected}"
        );
    }
}

/// 2. Joint velocity: the two-cycle cosine of `generate_joint_velocity_motion.cpp` over 4 s.
///
/// The joints move out and come back, so the net displacement is small while the excursion is
/// not; both are checked, together with the direction of the excursion.
#[test]
fn joint_velocity_motion_moves_out_and_back() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;
    let mut extreme = start;
    let mut time = 0.0;

    robot
        .control_joint_velocities(
            |state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                for index in 0..7 {
                    if (state.q[index] - start[index]).abs() > (extreme[index] - start[index]).abs()
                    {
                        extreme[index] = state.q[index];
                    }
                }
                let output = JointVelocities::new(velocity_profile(time));
                if time >= 4.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::JointImpedance,
            LIMIT_RATE,
            CUTOFF,
        )
        // This `Ok(())` *is* the terminal Move status assertion. `finish_motion`
        // claims the terminal `Move` reply and passes it through `handle_terminal_move_response`
        // (`robot/commands.rs`), which maps every status other than `MoveStatus::Success` to an
        // error -- including a second `MotionStarted`, unlike libfranka. So `Ok(())` from a
        // joint velocity motion means the motion ended with `MoveStatus::Success`, by construction.
        .expect("joint velocity motion failed");

    let state = common::settled_state(&robot, 5);
    // Joints 4 and 7 (indices 3 and 6) carry the profile.
    for index in [3usize, 6] {
        let excursion = extreme[index] - start[index];
        assert!(
            excursion.abs() > 0.05,
            "joint {} barely moved: {excursion} rad",
            index + 1
        );
        assert!(
            excursion > 0.0,
            "joint {} moved in the wrong direction: {excursion} rad",
            index + 1
        );
        assert!(
            (state.q[index] - start[index]).abs() < 0.05,
            "joint {} did not return: {} rad off",
            index + 1,
            state.q[index] - start[index]
        );
    }
}

/// The velocity profile of `examples/generate_joint_velocity_motion.cpp` with
/// `time_max = 2.0` and `omega_max = 0.5`, so one full out-and-back cycle takes 4 s.
fn velocity_profile(time: f64) -> [f64; 7] {
    let time_max = 2.0;
    let omega_max = 0.5;
    let cycle = (-1.0f64)
        .powf(((time - time % time_max) / time_max).floor())
        .floor();
    let omega = cycle * omega_max / 2.0 * (1.0 - (2.0 * PI / time_max * time).cos());
    [0.0, 0.0, 0.0, omega, omega, omega, omega]
}

/// 3. Cartesian pose: the circle of `generate_cartesian_pose_motion.cpp`, radius 0.1 m over 6 s.
#[test]
fn cartesian_pose_motion_returns_to_the_start() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let initial_pose = robot.read_once().expect("read_once").O_T_EE;
    let mut time = 0.0;

    robot
        .control_cartesian_pose(
            |_state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                const RADIUS: f64 = 0.1;
                let angle = PI / 4.0 * (1.0 - (PI / 3.0 * time).cos());
                let mut pose = initial_pose;
                pose[12] += RADIUS * angle.sin();
                pose[14] += RADIUS * (angle.cos() - 1.0);
                let output = CartesianPose::new(pose);
                if time >= 6.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::CartesianImpedance,
            LIMIT_RATE,
            CUTOFF,
        )
        // This `Ok(())` *is* the terminal Move status assertion. `finish_motion`
        // claims the terminal `Move` reply and passes it through `handle_terminal_move_response`
        // (`robot/commands.rs`), which maps every status other than `MoveStatus::Success` to an
        // error -- including a second `MotionStarted`, unlike libfranka. So `Ok(())` from a
        // Cartesian pose motion means the motion ended with `MoveStatus::Success`, by construction.
        .expect("Cartesian pose motion failed");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    let start = common::translation(&initial_pose);
    let end = common::translation(&state.O_T_EE);
    let distance =
        ((end[0] - start[0]).powi(2) + (end[1] - start[1]).powi(2) + (end[2] - start[2]).powi(2))
            .sqrt();
    assert!(
        distance < 0.025,
        "the end effector ended {distance} m away from the start ({start:?} -> {end:?})"
    );
}

/// 4. Cartesian velocity: the profile of `generate_cartesian_velocity_motion.cpp` over 4 s.
#[test]
fn cartesian_velocity_motion_runs() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let mut time = 0.0;
    robot
        .control_cartesian_velocities(
            |_state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                let time_max = 2.0;
                let v_max = 0.05;
                let angle = PI / 4.0;
                let cycle = (-1.0f64)
                    .powf(((time - time % time_max) / time_max).floor())
                    .floor();
                let velocity = cycle * v_max / 2.0 * (1.0 - (2.0 * PI / time_max * time).cos());
                let output = CartesianVelocities::new([
                    angle.cos() * velocity,
                    0.0,
                    -angle.sin() * velocity,
                    0.0,
                    0.0,
                    0.0,
                ]);
                if time >= 2.0 * time_max {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::CartesianImpedance,
            LIMIT_RATE,
            CUTOFF,
        )
        // This `Ok(())` *is* the terminal Move status assertion. `finish_motion`
        // claims the terminal `Move` reply and passes it through `handle_terminal_move_response`
        // (`robot/commands.rs`), which maps every status other than `MoveStatus::Success` to an
        // error -- including a second `MotionStarted`, unlike libfranka. So `Ok(())` from a
        // Cartesian velocity motion means it ended with `MoveStatus::Success`, by construction.
        .expect("Cartesian velocity motion failed");

    assert_eq!(common::settled_state(&robot, 5).robot_mode, RobotMode::Idle);
}

/// 5. Torque control: zero commanded torque for 1 s must not make the robot fall over, i.e.
///    the simulator gravity-compensates an external controller.
#[test]
fn zero_torque_control_holds_the_robot() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;
    let mut time = 0.0;
    let mut worst_tau_d = 0.0f64;

    robot
        .control_torques(
            |state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                for tau in state.tau_J_d {
                    worst_tau_d = worst_tau_d.max(tau.abs());
                }
                let output = Torques::new([0.0; 7]);
                if time >= 1.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            LIMIT_RATE,
            CUTOFF,
        )
        // This `Ok(())` *is* the terminal Move status assertion. `finish_motion`
        // claims the terminal `Move` reply and passes it through `handle_terminal_move_response`
        // (`robot/commands.rs`), which maps every status other than `MoveStatus::Success` to an
        // error -- including a second `MotionStarted`, unlike libfranka. So `Ok(())` from a
        // torque motion means the motion ended with `MoveStatus::Success`, by construction.
        .expect("torque control failed");

    let state = common::settled_state(&robot, 5);
    for (index, (actual, expected)) in state.q.iter().zip(start.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 0.1,
            "joint {} drifted by {} rad under zero torque",
            index + 1,
            actual - expected
        );
    }
    // The commanded torque the simulator echoes stays at the commanded zero: gravity is
    // compensated by the simulator, not by the client.
    assert!(
        worst_tau_d < 1e-6,
        "tau_J_d was not the commanded zero (max |tau_J_d| = {worst_tau_d} Nm)"
    );
}

/// 6. Torque control together with a joint position motion generator, holding the start pose.
#[test]
fn torque_and_joint_position_control_holds_the_pose() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;
    let mut time = 0.0;

    robot
        .control_torques_and_joint_positions(
            |_state: &RobotState, _period: Duration| Torques::new([0.0; 7]),
            |_state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                let output = JointPositions::new(start);
                if time >= 1.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            LIMIT_RATE,
            CUTOFF,
        )
        // This `Ok(())` *is* the terminal Move status assertion. `finish_motion`
        // claims the terminal `Move` reply and passes it through `handle_terminal_move_response`
        // (`robot/commands.rs`), which maps every status other than `MoveStatus::Success` to an
        // error -- including a second `MotionStarted`, unlike libfranka. So `Ok(())` from a
        // torque + joint position motion means it ended with `MoveStatus::Success`.
        .expect("torque + joint position control failed");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    for (index, (actual, expected)) in state.q.iter().zip(start.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 0.1,
            "joint {} drifted by {} rad",
            index + 1,
            actual - expected
        );
    }
}

/// 7. `ActiveControl`: an externally driven joint velocity motion, then torque control.
#[test]
fn active_control_joint_velocity_then_torques() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let start = robot.read_once().expect("read_once").q;

    {
        let mut control = robot
            .start_joint_velocity_control(MoveControllerMode::JointImpedance)
            .expect("start_joint_velocity_control failed");
        let mut velocities = [0.0; 7];
        for cycle in 0..1000 {
            control.read_once().expect("read_once failed");
            // Ramp up over the first 100 cycles: ActiveControl applies no rate limiting.
            velocities[0] = 0.1 * (cycle as f64 / 100.0).min(1.0);
            control
                .write_once(&JointVelocities::new(velocities), None)
                .expect("write_once failed");
        }
        control
            .write_once(&motion_finished(JointVelocities::new([0.0; 7])), None)
            .expect("final write_once failed");
    }

    let after_velocity = common::settled_state(&robot, 5);
    let moved = after_velocity.q[0] - start[0];
    assert!(
        (moved - 0.095).abs() < 0.03,
        "joint 1 moved {moved} rad, expected about 0.095 rad"
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

/// 8. The control loop runs at 1 kHz and (nearly) every command is accepted.
#[test]
fn control_loop_timing_and_success_rate() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let mut periods: Vec<u64> = Vec::with_capacity(5000);
    let mut success_rate = 0.0;
    let mut time = 0.0;

    robot
        .control_joint_velocities(
            |state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                periods.push(period.as_millis());
                success_rate = state.control_command_success_rate;
                let output = JointVelocities::new(velocity_profile(time));
                if time >= 4.0 {
                    motion_finished(output)
                } else {
                    output
                }
            },
            ControllerMode::JointImpedance,
            LIMIT_RATE,
            CUTOFF,
        )
        .expect("joint velocity motion failed");

    // The first cycle always reports a zero period (libfranka's `ControlLoop::loop`).
    assert!(periods.len() > 3000, "only {} cycles", periods.len());
    let one_millisecond = periods.iter().skip(1).filter(|&&p| p == 1).count();
    let ratio = one_millisecond as f64 / (periods.len() - 1) as f64;
    let worst = periods.iter().copied().max().unwrap_or(0);
    assert!(
        ratio >= 0.95,
        "only {:.1} % of the cycles were exactly 1 ms (worst {worst} ms)",
        ratio * 100.0
    );
    assert!(worst <= 20, "a cycle took {worst} ms");
    assert!(
        success_rate >= 0.9,
        "control_command_success_rate was {success_rate}"
    );
}

/// 9. Dropping an unfinished `ActiveMotionGenerator` cancels the motion.
#[test]
fn dropping_an_active_motion_generator_cancels_it() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    {
        let mut control = robot
            .start_joint_velocity_control(MoveControllerMode::JointImpedance)
            .expect("start_joint_velocity_control failed");
        for _ in 0..100 {
            control.read_once().expect("read_once failed");
            control
                .write_once(
                    &JointVelocities::new([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
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
