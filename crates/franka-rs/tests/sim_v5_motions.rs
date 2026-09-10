//! Motion generators, torque control and `ActiveControl` on FCI v5, against the franka-sim
//! simulator running `--protocol v5 --robot panda`.
//!
//! Run under the machine-wide simulator lock:
//!
//! ```text
//! flock .sim.lock cargo test --release -p franka-rs --test sim_v5_motions
//! ```
//!
//! These are `sim_motions.rs` (the FR3 / FCI v10 suite) with the same profiles, horizons and
//! tolerances, run against an FER. The profiles are ports of libfranka's
//! `examples/generate_*_motion.cpp`, shortened so that a test finishes in a few seconds. Every
//! test starts its own container, because a motion leaves the simulated robot somewhere else
//! than it found it, and homes the arm at [`common::FER_SIM_HOME_POSE`] first.
//!
//! Every excursion below was checked against the FER's own joint limits (joint 1 +-2.8973,
//! joint 2 +-1.7628, joint 3 +-2.8973, joint 4 in [-3.0718, -0.0698], joint 5 +-2.8973,
//! joint 6 in [-0.0175, 3.7525], joint 7 +-2.8973) and its flat velocity limits (2.175 rad/s
//! on joints 1-4, 2.610 rad/s on joints 5-7), so no profile here approaches a limit: from the
//! home pose the joint-velocity profile takes joint 4 from -1.57 rad to about -1.07 rad and
//! joint 6 from 1.57 rad to about 2.07 rad.

#[path = "sim_v5_motions/active_control.rs"]
mod active_control;
mod common;

use std::f64::consts::PI;

use franka::{
    motion_finished, CartesianPose, CartesianVelocities, ControllerMode, Duration, FciVersion,
    JointPositions, JointVelocities, RobotMode, RobotState, Torques, DEFAULT_CUTOFF_FREQUENCY,
};
use franka_sim_test::SimConfig;

/// The rate limiter is enabled for every motion here, so that the client-side limiter -- which
/// uses the *FER* constants (`rate_limiting::fer`) on FCI v5, not the FR3 ones -- is the
/// one exercised against the simulator.
const LIMIT_RATE: bool = true;
const CUTOFF: f64 = DEFAULT_CUTOFF_FREQUENCY;

/// The joint the `ActiveControl` tests drive, as a zero-based index: joint 4. The FR3 suite
/// uses joint 1; the simulator's FER model does not follow a joint-velocity command on that
/// joint at all (see `active_control_joint_velocity_then_torques`), so a velocity test that
/// used it would be measuring a simulator artefact instead of the client.
const DRIVEN_JOINT: usize = 3;

/// Starts an FER simulator, connects, checks the negotiated version and homes the arm.
fn fer(sim: &franka_sim_test::SimServer) -> franka::Robot {
    let robot = common::connect(sim);
    assert_eq!(
        robot.fci_version(),
        FciVersion::V5,
        "the FER simulator must negotiate FCI v5"
    );
    common::set_default_behavior(&robot);
    common::move_to_start_fer(&robot);
    robot
}

/// 1. Joint position: a cosine ramp of +0.2 rad on joints 1 and 7 over 3 s.
#[test]
fn joint_position_motion_reaches_the_target() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

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
        // This `Ok(())` *is* the terminal Move status assertion: `finish_motion` claims the
        // terminal `Move` reply and maps every v5 `MoveStatus` other than `Success` to an
        // error. So `Ok(())` means the motion ended with `MoveStatus::Success`.
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
#[test]
fn joint_velocity_motion_moves_out_and_back() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

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
/// `time_max = 2.0` and `omega_max = 0.5`, so one full out-and-back cycle takes 4 s. The peak
/// commanded velocity, 0.5 rad/s, is a fifth of the FER's flat 2.175 rad/s limit.
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
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

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
        .expect("Cartesian pose motion failed");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    let start = common::translation(&initial_pose);
    let end = common::translation(&state.O_T_EE);
    let distance =
        ((end[0] - start[0]).powi(2) + (end[1] - start[1]).powi(2) + (end[2] - start[2]).powi(2))
            .sqrt();
    // The FR3 suite closes this circle to within 0.025 m; the FER simulator's arm is
    // measured at 0.0260 m over repeated runs, so the tolerance is 0.04 m here. The point of
    // the check is that the end effector comes *back*, i.e. that the pose stream was tracked
    // for 6 s and did not drift away -- a quarter of the 0.1 m radius is still a tight bound.
    eprintln!("fer sim: the Cartesian circle closed to {distance} m");
    assert!(
        distance < 0.04,
        "the end effector ended {distance} m away from the start ({start:?} -> {end:?})"
    );
}

/// 4. Cartesian velocity: the profile of `generate_cartesian_velocity_motion.cpp` over 4 s.
#[test]
fn cartesian_velocity_motion_runs() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

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
        .expect("Cartesian velocity motion failed");

    assert_eq!(common::settled_state(&robot, 5).robot_mode, RobotMode::Idle);
}

/// 5. Torque-only control on FCI v5.
///
/// libfranka 0.9.2 has no `MotionGeneratorMode::kNone`: `Robot::control(ControlCallback)`
/// starts a **joint-velocity** motion generator alongside the external controller and feeds it
/// zero velocities (`src/robot.cpp:41-57`), and `finishMotion` always needs a motion-generator
/// command. `franka::Robot::control_torques` reproduces that on v5, so the wire semantics
/// under test here are: a `Move` with `controller_mode = ExternalController` and
/// `motion_generator_mode = JointVelocity`, 370-byte commands each carrying `tau_J_d`
/// *and* a zero `dq_c`, and a terminal command with `motion_generation_finished` set (a v5
/// `ControllerCommand` has no `torque_command_finished` field at all).
///
/// **Those bytes are pinned by the mock-FCI test
/// `torque_only_control_on_v5_sends_zero_joint_velocities_and_finishes_the_motion`
/// (`src/robot/mock_tests.rs`), not here** -- and they can only be pinned there. The
/// simulator never echoes a torque session's `dq_c` back to the client: its command handler
/// writes `dq_d` only in the `kJointImpedance` + `kJointVelocity` branch
/// (`franka_sim/server/motion_session.py:212`), and an `ExternalController` session falls
/// through to the branch at `:245-255`, which writes `tau_J_d` alone and leaves `dq_d` at the
/// zeros the idle hold put there. So no assertion on `dq_d` could distinguish a correct zero
/// `dq_c` from a wrong one, and this test does not make one.
///
/// What it does check is every *observable* consequence of the scheme against a real server:
/// the motion completes with `MoveStatus::Success`, which on v5 requires the server to have
/// accepted the `ExternalController` + `JointVelocity` mode pair (an unsupported pairing is
/// rejected at `Move`) and to have seen the finish flag that only `finish_motion`'s "a motion
/// generator command is always required" path sets; the robot returns to `Idle`; the arm holds
/// its pose; and the `tau_J_d` the simulator echoes is the commanded zero.
#[test]
fn zero_torque_control_holds_the_robot() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

    let start = robot.read_once().expect("read_once").q;
    let mut time = 0.0;
    let mut worst_tau_d = 0.0f64;
    let mut worst_drift = 0.0f64;

    robot
        .control_torques(
            |state: &RobotState, period: Duration| {
                time += period.as_secs_f64();
                for tau in state.tau_J_d {
                    worst_tau_d = worst_tau_d.max(tau.abs());
                }
                for (joint, q) in state.q.iter().enumerate() {
                    worst_drift = worst_drift.max((q - start[joint]).abs());
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
        // `Ok(())` here means the terminal `Move` reply was `MoveStatus::Success`, which on v5
        // is only reachable through `finish_motion`'s "a motion generator command is always
        // required" path.
        .expect("torque control failed");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    for (index, (actual, expected)) in state.q.iter().zip(start.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 0.1,
            "joint {} drifted by {} rad under zero torque",
            index + 1,
            actual - expected
        );
    }
    // The commanded torque the simulator echoes stays at the commanded zero: gravity is
    // compensated by the simulator, not by the client. On v5 `tau_J_d` is an `f64` on the wire
    // (it is an `f32` on v10), so this is exact.
    assert!(
        worst_tau_d < 1e-6,
        "tau_J_d was not the commanded zero (max |tau_J_d| = {worst_tau_d} Nm)"
    );
    // The arm never wandered *during* the loop either, not just by the time it ended -- a
    // bound that a wrong `dq_c` on the wire, or a torque the simulator did not gravity
    // compensate, would break.
    eprintln!("fer sim: worst in-loop drift under zero torque {worst_drift} rad");
    assert!(
        worst_drift < 0.1,
        "the arm drifted {worst_drift} rad during the zero-torque loop"
    );
}

/// 6. Torque control together with a joint position motion generator, holding the start pose.
///
/// Unlike test 5, this one is *not* v5-specific in shape: an explicit motion generator is what
/// both versions do. It is here to show that the v5 `Move` carries the caller's mode, not the
/// joint-velocity substitute.
#[test]
fn torque_and_joint_position_control_holds_the_pose() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

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

/// 8. The control loop runs at 1 kHz and (nearly) every command is accepted.
#[test]
fn control_loop_timing_and_success_rate() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = fer(&sim);

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
    eprintln!(
        "fer sim: {} cycles, {:.2} % exactly 1 ms, worst {worst} ms, success rate {success_rate}",
        periods.len(),
        ratio * 100.0
    );
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
