//! `Robot::stop()` from a second thread, and the reflex/recovery path, on FCI v5 against the
//! franka-sim simulator running `--protocol v5 --robot panda`.
//!
//! Run under the machine-wide simulator lock:
//!
//! ```text
//! flock .sim.lock cargo test -p franka-rs --test sim_v5_stop_and_reflex
//! ```
//!
//! The FCI v5 `Move::Status` and `StopMove::Status` enums are shorter than v10's and numbered
//! differently (`Preempted = 2` on v5 against 4 on v10; `ReflexAborted = 6` against 8). Test 3
//! is where that becomes evidence: it asserts `MoveStatus::ReflexAborted` on a reply whose
//! status byte the container log shows as 6, which only the v5 table maps that way.
//!
//! Test 1 is **not** such evidence, because this image answers a stopped `Move` with
//! `kSuccess` (see below): its `Protocol` branch is the client reporting "a success status
//! arrived where the state says the motion is over", which a mis-parsed `Preempted` would
//! produce too. What test 1 pins is the client's side of `stop()` -- prompt return, an error,
//! a surviving session -- plus the exact wording of both possible errors.
//!
//! Two properties of the `franka-sim:panda-v5` image shape this file, both established by
//! reading its container logs and its bundled sources (see `docs/book/src/fer.md`):
//!
//! 1. Its `StopMove` handler answers the running motion's `Move` with **`kSuccess`** instead
//!    of `kPreempted`. The newer `franka-sim:dev` image answers `kPreempted`, which is what
//!    `sim_stop_and_reflex.rs` (FR3) asserts and what a real robot does.
//! 2. Its motion-limit checker uses **FR3** limit tables for every arm
//!    (`franka_sim/limits/tables.py` has no per-robot table), so with
//!    `--enforce-motion-limits` it judges a correctly *FER*-rate-limited command stream
//!    against the FR3's numbers and aborts it. This is the single reason both
//!    `--enforce-motion-limits` tests below deviate from their FR3 counterparts; the
//!    *velocity* envelope it applies is sound (2.619 rad/s for joint 1 at 1.89 rad, and the
//!    reflex test's ramp genuinely reaches 2.72 rad/s).

mod common;

use std::f64::consts::PI;
use std::sync::Arc;
use std::time::Instant;

use franka::{
    motion_finished, ControllerMode, Duration, FciVersion, FrankaError, JointPositions, MoveStatus,
    RobotMode, RobotState, Torques, DEFAULT_CUTOFF_FREQUENCY, MAX_CUTOFF_FREQUENCY,
};
use franka_sim_test::SimConfig;

/// A gentle cosine joint move, from wherever the robot is.
///
/// The ramp starts from the robot's **`q_d`**, not its `q`. The client's rate limiter measures
/// every commanded step against `state.q_d`, so a first command placed at the *measured* `q`
/// asks for a step of whatever the tracking error happens to be, and the limiter answers with
/// a few cycles clipped to the FER's `kMaxJointJerk` -- which this simulator image rejects
/// against the FR3's smaller one (finding 2 in the module docs). That made this helper fail
/// about half the time when it ran under `--enforce-motion-limits` after a reflex, where the
/// gap between `q` and `q_d` is largest. Starting at `q_d` makes the first step zero and the
/// motion deterministic; it is also what libfranka's own `MotionGenerator` does.
fn cosine_joint_move(robot: &franka::Robot) -> franka::FrankaResult<()> {
    let start = robot.read_once()?.q_d;
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

/// A half-second hold at the pose measured on the **first control cycle**.
///
/// Used twice by the reflex test: once while the reflex errors are latched, where the `Move`
/// must be refused outright, and once after `automatic_error_recovery`, where it must run to
/// completion.
///
/// `limit_rate = false` is deliberate, and is the one deviation this test makes from its FR3
/// counterpart. With the limiter **on**, the same hold is aborted within two cycles for
/// `joint_motion_generator_acceleration_discontinuity` -- the simulator's FR3 jerk table again
/// (see `a_fer_rate_limited_motion_is_judged_against_fr3_limits`), reached like this:
///
/// * entering position mode, the simulator re-seeds `q_d` from the *measured* `q`
///   (`franka_sim/server/motion_session.py:195-196`). After a reflex the idle hold's `q_d` and
///   the measured `q` are apart -- 1.857997 vs 1.841221 rad, measured -- so from the second
///   cycle the client's limiter has about 0.017 rad to close;
/// * and no setpoint avoids it: joint 1's permanent +-0.36 rad/s limit cycle (see
///   `sim_v5_motions.rs`) moves `q` by ~0.37 mrad every cycle, fifty times the ~7.5 urad that
///   7500 rad/s^3 of jerk covers in 1 ms;
/// * so the client emits exactly `rate_limiting::fer::MAX_JOINT_JERK[0]` and the simulator rejects it
///   against the FR3's 5000. From the container log of this very test:
///   `joint_motion_generator_acceleration_discontinuity: q_c joint 1 = 7500 rad/s^3,
///   limit 5000 rad/s^3`.
///
/// With the limiter off the client sends the constant setpoint unchanged, the simulator's
/// motion-generator checks difference a constant to zero, and the measured 0.36 rad/s is well
/// inside the 2.619 rad/s its envelope allows there. Measured: 500/500 cycles on three
/// consecutive runs, against 2 cycles with the limiter on. So this isolates "did the motion
/// run?" from the jerk-table gap that would otherwise decide the verdict.
fn hold_at_current_pose(robot: &franka::Robot) -> franka::FrankaResult<()> {
    let mut hold: Option<[f64; 7]> = None;
    let mut cycles = 0;
    robot.control_joint_positions(
        |state: &RobotState, _period: Duration| {
            let setpoint = *hold.get_or_insert(state.q);
            cycles += 1;
            let output = JointPositions::new(setpoint);
            if cycles >= 500 {
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::JointImpedance,
        false,
        MAX_CUTOFF_FREQUENCY,
    )
}

/// 1. `stop()` on another thread ends a running torque control loop promptly, with an error.
///
/// On FCI v5 a torque-only loop is a joint-velocity motion generator paired with an external
/// controller (see `sim_v5_motions.rs`), so this also covers cancelling that pairing.
///
/// **What the error is depends on the server.** A real FER, and the newer `franka-sim:dev`
/// image, answer the interrupted `Move` with `kPreempted`, which the client turns into
/// `ControlException("libfranka: Move command preempted!")` carrying
/// `MoveStatus::Preempted` -- exactly what the FR3 suite asserts. The
/// `franka-sim:panda-v5` image answers `kSuccess` instead
/// (`franka_sim/server/motion_session.py`, `handle_stop_move_command`: `status =
/// self.codec.move_status.kSuccess`), which is a success status arriving where the state
/// already says the motion is over, and the client correctly reports that as
/// `Protocol("Unexpected reply to a Move command")` rather than pretending the motion
/// finished cleanly.
///
/// Both are accepted here, and the preempted branch is asserted in full, so rebuilding the
/// FER image from a newer franka-sim tightens this test automatically instead of breaking
/// it. What is asserted unconditionally is the part that is the client's responsibility: the
/// loop returns, it returns quickly, it returns an error, and the session survives.
#[test]
fn stop_from_another_thread_ends_the_motion() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = Arc::new(common::connect(&sim));
    assert_eq!(robot.fci_version(), FciVersion::V5);
    common::set_default_behavior(&robot);
    common::move_to_start_fer(&robot);

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
    eprintln!("fer sim: the stopped torque loop returned {result:?} after {elapsed:?}");
    match result {
        Err(FrankaError::Control(exception)) => {
            assert_eq!(exception.message, "libfranka: Move command preempted!");
            assert_eq!(exception.move_status, Some(MoveStatus::Preempted));
        }
        Err(FrankaError::Protocol(message)) => assert_eq!(
            message, "Unexpected reply to a Move command",
            "the only Protocol error this path may produce is the one the panda-v5 image's \
             kSuccess-on-StopMove forces"
        ),
        other => panic!("expected the stopped motion to fail, got {other:?}"),
    }

    assert_eq!(common::settled_state(&robot, 5).robot_mode, RobotMode::Idle);
    cosine_joint_move(&robot).expect("a new motion must start after stop()");
}

/// 2. A `stop()` with no motion running is accepted and leaves no stray reply behind.
#[test]
fn stop_without_a_motion_is_harmless() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = common::connect(&sim);
    assert_eq!(robot.fci_version(), FciVersion::V5);

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

/// 3. Driving joint 1 far past the FER's joint-velocity limit trips a reflex, and
///    `automatic_error_recovery` clears it.
///
/// The FER's velocity limit is *flat*, unlike the FR3's position-dependent envelope: 2.175
/// rad/s on joints 1-4 (`rate_limiting::fer::MAX_JOINT_VELOCITY`, which subtracts
/// `kLimitEps` and three lost packets' worth of acceleration and lands at about 2.129 rad/s).
/// The FR3 test trips its reflex with a 2.0 rad/s ramp *near the joint limit*, where the FR3
/// envelope has narrowed; on an FER 2.0 rad/s is legal everywhere, so this ramps to 3.0 rad/s
/// instead. The commanded position is capped short of the FER's 2.8973 rad joint-1 limit so
/// that velocity, not position, is what is violated.
///
/// The velocity is ramped in over **2 s**, not the FR3 test's 0.5 s, for the reason test 4
/// documents: this image checks against the FR3's `MAX_JOINT_JERK = 5000 - 1e-3` rad/s^3, and
/// a 2.5 rad/s ramp reached in 0.5 s puts exactly 5000 rad/s^3 of jerk into the first cycle,
/// so the motion was aborted for `joint_motion_generator_acceleration_discontinuity` before it
/// ever got fast enough to break a velocity limit (observed). Over 2 s the first cycle carries
/// 1500 rad/s^3 and the acceleration is a flat 1.5 rad/s^2, both well inside even the FR3
/// numbers, leaving the velocity as the only thing that goes out of bounds.
///
/// The *threshold* the simulator applies is the FR3's, not the FER's, for the same reason --
/// so what this test proves is that the v5 reflex/recovery path works end to end (abort with
/// `MoveStatus::ReflexAborted`, `RobotMode::Reflex`, `Move` rejected until
/// `automatic_error_recovery`, motion afterwards), not the exact rad/s at which a real FER
/// would trip.
///
/// **The arm is not homed first.** `move_to_start_fer` uses libfranka's point-to-point
/// `MotionGenerator` with the rate limiter on, and under `--enforce-motion-limits` this image
/// aborts that motion -- see `a_fer_rate_limited_motion_is_judged_against_fr3_limits` below
/// for the measurement and the reason. The container is fresh, so the arm is already at
/// [`common::FER_SIM_HOME_POSE`]; that is asserted rather than assumed.
#[test]
fn velocity_violation_trips_a_reflex_and_recovery_clears_it() {
    let sim = common::sim(SimConfig::fer_v5().with_motion_limits());
    let robot = common::connect(&sim);
    assert_eq!(robot.fci_version(), FciVersion::V5);
    common::set_default_behavior(&robot);

    let start = robot.read_once().expect("read_once").q;
    for (index, (actual, expected)) in start
        .iter()
        .zip(common::FER_SIM_HOME_POSE.iter())
        .enumerate()
    {
        assert!(
            (actual - expected).abs() < common::HOME_POSE_TOLERANCE,
            "this test does not home the arm, so the container must be fresh: q[{index}] = \
             {actual}, expected about {expected}"
        );
    }

    let mut time = 0.0;
    let mut commanded = start[0];

    // No rate limiting and no filtering: the client sends the raw ramp, which runs past the
    // FER's flat velocity limit.
    let result = robot.control_joint_positions(
        |_state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            let velocity = 3.0 * (time / 2.0).min(1.0);
            // Period-proportional, not a hardcoded 1 ms step: a lost cycle (`period` > 1 ms)
            // must still advance the ramp by the distance that velocity covers over the real
            // elapsed time, or the sim's backward-Euler acceleration check sees a step and
            // trips `joint_motion_generator_velocity_discontinuity` before the velocity
            // envelope this test means to provoke.
            commanded = (commanded + velocity * period.as_secs_f64()).min(2.85);
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
    eprintln!("fer sim: the over-speed ramp ended with {result:?}");

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
    eprintln!("fer sim: reflex reasons {errors}");
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
    eprintln!("fer sim: the reflex left joint 1 at {} rad", state.q[0]);
    assert!(
        state.q[0] < 2.8973,
        "joint 1 went past the FER's limit: {}",
        state.q[0]
    );
    assert_eq!(state.robot_mode, RobotMode::Reflex);

    // A new motion is refused outright until the errors are cleared.
    match hold_at_current_pose(&robot) {
        Err(FrankaError::Command(message)) => assert!(
            message.contains("command not possible in the current mode"),
            "unexpected rejection: {message}"
        ),
        other => panic!("expected a rejected Move, got {other:?}"),
    }

    robot
        .automatic_error_recovery()
        .expect("automatic_error_recovery failed");

    assert_eq!(
        common::settled_state(&robot, 5).robot_mode,
        RobotMode::Idle,
        "automatic_error_recovery must clear the reflex"
    );
    hold_at_current_pose(&robot).expect("a motion must run after error recovery");
}

/// 4. Characterisation: with `--enforce-motion-limits`, this simulator image judges a
///    *correctly FER-rate-limited* command stream against the **FR3's** limits and aborts it.
///
/// The FR3 counterpart of this test (`sim_stop_and_reflex.rs`,
/// `rate_limiting_keeps_the_motion_inside_the_envelope`) asserts the opposite: that a
/// rate-limited motion completes. It cannot on the FER image, and the reason is entirely on
/// the simulator's side. `franka_sim/limits/tables.py` holds one set of limits, lifted from
/// libfranka's FR3 constants, with no per-robot variant:
///
/// ```text
/// MAX_JOINT_JERK         = 5000 - 1e-3   (all joints)     FR3
/// MAX_JOINT_ACCELERATION = 10   - 1e-3   (all joints)     FR3
/// ```
///
/// The FER's are larger on most joints (`rate_limiting::fer`, libfranka 0.9.2's
/// `rate_limiting.h`): `kMaxJointJerk = [7500, 3750, 5000, 6250, 7500, 10000, 10000]`,
/// `kMaxJointAcceleration = [15, 7.5, 10, 12.5, 15, 20, 20]`. So a command stream the client's
/// *FER* limiter has clipped to `kMaxJointJerk[0] = 7500 - 1e-3` rad/s^3 -- a stream a real
/// FER accepts by construction, since 7500 is its own limit -- is 50 % over what this
/// simulator allows.
///
/// The demand here is a step: `q_c` jumps 0.5 rad on joint 1 in the first cycle, with
/// `limit_rate = true` and no low-pass filter, so the limiter is guaranteed to saturate rather
/// than incidentally saturating on the tracking error (which is what made an earlier version
/// of this test, built on libfranka's point-to-point `MotionGenerator`, pass about half the
/// time). Measured, from the container log:
///
/// ```text
/// WARNING franka_sim.motion_limits: motion limit violated:
///   joint_motion_generator_acceleration_discontinuity: q_c joint 1 = -7500 rad/s^3,
///   limit 5000 rad/s^3
/// INFO franka_sim.franka_sim_server: Sent Move response: status=<MoveStatus.kReflexAborted: 6>
/// ```
///
/// This test therefore pins the simulator gap, with the numbers, so that it fails loudly the
/// day franka-sim grows FER limit tables -- at which point it should be replaced by the FR3
/// version's assertion that the motion completes.
#[test]
fn a_fer_rate_limited_motion_is_judged_against_fr3_limits() {
    let sim = common::sim(SimConfig::fer_v5().with_motion_limits());
    let robot = common::connect(&sim);
    assert_eq!(robot.fci_version(), FciVersion::V5);
    common::set_default_behavior(&robot);

    let start = robot.read_once().expect("read_once").q_d;
    let mut target = start;
    target[0] += 0.5;

    let mut time = 0.0;
    let result = robot.control_joint_positions(
        |_state: &RobotState, period: Duration| {
            time += period.as_secs_f64();
            let output = JointPositions::new(target);
            if time >= 2.0 {
                motion_finished(output)
            } else {
                output
            }
        },
        ControllerMode::JointImpedance,
        // The rate limiter is *on*: this is a stream that has already been made safe for a
        // FER, and the simulator rejects it anyway.
        true,
        MAX_CUTOFF_FREQUENCY,
    );
    eprintln!("fer sim: the FER-rate-limited step ended with {result:?}");

    let exception = match result {
        Err(FrankaError::Control(exception)) => exception,
        other => panic!(
            "the FER-rate-limited step was accepted under --enforce-motion-limits \
             ({other:?}). Two different causes produce this, and they need opposite fixes:\n\
             (a) CLIENT REGRESSION -- version selection in src/robot/control_loop.rs \
             (`rate_limits(version)`) fell back to the FR3 constants, so the commanded jerk is \
             5000 - 1e-3 instead of the FER's kMaxJointJerk[0] = 7500 - 1e-3. That is \
             exactly this simulator's own limit, so the motion is accepted -- and a real FER \
             would then be driven with an FR3-sized budget. Check \
             control_loop.rs's rate-limit selection tests and \
             mock_tests::the_first_v5_joint_position_command_is_rate_limited_against_q_d first.\n\
             (b) SIMULATOR CHANGE -- franka-sim grew per-robot limit tables, in which case \
             replace this characterisation test with sim_stop_and_reflex.rs's \
             rate_limiting_keeps_the_motion_inside_the_envelope."
        ),
    };
    assert_eq!(exception.move_status, Some(MoveStatus::ReflexAborted));
    assert!(
        exception
            .last_motion_errors
            .get("joint_motion_generator_acceleration_discontinuity"),
        "expected the FR3 jerk table to be what rejects the motion, got {}",
        exception.last_motion_errors
    );
}
