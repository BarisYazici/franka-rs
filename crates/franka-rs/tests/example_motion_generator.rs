//! The examples' shared `MotionGenerator` seeds its start pose from a different `RobotState`
//! field depending on the FCI version, and getting that wrong trips a reflex on a real FER
//! hardware. This pins the behaviour down.
//!
//! **A simulator test cannot catch this.** franka-sim reports `q_d == q` (the simulated arm
//! tracks its command exactly, with no impedance deflection), so both seeds produce the same
//! number there. The bug this guards against only appears on a real arm, where `q` lags `q_d`
//! by the gravity/impedance deflection -- on the FER that regression tripped
//! `joint_motion_generator_velocity_discontinuity`, because FCI v5 rate-limits the first
//! command against the robot's own `q_d` and a `q` seed therefore implies a non-zero commanded
//! joint velocity out of the gate.

#[path = "../examples/common/mod.rs"]
mod examples_common;

use examples_common::{MotionGenerator, READY_POSE};
use franka::{Duration, FciVersion, RobotState};

/// A state whose measured `q` differs from the commanded `q_d`, as it always does on a real
/// arm holding position under gravity.
fn deflected_state(q_d: [f64; 7]) -> RobotState {
    let mut state = RobotState {
        q_d,
        q: q_d,
        ..RobotState::default()
    };
    // The deflection this rig actually shows, largest at joint 4 (it carries ~22.7 Nm).
    state.q[3] += 2.3e-3;
    state.q[1] -= 4.0e-4;
    state
}

/// The first setpoint of a point-to-point move is the seed itself (the ramp contributes
/// nothing at `t = 0`), so it reveals which field was used.
fn first_setpoint(version: FciVersion, state: &RobotState) -> [f64; 7] {
    let mut generator = MotionGenerator::new(version, 0.5, READY_POSE);
    generator.step(state, Duration::from_millis(0)).q
}

#[test]
fn v5_seeds_from_q_d_and_v10_from_q() {
    // Start away from the goal so the generator is in its ramp, not at its terminal case.
    let q_d = [0.1, -0.6, 0.05, -2.0, 0.02, 1.4, 0.7];
    let state = deflected_state(q_d);

    // libfranka 0.9.2 `examples/examples_common.cpp:119`: `q_start_ = robot_state.q_d`.
    assert_eq!(
        first_setpoint(FciVersion::V5, &state),
        state.q_d,
        "v5 must seed from the commanded q_d"
    );
    // libfranka 0.21.2 `examples/examples_common.cpp:120`: `q_start_ = robot_state.q`.
    assert_eq!(
        first_setpoint(FciVersion::V10, &state),
        state.q,
        "v10 must seed from the measured q"
    );
    assert_ne!(state.q, state.q_d, "the fixture must actually be deflected");
}

#[test]
fn v5_at_the_goal_commands_q_d_not_q() {
    // The exact case that reflexed on hardware: the arm has just been homed, so it sits at the
    // goal, `delta_q` is ~0 and the generator finishes on its first callback. That single
    // command must be `q_d`, which implies no commanded velocity; commanding the measured `q`
    // instead ends the Move while a velocity is still commanded.
    let state = deflected_state(READY_POSE);

    let mut generator = MotionGenerator::new(FciVersion::V5, 0.5, READY_POSE);
    let output = generator.step(&state, Duration::from_millis(0));

    assert!(
        output.motion_finished,
        "a zero-length move finishes at once"
    );
    assert_eq!(
        output.q, state.q_d,
        "v5 must command q_d, not the measured q"
    );
    assert_ne!(output.q, state.q);
}
