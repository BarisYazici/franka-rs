//! Motion conversion: the first-cycle reference rule, filtering, rate limiting, elbow
//! handling and the `Move` mode mapping.

use super::*;
use crate::control_types::{CartesianPose, CartesianVelocities, JointPositions, JointVelocities};

/// libfranka 0.9.2 has no `initialized_filter_`: its `convertMotion`
/// (`src/control_loop.cpp:188-205`) always references `robot_state.q_d`, so on an FER the
/// *first* joint-position setpoint of a motion is clamped to one integration step like
/// every other one. libfranka 0.21.2 references the command itself on the first cycle
/// (`src/control_loop.cpp:194-200`), which lets an arbitrarily large first jump through.
#[test]
fn the_first_setpoint_is_limited_against_q_d_on_v5_but_not_on_v10() {
    let state = valid_state();
    let jump = JointPositions::new([1.0; 7]);

    let mut v10 = MotionGeneratorCommand::default();
    jump.convert(
        &mut context_for(FciVersion::V10, true, false),
        &state,
        &mut v10,
    )
    .unwrap();
    assert_eq!(
        f64s_to_f64(&v10.q_c),
        [1.0; 7],
        "FCI v10 lets the first command through unchanged"
    );

    let mut v5 = MotionGeneratorCommand::default();
    jump.convert(
        &mut context_for(FciVersion::V5, true, false),
        &state,
        &mut v5,
    )
    .unwrap();
    let limited = f64s_to_f64(&v5.q_c);
    assert!(
        limited[0] > 0.0 && limited[0] < 1e-3,
        "FCI v5 must clamp the first command to one step from q_d, got {}",
        limited[0]
    );

    // On FCI v5 the first cycle is not special: the second one produces the same clamp.
    let mut second = MotionGeneratorCommand::default();
    let mut context = context_for(FciVersion::V5, true, false);
    context.initialized_filter = true;
    jump.convert(&mut context, &state, &mut second).unwrap();
    assert_eq!(f64s_to_f64(&second.q_c), limited);

    // ... whereas on FCI v10 the second cycle is where the clamp starts.
    let mut second_v10 = MotionGeneratorCommand::default();
    let mut context = context_for(FciVersion::V10, true, false);
    context.initialized_filter = true;
    jump.convert(&mut context, &state, &mut second_v10).unwrap();
    assert!(f64s_to_f64(&second_v10.q_c)[0] < 1e-3);
}

// Port of ControlLoops.SpinOnceWithMotionCallbackAndControllerMode for JointPositions:
// without the filter and the limiter the command reaches the wire unchanged.
#[test]
fn joint_positions_pass_through_unfiltered() {
    let state = valid_state();
    let mut command = MotionGeneratorCommand::default();
    let motion = JointPositions::new([0., 1., 2., 3., 4., 5., 6.]);
    motion
        .convert(&mut context(false, false), &state, &mut command)
        .unwrap();
    assert_eq!(f64s_to_f64(&command.q_c), motion.q);
}

// The first cycle filters against the commanded position itself, so it is a no-op; the
// second filters against `robot_state.q_d`.
#[test]
fn joint_positions_use_the_command_as_the_first_filter_reference() {
    let state = valid_state();
    let mut command = MotionGeneratorCommand::default();
    let motion = JointPositions::new([0.1; 7]);
    let mut ctx = context(false, true);
    motion.convert(&mut ctx, &state, &mut command).unwrap();
    assert_eq!(f64s_to_f64(&command.q_c), motion.q);
    assert!(ctx.initialized_filter);

    motion.convert(&mut ctx, &state, &mut command).unwrap();
    // state.q_d is zero, so the second cycle pulls the command towards zero.
    for value in f64s_to_f64(&command.q_c) {
        assert!(value < 0.1 && value > 0.0, "unexpected {value}");
    }
}

#[test]
fn joint_velocities_are_limited_below_the_commanded_value() {
    let state = valid_state();
    let mut command = MotionGeneratorCommand::default();
    let motion = JointVelocities::new([0., 1., 2., 3., 4., 5., 6.]);
    motion
        .convert(&mut context(true, false), &state, &mut command)
        .unwrap();
    for (limited, commanded) in f64s_to_f64(&command.dq_c).iter().zip(motion.dq) {
        assert!(*limited <= commanded, "{limited} !<= {commanded}");
    }
    // Unlimited and unfiltered it is passed through verbatim.
    let mut command = MotionGeneratorCommand::default();
    motion
        .convert(&mut context(false, false), &state, &mut command)
        .unwrap();
    assert_eq!(f64s_to_f64(&command.dq_c), motion.dq);
}

#[test]
fn invalid_motions_are_rejected() {
    let state = valid_state();
    for invalid in [f64::NAN, f64::INFINITY] {
        let mut command = MotionGeneratorCommand::default();
        assert_eq!(
            invalid_argument(
                JointPositions::new([0., 1., invalid, 3., 4., 5., 6.]).convert(
                    &mut context(false, false),
                    &state,
                    &mut command
                )
            ),
            "Commanding value is infinite or NaN."
        );
        assert_eq!(
            invalid_argument(
                JointVelocities::new([0., 1., invalid, 3., 4., 5., 6.]).convert(
                    &mut context(false, false),
                    &state,
                    &mut command
                )
            ),
            "Commanding value is infinite or NaN."
        );
        assert_eq!(
            invalid_argument(
                CartesianVelocities::new([0., 1., invalid, 3., 4., 5.]).convert(
                    &mut context(false, false),
                    &state,
                    &mut command
                )
            ),
            "Commanding value is infinite or NaN."
        );
    }
}

// Port of ControlLoopWithTransformationMatrix.SpinOnceWithInvalidTransformationMatrix.
#[test]
fn invalid_transformation_matrix_is_rejected() {
    let state = valid_state();
    let mut command = MotionGeneratorCommand::default();
    let result =
        CartesianPose::new([0.0; 16]).convert(&mut context(false, false), &state, &mut command);
    assert_eq!(
        invalid_argument(result),
        "libfranka: Attempt to set invalid transformation in motion generator. Has to be \
         column major!"
    );
}

// Port of ControlLoopWithElbow.SpinOnceWithInvalidElbowCallback, plus the non-finite elbow
// that the C++ scalar `limitRate` rejects.
#[test]
fn invalid_elbow_is_rejected() {
    let state = valid_state();
    let mut command = MotionGeneratorCommand::default();
    for limit_rate in [false, true] {
        let result = CartesianPose::with_elbow(IDENTITY, [0.0, 0.5]).convert(
            &mut context(limit_rate, false),
            &state,
            &mut command,
        );
        assert_eq!(
            invalid_argument(result),
            "Invalid elbow configuration given! Only +1 or -1 are allowed for the sign of \
             the 4th joint."
        );
        let result = CartesianVelocities::with_elbow([0.0; 6], [f64::NAN, 1.0]).convert(
            &mut context(limit_rate, false),
            &state,
            &mut command,
        );
        assert_eq!(
            invalid_argument(result),
            "Commanding value is infinite or NaN."
        );
    }
}

#[test]
fn elbow_is_only_written_when_present() {
    let state = valid_state();
    let mut command = MotionGeneratorCommand::default();
    CartesianPose::new(IDENTITY)
        .convert(&mut context(false, false), &state, &mut command)
        .unwrap();
    assert_eq!(command.valid_elbow, 0);
    assert_eq!(f64s_to_f64(&command.elbow_c), [0.0, 0.0]);

    CartesianPose::with_elbow(IDENTITY, [0.25, -1.0])
        .convert(&mut context(false, false), &state, &mut command)
        .unwrap();
    assert_eq!(command.valid_elbow, 1);
    assert_eq!(f64s_to_f64(&command.elbow_c), [0.25, -1.0]);
}

#[test]
fn move_modes_match_the_motion_generator_traits() {
    assert_eq!(
        JointPositions::MOVE_MODE,
        MoveMotionGeneratorMode::JointPosition
    );
    assert_eq!(
        JointVelocities::MOVE_MODE,
        MoveMotionGeneratorMode::JointVelocity
    );
    assert_eq!(
        CartesianPose::MOVE_MODE,
        MoveMotionGeneratorMode::CartesianPosition
    );
    assert_eq!(
        CartesianVelocities::MOVE_MODE,
        MoveMotionGeneratorMode::CartesianVelocity
    );
}
