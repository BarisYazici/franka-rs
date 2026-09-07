//! The four `ControlLoop<T>::convertMotion` specialisations, plus the elbow handling they
//! share.

use super::*;
use crate::control_types::{
    check_elbow, check_finite, CartesianPose, CartesianVelocities, JointPositions, JointVelocities,
};
use crate::lowpass_filter::{cartesian_low_pass_filter, low_pass_filter};
use crate::rate_limiting::{
    limit_rate_cartesian_pose, limit_rate_cartesian_velocity, limit_rate_joint_position,
    limit_rate_joint_positions, limit_rate_joint_velocities, DELTA_T,
};
use crate::wire::f64s_to_wire;

impl ControlLoopMotion for JointPositions {
    const MOVE_MODE: MoveMotionGeneratorMode = move_mode(Self::KIND);

    fn convert(
        &self,
        context: &mut ConvertContext<'_>,
        state: &RobotState,
        command: &mut MotionGeneratorCommand,
    ) -> FrankaResult<()> {
        let mut q_c = self.q;
        let reference_position =
            if !context.initialized_filter && context.first_command_is_its_own_reference {
                q_c
            } else {
                state.q_d
            };
        context.initialized_filter = true;
        if context.filtering() {
            for (i, value) in q_c.iter_mut().enumerate() {
                *value = low_pass_filter(
                    DELTA_T,
                    *value,
                    reference_position[i],
                    context.cutoff_frequency,
                )?;
            }
        }
        if context.limit_rate {
            q_c = limit_rate_joint_positions(
                &context
                    .limits
                    .upper_joint_velocity_limits(&reference_position),
                &context
                    .limits
                    .lower_joint_velocity_limits(&reference_position),
                &context.rate.max_joint_acceleration,
                &context.rate.max_joint_jerk,
                &q_c,
                &reference_position,
                &state.dq_d,
                &state.ddq_d,
            )?;
        }
        JointPositions::new(q_c).validate()?;
        command.q_c = f64s_to_wire(&q_c);
        Ok(())
    }
}

impl ControlLoopMotion for JointVelocities {
    const MOVE_MODE: MoveMotionGeneratorMode = move_mode(Self::KIND);

    fn convert(
        &self,
        context: &mut ConvertContext<'_>,
        state: &RobotState,
        command: &mut MotionGeneratorCommand,
    ) -> FrankaResult<()> {
        let mut dq_c = self.dq;
        if context.filtering() {
            for (i, value) in dq_c.iter_mut().enumerate() {
                *value = low_pass_filter(DELTA_T, *value, state.dq_d[i], context.cutoff_frequency)?;
            }
        }
        if context.limit_rate {
            dq_c = limit_rate_joint_velocities(
                &context.limits.upper_joint_velocity_limits(&state.q_d),
                &context.limits.lower_joint_velocity_limits(&state.q_d),
                &context.rate.max_joint_acceleration,
                &context.rate.max_joint_jerk,
                &dq_c,
                &state.dq_d,
                &state.ddq_d,
            )?;
        }
        JointVelocities::new(dq_c).validate()?;
        command.dq_c = f64s_to_wire(&dq_c);
        Ok(())
    }
}

impl ControlLoopMotion for CartesianPose {
    const MOVE_MODE: MoveMotionGeneratorMode = move_mode(Self::KIND);

    fn convert(
        &self,
        context: &mut ConvertContext<'_>,
        state: &RobotState,
        command: &mut MotionGeneratorCommand,
    ) -> FrankaResult<()> {
        let mut O_T_EE_c = self.O_T_EE;
        let (reference_pose, reference_elbow) =
            if !context.initialized_filter && context.first_command_is_its_own_reference {
                (O_T_EE_c, if self.has_elbow { self.elbow } else { [0.0; 2] })
            } else {
                (
                    state.O_T_EE_c,
                    if self.has_elbow {
                        state.elbow_c
                    } else {
                        [0.0; 2]
                    },
                )
            };
        context.initialized_filter = true;

        if context.filtering() {
            O_T_EE_c = cartesian_low_pass_filter(
                DELTA_T,
                &O_T_EE_c,
                &reference_pose,
                context.cutoff_frequency,
            )?;
        }
        if context.limit_rate {
            O_T_EE_c = limit_rate_cartesian_pose(
                context.rate.max_translational_velocity,
                context.rate.max_translational_acceleration,
                context.rate.max_translational_jerk,
                context.rate.max_rotational_velocity,
                context.rate.max_rotational_acceleration,
                context.rate.max_rotational_jerk,
                &O_T_EE_c,
                &reference_pose,
                &state.O_dP_EE_c,
                &state.O_ddP_EE_c,
            )?;
        }
        CartesianPose::new(O_T_EE_c).validate()?;
        command.O_T_EE_c = f64s_to_wire(&O_T_EE_c);

        let elbow_c = convert_elbow(
            self.has_elbow,
            &self.elbow,
            &reference_elbow,
            state,
            context,
        )?;
        write_elbow(command, self.has_elbow, &elbow_c);
        if self.has_elbow {
            CartesianPose::with_elbow(O_T_EE_c, elbow_c).validate()?;
        }
        Ok(())
    }
}

impl ControlLoopMotion for CartesianVelocities {
    const MOVE_MODE: MoveMotionGeneratorMode = move_mode(Self::KIND);

    fn convert(
        &self,
        context: &mut ConvertContext<'_>,
        state: &RobotState,
        command: &mut MotionGeneratorCommand,
    ) -> FrankaResult<()> {
        let mut O_dP_EE_c = self.O_dP_EE;
        if context.filtering() {
            for (i, value) in O_dP_EE_c.iter_mut().enumerate() {
                *value = low_pass_filter(
                    DELTA_T,
                    *value,
                    state.O_dP_EE_c[i],
                    context.cutoff_frequency,
                )?;
            }
        }
        if context.limit_rate {
            O_dP_EE_c = limit_rate_cartesian_velocity(
                context.rate.max_translational_velocity,
                context.rate.max_translational_acceleration,
                context.rate.max_translational_jerk,
                context.rate.max_rotational_velocity,
                context.rate.max_rotational_acceleration,
                context.rate.max_rotational_jerk,
                &O_dP_EE_c,
                &state.O_dP_EE_c,
                &state.O_ddP_EE_c,
            )?;
        }
        CartesianVelocities::new(O_dP_EE_c).validate()?;
        command.O_dP_EE_c = f64s_to_wire(&O_dP_EE_c);

        let elbow_c = convert_elbow(self.has_elbow, &self.elbow, &state.elbow_c, state, context)?;
        write_elbow(command, self.has_elbow, &elbow_c);
        if self.has_elbow {
            CartesianVelocities::with_elbow(O_dP_EE_c, elbow_c).validate()?;
        }
        Ok(())
    }
}

/// Filters and rate limits the elbow position, as both Cartesian `convertMotion`
/// specialisations do.
///
/// The C++ scalar `limitRate` overload throws `std::invalid_argument` on a non-finite
/// commanded value, and that throw is the only finiteness guard on `elbow_c[0]` before
/// `checkElbow` runs. The Rust scalar limiter is infallible, so the check is performed here,
/// before filtering, which produces the same `InvalidArgument` error.
fn convert_elbow(
    has_elbow: bool,
    elbow: &[f64; 2],
    reference_elbow: &[f64; 2],
    state: &RobotState,
    context: &ConvertContext<'_>,
) -> FrankaResult<[f64; 2]> {
    if !has_elbow {
        return Ok([0.0; 2]);
    }
    check_finite(elbow)?;

    let mut elbow_c = *elbow;
    if context.filtering() {
        elbow_c[0] = low_pass_filter(
            DELTA_T,
            elbow_c[0],
            reference_elbow[0],
            context.cutoff_frequency,
        )?;
    }
    if context.limit_rate {
        elbow_c[0] = limit_rate_joint_position(
            context.rate.max_elbow_velocity,
            context.rate.min_elbow_velocity,
            context.rate.max_elbow_acceleration,
            context.rate.max_elbow_jerk,
            elbow_c[0],
            reference_elbow[0],
            state.delbow_c[0],
            state.ddelbow_c[0],
        );
    }
    check_elbow(&elbow_c)?;
    Ok(elbow_c)
}

/// Writes the elbow part of a motion command (`valid_elbow` plus `elbow_c`).
fn write_elbow(command: &mut MotionGeneratorCommand, has_elbow: bool, elbow_c: &[f64; 2]) {
    command.valid_elbow = u8::from(has_elbow);
    command.elbow_c = f64s_to_wire(if has_elbow { elbow_c } else { &[0.0; 2] });
}
