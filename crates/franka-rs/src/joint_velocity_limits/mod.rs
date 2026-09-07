//! Position-dependent joint velocity limits read from the robot's URDF.
//!
//! Port of libfranka 0.21.2 `include/franka/joint_velocity_limits.h` and
//! `src/joint_velocity_limits.cpp`. libfranka parses the URDF returned by the `GetRobotModel`
//! command with tinyxml2; this port uses `roxmltree`, because the
//! `<position_based_velocity_limits>` element is a vendor extension that `urdf-rs` drops.
//!
//! The limits are the same curves that the (deprecated)
//! [`crate::rate_limiting::compute_upper_limits_joint_velocity`] hardcodes for the FR3, with the
//! parameters taken from the URDF instead:
//!
//! ```text
//! upper(q) = min(max_velocity, max(0, -velocity_offset + sqrt(max(0, 2 * deceleration_limit * (upper_position_limit - q))))) - tolerance
//! lower(q) = max(-max_velocity, min(0,  velocity_offset - sqrt(max(0, 2 * deceleration_limit * (q - lower_position_limit))))) + tolerance
//! ```

use crate::error::{FrankaError, FrankaResult};
use crate::math_utils::{cmax, cmin};
use crate::rate_limiting::JOINT_VELOCITY_LIMITS_TOLERANCE;

/// Number of joints. Port of `JointVelocityLimitsConfig::kNumJoints`.
pub const NUM_JOINTS: usize = 7;

const ROBOT_ELEMENT_NAME: &str = "robot";
const JOINT_ELEMENT_NAME: &str = "joint";
const POSITION_BASED_VELOCITY_LIMITS_ELEMENT_NAME: &str = "position_based_velocity_limits";
const LIMIT_ELEMENT_NAME: &str = "limit";
const NAME_ATTRIBUTE_NAME: &str = "name";
const VELOCITY_ATTRIBUTE_NAME: &str = "velocity";
const UPPER_ATTRIBUTE_NAME: &str = "upper";
const LOWER_ATTRIBUTE_NAME: &str = "lower";
const VELOCITY_OFFSET_ATTRIBUTE_NAME: &str = "velocity_offset";
const DECELERATION_LIMIT_ATTRIBUTE_NAME: &str = "deceleration_limit";

/// Joint name patterns, index 0..6. Port of `kJoint1Name` .. `kJoint7Name`.
const JOINT_NAMES: [&str; NUM_JOINTS] = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
];

/// Position-based joint velocity limit constants for a single joint, read from the URDF.
///
/// Port of `franka::PositionBasedJointVelocityLimitConstants`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PositionBasedJointVelocityLimitConstants {
    /// Maximum velocity (URDF `<limit velocity>`).
    pub max_velocity: f64,
    /// Velocity offset (URDF `<position_based_velocity_limits velocity_offset>`).
    pub velocity_offset: f64,
    /// Deceleration limit (URDF `<position_based_velocity_limits deceleration_limit>`).
    pub deceleration_limit: f64,
    /// Upper position limit (URDF `<limit upper>`).
    pub upper_position_limit: f64,
    /// Lower position limit (URDF `<limit lower>`).
    pub lower_position_limit: f64,
}

impl PositionBasedJointVelocityLimitConstants {
    /// Creates the constants for one joint.
    pub fn new(
        max_velocity: f64,
        velocity_offset: f64,
        deceleration_limit: f64,
        upper_position_limit: f64,
        lower_position_limit: f64,
    ) -> Self {
        PositionBasedJointVelocityLimitConstants {
            max_velocity,
            velocity_offset,
            deceleration_limit,
            upper_position_limit,
            lower_position_limit,
        }
    }
}

/// Position-based joint velocity limit parameters for all seven joints.
///
/// Port of `franka::JointVelocityLimitsConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JointVelocityLimitsConfig {
    joint_params: [PositionBasedJointVelocityLimitConstants; NUM_JOINTS],
}

impl JointVelocityLimitsConfig {
    /// Parses the joint velocity limit parameters from a URDF string.
    ///
    /// Port of `JointVelocityLimitsConfig::parseFromURDF`. Every `<joint>` element whose name
    /// matches (exactly, or as a substring, so `fr3_joint1` matches too) one of `joint1` ..
    /// `joint7` must carry a `<limit>` element with `velocity`, `upper` and `lower` attributes
    /// and a `<position_based_velocity_limits>` element with `velocity_offset` and
    /// `deceleration_limit` attributes; all seven joints must be present.
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] (libfranka throws `std::runtime_error`) if the string is
    /// not valid XML, has no `<robot>` root, misses one of the elements, attributes or joints,
    /// or if an attribute is not a number.
    pub fn from_urdf(urdf: &str) -> FrankaResult<Self> {
        // tinyxml2 tolerates whitespace in front of the XML declaration, roxmltree does not.
        let document = roxmltree::Document::parse(urdf.trim_start()).map_err(|_| {
            FrankaError::InvalidArgument(
                "Failed to parse URDF for joint velocity limits".to_string(),
            )
        })?;
        let robot = document.root_element();
        if robot.tag_name().name() != ROBOT_ELEMENT_NAME {
            return Err(FrankaError::InvalidArgument(
                "Failed to parse URDF: no <robot> element exists for joint velocity limits"
                    .to_string(),
            ));
        }

        let mut joint_params = [PositionBasedJointVelocityLimitConstants::default(); NUM_JOINTS];
        let mut found_joints = [false; NUM_JOINTS];

        for joint in robot
            .children()
            .filter(|n| n.is_element() && n.tag_name().name() == JOINT_ELEMENT_NAME)
        {
            let Some(name) = joint.attribute(NAME_ATTRIBUTE_NAME) else {
                continue;
            };
            let Some(index) = joint_index(name) else {
                continue;
            };

            let position_based_limits = joint
                .children()
                .find(|n| {
                    n.is_element()
                        && n.tag_name().name() == POSITION_BASED_VELOCITY_LIMITS_ELEMENT_NAME
                })
                .ok_or_else(|| {
                    FrankaError::InvalidArgument(format!(
                        "Missing <{POSITION_BASED_VELOCITY_LIMITS_ELEMENT_NAME}> element for \
                         joint: {name}"
                    ))
                })?;
            let limit = joint
                .children()
                .find(|n| n.is_element() && n.tag_name().name() == LIMIT_ELEMENT_NAME)
                .ok_or_else(|| {
                    FrankaError::InvalidArgument(format!(
                        "Missing <{LIMIT_ELEMENT_NAME}> element for joint: {name}"
                    ))
                })?;

            found_joints[index] = true;
            joint_params[index] = PositionBasedJointVelocityLimitConstants::new(
                parse_required_double(&limit, VELOCITY_ATTRIBUTE_NAME, name, LIMIT_ELEMENT_NAME)?,
                parse_required_double(
                    &position_based_limits,
                    VELOCITY_OFFSET_ATTRIBUTE_NAME,
                    name,
                    POSITION_BASED_VELOCITY_LIMITS_ELEMENT_NAME,
                )?,
                parse_required_double(
                    &position_based_limits,
                    DECELERATION_LIMIT_ATTRIBUTE_NAME,
                    name,
                    POSITION_BASED_VELOCITY_LIMITS_ELEMENT_NAME,
                )?,
                parse_required_double(&limit, UPPER_ATTRIBUTE_NAME, name, LIMIT_ELEMENT_NAME)?,
                parse_required_double(&limit, LOWER_ATTRIBUTE_NAME, name, LIMIT_ELEMENT_NAME)?,
            );
        }

        if !found_joints.iter().all(|found| *found) {
            let mut missing_joints = String::from("Missing required joints: ");
            for (i, found) in found_joints.iter().enumerate() {
                if !found {
                    missing_joints.push_str(&format!("joint{} ", i + 1));
                }
            }
            return Err(FrankaError::InvalidArgument(missing_joints));
        }

        Ok(JointVelocityLimitsConfig { joint_params })
    }

    /// Joint velocity limit parameters of all joints.
    ///
    /// Port of `JointVelocityLimitsConfig::getJointParams`.
    pub fn joint_params(&self) -> &[PositionBasedJointVelocityLimitConstants; NUM_JOINTS] {
        &self.joint_params
    }

    /// Computes the upper joint velocity limits at the given joint positions.
    ///
    /// Port of `JointVelocityLimitsConfig::getUpperJointVelocityLimits`.
    pub fn upper_limits(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        let mut result = [0.0; NUM_JOINTS];
        for i in 0..NUM_JOINTS {
            let params = &self.joint_params[i];
            result[i] = cmin(
                params.max_velocity,
                cmax(
                    0.0,
                    -params.velocity_offset
                        + cmax(
                            0.0,
                            2.0 * params.deceleration_limit * (params.upper_position_limit - q[i]),
                        )
                        .sqrt(),
                ),
            ) - JOINT_VELOCITY_LIMITS_TOLERANCE[i];
        }
        result
    }

    /// Computes the lower joint velocity limits at the given joint positions.
    ///
    /// Port of `JointVelocityLimitsConfig::getLowerJointVelocityLimits`.
    pub fn lower_limits(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        let mut result = [0.0; NUM_JOINTS];
        for i in 0..NUM_JOINTS {
            let params = &self.joint_params[i];
            result[i] = cmax(
                -params.max_velocity,
                cmin(
                    0.0,
                    params.velocity_offset
                        - cmax(
                            0.0,
                            2.0 * params.deceleration_limit * (-params.lower_position_limit + q[i]),
                        )
                        .sqrt(),
                ),
            ) + JOINT_VELOCITY_LIMITS_TOLERANCE[i];
        }
        result
    }
}

impl std::ops::Index<usize> for JointVelocityLimitsConfig {
    type Output = PositionBasedJointVelocityLimitConstants;

    /// Port of `JointVelocityLimitsConfig::operator[]`.
    fn index(&self, joint_index: usize) -> &Self::Output {
        &self.joint_params[joint_index]
    }
}

/// Joint index for a joint name, or `None`.
///
/// Port of `JointVelocityLimitsConfig::getJointIndex`: exact match first, then substring match,
/// so that prefixed names such as `fr3_joint1` are recognised as well.
fn joint_index(joint_name: &str) -> Option<usize> {
    if let Some(index) = JOINT_NAMES.iter().position(|name| *name == joint_name) {
        return Some(index);
    }
    JOINT_NAMES
        .iter()
        .position(|pattern| joint_name.contains(pattern))
}

/// Port of the `parse_required_double` lambda in `parseFromURDF`.
fn parse_required_double(
    element: &roxmltree::Node<'_, '_>,
    attribute: &str,
    joint_name: &str,
    element_name: &str,
) -> FrankaResult<f64> {
    let value = element.attribute(attribute).ok_or_else(|| {
        FrankaError::InvalidArgument(format!(
            "Missing '{attribute}' attribute in <{element_name}> for joint: {joint_name}"
        ))
    })?;
    value.trim().parse::<f64>().map_err(|_| {
        FrankaError::InvalidArgument(format!(
            "Invalid '{attribute}' attribute in <{element_name}> for joint: {joint_name}"
        ))
    })
}

#[cfg(test)]
mod tests;
