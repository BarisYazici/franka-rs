//! The 41 robot error flags, in the order of libfranka's `research_interface::robot::Error`.

/// Number of error flags carried in every `RobotState`.
pub const ERROR_COUNT: usize = 41;

/// Error names in wire order (index = bit position in `RobotState.errors` / `reflex_reason`).
///
/// Source: `common/include/research_interface/robot/error.h` (enum order) and its
/// `getErrorName` table, libfranka 0.21.2.
pub const ERROR_NAMES: [&str; ERROR_COUNT] = [
    "joint_position_limits_violation",
    "cartesian_position_limits_violation",
    "self_collision_avoidance_violation",
    "joint_velocity_violation",
    "cartesian_velocity_violation",
    "force_control_safety_violation",
    "joint_reflex",
    "cartesian_reflex",
    "max_goal_pose_deviation_violation",
    "max_path_pose_deviation_violation",
    "cartesian_velocity_profile_safety_violation",
    "joint_position_motion_generator_start_pose_invalid",
    "joint_motion_generator_position_limits_violation",
    "joint_motion_generator_velocity_limits_violation",
    "joint_motion_generator_velocity_discontinuity",
    "joint_motion_generator_acceleration_discontinuity",
    "cartesian_position_motion_generator_start_pose_invalid",
    "cartesian_motion_generator_elbow_limit_violation",
    "cartesian_motion_generator_velocity_limits_violation",
    "cartesian_motion_generator_velocity_discontinuity",
    "cartesian_motion_generator_acceleration_discontinuity",
    "cartesian_motion_generator_elbow_sign_inconsistent",
    "cartesian_motion_generator_start_elbow_invalid",
    "force_controller_desired_force_tolerance_violation",
    "start_elbow_sign_inconsistent",
    "communication_constraints_violation",
    "power_limit_violation",
    "cartesian_motion_generator_joint_position_limits_violation",
    "cartesian_motion_generator_joint_velocity_limits_violation",
    "cartesian_motion_generator_joint_velocity_discontinuity",
    "cartesian_motion_generator_joint_acceleration_discontinuity",
    "cartesian_position_motion_generator_invalid_frame_flag",
    "controller_torque_discontinuity",
    "joint_p2p_insufficient_torque_for_planning",
    "tau_J_range_violation",
    "instability_detected",
    "joint_move_in_wrong_direction",
    "cartesian_spline_motion_generator_violation",
    "joint_via_motion_generator_planning_joint_limit_violation",
    "base_acceleration_initialization_timeout",
    "base_acceleration_invalid_reading",
];

/// Set of robot error flags (mirrors `franka::Errors`).
///
/// With the `serde` feature it serialises as the list of the set flags' names in wire order
/// (`["joint_reflex", "cartesian_reflex"]`, `[]` when empty) rather than as 41 booleans, and
/// deserialising a name that is not in [`ERROR_NAMES`] is an error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Errors(pub [bool; ERROR_COUNT]);

impl Default for Errors {
    fn default() -> Self {
        Errors([false; ERROR_COUNT])
    }
}

impl Errors {
    /// True if any flag is set (mirrors `franka::Errors::operator bool`).
    pub fn any(&self) -> bool {
        self.0.iter().any(|&e| e)
    }

    /// Names of the set flags, in wire order.
    pub fn names(&self) -> Vec<&'static str> {
        self.0
            .iter()
            .zip(ERROR_NAMES.iter())
            .filter_map(|(&set, &name)| if set { Some(name) } else { None })
            .collect()
    }

    /// Whether the flag with the given libfranka name is set. Unknown names are `false`.
    pub fn get(&self, name: &str) -> bool {
        index_of(name).map(|i| self.0[i]).unwrap_or(false)
    }

    /// Index of the flag with the given name.
    pub fn index_of(name: &str) -> Option<usize> {
        index_of(name)
    }
}

fn index_of(name: &str) -> Option<usize> {
    ERROR_NAMES.iter().position(|&n| n == name)
}

#[cfg(feature = "serde")]
impl serde::Serialize for Errors {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_seq(self.names())
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for Errors {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Errors, D::Error> {
        let names: Vec<std::borrow::Cow<'de, str>> = serde::Deserialize::deserialize(deserializer)?;
        let mut flags = [false; ERROR_COUNT];
        for name in &names {
            let index = index_of(name).ok_or_else(|| {
                serde::de::Error::custom(format!("{name:?} is not a libfranka error name"))
            })?;
            flags[index] = true;
        }
        Ok(Errors(flags))
    }
}

impl From<[bool; ERROR_COUNT]> for Errors {
    fn from(flags: [bool; ERROR_COUNT]) -> Self {
        Errors(flags)
    }
}

impl From<[u8; ERROR_COUNT]> for Errors {
    fn from(flags: [u8; ERROR_COUNT]) -> Self {
        let mut out = [false; ERROR_COUNT];
        for (o, &f) in out.iter_mut().zip(flags.iter()) {
            *o = f != 0;
        }
        Errors(out)
    }
}

/// Formats like libfranka's `Errors::operator std::string`: `["name_a", "name_b"]`, `[]` when empty.
impl std::fmt::Display for Errors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[")?;
        let mut first = true;
        for name in self.names() {
            if !first {
                write!(f, ", ")?;
            }
            first = false;
            write!(f, "\"{name}\"")?;
        }
        write!(f, "]")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn has_41_unique_names() {
        let mut sorted = ERROR_NAMES.to_vec();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), ERROR_COUNT);
    }

    #[test]
    fn indices_match_libfranka_and_sim() {
        assert_eq!(Errors::index_of("joint_velocity_violation"), Some(3));
        assert_eq!(
            Errors::index_of("communication_constraints_violation"),
            Some(25)
        );
        assert_eq!(
            Errors::index_of("controller_torque_discontinuity"),
            Some(32)
        );
        assert_eq!(Errors::index_of("tau_J_range_violation"), Some(34));
        assert_eq!(
            Errors::index_of("base_acceleration_invalid_reading"),
            Some(40)
        );
    }

    #[test]
    fn display_matches_libfranka_format() {
        let mut flags = [false; ERROR_COUNT];
        assert_eq!(Errors(flags).to_string(), "[]");
        flags[3] = true;
        flags[25] = true;
        let errors = Errors(flags);
        assert_eq!(
            errors.to_string(),
            "[\"joint_velocity_violation\", \"communication_constraints_violation\"]"
        );
        assert!(errors.any());
        assert!(errors.get("joint_velocity_violation"));
        assert!(!errors.get("joint_reflex"));
        assert!(!errors.get("no_such_error"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialises_as_the_set_names() {
        let mut flags = [false; ERROR_COUNT];
        flags[6] = true;
        flags[40] = true;
        let errors = Errors(flags);
        let json = serde_json::to_string(&errors).unwrap();
        assert_eq!(
            json,
            "[\"joint_reflex\",\"base_acceleration_invalid_reading\"]"
        );
        assert_eq!(serde_json::from_str::<Errors>(&json).unwrap(), errors);
        assert_eq!(
            serde_json::from_str::<Errors>("[]").unwrap(),
            Errors::default()
        );
        let error = serde_json::from_str::<Errors>("[\"no_such_error\"]").unwrap_err();
        assert!(error.to_string().contains("no_such_error"), "{error}");
    }
}
