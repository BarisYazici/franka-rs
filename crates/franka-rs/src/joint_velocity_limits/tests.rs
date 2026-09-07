//! Unit tests for the URDF parser and the position-dependent velocity envelope.

use super::*;
use crate::rate_limiting::{
    compute_lower_limits_joint_velocity, compute_upper_limits_joint_velocity,
};

/// The seven FR3 joints of `reference/libfranka/test/fr3.urdf`, verbatim (that file is not
/// checked in, so the fixture is inlined here; `parses_the_real_fr3_urdf` below checks the
/// real file whenever it is available).
const FR3_URDF: &str = r#"<?xml version="1.0" ?>
<robot name="fr3">
  <joint name="base_joint" type="fixed">
<origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <joint name="link1_accelerometer_top_joint" type="fixed">
<origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <joint name="joint1" type="revolute">
<limit effort="87.0" lower="-2.750100" upper="2.75010" velocity="2.62"/>
<safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.9007" soft_upper_limit="2.9007"/>
<position_based_velocity_limits deceleration_limit="6.0" velocity_offset="0.30"/>
  </joint>
  <joint name="joint2" type="revolute">
<limit effort="87.0" lower="-1.791800" upper="1.79180" velocity="2.62"/>
<position_based_velocity_limits deceleration_limit="2.585" velocity_offset="0.20"/>
  </joint>
  <joint name="joint3" type="revolute">
<limit effort="87.0" lower="-2.906500" upper="2.90650" velocity="2.62"/>
<position_based_velocity_limits deceleration_limit="3.50" velocity_offset="0.20"/>
  </joint>
  <joint name="joint4" type="revolute">
<limit effort="87.0" lower="-3.048100" upper="-0.1458" velocity="2.62"/>
<position_based_velocity_limits deceleration_limit="4.00" velocity_offset="0.30"/>
  </joint>
  <joint name="joint5" type="revolute">
<limit effort="12.0" lower="-2.810100" upper="2.81010" velocity="5.26"/>
<position_based_velocity_limits deceleration_limit="17.0" velocity_offset="0.35"/>
  </joint>
  <joint name="joint6" type="revolute">
<limit effort="12.0" lower="0.54092" upper="4.52050" velocity="4.18"/>
<position_based_velocity_limits deceleration_limit="5.5" velocity_offset="0.35"/>
  </joint>
  <joint name="joint7" type="revolute">
<limit effort="12.0" lower="-3.019600" upper="3.01960" velocity="5.26"/>
<position_based_velocity_limits deceleration_limit="17.0" velocity_offset="0.35"/>
  </joint>
  <joint name="joint8" type="fixed">
<origin rpy="0 0 0" xyz="0 0 0.107"/>
  </joint>
</robot>
"#;

fn message(error: FrankaError) -> String {
    match error {
        FrankaError::InvalidArgument(msg) => msg,
        other => panic!("expected InvalidArgument, got {other:?}"),
    }
}

// ---- TEST(JointVelocityLimits, ParseConfigFromFR3URDF) ----
#[test]
fn parse_config_from_fr3_urdf() {
    let config = JointVelocityLimitsConfig::from_urdf(FR3_URDF).unwrap();
    let tolerance = 1e-4;
    let near = |a: f64, b: f64| (a - b).abs() < tolerance;

    assert!(near(config[0].max_velocity, 2.62));
    assert!(near(config[0].velocity_offset, 0.30));
    assert!(near(config[0].deceleration_limit, 6.0));
    assert!(near(config[0].upper_position_limit, 2.7501));
    assert!(near(config[0].lower_position_limit, -2.7501));

    assert!(near(config[1].max_velocity, 2.62));
    assert!(near(config[1].velocity_offset, 0.20));
    assert!(near(config[1].deceleration_limit, 2.585));
    assert!(near(config[1].upper_position_limit, 1.7918));
    assert!(near(config[1].lower_position_limit, -1.7918));

    assert!(near(config[4].max_velocity, 5.26));
    assert!(near(config[4].velocity_offset, 0.35));
    assert!(near(config[4].deceleration_limit, 17.0));
    assert!(near(config[4].upper_position_limit, 2.81009));
    assert!(near(config[4].lower_position_limit, -2.81009));

    assert!(near(config[6].max_velocity, 5.26));
    assert!(near(config[6].velocity_offset, 0.35));
    assert!(near(config[6].deceleration_limit, 17.0));
    assert!(near(config[6].upper_position_limit, 3.0196));
    assert!(near(config[6].lower_position_limit, -3.0196));

    // getJointParams returns the same data as operator[]
    for i in 0..NUM_JOINTS {
        assert_eq!(config.joint_params()[i], config[i]);
    }
}

/// The URDF-driven limits must reproduce the hardcoded FR3 formulas of `rate_limiting.h`.
#[test]
fn fr3_urdf_limits_match_the_hardcoded_formulas() {
    let config = JointVelocityLimitsConfig::from_urdf(FR3_URDF).unwrap();
    let test_positions: [[f64; 7]; 4] = [
        [0.0; 7],
        [0.5, -0.3, 0.2, -1.5, 0.7, 1.9, -0.4],
        [2.7, 1.7, 2.9, -0.2, 2.8, 4.5, 3.0],
        [-2.7, -1.7, -2.9, -3.0, -2.8, 0.6, -3.0],
    ];
    for q in test_positions.iter() {
        let upper = config.upper_limits(q);
        let lower = config.lower_limits(q);
        let upper_reference = compute_upper_limits_joint_velocity(q);
        let lower_reference = compute_lower_limits_joint_velocity(q);
        for i in 0..NUM_JOINTS {
            assert!(
                (upper[i] - upper_reference[i]).abs() < 1e-12,
                "upper joint {i} at {q:?}: {} vs {}",
                upper[i],
                upper_reference[i]
            );
            assert!(
                (lower[i] - lower_reference[i]).abs() < 1e-12,
                "lower joint {i} at {q:?}: {} vs {}",
                lower[i],
                lower_reference[i]
            );
        }
    }
}

/// Checks the real `reference/libfranka/test/fr3.urdf` when the (git-ignored) libfranka
/// checkout is present.
#[test]
fn parses_the_real_fr3_urdf() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../reference/libfranka/test/fr3.urdf"
    );
    let Ok(urdf) = std::fs::read_to_string(path) else {
        eprintln!("skipping: {path} is not available");
        return;
    };
    let config = JointVelocityLimitsConfig::from_urdf(&urdf).unwrap();
    let fixture = JointVelocityLimitsConfig::from_urdf(FR3_URDF).unwrap();
    assert_eq!(config, fixture);

    let q = [0.0; 7];
    let upper_reference = compute_upper_limits_joint_velocity(&q);
    let lower_reference = compute_lower_limits_joint_velocity(&q);
    for i in 0..NUM_JOINTS {
        assert!((config.upper_limits(&q)[i] - upper_reference[i]).abs() < 1e-12);
        assert!((config.lower_limits(&q)[i] - lower_reference[i]).abs() < 1e-12);
    }
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFNewFormatIncomplete) ----
#[test]
fn parse_config_from_urdf_new_format_incomplete() {
    let incomplete_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="fr3v2_joint1" type="revolute">
    <limit effort="87.0" lower="-2.9007400166666666" upper="2.9007400166666666" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.6520000381679385" deceleration_limit="6.0"/>
  </joint>
  <joint name="fr3v2_joint2" type="revolute">
    <limit effort="87.0" lower="-1.8360900166666667" upper="1.8360900166666667" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.2499685851463105" deceleration_limit="2.585"/>
  </joint>
  <!-- Missing joints 3-7 -->
</robot>
  "#;
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf(incomplete_urdf).unwrap_err()),
        "Missing required joints: joint3 joint4 joint5 joint6 joint7 "
    );
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFMissingTags) ----
#[test]
fn parse_config_from_urdf_missing_tags() {
    let missing_tags_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="fr3v2_joint1" type="revolute">
    <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
    <!-- Missing position_based_velocity_limits -->
  </joint>
</robot>
  "#;
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf(missing_tags_urdf).unwrap_err()),
        "Missing <position_based_velocity_limits> element for joint: fr3v2_joint1"
    );

    let missing_limit_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="fr3v2_joint1" type="revolute">
    <position_based_velocity_limits velocity_offset="0.65" deceleration_limit="6.0"/>
  </joint>
</robot>
  "#;
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf(missing_limit_urdf).unwrap_err()),
        "Missing <limit> element for joint: fr3v2_joint1"
    );
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFMalformedXML) ----
#[test]
fn parse_config_from_urdf_malformed_xml() {
    let malformed_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="joint1" type="revolute">
    <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.65" deceleration_limit="6.0
    <!-- Missing closing tag for deceleration_limit and position_based_velocity_limits -->
  </joint>
</robot>
  "#;
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf(malformed_urdf).unwrap_err()),
        "Failed to parse URDF for joint velocity limits"
    );
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFEmptyString) ----
#[test]
fn parse_config_from_urdf_empty_string() {
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf("").unwrap_err()),
        "Failed to parse URDF for joint velocity limits"
    );
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFInvalidXMLSyntax) ----
#[test]
fn parse_config_from_urdf_invalid_xml_syntax() {
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf("This is not valid XML at all!").unwrap_err()),
        "Failed to parse URDF for joint velocity limits"
    );
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFMinimalValidXML) ----
#[test]
fn parse_config_from_urdf_minimal_valid_xml() {
    let minimal_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
</robot>
  "#;
    // Should fail because no joints are present
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf(minimal_urdf).unwrap_err()),
        "Missing required joints: joint1 joint2 joint3 joint4 joint5 joint6 joint7 "
    );
}

/// No `<robot>` root element.
#[test]
fn parse_config_from_urdf_without_robot_element() {
    assert_eq!(
        message(
            JointVelocityLimitsConfig::from_urdf("<?xml version=\"1.0\" ?><notrobot/>")
                .unwrap_err()
        ),
        "Failed to parse URDF: no <robot> element exists for joint velocity limits"
    );
}

// ---- TEST(JointVelocityLimits, DefaultConstructor) ----
#[test]
fn default_constructor() {
    let config = JointVelocityLimitsConfig::default();
    for i in 0..NUM_JOINTS {
        assert_eq!(config[i].max_velocity, 0.0);
        assert_eq!(config[i].velocity_offset, 0.0);
        assert_eq!(config[i].deceleration_limit, 0.0);
        assert_eq!(config[i].upper_position_limit, 0.0);
        assert_eq!(config[i].lower_position_limit, 0.0);
    }
}

// ---- TEST(JointVelocityLimits, ParseFromURDFMethodIncomplete) ----
// ---- TEST(JointVelocityLimits, GetJointParamsMethodIncomplete) ----
#[test]
fn parse_from_urdf_method_incomplete() {
    for (offset, decel, lower, upper, velocity) in [
        ("0.5", "8.0", "-2.5", "2.5", "3.0"),
        ("0.3", "5.0", "-1.5", "1.5", "2.5"),
    ] {
        let incomplete_urdf = format!(
            r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="fr3v2_joint1" type="revolute">
    <limit effort="87.0" lower="{lower}" upper="{upper}" velocity="{velocity}"/>
    <position_based_velocity_limits velocity_offset="{offset}" deceleration_limit="{decel}"/>
  </joint>
  <!-- Missing joints 2-7 -->
</robot>
  "#
        );
        assert_eq!(
            message(JointVelocityLimitsConfig::from_urdf(&incomplete_urdf).unwrap_err()),
            "Missing required joints: joint2 joint3 joint4 joint5 joint6 joint7 "
        );
    }
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFCompleteValidFormat) ----
#[test]
fn parse_config_from_urdf_complete_valid_format() {
    let complete_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="joint1" type="revolute">
    <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.30" deceleration_limit="6.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <limit effort="87.0" lower="-1.8" upper="1.8" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.20" deceleration_limit="2.585"/>
  </joint>
  <joint name="joint3" type="revolute">
    <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.25" deceleration_limit="5.0"/>
  </joint>
  <joint name="joint4" type="revolute">
    <limit effort="87.0" lower="-3.1" upper="0.1" velocity="2.62"/>
    <position_based_velocity_limits velocity_offset="0.15" deceleration_limit="4.0"/>
  </joint>
  <joint name="joint5" type="revolute">
    <limit effort="12.0" lower="-2.8" upper="2.8" velocity="5.26"/>
    <position_based_velocity_limits velocity_offset="0.35" deceleration_limit="17.0"/>
  </joint>
  <joint name="joint6" type="revolute">
    <limit effort="12.0" lower="-3.8" upper="0.8" velocity="5.26"/>
    <position_based_velocity_limits velocity_offset="0.25" deceleration_limit="12.0"/>
  </joint>
  <joint name="joint7" type="revolute">
    <limit effort="12.0" lower="-3.0" upper="3.0" velocity="5.26"/>
    <position_based_velocity_limits velocity_offset="0.35" deceleration_limit="17.0"/>
  </joint>
</robot>
  "#;
    let config = JointVelocityLimitsConfig::from_urdf(complete_urdf).unwrap();
    for i in 0..NUM_JOINTS {
        assert_eq!(config.joint_params()[i], config[i]);
    }
    assert_eq!(config[0].max_velocity, 2.62);
    assert_eq!(config[0].velocity_offset, 0.30);
    assert_eq!(config[0].deceleration_limit, 6.0);
    assert_eq!(config[6].max_velocity, 5.26);
    assert_eq!(config[6].velocity_offset, 0.35);
    assert_eq!(config[6].deceleration_limit, 17.0);
}

// ---- TEST(JointVelocityLimits, ParseConfigFromURDFMissingAttributes) ----
#[test]
fn parse_config_from_urdf_missing_attributes() {
    let missing_attr_urdf = r#"<?xml version="1.0" ?>
<robot name="test_robot">
  <joint name="joint1" type="revolute">
    <limit effort="87.0" lower="-2.9" upper="2.9"/>
    <!-- Missing velocity attribute -->
    <position_based_velocity_limits velocity_offset="0.30" deceleration_limit="6.0"/>
  </joint>
</robot>
  "#;
    assert_eq!(
        message(JointVelocityLimitsConfig::from_urdf(missing_attr_urdf).unwrap_err()),
        "Missing 'velocity' attribute in <limit> for joint: joint1"
    );
}

// ---- TEST(JointVelocityLimits, PositionBasedJointVelocityLimitConstantsConstructors) ----
#[test]
fn position_based_joint_velocity_limit_constants_constructors() {
    let default_params = PositionBasedJointVelocityLimitConstants::default();
    assert_eq!(default_params.max_velocity, 0.0);
    assert_eq!(default_params.velocity_offset, 0.0);
    assert_eq!(default_params.deceleration_limit, 0.0);
    assert_eq!(default_params.upper_position_limit, 0.0);
    assert_eq!(default_params.lower_position_limit, 0.0);

    let params = PositionBasedJointVelocityLimitConstants::new(2.5, 0.3, 5.0, 1.5, -1.5);
    assert_eq!(params.max_velocity, 2.5);
    assert_eq!(params.velocity_offset, 0.3);
    assert_eq!(params.deceleration_limit, 5.0);
    assert_eq!(params.upper_position_limit, 1.5);
    assert_eq!(params.lower_position_limit, -1.5);
}

#[test]
fn joint_names_are_matched_exactly_and_as_substrings() {
    assert_eq!(joint_index("joint1"), Some(0));
    assert_eq!(joint_index("joint7"), Some(6));
    assert_eq!(joint_index("fr3_joint4"), Some(3));
    assert_eq!(joint_index("fr3v2_joint2"), Some(1));
    assert_eq!(joint_index("joint8"), None);
    assert_eq!(joint_index("base_joint"), None);
    assert_eq!(joint_index("link1_accelerometer_top_joint"), None);
}
