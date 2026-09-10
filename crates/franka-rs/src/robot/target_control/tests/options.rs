//! Validation of the two interfaces' options.

use super::super::*;
use super::is_invalid_argument;
use crate::otg::OtgLimits;
use crate::rate_limiting;
use crate::wire::robot::codec::FciVersion;
use crate::ControllerMode;

#[test]
fn default_options_are_valid_and_documented() {
    let cartesian = TargetControlOptions::default();
    assert!(cartesian.validate().is_ok());
    assert_eq!(cartesian.limits.max_velocity, 0.3);
    assert_eq!(cartesian.limits.max_acceleration, 0.5);
    assert_eq!(cartesian.limits.max_jerk, 20.0);
    assert_eq!(cartesian.rotation_limits.max_velocity, 0.5);
    assert_eq!(cartesian.rotation_limits.max_acceleration, 1.0);
    assert_eq!(cartesian.rotation_limits.max_jerk, 20.0);
    assert_eq!(
        cartesian.backend,
        Backend::Impedance(ImpedanceOptions::cartesian())
    );
    assert_eq!(cartesian.max_deviation, 0.30);
    assert_eq!(cartesian.max_angular_deviation, 0.5);
    assert_eq!(cartesian.settle, Settle::default());
    assert!(cartesian.limit_rate);
    assert_eq!(cartesian.realtime_priority, None);
    assert!(cartesian.observer.is_none());
    let debug = format!("{cartesian:?}");
    assert!(debug.contains("observer: false") && debug.contains("backend: Impedance("));

    let joint = JointTargetControlOptions::default();
    assert!(joint.validate().is_ok());
    assert_eq!(joint.limits, None);
    assert_eq!(joint.backend, Backend::Impedance(ImpedanceOptions::joint()));
    assert_eq!(joint.max_deviation, 1.0);
    assert!(joint.limit_rate);
    let robot = JointTargetControlOptions::default().with_backend(Backend::RobotController);
    assert!(robot.validate().is_ok());
    assert!(format!("{robot:?}").contains("backend: RobotController"));
}

#[test]
fn the_backend_is_validated_with_the_options() {
    let bad = ImpedanceOptions::cartesian().with_cutoff_frequency(0.0);
    let cartesian = TargetControlOptions::default().with_backend(Backend::Impedance(bad));
    assert!(is_invalid_argument(
        cartesian.validate(),
        "cutoff_frequency"
    ));
    let joint = JointTargetControlOptions::default().with_backend(Backend::Impedance(bad));
    assert!(is_invalid_argument(joint.validate(), "cutoff_frequency"));
    let mut gains = ImpedanceGains::JOINT;
    gains.joint_stiffness[0] = -1.0;
    let joint = JointTargetControlOptions::default().with_backend(Backend::Impedance(
        ImpedanceOptions::joint().with_gains(gains),
    ));
    assert!(is_invalid_argument(joint.validate(), "gains"));
}

#[test]
fn cartesian_options_reject_bad_fields() {
    let bad_limits = TargetControlOptions::default().with_limits(OtgLimits {
        max_velocity: 0.0,
        max_acceleration: 0.5,
        max_jerk: 20.0,
    });
    assert!(is_invalid_argument(bad_limits.validate(), "otg: limits"));
    let bad_rotation = TargetControlOptions::default().with_rotation_limits(OtgLimits {
        max_velocity: 0.5,
        max_acceleration: f64::NAN,
        max_jerk: 20.0,
    });
    assert!(is_invalid_argument(
        bad_rotation.validate(),
        "rotation_limits"
    ));
    let deviation = TargetControlOptions::default().with_max_deviation(f64::NAN);
    assert!(is_invalid_argument(deviation.validate(), "max_deviation"));
    let angular = TargetControlOptions::default().with_max_angular_deviation(0.0);
    assert!(is_invalid_argument(
        angular.validate(),
        "max_angular_deviation"
    ));
    let settle = TargetControlOptions::default().with_settle(Settle {
        tolerance: 1e-3,
        cycles: 0,
    });
    assert!(is_invalid_argument(settle.validate(), "settle"));
    let settle = TargetControlOptions::default().with_settle(Settle {
        tolerance: -1.0,
        cycles: 10,
    });
    assert!(is_invalid_argument(settle.validate(), "settle"));
    for priority in [0, 100, -5] {
        let options = TargetControlOptions::default().with_realtime_priority(Some(priority));
        assert!(is_invalid_argument(options.validate(), "realtime_priority"));
    }
    let fine = TargetControlOptions::default()
        .with_realtime_priority(Some(80))
        .with_controller_mode(ControllerMode::JointImpedance)
        .with_limit_rate(false)
        .with_observer(|_, _| {});
    assert!(fine.validate().is_ok());
    assert!(fine.observer.is_some());
}

#[test]
fn joint_options_reject_bad_fields() {
    let mut limits = JointTargetControlOptions::scaled_limits(FciVersion::V10, 0.2);
    limits[6].max_jerk = f64::INFINITY;
    let bad = JointTargetControlOptions::default().with_limits(limits);
    assert!(is_invalid_argument(bad.validate(), "otg: limits"));
    let deviation = JointTargetControlOptions::default().with_max_deviation(0.0);
    assert!(is_invalid_argument(deviation.validate(), "max_deviation"));
    let priority = JointTargetControlOptions::default().with_realtime_priority(Some(200));
    assert!(is_invalid_argument(
        priority.validate(),
        "realtime_priority"
    ));
}

#[test]
fn scaled_joint_limits_follow_the_version() {
    let fr3 = JointTargetControlOptions::scaled_limits(FciVersion::V10, DEFAULT_LIMIT_FRACTION);
    assert!((fr3[0].max_velocity - 0.2 * 2.62).abs() < 1e-12);
    assert!((fr3[4].max_velocity - 0.2 * 5.26).abs() < 1e-12);
    assert!(
        (fr3[0].max_acceleration - 0.2 * rate_limiting::MAX_JOINT_ACCELERATION[0]).abs() < 1e-12
    );
    assert!((fr3[0].max_jerk - 0.2 * rate_limiting::MAX_JOINT_JERK[0]).abs() < 1e-12);

    let fer = JointTargetControlOptions::scaled_limits(FciVersion::V5, 0.5);
    for (i, limits) in fer.iter().enumerate() {
        let (v, a, j) = (
            rate_limiting::fer::MAX_JOINT_VELOCITY[i],
            rate_limiting::fer::MAX_JOINT_ACCELERATION[i],
            rate_limiting::fer::MAX_JOINT_JERK[i],
        );
        assert!((limits.max_velocity - 0.5 * v).abs() < 1e-12);
        assert!((limits.max_acceleration - 0.5 * a).abs() < 1e-12);
        assert!((limits.max_jerk - 0.5 * j).abs() < 1e-12);
    }
    for version in [FciVersion::V5, FciVersion::V10] {
        let options = JointTargetControlOptions::default()
            .with_limits(JointTargetControlOptions::scaled_limits(version, 0.2));
        assert!(options.validate().is_ok());
    }
}

#[test]
fn joint_targets_and_postures_outside_the_inset_limits_are_refused_naming_the_joint() {
    for version in [FciVersion::V5, FciVersion::V10] {
        let limits = joint_position_limits(version);
        let ready = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];
        assert!(check_joint_limits(&ready, &limits, "target").is_ok());
        // The inset boundary itself is allowed; a hair past it is not.
        let mut q = ready;
        q[0] = limits.1[0] - JOINT_LIMIT_INSET;
        assert!(check_joint_limits(&q, &limits, "target").is_ok());
        q[0] += 1e-9;
        assert!(is_invalid_argument(
            check_joint_limits(&q, &limits, "target"),
            "target puts joint 1"
        ));
        q = ready;
        q[3] = limits.0[3];
        assert!(is_invalid_argument(
            check_joint_limits(&q, &limits, "posture"),
            "posture puts joint 4"
        ));
        // The posture of an impedance backend is checked at the start, other backends have none.
        let impedance = ImpedanceOptions::cartesian().with_posture(Some(q));
        assert!(is_invalid_argument(
            check_posture(&Backend::Impedance(impedance), &limits),
            "joint 4"
        ));
        let impedance = ImpedanceOptions::cartesian().with_posture(Some(ready));
        assert!(check_posture(&Backend::Impedance(impedance), &limits).is_ok());
        assert!(check_posture(&Backend::Impedance(ImpedanceOptions::joint()), &limits).is_ok());
        assert!(check_posture(&Backend::RobotController, &limits).is_ok());
    }
}
