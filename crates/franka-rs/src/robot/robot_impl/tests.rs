//! Unit tests for the deviation constant and the exception-text formatting.

use super::exception::format_default_float;
use super::*;

#[test]
fn default_deviation_matches_libfranka() {
    assert_eq!(DEFAULT_DEVIATION.0, 10.0);
    assert_eq!(DEFAULT_DEVIATION.1, 3.12);
    assert_eq!(DEFAULT_DEVIATION.2, 2.0 * std::f64::consts::PI);
}

#[test]
fn ostream_style_float_formatting() {
    assert_eq!(format_default_float(0.0), "0");
    assert_eq!(format_default_float(1.0), "1");
    assert_eq!(format_default_float(0.5), "0.5");
    assert_eq!(format_default_float(0.987654321), "0.987654");
    assert_eq!(format_default_float(123456.0), "123456");
    assert_eq!(format_default_float(1234567.0), "1.23457e+06");
    assert_eq!(format_default_float(-0.25), "-0.25");
}

#[test]
fn control_exception_text_without_reflex_is_the_bare_message() {
    let exception = create_control_exception(
        "libfranka: Move command preempted!",
        MoveStatus::Preempted,
        Errors::default(),
        Vec::new(),
    );
    assert_eq!(exception.message, "libfranka: Move command preempted!");
    assert_eq!(exception.move_status, Some(MoveStatus::Preempted));
}

#[test]
fn control_exception_text_with_reflex_lists_errors_and_success_rate() {
    let mut flags = [false; crate::errors::ERROR_COUNT];
    flags[3] = true;
    let errors = Errors(flags);

    let first = RobotState {
        time: Duration::from_millis(10),
        control_command_success_rate: 0.5,
        ..RobotState::default()
    };
    let second = RobotState {
        time: Duration::from_millis(13),
        ..RobotState::default()
    };

    let log = vec![
        Record {
            state: first,
            command: None,
        },
        Record {
            state: second,
            command: None,
        },
    ];
    let exception = create_control_exception(
        "libfranka: Move command aborted: motion aborted by reflex!",
        MoveStatus::ReflexAborted,
        errors,
        log,
    );
    assert_eq!(
        exception.message,
        "libfranka: Move command aborted: motion aborted by reflex! \
         [\"joint_velocity_violation\"]\ncontrol_command_success_rate: 0.49 packets lost in a \
         row in the last sample: 2"
    );
    assert!(exception.last_motion_errors.get("joint_velocity_violation"));
}
