//! libfranka's `createControlException`, including the `std::ostream` number formatting
//! that makes the exception texts byte-identical to the C++ ones.

use super::*;

/// Port of the anonymous-namespace `createControlException` in `robot_impl.cpp`.
pub(crate) fn create_control_exception(
    message: &str,
    move_status: MoveStatus,
    reflex_errors: Errors,
    log: Vec<Record>,
) -> ControlException {
    let mut text = String::from(message);
    if move_status == MoveStatus::ReflexAborted {
        text.push(' ');
        text.push_str(&reflex_errors.to_string());

        if log.len() >= 2 {
            // Count number of lost packets in the last and before last packets.
            let last = log[log.len() - 1].state.time.as_millis();
            let before_last = log[log.len() - 2].state.time.as_millis();
            let lost_packets = last.saturating_sub(before_last).saturating_sub(1);
            // Read second to last control_command_success_rate since the last one will always be
            // zero and consider in the success rate assuming all previous packets were lost.
            let rate = log[log.len() - 2].state.control_command_success_rate
                * (1.0 - lost_packets as f64 / 100.0);
            text.push('\n');
            text.push_str("control_command_success_rate: ");
            text.push_str(&format_default_float(rate));
            if lost_packets > 0 {
                text.push_str(&format!(
                    " packets lost in a row in the last sample: {lost_packets}"
                ));
            }
        }
    }

    ControlException {
        message: text,
        move_status: Some(move_status),
        last_motion_errors: reflex_errors,
        log,
    }
}

/// Formats a double the way a default-configured `std::ostream` does (`%g` with six significant
/// digits), so the exception texts are byte-identical to libfranka's.
pub(super) fn format_default_float(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    if !value.is_finite() {
        return if value.is_nan() {
            "nan".to_string()
        } else if value > 0.0 {
            "inf".to_string()
        } else {
            "-inf".to_string()
        };
    }
    let exponent = value.abs().log10().floor() as i32;
    if !(-4..6).contains(&exponent) {
        let mantissa = format!("{:.5e}", value);
        // Rust writes `1.23456e-5`, C++ writes `1.23456e-05`.
        let (mantissa, exp) = mantissa.split_once('e').expect("scientific format");
        let mantissa = trim_zeros(mantissa);
        let (sign, digits) = match exp.strip_prefix('-') {
            Some(rest) => ('-', rest),
            None => ('+', exp),
        };
        format!("{mantissa}e{sign}{digits:0>2}")
    } else {
        let decimals = (5 - exponent).max(0) as usize;
        trim_zeros(&format!("{value:.decimals$}")).to_string()
    }
}

/// Drops trailing zeros (and a trailing dot) from a fixed-point representation.
fn trim_zeros(text: &str) -> &str {
    if !text.contains('.') {
        return text;
    }
    text.trim_end_matches('0').trim_end_matches('.')
}
