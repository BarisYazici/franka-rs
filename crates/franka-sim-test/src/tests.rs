//! Unit tests for the configuration mapping (no Docker needed).

use super::*;

/// [`readiness_probe`] must send/expect the wire version and
/// `RobotState` size that actually match the container's protocol —
/// never the v5 shape against a v10 server or vice versa. This test needs no Docker;
/// it only checks the `Protocol` → (wire version, expected state size)
/// mapping the probe relies on.
#[test]
fn protocol_wire_version_and_expected_state_len_mapping() {
    assert_eq!(Protocol::V10.wire_version(), 10);
    assert_eq!(Protocol::V10.expected_robot_state_len(), 1377);

    assert_eq!(Protocol::V5.wire_version(), 5);
    assert_eq!(Protocol::V5.expected_robot_state_len(), 2373);

    // The two protocols must never share a wire version or state size —
    // otherwise a probe could "succeed" against the wrong server.
    assert_ne!(Protocol::V10.wire_version(), Protocol::V5.wire_version());
    assert_ne!(
        Protocol::V10.expected_robot_state_len(),
        Protocol::V5.expected_robot_state_len()
    );
}
