//! End-to-end checks of the FCI v5 (Franka Emika Robot, FER) client path against the franka-sim
//! simulator running `--protocol v5 --robot panda`.
//!
//! Run under the machine-wide simulator lock:
//!
//! ```text
//! flock .sim.lock cargo test -p franka-rs --test sim_v5_handshake
//! ```
//!
//! The image comes from [`SimConfig::fer_v5`]: `$FRANKA_SIM_FER_IMAGE`, else
//! `franka-sim:panda-v5`. `FRANKA_SIM_IMAGE` names the **FR3** image and is deliberately
//! ignored here; the FER image's build recipe is kept privately, outside this
//! repository. Without it these tests fail rather than skip.

mod common;

use std::time::Duration;

use franka::error::FrankaError;
use franka::{FciVersion, VersionPolicy};
use franka_sim_test::SimConfig;

/// An FER `RobotState` datagram, in bytes (`context-shared.md`, "FCI v5 facts").
const V5_STATE_LEN: usize = 2373;

/// (a) the `Connect` handshake reports version 5, (b) the state stream is 2373-byte datagrams
/// with contiguous `message_id`s, (c) `Robot` negotiates FCI v5 and (d) rejects an `Exact(V10)`
/// policy, (e) `read_once` parses, and (f) the v5-only commands behave.
///
/// All of these share one container and one FCI client slot, so they run inside a single test
/// and each session is dropped before the next one connects.
#[test]
fn v5_handshake_state_stream_and_commands() {
    let sim = common::sim(SimConfig::fer_v5());

    // (a) The handshake succeeds and the sim reports FCI version 5.
    // (b) 100 states of exactly 2373 bytes with `message_id` increasing by one.
    {
        let (network, version) =
            common::connect_network_with_version(&sim, 5).expect("v5 handshake failed");
        assert_eq!(version, 5);
        assert_eq!(network.server_ip.to_string(), "127.0.0.1");

        // One byte larger than a v5 state, so the assertion below is two-sided: a datagram
        // longer than 2373 bytes would otherwise be truncated by `recvfrom` and still report
        // exactly 2373.
        let mut buffer = [0u8; V5_STATE_LEN + 1];
        let received = common::read_state_bytes(&network, Duration::from_secs(5), &mut buffer)
            .expect("no state datagram arrived");
        assert_eq!(received, V5_STATE_LEN, "a v5 RobotState is 2373 bytes");
        let mut previous = u64::from_le_bytes(buffer[..8].try_into().unwrap());

        for index in 1..100 {
            let received = common::read_state_bytes(&network, Duration::from_secs(5), &mut buffer)
                .expect("state stream stopped");
            assert_eq!(received, V5_STATE_LEN, "state {index} was {received} bytes");
            let id = u64::from_le_bytes(buffer[..8].try_into().unwrap());
            assert_eq!(
                id,
                previous + 1,
                "state {index}: message_id jumped from {previous} to {id}"
            );
            previous = id;
        }
    }

    // (d) `Exact(V10)` never retries, so a v10 client on an FER is refused.
    {
        match common::connect_with_version(&sim, VersionPolicy::Exact(FciVersion::V10)) {
            Err(FrankaError::IncompatibleVersion {
                server_version: 5,
                library_version: 10,
            }) => {}
            other => panic!("expected IncompatibleVersion {{5, 10}}, got {other:?}"),
        }
    }

    // (c, e, f) The default `Auto` policy lands on FCI v5, and the session works.
    {
        let robot = common::connect(&sim);
        assert_eq!(robot.fci_version(), FciVersion::V5);
        assert_eq!(robot.server_version(), 5);

        let state = robot.read_once().expect("read_once failed");
        eprintln!("sim v5: robot_mode {:?}, q {:?}", state.robot_mode, state.q);
        assert_eq!(state.robot_mode, franka::RobotMode::Idle);
        assert!(
            state.q.iter().all(|value| value.is_finite()),
            "q is not finite: {:?}",
            state.q
        );

        // A setter that exists in both versions, under the v5 numbering (5, not 4).
        robot
            .set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])
            .expect("set_joint_impedance failed");

        // v5-only commands. What the simulator answers is recorded rather than assumed; see
        // the assertions below.
        let filters = robot.set_filters(100.0, 100.0, 100.0, 100.0, 100.0);
        eprintln!("sim v5: set_filters answered {filters:?}");
        filters.expect("set_filters failed");

        // `GetRobotModel` does not exist on FCI v5.
        match robot.robot_model() {
            Err(FrankaError::InvalidOperation(message)) => assert_eq!(
                message,
                "libfranka: Get Robot Model is not available on FCI version 5."
            ),
            other => panic!("expected InvalidOperation, got {other:?}"),
        }

        let wall = robot.virtual_wall(1);
        eprintln!("sim v5: virtual_wall(1) answered {wall:?}");
        assert!(
            wall.is_ok(),
            "virtual_wall(1) answered {wall:?}; update this assertion if the simulator's \
             behaviour changed"
        );

        // (g) `LoadModelLibrary`: the `.so` tail arithmetic (`header.size - 13`) is only
        // exercised against a real server here, so check that the downloaded library both
        // loads and evaluates. `Frame::Flange` sits 0.107 m along the last joint's z axis
        // (`franka_emika_panda` URDF, `panda_joint8`), which is the cheapest end-to-end proof
        // that the bound symbols return the FER's kinematics and not garbage.
        let model = robot
            .load_model_from_robot()
            .expect("load_model_from_robot failed on FCI v5");
        let state = robot.read_once().expect("read_once failed");
        let joint7 = model.pose(franka::Frame::Joint7, &state);
        let flange = model.pose(franka::Frame::Flange, &state);
        assert!(
            flange.iter().all(|value| value.is_finite()),
            "Frame::Flange pose is not finite: {flange:?}"
        );

        eprintln!("sim v5: Joint7 {joint7:?}\n        Flange {flange:?}");
        let offset = [
            flange[12] - joint7[12],
            flange[13] - joint7[13],
            flange[14] - joint7[14],
        ];
        let distance =
            (offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]).sqrt();
        assert!(
            (distance - 0.107).abs() < 1e-4,
            "Flange is {distance} m from Joint7, expected 0.107 m (offset {offset:?})"
        );
        // The offset is along joint 7's own z axis, i.e. the third column of its pose.
        let z_axis = [joint7[8], joint7[9], joint7[10]];
        let along_z = offset[0] * z_axis[0] + offset[1] * z_axis[1] + offset[2] * z_axis[2];
        assert!(
            (along_z - 0.107).abs() < 1e-4,
            "the Flange offset is {along_z} m along Joint7's z axis, expected 0.107 m"
        );
    }
}
