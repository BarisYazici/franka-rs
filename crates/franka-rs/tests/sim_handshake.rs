//! End-to-end checks of the wire and network layers against the franka-sim simulator, plus two
//! loopback-mock checks for behaviour the simulator does not exercise.
//!
//! Run with
//! `FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-rs --test sim_handshake`.

mod common;

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener};
use std::time::{Duration, Instant};

use franka::error::FrankaError;
use franka::network::{connect_handshake, Network};
use franka::wire::robot::{
    Command, CommandResponse, CommandStatus, ConnectResponse, ConnectStatus, Deviation,
    MoveControllerMode, MoveMotionGeneratorMode, MoveRequest, RobotState as WireRobotState,
};
use franka::wire::{message_payload, parse_response, HeaderLayout, ROBOT_COMMAND_PORT};
use franka_sim_test::SimConfig;
use zerocopy::IntoBytes;

/// (a) `Connect` handshake, (b) 100 UDP states at 1 kHz, (c) `GetRobotModel`, and the
/// simulator half of (d).
///
/// All of these share one container and one FCI client slot, so they run inside a single test
/// and each session is dropped before the next one connects.
#[test]
fn handshake_state_stream_and_robot_model() {
    let sim = common::sim(SimConfig::nominal());

    // (a) The handshake succeeds and the sim reports FCI version 10.
    {
        let (network, version) = common::connect_network(&sim).expect("handshake failed");
        assert_eq!(version, franka::wire::ROBOT_VERSION);
        assert_eq!(version, 10);
        assert_eq!(network.server_ip.to_string(), "127.0.0.1");
        assert_ne!(network.udp.port(), 0);
    }

    // (b) The state stream is contiguous and runs at 1 kHz.
    {
        let (network, _) = common::connect_network(&sim).expect("handshake failed");

        // The first datagram may arrive a little after the handshake; time from the *first*
        // state so the measured span covers 99 cycles of streaming only. `read_state` uses
        // `UdpChannel::blocking_receive_struct`, which enforces libfranka's
        // "incorrect object size" check on every datagram.
        let first: WireRobotState = common::read_state(&network, Duration::from_secs(5)).unwrap();
        let mut previous = first.message_id.get();
        let started = Instant::now();

        for index in 1..100 {
            let state: WireRobotState =
                common::read_state(&network, Duration::from_secs(5)).unwrap();
            let id = state.message_id.get();
            assert_eq!(
                id,
                previous + 1,
                "state {index}: message_id jumped from {previous} to {id}"
            );
            previous = id;

            // The conversion must accept every state the sim produces.
            let converted = franka::robot_state::RobotState::from_wire(&state);
            assert_eq!(converted.time.as_millis(), id);
            assert!(converted.q.iter().all(|v| v.is_finite()));
        }

        let elapsed = started.elapsed();
        assert!(
            elapsed >= Duration::from_millis(80) && elapsed <= Duration::from_millis(250),
            "99 states took {elapsed:?}, expected 80..250 ms"
        );
    }

    // (c) GetRobotModel returns the FR3 URDF.
    {
        let (network, _) = common::connect_network(&sim).expect("handshake failed");
        let command_id = network
            .tcp
            .send_request(Command::GetRobotModel.to_u32(), &[])
            .unwrap();
        let message = network.tcp.blocking_receive_response(command_id).unwrap();

        // GetRobotModel keeps `CommandBase::Status`, so its byte is a `CommandStatus`, not a
        // `GetterSetterStatus` (they only agree on 0 and 1).
        let response: CommandResponse = parse_response(HeaderLayout::Robot, &message).unwrap();
        assert_eq!(
            CommandStatus::from_u8(response.status),
            Some(CommandStatus::Success)
        );

        let urdf = std::str::from_utf8(&message_payload(HeaderLayout::Robot, &message)[1..])
            .expect("URDF is not valid UTF-8");
        assert!(
            urdf.starts_with("<?xml"),
            "URDF starts with {:?}",
            &urdf[..16]
        );
        assert!(urdf.contains("<robot name=\"fr3\""));
    }

    // (d, simulator half) The sim accepts an older library version — it does not check it.
    {
        let (_network, version) =
            common::connect_network_with_version(&sim, 9).expect("handshake with version 9 failed");
        assert_eq!(version, 10);
    }
}

/// (d) A server that rejects the library version produces
/// [`FrankaError::IncompatibleVersion`] with libfranka's exact message.
///
/// franka-sim always answers `kSuccess`, so this is checked against a loopback mock.
#[test]
fn incompatible_library_version_is_reported_like_libfranka() {
    let (addr, server) = mock_server(|mut stream| {
        let mut request = [0u8; 16];
        stream.read_exact(&mut request).unwrap();
        assert_eq!(
            HeaderLayout::Robot.decode_header(&request),
            Some((0, 0, 16))
        );

        let response = ConnectResponse::new(ConnectStatus::IncompatibleLibraryVersion, 10);
        let mut message = HeaderLayout::Robot.encode_header(
            Command::Connect.to_u32(),
            0,
            response.as_bytes().len(),
        );
        message.extend_from_slice(response.as_bytes());
        stream.write_all(&message).unwrap();
        stream.flush().unwrap();
        std::thread::sleep(Duration::from_millis(200));
    });

    let network = Network::connect(&addr.to_string(), ROBOT_COMMAND_PORT, HeaderLayout::Robot)
        .expect("mock connect failed");
    let error = connect_handshake(&network, 9).unwrap_err();
    assert!(matches!(
        error,
        FrankaError::IncompatibleVersion {
            server_version: 10,
            library_version: 9
        }
    ));
    assert_eq!(
        error.to_string(),
        "libfranka: Incompatible library version (server version: 10, library version: 9)."
    );

    drop(network);
    server.join().unwrap();
}

/// (e) A `Move` request occupies exactly 113 payload bytes on the wire, i.e. 125 bytes
/// including the 12-byte header, the way libfranka sends it.
#[test]
fn move_request_is_113_bytes_on_the_wire() {
    let (addr, server) = mock_server(|mut stream| {
        // Answer the Connect handshake first.
        let mut request = [0u8; 16];
        stream.read_exact(&mut request).unwrap();
        let response = ConnectResponse::new(ConnectStatus::Success, 10);
        let mut message = HeaderLayout::Robot.encode_header(
            Command::Connect.to_u32(),
            0,
            response.as_bytes().len(),
        );
        message.extend_from_slice(response.as_bytes());
        stream.write_all(&message).unwrap();
        stream.flush().unwrap();

        // Then read the Move message and report what arrived.
        let mut header = [0u8; 12];
        stream.read_exact(&mut header).unwrap();
        let (command, command_id, size) = HeaderLayout::Robot.decode_header(&header).unwrap();
        assert_eq!(command, Command::Move.to_u32());
        assert_eq!(command_id, 1);
        assert_eq!(size, 125, "Move message must be 12 + 113 bytes");
        let mut payload = vec![0u8; size as usize - 12];
        stream.read_exact(&mut payload).unwrap();
        assert_eq!(payload.len(), 113);
        assert_eq!(&payload[0..4], &2u32.to_le_bytes()); // ExternalController
        assert_eq!(&payload[4..8], &1u32.to_le_bytes()); // JointVelocity
        assert_eq!(payload[56], 0); // use_async_motion_generator
        std::thread::sleep(Duration::from_millis(200));
    });

    let network = Network::connect(&addr.to_string(), ROBOT_COMMAND_PORT, HeaderLayout::Robot)
        .expect("mock connect failed");
    assert_eq!(connect_handshake(&network, 10).unwrap(), 10);

    let request = MoveRequest::new(
        MoveControllerMode::ExternalController,
        MoveMotionGeneratorMode::JointVelocity,
        Deviation::new(0.1, 0.1, 0.1),
        Deviation::new(0.2, 0.2, 0.2),
    );
    assert_eq!(request.as_bytes().len(), 113);
    network
        .tcp
        .send_request(Command::Move.to_u32(), request.as_bytes())
        .unwrap();

    drop(network);
    server.join().unwrap();
}

/// Accepts a single connection on an ephemeral loopback port and runs `serve` on it.
fn mock_server<F>(serve: F) -> (SocketAddr, std::thread::JoinHandle<()>)
where
    F: FnOnce(std::net::TcpStream) + Send + 'static,
{
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let handle = std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        serve(stream);
    });
    (addr, handle)
}
