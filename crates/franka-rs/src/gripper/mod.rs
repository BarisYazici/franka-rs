//! Franka Hand gripper client.
//!
//! Port of libfranka 0.21.2 `include/franka/gripper.h`, `include/franka/gripper_state.h` and
//! `src/gripper.cpp`. [`Gripper`] maintains a network connection to the gripper, provides the
//! current gripper state via [`Gripper::read_once`], and executes the four gripper commands
//! (`homing`, `grasp`, `move_gripper`, `stop`).
//!
//! The gripper protocol reuses the FCI's TCP/UDP framing (see [`crate::network`]) with a
//! 10-byte command header and `u16` statuses (see [`crate::wire::gripper`]) instead of the
//! robot's 12-byte header and `u8` statuses.

use std::mem::size_of;

use zerocopy::{FromBytes, IntoBytes};

use crate::duration::Duration;
use crate::error::{FrankaError, FrankaResult};
use crate::network::{connect_gripper, Network};
use crate::wire::gripper::{
    GraspRequest, GripperCommand, GripperCommandResponse, GripperMoveRequest,
    GripperState as WireGripperState, GripperStatus,
};
use crate::wire::{parse_response, HeaderLayout};

/// Describes the gripper state (`franka::GripperState`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperState {
    /// Current gripper opening width, in metres.
    pub width: f64,
    /// Maximum gripper opening width, in metres. Estimated by [`Gripper::homing`]; see
    /// `franka::GripperState::max_width`.
    pub max_width: f64,
    /// Whether an object is currently grasped.
    pub is_grasped: bool,
    /// Current gripper temperature, in degrees Celsius.
    pub temperature: u16,
    /// Strictly monotonically increasing timestamp since the gripper server started.
    pub time: Duration,
}

impl From<&WireGripperState> for GripperState {
    /// Port of the anonymous `convertGripperState` in `src/gripper.cpp`.
    fn from(wire: &WireGripperState) -> Self {
        GripperState {
            width: wire.width.get(),
            max_width: wire.max_width.get(),
            is_grasped: wire.is_grasped != 0,
            temperature: wire.temperature.get(),
            time: Duration::from_millis(u64::from(wire.message_id.get())),
        }
    }
}

/// Maintains a network connection to the gripper, provides the current gripper state, and
/// allows the execution of commands (port of `franka::Gripper`).
///
/// The underlying [`Network`] is `Sync` (see [`crate::network::TcpSession`] and
/// [`crate::network::UdpChannel`]), so the members of this struct are threadsafe, matching
/// libfranka's documented guarantee for `franka::Gripper` -- except that only one
/// [`Gripper::read_once`] should be in flight at a time, since concurrent callers would race to
/// claim the same UDP datagram.
#[derive(Debug)]
pub struct Gripper {
    network: Network,
    server_version: u16,
}

impl Gripper {
    /// Establishes a connection with a gripper connected to a robot.
    ///
    /// `franka_address` is the IP/hostname of the robot the gripper is connected to, as
    /// `"host"` or `"host:port"`; the gripper command port (1338) is used unless an explicit
    /// port is embedded in the address.
    ///
    /// # Errors
    /// Returns [`FrankaError::Network`] if the connection is unsuccessful and
    /// [`FrankaError::IncompatibleVersion`] if this crate's gripper protocol version is not
    /// supported by the server.
    pub fn new(franka_address: &str) -> FrankaResult<Gripper> {
        let (network, server_version) = connect_gripper(franka_address)?;
        Ok(Gripper {
            network,
            server_version,
        })
    }

    /// Returns the software version reported by the connected server.
    pub fn server_version(&self) -> u16 {
        self.server_version
    }

    /// Performs homing of the gripper.
    ///
    /// After changing the gripper fingers, a homing needs to be done. This is needed to
    /// estimate the maximum grasping width.
    ///
    /// # Errors
    /// Returns [`FrankaError::Command`] if the command failed or was aborted, and
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout.
    ///
    /// See [`GripperState::max_width`] for the maximum grasping width.
    pub fn homing(&self) -> FrankaResult<bool> {
        self.execute_command(GripperCommand::Homing, &[])
    }

    /// Grasps an object.
    ///
    /// An object is considered grasped if the distance `d` between the gripper fingers
    /// satisfies `(width - epsilon_inner) < d < (width + epsilon_outer)`.
    ///
    /// `epsilon_inner` and `epsilon_outer` default to `0.005` in libfranka's C++ overload; this
    /// port takes both explicitly (there is no default-argument mechanism in a `fn`).
    ///
    /// # Errors
    /// Returns [`FrankaError::Command`] if the command failed or was aborted, and
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout.
    pub fn grasp(
        &self,
        width: f64,
        speed: f64,
        force: f64,
        epsilon_inner: f64,
        epsilon_outer: f64,
    ) -> FrankaResult<bool> {
        let request = GraspRequest::new(width, epsilon_inner, epsilon_outer, speed, force);
        self.execute_command(GripperCommand::Grasp, request.as_bytes())
    }

    /// Moves the gripper fingers to a specified width.
    ///
    /// # Errors
    /// Returns [`FrankaError::Command`] if the command failed or was aborted, and
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout.
    pub fn move_gripper(&self, width: f64, speed: f64) -> FrankaResult<bool> {
        let request = GripperMoveRequest::new(width, speed);
        self.execute_command(GripperCommand::Move, request.as_bytes())
    }

    /// Stops a currently running gripper move or grasp.
    ///
    /// # Errors
    /// Returns [`FrankaError::Command`] if the command failed or was aborted, and
    /// [`FrankaError::Network`] if the connection is lost, e.g. after a timeout.
    pub fn stop(&self) -> FrankaResult<bool> {
        self.execute_command(GripperCommand::Stop, &[])
    }

    /// Waits for a gripper state update and returns it.
    ///
    /// Port of `Gripper::readOnce`: first drains every datagram already queued on the UDP
    /// socket (stale data from before this call), then blocks for one fresh state.
    ///
    /// # Errors
    /// Returns [`FrankaError::Network`] if the connection is lost, e.g. after a timeout.
    pub fn read_once(&self) -> FrankaResult<GripperState> {
        let mut buf = [0u8; size_of::<WireGripperState>()];
        // Delete old data from the UDP buffer.
        while self.network.udp.try_receive(&mut buf)?.is_some() {}

        let n = self.network.udp.blocking_receive(&mut buf)?;
        if n != buf.len() {
            return Err(FrankaError::Protocol(
                "libfranka: incorrect object size".to_string(),
            ));
        }
        let wire_state = WireGripperState::read_from_bytes(&buf)
            .map_err(|_| FrankaError::Protocol("libfranka: incorrect object size".to_string()))?;
        Ok(GripperState::from(&wire_state))
    }

    /// Sends a gripper command and maps the response status, exactly like the anonymous
    /// `executeCommand<T>` in `src/gripper.cpp`.
    fn execute_command(&self, command: GripperCommand, payload: &[u8]) -> FrankaResult<bool> {
        let command_id = self
            .network
            .tcp
            .send_request(command.to_u16() as u32, payload)?;
        let message = self.network.tcp.blocking_receive_response(command_id)?;
        let response: GripperCommandResponse = parse_response(HeaderLayout::Gripper, &message)?;

        match GripperStatus::from_u16(response.status.get()) {
            Some(GripperStatus::Success) => Ok(true),
            Some(GripperStatus::Fail) => Err(FrankaError::Command(
                "libfranka gripper: Command failed!".to_string(),
            )),
            Some(GripperStatus::Unsuccessful) => Ok(false),
            Some(GripperStatus::Aborted) => Err(FrankaError::Command(
                "libfranka gripper: Command aborted!".to_string(),
            )),
            None => Err(FrankaError::Protocol(
                "libfranka gripper: Unexpected response while handling command!".to_string(),
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream, UdpSocket};
    use std::time::Instant;

    use zerocopy::little_endian::U16;

    use crate::wire::gripper::{GripperConnectResponse, GripperConnectStatus, GRIPPER_VERSION};
    use crate::wire::GRIPPER_HEADER_LEN;

    /// Accepts a single connection on an ephemeral loopback port and runs `serve` on it,
    /// returning whatever `serve` returns.
    fn accept_and_serve<F, T>(serve: F) -> (SocketAddr, std::thread::JoinHandle<T>)
    where
        F: FnOnce(TcpStream) -> T + Send + 'static,
        T: Send + 'static,
    {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let handle = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            serve(stream)
        });
        (addr, handle)
    }

    /// Reads one gripper command message (header + payload) off `stream`.
    fn read_message(stream: &mut TcpStream) -> (u16, u32, Vec<u8>) {
        let mut header = [0u8; GRIPPER_HEADER_LEN];
        stream.read_exact(&mut header).unwrap();
        let (command, command_id, size) = HeaderLayout::Gripper.decode_header(&header).unwrap();
        let mut payload = vec![0u8; size as usize - GRIPPER_HEADER_LEN];
        stream.read_exact(&mut payload).unwrap();
        (command as u16, command_id, payload)
    }

    /// Writes one gripper response message.
    fn write_message(stream: &mut TcpStream, command: u16, command_id: u32, payload: &[u8]) {
        let mut message =
            HeaderLayout::Gripper.encode_header(command as u32, command_id, payload.len());
        message.extend_from_slice(payload);
        stream.write_all(&message).unwrap();
        stream.flush().unwrap();
    }

    /// Reads the `Connect` request, replies with `status`/`version`, and hands the (still open)
    /// stream to `then`.
    fn serve_connect<F, T>(
        mut stream: TcpStream,
        status: GripperConnectStatus,
        version: u16,
        then: F,
    ) -> T
    where
        F: FnOnce(TcpStream) -> T,
    {
        let (command, command_id, payload) = read_message(&mut stream);
        assert_eq!(command, GripperCommand::Connect.to_u16());
        assert_eq!(payload.len(), 4, "Connect::Request is 4 bytes");
        let response = GripperConnectResponse {
            status: U16::new(status.to_u16()),
            version: U16::new(version),
        };
        write_message(&mut stream, command, command_id, response.as_bytes());
        then(stream)
    }

    #[test]
    fn wire_gripper_state_is_23_bytes() {
        assert_eq!(size_of::<WireGripperState>(), 23);
    }

    #[test]
    fn connect_reports_server_version() {
        let (addr, server) = accept_and_serve(|stream| {
            serve_connect(
                stream,
                GripperConnectStatus::Success,
                GRIPPER_VERSION,
                |stream| {
                    std::thread::sleep(std::time::Duration::from_millis(50));
                    drop(stream);
                },
            );
        });

        let gripper = Gripper::new(&addr.to_string()).expect("handshake failed");
        assert_eq!(gripper.server_version(), GRIPPER_VERSION);

        drop(gripper);
        server.join().unwrap();
    }

    /// Port of `Gripper.ThrowsOnIncompatibleLibraryVersion` (`test/gripper_tests.cpp`).
    #[test]
    fn connect_reports_incompatible_version_like_libfranka() {
        let (addr, server) = accept_and_serve(|stream| {
            serve_connect(
                stream,
                GripperConnectStatus::IncompatibleLibraryVersion,
                7,
                |stream| {
                    std::thread::sleep(std::time::Duration::from_millis(50));
                    drop(stream);
                },
            );
        });

        let error = Gripper::new(&addr.to_string()).expect_err("expected a handshake failure");
        match error {
            FrankaError::IncompatibleVersion {
                server_version,
                library_version,
            } => {
                assert_eq!(server_version, 7);
                assert_eq!(library_version, GRIPPER_VERSION);
            }
            other => panic!("expected IncompatibleVersion, got {other:?}"),
        }
        assert_eq!(
            error.to_string(),
            format!(
                "libfranka: Incompatible library version (server version: 7, library version: {GRIPPER_VERSION})."
            )
        );

        server.join().unwrap();
    }

    /// Connects a `Gripper`, then answers exactly one subsequent command message with
    /// `raw_status`, returning the gripper and a handle that yields the command byte the mock
    /// server actually received.
    fn mock_command_server(raw_status: u16) -> (Gripper, std::thread::JoinHandle<u16>) {
        let (addr, server) = accept_and_serve(move |stream| {
            serve_connect(
                stream,
                GripperConnectStatus::Success,
                GRIPPER_VERSION,
                move |mut stream| {
                    let (command, command_id, _payload) = read_message(&mut stream);
                    let response = GripperCommandResponse {
                        status: U16::new(raw_status),
                    };
                    write_message(&mut stream, command, command_id, response.as_bytes());
                    std::thread::sleep(std::time::Duration::from_millis(50));
                    command
                },
            )
        });
        let gripper = Gripper::new(&addr.to_string()).expect("handshake failed");
        (gripper, server)
    }

    /// What a status-mapping subtest expects `execute_command`'s result to look like.
    enum Expect {
        Ok(bool),
        Command(&'static str),
        Protocol,
    }

    /// Port of the `GripperCommand<T>` typed tests in `test/gripper_command_tests.cpp`
    /// (`CanSendAndReceiveSuccess/Fail/Unsucessful/Aborted`), plus an "unrecognized status"
    /// case (not exercised by libfranka's own C++ tests, since every status its mock server
    /// can send is a valid enumerator).
    #[test]
    fn command_status_mapping() {
        type Run = fn(&Gripper) -> FrankaResult<bool>;
        let commands: [(&str, u16, Run); 4] = [
            ("homing", GripperCommand::Homing.to_u16(), |g| g.homing()),
            ("move", GripperCommand::Move.to_u16(), |g| {
                g.move_gripper(0.05, 0.1)
            }),
            ("grasp", GripperCommand::Grasp.to_u16(), |g| {
                g.grasp(0.05, 0.1, 400.0, 0.004, 0.005)
            }),
            ("stop", GripperCommand::Stop.to_u16(), |g| g.stop()),
        ];

        let cases: [(u16, Expect); 5] = [
            (GripperStatus::Success.to_u16(), Expect::Ok(true)),
            (
                GripperStatus::Fail.to_u16(),
                Expect::Command("libfranka gripper: Command failed!"),
            ),
            (GripperStatus::Unsuccessful.to_u16(), Expect::Ok(false)),
            (
                GripperStatus::Aborted.to_u16(),
                Expect::Command("libfranka gripper: Command aborted!"),
            ),
            (99, Expect::Protocol),
        ];

        for (name, expected_command, run) in commands {
            for (raw_status, expect) in &cases {
                let (gripper, server) = mock_command_server(*raw_status);
                let result = run(&gripper);
                let received_command = server.join().unwrap();
                assert_eq!(
                    received_command, expected_command,
                    "{name}: wrong command byte on the wire"
                );

                match expect {
                    Expect::Ok(expected) => {
                        assert_eq!(result.unwrap(), *expected, "{name}, status {raw_status}")
                    }
                    Expect::Command(message) => {
                        let error = result.unwrap_err();
                        assert_eq!(&error.to_string(), message, "{name}, status {raw_status}");
                        assert!(matches!(error, FrankaError::Command(_)));
                    }
                    Expect::Protocol => {
                        let error = result.unwrap_err();
                        assert_eq!(
                            error.to_string(),
                            "libfranka gripper: Unexpected response while handling command!",
                            "{name}, status {raw_status}"
                        );
                        assert!(matches!(error, FrankaError::Protocol(_)));
                    }
                }

                drop(gripper);
            }
        }
    }

    /// Builds a wire `GripperState` datagram for the mock UDP server below.
    fn wire_state(message_id: u32, width: f64) -> WireGripperState {
        WireGripperState {
            message_id: zerocopy::little_endian::U32::new(message_id),
            width: zerocopy::little_endian::F64::new(width),
            max_width: zerocopy::little_endian::F64::new(0.08),
            is_grasped: 0,
            temperature: U16::new(25),
        }
    }

    #[test]
    fn read_once_drains_queued_datagrams_then_blocks_for_a_fresh_one() {
        // Serves the Connect handshake by hand (rather than via `serve_connect`) so the mock
        // server can read the client's UDP port out of the request payload.
        let (addr, server) = accept_and_serve(|mut stream| {
            let (command, command_id, payload) = read_message(&mut stream);
            assert_eq!(command, GripperCommand::Connect.to_u16());
            assert_eq!(payload.len(), 4, "Connect::Request is 4 bytes");
            let udp_port = u16::from_le_bytes([payload[2], payload[3]]);

            let response = GripperConnectResponse {
                status: U16::new(GripperConnectStatus::Success.to_u16()),
                version: U16::new(GRIPPER_VERSION),
            };
            write_message(&mut stream, command, command_id, response.as_bytes());
            std::thread::sleep(std::time::Duration::from_millis(50));
            drop(stream);
            udp_port
        });

        let gripper = Gripper::new(&addr.to_string()).expect("handshake failed");
        let udp_port = server.join().unwrap();

        let udp_server = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let peer: SocketAddr = (Ipv4Addr::LOCALHOST, udp_port).into();

        // Queue three stale datagrams before ever calling read_once.
        for id in 1..=3u32 {
            udp_server
                .send_to(wire_state(id, 0.01).as_bytes(), peer)
                .unwrap();
        }
        std::thread::sleep(std::time::Duration::from_millis(50));

        // A fresh datagram, sent only after read_once has had time to drain the stale ones and
        // start blocking.
        let sender = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(150));
            udp_server
                .send_to(wire_state(4, 0.02).as_bytes(), peer)
                .unwrap();
        });

        let started = Instant::now();
        let state = gripper.read_once().expect("read_once failed");
        let elapsed = started.elapsed();

        assert!((state.width - 0.02).abs() < 1e-9, "got {}", state.width);
        assert_eq!(state.time.as_millis(), 4);
        assert!(
            elapsed >= std::time::Duration::from_millis(100),
            "read_once returned after {elapsed:?}; it should have blocked for the fresh datagram \
             instead of returning stale data"
        );

        sender.join().unwrap();
        drop(gripper);
    }
}
