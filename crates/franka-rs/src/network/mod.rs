//! The two sockets of an FCI session and the `Connect` handshake.
//!
//! Port of `franka::Network` (libfranka 0.21.2 `src/network.{h,cpp}`) split into a TCP command
//! channel ([`TcpSession`]) and a UDP state/command channel ([`UdpChannel`]).

pub mod tcp;
pub mod udp;

use std::net::{IpAddr, SocketAddr, ToSocketAddrs};
use std::time::Duration;

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

use crate::error::{FrankaError, FrankaResult};
use crate::wire::gripper::{
    GripperConnectRequest, GripperConnectResponse, GripperConnectStatus, GRIPPER_VERSION,
};
use crate::wire::robot::{ConnectRequest, ConnectResponse, ConnectStatus};
use crate::wire::{parse_response, HeaderLayout};

pub use tcp::{TcpSession, DEFAULT_CONNECT_TIMEOUT, POLL_TIMEOUT};
pub use udp::{UdpChannel, DEFAULT_UDP_TIMEOUT};

/// A connected FCI session: the command socket, the state socket, and the server's IP.
#[derive(Debug)]
pub struct Network {
    /// TCP command channel.
    pub tcp: TcpSession,
    /// UDP state/command channel.
    pub udp: UdpChannel,
    /// IP address the command socket is connected to.
    pub server_ip: IpAddr,
}

impl Network {
    /// Connects to `host` and binds the UDP socket, exactly like `franka::Network`'s
    /// constructor. `host` may be `"host"` or `"host:port"`; an explicit port overrides
    /// `tcp_port`.
    ///
    /// `layout` selects the 12-byte robot or the 10-byte gripper header for TCP framing.
    pub fn connect(host: &str, tcp_port: u16, layout: HeaderLayout) -> FrankaResult<Network> {
        Network::connect_with_timeouts(
            host,
            tcp_port,
            layout,
            DEFAULT_CONNECT_TIMEOUT,
            DEFAULT_UDP_TIMEOUT,
        )
    }

    /// [`Network::connect`] with explicit timeouts (libfranka's `tcp_timeout` / `udp_timeout`).
    pub fn connect_with_timeouts(
        host: &str,
        tcp_port: u16,
        layout: HeaderLayout,
        tcp_timeout: Duration,
        udp_timeout: Duration,
    ) -> FrankaResult<Network> {
        let addr = resolve(host, tcp_port)?;
        let tcp = TcpSession::connect(addr, tcp_timeout, layout)?;
        let udp = UdpChannel::bind(udp_timeout)?;
        Ok(Network {
            tcp,
            udp,
            server_ip: addr.ip(),
        })
    }

    /// Source address of the last state datagram, i.e. where robot commands must be sent.
    pub fn udp_peer(&self) -> Option<SocketAddr> {
        self.udp.peer()
    }

    /// Reads one state datagram as a `T` if one is already queued, without blocking.
    ///
    /// Pass-through to [`UdpChannel::try_receive_struct`] that additionally shuts the command
    /// socket down when the receive fails at the network level, mirroring the
    /// `tcp_socket_.shutdown()` in libfranka's `udpBlockingReceiveUnsafe` catch block
    /// (`network.h:146`). A size mismatch is *not* a network failure — libfranka's
    /// `ProtocolException` is thrown inside the `try` and is not caught by that handler — so
    /// the socket stays up in that case, exactly as in the C++.
    pub fn try_receive_struct<T>(&self) -> FrankaResult<Option<T>>
    where
        T: FromBytes + IntoBytes + Immutable + KnownLayout,
    {
        self.shutdown_tcp_on_network_error(self.udp.try_receive_struct())
    }

    /// Blocks until a state datagram arrives and returns it as a `T`.
    ///
    /// Pass-through to [`UdpChannel::blocking_receive_struct`] with the same
    /// shut-down-the-TCP-socket-on-network-failure behaviour as
    /// [`Network::try_receive_struct`]. A receive timeout counts as a network failure, which is
    /// what makes the next TCP command fail fast instead of blocking on a dead session.
    pub fn blocking_receive_struct<T>(&self) -> FrankaResult<T>
    where
        T: FromBytes + IntoBytes + Immutable + KnownLayout,
    {
        self.shutdown_tcp_on_network_error(self.udp.blocking_receive_struct())
    }

    /// Reads one datagram into `buf` if one is already queued, without blocking, and returns how
    /// many bytes it held.
    ///
    /// Byte-slice counterpart of [`Network::try_receive_struct`], with the same
    /// shut-down-the-TCP-socket-on-network-failure behaviour. The length check is the caller's
    /// job: the version-aware state reader in [`crate::robot`] passes a buffer sized for the
    /// *largest* supported `RobotState` and compares the returned length against the negotiated
    /// version's size, so a datagram of the other version is rejected instead of being silently
    /// truncated.
    pub fn try_receive_bytes(&self, buf: &mut [u8]) -> FrankaResult<Option<usize>> {
        self.shutdown_tcp_on_network_error(self.udp.try_receive(buf))
    }

    /// Blocks until a datagram arrives, reads it into `buf` and returns its length.
    ///
    /// Byte-slice counterpart of [`Network::blocking_receive_struct`]; see
    /// [`Network::try_receive_bytes`] for why the length check is left to the caller.
    pub fn blocking_receive_bytes(&self, buf: &mut [u8]) -> FrankaResult<usize> {
        self.shutdown_tcp_on_network_error(self.udp.blocking_receive(buf))
    }

    fn shutdown_tcp_on_network_error<T>(&self, result: FrankaResult<T>) -> FrankaResult<T> {
        if let Err(FrankaError::Network(_)) = &result {
            self.tcp.shutdown();
        }
        result
    }

    /// Sends a robot command to the address the last state came from.
    ///
    /// Fails before the first state has been received, which cannot happen in a control loop:
    /// libfranka's `udp_server_address_` is likewise only valid after the first
    /// `receiveFrom`.
    pub fn send_udp(&self, data: &[u8]) -> FrankaResult<()> {
        let peer = self.udp_peer().ok_or_else(|| {
            FrankaError::Network(
                "libfranka: UDP send: no state received yet, server address unknown".to_string(),
            )
        })?;
        self.udp.send(peer, data)
    }
}

/// Resolves `"host"` or `"host:port"` to a single socket address, preferring the port embedded
/// in the string.
fn resolve(host: &str, default_port: u16) -> FrankaResult<SocketAddr> {
    if let Ok(mut addrs) = host.to_socket_addrs() {
        if let Some(addr) = addrs.next() {
            return Ok(addr);
        }
    }
    let mut addrs = (host, default_port)
        .to_socket_addrs()
        .map_err(|e| FrankaError::Network(format!("libfranka: Connection error: {e}")))?;
    addrs.next().ok_or_else(|| {
        FrankaError::Network(format!(
            "libfranka: Connection error: host {host} did not resolve to any address"
        ))
    })
}

/// Performs the `Connect` handshake and returns the server's protocol version.
///
/// Port of the `franka::connect<T, kLibraryVersion>` template in `src/network.h`: send the
/// request carrying the client's UDP port, block for the reply, and map the status.
/// `kIncompatibleLibraryVersion` becomes [`FrankaError::IncompatibleVersion`], any other
/// non-success status becomes `Protocol("libfranka: Protocol error during connection attempt")`.
///
/// The robot and the gripper differ only in the width of the status field (`u8` vs `u16`), so
/// the layout chosen at connect time selects the right response struct.
pub fn connect_handshake(network: &Network, library_version: u16) -> FrankaResult<u16> {
    use zerocopy::IntoBytes;

    let layout = network.tcp.layout();
    let udp_port = network.udp.port();

    // The robot's `Connect::Status` (`u8`) and the gripper's (`u16`) are two distinct enums
    // that happen to agree on their first two values today; each response is therefore parsed
    // against *its own* enum and only the outcome is shared.
    let (status, server_version) = match layout {
        HeaderLayout::Robot => {
            let request = ConnectRequest::new(library_version, udp_port);
            let command_id = network.tcp.send_request(
                crate::wire::robot::Command::Connect.to_u32(),
                request.as_bytes(),
            )?;
            let message = network.tcp.blocking_receive_response(command_id)?;
            let response: ConnectResponse = parse_response(layout, &message)?;
            let status = match ConnectStatus::from_u8(response.status) {
                Some(ConnectStatus::Success) => HandshakeStatus::Success,
                Some(ConnectStatus::IncompatibleLibraryVersion) => {
                    HandshakeStatus::IncompatibleLibraryVersion
                }
                None => HandshakeStatus::Other,
            };
            (status, response.version.get())
        }
        HeaderLayout::Gripper => {
            let request = GripperConnectRequest::new(library_version, udp_port);
            let command_id = network.tcp.send_request(
                crate::wire::gripper::GripperCommand::Connect.to_u16() as u32,
                request.as_bytes(),
            )?;
            let message = network.tcp.blocking_receive_response(command_id)?;
            let response: GripperConnectResponse = parse_response(layout, &message)?;
            let status = match GripperConnectStatus::from_u16(response.status.get()) {
                Some(GripperConnectStatus::Success) => HandshakeStatus::Success,
                Some(GripperConnectStatus::IncompatibleLibraryVersion) => {
                    HandshakeStatus::IncompatibleLibraryVersion
                }
                None => HandshakeStatus::Other,
            };
            (status, response.version.get())
        }
    };

    match status {
        HandshakeStatus::Success => Ok(server_version),
        HandshakeStatus::IncompatibleLibraryVersion => Err(FrankaError::IncompatibleVersion {
            server_version,
            library_version,
        }),
        HandshakeStatus::Other => Err(FrankaError::Protocol(
            "libfranka: Protocol error during connection attempt".to_string(),
        )),
    }
}

/// The outcome of a `Connect` handshake, shared by the robot's and the gripper's own status
/// enums so that neither is compared against the other's values.
enum HandshakeStatus {
    Success,
    IncompatibleLibraryVersion,
    Other,
}

/// Connects to a robot and performs the `Connect` handshake, returning the session and the
/// server's FCI version.
pub fn connect_robot(host: &str) -> FrankaResult<(Network, u16)> {
    let network = Network::connect(host, crate::wire::ROBOT_COMMAND_PORT, HeaderLayout::Robot)?;
    let version = connect_handshake(&network, crate::wire::ROBOT_VERSION)?;
    Ok((network, version))
}

/// Connects to a gripper and performs the `Connect` handshake, returning the session and the
/// server's gripper protocol version.
pub fn connect_gripper(host: &str) -> FrankaResult<(Network, u16)> {
    let network = Network::connect(
        host,
        crate::wire::gripper::GRIPPER_COMMAND_PORT,
        HeaderLayout::Gripper,
    )?;
    let version = connect_handshake(&network, GRIPPER_VERSION)?;
    Ok((network, version))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::gripper::GripperState;

    /// A UDP receive failure must shut the command socket down, as
    /// `Network::udpBlockingReceiveUnsafe` does (`network.h:146`), so the next TCP operation
    /// fails immediately instead of hanging on a session the robot has abandoned.
    #[test]
    fn udp_failure_shuts_down_the_command_socket() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let server = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            std::thread::sleep(Duration::from_millis(500));
            drop(stream);
        });

        let network = Network::connect_with_timeouts(
            &addr.to_string(),
            crate::wire::ROBOT_COMMAND_PORT,
            HeaderLayout::Robot,
            Duration::from_secs(1),
            Duration::from_millis(50),
        )
        .unwrap();
        assert!(network.tcp.send_request(0, &[]).is_ok());

        let error = network
            .blocking_receive_struct::<GripperState>()
            .unwrap_err();
        assert_eq!(error.to_string(), "libfranka: UDP receive: Timeout");
        assert!(network.tcp.send_request(0, &[]).is_err());

        drop(network);
        server.join().unwrap();
    }
}
