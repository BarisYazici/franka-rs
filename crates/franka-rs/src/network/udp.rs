//! UDP state/command channel.
//!
//! Port of `Network::udpReceive`, `Network::udpBlockingReceive` and `Network::udpSend` from
//! libfranka 0.21.2 (`src/network.h`).
//!
//! Like libfranka the socket is bound to `0.0.0.0:0` and the server address is *learned* from
//! the datagrams it sends (`Poco::DatagramSocket::receiveFrom` writes into
//! `udp_server_address_`, which `udpSend` then uses). The real robot sends its states from port
//! 1337, franka-sim from an ephemeral port; remembering the source address handles both.

use std::mem::MaybeUninit;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, SocketAddrV4, SocketAddrV6, UdpSocket};
use std::os::fd::{AsRawFd, RawFd};
use std::sync::Mutex;
use std::time::Duration;

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

use crate::error::{FrankaError, FrankaResult};

/// Default UDP receive timeout (libfranka's `Network` default `udp_timeout`).
pub const DEFAULT_UDP_TIMEOUT: Duration = Duration::from_secs(1);

/// The client's UDP socket for robot states and robot commands.
///
/// Neither [`UdpChannel::try_receive`] nor [`UdpChannel::send`] allocates, so both are safe to
/// call from a 1 kHz control loop.
#[derive(Debug)]
pub struct UdpChannel {
    socket: UdpSocket,
    port: u16,
    /// Source address of the most recently received datagram
    /// (libfranka's `udp_server_address_`).
    peer: Mutex<Option<SocketAddr>>,
}

impl UdpChannel {
    /// Binds `0.0.0.0:0` and sets the receive timeout, as `franka::Network`'s constructor does.
    pub fn bind(timeout: Duration) -> FrankaResult<UdpChannel> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))
            .map_err(|e| FrankaError::Network(format!("libfranka: {e}")))?;
        socket
            .set_read_timeout(Some(timeout))
            .map_err(|e| FrankaError::Network(format!("libfranka: {e}")))?;
        let port = socket
            .local_addr()
            .map_err(|e| FrankaError::Network(format!("libfranka: {e}")))?
            .port();
        Ok(UdpChannel {
            socket,
            port,
            peer: Mutex::new(None),
        })
    }

    /// The local port, which is announced to the server in the `Connect` request.
    pub fn port(&self) -> u16 {
        self.port
    }

    /// Source address of the last received datagram, if any.
    pub fn peer(&self) -> Option<SocketAddr> {
        *self.peer.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Reads one datagram if one is already queued, without blocking and without allocating.
    ///
    /// Uses `recvfrom(2)` with `MSG_DONTWAIT` rather than toggling `O_NONBLOCK` around every
    /// call, so a 1 kHz control loop performs exactly one syscall per poll. Returns `Ok(None)`
    /// when no datagram is waiting (libfranka's `udpReceive` returning `false`).
    ///
    /// # Note
    /// This raw byte API does **not** check the datagram length. libfranka's
    /// `udpBlockingReceiveUnsafe` rejects `bytes_received != sizeof(T)` with
    /// `ProtocolException("libfranka: incorrect object size")` (`network.h:140-142`); here that
    /// is the caller's job. Parsing a short datagram against a full-size buffer would silently
    /// read stale bytes, so prefer [`UdpChannel::try_receive_struct`], which performs the
    /// check.
    pub fn try_receive(&self, buf: &mut [u8]) -> FrankaResult<Option<usize>> {
        match self.recvfrom(buf, true) {
            Ok((n, from)) => {
                self.remember(from);
                Ok(Some(n))
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                Ok(None)
            }
            Err(e) => Err(FrankaError::Network(format!("libfranka: UDP receive: {e}"))),
        }
    }

    /// Blocks until a datagram arrives or the receive timeout expires.
    ///
    /// A timeout is reported as `Network("libfranka: UDP receive: Timeout")`, which is the text
    /// libfranka produces from `"libfranka: UDP receive: "s + Poco::TimeoutException::what()`.
    ///
    /// # Note
    /// Like [`UdpChannel::try_receive`], this raw byte API does not check the datagram length;
    /// use [`UdpChannel::blocking_receive_struct`] to get libfranka's
    /// `"libfranka: incorrect object size"` check.
    pub fn blocking_receive(&self, buf: &mut [u8]) -> FrankaResult<usize> {
        match self.recvfrom(buf, false) {
            Ok((n, from)) => {
                self.remember(from);
                Ok(n)
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                Err(FrankaError::Network(
                    "libfranka: UDP receive: Timeout".to_string(),
                ))
            }
            Err(e) => Err(FrankaError::Network(format!("libfranka: UDP receive: {e}"))),
        }
    }

    /// Reads one datagram as a `T` if one is already queued, without blocking.
    ///
    /// Typed counterpart of [`UdpChannel::try_receive`] and the port of
    /// `Network::udpReceive<T>` (`network.h:117-125`): the datagram is read into a `T`-sized
    /// stack buffer (no allocation, so this is control-loop safe) and its length must be
    /// exactly `size_of::<T>()`, otherwise
    /// `Protocol("libfranka: incorrect object size")` is returned (`network.h:140-142`).
    ///
    /// A datagram longer than `size_of::<T>()` is truncated by `recvfrom` and therefore looks
    /// like an exact-size read — the same blind spot libfranka has.
    pub fn try_receive_struct<T>(&self) -> FrankaResult<Option<T>>
    where
        T: FromBytes + IntoBytes + Immutable + KnownLayout,
    {
        let mut value = T::new_zeroed();
        match self.try_receive(value.as_mut_bytes())? {
            None => Ok(None),
            Some(n) if n == std::mem::size_of::<T>() => Ok(Some(value)),
            Some(_) => Err(FrankaError::Protocol(
                "libfranka: incorrect object size".to_string(),
            )),
        }
    }

    /// Blocks until a datagram arrives and returns it as a `T`.
    ///
    /// Typed counterpart of [`UdpChannel::blocking_receive`] and the port of
    /// `Network::udpBlockingReceive<T>` (`network.h:128-132`), including the
    /// `Protocol("libfranka: incorrect object size")` check on the datagram length.
    pub fn blocking_receive_struct<T>(&self) -> FrankaResult<T>
    where
        T: FromBytes + IntoBytes + Immutable + KnownLayout,
    {
        let mut value = T::new_zeroed();
        let received = self.blocking_receive(value.as_mut_bytes())?;
        if received != std::mem::size_of::<T>() {
            return Err(FrankaError::Protocol(
                "libfranka: incorrect object size".to_string(),
            ));
        }
        Ok(value)
    }

    /// Sends `data` to `addr`, which is normally [`UdpChannel::peer`].
    ///
    /// A short write is reported as `Network("libfranka: could not send UDP data")`, matching
    /// `udpSend`.
    pub fn send(&self, addr: SocketAddr, data: &[u8]) -> FrankaResult<()> {
        let sent = self
            .socket
            .send_to(data, addr)
            .map_err(|e| FrankaError::Network(format!("libfranka: UDP send: {e}")))?;
        if sent != data.len() {
            return Err(FrankaError::Network(
                "libfranka: could not send UDP data".to_string(),
            ));
        }
        Ok(())
    }

    fn remember(&self, from: SocketAddr) {
        *self.peer.lock().unwrap_or_else(|e| e.into_inner()) = Some(from);
    }

    /// `recvfrom(2)` into `buf`, retrying on `EINTR`.
    fn recvfrom(&self, buf: &mut [u8], non_blocking: bool) -> std::io::Result<(usize, SocketAddr)> {
        let fd: RawFd = self.socket.as_raw_fd();
        let flags = if non_blocking { libc::MSG_DONTWAIT } else { 0 };
        let mut storage = MaybeUninit::<libc::sockaddr_storage>::zeroed();
        loop {
            let mut len = std::mem::size_of::<libc::sockaddr_storage>() as libc::socklen_t;
            let n = unsafe {
                libc::recvfrom(
                    fd,
                    buf.as_mut_ptr().cast(),
                    buf.len(),
                    flags,
                    storage.as_mut_ptr().cast(),
                    &mut len,
                )
            };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                if e.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            let addr = unsafe { socket_addr_from_storage(storage.assume_init_ref()) }.ok_or_else(
                || {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "unsupported address family",
                    )
                },
            )?;
            return Ok((n as usize, addr));
        }
    }
}

/// Converts a `sockaddr_storage` filled by `recvfrom` into a [`SocketAddr`] without allocating.
///
/// # Safety
/// `storage` must have been filled by a successful `recvfrom`.
unsafe fn socket_addr_from_storage(storage: &libc::sockaddr_storage) -> Option<SocketAddr> {
    match storage.ss_family as libc::c_int {
        libc::AF_INET => {
            let addr = &*(storage as *const libc::sockaddr_storage as *const libc::sockaddr_in);
            Some(SocketAddr::V4(SocketAddrV4::new(
                Ipv4Addr::from(u32::from_be(addr.sin_addr.s_addr)),
                u16::from_be(addr.sin_port),
            )))
        }
        libc::AF_INET6 => {
            let addr = &*(storage as *const libc::sockaddr_storage as *const libc::sockaddr_in6);
            Some(SocketAddr::V6(SocketAddrV6::new(
                Ipv6Addr::from(addr.sin6_addr.s6_addr),
                u16::from_be(addr.sin6_port),
                u32::from_be(addr.sin6_flowinfo),
                addr.sin6_scope_id,
            )))
        }
        _ => None,
    }
}

/// Convenience: the loopback address of `port`, used when a test needs an explicit peer.
pub fn loopback(port: u16) -> SocketAddr {
    SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn learns_the_peer_and_sends_back() {
        let channel = UdpChannel::bind(Duration::from_millis(200)).unwrap();
        let server = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        server
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();

        let mut buf = [0u8; 64];
        assert_eq!(channel.try_receive(&mut buf).unwrap(), None);
        assert_eq!(channel.peer(), None);

        server
            .send_to(b"state", (Ipv4Addr::LOCALHOST, channel.port()))
            .unwrap();

        // The datagram may take a moment to be queued.
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        let received = loop {
            if let Some(n) = channel.try_receive(&mut buf).unwrap() {
                break n;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "datagram never arrived"
            );
        };
        assert_eq!(&buf[..received], b"state");
        assert_eq!(channel.peer(), Some(server.local_addr().unwrap()));

        channel.send(channel.peer().unwrap(), b"command").unwrap();
        let mut back = [0u8; 64];
        let (n, from) = server.recv_from(&mut back).unwrap();
        assert_eq!(&back[..n], b"command");
        assert_eq!(from.port(), channel.port());
    }

    #[test]
    fn typed_receive_checks_the_datagram_size() {
        use crate::wire::gripper::GripperState;
        use zerocopy::little_endian::{F64, U16, U32};
        use zerocopy::IntoBytes;

        let channel = UdpChannel::bind(Duration::from_secs(2)).unwrap();
        let server = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let target = (Ipv4Addr::LOCALHOST, channel.port());

        assert!(channel
            .try_receive_struct::<GripperState>()
            .unwrap()
            .is_none());

        // A short datagram is rejected with libfranka's ProtocolException text.
        server.send_to(&[1u8, 2, 3], target).unwrap();
        let error = channel
            .blocking_receive_struct::<GripperState>()
            .unwrap_err();
        assert_eq!(error.to_string(), "libfranka: incorrect object size");
        assert!(matches!(error, FrankaError::Protocol(_)));

        // An exact-size datagram parses.
        let state = GripperState {
            message_id: U32::new(7),
            width: F64::new(0.05),
            max_width: F64::new(0.08),
            is_grasped: 1,
            temperature: U16::new(21),
        };
        server.send_to(state.as_bytes(), target).unwrap();
        let received = channel.blocking_receive_struct::<GripperState>().unwrap();
        assert_eq!(received.message_id.get(), 7);
        assert_eq!(received.width.get(), 0.05);
        assert_eq!(received.is_grasped, 1);
        assert_eq!(received.temperature.get(), 21);
    }

    #[test]
    fn blocking_receive_reports_libfranka_timeout_text() {
        let channel = UdpChannel::bind(Duration::from_millis(50)).unwrap();
        let mut buf = [0u8; 64];
        let error = channel.blocking_receive(&mut buf).unwrap_err();
        assert_eq!(error.to_string(), "libfranka: UDP receive: Timeout");
        assert!(matches!(error, FrankaError::Network(_)));
    }

    #[test]
    fn blocking_receive_returns_a_datagram() {
        let channel = UdpChannel::bind(Duration::from_secs(2)).unwrap();
        let server = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let port = channel.port();
        let sender = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            server
                .send_to(b"hello", (Ipv4Addr::LOCALHOST, port))
                .unwrap();
        });

        let mut buf = [0u8; 64];
        let n = channel.blocking_receive(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"hello");
        sender.join().unwrap();
    }
}
