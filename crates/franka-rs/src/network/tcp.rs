//! TCP command channel: framing plus command-id demultiplexing.
//!
//! Port of `Network::tcpSendRequest`, `Network::tcpReadFromBuffer`,
//! `Network::tcpReceiveResponse`, `Network::tcpBlockingReceiveResponse`,
//! `Network::isTcpSocketAlive` and `Network::tcpThrowIfConnectionClosed` from libfranka 0.21.2
//! (`src/network.h`, `src/network.cpp`).
//!
//! The FCI server answers commands out of order — a `Move` reply can arrive long after a
//! `StopMove` sent later — so every reply is stored under the `command_id` of its request until
//! the thread that issued that request picks it up. This is exactly libfranka's
//! `received_responses_` map.

use std::collections::HashMap;
use std::io::{ErrorKind, Write};
use std::net::{SocketAddr, TcpStream};
use std::os::fd::{AsRawFd, RawFd};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;
use std::time::Duration;

use crate::error::{FrankaError, FrankaResult};
use crate::wire::HeaderLayout;

/// Poll/read granularity of [`TcpSession::blocking_receive_response`]
/// (libfranka's `franka::kTimeout`).
pub const POLL_TIMEOUT: Duration = Duration::from_millis(10);

/// Default TCP connect/read timeout (libfranka's `Network` default `tcp_timeout`).
pub const DEFAULT_CONNECT_TIMEOUT: Duration = Duration::from_secs(1);

/// Upper bound on a single TCP message's announced `size`, so that a malformed or hostile peer
/// cannot make [`TcpSession::read_from_buffer`] allocate an unbounded amount of memory purely
/// from an untrusted header field.
///
/// libfranka has no such bound (it resizes `pending_response_` to `header.size` unconditionally),
/// but one costs nothing here: the largest legitimate message is the v5 model library at roughly
/// 330 KB, and a URDF is tens of KB, so 16 MiB leaves generous headroom.
const MAX_TCP_MESSAGE_SIZE: usize = 16 * 1024 * 1024;

/// Partially received message plus the completed messages waiting to be claimed.
///
/// Mirrors `pending_response_`, `pending_response_offset_`, `pending_command_id_` and
/// `received_responses_` in `franka::Network`.
#[derive(Debug, Default)]
struct Demux {
    /// Complete messages (header **and** payload), keyed by `command_id`.
    received: HashMap<u32, Vec<u8>>,
    /// Bytes of the message currently being assembled.
    pending: Vec<u8>,
    /// Total length of the pending message once its header has been parsed.
    pending_len: Option<usize>,
    /// `command_id` of the pending message.
    pending_id: u32,
}

/// A connected FCI command socket.
///
/// `TcpSession` is `Sync`: one thread may block in
/// [`TcpSession::blocking_receive_response`] while another calls
/// [`TcpSession::send_request`], which is how `Robot::stop()` interrupts a running motion in
/// libfranka.
#[derive(Debug)]
pub struct TcpSession {
    socket: TcpStream,
    layout: HeaderLayout,
    state: Mutex<Demux>,
    next_command_id: AtomicU32,
}

impl TcpSession {
    /// Connects to `addr` and configures the socket the way `franka::Network`'s constructor
    /// does: blocking, 1 s connect timeout, TCP keepalive with a 1 s idle time, 3 probes and a
    /// 1 s interval (best effort — libfranka swallows failures of the three `setOption` calls).
    ///
    /// The receive timeout is set to [`POLL_TIMEOUT`] so that a blocking receive wakes every
    /// 10 ms exactly like libfranka's `poll(kTimeout, SELECT_READ)`.
    ///
    /// Error texts are libfranka's, verbatim.
    pub fn connect(
        addr: SocketAddr,
        timeout: Duration,
        layout: HeaderLayout,
    ) -> FrankaResult<TcpSession> {
        let socket = TcpStream::connect_timeout(&addr, timeout).map_err(|e| match e.kind() {
            ErrorKind::ConnectionRefused => FrankaError::Network(
                "libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk."
                    .to_string(),
            ),
            ErrorKind::TimedOut => FrankaError::Network(
                "libfranka: Connection timeout. Please check your network connection or settings."
                    .to_string(),
            ),
            _ => FrankaError::Network(format!("libfranka: Connection error: {e}")),
        })?;

        socket
            .set_nodelay(true)
            .map_err(|e| FrankaError::Network(format!("libfranka: Connection error: {e}")))?;
        socket
            .set_read_timeout(Some(POLL_TIMEOUT))
            .map_err(|e| FrankaError::Network(format!("libfranka: Connection error: {e}")))?;
        socket
            .set_write_timeout(Some(timeout))
            .map_err(|e| FrankaError::Network(format!("libfranka: Connection error: {e}")))?;

        // Best effort, like libfranka's `try { setOption(...) } catch (...) {}`.
        let keepalive = socket2::TcpKeepalive::new()
            .with_time(Duration::from_secs(1))
            .with_interval(Duration::from_secs(1))
            .with_retries(3);
        let _ = socket2::SockRef::from(&socket).set_tcp_keepalive(&keepalive);

        Ok(TcpSession {
            socket,
            layout,
            state: Mutex::new(Demux::default()),
            next_command_id: AtomicU32::new(0),
        })
    }

    /// The header layout this session frames with.
    pub fn layout(&self) -> HeaderLayout {
        self.layout
    }

    /// Local address of the command socket (used by tests).
    pub fn local_addr(&self) -> FrankaResult<SocketAddr> {
        self.socket
            .local_addr()
            .map_err(|e| FrankaError::Network(format!("libfranka: {e}")))
    }

    /// Sends a request and returns its `command_id`.
    ///
    /// The header's `size` is `header_len + payload.len()`, i.e. it counts the header, exactly
    /// like `CommandHeader::size` in libfranka. Command ids start at 0 and increase by one per
    /// request (`Network::command_id_`).
    pub fn send_request(&self, command: u32, payload: &[u8]) -> FrankaResult<u32> {
        let command_id = self.next_command_id.fetch_add(1, Ordering::Relaxed);
        let mut message = self
            .layout
            .encode_header(command, command_id, payload.len());
        message.extend_from_slice(payload);

        let _guard = self.state.lock().unwrap_or_else(|e| e.into_inner());
        (&self.socket)
            .write_all(&message)
            .map_err(|e| FrankaError::Network(format!("libfranka: TCP send bytes: {e}")))?;
        Ok(command_id)
    }

    /// Blocks until the response with `command_id` has been received and returns the whole
    /// message, header included.
    ///
    /// Port of `Network::tcpBlockingReceiveResponse`: lock, read for at most 10 ms, look the id
    /// up, unlock, `std::this_thread::yield()`, repeat.
    pub fn blocking_receive_response(&self, command_id: u32) -> FrankaResult<Vec<u8>> {
        loop {
            {
                let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
                self.read_from_buffer(&mut state, false)?;
                if let Some(message) = state.received.remove(&command_id) {
                    return Ok(message);
                }
            }
            std::thread::yield_now();
        }
    }

    /// Tries to claim the response with `command_id` without blocking.
    ///
    /// Port of `Network::tcpReceiveResponse`: a `try_lock` that gives up immediately if another
    /// thread holds the socket, then a zero-timeout read. Returns `Ok(None)` when the response
    /// has not arrived yet.
    pub fn try_receive_response(&self, command_id: u32) -> FrankaResult<Option<Vec<u8>>> {
        let Ok(mut state) = self.state.try_lock() else {
            return Ok(None);
        };
        self.read_from_buffer(&mut state, true)?;
        Ok(state.received.remove(&command_id))
    }

    /// Whether the socket is free of pending errors (`Network::isTcpSocketAlive`).
    pub fn is_alive(&self) -> bool {
        !poll_error(self.socket.as_raw_fd())
    }

    /// Returns `Err(Network("libfranka: server closed connection"))` when the peer performed an
    /// orderly shutdown (`Network::tcpThrowIfConnectionClosed`).
    ///
    /// Like libfranka this is a no-op when another thread holds the socket lock, and it peeks
    /// rather than consuming, so a queued response is not lost.
    pub fn throw_if_connection_closed(&self) -> FrankaResult<()> {
        let Ok(_state) = self.state.try_lock() else {
            return Ok(());
        };
        if !poll_readable(self.socket.as_raw_fd()) {
            return Ok(());
        }
        let mut buffer = [0u8; 1];
        let fd = self.socket.as_raw_fd();
        let received = unsafe {
            libc::recv(
                fd,
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                libc::MSG_PEEK | libc::MSG_DONTWAIT,
            )
        };
        let errno = if received < 0 {
            std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
        } else {
            0
        };
        classify_peek(received, errno)
    }

    /// Shuts the command socket down in both directions, best effort.
    ///
    /// libfranka calls `tcp_socket_.shutdown()` whenever a UDP receive fails
    /// (`network.h:146`) so that the next TCP operation fails fast instead of waiting on a
    /// connection the robot has already given up on; [`crate::network::Network`] does the same.
    pub fn shutdown(&self) {
        let _ = self.socket.shutdown(std::net::Shutdown::Both);
    }

    /// One pass of `Network::tcpReadFromBuffer`.
    ///
    /// Reads at most the bytes still missing from the current message, so a message is never
    /// read into the next one. `non_blocking` selects between the socket's 10 ms receive
    /// timeout and `MSG_DONTWAIT` (libfranka's `kTimeout` vs `0us` poll).
    fn read_from_buffer(&self, state: &mut Demux, non_blocking: bool) -> FrankaResult<()> {
        let fd = self.socket.as_raw_fd();
        if poll_error(fd) {
            return Err(FrankaError::Network(
                "libfranka: TCP connection got interrupted.".to_string(),
            ));
        }

        let header_len = self.layout.header_len();
        let target = state.pending_len.unwrap_or(header_len);
        let missing = target - state.pending.len();
        debug_assert!(missing > 0);

        let start = state.pending.len();
        state.pending.resize(target, 0);
        let read = recv(fd, &mut state.pending[start..], non_blocking);
        let read = match read {
            Ok(n) => n,
            Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {
                state.pending.truncate(start);
                return Ok(());
            }
            Err(e) => {
                state.pending.truncate(start);
                return Err(FrankaError::Network(format!("libfranka: TCP receive: {e}")));
            }
        };
        state.pending.truncate(start + read);

        if read == 0 {
            // Poco reports the orderly shutdown as a zero-byte read; libfranka detects it in
            // `tcpThrowIfConnectionClosed`. Reporting it here as well keeps a blocking receive
            // from spinning forever on a closed socket.
            return Err(FrankaError::Network(
                "libfranka: server closed connection".to_string(),
            ));
        }

        if state.pending_len.is_none() && state.pending.len() == header_len {
            let (_command, command_id, size) = self
                .layout
                .decode_header(&state.pending)
                .expect("header_len bytes are available");
            if (size as usize) < header_len || (size as usize) > MAX_TCP_MESSAGE_SIZE {
                state.pending.clear();
                return Err(FrankaError::Protocol(
                    "libfranka: Incorrect TCP message size.".to_string(),
                ));
            }
            state.pending_len = Some(size as usize);
            state.pending_id = command_id;
            state.pending.reserve(size as usize - header_len);
        }

        if Some(state.pending.len()) == state.pending_len {
            let message = std::mem::take(&mut state.pending);
            state.received.insert(state.pending_id, message);
            state.pending_len = None;
            state.pending_id = 0;
        }
        Ok(())
    }
}

impl Drop for TcpSession {
    /// `franka::Network::~Network` shuts the socket down; dropping the `TcpStream` closes it.
    fn drop(&mut self) {
        let _ = self.socket.shutdown(std::net::Shutdown::Both);
    }
}

/// `recv(2)` with `MSG_DONTWAIT` when `non_blocking`, retrying on `EINTR`.
fn recv(fd: RawFd, buf: &mut [u8], non_blocking: bool) -> std::io::Result<usize> {
    let flags = if non_blocking { libc::MSG_DONTWAIT } else { 0 };
    loop {
        let n = unsafe { libc::recv(fd, buf.as_mut_ptr().cast(), buf.len(), flags) };
        if n < 0 {
            let e = std::io::Error::last_os_error();
            if e.kind() == ErrorKind::Interrupted {
                continue;
            }
            return Err(e);
        }
        return Ok(n as usize);
    }
}

/// `poll(fd, 0)` checking for `POLLERR`/`POLLHUP`/`POLLNVAL`
/// (Poco's `Socket::SELECT_ERROR`).
fn poll_error(fd: RawFd) -> bool {
    poll_once(fd, 0) & (libc::POLLERR | libc::POLLNVAL) != 0
}

/// Turns the result of the `MSG_PEEK | MSG_DONTWAIT` probe into libfranka's verdict on the
/// connection.
///
/// `0` is an orderly shutdown by the server. `-1` is `EAGAIN`/`EWOULDBLOCK` on a healthy but
/// empty socket, or `EINTR` if a signal arrived -- neither says anything about the connection
/// -- but any other errno is a real socket error, which libfranka's Asio `available()` reports
/// by throwing rather than by returning "still connected".
fn classify_peek(received: isize, errno: i32) -> FrankaResult<()> {
    if received == 0 {
        return Err(FrankaError::Network(
            "libfranka: server closed connection".to_string(),
        ));
    }
    if received < 0 {
        let error = std::io::Error::from_raw_os_error(errno);
        if !matches!(
            error.kind(),
            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted
        ) {
            return Err(FrankaError::Network(format!(
                "libfranka: Connection error: {error}"
            )));
        }
    }
    Ok(())
}

/// `poll(fd, 0)` checking for `POLLIN` (Poco's `Socket::SELECT_READ`).
fn poll_readable(fd: RawFd) -> bool {
    poll_once(fd, libc::POLLIN) & libc::POLLIN != 0
}

fn poll_once(fd: RawFd, events: libc::c_short) -> libc::c_short {
    let mut pollfd = libc::pollfd {
        fd,
        events,
        revents: 0,
    };
    let rc = unsafe { libc::poll(&mut pollfd, 1, 0) };
    if rc <= 0 {
        return 0;
    }
    pollfd.revents
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;
    use std::net::TcpListener;

    /// `recv` returning `-1` is only "still connected" for `EAGAIN`/`EWOULDBLOCK`/`EINTR`;
    /// any other errno is a [`FrankaError::Network`], not a silently healthy connection.
    #[test]
    fn a_failed_peek_is_a_network_error_unless_it_would_block() {
        assert!(classify_peek(1, 0).is_ok());
        assert!(classify_peek(-1, libc::EAGAIN).is_ok());
        assert!(classify_peek(-1, libc::EWOULDBLOCK).is_ok());
        assert!(classify_peek(-1, libc::EINTR).is_ok());

        match classify_peek(0, 0) {
            Err(FrankaError::Network(message)) => {
                assert_eq!(message, "libfranka: server closed connection")
            }
            other => panic!("expected a closed connection, got {other:?}"),
        }
        for errno in [libc::ECONNRESET, libc::ENOTCONN, libc::EPIPE, libc::EBADF] {
            match classify_peek(-1, errno) {
                Err(FrankaError::Network(message)) => {
                    assert!(
                        message.starts_with("libfranka: Connection error: "),
                        "unexpected message {message}"
                    )
                }
                other => panic!("errno {errno}: expected a Network error, got {other:?}"),
            }
        }
    }

    /// Builds a complete message: robot header (`command`, `command_id`, `size`) + payload.
    fn message(command: u32, command_id: u32, payload: &[u8]) -> Vec<u8> {
        let mut out = HeaderLayout::Robot.encode_header(command, command_id, payload.len());
        out.extend_from_slice(payload);
        out
    }

    /// Accepts one connection, hands the stream to `serve`, and keeps the listener alive.
    fn mock_server<F>(serve: F) -> (SocketAddr, std::thread::JoinHandle<()>)
    where
        F: FnOnce(TcpStream) + Send + 'static,
    {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let handle = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            serve(stream);
        });
        (addr, handle)
    }

    fn client(addr: SocketAddr) -> TcpSession {
        TcpSession::connect(addr, Duration::from_secs(1), HeaderLayout::Robot).unwrap()
    }

    #[test]
    fn responses_are_demultiplexed_and_reassembled() {
        let (addr, server) = mock_server(|mut stream| {
            // Drain the two requests so the client's writes cannot block.
            let mut scratch = [0u8; 64];
            let _ = stream.read(&mut scratch);
            // Answer the *second* request first, split across three writes.
            let second = message(2, 1, b"second");
            stream.write_all(&second[..5]).unwrap();
            stream.flush().unwrap();
            std::thread::sleep(Duration::from_millis(30));
            stream.write_all(&second[5..14]).unwrap();
            stream.flush().unwrap();
            std::thread::sleep(Duration::from_millis(30));
            stream.write_all(&second[14..]).unwrap();
            stream.write_all(&message(1, 0, b"first")).unwrap();
            stream.flush().unwrap();
            std::thread::sleep(Duration::from_millis(200));
        });

        let session = client(addr);
        assert_eq!(session.send_request(1, b"a").unwrap(), 0);
        assert_eq!(session.send_request(2, b"b").unwrap(), 1);

        let first = session.blocking_receive_response(0).unwrap();
        assert_eq!(&first[12..], b"first");
        let second = session.blocking_receive_response(1).unwrap();
        assert_eq!(&second[12..], b"second");

        drop(session);
        server.join().unwrap();
    }

    #[test]
    fn unclaimed_response_stays_queued() {
        let (addr, server) = mock_server(|mut stream| {
            let mut scratch = [0u8; 64];
            let _ = stream.read(&mut scratch);
            stream.write_all(&message(11, 99, b"late")).unwrap();
            stream.flush().unwrap();
            std::thread::sleep(Duration::from_millis(300));
        });

        let session = client(addr);
        session.send_request(11, &[]).unwrap();

        // Polling for a different id must not consume the queued message.
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        loop {
            assert!(session.try_receive_response(0).unwrap().is_none());
            if let Some(message) = session.try_receive_response(99).unwrap() {
                assert_eq!(&message[12..], b"late");
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "response never arrived"
            );
            std::thread::sleep(Duration::from_millis(5));
        }

        drop(session);
        server.join().unwrap();
    }

    #[test]
    fn undersized_header_is_a_protocol_error() {
        let (addr, server) = mock_server(|mut stream| {
            let mut scratch = [0u8; 64];
            let _ = stream.read(&mut scratch);
            // size = 5 < sizeof(CommandHeader) = 12.
            let mut header = Vec::new();
            header.extend_from_slice(&1u32.to_le_bytes());
            header.extend_from_slice(&0u32.to_le_bytes());
            header.extend_from_slice(&5u32.to_le_bytes());
            stream.write_all(&header).unwrap();
            stream.flush().unwrap();
            std::thread::sleep(Duration::from_millis(200));
        });

        let session = client(addr);
        session.send_request(1, &[]).unwrap();
        let error = session.blocking_receive_response(0).unwrap_err();
        assert_eq!(error.to_string(), "libfranka: Incorrect TCP message size.");
        assert!(matches!(error, FrankaError::Protocol(_)));

        drop(session);
        server.join().unwrap();
    }

    #[test]
    fn oversized_header_is_a_protocol_error_without_allocating() {
        let (addr, server) = mock_server(|mut stream| {
            let mut scratch = [0u8; 64];
            let _ = stream.read(&mut scratch);
            // size = 0xFFFF_FFFF, far beyond MAX_TCP_MESSAGE_SIZE.
            let mut header = Vec::new();
            header.extend_from_slice(&1u32.to_le_bytes());
            header.extend_from_slice(&0u32.to_le_bytes());
            header.extend_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
            stream.write_all(&header).unwrap();
            stream.flush().unwrap();
            std::thread::sleep(Duration::from_millis(200));
        });

        let session = client(addr);
        session.send_request(1, &[]).unwrap();
        let error = session.blocking_receive_response(0).unwrap_err();
        assert_eq!(error.to_string(), "libfranka: Incorrect TCP message size.");
        assert!(matches!(error, FrankaError::Protocol(_)));

        drop(session);
        server.join().unwrap();
    }

    #[test]
    fn connection_refused_uses_libfranka_text() {
        // Bind and immediately drop, so the port is (almost certainly) closed.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        drop(listener);

        match TcpSession::connect(addr, Duration::from_secs(1), HeaderLayout::Robot) {
            Err(FrankaError::Network(message)) => assert_eq!(
                message,
                "libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk."
            ),
            other => panic!("expected a refused connection, got {other:?}"),
        }
    }

    #[test]
    fn closed_connection_is_detected() {
        let (addr, server) = mock_server(|stream| {
            drop(stream);
        });
        let session = client(addr);
        server.join().unwrap();

        // Give the FIN time to arrive.
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        loop {
            match session.throw_if_connection_closed() {
                Err(FrankaError::Network(message)) => {
                    assert_eq!(message, "libfranka: server closed connection");
                    break;
                }
                _ => assert!(std::time::Instant::now() < deadline, "FIN never observed"),
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}
