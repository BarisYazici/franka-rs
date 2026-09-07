//! The loopback FCI server the [parent module](super) tests `RobotImpl` against.

use super::*;

/// Server-side state shared with the scheduling threads.
pub(super) struct Shared {
    /// FCI version this server speaks.
    version: FciVersion,
    /// Write half of the accepted connection.
    stream: Mutex<Option<TcpStream>>,
    /// `Connect` answers to give, oldest first, as `(status, version)`. When the queue runs
    /// dry the server answers `kSuccess` with its own version.
    connect_answers: Mutex<Vec<(u8, u16)>>,
    /// Responses waiting to be sent when a request for that command id arrives.
    queued: Mutex<HashMap<u32, Vec<Vec<u8>>>>,
    /// Command ids of the requests seen so far, per command id.
    seen: Mutex<HashMap<u32, Vec<u32>>>,
    /// Request payloads seen so far, per command id.
    payloads: Mutex<HashMap<u32, Vec<Vec<u8>>>>,
    /// `"accept"` / `"close"` in the order the TCP server observed them, so a test can assert
    /// that a reconnect really opened a second connection after the first was closed.
    pub(super) connection_events: Mutex<Vec<&'static str>>,
    /// The state socket, which also receives the client's robot commands.
    udp: UdpSocket,
    /// Address the client's state datagrams go to.
    client: Mutex<Option<SocketAddr>>,
    /// Robot commands received from the client, as raw datagrams (370 or 371 bytes).
    commands: Mutex<Vec<Vec<u8>>>,
    /// `message_id` of the next state.
    next_message_id: AtomicU64,
    stop: AtomicBool,
}

impl Shared {
    /// Writes a response message for the command id `command` under `command_id`.
    pub(super) fn respond_id(&self, command: u32, command_id: u32, payload: &[u8]) {
        let mut message = HeaderLayout::Robot.encode_header(command, command_id, payload.len());
        message.extend_from_slice(payload);
        let mut guard = self.stream.lock().unwrap();
        if let Some(stream) = guard.as_mut() {
            let _ = stream.write_all(&message);
            let _ = stream.flush();
        }
    }

    /// [`Shared::respond_id`] for a command named by the FCI v10 enum.
    pub(super) fn respond(&self, command: Command, command_id: u32, payload: &[u8]) {
        self.respond_id(command.to_u32(), command_id, payload);
    }

    /// Writes a response for the most recent request of the command id `command`.
    pub(super) fn respond_last_id(&self, command: u32, payload: &[u8]) {
        let command_id = *self
            .seen
            .lock()
            .unwrap()
            .get(&command)
            .and_then(|ids| ids.last())
            .expect("no request seen for this command");
        self.respond_id(command, command_id, payload);
    }

    /// Writes a response for the most recent request of `command`, using this server's
    /// version numbering.
    pub(super) fn respond_last(&self, command: CommandKind, payload: &[u8]) {
        self.respond_last_id(self.command_id(command), payload);
    }

    /// The wire command id of `command` under this server's version.
    pub(super) fn command_id(&self, command: CommandKind) -> u32 {
        codec::command_id(self.version, command).expect("command exists in this version")
    }

    /// Sends one state datagram built from `template`, with a fresh `message_id`.
    ///
    /// The `message_id` is the first eight bytes of the datagram in both versions.
    pub(super) fn send_state_bytes(&self, template: &[u8]) {
        let id = self.next_message_id.fetch_add(1, Ordering::Relaxed);
        self.send_state_bytes_with_id(template, id);
    }

    pub(super) fn send_state_bytes_with_id(&self, template: &[u8], message_id: u64) {
        let mut state = template.to_vec();
        state[..8].copy_from_slice(&message_id.to_le_bytes());
        let client = *self.client.lock().unwrap();
        if let Some(client) = client {
            let _ = self.udp.send_to(&state, client);
        }
    }

    /// Sends one FCI v10 robot state built from `template`, with a fresh `message_id`.
    pub(super) fn send_state(&self, template: &WireRobotState) {
        self.send_state_bytes(template.as_bytes());
    }

    /// Sends one FCI v10 robot state with an explicit `message_id` (for the reordering test).
    pub(super) fn send_state_with_id(&self, template: &WireRobotState, message_id: u64) {
        self.send_state_bytes_with_id(template.as_bytes(), message_id);
    }
}

/// A loopback FCI server for the robot protocol.
pub(super) struct MockServer {
    address: String,
    pub(super) shared: Arc<Shared>,
    version: FciVersion,
}

impl MockServer {
    /// An FCI v10 (FR3) server.
    pub(super) fn start() -> MockServer {
        MockServer::start_with(FciVersion::V10, Vec::new())
    }

    /// An FCI v5 (FER) server.
    pub(super) fn start_v5() -> MockServer {
        MockServer::start_with(FciVersion::V5, Vec::new())
    }

    /// A server speaking `version`, answering the first `connect_answers.len()` `Connect`
    /// requests with the given `(status, version)` pairs and everything after them with
    /// `kSuccess` and its own version.
    pub(super) fn start_with(version: FciVersion, connect_answers: Vec<(u8, u16)>) -> MockServer {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap().to_string();
        let udp = UdpSocket::bind("127.0.0.1:0").unwrap();
        udp.set_read_timeout(Some(StdDuration::from_millis(20)))
            .unwrap();

        let shared = Arc::new(Shared {
            version,
            stream: Mutex::new(None),
            connect_answers: Mutex::new(connect_answers),
            queued: Mutex::new(HashMap::new()),
            seen: Mutex::new(HashMap::new()),
            payloads: Mutex::new(HashMap::new()),
            connection_events: Mutex::new(Vec::new()),
            udp,
            client: Mutex::new(None),
            commands: Mutex::new(Vec::new()),
            next_message_id: AtomicU64::new(1),
            stop: AtomicBool::new(false),
        });

        // TCP: serve one connection at a time, accepting a new one when the client reconnects.
        let tcp_shared = Arc::clone(&shared);
        std::thread::spawn(move || serve_tcp(&tcp_shared, listener));

        // UDP: record the robot commands the client sends.
        let udp_shared = Arc::clone(&shared);
        std::thread::spawn(move || {
            let mut buffer = [0u8; std::mem::size_of::<RobotCommand>()];
            while !udp_shared.stop.load(Ordering::Relaxed) {
                if let Ok((received, _)) = udp_shared.udp.recv_from(&mut buffer) {
                    udp_shared
                        .commands
                        .lock()
                        .unwrap()
                        .push(buffer[..received].to_vec());
                }
            }
        });

        MockServer {
            address,
            shared,
            version,
        }
    }

    /// Queues a response that is sent as soon as a request for the command id arrives.
    pub(super) fn queue_response_id(&self, command: u32, payload: &[u8]) {
        self.shared
            .queued
            .lock()
            .unwrap()
            .entry(command)
            .or_default()
            .push(payload.to_vec());
    }

    /// Queues a response for `command` under this server's version numbering.
    pub(super) fn queue_response_for(&self, command: CommandKind, payload: &[u8]) {
        self.queue_response_id(self.shared.command_id(command), payload);
    }

    /// Queues a response for an FCI v10 command.
    pub(super) fn queue_response(&self, command: Command, payload: &[u8]) {
        self.queue_response_id(command.to_u32(), payload);
    }

    /// Number of requests seen for the command id.
    pub(super) fn request_count_id(&self, command: u32) -> usize {
        self.shared
            .seen
            .lock()
            .unwrap()
            .get(&command)
            .map(|ids| ids.len())
            .unwrap_or(0)
    }

    /// Number of requests seen for an FCI v10 command.
    pub(super) fn request_count(&self, command: Command) -> usize {
        self.request_count_id(command.to_u32())
    }

    /// The request payloads seen for `command` under this server's version numbering.
    pub(super) fn payloads_for(&self, command: CommandKind) -> Vec<Vec<u8>> {
        self.shared
            .payloads
            .lock()
            .unwrap()
            .get(&self.shared.command_id(command))
            .cloned()
            .unwrap_or_default()
    }

    /// Streams `running` states for `running_ms`, then idle states for another `idle_ms`.
    ///
    /// Used by the cancellation tests, which need the state stream to keep going while the
    /// client is in a control loop and to go idle once the motion has been stopped.
    pub(super) fn stream_states_bytes(
        &self,
        running: Vec<u8>,
        idle: Vec<u8>,
        running_ms: u64,
        idle_ms: u64,
    ) {
        let shared = Arc::clone(&self.shared);
        std::thread::spawn(move || {
            for elapsed in 0..((running_ms + idle_ms) / 2) {
                if shared.stop.load(Ordering::Relaxed) {
                    return;
                }
                if elapsed * 2 < running_ms {
                    shared.send_state_bytes(&running);
                } else {
                    shared.send_state_bytes(&idle);
                }
                std::thread::sleep(StdDuration::from_millis(2));
            }
        });
    }

    /// [`MockServer::stream_states_bytes`] with FCI v10 states.
    pub(super) fn stream_states(&self, running: WireRobotState, running_ms: u64, idle_ms: u64) {
        self.stream_states_bytes(
            running.as_bytes().to_vec(),
            idle_state().as_bytes().to_vec(),
            running_ms,
            idle_ms,
        );
    }

    /// Runs `action` on the server side after `delay_ms`.
    pub(super) fn schedule<F>(&self, delay_ms: u64, action: F)
    where
        F: FnOnce(&Shared) + Send + 'static,
    {
        let shared = Arc::clone(&self.shared);
        std::thread::spawn(move || {
            std::thread::sleep(StdDuration::from_millis(delay_ms));
            action(&shared);
        });
    }

    /// The FCI v10 robot commands the client sent.
    pub(super) fn commands(&self) -> Vec<RobotCommand> {
        self.shared
            .commands
            .lock()
            .unwrap()
            .iter()
            .map(|bytes| RobotCommand::read_from_bytes(bytes).expect("v10 robot command"))
            .collect()
    }

    /// The FCI v5 robot commands the client sent.
    pub(super) fn commands_v5(&self) -> Vec<v5::RobotCommand> {
        self.shared
            .commands
            .lock()
            .unwrap()
            .iter()
            .map(|bytes| v5::RobotCommand::read_from_bytes(bytes).expect("v5 robot command"))
            .collect()
    }

    /// The raw robot command datagrams the client sent.
    pub(super) fn command_datagrams(&self) -> Vec<Vec<u8>> {
        self.shared.commands.lock().unwrap().clone()
    }

    /// The idle state of this server's version, as wire bytes.
    pub(super) fn idle_bytes(&self) -> Vec<u8> {
        match self.version {
            FciVersion::V5 => idle_state_v5().as_bytes().to_vec(),
            FciVersion::V10 => idle_state().as_bytes().to_vec(),
        }
    }

    /// A state reporting a running motion with the given modes, as wire bytes.
    pub(super) fn moving_bytes(
        &self,
        motion: StateMotionGeneratorMode,
        controller: StateControllerMode,
    ) -> Vec<u8> {
        match self.version {
            FciVersion::V5 => v5::RobotState {
                motion_generator_mode: motion.to_u8(),
                controller_mode: controller.to_u8(),
                robot_mode: WireRobotMode::Move.to_u8(),
                ..v5::RobotState::default()
            }
            .as_bytes()
            .to_vec(),
            FciVersion::V10 => moving_state(motion, controller).as_bytes().to_vec(),
        }
    }

    /// Connects a [`RobotImpl`] to this server, answering the startup sequence.
    pub(super) fn connect(&self, log_size: usize) -> RobotImpl {
        self.connect_with_policy(log_size, VersionPolicy::Exact(self.version))
            .expect("connect failed")
    }

    /// [`MockServer::connect`] with an explicit policy, returning the result.
    pub(super) fn connect_with_policy(
        &self,
        log_size: usize,
        policy: VersionPolicy,
    ) -> crate::error::FrankaResult<RobotImpl> {
        // The constructor blocks for one state right after the handshake.
        let state = self.idle_bytes();
        self.schedule(20, move |shared| shared.send_state_bytes(&state));
        RobotImpl::new_with_policy(&self.address, RealtimeConfig::Ignore, log_size, policy)
    }
}

impl Drop for MockServer {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
    }
}

/// Accepts connections one at a time and serves each until the client closes it.
fn serve_tcp(shared: &Arc<Shared>, listener: TcpListener) {
    while !shared.stop.load(Ordering::Relaxed) {
        let Ok((stream, _)) = listener.accept() else {
            return;
        };
        shared.connection_events.lock().unwrap().push("accept");
        serve_connection(shared, stream);
        shared.connection_events.lock().unwrap().push("close");
    }
}

/// Reads requests on one connection and answers `Connect`/`GetRobotModel` plus whatever a test
/// queued, returning when the connection is closed.
fn serve_connection(shared: &Arc<Shared>, stream: TcpStream) {
    stream
        .set_read_timeout(Some(StdDuration::from_millis(20)))
        .unwrap();
    *shared.stream.lock().unwrap() = Some(stream.try_clone().unwrap());
    let mut stream = stream;

    let mut header = [0u8; 12];
    while !shared.stop.load(Ordering::Relaxed) {
        match read_exact_with_timeout(&mut stream, &mut header, shared) {
            ReadOutcome::Closed => return,
            ReadOutcome::WouldBlock => continue,
            ReadOutcome::Complete => {}
        }
        let (command, command_id, size) = HeaderLayout::Robot.decode_header(&header).unwrap();
        let mut payload = vec![0u8; size as usize - 12];
        if !payload.is_empty()
            && read_exact_with_timeout(&mut stream, &mut payload, shared) != ReadOutcome::Complete
        {
            return;
        }

        shared
            .seen
            .lock()
            .unwrap()
            .entry(command)
            .or_default()
            .push(command_id);
        shared
            .payloads
            .lock()
            .unwrap()
            .entry(command)
            .or_default()
            .push(payload.clone());

        if command == Command::Connect.to_u32() {
            let udp_port = u16::from_le_bytes([payload[2], payload[3]]);
            *shared.client.lock().unwrap() = Some(format!("127.0.0.1:{udp_port}").parse().unwrap());
            let answers = &mut *shared.connect_answers.lock().unwrap();
            let (status, version) = if answers.is_empty() {
                (ConnectStatus::Success.to_u8(), shared.version.number())
            } else {
                answers.remove(0)
            };
            let mut response = vec![status];
            response.extend_from_slice(&version.to_le_bytes());
            shared.respond_id(command, command_id, &response);
            continue;
        }

        if shared.version == FciVersion::V10 && command == Command::GetRobotModel.to_u32() {
            let mut response = vec![0u8];
            response.extend_from_slice(TEST_URDF.as_bytes());
            shared.respond(Command::GetRobotModel, command_id, &response);
            continue;
        }

        let queued = shared
            .queued
            .lock()
            .unwrap()
            .get_mut(&command)
            .and_then(|responses| {
                if responses.is_empty() {
                    None
                } else {
                    Some(responses.remove(0))
                }
            });
        if let Some(response) = queued {
            shared.respond_id(command, command_id, &response);
        }
    }
}

/// What one `read_exact_with_timeout` call achieved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadOutcome {
    /// The buffer was filled.
    Complete,
    /// Nothing was waiting; the caller may retry.
    WouldBlock,
    /// The peer closed the connection (or the server is stopping).
    Closed,
}

/// `read_exact` that tolerates the socket's read timeout, so the thread can observe `stop`.
fn read_exact_with_timeout(
    stream: &mut TcpStream,
    buffer: &mut [u8],
    shared: &Arc<Shared>,
) -> ReadOutcome {
    let mut offset = 0;
    while offset < buffer.len() {
        match stream.read(&mut buffer[offset..]) {
            Ok(0) => return ReadOutcome::Closed,
            Ok(n) => offset += n,
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                if shared.stop.load(Ordering::Relaxed) {
                    return ReadOutcome::Closed;
                }
                if offset == 0 {
                    return ReadOutcome::WouldBlock;
                }
            }
            Err(_) => return ReadOutcome::Closed,
        }
    }
    ReadOutcome::Complete
}
