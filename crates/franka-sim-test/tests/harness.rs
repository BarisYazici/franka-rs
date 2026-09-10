//! Integration tests for the franka-sim-test harness itself. These start a
//! real franka-sim Docker container; see
//! `docs/book/src/howto/simulator-tests.md`.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream, UdpSocket};
use std::process::Command;
use std::time::{Duration, Instant};

use franka_sim_test::{Protocol, SimConfig, SimServer};

const ROBOT_ADDR: &str = "127.0.0.1:1337";

/// Names of currently-running containers matching the `franka-sim` filter.
/// Only one `SimServer` can be alive at a time within this test binary (they
/// share a process-wide lock), so this is exactly the harness's own
/// container while one is running, or empty otherwise.
fn docker_ps_franka_sim_names() -> Vec<String> {
    let output = Command::new("docker")
        .args([
            "ps",
            "--filter",
            "name=franka-sim",
            "--format",
            "{{.Names}}",
        ])
        .output()
        .expect("failed to run docker ps");
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect()
}

fn port_is_open(addr: &str) -> bool {
    let socket_addr: SocketAddr = addr.parse().expect("valid socket address");
    TcpStream::connect_timeout(&socket_addr, Duration::from_millis(200)).is_ok()
}

#[test]
fn nominal_start_and_teardown() {
    let sim = SimServer::start(SimConfig::nominal());
    assert_eq!(sim.host(), "127.0.0.1");
    assert_eq!(sim.address(), "127.0.0.1");

    // The FCI robot port must be accepting connections. Connect and drop
    // immediately without sending anything; the sim logs a disconnect for
    // this, which is expected and fine.
    let stream = TcpStream::connect_timeout(&ROBOT_ADDR.parse().unwrap(), Duration::from_secs(5))
        .expect("robot FCI port did not accept a connection");
    drop(stream);

    let names_while_running = docker_ps_franka_sim_names();
    assert_eq!(
        names_while_running.len(),
        1,
        "expected exactly one franka-sim container while the server is running, got \
         {names_while_running:?}"
    );

    drop(sim);

    // Port 1337 must close within a few seconds of container removal.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline && port_is_open(ROBOT_ADDR) {
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        !port_is_open(ROBOT_ADDR),
        "port 1337 was still open 5s after the container was removed"
    );

    let names_after = docker_ps_franka_sim_names();
    assert!(
        names_after.is_empty(),
        "expected no franka-sim containers after teardown, got {names_after:?}"
    );
}

#[test]
fn with_motion_limits_passes_the_flag() {
    let sim = SimServer::start(SimConfig::nominal().with_motion_limits());
    assert_eq!(sim.host(), "127.0.0.1");

    let names = docker_ps_franka_sim_names();
    assert_eq!(
        names.len(),
        1,
        "expected exactly one franka-sim container while the server is running, got {names:?}"
    );
    let name = &names[0];

    let output = Command::new("docker")
        .args(["inspect", "--format", "{{.Args}}", name])
        .output()
        .expect("failed to run docker inspect");
    assert!(
        output.status.success(),
        "docker inspect failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let args = String::from_utf8_lossy(&output.stdout);
    assert!(
        args.contains("--enforce-motion-limits"),
        "container args did not include --enforce-motion-limits: {args}"
    );

    drop(sim);
}

/// Starts the FER / FCI v5 simulator (image `franka-sim:panda-v5`, which
/// ships no `franka-sim-check`; [`SimServer::start`] falls back to its own
/// Connect handshake probe for readiness) and, independently of that
/// readiness probe, performs a second FCI v5 Connect handshake from this
/// test to verify the wire contract end-to-end: a 16-byte Connect request
/// (version 5, this test's UDP port), a 15-byte reply with `status == 0`
/// and `version == 5`, and one UDP `RobotState` datagram of exactly 2373
/// bytes.
#[test]
fn fer_v5_starts_and_streams_2373_byte_states() {
    let sim = SimServer::start(SimConfig::fer_v5());
    assert_eq!(sim.host(), "127.0.0.1");
    assert_eq!(sim.protocol(), Protocol::V5);

    let mut tcp = TcpStream::connect_timeout(&ROBOT_ADDR.parse().unwrap(), Duration::from_secs(5))
        .expect("robot FCI port did not accept a connection");
    tcp.set_read_timeout(Some(Duration::from_secs(5)))
        .expect("failed to set TCP read timeout");

    let udp = UdpSocket::bind("0.0.0.0:0").expect("failed to bind a UDP socket");
    let udp_port = udp
        .local_addr()
        .expect("failed to read the bound UDP socket's address")
        .port();

    let mut connect_request = Vec::with_capacity(16);
    connect_request.extend_from_slice(&0u32.to_le_bytes()); // command = Connect
    connect_request.extend_from_slice(&1u32.to_le_bytes()); // command_id
    connect_request.extend_from_slice(&16u32.to_le_bytes()); // size (header + payload)
    connect_request.extend_from_slice(&5u16.to_le_bytes()); // version = 5
    connect_request.extend_from_slice(&udp_port.to_le_bytes());
    tcp.write_all(&connect_request)
        .expect("failed to send the Connect request");

    let mut reply = [0u8; 15];
    tcp.read_exact(&mut reply)
        .expect("failed to read the Connect reply");
    let status = reply[12];
    let version = u16::from_le_bytes([reply[13], reply[14]]);
    assert_eq!(status, 0, "Connect reply status was not Success (0)");
    assert_eq!(version, 5, "Connect reply version was not 5");

    udp.set_read_timeout(Some(Duration::from_secs(5)))
        .expect("failed to set UDP read timeout");
    let mut datagram = [0u8; 4096];
    let (len, _src) = udp
        .recv_from(&mut datagram)
        .expect("did not receive a UDP RobotState datagram within 5s");
    assert_eq!(
        len, 2373,
        "RobotState datagram had unexpected length {len}, expected 2373"
    );

    drop(tcp);
    drop(udp);

    drop(sim);
}
