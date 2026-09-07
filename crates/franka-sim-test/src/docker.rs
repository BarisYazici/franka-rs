//! Talking to the Docker daemon and deciding when the simulator is ready.

use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};
use std::process::{Command, Output};
use std::time::{Duration, Instant};

use crate::{
    Protocol, CONTAINER_START_TIMEOUT, GRIPPER_PORT, PROBE_CONNECT_TIMEOUT, PROBE_UDP_TIMEOUT,
    ROBOT_HOST, ROBOT_PORT,
};

/// Runs `docker <args>`, panicking with a clear message if the `docker`
/// binary itself cannot be spawned (as opposed to it running and failing,
/// which callers inspect via the returned `Output`).
pub(crate) fn docker(args: &[String]) -> Output {
    Command::new("docker")
        .args(args)
        .output()
        .unwrap_or_else(|err| {
            panic!(
            "franka-sim-test: failed to run `docker {}` (is Docker installed and on PATH?): {err}",
            args.join(" ")
        )
        })
}

/// Panics if the FCI ports are already bound or a franka-sim container is
/// already running, per the "one FCI client slot" constraint.
pub(crate) fn refuse_if_busy() {
    let ss_output = Command::new("ss")
        .args(["-tlnp"])
        .output()
        .unwrap_or_else(|err| {
            panic!("franka-sim-test: failed to run `ss -tlnp` (is iproute2 installed?): {err}")
        });
    let listening = String::from_utf8_lossy(&ss_output.stdout);
    for line in listening.lines() {
        for token in line.split_whitespace() {
            if let Some(port) = token.rsplit(':').next() {
                if port == ROBOT_PORT.to_string() || port == GRIPPER_PORT.to_string() {
                    panic!(
                        "franka-sim-test: refusing to start, a process is already listening on \
                         port {ROBOT_PORT} or {GRIPPER_PORT}:\n{listening}"
                    );
                }
            }
        }
    }

    let ps_output = docker(&[
        "ps".to_string(),
        "--filter".to_string(),
        "name=franka-sim".to_string(),
        "--format".to_string(),
        "{{.Names}}".to_string(),
    ]);
    let names = String::from_utf8_lossy(&ps_output.stdout);
    let names = names.trim();
    if !names.is_empty() {
        panic!(
            "franka-sim-test: refusing to start, a franka-sim container is already running: \
             {names}"
        );
    }
}

/// Waits for the container to report `Running` (bounded by
/// [`CONTAINER_START_TIMEOUT`]), then runs the readiness probe exactly once.
/// franka-sim-check retries the FCI handshake internally for its own
/// `--timeout`, so this harness must never poll it periodically — it would
/// occupy the single FCI client slot repeatedly. For the same reason,
/// `readiness_probe` (used for [`Protocol::V5`], whose image ships no
/// `franka-sim-check` binary, and as a fallback for either protocol whenever
/// `franka-sim-check` itself is missing from the image) also runs exactly
/// once, and is always called with the container's own `protocol` so it
/// sends and expects the matching wire version and `RobotState` size — never
/// the v5 shape against what may be a v10 server, or vice versa.
pub(crate) fn wait_ready(name: &str, protocol: Protocol) {
    let deadline = Instant::now() + CONTAINER_START_TIMEOUT;
    while !is_running(name) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
    }

    if protocol == Protocol::V5 {
        if let Err(probe_err) = readiness_probe(protocol) {
            fail_ready(
                name,
                format!("own FCI {protocol:?} readiness probe failed: {probe_err}"),
            );
        }
        return;
    }

    let output = Command::new("docker")
        .args(["exec", name, "franka-sim-check", "--timeout", "60"])
        .output();

    let ready = matches!(&output, Ok(o) if o.status.success());
    if ready {
        return;
    }

    let executable_not_found = matches!(&output, Ok(o) if
        String::from_utf8_lossy(&o.stderr).contains("executable file not found"));
    if executable_not_found {
        if let Err(probe_err) = readiness_probe(protocol) {
            fail_ready(
                name,
                format!(
                    "franka-sim-check is not present in this image; own FCI {protocol:?} \
                     readiness probe also failed: {probe_err}"
                ),
            );
        }
        return;
    }

    let probe_detail = match output {
        Ok(o) => format!(
            "exit status {:?}, stdout: {}, stderr: {}",
            o.status.code(),
            String::from_utf8_lossy(&o.stdout),
            String::from_utf8_lossy(&o.stderr)
        ),
        Err(err) => format!("failed to run docker exec: {err}"),
    };
    fail_ready(
        name,
        format!("franka-sim-check --timeout 60: {probe_detail}"),
    );
}

/// Prints container logs, removes the container (it is never returned to
/// the caller, so nothing will run its `Drop` impl), and panics with
/// `detail`.
pub(crate) fn fail_ready(name: &str, detail: String) -> ! {
    eprintln!(
        "franka-sim-test: readiness probe failed for container {name}; container logs follow:"
    );
    if let Ok(logs) = Command::new("docker").args(["logs", name]).output() {
        eprintln!("{}", String::from_utf8_lossy(&logs.stdout));
        eprintln!("{}", String::from_utf8_lossy(&logs.stderr));
    }

    let _ = Command::new("docker").args(["rm", "-f", name]).output();

    panic!("franka-sim-test: franka-sim container {name} did not become ready: {detail}");
}

/// The harness's own protocol-aware readiness probe, used when
/// `franka-sim-check` is unavailable (notably in the `franka-sim:panda-v5`
/// image, which ships no such binary). Runs exactly once per
/// [`SimServer::start`] call (see `wait_ready`), always for the
/// container's own `protocol` — it never sends or expects the wrong
/// version/state-size pairing:
///
/// 1. Retries connecting a TCP socket to `127.0.0.1:1337` for up to
///    [`PROBE_CONNECT_TIMEOUT`].
/// 2. Sends a 16-byte Connect request (12-byte header with `command = 0`,
///    `command_id = 1`, `size = 16`, followed by `u16 version =`
///    [`Protocol::wire_version`] and the `u16` port of a UDP socket bound on
///    `0.0.0.0:0`).
/// 3. Reads the 15-byte reply (12-byte header, then `u8 status`, `u16
///    version`) and requires `status == 0` and `version ==
///    protocol.wire_version()`.
/// 4. Receives one UDP datagram on the bound socket within
///    [`PROBE_UDP_TIMEOUT`] and requires its length to be
///    [`Protocol::expected_robot_state_len`].
///
/// Both sockets are then dropped; the simulator resets its session on
/// disconnect, which is expected.
pub(crate) fn readiness_probe(protocol: Protocol) -> Result<(), String> {
    let wire_version = protocol.wire_version();
    let expected_state_len = protocol.expected_robot_state_len();

    let deadline = Instant::now() + PROBE_CONNECT_TIMEOUT;
    let mut tcp = loop {
        match TcpStream::connect((ROBOT_HOST, ROBOT_PORT)) {
            Ok(stream) => break stream,
            Err(err) => {
                if Instant::now() >= deadline {
                    return Err(format!(
                        "could not connect to {ROBOT_HOST}:{ROBOT_PORT} within \
                         {PROBE_CONNECT_TIMEOUT:?}: {err}"
                    ));
                }
                std::thread::sleep(Duration::from_millis(200));
            }
        }
    };
    tcp.set_read_timeout(Some(PROBE_UDP_TIMEOUT))
        .map_err(|err| format!("failed to set TCP read timeout: {err}"))?;

    let udp = UdpSocket::bind((ROBOT_HOST, 0))
        .map_err(|err| format!("failed to bind a UDP socket: {err}"))?;
    let udp_port = udp
        .local_addr()
        .map_err(|err| format!("failed to read the bound UDP socket's address: {err}"))?
        .port();

    let mut connect_request = Vec::with_capacity(16);
    connect_request.extend_from_slice(&0u32.to_le_bytes()); // command = Connect
    connect_request.extend_from_slice(&1u32.to_le_bytes()); // command_id
    connect_request.extend_from_slice(&16u32.to_le_bytes()); // size (header + payload)
    connect_request.extend_from_slice(&wire_version.to_le_bytes());
    connect_request.extend_from_slice(&udp_port.to_le_bytes());
    tcp.write_all(&connect_request)
        .map_err(|err| format!("failed to send the Connect request: {err}"))?;

    let mut reply = [0u8; 15];
    tcp.read_exact(&mut reply)
        .map_err(|err| format!("failed to read the Connect reply: {err}"))?;
    let status = reply[12];
    let reply_version = u16::from_le_bytes([reply[13], reply[14]]);
    if status != 0 {
        return Err(format!(
            "Connect reply status was {status}, expected 0 (Success)"
        ));
    }
    if reply_version != wire_version {
        return Err(format!(
            "Connect reply version was {reply_version}, expected {wire_version}"
        ));
    }

    udp.set_read_timeout(Some(PROBE_UDP_TIMEOUT))
        .map_err(|err| format!("failed to set UDP read timeout: {err}"))?;
    let mut datagram = [0u8; 4096];
    let (len, _src) = udp.recv_from(&mut datagram).map_err(|err| {
        format!("did not receive a UDP RobotState datagram within {PROBE_UDP_TIMEOUT:?}: {err}")
    })?;
    if len != expected_state_len {
        return Err(format!(
            "first UDP RobotState datagram was {len} bytes, expected {expected_state_len}"
        ));
    }

    drop(tcp);
    drop(udp);
    Ok(())
}

/// Reports whether Docker considers the container running.
pub(crate) fn is_running(name: &str) -> bool {
    let output = Command::new("docker")
        .args(["inspect", "-f", "{{.State.Running}}", name])
        .output();
    matches!(output, Ok(o) if o.status.success() && String::from_utf8_lossy(&o.stdout).trim() == "true")
}
