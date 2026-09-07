//! The container handle: starting the simulator, the process-wide single-client
//! lock it holds, and the `docker rm -f` its [`Drop`] performs.

use std::env;
use std::process::Command;
use std::sync::atomic::Ordering;
use std::sync::MutexGuard;

use crate::docker::{docker, refuse_if_busy, wait_ready};
use crate::{Protocol, SimConfig, COUNTER, DEFAULT_FER_IMAGE, DEFAULT_IMAGE, LOCK};

///
/// Dropping this removes the Docker container (`docker rm -f`) and releases
/// the process-wide lock so the next test may start a new one. Set
/// `FRANKA_SIM_KEEP=1` to skip removal and inspect the container afterwards.
pub struct SimServer {
    /// Docker container name, or empty when `external` is true.
    name: String,
    /// Address clients should connect to; see [`SimServer::host`].
    host: String,
    /// True when using a caller-managed server via `FRANKA_SIM_ADDR`; in
    /// that case this harness never touches Docker.
    external: bool,
    /// FCI protocol version this server was configured for; see
    /// [`SimServer::protocol`].
    protocol: Protocol,
    /// Held for the lifetime of the server so only one franka-sim runs at a
    /// time within this test binary.
    _guard: MutexGuard<'static, ()>,
}

impl SimServer {
    /// Starts a franka-sim server for the given configuration and blocks
    /// until it is ready to accept FCI connections.
    ///
    /// Panics with a clear message if Docker is unavailable, the FCI ports
    /// are already in use, a franka-sim container is already running, or the
    /// server does not become ready in time (container logs are printed
    /// first).
    ///
    /// If the `FRANKA_SIM_ADDR` environment variable is set, no container is
    /// started; `config` is ignored by this harness (the caller is
    /// responsible for the already-running server matching it) and
    /// [`SimServer::host`] returns that address instead.
    ///
    /// ```rust,no_run
    /// use franka_sim_test::{SimConfig, SimServer};
    /// let sim = SimServer::start(SimConfig::nominal());
    /// println!("sim running at {}", sim.host());
    /// ```
    pub fn start(config: SimConfig) -> SimServer {
        let guard = LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner());

        if let Ok(addr) = env::var("FRANKA_SIM_ADDR") {
            eprintln!(
                "franka-sim-test: using external server at {addr} (config is the caller's \
                 responsibility): {config:?}"
            );
            return SimServer {
                name: String::new(),
                host: addr,
                external: true,
                protocol: config.protocol,
                _guard: guard,
            };
        }

        refuse_if_busy();

        let image = config.image.clone().unwrap_or_else(|| {
            env::var("FRANKA_SIM_IMAGE").unwrap_or_else(|_| DEFAULT_IMAGE.to_string())
        });
        let counter = COUNTER.fetch_add(1, Ordering::Relaxed);
        let name = format!("franka-sim-{}-{}", std::process::id(), counter);

        eprintln!(
            "franka-sim-test: starting container {name} from image {image} with config {config:?}"
        );

        let mut docker_run_args = vec![
            "run".to_string(),
            "-d".to_string(),
            "--network".to_string(),
            "host".to_string(),
            "--name".to_string(),
            name.clone(),
        ];
        for (key, value) in &config.env {
            docker_run_args.push("-e".to_string());
            docker_run_args.push(format!("{key}={value}"));
        }
        let image_name = image.clone();
        docker_run_args.push(image);
        docker_run_args.extend(config.args());

        let output = docker(&docker_run_args);
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            // The FER image is not published, so "no such image" is the expected failure on
            // any machine but the one it was built on. Say what to do about it instead of
            // leaving a bare `docker run` error; these tests fail loudly by design rather
            // than skipping, because a silently skipped v5 suite is worse than a red one.
            let hint = if config.protocol == Protocol::V5 {
                format!(
                    "\nhint: the FER / FCI v5 image is not published anywhere. This run \
                     wanted `{image_name}` (override with FRANKA_SIM_FER_IMAGE; \
                     FRANKA_SIM_IMAGE names the FR3 image and is deliberately ignored here). \
                     Build it (recipe kept privately, outside this repository):\n  docker \
                     build -t {DEFAULT_FER_IMAGE} <franka-sim panda-v5 checkout>\nor point \
                     FRANKA_SIM_ADDR at an already-running v5 server."
                )
            } else {
                format!(
                    "\nhint: this run wanted `{image_name}` (override with FRANKA_SIM_IMAGE, \
                     default {DEFAULT_IMAGE}); `docker pull {DEFAULT_IMAGE}` gets the \
                     published FR3 image."
                )
            };
            panic!(
                "franka-sim-test: `docker run` failed to start container {name}:\n{stderr}{hint}"
            );
        }

        wait_ready(&name, config.protocol);

        eprintln!("franka-sim-test: container {name} is ready");

        SimServer {
            name,
            host: "127.0.0.1".to_string(),
            external: false,
            protocol: config.protocol,
            _guard: guard,
        }
    }

    /// The address clients should connect to. `"127.0.0.1"` for a
    /// Docker-managed server; the value of `FRANKA_SIM_ADDR` otherwise. The
    /// robot (1337) and gripper (1338) ports are fixed by the protocol and
    /// are not part of this value.
    pub fn host(&self) -> &str {
        &self.host
    }

    /// Same as [`SimServer::host`], returned as an owned `String`.
    pub fn address(&self) -> String {
        self.host.clone()
    }

    /// The FCI protocol version this server was configured for (the
    /// `protocol` field of the [`SimConfig`] passed to [`SimServer::start`]).
    pub fn protocol(&self) -> Protocol {
        self.protocol
    }
}

impl Drop for SimServer {
    fn drop(&mut self) {
        if self.external {
            return;
        }
        if matches!(env::var("FRANKA_SIM_KEEP"), Ok(v) if v == "1") {
            eprintln!(
                "franka-sim-test: FRANKA_SIM_KEEP=1, leaving container {} running",
                self.name
            );
            return;
        }
        // Best-effort: ignore errors, never panic in Drop.
        let _ = Command::new("docker")
            .args(["rm", "-f", &self.name])
            .output();
    }
}
