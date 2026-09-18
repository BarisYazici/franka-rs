"""Fixtures: `fake`, a node's side of one arm on the loopback for the offline tests, and
`node_endpoint`, the `franka-node` binary against franka-sim for the sim tests. The `sim`
fixture is franka-py's, loaded from its conftest (FRANKA_SIM_ADDR attaches to a running server).
"""

import importlib.util
import json
import os
import pathlib
import signal
import socket
import subprocess
import threading
import time

import numpy as np
import pytest
import zenoh

import franka_node
from franka_node import _wire

REPO = pathlib.Path(__file__).resolve().parents[4]

_spec = importlib.util.spec_from_file_location(
    "franka_py_conftest", REPO / "crates" / "franka-py" / "tests" / "conftest.py")
_franka_py = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_franka_py)
sim = _franka_py.sim

NODE_ENDPOINT = "tcp/127.0.0.1:17448"
NODE_NAME = "sim-test"
SIM_ARM = "sim"


def rotation_matrix(quaternion):
    """The rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def free_endpoint():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{s.getsockname()[1]}"


class FakeNode:
    """The node's keys for arm `fake`: `cmd/*` answered by the node's phase rules (or by
    `overrides[verb]`, a reply dict), the state published from `state` every `state_period` s
    unless `paused`, the targets kept. With `track`, the measured pose and `q` follow each target and
    `seq_accepted` echoes it. The gripper state goes out every 20 ms from `gripper`; a gripper
    command runs `gripper_script(msg)` on a thread of its own, if set.
    """

    ARM = "fake"
    POSE = [0.4, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0]  # xyz, then xyzw: tool pointing down

    def __init__(self):
        self.endpoint = free_endpoint()
        config = zenoh.Config()
        config.insert_json5("listen/endpoints", json.dumps([self.endpoint]))
        config.insert_json5("scouting/multicast/enabled", "false")
        self.session = zenoh.open(config)
        self.lock = threading.Lock()
        self.overrides, self.requests, self.targets, self.leases = {}, [], [], set()
        self.track = self.paused = False
        self.state_period = 0.01
        self.gripper_script = None
        self.gripper = np.zeros((), _wire.GRIPPER_STATE)
        self.gripper["version"], self.gripper["flags"] = _wire.VERSION, _wire.GRIPPER_CALIBRATED
        self.gripper["width"] = self.gripper["commanded"] = self.gripper["max_width"] = 0.08
        self.gripper["t_node_ns"] = 1
        self.state = np.zeros((), _wire.STATE)
        self.state["version"] = _wire.VERSION
        self.set_pose(self.POSE)
        self.state["target"] = self.POSE
        arm = f"franka/{self.ARM}"
        self._entities = [
            self.session.declare_queryable(f"{arm}/cmd/*", self._query),
            self.session.declare_subscriber(f"{arm}/target", self._target),
            self.session.liveliness().declare_subscriber(f"{arm}/lease/*", self._lease),
            self.session.declare_subscriber(f"{arm}/gripper/target", self._gripper_target),
        ]
        self._publisher = self.session.declare_publisher(f"{arm}/state")
        self._gripper_publisher = self.session.declare_publisher(f"{arm}/gripper/state")
        self._running = True
        self._thread = threading.Thread(target=self._publish, daemon=True)
        self._thread.start()

    def set_pose(self, pose, q=None):
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation_matrix(pose[3:]), pose[:3]
        self.state["o_t_ee"] = matrix.flatten(order="F")
        if q is not None:
            self.state["q"] = q

    def grip(self, reading=False, **fields):
        """Sets gripper state fields; `reading` makes it a new reading of the width."""
        with self.lock:
            for name, value in fields.items():
                self.gripper[name] = value
            if reading:
                self.gripper["t_node_ns"] += 1

    def _publish(self):
        tick, published = 0, 0.0
        while self._running:
            with self.lock:
                due = not self.paused and time.monotonic() - published >= self.state_period
                payload = self.state.tobytes() if due else None
                self.gripper["client_id"] = self.state["client_id"]
                gripper = self.gripper.tobytes()
            if payload is not None:
                self._publisher.put(payload)
                published = time.monotonic()
            if tick % 2 == 0:
                self._gripper_publisher.put(gripper)
            tick += 1
            time.sleep(0.01)

    def _gripper_target(self, sample):
        msg = np.frombuffer(sample.payload.to_bytes(), _wire.GRIPPER)[0]
        if self.gripper_script is not None:
            threading.Thread(target=self.gripper_script, args=(msg,), daemon=True).start()

    def _lease(self, sample):
        client = int(str(sample.key_expr).rsplit("/", 1)[1])
        with self.lock:
            if sample.kind == zenoh.SampleKind.PUT:
                self.leases.add(client)
            else:
                self.leases.discard(client)

    def _target(self, sample):
        payload = sample.payload.to_bytes()
        msg = np.frombuffer(payload, _wire.TARGET)[0]
        with self.lock:
            self.targets.append((time.monotonic(), msg))
            if self.track and self.state["phase"] == 2:
                self.state["seq_accepted"] = msg["seq"]
                self.state["t_send_ns_accepted"] = msg["t_send_ns"]
                self.state["target"] = msg["data"]
                if self.state["flags"] & _wire.FLAG_JOINTS:
                    self.state["q"] = msg["data"]
                else:
                    self.set_pose(msg["data"])

    def _query(self, query):
        verb = str(query.key_expr).rsplit("/", 1)[1]
        request = json.loads(query.payload.to_bytes())
        with self.lock:
            self.requests.append((verb, request))
            reply = self.overrides.get(verb) or self._verb(verb, request)
        query.reply(query.key_expr, json.dumps(reply))

    def _verb(self, verb, request):
        """`franka-node`'s rules of `arm/commands.rs`, without the robot."""
        s, client = self.state, request["client_id"]
        phase, holder = _wire.PHASES[int(s["phase"])], int(s["client_id"])

        def refuse(reason):
            return {"ok": False, "error": reason}

        not_holder = refuse(f"client {client} is not the holder")
        not_allowed = refuse(f"not allowed in {phase}")
        if verb == "acquire":
            if client not in self.leases:
                return refuse("no lease")
            if holder not in (0, client):
                return refuse(f"held by {holder}")
            if phase not in ("idle", "acquired"):
                return not_allowed
            s["client_id"], s["phase"] = client, 1
        elif verb == "release":
            if holder != client:
                return not_holder
            if phase in ("active", "homing"):
                return not_allowed
            s["client_id"] = 0
            if phase == "acquired":
                s["phase"] = 0
        elif verb == "enable":
            if holder != client:
                return not_holder
            if phase not in ("idle", "acquired"):
                return not_allowed
            joints = request.get("mode") == "joints"
            s["phase"], s["refused"], s["accepted"], s["seq_accepted"] = 2, 0, 0, 0
            s["flags"] = _wire.FLAG_JOINTS if joints else 0
            s["target"] = s["q"] if joints else self.POSE
        elif verb == "stop":
            if phase not in ("active", "homing"):
                return not_allowed
            s["phase"], s["flags"] = (0 if phase == "active" else 1), 0
        elif verb == "recover":
            if phase in ("active", "homing"):
                return not_allowed
            s["phase"] = 0
        elif verb in ("home", "gripper_home"):
            if holder != client:
                return not_holder
            if verb == "home" and phase != "acquired":
                return not_allowed
        return {"ok": True}

    def close(self):
        self._running = False
        self._thread.join()
        self.session.close()


@pytest.fixture
def fake():
    node = FakeNode()
    yield node
    node.close()


@pytest.fixture
def arm(fake):
    """The fake's arm, acquired by a fresh `Node`."""
    with franka_node.Node(fake.endpoint) as node:
        arm = node.arm(fake.ARM)
        arm.state(timeout=5.0)
        with arm:
            yield arm


@pytest.fixture(scope="session")
def node_endpoint(sim, tmp_path_factory):
    """The `franka-node` binary (FRANKA_NODE_BIN, default target/release/franka-node) serving
    arm `sim` with the Hand on the loopback; SIGINT on teardown, killed after 10 s."""
    binary = pathlib.Path(os.environ.get("FRANKA_NODE_BIN", REPO / "target/release/franka-node"))
    if not binary.is_file():
        pytest.fail(f"{binary} missing: cargo build --release -p franka-node")
    directory = tmp_path_factory.mktemp("franka-node")
    config = directory / "node.toml"
    # franka-sim's O_T_EE is the joint-7 frame, 0.107 m from the pose the loop tracks, and the
    # node measures an anchored step and the lead from it: max_step and max_lead are the
    # defaults (0.05) plus that offset, the rotation limits the defaults.
    config.write_text(f"""name = "{NODE_NAME}"

[zenoh]
listen = ["{NODE_ENDPOINT}"]
multicast_scouting = false

[[arm]]
name = "{SIM_ARM}"
host = "{sim}"
realtime = "ignore"
gripper = "hand"
max_step = 0.157
max_lead = 0.157
max_step_rotation = 0.26
max_lead_rotation = 0.26
""")
    log = open(directory / "node.log", "w")
    process = subprocess.Popen([str(binary), str(config)], stdout=log, stderr=subprocess.STDOUT)
    try:
        yield NODE_ENDPOINT
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        print((directory / "node.log").read_text()[-4000:])
