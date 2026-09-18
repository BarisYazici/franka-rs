"""`Node`: the Zenoh session, the client id, the sequence counters and the `cmd/*` queries."""

import atexit
import json
import secrets
import threading
import time
import warnings
import weakref

import zenoh

from . import _wire
from ._errors import NodeError, NodeTimeout, Refused
from ._wait import wait_for

# Seconds a verb waits for the node's reply.
TIMEOUTS = {"enable": 10.0, "stop": 10.0, "recover": 10.0, "home": 90.0, "gripper_home": 40.0}
DEFAULT_TIMEOUT = 3.0
# Zenoh's link lease, ms, as the node's: a client that vanishes loses its lease in about 1 s.
LEASE_MS = 1000

_nodes = weakref.WeakSet()


class Node:
    """A connection to franka-node.

    `Node("tcp/<node-host>:7447")` opens a peer session to that endpoint with multicast
    scouting off; `Node()` scouts. `Node(session=s)` borrows a `zenoh.Session` as it is
    configured (its link lease included) and never closes it. Each `Node` has its own random
    client id, so two processes never share one.
    """

    def __init__(self, connect=None, *, session=None):
        if session is not None and connect is not None:
            raise ValueError("pass connect or session, not both")
        self._owned = session is None
        if session is None:
            config = zenoh.Config()
            config.insert_json5("mode", '"peer"')
            config.insert_json5("listen/endpoints", "[]")
            if connect is not None:
                endpoints = [connect] if isinstance(connect, str) else list(connect)
                config.insert_json5("connect/endpoints", json.dumps(endpoints))
                config.insert_json5("scouting/multicast/enabled", "false")
            config.insert_json5("transport/link/tx/lease", str(LEASE_MS))
            session = zenoh.open(config)
        self._session = session
        self._client_id = secrets.randbelow(2**32 - 1) + 1
        self._lock = threading.Lock()
        self._seq = {"target": 0, "gripper": 0}
        self._arms = {}
        self._closed = False
        _nodes.add(self)

    @property
    def client_id(self):
        """This client's id in the lease key and in every message, a random non-zero u32."""
        return self._client_id

    def _next_seq(self, counter):
        """The next of the `"target"` or `"gripper"` sequence, from 1, never reset."""
        with self._lock:
            self._seq[counter] += 1
            return self._seq[counter]

    def arm(self, name):
        """The `Arm` served as `franka/<name>/*`; one per name, cached."""
        from ._arm import Arm

        with self._lock:
            if self._closed:
                raise NodeError("the node is closed")
            if name not in self._arms:
                self._arms[name] = Arm(self, name)
            return self._arms[name]

    def status(self, name=None, timeout=2.0):
        """The next `franka/node/<name>/status` sample (any node without `name`) as a dict."""
        samples, arrived = [], threading.Condition()

        def on_sample(sample):
            with arrived:
                samples.append(sample.payload.to_bytes())
                arrived.notify_all()

        key = f"franka/node/{name or '*'}/status"
        subscriber = self._session.declare_subscriber(key, on_sample)
        try:
            payload = wait_for(arrived, lambda: samples[0] if samples else None, timeout)
            if payload is None:
                raise NodeTimeout(f"no status on {key} within {timeout} s")
            return json.loads(payload)
        finally:
            subscriber.undeclare()

    def _cmd(self, arm, verb, *, timeout=None, **fields):
        """One `franka/<arm>/cmd/<verb>` round trip; `Refused` with the node's reason."""
        timeout = TIMEOUTS.get(verb, DEFAULT_TIMEOUT) if timeout is None else timeout
        payload = _wire.encode_request(self._client_id, **fields)
        replies = self._session.get(f"franka/{arm}/cmd/{verb}", payload=payload, timeout=timeout)
        deadline = time.monotonic() + timeout + 1.0
        while time.monotonic() < deadline:
            try:
                reply = replies.try_recv()
            except zenoh.ZError:
                break  # closed: the query timed out without a reply
            if reply is None:
                time.sleep(0.005)
                continue
            if reply.ok is None:
                raise NodeError(f"{verb}: {reply.err.payload.to_string()}")
            answer = json.loads(reply.ok.payload.to_bytes())
            if answer.get("ok"):
                return
            raise Refused(verb, answer.get("error", ""))
        raise NodeTimeout(f"{verb}: no reply from the node within {timeout} s")

    def close(self):
        """Ends every arm as its `with` exit does, then closes the session if it is ours.
        Idempotent."""
        self._close(quiet=False)

    def _close(self, quiet):
        """`quiet`: an exception is on its way, so cleanup failures are only warnings."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            arms = list(self._arms.values())
        errors = []
        for arm in arms:
            try:
                arm._close(quiet)
            except Exception as e:  # every arm is closed; the first failure is raised
                errors.append(e)
        if self._owned:
            self._session.close()
        if errors and not quiet:
            raise errors[0]
        for e in errors:
            warnings.warn(f"franka_node: close: {e}", RuntimeWarning, stacklevel=3)

    def _abandon(self):
        """At interpreter exit: halt the streamers and close the session, no verbs. The lease
        token goes with the session, which stops the arm on the node."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            arms = list(self._arms.values())
        for arm in arms:
            arm._abandon()
        if self._owned:
            self._session.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self._close(quiet=exc_type is not None)
        return False


@atexit.register
def _exit():
    for node in list(_nodes):
        node._abandon()
