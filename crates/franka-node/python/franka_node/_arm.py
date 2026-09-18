"""`Arm`: the state subscriber, the lease, the verbs and the sessions of one node-served arm."""

import logging
import threading
import time
import warnings

from . import _wire
from ._errors import NodeError, NodeTimeout, ProtocolError, Refused
from ._gripper import Gripper
from ._targets import CartesianTargets, JointTargets
from ._wait import wait_for

log = logging.getLogger("franka_node")

# A session ends when no state came for the longer of these: s, and state periods.
SILENT_MIN = 2.0
SILENT_PERIODS = 3


class Subscription:
    """An `on_state` callback's registration; `close()` removes it."""

    def __init__(self, arm, callback):
        self._arm, self._callback = arm, callback

    def close(self):
        self._arm._unlisten(self._callback)


class Arm:
    """The arm `franka/<name>/*`, from `Node.arm(name)`. Reading and `stop`/`recover` work
    without the lease; `with arm:` acquires it and gives it back on exit."""

    def __init__(self, node, name):
        self._node, self.name = node, name
        self._cv = threading.Condition()
        self._latest = None
        self._error = None
        self._listeners = []
        self._token = None
        self._session = None
        self._period_ns = None  # the state period, as observed
        zenoh_session = node._session
        self._target_pub = zenoh_session.declare_publisher(f"franka/{name}/target")
        self._gripper_pub = zenoh_session.declare_publisher(f"franka/{name}/gripper/target")
        self._subscriber = zenoh_session.declare_subscriber(f"franka/{name}/state", self._on_sample)
        self.gripper = Gripper(self)

    # -- state ----------------------------------------------------------------------------

    def _on_sample(self, sample):
        received = time.monotonic_ns()
        try:
            state = _wire.decode_state(sample.payload.to_bytes(), self._node.client_id, received)
        except ProtocolError as e:
            with self._cv:
                if self._error is None:
                    log.error("arm %s: %s", self.name, e)
                self._error = e
            return
        with self._cv:
            if self._latest is not None:
                # Up at once with a longer interval, down slowly: never shorter than the rate.
                interval = received - self._latest.received_ns
                period = self._period_ns or interval
                self._period_ns = max(interval, 0.9 * period + 0.1 * interval)
            self._latest, self._error = state, None
            listeners = list(self._listeners)
            self._cv.notify_all()
        for callback in listeners:
            try:
                callback(state)
            except Exception:
                log.exception("arm %s: on_state callback", self.name)

    def _wait_state(self, accept, timeout):
        """The latest state once `accept(state)`; `NodeTimeout` after `timeout` s."""
        def poll():
            if self._error is not None:
                raise self._error
            latest = self._latest
            return latest if latest is not None and accept(latest) else None

        state = wait_for(self._cv, poll, timeout)
        if state is None:
            raise NodeTimeout(f"arm {self.name}: no such state within {timeout} s")
        return state

    def _silent_after_ns(self):
        """How long without a state means the node is gone: 3 state periods, at least 2 s."""
        return max(SILENT_MIN * 1e9, SILENT_PERIODS * (self._period_ns or 0))

    def state(self, timeout=1.0):
        """The latest `ArmState`, waiting up to `timeout` s for the first."""
        try:
            return self._wait_state(lambda _: True, timeout)
        except NodeTimeout:
            raise NodeTimeout(f"arm {self.name}: no state within {timeout} s") from None

    def on_state(self, callback):
        """Calls `callback(ArmState)` for every state, on a Zenoh thread; it must return quickly."""
        self._listen(callback)
        return Subscription(self, callback)

    def _listen(self, callback):
        with self._cv:
            self._listeners.append(callback)

    def _unlisten(self, callback):
        with self._cv:
            if callback in self._listeners:
                self._listeners.remove(callback)

    # -- verbs ----------------------------------------------------------------------------

    def _cmd(self, verb, **fields):
        self._node._cmd(self.name, verb, **fields)

    def _stop_verb(self, timeout=None):
        """`stop`; `"not allowed in idle"` or `"... acquired"` is success, there being nothing to
        stop. Faulted raises."""
        try:
            self._cmd("stop", timeout=timeout)
        except Refused as e:
            if e.reason not in ("not allowed in idle", "not allowed in acquired"):
                raise

    def acquire(self, timeout=3.0):
        """Declares the lease token and acquires the arm, retrying while the node has not seen
        the token yet."""
        declared = self._token is None
        if declared:
            key = f"franka/{self.name}/lease/{self._node.client_id}"
            self._token = self._node._session.liveliness().declare_token(key)
        deadline = time.monotonic() + timeout
        try:
            while True:
                try:
                    return self._cmd("acquire")
                except Refused as e:
                    if e.reason != "no lease" or time.monotonic() >= deadline:
                        raise
                time.sleep(0.05)
        except BaseException:
            if declared:
                self._undeclare_token()
            raise

    def release(self):
        """Gives the arm back and undeclares the lease token."""
        self._cmd("release")
        self._undeclare_token()

    def _undeclare_token(self):
        token, self._token = self._token, None
        if token is not None:
            token.undeclare()

    def home(self, speed=0.2, *, episode=None, timeout=90.0):
        """Drives to the ready pose at `speed` of the joint limits; returns on arrival. Takes
        the lease first; refused while a session of this client runs."""
        with self._cv:
            if self._session is not None and not self._session._done():
                raise NodeError(f"arm {self.name}: stop the running session before home")
        self.acquire()
        self._cmd("home", speed=speed, episode=episode, timeout=timeout)

    def stop(self, timeout=10.0):
        """Stops whatever session runs on the arm, anyone's."""
        session = self._session
        if session is not None:
            session._halt()
        self._stop_verb(timeout)

    def recover(self, timeout=10.0):
        """`automatic_error_recovery`; a Faulted arm needs it before `acquire` or a session."""
        self._cmd("recover", timeout=timeout)

    # -- sessions -------------------------------------------------------------------------

    def cartesian_targets(self, *, episode=None, max_velocity=0.1, max_angular_velocity=0.3,
                          lead=0.03, angular_lead=0.15, rate=50.0):
        """Enables a Cartesian session; see `CartesianTargets`."""
        return CartesianTargets(
            self, episode, max_velocity, max_angular_velocity, lead, angular_lead, rate)

    def joint_targets(self, *, episode=None, max_velocity=0.5, lead=0.15, rate=50.0):
        """Enables a joints session; `max_velocity` rad/s per joint. See `JointTargets`."""
        return JointTargets(self, episode, max_velocity, lead, rate)

    def _claim(self, session):
        with self._cv:
            if self._session is not None and not self._session._done():
                raise NodeError(f"arm {self.name}: a session of this client is already running")
            if self._session is not None:
                self._listeners = [c for c in self._listeners if c != self._session._observe]
            self._session = session

    def _unclaim(self, session):
        with self._cv:
            if self._session is session:
                self._session = None
        self._unlisten(session._observe)

    # -- lifetime -------------------------------------------------------------------------

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, tb):
        self._exit(quiet=exc_type is not None)
        return False

    def _exit(self, quiet):
        """Stops this client's session, releases the arm if this client holds it, undeclares
        the token; all three are tried, the first failure raised (a warning when `quiet`)."""
        errors = []
        session = self._session
        steps = [session.stop] if session is not None else []
        steps += [self._release_if_held, self._undeclare_token]
        for step in steps:
            try:
                step()
            except Exception as e:
                errors.append(e)
        if errors and not quiet:
            raise errors[0]
        for e in errors:
            warnings.warn(f"franka_node: arm {self.name} exit: {e}", RuntimeWarning, stacklevel=3)

    def _release_if_held(self):
        """Only with a token: after a `release` the latest state may still show the holder."""
        state = self._latest
        if self._token is not None and state is not None and state.holder == self._node.client_id:
            self._cmd("release")

    def _close(self, quiet):
        try:
            self._exit(quiet)
        finally:
            for entity in (self._subscriber, self.gripper._subscriber, self._target_pub,
                           self._gripper_pub):
                entity.undeclare()

    def _abandon(self):
        session = self._session
        if session is not None:
            session._halt()
        if not self._node._owned:
            self._undeclare_token()
