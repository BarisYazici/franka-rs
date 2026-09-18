"""`Gripper`: the arm's gripper through `franka/<arm>/gripper/*`. A command is fire and forget
on the node's side, so the waits read the 20 Hz state."""

import logging
import math
import threading
import time

from . import _wire
from ._errors import NodeError, NodeTimeout, ProtocolError
from ._wait import wait_for

log = logging.getLogger("franka_node")

# The Hand driver does not resend a width within this of the last one, m.
DEADBAND = 0.001
# `move` counts as arrived within this, m.
ARRIVED = 0.003
# How long a command may take to show in the state, s.
TAKEN_WITHIN = 1.0
# How long, s, to wait for a reading newer than the one at rest: the driver clears `moving` when
# the command returns, before its 20 Hz reader has read the final width.
FRESH_WITHIN = 0.2


class Gripper:
    """`arm.gripper`. `move`, `grasp` and `homing` need the arm's lease; `stop` and `state`
    do not."""

    def __init__(self, arm):
        self._arm = arm
        self._cv = threading.Condition()
        self._latest = None
        self._error = None
        key = f"franka/{arm.name}/gripper/state"
        self._subscriber = arm._node._session.declare_subscriber(key, self._on_sample)

    def _on_sample(self, sample):
        received = time.monotonic_ns()
        try:
            state = _wire.decode_gripper_state(sample.payload.to_bytes(), received)
        except ProtocolError as e:
            with self._cv:
                if self._error is None:
                    log.error("arm %s: %s", self._arm.name, e)
                self._error = e
            return
        with self._cv:
            self._latest, self._error = state, None
            self._cv.notify_all()

    def _wait(self, accept, timeout):
        """The latest state once `accept(state)`, or `None` after `timeout` s."""
        def poll():
            if self._error is not None:
                raise self._error
            latest = self._latest
            return latest if latest is not None and accept(latest) else None

        return wait_for(self._cv, poll, timeout)

    def state(self, timeout=1.0):
        """The latest `GripperState`, waiting up to `timeout` s for the first."""
        state = self._wait(lambda _: True, timeout)
        if state is None:
            raise NodeTimeout(f"arm {self._arm.name}: no gripper state within {timeout} s")
        return state

    def _require_lease(self):
        arm = self._arm
        try:
            arm._wait_state(lambda s: s.holder == arm._node.client_id, 0.5)
        except NodeTimeout:
            raise NodeError(
                f"arm {arm.name}: the gripper needs the lease; acquire the arm") from None

    def _send(self, kind, width, force):
        node = self._arm._node
        sent = time.monotonic_ns()
        seq = node._next_seq("gripper")
        self._arm._gripper_pub.put(_wire.encode_gripper(kind, node.client_id, seq, sent, width,
                                                        force))
        return sent

    def _finish(self, sent, taken, timeout):
        """Waits for a state after `sent` that shows the command taken, `moving` or `taken(state)`
        (a command quicker than one state period is never seen moving), then for rest, then for
        a reading newer than the one at rest; the last state."""
        started = self._wait(lambda s: s.received_ns > sent and (s.moving or taken(s)),
                             TAKEN_WITHIN)
        if started is None:
            raise NodeError("gripper command not taken (refused or lost)")
        done = self._wait(lambda s: s.received_ns >= started.received_ns and not s.moving, timeout)
        if done is None:
            raise NodeTimeout(f"arm {self._arm.name}: gripper still moving after {timeout} s")
        return self._wait(lambda s: s.t_node_ns > done.t_node_ns, FRESH_WITHIN) or done

    @staticmethod
    def _check_width(width, state):
        if not (math.isfinite(width) and width >= 0.0
                and (state.max_width <= 0.0 or width <= state.max_width)):
            raise ValueError(f"width {width} m is outside 0..={state.max_width}")

    def move(self, width, *, wait=True, timeout=10.0):
        """Moves to `width` m. With `wait`, returns whether it ended within 3 mm; without, `True`
        once sent."""
        state = self.state()
        self._check_width(width, state)
        self._require_lease()
        sent = self._send("width", width, 0.0)
        if not wait:
            return True
        near = min(abs(width - state.width), abs(width - state.commanded)) <= DEADBAND
        if near and not state.moving:
            return abs(width - state.width) <= ARRIVED
        done = self._finish(sent, lambda s: s.commanded == width, timeout)
        return abs(done.width - width) <= ARRIVED

    def grasp(self, width, force, *, wait=True, timeout=10.0):
        """Closes on an object at `width` m with `force` N. With `wait`, returns `is_grasped`
        at the end; without, `True` once sent.

        A grasp shows as taken by `moving`, a width change or a change of `is_grasped`. One that
        ends within a state period without either, such as franka-sim's grasp at the current
        width, raises `NodeError` as not taken."""
        state = self.state()
        self._check_width(width, state)
        if not (math.isfinite(force) and force >= 0.0):
            raise ValueError(f"force {force} N must be finite and non-negative")
        self._require_lease()
        sent = self._send("grasp", width, force)
        if not wait:
            return True

        def taken(s):
            return abs(s.width - state.width) > DEADBAND or s.is_grasped != state.is_grasped

        return self._finish(sent, taken, timeout).is_grasped

    def homing(self, timeout=40.0):
        """Homes the gripper; returns `True` when done, `Refused` otherwise."""
        self._require_lease()
        self._arm._cmd("gripper_home", timeout=timeout)
        return True

    def stop(self):
        """Stops a move or grasp; anyone's."""
        self._arm._cmd("gripper_stop")
