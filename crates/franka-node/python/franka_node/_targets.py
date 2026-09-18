"""Target sessions: the streamer thread that paces goals under the node's guard, and its math.

Each cycle the target moves toward the goal by at most `max_velocity / rate` (and
`max_angular_velocity / rate`), then is projected into the ball of `lead` (`angular_lead`)
around the latest measured pose. Joints: per-joint, plus a step cap of `JOINT_STEP_CAP` per
message that always holds, because the node has no joint anchor.
"""

import contextlib
import logging
import threading
import time
import warnings

import numpy as np

from . import _wire
from ._errors import NodeError, NodeTimeout, SessionEnded
from ._wait import wait_for

log = logging.getLogger("franka_node")

# rad per joint per message, under the node's `max_step_joint` 0.2.
JOINT_STEP_CAP = 0.15
RATE_RANGE = (5.0, 200.0)
# How long `stop` waits for the streamer to finish its cycle, s.
JOIN_TIMEOUT = 1.0
# A stalled Python thread and the queued Zenoh callback compete to run again. Give that
# callback up to 250 ms (several streamer ticks), without resetting the node-silence timeout.
STALL_GRACE_NS = 250_000_000


def rotated(quaternion, rotation_vector):
    """`quaternion` (x, y, z, w) turned by `rotation_vector` (rad, base frame): the
    composition `CartesianTargets.move_by` applies, as `franka.rotated`."""
    q = np.asarray(quaternion, dtype=float)
    r = np.asarray(rotation_vector, dtype=float)
    angle = np.linalg.norm(r)
    axis = r / angle if angle > 0.0 else r
    a = np.r_[np.sin(angle / 2) * axis, np.cos(angle / 2)]
    return np.r_[a[3] * q[:3] + q[3] * a[:3] + np.cross(a[:3], q[:3]), a[3] * q[3] - a[:3] @ q[:3]]


def normalized(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError(f"not a quaternion: {quaternion}")
    return q / norm


def angle_between(a, b):
    """rad between two unit quaternions, sign-insensitive."""
    return 2.0 * np.arccos(min(1.0, abs(float(np.dot(a, b)))))


def toward(a, b, max_distance):
    """`b`, or the point `max_distance` from `a` on the way to it."""
    d = b - a
    n = np.linalg.norm(d)
    return b if n <= max_distance else a + d * (max_distance / n)


def slerp_toward(a, b, max_angle):
    """`b` in `a`'s hemisphere, or the rotation `max_angle` from `a` on the way to it."""
    if np.dot(a, b) < 0.0:
        b = -b
    angle = angle_between(a, b)
    if angle <= max_angle:
        return b
    half, t = angle / 2.0, max_angle / angle
    return normalized((np.sin((1.0 - t) * half) * a + np.sin(t * half) * b) / np.sin(half))


def cartesian_step(current, goal, measured, max_velocity, max_angular_velocity, lead,
                   angular_lead, rate):
    """The next Cartesian target (7,): stepped toward `goal`, then within `lead` of `measured`."""
    position = goal[:3] if max_velocity is None else toward(
        current[:3], goal[:3], max_velocity / rate)
    orientation = goal[3:] if max_angular_velocity is None else slerp_toward(
        current[3:], goal[3:], max_angular_velocity / rate)
    position = toward(measured[:3], position, lead)
    orientation = slerp_toward(measured[3:], normalized(orientation), angular_lead)
    return np.r_[position, normalized(orientation)]


def joint_step(current, goal, measured, max_velocity, lead, rate):
    """The next joint target (7,): stepped toward `goal`, within `lead` of `measured`, and never
    more than `JOINT_STEP_CAP` from `current`."""
    step = JOINT_STEP_CAP if max_velocity is None else min(JOINT_STEP_CAP, max_velocity / rate)
    q = current + np.clip(goal - current, -step, step)
    q = measured + np.clip(q - measured, -lead, lead)
    return current + np.clip(q - current, -JOINT_STEP_CAP, JOINT_STEP_CAP)


def _array(values, columns, what, rows=False):
    """`values` as floats of shape `(columns,)`, or `(N, columns)` with `rows`, all finite."""
    array = np.asarray(values, dtype=float)
    if array.ndim != (2 if rows else 1) or array.shape[-1] not in columns:
        expected = " or ".join(str(c) for c in columns)
        raise ValueError(f"{what}: shape {array.shape}, expected {expected} columns")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{what} must be finite")
    return array


class _Targets:
    _kind = None
    _columns = ()

    def __init__(self, arm, episode, rate):
        if not RATE_RANGE[0] <= rate <= RATE_RANGE[1]:
            raise ValueError(f"rate {rate} Hz is outside {RATE_RANGE[0]}..={RATE_RANGE[1]}")
        self._arm, self._node, self._rate = arm, arm._node, float(rate)
        self._cv = threading.Condition()
        self._chunk = None
        self._goal_seq = None
        self._ended = None
        self._lost = False  # ended here (a silent node, a failed streamer), not by the node
        self._halted = False
        self._stopped = False
        self._warned_at = 0.0
        self._resume_deadline_ns = None
        arm._claim(self)
        try:
            self._start(episode)
        except BaseException:
            arm._unclaim(self)
            raise

    def _start(self, episode):
        arm, cid, joints = self._arm, self._node.client_id, self._kind == "joints"
        arm._cmd("enable", mode=self._kind, episode=episode)
        try:
            state = arm._wait_state(
                lambda s: s.phase == "active" and s.holder == cid and s.joints == joints, 3.0)
        except NodeTimeout:
            with contextlib.suppress(NodeError):
                arm._stop_verb()
            raise NodeTimeout(f"arm {arm.name}: enabled, but no active state within 3 s") from None
        self._current = np.array(state.target)
        self._goal = self._current.copy()
        self._refused = state.refused
        arm._listen(self._observe)
        self._thread = threading.Thread(
            target=self._run, name=f"franka-node-{arm.name}-streamer", daemon=True)
        self._thread.start()

    # -- the node's side ------------------------------------------------------------------

    def _observe(self, state):
        with self._cv:
            self._observe_locked(state)
            self._cv.notify_all()

    def _observe_locked(self, state):
        if self._ended is not None:
            return
        if (state.phase != "active" or state.joints != (self._kind == "joints")
              or state.holder != self._node.client_id):
            self._ended = (state.phase, state.robot_mode, state.has_errors)

    def _check_locked(self):
        self._observe_locked(self._arm._latest)
        if self._ended is not None:
            raise SessionEnded(*self._ended)
        if self._halted:
            state = self._arm._latest
            raise SessionEnded(state.phase, state.robot_mode, state.has_errors)

    def _done(self):
        return self._halted or self._ended is not None

    # -- the streamer ---------------------------------------------------------------------

    def _run(self):
        try:
            self._stream()
        except Exception:
            log.exception("arm %s: the streamer failed, the session is over", self._arm.name)
            with self._cv:
                if self._ended is None:
                    self._ended, self._lost = ("unknown", "unknown", False), True
                self._cv.notify_all()

    def _stream(self):
        arm, cid = self._arm, self._node.client_id
        period, next_tick = 1.0 / self._rate, time.monotonic()
        last_tick = None
        while True:
            with self._cv:
                state = arm._latest
                self._observe_locked(state)
                self._note_silence(state, last_tick)
                last_tick = time.monotonic_ns()
                if self._done():
                    return
                self._advance_chunk(time.monotonic())
                self._note_refusals(state)
                self._current = self._step(self._current, self._goal, state)
                seq = self._node._next_seq("target")
                if self._goal_seq is None and self._equal(self._current, self._goal):
                    self._goal_seq = seq
                data = self._current
            payload = _wire.encode_target(
                self._kind, cid, seq, time.monotonic_ns(), data, anchor=self._kind == "cartesian")
            arm._target_pub.put(payload)
            next_tick += period
            next_tick = max(next_tick, time.monotonic())
            with self._cv:
                while not self._done() and next_tick > time.monotonic():
                    self._cv.wait(next_tick - time.monotonic())

    def _note_silence(self, state, last_tick):
        """Keep the normal silence deadline, with bounded callback grace after a local stall.

        Repeated stalls cannot extend that grace while the latest state remains stale.
        Phase/holder changes are checked separately, before this check, without any grace.
        """
        if self._ended is not None:
            return
        now, silent_after = time.monotonic_ns(), self._arm._silent_after_ns()
        if now - state.received_ns <= silent_after:
            self._resume_deadline_ns = None
            return
        stalled = last_tick is not None and now - last_tick >= silent_after / 2
        if stalled and self._resume_deadline_ns is None:
            self._resume_deadline_ns = now + STALL_GRACE_NS
        if self._resume_deadline_ns is not None and now < self._resume_deadline_ns:
            return
        self._ended, self._lost = ("unknown", "unknown", state.has_errors), True

    def _note_refusals(self, state):
        if state.refused <= self._refused:
            return
        self._refused = state.refused
        if self._kind == "joints":
            self._current = np.array(state.target)
        now = time.monotonic()
        if now - self._warned_at >= 1.0:
            self._warned_at = now
            log.warning("arm %s: %d targets or gripper commands refused in this session "
                        "(the node's debug log has the reasons)", self._arm.name, state.refused)

    def _advance_chunk(self, now):
        if self._chunk is None:
            return
        rows, dt, started = self._chunk
        index = min(int((now - started) / dt), len(rows) - 1)
        if index == len(rows) - 1:
            self._chunk = None
        self._set_goal_locked(rows[index])

    def _set_goal_locked(self, goal):
        if not self._equal(goal, self._goal):
            self._goal = goal
            self._goal_seq = None

    def _halt(self):
        with self._cv:
            self._halted = True
            self._cv.notify_all()
        thread = getattr(self, "_thread", None)
        if thread is not None and thread is not threading.current_thread():
            thread.join(JOIN_TIMEOUT)

    # -- the user's side ------------------------------------------------------------------

    def move_to(self, target):
        """Sets the goal; never blocks. Cancels a `follow`."""
        goal = self._absolute(_array(target, self._columns, "target"))
        with self._cv:
            self._check_locked()
            self._chunk = None
            self._set_goal_locked(goal)

    def follow(self, chunk, dt):
        """`(N, columns)` rows as goals, row `i` at `i * dt` s on the streamer thread; a later
        `move_*` or `follow` replaces it."""
        rows = _array(chunk, self._columns, "chunk", rows=True)
        if not (np.isfinite(dt) and dt > 0.0) or len(rows) == 0:
            raise ValueError("follow needs rows and a positive dt")
        with self._cv:
            self._check_locked()
            self._chunk = ([self._absolute(row) for row in rows], float(dt), time.monotonic())
            self._advance_chunk(time.monotonic())

    def target(self):
        """The goal set by `move_to`, `move_by` or `follow`, shape (7,)."""
        with self._cv:
            self._check_locked()
            return self._goal.copy()

    def state(self, timeout=1.0):
        with self._cv:
            self._check_locked()
        return self._arm.state(timeout)

    def _wait(self, timeout, within):
        def poll():
            self._check_locked()
            state = self._arm._latest
            arrived = (self._chunk is None and self._goal_seq is not None
                       and state.seq_accepted >= self._goal_seq and within(state, self._goal))
            return state if arrived else None

        state = wait_for(self._cv, poll, timeout)
        if state is None:
            raise NodeTimeout(f"arm {self._arm.name}: not at the goal within {timeout} s "
                              f"({self._arm._latest.refused} refused in this session)")
        return state

    def stop(self):
        """Halts the streamer and stops the session if it is still this object's (not ended by the
        node, not replaced by a newer session of this client), this client's and active. A second
        call does nothing."""
        self._halt()
        with self._cv:
            if self._stopped:
                return
            self._stopped = True
            mine = self._ended is None or self._lost
        arm = self._arm
        mine = mine and arm._session is self
        arm._unclaim(self)
        state = arm._latest
        if mine and state.holder == self._node.client_id and state.phase == "active":
            arm._stop_verb()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        except Exception as e:
            if exc_type is None:
                raise
            warnings.warn(f"franka_node: stop() failed: {e}", RuntimeWarning, stacklevel=2)
        return False


class CartesianTargets(_Targets):
    """A Cartesian session: goals are position (m) and unit quaternion (x, y, z, w) in the base
    frame. Every target carries the anchor flag."""

    _kind = "cartesian"
    _columns = (3, 7)

    def __init__(self, arm, episode, max_velocity, max_angular_velocity, lead, angular_lead, rate):
        self._limits = (max_velocity, max_angular_velocity, lead, angular_lead)
        super().__init__(arm, episode, rate)

    def _step(self, current, goal, state):
        measured = np.r_[state.position, state.quaternion]
        return cartesian_step(current, goal, measured, *self._limits, self._rate)

    def _absolute(self, row):
        if row.shape == (3,):
            return np.r_[row, self._goal[3:]]
        return np.r_[row[:3], normalized(row[3:])]

    @staticmethod
    def _equal(a, b):
        return (np.linalg.norm(a[:3] - b[:3]) < 1e-9
                and abs(abs(float(np.dot(a[3:], b[3:]))) - 1.0) < 1e-12)

    def move_by(self, delta):
        """Moves the goal by (dx, dy, dz) m and, with 6 elements, turns it by the rotation
        vector (rad, base frame). Relative to the goal, not the measured pose."""
        delta = _array(delta, (3, 6), "delta")
        with self._cv:
            self._check_locked()
            self._chunk = None
            goal = self._goal.copy()
            goal[:3] += delta[:3]
            if delta.shape == (6,):
                goal[3:] = normalized(rotated(goal[3:], delta[3:]))
            self._set_goal_locked(goal)

    def wait(self, timeout=None, tolerance=0.005, angular_tolerance=0.05):
        """Returns the `ArmState` once the node accepted a target equal to the goal and the
        measured pose is within `tolerance` m and `angular_tolerance` rad of it."""
        def within(state, goal):
            return (np.linalg.norm(state.position - goal[:3]) <= tolerance
                    and angle_between(state.quaternion, goal[3:]) <= angular_tolerance)
        return self._wait(timeout, within)


class JointTargets(_Targets):
    """A joints session: goals are the seven joint positions, rad."""

    _kind = "joints"
    _columns = (7,)

    def __init__(self, arm, episode, max_velocity, lead, rate):
        self._limits = (max_velocity, lead)
        super().__init__(arm, episode, rate)

    def _step(self, current, goal, state):
        return joint_step(current, goal, state.q, *self._limits, self._rate)

    def _absolute(self, row):
        return np.array(row, dtype=float)

    @staticmethod
    def _equal(a, b):
        return float(np.max(np.abs(a - b))) < 1e-12

    def move_by(self, delta):
        """Moves the goal by `delta` (7,), rad."""
        delta = _array(delta, self._columns, "delta")
        with self._cv:
            self._check_locked()
            self._chunk = None
            self._set_goal_locked(self._goal + delta)

    def wait(self, timeout=None, tolerance=0.01):
        """Returns the `ArmState` once the node accepted a target equal to the goal and every
        joint is within `tolerance` rad of it."""
        def within(state, goal):
            return float(np.max(np.abs(state.q - goal))) <= tolerance
        return self._wait(timeout, within)
