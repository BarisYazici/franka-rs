"""Where teleop.py's targets go: a stand-in arm for --dry-run, or franka-node over Zenoh.

A sink hands out its latest arm state as one `ArmState`, replaced whole on every StateMsg, so
the measured pose, dq and the arrival stamp the teleop reads always come from the same
message; `clock_ns()` is the clock that stamp is on.

The node's own limits come through here too: `connect()` opens the session and nothing else,
`fetch_limits()` reads `franka/<arm>/params/{schema,get}`, and `franka/<arm>/params/current`
is subscribed so the loop thread can pick up a changed budget. teleop.py validates against
those numbers BEFORE `start()` acquires anything.

Recovery (ZenohSink, unless --no-auto-recover): an arm that reads Faulted, Reflex or has
errors before acquire is recovered first; mid-session teleop.py calls `recover()`. The node's
`recover` blocks its arm thread for about 4 s, so a query that times out is not a failure:
the state decides, polled until it reads Idle with no errors.
"""
import json
import math
import sys
import time
from typing import NamedTuple, Optional, Tuple

from franka_node import ProtocolError, _wire

from . import node_limits, teleop_params
from .geometry import dist, qangle, qslerp, quat_of

PHASE = {0: "Idle", 1: "Acquired", 2: "Active", 3: "Stopping", 4: "Faulted", 5: "Homing"}
ROBOT_MODE = {0: "Other", 1: "Idle", 2: "Move", 3: "Guiding", 4: "Reflex", 5: "UserStopped",
              6: "AutomaticErrorRecovery"}
IDLE, ACTIVE, FAULTED = 0, 2, 4
REFLEX, AUTO_RECOVERY = 4, 6
GRIPPER_FLAGS = ("calibrated", "grasped", "moving", "fault")     # GripperStateMsg bits 0..3
QUERY_S = 10.0              # every cmd query's own timeout
RECOVER_QUERY_S = 6.0       # recover's: past it the state decides, and Ctrl-C is not blocked
STATE_WAIT_S = 3.0          # for the first state, and for Active after enable
PARAMS_QUERY_S = 2.0        # params/schema and params/get, before anything is acquired
RECOVER_WAIT_S = 20.0       # from sending recover to a state that reads Idle with no errors
READY_O_T_EE = (1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0.307, 0.0, 0.487, 1.0)


class ArmState(NamedTuple):
    t_ns: int                   # arrival, on the sink's clock_ns()
    measured: Tuple             # (p, q) of O_T_EE
    desired: Tuple              # (p, q) of the node's current target
    dq: Optional[Tuple]         # measured joint velocities, rad/s; None without joints

    def finite(self):
        """Every pose and dq number is finite: a NaN passes every `>` leash check."""
        (p, q), (tp, tq) = self.measured, self.desired
        return all(math.isfinite(c) for c in (*p, *q, *tp, *tq, *(self.dq or ())))


def fmt3(p):
    return "(" + ", ".join(f"{c:+.4f}" for c in p) + ")"


def fmt4(q):
    return "(" + ", ".join(f"{c:+.3f}" for c in q) + ")"


def _fields(raw, dtype, name):
    """One node message as plain Python values, on franka-node-client's own layout."""
    rec = _wire._record(raw, dtype, name)
    return {field: rec[field].tolist() for field in dtype.names}


def decode_state(raw, t_ns):
    """(ArmState, the non-motion fields) of one StateMsg."""
    f = _fields(raw, _wire.STATE, "state")
    m, target = f["o_t_ee"], f["target"]                # O_T_EE column-major
    q = quat_of(m) if all(math.isfinite(c) for c in m) else (math.nan,) * 4
    arm = ArmState(t_ns, ((m[12], m[13], m[14]), q),
                   (tuple(target[0:3]), tuple(target[3:7])), tuple(f["dq"]))
    info = dict(phase=f["phase"], robot_mode=f["robot_mode"], has_errors=f["has_errors"],
                flags=f["flags"], holder=f["client_id"], success=f["success_rate"],
                accepted=f["accepted"], refused=f["refused"], dropped=f["dropped"])
    return arm, info


class SimArm:
    """A stand-in arm for --dry-run: first-order tracking of the last target at the node's
    default Cartesian budget. NOT a model of the impedance loop -- it exists so the lead
    clamp, the step clamp and the summary's numbers are exercised with no robot."""

    def __init__(self, speed=0.3, rot_speed=0.5):
        self.p = READY_O_T_EE[12:15]
        self.q = quat_of(READY_O_T_EE)
        self.target = None
        self.speed, self.rot_speed = speed, rot_speed
        self.t = None

    def integrate(self, t):
        if self.t is None:
            self.t = t
        dt, self.t = max(0.0, (t - self.t) / 1e9), t
        if self.target is None or dt <= 0.0:
            return
        tp, tq = self.target
        d = dist(tp, self.p)
        step = self.speed * dt
        self.p = tp if d <= step else tuple(
            a + (b - a) * step / d for a, b in zip(self.p, tp))
        angle = qangle(self.q, tq)
        self.q = tq if angle <= self.rot_speed * dt else qslerp(
            self.q, tq, self.rot_speed * dt / angle)


class DryRunSink:
    """Decode, map, clamp and print. Touches no Zenoh at all. Its state is the stand-in arm
    at the stream's own time, so it is never stale, and it has no joints."""

    live = False
    gripper_ready = True

    def __init__(self, args, out=sys.stdout):
        self.arm = SimArm()
        self.out = out
        self.max_width = args.gripper_max_width
        self.quiet = args.quiet
        self.printed = 0
        self.every = args.print_every

    def connect(self):
        pass

    def disconnect(self):
        pass

    def take_params(self):
        """No node, so no params/current; the limits are whatever --node-limits gave."""
        return None

    def serve_params(self, owner):
        pass

    def start(self):
        p, q = self.arm.p, self.arm.q
        self.say(f"[dry-run] no Zenoh. simulated arm at p {fmt3(p)} q {fmt4(q)}")

    def say(self, line):
        if not self.quiet:
            print(line, file=self.out, flush=True)

    def state(self):
        pose = (self.arm.p, self.arm.q)
        return ArmState(self.clock_ns(), pose, pose, None)

    def clock_ns(self):
        return self.arm.t or 0

    def tick(self, t):
        self.arm.integrate(t)

    def put_target(self, raw, pos, quat, seq, why):
        self.arm.target = (pos, quat)
        self.printed += 1
        if self.every and self.printed % self.every == 0:
            self.say(f"[target] seq {seq:6d} {why:<9s} p {fmt3(pos)} q {fmt4(quat)} "
                     f"lead {dist(pos, self.arm.p) * 1000:5.1f} mm")

    def put_gripper(self, raw, width, seq):
        self.say(f"[gripper] seq {seq:4d} width {width * 1000:5.1f} mm")

    def fault(self):
        return None

    def recoverable(self):
        return False

    def gripper_info(self):
        return None

    def counters(self):
        return None

    def stop(self):
        pass


class ZenohSink:
    """The node's session dance, in order: declare a liveliness token on
    franka/<arm>/lease/<client>, subscribe to state, gripper state and params/current, read
    the node's limits, query the acquire and enable commands, then publish TargetMsg on
    franka/<arm>/target and GripperMsg on franka/<arm>/gripper/target until stop and
    release."""

    live = True

    def __init__(self, args):
        # zenoh is imported by connect(), not here: constructing a sink must not need the
        # module, so the offline tests drive this class on a fake session without it.
        self.zenoh = None
        self.args = args
        self.arm = args.arm
        self.client = args.client_id
        self.max_width = args.gripper_max_width
        self.info = {}
        self.latest = None
        self.session = self.token = self.pub = self.gpub = None
        self.enabled = False
        self.gripper_ready = False
        self.gripper_states = self.gripper_bad = 0
        self.gripper_flags = None
        self.params_body = None      # the latest params/current, from the Zenoh thread
        self.params_bad = 0
        self.owner = self.params_pub = None
        self.sleep, self.monotonic = time.sleep, time.monotonic     # replaced by the tests
        self.cancelled = lambda: False                             # teleop.py's Ctrl-C

    def _import_zenoh(self):
        if self.zenoh is not None:
            return self.zenoh
        try:
            import zenoh
        except ImportError as exc:
            raise SystemExit(
                f"cannot import zenoh ({exc}).\n"
                "The live path needs eclipse-zenoh AND pyzmq in one interpreter: install\n"
                "franka-vr-teleop into the venv you run this from, or pass --dry-run.") from exc
        self.zenoh = zenoh
        return zenoh

    def connect(self):
        """The session, its subscribers and its publishers. NOTHING is acquired or enabled
        here: the caller reads the node's limits and validates against them first."""
        if self.session is not None:
            return
        a = self.args
        zenoh = self._import_zenoh()
        conf = zenoh.Config()
        conf.insert_json5("mode", '"peer"')
        conf.insert_json5("connect/endpoints", json.dumps([a.connect]))
        conf.insert_json5("scouting/multicast/enabled", "false")
        self.session = zenoh.open(conf)
        self.token = self.session.liveliness().declare_token(
            f"franka/{self.arm}/lease/{self.client}")
        self.session.declare_subscriber(f"franka/{self.arm}/state", self._on_state)
        self.session.declare_subscriber(
            f"franka/{self.arm}/gripper/state", self._on_gripper_state)
        self.session.declare_subscriber(f"franka/{self.arm}/params/current", self._on_params)
        self.pub = self.session.declare_publisher(f"franka/{self.arm}/target")
        self.gpub = self.session.declare_publisher(f"franka/{self.arm}/gripper/target")
        self.sleep(0.5)                              # the lease token reaches the node first

    def disconnect(self):
        """Close a session that never acquired anything: --probe-limits' way out. `stop()`
        is the one that stops and releases a session that did."""
        if self.session is not None:
            self.session.close()
            self.session = None

    def fetch_limits(self):
        """This arm's limits, from the node itself. Raises `node_limits.LimitsError` when the
        node cannot be reached or does not publish them -- the caller exits, never guesses."""
        return node_limits.query(self.session, self.arm, PARAMS_QUERY_S, self.args.connect)

    def _on_params(self, sample):
        """Zenoh thread: keep the latest body and nothing else, as `_on_state` does."""
        try:
            self.params_body = json.loads(bytes(sample.payload).decode())
        except Exception:
            self.params_bad += 1

    def take_params(self):
        """The loop thread's read of that one slot; latest wins, and it clears."""
        body, self.params_body = self.params_body, None
        return body

    def serve_params(self, owner):
        """Publish teleop's own parameter surface beside the node's. Its `set` queryable runs
        on a Zenoh thread and only validates; the loop thread applies."""
        self.owner = owner
        self.params_pub = teleop_params.serve(self.session, self.arm, owner)

    def start(self):
        self.connect()
        self.bring_up()
        print(f"{self.arm}: Active, measured p {fmt3(self.latest.measured[0])}", flush=True)
        if not self.gripper_ready:
            print(f"{self.arm}: WARNING no franka/{self.arm}/gripper/state received: the "
                  f"trigger drives nothing until one arrives", flush=True)

    def bring_up(self):
        """Wait for the node's state, recover a faulted arm unless --no-auto-recover, then
        acquire and enable. Raises SystemExit with the reason."""
        if not self._await(lambda: "phase" in self.info, STATE_WAIT_S):
            if self.cancelled():
                raise SystemExit(f"{self.arm}: interrupted")
            print(f"{self.arm}: no state within {STATE_WAIT_S:.0f} s", flush=True)
        elif self.needs_recovery():
            if self.args.no_auto_recover:
                raise SystemExit(f"{self.arm}: {self.describe()} before acquire; recover it "
                                 f"or drop --no-auto-recover")
            holder = self.info.get("holder", 0)
            if holder not in (0, self.client):       # recover needs no lease: not ours to reset
                raise SystemExit(f"{self.arm}: {self.describe()}, held by client {holder}")
            failure = self._recover()
            if failure:
                raise SystemExit(f"{self.arm}: {failure}")
            print(f"{self.arm}: recovered before acquire", flush=True)
        self._enable()

    def recover(self):
        """Mid-session: recover, wait for Idle with no errors, acquire and enable again with
        the same episode. None once the arm reads Active, else why not."""
        self.enabled = False
        try:
            failure = self._recover()
            if failure is None:
                self._enable(echo=False)
            return failure
        except SystemExit as exc:
            return str(exc)

    def _recover(self):
        deadline = self.monotonic() + RECOVER_WAIT_S
        body, error = self._query("recover", RECOVER_QUERY_S)
        if body is not None and not body.get("ok", False):
            return f"recover refused: {body.get('error')}"
        if error:                                    # maybe still running: the state decides
            print(f"{self.arm} recover: {error}; confirming from the state", flush=True)
        if self._await(self.clean, deadline - self.monotonic()):
            return None
        if self.cancelled():
            return "interrupted"
        return f"not Idle without errors {RECOVER_WAIT_S:.0f} s after recover ({self.describe()})"

    def _enable(self, echo=True):
        if self.cancelled():
            raise SystemExit(f"{self.arm}: interrupted")
        self._verb("acquire", echo)
        extra = {"episode": self.args.episode} if self.args.episode else {}
        self._verb("enable", echo, mode="cartesian", **extra)
        self.enabled = True
        if not self._await(lambda: self.info.get("phase") == ACTIVE, STATE_WAIT_S):
            raise SystemExit(f"{self.arm}: not Active after enable ({self.describe()})")

    def _await(self, ready, seconds):
        deadline = self.monotonic() + seconds
        while not ready():
            if self.monotonic() >= deadline or self.cancelled():
                return False
            self.sleep(0.02)
        return True

    def needs_recovery(self):
        i = self.info
        return i.get("phase") == FAULTED or i.get("robot_mode") == REFLEX or \
            bool(i.get("has_errors"))

    def clean(self):
        i = self.info
        return i.get("phase") == IDLE and not i.get("has_errors") and \
            i.get("robot_mode") not in (REFLEX, AUTO_RECOVERY)

    def describe(self):
        i = self.info
        return (f"phase {PHASE.get(i.get('phase'), i.get('phase'))}, robot_mode "
                f"{ROBOT_MODE.get(i.get('robot_mode'), i.get('robot_mode'))}, errors "
                f"{'yes' if i.get('has_errors') else 'no'}")

    def _query(self, name, timeout=QUERY_S, **extra):
        """(reply body, None) or (None, the transport's error): a timeout is not a refusal."""
        request = json.dumps({"client_id": self.client, **extra})
        try:
            for reply in self.session.get(f"franka/{self.arm}/cmd/{name}",
                                          payload=request, timeout=timeout):
                if reply.ok is None:
                    return None, f"query error {reply.err.payload.to_string()!r}"
                return json.loads(reply.ok.payload.to_string()), None
        except Exception as exc:                     # zenoh's own errors have no public type
            return None, f"query failed: {exc}"
        return None, "no reply"

    def _verb(self, name, echo=True, **extra):
        body, error = self._query(name, **extra)
        if error:
            raise SystemExit(f"{self.arm} {name}: {error}")
        if echo:
            print(f"{self.arm} {name}: {body}", flush=True)
        if not body.get("ok", False):
            raise SystemExit(f"{self.arm} {name} refused: {body.get('error')}")
        return body

    def _on_state(self, sample):
        self.latest, self.info = decode_state(bytes(sample.payload), time.monotonic_ns())

    def _on_gripper_state(self, sample):
        try:
            f = _fields(bytes(sample.payload), _wire.GRIPPER_STATE, "gripper state")
        except ProtocolError:
            self.gripper_bad += 1
            return
        if f["max_width"] > 0.0:                     # a known max_width lands before ready
            self.max_width = f["max_width"]
        self.gripper_flags = f["flags"]
        self.gripper_states += 1
        self.gripper_ready = True

    def state(self):
        return self.latest

    def clock_ns(self):
        return time.monotonic_ns()

    def tick(self, t):
        pass

    def put_target(self, raw, pos, quat, seq, why):
        self.pub.put(raw)

    def put_gripper(self, raw, width, seq):
        self.gpub.put(raw)

    def fault(self):
        """Why the session cannot go on as it is, or None; `recoverable()` says whether
        teleop.py may recover it."""
        phase = self.info.get("phase")
        if phase is None or not self.enabled:
            return None
        if self.info.get("holder") not in (self.client, 0):
            return f"lease taken by client {self.info['holder']}"
        if phase == FAULTED:
            return f"arm Faulted (robot_mode {ROBOT_MODE.get(self.info.get('robot_mode'))})"
        if phase != ACTIVE:
            return f"arm left Active (phase {PHASE.get(phase, phase)})"
        return None

    def recoverable(self):
        i = self.info
        return self.enabled and i.get("phase") == FAULTED and i.get("holder") == self.client

    def gripper_info(self):
        flags = self.gripper_flags
        return {"states": self.gripper_states, "bad_states": self.gripper_bad,
                "max_width": self.max_width,
                "flags": None if flags is None else
                [n for b, n in enumerate(GRIPPER_FLAGS) if flags >> b & 1]}

    def counters(self):
        if "accepted" not in self.info:
            return None
        return (self.info["accepted"], self.info["refused"], self.info["dropped"],
                self.info.get("success", 0.0))

    def stop(self):
        """Stop a session unless the arm reads Faulted (which refuses it), then release."""
        if self.session is None:
            return
        try:
            if self.enabled and self.info.get("phase") != FAULTED:
                self._verb("stop")
        except SystemExit as exc:
            print(f"{self.arm}: shutdown: {exc}", flush=True)
        try:
            self._verb("release")
        except SystemExit as exc:
            print(f"{self.arm}: shutdown: {exc}", flush=True)
        finally:
            self.session.close()
            self.session = None
