"""Which controller drives which arm, and the proof of it before an arm is acquired.

The 88-byte VrTargetMsg carries no hand id, so the port is the only thing that says which
controller a message came from. The bridge runs `--controller l,r` with its endpoints in that
order (`vr.sh` does), and logs `5560 controller=l`, `5570 controller=r`. By default arm L pairs
with the left controller and arm R with the right. A whole session once listened on 5570, the RIGHT
controller, while driving arm L; nothing on the wire or in the process showed it.

Pure logic plus one thin ZMQ listener: selftest.py feeds HandCheck synthetic messages directly
and drives `listen` over inproc:// sockets, so no test binds or connects a TCP port.
"""
import time

from .sources import FLAG_CONTROLLER_ON, FLAG_ENGAGED, FLAG_FRESH, decode_vr, import_zmq

HAND_PORT = {"left": 5560, "right": 5570}    # the bridge's `--controller l,r` order
PORT_HAND = {port: hand for hand, port in HAND_PORT.items()}
ARM_HAND = {"L": "left", "R": "right"}      # the default pairing
OTHER = {"left": "right", "right": "left"}
AWAKE = FLAG_FRESH | FLAG_CONTROLLER_ON
CONFIRM_NS = 100_000_000        # a squeeze must hold this long, five bridge ticks, so the other
                                # port's sample of the same moment has arrived too
CONFIRM_SLACK_NS = 40_000_000   # two ticks: engaged must still be seen at 60 ms or later


def port_of(endpoint):
    tail = endpoint.rsplit(":", 1)[-1]
    return int(tail) if tail.isdigit() else None


def resolve(a, fail):
    """Sets `a.hand`, `a.endpoint` and `a.crossed` from --arm, --hand, --bridge-host and an
    explicit --endpoint; refuses an endpoint on neither hand's port, an endpoint and a --hand
    that name different controllers, and a crossed pairing without --cross."""
    if a.hand_check_seconds <= 0.0:
        fail("--hand-check-seconds must be positive")
    if a.endpoint is not None:
        if a.endpoint.isdigit():
            a.endpoint = f"tcp://{a.bridge_host}:{a.endpoint}"
        via = PORT_HAND.get(port_of(a.endpoint))
        if via is None:
            fail(f"--endpoint {a.endpoint}: the port is neither {HAND_PORT['left']} (left) "
                 f"nor {HAND_PORT['right']} (right)")
        if a.hand is not None and a.hand != via:
            fail(f"--endpoint {a.endpoint} is the {via.upper()} controller, --hand says "
                 f"{a.hand}; --cross pairs an arm with a hand, it does not relabel a port")
        a.hand = via
    paired = ARM_HAND.get(a.arm)
    if a.hand is None:
        if paired is None:
            fail(f"--arm {a.arm} has no default hand (L is left, R is right); pass --hand")
        a.hand = paired
    if a.endpoint is None:
        a.endpoint = f"tcp://{a.bridge_host}:{HAND_PORT[a.hand]}"
    a.crossed = paired is not None and a.hand != paired
    if a.crossed and not a.cross:
        fail(f"arm {a.arm} with the {a.hand.upper()} controller is a crossed pairing (the default "
             f"pairs L with left, R with right); pass --cross if that is meant")


def startup_line(a):
    """The one line that says which hand moves which arm."""
    line = f"arm {a.arm} <- {a.hand.upper()} controller ({a.endpoint})"
    return line + "  CROSSED (--cross)" if a.crossed else line


class HandCheck:
    """The chosen hand's controller is fresh and controller_on on its port, and a squeeze of
    its grip -- a released sample, then an engaged one, after the prompt -- shows `engaged`
    there on every sample of the next CONFIRM_NS. Once the operator is prompted the other port
    must publish (unless `one_controller`) and never show `engaged`. Fed (t_ns, hand,
    payload); no socket and no clock here."""

    def __init__(self, hand, endpoints, seconds, say=print, one_controller=False):
        self.hand, self.other = hand, OTHER[hand]
        self.endpoints = endpoints
        self.window = int(seconds * 1e9)
        self.say = say
        self.one_controller = one_controller
        self.last = {}                   # hand -> (pos, flags) of its latest message
        self.stage = "awake"
        self.deadline = None
        self.released = self.told_release = self.other_seen = False
        self.squeezed_ns = self.held_ns = None
        self.done, self.failure, self.dy = False, None, None

    def label(self, hand):
        return f"{hand.upper()} ({self.endpoints[hand]})"

    def start(self, t):
        self.deadline = t + self.window

    def fail(self, why):
        self.done, self.failure = True, why

    def feed(self, t, hand, payload):
        msg = decode_vr(payload)
        if self.done or isinstance(msg, str):
            return
        pos, _quat, _trigger, flags = msg
        self.last[hand] = (pos, flags)
        awake = (flags & AWAKE) == AWAKE
        if self.stage == "awake":
            if hand == self.hand and awake:
                self.stage, self.deadline = "squeeze", t + self.window
                self.say(f"[hand-check] {self.label(self.hand)} is awake. Squeeze the "
                         f"{self.hand.upper()} grip now, and only that one "
                         f"({self.window / 1e9:g} s)")
            return
        if hand == self.other:
            self.other_seen = True
            if flags & FLAG_ENGAGED:
                self.fail(f"{self.label(self.other)} shows engaged: the wrong grip is held, "
                          f"or the bridge pairs the controllers the other way round")
            return
        held = awake and bool(flags & FLAG_ENGAGED)
        if self.stage == "confirm":
            if not held:
                return self.fail(f"the {self.hand.upper()} grip did not stay engaged for "
                                 f"{CONFIRM_NS / 1e6:.0f} ms: squeeze it and hold it")
            self.held_ns = t
            return
        if not awake:
            return
        if not held:
            self.released = True
        elif not self.released:
            if not self.told_release:
                self.told_release = True
                self.say(f"[hand-check] the {self.hand.upper()} grip is already held: "
                         f"release it, then squeeze")
        else:
            self.stage, self.deadline = "confirm", t + CONFIRM_NS
            self.squeezed_ns = self.held_ns = t

    def passed(self):
        if self.held_ns - self.squeezed_ns < CONFIRM_NS - CONFIRM_SLACK_NS:
            return self.fail(f"{self.label(self.hand)} went quiet during the squeeze: "
                             f"engaged last seen {(self.held_ns - self.squeezed_ns) / 1e6:.0f} "
                             f"ms into it")
        other = self.last.get(self.other)
        if other and other[1] & FLAG_ENGAGED:
            return self.fail(f"{self.label(self.other)} was engaged when last heard from")
        if not self.other_seen:
            if not self.one_controller:
                return self.fail(f"{self.label(self.other)} was silent since the prompt: is "
                                 f"the bridge running both controllers? (--one-controller "
                                 f"if it is not)")
            self.say(f"[hand-check] WARNING: {self.label(self.other)} silent; passed only "
                     f"because of --one-controller")
        self.done = True
        self.say(f"[hand-check] passed: the {self.hand.upper()} grip shows on "
                 f"{self.endpoints[self.hand]} and not on {self.endpoints[self.other]}")
        if self.other_seen and other[1] & FLAG_FRESH:
            left, right = (self.last[self.hand][0], other[0]) if self.hand == "left" \
                else (other[0], self.last[self.hand][0])
            self.dy = left[1] - right[1]
            if self.dy < 0.0:
                self.say(f"[hand-check] WARNING: y(left) - y(right) = {self.dy:+.2f} m < 0; "
                         f"+y is the operator's left, so the controllers may be in the "
                         f"wrong hands")

    def tick(self, t):
        if self.done or self.deadline is None or t < self.deadline:
            return
        if self.stage == "confirm":
            return self.passed()
        if self.stage == "squeeze":
            return self.fail(f"no squeeze of the {self.hand.upper()} grip on "
                             f"{self.endpoints[self.hand]} within {self.window / 1e9:g} s")
        seen, other = self.last.get(self.hand), self.last.get(self.other)
        why = (f"{self.label(self.hand)} is not awake: "
               + ("no message at all" if seen is None
                  else f"last flags {seen[1]:#04x}, want fresh and controller_on"))
        if other and (other[1] & AWAKE) == AWAKE:
            why += f"; the {self.other.upper()} controller is awake"
        self.fail(why)


def listen(check, ctx=None, now=time.monotonic_ns, poll_ms=20):
    """Runs `check` on both of its endpoints until it passes or fails; returns the failure,
    or None. Every sample is read (no CONFLATE): a squeeze is an edge."""
    zmq = import_zmq()
    context = ctx or zmq.Context()
    poller, socks = zmq.Poller(), {}
    try:
        for hand, endpoint in check.endpoints.items():
            sock = context.socket(zmq.SUB)
            sock.setsockopt(zmq.SUBSCRIBE, b"")
            sock.setsockopt(zmq.LINGER, 0)
            sock.connect(endpoint)
            poller.register(sock, zmq.POLLIN)
            socks[sock] = hand
        check.start(now())
        while not check.done:
            for sock, _ in poller.poll(poll_ms):
                while True:
                    try:
                        payload = sock.recv(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    check.feed(now(), socks[sock], payload)
            check.tick(now())
    finally:
        for sock in socks:
            sock.close(0)
        if ctx is None:
            context.term()
    return check.failure


def check_hands(a, say=print):
    """The pre-acquire check on the bridge's two ports; None when it passed."""
    other = OTHER[a.hand]
    endpoints = {a.hand: a.endpoint,
                 other: f"{a.endpoint.rsplit(':', 1)[0]}:{HAND_PORT[other]}"}
    say(f"[hand-check] listening on {endpoints['left']} (left) and {endpoints['right']} "
        f"(right), up to {a.hand_check_seconds:g} s per step")
    if a.one_controller:
        say(f"[hand-check] WARNING: --one-controller: a silent {other} port will not fail "
            f"the check")
    return listen(HandCheck(a.hand, endpoints, a.hand_check_seconds, say, a.one_controller))
