#!/usr/bin/env python3
"""Which controller drives which arm, and the pre-acquire check that proves it. Run through
`selftest.py`. The ZMQ listener is driven over inproc:// only; no TCP port is touched."""
import contextlib
import io
import signal

from franka_vr.teleop import cli, hands, sources

from .selftest import AWAKE, HELD, MS, _rejects, _Src, _vr, check

PORTS = {"left": "tcp://127.0.0.1:5560", "right": "tcp://127.0.0.1:5570"}


def _both(t0, t1, left, right, y_left=0.2, y_right=-0.2):
    """Both ports at 50 Hz over [t0, t1) ms, left first on every tick."""
    return [(t, hand, flags, y) for t in range(t0, t1, 20)
            for hand, flags, y in (("left", left, y_left), ("right", right, y_right))]


def _left(t0, t1, flags):
    return [(t, "left", flags, 0.2) for t in range(t0, t1, 20)]


def _check(hand, script, seconds=5.0, one_controller=False):
    lines = []
    c = hands.HandCheck(hand, PORTS, seconds, lines.append, one_controller)
    c.start(0)
    for t, h, flags, y in script:
        c.tick(t * MS)
        c.feed(t * MS, h, _vr(flags, (0.0, y, 0.0)))
        if c.done:
            break
    return c, lines


@check("hands: --hand defaults from --arm and picks the port, left 5560 and right 5570")
def _():
    for argv, hand, endpoint in (
            ([], "left", PORTS["left"]),
            (["--arm", "R"], "right", PORTS["right"]),
            (["--arm", "R", "--bridge-host", "192.0.2.9"], "right", "tcp://192.0.2.9:5570"),
            (["--arm", "L", "--hand", "left"], "left", PORTS["left"]),
            (["--arm", "L", "--endpoint", PORTS["left"]], "left", PORTS["left"]),
            (["--arm", "R", "--endpoint", "5570"], "right", PORTS["right"]),
            (["--arm", "arm3", "--hand", "right"], "right", PORTS["right"])):
        a = cli.parse(["--dry-run", *argv])
        assert (a.hand, a.endpoint, a.crossed) == (hand, endpoint, False), (argv, vars(a))
    assert hands.startup_line(cli.parse(["--dry-run"])) == \
        "arm L <- LEFT controller (tcp://127.0.0.1:5560)"


@check("hands: a crossed pairing, a foreign port or a port that contradicts --hand is refused")
def _():
    for argv in (["--arm", "L", "--hand", "right"], ["--arm", "R", "--hand", "left"],
                 ["--arm", "L", "--endpoint", PORTS["right"]],
                 ["--arm", "R", "--endpoint", PORTS["left"]],
                 ["--hand", "left", "--endpoint", PORTS["right"], "--cross"],
                 ["--endpoint", "tcp://127.0.0.1:5580", "--cross"],
                 ["--arm", "arm3"], ["--hand-check-seconds", "0"]):
        assert _rejects(*argv), f"{argv} was accepted"


@check("hands: --cross accepts a crossed pairing and the startup line says CROSSED")
def _():
    a = cli.parse(["--dry-run", "--arm", "L", "--hand", "right", "--cross"])
    assert (a.hand, a.endpoint, a.crossed) == ("right", PORTS["right"], True)
    assert hands.startup_line(a) == \
        "arm L <- RIGHT controller (tcp://127.0.0.1:5570)  CROSSED (--cross)"
    b = cli.parse(["--dry-run", "--arm", "R", "--endpoint", PORTS["left"], "--cross"])
    assert (b.hand, b.crossed) == ("left", True)
    c = cli.parse(["--dry-run", "--arm", "L", "--cross"])     # nothing crossed: no marker
    assert not c.crossed and "CROSSED" not in hands.startup_line(c)


@check("hand check: awake, released, squeezed on its own port only -> passes, y in order")
def _():
    c, lines = _check("left", _both(0, 200, AWAKE, AWAKE) + _both(200, 400, HELD, AWAKE))
    assert c.done and c.failure is None, (c.failure, lines)
    assert abs(c.dy - 0.4) < 1e-12 and not any("WARNING" in s for s in lines), lines
    c, lines = _check("right", _both(0, 200, AWAKE, AWAKE) + _both(200, 400, AWAKE, HELD))
    assert c.failure is None and c.done, (c.failure, lines)


@check("hand check: y(left) - y(right) < 0 with both fresh passes with a warning")
def _():
    c, lines = _check("left", _both(0, 200, AWAKE, AWAKE, -0.1, 0.3)
                      + _both(200, 400, HELD, AWAKE, -0.1, 0.3))
    assert c.failure is None and abs(c.dy + 0.4) < 1e-12
    assert any("WARNING: y(left) - y(right) = -0.40" in s for s in lines), lines


@check("hand check: a grip already held must be released before its squeeze counts")
def _():
    c, lines = _check("left", _both(0, 300, HELD, AWAKE))
    assert not c.done and any("already held" in s for s in lines), (c.failure, lines)
    c, lines = _check("left", _both(0, 300, HELD, AWAKE) + _both(300, 400, AWAKE, AWAKE)
                      + _both(400, 600, HELD, AWAKE))
    assert c.done and c.failure is None, (c.failure, lines)


@check("hand check: the other port must publish after the prompt, unless --one-controller")
def _():
    silent = [(0, "right", AWAKE, -0.2)] + _left(0, 200, AWAKE) + _left(200, 400, HELD)
    c, _ = _check("left", silent)
    assert c.done and "silent since the prompt" in c.failure, c.failure
    c, lines = _check("left", silent, one_controller=True)
    assert c.done and c.failure is None and c.dy is None, (c.failure, lines)
    assert any("passed only because of --one-controller" in s for s in lines), lines
    held_then_silent = [(0, "right", HELD, -0.2)] + silent[1:]
    c, _ = _check("left", held_then_silent, one_controller=True)
    assert c.done and "engaged when last heard from" in c.failure, c.failure
    saved, built, lines = hands.listen, [], []
    try:
        hands.listen = lambda check, **_: built.append(check)
        hands.check_hands(cli.parse(["--dry-run", "--one-controller"]), lines.append)
    finally:
        hands.listen = saved
    assert built[0].one_controller and any("WARNING: --one-controller" in s for s in lines)
    assert not cli.parse(["--dry-run"]).one_controller


@check("hand check: the squeeze must hold 100 ms; a one-sample blip or a quiet port fails")
def _():
    blip = _both(0, 200, AWAKE, AWAKE) + _both(200, 220, HELD, AWAKE) \
        + _both(220, 400, AWAKE, AWAKE)
    c, _ = _check("left", blip)
    assert c.done and "did not stay engaged for 100 ms" in c.failure, c.failure
    quiet = _both(0, 200, AWAKE, AWAKE) + _both(200, 220, HELD, AWAKE) \
        + [(t, "right", AWAKE, -0.2) for t in range(220, 400, 20)]
    c, _ = _check("left", quiet)
    assert c.done and "went quiet during the squeeze" in c.failure, c.failure


@check("hand check fails closed: asleep, not fresh, no squeeze, wrong grip, both grips")
def _():
    cases = (
        ("left", _both(0, 5100, 0, AWAKE), "last flags 0x00"),                  # asleep
        ("left", _both(0, 5100, sources.FLAG_CONTROLLER_ON, AWAKE), "not awake"),
        ("left", _both(0, 5100, sources.FLAG_FRESH, AWAKE), "not awake"),       # not on
        ("left", [(t, "right", AWAKE, 0.0) for t in range(0, 5100, 20)], "no message at all"),
        ("left", _both(0, 10200, AWAKE, AWAKE), "no squeeze of the LEFT grip"),
        ("left", _both(0, 200, AWAKE, AWAKE) + _both(200, 400, AWAKE, HELD),    # the other
         "RIGHT (tcp://127.0.0.1:5570) shows engaged"),
        ("left", _both(0, 200, AWAKE, AWAKE) + _both(200, 400, HELD, HELD), "RIGHT"),
        ("right", _both(0, 200, AWAKE, AWAKE) + _both(200, 400, HELD, HELD), "LEFT"),
    )
    for hand, script, why in cases:
        c, lines = _check(hand, script)
        assert c.done and c.failure and why in c.failure, (hand, why, c.failure, lines)
    c, _ = _check("left", _both(0, 5100, 0, AWAKE))
    assert "the RIGHT controller is awake" in c.failure, c.failure


@check("hand check: listen() on two real ZMQ sockets (inproc://) passes LEFT, fails RIGHT")
def _():
    import threading
    import time
    import zmq
    for squeezed, passes in (("left", True), ("right", False)):
        ctx = zmq.Context()
        endpoints = {"left": "inproc://hand-left", "right": "inproc://hand-right"}
        pubs = {hand: ctx.socket(zmq.PUB) for hand in endpoints}
        for hand, sock in pubs.items():
            sock.bind(endpoints[hand])
        stop = threading.Event()

        def publish():
            t0 = time.monotonic()
            while not stop.is_set():
                held = time.monotonic() - t0 > 0.3
                for hand, sock in pubs.items():
                    sock.send(_vr(HELD if held and hand == squeezed else AWAKE))
                time.sleep(0.01)

        thread = threading.Thread(target=publish)
        thread.start()
        lines = []
        try:
            failure = hands.listen(hands.HandCheck("left", endpoints, 2.0, lines.append),
                                   ctx=ctx)
        finally:
            stop.set()
            thread.join()
            for sock in pubs.values():
                sock.close(0)
            ctx.term()
        assert (failure is None) == passes, (squeezed, failure, lines)


@check("hand check: a failure exits 3 before any source or sink; --skip-hand-check warns")
def _():
    saved = (hands.check_hands, cli.ZmqSource, cli.DryRunSink, cli.ZenohSink)
    handlers = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))

    def constructed(*_a):
        raise AssertionError("something was built after a failed hand check")

    try:
        hands.check_hands = lambda a: "synthetic failure"
        cli.ZmqSource = cli.DryRunSink = cli.ZenohSink = constructed
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = cli.main(["--arm", "L"])                   # live: acquire would follow
        text = out.getvalue()
        assert code == 3, (code, text)
        assert text.splitlines()[0] == "arm L <- LEFT controller (tcp://127.0.0.1:5560)", text
        assert "FAILED, nothing acquired: synthetic failure" in text, text
        hands.check_hands, cli.DryRunSink = constructed, saved[2]
        cli.ZmqSource = lambda endpoint: _Src()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = cli.main(["--arm", "L", "--dry-run", "--skip-hand-check", "--quiet",
                             "--print-every", "0"])
        text = out.getvalue()
        assert code == 0 and "[hand-check] WARNING: skipped" in text, text
        assert "hand check        SKIPPED" in text, text
        assert "node limits       FILE " in text, text
    finally:
        hands.check_hands, cli.ZmqSource, cli.DryRunSink, cli.ZenohSink = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGTERM, handlers[1])


@check("--probe-limits prints this node's derived block and acquires nothing")
def _():
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = cli.main(["--arm", "L", "--dry-run", "--probe-limits"])
    text = out.getvalue()
    assert code == 0, text
    assert "[probe-limits] arm L" in text and "nothing was acquired" in text, text
    assert '"max_lead": 0.05' in text and '"dq_limit"' in text, text
    assert "WARNING" in text and "NOT from the node" in text, text
    assert "vr-teleop summary" not in text and "[target]" not in text, text
    assert text.rstrip().endswith("the arm was not touched"), text
