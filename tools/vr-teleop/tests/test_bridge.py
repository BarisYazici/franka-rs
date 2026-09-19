"""`QuestBridge` and `_Channel` driven by a fake reader -- no headset, no adb.

`QuestBridge(reader=...)` is the injection seam the design has always had;
these tests are what finally use it. Every threshold is asserted from both
sides: 99 ms of gap republishes, 101 ms latches.

Nothing here needs `ppadb`: `build_reader()` imports it lazily, so the module
imports and `--help` runs on a laptop with no Android tooling at all.
"""
import threading
import time

import numpy as np
import pytest

from franka_vr.quest_bridge.bridge import QuestBridge, build_arg_parser, main
from franka_vr.quest_bridge.channel import (
    GAP_HOLD_S,
    IDENTITY_QUAT,
    _build_channels,
    _Channel,
)
from franka_vr.quest_bridge.mapping import VrMapper
from franka_vr.wire import VR_BUTTON_GRIP, unpack_vr_target


class FakeReader:
    """The whole seam: one method returning `(poses, buttons)`."""

    def __init__(self, frame=None):
        self.frame = frame if frame is not None else ({}, {})
        self.raises = None
        self.calls = 0
        self.stopped = False

    def get_transformations_and_buttons(self):
        self.calls += 1
        if self.raises is not None:
            exc, self.raises = self.raises, None
            raise exc
        return self.frame

    def stop(self):
        self.stopped = True


class FakePublisher:
    def __init__(self):
        self.sent = []

    def send(self, payload):
        self.sent.append(payload)


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = float(t)

    def __call__(self):
        return self.t


def pose(t=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    T[:3, 3] = t
    return T


def frame(*cids, grip=(), t=(0.0, 0.0, 0.0)):
    """A reader frame carrying every controller in `cids`."""
    poses, buttons = {}, {}
    for cid in cids:
        U = cid.upper()
        poses[cid] = pose(t)
        buttons[U + "G"] = cid in grip
        buttons[U + "J"] = False
        buttons["rightTrig" if cid == "r" else "leftTrig"] = [0.0]
    return poses, buttons


def make(cids=("r",), **kw):
    reader = FakeReader()
    pubs = [FakePublisher() for _ in cids]
    clock = FakeClock()
    br = QuestBridge(reader, pubs,
                     mapper=[VrMapper(controller_id=c) for c in cids],
                     clock=clock, keepalive=None, **kw)
    return br, reader, pubs, clock


def last(pub):
    return unpack_vr_target(pub.sent[-1])


# --------------------------------------------------------------- the tick


def test_every_tick_publishes_once_per_channel_with_a_shared_seq():
    br, reader, pubs, clock = make(("l", "r"))
    for i in range(5):
        clock.t += 0.02
        reader.frame = frame("l", "r", t=(0.01 * i, 0.0, 0.0))
        out = br.tick()
        assert len(out) == 2
    assert [len(p.sent) for p in pubs] == [5, 5]
    assert [unpack_vr_target(p)["seq"] for p in pubs[0].sent] == [1, 2, 3, 4, 5]
    assert [unpack_vr_target(p)["seq"] for p in pubs[1].sent] == [1, 2, 3, 4, 5]


def test_an_empty_frame_still_publishes_not_fresh_rather_than_nothing():
    br, reader, pubs, clock = make()
    reader.frame = ({}, {})
    br.tick()
    m = last(pubs[0])
    assert (m["fresh"], m["engaged"], m["controller_on"]) == (False, False, False)
    assert m["pos"] == (0.0, 0.0, 0.0) and m["quat"] == IDENTITY_QUAT
    assert m["gripper"] == 0.0


def test_the_reader_raising_is_not_fatal():
    br, reader, pubs, clock = make()
    reader.raises = RuntimeError("adb went away")
    br.tick()
    assert last(pubs[0])["fresh"] is False
    reader.frame = frame("r")
    clock.t += 0.02
    br.tick()
    assert last(pubs[0])["fresh"] is True


# ------------------------------------------------------ the clutch handshake


def test_startup_is_latched_so_a_held_grip_publishes_disengaged():
    br, reader, pubs, clock = make()
    assert br.forced_open == [True]

    reader.frame = frame("r", grip=("r",), t=(0.1, 0.0, 0.0))
    br.tick()
    m = last(pubs[0])
    assert m["engaged"] is False          # latched, whatever the grip says
    assert m["fresh"] is True
    assert m["buttons"] & VR_BUTTON_GRIP  # ... and the grip is visibly held

    clock.t += 0.02                       # release: the handshake
    reader.frame = frame("r", t=(0.2, 0.0, 0.0))
    br.tick()
    assert br.forced_open == [False]
    assert last(pubs[0])["engaged"] is False

    clock.t += 0.02                       # grip again: now it drives
    reader.frame = frame("r", grip=("r",), t=(0.3, 0.0, 0.0))
    br.tick()
    assert last(pubs[0])["engaged"] is True


def test_a_not_fresh_sample_latches_that_channel_alone():
    br, reader, pubs, clock = make(("l", "r"))
    clock.t += 0.02
    reader.frame = frame("l", "r", t=(0.1, 0.0, 0.0))
    br.tick()
    assert br.forced_open == [False, False]

    # l keeps moving, r is served from the reader's stale cache. Freezing the
    # whole frame is the only way to freeze one controller through this fake,
    # so freeze r by giving it its own mapper's last matrix: use two frames.
    for i, dt in enumerate((0.1, 0.16)):
        clock.t += dt
        poses, buttons = frame("l", "r", t=(0.2 + 0.1 * i, 0.0, 0.0))
        poses["r"] = pose((0.1, 0.0, 0.0))        # unchanged since the tick above
        reader.frame = (poses, buttons)
        br.tick()
    assert br.forced_open == [False, True]        # r only
    assert last(pubs[0])["fresh"] is True
    assert last(pubs[1])["fresh"] is False


def test_the_latch_clears_only_on_fresh_AND_released():
    br, reader, pubs, clock = make()
    reader.frame = frame("r", grip=("r",), t=(0.1, 0.0, 0.0))
    br.tick()
    assert br.forced_open == [True]

    for i in range(4):                            # still gripping: stays latched
        clock.t += 0.02
        reader.frame = frame("r", grip=("r",), t=(0.2 + 0.1 * i, 0.0, 0.0))
        br.tick()
        assert br.forced_open == [True]
        assert last(pubs[0])["engaged"] is False

    clock.t += 0.02
    reader.frame = frame("r", t=(0.9, 0.0, 0.0))  # released AND fresh
    br.tick()
    assert br.forced_open == [False]


# -------------------------------------------------------- the 100 ms gap hold


def body(payload):
    """Everything but the seq: what a verbatim republish must reproduce."""
    return payload[:8] + payload[16:]


def test_99ms_of_gap_republishes_verbatim_and_101ms_latches():
    assert GAP_HOLD_S == 0.1
    br, reader, pubs, clock = make()
    clock.t += 0.02
    reader.frame = frame("r", t=(0.1, 0.0, 0.0))
    br.tick()                                       # arm the clutch
    clock.t += 0.02
    reader.frame = frame("r", grip=("r",), t=(0.2, 0.0, 0.0))
    br.tick()
    good = pubs[0].sent[-1]
    assert unpack_vr_target(good)["engaged"] is True

    t_gap = clock.t + 0.001
    for dt in (0.0, 0.05, 0.099):                   # inside the hold
        clock.t = t_gap + dt
        reader.frame = (None, None)                 # one truncated logcat line
        br.tick()
        assert body(pubs[0].sent[-1]) == body(good), f"at {dt}"
        assert br.forced_open == [False]

    clock.t = t_gap + 0.101                         # past it
    br.tick()
    m = last(pubs[0])
    assert br.forced_open == [True]
    assert (m["fresh"], m["engaged"], m["gripper"]) == (False, False, 0.0)
    assert m["pos"] == unpack_vr_target(good)["pos"]      # the last good pose
    assert m["quat"] == unpack_vr_target(good)["quat"]


def test_resolve_holds_then_gives_up_and_forgets():
    ch = _Channel(mapper=None, publisher=None)
    assert ch.resolve(None, 0.0, 0.1) is None        # nothing to republish yet
    assert ch.resolve("S1", 1.0, 0.1) == "S1"
    assert ch.gap_since is None
    assert ch.resolve(None, 1.0, 0.1) == "S1"        # gap opens here
    assert ch.gap_since == 1.0
    assert ch.resolve(None, 1.099, 0.1) == "S1"
    assert ch.resolve(None, 1.1, 0.1) is None        # >= hold_s: gives up
    assert ch.last_sample is None                    # and DROPS the held sample
    assert ch.resolve(None, 1.101, 0.1) is None      # a later blip cannot revive it
    assert ch.resolve("S2", 2.0, 0.1) == "S2"        # a real sample re-arms
    assert ch.gap_since is None


def test_a_usable_sample_in_the_middle_of_a_gap_rearms_the_hold():
    ch = _Channel(mapper=None, publisher=None)
    ch.resolve("S1", 0.0, 0.1)
    ch.resolve(None, 0.05, 0.1)
    assert ch.resolve("S2", 0.09, 0.1) == "S2"
    assert ch.gap_since is None
    assert ch.resolve(None, 0.18, 0.1) == "S2"       # the clock restarts at 0.18
    assert ch.resolve(None, 0.27, 0.1) == "S2"


def test_gap_hold_s_is_honoured_as_given():
    """The bridge passes it through; the constant has no CLI knob but the
    constructor argument is what the tests and any future tuning use."""
    br, reader, pubs, clock = make(gap_hold_s=0.0)
    clock.t += 0.02
    reader.frame = frame("r", t=(0.1, 0.0, 0.0))
    br.tick()
    clock.t += 0.02
    reader.frame = ({}, {})
    br.tick()
    assert br.forced_open == [True]                  # no hold at all


# ------------------------------------------------------------- the keepalive


def test_tick_never_calls_the_keepalive():
    br, reader, pubs, clock = make()
    calls = []
    br.keepalive = lambda: calls.append(1) or True
    reader.frame = frame("r")
    for _ in range(10):
        clock.t += 0.02
        br.tick()
    assert calls == []


def test_keepalive_fires_immediately_then_on_its_own_period():
    br, _reader, _pubs, _clock = make()
    fired = threading.Semaphore(0)
    br.keepalive = lambda: (fired.release(), True)[1]
    br.keepalive_period_s = 0.01
    br.start_keepalive()
    try:
        assert fired.acquire(timeout=2.0)            # immediately
        assert fired.acquire(timeout=2.0)            # and again
        assert br._keepalive_thread.daemon is True
        assert br._keepalive_thread is not threading.current_thread()
    finally:
        br.stop_keepalive()
    assert br._keepalive_thread is None


def test_a_slow_keepalive_does_not_stall_tick():
    br, reader, pubs, clock = make()
    release = threading.Event()
    entered = threading.Event()

    def slow():
        entered.set()
        release.wait(5.0)
        return True

    br.keepalive = slow
    br.start_keepalive()
    try:
        assert entered.wait(2.0)
        reader.frame = frame("r")
        t0 = time.monotonic()
        for _ in range(20):
            clock.t += 0.02
            br.tick()
        assert time.monotonic() - t0 < 1.0           # the broadcast is still blocked
        assert len(pubs[0].sent) == 20
    finally:
        release.set()
        br.stop_keepalive()


def test_a_raising_keepalive_is_swallowed_and_warned_once(caplog):
    br, _reader, _pubs, _clock = make()
    br.keepalive = lambda: (_ for _ in ()).throw(OSError("no adb"))
    br._fire_keepalive_once()
    br._fire_keepalive_once()
    assert sum("keep-awake" in r.message for r in caplog.records) == 1


def test_no_keepalive_means_no_thread():
    br, _reader, _pubs, _clock = make()
    br.start_keepalive()
    assert br._keepalive_thread is None
    br.stop_keepalive()


# ------------------------------------------------------------ the channels


def test_build_channels_normalises_and_refuses_mismatches():
    p, q = FakePublisher(), FakePublisher()
    assert len(_build_channels(p)) == 1
    assert len(_build_channels([p, q])) == 2
    with pytest.raises(ValueError, match="at least one publisher"):
        _build_channels([])
    with pytest.raises(ValueError, match="one mapper per publisher"):
        _build_channels([p, q], mapper=VrMapper())


def test_the_single_arm_names_still_point_at_channel_zero():
    br, _reader, pubs, _clock = make(("l", "r"))
    assert br.publisher is pubs[0]
    assert br.mapper.controller_id == "l"
    assert [m.controller_id for m, _ in br.channels] == ["l", "r"]


def test_run_stops_on_the_event_and_on_max_ticks():
    br, reader, pubs, _clock = make(rate_hz=1000.0)
    reader.frame = frame("r")
    assert br.run(max_ticks=3) == 0
    assert len(pubs[0].sent) == 3
    stop = threading.Event()
    stop.set()
    assert br.run(stop_event=stop) == 0
    assert len(pubs[0].sent) == 3


# ------------------------------------------------------------------- the CLI


def test_the_parser_defaults():
    a = build_arg_parser().parse_args([])
    assert (a.endpoint, a.controller, a.rate) == ("tcp://127.0.0.1:5560", "r", 50.0)
    assert (a.spatial_scale, a.oculus_ip, a.no_keepalive) == (1.0, None, False)


def test_help_runs_without_adb():
    with pytest.raises(SystemExit) as e:
        build_arg_parser().parse_args(["--help"])
    assert e.value.code == 0


@pytest.mark.parametrize("argv,needle", [
    (["--controller", "l,r"], "same number of channels"),
    (["--controller", ""], "at least one channel"),
    (["--controller", "x"], "must be 'l' or 'r'"),
    (["--endpoint", "tcp://127.0.0.1:5560,tcp://127.0.0.1:5560",
      "--controller", "l,r"], "duplicate entries"),
])
def test_argv_is_validated_before_anything_touches_adb(argv, needle, capsys):
    assert main(argv) == 2
    err = capsys.readouterr().err
    assert needle in err
    assert "Quest reader" not in err      # it never got as far as the reader
