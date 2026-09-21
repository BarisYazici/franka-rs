"""The probe's decision tree, folded over messages a real headset produced.

`RECORDED_*` below are raw VrTargetMsgs lifted out of two recorded sessions:
a headset awake on a desk with no one holding a controller, and a two-handed
session across an engage edge. Driving `observe()`/`diagnose()` with those
bytes means the tree is checked against what the hardware actually emits, not
against what this file imagines it emits.

The socket tests bind an ephemeral port and publish into it, so the SUB side
and the `--endpoint` contract are exercised with no headset and no adb.
"""
import threading
import time

import pytest

from franka_vr.quest_bridge.channel import ZmqVrPublisher
from franka_vr.quest_bridge.probe import (
    DEFAULT_ENDPOINTS,
    ProbeSummary,
    _dedup,
    count_line,
    diagnose,
    main,
    observe,
    report,
    tracking_origin_warning,
)
from franka_vr.quest_bridge.subscriber import recv_vr_targets_multi, wait_for_vr_target
from franka_vr.wire import VR_BUTTON_RJ, pack_vr_target, unpack_vr_target

# Controller r, desk-idle session: the reader serving its stale cache. Three
# consecutive published messages, byte for byte.
RECORDED_IDLE = [bytes.fromhex(h) for h in (
    "48524c5601000000e309000000000000145ad6fd6321bebfe56c67833f08503f9b012ec896e59bbf"
    "010000000000e0bf000000000000e03f000000000000e03f000000000000e0bf0000000000000000"
    "0000000000000000",
    "48524c5601000000e409000000000000145ad6fd6321bebfe56c67833f08503f9b012ec896e59bbf"
    "010000000000e0bf000000000000e03f000000000000e03f000000000000e0bf0000000000000000"
    "0000000000000000",
    "48524c5601000000e509000000000000145ad6fd6321bebfe56c67833f08503f9b012ec896e59bbf"
    "010000000000e0bf000000000000e03f000000000000e03f000000000000e0bf0000000000000000"
    "0000000000000000",
)]

# Controller r, held-grips session, across the engage edge: two fresh but
# disengaged ticks (seq 964-965) then four engaged ones (seq 966-969).
RECORDED_ENGAGE = [bytes.fromhex(h) for h in (
    "48524c5601000000c4030000000000002176a6d0798dd53fa25ef0694e5ed0bf8786c5a86beda5bf"
    "000000000000e03f000000000000e0bffeffffffffffdfbf000000000000e03f0000000000000000"
    "0600000000000000",
    "48524c5601000000c5030000000000009cc58b852172d53f3dd68c0c7217d0bfb06df6ac10fba5bf"
    "000000000000e03f000000000000e0bf000000000000e0bf010000000000e03f0000000000000000"
    "0600000000000000",
    "48524c5601000000c603000000000000d6abc8e88024d53f78431a1538d9cebf51572d9f9beca5bf"
    "ce3c7dc253bfde3f15851a38b84ae0bf6251fb6fc7c9dfbff8ad09da686be03f0000000000000000"
    "0700000008000000",
    "48524c5601000000c703000000000000946934b91803d53f3eb2b96a9e23cebfdaba8a7be2efa5bf"
    "7662cf4e98cfddbf10148f74758fe03fc31468846115e03f0e700d26f165e0bf0000000000000000"
    "0700000008000000",
    "48524c5601000000c803000000000000f3e670adf6b0d43fafec82c13577ccbf665fc31db97fa6bf"
    "934ee624ee45dbbfb7214450002fe13ff9aa3c5f5a31e03fb34cbb95fdbfe0bf0000000000000000"
    "0700000008000000",
    "48524c5601000000c903000000000000e370e6577380d43f76887fd8d2a3cbbf7f15e0bbcd1ba7bf"
    "3bbcc9412e06dabf9230c4358e8ae13f3928ed731443e03f12e206ee10cfe0bf0000000000000000"
    "0700000008000000",
)]


def msgs(raws):
    return [unpack_vr_target(r) for r in raws]


def synth(n=1, **kw):
    """Messages with exactly the flags a test names."""
    f = dict(engaged=False, fresh=False, controller_on=False, buttons=0,
             pos=(0.0, 0.0, 0.0), gripper=0.0)
    f.update(kw)
    return [unpack_vr_target(pack_vr_target(
        i, f["pos"], (0.0, 0.0, 0.0, 1.0), f["gripper"], engaged=f["engaged"],
        fresh=f["fresh"], controller_on=f["controller_on"], buttons=f["buttons"]))
        for i in range(n)]


# ------------------------------------------------- recorded reality: the fold


def test_recorded_idle_session_is_the_stale_cache_verdict():
    s = observe(msgs(RECORDED_IDLE))
    assert (s.ticks, s.bad) == (3, 0)
    assert s.ever_message is True
    assert (s.fresh_ticks, s.controller_on_ticks, s.engaged_ticks) == (0, 0, 0)
    assert s.ever_fresh is False and s.ever_controller_on is False
    assert s.max_translation == 0.0 and s.ever_motion is False
    assert s.last_pos == (-0.118, 0.001, -0.027)
    assert "never reported on" in diagnose(s)


def test_recorded_engage_edge_reads_healthy():
    s = observe(msgs(RECORDED_ENGAGE))
    assert (s.ticks, s.bad) == (6, 0)
    assert (s.fresh_ticks, s.controller_on_ticks, s.engaged_ticks) == (6, 6, 4)
    assert s.engage_edges == 1                       # one edge, not four
    assert s.ever_rj is False and s.max_gripper == 0.0
    assert round(s.max_translation, 4) == 0.0431     # the hand really moved
    assert s.ever_motion is True
    assert "HEALTHY" in diagnose(s)


def test_the_motion_threshold_discriminates_on_the_same_recording():
    """The other direction: the same bytes, a span bound above them."""
    s = observe(msgs(RECORDED_ENGAGE), motion_eps=1.0)
    assert s.ever_motion is False
    assert "never moved" in diagnose(s)


def test_motion_is_the_span_not_the_per_tick_delta():
    slow = [unpack_vr_target(pack_vr_target(
        i, (0.0002 * i, 0.0, 0.0), (0, 0, 0, 1), 0.0,
        engaged=True, fresh=True, controller_on=True)) for i in range(30)]
    s = observe(slow)                                 # 0.2 mm per tick, 5.8 mm span
    assert s.ever_motion is True
    assert round(s.max_translation, 4) == 0.0058


# ----------------------------------------------------- the tree, both ways


def test_the_tree_in_the_order_the_links_break():
    assert "not publishing" in diagnose(observe([]))
    assert "never reported on" in diagnose(observe(synth(2, controller_on=False)))
    assert "NEVER CHANGES" in diagnose(observe(synth(2, controller_on=True)))
    assert "grip never registered" in diagnose(
        observe(synth(2, controller_on=True, fresh=True)))
    assert "never moved" in diagnose(
        observe(synth(2, controller_on=True, fresh=True, engaged=True)))


def test_engage_edges_count_transitions_not_ticks():
    seq = (synth(3, controller_on=True, fresh=True)
           + synth(3, controller_on=True, fresh=True, engaged=True)
           + synth(2, controller_on=True, fresh=True)
           + synth(2, controller_on=True, fresh=True, engaged=True))
    s = observe(seq)
    assert s.engage_edges == 2 and s.engaged_ticks == 5


def test_rj_and_gripper_are_picked_up():
    s = observe(synth(1, controller_on=True, fresh=True,
                      buttons=VR_BUTTON_RJ, gripper=0.42))
    assert s.ever_rj is True and s.max_gripper == 0.42
    assert observe(synth(1, buttons=0)).ever_rj is False


def test_an_undecodable_buffer_counts_as_a_message_but_not_as_data():
    s = observe([None, None] + msgs(RECORDED_ENGAGE)[:1])
    assert (s.ticks, s.bad, s.fresh_ticks) == (3, 2, 1)
    assert s.ever_message is True


def test_rate_needs_two_arrivals():
    assert observe([(m, None) for m in msgs(RECORDED_IDLE)]).rate_hz == 0.0
    timed = list(zip(msgs(RECORDED_ENGAGE), [0.0, 0.02, 0.04, 0.06, 0.08, 0.10]))
    assert round(observe(timed).rate_hz, 3) == 50.0


def test_tracking_origin_warning_both_sides_of_8m():
    ok = ProbeSummary(max_abs_pos_component=8.0)
    assert tracking_origin_warning(ok) is None
    far = ProbeSummary(max_abs_pos_component=8.01)
    assert "8.01 m" in tracking_origin_warning(far)


def test_count_line_and_report_shape():
    s = observe(list(zip(msgs(RECORDED_ENGAGE), [0.0, 0.02, 0.04, 0.06, 0.08, 0.10])))
    line = count_line("tcp://127.0.0.1:5570", s)
    assert line == ("tcp://127.0.0.1:5570: msgs=6 rate=50.0Hz fresh=6 "
                    "controller_on=6 engaged=4 bad=0 pos=(0.32, -0.216, -0.045)")
    block = report("tcp://127.0.0.1:5570", s)
    assert line in block and "HEALTHY" in block
    assert block.startswith("----- tcp://127.0.0.1:5570 -----")


def test_dedup_keeps_the_first_of_each():
    assert _dedup(["b", "a", "b", "c", "a"]) == ["b", "a", "c"]


# ----------------------------------------------------------- over a socket


class Feed:
    """A real PUB on an ephemeral port, publishing until stopped."""

    def __init__(self, payloads, period=0.002):
        self.pub = ZmqVrPublisher("tcp://127.0.0.1:0")
        self.endpoint = self.pub.endpoint
        self._payloads = list(payloads)
        self._period = period
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        i = 0
        while not self._stop.is_set():
            self.pub.send(self._payloads[i % len(self._payloads)])
            i += 1
            time.sleep(self._period)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(timeout=2.0)
        self.pub.close()


def test_two_endpoints_are_read_and_kept_apart():
    with Feed(RECORDED_IDLE) as idle, Feed(RECORDED_ENGAGE) as live:
        got = {idle.endpoint: [], live.endpoint: []}
        for ep, msg in recv_vr_targets_multi(
                [idle.endpoint, live.endpoint], 1.0, conflate=False):
            got[ep].append(msg)
            if len(got[idle.endpoint]) > 3 and len(got[live.endpoint]) > 3:
                break
        assert observe(got[idle.endpoint]).ever_fresh is False
        assert observe(got[live.endpoint]).ever_fresh is True
        assert observe(got[live.endpoint]).bad == 0


def test_a_foreign_publisher_is_counted_as_bad_not_crashed_on():
    with Feed([b"not a VrTargetMsg"]) as junk:
        got = []
        for _ep, msg in recv_vr_targets_multi([junk.endpoint], 1.0, conflate=False):
            got.append(msg)
            if len(got) >= 3:
                break
        s = observe(got)
        assert s.bad == s.ticks == len(got) >= 1
        assert "never reported on" in diagnose(s)


def test_wait_for_vr_target_requires_freshness_when_asked():
    with Feed(RECORDED_IDLE) as idle:
        assert wait_for_vr_target(idle.endpoint, timeout_s=2.0) is not None
        assert wait_for_vr_target(idle.endpoint, timeout_s=0.5,
                                  require_fresh=True) is None
    with Feed(RECORDED_ENGAGE) as live:
        assert wait_for_vr_target(live.endpoint, timeout_s=2.0,
                                  require_fresh=True) is not None


# ------------------------------------------------------------------- the CLI


def test_default_endpoints_are_the_bridge_s_two_channels():
    assert DEFAULT_ENDPOINTS == ("tcp://127.0.0.1:5560", "tcp://127.0.0.1:5570")


def test_help_runs():
    with pytest.raises(SystemExit) as e:
        main(["--help"])
    assert e.value.code == 0


def test_repeatable_endpoint_reports_each_one(capsys):
    with Feed(RECORDED_IDLE) as idle, Feed(RECORDED_ENGAGE) as live:
        rc = main(["--endpoint", idle.endpoint, "--endpoint", live.endpoint,
                   "--secs", "0.6"])
    out = capsys.readouterr().out
    assert rc == 0
    assert out.count("----- tcp://") == 2
    assert f"----- {idle.endpoint} -----" in out
    assert f"----- {live.endpoint} -----" in out
    assert "never reported on" in out and "HEALTHY" in out
    assert "msgs=" in out


def test_an_endpoint_with_nothing_on_it_says_so(capsys):
    rc = main(["--endpoint", "tcp://127.0.0.1:5399", "--secs", "0.2"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "not publishing" in out and "msgs=0" in out
