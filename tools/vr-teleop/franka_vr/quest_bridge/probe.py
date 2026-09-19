"""The Quest-teleop probe -- isolates the headset -> bridge -> ZMQ path.

THE FIRST THING TO RUN AT THE ROBOT, and the only one that answers the question
that actually matters when "VR teleop didn't work": does the headset produce a
FRESH pose stream AND an ENGAGED flag AND real motion when you hold the grip
and move your hand? Those four things fail in four completely different ways
that are indistinguishable from the arm's point of view (the arm just holds),
so the summary below reports each link separately and names the culprit.

NO robot, NO teleop client: just the bridge's sockets. One count line and one
diagnosis per endpoint, so a two-arm bridge is checked in a single run.

Run (with `franka-vr-bridge` already up):

    franka-vr-probe                                     # both channels, 30 s
    franka-vr-probe --secs 8
    franka-vr-probe --endpoint tcp://127.0.0.1:5560 --secs 60

What to do while it runs:
    1. Wear the headset (or let the bridge's keep-awake broadcasts run) so
       tracking is live.
    2. HOLD the grip button          -> expect `engaged` to flip True.
    3. MOVE your hand                -> expect |translation| well above ~1e-3.
    4. Squeeze the trigger           -> expect the gripper value to rise.
    5. Click the thumbstick (RJ)     -> expect the forward direction to relatch.
"""
import argparse
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .subscriber import recv_vr_targets_multi
from ..wire import VR_BUTTON_RJ

# The bridge's two channels, in the order `vr.sh bridge` binds them.
DEFAULT_ENDPOINTS = ("tcp://127.0.0.1:5560", "tcp://127.0.0.1:5570")


@dataclass
class ProbeSummary:
    ticks: int = 0
    ever_message: bool = False
    ever_fresh: bool = False
    ever_controller_on: bool = False
    ever_engaged: bool = False
    ever_motion: bool = False
    ever_rj: bool = False
    max_translation: float = 0.0
    max_gripper: float = 0.0
    max_abs_pos_component: float = 0.0
    engage_edges: int = 0
    # The per-message counts the two-port count line reports, plus the buffers
    # that were not a VrTargetMsg at all (a foreign publisher on the port).
    bad: int = 0
    fresh_ticks: int = 0
    controller_on_ticks: int = 0
    engaged_ticks: int = 0
    last_pos: Optional[tuple] = None
    times: list = field(default_factory=list)

    @property
    def rate_hz(self) -> float:
        """Messages per second over the window actually observed. 0.0 below
        two messages: one arrival dates nothing."""
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        return (len(self.times) - 1) / span if span > 0 else 0.0


def observe(samples, motion_eps: float = 1e-3) -> ProbeSummary:
    """Fold an iterable of decoded VrTargetMsg dicts into a summary. PURE --
    this is what the tests drive, so the decision tree is provable without a
    headset.

    An item may be `None` (a buffer that did not decode) or `(msg, t)` with an
    arrival time; a bare `msg` is accepted too and simply dates nothing. A bad
    buffer still counts as a message: something IS publishing on that port, and
    `bad` on the count line is what says it is not us.

    Motion is measured as the SPAN of the published position over the whole
    window, not per tick: a slow, deliberate hand motion has tiny per-tick
    deltas and would otherwise read as "frozen".
    """
    s = ProbeSummary()
    prev_engaged = False
    lo = None
    hi = None
    for item in samples:
        msg, t = item if isinstance(item, tuple) else (item, None)
        s.ticks += 1
        s.ever_message = True
        if t is not None:
            s.times.append(float(t))
        if msg is None:
            s.bad += 1
            continue
        fresh = bool(msg["fresh"])
        controller_on = bool(msg["controller_on"])
        s.ever_fresh = s.ever_fresh or fresh
        s.ever_controller_on = s.ever_controller_on or controller_on
        s.fresh_ticks += fresh
        s.controller_on_ticks += controller_on
        engaged = bool(msg["engaged"])
        s.ever_engaged = s.ever_engaged or engaged
        s.engaged_ticks += engaged
        if engaged and not prev_engaged:
            s.engage_edges += 1
        prev_engaged = engaged
        if int(msg["buttons"]) & VR_BUTTON_RJ:
            s.ever_rj = True
        s.max_gripper = max(s.max_gripper, float(msg["gripper"]))

        pos = np.asarray(msg["pos"], dtype=float)
        s.last_pos = tuple(round(float(v), 3) for v in pos)
        lo = pos.copy() if lo is None else np.minimum(lo, pos)
        hi = pos.copy() if hi is None else np.maximum(hi, pos)
        s.max_abs_pos_component = max(s.max_abs_pos_component, float(np.max(np.abs(pos))))

    if lo is not None:
        s.max_translation = float(np.linalg.norm(hi - lo))
        s.ever_motion = s.max_translation > motion_eps
    return s


# Review finding I2: `pos` is in the Quest's SLAM tracking frame (arbitrary
# origin, NOT robot-base). The consumer rejects any |pos| component beyond its
# 10 m sanity bound as a tracking glitch, but a genuinely far tracking origin
# (operator wandered off, or a SLAM re-localization) can walk into that same
# bound for real -- and does so PERMANENTLY, since every subsequent sample is
# measured from the same arbitrary origin. That produces a confusing split:
# this probe (reading the raw wire, no bound applied) says HEALTHY while the
# consumer reports no signal from the headset, because it is silently rejecting
# every sample. Warn well before that bound so the operator can recenter or
# reboot the headset instead of chasing a phantom "no signal" bug.
_TRACKING_ORIGIN_WARN_METERS = 8.0


def tracking_origin_warning(s: ProbeSummary) -> Optional[str]:
    """None if nothing to warn about; otherwise a loud one-line warning that
    the Quest's tracking origin looks far from the workspace."""
    if s.max_abs_pos_component <= _TRACKING_ORIGIN_WARN_METERS:
        return None
    return (
        f"WARNING: observed |pos| reached {s.max_abs_pos_component:.2f} m -- "
        "the Quest's tracking origin looks far from the workspace (or it "
        "re-localized). The consumer's sanity bound (10 m) may start "
        "REJECTING samples even though this probe reads healthy (it reads the "
        "raw wire, unfiltered). Recenter or reboot the headset near the robot "
        "workspace."
    )


def diagnose(s: ProbeSummary) -> str:
    """The decision tree, in the order the links actually break."""
    if not s.ever_message:
        return (
            "DIAGNOSIS: nothing on the socket -- the BRIDGE is not publishing. "
            "Start it (`./vr.sh bridge`) and check `./vr.sh status`; a bridge "
            "that cannot reach adb exits immediately."
        )
    if not s.ever_controller_on:
        return (
            "DIAGNOSIS: the bridge publishes but this channel's controller "
            "never reported on -- this is the SAME symptom for three different "
            "causes: either Hand Tracking is still enabled in the headset (the "
            "APK silently drops Touch Plus controllers under multimodal "
            "hands+controllers), or the headset is showing a 2D panel because "
            "the proximity sensor / prox-close broadcast is unresolved (so the "
            "app never entered the immersive session that has controller "
            "poses), or the controller itself is unpaired/powered off."
        )
    if not s.ever_fresh:
        return (
            "DIAGNOSIS: frames arrive but the pose NEVER CHANGES -- the reader "
            "is serving its stale cache, i.e. the headset stopped streaming. "
            "Usually the proximity sensor: put the headset on, or let the "
            "keep-awake broadcasts run. Also check the app is immersive, not a "
            "2D panel."
        )
    if not s.ever_engaged:
        return (
            "DIAGNOSIS: the stream is FRESH but the grip never registered. Hold "
            "the SIDE grip trigger (under your middle finger) on THIS channel's "
            "controller. If you were holding it, the grip button is not reaching "
            "the reader -- re-pair the controller."
        )
    if not s.ever_motion:
        return (
            "DIAGNOSIS: engaged, but the published position never moved -- no "
            "motion detected. The controller is not being TRACKED (out of the "
            "headset cameras' view, or the headset is face-down). The robot "
            "would freeze mid-takeover: this is the classic 'the intervention "
            "didn't work'."
        )
    return (
        "DIAGNOSIS: the Quest path is HEALTHY (fresh stream + grip + motion). If "
        "the arm still does not follow, the break is DOWNSTREAM -- check the "
        "teleop client is running against this endpoint and is not reporting "
        "'no signal from the headset'."
    )


def count_line(endpoint: str, s: ProbeSummary) -> str:
    """The two-port count line: what arrived, how fast, and how much of it
    carried each flag. `pos` is the last one seen, rounded to millimetres."""
    return (f"{endpoint}: msgs={s.ticks} rate={s.rate_hz:.1f}Hz "
            f"fresh={s.fresh_ticks} controller_on={s.controller_on_ticks} "
            f"engaged={s.engaged_ticks} bad={s.bad} pos={s.last_pos}")


def report(endpoint: str, s: ProbeSummary) -> str:
    """The full per-endpoint block: the count line, the link-by-link summary,
    the diagnosis and, if it applies, the tracking-origin warning."""
    out = [
        f"----- {endpoint} -----",
        " " + count_line(endpoint, s),
        f" controller_on ever True : {s.ever_controller_on}",
        f" stream ever FRESH       : {s.ever_fresh}",
        f" engaged (grip) ever True: {s.ever_engaged}   (edges: {s.engage_edges})",
        f" thumbstick (RJ) pressed : {s.ever_rj}",
        f" real hand motion seen   : {s.ever_motion}   (span = {s.max_translation:.4f} m)",
        f" max gripper value       : {s.max_gripper:.3f}",
        " " + diagnose(s),
    ]
    warning = tracking_origin_warning(s)
    if warning is not None:
        out.append(f" *** {warning} ***")
    return "\n".join(out)


def _dedup(items):
    """Order-preserving de-duplication: two --endpoint of the same port would
    otherwise be reported twice from one socket's messages."""
    seen = set()
    return [x for x in items if not (x in seen or seen.add(x))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--endpoint", action="append", metavar="EP",
                    help="ZMQ endpoint to listen on; repeat for more channels. "
                         f"Default: {' '.join(DEFAULT_ENDPOINTS)}")
    ap.add_argument("--secs", type=float, default=30.0)
    ap.add_argument("--motion-eps", type=float, default=1e-3,
                    help="position span above this counts as real hand motion")
    args = ap.parse_args(argv)
    endpoints = _dedup(args.endpoint or list(DEFAULT_ENDPOINTS))

    print(f"[probe] listening on {', '.join(endpoints)} for {args.secs:.0f}s ...")
    print("  t(s)   ep    fresh eng  |pos|                    grip  buttons")

    t0 = time.monotonic()
    collected = {ep: [] for ep in endpoints}
    last_print = {ep: 0.0 for ep in endpoints}
    # conflate=False: this is the probe, so every message must be counted --
    # keeping only the latest would silently understate the rate.
    for endpoint, msg in recv_vr_targets_multi(endpoints, args.secs, conflate=False):
        now = time.monotonic() - t0
        collected[endpoint].append((msg, now))
        if msg is not None and now - last_print[endpoint] >= 0.33:
            last_print[endpoint] = now
            pos = np.asarray(msg["pos"], dtype=float)
            print(f"  {now:5.1f}  {endpoint[-4:]}  {int(msg['fresh'])}     "
                  f"{int(msg['engaged'])}    "
                  f"({pos[0]:6.3f},{pos[1]:6.3f},{pos[2]:6.3f})  "
                  f"{msg['gripper']:5.2f}  0x{msg['buttons']:02x}")

    print("\n========== PROBE SUMMARY ==========")
    for endpoint in endpoints:
        print(report(endpoint, observe(collected[endpoint], motion_eps=args.motion_eps)))
    print("===================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
