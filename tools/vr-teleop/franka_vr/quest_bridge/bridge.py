"""The quest bridge daemon: OculusReader -> VrMapper -> ZMQ PUB (VrTargetMsg).

    franka-vr-bridge [--endpoint tcp://127.0.0.1:5560] [--controller r]
                     [--spatial-scale 1.0] [--rate 50] [--oculus-ip IP] [--no-keepalive]

ONE HEADSET, N CHANNELS. `--controller` and `--endpoint` are comma lists of
equal length (`--controller l,r --endpoint tcp://...:5560,tcp://...:5570`), and
each pair becomes one (VrMapper, ZmqVrPublisher) channel. There is still ONE
reader and ONE loop: a Quest logcat frame already carries both controllers, so
a second reader would mean a second adb logcat stream fighting for the same
device. One frame in, one message per channel out, at `--rate` (50 Hz by
default; the headset's own stream is ~72 Hz, so this is a slight downsample and
the consumer always reads the LATEST message, CONFLATE=1 on both ends).

WHY THIS FILE LOOKS THE WAY IT DOES. Every rule below is an incident fix and
docs/design.md carries the measurements and the reasoning behind each. Read it
before changing any of them:

* fail loud at startup, hold quietly afterwards -- `main()` exits nonzero if
  the reader or a bind fails; no per-frame failure after that is fatal.
* fail closed on the clutch, PER CHANNEL (`forced_open`, all True at startup):
  a channel publishes engaged=False until IT has a fresh sample with ITS OWN
  grip released. A not-fresh sample latches that channel alone; a sustained
  gap latches it once `_Channel.resolve()` has given up (channel.py).
* the proximity keep-awake broadcasts run on their OWN daemon thread, never on
  the tick path: `am broadcast` takes 100 ms-1 s on-device and used to stall
  the publish loop past the consumer's clutch-disengage window.
"""
import argparse
import logging
import sys
import threading
import time
from typing import List, Optional, Tuple

from .channel import (
    DEFAULT_ENDPOINT,
    GAP_HOLD_S,
    KEEPALIVE_PERIOD_S,
    ZmqVrPublisher,
    _build_channels,
    adb_proximity_keepalive,
)
from .mapping import VrMapper

logger = logging.getLogger(__name__)

DEFAULT_RATE_HZ = 50.0


class QuestBridge:
    """The 50 Hz loop. `reader` needs only
    `get_transformations_and_buttons() -> (poses, buttons)`, which is the
    injection seam every test uses instead of a headset.

    `publisher` and `mapper` each take a single object (one arm, the historical
    call) or an equal-length sequence (one channel per arm).
    """

    def __init__(
        self,
        reader,
        publisher,
        mapper=None,
        rate_hz: float = DEFAULT_RATE_HZ,
        keepalive=None,
        keepalive_period_s: float = KEEPALIVE_PERIOD_S,
        clock=time.monotonic,
        gap_hold_s: float = GAP_HOLD_S,
    ):
        self.reader = reader
        self._channels = _build_channels(publisher, mapper)

        # FAIL CLOSED, ONE FLAG PER CHANNEL. `forced_open[i]` True means
        # "publish engaged=False on channel i, whatever its grip says" -- see
        # the module docstring. All True at startup: every arm needs its own
        # released-grip handshake before it can be driven.
        self.forced_open: List[bool] = [True] * len(self._channels)

        self.rate_hz = float(rate_hz)
        # See channel.py: how long an unusable frame is ridden out before the
        # channel is declared gapped.
        self.gap_hold_s = float(gap_hold_s)
        self.keepalive = keepalive
        self.keepalive_period_s = float(keepalive_period_s)
        self.clock = clock

        self._seq = 0
        self._keepalive_warned = False
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_stop = threading.Event()

    # -- the channels ------------------------------------------------------

    @property
    def channels(self) -> List[Tuple[VrMapper, object]]:
        """`[(mapper, publisher), ...]`, in argv order."""
        return [(ch.mapper, ch.publisher) for ch in self._channels]

    @property
    def mapper(self) -> VrMapper:
        """Channel 0's mapper, under its historical single-arm name."""
        return self._channels[0].mapper

    @property
    def publisher(self):
        """Channel 0's publisher, under its historical single-arm name."""
        return self._channels[0].publisher

    # -- one tick ----------------------------------------------------------

    def tick(self) -> List[bytes]:
        """Read one frame, map it once per channel, publish one message on
        each. Returns the published bytes, in channel order. Never raises for
        anything short of a broken publisher.

        Does NOT touch the keepalive -- that runs on its own daemon thread,
        started separately via `start_keepalive()`/`run()`, so a slow adb
        broadcast can never stall this loop."""
        now = self.clock()

        # `frame_ok` False means the whole FRAME was unusable, not just one
        # controller's slice of it. It latches nothing by itself -- every
        # channel's own sample is None in that case anyway -- and only picks
        # the wording of the log line, which is what tells an operator whether
        # the headset went away or just one controller did.
        frame_ok = True
        try:
            poses, buttons = self.reader.get_transformations_and_buttons()
        except Exception:
            logger.exception("quest bridge: reader raised; publishing not-fresh")
            poses, buttons = {}, {}
            frame_ok = False
        if not poses:
            frame_ok = False

        # Map, then ride out a short gap PER CHANNEL (_Channel.resolve): both
        # arms must survive a dropped logcat line with no operator ritual.
        samples = [ch.resolve(ch.mapper.update(poses or {}, buttons or {}, now),
                              now, self.gap_hold_s)
                   for ch in self._channels]
        self._update_forced_open(samples, now, frame_ok=frame_ok)

        self._seq += 1
        return [ch.publish(self._seq, sample, latched)
                for ch, sample, latched in
                zip(self._channels, samples, self.forced_open)]

    def _channel_name(self, i: int) -> str:
        """`"0 (l)"` -- the index the operator sees in argv order, plus the
        controller that channel reads, so a log line names an ARM."""
        cid = getattr(self._channels[i].mapper, "controller_id", "?")
        return f"{i} ({cid})"

    def _update_forced_open(self, samples, now: float, *,
                            frame_ok: bool = True) -> None:
        """Update every channel's own latch from its own RESOLVED sample.

        A `None` here is already a SUSTAINED gap (resolve() rode out anything
        shorter than `gap_hold_s`), and a not-fresh sample is already the
        mapper's debounced 250 ms zero-change verdict, so either latches on
        the spot. Nothing is bridge-wide: a whole-frame failure reaches every
        channel through this same per-channel rule and can never latch an arm
        whose own stream is healthy. Channel i clears only on a tick where it
        is fresh AND its own grip is released -- the re-engage handshake, per
        arm.

        Logs ONLY the transitions (this runs at 50 Hz), which is what tells an
        operator why one arm has stopped following while the other has not.
        See docs/design.md.
        """
        for i, sample in enumerate(samples):
            gap = sample is None or not sample.fresh
            if gap:
                if not self.forced_open[i]:
                    logger.warning(
                        "quest bridge: channel %s -- %s; forcing its clutch open "
                        "until ITS grip is released",
                        self._channel_name(i),
                        self._gap_reason(i, now, sample, frame_ok=frame_ok),
                    )
                self.forced_open[i] = True
            elif not sample.engaged:
                if self.forced_open[i]:
                    logger.info(
                        "quest bridge: channel %s -- grip released, clutch re-armed",
                        self._channel_name(i),
                    )
                self.forced_open[i] = False

    def _gap_reason(self, i: int, now: float, sample, *, frame_ok: bool) -> str:
        """Why channel i just latched: whether the whole headset frame went
        away or only this controller did, and how long it had been gone."""
        if sample is not None:
            return "stream frozen (no pose change for longer than the mapper's window)"
        since = self._channels[i].gap_since
        held_ms = 0.0 if since is None else max(0.0, (now - since) * 1000.0)
        what = "whole-frame gap" if not frame_ok else "pose gap"
        return f"{what}, {held_ms:.0f} ms"

    # -- keepalive: its own daemon thread, never the tick path --------------

    def start_keepalive(self) -> None:
        """Start the background proximity keep-awake thread. No-op if
        `keepalive` is None (matches `--no-keepalive` and the tests) or a
        thread is already running. Fires immediately, then every
        `keepalive_period_s` -- entirely off the publish path, so a slow
        `am broadcast` (100 ms-1 s on-device) never stalls `tick()`."""
        if self.keepalive is None:
            return
        if self._keepalive_thread is not None and self._keepalive_thread.is_alive():
            return
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, name="vr-bridge-keepalive", daemon=True
        )
        self._keepalive_thread.start()

    def stop_keepalive(self) -> None:
        """Stop the background thread started by `start_keepalive()`, if any.
        Safe to call even if it was never started."""
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=2.0)
            self._keepalive_thread = None

    def _keepalive_loop(self) -> None:
        """Fire immediately, then every `keepalive_period_s`, until
        `stop_keepalive()` sets the stop event. Runs on its own thread: a slow
        or hung broadcast only delays the NEXT firing, never a `tick()`."""
        while not self._keepalive_stop.is_set():
            self._fire_keepalive_once()
            if self._keepalive_stop.wait(self.keepalive_period_s):
                return

    def _fire_keepalive_once(self) -> None:
        try:
            ok = self.keepalive()
        except Exception:
            ok = False
        if not ok and not self._keepalive_warned:
            self._keepalive_warned = True
            logger.warning(
                "quest bridge: proximity keep-awake broadcast failed (is `adb` on "
                "PATH?). The headset may suspend the teleop app when it reads "
                "'not worn' -- see docs/design.md."
            )

    # -- the loop ----------------------------------------------------------

    def run(self, stop_event: Optional[threading.Event] = None,
            max_ticks: Optional[int] = None) -> int:
        self.start_keepalive()
        try:
            period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.0
            ticks = 0
            next_tick = time.perf_counter()
            while True:
                if stop_event is not None and stop_event.is_set():
                    return 0
                if max_ticks is not None and ticks >= max_ticks:
                    return 0
                self.tick()
                ticks += 1
                next_tick += period
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.perf_counter()  # fell behind: resync, never spin
        finally:
            self.stop_keepalive()


def build_reader(oculus_ip: Optional[str] = None):
    """Construct the real OculusReader. The import is LAZY on purpose: it
    pulls in `ppadb` and talks to adb, so importing this module needs
    neither -- which is what makes the bridge testable off the hardware."""
    from .oculus_reader import OculusReader

    if oculus_ip:
        print(f"[vr_bridge] transport: Wi-Fi adb @ {oculus_ip}:5555", file=sys.stderr)
        return OculusReader(ip_address=oculus_ip)
    return OculusReader()


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Quest 3 controller -> VrTargetMsg bridge (see franka_vr.quest_bridge.bridge)."
    )
    p.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                   help="ZMQ PUB bind endpoint the teleop client SUBs to; "
                        "a comma list binds one per --controller")
    # No `choices=`: this is a comma list ('l,r'). main() validates each entry.
    p.add_argument("--controller", default="r",
                   help="'r' or 'l', or a comma list ('l,r') -- one per --endpoint")
    p.add_argument("--spatial-scale", type=float, default=1.0,
                   help="metres of robot motion per metre of hand motion")
    p.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    p.add_argument("--oculus-ip", default=None,
                   help="headset IP for Wi-Fi adb; omit for USB (preferred)")
    p.add_argument("--no-keepalive", action="store_true",
                   help="do not send the proximity keep-awake adb broadcasts")
    return p


def _split_list(raw: str):
    return [item.strip() for item in raw.split(",") if item.strip()]


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    args = build_arg_parser().parse_args(argv)

    # Validate argv BEFORE build_reader(): a typo must fail without touching
    # adb (which is slow, and whose own failure would mask this one).
    controllers = _split_list(args.controller)
    endpoints = _split_list(args.endpoint)
    if not controllers or not endpoints:
        print("[vr_bridge] FATAL: --controller and --endpoint must each name "
              "at least one channel", file=sys.stderr)
        return 2
    if len(controllers) != len(endpoints):
        print(f"[vr_bridge] FATAL: --controller ({len(controllers)}) and --endpoint "
              f"({len(endpoints)}) must name the same number of channels",
              file=sys.stderr)
        return 2
    for cid in controllers:
        if cid not in ("l", "r"):
            print(f"[vr_bridge] FATAL: --controller must be 'l' or 'r', got {cid!r}",
                  file=sys.stderr)
            return 2
    if len(set(endpoints)) != len(endpoints):
        # Two channels on one port: the second bind fails, and only the FIRST
        # arm would ever get poses. Cheaper to say so than to bind and unwind.
        print(f"[vr_bridge] FATAL: --endpoint has duplicate entries ({args.endpoint!r}); "
              "each channel needs its own port", file=sys.stderr)
        return 2

    try:
        reader = build_reader(args.oculus_ip)
    except Exception as exc:
        # Loud and NONZERO: whoever started the bridge shows this log tail
        # instead of leaving the operator with a mode that never engages.
        print(f"[vr_bridge] FATAL: could not start the Quest reader: {exc}",
              file=sys.stderr)
        print("[vr_bridge] check: `adb devices` shows the headset as 'device' "
              "(not 'unauthorized'), Hand Tracking is OFF, and the APK is "
              "installed (franka-vr-install-apk).",
              file=sys.stderr)
        return 2

    # The binds live INSIDE the try: a later endpoint failing to bind (a port
    # still held by a previous bridge) must not leak the earlier PUBs, and must
    # still reach reader.stop() -- an orphaned adb logcat stream would fight the
    # next start for the device.
    publishers = []
    try:
        try:
            for endpoint in endpoints:
                publishers.append(ZmqVrPublisher(endpoint))
        except Exception as exc:
            # Loud and NONZERO, like the reader failure above. The outer
            # `finally` closes whatever DID bind and stops the reader. Scoped
            # to the binds only, so a mid-flight run() failure is not
            # mislabelled as a bind failure.
            print(f"[vr_bridge] FATAL: could not bind {len(endpoints)} VR endpoint(s) "
                  f"{endpoints}: {exc}", file=sys.stderr)
            print("[vr_bridge] check: no other vr bridge is already running "
                  "(`pgrep -af franka-vr-bridge`) and each channel has its own port.",
                  file=sys.stderr)
            return 2
        bridge = QuestBridge(
            reader,
            publishers,
            mapper=[VrMapper(controller_id=cid, spatial_scale=args.spatial_scale)
                    for cid in controllers],
            rate_hz=args.rate,
            keepalive=None if args.no_keepalive else adb_proximity_keepalive,
        )
        for cid, pub in zip(controllers, publishers):
            print(f"[vr_bridge] ready. endpoint={pub.endpoint} "
                  f"controller={cid} rate={args.rate}Hz", file=sys.stderr)
        return bridge.run()
    except KeyboardInterrupt:
        return 0
    finally:
        for pub in publishers:
            pub.close()
        try:
            reader.stop()
        except Exception:
            logger.exception("vr_bridge: reader.stop() failed")


if __name__ == "__main__":
    raise SystemExit(main())
