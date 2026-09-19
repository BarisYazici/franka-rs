"""One arm's slice of the bridge: its publisher, its gap hold, its packed bytes.

`QuestBridge` (bridge.py) owns the reader and the loop; everything that is
*per arm* lives here, so the loop file stays about timing and the channel file
about one channel's state.

THREE RULES THIS FILE ENCODES, each an incident fix -- the reasoning is in
docs/design.md, which is the file to read before changing any of them:

* A one-tick reader hiccup is not a dead stream. `_Channel.resolve()`
  republishes the last usable sample verbatim for up to `GAP_HOLD_S` (100 ms),
  then gives up and drops it. `GAP_HOLD_S` has no CLI knob on purpose.
* The loop never skips a publish. An unusable frame still sends a
  not-fresh, not-engaged message carrying the last good pose: silence and
  not-fresh are different faults downstream and the operator needs to know
  which one they have.
* CONFLATE on the PUB side as well as the SUB side: a slow or dead consumer
  must never queue stale poses and then replay old hand motion.
"""
import dataclasses
import logging
import subprocess
from typing import List, Optional

import zmq

from .mapping import VrMapper
from ..wire import pack_vr_target

logger = logging.getLogger(__name__)

DEFAULT_ENDPOINT = "tcp://127.0.0.1:5560"
KEEPALIVE_PERIOD_S = 5.0
# How long an UNUSABLE frame republishes a channel's last usable sample before
# the channel is declared gapped and its clutch latched open. See resolve().
GAP_HOLD_S = 0.1
IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)

# The two broadcasts that stop the Quest suspending the teleop app when the
# proximity sensor reads "not worn". Verified on Quest 3 / HorizonOS.
_PROX_BROADCASTS = (
    "com.oculus.vrpowermanager.automation_disable",
    "com.oculus.vrpowermanager.prox_close",
)


class ZmqVrPublisher:
    """PUB socket bound to `endpoint`. `endpoint` may use port 0 (ephemeral);
    `self.endpoint` then holds the port actually bound."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        # CONFLATE on the PUB side too: a slow/dead consumer must never let a
        # queue of stale poses build up and then replay old hand motion.
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.bind(endpoint)
        self.endpoint = self._sock.getsockopt_string(zmq.LAST_ENDPOINT)

    def send(self, payload: bytes) -> None:
        self._sock.send(payload, flags=zmq.NOBLOCK)

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            logger.exception("ZmqVrPublisher.close failed")


def adb_proximity_keepalive(serial: Optional[str] = None) -> bool:
    """Send both proximity broadcasts through the `adb` BINARY (not ppadb,
    whose connection the reader's logcat stream is already using). Never
    raises; returns True only if both broadcasts exited 0."""
    ok = True
    for action in _PROX_BROADCASTS:
        argv = ["adb"]
        if serial:
            argv += ["-s", serial]
        argv += ["shell", "am", "broadcast", "-a", action]
        try:
            rc = subprocess.run(
                argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0
            ).returncode
            ok = ok and rc == 0
        except Exception:
            ok = False
    return ok


@dataclasses.dataclass
class _Channel:
    """One arm's slice of the bridge: the mapper that reads ITS controller out
    of the shared frame, the PUB that carries the result, and the last good
    pose republished through a dropped frame."""

    mapper: VrMapper
    publisher: object
    last_pos: tuple = (0.0, 0.0, 0.0)
    last_quat: tuple = IDENTITY_QUAT
    # The last USABLE sample, and when this channel's current run of unusable
    # frames began (None = not in one). Both exist only for the short-gap
    # republish -- see resolve() and docs/design.md.
    last_sample: object = None
    gap_since: Optional[float] = None

    def resolve(self, sample, now: float, hold_s: float):
        """The sample this channel should publish this tick, or None.

        A usable sample passes straight through and re-arms the hold. An
        unusable one (`None`: this controller missing from the frame, its
        buttons missing, the reader raised, or an empty frame) republishes the
        last usable sample VERBATIM for up to `hold_s`, then gives up and
        returns None -- which is the sustained-gap verdict `_update_forced_
        open` latches the clutch on.

        FAIL CLOSED AT THE EDGES: with nothing yet to republish (startup, or a
        gap that already gave up) the answer is None immediately. Giving up
        also DROPS the held sample, so a later blip can never resurrect a pose
        from before the outage.
        """
        if sample is not None:
            self.last_sample = sample
            self.gap_since = None
            return sample
        if self.last_sample is None:
            return None
        if self.gap_since is None:
            self.gap_since = now
        if (now - self.gap_since) < hold_s:
            return self.last_sample
        self.last_sample = None
        return None

    def publish(self, seq: int, sample, forced_open: bool) -> bytes:
        """Pack and send this channel's message for one tick. `sample is None`
        is an unusable frame: republish the last good pose, not-fresh."""
        if sample is None:
            payload = pack_vr_target(
                seq, self.last_pos, self.last_quat, 0.0,
                engaged=False, fresh=False, controller_on=False, buttons=0,
            )
        else:
            self.last_pos = tuple(float(v) for v in sample.pos)
            self.last_quat = tuple(float(v) for v in sample.quat)
            payload = pack_vr_target(
                seq, sample.pos, sample.quat, sample.gripper,
                engaged=bool(sample.engaged) and not forced_open,
                fresh=sample.fresh, controller_on=sample.controller_on,
                buttons=sample.buttons,
            )
        self.publisher.send(payload)
        return payload


def _build_channels(publisher, mapper=None) -> List[_Channel]:
    """Normalise the single-object-or-sequence `publisher`/`mapper` arguments
    into one channel per publisher."""
    pubs = list(publisher) if isinstance(publisher, (list, tuple)) else [publisher]
    if not pubs:
        raise ValueError("a QuestBridge needs at least one publisher (channel)")
    if mapper is None:
        maps = [VrMapper() for _ in pubs]
    elif isinstance(mapper, (list, tuple)):
        maps = list(mapper)
    else:
        maps = [mapper]
    if len(maps) != len(pubs):
        raise ValueError(
            f"got {len(maps)} mapper(s) for {len(pubs)} publisher(s); "
            "one mapper per publisher is required"
        )
    return [_Channel(m, p) for m, p in zip(maps, pubs)]
