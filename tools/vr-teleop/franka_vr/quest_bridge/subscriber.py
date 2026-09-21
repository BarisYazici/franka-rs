"""The SUB side of the VrTargetMsg channel: wait for one, or stream many.

`wait_for_vr_target` is the bounded gate -- prove poses are actually flowing
before anything is asked to follow them. `recv_vr_targets`/`_multi` are the
streams `probe` folds into its counts. Socket management lives
here so neither caller owns a detail the other also needs.
"""
import logging
import time
from typing import Iterable, Optional

import zmq

from ..wire import VR_MSG_SIZE, unpack_vr_target

logger = logging.getLogger(__name__)


def _sub_socket(ctx, endpoint: str, conflate: bool):
    """A connected SUB, or None if the endpoint cannot be used."""
    sub = ctx.socket(zmq.SUB)
    try:
        sub.setsockopt(zmq.SUBSCRIBE, b"")
        if conflate:
            sub.setsockopt(zmq.CONFLATE, 1)
        sub.setsockopt(zmq.LINGER, 0)
        sub.connect(endpoint)
        return sub
    except Exception:
        logger.exception("subscriber: setup failed for %s (bad endpoint?)", endpoint)
        try:
            sub.close(0)
        except Exception:
            logger.exception("subscriber: close after failed setup")
        return None


def wait_for_vr_target(endpoint: str, timeout_s: float = 10.0,
                       poll_s: float = 0.05, require_fresh: bool = False) -> Optional[dict]:
    """Return the first decodable VrTargetMsg seen on `endpoint`, or None on
    timeout. Never raises.

    `require_fresh`: when True, a decodable message whose `fresh` bit is
    False (the bridge's own "reader saw no new controller sample this tick"
    fallback -- see `mapping`) does NOT satisfy the wait; only a
    message with `fresh=True` does. A start-up gate passes True: a bridge that
    is alive and PUBbing but only ever emitting stale/fallback frames --
    headset asleep, hand tracking still on instead of controllers, the APK
    not actually streaming -- must fail the gate exactly like a silent
    bridge, not be mistaken for "poses are flowing".
    """
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    try:
        try:
            sub.setsockopt(zmq.SUBSCRIBE, b"")
            sub.setsockopt(zmq.CONFLATE, 1)
            sub.setsockopt(zmq.LINGER, 0)
            sub.connect(endpoint)
        except Exception:
            logger.exception("wait_for_vr_target: setup failed (bad endpoint?)")
            return None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                buf = sub.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(poll_s)
                continue
            except Exception:
                logger.exception("wait_for_vr_target: recv failed")
                return None
            if len(buf) != VR_MSG_SIZE:
                continue
            try:
                msg = unpack_vr_target(buf)
            except ValueError:
                continue  # garbage on the socket is not a reason to give up
            if require_fresh and not msg.get("fresh"):
                continue  # a fallback frame does not prove poses are flowing
            return msg
        return None
    except Exception:
        logger.exception("wait_for_vr_target: unexpected failure")
        return None
    finally:
        try:
            sub.close(0)
        except Exception:
            logger.exception("wait_for_vr_target: close failed")


def recv_vr_targets_multi(endpoints: Iterable[str], duration_s: float,
                          poll_s: float = 0.01, conflate: bool = True):
    """Yield `(endpoint, msg)` for `duration_s` across several endpoints at
    once. Generator; never raises.

    `msg` is the decoded dict, or None for a buffer that is not a VrTargetMsg
    -- the probe counts those as `bad`, because a foreign publisher on the
    port is itself a diagnosis rather than a reason to stop reading.

    `conflate` False keeps EVERY message, which is what a probe that counts
    and rate-checks needs; True (the default) keeps only the latest, which is
    what a consumer that wants the freshest pose needs.
    """
    ctx = zmq.Context.instance()
    socks = {}
    try:
        poller = zmq.Poller()
        for endpoint in endpoints:
            sub = _sub_socket(ctx, endpoint, conflate)
            if sub is None:
                continue
            socks[sub] = endpoint
            poller.register(sub, zmq.POLLIN)
        if not socks:
            return
        timeout_ms = max(1, int(poll_s * 1000.0))
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            try:
                ready = poller.poll(timeout_ms)
            except Exception:
                logger.exception("recv_vr_targets_multi: poll failed")
                return
            for sub, _ev in ready:
                try:
                    buf = sub.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    continue
                except Exception:
                    logger.exception("recv_vr_targets_multi: recv failed")
                    return
                try:
                    msg = unpack_vr_target(buf)
                except ValueError:
                    msg = None
                yield socks[sub], msg
    except Exception:
        logger.exception("recv_vr_targets_multi: unexpected failure")
        return
    finally:
        for sub in socks:
            try:
                sub.close(0)
            except Exception:
                logger.exception("recv_vr_targets_multi: close failed")


def recv_vr_targets(endpoint: str, duration_s: float, poll_s: float = 0.01):
    """Yield decoded VrTargetMsgs from one endpoint for `duration_s`,
    latest-only. Generator; never raises."""
    for _endpoint, msg in recv_vr_targets_multi([endpoint], duration_s, poll_s):
        if msg is not None:
            yield msg
