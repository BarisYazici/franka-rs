"""The wire in: the 88-byte VrTargetMsg, from a live bridge endpoint or from a
record_wire.py .jsonl (or .jsonl.gz). teleop.py follows one endpoint; hands.py listens on both.
"""
import gzip
import json
import time

# Re-exported so this module stays the teleop side's one wire import.
from ..wire import (FLAG_CONTROLLER_ON, FLAG_ENGAGED, FLAG_FRESH,  # noqa: F401
                    VR_FMT, VR_MSG_MAGIC, VR_MSG_SIZE, VR_MSG_VERSION, decode_vr)


def import_zmq():
    try:
        import zmq
    except ImportError as exc:
        raise SystemExit(
            f"cannot import zmq ({exc}).\n"
            "A live bridge needs pyzmq: install franka-vr-teleop into the venv you run this\n"
            "from, or pass --replay FILE.") from exc
    return zmq


class ZmqSource:
    """SUB to one bridge endpoint. CONFLATE=1: on this path the newest sample is the only
    one worth having, and the bridge publishes at 50 Hz regardless."""

    def __init__(self, endpoint):
        zmq = import_zmq()
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.CONFLATE, 1)      # before connect
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(endpoint)
        self.zmq = zmq
        self.done = False
        self.live = True

    def now(self):
        return time.monotonic_ns()

    def poll(self, timeout_ms):
        out = []
        if self.sock.poll(timeout=timeout_ms) & self.zmq.POLLIN:
            while True:
                try:
                    out.append((time.monotonic_ns(), self.sock.recv(self.zmq.NOBLOCK)))
                except self.zmq.Again:
                    break
        return out

    def close(self):
        self.sock.close(0)
        self.ctx.term()


class ReplaySource:
    """A record_wire.py .jsonl -- gzipped or not -- replayed on its recorded arrival stamps
    with no socket and no wall-clock wait: the whole pipeline is driven by the recording's own clock, so a
    replay is deterministic and a 60 s wire runs in well under a second. The endpoint is
    matched exactly or by its port, never by falling back to whatever the file holds, so a
    replay cannot label one hand's wire as the other's."""

    def __init__(self, path, endpoint, speed=0.0):
        rows, endpoints = [], set()
        opener = gzip.open if path.endswith(".gz") else open   # the fixture ships gzipped
        with opener(path, "rt") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue                       # a truncated tail line
                if rec.get("type") or "raw" not in rec:
                    continue
                endpoints.add(rec.get("ep", ""))
                rows.append((rec["t"], rec.get("ep", ""), rec["raw"]))
        port = endpoint.rsplit(":", 1)[-1]
        want = [e for e in endpoints if e == endpoint or e.rsplit(":", 1)[-1] == port]
        if not want:
            raise SystemExit(f"{path}: no messages from {endpoint}; the file has "
                             + ", ".join(sorted(endpoints)))
        self.rows = [(t, bytes.fromhex(raw)) for t, ep, raw in sorted(rows) if ep in want]
        self.endpoint = want[0]
        self.i = 0
        self.live = False
        self.done = not self.rows
        self.speed = speed
        self._t0 = None
        self._t_last = self.rows[0][0] if self.rows else 0

    def now(self):
        if self.speed > 0.0 and self._t0 is not None:
            # real-time replay: the clock must run between records too, or a long hole in
            # the recording would stall the keepalive
            return int((time.monotonic_ns() - self._t0) * self.speed)
        return self._t_last

    def poll(self, timeout_ms):
        if self.i >= len(self.rows):
            self.done = True
            return []
        t, payload = self.rows[self.i]
        if self.speed > 0.0:                       # optional real-time replay
            if self._t0 is None:
                self._t0 = time.monotonic_ns() - t
            delay = (self._t0 + t / self.speed - time.monotonic_ns()) / 1e9
            if delay > 0:
                time.sleep(min(delay, 0.25))
                if delay > 0.25:
                    return []
        self.i += 1
        self._t_last = t
        self.done = self.i >= len(self.rows)
        return [(t, payload)]

    def close(self):
        pass
