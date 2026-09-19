"""Operator-facing metrics from the node's 100 Hz state stream. Pure Python, no numpy.

The engine consumes normalised samples (see `Sample`), keeps a 60 s ring, and answers with
the numbers the panel shows: j4 jitter (RMS of high-passed dq[3]), all-joint jitter, lag
(cross-correlation of accepted target vs measured pose), tracking error, dq headroom, and the
before/after windows either side of the last Apply marker.

One engine is shared by the Zenoh subscriber (push), the 10 Hz ticker and the HTTP threads
(snapshot, mark); every public method takes the engine's lock. All times are the state stream's
own clock (`Sample.t`), never the bridge host's, so before/after windows stay aligned across hosts.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence

STATE_HZ = 100.0
RING_S = 60.0
JITTER_WINDOW_S = 0.5
LAG_WINDOW_S = 3.0
LAG_MAX_S = 0.5
BEFORE_S = 5.0
AFTER_SKIP_S = 0.5  # skip the slew before the after window starts
STALE_S = 1.0  # no new sample for this long (bridge clock) and the snapshot says so
JITTER_LINES = {"quiet": 0.025, "sim_pass": 0.05, "burst": 0.1}


@dataclass
class Sample:
    t: float
    dq: Sequence[float]
    target: Sequence[float]  # accepted target position, xyz
    ee: Sequence[float]  # measured end effector position, xyz
    valid: bool = True  # False during reflex/errors: excluded from all windows


def butter_highpass_sos(order: int, fc: float, fs: float) -> List[List[float]]:
    """Butterworth high-pass as second-order sections [b0,b1,b2,a1,a2], via bilinear transform."""
    assert order % 2 == 0
    wc = 2.0 * fs * math.tan(math.pi * fc / fs)  # prewarped analogue cutoff
    sos = []
    for k in range(order // 2):
        theta = math.pi * (2 * k + 1 + order) / (2 * order)
        # analogue low-pass pole pair p = wc * e^{j theta}; high-pass by s -> wc^2 / s
        # yields H(s) = s^2 / (s^2 + 2 zeta wc s + wc^2), zeta = -cos(theta)
        zeta = -math.cos(theta)
        c = 2.0 * fs
        a0 = c * c + 2 * zeta * wc * c + wc * wc
        b0 = c * c / a0
        b1 = -2 * c * c / a0
        b2 = c * c / a0
        a1 = (2 * wc * wc - 2 * c * c) / a0
        a2 = (c * c - 2 * zeta * wc * c + wc * wc) / a0
        sos.append([b0, b1, b2, a1, a2])
    return sos


class SosFilter:
    def __init__(self, sos: List[List[float]]):
        self.sos = sos
        self.z = [[0.0, 0.0] for _ in sos]

    def step(self, x: float) -> float:
        for (b0, b1, b2, a1, a2), z in zip(self.sos, self.z):
            y = b0 * x + z[0]
            z[0] = b1 * x - a1 * y + z[1]
            z[1] = b2 * x - a2 * y
            x = y
        return x


def _rms(xs: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


def _percentile(xs: List[float], p: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    k = (len(s) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def lag_samples(target: Sequence[float], measured: Sequence[float], max_lag: int) -> Optional[int]:
    """Argmax over lag of the normalised cross-correlation of two detrended series.
    Returns None when either series is flat (the arm is not moving, so there is no lag).
    The sum is not normalised by overlap length, so long lags are slightly penalised: the
    estimate biases short and is meant as a relative before/after number, not an absolute."""
    n = min(len(target), len(measured))
    if n < 2 * max_lag + 10:
        return None
    a = list(target[:n])
    b = list(measured[:n])
    ma, mb = sum(a) / n, sum(b) / n
    a = [x - ma for x in a]
    b = [x - mb for x in b]
    ea, eb = sum(x * x for x in a), sum(x * x for x in b)
    if ea < 1e-8 or eb < 1e-8:
        return None
    best, best_lag = -2.0, 0
    for lag in range(0, max_lag + 1):
        s = 0.0
        for i in range(lag, n):
            s += a[i - lag] * b[i]
        r = s / math.sqrt(ea * eb)
        if r > best:
            best, best_lag = r, lag
    return best_lag


@dataclass
class WindowStats:
    j4_rms: float = 0.0
    all_rms: float = 0.0
    lag_ms: Optional[float] = None
    track_p50: float = 0.0
    track_p99: float = 0.0
    n: int = 0

    def as_dict(self) -> Dict:
        return {"j4_rms": self.j4_rms, "all_rms": self.all_rms, "lag_ms": self.lag_ms,
                "track_p50_mm": self.track_p50 * 1e3, "track_p99_mm": self.track_p99 * 1e3, "n": self.n}


@dataclass
class _Row:
    t: float
    hp: List[float]  # high-passed dq, 7
    dq: List[float]
    target: List[float]
    ee: List[float]
    err: float
    valid: bool


@dataclass
class Marker:
    t: float
    label: str
    version: Optional[int] = None


class MetricsEngine:
    """`dq_limit` and `release_fraction` come from the owners (node `derived.dq_limit`, teleop
    `dq_release_fraction`); until both are known the headroom is reported as unavailable."""

    def __init__(self, fs: float = STATE_HZ):
        self.fs = fs
        self.dq_limit: Optional[List[float]] = None
        self.release_fraction: Optional[float] = None
        sos = butter_highpass_sos(4, 3.0, fs)
        self.filters = [SosFilter([list(s) for s in sos]) for _ in range(7)]
        self.rows: Deque[_Row] = deque()
        self.markers: List[Marker] = []
        self.peak_since_marker = 0.0
        self.last_arrival: Optional[float] = None  # bridge monotonic clock, for staleness only
        self.lock = threading.Lock()

    # ---- input -----------------------------------------------------------------------------
    def set_limits(self, dq_limit: Optional[Sequence[float]] = None,
                   release_fraction: Optional[float] = None) -> None:
        with self.lock:
            if dq_limit is not None:
                self.dq_limit = list(dq_limit)
            if release_fraction is not None:
                self.release_fraction = release_fraction

    def push(self, s: Sample) -> None:
        with self.lock:
            hp = [f.step(v) for f, v in zip(self.filters, s.dq)]
            err = math.dist(s.target[:3], s.ee[:3]) if len(s.target) >= 3 and len(s.ee) >= 3 else 0.0
            self.rows.append(_Row(s.t, hp, list(s.dq), list(s.target[:3]), list(s.ee[:3]), err, s.valid))
            while self.rows and self.rows[0].t < s.t - RING_S:
                self.rows.popleft()
            self.last_arrival = time.monotonic()
            if not s.valid:
                # a reflex pollutes before/after: invalidate every row so both windows restart
                for r in self.rows:
                    r.valid = False

    def mark(self, label: str, version: Optional[int] = None) -> Optional[Marker]:
        """Stamp a marker at the newest state sample's time; None when no state has arrived."""
        with self.lock:
            if not self.rows:
                return None
            m = Marker(self.rows[-1].t, label, version)
            self.markers.append(m)
            self.markers = self.markers[-20:]
            self.peak_since_marker = 0.0
            return m

    def clear_markers(self) -> None:
        with self.lock:
            self.markers.clear()
            self.peak_since_marker = 0.0

    # ---- output ----------------------------------------------------------------------------
    def _window(self, t0: float, t1: float) -> WindowStats:
        """Caller holds `self.lock`; this is the one reader of `rows` that does not take it."""
        rows = [r for r in self.rows if t0 <= r.t < t1 and r.valid]
        w = WindowStats(n=len(rows))
        if not rows:
            return w
        w.j4_rms = _rms([r.hp[3] for r in rows])
        w.all_rms = max(_rms([r.hp[j] for r in rows]) for j in range(7))
        errs = [r.err for r in rows]
        w.track_p50, w.track_p99 = _percentile(errs, 0.5), _percentile(errs, 0.99)
        lag = self._lag(rows)
        w.lag_ms = None if lag is None else lag * 1e3 / self.fs
        return w

    def _lag(self, rows: List[_Row]) -> Optional[int]:
        max_lag = int(LAG_MAX_S * self.fs)
        rows = rows[-int(LAG_WINDOW_S * self.fs):]  # a rolling after-window can be 60 s; lag is O(n·lags)
        lags = []
        for ax in range(3):
            lag = lag_samples([r.target[ax] for r in rows], [r.ee[ax] for r in rows], max_lag)
            if lag is not None:
                lags.append(lag)
        return max(lags) if lags else None

    def snapshot(self) -> Dict:
        with self.lock:
            if not self.rows:
                return {"ok": False, "reason": "no_state"}
            age = time.monotonic() - self.last_arrival
            if age > STALE_S:
                return {"ok": False, "reason": "stale_state", "age_s": age}
            now = self.rows[-1].t
            cur = self._window(now - JITTER_WINDOW_S, now + 1e-9)
            lagw = self._window(now - LAG_WINDOW_S, now + 1e-9)
            self.peak_since_marker = max(self.peak_since_marker, cur.j4_rms)
            last = self.rows[-1]
            out = {
                "ok": True, "t": now,
                "j4_rms": cur.j4_rms, "all_rms": cur.all_rms, "j4_peak_since_marker": self.peak_since_marker,
                "lag_ms": lagw.lag_ms, "track_p50_mm": lagw.track_p50 * 1e3, "track_p99_mm": lagw.track_p99 * 1e3,
                "dq_headroom": self._headroom(last.dq),
                "lines": JITTER_LINES, "valid": last.valid,
                "windows": {"before_s": BEFORE_S, "after_skip_s": AFTER_SKIP_S},
                "markers": [{"t": m.t, "label": m.label, "version": m.version} for m in self.markers],
            }
            if self.markers:
                m = self.markers[-1]
                before = self._window(m.t - BEFORE_S, m.t)
                after = self._window(m.t + AFTER_SKIP_S, now + 1e-9)  # rolls until the next marker
                out["before"] = before.as_dict()
                out["after"] = after.as_dict()
                out["compare"] = compare(before, after)
            return out

    def _headroom(self, dq: Sequence[float]) -> Optional[Dict]:
        if self.dq_limit is None or self.release_fraction is None:
            return None
        ratios = [abs(v) / lim for v, lim in zip(dq, self.dq_limit)]
        worst = max(range(len(ratios)), key=lambda i: ratios[i])
        return {"joint": worst, "fraction": ratios[worst], "release": self.release_fraction, "fractions": ratios}


def compare(before: WindowStats, after: WindowStats) -> Dict[str, Optional[str]]:
    """'better' / 'worse' / 'same' per metric, lower is better for every one of them."""
    def verdict(b: Optional[float], a: Optional[float], tol: float) -> Optional[str]:
        if b is None or a is None or before.n == 0 or after.n == 0:
            return None
        if a < b * (1 - tol):
            return "better"
        if a > b * (1 + tol):
            return "worse"
        return "same"
    return {"j4_rms": verdict(before.j4_rms, after.j4_rms, 0.1),
            "lag_ms": verdict(before.lag_ms, after.lag_ms, 0.05),
            "track_p99_mm": verdict(before.track_p99, after.track_p99, 0.1)}
