"""Generic statistics helpers: percentiles, medians, nanosecond/microsecond units."""

import math

NS_PER_MS = 1_000_000
NS_PER_US = 1_000

# ---------------------------------------------------------------------------
# statistics helpers
# ---------------------------------------------------------------------------

def pct(sorted_vals, q):
    if not sorted_vals:
        return 0.0
    k = max(0, min(len(sorted_vals) - 1, math.ceil(q * len(sorted_vals)) - 1))
    return sorted_vals[k]


def stats_us(values_ns):
    """min/p50/p99/p999/max/mean of a list of nanosecond values, in microseconds."""
    if not values_ns:
        return None
    s = sorted(values_ns)
    return {
        "n": len(s),
        "min": s[0] / NS_PER_US,
        "p50": pct(s, 0.50) / NS_PER_US,
        "p99": pct(s, 0.99) / NS_PER_US,
        "p999": pct(s, 0.999) / NS_PER_US,
        "max": s[-1] / NS_PER_US,
        "mean": (sum(s) / len(s)) / NS_PER_US,
    }


def fmt_stats(name, st):
    if st is None:
        return "  %-28s (no samples)" % name
    return ("  %-28s n=%-7d min=%9.1f  p50=%9.1f  p99=%9.1f  p999=%9.1f  "
            "max=%9.1f  mean=%9.1f" %
            (name, st["n"], st["min"], st["p50"], st["p99"], st["p999"],
             st["max"], st["mean"]))


def median(vals):
    if not vals:
        return 0
    s = sorted(vals)
    m = len(s) // 2
    return s[m] if len(s) % 2 else (s[m - 1] + s[m]) / 2
