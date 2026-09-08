#!/usr/bin/env python3
"""Plots a `nonrealtime_commander --log` CSV: target vs commanded vs measured position, one
panel per axis, with the commander's stall and burst marked.

Usage: plot.py run.csv out.png [--title TEXT]
"""

import argparse
import csv
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

AXES = ("x", "y", "z")
# target / commanded / measured: three categorical hues, one per series, fixed order.
COLORS = {"target": "#eb6834", "cmd": "#2a78d6", "meas": "#1baf7a"}


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {key: [float(r[key]) for r in rows] for key in rows[0]}


def target_changes(d):
    """Indices of the cycles on which the raw target differs from the previous cycle's."""
    keys = ["target_" + a for a in AXES]
    return [i for i in range(1, len(d["t"])) if any(d[k][i] != d[k][i - 1] for k in keys)]


def peak_speed(d, prefix):
    keys = [prefix + a for a in AXES]
    best = 0.0
    for i in range(1, len(d["t"])):
        dt = d["t"][i] - d["t"][i - 1]
        if dt > 0:
            step = math.sqrt(sum((d[k][i] - d[k][i - 1]) ** 2 for k in keys))
            best = max(best, step / dt)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("csv")
    parser.add_argument("png")
    parser.add_argument("--title", default=None)
    args = parser.parse_args()

    d = load(args.csv)
    t = d["t"]
    changes = target_changes(d)
    # The stall is the longest gap between two target changes (before the final hold); the
    # burst is the 100 ms window holding the most changes.
    stall = max(zip(changes, changes[1:]), key=lambda p: t[p[1]] - t[p[0]], default=None)
    burst = None
    for i, start in enumerate(changes):
        n = sum(1 for c in changes[i:] if t[c] - t[start] <= 0.1)
        if n >= 3 and (burst is None or n > burst[1]):
            burst = (start, n)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    for ax, axis in zip(axes, AXES):
        ax.step(t, d["target_" + axis], where="post", color=COLORS["target"], lw=1.2, label="target (raw)")
        ax.plot(t, d["cmd_" + axis], color=COLORS["cmd"], lw=2, label="commanded O_T_EE_c")
        ax.plot(t, d["meas_" + axis], color=COLORS["meas"], lw=1.2, ls="--", label="measured O_T_EE")
        ax.set_ylabel(f"{axis} [m]")
        ax.grid(True, color="#e5e5e5", lw=0.6)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        if stall is not None:
            ax.axvspan(t[stall[0]], t[stall[1]], color="#999999", alpha=0.12, lw=0)
        if burst is not None:
            ax.axvspan(t[burst[0]], t[burst[0]] + 0.1, color=COLORS["target"], alpha=0.15, lw=0)
    if stall is not None:
        axes[0].annotate(f"no new target for {t[stall[1]] - t[stall[0]]:.1f} s", (t[stall[0]], 1.0),
                         xycoords=("data", "axes fraction"), va="bottom", fontsize=9, color="#555555")
    if burst is not None:
        axes[0].annotate(f"burst: {burst[1]} targets in 100 ms", (t[burst[0]], 1.0),
                         xycoords=("data", "axes fraction"), va="bottom", ha="right", fontsize=9,
                         color=COLORS["target"])
    axes[-1].set_xlabel("time [s]")
    axes[0].legend(loc="upper right", frameon=False, fontsize=9)
    title = args.title or f"{args.csv}: {len(changes)} target changes, {len(t)} cycles"
    fig.suptitle(f"{title} -- peak commanded speed {peak_speed(d, 'cmd_'):.3f} m/s", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.png, dpi=130)
    print(f"{args.png}: peak commanded speed {peak_speed(d, 'cmd_'):.4f} m/s")


if __name__ == "__main__":
    main()
