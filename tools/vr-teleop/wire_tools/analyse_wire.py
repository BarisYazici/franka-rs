#!/usr/bin/env python3
"""Read a record_wire.py recording and report what a teleoperation consumer has
to know before it is written. Offline, stdlib only, read-only.

    python analyse_wire.py run.jsonl

Every number carries its units. The thresholds it compares against are
franka-node's and franka-rs's defaults, named at each use:

    max_step             0.05 m   per message, from the previous ACCEPTED target
    max_step_rotation    0.26 rad per message
    workspace box        x 0.2..0.8, y -0.5..0.5, z 0.0..0.8 m, robot base frame
    translation budget   0.3 m/s, 0.5 m/s^2  (a NORM; per axis / sqrt(3))
    rotation budget      0.5 rad/s, 1.0 rad/s^2 (a norm)
    rate_hz              250 per client
"""
import argparse
import json
import math
import statistics as st
import sys

MAX_STEP_M = 0.05
MAX_STEP_ROT_RAD = 0.26
WORKSPACE = ((0.2, 0.8), (-0.5, 0.5), (0.0, 0.8))
BUDGET_V = 0.3            # m/s, norm
BUDGET_A = 0.5            # m/s^2, norm
BUDGET_W = 0.5            # rad/s, norm
SQRT3 = math.sqrt(3.0)


def pct(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round(q / 100.0 * (len(s) - 1)))))
    return s[i]


def dist(xs, unit, scale=1.0, fmt="{:8.3f}"):
    if not xs:
        return "   (none)"
    f = lambda v: fmt.format(v * scale)
    return (f"min {f(min(xs))}  p50 {f(pct(xs,50))}  p90 {f(pct(xs,90))}  "
            f"p99 {f(pct(xs,99))}  max {f(max(xs))}  {unit}")


def load(path):
    header, footer, rows, truncated = None, None, [], 0
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                truncated += 1
                continue
            kind = rec.get("type")
            if kind == "header":
                header = rec
            elif kind == "footer":
                footer = rec
            else:
                rows.append(rec)
    rows.sort(key=lambda r: r["t"])
    return header, footer, rows, truncated


def quat_angle(a, b):
    """Radians between two orientations, sign-canonicalised (|dot|)."""
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def quat_angle_naive(a, b):
    """The same without canonicalising -- what a consumer that trusts the
    quaternion's sign would compute."""
    d = sum(x * y for x, y in zip(a, b))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def episodes(rows, key):
    """[(t_start_ns, t_end_ns, n_msgs)] of consecutive rows where key(row)."""
    out, start, n, last = [], None, 0, None
    for r in rows:
        if key(r):
            if start is None:
                start, n = r["t"], 0
            n += 1
            last = r["t"]
        elif start is not None:
            out.append((start, last, n))
            start = None
    if start is not None:
        out.append((start, last, n))
    return out


def track(samples, v_max, a_max):
    """Derived: a per-axis rate- and acceleration-limited follower over the
    recorded target path. Returns the peak lead, metres.

    Not the library's jerk-limited OTG -- it has no jerk term, so it is
    optimistic by the 0.625 s / 0.054 m ramp the real generator needs. Read it
    as a lower bound on the lead a real arm would show.
    """
    if len(samples) < 2:
        return float("nan")
    arm = list(samples[0][1])
    vel = [0.0, 0.0, 0.0]
    peak = 0.0
    for (t_prev, _), (t, tgt) in zip(samples, samples[1:]):
        dt = (t - t_prev) / 1e9
        if dt <= 0 or dt > 0.5:
            arm, vel = list(tgt), [0.0, 0.0, 0.0]
            continue
        for i in range(3):
            e = tgt[i] - arm[i]
            v_want = math.copysign(min(v_max, math.sqrt(2.0 * a_max * abs(e))), e)
            vel[i] = max(vel[i] - a_max * dt, min(vel[i] + a_max * dt, v_want))
            arm[i] += vel[i] * dt
        lead = math.dist(arm, tgt)
        peak = max(peak, lead)
    return peak


def report_endpoint(ep, rows, out):
    p = lambda *a: print(*a, file=out)
    good = [r for r in rows if "bad" not in r]
    p(f"\n{'='*78}\n{ep}: {len(rows)} messages ({len(rows)-len(good)} malformed)\n{'='*78}")
    if len(good) < 2:
        p("  too few messages to analyse")
        return
    span = (good[-1]["t"] - good[0]["t"]) / 1e9

    # --- 1. rate and jitter ------------------------------------------------
    dts = [(b["t"] - a["t"]) / 1e6 for a, b in zip(good, good[1:])]   # ms
    nominal = st.median(dts)
    p("\n1. PUBLICATION RATE AND JITTER (arrival times, this host's "
      "perf_counter)")
    p(f"   span              {span:8.3f} s, {len(good)} messages "
      f"-> {len(good)/span:7.3f} Hz mean")
    p(f"   inter-arrival     {dist(dts, 'ms')}")
    p(f"   implied rate      {1000.0/nominal:7.3f} Hz at the median interval "
      f"({nominal:.3f} ms)")
    p(f"   jitter (stdev)    {st.pstdev(dts):8.3f} ms; "
      f"mean absolute deviation {st.fmean(abs(d-nominal) for d in dts):8.3f} ms")
    late = [d for d in dts if d > 2 * nominal]
    p(f"   intervals > 2x nominal: {len(late)} ({100.0*len(late)/len(dts):.2f} %)"
      f"  longest {max(dts):.1f} ms")
    p(f"   vs franka-node rate_hz 250 per client: "
      f"{'OK' if 1000.0/min(dts) <= 250 else 'EXCEEDS at the shortest interval'}"
      f" (fastest interval {min(dts):.3f} ms = {1000.0/min(dts):.1f} Hz)")

    # --- 2. the pose's frame and units -------------------------------------
    p("\n2. THE POSE: FRAME AND UNITS")
    for i, ax in enumerate("xyz"):
        vals = [r["pos"][i] for r in good]
        p(f"   pos[{ax}]           min {min(vals):8.4f}  max {max(vals):8.4f}  "
          f"mean {st.fmean(vals):8.4f}  range {max(vals)-min(vals):7.4f}  m")
    norms = [math.dist(r["pos"], (0, 0, 0)) for r in good]
    p(f"   |pos|             {dist(norms, 'm')}")
    inside = sum(1 for r in good
                 if all(lo <= v <= hi for v, (lo, hi) in zip(r["pos"], WORKSPACE)))
    p(f"   inside franka-node's default workspace box: {inside}/{len(good)} "
      f"({100.0*inside/len(good):.1f} %)")
    p("   -> the units are metres; the axes are the robot's DIRECTIONS "
      "(rmat_reorder [-3,-1,2,4]),")
    p("      but the origin is the Quest's SLAM origin. A consumer must "
      "publish pos + a clutch")
    p("      offset, never pos itself, or every target is refused for "
      "Workspace.")
    # Constancy is only meaningful WITHIN one disengaged run: across runs the
    # latch has moved, and a grip held while the clutch is forced open reads as
    # disengaged while the orientation is still accumulating.
    spreads = []
    for a, b, _n in episodes(good, lambda r: not r["engaged"]):
        qs = [r["quat"] for r in good if a <= r["t"] <= b]
        if len(qs) > 1:
            spreads.append(max(quat_angle(qs[0], q) for q in qs))
    q_dis = [r["quat"] for r in good if not r["engaged"]]
    if spreads:
        flips = sum(1 for a, b in zip(q_dis, q_dis[1:])
                    if sum(x*y for x, y in zip(a, b)) < 0)
        p(f"   within one disengaged run the orientation is constant to "
          f"{dist([math.degrees(v) for v in spreads], 'deg')}")
        p(f"      ({len(spreads)} runs) -> confirms the wire quaternion is a "
          f"rotation-DELTA carrier, not an absolute pose")
        p(f"   quaternion SIGN flips between consecutive disengaged messages: "
          f"{flips}/{len(q_dis)-1} "
          f"({100.0*flips/max(1,len(q_dis)-1):.1f} %)")
        p("      -> scipy's from_matrix().as_quat() picks either sign; a "
          "consumer MUST canonicalise")
        p("         (|dot|, or force w >= 0) or it reads a ~180 deg step on "
          "half the messages")

    # --- 3. translation step ------------------------------------------------
    steps = [(math.dist(a["pos"], b["pos"]), (b["t"] - a["t"]) / 1e9,
              a["engaged"] and b["engaged"])
             for a, b in zip(good, good[1:])]
    all_s = [s for s, _, _ in steps]
    eng_s = [s for s, _, e in steps if e]
    p("\n3. TRANSLATION STEP PER MESSAGE  (franka-node max_step = 0.05 m)")
    p(f"   all messages      {dist(all_s, 'mm', 1000.0)}")
    p(f"   engaged only      {dist(eng_s, 'mm', 1000.0)}  ({len(eng_s)} pairs)")
    over = [s for s in all_s if s > MAX_STEP_M]
    over_e = [s for s in eng_s if s > MAX_STEP_M]
    p(f"   over max_step     {len(over)}/{len(all_s)} all "
      f"({100.0*len(over)/len(all_s):.3f} %), "
      f"{len(over_e)}/{max(1,len(eng_s))} engaged")
    p(f"   VERDICT           a 5 cm step limit at this rate: "
      f"{'PASSES' if not over else 'REFUSES ' + str(len(over)) + ' messages'}"
      f" (largest step {1000.0*max(all_s):.1f} mm)")
    speeds = [s / dt for s, dt, _ in steps if dt > 0]
    sp_eng = [s / dt for s, dt, e in steps if dt > 0 and e]
    p(f"   implied speed     {dist(speeds, 'm/s')}")
    p(f"   engaged only      {dist(sp_eng, 'm/s')}")
    p(f"   vs budget         norm {BUDGET_V} m/s, per axis "
      f"{BUDGET_V/SQRT3:.3f} m/s: "
      f"{sum(1 for s in speeds if s > BUDGET_V)} messages over the norm "
      f"({100.0*sum(1 for s in speeds if s > BUDGET_V)/len(speeds):.2f} %)")
    p(f"   max_step at this rate permits a target at "
      f"{MAX_STEP_M*1000.0/nominal:.2f} m/s "
      f"-- {MAX_STEP_M*1000.0/nominal/BUDGET_V:.1f}x the budget the arm can "
      f"chase")

    # --- 4. sequence --------------------------------------------------------
    seqs = [r["seq"] for r in good]
    incs = [b - a for a, b in zip(seqs, seqs[1:])]
    hist = {}
    for d in incs:
        hist[d] = hist.get(d, 0) + 1
    p("\n4. SEQUENCE (franka-node requires strictly increasing per client)")
    p(f"   range             {min(seqs)} .. {max(seqs)}  "
      f"({max(seqs)-min(seqs)+1} ticks spanned by {len(seqs)} messages)")
    p(f"   increments        " + ", ".join(
        f"+{d}: {n}" for d, n in sorted(hist.items())[:8])
      + (" ..." if len(hist) > 8 else ""))
    p(f"   strictly increasing: {all(d > 0 for d in incs)}   "
      f"duplicates: {sum(1 for d in incs if d == 0)}   "
      f"reversals: {sum(1 for d in incs if d < 0)}")
    skipped = sum(d - 1 for d in incs if d > 1)
    p(f"   SKIPS             {sum(1 for d in incs if d > 1)} gaps, "
      f"{skipped} ticks missing ({100.0*skipped/max(1,max(seqs)-min(seqs)):.2f} % "
      f"of the bridge's own ticks)")
    p("   -> seq counts BRIDGE TICKS and is shared by every channel; it is not "
      "a per-stream")
    p("      counter and it restarts at 1 when the bridge restarts. A consumer "
      "must generate")
    p("      its own seq for franka-node.")

    # --- 5. freshness and engagement ---------------------------------------
    p("\n5. FRESHNESS AND ENGAGEMENT")
    for name in ("engaged", "fresh", "controller_on"):
        on = sum(1 for r in good if r[name])
        p(f"   {name:14s}    {on}/{len(good)} messages set "
          f"({100.0*on/len(good):.1f} %)")
    eng = episodes(good, lambda r: r["engaged"])
    stale = episodes(good, lambda r: not r["fresh"])
    p(f"   engage episodes   {len(eng)}: durations "
      f"{dist([(b-a)/1e9 for a, b, _ in eng], 's')}")
    p(f"   not-fresh episodes {len(stale)}: durations "
      f"{dist([(b-a)/1e9 for a, b, _ in stale], 'ms', 1000.0, '{:8.1f}')}")
    if len(stale) > 1:
        gaps = [(stale[i+1][0] - stale[i][0]) / 1e9 for i in range(len(stale)-1)]
        p(f"   interval between not-fresh episodes "
          f"{dist(gaps, 's')}")
        near5 = sum(1 for g in gaps if 4.5 <= g <= 5.6)
        p(f"      {near5}/{len(gaps)} of them fall in 4.5..5.6 s -- the "
          f"KEEPALIVE_PERIOD_S = 5.0 signature")
        if near5 > len(gaps) / 2:
            p("      *** UNHEALTHY: this is the keepalive pathology, the clutch "
              "force-opening every 5 s")
        else:
            p("      (a healthy session shows no such cadence)")
    trans = []
    for a, b in zip(good, good[1:]):
        for name in ("engaged", "fresh", "controller_on"):
            if a[name] != b[name]:
                trans.append((b["t"], name, b[name]))
    p(f"   flag transitions  {len(trans)} total; first 12:")
    for t, name, val in trans[:12]:
        p(f"      +{(t-good[0]['t'])/1e9:7.3f} s  {name:14s} -> {int(val)}")
    # re-arm latency: end of a not-fresh episode to the next engaged=1
    lat = []
    for _, end, _ in stale:
        nxt = next((r["t"] for r in good if r["t"] > end and r["engaged"]), None)
        if nxt is not None:
            lat.append((nxt - end) / 1e9)
    if lat:
        p(f"   re-arm latency (not-fresh ends -> engaged again) "
          f"{dist(lat, 's')}")

    # --- 6. gripper ---------------------------------------------------------
    grips = [r["gripper"] for r in good]
    p("\n6. GRIPPER (a closed FRACTION, 0 = open, 1 = closed -- the index "
      "trigger)")
    p(f"   value             {dist(grips, '(fraction)')}")
    p(f"   distinct values   {len(set(grips))}; "
      f"out of [0,1]: {sum(1 for g in grips if g < 0.0 or g > 1.0)}")
    ge = [r["gripper"] for r in good if r["engaged"]]
    gd = [r["gripper"] for r in good if not r["engaged"]]
    p(f"   while engaged     {dist(ge, '(fraction)') if ge else '(never engaged)'}")
    p(f"   while disengaged  {dist(gd, '(fraction)') if gd else '(always engaged)'}")
    snap = sum(1 for r in good if not r["fresh"] and r["gripper"] == 0.0)
    nf = sum(1 for r in good if not r["fresh"])
    p(f"   snapped to 0.0 on a not-fresh message: {snap}/{nf}")
    p("   -> franka-node's GripperMsg is METRES: width = (1 - fraction) * "
      "max_width, and")
    p("      max_width is 0 until franka/<arm>/gripper/state publishes it. A "
      "consumer must")
    p("      hold the last engaged fraction through a gap or it opens the hand "
      "mid-grasp.")
    if any(r.get("btn_trigger") for r in good):
        p("   buttons bit 4 (trigger) is set: this bridge fills the button bits")

    # --- 7. rotation --------------------------------------------------------
    rots = [(quat_angle(a["quat"], b["quat"]), quat_angle_naive(a["quat"], b["quat"]),
             (b["t"] - a["t"]) / 1e9, a["engaged"] and b["engaged"])
            for a, b in zip(good, good[1:])]
    can = [r for r, _, _, _ in rots]
    naive = [n for _, n, _, _ in rots]
    can_e = [r for r, _, _, e in rots if e]
    p("\n7. ROTATION BETWEEN CONSECUTIVE ORIENTATIONS "
      f"(max_step_rotation = {MAX_STEP_ROT_RAD} rad)")
    p(f"   step, canonical   {dist(can, 'mrad', 1000.0, '{:8.2f}')}  "
      f"= {dist([math.degrees(v) for v in can], 'deg')}")
    p(f"   engaged only      {dist(can_e, 'mrad', 1000.0, '{:8.2f}')}")
    p(f"   step, NOT sign-canonicalised: p99 {pct(naive,99):.4f} rad, "
      f"max {max(naive):.4f} rad")
    p(f"      -> {sum(1 for v in naive if v > MAX_STEP_ROT_RAD)} messages would "
      f"be refused for RotationStep by a consumer that trusts the sign, "
      f"{sum(1 for v in can if v > MAX_STEP_ROT_RAD)} by one that does not")
    wr = [r / dt for r, _, dt, _ in rots if dt > 0]
    wr_e = [r / dt for r, _, dt, e in rots if dt > 0 and e]
    p(f"   implied rate      {dist(wr, 'rad/s')}")
    p(f"   engaged only      {dist(wr_e, 'rad/s')}")
    p(f"   the largest canonical steps sit at the CLUTCH EDGES: on release the "
      f"mapper re-latches")
    p(f"      vr_to_global = inv(pose), so the published orientation snaps back "
      f"to the constant.")
    p(f"      Only the engaged->engaged figure above is what a consumer would "
      f"ever publish.")
    p(f"   vs budget         norm {BUDGET_W} rad/s, per axis "
      f"{BUDGET_W/SQRT3:.3f} rad/s = {math.degrees(BUDGET_W/SQRT3):.1f} deg/s: "
      f"{sum(1 for v in wr if v > BUDGET_W)} messages over the norm "
      f"({100.0*sum(1 for v in wr if v > BUDGET_W)/max(1,len(wr)):.2f} %)")

    # --- 8. jumps -----------------------------------------------------------
    p("\n8. JUMPS (the existing guard: 0.5 m inside 0.2 s, engaged->engaged)")
    worst, worst_t = 0.0, None
    for i, a in enumerate(good):
        for b in good[i+1:]:
            if (b["t"] - a["t"]) / 1e9 > 0.2:
                break
            d = math.dist(a["pos"], b["pos"])
            if d > worst:
                worst, worst_t = d, (a["t"] - good[0]["t"]) / 1e9
    p(f"   largest move inside any 0.2 s window: {worst:.4f} m"
      + (f" at +{worst_t:.2f} s" if worst_t is not None else ""))
    p(f"   over the 0.5 m jump threshold: {'YES' if worst > 0.5 else 'no'}")

    # --- 9. derived: the lead ----------------------------------------------
    p("\n9. DERIVED -- how far the arm would fall behind (no jerk term, so a "
      "LOWER bound)")
    for label, vmax, amax in (
            ("default budget 0.3 m/s, 0.5 m/s^2", BUDGET_V / SQRT3, BUDGET_A / SQRT3),
            ("raised accel   0.3 m/s, 1.5 m/s^2", BUDGET_V / SQRT3, 1.5 / SQRT3)):
        peaks = []
        for a, b, _ in eng:
            seg = [(r["t"], r["pos"]) for r in good if a <= r["t"] <= b]
            if len(seg) > 2:
                peaks.append(track(seg, vmax, amax))
        if peaks:
            p(f"   {label}: peak lead per engage episode "
              f"{dist(peaks, 'mm', 1000.0, '{:8.1f}')}")
        else:
            p(f"   {label}: no engage episode long enough")
    p("   -> the coast on release is at least this lead; franka-rs needs "
      "0.054 m / 0.625 s")
    p("      just to brake from the per-axis cap at the default budget.")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("recording")
    ap.add_argument("--endpoint", action="append", default=None,
                    help="only these endpoints")
    args = ap.parse_args(argv)
    out = sys.stdout
    header, footer, rows, truncated = load(args.recording)
    print(f"{'#'*78}\n# {args.recording}\n{'#'*78}")
    if header:
        print(f"recorded   {header.get('t0_utc')}  note={header.get('note')!r}")
        print(f"endpoints  {header.get('endpoints')}  conflate="
              f"{header.get('conflate')}  struct={header.get('struct')}  "
              f"msg_size={header.get('msg_size')} bytes")
    if footer:
        print("footer     present: " + json.dumps(
            {k: footer.get(k) for k in
             ("total", "bad", "elapsed_s", "stopped_by")}))
    else:
        print("footer     MISSING -- the recorder was killed; the analysis "
              "below is still valid")
    if truncated:
        print(f"truncated  {truncated} unparseable line(s) skipped")
    print(f"messages   {len(rows)}")
    if not rows:
        return 1
    eps = args.endpoint or sorted({r.get("ep", "?") for r in rows})
    for ep in eps:
        report_endpoint(ep, [r for r in rows if r.get("ep") == ep], out)
    if len(eps) > 1:
        print(f"\n{'='*78}\nCROSS-CHANNEL\n{'='*78}")
        sets = {ep: {r["seq"] for r in rows
                     if r.get("ep") == ep and "bad" not in r} for ep in eps}
        a, b = eps[0], eps[1]
        both = sets[a] & sets[b]
        print(f"   seq values seen on both {a} and {b}: {len(both)} "
              f"(of {len(sets[a])} and {len(sets[b])})")
        print("   -> a shared value confirms seq identifies a BRIDGE TICK, not "
              "a channel.")
        for ep in eps:
            st_ep = episodes([r for r in rows if r.get("ep") == ep and "bad" not in r],
                             lambda r: not r["fresh"])
            print(f"   {ep}: {len(st_ep)} not-fresh episodes")
        print("   -> episodes starting at the same instant on both channels are "
              "the whole-frame")
        print("      gap that force-opens BOTH clutches at once (the dual-arm "
              "latch incident).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
