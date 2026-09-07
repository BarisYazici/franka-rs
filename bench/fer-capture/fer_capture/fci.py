"""FCI v5 protocol decoding: flow detection, gap/drift analysis and classification.

Wire layout used here comes from
`crates/franka-rs/src/wire/robot/v5/rbk_types.rs` (little endian, `#[repr(C, packed)]`):

  RobotState   (2373 B payload, rbk_types.rs:123-125)
      offset 0: message_id : u64 LE
      FCI v5 has **no separate `time` field** on the wire.  `message_id` *is* the
      robot's millisecond clock: it is a monotonic 1 kHz cycle counter and the
      client turns it straight into `RobotState::time` with
      `Duration::from_millis(w.message_id.get())` (robot_state.rs:410).
      So "robot time" below == message_id milliseconds.

  RobotCommand ( 370 B payload, rbk_types.rs:231-233)
      offset 0: message_id : u64 LE   (the id of the state it answers)

Why the capture can tell the two sides apart: tcpdump timestamps a packet when the
kernel (or, with `-j adapter_unsynced`, the NIC) sees it, *before* the client's
`recvmsg`. A late client therefore cannot create a gap in the state arrival times -
it can only create a gap in the commands it sends back.
"""

import bisect
import math
import struct

from .pcap import read_packets
from .reassembly import Reassembler, strip_link
from .stats import NS_PER_MS, NS_PER_US, median, stats_us

# ---------------------------------------------------------------------------
# wire offsets (see module docstring)
# ---------------------------------------------------------------------------
STATE_MESSAGE_ID_OFF = 0        # u64 LE, rbk_types.rs:125
STATE_ROBOT_TIME_OFF = 0        # same field: message_id == robot time in ms
COMMAND_MESSAGE_ID_OFF = 0      # u64 LE, rbk_types.rs:233
V5_STATE_LEN = 2373
V5_COMMAND_LEN = 370


# ---------------------------------------------------------------------------
# capture clock rate
# ---------------------------------------------------------------------------

def fit_clock_scale(xs, ys):
    """Fit capture-clock ticks against the robot's own millisecond clock.

    A NIC's free-running PHY clock (``tcpdump -j adapter_unsynced``) is not
    disciplined to anything: on this box's i219 it runs about 1.6x fast, which
    turns every normal 1 ms cadence into a 1.6 ms "gap" and makes every threshold
    in this tool meaningless.  The robot's message_id *is* a millisecond clock
    (see the module docstring), so the ratio between the two is measurable
    directly from the capture: fit arrival time against message_id and divide the
    timestamps by the slope.

    Robust in two steps, because real gaps and bursts are outliers that would
    drag a plain least-squares fit: a Theil-Sen style median slope over pairs
    half the capture apart first, then least squares over the 99 % of samples
    closest to that line.

    `xs` are robot milliseconds, `ys` the matching capture timestamps in
    nanoseconds. Returns (scale, info) where `scale` is capture seconds per real
    second.
    """
    n = len(xs)
    info = {"mode": "auto", "n": n, "slope_ns_per_ms": None, "rms_us": None,
            "window": None, "note": None}
    if n < 20:
        info["note"] = "only %d states: too few to fit, using 1.0" % n
        return 1.0, info
    # Theil-Sen style median slope over pairs one window apart. The window is a
    # 20th of the capture, not the whole capture: a stall shifts every later
    # arrival permanently, so a fit across the full span (or a plain
    # least-squares fit) would fold the robot's own lateness into the clock
    # ratio. Only the ~5 % of pairs that straddle a stall are affected, and the
    # median ignores them.
    k = max(1, n // 20)
    slopes = sorted((ys[i + k] - ys[i]) / (xs[i + k] - xs[i])
                    for i in range(n - k) if xs[i + k] != xs[i])
    if not slopes:
        info["note"] = "the robot clock does not advance, using 1.0"
        return 1.0, info
    a = slopes[len(slopes) // 2]
    if a <= 0:
        info["note"] = "fitted a non-positive slope, using 1.0"
        return 1.0, info
    b = median([y - a * x for x, y in zip(xs, ys)])
    # Residual spread around that line, over the closest 99 % of samples: it is
    # the fit quality *and* an upper bound on how much of the capture is real
    # sender slip rather than clock rate.
    resid = sorted((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    keep = resid[:max(1, int(0.99 * len(resid)))]
    rms_ns = math.sqrt(sum(keep) / len(keep))
    scale = a / 1e6                       # ns of capture clock per real ns
    info["slope_ns_per_ms"] = a
    info["kept"] = len(keep)
    info["window"] = k
    info["rms_us"] = (rms_ns / scale) / 1000.0
    return scale, info


def apply_clock_scale(states, cmds, scale, cmd_scale=None):
    """Divide every timestamp by its direction's scale, anchored on the first packet.

    The two directions can sit on different clocks: `tcpdump -j adapter_unsynced`
    only asks for *receive* hardware timestamps, so on an i219 the robot->host
    states carry the NIC's free-running clock while the host->robot commands are
    still stamped by the kernel. Each direction therefore gets its own fitted
    rate, and both are anchored on the same origin.
    """
    if not states and not cmds:
        return
    if cmd_scale is None:
        cmd_scale = scale
    t0 = min([s.t_first for s in states] + [c.t for c in cmds])
    for s in states:
        s.t_first = t0 + round((s.t_first - t0) / scale)
        s.t_last = t0 + round((s.t_last - t0) / scale)
    for c in cmds:
        c.t = t0 + round((c.t - t0) / cmd_scale)


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------

class StateRec:
    __slots__ = ("t_first", "t_last", "mid", "rt_ms", "plen", "frags")

    def __init__(self, t_first, t_last, mid, rt_ms, plen, frags):
        self.t_first, self.t_last = t_first, t_last
        self.mid, self.rt_ms = mid, rt_ms
        self.plen, self.frags = plen, frags


class CmdRec:
    __slots__ = ("t", "mid", "plen")

    def __init__(self, t, mid, plen):
        self.t, self.mid, self.plen = t, mid, plen


def u64le(buf, off):
    if len(buf) < off + 8:
        return None
    return struct.unpack_from("<Q", buf, off)[0]


def pick_flow(datagrams, robot_ip):
    """Pick the FCI flow: the robot's (ip, port) and the client's (ip, port).

    The robot is the endpoint sending the biggest datagrams; the client is the
    endpoint it sends the most of them to.  Everything else in the capture (DNS,
    another arm, a second client) is then simply not part of this flow - which
    matters, because one stray datagram parsed as a state would wreck the
    message_id range.  On a loopback capture the ports are the only thing that
    tells the two directions apart at all.
    """
    by_ep = {}
    for d in datagrams:
        if robot_ip is not None and d.src != robot_ip:
            continue
        by_ep.setdefault((d.src, d.sport), []).append(len(d.payload))
    if not by_ep:
        return None, None
    robot_ep = max(by_ep, key=lambda ep: (median(by_ep[ep]), len(by_ep[ep])))
    peers = {}
    for d in datagrams:
        if (d.src, d.sport) == robot_ep:
            peers[(d.dst, d.dport)] = peers.get((d.dst, d.dport), 0) + 1
    host_ep = max(peers, key=peers.get) if peers else None
    return robot_ep, host_ep


def analyze(path, robot_ip=None, gap_ms=1.5, burst_ms=3.0, match_ms=2.0,
            jump_ms=None, max_rows=200, clock_scale=None):
    gap_ns = int(gap_ms * NS_PER_MS)
    burst_ns = int(burst_ms * NS_PER_MS)
    match_ns = int(match_ms * NS_PER_MS)
    jump_ns = int((jump_ms if jump_ms is not None else gap_ms) * NS_PER_MS)

    asm = Reassembler()
    datagrams = []
    n_packets = n_ip = n_trunc = 0
    linktypes = set()
    for ts, linktype, data, _orig in read_packets(path):
        n_packets += 1
        linktypes.add(linktype)
        ip = strip_link(linktype, data)
        if ip is None:
            continue
        n_ip += 1
        d = asm.add(ts, ip)
        if d is not None:
            if d.truncated:
                n_trunc += 1
            datagrams.append(d)
    asm.flush()
    datagrams.sort(key=lambda d: (d.t_first, d.t_last))

    robot_ep, host_ep = pick_flow(datagrams, robot_ip)
    if robot_ep is not None:
        robot_ip, robot_port = robot_ep
    else:
        robot_port = None
    host_ip, host_port = host_ep if host_ep else (None, None)

    states, cmds, other = [], [], 0
    for d in datagrams:
        src_ep, dst_ep = (d.src, d.sport), (d.dst, d.dport)
        if src_ep == robot_ep and dst_ep == host_ep:
            mid = u64le(d.payload, STATE_MESSAGE_ID_OFF)
            if mid is None:
                other += 1
                continue
            rt = u64le(d.payload, STATE_ROBOT_TIME_OFF)   # v5: same field
            states.append(StateRec(d.t_first, d.t_last, mid, rt, len(d.payload),
                                   d.frags))
        elif dst_ep == robot_ep and d.src == host_ip:
            # the client's source port is normally host_ep's, but do not insist
            mid = u64le(d.payload, COMMAND_MESSAGE_ID_OFF)
            if mid is None:
                other += 1
                continue
            cmds.append(CmdRec(d.t_first, mid, len(d.payload)))
        else:
            other += 1

    # The capture clock may not run at one second per second (see
    # fit_clock_scale); correct every timestamp before any statistic is taken.
    scale_info = {"mode": "none"}
    scale = cmd_scale = 1.0
    if clock_scale in ("auto", "AUTO"):
        scale, scale_info = fit_clock_scale([float(s.rt_ms) for s in states],
                                            [float(s.t_first) for s in states])
        cmd_scale, cmd_info = fit_clock_scale([float(c.mid) for c in cmds],
                                              [float(c.t) for c in cmds])
        scale_info["cmd_scale"] = cmd_scale
        scale_info["cmd_rms_us"] = cmd_info.get("rms_us")
        # The two directions are only on the same clock if the fits agree.
        scale_info["split_clocks"] = abs(cmd_scale - scale) > 0.001 * scale
    elif clock_scale is not None:
        scale = cmd_scale = float(clock_scale)
        scale_info = {"mode": "fixed", "slope_ns_per_ms": scale * 1e6,
                      "rms_us": None, "note": None, "n": len(states),
                      "cmd_scale": cmd_scale, "split_clocks": False}
    if scale != 1.0 or cmd_scale != 1.0:
        apply_clock_scale(states, cmds, scale, cmd_scale)
    if scale_info.get("split_clocks"):
        # Two clocks also means an unknown constant offset between them, which
        # would make every command gap look like it misses every state gap. The
        # client answers each state within a few hundred microseconds, so line the
        # timelines up on the median turnaround; that is accurate to well inside
        # the +/- match window, and a constant shift changes no interval or gap.
        by_id = {st.mid: st.t_first for st in states}
        deltas = [c.t - by_id[c.mid] for c in cmds if c.mid in by_id]
        if deltas:
            shift = int(median(deltas))
            for c in cmds:
                c.t -= shift
            scale_info["cmd_align_us"] = shift / NS_PER_US

    res = {
        "clock_scale": scale, "clock_info": scale_info,
        "path": path, "robot_ip": robot_ip, "host_ip": host_ip,
        "robot_port": robot_port, "host_port": host_port,
        "gap_ms": gap_ms, "burst_ms": burst_ms, "match_ms": match_ms,
        "jump_ms": jump_ns / NS_PER_MS, "max_rows": max_rows,
        "n_packets": n_packets, "n_ip": n_ip, "n_datagrams": len(datagrams),
        "n_truncated": n_trunc, "n_incomplete": asm.incomplete,
        "n_other": other, "linktypes": sorted(linktypes),
        "states": states, "cmds": cmds,
        "state_gaps": [], "cmd_gaps": [], "drift_jumps": [],
    }
    if not states:
        res["classification"] = "NO_DATA"
        res["classification_detail"] = "NO_DATA"
        res["verdict"] = ("No robot->host state datagrams were found in %s - "
                          "check --robot-ip and the capture filter." % path)
        return res

    t0 = min(s.t_first for s in states)
    res["t0"] = t0
    res["duration_s"] = (max(s.t_last for s in states) - t0) / 1e9

    # ---- payload / id bookkeeping ----------------------------------------
    plens = {}
    for s in states:
        plens[s.plen] = plens.get(s.plen, 0) + 1
    res["state_payload_lens"] = sorted(plens.items(), key=lambda kv: -kv[1])
    res["state_frag_counts"] = sorted(
        {s.frags for s in states})

    ids = sorted(s.mid for s in states)
    res["id_min"], res["id_max"] = ids[0], ids[-1]
    res["expected"] = ids[-1] - ids[0] + 1
    res["duplicates"] = len(ids) - len(set(ids))
    # Walk the sorted unique ids, never the whole numeric range: one stray datagram
    # with a garbage id would otherwise mean an unbounded loop.
    uniq = sorted(set(ids))
    missing = []
    max_run = 0
    for a, b in zip(uniq, uniq[1:]):
        if b > a + 1:
            missing.append((a + 1, b - 1))
            max_run = max(max_run, b - a - 1)
    res["missing_ranges"] = missing
    res["n_missing"] = res["expected"] - len(uniq)
    res["max_consecutive_missing"] = max_run

    # ---- inter-arrival statistics ----------------------------------------
    d_first = [states[i].t_first - states[i - 1].t_first for i in range(1, len(states))]
    d_last = [states[i].t_last - states[i - 1].t_last for i in range(1, len(states))]
    frag_delay = [s.t_last - s.t_first for s in states]
    res["ia_first"] = stats_us(d_first)
    res["ia_last"] = stats_us(d_last)
    res["frag_delay"] = stats_us(frag_delay)
    res["frag_delay_zero"] = sum(1 for d in frag_delay if d == 0)
    nominal = median(d_first) if d_first else NS_PER_MS
    res["nominal_ns"] = nominal
    expected_in_burst = max(1, int(round(burst_ns / nominal))) if nominal else 1
    res["expected_in_burst"] = expected_in_burst

    # ---- state gap table --------------------------------------------------
    ft = [s.t_first for s in states]
    for i in range(1, len(states)):
        gap = ft[i] - ft[i - 1]
        if gap < gap_ns:
            continue
        after = ft[i]
        j = i + 1
        burst = 0
        while j < len(states) and ft[j] - after <= burst_ns:
            burst += 1
            j += 1
        a, b = states[i - 1], states[i]
        res["state_gaps"].append({
            "t_off_s": (b.t_first - t0) / 1e9,
            "gap_us": gap / NS_PER_US,
            "gap_last_us": (b.t_last - a.t_last) / NS_PER_US,
            "id_before": a.mid, "id_after": b.mid,
            "consecutive": (b.mid == a.mid + 1),
            "robot_dt_ms": (b.rt_ms - a.rt_ms),
            "burst": burst, "is_burst": burst > expected_in_burst,
            "t_before": a.t_first, "t_after": b.t_first,
        })

    # ---- robot-time vs arrival drift -------------------------------------
    # offset = robot clock (ms) - capture arrival (ms), zeroed on the first state.
    base = states[0].rt_ms * NS_PER_MS - states[0].t_first
    off_ns = [s.rt_ms * NS_PER_MS - s.t_first - base for s in states]
    res["drift_min_ms"] = min(off_ns) / NS_PER_MS
    res["drift_max_ms"] = max(off_ns) / NS_PER_MS
    res["drift_first_ms"] = off_ns[0] / NS_PER_MS
    res["drift_last_ms"] = off_ns[-1] / NS_PER_MS
    res["drift_offsets"] = off_ns
    for i in range(1, len(off_ns)):
        d = off_ns[i] - off_ns[i - 1]
        if abs(d) >= jump_ns:
            res["drift_jumps"].append({
                "t_off_s": (states[i].t_first - t0) / 1e9,
                "delta_ms": d / NS_PER_MS,
                "id_before": states[i - 1].mid, "id_after": states[i].mid,
                "consecutive": states[i].mid == states[i - 1].mid + 1,
            })

    # ---- command direction ------------------------------------------------
    if cmds:
        cd = [cmds[i].t - cmds[i - 1].t for i in range(1, len(cmds))]
        res["cmd_ia"] = stats_us(cd)
        cids = [c.mid for c in cmds]
        res["cmd_id_min"], res["cmd_id_max"] = min(cids), max(cids)
        res["cmd_missing"] = (max(cids) - min(cids) + 1) - len(set(cids))
        # state gaps are already in time order; bisect keeps the coincidence test
        # O(log n) per command gap instead of scanning every state gap.
        sg_after = [g["t_after"] for g in res["state_gaps"]]
        for i in range(1, len(cmds)):
            gap = cmds[i].t - cmds[i - 1].t
            if gap < gap_ns:
                continue
            lo, hi = cmds[i - 1].t, cmds[i].t
            match = None
            k = bisect.bisect_left(sg_after, lo - match_ns)
            if k < len(sg_after) and res["state_gaps"][k]["t_before"] <= hi + match_ns:
                match = res["state_gaps"][k]
            res["cmd_gaps"].append({
                "t_off_s": (cmds[i].t - t0) / 1e9,
                "gap_us": gap / NS_PER_US,
                "id_before": cmds[i - 1].mid, "id_after": cmds[i].mid,
                "matched": match is not None,
                "matched_gap_us": match["gap_us"] if match else 0.0,
            })
        res["cmd_gaps_unmatched"] = sum(1 for g in res["cmd_gaps"] if not g["matched"])
    else:
        res["cmd_ia"] = None
        res["cmd_gaps_unmatched"] = 0

    classify(res)
    return res


def classify(res):
    sg = res["state_gaps"]
    cg = res["cmd_gaps"]
    unmatched = res["cmd_gaps_unmatched"]
    sg_consecutive = [g for g in sg if g["consecutive"]]
    sg_lost = [g for g in sg if not g["consecutive"]]
    sg_burst = [g for g in sg if g["is_burst"]]

    if sg and unmatched:
        cls = "MIXED"
    elif sg:
        cls = "ROBOT_OR_WIRE_STALL"
    elif unmatched:
        cls = "HOST_STALL"
    else:
        cls = "NO_STALL"
    res["classification"] = cls

    # One host-only gap in a capture full of transit gaps is not the same finding
    # as a fifty-fifty split, so the three-way verdict always carries the counts
    # and which way they lean instead of a flat "MIXED".
    transit, host_only = len(sg), unmatched
    res["transit_gaps"], res["host_only_gaps"] = transit, host_only
    if transit >= 4 * max(1, host_only):
        lean = "predominantly ROBOT_OR_WIRE"
    elif host_only >= 4 * max(1, transit):
        lean = "predominantly HOST"
    else:
        lean = "no clear majority"
    res["lean"] = lean
    counts = "%d transit gap%s vs %d host-only gap%s" % (
        transit, "" if transit == 1 else "s",
        host_only, "" if host_only == 1 else "s")
    res["gap_counts"] = counts
    if cls == "MIXED":
        detail = "MIXED (%s: %s)" % (counts, lean)
    elif cls == "NO_STALL":
        detail = "NO_STALL (no gap >= %.2f ms in either direction)" % res["gap_ms"]
    else:
        detail = "%s (%s)" % (cls, counts)
    res["classification_detail"] = detail

    parts = []
    if res["clock_scale"] != 1.0:
        parts.append(
            "All timestamps were divided by %.6f before any of these numbers were "
            "taken (the capture clock runs %.2f%% %s than the robot's)."
            % (res["clock_scale"], abs(res["clock_scale"] - 1.0) * 100,
               "fast" if res["clock_scale"] > 1 else "slow"))
    parts.append(
        "%d of %d state datagrams arrived more than %.2f ms after the previous one; "
        "%d of those gaps carried the very next message_id (the robot's own clock "
        "advanced by only %s while the wire stayed silent), %d skipped at least one "
        "id, and %d were followed by a burst of more than %d datagrams within %.1f ms."
        % (len(sg), len(res["states"]), res["gap_ms"], len(sg_consecutive),
           ", ".join("%g" % g["robot_dt_ms"] for g in sg_consecutive[:5]) + " ms"
           if sg_consecutive else "n/a",
           len(sg_lost), len(sg_burst), res["expected_in_burst"], res["burst_ms"]))
    parts.append(
        "%d command gap(s) of the same size were seen; %d of them line up (+/- %.1f "
        "ms) with a state gap (the client was simply waiting for a state) and %d do "
        "not (the client itself was late)."
        % (len(cg), len(cg) - unmatched, res["match_ms"], unmatched))
    parts.append(
        "The robot-clock-minus-arrival offset moved over a range of %.3f ms with %d "
        "jump(s) of at least %.2f ms."
        % (res["drift_max_ms"] - res["drift_min_ms"], len(res["drift_jumps"]),
           res["jump_ms"]))
    if cls == "ROBOT_OR_WIRE_STALL":
        parts.append(
            "Verdict %s: every gap is in the arrival timestamps themselves, which "
            "tcpdump takes before the client ever calls recvmsg, so the packets were "
            "late on the wire or the robot emitted them late; a late client cannot "
            "produce this pattern." % detail)
    elif cls == "HOST_STALL":
        parts.append(
            "Verdict %s: the state stream kept its cadence at the capture point while "
            "the client stopped answering, so the loss is on this box (scheduling, "
            "page faults or a blocked control thread), not on the wire." % detail)
    elif cls == "MIXED":
        parts.append(
            "Verdict %s: %d gap(s) were already present in the arrival timestamps "
            "(robot or wire) and %d command gap(s) had no matching state gap (the "
            "client stalling on its own), a ratio of %s, so both sides contribute but "
            "%s."
            % (detail, transit, host_only,
               ("%.0f:1 towards transit" % (transit / host_only)) if host_only
               else "all transit",
               "the transit side accounts for almost all of it" if
               lean == "predominantly ROBOT_OR_WIRE" else
               ("the host side accounts for almost all of it" if
                lean == "predominantly HOST" else
                "neither side dominates")))
    else:
        parts.append(
            "Verdict %s: neither direction shows a gap of at least %.2f ms, so this "
            "capture contains no evidence of a stall at all." % (detail, res["gap_ms"]))
    res["verdict"] = " ".join(parts)
