"""Synthetic capture generation and the ``--self-test`` checks.

Builds synthetic pcap/pcapng captures of a 1 kHz FER stream with injected
pathologies (a sender pause, a drain burst, a lost id, a client stall, a fast
capture clock) entirely in memory, then runs ``analyze()``/``render()`` over
them and asserts the results. Self-contained: no external capture file needed.
"""

import os
import struct
import subprocess
import tempfile

from .fci import V5_COMMAND_LEN, V5_STATE_LEN, analyze
from .report import render

# ---------------------------------------------------------------------------
# synthetic capture generation (used by --self-test only)
# ---------------------------------------------------------------------------

MS = 1_000_000
US = 1_000


def _ip_checksum(hdr):
    if len(hdr) % 2:
        hdr += b"\x00"
    s = 0
    for i in range(0, len(hdr), 2):
        s += (hdr[i] << 8) | hdr[i + 1]
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def _ipv4(src, dst, ident, frag_off, mf, payload):
    total = 20 + len(payload)
    flags = (0x2000 if mf else 0) | (frag_off // 8)
    hdr = struct.pack("!BBHHHBBH4s4s", 0x45, 0, total, ident, flags, 64, 17, 0,
                      src, dst)
    ck = _ip_checksum(hdr)
    hdr = hdr[:10] + struct.pack("!H", ck) + hdr[12:]
    return hdr + payload


def _eth(ip):
    return (b"\x00\x11\x22\x33\x44\x55" + b"\x66\x77\x88\x99\xaa\xbb" +
            b"\x08\x00" + ip)


def _null_loopback(ip):
    return struct.pack("<I", 2) + ip


def _udp(sport, dport, payload):
    return struct.pack("!HHHH", sport, dport, 8 + len(payload), 0) + payload


def _frames(ts, src, dst, sport, dport, payload, ident, fragment, frag_delay,
            linktype):
    """Return [(ts, frame)] for one UDP datagram, fragmented at 1480 B if asked."""
    udp = _udp(sport, dport, payload)
    wrap = _eth if linktype == 1 else _null_loopback
    if not fragment or len(udp) <= 1480:
        return [(ts, wrap(_ipv4(src, dst, ident, 0, False, udp)))]
    out = []
    off = 0
    n = 0
    while off < len(udp):
        chunk = udp[off:off + 1480]
        mf = off + len(chunk) < len(udp)
        out.append((ts + n * frag_delay,
                    wrap(_ipv4(src, dst, ident, off, mf, chunk))))
        off += len(chunk)
        n += 1
    return out


def gen_stream(n=1200, sender_pause=True, burst=True, missing=True,
               host_stall=True, fragment=True, linktype=1,
               robot=b"\xc0\x00\x02\x0a", host=b"\xc0\x00\x02\x01"):
    """Build a synthetic 1 kHz FER stream with the injected pathologies."""
    deltas = [MS] * n
    deltas[0] = 0
    if sender_pause:
        deltas[300] = 7 * MS                       # (a) sender pause, ids consecutive
    if burst:
        deltas[600] = 4 * MS                       # (b) delay ...
        deltas[601] = deltas[602] = deltas[603] = 100 * US   # ... then a burst
    skip_states = set()
    if missing:
        skip_states.add(900)                       # (c) one id never sent
        deltas[901] = 2 * MS
    skip_cmds = set(range(450, 455)) if host_stall else set()   # (d) client stall

    frames = []
    t = 0
    base = 1_700_000_000 * 1_000_000_000 + 500 * MS
    first = True
    frag_delay = 12 * US
    for i in range(n):
        if i in skip_states:
            continue
        t = t if first else t + deltas[i]
        first = False
        mid = 1000 + i
        state = struct.pack("<Q", mid) + bytes(V5_STATE_LEN - 8)
        frames += _frames(base + t, robot, host, 1338, 40000, state,
                          0x4000 + (i & 0x3FFF), fragment, frag_delay, linktype)
        if i not in skip_cmds:
            cmd = struct.pack("<Q", mid) + bytes(V5_COMMAND_LEN - 8)
            ct = base + t + frag_delay + 200 * US
            frames += _frames(ct, host, robot, 40000, 1338, cmd,
                              0x8000 + (i & 0x3FFF), fragment, frag_delay, linktype)
    frames.sort(key=lambda x: x[0])
    return frames


def write_pcap(path, frames, endian="<", nano=False, linktype=1):
    magic = {("<", False): b"\xd4\xc3\xb2\xa1", (">", False): b"\xa1\xb2\xc3\xd4",
             ("<", True): b"\x4d\x3c\xb2\xa1", (">", True): b"\xa1\xb2\x3c\x4d"}[
        (endian, nano)]
    with open(path, "wb") as f:
        f.write(magic)
        f.write(struct.pack(endian + "HHiIII", 2, 4, 0, 0, 262144, linktype))
        for ts, frame in frames:
            sec, ns = divmod(ts, 1_000_000_000)
            frac = ns if nano else ns // 1000
            f.write(struct.pack(endian + "IIII", sec, frac, len(frame), len(frame)))
            f.write(frame)


def write_pcapng(path, frames, linktype=1, tsresol=9):
    def block(btype, body):
        pad = (4 - len(body) % 4) % 4
        body = body + b"\x00" * pad
        total = 12 + len(body)
        return (struct.pack("<II", btype, total) + body +
                struct.pack("<I", total))

    with open(path, "wb") as f:
        shb_body = struct.pack("<IHHq", 0x1A2B3C4D, 1, 0, -1)
        f.write(block(0x0A0D0D0A, shb_body))
        idb_body = (struct.pack("<HHI", linktype, 0, 262144) +
                    struct.pack("<HH", 9, 1) + bytes([tsresol]) + b"\x00" * 3 +
                    struct.pack("<HH", 0, 0))
        f.write(block(0x01, idb_body))
        div = 10 ** (9 - tsresol)
        for ts, frame in frames:
            ticks = ts // div
            body = (struct.pack("<IIIII", 0, ticks >> 32, ticks & 0xFFFFFFFF,
                                len(frame), len(frame)) + frame)
            pad = (4 - len(frame) % 4) % 4
            f.write(block(0x06, body + b"\x00" * pad))


def scale_frames(frames, factor):
    """Replay the same frames on a capture clock that runs `factor` times fast."""
    t0 = frames[0][0]
    return [(t0 + round((ts - t0) * factor), f) for ts, f in frames]


def split_clock_frames(frames, factor, offset_ns, robot=b"\xc0\x00\x02\x0a"):
    """Timestamp one direction on a fast, offset clock and the other as it is.

    This is what `-j adapter_unsynced` does on an i219: receive timestamps come
    from the NIC's free-running clock, transmit timestamps from the kernel's.
    """
    t0 = frames[0][0]
    out = []
    for ts, f in frames:
        if len(f) >= 30 and f[26:30] == robot:          # eth(14) + ip src at 12
            ts = t0 + offset_ns + round((ts - t0) * factor)
        out.append((ts, f))
    out.sort(key=lambda x: x[0])
    return out


# ---------------------------------------------------------------------------
# self test
# ---------------------------------------------------------------------------

class SelfTestError(AssertionError):
    pass


def _check(cond, msg):
    if not cond:
        raise SelfTestError(msg)
    print("    ok   %s" % msg)


def self_test(verbose=False):
    tmp = tempfile.mkdtemp(prefix="fer-capture-selftest-")
    failures = 0

    def section(name):
        print("\n[%s]" % name)

    # ---- scenario A: everything at once, four container formats -----------
    section("scenario A: sender pause + burst + lost id + client stall")
    frames = gen_stream()
    variants = [
        ("pcap-usec-le", lambda p: write_pcap(p, frames, "<", False)),
        ("pcap-nsec-le", lambda p: write_pcap(p, frames, "<", True)),
        ("pcap-usec-be", lambda p: write_pcap(p, frames, ">", False)),
        ("pcapng-nsec", lambda p: write_pcapng(p, frames)),
    ]
    renders = {}
    res = None
    for name, writer in variants:
        path = os.path.join(tmp, name + ".pcap")
        writer(path)
        r = analyze(path, robot_ip="192.0.2.10", gap_ms=1.5)
        renders[name] = render(r, show_path=False)
        if res is None:
            res = r
        print("    read %-14s %8d bytes, %d states, %d commands" %
              (name, os.path.getsize(path), len(r["states"]), len(r["cmds"])))
    base_name, base_render = list(renders.items())[0]
    for name, text in renders.items():
        _check(text == base_render,
               "%s decodes identically to %s" % (name, base_name))

    if verbose:
        print(base_render)

    _check(len(res["states"]) == 1199, "1199 state datagrams reassembled")
    _check(res["id_min"] == 1000 and res["id_max"] == 2199,
           "message_id range is 1000..2199")
    _check(res["expected"] == 1200, "expected count from the id range is 1200")
    _check(res["n_missing"] == 1 and res["max_consecutive_missing"] == 1,
           "exactly one missing id, never two in a row")
    _check(res["missing_ranges"] == [(1900, 1900)], "the missing id is 1900")
    _check(res["state_frag_counts"] == [2], "every state arrived as 2 IP fragments")
    _check(res["state_payload_lens"][0] == (V5_STATE_LEN, 1199),
           "every state payload is %d bytes" % V5_STATE_LEN)
    _check(abs(res["frag_delay"]["p50"] - 12.0) < 0.001,
           "first->last fragment delay is 12 us")
    _check(abs(res["ia_first"]["p50"] - 1000.0) < 0.001,
           "median state inter-arrival is 1000 us")
    _check(abs(res["ia_first"]["max"] - 7000.0) < 0.001,
           "worst state inter-arrival is 7000 us")

    g = res["state_gaps"]
    _check(len(g) == 3, "three state gaps >= 1.5 ms")
    _check(abs(g[0]["gap_us"] - 7000.0) < 0.001 and g[0]["consecutive"]
           and g[0]["robot_dt_ms"] == 1 and g[0]["id_before"] == 1299
           and g[0]["id_after"] == 1300,
           "(a) 7000 us gap, ids 1299->1300 consecutive, robot clock +1 ms")
    _check(g[0]["burst"] == 3 and not g[0]["is_burst"],
           "(a) is followed by normal 1 kHz cadence, not a burst")
    _check(abs(g[1]["gap_us"] - 4000.0) < 0.001 and g[1]["consecutive"]
           and g[1]["id_before"] == 1599 and g[1]["id_after"] == 1600,
           "(b) 4000 us gap, ids 1599->1600 consecutive")
    _check(g[1]["burst"] == 5 and g[1]["is_burst"],
           "(b) is followed by a 5-datagram burst within 3 ms")
    _check(abs(g[2]["gap_us"] - 2000.0) < 0.001 and not g[2]["consecutive"]
           and g[2]["id_before"] == 1899 and g[2]["id_after"] == 1901
           and g[2]["robot_dt_ms"] == 2,
           "(c) 2000 us gap skips id 1900, robot clock +2 ms")

    j = res["drift_jumps"]
    _check(len(j) == 2, "two robot-clock-vs-arrival jumps")
    _check(abs(j[0]["delta_ms"] + 6.0) < 0.001 and j[0]["consecutive"],
           "(a) drops the offset by 6 ms with consecutive ids (sender was late)")
    _check(abs(j[1]["delta_ms"] + 3.0) < 0.001 and j[1]["consecutive"],
           "(b) drops the offset by 3 ms with consecutive ids")
    _check(abs((res["drift_max_ms"] - res["drift_min_ms"]) - 9.0) < 0.001,
           "total drift range is 9 ms")

    c = res["cmd_gaps"]
    _check(len(res["cmds"]) == 1194, "1194 command datagrams (5 never sent)")
    _check(len(c) == 4, "four command gaps >= 1.5 ms")
    _check(res["cmd_gaps_unmatched"] == 1, "exactly one command gap is unmatched")
    u = [x for x in c if not x["matched"]][0]
    _check(abs(u["gap_us"] - 6000.0) < 0.001 and u["id_before"] == 1449
           and u["id_after"] == 1455,
           "(d) the unmatched command gap is 6000 us across ids 1449->1455")
    _check(all(x["matched"] for x in c if x is not u),
           "the other three command gaps line up with state gaps")
    _check(res["classification"] == "MIXED", "scenario A classifies as MIXED")
    _check(res["classification_detail"] ==
           "MIXED (3 transit gaps vs 1 host-only gap: no clear majority)",
           "the verdict carries the transit/host split: %s"
           % res["classification_detail"])

    # ---- scenario B: robot/wire only --------------------------------------
    section("scenario B: sender pause + burst only")
    p = os.path.join(tmp, "robot_only.pcap")
    write_pcap(p, gen_stream(missing=False, host_stall=False), "<", True)
    rb = analyze(p, robot_ip="192.0.2.10", gap_ms=1.5)
    _check(len(rb["states"]) == 1200, "no state is missing")
    _check(rb["n_missing"] == 0, "no missing message_id")
    _check(len(rb["state_gaps"]) == 2, "two state gaps")
    _check(all(x["consecutive"] for x in rb["state_gaps"]),
           "both state gaps have consecutive ids")
    _check(rb["cmd_gaps_unmatched"] == 0, "no unmatched command gap")
    _check(rb["classification"] == "ROBOT_OR_WIRE_STALL",
           "scenario B classifies as ROBOT_OR_WIRE_STALL")
    _check(rb["classification_detail"] ==
           "ROBOT_OR_WIRE_STALL (2 transit gaps vs 0 host-only gaps)",
           "and reports 2 transit / 0 host-only gaps")

    # ---- scenario C: host only --------------------------------------------
    section("scenario C: client stall only")
    p = os.path.join(tmp, "host_only.pcap")
    write_pcap(p, gen_stream(sender_pause=False, burst=False, missing=False), "<", True)
    rc = analyze(p, robot_ip="192.0.2.10", gap_ms=1.5)
    _check(len(rc["state_gaps"]) == 0, "the state stream keeps its cadence")
    _check(abs(rc["ia_first"]["max"] - 1000.0) < 0.001,
           "worst state inter-arrival is still 1000 us")
    _check(len(rc["cmd_gaps"]) == 1 and rc["cmd_gaps_unmatched"] == 1,
           "one command gap, unmatched")
    _check(rc["classification"] == "HOST_STALL",
           "scenario C classifies as HOST_STALL")

    # ---- scenario D: unfragmented datagrams on a DLT_NULL loopback --------
    section("scenario D: unfragmented states, DLT_NULL loopback link type")
    p = os.path.join(tmp, "loopback.pcap")
    fr = gen_stream(fragment=False, linktype=0, robot=b"\x7f\x00\x00\x01",
                    host=b"\x7f\x00\x00\x01")
    write_pcapng(p, fr, linktype=0)
    rd = analyze(p, robot_ip="127.0.0.1", gap_ms=1.5)
    _check(rd["state_frag_counts"] == [1], "datagrams arrive unfragmented")
    _check(rd["robot_port"] == 1338,
           "the robot side is told from the client side by its UDP source port")
    _check(len(rd["states"]) == 1199 and len(rd["cmds"]) == 1194,
           "states and commands are separated on a same-address capture")
    _check(len(rd["state_gaps"]) == 3 and rd["cmd_gaps_unmatched"] == 1,
           "the loopback capture yields the same gap table")
    _check(rd["classification"] == "MIXED", "scenario D classifies as MIXED")

    # ---- scenario E: auto-detection of the robot IP -----------------------
    section("scenario E: --robot-ip auto-detection")
    p = os.path.join(tmp, "auto.pcap")
    write_pcap(p, frames, "<", True)
    re_ = analyze(p, robot_ip=None, gap_ms=1.5)
    _check(re_["robot_ip"] == "192.0.2.10" and re_["host_ip"] == "192.0.2.1"
           and re_["robot_port"] == 1338 and re_["host_port"] == 40000,
           "the robot is picked as the side sending the big datagrams")
    _check(re_["classification"] == "MIXED", "auto-detected run still MIXED")

    # ---- scenario F: a NIC clock that runs fast ---------------------------
    section("scenario F: free-running capture clock, --clock-scale auto")
    p = os.path.join(tmp, "fastclock.pcap")
    write_pcap(p, scale_frames(frames, 1.6), "<", True)
    raw = analyze(p, robot_ip="192.0.2.10", gap_ms=1.5)
    _check(abs(raw["ia_first"]["p50"] - 1600.0) < 1.0,
           "uncorrected, the 1 kHz cadence reads as a 1600 us interval")
    _check(len(raw["state_gaps"]) > 1000,
           "uncorrected, nearly every interval looks like a gap (%d rows)"
           % len(raw["state_gaps"]))
    rf = analyze(p, robot_ip="192.0.2.10", gap_ms=1.5, clock_scale="auto")
    _check(abs(rf["clock_scale"] - 1.6) < 1e-4,
           "the fitted clock scale is %.6f (injected 1.6)" % rf["clock_scale"])
    _check(2000.0 < rf["clock_info"]["rms_us"] < 5000.0,
           "the fit residual RMS (%.0f us) reflects the 9 ms of injected slip, "
           "not the clock" % rf["clock_info"]["rms_us"])
    pc = os.path.join(tmp, "fastclock_clean.pcap")
    write_pcap(pc, scale_frames(gen_stream(sender_pause=False, burst=False,
                                           missing=False, host_stall=False), 1.6),
               "<", True)
    rfc = analyze(pc, robot_ip="192.0.2.10", gap_ms=1.5, clock_scale="auto")
    _check(abs(rfc["clock_scale"] - 1.6) < 1e-6 and rfc["clock_info"]["rms_us"] < 50.0,
           "on a clean stream the same fit gives %.6f with %.1f us RMS"
           % (rfc["clock_scale"], rfc["clock_info"]["rms_us"]))
    _check(rfc["classification"] == "NO_STALL" and not rfc["state_gaps"],
           "and leaves a clean 1 kHz stream with no gaps at all")
    _check(abs(rf["ia_first"]["p50"] - 1000.0) < 1.0,
           "after scaling the median inter-arrival is 1000 us again")
    _check(len(rf["state_gaps"]) == 3,
           "after scaling the gap table is back to three rows")
    gf = rf["state_gaps"]
    _check(abs(gf[0]["gap_us"] - 7000.0) < 1.0 and gf[0]["consecutive"]
           and abs(gf[1]["gap_us"] - 4000.0) < 1.0
           and abs(gf[2]["gap_us"] - 2000.0) < 1.0 and not gf[2]["consecutive"],
           "the scaled gaps are 7000/4000/2000 us again")
    _check(gf[1]["is_burst"] and not gf[0]["is_burst"],
           "the burst flag survives the scaling")
    _check(abs((rf["drift_max_ms"] - rf["drift_min_ms"]) - 9.0) < 0.05,
           "the drift range is back to 9 ms (was %.0f ms uncorrected)"
           % (raw["drift_max_ms"] - raw["drift_min_ms"]))
    _check(rf["classification"] == "MIXED" and rf["cmd_gaps_unmatched"] == 1,
           "scenario F still classifies as MIXED with one host-only gap")
    ps = os.path.join(tmp, "splitclock.pcap")
    write_pcap(ps, split_clock_frames(frames, 1.6, 5_000_000_000), "<", True)
    rs = analyze(ps, robot_ip="192.0.2.10", gap_ms=1.5, clock_scale="auto")
    _check(abs(rs["clock_scale"] - 1.6) < 1e-6
           and abs(rs["clock_info"]["cmd_scale"] - 1.0) < 1e-4
           and rs["clock_info"]["split_clocks"],
           "a receive-only fast clock is fitted per direction (%.6f / %.6f)"
           % (rs["clock_scale"], rs["clock_info"]["cmd_scale"]))
    gs = rs["state_gaps"]
    _check(len(gs) == 3 and abs(gs[0]["gap_us"] - 7000.0) < 1.0
           and abs(gs[1]["gap_us"] - 4000.0) < 1.0
           and abs(gs[2]["gap_us"] - 2000.0) < 1.0,
           "the split-clock capture still yields the 7000/4000/2000 us gaps")
    _check(rs["cmd_gaps_unmatched"] == 1 and len(rs["cmd_gaps"]) == 4,
           "and still finds exactly one host-only command gap across the 5 s "
           "offset between the two clocks")
    _check(rs["classification_detail"] ==
           "MIXED (3 transit gaps vs 1 host-only gap: no clear majority)",
           "and the same verdict as the single-clock capture")

    rfix = analyze(p, robot_ip="192.0.2.10", gap_ms=1.5, clock_scale=1.6)
    _check(len(rfix["state_gaps"]) == 3 and rfix["clock_info"]["mode"] == "fixed",
           "an explicit --clock-scale 1.6 gives the same table")

    # ---- cross-check the generated files with tcpdump itself --------------
    section("cross-check: tcpdump -r on the synthetic files")
    try:
        for name in ("pcap-usec-le.pcap", "pcapng-nsec.pcap"):
            fp = os.path.join(tmp, name)
            out = subprocess.run(["tcpdump", "-r", fp, "-nn", "-q"],
                                 capture_output=True, timeout=120)
            got = len([ln for ln in out.stdout.decode().splitlines() if ln.strip()])
            _check(out.returncode == 0 and got == len(frames),
                   "tcpdump reads %s and sees all %d packets" % (name, len(frames)))
    except (FileNotFoundError, subprocess.SubprocessError) as exc:
        print("    skip tcpdump cross-check (%s)" % exc)

    print("\nself-test: all checks passed (temp files in %s)" % tmp)
    return failures
