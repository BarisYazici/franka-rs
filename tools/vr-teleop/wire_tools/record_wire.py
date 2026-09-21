#!/usr/bin/env python3
"""Record the VR pose wire: every VrTargetMsg, byte for byte, with its
arrival time and every field decoded. Append-only, no analysis.

    python -m wire_tools.record_wire --secs 60 --out run.jsonl \
        --endpoint tcp://127.0.0.1:5560 --endpoint tcp://127.0.0.1:5570

The message is the 88-byte `VrTargetMsg` the quest bridge publishes
(`franka_vr/wire.py`):

    off  0  magic    u32   0x564C5248 ("HRLV" little-endian)
    off  4  version  u32   1
    off  8  seq      u64   bridge tick counter, SHARED by every channel
    off 16  pos[3]   f64   metres, axis-permuted into robot axis DIRECTIONS but
                           in the Quest's own SLAM frame (arbitrary origin)
    off 40  quat[4]  f64   x, y, z, w -- scipy order, NOT w-first; while the
                           grip is released this is a CONSTANT, and while it is
                           held it is a rotation-delta carrier, not an absolute
                           orientation
    off 72  gripper  f64   closed fraction [0,1], 0=open; = this channel's
                           index trigger (rightTrig for 'r', leftTrig for 'l')
    off 80  flags    u32   bit0 Engaged (grip AND fresh AND not forced-open),
                           bit1 Fresh (raw 4x4 changed within 250 ms),
                           bit2 ControllerOn (same detector, 5 s window)
    off 84  buttons  u32   bit0 A/X, bit1 B/Y, bit2 thumbstick click; bits
                           3 (grip held) and 4 (trigger past threshold) are set
                           only by a mapper that reads the buttons, and are 0
                           from the bridge in this tree

There is deliberately no timestamp on the wire, which is why this recorder
stamps arrival itself.

NO CONFLATE HERE, ON PURPOSE. A consumer sets ZMQ_CONFLATE=1 and therefore
drops messages silently; a recorder that did the same could not measure the
bridge's real rate or tell a bridge-side gap from a socket-side drop. We take every
message the PUB sends us and let the analyser say what arrived.

Crash-proof: opened for APPEND, one self-contained JSON object per line, the
line flushed as it is written and fsynced every `--fsync-every` messages. A
kill -9 costs at most the unsynced tail; the analyser tolerates a missing
footer and a truncated last line. Depends on pyzmq and nothing else.
"""
import argparse
import json
import os
import signal
import sys
import time

import zmq

from franka_vr.wire import (VR_FMT, VR_MSG_MAGIC, VR_MSG_SIZE,
                            VR_MSG_VERSION)

FLAG_BITS = (("engaged", 1), ("fresh", 2), ("controller_on", 4))
BUTTON_BITS = (("a", 1), ("b", 2), ("stick", 4), ("grip", 8), ("trigger", 16))

_stop = False


def _on_signal(signum, _frame):
    global _stop
    _stop = True


def decode(payload: bytes) -> dict:
    """Every field of one message, plus `bad` when it is not one."""
    if len(payload) != VR_MSG_SIZE:
        return {"bad": "length", "len": len(payload)}
    magic, version, seq, *rest = VR_FMT.unpack(payload)
    pos, quat, gripper = rest[0:3], rest[3:7], rest[7]
    flags, buttons = rest[8], rest[9]
    out = {
        "magic": magic,
        "version": version,
        "seq": seq,
        "pos": list(pos),
        "quat": list(quat),
        "gripper": gripper,
        "flags": flags,
        "buttons": buttons,
    }
    for name, bit in FLAG_BITS:
        out[name] = bool(flags & bit)
    for name, bit in BUTTON_BITS:
        out["btn_" + name] = bool(buttons & bit)
    if magic != VR_MSG_MAGIC:
        out["bad"] = "magic"
    elif version != VR_MSG_VERSION:
        out["bad"] = "version"
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--endpoint", action="append", default=None,
                   help="PUB endpoint to SUB to; repeat for several channels "
                        "(default: 5560 and 5570, i.e. both hands)")
    p.add_argument("--secs", type=float, default=60.0,
                   help="record for this long; 0 = until Ctrl-C")
    p.add_argument("--out", default=None,
                   help="output .jsonl (default: wire-<UTC stamp>.jsonl)")
    p.add_argument("--note", default="",
                   help="free text stored in the header, e.g. 'two-handed pick'")
    p.add_argument("--fsync-every", type=int, default=100,
                   help="fsync after this many messages (0 = never)")
    p.add_argument("--quiet", action="store_true", help="no progress line")
    args = p.parse_args(argv)

    endpoints = args.endpoint or ["tcp://127.0.0.1:5560", "tcp://127.0.0.1:5570"]
    out_path = args.out or time.strftime("wire-%Y%m%dT%H%M%SZ.jsonl", time.gmtime())

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    ctx = zmq.Context()
    poller = zmq.Poller()
    socks = {}
    for ep in endpoints:
        s = ctx.socket(zmq.SUB)
        s.setsockopt(zmq.LINGER, 0)
        s.setsockopt(zmq.RCVHWM, 200000)   # never drop; see the module docstring
        s.setsockopt(zmq.SUBSCRIBE, b"")   # no topic envelope: one raw frame
        s.connect(ep)                      # the bridge binds, we connect
        socks[s] = ep
        poller.register(s, zmq.POLLIN)

    counts = {ep: 0 for ep in endpoints}
    bad = {"length": 0, "magic": 0, "version": 0}
    total = 0
    fh = open(out_path, "a", buffering=1)
    t0 = time.perf_counter_ns()
    header = {
        "type": "header",
        "tool": "record_wire.py",
        "record_version": 1,
        "note": args.note,
        "endpoints": endpoints,
        "conflate": False,
        "struct": "<IIQ8dII",
        "msg_size": VR_MSG_SIZE,
        "magic": VR_MSG_MAGIC,
        "msg_version": VR_MSG_VERSION,
        "t0_perf_ns": t0,
        "t0_wall_ns": time.time_ns(),
        "t0_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "secs": args.secs,
    }
    fh.write(json.dumps(header) + "\n")

    deadline = None if args.secs <= 0 else t0 + int(args.secs * 1e9)
    next_report = t0 + 1_000_000_000
    reported = 0
    try:
        while not _stop:
            now = time.perf_counter_ns()
            if deadline is not None and now >= deadline:
                break
            timeout_ms = 200
            if deadline is not None:
                timeout_ms = min(timeout_ms, max(0, (deadline - now) // 1_000_000))
            for sock, _ in poller.poll(timeout=timeout_ms):
                ep = socks[sock]
                while True:                     # drain this socket
                    try:
                        payload = sock.recv(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    rec = {"t": time.perf_counter_ns(), "w": time.time_ns(), "ep": ep,
                           "raw": payload.hex()}
                    rec.update(decode(payload))
                    fh.write(json.dumps(rec) + "\n")
                    total += 1
                    counts[ep] += 1
                    if "bad" in rec:
                        bad[rec["bad"]] = bad.get(rec["bad"], 0) + 1
                    if args.fsync_every and total % args.fsync_every == 0:
                        fh.flush()
                        os.fsync(fh.fileno())
            if not args.quiet and time.perf_counter_ns() >= next_report:
                elapsed = (time.perf_counter_ns() - t0) / 1e9
                print(f"[record] {elapsed:5.1f}s  {total:6d} msgs  "
                      f"{total - reported:3d}/s  "
                      + "  ".join(f"{ep.rsplit(':', 1)[-1]}={n}"
                                  for ep, n in counts.items()),
                      file=sys.stderr, flush=True)
                reported = total
                next_report += 1_000_000_000
    except Exception:                           # never lose the recording
        import traceback
        traceback.print_exc()
    finally:
        elapsed_s = (time.perf_counter_ns() - t0) / 1e9
        footer = {"type": "footer", "total": total, "per_endpoint": counts,
                  "bad": bad, "elapsed_s": elapsed_s,
                  "stopped_by": "signal" if _stop else "deadline"}
        try:
            fh.write(json.dumps(footer) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
        finally:
            fh.close()
        for sock in socks:
            sock.close(0)
        ctx.term()
        print(f"[record] wrote {out_path}: {total} messages in {elapsed_s:.1f} s "
              f"({', '.join(f'{ep}={n}' for ep, n in counts.items())}), "
              f"bad={bad}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
