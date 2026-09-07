#!/usr/bin/env python3
"""Offline arrival-timing analysis of Franka Emika Robot (FER, FCI v5) robot-state datagrams.

Reads a tcpdump capture (classic pcap - microsecond or nanosecond magic, either
endianness - or pcapng), reassembles IPv4 fragments and reports *when* the 1 kHz
robot-state stream stalled and *which side* stalled.

Pure Python 3 standard library: no scapy, no dpkt.

Thin CLI entry point: parses argv and calls into the `fer_capture` package
(see `fer_capture/__init__.py` for the module layout, `fer_capture/fci.py`
for the FCI v5 wire-format background this reads).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fer_capture.fci import analyze
from fer_capture.pcap import CaptureError
from fer_capture.report import render
from fer_capture.selftest import SelfTestError, self_test


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Analyse the arrival timing of Franka Emika Robot (FER, FCI v5) robot-state "
                    "datagrams in a tcpdump capture.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Classification:\n"
               "  ROBOT_OR_WIRE_STALL  gaps are already in the arrival timestamps\n"
               "  HOST_STALL           states kept cadence, the client stopped answering\n"
               "  MIXED                both\n"
               "  NO_STALL             no gap above the threshold in either direction\n")
    ap.add_argument("pcap", nargs="?", help="capture file (pcap or pcapng)")
    ap.add_argument("--robot-ip", default=None,
                    help="robot address (default: auto-detect the big-datagram side)")
    ap.add_argument("--gap-ms", type=float, default=1.5,
                    help="report inter-arrival gaps at or above this (default 1.5)")
    ap.add_argument("--burst-ms", type=float, default=3.0,
                    help="burst window after a gap (default 3.0)")
    ap.add_argument("--match-ms", type=float, default=2.0,
                    help="state/command gap coincidence window (default 2.0)")
    ap.add_argument("--jump-ms", type=float, default=None,
                    help="drift jump threshold (default: --gap-ms)")
    ap.add_argument("--clock-scale", default=None, metavar="auto|FACTOR",
                    help="divide every timestamp by this factor before analysing. "
                         "`auto` fits the capture clock against the robot's own "
                         "millisecond clock (message_id) and prints the slope and "
                         "residual RMS. Needed for `tcpdump -j adapter_unsynced` "
                         "captures, whose free-running NIC clock is not disciplined "
                         "to anything (this box's i219 runs ~1.6x fast).")
    ap.add_argument("--max-rows", type=int, default=200,
                    help="maximum table rows to print (default 200)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the built-in synthetic tests and exit")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="with --self-test, also print one full report")
    args = ap.parse_args(argv)

    if args.self_test:
        try:
            self_test(verbose=args.verbose)
        except SelfTestError as exc:
            print("\nSELF-TEST FAILED: %s" % exc, file=sys.stderr)
            return 1
        return 0

    if not args.pcap:
        ap.error("a capture file is required (or use --self-test)")
    try:
        res = analyze(args.pcap, robot_ip=args.robot_ip, gap_ms=args.gap_ms,
                      burst_ms=args.burst_ms, match_ms=args.match_ms,
                      jump_ms=args.jump_ms, max_rows=args.max_rows,
                      clock_scale=args.clock_scale)
    except (CaptureError, OSError) as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 2
    print(render(res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
