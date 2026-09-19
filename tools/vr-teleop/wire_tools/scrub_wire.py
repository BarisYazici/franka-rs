#!/usr/bin/env python3
"""De-timestamp a record_wire.py recording so it can ship as a test fixture.

    python wire_tools/scrub_wire.py run.jsonl fixtures/desk-idle.jsonl.gz

A recording is real captured data. Everything a replay needs is in the payload and the
relative arrival times; the wall clock is not, so it goes:

  header   `t0_wall_ns`, `t0_utc` and the free-text `note` are dropped. `endpoints`,
           `struct`, `msg_size`, `magic`, `msg_version`, `secs` stay: ReplaySource matches
           `ep`, and an analyser reads the rest.
  records  `w` (the wall-clock stamp of every message) is dropped.
  clocks   `t` and `t0_perf_ns` are monotonic, so they leak the recording host's uptime.
           They are rebased so the first message sits at --t0 (1 s, as make_wire.py stamps
           its synthetic wires). Every interval is preserved exactly: the stamps are
           integers and the same constant is subtracted from all of them.

Nothing else is touched: the `raw` hex of every message, its decoded fields and the footer
are the recording as it was captured. The output is gzipped when its name ends in `.gz`.
"""
import argparse
import gzip
import json
import sys

DROP_HEADER = ("t0_wall_ns", "t0_utc", "note")
DROP_RECORD = ("w",)


def scrub(lines, t0=1_000_000_000):
    """Yield the scrubbed records. The first message's `t` sets the rebase offset, so the
    header is held back until it is known."""
    header, shift = None, None
    for line in lines:
        line = line.strip()
        if not line:
            continue
        rec = json.loads(line)
        kind = rec.get("type")
        if kind == "header":
            header = rec
            continue
        if kind:                                          # the footer, verbatim
            yield rec
            continue
        if shift is None:
            shift = rec["t"] - t0
            if header is not None:
                for key in DROP_HEADER:
                    header.pop(key, None)
                if "t0_perf_ns" in header:
                    header["t0_perf_ns"] -= shift
                yield header
        for key in DROP_RECORD:
            rec.pop(key, None)
        rec["t"] -= shift
        yield rec


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("src")
    p.add_argument("dst", help="output .jsonl, or .jsonl.gz to compress")
    p.add_argument("--t0", type=int, default=1_000_000_000,
                   help="arrival stamp of the first message, ns (default 1e9)")
    a = p.parse_args(argv)

    n = 0
    with open(a.src) as src, open(a.dst, "wb") as raw:
        # mtime 0 and no stored filename: the same input gives the same bytes
        dst = gzip.GzipFile(filename="", mode="wb", compresslevel=9, fileobj=raw, mtime=0) \
            if a.dst.endswith(".gz") else raw
        for rec in scrub(src, a.t0):
            dst.write((json.dumps(rec) + "\n").encode())
            n += 1
        if dst is not raw:
            dst.close()
    print(f"[scrub] {a.src} -> {a.dst} ({n} lines)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
