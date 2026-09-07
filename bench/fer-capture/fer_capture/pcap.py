"""Capture file readers: classic pcap (any magic/endianness) and pcapng.

Pure Python 3 standard library: no scapy, no dpkt.
"""

import os
import struct

# ---------------------------------------------------------------------------
# capture file readers
# ---------------------------------------------------------------------------

PCAP_MAGICS = {
    b"\xd4\xc3\xb2\xa1": ("<", 1000),   # microseconds, little endian
    b"\xa1\xb2\xc3\xd4": (">", 1000),   # microseconds, big endian
    b"\x4d\x3c\xb2\xa1": ("<", 1),      # nanoseconds,  little endian
    b"\xa1\xb2\x3c\x4d": (">", 1),      # nanoseconds,  big endian
}
PCAPNG_SHB = b"\x0a\x0d\x0d\x0a"


class CaptureError(Exception):
    pass


def read_packets(path):
    """Yield (ts_ns, linktype, data, orig_len) for every packet in the capture."""
    with open(path, "rb") as f:
        magic = f.read(4)
        if len(magic) < 4:
            raise CaptureError("%s: too short to be a capture file" % path)
        if magic in PCAP_MAGICS:
            yield from _read_classic(f, magic)
        elif magic == PCAPNG_SHB:
            yield from _read_pcapng(f)
        else:
            raise CaptureError(
                "%s: unknown file magic %s (not pcap, not pcapng)"
                % (path, magic.hex()))


def _read_classic(f, magic):
    endian, mult = PCAP_MAGICS[magic]
    hdr = f.read(20)
    if len(hdr) < 20:
        raise CaptureError("truncated pcap file header")
    _maj, _min, _tz, _sig, _snap, network = struct.unpack(endian + "HHiIII", hdr)
    linktype = network & 0xFFFF     # top bits carry FCS information
    rec = struct.Struct(endian + "IIII")
    while True:
        rh = f.read(16)
        if len(rh) < 16:
            return
        ts_sec, ts_frac, incl, orig = rec.unpack(rh)
        data = f.read(incl)
        if len(data) < incl:
            return                   # truncated last record
        yield (ts_sec * 1_000_000_000 + ts_frac * mult, linktype, data, orig)


def _tsresol_to_ns(ticks, tsresol):
    if tsresol & 0x80:
        return (ticks * 1_000_000_000) >> (tsresol & 0x7F)
    return ticks * 1_000_000_000 // (10 ** tsresol)


def _pcapng_options(body, endian):
    """Parse an option list, returning {code: value_bytes} (first occurrence wins)."""
    out = {}
    off = 0
    while off + 4 <= len(body):
        code, ln = struct.unpack_from(endian + "HH", body, off)
        off += 4
        if code == 0:
            break
        val = body[off:off + ln]
        off += ln + ((4 - ln % 4) % 4)
        out.setdefault(code, val)
    return out


def _read_pcapng(f):
    f.seek(0)
    endian = "<"
    ifaces = []          # (linktype, tsresol, tsoffset_ns)
    while True:
        head = f.read(8)
        if len(head) < 8:
            return
        if head[0:4] == PCAPNG_SHB:
            bom = f.read(4)
            if len(bom) < 4:
                return
            if bom == b"\x4d\x3c\x2b\x1a":
                endian = "<"
            elif bom == b"\x1a\x2b\x3c\x4d":
                endian = ">"
            else:
                raise CaptureError("pcapng: bad byte-order magic %s" % bom.hex())
            blen = struct.unpack(endian + "I", head[4:8])[0]
            if blen < 16:
                raise CaptureError("pcapng: bad section header length %d" % blen)
            f.seek(blen - 12, os.SEEK_CUR)
            ifaces = []                      # a new section restarts numbering
            continue
        btype, blen = struct.unpack(endian + "II", head)
        if blen < 12:
            raise CaptureError("pcapng: bad block length %d (type 0x%x)" % (blen, btype))
        body = f.read(blen - 12)
        if len(body) < blen - 12:
            return
        if len(f.read(4)) < 4:
            return
        if btype == 0x01:                                    # Interface Description
            linktype = struct.unpack_from(endian + "H", body, 0)[0]
            opts = _pcapng_options(body[8:], endian)
            tsresol = opts.get(9, b"\x06")[0] if 9 in opts else 6
            tsoff = 0
            if 14 in opts and len(opts[14]) >= 8:
                tsoff = struct.unpack(endian + "q", opts[14][:8])[0] * 1_000_000_000
            ifaces.append((linktype, tsresol, tsoff))
        elif btype == 0x06:                                  # Enhanced Packet Block
            iface, ts_hi, ts_lo, caplen, orig = struct.unpack_from(
                endian + "IIIII", body, 0)
            if iface >= len(ifaces):
                continue
            linktype, tsresol, tsoff = ifaces[iface]
            ts_ns = _tsresol_to_ns((ts_hi << 32) | ts_lo, tsresol) + tsoff
            yield (ts_ns, linktype, body[20:20 + caplen], orig)
        elif btype == 0x03:                                  # Simple Packet Block
            if not ifaces:
                continue
            orig = struct.unpack_from(endian + "I", body, 0)[0]
            linktype = ifaces[0][0]
            yield (0, linktype, body[4:4 + orig], orig)
        # every other block type is skipped
