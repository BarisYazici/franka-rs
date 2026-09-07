"""Link-layer stripping and IPv4 fragment reassembly into UDP datagrams."""

import struct

# ---------------------------------------------------------------------------
# link / IP / UDP decoding
# ---------------------------------------------------------------------------

def strip_link(linktype, data):
    """Return the IPv4 packet bytes carried by a link-layer frame, or None."""
    if linktype == 1:                                        # EN10MB
        if len(data) < 14:
            return None
        off = 12
        etype = struct.unpack_from("!H", data, off)[0]
        off += 2
        while etype in (0x8100, 0x88A8, 0x9100):             # VLAN tags
            if len(data) < off + 4:
                return None
            etype = struct.unpack_from("!H", data, off + 2)[0]
            off += 4
        return data[off:] if etype == 0x0800 else None
    if linktype == 113:                                      # LINUX_SLL
        if len(data) < 16:
            return None
        return data[16:] if struct.unpack_from("!H", data, 14)[0] == 0x0800 else None
    if linktype == 276:                                      # LINUX_SLL2
        if len(data) < 20:
            return None
        return data[20:] if struct.unpack_from("!H", data, 0)[0] == 0x0800 else None
    if linktype == 0:                                        # DLT_NULL (BSD loopback)
        if len(data) < 4:
            return None
        fam = struct.unpack_from("<I", data, 0)[0]
        if fam > 0xFFFF:
            fam = struct.unpack_from(">I", data, 0)[0]
        return data[4:] if fam == 2 else None
    if linktype == 108:                                      # DLT_LOOP
        if len(data) < 4:
            return None
        return data[4:] if struct.unpack_from(">I", data, 0)[0] == 2 else None
    if linktype in (12, 14, 101, 228):                       # raw IP
        return data
    return None


def ip_str(raw):
    return "%d.%d.%d.%d" % tuple(raw)


class Datagram:
    """One fully reassembled IPv4/UDP datagram."""
    __slots__ = ("t_first", "t_last", "src", "dst", "sport", "dport",
                 "payload", "frags", "truncated")

    def __init__(self, t_first, t_last, src, dst, sport, dport, payload, frags,
                 truncated):
        self.t_first = t_first
        self.t_last = t_last
        self.src = src
        self.dst = dst
        self.sport = sport
        self.dport = dport
        self.payload = payload
        self.frags = frags
        self.truncated = truncated


class Reassembler:
    """IPv4 fragment reassembly keyed by (src, dst, ident, proto)."""

    TIMEOUT_NS = 2 * 1_000_000_000

    def __init__(self):
        self.pending = {}
        self.incomplete = 0
        self.overlapping = 0

    def add(self, ts, ip):
        """Feed one IPv4 packet; return a Datagram if one completed."""
        if len(ip) < 20 or (ip[0] >> 4) != 4:
            return None
        ihl = (ip[0] & 0x0F) * 4
        if ihl < 20 or len(ip) < ihl:
            return None
        total_len = struct.unpack_from("!H", ip, 2)[0]
        ident = struct.unpack_from("!H", ip, 4)[0]
        flags_frag = struct.unpack_from("!H", ip, 6)[0]
        mf = bool(flags_frag & 0x2000)
        frag_off = (flags_frag & 0x1FFF) * 8
        proto = ip[9]
        src, dst = ip[12:16], ip[16:20]
        body = ip[ihl:total_len] if total_len >= ihl else ip[ihl:]
        truncated = len(body) < (total_len - ihl)
        if truncated:                       # snaplen cut the packet short
            body = ip[ihl:]
        if proto != 17:
            return None

        if frag_off == 0 and not mf:
            return self._finish(ts, ts, src, dst, body, 1, truncated)

        key = (src, dst, ident, proto)
        ent = self.pending.get(key)
        if ent is None or ts - ent["t_last"] > self.TIMEOUT_NS:
            if ent is not None:
                self.incomplete += 1
            ent = {"t_first": ts, "t_last": ts, "parts": [], "total": None,
                   "trunc": False, "n": 0}
            self.pending[key] = ent
        ent["t_first"] = min(ent["t_first"], ts)
        ent["t_last"] = max(ent["t_last"], ts)
        ent["parts"].append((frag_off, body))
        ent["n"] += 1
        ent["trunc"] = ent["trunc"] or truncated
        if not mf:
            ent["total"] = frag_off + len(body)

        if ent["total"] is None:
            return None
        parts = sorted(ent["parts"])
        buf = bytearray(ent["total"])
        covered = 0
        for off, chunk in parts:
            if off > covered:
                return None                 # hole: not complete yet
            if off + len(chunk) > ent["total"]:
                chunk = chunk[:ent["total"] - off]
            buf[off:off + len(chunk)] = chunk
            covered = max(covered, off + len(chunk))
        if covered < ent["total"]:
            return None
        del self.pending[key]
        return self._finish(ent["t_first"], ent["t_last"], src, dst, bytes(buf),
                            ent["n"], ent["trunc"])

    def _finish(self, t_first, t_last, src, dst, udp, frags, truncated):
        if len(udp) < 8:
            return None
        sport, dport, ulen, _cksum = struct.unpack_from("!HHHH", udp, 0)
        payload = udp[8:ulen] if 8 <= ulen <= len(udp) else udp[8:]
        return Datagram(t_first, t_last, ip_str(src), ip_str(dst), sport, dport,
                        payload, frags, truncated)

    def flush(self):
        self.incomplete += len(self.pending)
        self.pending.clear()
