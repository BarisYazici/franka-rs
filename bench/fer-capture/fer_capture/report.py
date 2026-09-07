"""Text report rendering for an ``analyze()`` result."""

import textwrap

from .fci import COMMAND_MESSAGE_ID_OFF, STATE_MESSAGE_ID_OFF
from .stats import fmt_stats

# ---------------------------------------------------------------------------
# report rendering
# ---------------------------------------------------------------------------

RULE = "=" * 100


def _sparkline(off_ns, width=72):
    if not off_ns:
        return ""
    ramp = " .:-=+*#%@"
    lo, hi = min(off_ns), max(off_ns)
    span = hi - lo
    n = len(off_ns)
    cols = []
    for c in range(width):
        a = c * n // width
        b = max(a + 1, (c + 1) * n // width)
        chunk = off_ns[a:b]
        v = sum(chunk) / len(chunk)
        idx = 0 if span == 0 else int((v - lo) / span * (len(ramp) - 1))
        cols.append(ramp[idx])
    return "".join(cols)


def render(res, show_path=True):
    out = []
    w = out.append
    w(RULE)
    w("Franka Emika Robot (FER, FCI v5) robot-state arrival analysis")
    if show_path:
        w("capture            : %s" % res["path"])
    w("flow               : robot %s:%s -> client %s:%s" %
      (res["robot_ip"], res["robot_port"], res["host_ip"], res["host_port"]))
    w("thresholds         : gap >= %.2f ms, burst window %.1f ms, "
      "state/command match +/- %.1f ms" %
      (res["gap_ms"], res["burst_ms"], res["match_ms"]))
    ci = res["clock_info"]
    if res["clock_scale"] != 1.0 or ci.get("mode") != "none":
        line = "clock scale        : %.9f (%s)" % (res["clock_scale"], ci.get("mode"))
        if ci.get("slope_ns_per_ms"):
            line += ", fitted %.1f ns of capture clock per robot ms" % \
                ci["slope_ns_per_ms"]
        if ci.get("rms_us") is not None:
            line += ", residual RMS %.1f us over %d states" % \
                (ci["rms_us"], ci.get("kept", ci.get("n", 0)))
        if ci.get("note"):
            line += " [%s]" % ci["note"]
        w(line)
        if ci.get("cmd_scale") is not None and ci.get("cmd_scale") != res["clock_scale"]:
            w("command clock      : %.9f%s" %
              (ci["cmd_scale"],
               ", residual RMS %.1f us" % ci["cmd_rms_us"]
               if ci.get("cmd_rms_us") is not None else ""))
        if ci.get("split_clocks"):
            w("                     the two directions are on DIFFERENT clocks "
              "(-j adapter_unsynced timestamps receive in hardware and transmit")
            sh = ci.get("cmd_align_us", 0.0)
            sh_s = ("%.1f us" % sh if abs(sh) < 1000 else
                    ("%.3f ms" % (sh / 1000) if abs(sh) < 1e6 else
                     "%.3f s" % (sh / 1e6)))
            w("                     in software); each is rate-corrected separately "
              "and the command timeline is then shifted by %s so" % sh_s)
            w("                     median turnaround is zero - read the "
              "state/command coincidence column with that in mind")
        if res["clock_scale"] != 1.0:
            w("                     every timestamp below is divided by that scale")
    w("wire offsets       : state message_id/robot-time u64 LE @%d, "
      "command message_id u64 LE @%d" % (STATE_MESSAGE_ID_OFF, COMMAND_MESSAGE_ID_OFF))
    w(RULE)
    w("")
    w("-- capture ---------------------------------------------------------")
    w("  link-layer types           : %s" % ", ".join(str(x) for x in res["linktypes"]))
    w("  packets read               : %d  (IPv4: %d)" % (res["n_packets"], res["n_ip"]))
    w("  reassembled UDP datagrams  : %d" % res["n_datagrams"])
    w("  incomplete fragment sets   : %d" % res["n_incomplete"])
    w("  snaplen-truncated datagrams: %d" % res["n_truncated"])
    w("  datagrams outside the flow : %d" % res["n_other"])
    if res["classification"] == "NO_DATA":
        w("")
        w(res["verdict"])
        return "\n".join(out)
    w("  capture span               : %.3f s" % res["duration_s"])
    w("")
    w("-- robot -> host state stream --------------------------------------")
    w("  complete state datagrams   : %d" % len(res["states"]))
    w("  payload lengths            : %s" %
      ", ".join("%d B x%d" % (k, v) for k, v in res["state_payload_lens"][:5]))
    w("  IP fragments per datagram  : %s" %
      ", ".join(str(x) for x in res["state_frag_counts"]))
    w("  message_id range           : %d .. %d" % (res["id_min"], res["id_max"]))
    w("  expected from id range     : %d" % res["expected"])
    w("  missing ids                : %d  (max consecutive: %d, duplicates: %d)" %
      (res["n_missing"], res["max_consecutive_missing"], res["duplicates"]))
    if res["missing_ranges"]:
        shown = res["missing_ranges"][:20]
        w("  missing id ranges          : %s%s" %
          (", ".join("%d" % a if a == b else "%d-%d" % (a, b) for a, b in shown),
           "" if len(shown) == len(res["missing_ranges"]) else
           " ... (%d more)" % (len(res["missing_ranges"]) - len(shown))))
    w("")
    w("  inter-arrival and fragment timing (microseconds)")
    w(fmt_stats("state gap, FIRST fragment", res["ia_first"]))
    w(fmt_stats("state gap, LAST fragment", res["ia_last"]))
    w(fmt_stats("first->last fragment", res["frag_delay"]))
    nz = res["frag_delay_zero"]
    w("    of those, exactly 0 us      : %d of %d (%.1f%%)%s" %
      (nz, len(res["states"]), 100.0 * nz / max(1, len(res["states"])),
       "  - the capture clock cannot resolve the two fragments"
       if nz > len(res["states"]) // 2 else ""))
    w("")
    w("-- state gaps >= %.2f ms --------------------------------------------" %
      res["gap_ms"])
    if not res["state_gaps"]:
        w("  (none)")
    else:
        w("     t_off[s]     gap_us  gap_last_us    id_before     id_after  consec  "
          "robot_dt_ms  next_%.0fms" % res["burst_ms"])
        for g in res["state_gaps"][:res["max_rows"]]:
            w("  %10.6f %10.1f  %11.1f %12d %12d  %-6s %12d  %5d %s" %
              (g["t_off_s"], g["gap_us"], g["gap_last_us"], g["id_before"],
               g["id_after"], "yes" if g["consecutive"] else "NO",
               g["robot_dt_ms"], g["burst"],
               "BURST" if g["is_burst"] else "cadence"))
        if len(res["state_gaps"]) > res["max_rows"]:
            w("  ... %d more rows suppressed (--max-rows)" %
              (len(res["state_gaps"]) - res["max_rows"]))
    w("")
    w("-- robot clock minus arrival (drift) -------------------------------")
    w("  offset is (message_id ms) - (first-fragment arrival), zeroed on the first state.")
    w("  min / max / range          : %.3f / %.3f / %.3f ms" %
      (res["drift_min_ms"], res["drift_max_ms"],
       res["drift_max_ms"] - res["drift_min_ms"]))
    w("  first / last               : %.3f / %.3f ms" %
      (res["drift_first_ms"], res["drift_last_ms"]))
    w("  %.3f ms |%s| %.3f ms" %
      (res["drift_min_ms"], _sparkline(res["drift_offsets"]), res["drift_max_ms"]))
    w("  jumps >= %.2f ms           : %d" % (res["jump_ms"], len(res["drift_jumps"])))
    for j in res["drift_jumps"][:res["max_rows"]]:
        w("     t_off=%10.6f s  delta=%+9.3f ms  ids %d -> %d (%s)" %
          (j["t_off_s"], j["delta_ms"], j["id_before"], j["id_after"],
           "consecutive" if j["consecutive"] else "id skipped"))
    w("")
    w("-- host -> robot command stream ------------------------------------")
    if not res["cmds"]:
        w("  (no command datagrams captured)")
    else:
        w("  command datagrams          : %d" % len(res["cmds"]))
        w("  message_id range           : %d .. %d  (missing: %d)" %
          (res["cmd_id_min"], res["cmd_id_max"], res["cmd_missing"]))
        w(fmt_stats("command send interval", res["cmd_ia"]))
        w("  gaps >= %.2f ms            : %d  (%d unmatched by a state gap)" %
          (res["gap_ms"], len(res["cmd_gaps"]), res["cmd_gaps_unmatched"]))
        if res["cmd_gaps"]:
            w("     t_off[s]     gap_us    id_before     id_after  matches a state gap")
            for g in res["cmd_gaps"][:res["max_rows"]]:
                w("  %10.6f %10.1f %12d %12d  %s" %
                  (g["t_off_s"], g["gap_us"], g["id_before"], g["id_after"],
                   "yes (state gap %.1f us: client was waiting)" % g["matched_gap_us"]
                   if g["matched"] else "NO  -> the client itself was late"))
            if len(res["cmd_gaps"]) > res["max_rows"]:
                w("  ... %d more rows suppressed (--max-rows)" %
                  (len(res["cmd_gaps"]) - res["max_rows"]))
    w("")
    w(RULE)
    w("CLASSIFICATION: %s" % res.get("classification_detail", res["classification"]))
    w(RULE)
    for line in textwrap.wrap(res["verdict"], 98):
        w(line)
    w(RULE)
    return "\n".join(out)
