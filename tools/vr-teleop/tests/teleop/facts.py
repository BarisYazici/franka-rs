"""One run's facts as shell assignments, for run_tests.sh.

    eval "$(python -m tests.teleop.facts run.summary.json run.emit.jsonl)"

Everything comes from franka-vr-teleop's own --summary-json (its counters, verbatim) and from
--emit (the packed bytes of every target and gripper command it published), so an assertion
is made against what the process really put on the wire, not against its log text.
"""
import json
import math
import shlex
import struct
import sys

TARGET = struct.Struct("<BBHIQQ7d")
GRIPPER = struct.Struct("<BBHIQQdd")


def main(argv):
    summary, emit = json.load(open(argv[1])), []
    with open(argv[2]) as fh:
        for line in fh:
            if line.strip():
                emit.append(json.loads(line))
    o = summary["options"]
    targets = [e for e in emit if e["kind"] == "target"]
    grips = [e for e in emit if e["kind"] == "gripper"]
    f = {
        "out": summary["out"], "driven": summary["driven"],
        "keepalive": summary["keepalive"], "gripper": summary["gripper"],
        "engage": summary["engagements"], "reanchor": summary["reanchors"],
        "lead_mm": summary["worst_lead_m"] * 1000.0,
        "lead_rot": summary["worst_lead_rot_rad"],
        "step_mm": summary["worst_step_m"] * 1000.0,
        "step_rot": summary["worst_step_rot_rad"],
        "hand_step_mm": summary["worst_hand_step_m"] * 1000.0,
        "cmd_speed": summary["worst_cmd_speed"],
        "cmd_rot_speed": summary["worst_cmd_rot_speed"],
        "hand_rot_speed": summary["worst_hand_rot_speed"],
        "gap_ms": summary["worst_pub_gap_ms"],
        "hand": summary["hand"], "port": summary["port"], "crossed": summary["crossed"],
    }
    # Where the run's node limits came from; "none" until a summary carries the block.
    f["limits"] = (summary.get("node_limits") or {}).get("source", "none")
    for name, count in summary["refusals"].items():
        f[name] = count
    for name, count in summary["clamps"].items():
        f["clamp_" + name] = count

    # -- the bytes, decoded back
    layout_ok, flags_max, client_ids = True, 0, set()
    seqs, stamps, sign_flips, reengage_lead = [], [], 0, 0.0
    # --workspace is optional: with no box there is nothing to be outside of, so `in_box`
    # is vacuously true and `workspace` says which case the run saw.
    box = o.get("workspace")
    in_box = True
    lo, hi = box if box is not None else (None, None)
    prev_q, prev_why = None, None
    for e in targets:
        raw = bytes.fromhex(e["raw"])
        if len(raw) != 80:
            layout_ok = False
            continue
        version, kind, flags, client, seq, stamp, *data = TARGET.unpack(raw)
        layout_ok &= (version == 1 and kind == 1)
        flags_max = max(flags_max, flags)
        client_ids.add(client)
        seqs.append(seq)
        stamps.append(stamp)
        pos, quat = data[0:3], data[3:7]
        layout_ok &= abs(math.sqrt(sum(c * c for c in quat)) - 1.0) < 1e-9
        layout_ok &= all(abs(a - b) < 1e-12 for a, b in zip(pos, e["pos"]))
        if box is not None and any(p < l - 1e-9 or p > h + 1e-9
                                   for p, l, h in zip(pos, lo, hi)):
            in_box = False
        if prev_q is not None and sum(x * y for x, y in zip(quat, prev_q)) < 0.0:
            sign_flips += 1
        if e["why"] == "drive" and prev_why != "drive" and e.get("lead") is not None:
            reengage_lead = max(reengage_lead, e["lead"] * 1000.0)
        prev_q, prev_why = quat, e["why"]

    f["seq_first"] = seqs[0] if seqs else 0
    f["seq_last"] = seqs[-1] if seqs else 0
    f["seq_ok"] = seqs == list(range(1, len(seqs) + 1))
    f["stamp_ok"] = all(b > a for a, b in zip(stamps, stamps[1:])) if len(stamps) > 1 else True
    f["client_id"] = client_ids.pop() if len(client_ids) == 1 else -1
    f["layout_ok"] = bool(layout_ok)
    f["flags_max"] = flags_max
    f["sign_flips"] = sign_flips
    f["reengage_lead_mm"] = reengage_lead
    f["in_box"] = in_box
    f["workspace"] = "box" if box is not None else "none"

    gaps = [(b["t_src"] - a["t_src"]) / 1e6 for a, b in zip(grips, grips[1:])]
    f["gripper_min_ms"] = min(gaps) if gaps else 1e9
    f["gripper_first_ms"] = ((grips[0]["t_src"] - targets[0]["t_src"]) / 1e6
                             if grips and targets else 1e9)
    f["gripper_width_ok"] = all(
        0.0 <= g["width"] <= o["gripper_max_width"] + 1e-12 for g in grips)
    gseqs = [g["seq"] for g in grips]
    f["gripper_seq_ok"] = gseqs == list(range(gseqs[0], gseqs[0] + len(gseqs))) if gseqs \
        else True

    for key, value in f.items():
        # Strings are quoted for the shell; the assertions quote them again for Python.
        print(f"r_{key}={shlex.quote(value) if isinstance(value, str) else value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
