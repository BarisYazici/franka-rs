#!/usr/bin/env python3
"""Summarises one bench/results/<timestamp> directory into markdown on stdout.

Also writes <dir>/summary.json with the same aggregated numbers, so the tables in
docs/ can be regenerated without re-running the matrix.

Usage: summarize.py bench/results/<timestamp>
"""

import json
import os
import re
import sys

CLIENTS = ("cpp", "rust")
VARIANTS = ("control", "active", "model")
CONDITIONS = ("plain", "rt", "rt_load")

CONDITION_LABELS = {
    "plain": "plain (SCHED_OTHER)",
    "rt": "chrt -f 80 + mlockall",
    "rt_load": "chrt -f 80 + mlockall + CPU hog",
}


def load_runs(out_dir):
    runs = []
    for name in sorted(os.listdir(out_dir)):
        if not name.endswith(".json") or name in ("meta.json", "summary.json", "stage.json"):
            continue
        # The hardware harness also drops probe_*.json / home_*.json here; those are not runs.
        if name.startswith(("probe_", "home_")):
            continue
        path = os.path.join(out_dir, name)
        try:
            with open(path) as handle:
                run = json.load(handle)
        except Exception as exc:  # a crashed run leaves no/partial JSON
            print(f"<!-- skipping {name}: {exc} -->", file=sys.stderr)
            continue
        if "lang" not in run:  # belt and braces: anything that is not a client run
            continue
        run["_name"] = name[: -len(".json")]
        run["_simlog"] = simlog_summary(os.path.join(out_dir, run["_name"] + ".simlog"))
        runs.append(run)
    return runs


STATE_STATS_RE = re.compile(r"State Update Stats.*")
LOST_RE = re.compile(r"lost|missed|late|skip", re.IGNORECASE)


def simlog_summary(path):
    """Pull the server-side cross-check lines out of one run's docker log slice."""
    if not os.path.exists(path):
        return {"state_update_stats": [], "lost_lines": [], "lost_line_count": 0}
    stats, lost = [], []
    with open(path, errors="replace") as handle:
        for line in handle:
            line = line.rstrip()
            if "State Update Stats" in line:
                stats.append(line)
            elif LOST_RE.search(line):
                lost.append(line)
    return {
        "state_update_stats": stats,
        "lost_lines": lost[:20],
        "lost_line_count": len(lost),
    }


def cell(runs, client, variant, condition):
    return [
        r
        for r in runs
        if r["lang"] == client and r["variant"] == variant and r["condition"] == condition
    ]


def mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else float("nan")


def fmt(value, digits=0):
    if value != value:  # NaN
        return "-"
    return f"{value:.{digits}f}"


def aggregate(cell_runs, key):
    """Mean over repetitions of a per-run statistic, and worst-case where that matters."""
    if not cell_runs:
        return None
    stats = [r[key] for r in cell_runs if r.get(key)]
    if not stats:
        return None
    return {
        "p50": mean([s["p50"] for s in stats]),
        "p99": mean([s["p99"] for s in stats]),
        "p999": mean([s["p999"] for s in stats]),
        "max": max(s["max"] for s in stats),
        "mean": mean([s["mean"] for s in stats]),
        "n": sum(s["n"] for s in stats),
        "reps": len(stats),
    }



def hardware_section(add, hardware_runs, stage, meta):
    """The FR3 hardware section: one row per run, plus guards, exceptions and reflex events.

    Hardware runs are never aggregated over repetitions the way the simulator cells are --
    every run against a real arm is its own event and is reported individually.
    """
    host = stage.get("host") or meta.get("host") or "?"
    # Which arm this was: the FR3 the harness was written for, or whatever `meta.json`
    # names (the FER campaign records `robot`).
    robot = meta.get("robot") or "FR3"
    add(f"## {robot} hardware ({host})")
    add("")
    add(
        "Real-arm runs. No container is involved; the harness probes the robot read-only "
        "before every run, recovers once from a `Reflex`, and drives the arm back to the "
        "ready pose with libfranka's `MotionGenerator` between runs. Both clients run with "
        "`--hardware`, which arms the |tau| and end-effector-deviation guards; a guard "
        "finishes the motion cleanly rather than killing the loop."
    )
    add("")
    if stage:
        add(
            f"Stage **{stage.get('stage', '?')}** — status **{stage.get('status', '?')}**, "
            f"{stage.get('reps', '?')} repetition(s) of {stage.get('duration_s', '?')} s, "
            f"{stage.get('pause_s', '?')} s pause between runs, return-to-ready at speed "
            f"{stage.get('home_speed', '?')}, "
            f"**{stage.get('reflex_events', 0)} reflex event(s)** "
            f"(stage abandoned after {stage.get('max_reflex_events', '?')})."
        )
        add("")

    add(
        "| run | provenance | order | cycles | model p50 (µs) | p99 | max | compute p50 (µs) "
        "| p99 | max | interval p50 | p99 | max | lost | success min | success avg | CPU % "
        "| max abs tau (Nm) | max EE dev (m) |"
    )
    add("|" + "---|" * 19)
    rows = []
    for run in sorted(hardware_runs, key=lambda r: (r.get("rep", 0), r.get("order_in_cell", 0))):
        model = run.get("model_us") or {}
        compute = run.get("compute_us") or {}
        interval = run["interval_us"]
        controller = run.get("controller", {})
        rows.append(
            {
                "name": run["_name"],
                "lang": run["lang"],
                "provenance": run.get("provenance", "?"),
                "cycles": run["cycles"],
                "model_us": model,
                "compute_us": compute,
                "interval_us": interval,
                "lost_cycles": run["lost"]["cycles"],
                "success_min": run["success_rate"]["min"],
                "success_avg": run["success_rate"]["avg"],
                "cpu_percent": run["cpu"]["percent"],
                "tau_max_abs": controller.get("tau_max_abs"),
                "ee_deviation_max_m": controller.get("ee_deviation_max_m"),
                "guard": run.get("guard"),
                "control_exception": run.get("control_exception"),
                "reflex_events_before_run": run.get("reflex_events_before_run"),
            }
        )
        add(
            f"| {run['_name']} | {run.get('provenance', '?')} "
            f"| {run.get('order_in_cell', '?')}/2, {run.get('cell_first_client', '?')} first "
            f"| {run['cycles']} "
            f"| {fmt(model.get('p50', float('nan')), 1)} | {fmt(model.get('p99', float('nan')), 1)} "
            f"| {fmt(model.get('max', float('nan')), 1)} "
            f"| {fmt(compute.get('p50', float('nan')), 1)} "
            f"| {fmt(compute.get('p99', float('nan')), 1)} "
            f"| {fmt(compute.get('max', float('nan')), 1)} "
            f"| {fmt(interval['p50'])} | {fmt(interval['p99'])} | {fmt(interval['max'])} "
            f"| {run['lost']['cycles']} | {run['success_rate']['min']:.3f} "
            f"| {run['success_rate']['avg']:.4f} | {fmt(run['cpu']['percent'], 1)} "
            f"| {fmt(controller.get('tau_max_abs', float('nan')), 3)} "
            f"| {fmt(controller.get('ee_deviation_max_m', float('nan')), 4)} |"
        )
    add("")

    tripped = [r for r in hardware_runs if (r.get("guard") or {}).get("tripped")]
    add("### Guards")
    add("")
    if tripped:
        add("| run | reason | cycle | tau at trip (Nm) | EE deviation at trip (m) |")
        add("|---|---|---|---|---|")
        for run in tripped:
            g = run["guard"]
            add(
                f"| {run['_name']} | {g['reason']} | {g['cycle']} "
                f"| {g['tau_at_trip']:.3f} | {g['ee_deviation_at_trip']:.4f} |"
            )
    else:
        limits = next((r.get("guard") for r in hardware_runs if r.get("guard")), {}) or {}
        add(
            "No guard tripped in any run "
            f"(limits: |tau| <= {limits.get('tau_limit_nm', '?')} Nm, "
            f"EE deviation <= {limits.get('ee_deviation_limit_m', '?')} m)."
        )
    add("")

    failed = [r for r in hardware_runs if r.get("control_exception")]
    add("### Control exceptions")
    add("")
    if failed:
        add("| run | exception |")
        add("|---|---|")
        for run in failed:
            text = str(run["control_exception"]).replace("|", "\\|")
            add(f"| {run['_name']} | {text} |")
    else:
        add("None: every run's control loop ended normally.")
    add("")

    return {"stage": stage, "runs": rows}


def main():
    out_dir = sys.argv[1]
    runs = load_runs(out_dir)
    meta_path = os.path.join(out_dir, "meta.json")
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}

    lines = []
    add = lines.append

    add(f"# franka-rs vs C++ libfranka — jitter benchmark ({meta.get('timestamp', '?')})")
    add("")
    add("## Machine")
    add("")
    add("| field | value |")
    add("|---|---|")
    for key in (
        "hostname",
        "cpu_model",
        "nproc",
        "kernel",
        "preempt_rt",
        "sched_rt_runtime_us",
        "ulimit_l",
        "docker_image",
        "sim_cpu_shares",
        "libfranka",
        "rustc",
        "gcc",
        "git_rev",
        "duration_s",
        "reps",
        "cyclictest",
        "stress_ng",
    ):
        if key in meta:
            add(f"| {key} | {meta[key]} |")
    add("")

    summary = {"meta": meta, "cells": []}

    for variant in VARIANTS:
        if not any(cell(runs, c, variant, k) for c in CLIENTS for k in CONDITIONS):
            continue
        title = {
            "control": "Variant 1 — `control` (libfranka `Robot::control` / "
            "`Robot::control_joint_velocities`, `limit_rate = true`)",
            "active": "Variant 2 — `active` (`startJointVelocityControl` / "
            "`start_joint_velocity_control`, `readOnce` → `writeOnce`)",
            "model": "Variant 3 — `model` (torque control with the robot model in the "
            "1 kHz loop: `Robot::control(Torques)` / `Robot::control_torques`)",
        }[variant]
        add(f"## {title}")
        add("")
        if variant == "control":
            add(
                "`interval` is the time between consecutive control-callback entries "
                "(the 1 kHz cycle as the client sees it)."
            )
        elif variant == "active":
            add(
                "`interval` is the time between consecutive `readOnce()` returns; "
                "`latency` is `readOnce()` return → `writeOnce()` return, i.e. the "
                "client's recv→send path."
            )
        else:
            add(
                "`model` is the time spent in the five `franka::Model` calls "
                "(`mass`, `coriolis`, `gravity`, `zeroJacobian`, `pose`); `compute` is the "
                "whole controller including those calls, the operational-space linear "
                "algebra and `limitRate`. `interval` is the time between consecutive "
                "control-callback entries."
            )
        add("")
        header = (
            "| condition | client | cycles | interval p50 (µs) | p99 | p99.9 | max "
            "| lost cycles | lost states | max consec | success min | success avg "
            "| CPU % | server lost-cycle lines |"
        )
        if variant == "active":
            header = (
                "| condition | client | cycles | interval p50 (µs) | p99 | p99.9 | max "
                "| latency p50 (µs) | p99 | p99.9 | max | lost cycles | max consec "
                "| success avg | CPU % |"
            )
        elif variant == "model":
            header = (
                "| condition | client | cycles | model p50 (µs) | p99 | p99.9 | max "
                "| compute p50 (µs) | p99 | p99.9 | max | interval p50 (µs) | p99 | max "
                "| lost cycles | success avg | CPU % |"
            )
        add(header)
        add("|" + "---|" * (header.count("|") - 1))

        for condition in CONDITIONS:
            for client in CLIENTS:
                cell_runs = cell(runs, client, variant, condition)
                if not cell_runs:
                    continue
                interval = aggregate(cell_runs, "interval_us")
                if interval is None:
                    continue
                latency = aggregate(cell_runs, "latency_us")
                compute = aggregate(cell_runs, "compute_us")
                model = aggregate(cell_runs, "model_us")
                cycles = mean([r["cycles"] for r in cell_runs])
                lost_cycles = mean([r["lost"]["cycles"] for r in cell_runs])
                lost_states = mean([r["lost"]["states"] for r in cell_runs])
                max_consec = max(r["lost"]["max_consecutive"] for r in cell_runs)
                sr_min = min(r["success_rate"]["min"] for r in cell_runs)
                sr_avg = mean([r["success_rate"]["avg"] for r in cell_runs])
                cpu = mean([r["cpu"]["percent"] for r in cell_runs])
                server_lost = sum(r["_simlog"]["lost_line_count"] for r in cell_runs)

                summary["cells"].append(
                    {
                        "variant": variant,
                        "condition": condition,
                        "client": client,
                        "reps": len(cell_runs),
                        "cycles": cycles,
                        "interval_us": interval,
                        "latency_us": latency,
                        "compute_us": compute,
                        "model_us": model,
                        "lost_cycles": lost_cycles,
                        "lost_states": lost_states,
                        "max_consecutive": max_consec,
                        "success_min": sr_min,
                        "success_avg": sr_avg,
                        "cpu_percent": cpu,
                        "server_lost_lines": server_lost,
                        "orders_in_cell": sorted(
                            r.get("order_in_cell", 0) for r in cell_runs
                        ),
                        "backwards_time_steps": sum(
                            r["lost"].get("backwards_time_steps", 0) for r in cell_runs
                        ),
                    }
                )
                if any("controller" in r for r in cell_runs):
                    summary["cells"][-1]["controller"] = {
                        "tau_max_abs": max(
                            r.get("controller", {}).get("tau_max_abs", 0.0)
                            for r in cell_runs
                        ),
                        "ee_deviation_max_m": max(
                            r.get("controller", {}).get("ee_deviation_max_m", 0.0)
                            for r in cell_runs
                        ),
                    }

                label = CONDITION_LABELS[condition]
                if variant == "control":
                    add(
                        f"| {label} | {client} | {fmt(cycles)} "
                        f"| {fmt(interval['p50'])} | {fmt(interval['p99'])} "
                        f"| {fmt(interval['p999'])} | {fmt(interval['max'])} "
                        f"| {fmt(lost_cycles, 1)} | {fmt(lost_states, 1)} | {max_consec} "
                        f"| {sr_min:.3f} | {sr_avg:.3f} | {fmt(cpu, 1)} | {server_lost} |"
                    )
                elif variant == "active":
                    add(
                        f"| {label} | {client} | {fmt(cycles)} "
                        f"| {fmt(interval['p50'])} | {fmt(interval['p99'])} "
                        f"| {fmt(interval['p999'])} | {fmt(interval['max'])} "
                        f"| {fmt(latency['p50'])} | {fmt(latency['p99'])} "
                        f"| {fmt(latency['p999'])} | {fmt(latency['max'])} "
                        f"| {fmt(lost_cycles, 1)} | {max_consec} "
                        f"| {sr_avg:.3f} | {fmt(cpu, 1)} |"
                    )
                else:
                    add(
                        f"| {label} | {client} | {fmt(cycles)} "
                        f"| {fmt(model['p50'], 1)} | {fmt(model['p99'], 1)} "
                        f"| {fmt(model['p999'], 1)} | {fmt(model['max'], 1)} "
                        f"| {fmt(compute['p50'], 1)} | {fmt(compute['p99'], 1)} "
                        f"| {fmt(compute['p999'], 1)} | {fmt(compute['max'], 1)} "
                        f"| {fmt(interval['p50'])} | {fmt(interval['p99'])} "
                        f"| {fmt(interval['max'])} "
                        f"| {fmt(lost_cycles, 1)} | {sr_avg:.3f} | {fmt(cpu, 1)} |"
                    )
        add("")
        if variant == "model":
            add(
                "Controller sanity check (worst over the repetitions of each cell): the "
                "peak commanded torque and how far the end effector ever moved from where "
                "it started. The two clients should agree; they run the same math."
            )
            add("")
            add("| condition | client | max abs tau (Nm) | max EE deviation (m) |")
            add("|---|---|---|---|")
            for condition in CONDITIONS:
                for client in CLIENTS:
                    cell_runs = cell(runs, client, variant, condition)
                    if not cell_runs:
                        continue
                    tau = max(
                        r.get("controller", {}).get("tau_max_abs", 0.0) for r in cell_runs
                    )
                    dev = max(
                        r.get("controller", {}).get("ee_deviation_max_m", 0.0)
                        for r in cell_runs
                    )
                    add(
                        f"| {CONDITION_LABELS[condition]} | {client} "
                        f"| {tau:.3f} | {dev:.4f} |"
                    )
            add("")

    hardware_runs = [r for r in runs if r.get("hardware")]
    if hardware_runs:
        stage_path = os.path.join(out_dir, "stage.json")
        stage = json.load(open(stage_path)) if os.path.exists(stage_path) else {}
        summary["hardware"] = hardware_section(add, hardware_runs, stage, meta)

    add("## Per-run detail")
    add("")
    add(
        "| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall "
        "| interval max (µs) | max at cycle | lost | back | sr avg |"
    )
    add("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for run in runs:
        sched = f"{run['sched']['policy']}/{run['sched']['priority']}"
        start = run.get("sched_at_start")
        sched_start = f"{start['policy']}/{start['priority']}" if start else "?"
        mlock = "ok" if run["mlockall"]["ok"] else (
            "failed" if run["mlockall"]["requested"] else "-"
        )
        order = run.get("order_in_cell", "?")
        first = run.get("cell_first_client", "?")
        add(
            f"| {run['_name']} | {order}/2, {first} first | {run['cycles']} "
            f"| {run['wall_s']:.2f} | {sched_start} "
            f"| {sched} | {mlock} | {run['interval_us']['max']:.0f} "
            f"| {run['interval_us'].get('max_at_cycle', '?')} | {run['lost']['cycles']} "
            f"| {run['lost'].get('backwards_time_steps', '?')} "
            f"| {run['success_rate']['avg']:.3f} |"
        )
    add("")

    add("## Server-side cross-check (`docker logs`)")
    add("")
    any_stats = False
    for run in runs:
        stats = run["_simlog"]["state_update_stats"]
        lost = run["_simlog"]["lost_lines"]
        if not stats and not lost:
            continue
        any_stats = True
        add(f"### {run['_name']}")
        add("")
        add("```")
        for line in stats[-4:]:
            add(line)
        for line in lost[:6]:
            add(line)
        if run["_simlog"]["lost_line_count"] > 6:
            add(f"... {run['_simlog']['lost_line_count'] - 6} more lost-cycle lines")
        add("```")
        add("")
    if not any_stats:
        add("The simulator emitted no `State Update Stats` or lost-cycle lines for these runs.")
        add("")

    cyclictest = os.path.join(out_dir, "cyclictest.txt")
    if os.path.exists(cyclictest):
        add("## OS baseline (cyclictest)")
        add("")
        add("```")
        with open(cyclictest, errors="replace") as handle:
            add(handle.read().strip())
        add("```")
        add("")

    with open(os.path.join(out_dir, "summary.json"), "w") as handle:
        json.dump(summary, handle, indent=2)

    print("\n".join(lines))


if __name__ == "__main__":
    main()
