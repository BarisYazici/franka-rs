"""One run as data (`facts`) and as the block a human reads at the end (`summary`).

Both take the live `Teleop`. Every node number they print comes from the run's `NodeLimits`,
so the summary cannot quote a budget the arm is not actually using.
"""
import sys

from . import hands, limits

REASONS = ("decode", "sanity_pos", "sanity_quat", "not_fresh", "not_armed", "dq_release",
           "no_state", "reanchor", "hand_jump", "hold_open", "released")
CLAMPS = ("workspace", "lead", "lead_rot", "step", "step_rot")


def facts(app):
    """Everything the summary prints, as data: for --summary-json and the tests."""
    a, node = app.a, app.node
    span = (app.last_msg_ns - app.t0) / 1e9 if app.last_msg_ns else 0.0
    counters = app.sink.counters()
    return {
        "arm": a.arm, "endpoint": a.endpoint, "client_id": a.client_id,
        "hand": a.hand, "port": hands.port_of(a.endpoint),
        "crossed": a.crossed, "hand_check": a.hand_check,
        "node_limits": node.as_facts(),
        "dq_releases": [{"t_s": t, "joint": j, "dq": v, "limit": m}
                        for t, j, v, m in app.dq_releases],
        "worst_dq": list(app.worst_dq) if app.worst_dq else None,
        "recoveries": [{"t_s": t, "reason": r, "seconds": s, "failure": f}
                       for t, r, s, f in app.recoveries],
        "gripper_state": app.sink.gripper_info(),
        "wire_in": app.n_in, "span_s": span,
        "out": app.n_out, "driven": app.n_out - app.n_keepalive - app.n_hold,
        "keepalive": app.n_keepalive, "dq_holds": app.n_hold, "gripper": app.n_gripper,
        "engagements": app.n_engage, "reanchors": app.n_reanchor,
        "refusals": dict(app.refusals), "clamps": dict(app.clamps),
        "worst_lead_m": app.worst_lead, "worst_lead_rot_rad": app.worst_lead_rot,
        "worst_lead_desired_m": app.worst_lead_desired,
        "worst_step_m": app.worst_step, "worst_step_rot_rad": app.worst_step_rot,
        "worst_hand_step_m": app.worst_hand_step,
        "worst_hand_speed": app.worst_hand_speed,
        "worst_cmd_speed": app.worst_cmd_speed,
        "worst_hand_rot_speed": app.worst_hand_rot_speed,
        "worst_cmd_rot_speed": app.worst_cmd_rot_speed,
        "worst_pub_gap_ms": app.worst_pub_gap / 1e6,
        "node": {"accepted": counters[0], "refused": counters[1],
                 "dropped": counters[2], "success_rate": counters[3]} if counters else None,
        "ended_because": app.abort,
        "options": {"spatial_scale": a.spatial_scale, "rotation_scale": a.rotation_scale,
                    "clamp": a.clamp, "clamp_rot": a.clamp_rot, "max_step": a.max_step,
                    "max_step_rot": a.max_step_rot, "max_hand_step": a.max_hand_step,
                    "rate": a.rate, "gripper_hz": a.gripper_hz,
                    "gripper_max_width": app.sink.max_width,
                    "workspace": None if a.workspace is None
                                 else [list(a.workspace[0]), list(a.workspace[1])],
                    "anchor_enabled": app.anchor_enabled},
    }


def summary(app, out=None):
    a, node = app.a, app.node
    w = lambda line: print(line, file=out or sys.stdout)
    span = (app.last_msg_ns - app.t0) / 1e9 if app.last_msg_ns else 0.0
    w("")
    w("=" * 72)
    w(f"vr-teleop summary   {hands.startup_line(a)}   client {a.client_id}")
    w(f"  hand check        {a.hand_check}")
    w(f"  node limits       {node.source}"
      + (f", arm {node.arm} boot {node.boot_id}" if node.from_node else ""))
    w(f"  wire              {app.n_in} messages over {span:.1f} s "
      f"({app.n_in / span if span else 0:.1f} Hz)")
    w(f"  targets out       {app.n_out}  ({app.n_out - app.n_keepalive - app.n_hold} "
      f"driven, {app.n_keepalive} keepalive, {app.n_hold} dq hold)   gripper commands "
      f"{app.n_gripper}")
    w(f"  clutch            {app.n_engage} engagements, {app.n_reanchor} re-anchors")
    g = app.sink.gripper_info()
    if g is not None:
        w(f"  gripper state     {g['states']} received ({g['bad_states']} malformed), "
          f"max_width {g['max_width'] * 1000:.1f} mm, flags {g['flags']}")
    for t, reason, took, failure in app.recoveries:
        w(f"  recovery          at {t:6.2f}s, {took:.1f} s: {reason} -> "
          f"{failure or 'Active again'}")
    w("  not commanded, by reason")
    for k in REASONS:
        if app.refusals.get(k):
            w(f"      {k:<12s} {app.refusals[k]:6d}")
    if not any(app.refusals.values()):
        w("      (none)")
    w("  clamped, by guard")
    for k in CLAMPS:
        if app.clamps[k]:
            w(f"      {k:<12s} {app.clamps[k]:6d}")
    if not any(app.clamps.values()):
        w("      (none)")
    w(f"  worst lead        {app.worst_lead * 1000:7.1f} mm vs measured "
      f"(clamp {a.clamp * 1000:.0f}), {app.worst_lead_rot:.3f} rad "
      f"(clamp {a.clamp_rot:.2f})")
    w(f"                    {app.worst_lead_desired * 1000:7.1f} mm vs the loop's "
      f"desired pose")
    w(f"  worst step        {app.worst_step * 1000:7.1f} mm between targets "
      f"(node max_step {node.max_step * 1000:.0f}), {app.worst_step_rot:.3f} rad "
      f"(node {node.max_step_rotation:.2f})")
    w(f"  worst hand step   {app.worst_hand_step * 1000:7.1f} mm in one sample "
      f"(re-anchor gate {a.max_hand_step * 1000:.0f})")
    w(f"  worst speed       hand  {app.worst_hand_speed:6.3f} m/s   -> commanded "
      f"{app.worst_cmd_speed:6.3f} m/s   at scale {a.spatial_scale} "
      f"(budget {node.budget:.3f})")
    w(f"                    wrist {app.worst_hand_rot_speed:6.3f} rad/s -> commanded "
      f"{app.worst_cmd_rot_speed:6.3f} rad/s at scale {a.rotation_scale} "
      f"(budget {node.budget_rotation:.3f})")
    w(f"  worst target gap  {app.worst_pub_gap / 1e6:7.1f} ms between published "
      f"targets (node holds at 200, stops at {node.stop_after_ms:.0f})")
    if app.worst_dq:
        w(f"  joint speed       worst {app.worst_dq[0] * 100:5.1f} % of its limit (joint "
          f"{app.worst_dq[1]}); release at {limits.DQ_RELEASE_FRACTION * 100:.0f} %")
        for t, j, v, m in app.dq_releases:
            w(f"      RELEASED at {t:6.2f}s: joint {j} |dq| {v:.3f} rad/s, limit {m}")
    else:
        w("  joint speed       no measured dq (the dry run's stand-in arm has no joints)")
    counters = app.sink.counters()
    if counters:
        w(f"  node counters     accepted {counters[0]} refused {counters[1]} "
          f"dropped {counters[2]} success {counters[3]:.3f}")
    if app.abort:
        w(f"  ENDED BECAUSE     {app.abort}")
    w("=" * 72)
