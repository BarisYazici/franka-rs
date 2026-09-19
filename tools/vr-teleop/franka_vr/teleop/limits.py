"""What franka-node will refuse, and the margin teleop.py keeps against each of it.

Pure functions of `(args, NodeLimits)`. Nothing here is a copy of one of the node's numbers:
they arrive from `node_limits.py`, which reads them off the node's own `params/schema` and
`params/get`. The constants that remain are the client's own, plus the two key-space shapes
the node's protocol fixes.

Every flag of ours that a node limit bounds -- rather than another flag of ours -- is
validated against it at STARTUP and before anything is acquired, because a refused target is
not followed: the arm holds its last ACCEPTED target (`holding` after 200 ms without one),
and the next step is measured from that target too, so a run of refusals stalls the arm
under a hand that keeps moving. It no longer kills the session: `stop_after` counts a target
refused on its content as a live commander. Real silence still stops it after `stop_after_ms`,
and that stop runs synchronously and can freeze state publication for seconds, which is what
STATE_STALE_MS below guards.
"""
import math
import re

from . import node_limits

# Protocol shapes, not tuning: the node's key space accepts these names and no others.
ARM_NAME = re.compile(r"^[A-Za-z0-9_-]+$")      # and neither `node` nor `cam`
EPISODE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,128}$")

# Ours. Nothing on the node's torque path bounds joint velocity, so teleop.py releases the
# clutch when the MEASURED StateMsg.dq of any joint passes the release fraction of its limit
# (the limit itself comes from the node's `derived.dq_limit`, which follows the connected
# arm's FCI version), and lets a new squeeze drive again once every joint is under the resume
# fraction.
DQ_RELEASE_FRACTION = 0.85
DQ_RESUME_FRACTION = 0.5

# How old the arm's state may be. The node publishes at its `state_hz`; past STATE_STALE_MS
# (five ticks at 100 Hz) nothing is driven from it -- not the clutch, the lead clamp or the dq
# release -- and past STATE_LOST_MS the session ends through the clean stop and release.
STATE_STALE_MS = 50.0
STATE_LOST_MS = 500.0

# The keepalive fires at this fraction of the nominal period: the bridge's p50 inter-arrival
# is 19.999 ms against a 20 ms period, so an exact comparison halves the keepalive rate. It
# also means the peak instantaneous publish rate is --rate / this.
KEEPALIVE_SLACK = 0.9


def worst_joint(dq, limit):
    """(index, |dq_i| / limit_i) of the joint nearest its velocity limit; a non-finite
    velocity counts as infinitely far past it. `limit` is the node's `derived.dq_limit`."""
    ratios = [abs(v) / m if math.isfinite(v) else math.inf for v, m in zip(dq, limit)]
    i = max(range(len(ratios)), key=ratios.__getitem__)
    return i, ratios[i]


def staleness(a, node):
    """How far the arm can have moved since the `StateMsg` our clamp references: one state
    tick at the arm's own budget. It is the gap between the lead WE measure and the lead the
    node measures, so the clamp has to leave room for it -- the reviewer's ~3 mm."""
    tick = 1.0 / node.state_hz
    return node.budget * tick, node.budget_rotation * tick


def lead_bounds(a, node):
    """The largest legal --clamp and --clamp-rot under this node's limits."""
    stale, stale_rot = staleness(a, node)
    return node.max_lead - stale, node.max_lead_rotation - stale_rot


def validate_shape(a, fail):
    """The checks that need no node: ours against ours, and the two protocol shapes. These
    run in parse(), so a typo never reaches a Zenoh session."""
    if not ARM_NAME.match(a.arm) or a.arm in ("node", "cam"):
        fail(f"--arm {a.arm!r} is not a franka-node arm name ([A-Za-z0-9_-]+, and neither "
             f"'node' nor 'cam')")
    if a.episode and not EPISODE_NAME.match(a.episode):
        fail(f"--episode {a.episode!r} is not [A-Za-z0-9_-]{{1,128}}: the node refuses "
             f"another shape and starts NOTHING")
    if not 1.0 <= a.client_id <= 0xFFFFFFFF:
        fail("--client-id is a non-zero u32")
    if not 0.0 < a.gripper_max_width <= 0.2:
        fail("--gripper-max-width outside (0, 0.2] m (the Franka Hand's stroke is 0.08; the "
             "gripper's own state overrides this once it publishes one)")
    if not 0.0 < a.spatial_scale <= 2.0:
        fail("--spatial-scale outside (0, 2]")
    if not 0.0 < a.rotation_scale <= 2.0:
        fail("--rotation-scale outside (0, 2]")
    if a.max_hand_step <= 0.0:
        fail("--max-hand-step must be positive")
    if a.reanchor_hold_ms < 0.0:
        fail("--reanchor-hold-ms must be zero or positive")
    if a.max_recoveries < 0:
        fail("--max-recoveries must be zero or positive")
    if a.gripper_deadband < 0.0:
        fail("--gripper-deadband must be zero or positive")
    if a.workspace is not None:
        lo, hi = a.workspace
        if any(l + a.workspace_inset >= h - a.workspace_inset for l, h in zip(lo, hi)):
            fail(f"--workspace is empty on some axis once inset by {a.workspace_inset} m")


def validate(a, node, fail):
    """Refuse, before anything is acquired, anything THIS node would refuse at run time.
    Each check names the limit it came from and where that limit was read."""
    stale, stale_rot = staleness(a, node)
    lead, lead_rot = lead_bounds(a, node)
    if not 0.0 < a.clamp <= lead:
        fail(f"--clamp outside (0, {lead:.4f}] m: {node.source} refuses a lead over max_lead "
             f"{node.max_lead} m and our reference is up to {stale * 1000:.1f} mm stale, so a "
             f"larger clamp refuses under a shove")
    if not 0.0 < a.clamp_rot <= lead_rot:
        fail(f"--clamp-rot outside (0, {lead_rot:.4f}] rad: {node.source} refuses a lead over "
             f"max_lead_rotation {node.max_lead_rotation} rad and our reference is up to "
             f"{stale_rot:.4f} rad stale")
    if not 0.0 < a.max_step <= node.max_step:
        fail(f"--max-step outside (0, {node.max_step}] m, max_step from {node.source}")
    if not 0.0 < a.max_step_rot <= node.max_step_rotation:
        fail(f"--max-step-rot outside (0, {node.max_step_rotation}] rad, max_step_rotation "
             f"from {node.source}")
    if not 0.0 < a.rate <= node.rate_hz * KEEPALIVE_SLACK:
        fail(f"--rate outside (0, {node.rate_hz * KEEPALIVE_SLACK:.0f}] Hz: we publish at up "
             f"to --rate / {KEEPALIVE_SLACK} and rate_hz from {node.source} is "
             f"{node.rate_hz:.0f}")
    if not 0.0 < a.source_timeout_ms <= node.stop_after_ms:
        fail(f"--source-timeout-ms outside (0, {node.stop_after_ms:.0f}], stop_after_ms from "
             f"{node.source}: past it the node has already stopped the session")


def source_line(node):
    """The loudest line of the startup: where these numbers came from. A file is the
    deliberate offline path and never a fallback, so it says so in capitals."""
    if node.from_node:
        return (f"[limits] from the node: arm {node.arm}, boot {node.boot_id}, "
                f"params version {node.version}")
    return (f"[limits] WARNING: from {node.source}, NOT from the node. Nothing has checked "
            f"these against the arm you are about to move; a stale file is the drift this "
            f"guards against. Run --probe-limits against the node to compare")


def headroom(a, node):
    """The one line the operator should read before an arm moves."""
    stale, stale_rot = staleness(a, node)
    return (f"[limits] clamp {a.clamp * 1000:.1f} mm + {stale * 1000:.1f} mm state staleness "
            f"vs max_lead {node.max_lead * 1000:.0f} mm "
            f"({node.max_lead / (a.clamp + stale):.1f}x); "
            f"rot {a.clamp_rot:.3f} + {stale_rot:.3f} vs {node.max_lead_rotation:.2f} rad "
            f"({node.max_lead_rotation / (a.clamp_rot + stale_rot):.1f}x). "
            f"A refused target at the node is not followed: the arm holds")


def adopt_budgets(a, node, body):
    """What a `params/current` body means for this run: the NodeLimits it implies, and the
    knobs it forces down. Pure -- the caller writes them, on the loop thread.

    Returns `(None, ())` when nothing moved, else `(node, ((flag, value, unit), ...))`.
    Raises `node_limits.LimitsError` on a body that cannot be read.
    """
    budget, budget_rotation = node_limits.budgets_from(body)
    boot = str(body.get("boot_id", node.boot_id))
    if (budget, budget_rotation, boot) == (node.budget, node.budget_rotation, node.boot_id):
        return None, ()
    node = node._replace(budget=budget, budget_rotation=budget_rotation, boot_id=boot,
                         version=int(body.get("version", node.version)))
    lead, lead_rot = lead_bounds(a, node)
    lowered = [(flag, bound, unit)
               for flag, bound, unit in (("clamp", lead, "m"), ("clamp_rot", lead_rot, "rad"))
               if getattr(a, flag) > bound]
    return node, tuple(lowered)
