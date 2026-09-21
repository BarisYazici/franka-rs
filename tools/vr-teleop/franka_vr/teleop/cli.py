"""The command line, and the startup order that keeps the node's limits authoritative.

The order matters and is the point of this file:

    parse           shape checks only -- ours against ours, and the two protocol shapes
    hand check      on a live bridge, before any Zenoh session exists
    sink.connect    the Zenoh session, its subscribers and publishers; NOTHING acquired yet
    load_limits     franka/<arm>/params/{schema,get}, or --node-limits FILE
    validate        every flag a node limit bounds, against THOSE numbers
    run             sink.start() acquires and enables

`--probe-limits` stops after `load_limits` and prints. It acquires nothing, so it is the
first command to run at the bench and the one that would have caught both of the drifts that
made this module necessary.

TUNABLES below is the single table behind `add_argument` and `teleop/params/schema`: the
flags and the bounds the panel shows cannot disagree because there is only one of them.
"""
import argparse
import json
import os
import signal
import sys

from . import hands, limits, node_limits, teleop_params
from .sinks import DryRunSink, ZenohSink
from .sources import ReplaySource, ZmqSource
from .teleop import Teleop

# The repository checkout's fixture; an installed wheel has none, so a dry run there needs
# --node-limits FILE.
DEFAULT_LIMITS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "fixtures", "node-limits.json"))


def _t(name, flag, default, lo, hi, help, **kw):
    t = dict(name=name, flag=flag, type=type(default), default=default, min=lo, max=hi,
             help=help, policy="step", group="teleop", settable=True)
    t.update(kw)
    return t


# Every flag that teleop/params may change, with the bounds the schema publishes. `bound`
# names the node `derived` entry a knob must stay under; teleop_params resolves it live.
TUNABLES = [
    _t("spatial_scale", "--spatial-scale", 0.4, 0.1, 2.0,
       "hand metres to arm metres (default 0.4: the measured 0.706 m/s peak hand speed maps "
       "to 0.28, inside the 0.3 m/s budget norm on a diagonal; one axis gets 0.173)",
       note="applies at the next clutch latch"),
    _t("rotation_scale", "--rotation-scale", 0.25, 0.1, 2.0,
       "hand radians to arm radians (default 0.25; why not 1.0: docs/design.md)",
       note="applies at the next clutch latch"),
    _t("clamp", "--clamp", 0.025, 0.005, 0.05,
       "m a published target may lead the MEASURED pose (default 0.025)",
       unit="m", bound="max_lead"),
    _t("clamp_rot", "--clamp-rot", 0.15, 0.02, 0.26, "rad, the same (default 0.15)",
       unit="rad", bound="max_lead_rotation"),
    _t("max_step", "--max-step", 0.04, 0.001, 0.05,
       "m between consecutive targets; the node refuses over its max_step",
       unit="m", bound="max_step"),
    _t("max_step_rot", "--max-step-rot", 0.20, 0.01, 0.26,
       "rad between consecutive targets; the node refuses over its max_step_rotation",
       unit="rad", bound="max_step_rotation"),
    _t("max_hand_step", "--max-hand-step", 0.05, 0.005, 0.5,
       "m of hand travel in one sample before we re-anchor instead of following "
       "(0.05 = 2.5 m/s at 50 Hz)", unit="m"),
    _t("reanchor_hold_ms", "--reanchor-hold-ms", 200.0, 0.0, 2000.0,
       "hold the clutch open this long after a re-anchor", unit="ms"),
    _t("rate", "--rate", 50.0, 10.0, 225.0, "targets per second",
       unit="Hz", bound="rate_hz"),
    _t("workspace_inset", "--workspace-inset", 0.005, 0.0, 0.1,
       "m the workspace box is shrunk by on every face", unit="m"),
    # Published so the panel's joint-headroom bar can compute, but not settable: the release
    # path is not tuned from the bus.
    _t("dq_release_fraction", None, limits.DQ_RELEASE_FRACTION, 0.3, 1.0,
       "fraction of a joint's velocity limit that releases the clutch", settable=False),
    _t("dq_resume_fraction", None, limits.DQ_RESUME_FRACTION, 0.1, 0.95,
       "fraction every joint must be under before a new squeeze drives", settable=False),
]


def parse(argv):
    p = argparse.ArgumentParser(
        prog="franka-vr-teleop", description="One Quest controller drives one franka-node arm.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", default="L", help="franka-node arm name (default L)")
    p.add_argument("--hand", choices=("left", "right"), default=None,
                   help="the controller that drives the arm, which picks the bridge port: "
                        "left 5560, right 5570 (default: L's is left, R's is right)")
    p.add_argument("--bridge-host", default="127.0.0.1", help="the Quest bridge's host")
    p.add_argument("--endpoint", default=None,
                   help="override the endpoint --hand picks; its port must be that hand's")
    p.add_argument("--cross", action="store_true",
                   help="allow a crossed pairing: arm L on the right controller, R on the left")
    p.add_argument("--skip-hand-check", action="store_true",
                   help="do not prove the hand on a live bridge before acquiring (warns)")
    p.add_argument("--hand-check-seconds", type=float, default=5.0,
                   help="how long each step of the hand check waits (default 5)")
    p.add_argument("--one-controller", action="store_true",
                   help="let the other hand's port be silent during the hand check (warns)")
    p.add_argument("--client-id", type=int, default=7200, help="non-zero u32, per process")
    p.add_argument("--connect", default="tcp/127.0.0.1:7447", help="a Zenoh endpoint")
    p.add_argument("--episode", default="", help="name the node's recording")
    p.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C or EOF")
    p.add_argument("--dry-run", action="store_true",
                   help="decode, map, clamp and print; no Zenoh at all")
    p.add_argument("--replay", default=None,
                   help="a record_wire.py .jsonl (or .jsonl.gz) instead of a live bridge")
    p.add_argument("--replay-speed", type=float, default=0.0,
                   help="0 = as fast as possible (default), 1 = real time")
    p.add_argument("--node-limits", default=None, metavar="FILE",
                   help="read the node's limits from a JSON file holding its params/schema "
                        "and params/get instead of from the node. THE OFFLINE PATH: a live "
                        "run says so loudly. A dry run in a checkout defaults to "
                        "fixtures/node-limits.json")
    p.add_argument("--probe-limits", action="store_true",
                   help="fetch and print this arm's limits and the headroom line, then exit "
                        "0 without acquiring anything. Run this first at the bench")
    for t in TUNABLES:
        if t["flag"]:
            p.add_argument(t["flag"], type=t["type"], default=t["default"], help=t["help"])
    p.add_argument("--source-timeout-ms", type=float, default=200.0,
                   help="stop the keepalive after this much bridge silence")
    p.add_argument("--gripper-hz", type=float, default=5.0, help="0 disables the rate limit")
    p.add_argument("--gripper-deadband", type=float, default=0.004,
                   help="m of commanded width change worth a message")
    p.add_argument("--gripper-max-width", type=float, default=0.08,
                   help="m, until franka/<arm>/gripper/state says otherwise")
    p.add_argument("--no-gripper", action="store_true", help="never touch the gripper")
    p.add_argument("--no-auto-recover", action="store_true",
                   help="a faulted arm ends the session (or refuses to start) instead of "
                        "being recovered")
    p.add_argument("--max-recoveries", type=int, default=5,
                   help="mid-session auto-recoveries before a fault ends the session "
                        "(default 5; the recovery before acquire is not counted)")
    p.add_argument("--dq-latch", action="store_true",
                   help="keep the joint-velocity release for the rest of the session")
    p.add_argument("--no-teleop-params", action="store_true",
                   help="do not serve franka/<arm>/teleop/params/* (the tuning panel's side)")
    p.add_argument("--workspace", default=None,
                   help="min xyz then max xyz, m. Off unless given, as the node's box now is: "
                        "this clamp used to mirror a node default that no longer exists, and a "
                        "mirror of someone else's limit is how they drift apart")
    p.add_argument("--poll-ms", type=int, default=5)
    p.add_argument("--summary-json", default=None,
                   help="write the exit summary as JSON as well")
    p.add_argument("--emit", default=None,
                   help="write one JSON line per published target and gripper command, "
                        "with the packed bytes; for the offline tests")
    p.add_argument("--no-canonicalise", action="store_true",
                   help="FOR THE TESTS ONLY: skip the quaternion sign fix of rule 4")
    p.add_argument("--print-every", type=int, default=50, help="0 = no target lines")
    p.add_argument("--quiet", action="store_true")
    a = p.parse_args(argv)
    if a.workspace is not None:
        box = [float(v) for v in a.workspace.split(",")]
        if len(box) != 6:
            p.error("--workspace wants six numbers")
        if not all(box[i] < box[i + 3] for i in range(3)):
            p.error("--workspace wants min below max on every axis")
        a.workspace = (tuple(box[0:3]), tuple(box[3:6]))
    limits.validate_shape(a, p.error)
    hands.resolve(a, p.error)
    a.hand_check = "not run"
    return a


def fail(message):
    """A startup refusal once argparse is out of the picture: exit 2, touching nothing."""
    print(f"franka-vr-teleop: error: {message}", file=sys.stderr, flush=True)
    raise SystemExit(2)


def load_limits(a, sink):
    """The node's limits, from the node or from --node-limits FILE. Fails closed: a node
    that cannot be reached, or whose schema cannot be read, ends the process here -- before
    `acquire` -- rather than falling back to a guess."""
    try:
        if a.node_limits or not sink.live:
            return node_limits.from_file(a.node_limits or DEFAULT_LIMITS)
        return sink.fetch_limits()
    except node_limits.LimitsError as exc:
        fail(f"{exc}\n  teleop refuses to start without the node's own limits. Pass "
             f"--node-limits FILE to run against a file instead, deliberately.")


def probe(a):
    """--probe-limits: connect, fetch, print, exit 0. Nothing is acquired or enabled."""
    sink = DryRunSink(a) if a.dry_run else ZenohSink(a)
    try:
        sink.connect()
        node = load_limits(a, sink)
    finally:
        sink.disconnect()
    print(f"[probe-limits] arm {a.arm} via {'--dry-run' if a.dry_run else a.connect}")
    print(limits.source_line(node))
    print(limits.headroom(a, node))
    print(f"[limits] live budgets: {node.budget:.3f} m/s, {node.budget_rotation:.3f} rad/s "
          f"(params/get version {node.version})")
    print("[limits] the derived block teleop validates against:")
    for line in json.dumps(node.derived, indent=2).splitlines():
        print(f"    {line}")
    lead, lead_rot = limits.lead_bounds(a, node)
    print(f"[limits] this node allows --clamp up to {lead:.4f} m and --clamp-rot up to "
          f"{lead_rot:.4f} rad; --max-step up to {node.max_step} m, --rate up to "
          f"{node.rate_hz * limits.KEEPALIVE_SLACK:.0f} Hz")
    print("[probe-limits] nothing was acquired; the arm was not touched")
    return 0


def main(argv=None):
    a = parse(argv)
    if a.probe_limits:
        return probe(a)
    print(hands.startup_line(a), flush=True)
    if a.replay:
        a.hand_check = "not run (replay)"
    elif a.skip_hand_check:
        a.hand_check = "SKIPPED (--skip-hand-check)"
        print("[hand-check] WARNING: skipped (--skip-hand-check); nothing has proved that the "
              f"controller on {a.endpoint} is the one in the operator's {a.hand} hand",
              flush=True)
    else:
        failure = hands.check_hands(a)                # before any Zenoh, so before acquire
        if failure:
            print(f"[hand-check] FAILED, nothing acquired: {failure}", flush=True)
            return 3
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))

    source = ReplaySource(a.replay, a.endpoint, a.replay_speed) if a.replay \
        else ZmqSource(a.endpoint)
    if a.replay:
        a.endpoint = source.endpoint
        print(f"[replay] {len(source.rows)} messages from {source.endpoint} "
              f"({a.replay})", flush=True)
    sink = DryRunSink(a) if a.dry_run else ZenohSink(a)
    sink.connect()                                   # a session, but nothing acquired yet
    try:
        node = load_limits(a, sink)
        limits.validate(a, node, fail)
    except SystemExit:                               # refused: drop the lease token and go
        sink.disconnect()
        source.close()
        raise
    print(limits.source_line(node), flush=True)
    if not a.quiet:
        print(limits.headroom(a, node), flush=True)
    owner = None
    if sink.live and not a.no_teleop_params:
        owner = teleop_params.TeleopParams(a, node, TUNABLES)
        sink.serve_params(owner)
    app = Teleop(a, source, sink, node, owner)
    try:
        app.run(lambda: bool(stop))
    finally:
        sink.stop()
        source.close()
        if app.emit_fh:
            app.emit_fh.close()
        if a.summary_json:
            with open(a.summary_json, "w") as fh:
                json.dump(app.facts(), fh, indent=1)
        app.summary()
    return 2 if app.abort else 0


if __name__ == "__main__":
    raise SystemExit(main())
