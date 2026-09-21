#!/usr/bin/env python3
"""A synthetic quest bridge: the exact 88-byte VrTargetMsg layout, at the real
rate, from no headset at all.

    python -m wire_tools.synth_wire --secs 70                 # healthy two-handed pick
    python -m wire_tools.synth_wire --gap-every 5             # the keepalive pathology
    python -m wire_tools.synth_wire --jump-at 20              # a SLAM re-localisation
    python -m wire_tools.synth_wire --replay run.jsonl        # re-emit a real recording

There is no other synthetic source: `franka-vr-bridge` has no
--replay and no --dry-run, it builds a real `OculusReader` before it binds
anything, and every existing test injects a fake reader in-process. So this is
the only way to exercise a consumer, or this recorder, without a Quest.

WHAT IT IMITATES, and where each choice comes from (the bridge it imitates is
`quest_bridge/`):

* the transport: PUB, bound (consumers connect), ZMQ_CONFLATE=1, one
  single-part frame of exactly 88 bytes, no topic envelope
  -- quest_bridge/bridge.py.
* the cadence: one message per channel per tick at --rate (50 Hz), and a `seq`
  incremented once per tick and SHARED by every channel -- quest_bridge/bridge.py.
  The loop never skips a publish, even through a gap.
* `pos`: metres, in the Quest's SLAM frame -- an arbitrary origin, only the
  AXIS DIRECTIONS are the robot's. Hence --origin, which by default sits
  nowhere near the robot's workspace box, exactly as a real headset's does.
* `quat`: xyzw. While the grip is released the mapper re-latches
  `vr_to_global = inv(pose)` every tick, so the published rotation is the
  CONSTANT quaternion of the axis-permutation matrix; only while the grip is
  held does it carry a rotation delta -- quest_bridge/mapping.py. That constant
  quaternion also FLIPS SIGN about half the time, because the rotation it
  encodes sits exactly on a branch boundary of
  `scipy.Rotation.from_matrix().as_quat()` and the float noise in
  `inv(pose) @ pose` decides which sign comes out (measured: 110 flips in 200
  disengaged ticks through the real mapper). --sign-flip-prob reproduces it, so
  a consumer that forgets to canonicalise the sign fails here rather than on an
  arm.
* `gripper`: the index trigger's closed fraction in [0,1]; it SNAPS to 0.0 on
  an unusable frame -- quest_bridge/bridge.py.
* the flags: engaged = grip AND fresh AND not forced-open; a gap publishes
  fresh=0, engaged=0, gripper=0 and the last pose verbatim, and the channel
  then stays forced open until ITS OWN grip is seen released
  -- quest_bridge/bridge.py.

Nothing here talks to adb, installs an APK, or broadcasts anything. Depends on
pyzmq and nothing else.
"""
import argparse
import json
import math
import random
import signal
import sys
import time

import zmq

from franka_vr.wire import (FLAG_CONTROLLER_ON, FLAG_ENGAGED, FLAG_FRESH,  # noqa: F401
                            VR_FMT, VR_MSG_MAGIC, VR_MSG_VERSION, pack_vr_target)

BUTTON_A, BUTTON_B, BUTTON_STICK = 1, 2, 4

# The quaternion of DEFAULT_RMAT_REORDER = [-3,-1,2,4], i.e. of
#   robot_x = -head_z, robot_y = -head_x, robot_z = +head_y
# which is what the mapper publishes on every tick the grip is released
# (rot = M @ pose @ inv(pose) = M). xyzw, unit, det(M) = +1.
DISENGAGED_QUAT = (0.5, -0.5, -0.5, 0.5)

_stop = False


def _on_signal(signum, _frame):
    global _stop
    _stop = True


pack = pack_vr_target        # the one codec; this module used to carry a copy


def qmul(a, b):
    """Hamilton product, both xyzw."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def axis_angle(axis, angle):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    s = math.sin(angle / 2.0) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


class Hand:
    """One controller doing a pick-and-place cycle, in SLAM-frame metres.

    The cycle is `period` seconds: approach (grip released), grip, descend,
    close the trigger, lift, traverse, open the trigger, release the grip,
    return. Peak hand speed is ~`reach`*4/period, which at the defaults is
    0.30 m/s -- a deliberately ordinary speed, not a flick.
    """

    def __init__(self, origin, period=8.0, reach=0.30, phase=0.0, rot_rate=0.5):
        self.origin = tuple(origin)
        self.period = float(period)
        self.reach = float(reach)
        self.phase = float(phase)
        self.rot_rate = float(rot_rate)   # rad/s of wrist roll while engaged
        self._engage_t = None

    def state(self, t):
        """(pos, grip_held, trigger) at time t."""
        u = ((t / self.period) + self.phase) % 1.0
        r = self.reach
        # x forward, y lateral, z up; a closed loop with a lift in the middle
        if u < 0.25:                      # approach, grip released
            s = u / 0.25
            pos = (r * s, 0.0, 0.10)
            return pos, False, 0.0
        if u < 0.40:                      # engaged, descend to the object
            s = (u - 0.25) / 0.15
            pos = (r, 0.0, 0.10 - 0.08 * s)
            return pos, True, 0.0
        if u < 0.50:                      # close the fingers
            s = (u - 0.40) / 0.10
            return (r, 0.0, 0.02), True, s
        if u < 0.60:                      # lift
            s = (u - 0.50) / 0.10
            return (r, 0.0, 0.02 + 0.15 * s), True, 1.0
        if u < 0.80:                      # traverse sideways
            s = (u - 0.60) / 0.20
            return (r, -r * s, 0.17), True, 1.0
        if u < 0.90:                      # open the fingers
            s = (u - 0.80) / 0.10
            return (r, -r, 0.17), True, 1.0 - s
        s = (u - 0.90) / 0.10             # grip released, return to the start
        return (r * (1.0 - s), -r * (1.0 - s), 0.17 - 0.07 * s), False, 0.0

    def sample(self, t):
        pos, grip, trig = self.state(t)
        if grip:
            if self._engage_t is None:
                self._engage_t = t
            quat = qmul(axis_angle((0.0, 0.0, 1.0),
                                   self.rot_rate * (t - self._engage_t)),
                        DISENGAGED_QUAT)
        else:
            self._engage_t = None
            quat = DISENGAGED_QUAT
        return (tuple(o + p for o, p in zip(self.origin, pos)), quat, grip, trig)


class Channel:
    """One (controller, endpoint) pair, with the bridge's own gap and
    forced-open behaviour so a consumer sees the real state machine."""

    def __init__(self, ctx, endpoint, hand, name, sign_flip_prob=0.5, seed=0):
        self.sock = ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.CONFLATE, 1)   # before bind, as the bridge does
        self.sock.bind(endpoint)
        self.endpoint = self.sock.getsockopt_string(zmq.LAST_ENDPOINT)
        self.hand = hand
        self.name = name
        self.forced_open = True                 # all channels start latched
        self.last_pos = (0.0, 0.0, 0.0)
        self.last_quat = DISENGAGED_QUAT
        self.sign_flip_prob = float(sign_flip_prob)
        self._rng = random.Random(seed)

    def tick(self, seq, t, *, gapped, frozen, jump):
        pos, quat, grip, trig = self.hand.sample(t)
        if frozen:                              # stale-but-publishing: pose held
            pos, quat = self.last_pos, self.last_quat
        pos = tuple(p + j for p, j in zip(pos, jump)) if any(jump) else pos
        if not grip and self._rng.random() < self.sign_flip_prob:
            quat = tuple(-c for c in quat)      # see the module docstring
        if gapped:
            # quest_bridge/bridge.py: last pose, gripper 0.0, every flag clear.
            payload = pack(seq, self.last_pos, self.last_quat, 0.0,
                           engaged=False, fresh=False, controller_on=False)
            self.forced_open = True
        else:
            fresh = not frozen
            if not fresh:
                self.forced_open = True
            elif not grip:
                self.forced_open = False        # its own grip released: re-arm
            self.last_pos, self.last_quat = pos, quat
            payload = pack(seq, pos, quat, trig,
                           engaged=bool(grip and fresh) and not self.forced_open,
                           fresh=fresh, controller_on=True)
        self.sock.send(payload, flags=zmq.NOBLOCK)
        return payload

    def close(self):
        self.sock.close(0)


def replay(args) -> int:
    """Re-emit a recording's raw bytes on their recorded cadence."""
    ctx = zmq.Context()
    socks = {}
    rows = []
    with open(args.replay) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue                        # a truncated tail line
            if rec.get("type") or "raw" not in rec:
                continue
            rows.append((rec["t"], rec.get("ep", args.endpoint[0]), rec["raw"]))
    if not rows:
        print(f"[synth] {args.replay}: no messages", file=sys.stderr)
        return 2
    rows.sort(key=lambda r: r[0])
    remap = dict(zip(sorted({r[1] for r in rows}), args.endpoint)) \
        if args.endpoint else {}
    for _, ep, _ in rows:
        ep = remap.get(ep, ep)
        if ep not in socks:
            s = ctx.socket(zmq.PUB)
            s.setsockopt(zmq.CONFLATE, 1)
            s.bind(ep)
            socks[ep] = s
    print(f"[synth] replaying {len(rows)} messages from {args.replay} on "
          f"{', '.join(socks)}", file=sys.stderr)
    time.sleep(0.3)                             # let subscribers attach
    t_wall0 = time.perf_counter_ns()
    t_rec0 = rows[0][0]
    n = 0
    for t_rec, ep, raw in rows:
        if _stop:
            break
        due = t_wall0 + int((t_rec - t_rec0) / args.speed)
        delay = (due - time.perf_counter_ns()) / 1e9
        if delay > 0:
            time.sleep(delay)
        socks[remap.get(ep, ep)].send(bytes.fromhex(raw), flags=zmq.NOBLOCK)
        n += 1
    print(f"[synth] replayed {n} messages", file=sys.stderr)
    for s in socks.values():
        s.close(0)
    ctx.term()
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--endpoint", action="append", default=None,
                   help="PUB bind endpoint, one per channel "
                        "(default: 5560 and 5570)")
    p.add_argument("--controller", default="l,r",
                   help="channel names, comma list, one per --endpoint")
    p.add_argument("--rate", type=float, default=50.0, help="Hz (the bridge's 50)")
    p.add_argument("--secs", type=float, default=0.0, help="0 = until Ctrl-C")
    p.add_argument("--period", type=float, default=8.0,
                   help="seconds per pick-and-place cycle")
    p.add_argument("--reach", type=float, default=0.30,
                   help="metres of hand travel per leg")
    p.add_argument("--rot-rate", type=float, default=0.5,
                   help="rad/s of wrist roll while engaged")
    p.add_argument("--origin", default="1.4,-0.7,0.9",
                   help="the SLAM origin offset the hand moves about, metres; "
                        "deliberately outside the node's workspace box")
    p.add_argument("--gap-every", type=float, default=0.0,
                   help="seconds between injected whole-frame gaps "
                        "(0 = healthy; 5 reproduces the keepalive "
                        "signature)")
    p.add_argument("--gap-ms", type=float, default=120.0, help="gap length, ms")
    p.add_argument("--freeze-at", type=float, default=-1.0,
                   help="freeze the pose (stale-but-publishing) at this second")
    p.add_argument("--freeze-ms", type=float, default=600.0)
    p.add_argument("--jump-at", type=float, default=-1.0,
                   help="apply a rigid SLAM re-localisation at this second")
    p.add_argument("--jump-m", type=float, default=0.65,
                   help="metres of that jump (0.652 was the measured one)")
    p.add_argument("--sign-flip-prob", type=float, default=0.5,
                   help="probability that a disengaged message carries the "
                        "antipodal (negated) quaternion, as the real bridge "
                        "does; 0 disables")
    p.add_argument("--replay", default=None, help="re-emit a recording instead")
    p.add_argument("--speed", type=float, default=1.0, help="--replay speed-up")
    args = p.parse_args(argv)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    if args.replay:
        return replay(args)

    endpoints = args.endpoint or ["tcp://127.0.0.1:5560", "tcp://127.0.0.1:5570"]
    names = [s.strip() for s in args.controller.split(",") if s.strip()]
    if len(names) != len(endpoints):
        names = [f"ch{i}" for i in range(len(endpoints))]
    origin = [float(v) for v in args.origin.split(",")]

    ctx = zmq.Context()
    chans = []
    for i, (ep, nm) in enumerate(zip(endpoints, names)):
        hand = Hand(origin, period=args.period, reach=args.reach,
                    phase=0.5 * i, rot_rate=args.rot_rate)
        chans.append(Channel(ctx, ep, hand, nm,
                             sign_flip_prob=args.sign_flip_prob, seed=1 + i))
        print(f"[synth] ready. endpoint={chans[-1].endpoint} controller={nm} "
              f"rate={args.rate}Hz", file=sys.stderr)

    period_ns = int(1e9 / args.rate)
    gap_len = args.gap_ms / 1000.0
    t0 = time.perf_counter_ns()
    seq = 0
    next_tick = t0
    sent = 0
    try:
        while not _stop:
            now = time.perf_counter_ns()
            if args.secs > 0 and (now - t0) / 1e9 >= args.secs:
                break
            if now < next_tick:
                time.sleep(min(0.005, (next_tick - now) / 1e9))
                continue
            t = (now - t0) / 1e9
            gapped = (args.gap_every > 0
                      and (t % args.gap_every) < gap_len and t > args.gap_every)
            frozen = (args.freeze_at >= 0
                      and args.freeze_at <= t < args.freeze_at + args.freeze_ms / 1000.0)
            jump = (args.jump_m, 0.0, 0.0) if (args.jump_at >= 0 and t >= args.jump_at) \
                else (0.0, 0.0, 0.0)
            seq += 1
            for ch in chans:
                ch.tick(seq, t, gapped=gapped, frozen=frozen, jump=jump)
                sent += 1
            next_tick += period_ns
            if next_tick < now - 5 * period_ns:   # fell far behind; resynchronise
                next_tick = now + period_ns
    finally:
        for ch in chans:
            ch.close()
        ctx.term()
        print(f"[synth] sent {sent} messages ({seq} ticks) in "
              f"{(time.perf_counter_ns() - t0) / 1e9:.1f} s", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
