"""Deterministic synthetic wire FILES, in record_wire.py's .jsonl format, with no socket,
no headset and no wall-clock wait. From tools/vr-teleop:

    python -m tests.teleop.make_wire --out w.jsonl --secs 24          # a clean two-handed pick
    python -m tests.teleop.make_wire --out w.jsonl --secs 24 --gap-every 5    # keepalive
    python -m tests.teleop.make_wire --out w.jsonl --secs 24 --hand-jump-at 12 --hand-jump-m 0.0777
    python -m tests.teleop.make_wire --out w.jsonl --secs 24 --flip-engaged   # rule 4, driving

`synth_wire.py` is the authority on what the bridge puts on the wire, and it only publishes
over ZMQ. This drives the SAME `synth_wire.Channel` -- its gap, freeze, forced-open and
sign-flip state machine, unmodified -- through a context object that hands out a socket which
keeps the payload instead of sending it, stamps each tick at an exact period, and decodes
every field with `record_wire.decode` so the file is one an analyser can read too. Nothing
here binds a port, so it cannot collide with a running bridge.

Two injections synth_wire does not model, both needed by the tests:
  --hand-jump-at/-m  one tick's position displaced and then back: a 77.7 mm one-tick step, the
                     size a still controller can report (fresh -> fresh, so no freshness edge
                     marks it)
  --flip-engaged     the antipodal quaternion on alternate ENGAGED ticks; synth_wire flips
                     only while the grip is released, where the real bridge flips about a
                     quarter of the ticks
"""
import argparse
import json
import sys
import time

from wire_tools import record_wire, synth_wire

VR_FMT = synth_wire.VR_FMT


class _Sock:
    """Everything `synth_wire.Channel` asks of a PUB socket, and nothing more."""

    def __init__(self):
        self.last = None

    def setsockopt(self, *_a):
        pass

    def bind(self, _ep):
        pass

    def getsockopt_string(self, _opt):
        return "file://"

    def send(self, payload, flags=0):
        self.last = payload

    def close(self, *_a):
        pass


class _Ctx:
    def socket(self, _kind):
        return _Sock()


def rewrite(payload, dpos=None, flip=False):
    f = list(VR_FMT.unpack(payload))        # magic, ver, seq, p0..p2, q0..q3, grip, flags, btn
    if dpos:
        f[3], f[4], f[5] = f[3] + dpos[0], f[4] + dpos[1], f[5] + dpos[2]
    if flip:
        f[6:10] = [-c for c in f[6:10]]
    return VR_FMT.pack(*f)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", required=True)
    p.add_argument("--endpoint", default="tcp://127.0.0.1:5560",
                   help="the left controller's port, arm L's pairing (default)")
    p.add_argument("--secs", type=float, default=24.0)
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--period", type=float, default=8.0, help="seconds per pick cycle")
    p.add_argument("--reach", type=float, default=0.30)
    p.add_argument("--rot-rate", type=float, default=0.5)
    p.add_argument("--origin", default="1.4,-0.7,0.9")
    p.add_argument("--sign-flip-prob", type=float, default=0.5)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--gap-every", type=float, default=0.0)
    p.add_argument("--gap-ms", type=float, default=120.0)
    p.add_argument("--freeze-at", type=float, default=-1.0)
    p.add_argument("--freeze-ms", type=float, default=600.0)
    p.add_argument("--jump-at", type=float, default=-1.0, help="a permanent rigid offset")
    p.add_argument("--jump-m", type=float, default=0.65)
    p.add_argument("--hand-jump-at", type=float, default=-1.0,
                   help="one tick displaced by --hand-jump-m in x, then back")
    p.add_argument("--hand-jump-m", type=float, default=0.0777)
    p.add_argument("--flip-engaged", action="store_true")
    p.add_argument("--note", default="")
    a = p.parse_args(argv)

    hand = synth_wire.Hand([float(v) for v in a.origin.split(",")], period=a.period,
                           reach=a.reach, rot_rate=a.rot_rate)
    ch = synth_wire.Channel(_Ctx(), a.endpoint, hand, "synth",
                            sign_flip_prob=a.sign_flip_prob, seed=a.seed)
    period_ns = int(1e9 / a.rate)
    t0 = 1_000_000_000
    gap_len = a.gap_ms / 1000.0
    ticks = int(a.secs * a.rate)
    engaged_seen = 0
    with open(a.out, "w") as fh:
        fh.write(json.dumps({
            "type": "header", "tool": "make_wire.py", "record_version": 1, "note": a.note,
            "endpoints": [a.endpoint], "conflate": False, "struct": "<IIQ8dII",
            "msg_size": synth_wire.VR_FMT.size, "magic": synth_wire.VR_MSG_MAGIC,
            "msg_version": synth_wire.VR_MSG_VERSION, "t0_perf_ns": t0,
            "t0_wall_ns": time.time_ns(), "secs": a.secs, "argv": sys.argv[1:],
        }) + "\n")
        for i in range(ticks):
            t = i / a.rate
            gapped = (a.gap_every > 0 and (t % a.gap_every) < gap_len and t > a.gap_every)
            frozen = (a.freeze_at >= 0
                      and a.freeze_at <= t < a.freeze_at + a.freeze_ms / 1000.0)
            jump = (a.jump_m, 0.0, 0.0) if (a.jump_at >= 0 and t >= a.jump_at) \
                else (0.0, 0.0, 0.0)
            payload = ch.tick(i + 1, t, gapped=gapped, frozen=frozen, jump=jump)
            engaged = bool(VR_FMT.unpack(payload)[11] & synth_wire.FLAG_ENGAGED)
            dpos = None
            if a.hand_jump_at >= 0 and abs(t - a.hand_jump_at) < 0.5 / a.rate:
                dpos = (a.hand_jump_m, 0.0, 0.0)
            flip = a.flip_engaged and engaged and (i % 2 == 0)
            if dpos or flip:
                payload = rewrite(payload, dpos, flip)
            engaged_seen += engaged
            t_ns = t0 + i * period_ns
            rec = {"t": t_ns, "w": t_ns, "ep": a.endpoint, "raw": payload.hex()}
            rec.update(record_wire.decode(payload))
            fh.write(json.dumps(rec) + "\n")
        fh.write(json.dumps({"type": "footer", "total": ticks,
                             "per_endpoint": {a.endpoint: ticks}, "bad": {},
                             "elapsed_s": a.secs, "stopped_by": "deadline"}) + "\n")
    print(f"[make_wire] {ticks} messages, {engaged_seen} engaged, {a.secs:.0f} s "
          f"at {a.rate:.0f} Hz -> {a.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
