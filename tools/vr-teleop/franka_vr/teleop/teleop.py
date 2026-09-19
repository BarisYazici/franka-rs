"""One Quest controller drives one franka-node arm: the 88-byte VrTargetMsg in, the node's
80-byte TargetMsg on Zenoh out, with a guard for every failure the measurements found.

    franka-vr-teleop --arm L --probe-limits --connect tcp/127.0.0.1:7447   # first, at the bench
    franka-vr-teleop --arm R --replay fixtures/desk-idle.jsonl.gz --dry-run
    franka-vr-teleop --arm L --client-id 7200 --connect tcp/127.0.0.1:7447 --seconds 60

The seven rules this implements, the measurement each comes from, the two scale tables and
the hand-to-port pairing are in `docs/design.md`. The command line and the startup order are
in `cli.py`; what the node will refuse, and where those numbers come from, in `limits.py`
and `node_limits.py`. Nothing here holds a copy of one of the node's limits.

The node's TargetMsg and GripperMsg bytes are franka-node-client's (`franka_node._wire`), not
a copy. A live bridge needs pyzmq and a live arm eclipse-zenoh; --replay --dry-run opens
neither.
"""
import json
import math
import time

from franka_node import _wire

from . import limits, node_limits
from . import summary as summary_mod
from .limits import (DQ_RELEASE_FRACTION, DQ_RESUME_FRACTION, KEEPALIVE_SLACK,
                    STATE_LOST_MS, STATE_STALE_MS, worst_joint)
from .geometry import Clutch, dist, qangle, qcanon, qlen, qslerp, qunit
from .sinks import fmt3
from .sources import FLAG_ENGAGED, FLAG_FRESH, decode_vr
from .summary import CLAMPS, REASONS


# TargetMsg.flags bit 0 = "anchor" (`_wire.TARGET_ANCHOR`): measure this target's step from
# where the ARM is rather than from the previous accepted target. The node ignores every other
# bit, and a node built before that commit ignores this one too -- a set bit is never a
# refusal. Left OFF until a node with the flag is confirmed running on the arm's host:
# everything here works correctly without it, because we re-latch the clutch instead of
# trying to send the jump.
ANCHOR_ENABLED = False

RAW_POS_LIMIT = 10.0        # m, |pos_i| sanity bound before mapping
QUAT_TOLERANCE = 1e-3       # the node refuses a quaternion norm off unit by more than this

# ---------------------------------------------------------------- the process


class Teleop:
    def __init__(self, args, source, sink, node, params=None):
        self.a = args
        self.node = node                            # the node's own limits; never a copy
        self.params = params                        # the teleop/params owner, or None
        self.anchor_enabled = ANCHOR_ENABLED
        self.source, self.sink = source, sink
        self.clutch = Clutch(args.spatial_scale, args.rotation_scale)
        self.seq = 0
        # The node keeps the last gripper (client, seq) for its lifetime, so a count from 1
        # would be refused after a restart with the same --client-id; wall-clock ns is not
        # (unless the clock steps back past the last session's count while the node runs).
        self.gseq = time.time_ns()
        self.n_in = self.n_out = self.n_keepalive = self.n_hold = 0
        self.n_engage = self.n_reanchor = self.n_gripper = 0
        self.recoveries = []                        # (s, reason, seconds, failure or None)
        self.refusals = {k: 0 for k in REASONS}
        self.clamps = {k: 0 for k in CLAMPS}
        self.worst_lead = self.worst_lead_rot = 0.0
        self.worst_lead_desired = self.worst_step = self.worst_step_rot = 0.0
        self.worst_hand_step = self.worst_hand_speed = self.worst_cmd_speed = 0.0
        self.worst_hand_rot_speed = self.worst_cmd_rot_speed = 0.0
        self.prev_hand_ns = None
        self.prev_quat = None                       # for the sign canonicalisation
        self.prev_hand_pos = None
        self.prev_engaged = False
        self.armed = False                          # fail closed: needs a grip release
        self.dq_tripped = False                     # RULE 6, until slow again (or --dq-latch)
        self.dq_releases = []                       # (s, joint 1..7, |dq| rad/s, limit)
        self.worst_dq = None                        # (fraction of its limit, joint 1..7)
        self.need_relatch = True
        self.hold_until = 0
        self.last_target = None
        self.last_pub_ns = 0
        self.last_msg_ns = None
        self.last_width = None
        self.last_gripper_ns = 0
        self.period_ns = int(1e9 / args.rate)
        self.gripper_period_ns = int(1e9 / args.gripper_hz) if args.gripper_hz > 0 else 0
        self.first = {}
        self.abort = None
        self.t0 = 0
        self.worst_pub_gap = 0
        self.emit_fh = open(args.emit, "w") if args.emit else None

    # -- bookkeeping ------------------------------------------------------------

    def refuse(self, reason, t, detail=""):
        self.refusals[reason] = self.refusals.get(reason, 0) + 1
        if reason not in self.first:
            self.first[reason] = True
            self.note(f"[{reason}] first at {self.elapsed(t):6.2f}s {detail}")

    def stamp(self, t):
        """t_send_ns: this process's own CLOCK_MONOTONIC, which is also the clock
        StateMsg.t_node_ns is on, so a same-host round trip is comparable."""
        return t if self.source.live else time.monotonic_ns()

    def emit(self, kind, raw, **fields):
        if self.emit_fh is None:
            return
        self.emit_fh.write(json.dumps({"kind": kind, "raw": raw.hex(), **fields}) + "\n")

    def note(self, line):
        if not self.a.quiet:
            print(line, flush=True)

    def elapsed(self, t):
        if self.last_msg_ns is None:
            return 0.0
        return (t - self.t0) / 1e9

    # -- the pipeline -----------------------------------------------------------

    def on_wire(self, payload, t):
        self.n_in += 1
        if self.n_in == 1:
            self.t0 = t
        self.last_msg_ns = t
        msg = decode_vr(payload)
        if isinstance(msg, str):
            return self.refuse("decode", t, msg)
        pos, quat, grip, flags = msg
        fresh = bool(flags & FLAG_FRESH)
        engaged = bool(flags & FLAG_ENGAGED)

        if any(not math.isfinite(c) for c in pos + quat) or \
                max(abs(c) for c in pos) > RAW_POS_LIMIT:
            self.need_relatch = True
            return self.refuse("sanity_pos", t, f"pos {fmt3(pos)}")
        previous_quat = self.prev_quat
        if not self.a.no_canonicalise:               # RULE 4, before anything reads it
            quat = qcanon(quat, self.prev_quat)
        if abs(qlen(quat) - 1.0) > QUAT_TOLERANCE:
            self.need_relatch = True
            return self.refuse("sanity_quat", t, f"norm {qlen(quat):.5f}")
        self.prev_quat = quat                        # only a good sample sets the sign

        if not fresh:                                # RULE 1
            self.need_relatch = True
            self.prev_engaged = False
            return self.refuse("not_fresh", t)
        hand_step = dist(pos, self.prev_hand_pos) if self.prev_hand_pos else 0.0
        hand_turn = qangle(quat, previous_quat) if previous_quat else 0.0
        hand_dt = (t - self.prev_hand_ns) / 1e9 if self.prev_hand_ns else 0.0
        self.prev_hand_pos, self.prev_hand_ns = pos, t
        self.worst_hand_step = max(self.worst_hand_step, hand_step)

        if not self.armed:                           # fail closed after enable and RULE 7
            if engaged:
                return self.refuse("not_armed", t, "grip held; release it")
            self.armed = True
            self.note(f"[armed] grip release seen at {self.elapsed(t):6.2f}s")
        self.gripper(grip, t)                        # the trigger needs only freshness
        self.joint_speed(t)                          # RULE 6, on a state newer than advance's
        if self.dq_tripped:
            return self.refuse("dq_release", t, "until the joints are slow and a new squeeze")
        state = self.fresh_state()
        if state is None:                            # RULE 7: release and squeeze again
            self.need_relatch, self.prev_engaged, self.armed = True, False, False
            return self.refuse("no_state", t, f"no arm state within {STATE_STALE_MS:.0f} ms")
        p_arm, q_arm = state.measured

        if self.need_relatch:                        # RULE 3
            self.clutch.latch(p_arm, q_arm, pos, quat)
            self.need_relatch = False
            self.hold_until = t + self.a.reanchor_hold_ms * 1e6
            self.n_reanchor += 1
            self.prev_engaged = False
            return self.refuse("reanchor", t, f"hand step {hand_step * 1000:.1f} mm")
        if hand_step > self.a.max_hand_step:         # RULE 3's fresh->fresh twin
            self.clutch.latch(p_arm, q_arm, pos, quat)
            self.hold_until = t + self.a.reanchor_hold_ms * 1e6
            self.n_reanchor += 1
            self.prev_engaged = False
            return self.refuse("hand_jump", t, f"{hand_step * 1000:.1f} mm in one sample")
        if t < self.hold_until:
            self.prev_engaged = False
            return self.refuse("hold_open", t)

        if not engaged:                              # RULE 2: released means hold
            self.prev_engaged = False
            return self.refuse("released", t)
        continuing = self.prev_engaged
        if not continuing:                           # the rising edge
            self.clutch.latch(p_arm, q_arm, pos, quat)
            self.prev_engaged = True
            self.n_engage += 1
            self.note(f"[engage] {self.n_engage} at {self.elapsed(t):6.2f}s "
                      f"arm p {fmt3(p_arm)}")
        if not self.clutch.latched:
            return self.refuse("no_state", t)
        if continuing and hand_dt > 0.0:             # speeds across a release mean nothing
            speed = hand_step / hand_dt
            self.worst_hand_speed = max(self.worst_hand_speed, speed)
            self.worst_cmd_speed = max(self.worst_cmd_speed, speed * self.a.spatial_scale)
            turn = hand_turn / hand_dt               # the number that predicts lead_rot
            self.worst_hand_rot_speed = max(self.worst_hand_rot_speed, turn)
            self.worst_cmd_rot_speed = max(self.worst_cmd_rot_speed,
                                           turn * self.a.rotation_scale)

        p_cmd, q_cmd = self.clutch.compose(pos, quat)
        self.publish(p_cmd, q_cmd, t, "drive", state)     # one sample for the whole decision

    # -- output -----------------------------------------------------------------

    def clamp(self, p, q, state):
        """The workspace box, the leash on `state`'s measured pose (none without a fresh
        state), then --max-step from the last target: a target that would step further is
        moved toward the request as far as the step allows, never refused."""
        if self.a.workspace is not None:
            lo, hi = self.a.workspace
            inset = self.a.workspace_inset
            boxed = tuple(min(max(c, l + inset), h - inset) for c, l, h in zip(p, lo, hi))
            if dist(boxed, p) > 1e-12:
                self.clamps["workspace"] += 1
            p = boxed
        if state is not None:
            p_arm, q_arm = state.measured
            lead = dist(p, p_arm)
            if lead > self.a.clamp:                  # the client-side leash, on MEASURED
                p = tuple(a + (c - a) * self.a.clamp / lead for a, c in zip(p_arm, p))
                self.clamps["lead"] += 1
            angle = qangle(q_arm, q)
            if angle > self.a.clamp_rot:
                q = qslerp(q_arm, q, self.a.clamp_rot / angle)
                self.clamps["lead_rot"] += 1
        if self.last_target is not None:             # keep the node's step guard slack
            p_prev, q_prev = self.last_target
            step = dist(p, p_prev)
            if step > self.a.max_step:
                p = tuple(a + (c - a) * self.a.max_step / step for a, c in zip(p_prev, p))
                self.clamps["step"] += 1
            angle = qangle(q_prev, q)
            if angle > self.a.max_step_rot:
                q = qslerp(q_prev, q, self.a.max_step_rot / angle)
                self.clamps["step_rot"] += 1
        return p, qunit(q)

    def publish(self, p, q, t, why, state=None):
        if self.sink.recoverable():                  # a Faulted arm: nothing until recovered
            return
        if state is None:
            state = self.fresh_state()
        p, q = self.clamp(p, q, state)
        self.seq += 1
        anchor = ANCHOR_ENABLED and why == "anchor"
        flags = _wire.TARGET_ANCHOR if anchor else 0
        # RULE 5: our own seq (strictly increasing from 1, nothing to do with the wire's
        # bridge-tick counter) and our own t_send_ns, which the node only echoes.
        raw = _wire.encode_target("cartesian", self.a.client_id, self.seq, self.stamp(t),
                                  (*p, *q), anchor)
        self.sink.put_target(raw, p, q, self.seq, why)
        if self.last_pub_ns:
            self.worst_pub_gap = max(self.worst_pub_gap, t - self.last_pub_ns)
        lead = dist(p, state.measured[0]) if state else None
        lead_rot = qangle(q, state.measured[1]) if state else None
        self.emit("target", raw, t_src=t, seq=self.seq, why=why, pos=list(p), quat=list(q),
                  flags=flags, lead=lead, lead_rot=lead_rot)
        if self.last_target is not None:
            self.worst_step = max(self.worst_step, dist(p, self.last_target[0]))
            self.worst_step_rot = max(self.worst_step_rot,
                                      qangle(q, self.last_target[1]))
        if state is not None:
            self.worst_lead = max(self.worst_lead, lead)
            self.worst_lead_rot = max(self.worst_lead_rot, lead_rot)
            self.worst_lead_desired = max(self.worst_lead_desired,
                                          dist(p, state.desired[0]))
        self.last_target = (p, q)
        self.last_pub_ns = t
        self.n_out += 1
        self.n_keepalive += why == "keepalive"
        self.n_hold += why == "dq_hold"

    def gripper(self, fraction, t):
        """The index trigger as a width, rate limited and deadbanded."""
        if self.a.no_gripper or self.gripper_period_ns == 0:
            return
        if not self.sink.gripper_ready:              # the arm has published no gripper state
            return
        width = max(0.0, min(1.0, 1.0 - fraction)) * self.sink.max_width
        if self.last_width is None:
            # Seed, never send: a released trigger maps to FULLY OPEN, and the first
            # message of a session must not fling the fingers open on whatever the
            # gripper is holding. The first command goes out when the trigger moves.
            self.last_width, self.last_gripper_ns = width, t
            return
        if t - self.last_gripper_ns < self.gripper_period_ns:
            return
        if self.last_width is not None and abs(width - self.last_width) < self.a.gripper_deadband:
            return
        self.gseq += 1
        raw = _wire.encode_gripper("width", self.a.client_id, self.gseq, self.stamp(t),
                                   width, 0.0)
        self.sink.put_gripper(raw, width, self.gseq)
        self.emit("gripper", raw, t_src=t, seq=self.gseq, width=width, fraction=fraction)
        self.last_width, self.last_gripper_ns = width, t
        self.n_gripper += 1

    def state_age_ms(self):
        """The sink's latest arm state and its age on the sink's clock; (None, inf) without."""
        state = self.sink.state()
        if state is None:
            return None, math.inf
        return state, (self.sink.clock_ns() - state.t_ns) / 1e6

    def fresh_state(self):
        """RULE 7: the latest arm state, or None once it is older than STATE_STALE_MS."""
        state, age = self.state_age_ms()
        return state if age <= STATE_STALE_MS else None

    def state_lost(self):
        """RULE 7: a reason to end the session once no state has arrived for STATE_LOST_MS."""
        if self.state_age_ms()[1] > STATE_LOST_MS:
            return f"no arm state for over {STATE_LOST_MS:.0f} ms"
        return None

    def joint_speed(self, t):
        """RULE 6 on the latest fresh state: the first joint over DQ_RELEASE_FRACTION of its
        limit releases the clutch and publishes, once, the pose that same state measured,
        which the keepalive then holds. Every joint under DQ_RESUME_FRACTION ends the release
        (not with --dq-latch); driving then needs a release and a new squeeze."""
        state = self.fresh_state()
        if state is None or state.dq is None:
            return
        j, fraction = worst_joint(state.dq, self.node.dq_limit)
        if self.worst_dq is None or fraction > self.worst_dq[0]:
            self.worst_dq = (fraction, j + 1)
        if self.dq_tripped:
            if not self.a.dq_latch and fraction < DQ_RESUME_FRACTION:
                self.disarm()
                print(f"[dq-resume] every joint under {DQ_RESUME_FRACTION} of its limit at "
                      f"{self.elapsed(t):6.2f}s: release and squeeze the grip to drive",
                      flush=True)
            return
        if fraction <= DQ_RELEASE_FRACTION:
            return
        self.dq_tripped = True
        speed = abs(state.dq[j])
        self.dq_releases.append((self.elapsed(t), j + 1, speed, self.node.dq_limit[j]))
        until = "for the rest of the session (--dq-latch)" if self.a.dq_latch else \
            f"until every joint is under {DQ_RESUME_FRACTION} of its limit and a new squeeze"
        print(f"[dq-release] joint {j + 1} |dq| {speed:.3f} rad/s > {DQ_RELEASE_FRACTION} x "
              f"{self.node.dq_limit[j]} at {self.elapsed(t):6.2f}s: holding the arm's measured "
              f"pose "
              f"{until}", flush=True)
        self.publish(*state.measured, t, "dq_hold", state)

    def disarm(self):
        """Release the clutch and end the dq release; the next drive needs a grip release,
        then a squeeze, re-latched on the arm's measured pose."""
        self.dq_tripped = False
        self.need_relatch, self.prev_engaged, self.armed = True, False, False

    def recover(self, reason, t):
        """Auto-recovery of a Faulted arm: nothing is published while the sink recovers and
        enables again, then the keepalive restarts from the new session's target. False when
        it failed, with the reason in `abort`."""
        latched = self.dq_tripped and self.a.dq_latch
        self.disarm()
        self.dq_tripped = latched
        started = time.monotonic()
        failure = self.sink.recover()
        took = time.monotonic() - started
        self.recoveries.append((self.elapsed(t), reason, took, failure))
        n = len(self.recoveries)
        if failure:
            self.abort = f"{reason}; auto-recovery {n} failed: {failure}"
            return False
        self.last_target, self.last_pub_ns = None, 0    # the node's new loop, its own target
        print(f"[recovered] {n}/{self.a.max_recoveries} {reason} at {self.elapsed(t):6.2f}s: "
              f"Active again after {took:.1f} s; release and squeeze the grip to drive",
              flush=True)
        return True

    def advance(self, t):
        """Move the clock on: the arm, the seed for the keepalive, the joint-speed release,
        and the two things that arrive from other threads -- the node's live budgets and an
        accepted teleop/params set. Both are applied HERE, between samples, because the loop
        is the only writer of `self.a`."""
        self.sink.tick(t)
        self.refresh_limits()
        self.apply_params()
        if self.last_target is None:
            state = self.sink.state()
            if state is not None:                    # seed on the loop's own desired pose
                self.last_target = state.desired
        self.joint_speed(t)

    def refresh_limits(self):
        """A `params/current` from the node: the budgets are live and tunable, and they move
        the lead bound through one state tick of staleness. If the node has tightened a bound
        under us, --clamp comes down to it and says so, rather than being refused mid-run."""
        body = self.sink.take_params()
        if body is None:
            return
        try:
            node, lowered = limits.adopt_budgets(self.a, self.node, body)
        except node_limits.LimitsError as exc:
            return self.note(f"[limits] ignoring a params/current we cannot read: {exc}")
        if node is None:
            return
        if node.boot_id != self.node.boot_id:
            print(f"[limits] the node's boot_id is now {node.boot_id}: it restarted under "
                  f"this session", flush=True)
        self.node = node
        for flag, bound, unit in lowered:
            setattr(self.a, flag, bound)
            print(f"[limits] --{flag.replace('_', '-')} lowered to {bound:.4f} {unit}: the "
                  f"node's budget is now {node.budget:.3f} m/s, "
                  f"{node.budget_rotation:.3f} rad/s", flush=True)
        if self.params is not None:
            self.params.retune(node)

    def apply_params(self):
        """An accepted `teleop/params/set`, applied on this thread and nowhere else."""
        if self.params is None:
            return
        for name in self.params.apply_pending():
            if name == "rate":
                self.period_ns = int(1e9 / self.a.rate)
            print(f"[params] {name} = {getattr(self.a, name)}", flush=True)
        # The clutch keeps its own scales so that changing one cannot move an engaged arm:
        # they are adopted while the grip is released, which is the next latch.
        if not self.prev_engaged:
            self.clutch.scale = float(self.a.spatial_scale)
            self.clutch.rot_scale = float(self.a.rotation_scale)
        self.params.tick()

    def keepalive(self, t):
        """Republish the LAST COMMANDED target at --rate so the node's watchdog never sees
        silence (RULE 1). A source that has gone quiet altogether is left to the node's own
        hold and stop instead -- keeping a session alive for a dead bridge would be wrong."""
        if self.last_target is None or self.last_msg_ns is None:
            return
        if (t - self.last_msg_ns) / 1e6 > self.a.source_timeout_ms:
            return
        if t - self.last_pub_ns >= KEEPALIVE_SLACK * self.period_ns:
            self.publish(self.last_target[0], self.last_target[1], t, "keepalive")

    # -- the loop ---------------------------------------------------------------

    def run(self, stop_flag):
        self.sink.cancelled = stop_flag
        self.sink.start()
        t_start = self.source.now()
        self.t0 = t_start
        deadline = t_start + int(self.a.seconds * 1e9) if self.a.seconds > 0 else None
        while not stop_flag():
            batch = self.source.poll(self.a.poll_ms)
            for t, payload in batch:
                self.advance(t)
                self.on_wire(payload, t)             # a driven target resets the keepalive
            now = self.source.now()
            self.advance(now)
            self.keepalive(now)
            fault = self.sink.fault()
            if fault and self.sink.recoverable():
                if self.a.no_auto_recover:
                    fault += " (--no-auto-recover)"
                elif len(self.recoveries) >= self.a.max_recoveries:
                    fault += f" (all {self.a.max_recoveries} auto-recoveries used)"
                elif self.recover(fault, now):
                    continue
                else:
                    break
            fault = fault or self.state_lost()
            if fault:
                self.abort = fault
                break
            if deadline is not None and self.source.now() >= deadline:
                break
            if self.source.done and not batch:
                break
        return self.abort

    # -- the summary ------------------------------------------------------------

    def facts(self):
        return summary_mod.facts(self)

    def summary(self, out=None):
        summary_mod.summary(self, out)

