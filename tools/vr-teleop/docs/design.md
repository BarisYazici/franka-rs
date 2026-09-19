# Why the VR teleop stack is built this way

Every rule below exists because a measurement or an incident asked for it. The numbers are
from a Quest 3 driving Franka arms through `franka-node`; they are recorded here so the
code can stay short and so a reader can tell a tuning constant from a safety one.

Two processes, two wires:

```
Quest ──adb logcat──▶ franka-vr-bridge ──ZMQ PUB, 88-byte VrTargetMsg, 50 Hz──▶ franka-vr-teleop
                                                                                        │
                                                                  Zenoh, 80-byte TargetMsg
                                                                                        ▼
                                                                                   franka-node ──▶ arm
```

The bridge holds no robot state at all, which is what makes its state machines testable.
The client holds no headset state beyond one sample, and the node is the last guard: it
enforces its own step, lead and rate limits whatever a client sends.

## The bridge

**Freshness is derived from change, not from the reader.**
`OculusReader.get_transformations_and_buttons()` returns the last cached frame forever —
when the headset sleeps, a controller loses tracking or adb drops, it hands back the same
4×4 with no error. That stale cache is the root cause of the historical "the intervention
did nothing / the arm froze but the UI looked fine" class of incident. So identical
consecutive matrices are normal for a tick or two (50 Hz polling of a ~72 Hz stream), but
**250 ms of zero change reads as not fresh**. `controller_on` is the same detector with a
**5 s** window: "has this controller reported anything at all lately", a UI-level
indicator, deliberately not what gates driving.

**The loop never skips a publish.** A dropped, garbled or incomplete reader frame still
publishes a not-fresh, not-engaged message carrying the last good pose. Silence and
not-fresh are different things downstream: silence means the bridge died, not-fresh means
the headset stopped tracking, and an operator needs to be told which one they have.

**A one-tick reader hiccup is not a dead stream.** `process_data()` returns `(None, None)`
for a single truncated logcat line and the reader stores that straight over its cache, so
one bad line out of a ~72 Hz stream used to read as a whole-frame gap and force *both*
arms' clutches open in the same tick — each then staying dead until its own grip was
released, a per-arm ritual an operator cannot guess at. A dual-arm session showed exactly
that: both channels latching on one identical timestamp, then re-arming **0.6 s** and
**9.4 s** later, whenever each hand happened to let go. So an unusable frame now
republishes that channel's last usable sample verbatim for up to `GAP_HOLD_S` = **100 ms**
(~7 logcat frames); past that the channel publishes `engaged=False, fresh=False`. This is
strictly tighter than the stale-cache path it replaced: had the bad line simply not
arrived, the mapper would have called the same frame fresh for a further 250 ms.
`GAP_HOLD_S` has no CLI knob on purpose.

**Fail closed on the clutch, per channel.** Every channel starts latched and clears only on
a tick where *that* channel has a fresh sample with *its own* grip released. A bridge
restarted mid-squeeze therefore cannot re-engage an arm under the operator's hand. A
not-fresh sample latches its own channel alone: one controller dropping out says nothing
about the other, and a bridge-wide latch made single-arm bring-up impossible — with only
one controller present, the absent channel held every arm disengaged forever.

**The keepalive runs on its own thread, never on the tick path.** The Quest pauses a
`vr_only` app the instant its proximity sensor reads "not worn", which kills the pose
stream, so two adb broadcasts tell the app to ignore the sensor, re-sent every ~5 s because
a headset state change can undo them. `am broadcast` takes **100 ms–1 s** on-device;
calling it from `tick()` used to stall the 50 Hz publish loop past the consumer's 150 ms
clutch-disengage window every ~5 s, so the arm stuttered and re-anchored mid-drive. A slow
broadcast now delays only the next keepalive.

**The axis remap is `[-3, -1, 2, 4]`,** for a headset placed upright facing the workspace:

```
robot_x (forward) = -head_z      robot_y (left) = -head_x      robot_z (up) = +head_y
```

The stock DROID map `[-2, -1, -3, 4]` was measured to swap up and forward. Translation uses
only this fixed remap and is deliberately **not** rotated by the forward-direction latch, so
a given hand motion always maps to the same robot axis however the controller is tilted.
Orientation does go through the latch (re-latched continuously while the grip is released,
frozen while it is held, force-relatched by the right thumbstick). The two therefore live in
different frames, which is fine: the clutch anchors position and orientation independently.

## The client: seven rules

1. **Freshness gate.** `fresh` was false **31 %** of the time with the headset merely idle,
   and a frozen pose is a valid-looking target the node structurally cannot see. While not
   fresh the client does not follow the controller, but keeps the session alive by
   republishing the **last commanded** target at `--rate`: the worst measured not-fresh
   episode was **1.78 s** and the node calls `stop()` after **2 s** without a target, which
   would drop the arm to Idle. A source that stops sending altogether is a different thing:
   after `--source-timeout-ms` the client stops republishing too and lets the node's own
   watchdog hold and then stop the arm.

2. **Clutch on the grip.** On the rising edge, latch `p_off = p_arm - scale * p_hand` (a
   base-frame vector, added, never rotated) and `q_off = conj(q_hand) ⊗ q_arm`
   (right-multiplied) — decoupled, never as one homogeneous transform. Composition then
   returns the arm's own measured pose exactly on the latch tick, so re-engaging never
   jumps. While released the client holds (keepalive only).

3. **Never publish the first fresh sample after a gap.** Measured: **77.7 mm** between two
   consecutive samples of a controller lying still, across a not-fresh → fresh edge — an
   implied **3.87 m/s**, well over the node's 5 cm `max_step`. The clutch re-latches on that
   sample and nothing is published. The same recording steps 77.7 mm **back** 60 ms later,
   fresh → fresh, which the literal rule does not cover, so a per-sample hand-step gate
   (`--max-hand-step`, **2.5 m/s** at 50 Hz) catches a jump that arrives without a freshness
   edge. Both hold the clutch open for `--reanchor-hold-ms` afterwards.

4. **Quaternion sign.** **26.1 %** of messages carry the antipodal quaternion (the released
   rotation sits on a branch boundary of scipy's `as_quat`), which a sign-trusting consumer
   reads as a 2π step. The client canonicalises against the previous sample's sign.

5. **Its own `seq` and `t_send_ns`.** The wire's `seq` counts bridge ticks, is shared by both
   channels and restarts at 1 when the bridge restarts; the node wants strictly increasing
   per client. Targets and gripper commands each get their own counter.

6. **Joint-velocity release.** A session ended in a `joint_velocity_violation` with joint 5
   at **3.48 rad/s**. When a measured `StateMsg.dq` passes `DQ_RELEASE_FRACTION` (**0.85**)
   of that joint's limit, the clutch releases: the joint and its speed are printed, and the
   pose that same `StateMsg` measured is published once as the target — not the last
   commanded one, which can lead the arm by the whole leash — through the same clamp, so a
   step past `--max-step` is shortened toward it rather than refused. Once every joint
   measures under `DQ_RESUME_FRACTION` (**0.5**) of its limit, a release and a fresh squeeze
   drive again; `--dq-latch` keeps the release for the rest of the session. The trigger
   still drives the gripper throughout.

7. **Arm-state freshness.** A state older than `STATE_STALE_MS` (**50 ms**, five of the
   node's ticks) drives nothing: no clutch latch, no lead clamp, no dq release. The sample
   is refused as `no_state` and the grip must be released and squeezed again once state is
   back. A fresh state with a non-finite pose or dq drives nothing either (a NaN passes every
   leash comparison), and the client prints `[state] the arm's state is not finite` once. No
   state for `STATE_LOST_MS` (**500 ms**) ends the session through the clean stop and
   release. The node's `stop` can freeze its state publication for seconds, and a frozen
   phase would otherwise hide it.

## Scales, because a human is in the loop

Both scales are the teleop client's, applied in its clutch. The bridge publishes at
`--spatial-scale 1.0`, its default; a scale there as well would multiply with these.

**`--spatial-scale 0.4`.** Measured with the operator holding the controllers, engaged
samples only: hand speed p50 **0.08 m/s**, p99 **0.50**, peak **0.706 m/s**, worst step
**14 mm** per 20 ms tick. So the node's 5 cm `max_step` is *not* the binding constraint
(3.5× of margin at scale 1.0) — the Cartesian budget is. That budget, **0.3 m/s** by default,
is a norm the node splits evenly over the three axes (÷√3): a move along one axis tops out
at **0.173 m/s**, and only a diagonal gets the full 0.3. At 1.0 a fast flick commands 2.4×
the norm and 4× what one axis can follow: the desired pose runs ahead, builds 5 cm of lead
within about **125 ms**, and the lead guard refuses while the arm stalls under a hand that is
still moving. That only bites on quick moves, which makes it a surprise rather than a
constant. 0.4 puts the measured peak at **0.28 m/s** and the worst step at **5.6 mm**:
inside the norm on a diagonal, but 1.6× what one axis gets, so a fast single-axis move still
leads the arm and the 25 mm clamp below is what bounds it.

**`--rotation-scale 0.25`,** which a 1:1 stack does not do. The same argument binds harder on
the wrist, because a wrist turns multiples of the arm's **0.5 rad/s** rotational budget (a
norm too: 0.29 rad/s about one axis) where a hand at scale 0.4 sits at 94 % of the
translational norm. Measured over 20 s of one
operator's hands, per channel — clamps saturating, worst lead against the 0.15 rad leash,
commanded peak:

| scale | clamps saturating (left / right) | worst lead (rad) | commanded peak (rad/s) |
|---|---|---|---|
| 1:1  | 282 of 527 / 336 of 518 | 0.150 / 0.150 | 4.59 / 5.86 |
| 0.6  | 176 / 196               | 0.150 / 0.150 | 2.76 / 3.52 |
| 0.4  | 0 / 72                  | 0.142 / 0.150 | 1.84 / 2.34 |
| 0.25 | 0 / 0                   | 0.035 / 0.079 | 1.15 / 1.47 |

0.4 clears the right channel with 5 % of margin and still saturates the left 72 times, so
the default is 0.25. Note what this does **not** claim: even at 0.25 the commanded peak is
about 3× the arm's rotational budget, so the **leash**, not the scale, is what bounds a fast
flick, and rotation lags a flick by design. The criterion that matters is that the clamp is
not saturated — a scale that put a 5.9 rad/s wrist inside 0.5 rad/s would be 0.085, which
would make rotation useless. The summary prints both commanded peaks against both budgets so
a session can see where it sat.

## The clamps, the gripper, and arming

Every published target is clamped to `--clamp` of the arm's **measured** pose, so the client
is safe against a node with no lead guard. The defaults, 25 mm and 0.15 rad, are the size of
the node's own leash (`derived.leash`, how far its torque backend lets the desired pose lead
the measured one), so in free motion that leash rarely binds: the target already sits inside
it. A second clamp keeps consecutive targets inside
`--max-step`, because the lead clamp alone does not: the arm moves between samples, and
25 + 6 + 25 mm at 0.3 m/s and 50 Hz is more than 50 mm. A refused target is worse than a
clamped one — the node's step reference is the last *accepted* target, so one refusal can
lock a hand-driven client out until it walks back in 5 cm hops. Clamp order is workspace →
lead → step.

The index trigger drives the gripper as a width, rate limited and deadbanded: the Franka
Hand blocks for seconds per move and the node's driver ignores a change under **1 mm**. The
trigger is gated on `fresh` too, because the bridge snaps gripper to 0.0 on a not-fresh
message (measured 3000 of 3000) and **0.0 is fully open** — ungated, every keepalive gap
would drop whatever the hand is holding.

**Fail-closed arming**: after enable, no motion is published until a grip *release* has been
seen, so a bridge or client restarted mid-squeeze cannot re-engage an arm under the
operator's hand.

**Auto-recovery** (`--no-auto-recover` restores ending the session). An arm that reads
Faulted or Reflex, or has errors, is recovered before acquire. Faulted mid-session: the
clutch releases, nothing is published, the node's `recover` runs and its state is polled
until Idle with no errors, then acquire and enable with the same `--episode`, one
`[recovered]` line, and driving needs a release and a fresh squeeze. At most
`--max-recoveries` (**5**) per session; past that the fault ends it. Any other loss of
Active — a node stop, a taken lease — still ends the session.

## Hands

The wire carries no hand id: **the port is the hand**, 5560 left and 5570 right (the
bridge's `--controller l,r`). `--hand` defaults from `--arm`, and a crossed pairing needs
`--cross`. On a live bridge, before the arm is acquired, the check listens on *both* ports:
the chosen controller must be fresh and `controller_on`, a squeeze of its grip must show
`engaged` on its port and stay engaged for **100 ms**, and the other port must publish and
never show `engaged` once the operator is prompted (`--one-controller` allows silence, with
a warning). Any failure exits 3 with nothing acquired; `--skip-hand-check` skips it, loudly.

## Limits

Every flag of the client's that one of the node's own limits bounds is validated against it
at **startup** and never discovered mid-run: a refused target is not followed, the arm holds
its last accepted target and measures the next step from it, so refusals stall the arm under
a moving hand. A refusal no longer ends the session — the node counts a target refused on
its content as a live commander.

The node publishes those limits (`params/schema`'s `derived` block, `params/get` for the
live budgets), which is where they must come from: a hand-maintained mirror in the client
has drifted from the node's configuration before, and a client that believes a 5 cm lead
bound while the node enforces 12 cm is a client that clamps for no reason. So the client reads
them from the node before it acquires anything and refuses to start without them;
`--probe-limits` prints them, and `--node-limits FILE` is the deliberate offline path.

## Not implemented, on purpose

The wire's `anchor` flag bit. Turning it on is a hardware decision, not a cleanup.
