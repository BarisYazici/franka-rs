# The impedance backend

[Target control](../howto/target-control.md) tracks the generator's setpoints with torques of
its own by default. This page is the derivation: where the law comes from, why it is a joint
law with a Cartesian term rather than the other way round, what the joint gains do to the
stiffness you feel, how the Cartesian interface gets a joint goal, how the goal and the arm
are kept under the joint velocity limits and inside the joint position limits, how the
generator is anchored without an echo, what happens at the handover, and how it differs from
the operational-space law of the `cartesian_impedance_active_control` example. The options and
defaults are on the how-to page; the code is `robot/target_control/impedance.rs`, `torque/`,
`ik/`, `position.rs` and `velocity.rs`.

## The law and its provenance

```text
Kp  = Jᵀ Kx J + diag(Kq)          Kd = Jᵀ Kxd J + diag(Kqd)
tau = Kp (q_goal − q) + Kd (g dq_goal − dq) + coriolis(q, dq),   clamped to ±torque_limits
```

This is the structure of `HybridJointImpedanceControl` from
[polymetis](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/python/torchcontrol/policies/impedance.py)
(its feedback module `HybridJointSpacePD` forms `Kp` and `Kd` exactly so), the controller
that DROID (Khazatsky et al., 2024, *DROID: A Large-Scale In-The-Wild Robot Manipulation
Dataset*, [arXiv:2403.12945](https://arxiv.org/abs/2403.12945)) ran on its Panda arms for
76k teleoperated trajectories. `ImpedanceGains::DROID` is its gains as they were, and with
`velocity_feedforward` off (`g = 0`, the damping on the absolute velocity, the default) the
law is polymetis's. `ImpedanceGains::CARTESIAN` differs from it: it raises the
translational damping from 37 to 50, 50, 90 Ns/m: the arm's apparent mass at the end effector
near the ready pose is about 0.94 kg along x and y and 3.9 kg along z (from the model's mass
matrix), so 37 Ns/m against 750 N/m leaves z at a damping ratio of 0.34, which rings; the
defaults bring all three to about 0.8. Damping the absolute velocity resists the motion the
goal asks for, and a goal moving at `v` is tracked `Kd v / Kp` behind it.
`velocity_feedforward = true` makes `g` the `velocity_feedforward_gain` (default 1), with
`dq_goal` the goal's velocity: the generator's on the joint interface, the finite difference of
the IK solution on the Cartesian one, zero while holding. It is off by default because real
arms vibrated with it on: `Kd` carries the finite difference's noise into the torque.
`velocity_feedforward_cutoff` low-pass filters `dq_goal` and is the knob to try against it; no
value has been validated on hardware. `J` is `Model::zero_jacobian(Frame::EndEffector, state)`, the
6x7 base-frame Jacobian at the *measured* `q`, so `Kx` acts at the configured end-effector
frame, the frame the `O_T_EE` targets are in. Gravity is compensated by the robot; `coriolis`
is the model's, as in every torque example of the crate. The clamp comes before the low-pass
filter (`cutoff_frequency`, 100 Hz by default) and the torque-rate limiter that
`control_torques` applies, in that order.

## Why a joint law with a Cartesian term

`Jᵀ Kx J` is the Cartesian spring expressed in joint coordinates: for a small joint error
`Δq`, `J Δq` is the end-effector error and `Jᵀ Kx J Δq` the joint torque of the spring `Kx`
pulling on it. It dominates the stiffness: a joint whose lever arm to the end effector is
0.5 m sees 750 N/m as `750 × 0.5² ≈ 190 Nm/rad`, against 25 to 50 Nm/rad in `Kq`. But on a
seven-joint arm the 6x7 Jacobian has a one-dimensional nullspace, the motion that leaves the
end effector where it is (the elbow's swing), and `Jᵀ Kx J` is zero along it. `diag(Kq)`
regularises that direction, and near a singularity, where the Cartesian term collapses in
more directions, it keeps every joint held. So the Cartesian gains set the compliance you
feel at the tool and the joint gains set how firmly the arm keeps its shape. The joint
interface uses the same code with `Kx = Kxd = 0`, which leaves a joint PD with Coriolis
feed-forward, the `fer_joint_impedance` example's law and gains.

## What the joint gains do to the stiffness you feel

`diag(Kq)` is not confined to the nullspace: it acts on every joint, and the part of it that
`J` maps to the end effector adds to `Kx`. The stiffness felt at the tool is
`(J Kp⁻¹ Jᵀ)⁻¹`, and with the default gains at the ready pose that is about 990 to 1180 N/m
in translation against the 750 N/m set, and two to three times `Kx` in rotation (computed
from the model). DROID ran the law this way. `ImpedanceOptions::project_joint_gains`
replaces `diag(Kq)` and `diag(Kqd)` by `N Kq N` and `N Kqd N`, with `N = I − J⁺ J` the
nullspace projector of the damped pseudoinverse `J⁺ = Jᵀ (J Jᵀ + λ² I)⁻¹` (`λ` the IK's
damping, 0.05): the joint springs then hold the elbow's swing and nothing else, and the end
effector feels `Kx` to within the damping of the projector, away from singularities, where
`N` widens and the projected springs take over the collapsing directions. The price is that
with the projection on the Cartesian gains are all that holds the tool, so zero `Kx` would
leave it free. It is off by default to keep the default law DROID's in structure.

## Payload

Gravity is not in the law: the robot compensates it from the load configured in Desk or by
`Robot::set_load`. The robot's own controller corrects a wrong load as a position error; a
spring does not. An unconfigured tool becomes a constant sag of its weight over the felt
stiffness, 0.5 kg → 5 N → 6.7 mm at 750 N/m (less at the felt ~1000 N/m), in every pose, and
a load heavier than configured pulls the same way. `set_load` matters more here than in
position mode.

## The differential inverse kinematics

The joint interface feeds the generator's `q` in as `q_goal`. The Cartesian interface needs a
joint goal for a pose, and gets one incrementally: from the previous `q_goal`, up to
`IkOptions::iterations` (3) damped-least-squares steps toward the desired pose per cycle,

```text
ẽ   = W [ p_des − p(q_goal) ;  log(R_des R(q_goal)ᵀ) ],   W = diag(I₃, w_r I₃),   J̃ = W J_g
δ   = argmin ½ ‖J̃ δ − ẽ‖² + ½ λ² ‖δ‖²   subject to   lb ≤ δ ≤ ub
      + (I − J̃_F⁺ J̃_F) k_null (posture − q_goal) dt       on the free joints F
```

with `J_g` the zero Jacobian at `q_goal` (not at the measured `q`), `λ` = 0.05 the damping
that keeps the step finite at a singularity, `w_r` = `IkOptions::rotation_weight` (0.1 m/rad),
`[lb, ub]` the cycle's box of the [joint position limit guard](#the-joint-position-limit-guard)
and the last term a drift toward `posture` (the start configuration unless set) at `k_null` =
1 /s, capped at 0.5 rad/s so that a far posture is approached rather than jumped at, projected
into the free joints' nullspace so it never moves the end effector. Iteration stops once
`‖ẽ‖` is below `tolerance` (1e-6, so 1 µm or 10 µrad). The limits are
`rate_limiting::JOINT_POSITION_LIMITS` for the FR3 and `rate_limiting::fer::JOINT_POSITION_LIMITS`
for the FER, from the URDFs in the repository. The
generator moves the pose by at most 0.3 mm a cycle under the default budget, so a step or
three from the previous solution keeps the residual near zero; the observer sees `‖ẽ‖` as
`CartesianSent::ik_error`. A pose out of reach, through a singularity or behind a joint limit
leaves a residual and `q_goal` lags instead of jumping, so the impedance never gets a step to
track.

## The joint velocity envelope

Nothing in the law bounds a joint's velocity: near a wrist singularity a modest turn of the hand
asks joints 5 and 7 to spin in opposite directions, about 2 rad/s of joint per rad/s of hand once
their axes are 30° apart. Teleoperated near such a pose, an FER can end in
`joint_velocity_violation` while the generator is well inside
its own budget and the arm follows the goal it is given.

**The cap.** No joint of the goal's step `Δq = q_goal − q_goal_prev` exceeds
`cap_i · 1 ms`, `cap_i = joint_velocity_fraction × limit_i` (0.7 by default), before the finite
difference that is `dq_goal`. On the joint interface the step is scaled as a whole by
`s = min(1, min_i cap_i · 1 ms / |Δq_i|)`, which keeps the joint-space direction. On the
Cartesian interface the cap is a bound of the IK's box, so the free joints make up what a
capped one cannot; a whole-step scale behind the IK remains as a backstop that does not bind
but for rounding. When the goal falls short the
generator's next cycle starts from what went out, not from its own plan: its velocity is its
end-of-cycle velocity scaled by `s`, not the step's mean (which sits above the end velocity
whenever the plan brakes inside the step, and would replan past the target), and its
acceleration is kept only where it brakes -- zeroed where it would speed the axis back up
(against a position limit, see [the restart](#the-joint-position-limit-guard)). The
leash's next reference is the capped goal too: the model's forward kinematics of `q_goal` on the
Cartesian interface, `q_goal` itself on the joint one. Without the re-anchor the generator would
keep planning from the velocity it wanted and arrive late, then decelerate from a speed the arm
never had. A stop's hold on the joint interface is not capped: that goal only moves with an arm
moved by hand, a leash ahead of it.

**The fade.** A joint lagging its capped goal catches up faster than the goal moves, and
unchecked that catch-up pull cancels the barrier below. So above `min(cap_i, onset_i −
FADE_BAND × limit_i)` of *measured* speed (never below rest), the law's torque along the
joint's motion -- the Coriolis term included -- fades out linearly to none at the barrier's
onset; torque against the motion is never scaled. Away from a position limit the `min` floors
the start `FADE_BAND` (0.15) under the onset regardless of the cap, because a narrower band adds
more gain than the loop tolerates: at the default cap 0.7 the band is exactly `[0.70, 0.85]` of
the limit, the same band a cap of 0.8 gets once floored, and both settle; fading instead from a
cap of 0.8 straight to the onset -- a third of that band -- rings on a 0.03 kg m² wrist
(`tests/velocity.rs`). Inside the position margin the band is `[0, onset]` instead, which on an
arm whose limit is flat -- the FER's -- is the narrower of the two; what bounds the gain there is
the position fade below.

**The barrier.** Above `velocity_barrier_fraction × limit_i` (0.85 by default) of *measured*
velocity the loop adds `−20 (|dq_i| − onset_i) sign(dq_i)` Nm, at most the joint's torque
limit, after the fade and before the clamp, the low-pass and the rate limiter. The gain is set
by stability, not by strength: the torque is computed from the velocity measured one cycle
before it acts and low-passed at 100 Hz, so a joint of effective inertia `I` damped with a
total `K` -- the law's own joint damping plus the barrier's, where it is active -- is stable
while `K × 1 ms / I` stays under 1; the rate limiter only slows the barrier's onset. The link
alone is light (0.003 kg m² on the FER's joint 7) but the drive's reflected inertia is not
(MuJoCo's FR3 carries 0.074 kg m² of armature on the wrist), and the `JOINT` preset's 15 Nm
s/rad on joint 7 is stable on an FER; the Cartesian preset puts at most about 6 Nm s/rad on a
wrist joint at 1200 N/m, so the barrier's 20 keeps `K` under 26. The offline velocity tests exercise the barrier behind the filter and rate limiter
(`tests/velocity.rs`). The stability bound assumes
that one cycle of delay; it says nothing about the push itself and does not claim to bound
it -- the velocity still overshoots the limit while the push is applied, before the barrier
settles it back under.

**Tests.** `tests/replay.rs` provides an optional replay check for the velocity cap,
tracking lag and error. `tests/sim_target_control/velocity_cap.rs` exercises the cap in
the simulator. These tests do not establish a bound on externally imposed motion.

The barrier's gain is lowered in proportion where `torque_limits` clamps a joint below its
preset (86 Nm on joints 1 to 4, 11.5 Nm on 5 to 7), so a lighter clamp does not leave a stiffer
barrier; at the presets it is 20 on every joint.

## The joint position limit guard

Clamping a goal inside a joint limit does not ensure that the moving arm can stop there.
The guard uses braking bounds in the IK, a stall flag that re-anchors the generator,
and a torque fade and spring near the measured joint limit. Three rules keep it
independent of the tool:

- **Joint space only.** Margin, box, pins, fade and spring are per joint; there is no workspace
  or link geometry.
- **The tool is the robot's.** `F_T_EE` and `EE_T_K` come from the state, the frame the targets
  are in. The rotation weight acts at that point: a 0.25 m tool with the end effector left at
  the flange turns 0.1 rad of rotation shortfall into 25 mm at the tip, so configure the tip.
- **No tool-specific constant.** Flange, Franka Hand, a third-party gripper and a long tool run
  the same numbers; the IK tests run all four.

**The box.** Per joint and side, with `s` the previous goal's distance to that limit,
`m = joint_position_margin` (0.05 rad), `f = joint_velocity_fraction` and `x = s − m`:

```text
E(x)  = min(v_flat, sqrt(k x + c²) − c)  for x > 0, else 0     braking envelope
u     = min(f · v_σ · Δt,  max(0, min(x, f · E(x) · Δt)))       step allowed toward that side
```

`v_σ` is the arm's velocity limit toward that side. On the FR3 it is position-dependent:
`min(dq_max, max(0, sqrt(2 ddq_dec d) − dq_offset))` at distance `d` from the limit,
less libfranka's tolerance, using Franka's robot-specification parameters. On the FER it
uses the flat `MAX_JOINT_VELOCITY`; no published position-dependent coefficients are
available for that arm.

`E` is the guard's own braking profile, `sqrt(k x + c²) − c` with `k = 2a`, held under `v_flat`,
the joint's velocity limit away from the position limits (the FER's flat `MAX_JOINT_VELOCITY`,
the FR3's `dq_max`): it stays under
`sqrt(2 a x)`, the speed a joint decelerating at `a` still stops from in `x`, and `c` (0.2 to
0.35 rad/s, libfranka's FR3 offsets) keeps its slope at the stop finite. `a` is the FR3's
published `ddq_dec` (2.585 to 17 rad/s²) on that arm and half the FER's published joint
acceleration limit (3.75 to 10 rad/s²) on the FER, which leaves the other half to the arm
following the goal and, unlike the FR3 constants, stays inside the FER's own rating on every
joint — 10 mrad before the stop it allows 0.19 rad/s on joint 1 and 0.17 on joint 5, against
0.16 and 0.33 for the FR3's. `v_σ` and `E` are both taken where the step ends
(`u ≤ f · v_σ(from ± u) · Δt`, `u ≤ f · E(x − u) · Δt`), and the loop's onset and fade start
below at the arm and a cycle on. The box always contains 0: a goal started inside the margin
stays where it is rather than jumping out.

**The solve.** A primal active set, warm-started from the last cycle: each pass pins the free
joint that violates its bound most, or releases the pinned joint whose multiplier points
inward, and re-solves the free joints with the pinned ones on their bounds, at most 12 passes
per solve (`CartesianSent::ik_passes`, over the cycle's solves). The result is the exact
box-constrained least-squares step, so
the free joints take over what a pinned one cannot give, and it is continuous however the set
changes, with no dwell. A solve that does not converge is clipped into the box; a non-finite
one keeps the previous goal. Fixed-size, nothing allocates. The posture is clamped `m + 0.3` rad
inside the limits and its pull fades out as the nearest joint closes from `m + 0.3` to `m`, so
it cannot drive a joint along the margin.

**Pins and the stall.** `pinned` reports per joint 0 free, ∓1 on the lower / upper position
bound, ∓2 on a velocity bound. A pin's pressure is `side · ĵ_iᵀ ẽ`, the residual along its unit
column and only into its bound, over the whole weighted task or its translation rows alone,
whichever pushes harder; above 2e-5 (weighted m per cycle) on a position pin the goal is
`stalled`, cleared on 20 of the last 40 cycles in which no position pin pushes at all.
Position first takes the goal as far as the pins allow, which lowers the very pressure that set
the flag, so a flag
cleared on a magnitude can cycle between the two modes. The generator is re-anchored on its own
anchor moved by the goal's own step, and never by more than it planned, which keeps the IK's lag
behind it without a goal closing a lag dragging
the desired along. What went out is always the goal's own step. While held, every cycle, its
velocity and acceleration *into* the wall are removed per block (translation, rotation), the lead
into the wall with them, and what is left is cut to the speed the goal's step carried, but for a
cycle of its acceleration: the generator neither winds up behind a wall nor runs ahead of a goal
that follows only in part. The wall is the direction of the motion the pins withhold and the free
joints cannot make up, refreshed by every push and dropped 20 cycles after the last. On a
velocity pin alone each block restarts at the fraction of its step that went out, which
`cap_scale` reports (exactly 1 on every other cycle). The removal is one-sided, so a reversed
target leaves the generator as fast as a start from rest. A `stop()` held at a wall the target
lies beyond counts as landed once the goal is within `Settle::tolerance` of the target on every
other axis and has come no tolerance closer to it for the time a generator needs from rest to
cover two of them (0.1 s at the default budget). On
the joint interface the step is scaled as a whole into the box, and a joint target, like a
posture, is refused inside `max(JOINT_LIMIT_INSET, m)`, so a joint goal always reaches its target.
The one goal not taken through the box is the joint hold: it is the leashed measured anchor,
which lies between the frozen goal and the measured `q` and so is never further out than the arm.

**At the measured arm.** With `b = m − POSITION_FADE_BAND` (0.03 rad), the law's torque toward
a limit fades out linearly as the measured joint closes from `m` to `b`; torque away from it is
never scaled. Inside `b` a spring of `POSITION_BARRIER_STIFFNESS × torque_limits_i` per rad
(1075 Nm/rad at 86 Nm, 144 at 11.5) pushes the joint out, at most its clamp, ramped in over the
loop's first 500 cycles so an arm started inside eases out. The velocity barrier's onset toward
the limit drops to `velocity_barrier_fraction × E(s − b)` and the velocity fade's cap to
`f × E(s − m)`, both at most the arm's limit there. A 0.02 rad overshoot of the margin reaches
`b`; 10 mrad from the limit the spring gives 21.5 Nm on joints 1 to 4 and 2.9 Nm on the wrist.
`tau_position` is the spring less the faded torque, so `tau = clamp(law + tau_envelope +
tau_position)`.

The two fades multiply, and inside the margin the velocity fade has no band of its own: its cap
is `f × E(s − m)`, which is 0 at and past the margin, so it starts at rest and runs to the onset
rather than over `FADE_BAND` of the limit. On the FER, whose limit is flat, that is the narrower
band, so what bounds the gain there is this position fade taking the same push to nothing over
the same rad. Both vanish linearly at `b`, so the product stays finite; it rises across the
margin and peaks at `m`, at `|law| / (velocity_barrier_fraction × E(POSITION_FADE_BAND))`. At the
86 Nm clamp and the default fraction that is 429 Nm s/rad on joint 2 and 357 on joint 4, 1.61 and
1.33 times the `|law| / (FADE_BAND × limit)` the band alone would give (266 and 268)
(`tests/torque_position.rs`); across the arm the factor runs 1.16 to 1.61, and the cycle's
lookahead adds about another 1 %. On the FR3 the limit has itself come down by the margin, so
the velocity band is the wider of the two there and `FADE_BAND` does bound the gain.

**The rotation weight.** When several joints are pinned at their bounds, the pose may be
unreachable and the IK must trade position against orientation. The default
`rotation_weight` is 0.1 m/rad: 1 mm of position error has the same weight as 10 mrad of
orientation error. The weight is fixed rather than derived from `cartesian_stiffness`,
which may be zero in rotation.

**Position first at a wall.** While the goal is stalled, each iteration first solves
translation within the joint bounds, then solves the full task with translation weighted
30 times more strongly than orientation. Orientation follows as far as the remaining
freedom allows. The solve can settle at a local optimum. The switch eases in and out over
0.5 s so the goal velocity has no step; away from a wall the weighted solve is unchanged.
Under a speed limit, translation takes priority within the velocity bounds too.

| constant | value | why |
|---|---|---|
| `ImpedanceOptions::joint_position_margin` | 0.05 rad, in [0.035, 0.5] | an overshoot of 0.02, the FR3 envelope's zero up to 12 mrad in, and the fade band |
| `IkOptions::rotation_weight` | 0.1 m/rad, in (0, 1] | the trade above; 1 weighs 1 m as 1 rad |
| `POSITION_FADE_BAND` | 0.02 rad | `b ≥ 0.015` keeps `E(s − b)` under the arm's velocity limit near the limit |
| `POSITION_BARRIER_STIFFNESS` | 12.5 /rad × torque clamp | stable on the wrist behind the 1 ms delay and the low-pass |
| active-set passes | 12 | bounds the work per solve; position first solves twice per iteration |
| stall on / quiet cycles | 2e-5 weighted m per cycle / 20 of the last 40 | above the damped solve's lag; the flag holds while a pin pushes at all, and a pin brushed now and then still clears it |
| posture distance | 0.3 rad beyond the margin | where the posture is clamped and its pull starts to fade |
| spring ramp | 500 cycles from the loop's start | an arm started inside `b` eases out |

**Recorded.** `CartesianSent` and `JointSent` carry `pinned` and `tau_position`,
`CartesianSent` also `stall_pressure`, `stalled` and `ik_passes`; `ik_error` is the weighted
norm. `CartesianSent` reports the solve itself too: `ik_step` and `ik_step_clipped` are the
largest raw step norm of the cycle's solves and how much of it the box took off, `ik_blend` is
the position-first blend, and `held` with `wall_age` say whether a wall was standing and how
long since it was last pushed on. The flight recorder's entities are on [Record and replay a
run](../howto/flight-recorder.md#replay-a-reflex).

**Limits.** Self-collision is not modelled; a joint position margin does not prevent it. The
spring is sized for an overshoot of the margin by 0.02 rad; a push that drives a joint further
meets at most its torque clamp, and the robot's own limit reflex remains the last stop. The guard
is exercised offline, on the model with a simulated plant (`tests/position.rs`,
`tests/active_set.rs`, `tests/ik/`, `tests/torque_position/`), not yet on a robot.

## Near full stretch: a self-motion the law cannot see

Near a singular configuration, the joints can move substantially while the tool's
position barely changes. Cartesian position gains cannot damp motion in that direction;
joint gains and, where orientation changes, rotational gains remain relevant.

Oscillation near full stretch is an unresolved limitation. The initiating mechanism is
not yet identified, and no specific remedy is implemented. A small end-effector error
does not establish that joint motion is well controlled. Use the recorded joint states,
IK diagnostics and commanded orientation when investigating this behavior.

## Anchoring without an echo: the leash

In the robot-controller backend the third rule of the [generator](./otg.md) re-anchors the
plan every cycle on the robot's echo of the last command (`O_T_EE_c`, `q_d`), so that the
backstop behind the generator can shape a command but never accumulate a lag. There is no
echo of a torque command: the robot reports `tau_J_d`, not a pose or joint goal. Neither
extreme works as a substitute. Anchoring on the measured pose would fold the spring's
deflection into the plan every cycle, so that a load would stall the plan short of the target
and a push on the arm would move it, since a compliant arm is *meant* to sit off its setpoint
under a force. Anchoring on the generator's own last output runs it open loop: held back by a
hand, an obstacle or an unreachable pose, the arm falls arbitrarily far behind a plan that
goes on without it, the spring force grows with the distance until a collision reflex ends
the session, and on release the arm springs to wherever the plan has got to.

The `Leash` is the middle: each cycle the anchor is the measured pose (the model's, for the
measured `q`, the frame the desired pose and the IK live in) pulled toward the previous
desired pose by at most `leash.translation` (0.025 m) and `leash.rotation` (0.15 rad),

```text
e_t = p_des − p_meas        e_r = log(R_des R_measᵀ)
s   = min(1, leash.translation / |e_t|, leash.rotation / |e_r|)
anchor = ( p_meas + s e_t,  exp(s e_r) R_meas )
```

and on the joint interface each joint's goal clamped to within `leash.joint` (0.1 rad) of
the measured joint, the feedforward velocity being the finite difference of that leashed
goal. While the arm follows, `s = 1` and the anchor *is* the previous desired, so the
generator runs from its own output and its budget is the whole budget, as before. Held back,
the desired stays within the leash of the arm, so the spring force on whoever holds it
is limited by the leashed position error. This is not a limit on total reaction force,
which also includes damping.
On the joint interface it is the torque clamp, not the leash, that bounds the torque: the
`JOINT` preset's 600 Nm/rad × 0.1 rad is 60 Nm on joints 1 to 4, under their 86 Nm clamp but
far over the 20 Nm joint threshold the examples set, which such a joint reaches at 0.033 rad
of error; on the wrist (250 / 150 / 50 Nm/rad) joints 5 and 6 meet the 11.5 Nm clamp before the
leash does.
Target control sets no collision thresholds; see
[Collision thresholds](../howto/target-control.md#backends). When the arm is let go the
generator resumes from where the arm is, under its budget. The leash keeps acting during the
stop's hold, so an arm held during `stop()` is not pulled harder either. A deflection under
load smaller than the leash costs nothing, and the from-start deviation guard is unchanged.
What the leash took off is reported to the observer as `leash_alteration` (m, or rad on the
joint interface) and `leash_angular_alteration` (rad), zero in normal tracking. The backstop
role otherwise passes to the torque clamp, the filter and the torque-rate limiter, which
shape the torque and leave the setpoint alone.

## The handover at start and stop

At the start, the desired pose is the model's pose of the measured configuration and
`q_goal = q`. The IK therefore starts with zero residual. The first command is
`−Kd dq + coriolis(q, dq)`.

At `stop()`, the generator lands on the last target and holds the setpoint for
`Settle::cycles` (250). It sets `motion_finished` once every joint moves slower than
`REST_JOINT_VELOCITY` (0.01 rad/s), or after `STOP_TIMEOUT_CYCLES` (5 s) more cycles.
The impedance law continues to act on the held goal until then. The velocity check
allows the arm to settle before control returns to the robot's controller.

The law itself, `impedance_torques`, is public at the crate root for a loop of your own.

## What a spring does not do

A spring has no integrator. The tracking error at a hold is the arm's residual force (load or
friction) over the stiffness, as it is under the robot's own impedance controller, so it
halves when the stiffness doubles; millimetre placement needs a higher stiffness. And the
leash bounds the *position* error, not the force: a fast push adds the damping term (50 to
90 N s/m times the speed), so there is no dedicated cap on the reaction force, spring and
damper together, beyond the torque clamp.
The backend is not yet validated on an FR3.

## Compared with the operational-space law

`examples/cartesian_impedance_active_control.rs` is libfranka's Cartesian impedance example:

```text
tau = Jᵀ (−K e − D J dq) + coriolis,   e = [ p − p_d ; −R vec(q_ee⁻¹ q_d) ]
```

with `K = diag(150, 150, 150, 10, 10, 10)` and `D = 2√K`. The two are first-order the same in
the six task directions, `Jᵀ Kx J (q_goal − q) ≈ Jᵀ Kx (x_goal − x)`, and differ in three
ways. The operational-space law measures the error in the task space, orientation through a
quaternion, and needs no inverse kinematics; the hybrid law measures it in joint space and
needs the joint goal, which is why the Cartesian interface runs the IK above. The
operational-space law leaves the nullspace free unless a separate term fills it, as
`cartesian_impedance_figure_eight` does with a nullspace joint spring through a projector;
the hybrid law fills it with `Kq` in the same expression. And a joint goal makes the two
interfaces one code path, with a joint target the degenerate case `Kx = 0`. Both add the
model's Coriolis term and leave gravity to the robot.
