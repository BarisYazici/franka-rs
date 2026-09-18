# Online trajectory generation

The `otg` module turns a stream of stepped targets into a 1 kHz command whose velocity,
acceleration and jerk never exceed a budget. It is what [target
control](../howto/target-control.md) runs on its thread, and it is public (`Otg`,
`MultiOtg<N>`, `CartesianOtg`, `OtgLimits`) for a loop of your own. This page is the
reasoning: why a generator and not a filter, how it plans, the three rules for putting it in
a control loop, why its budget is smaller than the rate limiter's, and what the loop does
every cycle.

## The problem

Most programs that want to move the arm are not 1 kHz programs: a planner, a vision loop, a
script on a socket, a person at a keyboard. What they produce is a stream of **targets**,
irregular, sometimes bursty, sometimes silent for seconds, and every one of them a step. Sent
as is, a 5 cm step reaches the robot as a 50 m/s jump and the motion generator refuses it with
`cartesian_motion_generator_velocity_discontinuity` (that is what `nonrealtime_commander --raw`
demonstrates). Something has to turn the steps into a motion, causally, with only the latest
target in hand.

The alternatives each fail one requirement. A **spline** needs future knots, and a commander
at 10 Hz, or silent for two seconds, cannot supply them. A **quintic** re-fitted from the
current state to the target every cycle is smooth but has no notion of the limits: its peak
velocity and acceleration scale with the step and shrink with the chosen duration, and no
duration is right for both a 1 mm and a 10 cm step. A **first-order low-pass filter** is
causal and cheap, but its first-cycle demand is the step times the gain (0.31 m/s for 5 cm
at 1 Hz), so the rate limiter behind it does the real work, the peak speed still scales with
the step, and the approach is exponential, never quite arriving.

## What the generator does

An *online trajectory generator* is a small state machine that owns the commanded position,
velocity and acceleration and, every cycle, re-plans the time-optimal jerk-limited profile
from that state to rest at whatever the latest target is, then follows it for one cycle. It
is causal (only the latest target), limit-respecting by construction (velocity, acceleration
and jerk within the budget on every cycle, including a 2 ms one after a lost packet), C2 (the
acceleration is continuous; only the jerk switches), time-optimal within its profile family,
and it lands exactly on the target and stays there.

The plan is the seven-segment profile: a jerk-limited transfer from the current `(v, a)` to a
peak velocity `v_p` with zero acceleration (three segments with closed-form durations), a
cruise at `v_p`, and the mirror transfer to rest. The one free parameter is `v_p`: `±v_max`
with a cruise when the target is far enough, otherwise a root of the displacement function,
which is increasing in `v_p` except for a hump next to `v_rd = v + a|a| / (2 j_max)`, the
velocity reached by ramping the acceleration straight to zero. So the domain is split at
`v_rd` and `v_rd ± a_max² / j_max`, every piece whose ends bracket the target is bisected,
and the shortest plan wins. This is the profile structure of Haschke, Weitnauer and Ritter
(*On-line planning of time-optimal, jerk-limited trajectories*, IROS 2008), evaluated one
cycle at a time the way Ruckig (Berscheid and Kröger, RSS 2021) does. No dependencies, no
allocation.

A target that moves mid-motion is re-planned from the current velocity and acceleration. A
target closer than the braking distance gives a root of the opposite sign: the profile passes
the target, brakes, and comes back, within the limits and without a jerk spike. A target that
stays for seconds is reached and held exactly; the target velocity is always zero, so this is
a generator for positional targets, not for velocity tracking.

```rust,no_run
# extern crate franka;
use franka::otg::{Otg, OtgLimits};
let limits = OtgLimits { max_velocity: 0.3, max_acceleration: 0.5, max_jerk: 20.0 };
let mut otg = Otg::new(0.0, limits).unwrap();
otg.set_target(0.05).unwrap();
let mut t = 0.0f64;
while otg.position() != 0.05 {
    otg.step(0.001);
    t += 0.001;
}
// The full norm budget on one axis; the target-control loop gives each axis 1/sqrt(3)
// of it, which stretches the same step to about 0.85 s.
assert!((t - 0.658).abs() < 0.01, "a 5 cm S-curve under these limits takes 0.658 s");
```

`MultiOtg<N>` runs one generator per axis (`with_limits` gives each axis limits of its own,
which seven joints need) and, when synchronised, stretches the faster axes to the slowest
one's duration by lowering their peak velocity (a second bisection); an axis that is exactly
braking to its target has no peak to lower and keeps its minimum duration. `CartesianOtg` is
`MultiOtg<3>`. Synchronised axes arrive together, so a diagonal target moves along a straight
line.

### The property test

`crates/franka-rs/src/otg/tests.rs` simulates two thousand random target sequences (every
third with random limits): steps at random times, holds up to three seconds, bursts of twenty
targets 5 ms apart, gaps. Every cycle it asserts that the velocity, acceleration and
finite-difference jerk stay within the limits; that a target beyond the braking distance is
never overshot; that every target is reached within 10 % of the time an admissible
brake-then-move profile would take; and, on the synchronised three-axis variant that every
eighth sequence also runs, that an axis at rest whose target did not change stays bit-exact.

## The three rules

The generator and the rate limiter must use compatible budgets and state. Otherwise the
limiter can alter the command while the generator continues planning from an unsent
position. The resulting lag can produce oscillation: the limiter bounds derivatives but
does not plan braking to a target. The module's regression tests cover this interaction.
Three rules keep the two in sync.

1. **The limits are per axis.** Two axes at full acceleration have a vector norm √2 above it,
   and a Cartesian budget is a norm (that is what `limit_rate_cartesian_pose` bounds). So the
   generator gets `OtgLimits::per_axis_for_norm(3)`: each limit divided by √3, which is also
   what keeps a synchronised diagonal move inside the budget.
2. **Step one nominal cycle per command** (`DELTA_T`, 1 ms), not the measured period. The
   robot and the rate limiter check every packet against a 1 ms budget, so a 2 ms step after
   a lost packet is a doubled velocity to them.
3. **Re-anchor on the robot's echo of the position every cycle** with `set_position(O_T_EE_c)`
   (or `q_d`), so that whatever runs behind the generator can shape one command but never
   accumulate a lag it plans against. The position only: the echoed twist is a mean over the
   cycle, half an acceleration step behind the generator's end-of-cycle state, and
   re-anchoring on it throttles the plan to a crawl.

A fourth rule applies only to a caller that moves the limits of a *running* generator, with
`Otg::set_limits` or `MultiOtg::set_limits`. Those keep the position, velocity, acceleration
and target exactly as they are, and the next step simply re-plans, so **raising** a limit may
step. **Lowering** one below the state the generator is already in may not: nothing in
`set_limits` clamps, but `set_state` — which rule 3's `set_position` runs every cycle — clamps
the stored velocity and acceleration into the limits, and the end of every `step` clamps them
again. A velocity truncated by `dv` in one cycle is `dv / dt` of acceleration in the command,
a thousand times `dv` at 1 kHz. So a lowered limit is walked down: `max_velocity` no faster
than the `max_acceleration` in force, `max_acceleration` no faster than `max_jerk`. Each clamp
is then at most one cycle of the next order, which is the generator's own bound rather than an
impulse. `max_jerk` is not stored in the state and clamps nothing, so it may step either way.

Per-axis budgets and nominal stepping prevent the backstop from binding under normal
conditions. Re-anchoring prevents any remaining alteration from accumulating as position
error. Re-anchoring alone does not ensure exact landing: a vector-norm clamp can distort
one axis's corrections while another saturates.

## Why the budget is smaller than the limiter's

The crate's rate limiter with `limit_rate = true` is a port of libfranka's, and its constants
(13 m/s² and 6500 m/s³ on an FER, 9 m/s² and 4500 m/s³ on an FR3) are what the robot accepts
*in Cartesian space*. The robot also runs inverse kinematics on every commanded pose and
checks the continuity of the result in **joint space**, and that is the check a stepped target
stream can trip. A Cartesian acceleration within the limiter's budget can still exceed a
joint's acceleration limit, depending on the pose and the inverse-kinematics mapping. See
[FER / Panda specifics](./fer.md).

Collision thresholds also apply independently of the kinematic limits. The external-force
estimate `O_F_ext_hat_K` can cross a configured threshold during motion and raise
`cartesian_reflex`, even when the commanded derivatives remain within their budgets.
Configure thresholds for the task; the commander example sets libfranka's example
thresholds explicitly.

Hence the defaults: a translational budget of **0.3 m/s, 0.5 m/s², 20 m/s³**, under which a
5 cm step along one axis becomes an S-curve that peaks at about 0.12 m/s and lands after
about 0.85 s (each axis gets 1/√3 of the norm budget); a rotational
budget of 0.5 rad/s, 1.0 rad/s², 20 rad/s³ (a fifth of the FR3's rotational velocity limit
and a seventeenth of its acceleration limit); and for joints 20 % of the negotiated
version's joint limits (`JointTargetControlOptions::scaled_limits(version, fraction)`).

The backstop behind the generator, `limit_rate_cartesian_pose` under the same budget, must
never bind, and to keep it from binding on noise it references the twist and acceleration
it *sent*, not the echoed `O_dP_EE_c` / `O_ddP_EE_c`: on FCI v10 the echo is `float32`, and
the rounding of a rotation matrix is worth about 200 rad/s³ of jerk against a 20 rad/s³
budget. The rotational axes of the generator also carry libfranka's pose-interface factor
(0.99) that the backstop applies to its rotational limits; without it the backstop binds on
every synchronised jerk.

## What the loop does every cycle

`start_cartesian_target_control` / `start_joint_target_control` spawn a named thread that
runs `control_cartesian_pose` / `control_joint_positions` with the loop's own rate limiter
on and the low-pass filter off. Every cycle, on that thread:

1. **Anchor.** The first cycle takes the robot's echo of its commanded position (`O_T_EE_c`,
   `q_d`) as the start, the initial target and the first setpoint, so the first command of the
   motion is the echo itself: on FCI v10 the first command is its own filter reference and
   would otherwise go out as a jump. `start_*` returns only once that cycle has run, so
   `target()` and `state()` are valid from the first call.
2. **Read the slot.** The latest target comes through a single-writer seqlock
   (`robot::target_control::TargetSlot`) the loop polls without blocking; a torn read keeps the
   previous target for that one cycle. `set_*` serialises its callers with a mutex on the user
   side only. The stop flag is read before the slot, so a target published just before
   `stop()` is not lost.
3. **Generate under the three rules.** Per-axis limits (the Cartesian budgets are norms and get
   `per_axis_for_norm(3)`; the joint limits are per joint already), one `DELTA_T` per command
   whatever the measured period, `set_position` on the echo before every re-plan, axes
   synchronised. The orientation runs on three more axes of the same generator, on the
   rotation vector of the orientation error in the base frame (`log(R_target R_echo^T)`),
   re-anchored at zero every cycle and composed back as `exp(step) R_echo`: a constant-axis
   turn under `rotation_limits`, arriving together with the translation.
4. **Backstop.** `limit_rate_cartesian_pose` / `limit_rate_joint_positions` under the same
   budget (the joint one tightened to the robot's velocity envelope at `q`), against the echo,
   with the loop's libfranka limiter behind it. Neither is meant to bind; the observer is told
   by how much the backstop moved the command (`backstop_alteration`).
5. **Guard.** If the *measured* position strays more than `max_deviation` from the start
   (0.30 m Cartesian, 1.0 rad joint by default) or the measured orientation turns more than
   `max_angular_deviation` (0.5 rad), the target freezes where the command is, the generator
   brings it to rest, and the loop ends with `FrankaError::Control` carrying
   `target_control::DEVIATION_MESSAGE`.
6. **Land, hold, finish.** After `stop()` the generator runs on until every axis has landed:
   within `Settle::tolerance` of the target (1e-3: 1 mm or 1 mrad), slower than
   `REST_VELOCITY` (1e-4 m/s or rad/s) and accelerating less than `REST_ACCELERATION` (0.05).
   The rest thresholds limit the discontinuity when entering the hold while allowing for
   the small residual motion caused by the FR3's `float32` command echo. Then the loop stops
   stepping the generator and sends the robot's echo of the last command, bit for bit and
   past the backstop, for `Settle::cycles` cycles (250), and sets `motion_finished` on one
   more of it. A motion never finishes on a moving command: the robot refuses exactly that
   with `cartesian_motion_generator_velocity_discontinuity`. If the generator has not landed
   within `STOP_TIMEOUT_CYCLES` (5000, five seconds) the same hold starts from wherever the
   command is.

Nothing allocates on the realtime thread after the start; the observer, called every cycle
with the state and what was sent, must keep it that way. Dropping a handle without `stop()`
requests the stop and detaches: the loop settles and finishes on its own, holding its
`Arc<Robot>` until it has.
