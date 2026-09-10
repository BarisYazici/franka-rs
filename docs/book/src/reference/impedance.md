# The impedance backend

[Target control](../howto/target-control.md) tracks the generator's setpoints with torques of
its own by default. This page is the derivation: where the law comes from, why it is a joint
law with a Cartesian term rather than the other way round, what the joint gains do to the
stiffness you feel, how the Cartesian interface gets a joint goal, how the generator is
anchored without an echo, what happens at the handover, and how it differs from the
operational-space law of the `cartesian_impedance_active_control` example. The options and
defaults are on the how-to page; the code is `robot/target_control/impedance.rs`,
`torque.rs` and `ik.rs`.

## The law and its provenance

```text
Kp  = Jᵀ Kx J + diag(Kq)          Kd = Jᵀ Kxd J + diag(Kqd)
tau = Kp (q_goal − q) + Kd (dq_goal − dq) + coriolis(q, dq),   clamped to ±torque_limits
```

This is the structure of `HybridJointImpedanceControl` from
[polymetis](https://github.com/facebookresearch/fairo/blob/main/polymetis/polymetis/python/torchcontrol/policies/impedance.py)
(its feedback module `HybridJointSpacePD` forms `Kp` and `Kd` exactly so), the controller
that DROID (Khazatsky et al., 2024, *DROID: A Large-Scale In-The-Wild Robot Manipulation
Dataset*, [arXiv:2403.12945](https://arxiv.org/abs/2403.12945)) ran on its Panda arms for
76k teleoperated trajectories. `ImpedanceGains::DROID` is its gains as they were, and with
`velocity_feedforward` off (`dq_goal = 0`, the damping on the absolute velocity) the law is
polymetis's. The default differs in two places. `ImpedanceGains::CARTESIAN` raises the
translational damping from 37 to 50, 50, 90 Ns/m: the arm's apparent mass at the end effector
near the ready pose is about 0.94 kg along x and y and 3.9 kg along z (from the model's mass
matrix), so 37 Ns/m against 750 N/m leaves z at a damping ratio of 0.34, which rings; the
defaults bring all three to about 0.8. And the damping acts on the velocity
error by default, with `dq_goal` the goal's velocity: the generator's on the joint interface,
the finite difference of the IK solution on the Cartesian one, zero while holding. Damping the
absolute velocity resists the motion the goal asks for, and a goal moving at `v` is tracked
`Kd v / Kp` behind it; on franka-sim 1.1.6 a 5 cm step peaks 12.3 mm behind without the
feedforward and 3.7 mm with it. `J` is `Model::zero_jacobian(Frame::EndEffector, state)`, the
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
e   = [ p_des − p(q_goal) ;  log(R_des R(q_goal)ᵀ) ]
dq  = J_gᵀ (J_g J_gᵀ + λ² I)⁻¹ e  +  (I − J_g⁺ J_g) k_null (posture − q_goal) dt
```

with `J_g` the zero Jacobian at `q_goal` (not at the measured `q`), `λ` = 0.05 the damping
that keeps the step finite at a singularity, and the second term a drift toward `posture`
(the start configuration unless set) at `k_null` = 1 /s, capped at 0.5 rad/s so that a far
posture is approached rather than jumped at, projected into the nullspace so it never moves
the end effector. If any joint of the step exceeds `max_step` (0.01 rad, so 10 rad/s) the
whole step is scaled so that the largest component equals it; then the result is clamped to
the joint position limits (`rate_limiting::JOINT_POSITION_LIMITS` for the FR3,
`rate_limiting::fer::JOINT_POSITION_LIMITS` for the FER, from the URDFs in the repository)
inset by `limit_margin` (0.02 rad), and iteration stops below `tolerance` (1e-6). A `posture`
outside those inset limits, like a joint target outside them, is refused with
`InvalidArgument` before anything starts. The
generator moves the pose by at most 0.3 mm a cycle under the default budget, so a step or
three from the previous solution keeps the residual near zero (`tolerance` ends the iteration
early on a landed target); the observer sees the residual as `CartesianSent::ik_error`. A
pose out of reach or through a singularity leaves a residual and `q_goal` moves toward it at
most `max_step` a cycle instead of jumping, so the impedance never gets a step to track.

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
the desired stays within the leash of the arm, so the spring force on whoever holds it is
bounded by the felt stiffness times the leash: roughly 25 to 30 N at the default gains at
the ready pose (990 to 1180 N/m × 0.025 m on the FER model), 18.75 N with `project_joint_gains`
(750 × 0.025).
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

At the start the desired pose is the model's pose of the measured configuration, not
`O_T_EE` (the two differ by the model's accuracy on a robot, and by 0.107 m on franka-sim,
whose `O_T_EE` is the joint-7 frame), so the IK's residual is zero and `q_goal = q`: the
first command is `−Kd dq + coriolis(q, dq)`, near zero on an arm at rest, and the robot's
controller hands over without a step. On franka-sim 1.1.6 the arm did not move over the
first 500 cycles of a session (measured change 0 to within floating point). At `stop()` the
generator lands on the last target;
the loop then holds the landed setpoint for `Settle::cycles` (250) and sets
`motion_finished` once every joint moves slower than `REST_JOINT_VELOCITY` (0.01 rad/s), or
after `STOP_TIMEOUT_CYCLES` (5 s) more cycles, the law kept on the held goal until then. The
generator's rest is not the arm's: an arm still closing its lag, handed to the robot's
controller at the moment of the finish, is held wherever it was (0.048 rad short on joint 7
on the simulator, before the gate). With the gate the arm is at rest on the setpoint and the
torques are near zero, so the robot's controller takes over from rest, as it does after the
`cartesian_impedance_active_control` example's final zero-torque command. On franka-sim 1.1.6
a 5 cm step lands 0.5 to 0.8 mm from the target. The law itself, `impedance_torques`, is
public at the crate root for a loop of your own.

## Measured on two FERs (2026-09-10)

Both Pandas of the earlier campaigns, system 4.2.1, `PREEMPT_RT` host, `FRANKA_REALTIME=enforce`,
default gains, collision thresholds 40 N unless stated. No run ended in a reflex except the one
that was meant to find the threshold.

| run | result |
|---|---|
| 5 s at rest, then `stop()` | first-cycle torque under 0.04 Nm, peak 0.22 Nm, tracking 0.13 mm (L) / 0.23 mm (R), `stop()` 0.44 s |
| the commander's 19 s stepped sequence | no reflex, IK residual under 1e-6, leash never bound, peak torque 4.4 Nm; tracking error at the holds 4.6 mm (L) / 2.8 mm (R), moving p95 9.6 / 8.7 mm; the robot's own controller on the same sequence: 3.7 mm at the holds, 4.4 mm moving |
| the same at Kx 1500 N/m (damping 75) | 2.7 mm at the holds, 6.3 mm moving |
| the sequence with the ±15° yaw sweep | no reflex, same tracking figures |
| joint targets (Python, `JOINT` preset, 20 % budget) | a 0.2 rad step on joint 1 landed within 0.6 mrad, a three-joint step within 4 mrad (joint 6), `stop()` mid-motion 0.9 s, arm `Idle` |
| a 4 cm circle at 5, 10 and 30 Hz (Python) | rate-independent, 8 to 10 mm p50 along the slow circle, back at the start within 7 to 9 mm |
| push tests, 40 N thresholds | two light pushes: 16.6 mm for 12 N, felt stiffness 725 N/m along the push, back within 2 mm in 0.3 s; a fast push reached 50 N in 250 ms at 25 mm and tripped `cartesian_reflex` |
| push tests from the other side, 60 N thresholds | 24.8 N at 22.8 mm quasi-static (about 1090 N/m felt), the leash held the error at exactly 25.0 mm under 45 to 47 N at 0.26 m/s, no reflex; a push that dragged the hand 12 cm and turned the wrist past 0.5 rad ended the loop through the deviation guard, the arm held in place |

Two things the numbers settle. The tracking error at rest scales with 1/K (4.6 mm at 750,
2.7 mm at 1500) and the robot's own external-force estimate reads 3 to 4 N at those holds:
a constant residual force of the arm (load or friction) that the robot's own impedance
controller deflects under as well; a spring has no integrator, so users who need millimetre
placement raise the stiffness. And the leash bounds the *position* error, not the force: a
fast push adds the damping term (50 to 90 N s/m times the speed), which is why 45 to 50 N
appeared at 0.25 m/s. A cap on the reaction force, spring and damper together, is the
follow-up. The FR3 was not reachable that day.

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
