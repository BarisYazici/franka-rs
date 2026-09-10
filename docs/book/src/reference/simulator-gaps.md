# Simulator gaps

[franka-sim](https://github.com/BarisYazici/libfranka-sim) speaks the real FCI wire protocol
over a MuJoCo model, and the crate's integration tests drive it as they would an arm. It is
not an arm. This page lists the known divergences from a real robot, so that a result on the
simulator is read for what it is. How to run the tests is on [Test against the
simulator](../howto/simulator-tests.md).

## What the simulator is not

- It does not run under `PREEMPT_RT`, and neither does CI, so every test and example connects
  with `RealtimeConfig::Ignore`. Timing measured against it is a same-box A/B, not an FCI
  qualification; see [Benchmarks](./benchmarks.md).
- There are two images. The FR3 / FCI v10 image is published
  (`ghcr.io/barisyazici/franka-sim:latest`). The FER / FCI v5 image, `franka-sim:panda-v5`,
  is a local build and not published; CI's `sim-fer-v5` job is gated on the
  `FRANKA_SIM_FER_IMAGE` repository variable and skipped until someone sets it, so the v5
  simulator suite otherwise runs locally, and CI covers the v5 protocol offline through
  `wire_sizes_v5.rs`, `fer_native_conformance.rs` and the mock-FCI unit tests.

## Known gaps

Every gap is pinned by a characterisation assertion that names the stub in its failure
message, so a rebuilt image tightens the test instead of breaking it silently.

| gap | images | consequence | pinned in |
|---|---|---|---|
| `O_ddP_O` is `[0, 0, 0]` | both | `Model::gravity(&state)` is identically zero; use `gravity_q` with an explicit vector. | `sim_v5_commands.rs` |
| `O_T_EE` is the joint-7 frame | both | 0.107 m short of the flange along the tool z axis (measured 0.10700011 m), with `F_T_EE` and `NE_T_EE` both identity. | `sim_commands.rs`, `sim_v5_commands.rs` |
| `--enforce-motion-limits` uses the FR3's limit tables | `panda-v5` | The FER's jerk and acceleration limits are larger on most joints (`MAX_JOINT_JERK = [7500, 3750, 5000, 6250, 7500, 10000, 10000]` against a flat 5000), so whenever the client's FER rate limiter saturates it emits a command a real FER accepts and this image rejects. Do not home with `MotionGenerator` on this image with limits enforced, and read a reflex threshold as the FR3's. | `sim_v5_stop_and_reflex.rs` |
| joint 1 ignores velocity commands | `panda-v5` | The velocity servo uses FR3 gains; on the Menagerie `franka_emika_panda` model joint 1 limit-cycles at the 500 Hz Nyquist frequency with ±0.36 rad/s, so the clipped torque averages to zero. Commanding 0.1 rad/s for 1 s moved joint 1 by −0.00189 rad against about 0.099 on joints 2, 4 and 7. Position commands on joint 1 are fine. | `sim_v5_motions.rs` (`active_control` module) |
| `GetCartesianLimit` is a stub | `panda-v5` | `kSuccess` with an all-zero 154-byte body, so `virtual_wall(1)` returns zeros and `active: false`. The framing is real, the content is not; only a real FER can confirm the field mapping. | `sim_v5_commands.rs` |
| `StopMove` answers `kSuccess` | `panda-v5` | A real robot, and the FR3 image, answer `kPreempted`. `control_torques` then returns `Protocol("Unexpected reply to a Move command")` instead of the usual preemption `ControlException`. | `sim_v5_stop_and_reflex.rs` |
| no graspable object | `panda-v5` | The image predates `--gripper-object-width`, so a *successful* grasp is only covered on the FR3 image (`--gripper-object-width 0.04`). The gripper protocol is byte-identical on both versions anyway. | `sim_gripper.rs` (FR3) |
| no `franka-sim-check` binary | `panda-v5` | The test harness falls back to its own readiness probe: a version-5 `Connect` handshake plus one 2373-byte UDP datagram. | `crates/franka-sim-test/src/docker.rs` |

## The joint-side continuity check

A real robot runs inverse kinematics on every commanded Cartesian pose and checks the
continuity of the result in joint space (see [FER / Panda specifics](./fer.md)). Since
franka-sim 1.1.5 the FR3 image does the same under `--enforce-motion-limits`, scaled by
`--joint-discontinuity-scale` (1.0 is the robot's own limit); its FR3 acceleration table,
10 rad/s², was confirmed at scale 1.0 on a real FR3 on 2026-09-09. The `panda-v5` image has
no joint-side check and accepts what a real FER refuses. `sim_target_control` needs
franka-sim 1.1.6 or later.
