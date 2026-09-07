# franka-rs vs C++ libfranka — jitter benchmark (20260904T162453)

## Machine

| field | value |
|---|---|
| hostname | redacted |
| cpu_model | a 2021 laptop-class x86-64 CPU |
| nproc | 12 |
| kernel | 6.8.0-generic |
| preempt_rt | False |
| sched_rt_runtime_us | 950000 |
| ulimit_l | unlimited |
| docker_image | franka-sim:dev |
| sim_cpu_shares | 4096 |
| libfranka | <libfranka 0.20.4 build>/libfranka.so.0.20 |
| rustc | rustc 1.96.0 (ac68faa20 2026-05-25) |
| gcc | c++ 11.4.0 |
| git_rev | b686e761d98a75617a9980730925ea7fcece1453 |
| duration_s | 30 |
| reps | 1 |
| cyclictest | None |
| stress_ng | None |

## Variant 3 — `model` (torque control with the robot model in the 1 kHz loop: `Robot::control(Torques)` / `Robot::control_torques`)

`model` is the time spent in the five `franka::Model` calls (`mass`, `coriolis`, `gravity`, `zeroJacobian`, `pose`); `compute` is the whole controller including those calls, the operational-space linear algebra and `limitRate`. `interval` is the time between consecutive control-callback entries.

| condition | client | cycles | model p50 (µs) | p99 | p99.9 | max | compute p50 (µs) | p99 | p99.9 | max | interval p50 (µs) | p99 | max | lost cycles | success avg | CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 29994 | 30.5 | 55.7 | 80.2 | 248.8 | 32.6 | 60.2 | 89.4 | 251.3 | 1000 | 1087 | 4548 | 4.0 | 0.999 | 5.9 |

Controller sanity check (worst over the repetitions of each cell): the peak commanded torque and how far the end effector ever moved from where it started. The two clients should agree; they run the same math.

| condition | client | max abs tau (Nm) | max EE deviation (m) |
|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 3.946 | 0.0483 |

## FR3 hardware (<robot-ip>)

Real-arm runs. No container is involved; the harness probes the robot read-only before every run, recovers once from a `Reflex`, and drives the arm back to the ready pose with libfranka's `MotionGenerator` between runs. Both clients run with `--hardware`, which arms the |tau| and end-effector-deviation guards; a guard finishes the motion cleanly rather than killing the loop.

Stage **b** — status **ok**, 1 repetition(s) of 30 s, 20 s pause between runs, return-to-ready at speed 0.2, **0 reflex event(s)** (stage abandoned after 2).

| run | provenance | order | cycles | model p50 (µs) | p99 | max | compute p50 (µs) | p99 | max | interval p50 | p99 | max | lost | success min | success avg | CPU % | max abs tau (Nm) | max EE dev (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r1 | harness | 1/2, cpp first | 29994 | 30.5 | 55.7 | 248.8 | 32.6 | 60.2 | 251.3 | 1000 | 1087 | 4548 | 4 | 0.960 | 0.9995 | 5.9 | 3.946 | 0.0483 |

### Guards

No guard tripped in any run (limits: |tau| <= 20.0 Nm, EE deviation <= 0.1 m).

### Control exceptions

None: every run's control loop ended normally.

## Per-run detail

| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | back | sr avg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r1 | 1/2, cpp first | 29994 | 30.00 | FIFO/80 | FIFO/99 | ok | 4548 | 16779 | 4 | 0 | 0.999 |

## Server-side cross-check (`docker logs`)

The simulator emitted no `State Update Stats` or lost-cycle lines for these runs.

## OS baseline (cyclictest)

```
cyclictest not installed; skipping the OS baseline
```

