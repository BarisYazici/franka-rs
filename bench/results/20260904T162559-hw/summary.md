# franka-rs vs C++ libfranka — jitter benchmark (20260904T162559)

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
| git_rev | cca30673309a1357e7bcb731b6b8d7708c239860 |
| duration_s | 30 |
| reps | 3 |
| cyclictest | None |
| stress_ng | None |

## Variant 3 — `model` (torque control with the robot model in the 1 kHz loop: `Robot::control(Torques)` / `Robot::control_torques`)

`model` is the time spent in the five `franka::Model` calls (`mass`, `coriolis`, `gravity`, `zeroJacobian`, `pose`); `compute` is the whole controller including those calls, the operational-space linear algebra and `limitRate`. `interval` is the time between consecutive control-callback entries.

| condition | client | cycles | model p50 (µs) | p99 | p99.9 | max | compute p50 (µs) | p99 | p99.9 | max | interval p50 (µs) | p99 | max | lost cycles | success avg | CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 30001 | 23.5 | 50.1 | 66.5 | 576.4 | 25.2 | 54.2 | 72.5 | 578.3 | 1000 | 1096 | 2341 | 0.0 | 1.000 | 5.9 |
| chrt -f 80 + mlockall | rust | 30001 | 12.8 | 34.5 | 38.7 | 65.8 | 14.1 | 37.4 | 43.2 | 66.8 | 1000 | 1085 | 2397 | 0.0 | 1.000 | 3.3 |

Controller sanity check (worst over the repetitions of each cell): the peak commanded torque and how far the end effector ever moved from where it started. The two clients should agree; they run the same math.

| condition | client | max abs tau (Nm) | max EE deviation (m) |
|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 4.017 | 0.0484 |
| chrt -f 80 + mlockall | rust | 3.980 | 0.0484 |

## FR3 hardware (<robot-ip>)

Real-arm runs. No container is involved; the harness probes the robot read-only before every run, recovers once from a `Reflex`, and drives the arm back to the ready pose with libfranka's `MotionGenerator` between runs. Both clients run with `--hardware`, which arms the |tau| and end-effector-deviation guards; a guard finishes the motion cleanly rather than killing the loop.

Stage **c** — status **ok**, 3 repetition(s) of 30 s, 20 s pause between runs, return-to-ready at speed 0.2, **0 reflex event(s)** (stage abandoned after 2).

| run | provenance | order | cycles | model p50 (µs) | p99 | max | compute p50 (µs) | p99 | max | interval p50 | p99 | max | lost | success min | success avg | CPU % | max abs tau (Nm) | max EE dev (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r1 | harness | 1/2, cpp first | 30001 | 21.5 | 51.9 | 166.4 | 23.3 | 55.8 | 194.1 | 1000 | 1106 | 2341 | 0 | 0.990 | 0.9997 | 4.7 | 3.966 | 0.0484 |
| rust_model_rt_r1 | harness | 2/2, cpp first | 30001 | 12.5 | 33.2 | 65.8 | 13.9 | 35.6 | 66.8 | 1000 | 1079 | 1501 | 0 | 1.000 | 1.0000 | 2.7 | 3.941 | 0.0484 |
| rust_model_rt_r2 | harness | 1/2, rust first | 30001 | 12.1 | 35.4 | 54.3 | 13.4 | 39.1 | 58.2 | 1000 | 1090 | 2347 | 0 | 0.990 | 0.9996 | 3.5 | 3.980 | 0.0484 |
| cpp_model_rt_r2 | harness | 2/2, rust first | 30001 | 21.3 | 46.3 | 229.0 | 23.1 | 51.7 | 231.5 | 1000 | 1097 | 2174 | 0 | 0.990 | 0.9998 | 7.4 | 3.946 | 0.0484 |
| cpp_model_rt_r3 | harness | 1/2, cpp first | 30001 | 27.6 | 51.9 | 576.4 | 29.3 | 55.0 | 578.3 | 1000 | 1086 | 2283 | 0 | 0.990 | 0.9997 | 5.6 | 4.017 | 0.0483 |
| rust_model_rt_r3 | harness | 2/2, cpp first | 30001 | 13.8 | 34.8 | 38.7 | 15.1 | 37.4 | 52.4 | 1000 | 1087 | 2397 | 0 | 0.990 | 0.9998 | 3.6 | 3.942 | 0.0483 |

### Guards

No guard tripped in any run (limits: |tau| <= 20.0 Nm, EE deviation <= 0.1 m).

### Control exceptions

None: every run's control loop ended normally.

## Per-run detail

| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | back | sr avg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r1 | 1/2, cpp first | 30001 | 30.00 | FIFO/80 | FIFO/99 | ok | 2341 | 12190 | 0 | 0 | 1.000 |
| cpp_model_rt_r2 | 2/2, rust first | 30001 | 30.00 | FIFO/80 | FIFO/99 | ok | 2174 | 6537 | 0 | 0 | 1.000 |
| cpp_model_rt_r3 | 1/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 2283 | 21921 | 0 | 0 | 1.000 |
| rust_model_rt_r1 | 2/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 1501 | 21633 | 0 | 0 | 1.000 |
| rust_model_rt_r2 | 1/2, rust first | 30001 | 30.00 | FIFO/80 | FIFO/99 | ok | 2347 | 5502 | 0 | 0 | 1.000 |
| rust_model_rt_r3 | 2/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 2397 | 27742 | 0 | 0 | 1.000 |

## Server-side cross-check (`docker logs`)

The simulator emitted no `State Update Stats` or lost-cycle lines for these runs.

## OS baseline (cyclictest)

```
cyclictest not installed; skipping the OS baseline
```

