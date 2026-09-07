# franka-rs vs C++ libfranka — jitter benchmark (20260905T191844)

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
| libfranka | <a libfranka 0.9.2 build>/libfranka.so.0.9 |
| rustc | rustc 1.96.0 (ac68faa20 2026-05-25) |
| gcc | c++ 11.4.0 |
| git_rev | 7ba49f9ea7eb0b87e092efc26e16be1448cd249d |
| duration_s | 30 |
| cyclictest | None |
| stress_ng | None |

## Variant 3 — `model` (torque control with the robot model in the 1 kHz loop: `Robot::control(Torques)` / `Robot::control_torques`)

`model` is the time spent in the five `franka::Model` calls (`mass`, `coriolis`, `gravity`, `zeroJacobian`, `pose`); `compute` is the whole controller including those calls, the operational-space linear algebra and `limitRate`. `interval` is the time between consecutive control-callback entries.

| condition | client | cycles | model p50 (µs) | p99 | p99.9 | max | compute p50 (µs) | p99 | p99.9 | max | interval p50 (µs) | p99 | max | lost cycles | success avg | CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 29872 | 7.7 | 16.7 | 25.4 | 40.7 | 9.5 | 21.8 | 29.9 | 42.8 | 999 | 1184 | 6427 | 87.0 | 0.988 | 3.0 |
| chrt -f 80 + mlockall | rust | 29869 | 13.1 | 31.8 | 43.1 | 53.6 | 14.4 | 33.6 | 44.8 | 55.4 | 999 | 1181 | 6348 | 91.7 | 0.987 | 3.2 |

Controller sanity check (worst over the repetitions of each cell): the peak commanded torque and how far the end effector ever moved from where it started. The two clients should agree; they run the same math.

| condition | client | max abs tau (Nm) | max EE deviation (m) |
|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 3.633 | 0.0472 |
| chrt -f 80 + mlockall | rust | 3.613 | 0.0472 |

## FER hardware (R)

Real-arm runs. No container is involved; the harness probes the robot read-only before every run, recovers once from a `Reflex`, and drives the arm back to the ready pose with libfranka's `MotionGenerator` between runs. Both clients run with `--hardware`, which arms the |tau| and end-effector-deviation guards; a guard finishes the motion cleanly rather than killing the loop.

Stage **A+C** — status **complete**, 3 repetition(s) of 30 s, 20 s pause between runs, return-to-ready at speed 0.2, **0 reflex event(s)** (stage abandoned after ?).

| run | provenance | order | cycles | model p50 (µs) | p99 | max | compute p50 (µs) | p99 | max | interval p50 | p99 | max | lost | success min | success avg | CPU % | max abs tau (Nm) | max EE dev (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rust_model_rt_r1 | harness | 1/2, rust first | 29858 | 8.3 | 27.4 | 53.6 | 9.7 | 29.6 | 55.0 | 999 | 1167 | 5325 | 102 | 0.930 | 0.9876 | 2.5 | 3.114 | 0.0472 |
| rust_model_rt_r2 | harness | 1/2, rust first | 29886 | 16.4 | 34.3 | 49.7 | 17.6 | 36.0 | 50.9 | 999 | 1179 | 6348 | 81 | 0.930 | 0.9892 | 3.7 | 3.613 | 0.0472 |
| cpp_model_rt_r2 | harness | 2/2, rust first | 29877 | 10.4 | 22.6 | 40.7 | 12.1 | 30.1 | 42.3 | 999 | 1174 | 5396 | 84 | 0.920 | 0.9889 | 3.6 | 3.633 | 0.0472 |
| cpp_model_rt_r3 | harness | 1/2, cpp first | 29866 | 5.0 | 10.8 | 40.4 | 7.0 | 13.6 | 42.8 | 999 | 1194 | 6427 | 90 | 0.940 | 0.9875 | 2.4 | 3.632 | 0.0472 |
| rust_model_rt_r3 | harness | 2/2, cpp first | 29864 | 14.7 | 33.8 | 53.4 | 15.9 | 35.2 | 55.4 | 999 | 1198 | 5616 | 92 | 0.920 | 0.9850 | 3.4 | 3.566 | 0.0472 |

### Guards

No guard tripped in any run (limits: |tau| <= 20.0 Nm, EE deviation <= 0.1 m).

### Control exceptions

None: every run's control loop ended normally.

## Per-run detail

| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | back | sr avg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r2 | 2/2, rust first | 29877 | 30.21 | FIFO/80 | FIFO/99 | ok | 5396 | 28722 | 84 | 0 | 0.989 |
| cpp_model_rt_r3 | 1/2, cpp first | 29866 | 30.21 | FIFO/80 | FIFO/99 | ok | 6427 | 7839 | 90 | 0 | 0.987 |
| rust_model_rt_r1 | 1/2, rust first | 29858 | 30.21 | FIFO/80 | FIFO/99 | ok | 5325 | 14095 | 102 | 0 | 0.988 |
| rust_model_rt_r2 | 1/2, rust first | 29886 | 30.21 | FIFO/80 | FIFO/99 | ok | 6348 | 1267 | 81 | 0 | 0.989 |
| rust_model_rt_r3 | 2/2, cpp first | 29864 | 30.21 | FIFO/80 | FIFO/99 | ok | 5616 | 9699 | 92 | 0 | 0.985 |

## Server-side cross-check (`docker logs`)

The simulator emitted no `State Update Stats` or lost-cycle lines for these runs.

