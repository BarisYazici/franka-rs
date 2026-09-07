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
| chrt -f 80 + mlockall | cpp | 29963 | 5.7 | 22.4 | 30.3 | 45.2 | 7.3 | 28.7 | 36.8 | 557.4 | 999 | 1187 | 5918 | 29.5 | 0.987 | 2.6 |
| chrt -f 80 + mlockall | rust | 29966 | 15.4 | 33.9 | 44.8 | 77.6 | 16.7 | 36.2 | 46.3 | 83.3 | 999 | 1172 | 7646 | 27.5 | 0.986 | 3.5 |

Controller sanity check (worst over the repetitions of each cell): the peak commanded torque and how far the end effector ever moved from where it started. The two clients should agree; they run the same math.

| condition | client | max abs tau (Nm) | max EE deviation (m) |
|---|---|---|---|
| chrt -f 80 + mlockall | cpp | 3.997 | 0.0480 |
| chrt -f 80 + mlockall | rust | 4.005 | 0.0481 |

## FER hardware (L)

Real-arm runs. No container is involved; the harness probes the robot read-only before every run, recovers once from a `Reflex`, and drives the arm back to the ready pose with libfranka's `MotionGenerator` between runs. Both clients run with `--hardware`, which arms the |tau| and end-effector-deviation guards; a guard finishes the motion cleanly rather than killing the loop.

Stage **A+C** — status **complete**, 4 repetition(s) of 30 s, 20 s pause between runs, return-to-ready at speed 0.2, **0 reflex event(s)** (stage abandoned after ?).

| run | provenance | order | cycles | model p50 (µs) | p99 | max | compute p50 (µs) | p99 | max | interval p50 | p99 | max | lost | success min | success avg | CPU % | max abs tau (Nm) | max EE dev (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r1 | harness | 1/2, cpp first | 29964 | 4.3 | 22.6 | 45.2 | 6.0 | 30.1 | 557.4 | 999 | 1161 | 5320 | 28 | 0.930 | 0.9888 | 2.4 | 3.960 | 0.0480 |
| rust_model_rt_r1 | harness | 1/2, rust first | 29968 | 16.7 | 38.6 | 77.6 | 17.9 | 39.9 | 78.9 | 999 | 1173 | 7646 | 27 | 0.890 | 0.9851 | 3.6 | 3.920 | 0.0481 |
| rust_model_rt_r2 | harness | 1/2, rust first | 29962 | 11.7 | 26.7 | 56.1 | 13.2 | 32.2 | 57.0 | 999 | 1166 | 7219 | 33 | 0.880 | 0.9895 | 3.2 | 3.983 | 0.0479 |
| cpp_model_rt_r2 | harness | 2/2, rust first | 29967 | 4.4 | 22.3 | 43.4 | 6.0 | 27.3 | 47.9 | 999 | 1233 | 5550 | 26 | 0.920 | 0.9868 | 2.4 | 3.939 | 0.0479 |
| cpp_model_rt_r3 | harness | 1/2, cpp first | 29959 | 9.9 | 22.6 | 35.4 | 11.5 | 30.2 | 43.6 | 999 | 1208 | 5634 | 35 | 0.890 | 0.9856 | 2.9 | 3.941 | 0.0479 |
| rust_model_rt_r3 | harness | 2/2, cpp first | 29968 | 16.3 | 34.9 | 64.4 | 17.5 | 36.0 | 65.3 | 1000 | 1187 | 6730 | 25 | 0.900 | 0.9849 | 3.6 | 4.005 | 0.0480 |
| rust_model_rt_r4 | harness | 1/2, rust first | 29968 | 17.1 | 35.4 | 72.3 | 18.2 | 36.7 | 83.3 | 999 | 1161 | 5876 | 25 | 0.920 | 0.9854 | 3.7 | 3.950 | 0.0480 |
| cpp_model_rt_r4 | harness | 2/2, rust first | 29962 | 4.2 | 22.1 | 43.2 | 5.8 | 27.3 | 47.8 | 1000 | 1146 | 5918 | 29 | 0.910 | 0.9875 | 2.6 | 3.997 | 0.0480 |

### Guards

No guard tripped in any run (limits: |tau| <= 20.0 Nm, EE deviation <= 0.1 m).

### Control exceptions

None: every run's control loop ended normally.

## Per-run detail

| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | back | sr avg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_rt_r1 | 1/2, cpp first | 29964 | 30.21 | FIFO/80 | FIFO/99 | ok | 5320 | 26063 | 28 | 0 | 0.989 |
| cpp_model_rt_r2 | 2/2, rust first | 29967 | 30.21 | FIFO/80 | FIFO/99 | ok | 5550 | 23711 | 26 | 0 | 0.987 |
| cpp_model_rt_r3 | 1/2, cpp first | 29959 | 30.21 | FIFO/80 | FIFO/99 | ok | 5634 | 9176 | 35 | 0 | 0.986 |
| cpp_model_rt_r4 | 2/2, rust first | 29962 | 30.21 | FIFO/80 | FIFO/99 | ok | 5918 | 19542 | 29 | 0 | 0.987 |
| rust_model_rt_r1 | 1/2, rust first | 29968 | 30.21 | FIFO/80 | FIFO/99 | ok | 7646 | 1655 | 27 | 0 | 0.985 |
| rust_model_rt_r2 | 1/2, rust first | 29962 | 30.21 | FIFO/80 | FIFO/99 | ok | 7219 | 8780 | 33 | 0 | 0.989 |
| rust_model_rt_r3 | 2/2, cpp first | 29968 | 30.21 | FIFO/80 | FIFO/99 | ok | 6730 | 5666 | 25 | 0 | 0.985 |
| rust_model_rt_r4 | 1/2, rust first | 29968 | 30.21 | FIFO/80 | FIFO/99 | ok | 5876 | 14519 | 25 | 0 | 0.985 |

## Server-side cross-check (`docker logs`)

The simulator emitted no `State Update Stats` or lost-cycle lines for these runs.

