# franka-rs vs C++ libfranka — jitter benchmark (20260904T152938)

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
| git_rev | a987dc56aaad3cdec540402cabe72b253e63f9d9 |
| duration_s | 30 |
| reps | 3 |
| cyclictest | None |
| stress_ng | None |

## Variant 3 — `model` (torque control with the robot model in the 1 kHz loop: `Robot::control(Torques)` / `Robot::control_torques`)

`model` is the time spent in the five `franka::Model` calls (`mass`, `coriolis`, `gravity`, `zeroJacobian`, `pose`); `compute` is the whole controller including those calls, the operational-space linear algebra and `limitRate`. `interval` is the time between consecutive control-callback entries.

| condition | client | cycles | model p50 (µs) | p99 | p99.9 | max | compute p50 (µs) | p99 | p99.9 | max | interval p50 (µs) | p99 | max | lost cycles | success avg | CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| plain (SCHED_OTHER) | cpp | 30001 | 27.3 | 56.9 | 79.0 | 263.4 | 29.3 | 60.7 | 84.6 | 266.8 | 1000 | 1047 | 2659 | 0.0 | 1.000 | 5.3 |
| plain (SCHED_OTHER) | rust | 30001 | 9.6 | 33.3 | 37.1 | 193.6 | 11.5 | 37.0 | 44.0 | 196.1 | 1000 | 1048 | 4207 | 0.0 | 1.000 | 2.9 |
| chrt -f 80 + mlockall | cpp | 30001 | 30.8 | 67.0 | 96.3 | 227.7 | 33.3 | 73.2 | 107.0 | 234.4 | 1000 | 1084 | 5449 | 0.0 | 1.000 | 5.7 |
| chrt -f 80 + mlockall | rust | 30001 | 10.8 | 32.9 | 36.8 | 75.6 | 12.5 | 37.0 | 42.8 | 76.7 | 1000 | 1047 | 3628 | 0.0 | 1.000 | 2.9 |

Controller sanity check (worst over the repetitions of each cell): the peak commanded torque and how far the end effector ever moved from where it started. The two clients should agree; they run the same math.

| condition | client | max abs tau (Nm) | max EE deviation (m) |
|---|---|---|---|
| plain (SCHED_OTHER) | cpp | 1.940 | 0.0479 |
| plain (SCHED_OTHER) | rust | 1.940 | 0.0479 |
| chrt -f 80 + mlockall | cpp | 1.940 | 0.0479 |
| chrt -f 80 + mlockall | rust | 1.940 | 0.0479 |

## Per-run detail

| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | back | sr avg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_model_plain_r1 | 1/2, cpp first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 2041 | 10843 | 0 | 0 | 1.000 |
| cpp_model_plain_r2 | 2/2, rust first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 2659 | 10011 | 0 | 0 | 1.000 |
| cpp_model_plain_r3 | 1/2, cpp first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 1627 | 27755 | 0 | 0 | 1.000 |
| cpp_model_rt_r1 | 1/2, cpp first | 30001 | 30.09 | FIFO/80 | FIFO/99 | ok | 4485 | 11227 | 0 | 0 | 1.000 |
| cpp_model_rt_r2 | 2/2, rust first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 4527 | 17349 | 0 | 0 | 1.000 |
| cpp_model_rt_r3 | 1/2, cpp first | 30001 | 30.19 | FIFO/80 | FIFO/99 | ok | 5449 | 19134 | 0 | 0 | 1.000 |
| rust_model_plain_r1 | 2/2, cpp first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 1740 | 26565 | 0 | 0 | 1.000 |
| rust_model_plain_r2 | 1/2, rust first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 3038 | 5627 | 0 | 0 | 1.000 |
| rust_model_plain_r3 | 2/2, cpp first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 4207 | 25958 | 0 | 0 | 1.000 |
| rust_model_rt_r1 | 2/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 1712 | 15841 | 0 | 0 | 1.000 |
| rust_model_rt_r2 | 1/2, rust first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 2008 | 3637 | 0 | 0 | 1.000 |
| rust_model_rt_r3 | 2/2, cpp first | 30001 | 30.02 | FIFO/80 | FIFO/99 | ok | 3628 | 17420 | 0 | 0 | 1.000 |

## Server-side cross-check (`docker logs`)

### cpp_model_plain_r1

```
2026-09-04 13:30:07,887 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:30:08,888 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:30:09,889 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:30:10,890 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### cpp_model_plain_r2

```
2026-09-04 13:31:49,847 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:31:50,848 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:31:51,849 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:31:52,850 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_model_plain_r3

```
2026-09-04 13:32:24,891 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:32:25,892 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:32:26,893 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:32:27,893 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### cpp_model_rt_r1

```
2026-09-04 13:33:33,424 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:33:34,424 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:33:35,425 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:33:36,426 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### cpp_model_rt_r2

```
2026-09-04 13:35:15,443 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:35:16,444 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:35:17,444 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:35:18,444 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### cpp_model_rt_r3

```
2026-09-04 13:35:50,503 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 13:35:51,504 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:35:52,505 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:35:53,506 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### rust_model_plain_r1

```
2026-09-04 13:30:41,345 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:30:42,345 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:30:43,345 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 13:30:44,346 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_model_plain_r2

```
2026-09-04 13:31:16,380 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:31:17,381 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:31:18,381 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:31:19,382 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### rust_model_plain_r3

```
2026-09-04 13:32:58,344 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:32:59,344 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 13:33:00,344 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:33:01,345 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### rust_model_rt_r1

```
2026-09-04 13:34:06,970 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:34:07,971 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:34:08,972 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:34:09,973 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### rust_model_rt_r2

```
2026-09-04 13:34:41,976 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:34:42,977 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:34:43,978 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:34:44,979 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### rust_model_rt_r3

```
2026-09-04 13:36:24,152 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:36:25,153 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:36:26,154 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 13:36:27,155 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

## OS baseline (cyclictest)

```
cyclictest not installed; skipping the OS baseline
```

