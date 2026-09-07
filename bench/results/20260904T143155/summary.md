# franka-rs vs C++ libfranka — jitter benchmark (20260904T143155)

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
| git_rev | 15a00b2810cd0693757cf476c7cb29945584dac0 |
| duration_s | 30 |
| reps | 3 |
| cyclictest | None |
| stress_ng | None |

## Variant 1 — `control` (libfranka `Robot::control` / `Robot::control_joint_velocities`, `limit_rate = true`)

`interval` is the time between consecutive control-callback entries (the 1 kHz cycle as the client sees it).

| condition | client | cycles | interval p50 (µs) | p99 | p99.9 | max | lost cycles | lost states | max consec | success min | success avg | CPU % | server lost-cycle lines |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| plain (SCHED_OTHER) | cpp | 30001 | 1000 | 1061 | 1142 | 1776 | 0.0 | 0.0 | 0 | 1.000 | 1.000 | 2.7 | 0 |
| plain (SCHED_OTHER) | rust | 30001 | 1000 | 1057 | 1147 | 4754 | 0.0 | 0.0 | 0 | 1.000 | 1.000 | 2.4 | 0 |
| chrt -f 80 + mlockall | cpp | 30000 | 1000 | 1071 | 1171 | 4665 | 0.3 | 0.7 | 1 | 0.970 | 1.000 | 2.7 | 0 |
| chrt -f 80 + mlockall | rust | 30000 | 1000 | 1050 | 1136 | 4719 | 0.3 | 0.7 | 1 | 0.970 | 1.000 | 2.4 | 0 |
| chrt -f 80 + mlockall + CPU hog | cpp | 30001 | 1000 | 1341 | 3859 | 8033 | 0.0 | 0.0 | 0 | 1.000 | 1.000 | 2.4 | 0 |
| chrt -f 80 + mlockall + CPU hog | rust | 30000 | 1000 | 1439 | 4012 | 9857 | 0.3 | 0.7 | 1 | 0.970 | 1.000 | 1.9 | 0 |

## Variant 2 — `active` (`startJointVelocityControl` / `start_joint_velocity_control`, `readOnce` → `writeOnce`)

`interval` is the time between consecutive `readOnce()` returns; `latency` is `readOnce()` return → `writeOnce()` return, i.e. the client's recv→send path.

| condition | client | cycles | interval p50 (µs) | p99 | p99.9 | max | latency p50 (µs) | p99 | p99.9 | max | lost cycles | max consec | success avg | CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| plain (SCHED_OTHER) | cpp | 30001 | 1000 | 1066 | 1186 | 4544 | 17 | 39 | 65 | 639 | 0.0 | 0 | 1.000 | 3.0 |
| plain (SCHED_OTHER) | rust | 30000 | 1000 | 1065 | 1587 | 4746 | 12 | 26 | 37 | 501 | 0.7 | 1 | 1.000 | 2.4 |
| chrt -f 80 + mlockall | cpp | 30001 | 1000 | 1046 | 1140 | 4252 | 17 | 36 | 58 | 816 | 0.0 | 0 | 1.000 | 3.0 |
| chrt -f 80 + mlockall | rust | 30001 | 1000 | 1054 | 1134 | 4772 | 12 | 28 | 40 | 396 | 0.0 | 0 | 1.000 | 2.4 |
| chrt -f 80 + mlockall + CPU hog | cpp | 30001 | 1000 | 1093 | 2866 | 7106 | 16 | 28 | 86 | 426 | 0.0 | 0 | 1.000 | 2.8 |
| chrt -f 80 + mlockall + CPU hog | rust | 30001 | 1000 | 1102 | 2412 | 6083 | 8 | 14 | 38 | 381 | 0.0 | 0 | 1.000 | 1.7 |

## Per-run detail

| run | order in cell | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | back | sr avg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cpp_active_plain_r1 | 1/2, cpp first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 1779 | 17484 | 0 | 0 | 1.000 |
| cpp_active_plain_r2 | 2/2, rust first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 2022 | 6567 | 0 | 0 | 1.000 |
| cpp_active_plain_r3 | 1/2, cpp first | 30001 | 30.06 | OTHER/0 | FIFO/99 | - | 4544 | 23938 | 0 | 0 | 1.000 |
| cpp_active_rt_load_r1 | 1/2, cpp first | 30001 | 30.20 | FIFO/80 | FIFO/99 | ok | 4772 | 8502 | 0 | 0 | 1.000 |
| cpp_active_rt_load_r2 | 2/2, rust first | 30001 | 30.27 | FIFO/80 | FIFO/99 | ok | 7106 | 12274 | 0 | 0 | 1.000 |
| cpp_active_rt_load_r3 | 1/2, cpp first | 30001 | 30.07 | FIFO/80 | FIFO/99 | ok | 4647 | 27609 | 0 | 0 | 1.000 |
| cpp_active_rt_r1 | 1/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 1616 | 21297 | 0 | 0 | 1.000 |
| cpp_active_rt_r2 | 2/2, rust first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 1255 | 26952 | 0 | 0 | 1.000 |
| cpp_active_rt_r3 | 1/2, cpp first | 30001 | 30.03 | FIFO/80 | FIFO/99 | ok | 4252 | 16812 | 0 | 0 | 1.000 |
| cpp_control_plain_r1 | 1/2, cpp first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 1318 | 15756 | 0 | 0 | 1.000 |
| cpp_control_plain_r2 | 2/2, rust first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 1776 | 29002 | 0 | 0 | 1.000 |
| cpp_control_plain_r3 | 1/2, cpp first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 1573 | 19928 | 0 | 0 | 1.000 |
| cpp_control_rt_load_r1 | 1/2, cpp first | 30001 | 30.86 | FIFO/80 | FIFO/99 | ok | 6670 | 7421 | 0 | 0 | 1.000 |
| cpp_control_rt_load_r2 | 2/2, rust first | 30001 | 30.22 | FIFO/80 | FIFO/99 | ok | 4321 | 21646 | 0 | 0 | 1.000 |
| cpp_control_rt_load_r3 | 1/2, cpp first | 30001 | 30.22 | FIFO/80 | FIFO/99 | ok | 8033 | 3587 | 0 | 0 | 1.000 |
| cpp_control_rt_r1 | 1/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 2731 | 29455 | 0 | 0 | 1.000 |
| cpp_control_rt_r2 | 2/2, rust first | 29999 | 30.02 | FIFO/80 | FIFO/99 | ok | 4665 | 26787 | 1 | 0 | 1.000 |
| cpp_control_rt_r3 | 1/2, cpp first | 30001 | 30.03 | FIFO/80 | FIFO/99 | ok | 3371 | 22343 | 0 | 0 | 1.000 |
| rust_active_plain_r1 | 2/2, cpp first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 4217 | 26787 | 0 | 0 | 1.000 |
| rust_active_plain_r2 | 1/2, rust first | 30001 | 30.12 | OTHER/0 | FIFO/99 | - | 4739 | 15275 | 0 | 0 | 1.000 |
| rust_active_plain_r3 | 2/2, cpp first | 29997 | 30.01 | OTHER/0 | FIFO/99 | - | 4746 | 1130 | 2 | 0 | 1.000 |
| rust_active_rt_load_r1 | 2/2, cpp first | 30001 | 30.19 | FIFO/80 | FIFO/99 | ok | 4474 | 16262 | 0 | 0 | 1.000 |
| rust_active_rt_load_r2 | 1/2, rust first | 30001 | 30.19 | FIFO/80 | FIFO/99 | ok | 6083 | 71 | 0 | 0 | 1.000 |
| rust_active_rt_load_r3 | 2/2, cpp first | 30001 | 30.04 | FIFO/80 | FIFO/99 | ok | 4098 | 15909 | 0 | 0 | 1.000 |
| rust_active_rt_r1 | 2/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 4772 | 27282 | 0 | 0 | 1.000 |
| rust_active_rt_r2 | 1/2, rust first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 1990 | 14700 | 0 | 0 | 1.000 |
| rust_active_rt_r3 | 2/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 2064 | 18868 | 0 | 0 | 1.000 |
| rust_control_plain_r1 | 2/2, cpp first | 30001 | 30.01 | OTHER/0 | FIFO/99 | - | 2876 | 15720 | 0 | 0 | 1.000 |
| rust_control_plain_r2 | 1/2, rust first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 1934 | 6665 | 0 | 0 | 1.000 |
| rust_control_plain_r3 | 2/2, cpp first | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 4754 | 4963 | 0 | 0 | 1.000 |
| rust_control_rt_load_r1 | 2/2, cpp first | 30001 | 31.02 | FIFO/80 | FIFO/99 | ok | 9857 | 8329 | 0 | 0 | 1.000 |
| rust_control_rt_load_r2 | 1/2, rust first | 29999 | 30.41 | FIFO/80 | FIFO/99 | ok | 8139 | 20105 | 1 | 0 | 1.000 |
| rust_control_rt_load_r3 | 2/2, cpp first | 30001 | 30.14 | FIFO/80 | FIFO/99 | ok | 5086 | 3022 | 0 | 0 | 1.000 |
| rust_control_rt_r1 | 2/2, cpp first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 2623 | 11093 | 0 | 0 | 1.000 |
| rust_control_rt_r2 | 1/2, rust first | 30001 | 30.01 | FIFO/80 | FIFO/99 | ok | 1217 | 3642 | 0 | 0 | 1.000 |
| rust_control_rt_r3 | 2/2, cpp first | 29999 | 30.01 | FIFO/80 | FIFO/99 | ok | 4719 | 27035 | 1 | 0 | 1.000 |

## Server-side cross-check (`docker logs`)

### cpp_active_plain_r1

```
2026-09-04 12:35:50,315 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:35:51,316 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:35:52,316 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:35:53,317 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_plain_r2

```
2026-09-04 12:37:32,337 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:37:33,338 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:37:34,339 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:37:35,339 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_plain_r3

```
2026-09-04 12:38:07,339 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:38:08,339 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:38:09,340 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:38:10,340 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_active_rt_load_r1

```
2026-09-04 12:55:28,336 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:55:29,337 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:55:30,338 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:55:31,338 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_active_rt_load_r2

```
2026-09-04 13:00:07,181 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 13:00:08,182 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 994.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 13:00:09,183 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 984.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 13:00:10,184 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_active_rt_load_r3

```
2026-09-04 13:01:42,002 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 13:01:43,002 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 13:01:44,004 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 979.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 13:01:45,004 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_rt_r1

```
2026-09-04 12:42:40,818 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:42:41,819 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:42:42,819 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:42:43,820 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_rt_r2

```
2026-09-04 12:44:22,601 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:44:23,602 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:44:24,603 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:44:25,603 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_rt_r3

```
2026-09-04 12:44:57,449 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:44:58,449 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:44:59,449 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:45:00,450 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_plain_r1

```
2026-09-04 12:32:25,216 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:32:26,217 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:32:27,217 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:32:28,218 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_plain_r2

```
2026-09-04 12:34:07,032 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:34:08,033 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:34:09,033 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:34:10,034 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_plain_r3

```
2026-09-04 12:34:41,935 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:34:42,936 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:34:43,937 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:34:44,938 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_rt_load_r1

```
2026-09-04 12:46:08,942 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 984.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:46:09,943 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 983.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:46:10,944 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 981.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:46:11,945 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 985.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_control_rt_load_r2

```
2026-09-04 12:50:46,761 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:50:47,762 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:50:48,762 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:50:49,763 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_rt_load_r3

```
2026-09-04 12:52:21,523 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 992.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:52:22,524 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 991.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:52:23,525 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:52:24,525 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_rt_r1

```
2026-09-04 12:39:15,804 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:39:16,805 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:39:17,806 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:39:18,806 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_control_rt_r2

```
2026-09-04 12:40:57,577 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:40:58,577 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:40:59,578 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:41:00,578 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_rt_r3

```
2026-09-04 12:41:32,437 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:41:33,437 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:41:34,437 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:41:35,438 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_plain_r1

```
2026-09-04 12:36:23,786 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:36:24,787 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:36:25,788 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:36:26,789 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_plain_r2

```
2026-09-04 12:36:58,757 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:36:59,758 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:37:00,759 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:37:01,760 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_plain_r3

```
2026-09-04 12:38:40,861 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:38:41,862 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:38:42,862 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:38:43,863 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_rt_load_r1

```
2026-09-04 12:57:00,383 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:57:01,383 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:57:02,384 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 992.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:57:03,385 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 989.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_rt_load_r2

```
2026-09-04 12:58:35,118 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:58:36,118 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:58:37,119 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:58:38,119 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_rt_load_r3

```
2026-09-04 13:03:14,037 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 13:03:15,038 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 13:03:16,039 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 13:03:17,040 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_rt_r1

```
2026-09-04 12:43:14,284 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
2026-09-04 12:43:15,284 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:43:16,284 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:43:17,285 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.03ms
```

### rust_active_rt_r2

```
2026-09-04 12:43:49,131 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:43:50,132 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:43:51,132 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:43:52,132 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_rt_r3

```
2026-09-04 12:45:30,937 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:45:31,938 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:45:32,939 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:45:33,940 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_plain_r1

```
2026-09-04 12:32:58,677 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:32:59,678 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:33:00,678 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:33:01,679 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_plain_r2

```
2026-09-04 12:33:33,548 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:33:34,549 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:33:35,549 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:33:36,550 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_plain_r3

```
2026-09-04 12:35:15,395 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:35:16,395 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:35:17,396 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:35:18,397 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_rt_load_r1

```
2026-09-04 12:47:40,970 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 946.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:47:41,970 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 974.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:47:42,970 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 970.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:47:43,972 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 978.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_control_rt_load_r2

```
2026-09-04 12:49:14,708 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 980.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:49:15,709 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 990.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:49:16,710 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:49:17,711 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_rt_load_r3

```
2026-09-04 12:53:53,547 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:53:54,547 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:53:55,548 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:53:56,549 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_rt_r1

```
2026-09-04 12:39:49,272 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:39:50,272 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:39:51,272 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:39:52,273 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_rt_r2

```
2026-09-04 12:40:24,106 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:40:25,107 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:40:26,108 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:40:27,109 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_rt_r3

```
2026-09-04 12:42:05,926 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:42:06,927 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:42:07,928 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:42:08,929 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

## OS baseline (cyclictest)

```
cyclictest not installed; skipping the OS baseline
```

