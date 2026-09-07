# franka-rs vs C++ libfranka — jitter benchmark (20260904T135840)

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
| git_rev | fe8ff8bd49774cbf929d60cbf622a86161caa94f |
| duration_s | 30 |
| reps | 2 |
| cyclictest | None |
| stress_ng | None |

## Variant 1 — `control` (libfranka `Robot::control` / `Robot::control_joint_velocities`, `limit_rate = true`)

`interval` is the time between consecutive control-callback entries (the 1 kHz cycle as the client sees it).

| condition | client | cycles | interval p50 (µs) | p99 | p99.9 | max | lost cycles | lost states | max consec | success min | success avg | CPU % | server lost-cycle lines |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| plain (SCHED_OTHER) | cpp | 29966 | 1000 | 1107 | 3177 | 65902 | 1.5 | 34.5 | 1 | 0.330 | 0.999 | 2.9 | 1 |
| plain (SCHED_OTHER) | rust | 29995 | 1000 | 1101 | 1244 | 8182 | 1.5 | 6.0 | 1 | 0.930 | 1.000 | 2.3 | 0 |
| chrt -f 80 + mlockall | cpp | 30000 | 1000 | 1114 | 1388 | 4113 | 1.0 | 1.0 | 1 | 0.970 | 1.000 | 2.8 | 0 |
| chrt -f 80 + mlockall | rust | 29999 | 1000 | 1098 | 1316 | 4981 | 1.0 | 2.0 | 1 | 0.970 | 1.000 | 2.4 | 0 |
| chrt -f 80 + mlockall + CPU hog | cpp | 30001 | 1000 | 1281 | 4090 | 6096 | 0.0 | 0.0 | 0 | 0.990 | 1.000 | 2.8 | 0 |
| chrt -f 80 + mlockall + CPU hog | rust | 30001 | 1000 | 1209 | 4029 | 9464 | 0.0 | 0.0 | 0 | 0.990 | 1.000 | 2.0 | 0 |

## Variant 2 — `active` (`startJointVelocityControl` / `start_joint_velocity_control`, `readOnce` → `writeOnce`)

`interval` is the time between consecutive `readOnce()` returns; `latency` is `readOnce()` return → `writeOnce()` return, i.e. the client's recv→send path.

| condition | client | cycles | interval p50 (µs) | p99 | p99.9 | max | latency p50 (µs) | p99 | p99.9 | max | lost cycles | max consec | success avg | CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| plain (SCHED_OTHER) | cpp | 30000 | 1000 | 1086 | 1186 | 4510 | 16 | 37 | 56 | 286 | 0.5 | 1 | 1.000 | 2.8 |
| plain (SCHED_OTHER) | rust | 30000 | 1000 | 1101 | 1273 | 4861 | 10 | 29 | 47 | 874 | 1.0 | 1 | 1.000 | 2.1 |
| chrt -f 80 + mlockall | cpp | 29994 | 1000 | 1089 | 1212 | 7802 | 17 | 39 | 67 | 362 | 2.0 | 1 | 1.000 | 3.0 |
| chrt -f 80 + mlockall | rust | 30001 | 1000 | 1073 | 1189 | 4775 | 12 | 30 | 46 | 513 | 0.0 | 0 | 1.000 | 2.6 |
| chrt -f 80 + mlockall + CPU hog | cpp | 30001 | 1000 | 1565 | 4121 | 30105 | 17 | 29 | 96 | 1253 | 0.0 | 0 | 1.000 | 2.9 |
| chrt -f 80 + mlockall + CPU hog | rust | 30001 | 1000 | 1883 | 4346 | 26013 | 12 | 19 | 64 | 611 | 0.0 | 0 | 1.000 | 2.2 |

## Per-run detail

| run | cycles | wall s | sched at exec | sched in loop | mlockall | interval max (µs) | max at cycle | lost | sr avg |
|---|---|---|---|---|---|---|---|---|---|
| cpp_active_plain_r1 | 30000 | 30.03 | OTHER/0 | FIFO/99 | - | 4510 | 6558 | 1 | 1.000 |
| cpp_active_plain_r2 | 30001 | 30.02 | OTHER/0 | FIFO/99 | - | 1473 | 19376 | 0 | 1.000 |
| cpp_active_rt_load_r1 | 30001 | 30.72 | FIFO/80 | FIFO/99 | ok | 4934 | 13660 | 0 | 1.000 |
| cpp_active_rt_load_r2 | 30001 | 30.83 | FIFO/80 | FIFO/99 | ok | 30105 | 16318 | 0 | 1.000 |
| cpp_active_rt_r1 | 29993 | 30.06 | FIFO/80 | FIFO/99 | ok | 6836 | 9688 | 3 | 1.000 |
| cpp_active_rt_r2 | 29996 | 30.03 | FIFO/80 | FIFO/99 | ok | 7802 | 16714 | 1 | 1.000 |
| cpp_control_plain_r1 | 29937 | 30.39 | OTHER/0 | FIFO/99 | - | 65902 | 9318 | 1 | 0.998 |
| cpp_control_plain_r2 | 29996 | 30.06 | OTHER/0 | FIFO/99 | - | 5371 | 13539 | 2 | 1.000 |
| cpp_control_rt_load_r1 | 30001 | 30.75 | FIFO/80 | FIFO/99 | ok | 6091 | 13154 | 0 | 1.000 |
| cpp_control_rt_load_r2 | 30001 | 30.52 | FIFO/80 | FIFO/99 | ok | 6096 | 15398 | 0 | 1.000 |
| cpp_control_rt_r1 | 30000 | 30.05 | FIFO/80 | FIFO/99 | ok | 3894 | 2352 | 1 | 1.000 |
| cpp_control_rt_r2 | 30000 | 30.11 | FIFO/80 | FIFO/99 | ok | 4113 | 22369 | 1 | 1.000 |
| rust_active_plain_r1 | 30000 | 30.05 | OTHER/0 | FIFO/99 | - | 3963 | 17461 | 1 | 1.000 |
| rust_active_plain_r2 | 29999 | 30.06 | OTHER/0 | FIFO/99 | - | 4861 | 4712 | 1 | 1.000 |
| rust_active_rt_load_r1 | 30001 | 30.77 | FIFO/80 | FIFO/99 | ok | 26013 | 1524 | 0 | 1.000 |
| rust_active_rt_load_r2 | 30001 | 30.93 | FIFO/80 | FIFO/99 | ok | 5300 | 15245 | 0 | 1.000 |
| rust_active_rt_r1 | 30001 | 30.02 | FIFO/80 | FIFO/99 | ok | 2861 | 14383 | 0 | 1.000 |
| rust_active_rt_r2 | 30001 | 30.05 | FIFO/80 | FIFO/99 | ok | 4775 | 27036 | 0 | 1.000 |
| rust_control_plain_r1 | 30001 | 30.04 | OTHER/0 | FIFO/99 | - | 4484 | 65 | 0 | 1.000 |
| rust_control_plain_r2 | 29989 | 30.06 | OTHER/0 | FIFO/99 | - | 8182 | 26064 | 3 | 0.999 |
| rust_control_rt_load_r1 | 30001 | 30.34 | FIFO/80 | FIFO/99 | ok | 4855 | 3739 | 0 | 1.000 |
| rust_control_rt_load_r2 | 30001 | 30.42 | FIFO/80 | FIFO/99 | ok | 9464 | 12956 | 0 | 1.000 |
| rust_control_rt_r1 | 29999 | 30.07 | FIFO/80 | FIFO/99 | ok | 4611 | 17300 | 1 | 1.000 |
| rust_control_rt_r2 | 29999 | 30.06 | FIFO/80 | FIFO/99 | ok | 4981 | 6559 | 1 | 1.000 |

## Server-side cross-check (`docker logs`)

### cpp_active_plain_r1

```
2026-09-04 12:01:23,683 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:01:24,684 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:01:25,685 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:01:26,686 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_plain_r2

```
2026-09-04 12:01:56,887 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:01:57,887 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:01:58,887 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:01:59,888 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_active_rt_load_r1

```
2026-09-04 12:14:17,301 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
2026-09-04 12:14:18,302 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 966.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
2026-09-04 12:14:19,303 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:14:20,304 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
```

### cpp_active_rt_load_r2

```
2026-09-04 12:15:51,288 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 973.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:15:52,288 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 979.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:15:53,289 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 965.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:15:54,290 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 978.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_active_rt_r1

```
2026-09-04 12:05:49,899 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:05:50,899 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:05:51,900 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:05:52,901 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_active_rt_r2

```
2026-09-04 12:06:23,168 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:06:24,168 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:06:25,169 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:06:26,170 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### cpp_control_plain_r1

```
2026-09-04 11:59:09,973 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 990.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 11:59:10,974 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 11:59:11,975 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 11:59:12,976 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 11:58:52,803 WARNING franka_sim.franka_sim_server: 20 consecutive lost command cycles: no longer extrapolating the motion generator, holding the last reference (not enforced)
```

### cpp_control_plain_r2

```
2026-09-04 11:59:43,846 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 11:59:44,846 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 11:59:45,847 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 11:59:46,848 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_control_rt_load_r1

```
2026-09-04 12:08:05,028 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 986.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:08:06,029 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 975.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:08:07,030 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 966.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:08:08,031 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 907.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_control_rt_load_r2

```
2026-09-04 12:09:38,104 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 991.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:09:39,105 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 992.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:09:40,106 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 985.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:09:41,107 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 990.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_control_rt_r1

```
2026-09-04 12:03:36,596 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:03:37,596 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:03:38,597 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:03:39,597 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### cpp_control_rt_r2

```
2026-09-04 12:04:09,815 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:04:10,816 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:04:11,817 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 992.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:04:12,817 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_plain_r1

```
2026-09-04 12:02:30,120 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:02:31,120 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 994.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
2026-09-04 12:02:32,121 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 993.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:02:33,121 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_plain_r2

```
2026-09-04 12:03:03,302 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:03:04,303 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 992.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:03:05,303 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:03:06,304 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_active_rt_load_r1

```
2026-09-04 12:17:23,324 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 987.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:17:24,325 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 985.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:17:25,325 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 981.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:17:26,326 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 967.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_rt_load_r2

```
2026-09-04 12:18:57,363 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 963.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:18:58,363 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 960.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:18:59,364 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 961.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:19:00,366 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 962.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_rt_r1

```
2026-09-04 12:06:56,449 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:06:57,449 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:06:58,450 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 1000.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:06:59,450 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_active_rt_r2

```
2026-09-04 12:07:29,694 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:07:30,694 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:07:31,695 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:07:32,695 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_control_plain_r1

```
2026-09-04 12:00:17,168 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:00:18,168 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.8Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
2026-09-04 12:00:19,169 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:00:20,170 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_plain_r2

```
2026-09-04 12:00:50,421 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
2026-09-04 12:00:51,422 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:00:52,422 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:00:53,423 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
```

### rust_control_rt_load_r1

```
2026-09-04 12:11:11,136 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:11:12,137 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 995.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:11:13,138 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 989.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:11:14,139 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 991.2Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

### rust_control_rt_load_r2

```
2026-09-04 12:12:44,183 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 990.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
2026-09-04 12:12:45,183 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 989.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:12:46,185 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.1Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:12:47,186 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 980.4Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
```

### rust_control_rt_r1

```
2026-09-04 12:04:43,100 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.0Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:04:44,101 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 997.3Hz, Genesis Time: 0.00ms, Total Cycle: 0.06ms
2026-09-04 12:04:45,101 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 996.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:04:46,102 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.6Hz, Genesis Time: 0.00ms, Total Cycle: 0.04ms
```

### rust_control_rt_r2

```
2026-09-04 12:05:16,574 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.7Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:05:17,575 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:05:18,576 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 998.5Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
2026-09-04 12:05:19,576 INFO franka_sim.franka_sim_server: State Update Stats - Freq: 999.9Hz, Genesis Time: 0.00ms, Total Cycle: 0.05ms
```

## OS baseline (cyclictest)

```
cyclictest not installed; skipping the OS baseline
```

