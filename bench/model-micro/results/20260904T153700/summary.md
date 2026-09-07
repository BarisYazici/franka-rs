# `franka::Model` microbenchmark (10000 random states, 1000 warm-up, seed 20260904)

Per-call wall time in microseconds, `CLOCK_MONOTONIC` around each individual call.

| call | C++ p50 | C++ p99 | C++ max | Rust p50 | Rust p99 | Rust max | p50 ratio |
|---|---|---|---|---|---|---|---|
| `mass` | 0.67 | 1.04 | 5.49 | 0.68 | 0.80 | 9.79 | 1.0x |
| `coriolis` | 1.93 | 2.64 | 16.26 | 1.13 | 1.32 | 43.33 | 1.7x |
| `gravity` | 0.57 | 0.78 | 3.77 | 0.57 | 0.65 | 40.39 | 1.0x |
| `zero_jacobian` | 6.97 | 9.43 | 37.20 | 0.32 | 0.38 | 8.97 | 21.6x |
| `pose` | 6.34 | 8.71 | 33.38 | 0.27 | 0.30 | 2.10 | 23.7x |
| **all five** | **16.48** | 22.13 | 78.44 | **2.97** | 3.86 | 45.61 | **5.5x** |
| `coriolis` (C++ non-deprecated RNEA overload) | 1.51 | 2.09 | 8.93 | - | - | - | - |

As a fraction of the 1 ms control cycle: C++ 1.6 %, Rust 0.3 % at p50.

## Backend agreement

Maximum absolute difference between the two backends over the identical inputs.

| quantity | max abs diff | worst sample |
|---|---|---|
| `mass` | 3.553e-15 | 285 |
| `coriolis` | 4.319e-14 | 4799 |
| `coriolis_rnea` | 4.974e-14 | 6186 |
| `gravity` | 4.263e-14 | 2582 |
| `zero_jacobian` | 9.992e-16 | 3398 |
| `pose` | 1.055e-15 | 761 |

Worst disagreement anywhere: **4.974e-14**.
