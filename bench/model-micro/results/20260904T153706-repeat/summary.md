# `franka::Model` microbenchmark (10000 random states, 1000 warm-up, seed 20260904)

Per-call wall time in microseconds, `CLOCK_MONOTONIC` around each individual call.

| call | C++ p50 | C++ p99 | C++ max | Rust p50 | Rust p99 | Rust max | p50 ratio |
|---|---|---|---|---|---|---|---|
| `mass` | 0.66 | 2.56 | 32.27 | 0.69 | 1.03 | 14.76 | 1.0x |
| `coriolis` | 1.95 | 4.79 | 35.72 | 1.14 | 1.65 | 6.51 | 1.7x |
| `gravity` | 0.58 | 1.86 | 4.24 | 0.58 | 0.85 | 9.16 | 1.0x |
| `zero_jacobian` | 7.07 | 13.39 | 57.44 | 0.33 | 0.46 | 2.50 | 21.7x |
| `pose` | 6.48 | 12.53 | 46.27 | 0.27 | 0.37 | 2.81 | 24.1x |
| **all five** | **16.75** | 31.03 | 111.97 | **2.99** | 4.55 | 17.18 | **5.6x** |
| `coriolis` (C++ non-deprecated RNEA overload) | 1.56 | 3.66 | 35.80 | - | - | - | - |

As a fraction of the 1 ms control cycle: C++ 1.7 %, Rust 0.3 % at p50.

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
