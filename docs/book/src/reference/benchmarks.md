# Benchmarks

Measured against **libfranka 0.20.4** and libfranka `main`, with the harness in `bench/`:
loop jitter against `franka-sim` over loopback, the model inside the 1 kHz loop, and the
model calls offline.

Every measurement ran on one machine: a laptop-class x86-64 CPU with 12 logical cores, Linux
6.8, **not `PREEMPT_RT`**. That matters for how to read the tail: every multi-millisecond
`max` in these tables is a scheduling event on a desktop kernel, and these numbers are a
same-machine A/B, **not an FCI qualification of either client**.

The fairness protocol: a *cell* is one (condition, variant, repetition), both clients run
back to back inside it, the order alternates per repetition, and the simulator container is
restarted per cell. The **paired within-cell difference** is the primary evidence everywhere
below; pooled tables are context.

## Simulator: loop jitter

Joint-velocity motion, 30 s per run, 3 repetitions, `chrt -f 80` + `mlockall`.

| variant | client | interval p50 | p99 | lost cycles | success avg | CPU % |
|---|---|---|---|---|---|---|
| `control` (callback) | C++ | 1000 µs | 1071 µs | 0.3 | 1.000 | 2.7 |
| `control` (callback) | **Rust** | 1000 µs | **1050 µs** | 0.3 | 1.000 | **2.4** |
| `active` (`readOnce`/`writeOnce`) | C++ | 1000 µs | **1046 µs** | 0.0 | 1.000 | 3.0 |
| `active` (`readOnce`/`writeOnce`) | **Rust** | 1000 µs | 1054 µs | 0.0 | 1.000 | **2.4** |

Paired over all 18 cells:

| statistic | C++ minus Rust | verdict |
|---|---|---|
| CPU | mean **+0.59 pp**, C++ higher in **18/18** cells | real |
| interval p99 | mean −15 µs, median **−3 µs**, C++ higher in 8/18 | noise |
| `active` recv→send latency p50 / p99 | 17 / 34 µs vs **11 / 23 µs** (9 cells) | real |

The CPU gap holds equally whether C++ ran first (+0.57) or Rust ran first (+0.63), so it is
not an ordering artefact. **Loop timing is indistinguishable**: median cycle time is exactly
1000 µs everywhere and the p99 difference changes sign between variants. Under a 12-core
CPU hog, p99 rises to 1.3–1.4 ms (`control`) or ~1.1 ms (`active`) for both clients, p50
stays at 1000 µs, and neither client lost a cycle.

## The model in the loop

The same operational-space impedance controller in both languages, with all five
`franka::Model` calls evaluated inside every 1 kHz cycle. Simulator, `chrt -f 80` +
`mlockall`, 3 repetitions of 30 s.

| build | model p50 | p99 | compute p50 | interval p50 | p99 | lost | CPU % |
|---|---|---|---|---|---|---|---|
| libfranka 0.20.4 | 30.8 µs | 67.0 | 33.3 µs | 1000 µs | 1084 | 0 | 5.7 |
| libfranka `main` (0.21.3) | 31.26 | 64.44 | 33.96 | 1000 | 1049 | 0 | 5.59 |
| libfranka `main` **+ Data-reuse patch** | **11.07** | 27.15 | 13.88 | 1000 | 1043 | 0 | 3.49 |
| **franka-rs** | **10.8** | 32.9 | 12.5 | 1000 | 1047 | 0 | 2.9 |

Both clients produce the same motion (peak |τ| 1.925–1.940 Nm, peak end-effector excursion
0.04785–0.04789 m), and neither lost a cycle in any of the 12 runs.

The five calls measured offline, on 10 000 identical random FR3 states:

| call | libfranka 0.20.4 | libfranka `main` | + patch commit 1 | + commit 2 | **franka-rs** |
|---|---|---|---|---|---|
| `mass` | 0.67 | 0.63 | 0.62 | 0.61 | 0.68 |
| `coriolis` (state overload) | 1.93 | 1.96 | 1.91 | 1.86 | **1.13** |
| `gravity` | 0.57 | 0.60 | 0.57 | 0.58 | 0.57 |
| `zeroJacobian` | 6.97 | 7.17 | 0.81 | 0.50 | **0.32** |
| `pose` | 6.34 | 6.55 | 0.36 | 0.37 | **0.27** |
| **all five** | **16.48** | **16.93** | **4.28** | **3.92** | **2.97** |

p50 microseconds. The 21–24× gap on `pose` and `zeroJacobian` was **not** a C++/Rust
difference: libfranka's Pinocchio wrapper constructed a whole fresh `pinocchio::Data` on
every kinematics call — 143 heap allocations. A two-commit patch (see `patches/`)
takes that to **zero allocations per call** and closes essentially the whole gap; the
remaining ratio is about **1.3× offline and 1.0× in the loop**, and it is not all in the
same direction (C++ ahead on `mass`, franka-rs on `coriolis` and the Jacobians). Every
output is byte-identical before and after the patch, and libfranka's own 805/805 tests pass
on both.

**Numerical agreement** over the same 10 000 inputs: the largest absolute difference
anywhere between the two backends is **4.97e-14** (`zeroJacobian` 9.99e-16, `pose`
1.06e-15, `mass` 3.55e-15). That is double-precision round-off.

### Reading in-loop model timings

An in-loop `model_us` figure prices the CPU's post-idle frequency ramp as much as the
arithmetic: a duty-cycled loop starts each cycle on a core that has just idled. The five FER
model-library calls in C++ (`bench/so-micro`), timed after increasing amounts of untimed
filler work, swing from 21.37 µs to 3.98 µs on identical inputs. So a client whose receive
path does less work can read slower in the loop; compare the arithmetic with the offline
table.

## Reproducing

```sh
cmake -S bench/cpp -B bench/cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build bench/cpp/build -j
cargo build --release --manifest-path bench/rust/Cargo.toml

bench/run.sh --duration 30 --reps 3      # takes the simulator lock, one container per cell
```

`bench/README.md` documents the harness, the `--hardware` mode and its guards, and
`bench/fer-capture/README.md` the wire-capture tooling. Raw per-run JSON of the simulator
runs lives under `bench/results/`.
