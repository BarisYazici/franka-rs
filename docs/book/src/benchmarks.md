# Benchmarks and hardware validation

Two measurement campaigns, both reproduced from the harness in `bench/`:

- **FR3 benchmark, 2026-09-04** — `franka-sim` over loopback plus a real FR3 at
  `172.16.0.2`, against **libfranka 0.20.4**.
- **Franka Emika Robot (FER) hardware run, 2026-09-05** — two real FERs on system 4.2.1, against
  **libfranka 0.9.2**.

The full measurement records for both campaigns are kept privately and are not part of this
repository; this page is the public summary of what they found. The hardware runs that
followed — target control, rotation targets and the Python bindings on an FER, target
control on an FR3 — are listed under
[Validation in the README](https://github.com/BarisYazici/franka-rs#validation); they are
functional checks, not timing campaigns, and are not repeated here.

Both were run on the same box: a laptop-class x86-64 CPU with 12 logical cores, Linux 6.8, **not
`PREEMPT_RT`**. That matters for how to read the tail: every multi-millisecond `max` in
these tables is a scheduling event on a desktop kernel, and these numbers are a same-box
A/B, **not an FCI qualification of either client**.

Both campaigns use the same fairness protocol: a *cell* is one (condition, variant,
repetition), both clients run back to back inside it, the order alternates per repetition,
and the simulator container is restarted per cell. The **paired within-cell difference** is
the primary evidence everywhere below; pooled tables are context.

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

## FR3 hardware

A real FR3 at `172.16.0.2`, model variant, `chrt -f 80` + `mlockall`. Three interleaved
repetitions of both clients (stage c) plus one C++ shakedown run (stage b), a read-only
`Idle`-with-no-errors probe before every run, a return-to-ready move before and a 20 s cool
pause after, and torque/deviation guards checked outside the timed region.

**No guard tripped, no run ended in a `ControlException`, and 0 reflex events across all 8
runs.** The arm reported `Idle` with no error flags immediately before and after every one.

| statistic (stage c, paired, 3 reps) | C++ minus Rust |
|---|---|
| model p50 | **+10.67 µs** mean (+9.20 median), C++ higher in 3/3 |
| CPU | **+2.61 pp** mean (+2.01 median), C++ higher in 3/3 |
| interval p99 | 1079–1106 µs for every run, both clients — no difference |

Representative per-run figures: Rust model p50 12.1–13.8 µs at 2.7–3.6 % CPU against C++'s
21.3–27.6 µs at 4.7–7.4 %; interval p99 1079–1106 µs; success rate average 0.9996–1.0000.
The C++ side is unpatched libfranka 0.20.4 — the Data-reuse patch was never built for
hardware, so every C++ row still pays the allocation, and would be expected to close the
same way it did on the simulator.

## FER hardware

Two FERs — **L** on an onboard NIC, **R** on a USB
Ethernet adapter — on robot system 4.2.1, against libfranka 0.9.2. Thirteen 30 s runs of
1 kHz model-in-the-loop torque control, `chrt -f 80` + `mlockall`.

**No reflex, no guard trip, no `ControlException`, no
`communication_constraints_violation`, and `max_consecutive` lost cycles was 1 in every
run.** Both arms were `Idle` and error-free afterwards.

| arm | client | interval p50 | p99 | lost / 30 000 | success avg |
|---|---|---|---|---|---|
| L (onboard NIC) | **franka-rs** | 999.4 µs | **1171.7 µs** | **27.5** | 0.9862 |
| L (onboard NIC) | libfranka 0.9.2 | 999.5 µs | 1186.8 µs | 29.5 | 0.9872 |
| R (USB Ethernet) | **franka-rs** | 999.1 µs | 1181.1 µs | 91.7 | 0.9873 |
| R (USB Ethernet) | libfranka 0.9.2 | 999.1 µs | 1183.9 µs | 87.0 | 0.9882 |

Every gap there is smaller than the run-to-run spread of either client. **Verdict: for
running an FER, neither client is meaningfully better at the job.**

Model agreement on the robot, both clients evaluating the same `libfcimodels.so` at
bit-identical inputs: `gravity` 2.31e−14, `coriolis` 7.68e−22, `mass` 1.09e−14, `pose`
5.27e−15, `zeroJacobian` 5.27e−15 max absolute difference. Double round-off — the v5 model
path is numerically identical to libfranka 0.9.2 on real hardware.

### A measurement trap worth knowing about

The campaign's `model_us` **p50** column read 15.45 µs (Rust) against 5.68 µs (C++) on arm
L, and that is **not** a valid comparison. Offline, on the same shared object and the same
inputs, the two wrappers are within **1.2 %** of each other (4.011 vs 3.964 µs for all five
calls). What the in-loop measurement priced is the CPU's post-idle frequency ramp: the same
C++ code, timed after increasing amounts of untimed filler work, swings from 21.37 µs to
3.98 µs — a 5.4× spread on identical inputs. The tell is that `model_us` **min** is 4.0 µs
in all thirteen runs for both clients.

The client that does more work between the datagram arriving and the callback starting
reaches the model calls on a warmer core. franka-rs's receive path retires ~1200–1500
instructions where libfranka 0.9.2 retires ~3500–4000, copies 13.4 KB where libfranka
copies ~19 KB, and allocates nothing where libfranka does two `malloc`/`free` pairs. **The
crate's model region measured slower because the crate's receive path is leaner.** No crate
change was made; there is no defect. The deadline, lost-cycle, success-rate and guard
results are unaffected — those are not sub-10-microsecond measurements.

### Where the FER's lost cycles come from

Wire captures settle this, and the answer is not the client. From `tcpdump` on the link,
including one run stamped with the NIC's **hardware receive timestamps** (before the
kernel's interrupt path):

- **Nothing is lost on the network.** Every `message_id` the robot emitted appears in every
  capture — **0 missing ids in 141 000 states across six runs** — and IP reassembly, socket
  and NIC counters are all zero.
- **The gaps are already on the wire.** ~70 inter-arrival gaps ≥ 1.5 ms per 10 s run
  (about **7 per second**), max 4.77–6.91 ms, present in the PHY timestamps. Median
  inter-arrival is 999.3–999.4 µs, so this is a rare event, not a shifted distribution.
- **The robot's control loop keeps running.** Across each gap the millisecond counter
  advances by exactly one, and 63–87 % of gaps are followed by a burst draining the
  backlog: the control box's *transmit path* stalls for 2–7 ms a few times per second.
- **The host is excluded.** Two `SCHED_FIFO` stall probes on the NIC IRQ core and another
  core saw 1 late wakeup in 93 s (and 0 in 13 s during which the wire showed 97 gaps).
  Disabling the NIC's Energy-Efficient Ethernet changed nothing.
- **Both clients are excluded.** Both drain the socket keeping the newest `message_id`, so
  when two states arrive together the older is discarded *by design*. A "lost cycle" here
  is a discarded queued state, not packet loss — which is why `max_consecutive` is never
  above 1 and the robot's own packet-loss watchdog is never approached.

The remaining lever is the control box, not this repository.

Arm R loses ~3× more than L on **both** clients, which tracks its interface, not its
client: the USB Ethernet adapter has an RX ring of 100 (vs 256) and `rx-usecs` of **15000**
(vs 3 on the onboard NIC). 15 ms of interrupt coalescing is pathological for a 1 kHz loop.

The v5 state datagram is 2373 bytes, over the 1500-byte MTU, so every state arrives as
exactly 2.0000 IP fragments — 2000 packets/s instead of 1000 — with **zero** `ReasmFails`,
`ReasmTimeout`, `Udp.InErrors` or NIC drops. That doubles the packet rate the host must
service, which is the most plausible reason the FER loses cycles where the FR3 lost none,
but it is a load effect, not a reassembly failure.

### Two arms at once

Both FERs driven simultaneously, per arm, 10 000 cycles each:

| configuration | L lost | R lost | success avg |
|---|---|---|---|
| two independent processes (`taskset -c 3` / `-c 5`) | 39 | 29 | 0.99 / 0.99 |
| one process, one `Robot` per thread, 3 runs | 24–32 | 43–55 | 0.98–0.99 |

**No cross-interference.** Running both at once does not raise either arm's loss rate,
success rate or interval spread beyond what it shows alone, and both figures match the
single-arm campaign. The single-process example (`dual_communication_test`) gives each
thread its own `Robot`, its own TCP and UDP sockets, and a `Barrier` that releases both
timed loops together; from that point the two robots share no state at all, and if one
errors the other runs its cycles to completion.

### `ActiveControl` on the FER

First real-hardware exercise of the `read_once`/`write_once` API on an FER. Robot L, 10 s
per run:

| variant | cycles | lost | interval p50 | p99 | success avg | CPU % |
|---|---|---|---|---|---|---|
| `active` (`read_once`/`write_once`) | 9957 | 36 | 999.2 µs | 1236.6 | 0.981 | 2.55 |
| `control` (callback) | 9979 | 20 | 999.1 µs | 1271.6 | 0.985 | 2.65 |

No errors, no reflex, `Idle` after both. **Equivalent within noise** — the lost-cycle gap
is well inside the run-to-run spread seen throughout the campaign. libfranka 0.9.2 has no
`ActiveControl` at all, so on an FER this is a control style the C++ client cannot offer.

## Reproducing

```sh
cmake -S bench/cpp -B bench/cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build bench/cpp/build -j
cargo build --release --manifest-path bench/rust/Cargo.toml

bench/run.sh --duration 30 --reps 3      # takes the simulator lock, one container per cell
```

`bench/README.md` documents the harness, the `--hardware` mode and its guards, and
`bench/fer-capture/README.md` the wire-capture tooling. Raw per-run JSON lives under
`bench/results/`.
