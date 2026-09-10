# Jitter / latency benchmark: franka-rs vs C++ libfranka

Both clients drive the **same** `franka-sim` container over the FCI, run the **same** motion
(`generate_joint_velocity_motion`, `time_max = 1 s`, `omega_max = 1 rad/s`) for the same
duration, and record the same statistics with the same code shape.

Contents:

| path | what |
|---|---|
| `cpp/bench_joint_velocity.cpp` | the C++ joint-velocity client, linked against a prebuilt libfranka |
| `cpp/bench_model_control.cpp` | the C++ model-in-the-loop torque client |
| `cpp/robot_home.cpp` | return-to-ready move, libfranka's own `MotionGenerator`, as its own step |
| `cpp/cpu_hog.c` | busy-loop load generator (this box has no `stress-ng`) |
| `cpp/CMakeLists.txt` | builds all three of the above |
| `rust/` | the Rust clients — a standalone crate depending on `franka-rs` by path |
| `run.sh` | runs the whole matrix and writes `results/<timestamp>/` |
| `summarize.py` | turns one results directory into `summary.md` + `summary.json` |
| `results/<timestamp>/` | raw per-run JSON, per-run simulator log slices, summary |
| `rust/src/bin/robot_probe.rs` | read-only precondition probe (mode + errors as JSON), `--recover`, `--home` |
| `model-micro/` | the offline `franka::Model` microbenchmark (FR3 / native backend) — no robot, no simulator |
| `so-micro/` | the offline `libfcimodels` microbenchmark (FER / FCI v5 `.so` backend) — no robot, no simulator |
| `fer-capture/` | tcpdump capture + offline pcap analysis of the FER state-arrival timing (which side stalls) — see its own [`README.md`](fer-capture/README.md) |

The write-up of these numbers is summarised in the [book's benchmarks
page](../docs/book/src/reference/benchmarks.md); the full record is kept privately.

## What is measured

Per control cycle, inside the loop, with no allocation and no printing (the sample array is
sized and zeroed before the motion starts):

* `CLOCK_MONOTONIC` at callback entry (`control` variant) or at `readOnce()` return
  (`active` variant),
* `CLOCK_MONOTONIC` after `writeOnce()` returned (`active` variant only),
* `RobotState::time` (ms),
* `control_command_success_rate`.

After the run:

* **interval** — time between consecutive callback entries / `readOnce()` returns:
  p50 / p99 / p99.9 / max / min / mean, in µs. This is the loop jitter as the *client* sees
  it; the nominal value is 1000 µs.
* **latency** (`active` only) — `readOnce()` return → `writeOnce()` return, i.e. the client's
  recv→send path. Not measurable in the `control` variant, where libfranka owns the send.
* **lost states** — cycles where `RobotState::time` advanced by more than 1 ms, the number of
  1 ms states thereby skipped, and the longest consecutive run of such cycles.
* **`control_command_success_rate`** min / avg / max / final, skipping cycle 0 (nothing has
  been acknowledged yet at that point, so it always reads 0).
* **CPU** — `getrusage(RUSAGE_SELF)` user + system time over the measured loop only, as a
  percentage of wall time, plus page faults and context switches.
* **scheduling** — the policy/priority the process was launched with *and* the one it is
  running under when the loop ends (they differ; see the note below).

### Variants

* `--variant control` — `Robot::control(motion_generator_callback, kJointImpedance,
  limit_rate = true, kDefaultCutoffFrequency)` /
  `Robot::control_joint_velocities(cb, ControllerMode::JointImpedance, true,
  DEFAULT_CUTOFF_FREQUENCY)`. The library owns the loop.
* `--variant active` — `startJointVelocityControl` / `start_joint_velocity_control`, then
  `readOnce()` → profile → `writeOnce()`. Neither library rate-limits or low-pass-filters in
  this path (checked in both sources), so `limit_rate` is reported as `null` for this variant
  in the JSON.
* `--variant model` — a different program (`cpp/bench_model_control.cpp` /
  `rust/src/bin/bench_model_control.rs`): torque control through
  `Robot::control(std::function<Torques(...)>)` / `Robot::control_torques`, with the **robot
  model evaluated inside every 1 kHz cycle**. See "The `model` variant" below.

Both joint-velocity programs home the arm with the shared `MotionGenerator` (C++ links
`libexamples_common.a`; Rust `#[path]`-includes `crates/franka-rs/examples/common/mod.rs`)
and call `setCollisionBehavior` with the example's thresholds **before** the measured loop
starts. Homing is not measured.

### The `model` variant

Both programs run the *same* operational-space impedance controller with inertia shaping.
Per cycle:

1. `M = mass(state)`, `c = coriolis(state)`, `g = gravity(state)`,
   `J = zeroJacobian(EndEffector, state)`, `T = pose(EndEffector, state)`;
2. `Λ = (J M⁻¹ Jᵀ + 1e-6·I₆)⁻¹` with `M⁻¹` from a Cholesky factorisation,
   `e = [p_d − p ; orientation error from the quaternion between T and the initial pose]`,
   `ẋ = J q̇`, `F = Λ (K_p e − K_d ẋ)`, `J̄ = M⁻¹ Jᵀ Λ`,
   `τ = Jᵀ F + (I₇ − Jᵀ J̄ᵀ)(−K_n q̇) + c`,
   then `limitRate(kMaxTorqueRate, τ, state.tau_J_d)`.

with `K_p = diag(200 ×3, 20 ×3)`, `K_d = 2·sqrt(K_p)`, `K_n = 0.5`, and `p_d` the initial
end-effector position plus a 0.05 m, 0.5 Hz sinusoid along z. Eigen fixed-size types on the
C++ side (as in libfranka's `cartesian_impedance_control` example), `nalgebra`
`SMatrix`/`Cholesky` on the Rust side; neither controller allocates.

Both are launched with `limit_rate = false` and `cutoff_frequency = kMaxCutoffFrequency`, so
the client-side `limitRate` above is the only limiter and no low-pass filter runs inside
either library. Both raise the collision thresholds to the torque example's 100.0 before the
loop (the joint-velocity example's tighter values trip on an impedance controller).

`gravity` is evaluated but not part of `τ` — the robot compensates gravity itself. It is in
the loop because the point of the variant is to price the five model calls a model-based
controller makes; its first element is accumulated into a checksum so neither compiler can
elide it. (Against `franka-sim` that checksum is 0: the simulator publishes
`O_ddP_O = {0,0,0}`, and `Model::gravity(state)` uses `O_ddP_O` as the gravity vector. The
call still runs; only its result is zero, identically in both languages.)

Extra JSON fields for this variant:

* `model_us` — time spent in the five `franka::Model` calls, per cycle;
* `compute_us` — time spent in the whole controller, model calls included;
* `controller.tau_max_abs`, `controller.ee_deviation_max_m` — the sanity check that both
  clients really ran the same controller (they should agree closely);
* `latency_us` is `null` and `limit_rate` is `false`.

The `model` variant runs the `plain` and `rt` conditions only: it measures compute cost, not
scheduling contention, so the CPU-hog cell is skipped.

### Conditions

| id | how the client is launched |
|---|---|
| `plain` | no wrapper |
| `rt` | `chrt -f 80 <client> --mlock` |
| `rt_load` | `chrt -f 80 <client> --mlock` with `cpu_hog $(nproc)` running |

`--mlock` calls `mlockall(MCL_CURRENT | MCL_FUTURE)` and reports `RLIMIT_MEMLOCK` and any
failure in the JSON instead of aborting.

`rt_load` keeps the `rt` configuration and only adds the load, so the `rt` → `rt_load`
comparison isolates the CPU contention. The simulator container always runs with
`--cpu-shares 4096`.

### Fairness protocol

A fixed "C++ first, then Rust" order would confound client identity with anything that drifts
over a 30-minute run — CPU thermal/turbo state, page-cache warmth, whatever the simulator
accumulates. So:

* a **cell** is one `(condition, variant, rep)`; both clients run back to back inside it;
* the order **alternates per repetition** — odd reps run C++ first, even reps Rust first — and
  every run records `order_in_cell` (1 or 2) and `cell_first_client` in its JSON, so the
  effect can be checked after the fact;
* the simulator container is **restarted at the start of every cell** and removed at the end,
  so neither client ever inherits a server the other has just driven for 30 s.

With the default 3 repetitions each client is therefore first in 2 cells and second in 1 for
every `(condition, variant)`, and both see a same-age server in every cell.

### Equal build settings

The C++ program is compiled `-O3` (`bench/cpp/CMakeLists.txt`); the Rust program's release
profile pins `opt-level = 3` explicitly (`bench/rust/Cargo.toml`). `debug = true` there adds
symbols only and does not change codegen.

### Identical loss accounting

Both programs derive lost states from `RobotState::time` the same way: a step of more than
1 ms counts `dt - 1` lost states, a **backwards** step counts as no loss and is reported
separately as `lost.backwards_time_steps` (none have been observed). This is saturating in
both languages — the C++ side no longer relies on unsigned wraparound.

### Two things to know about scheduling on this box

1. `RealtimeConfig::Enforce` needs `/sys/kernel/realtime`, which this machine does not have
   (it is `PREEMPT_DYNAMIC`, not `PREEMPT_RT`). Both clients therefore use
   `RealtimeConfig::kIgnore` / `RealtimeConfig::Ignore`, and realtime priority is applied
   externally with `chrt -f 80`.
2. **Even so, neither client actually runs at priority 80.** Both libfranka's `Robot::Impl`
   constructor and `franka-rs`'s `RobotImpl::new` call
   `setCurrentThreadToHighestSchedulerPriority()` /
   `set_current_thread_to_highest_scheduler_priority()` *unconditionally* — under
   `kIgnore` they merely swallow the failure. On this box `ulimit -r` is 99, so the call
   succeeds and both clients end up at `SCHED_FIFO` **99** regardless of how they were
   launched. That is why the JSON records `sched_at_start` (what `chrt` set) as well as
   `sched` (what the loop actually ran under), and why the `plain` column is not a
   SCHED_OTHER baseline. Discussed further in the full write-up, kept privately.

## Building

### C++ clients

Needs a prebuilt libfranka (shared library **and** `libexamples_common.a` from its
`examples/` target) plus Eigen3 headers. Defaults point at libfranka **0.20.4** under
`$HOME/libfranka`:

```sh
cmake -S bench/cpp -B bench/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build bench/cpp/build -j
```

This builds `bench_joint_velocity`, `bench_model_control`, `robot_home` and `cpu_hog`.

Point it elsewhere with
`-DLIBFRANKA_SOURCE_DIR=/path/to/libfranka -DLIBFRANKA_BUILD_DIR=/path/to/libfranka/build`.
The executable gets an RPATH to the build directory, so no `LD_LIBRARY_PATH` is needed.

### Rust clients

`bench/rust` is a **standalone** crate — its `Cargo.toml` carries an empty `[workspace]`
table so the repository's root workspace and its `Cargo.lock` stay untouched. It depends on
`franka-rs` by path, and builds three binaries: `bench_joint_velocity`,
`bench_model_control` and `robot_probe`.

```sh
cargo build --release --manifest-path bench/rust/Cargo.toml   # all three binaries
cargo fmt --manifest-path bench/rust/Cargo.toml --check
cargo clippy --manifest-path bench/rust/Cargo.toml --release --all-targets -- -D warnings
```

## Running

One simulator per machine. `run.sh` handles this itself: it re-executes under
`flock ${SIM_LOCK}` (default `.sim.lock` at the repository root), refuses to start if a `franka-sim`
container is already running or ports 1337/1338 are in use, starts
`franka-sim:dev` as `franka-sim-bench` with `--cpu-shares 4096`, waits for
`franka-sim-check --timeout 60` after each start, and removes **only** the container it
started, at the end of every cell and on any exit path.

```sh
bench/run.sh                                  # 30 s per run, 3 repetitions: the full matrix
bench/run.sh --duration 30 --reps 3           # the same, explicitly
bench/run.sh --variants model                 # only the model-in-the-loop variant
bench/run.sh --variants control,active,model  # everything
bench/run.sh --quick                          # 5 s, 1 repetition — a smoke test
bench/run.sh --host 127.0.0.1 --duration 10 --reps 1
```

`--variants` selects which of `control`, `active` and `model` to run; the default is
`control,active`.

Environment overrides: `SIM_LOCK`, `SIM_IMAGE`, `SIM_NAME`, `SIM_CPU_SHARES`.

The default matrix is 2 clients × 2 variants × 3 conditions × 3 repetitions = 36 runs of 30 s
plus homing, plus 18 container restarts (one per cell): roughly 35 minutes. `--variants model`
adds 2 clients × 2 conditions × 3 repetitions = 12 runs and 6 restarts, roughly 12 minutes.

Each cell writes:

* `<client>_<variant>_<condition>_r<rep>.json` — the statistics,
* `<client>_<variant>_<condition>_r<rep>.stdout` — the client's own output,
* `<client>_<variant>_<condition>_r<rep>.simlog` — the slice of `docker logs
  franka-sim-bench` produced during that run, the server-side cross-check (the simulator
  prints `State Update Stats - Freq: …Hz` once a second and warns about lost cycles).

and the directory also gets `meta.json` (machine description), `cyclictest.txt`,
`sim-full.log`, `summary.json` and `summary.md`.

Regenerate a summary without re-running:

```sh
python3 bench/summarize.py bench/results/<timestamp> > bench/results/<timestamp>/summary.md
```

## Running a single cell by hand

```sh
flock <repo>/.sim.lock bash -c '
  docker run -d --network host --cpu-shares 4096 --name franka-sim-bench franka-sim:dev
  docker exec franka-sim-bench franka-sim-check --timeout 60
  chrt -f 80 bench/cpp/build/bench_joint_velocity 127.0.0.1 \
      --variant active --duration 30 --mlock --condition rt --out /tmp/cpp.json
  chrt -f 80 bench/rust/target/release/bench_joint_velocity 127.0.0.1 \
      --variant active --duration 30 --mlock --condition rt --out /tmp/rust.json
  docker rm -f franka-sim-bench
'
```

## The offline model microbenchmark

`bench/model-micro/` prices the five `franka::Model` calls on their own, with no robot and no
simulator, so the model cost can be separated from the control loop it sits in.

* `cpp/` draws 10 000 random `(q, dq)` pairs inside the FR3 joint limits with a fixed seed,
  writes them to `states.json`, evaluates the five calls on each of them behind a
  `franka::Model` wrapped around libfranka's Pinocchio `franka::RobotModel` (the same
  construction `tools/model-reference` uses), times each call individually, and dumps every
  output to `reference.bin` as little-endian `f64`.
* `rust/` **replays exactly those inputs** through `franka::Model::from_urdf`, times the same
  five calls the same way, and cross-checks every output against `reference.bin`, reporting
  the maximum absolute difference per quantity.

Both use the same nearest-rank percentile code as the loop benchmarks. A 1 000-sample warm-up
runs first in both, on identical inputs, and is excluded from the statistics.

One asymmetry is measured rather than hidden: `franka::Model::coriolis(const RobotState&)`
forwards to the *deprecated* five-argument `RobotModel::coriolis`, which builds the full
Coriolis matrix (`pinocchio::computeCoriolisMatrix`) and multiplies by `dq`, whereas the
non-deprecated six-argument overload evaluates `rnea(q, dq, 0) − generalizedGravity(q)` —
which is what franka-rs's native backend does for *both* entry points. The C++ side times both
and reports them as `coriolis` and `coriolis_rnea`.

```sh
cmake -S bench/model-micro/cpp -B bench/model-micro/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build bench/model-micro/cpp/build -j
cargo build --release --manifest-path bench/model-micro/rust/Cargo.toml

bench/model-micro/run.sh                      # 10 000 states, writes results/<timestamp>/
bench/model-micro/run.sh --count 50000 --warmup 5000
```

The C++ program needs the same Pinocchio/Eigen include paths as `tools/model-reference`
(`-DPINOCCHIO_PREFIX=...`, `-DEIGEN_INCLUDE_DIR=...`); the defaults match this box.

## The offline model-library microbenchmark

`bench/so-micro/` is the FCI v5 counterpart: it prices the same five calls on the FER's
*compiled* model, the `libfcimodels_x64.so` the robot serves over `LoadModelLibrary`. Both
programs `dlopen` the **same** captured library — `$FRANKA_FER_MODEL_SO`, the fixture
`crates/franka-rs/tests/fer_model_conformance.rs` uses — and evaluate the same `q`, `dq`,
`F_T_EE` and load parameters, taken from robot L's read-only probe of 2026-09-05.

* `rust/` goes through `franka::Model` and `SoModelBackend`.
* `cpp/` does **not** link libfranka: 0.9.2's `franka::Model` can only be constructed from a
  `franka::Network`, i.e. by downloading the library from a robot, which is exactly what an
  offline benchmark cannot do. `main.cpp` instead reproduces `LibraryLoader`, `ModelLibrary`
  and the `franka::Model` member bodies verbatim from the 0.9.2 sources — `std::function`
  members, uninitialised `std::array` outputs and all — so the only external dependency is
  the `Poco::SharedLibrary` that `LibraryLoader` is built on.

Each program reports the five calls three ways: **steady state** (back to back, warm core —
this is the Rust-versus-C++ comparison), **duty cycled** (a ~1 ms `nanosleep` before each
cycle, as a 1 kHz control loop waits on the state datagram), and duty cycled **after
increasing amounts of untimed filler work**. The last one exists because the middle one is
not a property of the model code: on a laptop-class part the core idles during that
millisecond and needs tens of microseconds of work to climb back to its top frequency, so the
identical sequence reads 21 us cold and 4 us warm. That is the trap the FER campaign fell
into; see the "Model wrapper follow-up" section of the full write-up, kept privately.

```sh
cmake -S bench/so-micro/cpp -B bench/so-micro/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build bench/so-micro/cpp/build -j
cargo build --release --manifest-path bench/so-micro/rust/Cargo.toml

bench/so-micro/run.sh                          # 100 000 iterations, writes results/<timestamp>/
bench/so-micro/run.sh --count 20000 --so /path/to/libfcimodels_x64.so
```

## Running against a real arm

> **This drives a physical robot.** Everything below commands torques to a real FR3. Do not
> run it without the robot's owner saying so, with somebody at the user stop.

`bench/run.sh --hardware` is a different harness from the simulator matrix above:

* **no docker at all** — no container is started, stopped, inspected or logged, and the port
  and container preconditions are skipped;
* **the `rt` condition and the `model` variant only** — no `plain` cell, no CPU-hog cell;
* **guards on both clients** — `--hardware` is passed to `bench_model_control`, which then
  checks, once per cycle and outside its timed region, that the commanded torque stays within
  **20 Nm** and that the end effector stays within **0.10 m** of where the loop started. On a
  violation it replaces the command with a rate-limited step towards zero torque and returns
  it with the motion-finished flag set, so the library ends the motion the normal way — the
  loop is never killed. The reason, the cycle and the offending values land in the JSON under
  `guard`;
* **a precondition check before every run** — `robot_probe` reads the state once
  (read-only) and the run only proceeds from `RobotMode::Idle` with no error flags. If the
  mode is `Reflex`, `automatic_error_recovery()` is run **once**, the event is counted, and
  the stage is abandoned after the second one. Any other non-idle mode abandons the stage
  immediately;
* **a return-to-ready move before every run** — `robot_home` at `--home-speed` (default 0.2),
  as its own step so it can also be run by hand;
* **a pause after every run** — `--pause` seconds, default 20, so the joints cool;
* **results in `results/<timestamp>-hw/`**, with `stage.json` (stage, status, reflex-event
  count) alongside the per-run JSON, the probe JSON and the homing JSON. `summarize.py`
  detects them and writes an `FR3 hardware` section: one row per run (never aggregated over
  repetitions — every run against a real arm is its own event), plus the guard and
  control-exception tables.

A control loop that ends with a `ControlException` is **not** a lost run: the exception text
goes into the JSON as `control_exception`, the samples collected so far are still summarised,
and the client exits **3** instead of 1 so the harness can tell it apart from a failure that
produced no JSON at all.

### The two stages

```sh
# Stage B — one 30 s C++ run
bench/run.sh --hardware --host <robot-ip> --stage b --duration 30 --pause 20

# Stage C — 3 interleaved repetitions of both clients
#   (rep 1 C++ first, rep 2 Rust first, rep 3 C++ first)
bench/run.sh --hardware --host <robot-ip> --stage c --reps 3 --duration 30 --pause 20
```

Both take the simulator lock. That is deliberate: it is what stops a 12-core simulator matrix
from competing for this box's CPU while a real arm is under torque control.

### The individual steps, by hand

```sh
# read-only: exits 0 if Idle with no errors, 4 if not, 1 if unreachable
bench/rust/target/release/robot_probe <robot-ip>

# recover once from a Reflex, then re-probe
bench/rust/target/release/robot_probe <robot-ip> --recover

# return to the ready pose (libfranka's MotionGenerator; the Rust port is `robot_probe --home`)
bench/cpp/build/robot_home <robot-ip> --speed 0.2

# one guarded run, by hand
chrt -f 80 bench/cpp/build/bench_model_control <robot-ip> --variant model --duration 30 \
    --mlock --hardware --condition rt --rep 1 --order 1 --cell-first cpp --out /tmp/cpp-hw.json
```

`--provenance NAME` labels a run in the summary table; use it (or add `"provenance"` to an
existing JSON) to mark a measurement somebody else produced, so it is not read as a harness
run.

### Testing the guards without a robot

The guard limits are overridable **only** so the guard path itself can be exercised against
the simulator, where the controller never comes near 20 Nm or 0.10 m. The hardware defaults
are the compiled-in constants; do not pass these flags on hardware.

```sh
# trips the EE-deviation guard after ~140 cycles, then finishes the motion cleanly
bench/cpp/build/bench_model_control 127.0.0.1 --variant model --duration 20 \
    --hardware --guard-ee 0.005 --out /tmp/guard.json
# trips the torque guard after ~10 cycles
bench/rust/target/release/bench_model_control 127.0.0.1 --variant model --duration 20 \
    --hardware --guard-tau 0.5 --out /tmp/guard.json
```

`--hardware --host 127.0.0.1` also runs the whole stage machinery against the simulator, which
is how it was tested here.

## OS baseline

`run.sh` runs `cyclictest -q -m -p 80 -i 1000 -h 400 -D 60` under `chrt -f 80` if
`cyclictest` is on `PATH`. It is **not installed** on this box and the benchmark does not
install system packages, so `cyclictest.txt` records that instead. `stress-ng` is likewise
absent, hence `cpu_hog.c`.

## Caveats

* The reference C++ library here is **0.20.4**, while the Rust port targets 0.21.2. The
  joint-velocity control path and `ActiveControl` are unchanged between the two, but this is
  not a same-version comparison.
* This is a **simulator**, not an FR3, and the sim's server runs on the same box over
  loopback. The numbers describe the client-side loop and the local network stack, not a
  real FCI link.
* The box is not `PREEMPT_RT`; see the scheduling note above.
