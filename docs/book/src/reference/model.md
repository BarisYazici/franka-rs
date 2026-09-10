# Model parameters and conformance

[Use the model](../howto/model.md) covers the API. This page is what is behind it: the
native backend, where the parameters of each robot come from, how closely the results agree
with libfranka's, the caveats of the opt-in download path, and what the evaluation costs.

## The native backend

The default backend (`crates/franka-rs/src/model/native_backend.rs`) is a serial-chain
implementation of forward kinematics, geometric Jacobians, CRBA (mass matrix) and RNEA
(Coriolis, gravity), written against the conventions Pinocchio uses, because libfranka's
`franka::RobotModel` is built on Pinocchio: poses are 4x4 column-major, Jacobians 6x7
column-major with the linear rows first (Pinocchio's `Motion` layout), the body Jacobian is
Pinocchio's `LOCAL` frame and the zero Jacobian its `LOCAL_WORLD_ALIGNED`. It allocates
nothing per call and needs no C++ library.

Against the conformance fixture generated from libfranka 0.20.4 itself
(`tests/data/model_reference_fr3.json`, produced by `tools/model-reference`, a C++ program
linked against libfranka that drives the real `franka::Model`: 128 joint configurations × 3
load configurations, all ten frames):

| quantity | tolerance | max abs error |
|---|---|---|
| `pose` | 1e-9 | 7.216e-16 |
| `body_jacobian` | 1e-9 | 9.992e-16 |
| `zero_jacobian` | 1e-9 | 7.216e-16 |
| `mass` | 1e-6 | 3.109e-15 |
| `coriolis` | 1e-6 | 2.864e-14 |
| `gravity` | 1e-6 | 2.132e-14 |
| `gravity`, `g = {0.1, -0.2, -9.7}` | 1e-6 | 2.842e-14 |

That is double-precision round-off: the two implementations differ only in summation order.
`cargo test -p franka-rs --test model_conformance` runs it offline; the fixture carries the
SHA-256 of the URDF it was generated from, so `tests/data/fr3.urdf` and the fixture cannot
drift apart unnoticed. The same agreement holds on 10 000 random FR3 states in the
benchmark harness (largest difference anywhere 4.97e-14; see [Benchmarks](./benchmarks.md)).

## Where the parameters come from

### FR3 (FCI v10): the robot's own URDF

FCI v10 has a `GetRobotModel` command that serves the arm's URDF over the TCP channel.
`Robot::load_model()` fetches it and hands it to the native backend; `Robot::robot_model()`
returns the URDF text itself. Nothing is downloaded that is not data, and nothing is
`dlopen`ed.

### FER (FCI v5): shipped identified parameters

An FER has no `GetRobotModel`. libfranka 0.9.2 instead issues `LoadModelLibrary`, which
streams Franka's closed-source `libfcimodels.so` (329 592 bytes on system 4.2.1) down the
command channel, writes it to a temporary file and `dlopen`s it.

The crate does not need that. It ships the FER's model as *parameters*,
`crates/franka-rs/tests/data/fer.urdf`, exposed as `franka::model::FER_URDF`, and evaluates
them with the same native backend the FR3 uses. On an FER, `Robot::load_model()` is exactly
`Model::native_fer()`: it cannot fail, needs no network, no `model-library` feature and no
x86-64 Linux host.

The kinematics were never in doubt: `franka_description`'s joint origins and axes reproduce
the shared object's poses and both Jacobians to 4e-16. The dynamics differ, and by a lot:
feeding the *FR3's* URDF to the backend puts gravity 1.6 Nm out, the mass matrix 0.16 kg m²
out and Coriolis 0.8 Nm out (2.7 %, 3.9 %, 3.8 %). So the seven links' inertial parameters
were identified from the shared object itself by `tools/fer-model-fit`: rigid-body dynamics
is linear in the ten inertial parameters per link, so with the kinematics fixed the problem
is 70 unknowns, solved by truncated SVD against the library's answers at 208 joint
configurations. The regressor has rank 45, the classical number of base parameters, and the
worst residual is 3.6e-14.

Agreement against a real FER's `libfcimodels_x64.so`, over 208 joint configurations × 4
load configurations, all ten frames:

| quantity | payload | max abs difference | relative |
|---|---|---|---|
| pose, all frames | any | 4.441e-16 | 3.5e-16 |
| body Jacobian | any | 8.882e-16 | 8.9e-16 |
| zero Jacobian | any | 4.441e-16 | 4.4e-16 |
| gravity | any | 4.974e-14 | 8.4e-16 |
| mass matrix | none | 3.553e-15 | 1.3e-15 |
| Coriolis | none | 3.553e-14 | 2.4e-15 |
| mass matrix | 1.23 kg | 2.980e-3 | 7.5e-4 |
| Coriolis | 1.23 kg | 5.008e-2 | 2.4e-3 |

### The payload caveat

The last two rows are the whole residual, and they are the shared object's doing, not the
fit's: its `M_NE` is not affine in `m_load`. For a rigid payload `M(2m) − M(0)` must be
exactly `2 (M(m) − M(0))`; the library misses that by 7e-4, behaving as though the payload
were up to 18 g lighter than it was told (0.7 % of a 1 kg payload, saturating). No rigid-body
model can follow it there, libfranka's own included if it were reimplemented. `g_NE` *is*
affine, and the payload's rotational inertia `I_load` enters exactly as a rigid body's does,
so gravity, the quantity a gravity-compensation loop needs, agrees to 5e-14 whatever the
payload. The mass matrix and Coriolis agree to 4e-14 with no payload and carry that
1e-3-scale offset with one.

One more property of the identified set: some moments of inertia come out negative. That is a
property of the robot's own model, not of the fit: alternating projections between the affine
set of exact fits and the cone of realisable rigid bodies converge with the smallest
pseudo-inertia eigenvalue still at −1.5e-2, so the shared object's parameters are not those
of any set of real bodies. Use the file to reproduce libfranka's numbers, not to seed a
physics simulation.

### The tests

- `tests/model_conformance.rs`: the FR3 table above, against the libfranka 0.20.4 fixture.
- `tests/fer_native_conformance.rs`: the FER table above, against a committed fixture
  (`tests/data/model_reference_fer.json`, 40 joint configurations × 4 load configurations,
  dumped from the shared object), so it runs in CI with no `.so` present. With
  `$FRANKA_FER_MODEL_SO` set, two further tests drive the library live: one checks the
  fixture is still what the library answers, the other pins the non-affine `M_NE`.
- `tests/fer_model_conformance.rs`: drives the shared object itself through
  `Model::from_model_library_path` / `from_model_library_bytes` at eight fixed
  configurations, checks the invariants `franka::Model` guarantees, and *reports* the
  native backend's differences against it. Needs the `model-library` feature and the `.so`;
  skips loudly without it.

The kinematics have also been checked against the robot rather than its library: converting
the 1 kHz logs of the 2026-09-08 hardware runs with `franka-rerun csv --robot fer`, the
native model's end effector, from the logged `q` and the tool offset identified from the
first row, matched the measured `O_T_EE` to under 0.01 mm on every row.

## `load_model_from_robot`: the opt-in download path

The v5 download path is still there and still supported:

```rust,no_run
# extern crate franka;
# use franka::Robot;
# fn f(robot: &Robot) -> franka::FrankaResult<()> {
let model = robot.load_model_from_robot()?;   // LoadModelLibrary + dlopen, as libfranka does
# let _ = model; Ok(()) }
```

Use it when you want the robot's own binary in the loop, for a conformance check, say. On an
FR3 it is `load_model()` exactly. On an FER its consequences are those of a closed-source
blob:

- **x86-64 Linux in practice.** The request encodes the host architecture, but the FER
  control unit only ships `libfcimodels_x64.so`, so anywhere else the robot answers `kError`
  or `dlopen` refuses the object. `load_model()` has no such limitation.
- **It goes to a temporary file.** `dlopen` needs a path, so the bytes are written to a
  private `0600` temp file removed when the `Model` is dropped. The directory must be
  executable (a `noexec` `/tmp` breaks it). Removal covers a normal drop and an unwind but
  not `SIGINT`/`SIGTERM`/`abort`, so a loop stopped with Ctrl-C leaves one ~330 KB file in
  `$TMPDIR`.
- **`dlopen` executes code the robot chose**, in your process, over an unauthenticated
  plaintext TCP socket, with no validation. This is what libfranka 0.9.2 does, and it stays
  a *safe* `fn` here for the same reason: the FCI peer is already fully trusted, because it
  is the thing that commands the arm. Give the FCI its own isolated link, as Franka's setup
  guide requires.
- **Opt out entirely** by building without the default `model-library` feature:
  `load_model_from_robot()` then returns a `FrankaError::Model`, `libloading` is not linked
  at all, and `load_model()` is unaffected. The offline entry points
  `Model::from_model_library_bytes` and `Model::from_model_library_path`, which take the
  bytes from the caller, are `unsafe fn` and carry the corresponding `# Safety` contract.
- **The library needs libm in the loading process.** `libfcimodels_x64.so` imports `sin`,
  `cos` and `sincos` and carries no `DT_NEEDED` entries of its own, so it resolves them from
  the global scope of whichever process `dlopen`s it. C++ clients get libm transitively
  through libstdc++; a Rust binary only links it if something in the binary uses it. A lean
  binary fails with `undefined symbol: sincos` at the first call; name the dependency
  explicitly if you rely on this path.

## Cost

Evaluating the five model calls a model-based controller makes costs about 3 µs offline
(2.97 µs p50 on 10 000 random FR3 states) and 11–15 µs inside a 1 kHz loop on a laptop-class
CPU, about 1.5 % of the cycle. The in-loop figure is higher than the offline one in every
implementation measured, because a duty-cycled loop starts each cycle on a core that has just
idled and the measurement prices the CPU's post-idle frequency ramp, not the arithmetic. The
per-call numbers, the same comparison for libfranka, and that frequency-ramp caveat are on
[Benchmarks](./benchmarks.md).
