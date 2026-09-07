# model-reference

C++ reference dumper for the `franka::Model` conformance suite.

It builds libfranka's own Pinocchio-based `franka::RobotModel` from the FR3
URDF, wraps it in the public `franka::Model` facade (`franka::Model(
std::unique_ptr<RobotModelBase>)`, the constructor libfranka provides for its
own tests) and writes every quantity the API exposes for a deterministic sample
set to JSON with 17 significant digits.

`combineCenterOfMass` and `combineInertiaTensor` are ported verbatim from
libfranka's `src/load_calculations.cpp` into `main.cpp`, so `m_total`,
`F_x_Ctotal` and `I_total` are derived exactly the way
`Robot::Impl::convertRobotState` derives them from `m_ee` / `m_load`.

## What is dumped

* **Samples** — the zero pose, the libfranka "ready" pose
  `{0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4}`, six poses at random joint-limit
  corners, then uniformly random configurations inside the FR3 joint limits with
  random `dq` in +/-2 rad/s. The RNG is `std::mt19937_64` seeded with `20260904`;
  the seed, the URDF SHA-256 and the libfranka version are stamped into the
  JSON header.
* **Load configurations** — three of them: no load; the Franka Hand `F_T_EE`
  with `m_ee = 0.73` and a 0.5 kg payload; the same plus a stiffness frame
  offset of `{0, 0, 0.05}` rotated 30 degrees about z.
* **Per sample and load configuration** — `pose`, `body_jacobian` and
  `zero_jacobian` for all ten `franka::Frame` values, the `mass` matrix, the
  `coriolis` vector with `g = {0, 0, -9.81}`, and the `gravity` vector for both
  `g = {0, 0, -9.81}` and `g = {0.1, -0.2, -9.7}`.

`mass()` is deliberately called before `gravity()` for each load configuration:
`franka::RobotModel` caches the last-link inertia and its `gravity()` only
refreshes that cache when `m_total > 0`, so calling `mass()` first pins the
cache to the configuration being dumped.

## Build

Requires libfranka 0.20.4 built with Pinocchio, Eigen 3 and Pinocchio 4.1. The
default source/build paths are `$HOME/libfranka` and
`$HOME/libfranka/build-rt-ignore`; override them with `-D` if your libfranka
tree lives elsewhere (see the cache variables below).

```sh
cmake -S tools/model-reference -B tools/model-reference/build -DCMAKE_BUILD_TYPE=Release
cmake --build tools/model-reference/build -j4
```

Useful cache variables: `LIBFRANKA_DIR`, `LIBFRANKA_BUILD_DIR`,
`EIGEN_INCLUDE_DIR`, `PINOCCHIO_PREFIX`.

## Run

```sh
./tools/model-reference/build/model-reference \
    reference/libfranka/test/fr3.urdf \
    crates/franka-rs/tests/data/model_reference_fr3.json \
    120
```

The third argument is the number of random configurations (default 192). It is
120 for the committed fixture, which keeps the file at 5.7 MB, under the 6 MB
budget; 8 fixed poses are always dumped on top of it.

`crates/franka-rs/tests/data/fr3.urdf` is a byte-identical copy of the URDF the
fixture was generated from (SHA-256
`86cc04711b376a1a0760f5572cebd27aa92e70b357dbeb4709282650799295f9`), so the
Rust test suite does not depend on the git-ignored `reference/libfranka`
checkout. `model_conformance.rs` re-hashes it and fails if the two drift apart.
