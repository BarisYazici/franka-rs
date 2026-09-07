# fer-model-fit

Identifies the Franka Emika Robot (FER)'s link inertial parameters from a real
FER's own `libfcimodels_x64.so`, and writes the two files the crate ships:

* `crates/franka-rs/tests/data/fer.urdf` — the parameter set behind
  `franka::model::FER_URDF` and `Model::native_fer()`;
* `crates/franka-rs/tests/data/model_reference_fer.json` — the committed
  oracle `crates/franka-rs/tests/fer_native_conformance.rs` runs against in
  CI, dumped from the shared object.

Neither file needs regenerating unless the shared object changes. This tool is
kept so the numbers in [`docs/book/src/fer.md`](../../docs/book/src/fer.md) and
[`docs/book/src/model.md`](../../docs/book/src/model.md) can be reproduced.

## Running it

The shared object is not committed and not published; capture one from an FER
(`Robot::load_model_from_robot()` downloads it) and point the tool at it.

```sh
export FRANKA_FER_MODEL_SO=/path/to/libfcimodels_x64.so
cargo run --release --manifest-path tools/fer-model-fit/Cargo.toml -- <command>
```

| command | what it does |
|---|---|
| `compare-fr3` | how far `tests/data/fr3.urdf` is from the FER's model — identical kinematics, dynamics out by a few per cent |
| `fit` | identifies the parameters and **writes `tests/data/fer.urdf`**, then reports the agreement it reaches |
| `verify` | reports the committed `fer.urdf` against the shared object |
| `load-model` | characterises how the library handles the payload, including the `M_NE` non-affinity in `m_load` that sets the conformance suite's loaded tolerances |
| `fixture` | **writes `tests/data/model_reference_fer.json`** |

Run `fit` before `fixture`: the fixture records the SHA-256 of the URDF it was
generated with, and `fixture_matches_the_committed_urdf` fails if they drift
apart.

## How the fit works

Rigid-body dynamics is linear in the ten inertial parameters of each link
(`m`, `m·c`, and the inertia about the joint frame), so with the kinematics
fixed — `franka_description`'s `fer_joint1..8`, which match the shared object
to 4e-16 — the mass matrix and the gravity torques are an affine function of 70
unknowns.

The regressor is built by evaluating **franka-rs's own backend** at each unit
parameter, one generated URDF per column; there is no second implementation of
CRBA or RNEA anywhere in this repository. It is solved with a truncated SVD
against the library's answers at 208 joint configurations (the full symmetric
mass matrix plus gravity along three axes), which gives the minimum-norm
correction to `franka_description`'s published inertials. Rank 45 of 70 — the
classical base-parameter count — and a worst residual of 3.6e-14.
