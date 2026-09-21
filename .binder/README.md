# Binder simulation lab: maintainer notes

[Launch this branch](https://mybinder.org/v2/gh/BarisYazici/franka-rs/codex%2Fbinder-simulation-lab?urlpath=lab/tree/crates/franka-py/examples/cartesian_sim_lab.ipynb)
opens `crates/franka-py/examples/cartesian_sim_lab.ipynb`. The public image build and
Jupyter launch have been exercised. Before promoting a release, test its exact revision
and point stable documentation links at that tested release or commit.

## Build and runtime

Binder uses Python 3.12, `apt.txt` and pinned `requirements.txt`, then runs `postBuild`
once while building the image for a repository revision. Subsequent sessions reuse that
image when cached. `postBuild` installs Rust 1.98.0 if needed, builds the native Python
bindings from **this exact checkout** with maturin and `--locked`, and installs the wheel.
There is no published `franka-rs==0.4.0` wheel available for this setup. Compilation uses
one Cargo build job to limit peak memory.

The notebook environment uses NumPy 2 for Rerun. `postBuild` creates `.binder/sim-venv`
with `franka-sim==1.1.7`, NumPy 1.26.4 and MuJoCo 3.2.7 to keep the simulator's dependency
requirements separate. The client and simulator communicate through the FCI protocol;
they do not share a Python interpreter. No Docker daemon is needed in a user session.

`fetch_model.py` fetches a sparse checkout of MuJoCo Menagerie's `franka_fr3_v2` model at
commit `feadf76d42f8a2162426f7d226a3b539556b3bf5` into `.binder/assets`, retaining its
license. Assets are fetched during image build, not during notebook experiments.
`smoke.py` then starts the installed simulator and checks that native target control
remains running and receives advancing state after a small motion command.

`start` sets realtime-ignore and single-thread numerical-library defaults, then forwards
Binder's launch command. The notebook's `sim_runtime.py` starts a fresh headless simulator
on demand at `127.0.0.1:1437`. Its context manager waits for readiness, prevents concurrent
experiments and stops its child on exit; a parent-pipe watcher also stops it if the kernel
dies. `FRANKA_SIM_PYTHON` can override the simulator interpreter for local development.

## Reproduce locally

On Linux with Python 3.12 (including `venv`), Git and the packages in `apt.txt` installed,
run from the repository root. The build downloads dependencies, a Rust toolchain if
missing, and the pinned model; it needs internet access.

```sh
python3.12 -m venv .binder/notebook-venv
source .binder/notebook-venv/bin/activate
python -m pip install -r .binder/requirements.txt
bash .binder/postBuild
bash .binder/start jupyter lab crates/franka-py/examples/cartesian_sim_lab.ipynb
```

This reproduces the build steps, not Binder's full container or resource limits. For
release CI, build the exact revision with repo2docker, run the smoke test and notebook
under bounded CPU/memory, and verify inline rendering in a browser. Also check interruption,
kernel restart and simulator cleanup. Finally test the public launch link: a successful
local build alone does not establish Binder compatibility. Public sessions are temporary
and shared-cloud timing is not a realtime qualification.

## Notebook checks

```sh
python .binder/verify_notebook.py
python .binder/verify_notebook.py quickstart.ipynb
```

Both notebooks own simulator startup and cleanup; users do not need to launch a server
separately. The guided lab begins with a stationary preview and single 2 cm move.
Its default playback embeds recorded joint states in a self-contained SVG viewer, so
browsers without WebGL2 can still see the robot. Rerun remains optional in the quickstart.
Notebook execution alone does not validate browser playback: check the visible robot,
Play/Pause, time scrubbing and view rotation in the actual hosted notebook as well.
