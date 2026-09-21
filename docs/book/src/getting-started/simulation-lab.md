# Try target control online

Experiment with Cartesian targets on a simulated FR3 in a Jupyter notebook, with no local
installation. Edit Python code, change velocity, acceleration and jerk limits, then watch
the arm and compare its measured motion.

**[Launch the notebook on Binder](https://mybinder.org/v2/gh/BarisYazici/franka-rs/codex%2Fbinder-simulation-lab?urlpath=lab/tree/crates/franka-py/examples/cartesian_sim_lab.ipynb)**

This preview launches the `codex/binder-simulation-lab` branch. Cloud launch validation is
still pending. You can also
[read the notebook source](https://github.com/BarisYazici/franka-rs/blob/codex/binder-simulation-lab/crates/franka-py/examples/cartesian_sim_lab.ipynb).

## Run an experiment

1. Open the launch link and wait for Jupyter. Binder builds the environment for the
   repository revision and caches it; the first launch can take longer.
2. Run the setup cells in order. They start your session's simulator and connect the
   Python client to it.
3. Run the default motion, then change one limit at a time and run it again. Compare the
   arm's movement and plots before changing another setting.
4. Edit the target-control cells themselves to try a different target or motion. The
   notebook explains the settings alongside the runnable code.

The control calls are the same ones used in [From Python](./python.md):

```python
import time

with robot.cartesian_targets(
    max_velocity=0.3,       # m/s
    max_acceleration=0.5,   # m/s²
    max_jerk=20.0,          # m/s³
) as arm:
    arm.move_by([0.04, 0.0, 0.0])
    time.sleep(3.0)         # allow the controller to move before leaving the block
```

`move_by` updates the target and returns immediately; leaving the `with` block stops
control. Velocity limits constrain how fast the generated target moves, acceleration
limits constrain changes in velocity, and jerk limits constrain changes in acceleration.
A short move may never reach its maximum velocity. The measured arm can lag the generated
target, so a higher limit does not guarantee proportionally faster motion.

Python sends targets to the native Rust control pipeline. A native `franka-sim` process
runs in the same session and exchanges robot commands and state with that controller.
The notebook visualizes that simulation's state. See
[Command from a low-rate program](../howto/target-control.md) for the full API and backends.

## Keep your changes

Binder sessions are temporary. Download your edited notebook and any results you want to
keep before closing the session. Opening the launch link again creates a fresh environment;
it does not restore earlier edits.

The public service has limited shared CPU and memory, and may stop idle sessions. Treat
these experiments as comparisons of simulated behavior: cloud timing is not a realtime
qualification, and simulator motion does not establish physical FR3 performance. See
[Binder's usage guidelines](https://mybinder.readthedocs.io/en/latest/about/user-guidelines.html)
and the [simulator gaps](../reference/simulator-gaps.md).

For longer experiments or consistent access, use the
[local simulator](./simulator.md) and run the notebook locally.
