# franka-rs for Python

`import franka`: drive a Franka FR3 (FCI v10) or Franka Emika Robot / Panda (FCI v5) from
a program that runs at its own rate -- a policy at 5 to 30 Hz, a script, a keyboard. The
1 kHz control loop is [`franka-rs`](https://github.com/BarisYazici/franka-rs)'s target
control on a Rust thread; **the GIL is never on the 1 kHz path**. `move_to` / `move_by`
are one write into that loop's seqlock slot, `follow` hands a chunk of targets to a Rust
timer thread, and `state()` is a copy out of a mutex.

```python
import franka

robot = franka.Robot("192.168.0.1")   # FRANKA_REALTIME=ignore for franka-sim

with robot.cartesian_targets(max_velocity=0.3, max_acceleration=0.5, max_jerk=20.0) as arm:
    while not done:
        obs = arm.state().flat()      # 1-D float64 in the order of franka.FLAT_LAYOUT
        action = policy(obs)
        arm.move_by(action[:3])       # metres, relative to the current target
        # or: arm.move_to([x, y, z]); arm.follow(chunk, dt=0.02)  # chunk: (N, 3) absolute
```

`robot.joint_targets(fraction=0.2)` is the same over the seven joints. `__exit__` stops the
loop; an exception inside the block still stops the motion and is re-raised. Errors are
`franka.FrankaError`; a motion the robot aborted is a `franka.ControlException` with the
reflex `reason`. `robot.gripper()` drives the Franka Hand, releasing the GIL while it waits.

A Cartesian target of 7 elements (`move_to`, the rows of `follow`) sets the orientation
too, as a unit quaternion `x, y, z, w` after the position; a `move_by` of 6 elements
appends a rotation vector (axis times angle, rad) composed onto the target orientation in
the base frame. `arm.target()` is that 7-vector, `arm.target_pose()` the 4x4 matrix.
`examples/policy_loop.py` is a jittery 6-10 Hz loop with a circle, a yaw and tilt sweep
and a `follow` chunk, as run on a Panda; `examples/rotate.py` is the rotation alone, and
`examples/quickstart.ipynb` a notebook that connects, moves, and replays the motion inline
with Franka's meshes. `robot.model()` is the kinematics and dynamics over numpy
(`pose`, `body_jacobian`, `zero_jacobian`, `mass`, `coriolis`, `gravity`, `link_poses`,
`hand_pose`).

## Install

```sh
pip install franka-rs
```

The wheel is named `franka-rs`, the package `franka`. To build from the source tree into a
virtualenv instead (prefix with `env -u CONDA_PREFIX` under an active conda env):

```sh
pip install maturin
maturin develop --release -m crates/franka-py/Cargo.toml
```

The crate is `publish = false` on crates.io; its tests are

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev pytest crates/franka-py/tests
```

against the simulator (see the book's
[Testing with franka-sim](https://barisyazici.github.io/franka-rs/howto/simulator-tests.html)).
