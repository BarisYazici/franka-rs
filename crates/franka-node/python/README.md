# franka-node-client

Python client for [franka-node](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node),
the Zenoh node in front of `franka-rs`'s target control. Drive a node-served Franka arm from a
laptop, a workstation or a notebook, on Linux, macOS or Windows, without a byte offset, a
sequence number, a timestamp or a lease token in your own code.

```sh
python -m pip install franka-node-client     # the same version as the node
```

From a checkout of the node's revision: `python -m pip install ./crates/franka-node/python`.

```python
import franka_node

node = franka_node.Node("tcp/<node-host>:7447")         # or Node(session=my_zenoh_session)
with node.arm("fr3") as arm:                              # lease token + acquire, release on exit
    with arm.cartesian_targets() as t:                    # enable; stop on exit
        t.move_by([0.0, 0.0, 0.05])                       # +5 cm z, relative to the goal
        state = t.wait(timeout=5.0)                       # until the arm is there
        print(state.position, state.quaternion, state.refused)
    arm.gripper.move(0.04)
```

The verbs and conventions are those of the `franka` Python bindings: quaternions `(x, y, z, w)`,
rotation vectors in the base frame, `O_T_EE` as a `(4, 4)` matrix, robot mode names such as
`"move"` and `"user_stopped"`.

## API

- `Node(connect=None, *, session=None)`: `close()`, context manager, `client_id`,
  `status(name=None, timeout=2.0) -> dict`, `arm(name) -> Arm`. A borrowed `session` is used as
  it is configured and never closed.
- `Arm`: `state(timeout=1.0) -> ArmState`, `on_state(callback) -> Subscription`,
  `acquire(timeout=3.0)`, `release()`, context manager, `home(speed=0.2, *, episode=None,
  timeout=90.0)`, `stop()`, `recover()`, `gripper`,
  `cartesian_targets(*, episode=None, max_velocity=0.1, max_angular_velocity=0.3, lead=0.03,
  angular_lead=0.15, rate=50.0)` and `joint_targets(*, episode=None, max_velocity=0.5,
  lead=0.15, rate=50.0)`.
- `CartesianTargets` / `JointTargets`: `move_to`, `move_by`, `follow(chunk, dt)`, `target()`,
  `state()`, `wait(timeout=None, ...)`, `stop()`, context manager.
- `Gripper`: `move(width, *, wait=True, timeout=10.0)`, `grasp(width, force, *, wait=True,
  timeout=10.0)`, `homing(timeout=40.0)`, `stop()`, `state(timeout=1.0) -> GripperState`.
- `franka_node.rotated(quaternion, rotation_vector)`.
- Errors: `NodeError`, and under it `Refused` (the node's reason verbatim), `NodeTimeout`,
  `SessionEnded` and `ProtocolError`.

What the client does for you (the lease, the keepalive, pacing under the node's `max_step` and
`max_lead`, the anchor flag, `wait`, `SessionEnded`) is described on the book page
[Serve arms over Zenoh](https://barisyazici.github.io/franka-rs/howto/franka-node.html).

## Tuning panel

The client installs `franka-tuning-panel`, a browser page for the live parameters of a running
Cartesian impedance session: `franka-tuning-panel --connect tcp/<node-host>:7447`. It binds to
loopback; see [Tune a running controller](https://barisyazici.github.io/franka-rs/howto/live-tuning.html)
for the tunnel, presets and limits.

The package version is the `franka-rs` workspace version. Apache-2.0.
