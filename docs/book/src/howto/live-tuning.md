# Tune a running controller

Live tuning changes how an existing Cartesian control session tracks its targets, without
restarting the node. Use it to compare tracking, damping and motion limits. Ordinary motion
does not require the panel or any live parameter changes: start with the shipped configuration.

<figure class="flow-figure">
  <div class="flow-scroll" tabindex="0" role="region" aria-label="Live tuning diagram; scroll horizontally on small screens">
    <img src="../assets/live-tuning-flow.svg" alt="The motion client sends goals and owns the session. The tuning panel sends parameter updates and receives accepted settings and robot state. The node validates both inputs; franka-rs runs the local 1 kHz torque loop on the Pi or realtime PC." width="800" height="970">
  </div>
  <figcaption>Motion goals and tuning are separate inputs. Both reach the same local controller. <a href="../assets/live-tuning-flow.svg">Open full-size diagram</a>.</figcaption>
</figure>

The motion client still owns the arm and keeps the session alive. The panel changes controller
parameters; it does not acquire the arm, start motion or replace the motion client. Live tuning
currently requires a running **Cartesian impedance session**. An idle arm or a joint-target
session returns `not_ready` when asked to change parameters.

## Open the optional panel

Start the node and your Cartesian client using [Serve arms over Zenoh](./franka-node.md).
The panel is a source tool in `tools/tuning-panel`, separate from the Python client package.
From the repository root, in a Python virtual environment:

```sh
python -m pip install 'eclipse-zenoh>=1.10,<2'
python tools/tuning-panel/web.py \
  --connect tcp/127.0.0.1:7447 --no-multicast \
  --presets ./tuning-presets.json
```

This example runs the bridge on the same machine as a node listening on port 7447. Open
`http://127.0.0.1:8765/` and select the arm. For a Pi-hosted bridge, tunnel it from the laptop:

```sh
ssh -N -L 8765:127.0.0.1:8765 <pi-user>@<pi-address>
```

Then open the same local URL on the laptop. The bridge binds to loopback by default and has
no authentication; keep that default. Access to the node's Zenoh bus also permits parameter
changes: tuning requests identify their sender but do not require the motion lease holder.

To explore the interface without a robot, run these in two terminals instead:

```sh
python tools/tuning-panel/mock_owner.py --listen tcp/127.0.0.1:17447 --no-multicast
```

```sh
python tools/tuning-panel/web.py \
  --connect tcp/127.0.0.1:17447 --no-multicast \
  --presets ./tuning-presets.json
```

The mock's teleoperation controls demonstrate an additional parameter owner; they do not
mean that a teleoperation service is included in `franka-node`.

## What the controls change

There are nine named fields containing 25 scalar values. The panel gets ranges, units,
defaults and confirmation thresholds from the running node's schema, generated from the
same `LiveTuning::BOUNDS` table that checks updates. There is no separate panel limits table.

| Field | Meaning |
|---|---|
| `joint_stiffness[7]` | Joint springs in the impedance law. |
| `joint_damping[7]` | Joint damping; the node enforces a floor tied to each joint spring. |
| `cartesian_stiffness` | Scales the Cartesian stiffness preset and its associated damping. |
| `ik_damping` | Damping of the inverse-kinematics solve near poorly conditioned poses. |
| `ik_nullspace_gain` | Posture correction in the redundant degree of freedom; zero disables it. |
| `velocity_feedforward_gain` | Weight of the goal velocity in the damping term; zero disables feedforward. |
| `velocity_feedforward_cutoff` | Filter cutoff for that velocity; the schema marks the filter-off value. |
| `budget[3]` | Translation velocity, acceleration and jerk norm limits. |
| `rotation_budget[3]` | Rotation velocity, acceleration and jerk norm limits. |

Change a small group of related settings, apply, and compare the same motion before and
after. Read the accepted values: the node can clamp a request or raise damping when a spring
is increased. Invalid requests change nothing. Crossing a budget's confirmation threshold
requires an explicit confirmation; already being above it does not ask again.

Gains approach their targets with a 0.3 s time constant. Budget velocity and acceleration
limits rise immediately but ramp downward, to avoid abruptly truncating the generator's
velocity or acceleration. Budget jerk and the feedforward filter cutoff step immediately.
This handling removes the need to design those parameter transitions in the client; it does
not make every combination or every robot pose feasible.

The displayed `params` are **accepted targets**, not a measurement of the gains applied on
each realtime cycle. `slewing` estimates the fraction of the latest change **still to go**:
near 1 just after acceptance, approaching 0. It is calculated from elapsed time, not fed back
from the controller; overlapping changes and missed cycles limit its accuracy.

## Why feedforward can read zero

This TOML starts with feedforward disabled:

```toml
# Inside an [[arm]] table
velocity_feedforward = false
velocity_feedforward_gain = 1.0
```

The boolean takes precedence at session start, so the live weight is **0**, despite the
gain line. Setting `velocity_feedforward_gain` above zero live enables its contribution for
that session. To enable it in future sessions, set `velocity_feedforward = true` and the
desired gain in TOML, then restart the node to load the configuration.

## What is saved

Live changes last only for the current session. On session end, values revert to the loaded
TOML baseline, `origin` becomes `null`, and `dirty` becomes `false`. The next session starts
from that baseline. A node restart also restores the configuration and changes `boot_id`.

The version counts accepted updates during a node run: session end does **not** reset it;
node restart does. `dirty` compares accepted targets with the baseline, not the controller's
instantaneous gains. Panel presets are separate JSON files. They do not edit TOML, and there
is no `params/save` endpoint. Put a chosen setup into TOML explicitly after reviewing it.

## Read the diagnostics with their limits

Panel jitter, lag and tracking measurements are computed from published robot state, at the
configured `state_hz`; they are not a complete trace of the 1 kHz loop. The node does not yet
publish leash-alteration counts or cap-active fractions. Joint inertia hints are unavailable,
and the headroom display also needs a compatible teleoperation owner's limits. A missing
metric is not evidence that a limit was never reached.

Larger Cartesian budgets can still reach joint-side limits depending on pose, payload and
trajectory; there is no universal translation speed that guarantees success. Robot reflexes,
collision settings, session deviation guards and the client's own target pacing remain in
effect. Those guards are not loosened by tuning a budget.

For client implementations and precise message semantics, see the
[node parameters protocol](../reference/node-parameters.md).
