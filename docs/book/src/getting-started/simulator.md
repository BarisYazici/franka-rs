# Without a robot: franka-sim

At the end of this page every example, the Python notebook and your own program run against
a simulated FR3 on your machine.

Prerequisites: Docker.

```sh
docker run --rm --network host ghcr.io/barisyazici/franka-sim:latest
```

[franka-sim](https://github.com/BarisYazici/libfranka-sim) is a MuJoCo-based simulator that
speaks the FCI wire protocol, so a client connects to it as it would to an arm. The
container serves the robot on `127.0.0.1:1337` and a Franka Hand on port 1338. It does not
run under `PREEMPT_RT`, so connect with `RealtimeConfig::Ignore`; for the examples that is

```sh
FRANKA_REALTIME=ignore cargo run --release --example echo_robot_state -- 127.0.0.1
```

and any other example the same way ([Run the examples](../howto/examples.md)). The
[notebook](./python.md) defaults to `127.0.0.1` and `FRANKA_REALTIME=ignore`, so it runs
against the container as it is.

## Server flags

Arguments after the image name go to the server:

| flag | effect |
|---|---|
| `--enforce-motion-limits` | Runs the robot's limit checks on every command, including (since franka-sim 1.1.5) the joint-side continuity check on Cartesian poses, scaled by `--joint-discontinuity-scale` (1.0 is the robot's own limit). Without it the server accepts commands a robot would refuse. |
| `--gripper-object-width <m>` | Places a graspable object of that width between the fingers. |
| `--enforce-comm-constraints` | Enables the communication-constraint checks. |
| `--no-gripper` | No server on port 1338. |

## One slot per host

The FCI ports are fixed by the protocol and one client holds them at a time, so run one
container at a time and check before starting one:

```sh
ss -tlnp | grep -E '133[78]'   # nothing may be listening
docker ps                      # no franka-sim container may be running
```

`--rm` removes the container when you stop it with Ctrl-C.

## What it is not

- Not a realtime system: timing measured against it is a same-machine comparison, not an FCI
  qualification ([Benchmarks and hardware validation](../reference/benchmarks.md)).
- Not the robot: it has documented divergences from real hardware, each pinned by a test that
  names the gap in its failure message ([Simulator gaps](../reference/simulator-gaps.md)).
- Only the FR3 image is published. The FER / FCI v5 image (`franka-sim:panda-v5`) is a local
  build whose recipe is kept outside this repository.

Running the crate's test suites against it, including the harness that starts the container
for you, is described in [Test against the simulator](../howto/simulator-tests.md).
