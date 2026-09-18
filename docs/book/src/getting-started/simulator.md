# Without a robot: franka-sim

Run a simulated FR3 locally, connect a Rust or Python client, and optionally watch it move
in a MuJoCo window. Choose the [desktop viewer](#start-with-visualization) to watch motion,
or the [headless server](#start-without-a-window) for scripts and CI.

## Start without a window

Prerequisites: Docker on Linux. Check that no other simulator is using ports 1337/1338.

```sh
docker run --rm --network host ghcr.io/barisyazici/franka-sim:1.1.7
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

## Start with visualization

On a **Linux desktop with X11 or XWayland**, `--vis` opens a native MuJoCo window.
It is not a browser page. Stop the headless container first: the viewer runs the same
simulation server and uses the same ports.

The **published viewer image includes the graphics libraries**. No local image build or
registry login is needed. This image targets Linux amd64 (x86-64); it is not an ARM64
image for the Raspberry Pi.

```sh
docker pull ghcr.io/barisyazici/franka-sim:1.1.7-viewer
```

Run it from a terminal in your desktop session:

```sh
SIM_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
test -n "$DISPLAY" && test -f "$SIM_XAUTHORITY"

docker run --rm -it --name franka-sim-viewer --network host \
  --user "$(id -u):$(id -g)" \
  -e DISPLAY -e XAUTHORITY=/tmp/sim.xauthority \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$SIM_XAUTHORITY:/tmp/sim.xauthority:ro" \
  ghcr.io/barisyazici/franka-sim:1.1.7-viewer --physics mujoco --vis
```

The `test` command must succeed before running Docker. This passes your existing X11
session credentials into the container. Software rendering avoids needing GPU passthrough
and is sufficient for this single-arm preview.

Keep that terminal running. In a **second terminal**, from this repository's root:

```sh
FRANKA_REALTIME=ignore cargo run --release --example generate_joint_position_motion -- 127.0.0.1
```

Press Enter when the example prompts. The simulated arm moves to its starting pose, then
performs a joint motion that you can watch in the window. Run one client example at a time.
Use the mouse to orbit and zoom the view; Ctrl-C in the server terminal stops the simulator.
To include moving gripper fingers, append `--gripper-physics` to the viewer command.

<details>
<summary>If the window does not open</summary>

- **`Failed to load Xlib` or a missing OpenGL library:** use `ghcr.io/barisyazici/franka-sim:1.1.7-viewer`,
  rather than adding `--vis` to the headless image.
- **`Failed to open display` or an authorization error:** check `echo "$DISPLAY"` and that
  `SIM_XAUTHORITY` points to your current desktop's readable Xauthority file. On Wayland,
  XWayland must be available. Do not continue if the `test` command fails.
- **SSH or a server without a desktop:** use the headless command. The desktop recipe
  requires a local graphical session; it has not been validated on macOS or Windows.
- **Port already in use:** stop the other simulator before launching this one.

</details>

<details>
<summary>Optional: build the viewer image locally</summary>

From this repository's root:

```sh
docker build -t franka-sim:docs-viewer tools/franka-sim-viewer
```

Use `franka-sim:docs-viewer` in place of the published image in the launch command.
The local recipe adds the graphics libraries to the `1.1.7` headless image.

</details>

## Server flags

Arguments after the image name go to the server:

| flag | effect |
|---|---|
| `--vis` / `-v` | Opens the native visualization; requires the desktop setup above. |
| `--gripper-physics` | Includes the physics gripper so its fingers move in the viewer. |
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
  qualification ([Benchmarks](../reference/benchmarks.md)).
- Not the robot: it has documented divergences from real hardware, each pinned by a test that
  names the gap in its failure message ([Simulator gaps](../reference/simulator-gaps.md)).
- Only the FR3 image is published; the FER / FCI v5 image (`franka-sim:panda-v5`) needs a
  local build, or `FRANKA_SIM_ADDR` pointed at a running v5 server.

Running the crate's test suites against it, including the harness that starts the container
for you, is described in [Test against the simulator](../howto/simulator-tests.md).
