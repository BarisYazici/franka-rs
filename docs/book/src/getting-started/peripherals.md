# Peripherals: gripper, cameras, recording

At the end of this page you know what each optional part of a setup does, which process
owns it, and where its full description is. Nothing here is required for moving the arm;
the two setups are described in [Choose your setup](./choose-your-setup.md).

## The Franka Hand

The gripper has its own connection on port 1338 of the robot's host, separate from the arm's
1 kHz link, and its own commands: homing (measures the finger stroke; needed after changing
fingers), move to a width, grasp at a force, stop. Widths are metres.

**Setup A, your program on the realtime machine:** `franka::Gripper` in Rust, or
`robot.gripper()` in Python, talks to the hand directly.
[Use the gripper](../howto/gripper.md) is the walkthrough.

**Setup B, through franka-node:** the node owns the hand when the arm's table names it:

```toml
[[arm]]
name = "fr3"
host = "172.16.0.2"
gripper = "hand"                       # the Franka Hand at host, port 1338
gripper_speed = 0.1                    # m/s of every move and grasp
```

The hand then answers to the arm's lease holder in every phase; the arm does not have to be
enabled. In the Python client:

```python
with node.arm("fr3") as arm:
    arm.gripper.homing()                   # blocks until the homing is over
    print(arm.gripper.state().width)       # measured width, m
    arm.gripper.move(0.04)                 # to 4 cm; waits until it is there
    held = arm.gripper.grasp(0.02, 20.0)   # close on an object at 2 cm with 20 N; True when held
```

`arm.gripper.stop()` works from any client. Stopping the arm, a lost lease or the watchdog
do not stop an ongoing gripper action; use `arm.gripper.stop()` explicitly. The status of the driver through the node is in the crate
README: at the time of writing it is tested against the simulator's gripper server and not yet
validated on a hand. The keys, the wire layout and how to add a driver for another gripper are
in [Serve arms over Zenoh](../howto/franka-node.md#grippers).

## Cameras

`franka-cam` is a second Zenoh node, a separate process on purpose: USB is where the flakiness
lives, and a camera that stalls must not take the robot's process with it. It opens V4L2
cameras (USB webcams, MJPEG or raw), publishes every frame with the timestamp the driver
captured it at, and reports each camera once a second. It belongs on the same host as
`franka-node`, because the two then stamp frames and arm states with the same clock and a
policy or a recording can line them up.

Install it as you installed the node: `cargo binstall franka-cam` fetches the prebuilt binary
from the release, `cargo install franka-cam --locked` compiles it from crates.io, and from a
checkout it is `cargo build --release -p franka-cam`. A minimal configuration:

```toml
name = "cameras"                       # distinct from the arm node name

[zenoh]
listen = ["tcp/0.0.0.0:7448"]          # 7448: the arm node has 7447

[[camera]]
name = "wrist"
device = "/dev/v4l/by-path/<your-camera>-video-index0"
width = 640
height = 480
fps = 30
preview_fps = 5                        # every 6th frame on franka/cam/wrist/preview
format = "mjpeg"
```

After the source build, run `./target/release/franka-cam cam.toml` (or
`franka-cam cam.toml` if installed). Replace the device placeholder with your camera's
actual V4L2 path. Frames are published on `franka/cam/wrist/frame`; the lower-bandwidth
preview is on `franka/cam/wrist/preview`. Your application subscribes over Zenoh;
this is a camera stream, not a browser video URL. The crate README documents decoding.

Publishing has been exercised with USB cameras beside the arm node on a Pi 5.
**Camera recording is tested with synthetic inputs and is not yet validated beside real
arms.** Recording supports MJPEG and requires the camera and arm node on the same host.

Prefer a `by-path` device name when identical cameras have duplicate serial numbers. Device options and frame decoding are documented in
`crates/franka-cam/README.md` in your source revision; it also lists hardware validation.

## Recording

Both the library and the node can write a [Rerun](https://rerun.io) recording: every 1 kHz
cycle's state, the command that was sent and, through the node, each target accepted or
refused with its reason. That is the material for the question every reflex raises: what
happened in the second before?

**Setup A:** [Record and replay a run](../howto/flight-recorder.md) covers the control log the
crate keeps, the `franka-rerun` tools that turn it into a recording, and the live `Recorder`.

**Setup B:** build the node with the `record` feature (Rust 1.96 or newer) and name a
writable directory in the existing arm table (do not add a duplicate `[[arm]]`):

```sh
cargo build --release --locked -p franka-node --bin franka-node --features record
mkdir -p recordings
```

```toml
[[arm]]
name = "fr3"
host = "172.16.0.2"
record_dir = "./recordings"            # create this writable directory first
```

Every arm session, an `enable` or an arm `home`, becomes one `<arm>-<UTC stamp>.rrd`; a session enabled
with an episode name, `arm.cartesian_targets(episode="pick-0042")` in Python, writes
`pick-0042-fr3.rrd` instead, and two arms enabled with the same name load in the viewer as one
recording. A camera whose config says `record_with = "fr3"`, with a `record_dir` of its own
and `franka-cam` built with the same feature, records its frames between that arm's session
start and end into a file the viewer loads alongside. In the camera config, `record_dir`
is a top-level key before `[zenoh]`, while `record_with` belongs inside `[[camera]]`.
Connect the camera node to the arm node with `connect = ["tcp/127.0.0.1:7447"]` under
`[zenoh]`. If you restricted the arm node to a network interface, also add
`"tcp/127.0.0.1:7447"` to its `listen` list, or use its reachable interface address here.
See the camera README for a complete recording configuration.

Open a file with the viewer of exactly the version the crates pin, **0.37.1**:

```sh
pip install rerun-sdk==0.37.1
rerun recordings/pick-0042-fr3.rrd    # replace with your actual recording filename
```

The arm is drawn with the built-in meshes of `franka-rerun`; the recording holds the arm's
joints, the end effector, the external wrench and the commander's targets on a shared time
axis. The details, the two timelines every row carries and what is validated on hardware are
in [Serve arms over Zenoh](../howto/franka-node.md#recording).
