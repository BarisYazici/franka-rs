# franka-cam

A Zenoh camera node that belongs next to [`franka-node`](../franka-node): V4L2 frames with the
timestamp the driver captured them at, so a policy on another host can line a frame up with
the arm state of the same instant and a recording can hold both. A separate process on
purpose, not a feature of the arm node: USB is where the flakiness lives, and a camera that
stalls or leaks must not take the robot's process with it.

`franka-cam <config.toml>` opens every `[[camera]]` of the file on its own thread, publishes the
frames, and reports each camera once a second. Built with `--features record`, a camera whose
`record_with` names an arm also follows that arm's episodes: between the `start` and the `end` of
one, its frames go into `<record_dir>/<recording_id>-<arm>-cam.rrd` under the arm episode's own
`RecordingId`, so a viewer handed both files shows the arm and its cameras as a single recording.
Every frame carries two timelines: `host_time`, its own `CLOCK_MONOTONIC` exposure stamp, which is
what a camera and *any* arm of the episode line up on, and `robot_time`, that stamp mapped through
the followed arm's clock pairs. A frame the pairs cannot map is still written, on `host_time`
alone, and counted as unmapped rather than dropped. The arm is in the file name because the
recording id need not be: a collector that names one episode for two arms gives both the same id.
Frames taken outside an episode are published and not recorded.

## Keys

| key | kind | content |
|---|---|---|
| `franka/cam/<name>/frame` | publisher | `CameraMsg` header and the encoded frame, at the camera's rate |
| `franka/cam/<name>/preview` | publisher | every nth frame, at `preview_fps`, for a consumer that must not pull the full rate |
| `franka/cam/<name>/state` | publisher | JSON at 1 Hz: the device, format, rate and counters |
| `franka/node/<node>/status` | publisher | JSON at 1 Hz: one entry per camera of the process |
| `franka/<arm>/episode` | subscriber | with `record`, per followed arm: the `start` that opens a `.rrd` and the `end` that closes it |
| `franka/<arm>/state` | subscriber | with `record`, per followed arm: the phase, which says whether a session runs, and the `(t_node_ns, robot_time_ms)` pair that puts a frame on the robot's clock |

`<name>` is the `name` of a `[[camera]]` table and `<node>` the file's own `name`, both
`[A-Za-z0-9_-]+`, so `cam` is not a usable arm name for the node next door, which already
refuses it. The node's health goes under `franka/node/`, where the arm node publishes its own,
so one subscription to `franka/node/*/status` sees every node of a lab, and a node's status
stays apart from its cameras' keys.

## The frame header

36 bytes, little-endian, `#[repr(C)]`, every field of alignment one, so Python unpacks it with
`struct.Struct("<BBHHHIQQQ")` and slices the frame off at byte 36.

| field | type | offset | content |
|---|---|---|---|
| `version` | u8 | 0 | 1 |
| `format` | u8 | 1 | 1 MJPEG (one complete JPEG), 2 YUYV, 3 H.264 Annex B, 4 NV12 |
| `flags` | u16 | 2 | bit 0: `t_capture_ns` is `CLOCK_MONOTONIC`; bit 1: the stamp is the start of exposure; bit 2: the driver flagged the buffer as erroneous. A bit unset means the producer could not tell |
| `width`, `height` | u16, u16 | 4, 6 | pixels, as the driver granted them, not as asked |
| `seq` | u32 | 8 | the driver's `v4l2_buffer.sequence`; a gap is a frame that never reached the node |
| `t_capture_ns` | u64 | 12 | the capture stamp, ns |
| `t_node_ns` | u64 | 20 | the host's `CLOCK_MONOTONIC` after the buffer was dequeued |
| `t_wall_ns` | u64 | 28 | `CLOCK_REALTIME` next to it, for a consumer on another host; 0 when this host's wall clock is plainly unset, as a board without a real-time clock reports before NTP |

`t_node_ns` is the host's `CLOCK_MONOTONIC`, the same clock the arm node stamps
`StateMsg::t_node_ns` with, which is how a frame and an arm state on one host are put on one
timeline. `t_wall_ns` is what a second host has to work with until the clocks are aligned; a
clock that is set but wrong looks like a good one here.

## What a node is made of

| module | what it does |
|---|---|
| `v4l2` | the `videodev2.h` structs, the ioctl numbers computed from their sizes, `Device`, and the mmap `Stream` whose `Frame` re-queues its buffer on drop. With `sys` the only `unsafe` here; raw ioctls so no build host needs libclang |
| `source` | `FrameSource`, the `V4l2Source` over a camera, and behind the `fake` feature a source that needs no device |
| `capture` | one thread per camera: dequeue, trim an MJPEG frame at its last end-of-image, stamp, publish, count. Normal priority, never `SCHED_FIFO`, pinned to the config's `cpu`. A camera that goes away is reopened with a growing backoff; a config the device will never satisfy stops that camera instead of spinning |
| `transport` | the session, the frame publishers (`CongestionControl::Drop`, `express`) and the 1 Hz state and status thread |
| `status` | the per-camera atomics and the JSON both topics carry |
| `record` | the `record` feature: an arm's episode and state topics, the map from a frame's exposure time to the robot's clock, and one writer thread per followed arm logging `EncodedImage` rows under `<arm>/cam/<name>`, where that arm's camera view looks. It closes an episode nothing else closed. Without the feature the same surface does nothing and no Rerun code is compiled |

A frame costs one `Vec<u8>`: the header and the frame bytes are written into it once and it is
handed to Zenoh rather than copied. The dequeued buffer goes back to the driver before the
publisher is called, so a slow subscriber cannot stall the camera's queue — with
`CongestionControl::Drop` a full link drops the frame, which for frames is the right answer.

While an episode is open the frame bytes are copied once more, out of that same `Vec`, and handed
to the writer thread through a bounded channel with `try_send`: the capture thread never waits on
the Rerun SDK or on the disk, and a full queue drops the frame and counts it in the camera's
`record_dropped`. Between episodes the recorder reads one atomic and allocates nothing. Only
MJPEG is recorded — a raw stream is twenty times the bytes of the same frame as JPEG, and this
recorder does not use Rerun's H.264 `VideoStream`, which needs `ffmpeg` in the viewer — so another
format is published, refused once with a warning, and counted.

The clock is the reason the cameras belong on the arm node's host: a frame is logged at the time
the driver says its exposure began, on this host's `CLOCK_MONOTONIC`, and the arm's state carries
that same clock next to the robot's, so the map is one offset. That stamp is the frame's own data
and goes into the recording as `host_time` whatever the arm says. The map to `robot_time` needs
more: the episode's stamp has to be one this host could have made, and the arm has to say a
session is running, because an idle arm refreshes the snapshot its `robot_time_ms` comes from at
most once a second while stamping `t_node_ns` at every encode, so such a sample is two clocks read
a second apart; two consecutive in-session samples have to agree on the offset before either is
believed. Short of that a frame gets `host_time` and no `robot_time` — a row must not claim a
robot time it never had — and an episode stamped on *another host's* clock is refused
outright, because then even the host stamp means nothing here.

The `end` sample is best effort — the arm's publisher drops rather than blocks, and a killed node
sends none — so the writer also closes the file itself when the arm reports no session running
for two seconds, or stops publishing states for five. The state topic stops naming the file at
the same moment.

A stream the driver does not stamp with `CLOCK_MONOTONIC` is refused at start (`uvcvideo`'s
`clock=realtime` module parameter is what would do that): a stamp on another clock cannot be
lined up with the arm node's states, which is the reason this node exists.

## Install

`cargo binstall franka-cam`, or any of the ways
[`franka-node` installs](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node#install)
with `franka-cam` in its place (its tarball has no systemd unit).

## Configuration

```toml
name = "pi"                            # the <node> of franka/node/<node>/status
# record_dir = "/var/lib/franka-cam"   # --features record: one <recording_id>-<arm>-cam.rrd per episode

[zenoh]
mode = "peer"                          # or "client": dial the routers in connect, listen on nothing
listen = ["tcp/0.0.0.0:7448#iface=eth0"]   # 7448: the arm node has 7447
connect = []
scouting_interface = "eth0"            # optional; "auto" when absent
lease_ms = 1000                        # transport.link.tx.lease, keepalive is lease/4
# zenoh_config = "/etc/franka/zenoh.json5"  # a Zenoh file to start from: TLS material, auth, ACLs

[[camera]]
name = "wrist"
# by-path keys on the socket, which is what distinguishes two cameras of the same model: they
# report the same serial, so /dev/v4l/by-id names only one of them.
device = "/dev/v4l/by-path/<your-camera>-video-index0"
width = 640
height = 480
fps = 30
preview_fps = 5                        # every 6th frame on franka/cam/wrist/preview
format = "mjpeg"                       # or yuyv, h264, nv12
buffers = 4
cpu = [0, 1]                           # never an isolated core: those are a realtime loop's
controls = { exposure_auto = 1 }       # UVC controls by name, set once at start
# record_with = "L"                    # the arm whose episode opens and closes the recording
```

A consumer that is not on this network needs `mode = "client"` and a router in `connect`: the
node then dials out and listens on nothing, so no inbound rule is needed, and an endpoint that
leaves the local segment should be `tls/` rather than `tcp/`. The TLS transport is compiled in;
the certificates, authentication and any access control belong in a Zenoh configuration file
named by `zenoh_config`, which this table's keys are applied on top of. Frames are camera footage
of a lab, so nothing should leave a network you trust without one. Such a consumer usually wants the preview
key rather than the frame key: `preview_fps = 5` next to `fps = 30` puts every sixth frame on
`franka/cam/<name>/preview` for a sixth of the bandwidth, out of the same bytes and with no
re-encoding. The divider is rounded up, so the preview is never faster than the rate asked for.

`name` and `device` of a camera are required, the rest has the defaults above. Unknown keys are
errors; camera names and devices must be unique and a camera may not take the node's own name;
an affinity must name cores the host has; `preview_fps` must be below the camera's `fps`; a
client must name a router; and `record_dir` without the `record` feature, or `record_with`
without a `record_dir`, is a config error. A `controls` key is a V4L2 control by
name (`exposure_auto`, `auto_exposure`, `gain`, `power_line_frequency`, …) or its id written out
(`"0x009a0902"`); a name the crate does not know, or a control the camera does not have, stops
that camera with the name in the message rather than leaving an episode unreproducible.

## Status

Publishing runs on a Raspberry Pi 5 with two USB cameras at 30 fps beside an arm node driving
two arms. An MJPEG frame at 640x480 is typically 50 to 60 kB, so about 12 to 13 Mbit/s per camera
at 30 fps, and a 5 fps preview about 2 Mbit/s.

Recording is not yet validated on hardware beside the arms: it is tested in process, against a
fake camera and a synthetic episode, and the `.rrd` it writes is read back for its store ids and
its `<arm>/cam/<name>` rows. What an episode costs the storage is the arm's own file plus the
camera bandwidth above.

## Tests

No camera is needed for any of this:

```sh
cargo test -p franka-cam                     # the unit tests
cargo test -p franka-cam --features fake     # plus the node in process on the loopback
```

The capture loop, the wire and the status topic are tested through the `fake` source, which
emits a padded JPEG at a set rate and skips sequence numbers on demand. A real camera is one
ignored test:

```sh
FRANKA_CAM_DEVICE=/dev/video0 cargo test -p franka-cam --test hardware -- --ignored --nocapture
```

It takes 60 frames, checks the driver's own stamps against the rate it granted and prints what
the frames look like.
