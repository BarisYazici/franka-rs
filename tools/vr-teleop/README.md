# franka-vr-teleop — drive a Franka arm from a Meta Quest controller

Two small Python processes between a Quest headset and
[`franka-node`](https://github.com/BarisYazici/franka-rs/tree/main/crates/franka-node):

```
Quest ──adb logcat──▶ franka-vr-bridge ──ZMQ PUB, 88-byte VrTargetMsg, 50 Hz──▶ franka-vr-teleop
                                                                                        │
                                                                  Zenoh, 80-byte TargetMsg
                                                                                        ▼
                                                                                   franka-node ──▶ arm
```

`franka-vr-bridge` reads both controllers from one adb logcat stream, maps each into the
robot base frame and publishes one channel per controller. `franka-vr-teleop` takes one
channel, holds a clutch on the grip, clamps what it sends and drives one arm; a second process
with `--arm R` and its own client id drives the other. Nothing is shared between the two.

**This moves a real robot.** Read
[`docs/design.md`](https://github.com/BarisYazici/franka-rs/blob/main/tools/vr-teleop/docs/design.md)
before changing a threshold: every rule there is a measurement or an incident, and the rotation
and translation scales in particular are what keep a wrist flick inside the arm's budget. The
node is the last guard and enforces its own step, lead and rate limits whatever a client
sends — but it is the last one, not the only one you should rely on.

## Install

- A **Meta Quest** (verified on a Quest 3) with developer mode on, and `adb` (the Android
  platform-tools) on PATH. USB to the bridge host is preferred: lower latency and far less
  jitter than Wi-Fi.
- A machine to run the bridge and the clients on, next to the robot rather than on a laptop
  across the network. A Raspberry Pi is enough for both.
- A running **`franka-node`** per arm, reachable over Zenoh, of the same release as this
  package.

```sh
python3 -m venv .venv && . .venv/bin/activate
pip install franka-vr-teleop              # the bridge and the teleop client
```

The teleop client talks to the node through
[`franka-node-client`](https://pypi.org/project/franka-node-client/), which it pins to its
own version. From a checkout: `pip install ./crates/franka-node/python ./tools/vr-teleop`.

| command | what it does |
|---|---|
| `franka-vr-bridge` | the headset → ZMQ bridge, one channel per `--controller` |
| `franka-vr-probe` | what the bridge publishes right now, per channel, with a diagnosis |
| `franka-vr-teleop` | one controller drives one arm; `--probe-limits` first, at the bench |
| `franka-vr-fetch-apk` | download the headset APK and verify its hash |
| `franka-vr-install-apk` | fetch the APK if needed, then `adb install` it |

## The headset APK

The headset runs upstream's `com.rail.oculus.teleop` app, which streams controller poses over
logcat. **The APK is not in this package or repository**: it embeds Meta's Oculus Mobile SDK
under Meta's licence, which is not ours to redistribute. Fetch it from the upstream commit this
bridge was verified against, with the headset plugged in:

```sh
franka-vr-fetch-apk        # downloads and hash-checks
franka-vr-install-apk      # fetches if needed, then adb install
```

`franka-vr-fetch-apk` verifies sha256 `97b49f94…` (4,865,715 bytes) and **refuses** anything
else: upstream later replaced this APK with a different build and a community fork carries a
third, neither of which has been tested here. A mismatch is a hard failure, not a warning —
nothing is written to disk. The file is kept per user in `$XDG_DATA_HOME/franka-vr/` (default
`~/.local/share/franka-vr/`), never in the installed package, and it is verified again before
every install, including the one `franka-vr-bridge` makes when the app is missing. Once the
app is on the headset the file is never needed again.
See [`NOTICE`](https://github.com/BarisYazici/franka-rs/blob/main/tools/vr-teleop/NOTICE) for
what is vendored and under which licence.

## Running it

```sh
franka-vr-bridge --controller l,r \
    --endpoint tcp://127.0.0.1:5560,tcp://127.0.0.1:5570 [--oculus-ip IP]
franka-vr-probe --secs 8
franka-vr-teleop --arm L --probe-limits --connect tcp/127.0.0.1:7447
franka-vr-teleop --arm L --client-id 7201 --connect tcp/127.0.0.1:7447
```

**The port is the hand**: 5560 is the left controller, 5570 the right. Before it acquires
anything, `franka-vr-teleop` listens on both and makes you squeeze the grip of the one you
named, so a crossed pairing is caught by hand rather than by the arm. The first squeeze after
enable never moves the arm — arming is fail-closed, and a release must be seen first.

In a checkout, `tools/vr-teleop/vr.sh` wraps these for the machine beside the robot: it picks
USB or Wi-Fi adb, starts the bridge once in the background, and gives each arm its client id.

```sh
./vr.sh status               # what is running, and how the headset is attached
./vr.sh bridge               # start the bridge (idempotent; USB if present, else Wi-Fi)
./vr.sh probe 8              # what the bridge is publishing right now, both channels
./vr.sh teleop L --seconds 30
./vr.sh stop
```

It finds the console scripts on PATH, or in `VENV/bin`. Settings come from the environment,
or from an optional **`vr.env`** beside `vr.sh` that is gitignored — that is where a site's
own values belong, not in the repository:

| variable | default | what |
|---|---|---|
| `VENV` | *(none)* | a venv holding `franka-vr-teleop`, when it is not the active one |
| `QUEST_IP` | *(none)* | headset IP, **required for Wi-Fi**, unused on USB. `adb shell ip route` on the headset reports it. |
| `ZENOH` | `tcp/127.0.0.1:7447` | the node's Zenoh endpoint |
| `EP_L`, `EP_R` | `tcp://127.0.0.1:5560`, `:5570` | the bridge's two channels |
| `CLIENT_ID_L`, `CLIENT_ID_R` | `7201`, `7202` | node client ids, one per teleop process |
| `BRIDGE_LOG` | `$TMPDIR/franka-vr-bridge.log` | the background bridge's log |

## Testing

From `tools/vr-teleop/` in a checkout, with the package installed (`pip install -e .` after
the client):

```sh
python -m pytest             # the bridge, the wire, the fixtures, the version
tests/teleop/run_tests.sh    # the teleop client's rules, on recorded and synthetic wires
```

No headset and no robot: the client was built to run offline, with files as wires, a stand-in
arm, and an in-process ZMQ pair for the hand check. `tests/teleop/selftest.py` and
`selftest_recovery.py` are the two check suites (`python -m tests.teleop.selftest`);
`run_tests.sh` runs both and drives a recorded wire through `--dry-run --replay`, asserting on
the emitted bytes. A dry run outside a checkout needs `--node-limits FILE`, since the fixture
is not in the wheel. The bridge's `main()` needs adb and a headset and is not covered beyond
`--help`; its state machines are pure and are. The public-safety gate also checks names listed
in `FRANKA_PRIVATE_NEEDLES`, or in the checkout's untracked private needle file, when given.

## The two wire contracts

`VrTargetMsg` (88 bytes, little-endian) is defined once in `franka_vr/wire.py` — magic,
version, `seq`, `pos[3]`, `quat[4]` in scipy's x-y-z-w order, gripper as a closed fraction,
flags (`engaged`, `fresh`, `controller_on`) and buttons. `TargetMsg` (80 bytes) and the node's
`params`, `state` and `cmd` keys are the node's own, documented in
[its README](https://github.com/BarisYazici/franka-rs/blob/main/crates/franka-node/README.md);
the teleop client reads and writes those bytes through `franka-node-client`.

## What is site-specific, honestly

- **The scales are measured on one operator's hands** with one headset
  (`--spatial-scale 0.4`, `--rotation-scale 0.25`). They are the right *kind* of number — see
  the table in `docs/design.md` — but yours may differ; watch the commanded peaks the summary
  prints before raising them. Both are applied in the teleop client's clutch; the bridge
  publishes at spatial scale 1.0.
- **`L`/`R`, the two ports and the two client ids are conventions**, not protocol. They are
  defaults in `vr.sh`, overridable from `vr.env`.
- **The measurements in `docs/design.md` are from a Quest 3 on one workspace**: the 31 %
  not-fresh rate, the 77.7 mm freshness-edge jump and the 26 % antipodal quaternions are
  properties of that reader and that headset. The *rules* they justify are general; the exact
  percentages are not.
- **The axis remap assumes a headset placed upright facing the workspace.** A different room
  orientation needs a different `rmat_reorder`, and the forward-direction latch (right
  thumbstick) is what makes standing anywhere work within that.
- **The node's limits come from the node**: `franka-vr-teleop` reads `params/schema` and
  `params/get` before it acquires anything and refuses to start without them.
- **Not implemented on purpose**: the wire's `anchor` flag bit.

`franka_vr/quest_bridge/oculus_reader/` is vendored from `rail-berkeley/oculus_reader`
(Apache-2.0); see `NOTICE`, which is the file to read before reusing it.
`franka_vr/quest_bridge/mapping.py` is our reimplementation of the ideas in DROID's VR front
end -- no DROID function or block remains; see `NOTICE`.

The package version is the `franka-rs` workspace version. Apache-2.0.
