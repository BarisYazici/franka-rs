# 3. Node and laptop client

The Pi has the kernel, the realtime limits and both network paths from
[System and network](./pi-system.md). This page puts `franka-node` on it and a Python client
on your laptop, then reads the arm's state. Motion is a separate, marked section at the end.

## Install franka-node on the Pi

**From a release**, `cargo binstall franka-node` (prebuilt) or `cargo install franka-node --locked`.

**From a source checkout.** Install Rust 1.89 or newer with [rustup](https://rustup.rs).
Put a copy of this repository on the Pi, at a revision that contains `crates/franka-node`,
and from its root run:

```sh
cargo install --path crates/franka-node --locked
```

The binary lands in Cargo's binary directory, normally `~/.cargo/bin`. Building on the Pi
takes a while; [Build for another machine](../howto/cross-compile.md) builds it on a PC
instead. Per-session recording needs `--features record` and Rust 1.96, see
[Peripherals](./peripherals.md#recording).

**From a published release.** Only when a node release of the matching version exists on
crates.io and GitHub; older core-only releases do not include it. `cargo binstall franka-node`
fetches the prebuilt aarch64 binary, `cargo install franka-node --locked` compiles it from
crates.io, and the [how-to page](../howto/franka-node.md#installing) has the release tarball
for a Pi without cargo. When in doubt, use the source checkout.

## A minimal configuration

Save this as `node.toml` on the Pi:

```toml
name = "pi"                          # this node's name in franka/node/pi/status

[zenoh]
listen = ["tcp/0.0.0.0:7447"]        # clients connect to tcp/<pi-address>:7447
multicast_scouting = false           # the client names the Pi, no discovery needed

[[arm]]
name = "left"                        # the <arm> in every key: franka/left/state, ...
host = "172.16.0.2"                  # the robot on the wired link
realtime = "enforce"                 # refuse to start unless the Pi can run realtime
```

Every other key has a default. The arm's `name` is a label you choose (letters, digits, `-`
and `_`). Keep `realtime` on `enforce` for a robot; `"ignore"` is for the
[simulator](./simulator.md). Plain TCP is for a trusted network; the
[node's status section](../howto/franka-node.md#status) covers TLS and authentication.

A second arm is one more `[[arm]]` table, `name = "right"` and `host = "172.16.2.2"` on its own
link; the addressing is on [System and network](./pi-system.md). The complete
[two-arm example](https://github.com/BarisYazici/franka-rs/blob/main/crates/franka-node/config.two-arms.toml)
keeps the shipped control defaults, with optional per-arm CPU pinning and Franka Hands.
[Live tuning](../howto/live-tuning.md) explains which TOML values can change during a session.

## Start the node and check it

With FCI mode unlocked in Desk, run the node in the foreground:

```sh
franka-node node.toml
```

It connects to each arm and publishes the arm's state at 100 Hz. Nothing moves: the node
holds no session until a client asks for one. Ctrl-C stops every arm and exits;
`RUST_LOG=debug` adds every refused target with its reason. Before going on, make sure the
log shows no `Realtime` error and that port 7447 is open on any firewall the Pi runs.

## Install the Python client on the laptop

With Python 3.9 or newer, the release of the node's version is `pip install franka-node-client`;
from the root of the node's repository revision:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install ./crates/franka-node/python
```

<details>
<summary>Windows PowerShell</summary>

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install .\crates\franka-node\python
```

</details>

The package imports as `franka_node` and depends only on `eclipse-zenoh` and numpy. Once a
release of the matching version is on PyPI, `pip install franka-node-client` installs the
same package. The [Python bindings](./python.md) need the robot link on the machine they run
on, so they are not what this setup wants.

## Read the state, no motion

Replace `<pi-address>` with the Pi's address on the laptop's path. This takes no lease and
cannot move the arm:

```python
import franka_node

with franka_node.Node("tcp/<pi-address>:7447") as node:
    print(node.status("pi"))                # {"node": "pi", "arms": [{"name": "left", "phase": "idle", ...}]}
    state = node.arm("left").state()        # the latest state, waiting up to 1 s for the first
    print("phase", state.phase, "robot mode", state.robot_mode, "errors", state.has_errors)
    print("position [m]", state.position)   # end effector in the base frame
    print("q [rad]", state.q)
```

`phase` is `idle` on a fresh node, `robot_mode` is the robot's own mode and `has_errors`
says whether the robot reports an error. A `NodeTimeout` means nothing arrived: check the
address, the port, the node name and that the node is running. The fields and
`arm.on_state(callback)` are in [Serve arms over Zenoh](../howto/franka-node.md#from-python).

## First motion (moves the arm)

> **This section moves the robot.** Before running it: the workspace above the end effector
> is clear, a person is at the external activation device, the brakes are open and FCI mode is
> unlocked. The arm rises by 5 cm and stops there.

```python
import franka_node

with franka_node.Node("tcp/<pi-address>:7447") as node:
    with node.arm("left") as arm:                    # takes the lease; released on exit
        with arm.cartesian_targets() as targets:     # starts the 1 kHz loop; stopped on exit
            targets.move_by([0.0, 0.0, 0.05])        # +5 cm in z from the current target
            state = targets.wait(timeout=5.0)
            print("arrived at", state.position, "refused targets:", state.refused)
```

`move_by` sets the goal and a client thread streams targets toward it at 50 Hz, paced under
the node's limits. A lost lease or the node's watchdog requests a controlled stop, which may
finish at the last accepted target; it is not an emergency stop. `Refused` with `held by <id>`
means another client holds the arm. After a robot stop or fault, resolve its condition first,
then `arm.recover()`.

## Run the node as a service

Once the foreground run works, stop it (only one FCI client can own the robot) and install
the supplied systemd unit. It sets the realtime limits itself, restarts the node after a
failure and stops it with SIGINT so every arm settles first;
[The systemd unit](../howto/franka-node.md#the-systemd-unit) explains it.

<details>
<summary>Install the binary, the configuration and the unit</summary>

From the repository root on the Pi, with the `node.toml` you just tested:

```sh
sudo install -m 755 "$(command -v franka-node)" /usr/local/bin/franka-node
sudo install -d /etc/franka-node
sudo install -m 644 node.toml /etc/franka-node/node.toml
sudo install -m 644 crates/franka-node/deploy/franka-node.service /etc/systemd/system/
```

Edit `/etc/systemd/system/franka-node.service`: replace `User=CHANGE_ME` with the account that
runs the node. `Group=realtime` is the group from the system page; an unedited unit refuses to
start. Then:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now franka-node
journalctl -u franka-node -f
```

</details>

<nav class="guide-nav" aria-label="Setup steps"><a rel="prev" href="./pi-system.html">Previous: system and network</a><a rel="next" href="./pi-enclosure.html">Optional: enclosure and models</a></nav>
