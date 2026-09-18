# Running franka-node as a service

`franka-node.service` is a systemd unit template for a host that sits next to the robots
(a Raspberry Pi on the robot network, for example). It starts the node after the network is
up, restarts it two seconds after a failure, sets the limits Franka's realtime setup
recommends (rtprio, memlock), and stops it with SIGINT so every arm is stopped before the
process exits.

Get a release tarball as in the [README's Install section](../README.md#install) and, on the
target, in the directory it unpacks to:

```sh
sudo install -m 755 franka-node /usr/local/bin/franka-node
sudo install -d /etc/franka-node
sudo install -m 644 config.example.toml /etc/franka-node/node.toml
sudo install -m 644 franka-node.service /etc/systemd/system/
```

After `cargo binstall` or `cargo install`, the binary is `~/.cargo/bin/franka-node` and the two
files are in `crates/franka-node/` of the repository.

For two arms, use the supplied `config.two-arms.toml` instead of `config.example.toml`;
it retains the shipped control defaults and documents the second network link.

Edit `/etc/franka-node/node.toml` (the arms' hosts, the interface Zenoh listens on) and set
`User=` in the unit to the account that runs the node (`Group=` is `realtime`; an unedited
copy refuses to start). Then:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now franka-node
journalctl -u franka-node -f              # transitions at info; RUST_LOG=debug for refusals
```

`systemctl stop franka-node` sends SIGINT and waits up to `TimeoutStopSec` (20 s: the arms
stop one after the other, each within the library's 5 s stop timeout). The node's own health
is on the Zenoh key `franka/node/<name>/status`, once a second.
