# 2. System and network

The result of this step is a Pi with realtime scheduling and a separate wired link to each
robot. No robot motion is needed here.

## Install a 64-bit realtime system

A documented route is **Ubuntu Server 24.04 LTS for Raspberry Pi + the Raspberry Pi
real-time kernel**. Write the image to your microSD with
[Raspberry Pi Imager](https://www.raspberrypi.com/software/) using the
[Ubuntu Raspberry Pi image](https://ubuntu.com/download/raspberry-pi). Set up your account,
SSH, and an uplink on the Pi's built-in Ethernet or Wi-Fi.

The kernel route below requires an Ubuntu Pro entitlement. Follow
[Canonical's enablement guide](https://ubuntu.com/pro-client/docs/en/v35/howtoguides/enable_realtime_kernel/):

```sh
sudo apt update
sudo apt install ubuntu-advantage-tools pciutils ethtool
sudo pro attach
sudo pro enable realtime-kernel --variant=raspi
sudo reboot
```

Use **`--variant=raspi`**, not the generic real-time kernel. After reconnecting:

```sh
uname -m                    # aarch64
cat /sys/kernel/realtime     # 1
```

Already have a Pi-compatible `PREEMPT_RT` image? Keep it and check the same outputs.
An ordinary Raspberry Pi OS install is not automatically a realtime system; custom kernel
builds are an advanced alternative, not required by this guide.

## Enable PCIe and identify the Intel ports

For the P02, edit `/boot/firmware/config.txt` and ensure this line is active under `[all]`:

```ini
dtparam=pciex1
```

Reboot. Keep the Pi's default PCIe Gen 2 speed; no Gen 3 override is needed here.
See [Raspberry Pi's PCIe instructions](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#enable-pcie).

```sh
lspci -nnk
ip -br link
```

Find the Intel I350 functions in `lspci`; the driver in use should be **`igb`**.
List interface names with `ip -br link`, then use `ethtool -i <interface>` to match each
interface's `bus-info` to its PCI address in `lspci` (replace the placeholder).
If no card is listed, power down and recheck seating, ribbon orientation, and adapter power.
If no driver is bound, resolve the kernel's `igb` support before continuing.

Plug in one cable at a time and inspect `ip -br link` / `ethtool <interface>` to map the
physical L and R ports. Write down which interface belongs to each arm. The built-in
Ethernet is the uplink and is not one of these Intel interfaces.

For one arm without the Intel card, use the built-in Ethernet as `LEFT_INTERFACE` below,
omit the right interface, and use Wi-Fi for the laptop uplink. Skip the PCIe checks.

## Give each arm its own subnet

Example addresses below are a setup plan, not values discovered on your robot. Use subnets
that do not overlap your uplink, VPN, or another interface. Confirm the robot-side address
in its network configuration before applying the Pi-side settings.

| Link | Pi address | Robot address |
|---|---|---|
| Left Intel port | `172.16.0.1/24` | `172.16.0.2` |
| Right Intel port, optional | `172.16.2.1/24` | `172.16.2.2` |
| Built-in Ethernet or Wi-Fi | Your existing uplink address | Laptop reaches this address |

For two arms, configure the second robot for its separate subnet through the robot's
network settings. Setting the Pi to `172.16.2.1` does not change the robot's address.
Do not put two default `172.16.0.2` robots on separate interfaces in the same routing table
and expect automatic selection. Start with one arm if both still have the same address.

On **Ubuntu Server using Netplan/networkd**, add the following to
`/etc/netplan/60-franka.yaml`. Replace `LEFT_INTERFACE` and `RIGHT_INTERFACE` with the actual
Intel interface names. Remove the right entry for one arm. Keep your existing uplink config;
if another Netplan file already configures these Intel ports, reconcile it first.

```yaml
network:
  version: 2
  ethernets:
    LEFT_INTERFACE:
      renderer: networkd
      dhcp4: false
      dhcp6: false
      addresses: [172.16.0.1/24]
      optional: true
    RIGHT_INTERFACE:
      renderer: networkd
      dhcp4: false
      dhcp6: false
      addresses: [172.16.2.1/24]
      optional: true
```

No default gateway or DNS belongs on the robot links. Those stay on the uplink.
Check and try the configuration with rollback rather than replacing your SSH connection:

```sh
sudo chmod 600 /etc/netplan/60-franka.yaml
sudo netplan generate
sudo netplan try
ip route get 172.16.0.2
ip route get 172.16.2.2       # only with a second arm
```

Each route should show the intended Intel interface and matching Pi source address.
[Netplan's static-address guide](https://netplan.readthedocs.io/en/stable/examples/#how-to-configure-a-static-ip-address-on-an-interface)
explains the configuration. A NetworkManager-managed system needs its corresponding
connection profiles instead.

## Allow realtime scheduling

Create a group and add your login account:

```sh
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
```

Create `/etc/security/limits.d/99-franka-realtime.conf` with:

```text
@realtime - rtprio 99
@realtime - memlock unlimited
```

Log out and back in, then check `ulimit -r` reports `99`. The service in the next step also
sets these limits. See [the realtime machine](./realtime-machine.md) for details.

## Before moving on

- `cat /sys/kernel/realtime` reports `1` and `ulimit -r` reports `99`.
- Each Intel port is identified, and routing selects the expected port for each robot.
- `ping 172.16.0.2` works (and `172.16.2.2` for two arms).
- The laptop can still reach the Pi's uplink address. This is the address used for Zenoh.

Ping verifies basic reachability, not 1 kHz timing. The later
[communication test](./realtime-machine.md#check-the-link-first) **moves the robot** and
must follow the robot's motion prerequisites. Do not run it alongside an active node.

<nav class="guide-nav" aria-label="Setup steps"><a rel="prev" href="./pi-hardware.html">Parts and assembly</a><a rel="next" href="./pi-software.html">Next: node and client</a></nav>
