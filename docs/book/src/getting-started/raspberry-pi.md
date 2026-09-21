# Build a Pi controller

Keep the control loop beside the robot. Use your laptop for Python, policies, and teleoperation.
This guide builds a Pi 5 controller with an Intel dual-port Ethernet card, for one or two arms.

<figure class="hardware-figure">
<img src="../assets/pi5/assembly.png" alt="Pi 5 with active cooler, a PCIe adapter beneath it, and an upright dual-port Intel Ethernet card. Optional USB cameras are shown beside the assembly.">
<figcaption>Reference assembly: Pi 5 + Active Cooler + P02 PCIe adapter + Intel I350-T2 V2. Cameras are optional. This is a CAD illustration.</figcaption>
</figure>

## Follow these steps

<div class="setup-path">

1. [**Parts and assembly**](./pi-hardware.md)\
   Shopping links, the cooler and PCIe board, and which cable goes where.
2. [**System and network**](./pi-system.md)\
   Install the realtime kernel, enable the Intel NIC, and connect the arms.
3. [**Node and laptop client**](./pi-software.md)\
   Install franka-node, read the first state, then try a small motion.
4. [**Optional enclosure**](./pi-enclosure.md)\
   Explore the 3D assembly and download the printable case or editable CAD.

</div>

## What runs where?

| On your laptop | On the Pi | At the robot |
|---|---|---|
| Your application sends targets over Zenoh | `franka-node` runs `franka-rs` at 1 kHz | One wired Intel NIC port per arm |

The Pi's built-in Ethernet or Wi-Fi connects to your laptop's network. The Intel card's
ports connect directly to the robot control units. Cameras plug into the Pi's USB ports.

Have only one arm? Use one Intel port and omit the second arm's configuration.
You can also use a Pi's built-in Ethernet for a single arm and Wi-Fi for the laptop;
the Intel card and P02 are the illustrated dual-arm build, not a library requirement.

Already have a working realtime host? Go straight to [step 3](./pi-software.md).
For direct Rust or Python without a node, [choose your setup](./choose-your-setup.md).

<nav class="guide-nav" aria-label="Setup steps">

[← Choose another setup](./choose-your-setup.md)

[Start: parts and assembly →](./pi-hardware.md)

</nav>
