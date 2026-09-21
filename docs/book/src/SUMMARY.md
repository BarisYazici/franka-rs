# Summary

[franka-rs](./introduction.md)

# Start here

- [How the pieces fit](./getting-started/architecture.md)
  - [Architecture in detail](./getting-started/architecture-details.md)
- [Choose your setup](./getting-started/choose-your-setup.md)
- [Without a robot: franka-sim](./getting-started/simulator.md)

# Raspberry Pi setup

- [Build a Pi controller](./getting-started/raspberry-pi.md)
  - [1. Parts and assembly](./getting-started/pi-hardware.md)
  - [2. System and network](./getting-started/pi-system.md)
  - [3. Node and laptop client](./getting-started/pi-software.md)
  - [4. Optional enclosure](./getting-started/pi-enclosure.md)

# Direct Rust or Python

- [Install](./getting-started/install.md)
- [The realtime machine](./getting-started/realtime-machine.md)
- [First program](./getting-started/first-program.md)
- [From Python](./getting-started/python.md)
- [Try target control online](./getting-started/simulation-lab.md)

# Add capabilities

- [Peripherals: gripper, cameras, recording](./getting-started/peripherals.md)
- [Command from a low-rate program](./howto/target-control.md)
- [Use the gripper](./howto/gripper.md)
- [Record and replay a run](./howto/flight-recorder.md)
- [Serve arms over Zenoh: franka-node](./howto/franka-node.md)
- [Tune a running controller](./howto/live-tuning.md)
- [Run the examples](./howto/examples.md)

# Advanced

- [How the FCI works](./concepts/fci.md)
- [Three ways to control the arm](./concepts/control-interfaces.md)
- [The realtime rules](./concepts/realtime-rules.md)
- [Reflexes, limits and recovery](./concepts/safety.md)
- [State and errors](./concepts/state-and-errors.md)
- [Write a 1 kHz callback](./howto/callback-control.md)
- [Drive the loop yourself](./howto/active-control.md)
- [Use the model](./howto/model.md)
- [Test against the simulator](./howto/simulator-tests.md)
- [Build for another machine](./howto/cross-compile.md)

# Reference

- [Compared with libfranka](./reference/libfranka.md)
- [FCI v10 and FCI v5 on the wire](./reference/wire-protocol.md)
- [FER / Panda specifics](./reference/fer.md)
- [Rate limiting and filtering](./reference/rate-limiting.md)
- [Online trajectory generation](./reference/otg.md)
- [The impedance backend](./reference/impedance.md)
- [Live parameter protocol](./reference/node-parameters.md)
- [Model parameters and conformance](./reference/model.md)
- [Benchmarks](./reference/benchmarks.md)
- [Simulator gaps](./reference/simulator-gaps.md)
- [API reference](./api-reference.md)
- [Changelog](./changelog.md)

---

[Contributing](./contributing.md)
