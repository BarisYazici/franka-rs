# Install

At the end of this page the crate builds in your project, or `import franka` works in your
Python environment.

## Rust

```sh
cargo add franka-rs
```

or in `Cargo.toml`:

```toml
[dependencies]
franka-rs = "0.3"
```

The library is named `franka`, so you write `use franka::Robot;`. Rust 1.85 or newer
(edition 2021). Build with `--release`: a debug build of a stiff torque controller misses
cycles. To follow `main` instead of a release, depend on the git repository:
`franka-rs = { git = "https://github.com/BarisYazici/franka-rs" }`.

| feature | default | what it does |
|---|---|---|
| `model-library` | on | Compiles `Robot::load_model_from_robot`, the `dlopen` path for the model library an FER serves (pulls in `libloading`). Turn it off for a static musl build or to keep `dlopen` out of your process; `Robot::load_model()` works either way, on both robots. |
| `serde` | off | `Serialize` / `Deserialize` on `RobotState`, `RobotMode`, `Errors`, `Duration`, `Record`, `RobotCommandLog`, `MoveStatus` and `ControlException`, so a control log can be saved as JSON and replayed; see [Record and replay a run](../howto/flight-recorder.md). |

## Python

```sh
pip install franka-rs
```

The wheel is `franka-rs`, the import name `franka`; Python 3.9 or newer, numpy is the only
dependency. See [From Python](./python.md).

## Rerun viewer (optional)

Only needed for the [flight recorder](../howto/flight-recorder.md) and the Rerun examples.
The viewer must be 0.37.1, the SDK version `crates/franka-rerun` pins:

```sh
pip install rerun-sdk==0.37.1
# or
cargo install rerun-cli --locked --version 0.37.1
```

Both put a `rerun` binary on the path; check with `rerun --version`.

Next: [The realtime machine](./realtime-machine.md), or
[Without a robot: franka-sim](./simulator.md) if you have no arm at hand.
