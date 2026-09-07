# API reference

The generated rustdoc for the `franka` crate is published alongside this book:

**[→ API reference (`api/franka/index.html`)](./api/franka/index.html)**

It is built by `.github/workflows/docs.yml` with

```sh
cargo doc --no-deps -p franka-rs
```

under `RUSTDOCFLAGS="-D warnings"`, so a broken intra-doc link fails the build, and copied
into the site at `api/`. The library target is named `franka`, which is why the path is
`api/franka/` rather than `api/franka-rs/`.

Good places to start:

| item | what it is |
|---|---|
| `franka::Robot` | The connection. Setters, `read_once`, the `control_*` loops, `start_*_control`, `stop`, `automatic_error_recovery`. |
| `franka::RobotState` | Every field the robot publishes each millisecond. |
| `franka::Model` | Poses, both Jacobians, mass, Coriolis, gravity. |
| `franka::Gripper` | The Franka Hand on port 1338. |
| `franka::FrankaError` | The error taxonomy, one variant per libfranka exception. |
| `franka::rate_limiting` | The FR3 constants and `limit_rate_*`; the FER's live in the `fer` submodule. |

Building it locally:

```sh
cargo doc --no-deps -p franka-rs --open
```
