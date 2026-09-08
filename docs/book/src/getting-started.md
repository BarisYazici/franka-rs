# Getting started

## Adding the dependency

The crate is on [crates.io](https://crates.io/crates/franka-rs):

```toml
[dependencies]
franka-rs = "0.1"
```

To follow `main` instead, depend on the git repository:
`franka-rs = { git = "https://github.com/BarisYazici/franka-rs" }`.

The library is named `franka`, so `use franka::Robot;` is what you write. Rust 1.85 or
newer (edition 2021) is required.

One thing is worth knowing about features:

| feature | default | what it does |
|---|---|---|
| `model-library` | **on** | Compiles the FCI v5 `LoadModelLibrary` + `dlopen` path (`Robot::load_model_from_robot`), pulling in `libloading`. Turn it off for a static musl build, or if you do not want `dlopen` in your process. Nothing else changes: `Robot::load_model()` works either way, on both robots. |
| `serde` | off | Derives `Serialize` / `Deserialize` for `RobotState`, `RobotMode`, `Errors`, `Duration`, `Record`, `RobotCommandLog`, `MoveStatus` and `ControlException`, so a `ControlException`'s control log can be written to JSON and replayed; see [Flight recorder](./flight-recorder.md). `Errors` serialises as the list of the set flags' names. |

Build with `--release`. A debug build of a stiff torque controller will miss cycles.

## Realtime prerequisites

A 1 kHz FCI control loop has a hard 1 ms deadline. Two things are needed to keep it on real
hardware:

1. **A `PREEMPT_RT` kernel.** The crate checks for `/sys/kernel/realtime`, exactly as
   libfranka does.
2. **Permission to raise the thread to `SCHED_FIFO`.** That needs `CAP_SYS_NICE` or a
   nonzero `RLIMIT_RTPRIO`. As an unprivileged user, `ulimit -r 99` in the current shell —
   which only works if a `rtprio` limit is configured for the user, e.g. via
   `/etc/security/limits.d/99-realtime.conf` containing `<user> - rtprio 99` (needs
   `pam_limits` and a fresh login). Ubuntu's realtime-kernel variant sets up a `realtime`
   group that already has `rtprio` and `memlock`.

`Robot::new` takes a [`RealtimeConfig`](./api-reference.md):

- **`RealtimeConfig::Enforce`** — the default, matching libfranka. Checks that the calling
  thread can be raised to `SCHED_FIFO` and that the kernel advertises realtime
  capabilities, and fails with `FrankaError::Realtime` and libfranka's exact message
  otherwise.
- **`RealtimeConfig::Ignore`** — skips both checks and runs the control loop at whatever
  priority the process already has.

Against a simulator, `Ignore` is harmless. Against real hardware it means the loop can miss
its 1 ms deadline under load, which the robot reports as a falling
`control_command_success_rate` and, eventually, a `communication_constraints_violation`
reflex — exactly as it would with libfranka's own realtime checks disabled.

Every example in the repository picks this up from the `FRANKA_REALTIME` environment
variable (`enforce`, the default, or `ignore`), so the same binary serves the RT box and the
simulator:

```sh
FRANKA_REALTIME=enforce cargo run --release --example echo_robot_state 172.16.0.2  # real robot
FRANKA_REALTIME=ignore  cargo run --release --example echo_robot_state 127.0.0.1   # simulator
```

Note that this is *not* the whole story on either side: the crate's `RobotImpl::new` raises
its own control thread's priority unconditionally (libfranka's `Robot::Impl` constructor
does the same), and under `Ignore` it merely swallows a failure. So on a box where
`ulimit -r` is nonzero, the control thread ends up at `SCHED_FIFO` even with `Ignore`.

## Connecting

The FCI is reached at the robot's own address — `172.16.0.2` in Franka's default setup —
once FCI mode is unlocked in Desk and the brakes are open. The ports are fixed by the
protocol: **1337** for the robot, **1338** for the Franka Hand, and **only one FCI client
may be connected at a time**.

## First program: read the state

```rust,no_run
# extern crate franka;
use franka::{RealtimeConfig, Robot};

fn main() -> franka::FrankaResult<()> {
    let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
    println!("FCI version {:?}, server version {}", robot.fci_version(), robot.server_version());

    let state = robot.read_once()?;
    println!("q      = {:?}", state.q);
    println!("O_T_EE = {:?}", state.O_T_EE);
    println!("mode   = {}", state.robot_mode);
    Ok(())
}
```

`read_once` takes a single state datagram. To stream them, `read` takes a callback and
keeps going until it returns `false`:

```rust,no_run
# extern crate franka;
# use franka::{RealtimeConfig, Robot};
# fn main() -> franka::FrankaResult<()> {
# let robot = Robot::new("172.16.0.2", RealtimeConfig::Enforce)?;
let mut count = 0;
robot.read(|state| {
    println!("{:?}", state.q);
    count += 1;
    count < 100
})?;
# Ok(()) }
```

The repository's `examples/echo_robot_state.rs` is the same thing, ported from libfranka's
example of that name. `examples/communication_test.rs` runs a zero-torque loop and reports
lost states and the `control_command_success_rate` min/avg/max, which is the right first
thing to point at a new PC or NIC.

## The examples

Every example takes the robot's hostname as `argv[1]` and reads `RealtimeConfig` from
`FRANKA_REALTIME`, so the same binary runs against a robot and against the simulator. All
but the last two live in `crates/franka-rs/examples/`:

```sh
cargo run --release --example <name> -- 172.16.0.2
```

| Example | What it does |
|---|---|
| `echo_robot_state` | Prints `RobotState` for 100 cycles. Read-only; the arm does not move. |
| `communication_test` | A zero-torque loop that reports lost states and `control_command_success_rate`. The link/NIC check. |
| `dual_communication_test` | The same loop against two robots at once from one process, with per-robot accounting. |
| `generate_joint_position_motion` | A joint-position motion generator: a cosine ramp on joints 4, 5 and 7. |
| `generate_joint_velocity_motion` | A joint-velocity motion generator: a smooth velocity profile on joints 4-7. |
| `generate_cartesian_pose_motion` | A Cartesian pose motion generator: a circle in the end effector's x/z plane. |
| `generate_cartesian_velocity_motion` | A Cartesian velocity motion generator: a diagonal x/z sweep. |
| `fer_joint_impedance` | 1 kHz joint-impedance torque control against a motion generator, plus the v5-only `set_filters`. |
| `cartesian_impedance_active_control` | A Cartesian impedance controller (the initial pose is the equilibrium) driven through `ActiveControl`'s `read_once` / `write_once`, on both FR3 and FER. Takes `[--duration SEC] [--yes]`. |
| `cartesian_impedance_figure_eight` | The same `ActiveControl` impedance loop with a moving equilibrium: a figure eight around the start pose, ramped in and out, with a nullspace joint spring and a one-sided virtual floor you can press against. Takes `[--duration SEC] [--period SEC] [--amplitude M] [--floor M] [--yes]`. |
| `grasp_object` | The gripper: homes it, then grasps an object of the given width. Takes `<hostname> <object-width>`. Needs a Franka Hand. |
| `nonrealtime_commander` | A scripted (or stdin) non-realtime commander sets jittery, bursty, stalling Cartesian targets through `start_cartesian_target_control`, whose loop on its own thread bridges them with the online trajectory generator and the rate limiter under its own budget (`--bridged`, the default; `--budget V,A,J`), or hands them to a bare `control_cartesian_pose` so the robot's reflex refuses the first step (`--raw`). Takes `[--bridged \| --raw] [--stdin] [--log PATH] [--yes] [--budget V,A,J] [--rotate]`; `--rotate` adds a slow yaw sweep of +-15 degrees through `set_orientation` (bridged only). |
| `automatic_error_recovery` | A command-line `automatic_error_recovery()`: prints the robot mode and error flags before and after clearing a reflex. Read-only apart from the recovery; the arm does not move. |
| `readme_joint_move` | The README's "Quick example", byte for byte; CI runs it against the simulator. |
| `reflex_replay` (in `crates/franka-rerun/examples/`) | The [flight recorder](./flight-recorder.md): a slow joint swing with lowered collision thresholds, recorded live with `Recorder`, and the control log of the reflex a push provokes written as a Rerun recording. `cargo run --release -p franka-rerun --example reflex_replay -- <hostname>`. |
| `commander_live` (in `crates/franka-rerun/examples/`) | `nonrealtime_commander` streamed live into a Rerun viewer: the raw staircase target, the sent and measured position per axis, the arm (with `--meshes DIR`, Franka's link meshes), the derivatives of the sent position against the limits, and the commander's events as they happen. Takes `(--live ADDR \| --out FILE) [--bridged \| --raw] [--stdin] [--budget V,A,J] [--meshes DIR] [--yes]`; start the viewer first (`rerun --port 9876`). Tested on the simulator. |

The four `generate_*` examples and `echo_robot_state` are ports of libfranka's examples of
the same name, so their trajectories can be compared directly.
`cartesian_impedance_active_control` is a port of libfranka's
`cartesian_impedance_control.cpp`, with the callback replaced by the `ActiveControl` loop.
`cartesian_impedance_figure_eight` builds on it and is the one to run with someone standing
next to the robot. Both have run on a real FER through `ActiveControl`.

Once that works, go to [Controlling the robot](./controlling-the-robot.md).
