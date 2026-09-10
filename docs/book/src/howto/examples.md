# Run the examples

Every example takes the robot's hostname as `argv[1]` and reads `RealtimeConfig` from
`FRANKA_REALTIME`: `enforce`, the default, or `ignore` for the
[simulator](../getting-started/simulator.md).

```sh
cargo run --release --example <name> -- 172.16.0.2
FRANKA_REALTIME=ignore cargo run --release --example <name> -- 127.0.0.1
```

The examples that move the arm print a warning and wait for Enter (`--yes` skips it where
offered). Have the user stop button at hand and free space around the arm.

## State and link

| example | what it does |
|---|---|
| `echo_robot_state` | Prints `RobotState` for 100 cycles. The arm does not move. |
| `communication_test` | Moves to the ready pose, then runs a zero-torque loop and reports lost states and the min/avg/max `control_command_success_rate`; exits non-zero when the average is below 0.9. |
| `dual_communication_test` | The same loop against two robots from one process, a `Robot` per thread, with per-robot accounting. Takes two hostnames, `[--cycles N] [--pin CPU1,CPU2]`. |

## Motion generators

| example | what it does |
|---|---|
| `generate_joint_position_motion` | A cosine ramp on joints 4, 5 and 7. |
| `generate_joint_velocity_motion` | A velocity profile on joints 4 to 7. |
| `generate_cartesian_pose_motion` | A circle in the end effector's x/z plane. |
| `generate_cartesian_velocity_motion` | A diagonal x/z sweep. |
| `readme_joint_move` | The README's quick example, byte for byte; CI runs it against the simulator. |
| `move_to_ready` | Moves to libfranka's "ready" joint configuration. Takes `[speed-factor] [--yes]`, default 0.2. |

## Torque and impedance

| example | what it does |
|---|---|
| `fer_joint_impedance` | 1 kHz joint-impedance torque control on an FER, rate-limited by hand against the robot's `tau_J_d`; refuses to run on an FR3. |
| `cartesian_impedance_active_control` | A Cartesian impedance controller whose equilibrium is the start pose, driven through `ActiveControl`'s `read_once` / `write_once`. Takes `[--duration SEC] [--yes]`. |
| `cartesian_impedance_figure_eight` | The same loop with a moving equilibrium, a figure eight around the start pose, ramped in and out, with a nullspace joint spring and a one-sided virtual floor. Takes `[--duration SEC] [--period SEC] [--amplitude M] [--floor M] [--yes]`. |

## Target control

| example | what it does |
|---|---|
| `nonrealtime_commander` | A scripted (or stdin) commander sets stepped, bursty, stalling Cartesian targets through `start_cartesian_target_control` (`--bridged`, the default; `--budget V,A,J`; `--rotate` adds a yaw sweep), or hands them to a bare `control_cartesian_pose` so the robot refuses the first step (`--raw`). Takes `[--bridged \| --raw] [--stdin] [--log PATH] [--yes] [--budget V,A,J] [--rotate]`. |

## Gripper and recovery

| example | what it does |
|---|---|
| `grasp_object` | Homes the Franka Hand, then grasps an object of the given width. Takes `<hostname> <object-width>`. |
| `automatic_error_recovery` | A command-line `automatic_error_recovery()`: prints the robot mode and error flags before and after clearing a reflex. |

## Rerun examples (`crates/franka-rerun/examples`)

Run with `cargo run --release -p franka-rerun --example <name> -- <hostname>`; the viewer
must be 0.37.1 ([Install](../getting-started/install.md)).

| example | what it does |
|---|---|
| `reflex_replay` | A slow joint swing with lowered collision thresholds, recorded live with `Recorder`; the control log of the reflex a push provokes is written as a Rerun recording. See [Record and replay a run](./flight-recorder.md). |
| `commander_live` | `nonrealtime_commander` streamed live into a viewer: the raw target, the sent and measured position per axis, the arm (with `--meshes DIR`), the derivatives against the limits, the commander's events. Takes `(--live ADDR \| --out FILE) [--bridged \| --raw] [--stdin] [--budget V,A,J] [--controller joint\|cartesian] [--meshes DIR] [--yes]`; start the viewer first (`rerun --port 9876`). |

`echo_robot_state`, `communication_test`, the four `generate_*` examples and `grasp_object`
are ports of libfranka's examples of the same name (`grasp_object` always homes and takes
no `<homing>` flag); `cartesian_impedance_active_control` is libfranka's
`cartesian_impedance_control.cpp` with the callback replaced by the `ActiveControl` loop.
The Python examples are described in [From Python](../getting-started/python.md).
