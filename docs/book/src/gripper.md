# Gripper

The Franka Hand speaks its own protocol on **TCP port 1338** of the same host as the robot
FCI — a separate connection from the arm's, with its own version handshake and its own UDP
state stream. `franka::Gripper` is the port of `franka::Gripper`, and the protocol
(**version 3**) is **byte-identical on FCI v5 and v10**: the same `Gripper` code drives a
Franka Emika Robot (FER)'s hand and an FR3's.

```rust,no_run
# extern crate franka;
use franka::Gripper;

# fn main() -> franka::FrankaResult<()> {
let gripper = Gripper::new("172.16.0.2")?;
println!("gripper server version {}", gripper.server_version());

gripper.homing()?;                       // estimates max_width; needed after changing fingers

let state = gripper.read_once()?;
println!("width {} of {}, {} C", state.width, state.max_width, state.temperature);

gripper.move_gripper(0.08, 0.1)?;        // width [m], speed [m/s]
let grasped = gripper.grasp(0.03, 0.1, 40.0, 0.005, 0.005)?;   // width, speed, force [N], eps
if grasped {
    println!("holding it");
}
gripper.stop()?;
# Ok(()) }
```

`Gripper::new` takes the robot's address as `"host"` or `"host:port"`; port 1338 is used
unless a port is embedded. It fails with `FrankaError::Network` if the connection cannot be
made and `FrankaError::IncompatibleVersion` if the server does not support the crate's
gripper protocol version.

## The commands

| method | what it does |
|---|---|
| `homing()` | Homes the gripper and estimates the maximum grasping width. Required after changing the fingers. |
| `move_gripper(width, speed)` | Moves the fingers to `width` metres at `speed` m/s. Named `move_gripper` because `move` is a Rust keyword. |
| `grasp(width, speed, force, epsilon_inner, epsilon_outer)` | Grasps at `force` newtons. An object counts as grasped when the finger distance `d` satisfies `(width - epsilon_inner) < d < (width + epsilon_outer)`. libfranka's C++ overload defaults both epsilons to `0.005`; Rust has no default arguments, so both are explicit here. |
| `stop()` | Stops a running move or grasp. |
| `read_once()` | Drains stale datagrams, then blocks for one fresh `GripperState`. |

All four commands return `FrankaResult<bool>` — `false` means the gripper reports the
command did not achieve its goal (a grasp that closed on nothing), while an error means the
command failed or was aborted or the connection was lost.

## `GripperState`

| field | meaning |
|---|---|
| `width: f64` | Current opening width, in metres. |
| `max_width: f64` | Maximum opening width, in metres — estimated by `homing()`. |
| `is_grasped: bool` | Whether an object is currently grasped. |
| `temperature: u16` | Gripper temperature, in degrees Celsius. |
| `time: Duration` | Strictly monotonic timestamp since the gripper server started. |

The wire struct is 23 bytes, and its size and every field offset are asserted against
libfranka's on both FCI versions.

## Threading

`Gripper`'s network layer is `Sync`, matching libfranka's documented guarantee for
`franka::Gripper`, so the struct's members are threadsafe. The one caveat: only one
`read_once()` should be in flight at a time, since concurrent callers would race to claim
the same UDP datagram.

## Notes from the field

- **An FER's hand may not be on the FCI at all.** On the two FERs used for the hardware
  validation the Franka Hands were wired to direct CAN rather than libfranka's TCP gripper
  protocol, and port 1338 was closed. Check before assuming `Gripper::new` will connect.
- **`examples/grasp_object.rs`** is the port of libfranka's example of that name, and is
  the quickest way to check a hand end to end.
- Against `franka-sim`, a *successful* grasp needs an object between the fingers — the FR3
  image takes `--gripper-object-width`; see [Testing with
  franka-sim](./testing-with-franka-sim.md).
