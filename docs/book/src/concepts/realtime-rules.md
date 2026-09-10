# The realtime rules

A control loop against the FCI answers a state every millisecond, and the robot ends the
motion when the answers stop coming. These are the rules that keep them coming, each with
what happens when it is broken. They apply in full to the callback and `ActiveControl`
paths; under target control the crate's thread follows them for you, and only the observer
you may install is bound by them.

## 1. Answer every state within the millisecond

The robot publishes `control_command_success_rate` in every state: the fraction of the last
100 commands it accepted. Cycle 0 reads 0, because nothing has been acknowledged yet, so skip
it when averaging. A rate below 1 means commands arrived late or not at all; a sustained fall
ends the motion with a `communication_constraints_violation` reflex.
`examples/communication_test.rs` runs a zero-torque loop and prints the minimum, average and
maximum of the rate; it is the first thing to run on a new machine or network interface.

## 2. Build with `--release`

A debug build of a stiff torque controller misses cycles. Every command in this book uses
`--release`.

## 3. Nothing on the control thread may allocate, block or do I/O

Inside a callback, an `ActiveControl` loop body or a target-control observer: no `Vec` that
grows, no `String` formatting, no `println!`, no file or socket, no mutex another thread may
be holding. The crate's own loops allocate nothing after the start.
`franka_rerun::Recorder::push` shows the pattern: it copies the record into a bounded channel
created before the loop, with `try_send`, which never blocks and never allocates, and drops
the record when the channel is full. A slow cycle costs success rate; a blocked one costs the
motion.

## 4. Only the control thread is `SCHED_FIFO`

`Robot::new` raises the calling thread to the highest `SCHED_FIFO` priority; target control
raises its own loop thread the same way, or to `realtime_priority` when set. Leave the rest
of the process at normal priority. Running the whole process under `chrt -f` while it also
had Rerun threads starved the control loop; do not do that to a process with other threads.

## 5. Seed a motion from the commanded values, not the measured ones

Start a joint motion from `state.q_d` and a Cartesian one from `state.O_T_EE_c`, not from
`q` or `O_T_EE`. Commanded and measured differ by the tracking error, and a first setpoint
built on the measured value is a step of that size. On an FR3 the first command of a motion
is its own filter and rate-limiter reference, so the step reaches the robot as it is; on an
FER the first setpoint is rate limited against the robot's own `q_d` / `O_T_EE_c`, and the
limiter absorbs the step and distorts the start of the motion. Target control anchors on the
echo for you. Details in [FER / Panda specifics](../reference/fer.md).

## 6. Count lost states with `state.time`

`RobotState::time` is the robot's millisecond counter, not a host clock. Two consecutive
states whose `time` differs by more than 1 ms mean states were lost or discarded in between;
the `Duration` a callback receives is that difference, so integrate against it rather than
assuming 1 ms. Not every gap is yours: on an FER the control box's own transmit path stalls
for a few milliseconds a few times per second. Packet captures at the network card show
every state present and arriving in a burst after the stall; the client keeps the newest and
discards the rest, and that is the gap it sees. Measured in one campaign, not a
specification; see [Benchmarks and hardware validation](../reference/benchmarks.md).

## 7. `RealtimeConfig::Ignore` is for the simulator

`RealtimeConfig::Enforce`, the default, requires `/sys/kernel/realtime` and the right to
`SCHED_FIFO`, and fails with `FrankaError::Realtime` otherwise. `Ignore` skips both checks; it
still raises the priority when it can and only swallows the failure. Against franka-sim on an
ordinary kernel `Ignore` is right. Against a robot it means the loop runs at whatever
priority it got, and rule 1 decides how long the motion lasts.

## What a `PREEMPT_RT` kernel does and does not buy

It bounds the time between a datagram arriving and your `SCHED_FIFO` thread running, and it
lets that thread pre-empt almost everything else on the machine. It does not make a slow
callback fast, it does not shorten a `println!`, and it does not repair a bad network path:
in the FER benchmark a USB Ethernet adapter with 15 ms of interrupt coalescing lost about
three times as many cycles as the onboard card, for both clients. The benchmarks themselves
ran on a non-realtime kernel, which is why their tails are in the milliseconds.
