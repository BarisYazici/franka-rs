# The realtime machine

At the end of this page the PC that runs your program can hold the 1 kHz loop's 1 ms
deadline, and you have checked the link to the robot before the first motion.

## The link to the robot

- A wired Ethernet connection to the robot's control unit. `172.16.0.2` is the address in
  Franka's documented default setup; the examples take it as `argv[1]`.
- FCI mode unlocked in Desk and the brakes open.
- The ports are fixed by the protocol, 1337 for the robot and 1338 for the Franka Hand, and
  only one FCI client may be connected at a time. [How the FCI works](../concepts/fci.md)
  has the rest.

## A `PREEMPT_RT` kernel

The loop has a hard 1 ms deadline. The crate checks `/sys/kernel/realtime`, as libfranka
does:

```sh
cat /sys/kernel/realtime    # 1 on a PREEMPT_RT kernel
```

## Permission for `SCHED_FIFO`

Raising the control thread to `SCHED_FIFO` needs `CAP_SYS_NICE` or a nonzero
`RLIMIT_RTPRIO`. As an unprivileged user:

```sh
ulimit -r 99
```

This works only once an `rtprio` limit is configured for the user, for example
`/etc/security/limits.d/99-realtime.conf` containing `<user> - rtprio 99` (needs
`pam_limits` and a fresh login). Ubuntu's realtime-kernel variant sets up a `realtime` group
that already has `rtprio` and `memlock`.

## `RealtimeConfig`

`Robot::new` takes a [`RealtimeConfig`](../api-reference.md):

- `Enforce`, the default as in libfranka: checks that the calling thread can be raised to
  `SCHED_FIFO` and that the kernel advertises realtime capabilities, and fails with
  `FrankaError::Realtime` and libfranka's message otherwise.
- `Ignore`: skips both checks and runs the loop at whatever priority the process has.

Against the simulator `Ignore` costs nothing. Against a robot it means the loop can miss its
deadline under load, which the robot reports as a falling `control_command_success_rate`
and, eventually, a `communication_constraints_violation` reflex.

Every example reads the choice from `FRANKA_REALTIME` (`enforce`, the default, or
`ignore`), so one binary serves the realtime PC and the simulator:

```sh
FRANKA_REALTIME=enforce cargo run --release --example echo_robot_state -- 172.16.0.2
FRANKA_REALTIME=ignore  cargo run --release --example echo_robot_state -- 127.0.0.1
```

Only the control thread should be `SCHED_FIFO`. Do not start the whole process with
`chrt -f 80`: with other busy threads in the same process (a Rerun recorder, for instance)
that starved the loop. The crate raises the control thread itself; a target control loop
takes `realtime_priority` when a lower priority is wanted next to other realtime threads.
[The realtime rules](../concepts/realtime-rules.md) lists what the loop must and must not do.

## Check the link first

Before any motion, run `communication_test`: after moving the arm to the ready pose it runs
a zero-torque loop and reports how many robot states were lost and the min/avg/max
`control_command_success_rate`, exiting non-zero when the average is below 0.9. It is the
first thing to point at a new PC or network card.

```sh
cargo run --release --example communication_test -- 172.16.0.2
```

A degraded Ethernet cable does not show up in `ping`: on a real FER (2026-09-09) it appeared
as a `communication_constraints_violation` reflex with a clean `ping`, and a packet capture
of the 1 kHz stream is what diagnosed it.

## Details

`RobotImpl::new` raises its own thread's priority unconditionally (libfranka's `Robot::Impl`
does the same) and under `Ignore` only swallows a failure, so on a machine where `ulimit -r`
is nonzero the control thread ends up at `SCHED_FIFO` even with `Ignore`.
