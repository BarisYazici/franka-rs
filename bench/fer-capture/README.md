# Franka Emika Robot (FER, FCI v5) state-arrival capture

Offline tooling to find out **which side stalls** when an FER control loop loses robot
states: the robot (or the wire), or the host.

A loop can lose states while every NIC, reassembly and UDP counter stays at zero and never more
than one `message_id` is missing in a row. That pattern points at a *late sender*, not a lost
packet, and timestamping the packets before the client sees them tells the two apart. That is
what this directory does.

Why a capture settles it: tcpdump timestamps a frame when the kernel (or, with
`-j adapter_unsynced`, the NIC itself) receives it, i.e. **before** the client's `recvmsg`. A
client that is late cannot open a gap in those timestamps; it can only open a gap in the
commands it sends back. So:

* a gap in the **state arrival** timestamps -> the robot sent late or the packet sat on the
  wire / in the NIC (`ROBOT_OR_WIRE_STALL`),
* a smooth state stream plus a gap in the **commands** -> the host stalled (`HOST_STALL`).

## Files

| file | what |
|---|---|
| `capture.sh` | starts/stops a background `tcpdump` for one arm, records which timestamp source it got |
| `analyze.py` | pure-stdlib pcap/pcapng parser, IPv4 reassembly, gap tables and the classification |
| `README.md` | this file |

`analyze.py` needs nothing but Python 3 (no scapy, no dpkt). `capture.sh` needs `tcpdump` with
`cap_net_raw`; check with:

```
$ getcap "$(command -v tcpdump)"
/usr/bin/tcpdump cap_net_admin,cap_net_raw=eip
```

If it is missing, grant it once (this is the only step that needs root, and neither script ever
elevates by itself):

```
sudo setcap cap_net_raw,cap_net_admin=eip "$(command -v tcpdump)"
```

## Running it

`<robot-ip>` is the arm's address and `<iface>` the host interface on its link.

```bash
cd <repo>
cargo build --release --example communication_test

# 1. arm the capture (returns as soon as tcpdump is listening)
bench/fer-capture/capture.sh start <robot-ip> <iface> /tmp/fer.pcap

# 2. run the client that loses states
FRANKA_REALTIME=ignore chrt -f 80 target/release/examples/communication_test <robot-ip>

# 3. stop the capture (SIGINT, then wait for tcpdump to flush)
bench/fer-capture/capture.sh stop /tmp/fer.pcap

# 4. analyse
python3 bench/fer-capture/analyze.py /tmp/fer.pcap --robot-ip <robot-ip> --gap-ms 1.5
```

`capture.sh status /tmp/fer.pcap` says whether a capture is still running.

`capture.sh` first tries `-j adapter_unsynced` so the timestamps come from the NIC's own clock
(`ethtool -T <iface>` lists `hardware-receive` if the adapter can do it) and falls back to
kernel timestamps if libpcap rejects it. Which one was used is recorded in
`<out.pcap>.meta`, together with the full `ethtool -T` output - read that before trusting
sub-100 µs numbers. The filter is `udp and host <robot-ip>`, the snaplen is 0 (whole frames,
needed to read `message_id` out of the payload) and the kernel buffer is 64 MiB, which is
several seconds of 1 kHz FCI traffic.

Side files: `<out.pcap>.pid`, `<out.pcap>.log` (tcpdump's own "packets captured / dropped by
kernel" counters - check that "dropped by kernel" is 0), `<out.pcap>.meta`.

## Reading the report

```
python3 bench/fer-capture/analyze.py /tmp/fer.pcap
```

`--robot-ip` is optional: without it the tool picks the side that sends the big datagrams. Other
knobs: `--gap-ms` (gap threshold, default 1.5), `--burst-ms` (burst window after a gap, default
3.0), `--match-ms` (state/command coincidence window, default 2.0), `--jump-ms` (drift jump
threshold, defaults to `--gap-ms`), `--max-rows`, and `--clock-scale` (below).

### `--clock-scale auto|FACTOR` - capture clock rate

**A `-j adapter_unsynced` capture must be analysed with `--clock-scale auto`.** The NIC's
timestamp clock is free-running and disciplined to nothing; a free-running NIC clock can be
far off nominal (about 1.6x has been seen on Intel i219 adapters), which turns the 1 kHz
cadence into a 1.6 ms interval and makes every normal interval cross a 1.5 ms threshold,
inflating the drift range. Nothing is wrong with the capture - only with the units.

The robot's `message_id` is a millisecond clock, so the ratio is measurable from the capture
itself. `auto` fits arrival time against `message_id` with a Theil-Sen median slope over pairs a
twentieth of the capture apart (a plain least-squares fit over the whole span would fold the
robot's own cumulative lateness into the rate), then divides every timestamp by that slope
*before* any statistic, threshold, burst window or drift number is computed. The header prints
the fitted slope and the residual RMS around it:

```
clock scale        : <slope> (auto), fitted <slope> ns of capture clock per robot ms, residual RMS <rms> us over <n> states
```

`--clock-scale 1.6` applies a fixed factor instead. A kernel-timestamped capture (`-j host`, or
no `-j` at all) needs neither: it fits to 1.000 and the option can be left off.

Two more things about hardware timestamps:

* **The two directions are on different clocks.** `-j adapter_unsynced` asks for *receive*
  hardware timestamps; the outgoing commands are still stamped by the kernel. The tool fits each
  direction separately, notices when the two rates disagree, says so in the header, and shifts
  the command timeline by the constant offset between the two clocks (median turnaround = 0) so
  that the state/command coincidence test still means something. On an i219, for example: 1.5999
  for receive, 0.99999 for transmit.
* **Fragment timing is not resolvable.** The i219 gives both fragments of a state the same
  timestamp, so `first->last fragment` is exactly 0 for almost every datagram (the report
  prints that count). Use the software capture for fragment-level questions.

Sections, in order:

* **capture** - packets read, reassembled datagrams, incomplete fragment sets (should be 0) and
  snaplen-truncated datagrams (must be 0, or `message_id` may be unreadable).
* **robot -> host state stream** - how many complete 2373-byte states arrived, how many IP
  fragments each took, the `message_id` range, and how many ids are missing from that range,
  with the longest run of consecutive missing ids.
* **inter-arrival and fragment timing** - p50/p99/p999/max in µs, computed twice: on the arrival
  of each datagram's *first* fragment and on its *last* fragment. The two differ only if
  fragments are being delayed against each other; `first->last fragment` is that delay directly
  (on a 1 GbE link the second fragment of a 2373-byte state follows ~12 µs behind the first).
* **state gaps** - one row per inter-arrival gap at or above `--gap-ms`:
  | column | meaning |
  |---|---|
  | `t_off[s]` | offset from the first state in the capture |
  | `gap_us` / `gap_last_us` | the gap measured on first- and on last-fragment arrival |
  | `id_before` / `id_after` | the `message_id`s on either side of the silence |
  | `consec` | `yes` = the next id followed, so **nothing was lost**, the robot simply sent late; `NO` = an id really never appeared on the wire |
  | `robot_dt_ms` | how far the robot's own clock advanced across the gap (see below) |
  | `next_3ms` | how many datagrams arrived in the 3 ms after the gap; `BURST` means more than a 1 kHz cadence would deliver, i.e. the packets were queued somewhere (wire, NIC, kernel) and drained at once, while `cadence` means the sender itself paused |
* **robot clock minus arrival (drift)** - `message_id` milliseconds minus arrival time, zeroed on
  the first state, with an ASCII trace and a list of jumps. A step *down* at a gap means the
  robot's clock advanced less than the wall clock did, i.e. the robot side stalled; a flat
  offset across a gap means the datagram was simply lost or held up somewhere neutral.
* **host -> robot command stream** - the same for the 370-byte commands, plus, per gap, whether
  it coincides (± `--match-ms`) with a state gap. Coinciding = the client was blocked waiting
  for a state it had not received. Not coinciding = the client itself was late.
* **CLASSIFICATION** - one of:
  * `ROBOT_OR_WIRE_STALL` - the gaps are already in the arrival timestamps. The robot emitted
    late (or the packet was held on the wire); the host is exonerated, because tcpdump stamped
    the packet before the client could touch it.
  * `HOST_STALL` - the state stream kept its cadence at the capture point while the client
    stopped answering. Look at scheduling, page faults, or a blocked control thread.
  * `MIXED` - both happen in the same capture. The verdict always carries the counts and which
    way they lean, e.g. `MIXED (70 transit gaps vs 1 host-only gap: predominantly ROBOT_OR_WIRE)`
    - one host-side event in a capture full of transit gaps is a very different finding from an
    even split.
  * `NO_STALL` - no gap above `--gap-ms` in either direction; raise the run time or lower the
    threshold.

A late sender reads `ROBOT_OR_WIRE_STALL` with `consec = yes` and `robot_dt_ms = 1` on every
row: the robot's millisecond clock advanced by one while the wire stayed quiet, which no
host-side delay can produce.

### The v5 wire fields this reads

From `crates/franka-rs/src/wire/robot/v5/rbk_types.rs` (little endian, `#[repr(C, packed)]`):

* `RobotState` (2373 B payload, `rbk_types.rs:123-125`): `message_id: u64` at **offset 0**.
  FCI v5 has **no separate `time` field on the wire** - `message_id` *is* the robot's
  millisecond clock (a monotonic 1 kHz cycle counter that the client converts straight into
  `RobotState::time` with `Duration::from_millis(w.message_id.get())`,
  `crates/franka-rs/src/robot_state.rs:410`). So "robot time" in the report is the message id
  read as milliseconds, and `robot_dt_ms` across a gap equals the id delta.
* `RobotCommand` (370 B payload, `rbk_types.rs:231-233`): `message_id: u64` at **offset 0**.

At an MTU of 1500 a state is two IP fragments (1480 + 901 bytes of IP payload); the tool
reassembles them by `(src, dst, ip_id, proto)` and keeps both the first and the last fragment's
timestamp. Unfragmented datagrams (loopback, jumbo frames) are handled too.

## Tests

```
python3 bench/fer-capture/analyze.py --self-test
shellcheck bench/fer-capture/capture.sh
```

The self-test builds synthetic 1 kHz captures in a temp directory - fragmented states plus
commands - with four injected pathologies and asserts the whole gap table and the
classification:

| case | injected | expected in the report |
|---|---|---|
| a | a 7 ms sender pause, ids stay consecutive | one 7000 µs state gap, `consec yes`, `robot_dt_ms 1`, followed by normal cadence, drift jump -6 ms |
| b | a 4 ms delay followed by a drain burst | one 4000 µs gap with 5 datagrams in the next 3 ms, flagged `BURST` |
| c | one id genuinely never sent | one 2000 µs gap with `consec NO`, 1 missing id, max consecutive 1 |
| d | the client skips 5 commands with the states flowing | one 6000 µs command gap that matches no state gap |
| f | the same frames replayed on a 1.6x capture clock | `--clock-scale auto` fits 1.600000, and the gap table, burst flags, drift range and verdict come back identical to the unscaled run |
| f' | only the receive direction on that fast clock, 5 s offset | the two clocks are fitted separately and the one host-only command gap is still found |

Scenario A (all four) must classify as `MIXED`, a/b alone as `ROBOT_OR_WIRE_STALL`, d alone as
`HOST_STALL`. The same frames are written as microsecond pcap, nanosecond pcap, big-endian
pcap and pcapng (with `if_tsresol` = ns) and all four must produce a byte-identical report;
`tcpdump -r` is run over two of them as a cross-check that the synthetic files are real capture
files. A fifth scenario replays everything unfragmented over a `DLT_NULL` loopback link type,
where source and destination address are equal and the two directions are told apart by the
robot's UDP source port.

### End-to-end check against the FER simulator

The whole pipeline also runs end to end without a robot, over loopback: `franka-sim:panda-v5`
(`--protocol v5 --robot panda --no-gripper`, `PANDA_MJCF=/opt/mujoco_menagerie/franka_emika_panda/panda_nohand.xml`)
on `--network host` under `flock .sim.lock`, `capture.sh start 127.0.0.1 lo ...`,
`FRANKA_REALTIME=ignore target/release/examples/communication_test 127.0.0.1`, `capture.sh stop`,
then `analyze.py`. 22942 packets, 11380 states, 0 dropped by the kernel, one missing id, and a
handful of ~1.7 ms state gaps with consecutive ids (the simulator itself being late). `lo` has no
hardware clock, so `-j adapter_unsynced` is refused there and the meta file says so:

```
timestamp mode : host (software, adapter_unsynced refused)
```

On loopback the states are unfragmented (MTU 65536) and both addresses are `127.0.0.1`, so that
run also covers the unfragmented path and the port-based direction split. Note that
`udp and host 127.0.0.1` also catches the host's DNS traffic; the analyzer keeps only the busiest
FCI flow and counts the rest under "datagrams outside the flow".
