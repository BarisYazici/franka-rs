# Wire tools

Five scripts for the 88-byte `VrTargetMsg` the quest bridge publishes. They are repository
tools, not part of the `franka-vr-teleop` wheel; run them from `tools/vr-teleop/` with
`python -m wire_tools.<name>`. They share the package's one codec (`franka_vr/wire.py`) and none
of them needs a robot. Only `record_wire.py` and `synth_wire.py` touch a socket.

| tool | what it does |
|---|---|
| `record_wire.py` | SUB to one or both bridge endpoints and append every message, byte for byte and decoded, to a `.jsonl`. No CONFLATE, so it measures the bridge's real rate. Needs pyzmq. |
| `synth_wire.py` | A synthetic bridge: the real layout at the real rate, with injectable gaps, freezes, SLAM jumps and sign flips. `tests/teleop/make_wire.py` drives its state machine with no socket at all. Needs pyzmq. |
| `analyse_wire.py` | Read a recording and report what a consumer has to handle: freshness episodes, steps, speeds, rotation, the flag transitions. Stdlib only, read-only. |
| `scrub_wire.py` | De-timestamp a recording so it can ship as a fixture. Stdlib only. |
| `make_node_limits.py` | Generate `fixtures/node-limits.json` from franka-node's schema. Stdlib only. |

```sh
python -m wire_tools.record_wire --secs 60 --out run.jsonl          # a real headset
python -m wire_tools.analyse_wire run.jsonl                         # what it says
python -m wire_tools.scrub_wire run.jsonl fixtures/desk-idle.jsonl.gz
python -m wire_tools.make_node_limits --check                       # is the fixture current
```

`fixtures/README.md` describes the two files the tests run on and how to regenerate each.
