"""The tuning panel: a browser page for a running node's live parameters, `franka-tuning-panel`.

`web` serves `static/` (no build step) and the JSON/SSE routes on loopback; `bridge` talks to
the owners over Zenoh through `zbus`; `validation` checks types only (the owner is the authority
on bounds); `metrics` computes the estimates from published state; `presets` stores the JSON file.

- Arms are discovered through wildcard schema queries; state, `params/current` and the node's
  status topic are subscribed to.
- A batched Apply sends the node update first; a node refusal aborts the teleop update. It is
  not atomic across owners or arms.
- Metrics run on the state's own timestamps, staleness on the bridge's clock. Markers and the
  before/after windows are shared by every tab on one bridge for that arm.
- Undecodable or absent state is shown as such, never as a healthy arm.
- A teleop slider's node bound is an advisory line, not a hard tick.
"""
