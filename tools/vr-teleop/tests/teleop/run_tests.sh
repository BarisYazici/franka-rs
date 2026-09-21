#!/usr/bin/env bash
# Every rule the teleop client implements, proved with no hardware, no node, no headset and no
# port.
#
#   tests/teleop/run_tests.sh [WORKDIR]     # default /tmp/vr-teleop-tests
#
# Nothing here binds or connects a TCP socket: the wires are files, the arm is the client's own
# stand-in, selftest.py drives the hand check's listener over inproc:// only, the bridge's
# 5560/5570 are never touched, and franka-sim's ports 1337/1338 and its lock are untouched.
#
# Everything it needs is in the tree: `fixtures/desk-idle.jsonl.gz` is the recorded wire
# (60 s of a headset awake on a desk, not worn, one controller absent), the synthetic ones
# are built by make_wire.py from synth_wire.py's own bridge state machine, and the node's
# limits come from `fixtures/node-limits.json`, generated from franka-node's schema.
# It runs from tools/vr-teleop/, so `franka_vr`, `wire_tools` and `tests` import from the
# checkout; the interpreter needs the package's dependencies (pip install -e tools/vr-teleop).
# PYTHON=... picks the interpreter.
set -u

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORK="${1:-/tmp/vr-teleop-tests}"
PY="${PYTHON:-python3}"
FIXTURES="$ROOT/fixtures"
REC_GZ="$FIXTURES/desk-idle.jsonl.gz"
LIMITS="$FIXTURES/node-limits.json"
PASS=0
FAIL=0

say() { printf '\n=== %s ===\n' "$*"; }
ok()  { printf '  PASS  %s\n' "$1"; PASS=$((PASS + 1)); }
no()  { printf '  FAIL  %s\n' "$1"; FAIL=$((FAIL + 1)); }

# want <name> <expression over the summary/emit facts in $F>
# The facts are extracted by facts.py into shell-sourceable variables.
want() {
  local name=$1; shift
  if "$PY" -c "import sys; sys.exit(0 if ($*) else 1)" 2>/dev/null; then ok "$name";
  else no "$name  [$*]"; fi
}

rm -rf "$WORK"; mkdir -p "$WORK"; WORK="$(cd "$WORK" && pwd)"
cd "$ROOT" || exit 1
[ -r "$REC_GZ" ] || { echo "missing the recorded wire: $REC_GZ"; exit 1; }
[ -r "$LIMITS" ] || { echo "missing the node limits fixture: $LIMITS"; exit 1; }

# The fixture ships gzipped. ReplaySource reads .gz directly once it can; until then (and on
# any interpreter whose gzip is unhappy) the same file is expanded into $WORK. Either way the
# runs below see one path.
REC="$REC_GZ"
if ! "$PY" -c '
import sys
from franka_vr.teleop import sources
sources.ReplaySource(sys.argv[1], "tcp://127.0.0.1:5570")' "$REC_GZ" 2>/dev/null; then
  "$PY" -c '
import gzip, shutil, sys
with gzip.open(sys.argv[1], "rb") as src, open(sys.argv[2], "wb") as dst:
    shutil.copyfileobj(src, dst)' "$REC_GZ" "$WORK/desk-idle.jsonl" \
    || { echo "cannot expand $REC_GZ"; exit 1; }
  REC="$WORK/desk-idle.jsonl"
  echo "note: the replay source does not read .gz yet; expanded into $REC"
fi

# The node's own limits, for T6c: read from the fixture, never typed here. `*_bad` is a value
# the node's numbers make illegal, `*_ok` the largest legal one.
eval "$("$PY" - "$LIMITS" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
der, par = d["schema"]["derived"], d["schema"]["params"]
stale = par["budget"]["default"][0] / der["state_hz"]              # one state tick of drift
stale_rot = par["rotation_budget"]["default"][0] / der["state_hz"]
print(f"n_clamp_bad={der['max_lead']:.6f}")
print(f"n_clamp_ok={der['max_lead'] - stale:.6f}")
print(f"n_clamp_rot_bad={der['max_lead_rotation']:.6f}")
print(f"n_clamp_rot_ok={der['max_lead_rotation'] - stale_rot:.6f}")
print(f"n_rate_bad={der['rate_hz']:.6f}")
print(f"n_rate_ok={der['rate_hz'] * 0.9:.6f}")                     # limits.KEEPALIVE_SLACK
print(f"n_max_step_rot_bad={der['max_step_rotation'] * 1.01:.6f}")
print(f"n_timeout_bad={der['stop_after_ms'] * 1.5:.6f}")
PY
)"
: "${n_clamp_ok:?cannot read the node limits from $LIMITS}"

# One run of the whole pipeline, and its facts as shell variables.
#   run <tag> <wire> [extra teleop args...]
run() {
  local tag=$1 wire=$2; shift 2
  unset $(compgen -v r_ 2>/dev/null) 2>/dev/null || true
  "$PY" -m franka_vr.teleop.cli --dry-run --quiet --print-every 0 --replay "$wire" \
      --emit "$WORK/$tag.emit.jsonl" --summary-json "$WORK/$tag.summary.json" \
      "$@" > "$WORK/$tag.log" 2>&1
  if [ ! -s "$WORK/$tag.summary.json" ]; then
    no "$tag: teleop.py produced no summary -- the run failed"
    sed -n '1,25p' "$WORK/$tag.log"
    exit 1
  fi
  eval "$("$PY" -m tests.teleop.facts "$WORK/$tag.summary.json" "$WORK/$tag.emit.jsonl")"
}

# ---------------------------------------------------------------- the wires
say "W  synthetic wires (make_wire.py -> synth_wire.py's own bridge state machine)"
"$PY" -m tests.teleop.make_wire --out "$WORK/clean.jsonl"  --secs 24
"$PY" -m tests.teleop.make_wire --out "$WORK/gaps.jsonl"   --secs 24 --gap-every 5 --gap-ms 120
"$PY" -m tests.teleop.make_wire --out "$WORK/freeze.jsonl" --secs 24 --freeze-at 9 --freeze-ms 600
"$PY" -m tests.teleop.make_wire --out "$WORK/jump.jsonl"   --secs 24 --hand-jump-at 11.02 \
      --hand-jump-m 0.0777
"$PY" -m tests.teleop.make_wire --out "$WORK/flip.jsonl"   --secs 24 --flip-engaged

# ---------------------------------------------------------------- T0 unit
say "T0  the arithmetic, in process"
"$PY" -m tests.teleop.selftest && ok "selftest.py (clutch algebra, sign, wire layout, timeout, hands, dq release)" \
  || no "selftest.py"
"$PY" -m tests.teleop.selftest_recovery && \
  ok "selftest_recovery.py (auto-recovery, recover timeout, dq resume, gripper path)" \
  || no "selftest_recovery.py"

# ---------------------------------------------------------------- rule 1
say "T1  RULE 1  gate on fresh, and keep the session alive while not fresh"
run r1 "$REC" --arm R
# 938 of 3000 recorded messages are not fresh (31.3 %, the measured figure).
want "recorded wire: 938 not-fresh samples gated"      "$r_not_fresh == 938"
want "not one of them followed the controller"         "$r_driven == 0"
want "every one of them kept the session alive"        "$r_out >= 2900"
want "worst gap between targets under the node's 200 ms hold" "$r_gap_ms < 200.0"
want "and under 2000 ms, so stop() never fires"        "$r_gap_ms < 2000.0"
run r1b "$WORK/gaps.jsonl"
# four 120 ms gaps in 24 s, six 20 ms ticks each
want "synthetic 120 ms gaps: all 24 ticks gated"       "$r_not_fresh >= 20"
want "synthetic 120 ms gaps: session still fed"        "$r_gap_ms < 200.0"
want "and each gap re-anchored instead of publishing"  "$r_reanchor >= 4"
run r1c "$WORK/freeze.jsonl"
want "a 600 ms stale-but-publishing freeze: gated"     "$r_not_fresh >= 29"
want "a 600 ms freeze: session still fed"              "$r_gap_ms < 200.0"
want "a 600 ms freeze: not followed (arm held)"        "$r_step_mm < 6.0"

# ---------------------------------------------------------------- rule 2
say "T2  RULE 2  clutch on the grip: drive while held, hold while released, never jump"
run r2 "$WORK/clean.jsonl"
want "three grip cycles became three engagements"      "$r_engage == 3"
want "released samples were held, not followed"        "$r_released > 300"
want "no re-engage jump: worst first-target lead under 1 mm" "$r_reengage_lead_mm < 1.0"
want "no clamp ever had to save it (lead)"             "$r_clamp_lead == 0"
want "no clamp ever had to save it (step)"             "$r_clamp_step == 0"
want "worst step between targets far inside the node's 50 mm" "$r_step_mm < 10.0"
want "worst lead far inside the node's max_lead 50 mm" "$r_lead_mm < 10.0"

# ---------------------------------------------------------------- rule 3
say "T3  RULE 3  never publish the first fresh sample after a gap"
run r3 "$REC" --arm R
# +41.600 s: 77.7 mm across a not-fresh -> fresh edge. +41.660 s: 77.7 mm back, fresh -> fresh.
want "the recorded 77.7 mm hand step was seen"         "$r_hand_step_mm > 77.0"
want "the fresh edge re-anchored instead of publishing" "$r_reanchor >= 12"
want "its fresh->fresh twin re-anchored too"           "$r_hand_jump == 1"
want "so no step over 1 mm ever reached the arm"       "$r_step_mm < 1.0"
run r3b "$WORK/jump.jsonl"
want "injected 77.7 mm mid-drive jump: re-anchored"    "$r_hand_jump >= 1"
want "injected jump: nothing over 10 mm published"     "$r_step_mm < 10.0"
want "injected jump: no lead clamp needed"             "$r_clamp_lead == 0"
run r3c "$WORK/jump.jsonl" --max-hand-step 10.0
want "CONTROL, gate off: the jump reaches the clamp"   "$r_clamp_lead > 0"
want "CONTROL, gate off: and commands the full 25 mm"  "$r_lead_mm > 24.0"

# ---------------------------------------------------------------- rule 4
say "T4  RULE 4  canonicalise the quaternion sign"
run r4 "$WORK/flip.jsonl"
want "a wire that flips sign every engaged tick"       "$r_engage >= 2"
want "publishes a sign-continuous stream"              "$r_sign_flips == 0"
# the synthetic wrist rolls at 0.5 rad/s, i.e. 0.010 rad per tick at 1:1
want "with no rotation step over one tick of the roll" "$r_step_rot < 0.0101"
run r4d "$WORK/flip.jsonl" --rotation-scale 1.0
want "at 1:1 a tick of the roll arrives whole"         "0.0099 < $r_step_rot < 0.0101"
run r4e "$WORK/flip.jsonl" --rotation-scale 0.25
want "at 0.25 it arrives quartered"                    "0.0024 < $r_step_rot < 0.0026"
run r4b "$WORK/flip.jsonl" --no-canonicalise --rotation-scale 1.0
want "CONTROL, rule 4 off at 1:1: flips go straight out" "$r_sign_flips > 300"
run r4f "$WORK/flip.jsonl" --no-canonicalise
want "and a scaled delta fixes the sign on its own too"  "$r_sign_flips == 0"
run r4c "$REC" --arm R
want "the recorded wire's 26 % flips, canonicalised"   "$r_sign_flips == 0"

# ---------------------------------------------------------------- rule 5
say "T5  RULE 5  our own seq and t_send_ns"
run r5 "$REC" --arm R --client-id 4242
want "seq starts at 1, whatever the wire's counter"    "$r_seq_first == 1"
want "and is strictly increasing with no gaps"         "$r_seq_ok"
want "the wire's own seq (2531..5530) is never reused" "$r_seq_last == $r_out"
want "t_send_ns is strictly increasing"                "$r_stamp_ok"
want "every target carries our --client-id"            "$r_client_id == 4242"
want "every target is 80 bytes, version 1, kind 1"     "$r_layout_ok"
want "flags are 0: the anchor bit stays off by default" "$r_flags_max == 0"

# ---------------------------------------------------------------- the extras
say "T6  the clamp, the gripper and the workspace"
run r6 "$WORK/clean.jsonl" --spatial-scale 2.0 --clamp 0.005
want "scale 2.0 on a 5 mm leash outruns the arm"       "$r_clamp_lead > 0"
want "and the clamp holds every target inside it"      "$r_lead_mm <= 5.01"
want "and every step inside --max-step"                "$r_step_mm <= 40.01"
want "so nothing the node would refuse is ever sent"   "$r_lead_mm < 50.0 and $r_step_mm < 50.0"
run r6b "$WORK/clean.jsonl"
want "gripper: the trigger produced commands"          "$r_gripper > 10"
want "gripper: never faster than --gripper-hz 5"       "$r_gripper_min_ms >= 199.0"
want "gripper: widths inside 0..max_width"             "$r_gripper_width_ok"
want "gripper: no fling open on the first sample"      "$r_gripper_first_ms > 100.0"
want "gripper: its seq strictly increasing by one"      "$r_gripper_seq_ok"
run r6c "$WORK/clean.jsonl" --no-gripper
want "--no-gripper sends none"                         "$r_gripper == 0"
want "and with no --workspace there is no box to leave" "'$r_workspace' == 'none' and $r_in_box"
run r6d "$WORK/clean.jsonl" --workspace 0.25,-0.1,0.4,0.45,0.1,0.6
want "a tight workspace box clamps instead of leaving it" "$r_clamp_workspace > 0"
want "and every target stays in the box"               "'$r_workspace' == 'box' and $r_in_box"

# T6b measured --rotation-scale against 20 s of engaged hands. That recording was never
# publishable and is gone; its numbers are in docs/design.md as a measurement, not a test.

say "T6c  the node's limits refuse at startup, not mid-run  (fixtures/node-limits.json)"
for bad in "--clamp-rot $n_clamp_rot_bad" "--clamp $n_clamp_bad" "--rate $n_rate_bad" \
           "--arm cam" "--episode bad/name" \
           "--source-timeout-ms $n_timeout_bad" "--max-step-rot $n_max_step_rot_bad" \
           "--client-id 0"; do
  # shellcheck disable=SC2086
  if "$PY" -m franka_vr.teleop.cli --dry-run --replay "$WORK/clean.jsonl" $bad >/dev/null 2>&1; then
    no "$bad was accepted"
  else ok "refused at startup: $bad"; fi
done
if "$PY" -m franka_vr.teleop.cli --dry-run --quiet --print-every 0 --replay "$WORK/clean.jsonl" \
      --clamp "$n_clamp_ok" --clamp-rot "$n_clamp_rot_ok" --rate "$n_rate_ok" \
      >/dev/null 2>&1; then
  ok "and the value just inside every node limit is accepted"
else no "a value inside the node's limits was refused"; fi
if "$PY" -m franka_vr.teleop.cli --dry-run --quiet --print-every 0 --replay "$WORK/clean.jsonl" \
      --no-auto-recover --max-recoveries 0 --dq-latch >/dev/null 2>&1; then
  ok "the recovery and dq flags are accepted in a dry run"
else no "the recovery and dq flags were refused"; fi

say "T7  --dry-run touches no Zenoh"
if grep -qiE 'zenoh' "$WORK"/r1.log; then no "a dry-run log mentions zenoh"; else
  ok "no Zenoh in a --dry-run log"; fi
# franka-node-client imports the zenoh module, so the check is that no session is opened.
REC="$REC" "$PY" - <<'PY' && ok "--dry-run never opens a Zenoh session" \
  || no "--dry-run opened a Zenoh session"
import os, sys
import zenoh
def _refuse(*_a, **_k):
    raise SystemExit("zenoh.open was called")
zenoh.open = _refuse
from franka_vr.teleop import cli
sys.exit(cli.main(["--dry-run", "--quiet", "--print-every", "0", "--arm", "R",
                   "--replay", os.environ["REC"]]))
PY

say "T8  the hand is the port: pairing, startup line, summary"
run r8 "$WORK/clean.jsonl"
want "default --arm L drives from the LEFT controller" "'$r_hand' == 'left' and $r_port == 5560"
want "and the pairing is not crossed"                  "not $r_crossed"
if [ "$(head -n 1 "$WORK/r8.log")" = "arm L <- LEFT controller (tcp://127.0.0.1:5560)" ]; then
  ok "the first line printed is the pairing"; else no "the first line printed is the pairing"; fi
if grep -qF "vr-teleop summary   arm L <- LEFT controller (tcp://127.0.0.1:5560)" "$WORK/r8.log"
  then ok "the exit summary names the hand and the port"
  else no "the exit summary names the hand and the port"; fi
run r8b "$REC" --arm R
want "--arm R drives from the RIGHT controller"        "'$r_hand' == 'right' and $r_port == 5570"
for bad in "--hand right" "--arm R --hand left" "--endpoint tcp://127.0.0.1:5570" \
           "--endpoint tcp://127.0.0.1:5580 --cross" \
           "--hand left --endpoint tcp://127.0.0.1:5570 --cross" "--arm arm3"; do
  # shellcheck disable=SC2086
  if "$PY" -m franka_vr.teleop.cli --dry-run --replay "$REC" $bad >/dev/null 2>&1; then
    no "$bad was accepted"
  else ok "refused at startup: $bad"; fi
done
if "$PY" -m franka_vr.teleop.cli --dry-run --replay "$WORK/clean.jsonl" --arm R >/dev/null 2>&1; then
  no "a replay of the left wire ran as arm R"
else ok "a replay holding only the other hand's port is refused, not relabelled"; fi
run r8c "$REC" --hand right --cross
want "--cross runs arm L from the RIGHT controller"    "'$r_hand' == 'right' and $r_crossed"
if head -n 1 "$WORK/r8c.log" | grep -qF "arm L <- RIGHT controller (tcp://127.0.0.1:5570)  CROSSED (--cross)"
  then ok "and its pairing line says CROSSED"; else no "and its pairing line says CROSSED"; fi

printf '\n%s\n' "----------------------------------------------------------------"
printf 'vr-teleop: %d passed, %d failed   (work in %s)\n' "$PASS" "$FAIL" "$WORK"
[ "$FAIL" -eq 0 ] || exit 1
