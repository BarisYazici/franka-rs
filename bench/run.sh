#!/usr/bin/env bash
# Runs the whole franka-rs vs C++ libfranka jitter/latency matrix against one franka-sim
# container and writes bench/results/<timestamp>/{*.json,*.simlog,meta.json,summary.md}.
#
# The matrix is  2 clients (cpp, rust)  x  the selected variants
#                x the conditions each variant supports  x  REPS repetitions.
#
# Variants:
#   control  joint-velocity motion through the library-owned control loop
#   active   joint-velocity motion through readOnce/writeOnce
#   model    an operational-space impedance controller that evaluates the robot model
#            (mass, coriolis, gravity, zeroJacobian, pose) inside every 1 kHz cycle and
#            sends torques.  Conditions plain and rt only -- the hog cell is skipped
#            because this variant is about compute cost, not contention.
#
# Fairness protocol (a fixed cpp-then-rust order would confound client identity with any
# drift over the run):
#   * a cell is one (condition, variant, rep); both clients run back to back inside it,
#   * the order alternates per repetition -- odd reps run cpp first, even reps rust first,
#     and each run records `order_in_cell` and `cell_first_client` in its JSON,
#   * the simulator container is restarted at the start of every cell, so both clients see
#     an equally fresh server, and removed again at the end of the cell.
#
# Usage:  bench/run.sh [--duration 30] [--reps 3] [--host 127.0.0.1] [--quick]
#                      [--variants control,active,model]
#         bench/run.sh --hardware --host <ip> --stage {b,c} [--duration 30] [--pause 20]
#
# --hardware drives a REAL arm.  It starts no container, stops no container and talks to no
# docker at all; it runs the `rt` condition only (chrt -f 80 + mlockall) and the `model`
# variant only, refuses the `plain` and hog conditions, and before every run it
#   1. probes the robot read-only (bench/rust/.../robot_probe): the run only proceeds from
#      RobotMode::Idle with no error flags,
#   2. runs automatic_error_recovery ONCE if the mode is Reflex, counting the event and
#      aborting the stage after the second one,
#   3. drives the arm back to the ready pose (bench/cpp/build/robot_home, libfranka's own
#      MotionGenerator at --home-speed, default 0.2),
# and it pauses --pause seconds after every run so the joints cool.  Both clients get the
# --hardware flag, which arms their |tau| <= 20 Nm and EE-deviation <= 0.10 m guards; on a
# violation the client finishes the motion cleanly and records the reason in its JSON.
# Results go to bench/results/<timestamp>-hw/.
#
#   --stage b   one C++ run
#   --stage c   REPS interleaved repetitions of both clients (odd reps C++ first)
#
# The simulator lock is still taken in hardware mode: it is what keeps a 12-core simulator
# matrix from competing for this box's CPU while a real arm is under torque control.
#
# The script takes the simulator lock itself (one simulator per machine), refuses to start
# if ports 1337/1338 are in use or another franka-sim container exists, and removes only the
# container it started.
set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${BENCH_DIR}/.." && pwd)"

SIM_LOCK="${SIM_LOCK:-${REPO_DIR}/.sim.lock}"
SIM_IMAGE="${SIM_IMAGE:-franka-sim:dev}"
SIM_NAME="${SIM_NAME:-franka-sim-bench}"
SIM_CPU_SHARES="${SIM_CPU_SHARES:-4096}"

DURATION=30
REPS=3
HOST=127.0.0.1
CONDITIONS=(plain rt rt_load)
VARIANTS=(control active)
HARDWARE=""
STAGE=""
PAUSE=20
HOME_SPEED=0.2
PROVENANCE=harness
# At most this many Reflex recoveries before the stage is abandoned.
MAX_REFLEX_EVENTS=2
# The `model` variant is about per-cycle compute cost, so it skips the CPU-hog condition.
MODEL_CONDITIONS="plain rt"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --reps)     REPS="$2"; shift 2 ;;
    --host)     HOST="$2"; shift 2 ;;
    --quick)    DURATION=5; REPS=1; shift ;;
    --variants) IFS=, read -r -a VARIANTS <<< "$2"; shift 2 ;;
    --hardware) HARDWARE=1; shift ;;
    --stage)    STAGE="$2"; shift 2 ;;
    --pause)    PAUSE="$2"; shift 2 ;;
    --home-speed) HOME_SPEED="$2"; shift 2 ;;
    --provenance) PROVENANCE="$2"; shift 2 ;;
    -h|--help)  sed -n '2,29p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# --- one simulator per machine: re-exec under flock ------------------------------------
mkdir -p "$(dirname "${SIM_LOCK}")"
touch "${SIM_LOCK}"
if [[ -z "${BENCH_HOLDS_SIM_LOCK:-}" ]]; then
  echo "waiting for the simulator lock (${SIM_LOCK}) ..."
  reexec_args=(--duration "${DURATION}" --reps "${REPS}" --host "${HOST}"
               --variants "$(IFS=,; echo "${VARIANTS[*]}")")
  if [[ -n "${HARDWARE}" ]]; then
    reexec_args+=(--hardware --stage "${STAGE}" --pause "${PAUSE}"
                  --home-speed "${HOME_SPEED}" --provenance "${PROVENANCE}")
  fi
  exec env BENCH_HOLDS_SIM_LOCK=1 flock "${SIM_LOCK}" "${BASH_SOURCE[0]}" "${reexec_args[@]}"
fi

HOG_BIN="${BENCH_DIR}/cpp/build/cpu_hog"
CPP_JV_BIN="${BENCH_DIR}/cpp/build/bench_joint_velocity"
RUST_JV_BIN="${BENCH_DIR}/rust/target/release/bench_joint_velocity"
CPP_MODEL_BIN="${BENCH_DIR}/cpp/build/bench_model_control"
RUST_MODEL_BIN="${BENCH_DIR}/rust/target/release/bench_model_control"
PROBE_BIN="${BENCH_DIR}/rust/target/release/robot_probe"
HOME_BIN="${BENCH_DIR}/cpp/build/robot_home"

# The binary a (client, variant) pair runs.
bin_for() {
  case "$1/$2" in
    cpp/model)  echo "${CPP_MODEL_BIN}" ;;
    rust/model) echo "${RUST_MODEL_BIN}" ;;
    cpp/*)      echo "${CPP_JV_BIN}" ;;
    rust/*)     echo "${RUST_JV_BIN}" ;;
  esac
}

# Whether a variant is run under a given condition.
variant_uses_condition() {
  if [[ "$1" == "model" ]]; then
    [[ " ${MODEL_CONDITIONS} " == *" $2 "* ]]
  else
    return 0
  fi
}

if [[ -n "${HARDWARE}" ]]; then
  case "${STAGE}" in
    b|c) ;;
    *) echo "--hardware needs --stage b or --stage c" >&2; exit 2 ;;
  esac
  # A hardware run is about the model variant under the rt condition; nothing else applies.
  VARIANTS=(model)
  CONDITIONS=(rt)
  MODEL_CONDITIONS="rt"
  if [[ "${STAGE}" == "b" ]]; then
    REPS=1
  fi
fi

REQUIRED_BINS=("${HOG_BIN}")
if [[ -n "${HARDWARE}" ]]; then
  REQUIRED_BINS=("${PROBE_BIN}" "${HOME_BIN}")
fi
for variant in "${VARIANTS[@]}"; do
  case "${variant}" in
    control|active|model) ;;
    *) echo "unknown variant: ${variant}" >&2; exit 2 ;;
  esac
  REQUIRED_BINS+=("$(bin_for cpp "${variant}")" "$(bin_for rust "${variant}")")
done
for bin in "${REQUIRED_BINS[@]}"; do
  if [[ ! -x "${bin}" ]]; then
    echo "missing ${bin}; see bench/README.md for the build steps" >&2
    exit 1
  fi
done

STAMP="$(date +%Y%m%dT%H%M%S)"
OUT_DIR="${BENCH_DIR}/results/${STAMP}"
if [[ -n "${HARDWARE}" ]]; then
  OUT_DIR="${BENCH_DIR}/results/${STAMP}-hw"
fi
mkdir -p "${OUT_DIR}"
echo "results -> ${OUT_DIR}"

# --- port / container preconditions ------------------------------------------------------
# None of this applies to a hardware run: no container is started, so no port or container
# check is meaningful and `docker` is never invoked.
if [[ -z "${HARDWARE}" ]] && ss -tlnp 2>/dev/null | grep -qE '133[78]'; then
  echo "ports 1337/1338 are already in use; refusing to start a simulator" >&2
  ss -tlnp | grep -E '133[78]' >&2
  exit 1
fi
if [[ -z "${HARDWARE}" ]] && [[ -n "$(docker ps -q --filter name=franka-sim)" ]]; then
  echo "a franka-sim container is already running; refusing to touch it" >&2
  docker ps --filter name=franka-sim >&2
  exit 1
fi
if [[ -z "${HARDWARE}" ]] && [[ -n "$(docker ps -aq --filter "name=^/${SIM_NAME}$")" ]]; then
  echo "a container named ${SIM_NAME} already exists; refusing to touch it" >&2
  exit 1
fi

HOG_PID=""
cleanup() {
  if [[ -n "${HOG_PID}" ]] && kill -0 "${HOG_PID}" 2>/dev/null; then
    kill "${HOG_PID}" 2>/dev/null || true
    wait "${HOG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SIM_STARTED:-}" ]]; then
    docker logs "${SIM_NAME}" >> "${OUT_DIR}/sim-full.log" 2>&1 || true
    docker rm -f "${SIM_NAME}" >/dev/null 2>&1 || true
    SIM_STARTED=""
  fi
}
trap cleanup EXIT

# The simulator is started fresh for every cell and removed again afterwards, so neither
# client ever inherits a server the other one has just run 30 s of motion against.
start_sim() {
  if [[ -n "${HARDWARE}" ]]; then return 0; fi
  docker run -d --network host --cpu-shares "${SIM_CPU_SHARES}" --name "${SIM_NAME}" \
    "${SIM_IMAGE}" >/dev/null
  SIM_STARTED=1
  docker exec "${SIM_NAME}" franka-sim-check --timeout 60 >/dev/null
}

stop_sim() {
  if [[ -n "${HARDWARE}" ]]; then return 0; fi
  if [[ -n "${SIM_STARTED:-}" ]]; then
    docker logs "${SIM_NAME}" >> "${OUT_DIR}/sim-full.log" 2>&1 || true
    docker rm -f "${SIM_NAME}" >/dev/null 2>&1 || true
    SIM_STARTED=""
  fi
}

if [[ -n "${HARDWARE}" ]]; then
  echo "HARDWARE mode: stage ${STAGE}, host ${HOST}, no container will be started or removed"
else
  echo "checking ${SIM_IMAGE} starts as ${SIM_NAME} (--cpu-shares ${SIM_CPU_SHARES})"
fi
start_sim

# --- machine description -----------------------------------------------------------------
CYCLICTEST="$(command -v cyclictest || true)"
STRESS_NG="$(command -v stress-ng || true)"

python3 - "${OUT_DIR}/meta.json" <<PY
import json, os, platform, subprocess, sys

def sh(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""

meta = {
    "timestamp": "${STAMP}",
    "hostname": platform.node(),
    "kernel": sh("uname -r"),
    "kernel_full": sh("uname -a"),
    "preempt_rt": os.path.exists("/sys/kernel/realtime"),
    "sys_kernel_realtime": os.path.exists("/sys/kernel/realtime"),
    "nproc": int(sh("nproc") or 0),
    "cpu_model": sh("grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | xargs"),
    "cpu_mhz_max": sh("lscpu | grep -i 'CPU max MHz' | cut -d: -f2- | xargs"),
    "mem_total_kb": int(sh("grep MemTotal /proc/meminfo | awk '{print \$2}'") or 0),
    "ulimit_l": sh("ulimit -l") or "unknown",
    "sched_rt_runtime_us": sh("cat /proc/sys/kernel/sched_rt_runtime_us"),
    "docker_image": "${SIM_IMAGE}",
    "docker_image_id": sh("docker image inspect -f '{{.Id}}' ${SIM_IMAGE}"),
    "sim_cpu_shares": ${SIM_CPU_SHARES},
    "git_rev": sh("git -C ${REPO_DIR} rev-parse HEAD"),
    "git_dirty": sh("git -C ${REPO_DIR} status --porcelain") != "",
    "libfranka": sh("ls ${LIBFRANKA_BUILD_DIR:-$HOME/libfranka/build-rt-ignore}/libfranka.so.* 2>/dev/null | head -1"),
    "rustc": sh("rustc --version"),
    "gcc": sh("c++ --version | head -1"),
    "cyclictest": "${CYCLICTEST}" or None,
    "stress_ng": "${STRESS_NG}" or None,
    "duration_s": ${DURATION},
    "reps": ${REPS},
    "variants": "${VARIANTS[*]}",
    "hardware": bool(${HARDWARE:-0}),
    "hardware_stage": "${STAGE}",
    "host": "${HOST}",
    "pause_s": ${PAUSE},
    "home_speed": ${HOME_SPEED},
}
with open(sys.argv[1], "w") as f:
    json.dump(meta, f, indent=2)
print(json.dumps({k: meta[k] for k in ("kernel", "cpu_model", "nproc", "preempt_rt")}, indent=2))
PY

# --- OS baseline -------------------------------------------------------------------------
if [[ -n "${CYCLICTEST}" ]]; then
  echo "cyclictest baseline (60 s, SCHED_FIFO 80)"
  chrt -f 80 "${CYCLICTEST}" -q -m -p 80 -i 1000 -h 400 -D 60 \
    > "${OUT_DIR}/cyclictest.txt" 2>&1 || echo "cyclictest failed" >&2
else
  echo "cyclictest not installed; skipping the OS baseline" | tee "${OUT_DIR}/cyclictest.txt"
fi

# --- the matrix --------------------------------------------------------------------------
run_one() {
  local client="$1" variant="$2" condition="$3" rep="$4" order="$5" cell_first="$6"
  local name="${client}_${variant}_${condition}_r${rep}"
  local json="${OUT_DIR}/${name}.json"
  local log="${OUT_DIR}/${name}.stdout"
  local simlog="${OUT_DIR}/${name}.simlog"

  local bin
  bin="$(bin_for "${client}" "${variant}")"

  local cmd=()
  local extra=()
  case "${condition}" in
    plain)   ;;
    rt)      cmd+=(chrt -f 80); extra+=(--mlock) ;;
    rt_load) cmd+=(chrt -f 80); extra+=(--mlock) ;;
  esac
  if [[ -n "${HARDWARE}" ]]; then
    # A hardware run is always the rt condition, and always with the client-side guards armed.
    cmd=(chrt -f 80)
    extra=(--mlock --hardware --reflex-events "${REFLEX_EVENTS}" --provenance "${PROVENANCE}")
  fi
  cmd+=("${bin}" "${HOST}" --variant "${variant}" --duration "${DURATION}"
        --condition "${condition}" --rep "${rep}" --order "${order}"
        --cell-first "${cell_first}" --out "${json}" "${extra[@]}")

  if [[ "${condition}" == "rt_load" ]]; then
    # No stress-ng on this box: our own busy-loop hog, one worker per core, at the default
    # scheduling policy so the SCHED_FIFO client can still preempt it.
    "${HOG_BIN}" "$(nproc)" $((DURATION + 60)) &
    HOG_PID=$!
    sleep 2
  fi

  local before=0
  if [[ -z "${HARDWARE}" ]]; then
    before="$(docker logs "${SIM_NAME}" 2>&1 | wc -l)"
  fi
  echo "--- ${name} (${order}/2 in cell, ${cell_first} first)"
  local status=0
  "${cmd[@]}" > "${log}" 2>&1 || status=$?
  if [[ -z "${HARDWARE}" ]]; then
    docker logs "${SIM_NAME}" 2>&1 | tail -n "+$((before + 1))" > "${simlog}" || true
  fi

  if [[ -n "${HOG_PID}" ]]; then
    kill "${HOG_PID}" 2>/dev/null || true
    wait "${HOG_PID}" 2>/dev/null || true
    HOG_PID=""
  fi

  if [[ ${status} -eq 3 ]]; then
    # Exit 3 means the control loop ended with a ControlException but the JSON was still
    # written; the run counts as a recorded result, not as a harness failure.
    echo "    control loop ended with an exception (recorded in the JSON); see ${log}" >&2
    tail -3 "${log}" >&2 || true
  fi
  if [[ ${status} -ne 0 && ${status} -ne 3 ]]; then
    echo "    FAILED (exit ${status}); see ${log}" >&2
    tail -3 "${log}" >&2 || true
  else
    python3 -c "
import json,sys
d=json.load(open('${json}'))
i=d['interval_us']
line='    cycles=%d p50=%.0f p99=%.0f max=%.0f lost=%d sr_avg=%.3f cpu=%.1f%%' % (
    d['cycles'], i['p50'], i['p99'], i['max'], d['lost']['cycles'],
    d['success_rate']['avg'], d['cpu']['percent'])
if d.get('compute_us') and d.get('model_us'):
    line += ' model_p50=%.1fus compute_p50=%.1fus' % (
        d['model_us']['p50'], d['compute_us']['p50'])
g = d.get('guard') or {}
if g.get('tripped'):
    line += ' GUARD=%s@%d (tau=%.2f ee=%.3f)' % (
        g['reason'], g['cycle'], g['tau_at_trip'], g['ee_deviation_at_trip'])
if d.get('control_exception'):
    line += ' EXCEPTION=%s' % d['control_exception'][:80]
print(line)"
  fi
  sleep 3
}

# --- hardware preconditions, run once before every hardware run ---------------------------
REFLEX_EVENTS=0

# Read-only probe; recovers once from Reflex and counts the event, then drives the arm back to
# the ready pose. Returns non-zero when the stage must not continue.
hardware_precheck() {
  local tag="$1"
  local probe="${OUT_DIR}/probe_${tag}.json"
  local status=0
  "${PROBE_BIN}" "${HOST}" --out "${probe}" > "${OUT_DIR}/probe_${tag}.stdout" 2>&1 || status=$?

  if [[ ${status} -eq 4 ]]; then
    local mode
    mode="$(python3 -c "import json;print(json.load(open('${probe}'))['mode'])" 2>/dev/null || echo unknown)"
    if [[ "${mode}" != "Reflex" ]]; then
      echo "    robot is ${mode}, not Idle, and that is not something to recover from" >&2
      cat "${OUT_DIR}/probe_${tag}.stdout" >&2
      return 1
    fi
    REFLEX_EVENTS=$((REFLEX_EVENTS + 1))
    echo "    robot is in Reflex (event ${REFLEX_EVENTS} of at most ${MAX_REFLEX_EVENTS}); recovering once"
    if (( REFLEX_EVENTS > MAX_REFLEX_EVENTS )); then
      echo "    too many reflex events; abandoning the stage" >&2
      return 1
    fi
    status=0
    "${PROBE_BIN}" "${HOST}" --recover --out "${probe}" \
      > "${OUT_DIR}/probe_${tag}.stdout" 2>&1 || status=$?
    if [[ ${status} -ne 0 ]]; then
      echo "    automatic_error_recovery did not leave the robot Idle; abandoning the stage" >&2
      cat "${OUT_DIR}/probe_${tag}.stdout" >&2
      return 1
    fi
  elif [[ ${status} -ne 0 ]]; then
    echo "    could not probe ${HOST}; abandoning the stage" >&2
    cat "${OUT_DIR}/probe_${tag}.stdout" >&2
    return 1
  fi

  # Back to the ready pose with libfranka's own MotionGenerator, as its own step.
  if ! "${HOME_BIN}" "${HOST}" --speed "${HOME_SPEED}" --out "${OUT_DIR}/home_${tag}.json" \
       > "${OUT_DIR}/home_${tag}.stdout" 2>&1; then
    echo "    the return-to-ready move failed; abandoning the stage" >&2
    cat "${OUT_DIR}/home_${tag}.stdout" >&2
    return 1
  fi
  return 0
}

if [[ -n "${HARDWARE}" ]]; then
  STAGE_STATUS=ok
  hardware_run() {
    local client="$1" rep="$2" order="$3" cell_first="$4"
    local tag="${client}_model_rt_r${rep}"
    if ! hardware_precheck "${tag}"; then
      STAGE_STATUS=aborted
      return 1
    fi
    run_one "${client}" model rt "${rep}" "${order}" "${cell_first}"
    echo "    pausing ${PAUSE} s so the joints cool"
    sleep "${PAUSE}"
    return 0
  }

  if [[ "${STAGE}" == "b" ]]; then
    # Stage B: a single C++ run.
    hardware_run cpp 1 1 cpp || true
  else
    # Stage C: REPS interleaved repetitions, odd reps C++ first.
    for rep in $(seq 1 "${REPS}"); do
      first=cpp
      second=rust
      if (( rep % 2 == 0 )); then
        first=rust
        second=cpp
      fi
      hardware_run "${first}" "${rep}" 1 "${first}" || break
      hardware_run "${second}" "${rep}" 2 "${first}" || break
    done
  fi

  python3 - "${OUT_DIR}/stage.json" <<STAGEPY
import json, sys
with open(sys.argv[1], "w") as f:
    json.dump({
        "stage": "${STAGE}",
        "status": "${STAGE_STATUS}",
        "host": "${HOST}",
        "reps": ${REPS},
        "duration_s": ${DURATION},
        "pause_s": ${PAUSE},
        "home_speed": ${HOME_SPEED},
        "reflex_events": ${REFLEX_EVENTS},
        "max_reflex_events": ${MAX_REFLEX_EVENTS},
    }, f, indent=2)
STAGEPY
  echo "stage ${STAGE}: ${STAGE_STATUS}, ${REFLEX_EVENTS} reflex event(s)"

  python3 "${BENCH_DIR}/summarize.py" "${OUT_DIR}" > "${OUT_DIR}/summary.md"
  echo
  echo "wrote ${OUT_DIR}/summary.md"
  cat "${OUT_DIR}/summary.md"
  exit 0
fi

for condition in "${CONDITIONS[@]}"; do
  for variant in "${VARIANTS[@]}"; do
    variant_uses_condition "${variant}" "${condition}" || continue
    for rep in $(seq 1 "${REPS}"); do
      # Alternate which client goes first so neither one is systematically the "warm" or the
      # "cold" run of every cell.
      local_first=cpp
      local_second=rust
      if (( rep % 2 == 0 )); then
        local_first=rust
        local_second=cpp
      fi

      # A fresh server for both clients of this cell.
      stop_sim
      start_sim

      run_one "${local_first}"  "${variant}" "${condition}" "${rep}" 1 "${local_first}"
      run_one "${local_second}" "${variant}" "${condition}" "${rep}" 2 "${local_first}"

      stop_sim
    done
  done
done

python3 "${BENCH_DIR}/summarize.py" "${OUT_DIR}" > "${OUT_DIR}/summary.md"
echo
echo "wrote ${OUT_DIR}/summary.md"
cat "${OUT_DIR}/summary.md"
