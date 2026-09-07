#!/usr/bin/env bash
# Runs the offline `franka::Model` microbenchmark in both languages and prints a comparison
# table. No robot, no simulator, no simulator lock: this only exercises the two model
# backends on the same inputs.
#
# The C++ program draws the random (q, dq) inputs and writes them to `states.json` plus its
# own outputs to `reference.bin`; the Rust program replays exactly those inputs and
# cross-checks every output against the reference.
#
# Usage:  bench/model-micro/run.sh [--count 10000] [--warmup 1000] [--seed N] [--out DIR]

set -euo pipefail

MICRO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${MICRO_DIR}/../.." && pwd)"

COUNT=10000
WARMUP=1000
SEED=20260904
OUT_DIR=""
URDF="${REPO_DIR}/crates/franka-rs/tests/data/fr3.urdf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count)  COUNT="$2"; shift 2 ;;
    --warmup) WARMUP="$2"; shift 2 ;;
    --seed)   SEED="$2"; shift 2 ;;
    --out)    OUT_DIR="$2"; shift 2 ;;
    --urdf)   URDF="$2"; shift 2 ;;
    -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

CPP_BIN="${MICRO_DIR}/cpp/build/model_micro_cpp"
RUST_BIN="${MICRO_DIR}/rust/target/release/model_micro_rust"
for bin in "${CPP_BIN}" "${RUST_BIN}"; do
  if [[ ! -x "${bin}" ]]; then
    echo "missing ${bin}; see bench/README.md for the build steps" >&2
    exit 1
  fi
done

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${MICRO_DIR}/results/$(date +%Y%m%dT%H%M%S)"
fi
mkdir -p "${OUT_DIR}"
echo "results -> ${OUT_DIR}"

# The C++ side goes first because it generates the shared input set.
"${CPP_BIN}" "${URDF}" --out-dir "${OUT_DIR}" --count "${COUNT}" --warmup "${WARMUP}" \
  --seed "${SEED}" > "${OUT_DIR}/cpp.stdout"
"${RUST_BIN}" "${URDF}" --dir "${OUT_DIR}" > "${OUT_DIR}/rust.stdout"

python3 - "${OUT_DIR}" <<'PY' | tee "${OUT_DIR}/summary.md"
import json, sys

out_dir = sys.argv[1]
cpp = json.load(open(f"{out_dir}/cpp.json"))
rust = json.load(open(f"{out_dir}/rust.json"))

calls = ["mass", "coriolis", "gravity", "zero_jacobian", "pose"]
print(f"# `franka::Model` microbenchmark ({cpp['count']} random states, "
      f"{cpp['warmup']} warm-up, seed {cpp['seed']})")
print()
print("Per-call wall time in microseconds, `CLOCK_MONOTONIC` around each individual call.")
print()
print("| call | C++ p50 | C++ p99 | C++ max | Rust p50 | Rust p99 | Rust max | p50 ratio |")
print("|---|---|---|---|---|---|---|---|")
for name in calls:
    c, r = cpp["calls"][name], rust["calls"][name]
    ratio = c["p50"] / r["p50"] if r["p50"] else float("nan")
    print(f"| `{name}` | {c['p50']:.2f} | {c['p99']:.2f} | {c['max']:.2f} "
          f"| {r['p50']:.2f} | {r['p99']:.2f} | {r['max']:.2f} | {ratio:.1f}x |")
c, r = cpp["total"], rust["total"]
ratio = c["p50"] / r["p50"] if r["p50"] else float("nan")
print(f"| **all five** | **{c['p50']:.2f}** | {c['p99']:.2f} | {c['max']:.2f} "
      f"| **{r['p50']:.2f}** | {r['p99']:.2f} | {r['max']:.2f} | **{ratio:.1f}x** |")
extra = cpp["calls"].get("coriolis_rnea")
if extra:
    print(f"| `coriolis` (C++ non-deprecated RNEA overload) | {extra['p50']:.2f} "
          f"| {extra['p99']:.2f} | {extra['max']:.2f} | - | - | - | - |")
print()
print("As a fraction of the 1 ms control cycle: "
      f"C++ {cpp['total']['p50'] / 1000.0 * 100:.1f} %, "
      f"Rust {rust['total']['p50'] / 1000.0 * 100:.1f} % at p50.")
print()
print("## Backend agreement")
print()
print("Maximum absolute difference between the two backends over the identical inputs.")
print()
print("| quantity | max abs diff | worst sample |")
print("|---|---|---|")
for name, entry in rust["agreement_vs_cpp"].items():
    print(f"| `{name}` | {entry['max_abs_diff']:.3e} | {entry['worst_sample']} |")
print()
worst = max(e["max_abs_diff"] for e in rust["agreement_vs_cpp"].values())
print(f"Worst disagreement anywhere: **{worst:.3e}**.")
PY
