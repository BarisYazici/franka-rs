#!/usr/bin/env bash
# Runs the offline FCI v5 (Franka Emika Robot, FER) `libfcimodels` microbenchmark in both languages and
# prints the two reports side by side. No robot, no simulator, no simulator lock: this
# loads a captured `libfcimodels_x64.so` and calls functions out of it.
#
# The Rust program drives the crate's `SoModelBackend` through `franka::Model`; the C++
# program drives the same shared object through libfranka 0.9.2's own `LibraryLoader` +
# `ModelLibrary` + `franka::Model` call path, reproduced verbatim from the 0.9.2 sources.
# Both evaluate the same `q`, `dq`, `F_T_EE` and load parameters.
#
# Read the `steady state` block for the Rust-versus-C++ comparison. The `duty cycled`
# blocks below it show what the *same* calls read as when they run first in a 1 kHz cycle
# after a ~1 ms idle wait: on a laptop-class part that inflates them several-fold, and by
# an amount that depends on how much unrelated work ran first. See the "Model wrapper
# follow-up" section of the full FER benchmark write-up, kept privately.
#
# Build:
#   cmake -S bench/so-micro/cpp -B bench/so-micro/cpp/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build bench/so-micro/cpp/build -j
#   cargo build --release --manifest-path bench/so-micro/rust/Cargo.toml
#
# Usage:  bench/so-micro/run.sh [--count 100000] [--so PATH] [--out DIR]

set -euo pipefail

MICRO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COUNT=100000
OUT_DIR=""
# Same fixture, and the same environment variable, as
# `crates/franka-rs/tests/fer_model_conformance.rs`.
MODEL_SO="${FRANKA_FER_MODEL_SO:-${FRANKA_FER_MODEL_LIBRARY:-}}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count) COUNT="$2"; shift 2 ;;
    --so)    MODEL_SO="$2"; shift 2 ;;
    --out)   OUT_DIR="$2"; shift 2 ;;
    -h|--help) sed -n '2,21p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

CPP_BIN="${MICRO_DIR}/cpp/build/so_micro_cpp"
RUST_BIN="${MICRO_DIR}/rust/target/release/so_micro_rust"
for bin in "${CPP_BIN}" "${RUST_BIN}"; do
  if [[ ! -x "${bin}" ]]; then
    echo "missing ${bin}; see the build steps in the header of this script" >&2
    exit 1
  fi
done

if [[ ! -f "${MODEL_SO}" ]]; then
  echo "no model library at ${MODEL_SO}" >&2
  echo "set FRANKA_FER_MODEL_SO to a captured libfcimodels_x64.so" >&2
  exit 1
fi

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${MICRO_DIR}/results/$(date +%Y%m%dT%H%M%S)"
fi
mkdir -p "${OUT_DIR}"
echo "model library -> ${MODEL_SO}"
echo "results       -> ${OUT_DIR}"
echo

echo "=== C++ (libfranka 0.9.2 call path) ==="
"${CPP_BIN}" "${MODEL_SO}" "${COUNT}" | tee "${OUT_DIR}/cpp.stdout"
echo
echo "=== Rust (franka-rs SoModelBackend) ==="
"${RUST_BIN}" "${MODEL_SO}" "${COUNT}" | tee "${OUT_DIR}/rust.stdout"
