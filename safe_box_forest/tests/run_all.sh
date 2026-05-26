#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
LIE_SOURCE_DIR="${SBF_LINK_INTERVAL_ENVELOPE_SOURCE_DIR:-$ROOT/../link_interval_envelope}"

case "$BUILD_DIR" in
  /*) ;;
  *) BUILD_DIR="$(cd "$(dirname "$BUILD_DIR")" && pwd)/$(basename "$BUILD_DIR")" ;;
esac

cmake -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DSBF_LINK_INTERVAL_ENVELOPE_SOURCE_DIR="$LIE_SOURCE_DIR" \
  -DSBF_BUILD_TESTS=ON \
  -DSBF_BUILD_EXPERIMENTS="${SBF_BUILD_EXPERIMENTS:-OFF}" \
  -DSBF_WITH_PYTHON="${SBF_WITH_PYTHON:-ON}" \
  -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE:-python}"

cmake --build "$BUILD_DIR" -j"${JOBS:-$(nproc)}"
ctest --test-dir "$BUILD_DIR" --output-on-failure

if grep -R "cpp/v6\|<sbf/forest/\|<sbf/ffb/\|<sbf/planner/\|<sbf/lect/" "$ROOT/include" "$ROOT/src" "$ROOT/python" >/tmp/sbf_v6_grep.txt; then
  cat /tmp/sbf_v6_grep.txt
  echo "SBF independence check failed: found v6-only include or path" >&2
  exit 1
fi

echo "All SBF tests passed."