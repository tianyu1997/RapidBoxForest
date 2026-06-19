#!/usr/bin/env bash
set -euo pipefail

MODULE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "$MODULE_ROOT/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$WORKSPACE_ROOT/build-sbf}"

case "$BUILD_DIR" in
  /*) ;;
  *) BUILD_DIR="$(cd "$(dirname "$BUILD_DIR")" && pwd)/$(basename "$BUILD_DIR")" ;;
esac

RBF_WITH_PYTHON_VALUE="${RBF_WITH_PYTHON:-${SBF_WITH_PYTHON:-ON}}"
CMAKE_ARGS=()
case "${RBF_WITH_PYTHON_VALUE^^}" in
  ON|TRUE|1|YES)
    CMAKE_ARGS+=("-DPython3_EXECUTABLE=${PYTHON_EXECUTABLE:-python}")
    ;;
esac

cmake -S "$WORKSPACE_ROOT" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=ON \
  -DRBF_BUILD_SBF=ON \
  -DRBF_BUILD_TESTS=ON \
  -DRBF_WITH_PYTHON="$RBF_WITH_PYTHON_VALUE" \
  "${CMAKE_ARGS[@]}"

cmake --build "$BUILD_DIR" -j"${JOBS:-$(nproc)}"
ctest --test-dir "$BUILD_DIR" --output-on-failure

GREP_OUTPUT="$BUILD_DIR/sbf_v6_grep.txt"
if grep -R "cpp/v6\|<sbf/forest/\|<sbf/ffb/\|<sbf/planner/\|<sbf/lect/" "$MODULE_ROOT/include" "$MODULE_ROOT/src" "$MODULE_ROOT/python" >"$GREP_OUTPUT"; then
  cat "$GREP_OUTPUT"
  echo "SBF independence check failed: found v6-only include or path" >&2
  exit 1
fi

echo "All SBF tests passed."
