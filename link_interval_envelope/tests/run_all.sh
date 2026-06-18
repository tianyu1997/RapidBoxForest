#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-$(command -v python3 || command -v python)}"

case "$BUILD_DIR" in
  /*) ;;
  *) BUILD_DIR="$(cd "$(dirname "$BUILD_DIR")" && pwd)/$(basename "$BUILD_DIR")" ;;
esac

cmake -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DLIE_BUILD_TESTS=ON \
  -DLIE_WITH_PYTHON=ON \
  -DPython3_EXECUTABLE="$PYTHON_EXECUTABLE"

cmake --build "$BUILD_DIR" -j"${JOBS:-$(nproc)}"
ctest --test-dir "$BUILD_DIR" --output-on-failure

export PYTHONPATH="$BUILD_DIR/python:$ROOT/python${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON_EXECUTABLE" -m unittest discover -s "$ROOT/tests" -p "test_python_api.py"

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT
"$PYTHON_EXECUTABLE" -m link_interval_envelope compute \
  --robot "$ROOT/examples/data/2dof_planar.json" \
  --intervals-json '[[-0.4, 0.4], [-0.2, 0.2]]' \
  --endpoint-source ifk \
  --env link_iaabb \
  --n-sub 4 \
  --out-json "$tmpdir/lie.json" \
  --out-html "$tmpdir/lie.html"
test -s "$tmpdir/lie.json"
test -s "$tmpdir/lie.html"

echo "All link_interval_envelope tests passed."
