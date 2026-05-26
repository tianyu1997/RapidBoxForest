#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build_experiments}"

SBF_BUILD_EXPERIMENTS=ON BUILD_DIR="$BUILD_DIR" bash "$ROOT/tests/run_all.sh"

echo "All SBF experiments passed."