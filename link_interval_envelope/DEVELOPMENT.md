# Development

This repository is a self-contained CMake/Python package. It intentionally does not depend on the original monorepo's planner, LECT, forest, Drake, or OMPL code.

## Build

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIE_BUILD_TESTS=ON \
  -DLIE_WITH_PYTHON=ON \
  -DPython3_EXECUTABLE=/path/to/python
cmake --build build -j
ctest --test-dir build --output-on-failure
```

When switching Python versions, use a fresh build directory or delete `CMakeCache.txt`; the native extension ABI must match the runtime interpreter.

## Full Test Script

```bash
PYTHON_EXECUTABLE=/path/to/python BUILD_DIR=build bash tests/run_all.sh
```

The script builds the C++ core, builds the Python extension, runs CTest, runs Python unittest, and performs a CLI JSON/HTML smoke test.

## Optimized Paths

- IFK uses the v6-compatible `FKState` and in-place incremental FK updates when `changed_dim` is known.
- CritSample caches per-joint critical candidates, capped candidates, precomputed DH matrices, and unchanged endpoint iAABBs.
- `compute_envelope_batch()` parallelizes independent boxes for IFK and CritSample. Other endpoint sources keep their one-shot implementations in this phase.

## Clean Source Boundary

Keep source releases limited to:

- `README.md`, `CMakeLists.txt`, `cmake/`
- `include/`, `src/`, `python/`
- `examples/`, `tests/`, `docs/`
- metadata files such as `LICENSE`, `DEVELOPMENT.md`, `pyproject.toml`, `.github/`

Do not commit `build*`, `examples/out`, `__pycache__`, native extension `.so` files, or artifacts copied from the monorepo's v6 planner pipeline.