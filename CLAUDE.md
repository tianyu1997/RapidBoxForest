# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

RapidBoxForest is a workspace-style monorepo containing three first-party modules with strict unidirectional dependencies:

```
link_interval_envelope -> lect_database -> safe_box_forest
```

- **link_interval_envelope/**: Envelope and incremental-FK primitives (lowest level)
- **lect_database/**: Database, online cache, and SBF adapter layer (middle)
- **safe_box_forest/**: Planner and query pipeline (highest level)

The root `CMakeLists.txt` is the integrated entry point that builds modules in dependency order. Each module can also be built standalone for development.

## Common Commands

### Full Workspace Build and Test

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=ON \
  -DRBF_BUILD_SBF=ON \
  -DRBF_BUILD_TESTS=ON \
  -DRBF_WITH_PYTHON=ON \
  -DPython3_EXECutable=python3

cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

### Module-Specific Testing

Each module has its own test script that configures from the workspace root:

```bash
# SBF tests (includes independence check for v6-only includes)
bash safe_box_forest/tests/run_all.sh

# Link Interval Envelope tests
bash link_interval_envelope/tests/run_all.sh
```

Environment overrides for test scripts:
- `BUILD_DIR`: override build directory (default varies by module)
- `PYTHON_EXECUTABLE`: choose Python interpreter
- `RBF_WITH_PYTHON`: build Python bindings
- `SBF_BUILD_EXPERIMENTS`: build SBF experiment drivers

### Python Environment

The repository uses Python 3.10.12 (configured in `.venv`). The Python package is available at `${BUILD_DIR}/python` after building with `RBF_WITH_PYTHON=ON`.

```bash
# Set PYTHONPATH for Python scripts
export PYTHONPATH="${BUILD_DIR}/python"
```

### Experiment Runners

New-style experiment runners are under `experiments/` and use a common framework:

```bash
# Dry-run to check manifest before execution
python3 experiments/exp04_shelf_ablation/run_shelf_ablation.py --dry-run

# Smoke test for quick validation
python3 experiments/exp04_shelf_ablation/run_shelf_ablation.py --smoke --dry-run
```

Experiment outputs are written to `outputs/new_experiments/<experiment>/`. See `experiments/00_experiment_principles.md` for common experimental principles and `experiments/README.md` for experiment-specific plans.

## Architecture

### Dependency Direction

```
safe_box_forest
    depends on -> lect_database (via LECTDatabase::core, LECTDatabase::sbf_adapter, LECTDatabase::online_cache)
        depends on -> link_interval_envelope (via low-level sbf types)
```

### Key C++ Headers

- `safe_box_forest/include/rbf/sbf.h` - Primary SBF facade include
- `link_interval_envelope/include/sbf/*` - Low-level shared types (Robot, Interval, FKState, etc.)
- `link_interval_envelope/include/link_interval_envelope/*` - Package-level API (batch, incremental context)
- `lect_database/include/lectdb/*` - Database and adapter layer

### SBF Pipeline Stages

The `SafeBoxForest` facade orchestrates these stages:
1. **Growers**: `RrtGrower`, `FrontwaveGrower` - sample and expand boxes
2. **Free-box construction**: `FindFreeBoxService` - find collision-free intervals
3. **Merger**: `Consolidator` - consolidate overlapping boxes
4. **Connector**: RRT-Connect, chain paving, island connector - connect disconnected components
5. **Query**: `CorridorQuery` - extract paths from built forest

### Parallel Execution

SBF is serial by default. Parallel execution requires explicit `RuntimeConfig`:
```cpp
sbf::RuntimeConfig runtime;
runtime.mode = sbf::ExecutionMode::Parallel;
runtime.n_threads = 4;
```

Worker tasks use `BorrowedBoxOracleSession` for LECT-backed oracle isolation. Mutable commits happen serially in master thread.

### Online Cache and Warm Reuse

For warm evidence reuse, the recommended split is:
- Active planning cache: writable `LectDatabaseRuntimeConfig.path`
- Warm evidence source: read-only `LectDatabaseRuntimeConfig.external_evidence_path`

Use snapshot mode for faster warm reads:
```cpp
config.database.external_evidence_use_snapshot = true;
```

## Source Layout

- `include/`: Public headers (module-specific subdirectories)
- `src/`: Implementation files
- `python/`: Python bindings and pure-Python utilities
- `tests/`: C++ tests and Python tests
- `experiments/`: Experiment scripts and common utilities
- `docs/`: Module documentation
- `examples/`: Example code and data

## Testing Notes

- `safe_box_forest/tests/run_all.sh` includes a grep check for v6-only includes (`cpp/v6`, `<sbf/forest/`, etc.)
- When switching Python versions, use a fresh build directory or delete `CMakeCache.txt` (native extension ABI must match)
- CTest is the primary C++ test runner
- Python tests can be run with `python -m unittest discover -s tests -p 'test_python_api.py'`

## Build Options

Root CMake options:
- `RBF_BUILD_ENVELOPE`: Build link_interval_envelope module (default: ON)
- `RBF_BUILD_LECT_DATABASE`: Build lect_database module (default: ON)
- `RBF_BUILD_SBF`: Build safe_box_forest module (default: OFF)
- `RBF_BUILD_TESTS`: Build tests (default: ON)
- `RBF_BUILD_EXPERIMENTS`: Build optional experiment drivers (default: OFF)
- `RBF_WITH_PYTHON`: Build Python bindings (default: OFF)

## Module-Specific Notes

### link_interval_envelope

Exposes optimized paths for:
- IFK (Incremental Forward Kinematics) with in-place FK updates
- CritSample with candidate caching and precomputed DH matrices
- Batch computation for IFK and CritSample endpoint sources
- Incremental envelope context for repeated neighboring box queries

See `link_interval_envelope/DEVELOPMENT.md` for clean source boundary rules.

### lect_database

Provides database-backed oracle and online cache. The adapter layer owns:
- `Scene`, `CollisionChecker`, `DatabaseBoxOracle`
- Worker session support for parallel SBF execution
- Shared endpoint evidence cache

### safe_box_forest

The paper-facing `SBF-SH` profile is the default config:
- CritSample endpoint evidence
- SupportHull envelopes with retained KDOP26 slabs
- 64-box connected quality floor
- Strict path audit
- Collision-checked path post-processing

See `safe_box_forest/docs/API.md` for detailed C++ facade and Python API documentation.