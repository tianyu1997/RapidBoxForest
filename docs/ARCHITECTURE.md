# Architecture

RapidBoxForest is a C++20 workspace with optional Python bindings. The code is
split into three packages so envelope math, persistent evidence, and planning
logic can evolve independently while still building together from the repository
root.

## Dependency Graph

```text
link_interval_envelope
        |
        v
lect_database
        |
        v
safe_box_forest
```

Dependencies only point downward. `link_interval_envelope` must not depend on
LECT or planner code. `lect_database` may depend on envelope primitives but not
planner implementation. `safe_box_forest` is the consumer that wires the
database-backed oracle into planning and query stages.

## Top-Level Build

The root `CMakeLists.txt` exposes workspace options and adds module
subdirectories in dependency order:

- `RBF_BUILD_ENVELOPE` controls `link_interval_envelope`.
- `RBF_BUILD_LECT_DATABASE` controls `lect_database`.
- `RBF_BUILD_SBF` controls `safe_box_forest`.
- `RBF_BUILD_TESTS`, `RBF_BUILD_TOOLS`, `RBF_WITH_PYTHON`, and
  `RBF_BUILD_EXPERIMENTS` are forwarded to module-specific options.

The top-level build does not define implementation targets of its own.

## Module Responsibilities

### `link_interval_envelope`

Primary target: `link_interval_envelope::core`.

Responsibilities:

- robot model, joint interval, FK, and interval math primitives;
- endpoint evidence sources such as IFK, CritSample, Analytical, GCPC, and MC;
- envelope types such as LinkIAABB, KDOP, and SupportHull;
- `compute_envelope_batch(...)` for independent interval boxes;
- `IncrementalEnvelopeContext` for repeated nearby interval queries;
- Python package `link_interval_envelope` and CLI/visualization helpers.

Important directories:

```text
include/sbf/core/                  low-level robot, interval, FK, math types
include/sbf/envelope/              endpoint source and envelope APIs
include/link_interval_envelope/    package-level C++ APIs
src/                               implementation
python/link_interval_envelope/     Python facade and CLI
tests/                             C++ and Python coverage
```

### `lect_database`

Primary targets:

- `LECTDatabase::core`
- `LECTDatabase::sbf_adapter`
- `LECTDatabase::online_cache`

Responsibilities:

- persistent LECT tree storage, node pages, evidence payloads, and manifests;
- split policy, canonicalization, identity checks, read snapshots, and compact
  storage modes for bulk or streaming prewarm;
- online envelope cache for repeated planner validation;
- SBF adapter layer: scene, collision checker, database-backed oracle, worker
  sessions, and shared endpoint evidence cache.

Important directories:

```text
include/rbf/lect_database/         core database API
include/LECTDatabase/              compatibility and package-facing headers
include/LECTDatabase/sbf/          planner adapter API
src/lect_database/                 core persistence implementation
src/online_cache/                  online cache implementation
src/sbf/                           planner adapter implementation
tools/                             database CLI and benchmark
tests/                             core, cache, and adapter tests
```

### `safe_box_forest`

Primary target: `SBF::core`.

Responsibilities:

- `RBFPlanningForest` facade for build, query, coverage build, dynamic update,
  and debug/benchmark entry points;
- grower stage, free-box search, graph construction, merger, connector, and
  corridor query;
- runtime configuration and stage execution helpers;
- Python package `sbf` and paper/experiment-facing helper scripts.

Important directories:

```text
include/SBF/                       planner public API
src/                               planner implementation
python/sbf/                        Python facade and scene helpers
experiments/                       package-local legacy/current scripts
tests/                             C++ and Python coverage
```

Current source-file boundaries:

```text
src/runtime.cpp                    runtime budget, deadline, and diagnostics helpers
src/find_free_box.cpp              seed-to-certified-box search service
src/grower.cpp                     forest grower implementations
src/leaf_sweep_grower.cpp          leaf-sweep coverage grower
src/adaptive_grid_partition.cpp    partition-native coverage/query data structure
src/box_graph.cpp                  explicit box graph, segment edges, Dijkstra helpers
src/connector.cpp                  island connector and chain-pave logic
src/merger.cpp                     box containment and merge helpers
src/query.cpp                      query result/path utility helpers
src/planning_forest_audit.cpp      path audit, audit checker, and segment-edge
                                   survival helpers shared by forest modules
src/planning_forest_core.cpp       `RBFPlanningForest` state reset/cache/core methods
src/planning_forest_partition.cpp  adaptive-partition maintenance/query helpers
src/planning_forest_dynamic_cache.cpp
                                   dynamic collision cache, obstacle-removal regrow,
                                   dynamic segment fallback, and removed-box
                                   refill helpers
src/safe_box_forest.cpp            remaining facade orchestration and legacy-large
                                   implementation blocks pending extraction
```

New production features should normally land in the smallest matching module
above. `src/safe_box_forest.cpp` is currently a compatibility/orchestration
holding file; adding new algorithm branches there should be treated as a
temporary step with a follow-up extraction.

## Planner Data Flow

1. A caller builds a `Robot`, scene obstacles, and `RBFPlanningConfig`.
2. `RBFPlanningForest` opens or creates a `LectDatabase` and an
   `OnlineEnvelopeCacheTree`.
3. Grower and connector stages propose seeds or segments in configuration
   space.
4. `FindFreeBoxService` asks the database-backed oracle to validate interval
   boxes.
5. The oracle resolves LECT nodes, reuses existing evidence when available, and
   calls `link_interval_envelope` to compute missing endpoint/envelope evidence.
6. Accepted boxes are added to the forest, adjacency and segment edges are
   updated, and optional merger/connector stages improve connectivity.
7. `CorridorQuery` searches the box graph and returns an audited path result.

Dynamic-update paths reuse the same forest state, mark dirty regions after
obstacle edits, locally regrow when possible, and fall back to warm rebuilds
when the configured thresholds require it.

## Planner Cleanup Direction

The open-source codebase should remain a single workspace rather than growing a
parallel "clean" implementation directory. A second implementation would split
CI, experiment reproduction, Python bindings, and public release tooling across
two trees. The maintainable path is to keep the current module layout and
separate production responsibilities inside `safe_box_forest`.

Current cleanup priorities:

1. Keep `RBFPlanningForest` as the stable public facade, but move large
   implementation blocks out of `src/safe_box_forest.cpp` by responsibility:
   coverage build, adaptive partition maintenance, query repair/corridors,
   dynamic update, and diagnostics.
2. Treat environment-variable controls as temporary experiment overrides.
   Production behavior should come from typed config structs or named
   experiment profiles; debug-only `RBF_*` switches should not define the
   default algorithm.
3. Keep paper-facing runners in top-level `experiments/`. Historical or
   diagnostic scripts must remain outside the public export and must not be
   required by current tables or figures.
4. Keep cache, canonicalization, and oracle semantics inside `lect_database`.
   Planner and experiment layers should pass native joint-space inputs and
   receive native boxes/paths.
5. Prefer small, behavior-preserving extractions before algorithm changes. A
   refactor is acceptable only when the existing CTest suite, public release
   checks, and paper provenance checks still pass.

## Experiments And Local Paper Artifacts

Current experiment runners are in top-level `experiments/`, with common helpers
under `experiments/common/`. They write generated data under ignored `outputs/`
paths. Manuscript sources and generated paper assets live in `paper/` in the
private development checkout, but the clean public source export intentionally
omits `paper/`. Historical package-local scripts and old manuscript trees may
exist in the private development checkout, but they are excluded from the clean
public export by default and should not be treated as preferred entry points for
new runs.

## Cleanup Rules

Safe to remove:

- `build*/`, `**/build*/`, CMake cache directories, and `_deps/`;
- `__pycache__/`, `*.pyc`, `.pytest_cache/`, and `.cache/`;
- `.sbf_lect_database/` and other local generated database caches;
- LaTeX intermediate files such as `*.aux`, `*.log`, `*.fls`,
  `*.fdb_latexmk`, `*.xdv`, `*.out`, and `*.blg`.

Review before removing:

- experiment result directories under `outputs/`;
- generated tables, figures, manifests, PDFs, and `.bbl` files used for paper
  reproducibility;
- archived scripts and planning notes that still document historical workflows.
