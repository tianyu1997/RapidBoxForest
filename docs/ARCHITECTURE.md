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
src/grower.cpp                     RRT-style forest grower implementation
src/frontwave_grower.cpp           frontwave coverage grower implementation
src/grower_failure_cooling.cpp     RRT grower failure-cooling and hard-frontier
                                   stop-loss policy
src/grower_frontier.cpp            RRT grower frontier face selection, face
                                   memory, and seed tracing helpers
src/grower_internal.h              grower-local shared commit, lookup, and
                                   diagnostics helpers
src/grower_options.cpp             RRT grower depth-stage, component-connect
                                   FFB option selection, and FFB/oracle
                                   diagnostics helpers
src/grower_task_builder.cpp        RRT grower batched target sampling, task
                                   generation, frontier seed selection, and
                                   worker-domain filtering
src/grower_trace.cpp               optional RRT grower JSON trace output
src/leaf_sweep_grower.cpp          leaf-sweep coverage grower orchestration,
                                   group sweep, and common validation helpers
src/leaf_sweep_grower_cluster.cpp  obstacle AABB clustering for leaf sweep
src/leaf_sweep_grower_compose.cpp  final leaf-sweep free/collision set
                                   composition across obstacle groups
src/leaf_sweep_grower_frontier.cpp start-frontier materialization for leaf
                                   sweep, including virtual topology splitting
src/adaptive_grid_partition.cpp    partition-native coverage/query data structure
src/adaptive_grid_partition_geometry.cpp
                                   pure geometry, distance, interval, and grid
                                   utility routines used by the partition
src/adaptive_grid_partition_indices.cpp
                                   partition runtime index rebuild, incremental
                                   append, hash, and island-update helpers
src/adaptive_grid_partition_overlay.cpp
                                   overlay segment edges and overlay-aware
                                   component queries for the partition
src/adaptive_grid_partition_keys.h
                                   internal grid hash keys for partition merge,
                                   broadphase, and adjacency indices
src/box_graph.cpp                  explicit box adjacency, graph cache, point
                                   location, and path-length helpers
src/box_graph_edges.cpp            segment-edge, portal-corridor, and
                                   segment-edge adjacency helpers
src/box_graph_search.cpp           Dijkstra graph search and waypoint
                                   extraction
src/box_graph_sequence.cpp         box-sequence shortcut and bridge-node
                                   compression helpers
src/box_graph_topology.cpp         island and articulation-point graph
                                   topology helpers
src/connector.cpp                  island connector orchestration, frontier
                                   bridge, point-gap fallback, and bridge
                                   commit loop
src/connector_birrt.cpp            RRTConnect/BiRRT connector path search and
                                   diagnostics
src/connector_internal.cpp         connector-local geometry, broadphase,
                                   incremental graph, and diagnostics helpers
src/connector_chain_pave.cpp       connector chain-pave box insertion along
                                   waypoint paths
src/grower_components.cpp          root/component grouping and distance
                                   helpers for RRT grower connectivity
src/merger.cpp                     box containment and merge helpers
src/query.cpp                      query result/path utility helpers
src/planning_forest_adaptive_build.cpp
                                   adaptive deep leaf sweep orchestration and
                                   query-root refinement backend
src/planning_forest_adaptive_fixed.cpp
                                   fixed virtual leaf-sweep materialization,
                                   merge, partition rebuild, coverage probe,
                                   and build-profile finalization
src/planning_forest_adaptive_cover_utils.cpp
                                   adaptive cover frontier, depth, probe, and
                                   connectivity scoring helpers
src/planning_forest_adaptive_merge.cpp
                                   budgeted/grid/tree/exact merge helpers for
                                   adaptive build backends
src/planning_forest_build.cpp      build and leaf-sweep build entry points
src/planning_forest_database.cpp   default config, LECT database identity/root
                                   setup, external evidence, and forest
                                   construction
src/planning_forest_debug.cpp      debug chain-pave entry points and explicit
                                   query-corridor refinement helpers
src/planning_forest_audit.cpp      path audit, audit checker, and segment-edge
                                   survival helpers shared by forest modules
src/planning_forest_core.cpp       `RBFPlanningForest` state reset/cache/core methods
src/planning_forest_partition.cpp  adaptive-partition maintenance/query helpers
src/planning_forest_shortcut.cpp   offline shortcut-edge selection and audited
                                   shortcut/corridor insertion
src/planning_forest_dynamic_cache.cpp
                                   dynamic obstacle insertion/removal rebuild,
                                   dirty-region checks, collision-cache promotion,
                                   dynamic segment fallback, and removed-box refill
                                   helpers
src/planning_forest_overlay.cpp    partition box/portal overlay corridor helpers
src/planning_forest_overlay_edges.cpp
                                   partition-first segment edge insertion,
                                   OBB bridge/segment cover, and edge sync
src/planning_forest_query.cpp      online query entry point, strict audit,
                                   final simplify, and local repair
src/planning_forest_query_utils.cpp
                                   query/path utility functions shared by graph,
                                   partition, OBB, and bridge stages
src/planning_forest_query_utils_shortcut.cpp
                                   collision-checked shortcut and path
                                   hybridization helpers
src/planning_forest_query_utils_rrt.cpp
                                   audited RRT bridge attempt selection and
                                   parallel attempt diagnostics
src/planning_forest_qroot_helpers.cpp
                                   query-root growth DSU/index/commit helpers
src/planning_forest_query_bridge_batch_utils.cpp
                                   query bridge batch task records, acceptance,
                                   and route scoring helpers
src/planning_forest_query_bridge_batch_diagnostics.cpp
                                   query bridge oracle counter deltas, task
                                   skip records, and direct-corridor diagnostic
                                   aggregation
src/planning_forest_query_bridge_batch_options.cpp
                                   environment/config parsing for batch bridge
                                   retries, parallel RRT, detours, path-quality
                                   retry, and index-selection controls
src/planning_forest_query_bridge_path_utils.cpp
                                   waypoint path length, shortcut, and internal
                                   simplification helpers shared by bridge
                                   attempts
src/planning_forest_query_bridge_corridor_options.cpp
                                   runtime option parsing for direct corridor,
                                   reusable edge, shortcut, residual, and
                                   detailed-timing controls
src/planning_forest_query_bridge_repair_options.cpp
                                   subdivision, adaptive, and lateral repair
                                   option builders for direct corridor paving
src/planning_forest_query_bridge_direct_segments.cpp
                                   direct start-goal and fast post-RRT segment
                                   edge insertion helpers
src/planning_forest_query_bridge_edges.cpp
                                   query bridge box-corridor, residual segment,
                                   and waypoint task edge completion helpers
src/planning_forest_query_bridge_endpoint_direct.cpp
                                   direct endpoint-to-main audited segment
                                   insertion helper
src/planning_forest_query_bridge_endpoint.cpp
                                   endpoint-to-main box-corridor attachment
                                   implementation
src/planning_forest_query_bridge_pave.cpp
                                   query bridge chain-pave execution and
                                   partition append helpers
src/planning_forest_query_bridge_waypoint.cpp
                                   waypoint-path corridor paving and local
                                   repair implementation
src/planning_forest_query_bridge_corridor_utils.cpp
                                   direct-corridor paving local data structures,
                                   DSU, FFB task generation, transition
                                   ordering, lateral repair candidate, and
                                   residual-gap helper functions
src/planning_forest_query_bridge_batch.cpp
                                   batched query bridge scheduling and per-query
                                   repair accounting
src/planning_forest_query_bridge_endpoint.cpp
                                   endpoint anchoring and endpoint-to-main
                                   corridor repair
src/planning_forest_query_bridge_endpoint_options.h
                                   endpoint-anchor option helpers kept header-only
                                   for the endpoint repair module
src/planning_forest_query_bridge_pair.cpp
                                   pair-level query bridge orchestration and
                                   RRT/segment fallback dispatch
```

New production features should normally land in the smallest matching module
above. Large experimental branches should be isolated behind typed
configuration and placed in the matching module instead of reintroducing a
monolithic facade implementation file.

The former sidecar prototype tree has been retired. Its useful mechanisms are
now production code in `lect_database` and `safe_box_forest`; new experiments
and implementations should not depend on a parallel workspace.

Integrated mechanisms that previously lived in prototype workspace form now
belong to the main source tree:

- sparse/binary FFB execution belongs in `safe_box_forest/src/find_free_box.cpp`
  and `safe_box_forest/src/virtual_sparse_ffb.*`;
- adaptive-grid sparse indexing belongs in
  `safe_box_forest/src/adaptive_grid_partition_sparse.cpp` and related
  partition modules;
- HiPaC, portal, OBB, direct-corridor, and query-bridge repair behavior belongs
  in the `planning_forest_query_bridge_*` modules and typed
  `RBFPlanningConfig`/`AdaptiveLeafSweepConfig` options;
- LECT evidence, canonical mapping, split policy, and cache reuse optimizations
  belong in `lect_database`, not in planner- or experiment-local sidecars.

The source release tools enforce this rule. If an `improve_workspace/`
directory exists anywhere in the source tree, release export and readiness
checks fail; the correct fix is to migrate the code into the owning module and
delete the sidecar directory.

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

1. Keep `RBFPlanningForest` as the stable public facade and keep production
   code in responsibility-oriented modules. Do not reintroduce a monolithic
   `safe_box_forest.cpp`; remaining large modules should be split only when a
   clearer responsibility boundary is available.
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
