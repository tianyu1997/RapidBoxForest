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
  oracle_types.h                   SBF oracle ids, validation configs,
                                   counters, session configs, and detail records
  oracle.h                         concrete BoxOracle/DatabaseBoxOracle
                                   interfaces and database-backed adapter
src/lect_database/                 core persistence implementation
  database.cpp                     database open/create state machine,
                                   manifests, evidence, journal, checkpoint,
                                   and parent-hull materialization
  database_evidence_api.cpp        public evidence read/write/delete entry
                                   points and endpoint exact-box evidence API
  database_evidence_codec.cpp      evidence binary/text codec, sidecar layout,
                                   payload checksum, and half-float quantizer
  database_evidence_index.cpp      in-memory open-addressed evidence index
                                   reserve/find/upsert operations
  database_evidence_store.cpp      append-only evidence file, mmap read path,
                                   scan/load/save, append, and resident cache trim
  database_evidence_sidecar.cpp    evidence index sidecar load/save and mapped
                                   evidence prefetch range scheduling
  database_file_layout.cpp         manifest, node-page, journal, and evidence
                                   file-path helpers plus node-row text codec
  database_journal.cpp             write-ahead journal replay, append stream,
                                   and committed transaction writer
  database_mapped_file.cpp         platform-specific read-only evidence mmap
                                   and prefetch RAII wrapper
  database_node_pages.cpp          LECT node page cache, resident-page
                                   accounting, and node-row flush/read/write
  database_parent_hull.cpp         parent-hull evidence propagation,
                                   deferred parent writes, and bottom-up
                                   internal hull materialization
  database_query.cpp               tree topology navigation, node box lookup,
                                   exact box lookup, and interval range query
  database_tree.cpp                LECT tree split/build mutation, node-id
                                   allocation, layer/page index maintenance,
                                   and interval geometry helpers
  read_snapshot_mapped_file.cpp    read-snapshot mmap RAII wrapper shared by
                                   snapshot open/load paths
  read_snapshot_builder.cpp        legacy LECT database to read-snapshot
                                   conversion
  read_snapshot_format.h           read-snapshot and legacy sidecar binary
                                   layout constants/records
  read_snapshot_legacy.cpp         legacy node-page text parsing and sidecar
                                   payload layout helpers
  read_snapshot_manifest.cpp       legacy manifest key-value parsing and root
                                   interval extraction for snapshot builds
  read_snapshot_payload.cpp        half-float payload decode helper for
                                   read-only snapshots
  read_snapshot_paths.cpp          legacy/snapshot file path helpers and
                                   atomic staging directory publish
src/online_cache/                  online cache implementation
src/sbf/                           planner adapter implementation
  oracle.cpp                       database-backed SBF oracle, evidence lookup,
                                   split execution, and box validation
  oracle_best_tighten.cpp          best-tighten split-dimension scoring and
                                   schedule replay helpers
  oracle_canonical.cpp             canonical-sector and native/canonical
                                   interval mapping helpers for the oracle
  oracle_classify.cpp              endpoint-payload envelope construction,
                                   collision classification, and blockers
  oracle_endpoint_materialization.cpp
                                   endpoint materialization config helpers for
                                   database-backed oracle validation
  oracle_endpoint_payload.cpp      endpoint evidence lookup, external replay,
                                   shared cache reuse, and live materialization
  oracle_material_point.cpp        material-point occupied-certificate helper
                                   for oracle validation
  oracle_node.cpp                  node topology, canonical query intervals,
                                   and native interval reflection
  oracle_session.cpp               worker oracle sessions, temporary worker DB,
                                   structure replay, and evidence replay
  oracle_options.h                 oracle-local debug-print accessors
  oracle_support.cpp               oracle-local hashing, timing, blocker,
                                   cache-key, and counter helpers
tools/                             database CLI and benchmark
tests/                             core, cache, and adapter tests
```

### `safe_box_forest`

Primary target: `SBF::core`.

Responsibilities:

- `RBFPlanningForest` facade for build, query, and coverage build; archived
  dynamic-maintenance and debug entry points are diagnostic-only;
- grower stage, free-box search, graph construction, merger, connector, and
  corridor query;
- runtime configuration and stage execution helpers;
- Python package `sbf` and scene helper APIs.

Important directories:

```text
include/SBF/                       planner public API
src/                               planner implementation
python/sbf/                        Python facade and scene helpers
tests/                             C++ and Python coverage
```

Python binding ownership:

```text
python/bindings.cpp                pybind11 module registration only
python/binding_adaptive_types.h    adaptive leaf-sweep/refine config and
                                   result registration
python/baseline/binding_baseline_planner_functions.h
                                   baseline planner registration aggregator only
python/baseline/binding_baseline_bitstar_functions.h
                                   robot and C-space BIT* baseline/trace
                                   function registration
python/baseline/binding_baseline_cspace_functions.h
                                   C-space RRTConnect and PRM baseline
                                   function registration
python/baseline/binding_baseline_prm_functions.h
                                   robot-space PRM, PRM*, and LazyPRM
                                   multi-query function registration
python/baseline/binding_baseline_rrt_functions.h
                                   path length, in-tree RRTConnect, and OMPL
                                   RRTConnect function registration
python/baseline/binding_baseline_utility_functions.h
                                   OMPL path simplification and direct
                                   configuration-collision utility registration
python/binding_basic_types.h       robot, interval, canonical helper, and
                                   common enum registration
python/binding_planner_core_types.h
	                                   envelope, graph node/edge, runtime,
	                                   and leaf-sweep type registration
python/binding_planning_forest_database_methods.h
	                                   RBFPlanningForest LECT database, snapshot,
	                                   checkpoint, and prewarm method registration
python/binding_planning_forest_build_methods.h
	                                   RBFPlanningForest build, coverage, and
	                                   leaf-sweep method registration
python/binding_planning_forest_query_methods.h
	                                   RBFPlanningForest query, bridge, endpoint,
	                                   and offline-shortcut method registration
python/diagnostic/binding_diagnostic_types.h
	                                   opt-in dynamic update, subtractive build,
	                                   and rebuild-profile type registration
python/diagnostic/binding_debug_chain_pave_result.h
	                                   chain-pave diagnostic result to Python
	                                   dictionary conversion
python/diagnostic/binding_debug_chain_pave_methods.h
	                                   chain-pave diagnostic method registration
python/diagnostic/binding_debug_endpoint_oracle_methods.h
	                                   endpoint evidence, interval validation,
	                                   and envelope-summary diagnostic method
	                                   registration
python/diagnostic/binding_debug_find_free_box_methods.h
	                                   single-seed FFB/oracle trace diagnostic
	                                   method registration
python/diagnostic/binding_debug_path_cover_methods.h
	                                   path-cover FFB diagnostic method
	                                   registration
python/diagnostic/binding_planning_forest_diagnostic_facade_methods.h
	                                   opt-in diagnostic facade method registration
python/diagnostic/binding_planning_forest_debug_methods.h
	                                   opt-in diagnostic debug method
	                                   registration aggregator only
python/binding_planning_forest_methods.h
	                                   RBFPlanningForest class registration
	                                   aggregator only
python/binding_planning_option_types.h
	                                   split, validation, grower, connector,
	                                   query, database, planner, profile, and
	                                   query-result type registration
python/binding_oracle_utils.h       diagnostic oracle detail/counter conversion
                                   helpers
python/binding_utils.h             Python/Eigen/interval conversion helpers
python/ompl_binding_utils.h        OMPL baseline sampler, seeding, C-space,
                                   planner, and path extraction helpers
python/sbf/diagnostic_exports.py   optional diagnostic Python package exports
python/sbf/__init__.py             production package exports and aliases;
                                   consumes `_diagnostic_exports`
```

Python binding headers follow the same include-boundary rule as C++ headers.
Type/helper bindings that do not instantiate `RBFPlanningForest` include the
narrow public type headers they register instead of the `SBF/sbf.h` facade:
`binding_utils.h` uses `SBF/scene_types.h` for shared Robot/Eigen/interval
conversion helpers; `binding_oracle_utils.h` owns LECT oracle detail/counter
conversion for diagnostic methods. `binding_basic_types.h` uses
`SBF/scene_types.h`, `SBF/query_result.h`,
envelope enum headers, and LECT canonicalization; and
`binding_planner_core_types.h` uses grower, leaf-sweep, runtime, segment-edge,
core, and envelope type headers directly. `binding_adaptive_types.h` uses
adaptive config, query runtime config, and planning result headers directly.
`binding_planning_option_types.h` uses planning config, build/query result, and
LECT split/cache/oracle type headers directly. `diagnostic/binding_diagnostic_types.h`
uses diagnostic build/result headers plus `SBF/planning_config.h` directly for
the guarded `dynamic_update` field. Forest method bindings register the
`RBFPlanningForest` facade and include `SBF/safe_box_forest.h` directly, not
the broader `SBF/sbf.h` package entry. Baseline and OMPL helper bindings also
avoid the package facade: robot-space OMPL wrappers include `SBF/scene.h` for
`Robot`, `Scene`, and `CollisionChecker`, the in-tree RRT wrapper additionally
includes `SBF/connector.h` and `SBF/box_graph.h`, and the pure C-space OMPL
wrappers depend only on the shared OMPL helper layer. The release self-test
enforces these narrow binding headers so the Python layer does not silently
become another monolithic aggregate include path.

The release self-test checks that production forest binding groups do not
contain diagnostic methods or macros, and that diagnostic Python types/methods
are guarded by both `SBF_DIAGNOSTIC_API` and `SBF_PYTHON_DEBUG_METHODS`.
This includes all checked occurrences of `DynamicUpdateConfig`,
`SubtractiveObstacleGroup`, `SubtractiveBuildOptions`, `RebuildProfile`, and
the diagnostic facade method bindings. The Python package root exports those
diagnostic types only through `_diagnostic_exports` from
`sbf.diagnostic_exports`, which is the only package file allowed to probe the
compiled extension for optional diagnostic symbols. It also exposes the planner
root as the stable Python aliases `SBFConfig` and `SafeBoxForest`; the raw
pybind class names remain implementation details of the compiled extension.

Public grower types are split by responsibility:
`include/SBF/grower_types.h` contains target/task/config/result records, while
`include/SBF/grower.h` contains grower interfaces and concrete grower classes.
`grower.h` uses `include/SBF/find_free_box_types.h` for worker result
signatures plus a forward declaration for the FFB service; implementation files
include `SBF/find_free_box.h` locally when they construct or call the service.
Connector public types follow the same pattern:
`include/SBF/connector_types.h` contains RRT, chain-paving, and island-connector
configuration/result records, while `include/SBF/connector.h` contains connector
algorithm entry points and the `IslandConnector` class.
`connector_types.h` depends on `include/SBF/find_free_box_config.h` for
chain-paving FFB options, but it must not include FFB result payloads from
`SBF/find_free_box_types.h` or graph/cache/segment-edge payloads from
`SBF/box_graph_types.h`. `connector.h` owns the narrow graph dependencies
because its algorithm entry signatures take `BoxNode`, `AdjacencyGraph`, and
`SegmentEdgeList`. Detailed chain-pave debug payloads such as
`DebugBoundaryFfbFailure` live in `include/SBF/debug.h`; `connector_types.h`
may only expose guarded diagnostic plumbing fields, and `SBF/debug.h` itself
has an empty default include path: its dependencies and payload types are all
behind `SBF_DIAGNOSTIC_API`. `SBF/safe_box_forest.h` forward declares
diagnostic debug payload return types such as `DebugChainPaveResult`, dynamic
update result payloads such as `RebuildProfile`, and subtractive-build config
records instead of including their payload headers; diagnostic implementations,
Python diagnostic bindings, and compile-time surface tests include the concrete
diagnostic headers explicitly when they inspect fields. Diagnostic-only
`RBFPlanningForest` member declarations live in class-body fragments under
`include/SBF/detail/`: public facade entry points, adaptive-partition update
helpers, dynamic collision cache helpers, and diagnostic cache state are kept in
separate fragments that `SBF/safe_box_forest.h` includes only inside
`SBF_DIAGNOSTIC_API` guards.
Box graph public records follow the same split:
`include/SBF/box_adjacency_types.h` owns `AdjacencyGraph` and adjacency-build
stats, `include/SBF/query_graph_types.h` owns Dijkstra/search result and cost
option records, and `include/SBF/query_graph_cache_types.h` owns
`QueryGraphCache` because that cache depends on both adjacency records and
`SegmentEdgeList`. `include/SBF/box_graph_types.h` is now only a compatibility
aggregate over those narrow headers. `include/SBF/box_graph.h` owns graph
algorithm entry points such as adjacency construction, Dijkstra search, path
extraction, and segment-edge helpers, and it includes the narrow graph record
headers directly rather than the compatibility aggregate. Public type/facade
headers and private helper headers include the narrow graph header that matches
their signatures; implementation files include `SBF/box_graph.h` locally when
they call graph algorithms.
Oracle-facing public algorithm entry headers, including `SBF/connector.h`,
`SBF/grower.h`, `SBF/leaf_sweep_grower.h`, and `SBF/merger.h`, include
`LECTDatabase/sbf/oracle_types.h` for signatures and do not include the full
`SBF/oracle.h` class header. Implementation files include `SBF/oracle.h`
locally when they call concrete oracle methods or need the concrete class
hierarchy.
Public facade and algorithm entry headers that only mention `StageContext&`,
including `SBF/safe_box_forest.h`, `SBF/connector.h`, `SBF/find_free_box.h`,
`SBF/grower.h`, `SBF/leaf_sweep_grower.h`, and `SBF/merger.h`, include
`SBF/runtime_fwd.h` instead of the full runtime execution header.
Implementation files include `SBF/runtime.h` locally when they construct stage
contexts, deadlines, timers, or access stage diagnostics.

SBF CMake source ownership is explicit in
`safe_box_forest/cmake/SBFSources.cmake`. `safe_box_forest/CMakeLists.txt` only
creates targets and attaches `SBF_CORE_SOURCES` or, when the diagnostic API is
enabled, `SBF_DIAGNOSTIC_SOURCES`. Default production sources are grouped by
component:

```text
SBF_GRAPH_PARTITION_SOURCES         graph and adaptive partition data structures
SBF_FFB_GROWER_SOURCES              FFB, RRT grower, frontwave, and leaf sweep
SBF_CONNECTOR_SOURCES               merger, connector, RRT bridge, chain paving
SBF_PLANNING_BUILD_SOURCES          RBFPlanningForest build facade aggregator
SBF_OBB_OVERLAY_SOURCES             OBB, portal, and overlay corridor support
SBF_QUERY_BRIDGE_SOURCES            query bridge, endpoint, HiPaC, batch bridge
SBF_QUERY_RUNTIME_SOURCES           query runtime, repair, shortcut, utilities
SBF_DIAGNOSTIC_SOURCES              archived dynamic/debug/subtractive facade
```

All default top-level source groups above are aggregator-only lists: they should
contain child source-group references, not direct `src/*.cpp` entries. Add new
production implementation files to the most specific child group below, or add a
new child group when a stable responsibility boundary is missing.

`SBF_GRAPH_PARTITION_SOURCES` is an aggregator for the two graph substrates used
by the planner:

```text
SBF_ADAPTIVE_GRID_PARTITION_SOURCES partition-native coverage/query data
                                   structure and overlay support
SBF_BOX_GRAPH_SOURCES               explicit box graph adjacency, path search,
                                   sequence compression, and topology helpers
```

`SBF_FFB_GROWER_SOURCES` is an aggregator for free-box search and growth-stage
implementations:

```text
SBF_FFB_CORE_SOURCES                standalone FFB and BinaryDepth FFB services
SBF_FRONTWAVE_GROWER_SOURCES        frontwave coverage grower
SBF_RRT_GROWER_SOURCES              RRT grower orchestration, roots, frontier,
                                   component connect, tasks, and workers
SBF_LEAF_SWEEP_GROWER_SOURCES       leaf-sweep grower, grouping, diagnostics,
                                   and virtual sweep support
```

`SBF_CONNECTOR_SOURCES` is an aggregator for post-grow graph consolidation and
island connection:

```text
SBF_MERGER_SOURCES                  box containment and merge helpers
SBF_CONNECTOR_CORE_SOURCES          connector orchestration, broadphase, BiRRT,
                                   frontier bridge, and shared connector utils
SBF_CONNECTOR_PAIR_PIPELINE_SOURCES bridge-pair candidate planning, execution,
                                   and deterministic commit
SBF_CHAIN_PAVE_SOURCES              chain-pave public entry and internal helpers
```

`SBF_PLANNING_BUILD_SOURCES` is an aggregator for the public planning facade
and current coverage-build variants:

```text
SBF_PLANNING_FACADE_SOURCES         facade core, build entry, FFB, partition
SBF_PLANNING_ADAPTIVE_SOURCES       adaptive deep leaf-sweep build family
SBF_PLANNING_QROOT_SOURCES          query-root coverage growth helpers
```

`SBF_OBB_OVERLAY_SOURCES` is an aggregator for OBB validation backends and the
planner overlay stages that consume certified OBB regions:

```text
SBF_OBB_VALIDATION_SOURCES          OBB option mapping, geometry, path cover,
                                   stats, diagnostics, and zonotope validation
SBF_OBB_SAMPLED_BACKEND_SOURCES     sampled-support and clearance-sampled OBB
                                   validation backends
SBF_OVERLAY_PORTAL_SOURCES          partition overlays, OBB edge retry, and
                                   HiPaC portal corridor resolvers
```

`SBF_QUERY_BRIDGE_SOURCES` is itself an aggregator. Its children separate the
high-traffic query bridge implementation into stable subresponsibilities:

```text
SBF_QUERY_BRIDGE_CORE_SOURCES       shared options, graph edges, HiPaC, paving
SBF_QUERY_BRIDGE_BATCH_SOURCES      batch task planning and execution policy
SBF_QUERY_BRIDGE_CORRIDOR_SOURCES   corridor graph, sampling, repair, commit
SBF_QUERY_BRIDGE_DIRECT_SOURCES     direct-corridor and residual segments
SBF_QUERY_BRIDGE_ENDPOINT_SOURCES   endpoint anchoring, island, and targets
SBF_QUERY_BRIDGE_RRT_SOURCES        RRT schedules, parallelism, and utilities
```

`SBF_QUERY_RUNTIME_SOURCES` is an aggregator for online query execution and
runtime support:

```text
SBF_QUERY_FACADE_SOURCES            online query entry point and final audit
SBF_QUERY_UTILITY_SOURCES           query geometry, repair, RRT attempts, and
                                   shortcut/hybridization utilities
SBF_QUERY_SHORTCUT_SOURCES          offline shortcut-edge selection and commit
SBF_QUERY_RESULT_SOURCES            QueryResult and path utility helpers
SBF_RUNTIME_INFRA_SOURCES           runtime budgets, deadlines, worker ids, and
                                   diagnostic stage context helpers
```

Do not add dynamic-update, subtractive-build, or debug-only implementation files
to any default source group. They belong in `SBF_DIAGNOSTIC_SOURCES` and are
compiled into `sbf_core` only when `SBF_DIAGNOSTIC_API` is enabled. The release
self-test enforces the aggregator-only and diagnostic source boundaries,
rejects diagnostic helper-header includes from default source
groups, requires any public diagnostic payload include that remains in a
default source file to be behind `SBF_DIAGNOSTIC_API`, rejects dynamic-update
`RebuildProfile` helper implementations from default source groups, and keeps
default-source `SBF_DIAGNOSTIC_API` branches limited to explicit lifecycle
hook files. It scans
`SBF/safe_box_forest.h`, `SBF/build_config.h`,
`SBF/planning_config.h`, `SBF/dynamic_update_config.h`,
`SBF/subtractive_build_config.h`, `SBF/diagnostic_result.h`, and
`SBF/planning_result.h` so diagnostic facade,
config, and result declarations remain inside `SBF_DIAGNOSTIC_API` guards. It
also checks the ordered public `RBFPlanningForest` facade sections.
`check_release_readiness.py --public-tree` runs that release self-test inside
the exported tree, so these boundaries are verified on the artifact that would
be published.
`test_sbf_facade_surface` enforces the default-vs-diagnostic C++ facade/config
surface at compile time.

Current source-file boundaries:

```text
src/query_runtime/runtime.cpp                    runtime budget, deadline, and diagnostics helpers
src/free_box/find_free_box.cpp              seed-to-certified-box search service
src/free_box/find_free_box_binary.cpp       BinaryDepth and virtual sparse FFB search
                                   implementation
src/free_box/find_free_box_internal.h       FFB-local diagnostics and depth-schedule
                                   helpers shared by linear and binary search
src/grower/grower.cpp                     RRT-style forest grower orchestration and
                                   main run loop
src/grower/grower_entry.cpp               grower connectivity helper and public
                                   grower factory
src/grower/grower_sampling.cpp            RRT grower uniform and unexplored-node
                                   sampling helpers
src/grower/grower_commit.cpp              RRT grower FFB result validation, box
                                   creation, and commit checks
src/grower/frontwave_grower.cpp          frontwave coverage grower implementation
src/grower/grower_failure_cooling.cpp     RRT grower failure-cooling and hard-frontier
                                   stop-loss policy
src/grower/grower_frontier.cpp            RRT grower frontier seed selection flow
src/grower/grower_frontier_helpers.cpp    frontier face scoring, face-bin memory,
                                   seed materialization, and trace-face helpers
src/grower/grower_frontier_memory.cpp     `RrtGrower` frontier coverage cache and
                                   face-memory seed de-duplication
src/grower/grower_internal.h              grower-local shared commit, lookup, and
                                   diagnostics helpers
src/grower/grower_options.cpp             RRT grower depth-stage, component-connect
                                   FFB option selection, and FFB/oracle
                                   diagnostics helpers
src/grower/grower_task_builder.cpp        RRT grower task assembly and frontier seed
                                   selection
src/grower/grower_task_filter.cpp         RRT grower task-domain filtering, failure
                                   cooling skips, and task trace emission
src/grower/grower_task_requests.cpp       RRT grower batched target sampling,
                                   request generation, and anchor waves
src/grower/grower_component_connect.cpp   source-root component-connect target
                                   ranking, refinement, and lateral seed
                                   selection
src/grower/grower_component_connect_chain.cpp
                                   component-connect chain execution, staged
                                   targets, and pair-failure bookkeeping
src/grower/grower_component_connect_global.cpp
                                   global component-connect seed candidate
                                   ranking and target selection
src/grower/grower_trace.cpp               optional RRT grower JSON trace output
src/grower/grower_workers.cpp             worker-local FFB sessions, parallel task
                                   execution, and master-node remapping
src/leaf_sweep_grower/leaf_sweep_grower.cpp          leaf-sweep coverage grower orchestration,
                                   group sweep, and common validation helpers
src/leaf_sweep_grower/leaf_sweep_grower_cluster.cpp  obstacle AABB clustering for leaf sweep
src/leaf_sweep_grower/leaf_sweep_grower_compose.cpp  final leaf-sweep free/collision set
                                   composition across obstacle groups
src/leaf_sweep_grower/leaf_sweep_grower_frontier.cpp start-frontier materialization for leaf
                                   sweep
src/leaf_sweep_grower/leaf_sweep_grower_group.cpp    per-obstacle-group sweep loop, checkpoint
                                   advancement, validation, and split dispatch
src/leaf_sweep_grower/leaf_sweep_grower_virtual.cpp  heap-style virtual topology depth and split
                                   helpers for cache-light leaf sweep
src/leaf_sweep_grower/leaf_sweep_grower_internal.h   leaf-sweep internal diagnostics, interval,
                                   box creation, and scoped oracle helpers
src/graph_partition/adaptive_grid_partition.cpp    partition-native coverage/query data structure
src/graph_partition/adaptive_grid_partition_geometry.cpp
                                   pure geometry, distance, interval, and grid
                                   utility routines used by the partition
src/graph_partition/adaptive_grid_partition_indices.cpp
                                   partition runtime index rebuild, incremental
                                   append, hash, and island-update helpers
src/graph_partition/adaptive_grid_partition_overlay.cpp
                                   overlay segment-edge lifecycle and sync
                                   entry points for the partition
src/graph_partition/adaptive_grid_partition_overlay_components.cpp
                                   overlay DSU maintenance and overlay-aware
                                   component queries for the partition
src/graph_partition/adaptive_grid_partition_path_query.cpp
                                   partition-native path search, box sequence
                                   shortcutting, and overlay-edge expansion
src/graph_partition/adaptive_grid_partition_query.cpp
                                   partition point, nearest-box, landmark, and
                                   local interval query helpers
src/graph_partition/adaptive_grid_partition_keys.h
                                   internal grid hash keys for partition merge,
                                   broadphase, and adjacency indices
src/graph_partition/box_graph.cpp                  explicit box adjacency construction and
                                   adjacency build statistics
src/graph_partition/box_graph_edges.cpp            segment-edge, portal-corridor, and
                                   segment-edge adjacency helpers
src/graph_partition/box_graph_query.cpp            query graph cache, point location, and
                                   path-length helpers
src/graph_partition/box_graph_search.cpp           Dijkstra graph search and waypoint
                                   extraction
src/graph_partition/box_graph_sequence.cpp         box-sequence shortcut and bridge-node
                                   compression helpers
src/graph_partition/box_graph_topology.cpp         island and articulation-point graph
                                   topology helpers
src/connector/connector.cpp        island connector orchestration and bridge
                                   round control
src/connector/connector_broadphase.cpp
                                   connector bridge-pair broadphase candidate
                                   generation
src/connector/connector_birrt.cpp  RRTConnect/BiRRT connector path search and
                                   diagnostics
src/connector/connector_entry.cpp  island connector constructors and
                                   `connect_all` overload entry points
src/connector/connector_frontier_bridge.cpp
                                   frontier bridge box insertion and
                                   point-gap fallback helpers
src/connector/connector_internal.cpp
                                   connector-local geometry, incremental graph,
                                   and diagnostics helpers
src/connector/connector_pair_candidates.cpp
                                   bridge-pair round candidate planning,
                                   pruning, ordering, and diagnostics
src/connector/connector_pair_commit.cpp
                                   deterministic bridge-pair commit, chain-pave
                                   accounting, and segment-edge fallback commit
src/connector/connector_pair_tasks.cpp
                                   serial/parallel bridge-pair execution and
                                   successful pair collection for the island
                                   connector
src/connector/connector_chain_pave.cpp
                                   connector chain-pave box insertion along
                                   waypoint paths
src/connector/connector_chain_pave_commit.cpp
                                   chain-pave commit/index bookkeeping,
                                   duplicate-node coverage boxes, and graph
                                   edge synchronization
src/connector/connector_chain_pave_hooks.cpp
                                   no-op/default and opt-in diagnostic hook for
                                   chain-pave boundary-failure debug payloads
src/connector/connector_chain_pave_internal.cpp
                                   production chain-pave boundary seeds,
                                   failure counters, connected-chain stats, and
                                   neutral hook invocation
src/diagnostic/connector_chain_pave_debug.cpp
                                   opt-in DebugBoundaryFfbFailure payload
                                   assembly for chain-pave diagnostics; added
                                   only when `SBF_DIAGNOSTIC_API` is enabled
src/grower/grower_components.cpp          root/component grouping and distance
                                   helpers for RRT grower connectivity
src/connector/merger.cpp           box containment and merge helpers
src/query_runtime/query.cpp                      query result/path utility helpers
src/planning_adaptive/planning_forest_adaptive_build.cpp
                                   adaptive deep leaf sweep orchestration and
                                   frontier validation loop coordination
src/planning_adaptive/planning_forest_adaptive_validation.cpp
                                   adaptive frontier validation sessions,
                                   worker oracle setup, and batch validation
src/planning_adaptive/planning_forest_adaptive_frontier.cpp
                                   adaptive frontier queue ordering, seed-hit
                                   promotion, split-child enqueue, and
                                   planning-domain filtering
src/planning_adaptive/planning_forest_adaptive_commit.cpp
                                   adaptive free-box commit, containment
                                   rejection, and adjacency/partition batching
src/planning_adaptive/planning_forest_adaptive_checkpoint.cpp
                                   adaptive depth checkpoint stop-reason and
                                   next-checkpoint state transitions
src/planning_adaptive/planning_forest_adaptive_fast_candidate.cpp
                                   fast checkpoint scene materialization,
                                   merge, coverage probe, and profile assembly
src/planning_adaptive/planning_forest_adaptive_fast_checkpoint.cpp
                                   fast virtual checkpoint orchestration,
                                   in-sweep callback handling, and profile
                                   selection
src/planning_adaptive/planning_forest_adaptive_connectivity.cpp
                                   adaptive largest-island lookup, local
                                   adjacency checks, and frontier score terms
src/planning_adaptive/planning_forest_adaptive_depth.cpp
                                   adaptive depth readiness gates, snapshot
                                   JSON, and final depth projection
src/planning_adaptive/planning_forest_adaptive_finalize.cpp
                                   adaptive deep leaf sweep result/profile
                                   finalization and diagnostics collation
src/planning_adaptive/planning_forest_adaptive_fixed.cpp
                                   fixed virtual leaf-sweep materialization,
                                   merge, partition rebuild, coverage probe,
                                   and build-profile finalization
src/planning_adaptive/planning_forest_adaptive_cover_utils.cpp
                                   adaptive cover frontier, probe, split, and
                                   scoped oracle-overlap utilities
src/planning_adaptive/planning_forest_adaptive_merge.cpp
                                   budgeted merge orchestration, containment
                                   pruning, and exact-face merge helpers
src/planning_adaptive/planning_forest_adaptive_merge_grid.cpp
                                   LECT-grid line merge and tree-sibling merge
                                   helpers for adaptive build backends
src/planning_adaptive/planning_forest_adaptive_snapshot.cpp
                                   adaptive depth checkpoint coverage and
                                   anchor-probe snapshot evaluation
src/planning_adaptive/planning_forest_adaptive_topology.cpp
                                   adaptive build post-leaf merge, partition
                                   initialization, and initial island state
src/planning_build/planning_forest_build.cpp      build and leaf-sweep build entry points
src/diagnostic/planning_forest_corridor_refine.cpp
                                   opt-in corridor refinement diagnostic entry;
                                   added to `sbf_core` only when
                                   `SBF_DIAGNOSTIC_API` is enabled
src/planning_core/planning_forest_database.cpp   default config, LECT database identity/root
                                   setup, external evidence, and forest
                                   construction
src/diagnostic/planning_forest_debug.cpp
                                   opt-in diagnostic facade entry points,
                                   oracle-counter accessor, and debug payload
                                   assembly; added
                                   to `sbf_core` only when
                                   `SBF_DIAGNOSTIC_API` is enabled
src/planning_core/planning_forest_audit.cpp      path audit, audit checker, and segment-edge
                                   survival helpers shared by forest modules
src/planning_core/planning_forest_diagnostic_hooks.cpp
                                   no-op/default and opt-in diagnostic
                                   lifecycle hooks for dynamic collision-cache
                                   setup, reset, population, and metrics
src/planning_core/planning_forest_core.cpp       `RBFPlanningForest` state reset/cache/core methods
src/planning_build/planning_forest_ffb.cpp        forest/domain-aware FFB entry point,
                                   prechecks, and linear descent fallback
src/planning_build/planning_forest_ffb_binary.cpp forest/domain-aware BinaryDepth FFB
                                   orchestration and materialized fallback
src/planning_build/planning_forest_ffb_binary_sparse.cpp
                                   virtual sparse BinaryDepth FFB probe,
                                   materialization, and diagnostics
src/planning_build/planning_forest_partition.cpp  adaptive-partition rebuild, overlay sync,
                                   and partition-first query helpers
src/planning_build/planning_forest_partition_diagnostics.cpp
                                   adaptive-partition diagnostics for build
                                   profiles
src/query_runtime/planning_forest_shortcut.cpp   offline shortcut-edge selection and audited
                                   shortcut/corridor insertion
src/diagnostic/planning_forest_dynamic_cache.cpp
                                   opt-in dynamic obstacle insertion rebuild;
                                   added to `sbf_core` only when
                                   `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_dynamic_collision_cache_state.h
                                   internal dynamic collision-cache entries,
                                   blocker index, and active-count state used
                                   only when `SBF_DIAGNOSTIC_API` is enabled;
                                   default source files reach it only through
                                   guarded facade helpers and the diagnostic
                                   source-owned deleter
src/diagnostic/planning_forest_dynamic_helpers.cpp
                                   shared dirty-region and local-adjacency
                                   helpers used by opt-in subtractive and
                                   diagnostic dynamic-maintenance code; added
                                   only when `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_dynamic_partition.cpp
                                   opt-in dynamic partition refresh and
                                   RebuildProfile partition diagnostics; added
                                   only when `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_dynamic_remove.cpp
                                   opt-in dynamic obstacle removal and
                                   suffix-removal regrow entry points; added
                                   only when `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_dynamic_refill.cpp
                                   opt-in removed-box leaf-sweep refill helper
                                   for diagnostic dynamic maintenance builds
src/diagnostic/planning_forest_dynamic_segment_endpoint.cpp
                                   opt-in endpoint-specific dynamic segment
                                   fallback diagnostic; added to `sbf_core`
                                   only when `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_dynamic_segment_fallback.cpp
                                   opt-in global dynamic segment-edge fallback
                                   diagnostic; added to `sbf_core` only when
                                   `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_subtractive.cpp
                                   opt-in subtractive scene build, obstacle
                                   carving, certified regrow commit, and
                                   finalization; added only when
                                   `SBF_DIAGNOSTIC_API` is enabled
src/diagnostic/planning_forest_subtractive_seeds.cpp
                                   opt-in subtractive dirty-domain regrow seed
                                   generation and domain lookup helpers
src/overlay/planning_forest_overlay.cpp    partition box/portal overlay corridor helpers
src/overlay/planning_forest_overlay_edges.cpp
                                   partition-first segment edge insertion,
                                   OBB bridge/segment cover, and edge sync
src/overlay/planning_forest_overlay_obb_retry.cpp
                                   clearance-aware RRT retry path generation
                                   for strict OBB bridge replacement
src/overlay/planning_forest_overlay_portal_resolvers.cpp
                                   HiPaC portal chain resolvers using
                                   cell-native validation
src/overlay/planning_forest_overlay_portal_resolvers_ffb.cpp
                                   HiPaC portal chain resolver using legacy
                                   FFB validation
src/obb/planning_forest_obb.cpp       OBB validation option mapping from planner
                                   configuration
src/obb/planning_forest_obb_diagnostics.cpp
                                   OBB cover/portal validation diagnostic
                                   emission shared by overlay stages
src/obb/planning_forest_obb_geometry.cpp
                                   OBB candidate basis construction and
                                   joint-space orientation helpers
src/obb/planning_forest_obb_path_cover.cpp
                                   greedy/recursive path-window covering with
                                   certified OBB regions
src/obb/planning_forest_obb_path_windows.cpp
                                   OBB path-window validation and greedy
                                   window-size search
src/obb/planning_forest_obb_sampled.cpp
                                   sampled-support OBB validation backend and
                                   shared sampled FK helpers
src/obb/planning_forest_obb_sampled_clearance.cpp
                                   clearance-sampled OBB validation backend
src/obb/planning_forest_obb_stats.cpp  OBB validation-stat accumulation and
                                   certified region volume accounting
src/obb/planning_forest_obb_validation.cpp
                                   OBB zonotope portal validation, orientation
                                   fallback, and radius growth/refinement
src/obb/planning_forest_obb_zonotope.cpp
                                   Taylor/zonotope OBB endpoint-envelope
                                   validation
src/query_runtime/planning_forest_query.cpp      online query entry point, strict audit
                                   flow, and final simplify orchestration
src/query_runtime/planning_forest_query_geometry.cpp
                                   query-local interval, box, boundary-seed,
                                   and segment-coverage geometry helpers
src/query_runtime/planning_forest_query_repair.cpp
                                   query path statistics, graph cost options,
                                   and local BiRRT audit repair helpers
src/query_runtime/planning_forest_query_utils.cpp
                                   query/path orchestration utilities shared by
                                   graph, partition, OBB, and bridge stages
src/query_runtime/planning_forest_query_utils_shortcut.cpp
                                   collision-checked shortcut and path
                                   hybridization helpers
src/query_runtime/planning_forest_query_utils_rrt.cpp
                                   audited RRT bridge attempt selection and
                                   parallel attempt diagnostics
src/qroot/planning_forest_qroot_dsu.cpp
                                   query-root growth disjoint-set state and
                                   graph-to-DSU initialization
src/qroot/planning_forest_qroot_helpers.cpp
                                   query-root interval, local-edge, commit, and
                                   dynamic commit-policy helpers
src/qroot/planning_forest_qroot_offline.cpp
                                   offline anchor root growth and associated
                                   query-independent box commits
src/qroot/planning_forest_qroot_spatial_index.cpp
                                   query-root box spatial index and coverage
                                   candidate lookup
src/query_bridge/planning_forest_query_bridge_acceptance_options.cpp
                                   query bridge acceptance-threshold option
                                   parsing, diagnostics, and acceptance checks
src/query_bridge/planning_forest_query_bridge_attempt_paths.cpp
                                   RRT attempt path selection and
                                   hybridization
src/query_bridge/planning_forest_query_bridge_batch_policy.cpp
                                   query bridge batch task policy, edge
                                   ownership, and attempt-plan helpers
src/query_bridge/planning_forest_query_bridge_batch_diagnostics.cpp
                                   query bridge oracle counter deltas, task
                                   skip records, and direct-corridor diagnostic
                                   aggregation
src/query_bridge/planning_forest_query_bridge_batch_finalize.cpp
                                   query bridge batch oracle-counter flush,
                                   partition refresh, and result finalization
src/query_bridge/planning_forest_query_bridge_batch_options.cpp
                                   environment/config parsing for batch bridge
                                   hybridization and batch execution controls
src/query_bridge/planning_forest_query_bridge_task_key.cpp
                                   query bridge per-task diagnostics key helper
src/query_bridge/planning_forest_query_bridge_path_utils.cpp
                                   waypoint path length, shortcut, and internal
                                   simplification helpers shared by bridge
                                   attempts
src/query_bridge/planning_forest_query_bridge_corridor_options.cpp
                                   runtime option parsing for direct corridor,
                                   reusable edge, shortcut, residual, and
                                   detailed-timing controls
src/query_bridge/planning_forest_query_bridge_corridor_diagnostics.cpp
                                   direct-corridor summary and detailed timing
                                   diagnostic emission
src/query_bridge/planning_forest_query_bridge_corridor_commit.cpp
                                   direct-corridor FFB result commit, duplicate
                                   detection, and partition batch append helpers
src/query_bridge/planning_forest_query_bridge_corridor_graph.cpp
                                   direct-corridor sample-assimilation
                                   adjacency candidates and cover-index helpers
src/query_bridge/planning_forest_query_bridge_corridor_samples.cpp
                                   direct-corridor sample coverage, local DSU,
                                   transition metrics, and gap ordering
src/query_bridge/planning_forest_query_bridge_corridor_local_graph.cpp
                                   direct-corridor internal local-graph search,
                                   component slicing, and HiPaC promotion gates
src/query_bridge/planning_forest_query_bridge_corridor_tasks.cpp
                                   direct-corridor uncovered-sample FFB task
                                   generation and task execution loop
src/query_bridge/planning_forest_query_bridge_corridor_repair.cpp
                                   subdivision, adaptive, lateral, and residual
                                   segment repair passes for direct corridor
src/query_bridge/planning_forest_query_bridge_repair_options.cpp
                                   subdivision, adaptive, and lateral repair
                                   option builders for direct corridor paving
                                   audited direct-line fallback option parsing
                                   and path validation
src/query_bridge/planning_forest_query_bridge_direct_segments.cpp
                                   direct start-goal and fast post-RRT segment
                                   edge insertion helpers
src/query_bridge/planning_forest_query_bridge_direct_finalize.cpp
                                   direct-corridor partition refresh and result
                                   finalization helpers
src/query_bridge/planning_forest_query_bridge_edges.cpp
                                   query bridge box-corridor, residual segment,
                                   and waypoint task edge completion helpers
src/query_bridge/planning_forest_query_bridge_endpoint_direct.cpp
                                   direct endpoint-to-main audited segment
                                   insertion helper
src/query_bridge/planning_forest_query_bridge_endpoint.cpp
                                   endpoint anchoring and endpoint-to-main
                                   box-corridor repair
src/query_bridge/planning_forest_query_bridge_endpoint_runtime.cpp
                                   endpoint-to-main runtime box lookup,
                                   partition/graph adjacency, and seed helpers
src/query_bridge/planning_forest_query_bridge_pave.cpp
                                   query bridge chain-pave execution and graph
                                   bridge partition append helpers
src/query_bridge/planning_forest_query_bridge_pave_guard.h
                                   partition-native graph-pave skip guard shared
                                   by query-bridge pave and waypoint stages
src/query_bridge/planning_forest_query_bridge_rrt_options.cpp
                                   query bridge retry and parallel-RRT option
                                   parsing plus option diagnostics
src/query_bridge/planning_forest_query_bridge_rrt_parallel.cpp
                                   query bridge parallel-RRT early-stop
                                   predicates, cancel flag, and diagnostics
src/query_bridge/planning_forest_query_bridge_rrt_schedule.cpp
                                   query bridge RRT attempt profiles, attempt
                                   count planning, and seed scheduling
src/query_bridge/planning_forest_query_bridge_rrt_utils.cpp
                                   query bridge RRT attempt execution, audited
                                   path adoption, and no-path retry stages
src/query_bridge/planning_forest_query_bridge_waypoint.cpp
                                   waypoint-path corridor paving and local
                                   repair implementation
src/query_bridge/planning_forest_query_bridge_batch.cpp
                                   batched query bridge scheduling and per-query
                                   repair accounting
src/query_bridge/planning_forest_query_bridge_task.h
                                   query bridge task/job state shared by batch,
                                   RRT, direct segment, HiPaC, and edge modules
src/query_bridge/planning_forest_query_bridge_*_options.cpp
                                   query bridge typed runtime option adapters
                                   derived from RBFPlanningConfig
src/query_bridge/planning_forest_query_bridge_policy.h
                                   batch-policy and edge-ownership declarations
                                   for query bridge task execution
src/query_bridge/planning_forest_query_bridge_diagnostics.h
                                   per-task key, oracle-counter delta, skip, and
                                   direct-corridor aggregate diagnostic declarations
src/query_bridge/planning_forest_query_bridge_attempt_paths.h
                                   attempt-path adoption and waypoint retry
                                   declarations for the batch scheduler
src/query_bridge/planning_forest_query_bridge_pair.cpp
                                   pair-level query bridge orchestration and
                                   RRT/segment fallback dispatch
```

New production features should normally land in the smallest matching module
above. Large experimental branches should be isolated behind typed
configuration and placed in the matching module instead of reintroducing a
monolithic facade implementation file.

Public planner header ownership follows the same rule. `SBF/safe_box_forest.h`
is the facade for `RBFPlanningForest` and should not accumulate unrelated data
models or pull concrete grower/connector algorithms into every consumer.
The public facade section is intentionally split into build/coverage, query
and bridge, diagnostic-only, and state-accessor groups; release self-tests
check that ordering so archived diagnostic entry points do not drift back into
the default method groups.
Private `RBFPlanningForest` helper declarations are class-body fragments under
`include/SBF/detail/`, grouped by the implementation responsibility they
dispatch to: FFB/query entry, query-bridge direct corridor, adaptive build,
query-bridge batch/HiPaC/paving, topology/partition, OBB/portal overlay, and
query cache. The main facade header keeps only the ordered include slots plus
core state, which makes the production public surface visible without changing
the C++ class layout or forcing a PIMPL boundary.
Facade-visible signatures depend on explicit type-record headers:
`adaptive_leaf_sweep_config.h`, `connector_types.h`,
`find_free_box_types.h`, `leaf_sweep_types.h`, `planning_result.h`,
`query_bridge_config.h`, and `query_runtime_config.h`. Implementation files that construct `IslandConnector`,
`RrtGrower`, `Consolidator`, or call connector/merger entry points include the
full algorithm headers directly. The facade avoids direct LECT storage,
`SBF/build_config.h`, `SBF/query.h`, and broad grower type includes; it uses
type-record includes and forward declarations for exposed cache/database
handles. `SBF/build_config.h` remains a compatibility aggregate only; new code
includes the narrow type header directly. Planner database runtime options live
in `SBF/database_runtime_config.h`; leaf/adaptive build options live in
`SBF/adaptive_leaf_sweep_config.h`; diagnostic dynamic-update options live in
`SBF/dynamic_update_config.h`, subtractive-build options live in
`SBF/subtractive_build_config.h`, and `SBF/diagnostic_build_config.h` remains
a compatibility aggregate for those diagnostic option headers. The planner
facade forward declares diagnostic config/result records and uses explicit
diagnostic overloads rather than default arguments that force full payload
includes into `SBF/safe_box_forest.h`. Runtime execution mode/config
records live in `SBF/runtime_config.h`, while `SBF/runtime.h` owns executors,
deadlines, and stage context helpers. Forward declarations for runtime execution
types live in `SBF/runtime_fwd.h`. Segment-edge enums, the `SegmentEdge`
forward declaration, and the `SegmentEdgeList` alias live in
`SBF/segment_edge_fwd.h`; the complete segment-edge payload record lives in
`SBF/segment_edge_types.h`. Query results and audit status live in
`SBF/query_result.h`, and default build/coverage profiles live in
`SBF/build_profile.h`; `SBF/api.h` remains only a compatibility aggregate for
those narrow type headers. `SBF/sbf.h` remains the public package entry for
downstream examples and external callers; production, binding, and test code
inside the repository include `SBF/safe_box_forest.h` or narrower headers
directly. `SBF/planning_forest.h` remains a legacy alias for
`SBF/safe_box_forest.h`; repository code includes the target header directly.
`SBF/detail.h` remains a legacy compatibility aggregate for full algorithm
headers only; production, binding, and test code must include the specific
headers they use instead. `SBF/planning_config.h` depends on
`SBF/runtime_config.h` rather than pulling runtime execution helpers; online
query options live in `SBF/query_config.h`, query runtime and endpoint-corridor
option records live in `SBF/query_runtime_config.h`, query bridge batch option
records live in `SBF/query_bridge_config.h`, default build/coverage result
payloads live in `SBF/planning_result.h`, which depends directly on
`SBF/build_profile.h` rather than runtime execution helpers or diagnostic
payloads; diagnostic dynamic-update result payloads live in
`SBF/diagnostic_result.h`; facade methods that return isolated debug or
dynamic-update payloads use guarded forward declarations in
`SBF/safe_box_forest.h`, while their diagnostic-only class member declarations
are routed through guarded `SBF/detail/planning_forest_diagnostic_*.inc`
fragments. Code that reads those payloads includes `SBF/debug.h` or
`SBF/diagnostic_result.h` locally. Leaf-sweep
config/result records live in `SBF/leaf_sweep_types.h`, FFB option records live
in `SBF/find_free_box_config.h`, FFB result payloads live in
`SBF/find_free_box_types.h`, and isolated diagnostic types live in
`SBF/debug.h`. Oracle-facing type records live in
`LECTDatabase/sbf/oracle_types.h`; only implementation files or algorithm
headers that call `BoxOracle` or `DatabaseBoxOracle` methods include full
`SBF/oracle.h`. If a new option or result is specific to adaptive coverage, query
bridge, HiPaC, OBB, dynamic updates, or debugging, add it to the matching typed
header rather than extending the facade header directly.
`LectDatabaseRuntimeConfig` uses `LECTDatabase/online_cache/config.h` for the
online-cache option record; it does not include the full
`LECTDatabase/online_cache.h` facade or `OnlineEnvelopeCacheTree` class.
Implementations that construct or query the cache tree include the full cache
header locally.
Basic scene model aliases such as `Robot`, `Obstacle`, and `Interval` are
available through `SBF/scene_types.h`; it also forward declares `Scene` and
`CollisionChecker` for signatures that only pass those objects by reference.
Public type/config headers such as `SBF/leaf_sweep_types.h` and
`SBF/subtractive_build_config.h` include that narrow type facade when they only
store obstacle records. `SBF/dynamic_update_config.h` contains scalar
diagnostic update thresholds and must not include scene records.
`SBF/leaf_sweep_types.h`
depends directly on `rbf/core.h` for `BoxNode`; it must not include
`SBF/box_graph_types.h` because leaf-sweep payloads do not own adjacency,
query-cache, or segment-edge records. `SBF/adaptive_grid_partition_types.h`
uses `SBF/query_graph_types.h` for query cost options and `rbf/core.h` for
interval records; it must not include graph cache or segment-edge payloads. The
full `SBF/scene.h` facade remains for implementation files and public aggregate
facades that construct `Scene` or `CollisionChecker`.
Internal helper headers follow the same ownership rule: include the narrow type
headers they declare against and avoid `SBF/safe_box_forest.h` as a convenience
aggregate. The release self-test rejects private headers that include
`SBF/safe_box_forest.h`, `SBF/planning_config.h`, `SBF/planning_result.h`,
`SBF/build_config.h`, `SBF/query.h`, or `SBF/box_graph.h`; implementation files
include those full definitions locally only when they inspect the corresponding
data. When a helper implementation calls a concrete algorithm entry point, the
corresponding `.cpp` file should include the full algorithm header locally.
Connector private headers follow this rule explicitly: declarations for bridge
pair selection, pair execution, frontier bridging, and BiRRT internals include
`SBF/connector_types.h` plus `SBF/runtime_fwd.h`, explicit narrow type headers
such as `SBF/scene_types.h`, `SBF/box_adjacency_types.h`, or
`SBF/segment_edge_fwd.h`, and local forward
declarations instead of the public `SBF/connector.h` algorithm-entry header;
shared graph helpers such as `connector_internal.h` include
`SBF/box_adjacency_types.h` directly for `AdjacencyGraph`.
Private implementation headers under `safe_box_forest/src` do not include the
compatibility aggregate `SBF/box_graph_types.h`; they include `rbf/core.h`,
`SBF/box_adjacency_types.h`, `SBF/query_graph_types.h`, or
`SBF/query_graph_cache_types.h` directly according to the records they declare.
Private declaration headers that only pass `StageContext` or `StageDiagnostics`
by reference include `SBF/runtime_fwd.h`; helper bodies that call runtime
methods such as `context.diagnostics()` live in implementation files wherever
the call can be kept out of header-only utilities. Private headers under
`safe_box_forest/src` do not include the full `SBF/runtime.h` execution header.
Grower private headers follow the same split: helper declarations include
`SBF/grower_types.h`, `SBF/runtime_fwd.h`, or
`LECTDatabase/sbf/oracle_types.h` according to the records they declare; inline
geometry helpers stay header-only, while helpers that call `StageContext` or
`BoxOracle` methods live in `grower_internal.cpp`. Member implementation
`.cpp` files keep the public `SBF/grower.h` entry header local to the
implementation.
Leaf-sweep grower private headers likewise include `SBF/leaf_sweep_types.h`
and `SBF/runtime_fwd.h` or `LECTDatabase/sbf/oracle_types.h` for declarations;
diagnostic helper bodies and `DatabaseBoxOracle` method calls stay in
leaf-sweep implementation files. The
public `SBF/leaf_sweep_grower.h` class entry uses
`LECTDatabase/sbf/oracle_types.h` for oracle-facing signatures.
Find-free-box private headers and query-bridge corridor helpers use
`SBF/find_free_box_config.h` for options and `SBF/find_free_box_types.h` only
when they also need result payloads, plus `SBF/runtime_fwd.h` or
`LECTDatabase/sbf/oracle_types.h` for oracle declarations; diagnostic helpers
that call `StageContext` live in `find_free_box_internal.cpp`. Virtual sparse
FFB declares against oracle type records in `virtual_sparse_ffb.h` and keeps
full `BoxOracle` method calls in `virtual_sparse_ffb.cpp`. The public
`SBF/find_free_box.h` service entry stays local to implementation files that
construct or define `FindFreeBoxService`.
Query-bridge endpoint/corridor helper headers forward declare
`AdaptiveGridPartition` when they only pass it by pointer or reference; the
implementation files that call partition methods include
`SBF/adaptive_grid_partition.h` locally.
Adaptive grid partition option, result, cell, and stat records live in
`SBF/adaptive_grid_partition_types.h`; graph-partition helper headers and
diagnostic helpers include that type header instead of the full partition class
entry header.
Adaptive coverage helper headers, for example, declare against
`SBF/adaptive_leaf_sweep_config.h`, `SBF/leaf_sweep_types.h`,
`LECTDatabase/sbf/oracle_types.h`, and `rbf/lect_database/split_policy.h`;
only implementations that call `DatabaseBoxOracle` methods include
`SBF/oracle.h`, and only implementations that inspect
`AdaptiveLeafSweepResult` include `SBF/planning_result.h`. Adaptive diagnostic
helper headers forward declare result, merge, and adjacency stat records rather
than including those full result/graph headers. OBB helper headers declare
validation and path-cover signatures against `SBF/adaptive_leaf_sweep_config.h`
and `SBF/scene_types.h`; implementation files include full `SBF/scene.h` when
they construct checkers or scenes. Query runtime and query-bridge helper headers
that only mention `RBFPlanningConfig` by reference forward declare it instead of
including `SBF/planning_config.h`; the implementation files that read config
fields include `SBF/planning_config.h` locally. Query-bridge option/gate and
batch-policy implementation files that do not define `RBFPlanningForest` methods
do not include the planner facade; they include the narrow config/policy headers
they evaluate directly. Query-bridge task records store connector retry profiles
through `SBF/connector_types.h`, not the public connector algorithm entry
header. Query bridge direct-corridor graph/commit helpers include their local
corridor graph, runtime, query-geometry, and qroot helper headers rather than
the planner facade.

The former sidecar prototype tree has been retired. Its useful mechanisms are
now production code in `lect_database` and `safe_box_forest`; new experiments
and implementations should not depend on a parallel workspace.

Integrated mechanisms that previously lived in prototype workspace form now
belong to the main source tree:

- sparse/binary FFB execution belongs in
  `safe_box_forest/src/free_box/find_free_box_binary.cpp`,
  `safe_box_forest/src/planning_build/planning_forest_ffb_binary*.cpp`, and
  `safe_box_forest/src/free_box/virtual_sparse_ffb.*`;
- adaptive-grid sparse indexing belongs in
  `safe_box_forest/src/graph_partition/adaptive_grid_partition_sparse.cpp` and related
  partition modules;
- HiPaC, portal, OBB, direct-corridor, and query-bridge repair behavior belongs
  in the `planning_forest_query_bridge_*` modules and typed
  `RBFPlanningConfig`/`AdaptiveLeafSweepConfig` options;
- LECT evidence, canonical mapping, split policy, and cache reuse optimizations
  belong in `lect_database`, not in planner- or experiment-local sidecars.
- read-snapshot evidence direct/slot lookup helpers belong in
  `lect_database/src/lect_database/read_snapshot_evidence.*`, keeping the
  snapshot facade focused on file loading and public query methods.

The source release tools enforce this rule. If a forbidden prototype sidecar
directory such as `improve_workspace`, `rbf_v2`, `sbf_v2`, or
`sbf-standalone` exists anywhere in the source tree, release export and
readiness checks fail; the correct fix is to migrate the code into the owning
module and delete the sidecar directory.

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

The cleanup decision is to refactor in place and generate clean release trees
with `scripts/export_public_release.py` when a separate public/source snapshot
is needed. Do not create a second long-lived implementation directory inside the
repository. A parallel "clean" tree would split CI, experiment reproduction,
Python bindings, and public release tooling across two implementations. The
maintainable path is to keep the current module layout, archive non-current
diagnostic branches, and separate production responsibilities inside
`safe_box_forest`.

Current cleanup priorities:

1. Keep `RBFPlanningForest` as the stable public facade and keep production
   code in responsibility-oriented modules. Do not reintroduce a monolithic
   `safe_box_forest.cpp`; remaining large modules should be split only when a
   clearer responsibility boundary is available. Keep `RBFPlanningForest`
   private helper declarations in responsibility-named
   `SBF/detail/planning_forest_private_*.inc` fragments, and keep
   diagnostic-only declarations in guarded
   `SBF/detail/planning_forest_diagnostic_*.inc` fragments, so the default
   facade view stays focused on production build/query/state responsibilities
   without changing the public or diagnostic API surfaces.
2. Treat environment-variable controls as temporary experiment overrides.
   Production behavior should come from typed config structs or named
   experiment profiles; debug-only `RBF_*` switches should not define the
   default algorithm. Release readiness rejects direct core `getenv` reads
   outside the documented allowlist in `scripts/check_release_readiness.py`;
   current exceptions are centralized oracle debug toggles and logging setup.
3. Keep archived diagnostic methods out of the default facade and default
   `sbf_core` source list. C++ diagnostic `RBFPlanningForest` methods and their
   implementation file are opt-in through `SBF_DIAGNOSTIC_API` or the workspace
   `RBF_SBF_DIAGNOSTIC_API` bridge. Python `debug_*` bindings are a second
   opt-in layer through `SBF_PYTHON_DEBUG_METHODS` or
   `RBF_SBF_PYTHON_DEBUG_METHODS`, and automatically enable the C++ diagnostic
   API for that build. Keep the release self-test and facade-surface test in
   sync with this boundary whenever public methods, config fields, or binding
   headers move.
4. Keep paper-facing runners in top-level `experiments/`. Historical or
   diagnostic scripts must live only in private archive paths excluded from the
   public export, and not be required by current tables or figures unless the
   paper asset manifest records them explicitly as archived source artifacts.
   The release self-test rejects retired Exp.7/Exp.8 entries in the active
   TRO dispatcher and rejects the retired dynamic-update table from the active
   paper asset pipeline or active generated paper artifacts.
5. Keep cache, canonicalization, and oracle semantics inside `lect_database`.
   Planner and experiment layers should pass native joint-space inputs and
   receive native boxes/paths.
6. Prefer small, behavior-preserving extractions before algorithm changes. A
   refactor is acceptable only when the existing CTest suite, public release
   checks, and paper provenance checks still pass.

Remaining cleanup targets:

- C++ `RBFPlanningForest` still owns diagnostic implementations for archived
  private scripts, but those declarations and Python bindings are now opt-in.
  A narrower non-member diagnostic facade can replace the member functions once
  archived callers are no longer useful.
- `safe_box_forest/src` is already split into responsibility-oriented files and
  CMake source groups. Adaptive-build internals remain a broad family; further
  splits should follow stable subresponsibilities such as checkpointing,
  frontier scheduling, validation, commit/finalization, and diagnostic
  aggregation.
- The only direct environment-variable reads in the core modules should remain
  in local option/debug helper files. Any new production behavior must be added
  as typed config or a registered experiment profile instead of another
  implicit environment switch.

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
