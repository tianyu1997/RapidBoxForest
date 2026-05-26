# SBF API

Primary include:

```cpp
#include <SBF/sbf.h>
```

Public CMake target:

```cmake
target_link_libraries(my_target PRIVATE SBF::core)
```

Main modules exposed through `sbf.h`:

- `sbf::Scene`, `sbf::CollisionChecker`
- `sbf::RuntimeConfig`, `sbf::StageContext`, `sbf::TaskExecutor`
- `sbf::BoxOracle`, `sbf::DatabaseBoxOracle`
- `sbf::BoxOracleSession`, `sbf::BoxOracleFactory`,
	`sbf::DatabaseBoxOracleFactory`
- `sbf::FindFreeBoxService`
- `sbf::RrtGrower`, `sbf::FrontwaveGrower`
- `sbf::Consolidator`
- `sbf::rrt_connect`, `sbf::chain_pave_along_path`, `sbf::IslandConnector`
- `sbf::CorridorQuery`
- `sbf::SafeBoxForest`

`SafeBoxForest` is intentionally a thin facade. Advanced callers can instantiate
the grower, merger, connector, and query components independently.

The package reuses base types from the standalone envelope package, including
`sbf::Robot`, `sbf::Interval`, `sbf::Obstacle`, and the endpoint/envelope config
types.

## Configuration Graph

`sbf::SBFConfig` is the main public configuration root. It contains:

- `grower`: `GrowerConfig`
- `merger`: `MergerConfig`
- `connector`: `IslandConnectorConfig`
- `query`: `QueryConfig`
- `endpoint_source`: `EndpointSourceConfig`
- `envelope_type`: `EnvelopeTypeConfig`
- `validation`: `OracleValidationConfig`
- `database`: `LectDatabaseRuntimeConfig`
- `runtime`: `RuntimeConfig`
- `dynamic_update`: `DynamicUpdateConfig`
- feature toggles: `enable_merger`, `enable_connector`,
	plus database/runtime options for replay, checkpointing, and online-cache size

The default `SBFConfig()` constructor initializes the paper-facing `SBF-SH`
profile.

High-value nested config types:

- `GrowerConfig`: grower mode, find-free-box options, root sampling, component
	connect heuristics, failure cooling, frontwave staging, tracing, and worker
	local FFB behavior.
- `MergerConfig`: merge strategy, round limits, score threshold, and parallel
	scan settings.
- `IslandConnectorConfig`: RRT bridge settings, chain paving policy,
	frontier-bridge heuristics, segment-edge generation, and parallel bridge-pair
	solving knobs.
- `QueryConfig`: nearest-box lookup, shortcutting, strict path audit, and local
	repair controls.
- `OracleValidationConfig`: strict-vs-coverage validation mode, grid
	refinement, validation caching, endpoint cache policy, and external evidence
	reuse.
- `DynamicUpdateConfig`: dirty-region detection, local regrow limits, and warm
	rebuild fallback thresholds.

## Facade: `sbf::SafeBoxForest`

Constructor:

```cpp
sbf::SafeBoxForest forest(robot, config);
```

Public build and query methods:

- `build(start, goal, obstacles)`
- `build(start, goal, obstacles, context)`
- `build_coverage(obstacles, seeds)`
- `build_coverage(obstacles, seeds, context)`
- `query(start, goal)`
- `refine_query_corridor(start, goal, max_boxes_to_add)`
- `bridge_query(start, goal)`
- `bridge_query_known_needed(start, goal)`

Dynamic update and maintenance methods:

- `add_obstacle_and_rebuild(obstacle)`
- `remove_obstacle_and_regrow(obstacle_index)`
- `remove_obstacle_suffix_and_regrow(target_obstacle_count)`
- `clear_forest()`

State accessors:

- `boxes()`
- `raw_boxes()`
- `adjacency()`
- `segment_edges()`
- `last_build_profile()`
- `lect()`

`SafeBoxForest::build()` runs the active pipeline in this order:

1. Reset scene and oracle.
2. Grow raw boxes with the configured grower.
3. Optionally consolidate boxes with `Consolidator`.
4. Optionally connect disconnected islands with `IslandConnector`.
5. Rebuild final adjacency and save the LECT cache when enabled.

## Result Types

### `BuildProfile`

`BuildProfile` reports:

- stage timings: `total_ms`, `grow_ms`, `merge_ms`, `connector_ms`,
	`adjacency_ms`
- box counts: `raw_boxes`, `final_boxes`, `bridge_boxes_added`
- segment-edge counts: `segment_edges`, `segment_edges_added`,
	`rrt_segment_edges_added`, `point_gap_segment_edges_added`
- connector summary: `connector_attempted_pairs`, `connector_connected`,
	`adjacency_islands`
- arbitrary per-stage diagnostics in `diagnostics`

### `RebuildProfile`

Returned by the dynamic obstacle update methods. It tracks boxes/obstacles
before and after the update, dirty-region usage, regrow attempts, warm rebuild
fallback, and stage timings.

### `QueryResult`

Returned by `query(...)`. It includes:

- success and start/goal box ids
- `box_sequence` and `segment_edge_sequence`
- geometric `path` plus `path_length`
- audit and repair outcomes (`audit_status`, `audit_passed`,
	`repair_count`, `failed_segment_index`)
- path-quality counters such as `certified_box_length`,
	`provisional_audited_length`, `segment_edge_length`, and
	`remaining_unsafe_assumptions`

## Runtime And Stage Context

All heavy stages keep their original serial APIs and also accept an explicit
`StageContext` for cancellation, deadlines, diagnostics, and safe parallel
execution:

```cpp
sbf::RuntimeConfig runtime;
runtime.mode = sbf::ExecutionMode::Parallel;
runtime.n_threads = 4;
runtime.deterministic_reduce = true;

sbf::StageContext context(runtime, sbf::Deadline::after_ms(5000.0));
auto profile = forest.build(start, goal, obstacles, context);
```

Key runtime types:

- `ExecutionMode::{Inline, Parallel}`
- `RuntimeConfig`
- `CancellationToken`
- `Deadline`
- `TaskExecutor`, `InlineExecutor`, `ThreadPoolExecutor`
- `StageDiagnostics`
- `ScopedStageTimer`
- `StageContext`

The default constructors keep serial behavior. Passing a parallel context only
enables parallel paths in stages that have a safe read-only task split; mutable
forest reductions remain deterministic and master-owned.

Implemented safe parallel splits:

- `RrtGrower` can batch proposal generation. Nearest-parent and snapped-seed
	proposal work runs over an immutable box snapshot. With `DatabaseBoxOracle`, each
	batch task can also run FFB inside a worker-local database session leased from a
	unique leaf domain; commits are reduced serially in task order.
- `FrontwaveGrower` can run a frontier box's boundary-seed FFB tasks through the
	same worker-local LECT sessions.
- `Consolidator` can parallelize greedy merge candidate scanning and, with
	`DatabaseBoxOracle`, read-only oracle validation through worker-local sessions.
	Accepted-merge commits stay serial.
- `IslandConnector` can parallelize RRT bridge-pair solving; chain paving stays
	serial because it mutates boxes, adjacency, and reservations.

## Oracle Boundary And Sessions

`BoxOracle` is the stage boundary for LECT/materializer/collision access.
Important public oracle-side types:

- `BoxOracle`
- `OracleValidationMode`
- `OracleValidationConfig`
- `OracleValidationDetail`
- `OracleCounters`
- `OracleSplitOptions`
- `OracleSessionConfig`
- `BoxOracleSession`
- `BorrowedBoxOracleSession`
- `BoxOracleFactory`
- `BorrowedBoxOracleFactory`
- `DatabaseBoxOracle`
- `DatabaseBoxOracleSession`
- `DatabaseBoxOracleFactory`

The session layer provides temporary worker-local oracle state:

```cpp
sbf::BorrowedBoxOracleFactory factory(oracle);
auto session = factory.make_session({.worker_id = 0, .read_only = true});
auto& worker_oracle = session->oracle();
```

`DatabaseBoxOracleFactory` returns database sessions with worker-local cache and
counter state:

```cpp
sbf::DatabaseBoxOracleFactory factory(database_oracle);
sbf::OracleSessionConfig config;
config.worker_id = 0;
config.read_only = false;
config.domain_root = leased_leaf_node;

auto session = factory.make_session(config);
auto result = sbf::FindFreeBoxService(session->oracle()).find(seed, context, ffb_options);
if (result.found && session->commit()) {
		int master_node = session->map_node_to_master(result.node);
		database_oracle.reserve_node(master_node, box_id);
}
```

Mutable sessions must start from an exclusive leaf domain. `commit()` transplants
that worker domain back into the master database tree and records the node remap.
Read-only sessions do not need commit and are used for parallel merger
validation.

## Stage-Context Overloads

The following APIs have explicit context overloads in C++:

```cpp
auto grow = grower->grow(seeds, context);
auto merge = consolidator.run(boxes, context, protected_ids);
auto bridge = connector.connect_all(boxes, graph, next_box_id, context);
auto path = sbf::rrt_connect(start, goal, checker, robot, context, rrt_config);
auto added = sbf::chain_pave_along_path(path, anchor_id, boxes, oracle, graph,
																				next_box_id, context, pave_config);
auto ffb = find_free_box.find(seed, context, ffb_options);
```

The no-context overloads construct an inline serial context internally.

## Python Package: `sbf`

When `SBF_WITH_PYTHON=ON`, the build copies the package to `${build}/python/sbf`.

Stable top-level exports include:

- low-level and shared types: `Robot`, `Interval`, `DHParam`, `JointLimits`,
	`Obstacle`, `BoxNode`, `SegmentEdge`
- enums and config/result types: `ExecutionMode`, `SplitStrategy`,
	`SplitPolicyDescriptor`, `OnlineEnvelopeCacheConfig`, `LectDatabaseRuntimeConfig`,
	`OracleSplitOptions`, `OracleValidationConfig`,
	`FindFreeBoxOptions`, `GrowerConfig`, `GrowerDepthStage`, `MergerConfig`,
	`RRTConnectConfig`, `IslandConnectorConfig`, `QueryConfig`, `RuntimeConfig`,
	`DynamicUpdateConfig`, `SBFConfig`, `BuildProfile`, `RebuildProfile`,
	`QueryResult`, plus the shared endpoint/envelope config types
- main facade: `SafeBoxForest`
- free functions: `check_config_collision`, `path_length`, `rrt_connect_path`,
	`ompl_rrt_connect_path`, `ompl_prm_multiquery`, `ompl_simplify_path`,
	`ompl_bitstar_path`, `ompl_bitstar_trace`
- Marcucci helpers re-exported from `python/sbf/marcucci.py`: `ANCHORS`,
	`QueryPair`, `iiwa14_robot_json`, `load_iiwa14_robot`,
	`make_combined_obstacles`, `make_combined_queries`, `make_coverage_seeds`,
	`make_shelves_obstacles`, `make_bins_obstacles`, `make_table_obstacles`

The Python layer currently centers on the facade and utility functions. It does
not expose the grower/merger/query stage classes as first-class Python objects.

### Python facade example

```python
import sbf

robot = sbf.Robot.from_json("robot.json")
config = sbf.SBFConfig()
config.runtime.mode = sbf.ExecutionMode.Parallel
config.runtime.n_threads = 4

forest = sbf.SafeBoxForest(robot, config)
profile = forest.build([0.0] * robot.n_joints(), [0.1] * robot.n_joints(), [])
result = forest.query([0.0] * robot.n_joints(), [0.1] * robot.n_joints())
```

Public bound `SafeBoxForest` methods mirror the current facade surface:

- `build(...)`
- `build_coverage(...)`
- `query(...)`
- `refine_query_corridor(...)`
- `bridge_query(...)`
- `bridge_query_known_needed(...)`
- `add_obstacle_and_rebuild(...)`
- `remove_obstacle_and_regrow(...)`
- `remove_obstacle_suffix_and_regrow(...)`
- `clear_forest()`
- `boxes()`, `raw_boxes()`, `adjacency()`, `segment_edges()`,
	`last_build_profile()`

## OMPL Convenience Helpers

The Python binding exposes several deterministic helper planners and path tools.
All of them validate dimensions against `robot.n_joints()` and use the same
`Obstacle` scene representation as `SafeBoxForest` itself.

- `rrt_connect_path(robot, obstacles, start, goal, config=RRTConnectConfig(), seed=42)`
	returns a simple waypoint list built by the native non-OMPL RRT-Connect path.
- `ompl_rrt_connect_path(...)` returns a dictionary with `ok`, `reason`,
	`status`, `path`, `nodes`, `checking_resolution`, and elapsed time.
- `ompl_prm_multiquery(...)` builds a reusable PRM roadmap once and answers many
	start/goal queries, returning a per-query result list.
- `ompl_simplify_path(...)` simplifies an existing waypoint path.
- `ompl_bitstar_path(...)` runs BIT* once and returns path plus planner stats.
- `ompl_bitstar_trace(...)` runs BIT* with periodic checkpoints and returns a
	`checkpoints` list for convergence tracking.
- `check_config_collision(robot, obstacles, q)` returns the current collision
	status for a single configuration.

These helpers are meant for comparison and orchestration scripts; the core SBF
pipeline does not depend on OMPL at runtime.