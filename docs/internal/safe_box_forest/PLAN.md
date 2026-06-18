# Standalone SBF Implementation Plan

This package extracts SafeBoxForest into `cpp/SBF`, independent from `cpp/v6`.
It now consumes the consolidated `LECTDatabase` core, online-cache, and SBF
adapter targets plus `link_interval_envelope::core`.

The implementation goal is a modular, serial-correct core with explicit
extension points for parallel growth, merge validation, island connection, and
worker-local LECT sessions. Serial behavior must remain the default and must
stay deterministic.

## Module Boundary

- `link_interval_envelope` owns robot kinematics, endpoint sources, link
	envelopes, and voxel envelopes. It must remain a pure geometry package.
- `LECTDatabase` owns tree storage, evidence channels, cache materialization, and
	snapshot/worker-session mechanics.
- `SBF` owns planning semantics: scenes, collision checking against obstacles,
	reservation-aware free-box search, grow/merge/connect stages, corridor query,
	and the build/query facade.
- `Scene` and `CollisionChecker` stay in SBF. They are not moved into the
	envelope package because collision requires planning-scene ownership and robot
	obstacle policy, not only geometric envelope construction.

## Implemented Baseline

- `Scene` and `CollisionChecker` for point, box, and segment checks.
- `BoxGraph` adjacency, islands, articulation points, Dijkstra search, waypoint
	extraction, and shortcutting.
- `BoxOracle`, `DatabaseBoxOracle`, `BoxOracleSession`, and
	`DatabaseBoxOracleFactory` over the consolidated database.
- SBF-owned `FindFreeBoxService` with forest reservation semantics.
- `RrtGrower` and `FrontwaveGrower` as separate grower classes.
- `Consolidator` for exact merge, greedy hull merge, and containment pruning.
- RRT-Connect, chain paving, and island connector primitives.
- `CorridorQuery` and a thin `SafeBoxForest` facade.
- Localized obstacle invalidation and adjacency rebuild.
- C++ tests, Python smoke bindings, toy experiment, and v6-independence checks.

## Runtime Layer

Add a small runtime API before adding heavy parallel algorithms:

- `RuntimeConfig`: execution mode, thread budget, deterministic-reduce flag,
	batch sizes, and cancellation/deadline defaults.
- `CancellationToken`: shared cancellation state that can be passed to RRT,
	grower batches, merge validators, and connector tasks.
- `Deadline`: monotonic deadline helper used by long-running stages.
- `TaskExecutor`: common executor interface with an inline executor and a
	simple parallel executor. The executor is a stage dependency, not a global.
- `StageContext`: per-stage view containing runtime config, executor,
	cancellation, deadline, and stage-local diagnostics hooks.

The first runtime implementation keeps all existing defaults serial. Parallel
execution is enabled only when the caller opts in through config or passes a
parallel `StageContext`.

## Oracle Session Layer

SBF stages depend on `BoxOracle`, while parallel workers obtain read-only or
worker-local oracle state through sessions:

- Add `BoxOracleSession` as a temporary oracle handle for one worker task.
- Add `BoxOracleFactory` with a serial borrowed implementation for generic
	oracles.
- Add `DatabaseBoxOracleFactory` for database-backed worker sessions. Each session owns
	a worker database snapshot, a worker `DatabaseBoxOracle`, worker-local materializer
	state, worker-local counters, and a domain-restricted root.
- Read-only sessions validate intervals without touching master materializer or
	counter state.
- Mutable sessions run FFB inside an exclusive LECT domain. On success, the
	master serially commits the worker domain through the database session commit path,
	records the worker-to-master node remap, and only then reserves the accepted
	master node.

This separates API readiness from the harder correctness problem of mutating a
shared LECT tree concurrently. Master graph, box-id, and reservation commits
remain serial.

## Grower Plan

The grower is split into deterministic master logic and parallelizable worker
proposals:

- Master owns sampling, nearest-box selection, graph mutation, box IDs, and
	node reservations.
- Workers receive `GrowTask` values: seed, parent box, root id, task id, and
	FFB options.
- Workers return `GrowWorkerResult`: task id, FFB result, proposed intervals,
	counters, and failure reason.
- Master performs deterministic reduction: accept proposals in task order,
	check adjacency to parent, reserve accepted LECT nodes, assign IDs, and append
	boxes.
- `RrtGrower` and `FrontwaveGrower` both expose context overloads while old
	`grow(seeds)` remains source-compatible.

Current implementation status:

- RRT grower supports opt-in batch task generation. The executor computes
	nearest-parent and face-snapped seed proposals over an immutable box snapshot.
- RRT worker-local FFB is enabled when the oracle is `DatabaseBoxOracle`, a parallel
	context is active, and each task can lease a unique unreserved leaf domain.
	Otherwise the grower falls back to the original serial FFB path.
- Frontwave grower applies the same worker-local FFB path to a frontier box's
	boundary seed batch, with the same deterministic fallback.
- Worker sessions commit in task order; master reservation, adjacency checks,
	box ID assignment, and vector mutation stay serial.

## Merger Plan

The merger is divided into candidate generation, validation, and reduction:

- Candidate generation enumerates exact-face and adjacent-hull merge pairs.
- Validation checks hull safety with `BoxOracle::validate_intervals` and can run
	independently for candidate pairs.
- Reduction is master-serial and conflict-aware: it picks deterministic winners,
	merges boxes, releases retired reservations, and updates metrics.
- Containment pruning stays deterministic and can later receive a parallel
	containment scan when needed.

The public API adds runtime knobs such as `n_threads`, `candidate_batch_size`,
`parallel_threshold`, and `deterministic_reduce` without changing default serial
output.

Current implementation status:

- Greedy hull merge builds the geometric candidate list through the stage
	executor when a parallel context is active.
- Candidate ordering, reservation release, and box mutation remain
	master-serial and deterministic.
- Oracle-backed candidate validation runs in parallel through read-only
	`DatabaseBoxOracleFactory` sessions when a database oracle and parallel context are
	available. Non-LECT oracles keep the serial validation path.

## Connector Plan

The connector has two different mutability levels:

- RRT pair solving is read-only with respect to the forest and can run in
	parallel across candidate island pairs.
- Chain paving mutates boxes, graph, oracle reservations, and box IDs; it stays
	master-serial in the first parallel-ready implementation.

Implementation order:

1. Generate bridge-pair tasks from the current island graph.
2. Solve RRT-Connect tasks through `StageContext` and cancellation tokens.
3. Deterministically choose the first successful task or best-scored path.
4. Run chain paving on the chosen path serially.
5. Rebuild adjacency and repeat until connected or budget is exhausted.

This provides useful parallelism without corrupting graph/oracle state.

## Facade Plan

`SafeBoxForest` owns the default runtime configuration and builds a
`StageContext` for each build/rebuild call.

- Add `SBFConfig::runtime`.
- Add overloads accepting an explicit `StageContext` for advanced callers.
- Forward the context to grower, merger, and connector layers.
- Record stage timings and keep query behavior independent from build runtime.

## Feature Checklist

Required SBF features for the standalone package:

- Independent CMake package, public headers under `include/SBF`, target
	`SBF::core`.
- Scene and collision checking inside SBF.
- Oracle abstraction over LECT and envelope materialization.
- Reservation-aware FFB.
- RRT grower and frontwave grower.
- Box graph, island detection, and corridor query.
- Consolidation/merge passes.
- RRT-Connect bridge solving and chain paving.
- Build/query facade and localized rebuild.
- Runtime/context/cancellation/deadline API.
- Oracle session/factory API.
- LECT-backed worker sessions with domain commit/remap.
- Worker-local FFB for RRT and Frontwave grower batches.
- Parallel read-only oracle validation for merger candidates.
- Parallel-ready grower, merger, and connector task interfaces.
- Python facade bindings and C++ smoke tests.
- Stage diagnostics hooks surfaced through `BuildProfile::diagnostics`.
- Opt-in CMake install/export metadata for downstream `find_package(SBF)` users.
- Lightweight toy experiment.
- Documentation for API, testing, runtime, and module boundaries.

## Validation Roadmap

Each stage must keep the standalone validation command green:

```bash
cd cpp/SBF
SBF_BUILD_EXPERIMENTS=ON \
BUILD_DIR=$PWD/build_initial \
PYTHON_EXECUTABLE=/home/tian/miniconda3/bin/python \
bash tests/run_all.sh
```

Additional checks after API/runtime work:

- Compile a direct caller using old serial APIs.
- Compile a direct caller using `StageContext` overloads.
- Check cancellation/deadline helpers with unit tests.
- Compare serial and inline-context output on the toy robot.
- Check that no `cpp/v6` include or path enters `cpp/SBF/include`, `src`, or
	`python`.

## Deferred Heavy Work

- Copy-on-write LECT snapshots; current worker sessions use safe copy-based
	snapshots plus subtree transplantation.
- Richer benchmark diagnostics and trace export for long experiment runs.
- Persistent forest snapshots beyond the LECT fingerprint cache.
- OMPL/BIT* and GCS baseline wrappers; these remain outside core SBF.
- Heavy Marcucci/UR5/Panda parity experiments.