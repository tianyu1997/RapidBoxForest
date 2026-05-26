# Implementation Status

This status tracks the standalone SBF plan in `docs/PLAN.md`.

## Planned Features Implemented

- Standalone package boundary under `cpp/SBF`, independent from `cpp/v6`.
- Explicit stage modules: scene/collision, oracle, FFB, grower, merger, connector, query, facade.
- Growers: `RrtGrower` and `FrontwaveGrower`.
- Facade: `SafeBoxForest::build`, `build_coverage`, `query`, `add_obstacle_and_rebuild`, and forest clearing.
- Query layer: box lookup, Dijkstra corridor extraction, shortcutting, waypoint extraction.
- Merger layer: exact face merge, greedy hull merge, containment prune.
- Connector layer: RRT-Connect pair solving, chain paving, island connection.
- Runtime layer: `RuntimeConfig`, `StageContext`, cancellation, deadlines, inline executor, thread-pool executor.
- Parallel extension points for grower, merger, and connector.
- Worker-local LECT oracle sessions with mutable commit/remap and read-only validation sessions.
- Worker-local FFB for RRT and Frontwave batches when safe exclusive domain roots are available.
- Oracle-backed parallel merger validation using read-only LECT sessions.
- Stage diagnostics hooks and facade-level `BuildProfile::diagnostics` snapshots.
- C++ integration tests and Python facade smoke tests.

## Planned Features Deferred

- Copy-on-write LECT snapshots. Current worker sessions use copy-based snapshots and domain transplantation.
- Persistent forest snapshot serialization. The runtime supports deterministic rebuild/query flows, but durable snapshots are not yet a first-class artifact.
- Heavy benchmark/reproduction drivers. The standalone package has smoke tests and a toy bench; paper-scale experiment orchestration remains outside this repo.
- GPU or SIMD envelope kernels. The envelope package remains the owner for low-level geometry acceleration.

## Added Beyond The Original Plan

- Python facade bindings for `Robot`, `SafeBoxForest`, `QueryConfig`, `RebuildProfile`, endpoint/envelope configs, and build/query/rebuild smoke coverage.
- Opt-in CMake package export scaffold for downstream `find_package(SBF)` use with installed dependencies.
- Per-worker read-only merger validation sessions instead of one LECT snapshot per candidate.
- User-facing README quickstarts and a documented parallelism contract.

## Current Validation Command

```bash
SBF_BUILD_EXPERIMENTS=ON BUILD_DIR="$PWD/build_initial" PYTHON_EXECUTABLE=/home/tian/miniconda3/bin/python bash tests/run_all.sh
```
