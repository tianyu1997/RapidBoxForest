# SBF Architecture Cleanup Backlog

Status date: 2026-07-08

This document records the unfinished architecture cleanup work for
`safe_box_forest`. It is a handoff backlog for the current in-place refactor,
not a request to create a second implementation tree.

## Working Decision

Continue refactoring in the current repository.

Do not create a long-lived clean sidecar implementation such as `sbf_v2`,
`rbf_v2`, or `improve_workspace`. A sidecar would split CI, Python bindings,
experiment reproduction, release export rules, and paper artifact provenance
across two implementations. The maintainable path is:

- keep `RBFPlanningForest` as the stable public facade;
- archive diagnostic and retired experiment paths behind explicit guards;
- split production implementation files along responsibility boundaries;
- use `scripts/export_public_release.py` when a clean source snapshot is needed.

## Current Refactor State

The major direction is already established:

- `safe_box_forest/src` is organized into responsibility directories such as
  `graph_partition`, `grower`, `connector`, `planning_adaptive`,
  `planning_build`, `planning_core`, `query_bridge`, and `query_runtime`.
- `safe_box_forest/cmake/SBFSources.cmake` owns component source groups.
  `safe_box_forest/CMakeLists.txt` should stay focused on target wiring.
- Default production sources and diagnostic sources are separated. Diagnostic
  facade methods are guarded by `SBF_DIAGNOSTIC_API`; Python debug methods are
  guarded by `SBF_PYTHON_DEBUG_METHODS`.
- Public facade declarations in `SBF/safe_box_forest.h` have been reduced with
  class-body fragments under `SBF/detail/`.
- Python bindings have been split into production, baseline, and diagnostic
  registration files.
- Exp.7/Exp.8 dynamic-update paper artifacts are retired from the active TRO
  plan and should not re-enter active generated paper assets.

Recent cleanup extractions that should be preserved:

- diagnostic Python binding groups under `safe_box_forest/python/diagnostic/`;
- `RBFPlanningForest` private and diagnostic class-body fragments under
  `safe_box_forest/include/SBF/detail/`;
- query bridge direct finalization helper;
- grower entry and sampling helpers;
- chain-pave commit helper;
- adaptive grid overlay component helper;
- adaptive frontier validation session helper.

## Unfinished Work

### 1. Keep source grouping complete

Every new production implementation file must be added to the most specific
source group in `safe_box_forest/cmake/SBFSources.cmake`. Add a new child group
only when a stable responsibility boundary is missing.

Required checks after source movement:

```bash
python3 scripts/self_test_release_tools.py --repo-root .
python3 scripts/check_release_readiness.py --repo-root .
python3 scripts/export_public_release.py --out-dir /tmp/RapidBoxForest-public-architecture-check --force
python3 scripts/check_public_release.py /tmp/RapidBoxForest-public-architecture-check --check-release-tools --run-smoke-dry-run
```

### 2. Finish production file responsibility splits

Use file size only as a signal. Split a file only when the destination has a
clear owner and the public behavior remains unchanged.

Current largest production candidates:

```text
461 safe_box_forest/src/grower/grower.cpp
449 safe_box_forest/src/grower/grower_task_requests.cpp
447 safe_box_forest/src/leaf_sweep_grower/leaf_sweep_grower_group.cpp
444 safe_box_forest/src/connector/connector_birrt.cpp
440 safe_box_forest/src/free_box/find_free_box_binary.cpp
438 safe_box_forest/src/query_bridge/planning_forest_query_bridge_direct_corridor.cpp
434 safe_box_forest/src/obb/planning_forest_obb_validation.cpp
427 safe_box_forest/src/graph_partition/box_graph_search.cpp
426 safe_box_forest/src/qroot/planning_forest_qroot_growers.cpp
426 safe_box_forest/src/graph_partition/adaptive_grid_partition_path_query.cpp
```

Recommended next splits:

- `grower/grower.cpp`: keep it as grower orchestration. Candidate extractions
  are root initialization, depth-stage state, and stop-policy/final diagnostic
  helpers.
- `grower/grower_task_requests.cpp`: separate target category selection from
  request assembly if the component-connect cached-seed logic can keep a narrow
  interface.
- `leaf_sweep_grower/leaf_sweep_grower_group.cpp`: mirror the adaptive
  validation cleanup by extracting virtual-validation session setup and
  checkpoint transition helpers.
- `connector/connector_birrt.cpp`: split only if the BiRRT tree/geometry
  helpers become reusable or obscure the public `rrt_connect` entry point.
- `free_box/find_free_box_binary.cpp`: inspect for a stable split between
  binary-depth traversal, candidate materialization, and validation outcome
  commit before editing.

### 3. Keep diagnostic code out of default surfaces

Archived dynamic-update, subtractive-build, and debug helper code is still
available for regression forensics. It should remain opt-in and must not leak
back into:

- default `sbf_core` source groups;
- default C++ facade surface;
- default Python package exports;
- active TRO paper artifact generation.

Self-test rules that protect this boundary should stay updated whenever a
diagnostic file, binding, config field, or generated paper artifact moves.

### 4. Avoid new implicit runtime switches

Environment-variable controls are temporary experiment overrides. New
production behavior should be represented by typed config structs, named
profiles, or experiment-runner arguments.

Before adding any new `getenv` call, check whether the behavior belongs in:

- `SBFConfig` or a nested config struct;
- a module-local options helper;
- an experiment runner;
- a diagnostic-only path.

### 5. Stabilize public include boundaries

Continue replacing broad public includes with narrow type/config headers when
the signature allows it.

Important invariants:

- public algorithm headers should include `SBF/runtime_fwd.h` when they only
  mention `StageContext&`;
- private helper headers should not include full facade headers unless they
  instantiate or call the facade;
- diagnostic payload headers should stay behind `SBF_DIAGNOSTIC_API`;
- implementation files should include concrete headers locally when they call
  concrete methods.

### 6. Keep paper-facing experiment scope current

The current TRO plan no longer includes Exp.7 and Exp.8. Exp.4 shelf ablation
uses the baseline with the d23 warm cache. Non-baseline groups follow their
registered profile-specific cache assignments: HIFK-5 is the no-cache
contrast; Critical-sample replays its matching d23 cache; and Link-AABB and
SupportHull-only retain IFK-AA+d23. Do not apply a blanket no-cache default to
the non-baseline rows. Changing any of these assignments defines a new study
and must be documented explicitly.

Do not re-add retired dynamic-update tables or runners to active generation
paths. In particular, `paper/generated/tab_tro_dynamic_update.tex` should not
exist as an active generated artifact.

## Validation Gate For Each Cleanup Chunk

Minimum gate for pure documentation changes:

```bash
git diff --check
```

Minimum gate for Python release-tool changes:

```bash
python3 -m py_compile scripts/self_test_release_tools.py
python3 scripts/self_test_release_tools.py --repo-root .
git diff --check
```

Minimum gate for C++ source or CMake source-list changes:

```bash
python3 -m py_compile scripts/self_test_release_tools.py
python3 scripts/self_test_release_tools.py --repo-root .
cmake --build build-architecture-default --target test_sbf_facade_surface -j2
cmake --build build-architecture-diagnostic --target test_sbf_facade_surface -j2
cmake --build build-architecture-python-debug --target _sbf_cpp -j2
ctest --test-dir build-architecture-default -R test_sbf_facade_surface --output-on-failure
ctest --test-dir build-architecture-diagnostic -R test_sbf_facade_surface --output-on-failure
git diff --check
python3 scripts/check_release_readiness.py --repo-root .
python3 scripts/export_public_release.py --out-dir /tmp/RapidBoxForest-public-architecture-check --force
python3 scripts/check_public_release.py /tmp/RapidBoxForest-public-architecture-check --check-release-tools --run-smoke-dry-run
python3 /tmp/RapidBoxForest-public-architecture-check/scripts/check_release_readiness.py --repo-root /tmp/RapidBoxForest-public-architecture-check
```

## Completion Criteria

The cleanup can be considered complete only when current evidence proves all of
the following:

- no long-lived sidecar implementation tree is needed or present;
- production source groups are explicit and release-exported;
- default production builds exclude archived diagnostic paths;
- Python default exports exclude diagnostic-only types and methods;
- remaining large files have either been split along stable responsibility
  boundaries or documented as intentional orchestration files;
- active paper generation references only current TRO experiment artifacts;
- release self-test, facade-surface tests, release readiness, and public export
  checks pass from the current worktree.

