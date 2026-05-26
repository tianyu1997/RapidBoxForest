# LECTDatabase

This module contains the `LECTDatabase` package surface inside the workspace.
It provides the database core, the online cache, and the SafeBoxForest adapter
layer.

Exported targets:

- `LECTDatabase::core`
- `LECTDatabase::sbf_adapter`
- `LECTDatabase::online_cache`

Dependency direction:

`LECTDatabase -> link_interval_envelope`

Downstream planner dependency:

`safe_box_forest -> LECTDatabase`

This module is organised in three layers:

1. Standalone database core.
2. SafeBoxForest adapter layer.
3. SafeBoxForest wiring and regression.

Current status:

- Standalone database core is extracted and builds independently.
- SafeBoxForest adapter target is extracted and tested.
- The adapter owns the shared SafeBoxForest-facing `Scene`, `CollisionChecker`, `DatabaseBoxOracle`, and database-backed worker session support.
- Grid/voxel compatibility fields have been removed from the adapter-facing oracle API.
- Sibling `safe_box_forest` now configures, builds, and passes its current test suite against this module.
- SafeBoxForest now requests worker sessions through the generic oracle seam and reuses this package's scene/collision implementation.
- SafeBoxForest consumes this module through the database-backed oracle and online-cache targets in the integrated workspace build.

Validation snapshot:

```sh
cmake -S . -B build-local
cmake --build build-local -j2
ctest --test-dir build-local --output-on-failure
```
