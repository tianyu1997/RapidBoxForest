# LECTDatabase

Standalone sibling package for the RapidBoxForest `lect_database` core and the SafeBoxForest adapter layer.

Exported targets:

- `LECTDatabase::core`
- `LECTDatabase::sbf_adapter`

This package is being extracted in three ordered stages:

1. Standalone database core.
2. SafeBoxForest adapter layer.
3. SafeBoxForest wiring and regression.

Current status:

- Standalone database core is extracted and builds independently.
- SafeBoxForest adapter target is extracted and tested.
- The adapter owns the shared SafeBoxForest-facing `Scene`, `CollisionChecker`, `DatabaseBoxOracle`, and database-backed worker session support.
- Grid/voxel compatibility fields have been removed from the adapter-facing oracle API.
- Sibling `SafeBoxForest` now configures, builds, and passes its current test suite against the standalone dependencies.
- SafeBoxForest now requests worker sessions through the generic oracle seam and reuses this package's scene/collision implementation.
- The remaining work is the runtime backend switch inside SafeBoxForest from legacy `lect::LECT` ownership to direct `lect_database::LectDatabase` ownership, after the SBF split-policy options are unified with the database split-policy descriptor.

Validation snapshot:

```sh
cmake --build build-local -j2
ctest --test-dir build-local --output-on-failure
```
