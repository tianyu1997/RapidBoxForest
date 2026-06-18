# LECTDatabase API

This document tracks the standalone public API surface for:

- `LECTDatabase::core`
- the database core headers
- the SafeBoxForest adapter layer

Current standalone public surface in this phase:

- compatibility include tree under `include/rbf/lect_database*.h`
- umbrella header under `include/LECTDatabase/lect_database.h`
- target `LECTDatabase::core`

This phase intentionally preserves the existing `rbf::lect_database` namespace and header surface while the package is extracted.
