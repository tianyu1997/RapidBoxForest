# RapidBoxForest

This repository root is the RapidBoxForest workspace-style monorepo for three first-party modules:

- `link_interval_envelope/`: envelope and incremental-FK primitives.
- `lect_database/`: database, online cache, and the SBF adapter layer.
- `safe_box_forest/`: planner and query pipeline built on `LECTDatabase`.

Dependency direction is intentional and one-way:

`safe_box_forest -> lect_database -> link_interval_envelope`

The root does not own implementation code for any one module. It only provides a
single top-level CMake entry point that can configure the modules together in
dependency order.

## Layout

```text
.
|-- CMakeLists.txt
|-- .gitignore
|-- docs/
|-- link_interval_envelope/
|-- lect_database/
`-- safe_box_forest/
```

## Root Build

```bash
cmake -S . -B build \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=ON \
  -DRBF_BUILD_SBF=ON \
  -DRBF_BUILD_TESTS=ON \
  -DRBF_WITH_PYTHON=OFF

cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Each module still keeps its own package-facing CMake entry and documentation.