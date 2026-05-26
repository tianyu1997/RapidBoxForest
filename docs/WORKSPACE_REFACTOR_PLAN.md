# RBF Workspace Refactor Plan

Date: 2026-05-26

## Goal

Turn the former `LECTDatabase` checkout into a coherent RBF workspace whose physical layout, CMake ownership, public API, tests, and documentation all agree on one dependency direction:

```text
link_interval_envelope -> lect_database -> safe_box_forest
```

The root repository must no longer read as "LECTDatabase owns SafeBoxForest and link_interval_envelope". The root is the workspace orchestrator; `lect_database`, `link_interval_envelope`, and `safe_box_forest` are sibling first-party modules.

## Final Repository Shape

```text
RBFWorkspace/
  CMakeLists.txt
  README.md
  .gitignore
  docs/
    WORKSPACE_REFACTOR_PLAN.md
  link_interval_envelope/
  lect_database/
  safe_box_forest/
```

The physical checkout directory has been renamed to `RBFWorkspace`. Build directories created before the rename are treated as disposable; validation uses freshly configured build trees under the new root path.

## Naming Rules

1. Root project name: `RBFWorkspace`.
2. Root documentation should describe a workspace or integrated repo, not the `LECTDatabase` package.
3. The database module directory is `lect_database`.
4. The envelope module directory is `link_interval_envelope` until/unless its public package is renamed separately.
5. The planning module directory is `safe_box_forest`; public compatibility aliases may remain only where they are already part of the Python/API surface.
6. New root CMake options use the `RBF_` prefix. Module-local options may keep their local prefixes (`LIE_`, `LECTDB_`, `SBF_`) inside module CMake files.

## CMake Ownership Plan

1. The root `CMakeLists.txt` is the only integrated entrypoint.
2. Root CMake adds modules in dependency order: `link_interval_envelope`, `lect_database`, then `safe_box_forest`.
3. `lect_database` may be built standalone, but in workspace mode it must consume the already-defined envelope target rather than re-owning the envelope source tree.
4. `safe_box_forest` may be built standalone for development, but in workspace mode it must consume `LECTDatabase::core`, `LECTDatabase::sbf_adapter`, and `LECTDatabase::online_cache`.
5. `safe_box_forest` must not require direct root-level knowledge of the envelope source directory during workspace builds.
6. Old integrated options such as `LECTDB_BUILD_SBF`, `LECTDB_BUILD_PLANNING`, and `SBF_LINK_INTERVAL_ENVELOPE_SOURCE_DIR` must not be part of the workspace path.

## API Cleanup Plan

1. Remove the public online-cache warmup method from C++ and Python surfaces.
2. Remove subtractive-build prewarm fields that only existed to drive the warmup method.
3. Keep online cache behavior demand-driven through normal planning and validation flows.
4. Remove direct experiment calls to `warm_online_cache_bfs`.
5. Keep database/cache configuration surfaces that are still used for runtime capacity, spill, and adapter behavior.

## Split Descriptor And Database Scaffolding Plan

1. Keep a single split descriptor boundary between planning and database/oracle code.
2. SBF should request splits through database/cache abstractions rather than mutating legacy LECT internals.
3. Canonical database headers, source files, tests, and tools live under `lect_database/`.
4. Online cache tree code lives under `lect_database` and is used by SBF through `LECTDatabase::online_cache`.
5. Any legacy standalone `LECT` dependency must not be required for the integrated workspace build.

## Gitignore Plan

1. Root `.gitignore` owns build-tree, CMake, compiled artifact, Python cache, test cache, and editor/system ignores for the whole workspace.
2. Nested `.gitignore` files may keep module-specific generated outputs, but must not hide source, docs, tests, or tracked experiment inputs.
3. Build directories created before the restructure are disposable and should not influence source layout decisions.

## Documentation And Script Plan

1. Root `README.md` describes the workspace architecture and the authoritative configure commands.
2. `lect_database/README.md` describes only the database module.
3. `safe_box_forest/README.md` and `safe_box_forest/docs/TESTING.md` describe workspace-first integration.
4. Test helper scripts should configure from the workspace root when testing integrated behavior.
5. Documentation must not recommend removed warmup APIs or old nested source-dir options.

## Verification Checklist

Run these checks after implementing the refactor. A failed check must be fixed or explicitly documented before the task is considered complete.

1. Structural audit:
   - Root contains `CMakeLists.txt`, `README.md`, `.gitignore`, `docs/`, `link_interval_envelope/`, `lect_database/`, and `safe_box_forest/`.
   - Former package directories `include/`, `src/`, `tests/`, and `tools/` are not present at root; they live under `lect_database/`.
2. Legacy API scan:
   - No source, docs, tests, or experiments reference `warm_online_cache_bfs`.
   - No public subtractive options reference `prewarm_time_budget_ms`, `prewarm_max_nodes`, `prewarm_max_depth`, `split_prewarm_nodes`, or `emit_prewarm_leaves`.
3. Legacy build-option scan:
   - Workspace path does not use `LECTDB_BUILD_SBF`, `LECTDB_BUILD_PLANNING`, or `SBF_LINK_INTERVAL_ENVELOPE_SOURCE_DIR`.
4. Configure/build/test without Python:
   - `cmake -S . -B build-consolidated-sbf-tests -DRBF_BUILD_ENVELOPE=ON -DRBF_BUILD_LECT_DATABASE=ON -DRBF_BUILD_SBF=ON -DRBF_BUILD_TESTS=ON -DRBF_WITH_PYTHON=OFF`
   - `cmake --build build-consolidated-sbf-tests -j2`
   - `ctest --test-dir build-consolidated-sbf-tests --output-on-failure`
5. Python/script syntax checks:
   - `python -m compileall -q safe_box_forest/python safe_box_forest/experiments`
6. Optional Python-enabled integrated validation when dependencies are available:
   - `cmake -S . -B build-consolidated-python -DRBF_BUILD_ENVELOPE=ON -DRBF_BUILD_LECT_DATABASE=ON -DRBF_BUILD_SBF=ON -DRBF_BUILD_TESTS=ON -DRBF_WITH_PYTHON=ON`
   - `cmake --build build-consolidated-python -j2`
   - `ctest --test-dir build-consolidated-python --output-on-failure`

## Completion Criteria

The refactor is complete only when:

1. The physical module layout matches the final repository shape under the `RBFWorkspace` checkout folder.
2. CMake configures, builds, and tests the integrated C++ workspace.
3. Removed warmup APIs are absent from C++, Python bindings, docs, and experiments.
4. Documentation and scripts describe the same workspace model that CMake actually builds.
5. The verification checklist above has been run and the result is recorded in the final response.

## Execution Record

Status on 2026-05-26:

- Physical checkout directory renamed from `LECTDatabase` to `RBFWorkspace`.
- Root stale CMake build directories from the old path were removed and fresh build trees were generated under the new root.
- Root workspace CMake owns integrated module ordering and top-level CTest aggregation.
- Module-local standalone override names now use module-dir wording: `LECTDB_ENVELOPE_MODULE_DIR` and `SBF_LECT_DATABASE_MODULE_DIR`.
- Nested module CI workflows were replaced by a root workspace CI workflow.
- Public online-cache BFS warmup API and public prewarm capacity fields were removed from active SBF C++/Python surfaces.
- `safe_box_forest/tests/run_all.sh` now configures the workspace root and writes its grep output inside the selected build directory.
- Static scans passed for removed warmup identifiers, old integrated build options, and legacy original-LECT build/API symbols.
- Integrated C++ configure/build/CTest passed with `RBF_WITH_PYTHON=OFF` (`6/6` tests).
- Python syntax check passed for `safe_box_forest/python` and `safe_box_forest/experiments`.
- Integrated Python-enabled configure/build/CTest passed with `RBF_WITH_PYTHON=ON` (`8/8` tests).
- The SBF module test helper passed from `safe_box_forest/` with `RBF_WITH_PYTHON=OFF` (`6/6` tests).