# IFK_AA / HIFK_AA Integration Plan

Note: this is a historical execution plan. The current public standalone names
have since been simplified to `IFK` and `HIFK`, where `IFK` is the unsplit
AA-FK baseline and `HIFK` is the hierarchical AA-FK family.

This document is the execution baseline for integrating IFK_AA and HIFK_AA
from RapidBoxForest into the standalone link-interval-envelope package.

Scope decisions:

- Public names in link-interval-envelope are `IFK_AA` and `HIFK_AA` only.
- Both new sources are classified as safe.
- The full standalone surface must be updated: C++ core, package-level wrappers,
  Python bindings, CLI, tests, and docs.
- RapidBoxForest-only surfaces remain out of scope, including standalone `HIFK`,
  LECT cache metadata helpers, and grower or split-policy logic.

## 1. Public Endpoint Model Alignment

Update [include/sbf/envelope/endpoint_source.h](include/sbf/envelope/endpoint_source.h)
to add `EndpointSource::IFK_AA` and `EndpointSource::HIFK_AA`, extend
`EndpointSourceConfig` with `hifk_max_depth`, `hifk_n_threads`, and
`hifk_aa_vol_ratio_thresh`, and mark both new sources as safe in:

- `endpoint_source_default_safety()`
- `endpoint_source_name()`
- `source_channel()`
- `kSourceSubstitutionMatrix`

Do not add standalone `HIFK` to the standalone package. The standalone enum
should grow from 5 values to 7 values, not to RapidBoxForest's 8-value model.

## 2. Port AA-FK Core

Add a standalone AA-FK core header and source under:

- [include/sbf/core/](include/sbf/core)
- [src/core/](src/core)

Adapt RapidBoxForest's AA-FK implementation to the standalone include layout
and naming. Keep the low-level module name `aa_fk`, but avoid leaking the
public name `AAFK` into the standalone public API.

## 3. Add IFK_AA / HIFK_AA Endpoint Wrappers

Introduce a public wrapper header and source under:

- [include/sbf/envelope/](include/sbf/envelope)
- [src/envelope/](src/envelope)

Expose:

- `compute_endpoint_iaabb_ifk_aa()`
- `compute_endpoint_iaabb_hifk_aa()`

Reuse RapidBoxForest's HIFK-AA logic for both fixed-depth and adaptive BFS
modes, but map its public AAFK-facing result and source names to `IFK_AA` in
the standalone package.

Then wire both sources into:

- [src/envelope/endpoint_source.cpp](src/envelope/endpoint_source.cpp)
- [CMakeLists.txt](CMakeLists.txt)

## 4. Broaden Package-Level Execution Paths

Update [src/batch.cpp](src/batch.cpp) so `compute_envelope_batch()` no longer
rejects every source except `IFK` and `CritSample`; `IFK_AA` and `HIFK_AA`
should run through the generic one-shot endpoint path without adding new
per-worker caches.

Audit [src/incremental_context.cpp](src/incremental_context.cpp) to confirm the
new sources stay on the existing generic compute path and are not incorrectly
treated as FK-reusable or CritSample-cacheable.

## 5. Expose the New Sources Through Python and CLI

Update:

- [python/bindings.cpp](python/bindings.cpp)
- [python/link_interval_envelope/__init__.py](python/link_interval_envelope/__init__.py)
- [python/link_interval_envelope/cli.py](python/link_interval_envelope/cli.py)

Requirements:

- Bind `EndpointSource.IFK_AA` and `EndpointSource.HIFK_AA`
- Bind the three new config fields
- Accept public names `ifk_aa` and `hifk_aa`
- Accept tuning kwargs and CLI flags for HIFK_AA

## 6. Update User-Visible Docs and Schema

Revise:

- [docs/API.md](API.md)
- [docs/OUTPUT_SCHEMA.md](OUTPUT_SCHEMA.md)

Document:

- `IFK_AA` and `HIFK_AA` as safe endpoint sources
- the new HIFK_AA tuning fields
- that optimized stateful reuse still exists only for `IFK` and `CritSample`

## 7. Extend Automated Coverage

Add C++ tests in [tests/test_link_interval_envelope.cpp](../tests/test_link_interval_envelope.cpp)
for:

- IFK_AA / HIFK_AA dispatch
- safe flags
- source naming
- a structural invariant: `HIFK_AA` with `hifk_max_depth = 0` matches `IFK_AA`

Add Python tests in [tests/test_python_api.py](../tests/test_python_api.py)
for:

- string mapping
- normalized `endpoint.source` values
- new config kwargs
- CLI acceptance
- batch acceptance for the new sources

## 8. Validate End-to-End

Run the existing standalone test harness:

- [tests/run_all.sh](../tests/run_all.sh)

Use a fresh build directory with Python enabled, and add targeted smoke checks
that explicitly exercise:

- `--endpoint-source ifk_aa`
- `--endpoint-source hifk_aa`
- the new HIFK_AA tuning arguments

## Completion Checklist

- [x] Step 1 completed
- [x] Step 2 completed
- [x] Step 3 completed
- [x] Step 4 completed
- [x] Step 5 completed
- [x] Step 6 completed
- [x] Step 7 completed
- [x] Step 8 completed