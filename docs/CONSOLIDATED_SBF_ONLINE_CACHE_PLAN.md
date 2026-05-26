# Consolidated SBF + LECTDatabase Execution Plan

## Goal

Build one complete GitHub repository that contains `SafeBoxForest`, `link-interval-envelope`, and `LECTDatabase`, while removing the dependency on the original `LECT` package. The final runtime architecture is:

1. `LECTDatabase` is a lifelong, disk-backed, scene-independent robot box to envelope/evidence dataset.
2. `LECTDatabase` owns Z4 canonical symmetry handling for storage and query.
3. `OnlineEnvelopeCacheTree` is an in-memory, scene-specific cache tree between `LECTDatabase` and `SafeBoxForest`.
4. `SafeBoxForest` never calls original `LECT` and never talks directly to the disk database; it only calls the online cache tree oracle.

The work must preserve the split tree semantics: the online cache tree uses the same split policy as `LECTDatabase`, and parent envelope evidence is valid only when it equals the union of complete child envelopes.

## Phase A: Consolidated Repository Boundary

### A0. Documentation

- Write this plan into the target repository.
- Keep the status section at the end updated after each implementation slice.

### A1. Repository Layout

Use the existing GitHub repository `tianyu1997/LECTDatabase` as the target repository unless a different remote is explicitly requested.

Target layout:

```text
LECTDatabase/
  CMakeLists.txt
  README.md
  docs/
  cmake/
  lect_database/
  link_interval_envelope/
  safe_box_forest/
  tests/
  tools/
```

Initial consolidation may keep existing root `include/`, `src/`, `tests/`, and `tools/` while adding vendored sibling source directories. The final cleanup can move root database sources under `lect_database/` once build behavior is stable.

### A2. Root CMake

The consolidated root build must expose these targets:

- `link_interval_envelope::core`
- `LECTDatabase::core`
- `LECTDatabase::online_cache`
- `LECTDatabase::sbf_adapter`
- `SBF::core`

The root build must not require `find_package(LECT)` or `LECT::core`.

### A3. GitHub Packaging

- Exclude all build directories and generated artifacts.
- Keep `LECTDatabase` as a git repo with a clean worktree after commits.
- Push every completed milestone to the GitHub remote.

## Phase B: Delete Original LECT Dependency

### B1. CMake Dependency Removal

Remove from SafeBoxForest:

- `SBF_LECT_SOURCE_DIR`
- `find_package(LECT)`
- fallback `add_subdirectory(.../LECT)`
- `SBF_LECT_TARGET`
- `SBF_LECT_INCLUDE_DIR`
- install interface dependency on `LECT::core`

`SBF::core` should link only against the consolidated low-level targets, especially `LECTDatabase::sbf_adapter`, `LECTDatabase::online_cache`, `LECTDatabase::core`, `link_interval_envelope::core`, and `Threads::Threads`.

### B2. Public Header Cleanup

Remove from the active SafeBoxForest public/runtime API:

- `#include <LECT/...>`
- `lect::LECT`
- `LectBoxOracle`
- `LectBoxOracleSession`
- `LectBoxOracleFactory`
- `lect::BestTightenOptions`
- `lect::StorageProfile`

If temporary compatibility is needed for tests, it must be isolated from the main runtime and removed before the phase is complete.

### B3. Runtime Ownership Cleanup

`RBFPlanningForest` must no longer own:

- `std::unique_ptr<lect::LECT>`
- external `lect::LECT` cache
- `LectBoxOracle`

It must own or reference:

- a database runtime/open handle
- a scene-specific online cache tree
- an online-cache-backed oracle

## Phase C: Unified Split Policy

Use `rbf::lect_database::SplitPolicyDescriptor` and `SplitPolicy` as the only split policy surface.

Required changes:

- Replace `OracleSplitOptions::best_tighten` with database split policy fields.
- Make `FindFreeBoxService`, online cache splitting, and `LectDatabase::split_leaf` agree on split dim/value.
- Preserve `min_width`, `midpoint`, `depth_dimensions`, and deterministic tie-break behavior.
- Remove Python bindings and experiment settings for LECT-specific best-tighten fields.

## Phase D: Z4 Canonical LECTDatabase

### D1. Identity

Extend `LectDatabaseIdentity` beyond the existing `canonical_mode` flag:

- add `symmetry_descriptor`
- add `symmetry_hash`
- verify both in `identity_compatible`
- bump schema version if the on-disk format changes

### D2. Canonical Mapping

Reuse `link_interval_envelope/include/sbf/core/joint_symmetry.h`:

- `JointSymmetry::canonicalize`
- `JointSymmetry::map_interval`
- `JointSymmetry::transform_all_endpoint_iaabbs`
- `detect_joint_symmetries`

External full-space box queries must map to canonical intervals plus sector before topology/evidence lookup.

### D3. Storage And Query

- Store topology only in canonical space.
- Use `EvidenceKey::sector` for real Z4 sector semantics.
- Store canonical payload once when possible.
- On non-primary sector query, lazy-load canonical payload and transform it to the requested sector.
- Include canonical/sector semantics in evidence header, sidecar index, checksum, and manifest.

### D4. Dataset Prewarm

Prewarm belongs to `LECTDatabase`, not to SBF:

- topology prewarm expands canonical tree only
- materialization prewarm computes scene-independent endpoint/envelope payloads
- checkpoint writes dataset changes to disk
- SBF build must not full-prewarm the database

### D5. Out-of-Core Evidence

`LECTDatabase` must be able to open large datasets without loading all payloads into memory:

- load manifest and index eagerly
- lazy-load evidence payloads by offset/size
- bound node pages with `max_resident_pages`
- add a payload memory budget such as `max_resident_evidence_bytes`

## Phase E: Online Envelope Cache Tree

### E1. New Target

Add `LECTDatabase::online_cache` under `include/LECTDatabase/online_cache/` and `src/online_cache/`.

### E2. Responsibilities

The online cache tree is scene-specific and in-memory. It must:

- expose a tree/oracle API for SBF
- use the same split policy as `LECTDatabase`
- lazy-load evidence from `LECTDatabase`
- compute envelope online when the dataset has no payload
- opportunistically backfill scene-independent evidence to `LECTDatabase`
- maintain parent envelope equals union of children
- enforce memory limits and avoid OOM

### E3. Node And Payload Model

Node state should track:

- local node id
- canonical database node id
- parent/left/right
- split dim/value
- intervals
- generation
- completeness state

Payload state should track:

- endpoint/link envelope payload
- payload kind
- source/channel/sector
- child_hull flag
- memory cost
- dirty/backfill eligibility

### E4. Lazy Load Flow

`get_envelope` should follow this order:

1. online cache hit
2. database lazy payload lookup
3. online compute through `link_interval_envelope`
4. insert into online cache
5. optional safe backfill to database

### E5. Parent Union Invariant

Only create parent child-hull evidence when:

- both children exist
- both child envelopes are complete
- generations match
- the union operation succeeds

Incomplete parent evidence must not be used as certified collision evidence.

### E6. OOM Strategy

The cache must have configurable limits:

- `max_nodes`
- `max_payload_bytes`
- `eviction_policy`
- `writeback_batch_size`
- `checkpoint_interval_updates`

When limits are exceeded, evict payloads by LRU while preserving the active search path/topology spine when possible. The fallback behavior is lazy reload or recompute, not process failure.

### E7. Sessions

The online cache layer must provide worker sessions for SBF parallel grower/merger:

- read-only sessions can share the database lazy index
- mutable sessions can expand a local subtree
- commit replays split/generation updates into the master cache

## Phase F: SafeBoxForest Adapter And Runtime Wiring

### F1. Adapter Rewrite

`LECTDatabase::sbf_adapter` should expose an online-cache-backed oracle. The adapter may keep the name `DatabaseBoxOracle` temporarily, but internally it must call only `OnlineEnvelopeCacheTree`.

### F2. RBFPlanningForest Wiring

`RBFPlanningForest::reset_oracle` should:

1. ensure the disk dataset is open
2. create a scene-specific online cache tree
3. create the online-cache-backed oracle

Obstacle changes must clear or generation-bump the scene cache.

### F3. Prewarm Rename

Old SBF methods named `prewarm_lect_*` should be removed, renamed, or narrowed to online-cache warmup. Full dataset prewarm lives in `LECTDatabase` tools/API only.

### F4. Python/API Cleanup

Remove bindings for original LECT-specific types. Add bindings for:

- `SplitPolicyDescriptor`
- `LectDatabaseConfig`
- `OnlineEnvelopeCacheConfig`
- online cache stats
- canonical symmetry config

## Phase G: Verification

Run these checks repeatedly until all pass:

1. Configure/build/test consolidated repo with Python disabled.
2. Configure/build/test consolidated repo with Python enabled.
3. Search active SBF code for forbidden LECT symbols.
4. Test LECTDatabase out-of-core page and payload loading.
5. Test Z4 canonical storage/query/evidence transform.
6. Test online cache parent union invariant.
7. Test online cache memory budget eviction/recompute.
8. Test SBF build/query/grower/merger through online cache only.
9. Confirm git status is clean after committed milestones.
10. Push to GitHub.

## Forbidden End State

The task is not complete if any active SBF runtime or public API still depends on:

- original `LECT`
- `LECT::core`
- `lect::LECT`
- `LectBoxOracle`
- `LectBoxOracleFactory`
- `lect::BestTightenOptions`

The task is also not complete if SBF directly calls `LECTDatabase` for envelope data instead of going through `OnlineEnvelopeCacheTree`.

## Current Execution Status

- [x] Plan document created.
- [ ] Consolidated repo skeleton in `LECTDatabase`.
- [ ] Root CMake for consolidated build.
- [ ] SafeBoxForest original LECT dependency removed.
- [ ] Unified split policy surface.
- [ ] Z4 canonical identity/query/storage implemented.
- [ ] Online envelope cache tree implemented.
- [ ] SBF runtime wired through online cache tree.
- [ ] Python/API surface cleaned.
- [ ] Full verification and GitHub push complete.
