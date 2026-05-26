# Safe/Unsafe Hybrid Coverage SBF Implementation Plan

This document is the phased implementation plan for turning standalone SBF into a coverage-first hybrid planner. The central split is:

- **Safe path**: conservative endpoint source, strict padding, certified boxes.
- **Unsafe path**: fast heuristic endpoint source, relaxed grid padding, provisional boxes, final path audit and local repair.

The current project goal is not to prove every near-obstacle point with a large box. The grower should spend most time producing few, large boxes that cover as much free space as possible and keep the number of islands small. Hard areas are handed to the connector as point-validated segment edges.

## Current Implementation Checkpoint

Implemented in the current SBF branch:

- Phase 0 API: `EndpointSafetyLevel` is stored on endpoint and batch results, exposed through C++/Python bindings, and reported by the Python wrapper.
- Phase 1 API: `GridPadPolicy` with `StrictHalfDiagonal`, `NoExtraPad`, and `Custom` is wired into `GridConfig` and `SparseVoxelGrid` construction. The default remains strict.
- Phase 1 correctness guard: LECT grid lookup now requires matching `safety_pad`, so strict and no-pad grids are not silently reused across modes.
- Phase 2 foundation: `OracleValidationMode` supports `StrictCertificate` and `CoverageHeuristic`. Strict mode rejects unsafe endpoint free evidence; coverage mode can classify it as `ProvisionalFree`.
- Phase 3 foundation: `BoxCommitPolicy` supports `CommitCertifiedOnly`, `CommitProvisionalAllowed`, and `AuditBeforeCommit`. Committed `BoxNode`s carry `safety_status` and `strict_audit_required`.
- Phase 4 foundation: `GrowerConfig.coverage_first_stop_loss` turns repeated hard FFB leaves into hard-frontier stop-loss diagnostics while preserving strict defaults. Build diagnostics now separate certified/provisional committed volume.
- Phase 5 foundation: `SegmentEdge` is a first-class graph artifact. The forest owns segment edges, connector RRT/direct gap successes add segment edges immediately, query traverses box adjacency plus segment edges, and Python exposes segment edge metadata.
- Query post-processing: `QueryConfig.collision_shortcut` provides an opt-in exact collision-checked path shortcut for experiment runs and future Phase 6 audit/repair integration.
- Experiment migration checkpoint: standalone Exp.4 and Exp.6-style entry points live under `cpp/SBF/experiments/paper_04_marcucci_combined.py` and `cpp/SBF/experiments/paper_06_obstacle_rebuild.py`.

Minimal unsafe coverage configuration:

```python
cfg.endpoint_source.source = sbf.EndpointSource.CritSample
cfg.envelope_type.type = sbf.EnvelopeType.Hull_Grid
cfg.envelope_type.grid_config.pad_policy = sbf.GridPadPolicy.NoExtraPad
cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
cfg.validation.enable_grid_refinement = True
cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
cfg.grower.coverage_first_stop_loss = True
cfg.connector.segment_edges_enabled = True
cfg.connector.rrt_segment_edges = True
cfg.query.collision_shortcut = True
```

Strict default behavior remains:

```python
cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
```

## Phase 0: Safety Semantics

**Goal**: make source safety explicit and separate it from LECT storage channels.

Current state:

- `EndpointIAABBResult::is_safe` already exists.
- `source_channel()` maps IFK to channel 0 and all other sources to channel 1.
- That channel mapping is a storage/cache convention and must not be treated as the full safety policy.

Implementation tasks:

1. Add `EndpointSafetyLevel`:
   - `SafeCertified`: can certify boxes when downstream envelope/query is also strict.
   - `UnsafeHeuristic`: can guide coverage, but cannot certify boxes.
   - `SafeAfterAudit`: may be promoted only after an explicit audit or proof.
2. Add helpers:
   - `endpoint_source_default_safety(source)`.
   - `endpoint_result_safety_level(source, is_safe)`.
   - `endpoint_safety_level_name(level)`.
3. Store `safety_level` in `EndpointIAABBResult` and batch envelope outputs.
4. Expose the level in Python bindings and normalized experiment outputs.

Acceptance criteria:

- IFK is `SafeCertified` by default.
- CritSample and MC are `UnsafeHeuristic`.
- Analytical/GCPC are not silently promoted by channel; they remain explicit policy decisions.
- Existing source/channel behavior remains backward-compatible.

## Phase 1: Grid Pad Policy

**Goal**: allow unsafe coverage to avoid the strict voxel safety pad without claiming a certificate.

Current state:

- `SparseVoxelGrid(delta)` defaults to `sqrt(3) * delta / 2` safety pad.
- With `delta=0.04`, this is `0.034641`, which is larger than the clearance of the 177 Marcucci depth-cap point leaves.

Implementation tasks:

1. Add `GridPadPolicy`:
   - `StrictHalfDiagonal`: current default, sound baseline.
   - `NoExtraPad`: removes only the voxel-discretization pad, not link radius.
   - `Custom`: caller-provided pad.
2. Add fields to `GridConfig`:
   - `pad_policy`.
   - `custom_safety_pad`.
3. Route `GridConfig` into `SparseVoxelGrid(delta, ..., safety_pad)` in `compute_link_envelope()`.
4. Expose policy and custom pad in Python bindings and wrappers.
5. Add metadata in experiment outputs: `safety_pad` already reports the effective pad; outputs should also report the policy.

Acceptance criteria:

- Default behavior stays strict and unchanged.
- Unsafe configurations can set `NoExtraPad`.
- Any no-pad grid result is metadata-marked and must not be used as certified free by itself.

## Phase 2: Dual Oracle Modes

**Goal**: split validation into strict certification and fast coverage classification.

Implementation tasks:

1. Add `ValidationMode` or equivalent:
   - `StrictCertificate`.
   - `FastCoverage`.
2. Add validation detail fields:
   - source, safety level, pad policy.
   - reason code: AABB free, sub-AABB free, grid free, grid overlap, hard unknown, occupied.
   - overlap link/obstacle pair and grid voxel count when available.
3. Implement:
   - `validate_node_safe()` using safe source and strict pad.
   - `validate_node_unsafe_fast()` using unsafe source and relaxed pad.
4. Preserve `validate_node()` as the compatibility entry point, dispatching based on config.

Acceptance criteria:

- Strict mode never promotes unsafe evidence to certified free.
- Fast mode can produce `ProvisionalFree` but records `strict_audit_needed`.
- FFB traces can explain why a leaf failed or was provisional.

## Phase 3: FFB Commit Policies

**Goal**: allow the grower to commit provisional boxes while retaining strict-mode behavior.

Implementation tasks:

1. Add `CommitPolicy`:
   - `CommitCertifiedOnly`.
   - `CommitProvisionalAllowed`.
   - `AuditBeforeCommit`.
2. Extend `FindFreeBoxResult` with:
   - `box_status`.
   - `strict_audit_needed`.
   - terminal leaf intervals and validation reason.
3. Keep `found` semantics backward-compatible for certified-only runs.

Acceptance criteria:

- Existing tests and default strict behavior continue to work.
- Coverage mode can create provisional boxes and mark them clearly.

## Phase 4: Coverage-First Grower

**Goal**: maximize free-space coverage with few large boxes and few islands under a time budget.

Implementation tasks:

1. Add grower mode/config:
   - `CoverageFast` default for the hybrid pipeline.
   - `CertificateOnly` for strict experiments.
2. Add hard-frontier stop-loss:
   - repeated hard leaves are cached by root/component pair and node/face.
   - hard pairs are sent to connector queue instead of increasing depth indefinitely.
3. Adjust frontier scoring:
   - prefer large-volume expansion.
   - prefer island-reducing faces.
   - penalize hard-frontier cache hits.
4. Report metrics:
   - certified volume.
   - provisional volume.
   - mean/max box volume.
   - island count and largest island ratio.
   - hard frontier count.

Acceptance criteria:

- In fixed-time Marcucci runs, coverage mode produces fewer/larger boxes and fewer islands than the depth-chasing baseline.
- It no longer spends most of its time on point-scale hard leaves.

## Phase 5: Segment Edges In The Graph

**Goal**: make connector outputs first-class graph edges.

Implementation tasks:

1. Add `SegmentEdge` metadata:
   - id, source box, target box.
   - waypoints.
   - edge type: point-validated, RRT, chain-pave, repair.
   - checker resolution and validation status.
2. Keep existing adjacency for compatibility, but add a segment edge store.
3. Update connector:
   - if RRT succeeds, add a segment edge immediately.
   - chain-pave becomes opportunistic and records intermediate boxes if created.
4. Update query to traverse both box adjacency and segment edges.

Acceptance criteria:

- Segment edges reduce hybrid island count.
- Query paths can include segment edges.
- Segment metadata is available in build/query results.

## Phase 6: Path Audit And Local Repair

**Goal**: make final output safe even when the build used unsafe coverage.

Implementation tasks:

1. Add `PathAuditor`:
   - certified box edges pass quickly.
   - provisional boxes are checked with strict IFK validation or dense samples.
   - segment edges are checked with configured/adaptive `check_segment` resolution.
2. On failure:
   - locate the bad box or segment bracket.
   - try strict local FFB.
   - if hard, run local RRT repair and replace the edge.
3. Report decomposition:
   - certified box length.
   - audited provisional length.
   - segment length.
   - repair count.
   - remaining assumptions.

Acceptance criteria:

- Returned hybrid paths pass strict collision audit.
- Audit failures trigger local repair or a clear failure reason.

## Phase 7: Experiments And Defaults

**Goal**: validate the architecture and switch defaults carefully.

Experiment modes:

1. `CertificateOnly`: IFK, strict pad, certified boxes only.
2. `CoverageFast`: CritSample, no extra grid pad, provisional boxes, segment connector.
3. `HybridStrictReturn`: CoverageFast build plus strict audit/repair before returning paths.

Metrics:

- build time.
- box count.
- mean and max box volume.
- certified/provisional volume.
- island count.
- segment edge count.
- query success.
- audit failure count.
- repair success.
- box/segment path decomposition.

Regression anchors:

- Marcucci 177 depth-cap q-point leaves.
- Dense path root-only runs.
- Five-query hybrid build/query profile.

## Implementation Order

1. Phase 0 and Phase 1 public API, bindings, and docs.
2. Phase 2 oracle diagnostics without changing default behavior.
3. Phase 3 provisional commit support behind an opt-in config.
4. Phase 4 coverage-first grower heuristics.
5. Phase 5 segment edge graph integration.
6. Phase 6 path auditor and local repair.
7. Phase 7 benchmark sweep and default switch.
