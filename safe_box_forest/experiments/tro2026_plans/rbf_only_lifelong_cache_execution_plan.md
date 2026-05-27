# RBF-Only Lifelong Cache Execution Plan

Date: 2026-05-26

This document freezes the current RapidBoxForest experiment plan for the RBF-only track. It supersedes the older order for this track: the Lifelong/canonical cache mechanism study is executed first, and every later RBF experiment uses that mechanism by default unless the experiment row explicitly says it is a cold-cache, no-cache, or non-canonical ablation.

## Non-Negotiable Scope

- Run only RBF, formerly SBF. Do not run IRIS, OMPL PRM, RRTConnect, BIT*, GCS, or other non-RBF algorithms in this track.
- Use the current RapidBoxForest project under `RapidBoxForest/`, not the legacy standalone repository, except for reading old paper/script design.
- Treat old `paper/sbf_old` numbers as historical context only. New tables must come from newly generated JSON artifacts.
- Use `EndpointSource.IFK` as the public API spelling for the requested IFK_AA source. The implementation dispatches to `compute_endpoint_iaabb_ifk_aa()`.
- Use `SplitStrategy.AAFKVolumeMin` with an explicit depth schedule at least through depth 39.
- Use max FFB depth 40 and FFB start depth 15. In the current code this means `find_free_box.max_depth = 40` and `find_free_box.skip_to_depth = 15`.
- Use canonical lifelong cache by default after the mechanism gate passes.
- Use strict audit success as planner success. Solver/query success without strict audit pass is not paper success.

## Default RBF Configuration

The canonical configuration name is:

```text
rbf_ifk_aa_aafkvol_d40_s15_canonical_lifelong
```

Required fields:

- Endpoint source: `sbf.EndpointSource.IFK`.
- Envelope representation: selected by the appendix sweep; before the sweep, run LinkIAABB/KDOP26/SupportHull candidates.
- Validation: `StrictCertificate` / certified-only commit.
- Database split policy: `sbf.SplitStrategy.AAFKVolumeMin`.
- Split schedule: deterministic `depth_dimensions` for depths `0..39`, plus a stored schedule hash.
- Database identity: `canonical_mode = true`, with a nonempty symmetry descriptor if canonical sector mapping is actually enabled.
- FindFreeBox: `max_depth = 40`, `skip_to_depth = 15`, split reserved and unknown leaves enabled.
- Cache: persistent lifelong cache namespace under `safe_box_forest/outputs/paper/rbf_lifelong_cache/` or an explicit run-local subdirectory.
- Prewarm: d18 lifelong cache is built before the target experiment and reported separately from target build time. Only the target leaf layer is materialized with FK/link-envelope evaluation; upper layers are reconstructed from child-hull propagation during checkpoint.
- Warm-build persistence: Shelf+IIWA evaluation rows must keep the external cache read-only and disable active-database backfill; balanced random-scene rows may backfill newly materialized online evidence into the active database.

## Updated Formal Order

### Gate 0: Implementation And API Smoke

Purpose: make the requested configuration executable before running expensive trials.

Required checks:

- Python import exposes `EndpointSource.IFK`, `SplitStrategy.AAFKVolumeMin`, `SplitPolicyDescriptor.depth_dimensions`, and `LectDatabaseRuntimeConfig.canonical_mode`.
- A generated AAFKVolumeMin schedule has at least 40 entries.
- A database opened with that schedule can reach depth 40.
- `skip_to_depth = 15` is configurable from the experiment runner. If the binding is missing, add it before any official run.

Exit artifact:

- `safe_box_forest/outputs/paper/rbf_only/gate0_api_smoke.json`.

### E5-First: Lifelong/canonical Cache Mechanism

Purpose: establish the cache semantics used by all later RBF experiments.

Design:

- Build a canonical lifelong cache with depth-18 prewarm.
- During prewarm, materialize only the depth-18 leaves; parent records must come from child-hull propagation rather than direct FK at internal nodes.
- Compare cold target build, same-database replay, d18 warm external materialization, and, if implemented, guided active replay.
- Keep target scene/query/seed fixed for paired comparison.
- Verify that warm rows actually reuse evidence: `materialization_reused_endpoint_cache`, database evidence loads, or equivalent counters must be positive.
- Verify that canonical mode is more than a label. If sector canonicalization is not active, report the mechanism as canonical-identity lifelong cache and do not claim sector reuse.

Primary metrics:

- Prewarm wall time, cache bytes, node count, evidence count.
- Cold build time, warm target build time, replay build time.
- Query time, path length, strict audit success.
- Cache/evidence hit rates and materialization counters.
- Identity fields: endpoint descriptor, envelope descriptor, split schedule hash, canonical mode, symmetry descriptor.

Acceptance criteria:

- d18 cache can be reopened and verified with the same identity.
- Warm target build uses cached evidence in at least one materialization path.
- Strict audit pass rate is not reduced by cache reuse in the smoke scene.
- All later RBF experiment commands point at the accepted cache mechanism unless explicitly marked as a cache ablation.

Exit artifacts:

- `safe_box_forest/outputs/paper/rbf_only/e5_lifelong_cache_mechanism_smoke.json`.
- `safe_box_forest/outputs/paper/rbf_only/e5_lifelong_cache_mechanism_full.json` after scale-up.

### E1: Appendix Configuration Sweep

Purpose: choose the best RBF configuration while holding the accepted E5 cache mechanism fixed.

Design:

- Fixed: `EndpointSource.IFK`, `AAFKVolumeMin`, max depth 40, skip depth 15, canonical lifelong cache, d18 prewarm.
- Sweep: LinkIAABB vs KDOP26 vs SupportHull, budget tiers, connector budget, online cache/storage profile, and optional thread count.
- Use the E5 cache mechanism by default for warm rows.
- Include cold-cache rows only as explicitly labeled ablations.

Ranking rule:

1. Strict audit success rate.
2. Raw query success if audit SR ties.
3. Median target build plus query time.
4. Median audited path length.
5. Cache footprint and materialization cost.

Exit artifact:

- `safe_box_forest/outputs/paper/rbf_only/e1_appendix_config_sweep.json`.

### E2: Warm d18 Baseline

Purpose: generate the first baseline table using the selected sweep configuration and the accepted E5 cache mechanism.

Design:

- Use the E1 winner.
- Report prewarm cost separately.
- Run cold and warm-d18 target builds on Shelf+IIWA and at least one random-IIWA profile.
- Shelf rows must disable active-database backfill; random-IIWA rows may keep it enabled.
- Do not mix old CritSample/SBF numbers into the aggregate.

Exit artifact:

- `safe_box_forest/outputs/paper/rbf_only/e2_warm_d18_baseline.json`.

### E3: Shelf+IIWA RBF-Only Main Experiment

Purpose: reproduce the old main Shelf+IIWA workload with the RBF-only, certified, lifelong-cache protocol.

Design:

- Five Marcucci query pairs.
- Cold and warm-d18 rows, using E5 cache by default.
- Warm Shelf+IIWA rows must keep the external d18 cache read-only and disable active-database backfill during evaluation.
- Seeds: pilot 5 to 10; formal run 20 or more if timing permits.
- Metrics: build time, query time, audited path length, audit success rate, box count, segment edges, repair count, cache reuse.

Exit artifact:

- `safe_box_forest/outputs/paper/rbf_only/e3_shelf_iiwa_main.json`.

### E4: Random Robot Scenes RBF-Only Main Experiment

Purpose: test generalization without non-RBF algorithms.

Design:

- Robots: IIWA, UR5, Panda.
- Difficulties: easy, medium, hard.
- Pilot: 5 scene seeds.
- Formal: old-paper target of 50 scene seeds when the pilot passes.
- Use E5 lifelong/canonical cache by default; cold-cache rows only for labeled ablations.
- Random-scene rows may backfill newly materialized online evidence into the active database.

Exit artifact:

- `safe_box_forest/outputs/paper/rbf_only/e4_random_robot_scenes.json`.

### E6: Dynamic Update RBF-Only Experiment

Purpose: measure dynamic obstacle update behavior inside RBF only.

Design:

- Compare RBF full warm rebuild against RBF incremental add/remove/regrow.
- Use the E5 lifelong/canonical cache mechanism unless the row is explicitly a cold-cache ablation.
- Metrics: incremental time, warm rebuild time, dirty boxes, boxes added/removed, audit success, speedup.

Exit artifact:

- `safe_box_forest/outputs/paper/rbf_only/e6_dynamic_update.json`.

## Execution Checklist

The task is not complete until each item below is either passed or explicitly recorded as blocked with the blocking reason and the next concrete fix.

- [x] Write this plan to the repository and link it from the TRO plan index.
- [x] Add or verify the fixed RBF configuration entry.
- [x] Add or verify AAFKVolumeMin depth schedule generation for depths `0..39`.
- [x] Add or verify d18 lifelong prewarm generation.
- [x] Run Gate 0 API smoke and save JSON.
- [x] Run E5-first cache mechanism smoke and save JSON.
- [x] Confirm the E5 mechanism is the default for all later RBF commands through the shared `rbf_ifk_aa_aafkvol_d40_s15_canonical_lifelong` preset.
- [x] Run the appendix configuration sweep pilot.
- [x] Select and record the winning RBF configuration for the first executable baseline.
- [x] Run the warm d18 baseline.
- [x] Run Shelf+IIWA pilot, then full run when pilot passes.
- [ ] Run random-scene formal 50-seed expansion now that the 5-seed pilot passes.
- [ ] Run dynamic-update pilot.
- [x] Generate or update RBF-only paper/appendix tables from JSON artifacts.
- [x] Re-check strict audit accounting and artifact completeness.

## Current Execution Status

- Gate 0 artifact: `safe_box_forest/outputs/paper/rbf_only/gate0_api_smoke.json`, `ok=true`, with 11/11 configuration and binding checks passing.
- E5/d18 artifact: `safe_box_forest/outputs/paper/rbf_only/e5_lifelong_cache_mechanism_smoke.json`, `ok=true`.
- D18 prewarm baseline used LinkIAABB for the first executable mechanism baseline under the leaf-only FK rule: 524287 nodes, 524287 evidence records, 262144 direct leaf materializations, 20.22 s first prewarm wall time, 1.460 s reopen pass, and 262144 reused endpoint evidence records.
- The current canonical mode is canonical-identity lifelong cache: `canonical_mode=1`, empty `symmetry_descriptor`, and evidence keys still use the primary sector.
- E1 artifact: `safe_box_forest/outputs/paper/rbf_only/e1_appendix_config_sweep.json`, `ok=true`. All six no-grid rows (`LinkIAABB`, `KDOP26`, `SupportHull` at budgets 256/512) achieved strict-audit success rate 1.0 under the read-only Shelf+IIWA warm-cache protocol. The ranking winner is LinkIAABB at budget 256 with 0.00322 s median build, 46.05 ms median query, 205 reused evidence records, and roughly 47 KiB active-cache footprint.
- Selected first-baseline winner: LinkIAABB with `rbf_ifk_aa_aafkvol_d40_s15_canonical_lifelong`, max depth 40, skip depth 15, IFK_AA-backed endpoint evidence, AAFKVolumeMin schedule, canonical-identity lifelong cache.
- E2 warm d18 baseline artifact: `safe_box_forest/outputs/paper/rbf_only/e2_warm_d18_baseline.json`, `ok=true`. After excluding propagated `child_hull` records from exact endpoint reuse, both warm rows again build certified forests instead of falling back to query repair: Shelf+IIWA warm now reports 128 boxes / 4 segment edges with wall time 2.304 s, and random-IIWA easy warm reports 128 boxes / 1 segment edge with wall time 0.1316 s. Warm rows now reuse active exact endpoint evidence (`5306` shelf hits, `3319` random hits) and no longer claim external direct-evidence hits from the smoke cache.
- E3 Shelf+IIWA artifact: `safe_box_forest/outputs/paper/rbf_only/e3_shelf_iiwa_main.json`, `ok=true` at seed 0. Cold and warm-d18 rows both pass all five Marcucci query pairs under strict audit with 256 certified boxes and 3 segment edges; the repaired-only warm degeneracy is gone, but the corrected warm row now behaves as a full rebuild (2.501 s wall time, 11562 active endpoint-cache hits, 0 external direct-evidence hits) rather than the earlier invalid 0-box / repair-only fast path.
- E4 random-scene artifact: `safe_box_forest/outputs/paper/rbf_only/e4_random_robot_scenes.json`, `ok=true` on the corrected 5-seed pilot. Rechecking the prior UR5 failures showed that the issue was an under-budgeted random-scene runner, not center-to-center stitching or shortcut drift; after restoring the original random-scene growth and parallelism defaults, all nine warm summaries reached strict-audit success rate 1.0. The resulting warm-build means now span 0.00155--0.55695 s across IIWA/UR5/Panda; IIWA remains a repair-dominated diagnostic case with some zero-box warm builds, while Panda and UR5 now report nontrivial corridor builds and no longer block the formal 50-seed expansion.
- The paper table generator now writes the RBF-only appendix tables into the actual paper include directory `paper/sbf_old/generated/`, preserves existing main-paper generated assets when backing artifacts are missing, and extends `tro_macros.tex` with the RBF-only appendix metrics used in the manuscript text.

## Expected Results

- The IFK_AA-certified configuration should have a cleaner soundness story than old CritSample rows, but it may build more slowly and produce more conservative boxes.
- The E5-first cache mechanism should recover a visible fraction of target build cost when d18 prewarmed evidence overlaps the target route.
- Hard random scenes are expected to be the first place where `max_depth=40` and `skip_to_depth=15` expose failures or repair pressure.
- If canonical sector reuse is not actually active, the result should still be useful as a lifelong persistent cache result, but the paper wording must avoid claiming sector canonicalization speedup.

## Verification Notes

- Every successful path reported in final artifacts must have strict audit pass.
- Every JSON must include command line, seed base, cache namespace, split schedule hash, endpoint/envelope descriptors, build directory, thread count, and date.
- A failed experiment row remains in the artifact and counts against success-rate denominators.
- RBF-only tables must not include old external baseline rows unless the table is explicitly historical context.