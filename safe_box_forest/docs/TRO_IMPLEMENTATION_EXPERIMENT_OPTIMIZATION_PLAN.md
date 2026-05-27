# TRO Implementation-Level Experiment Optimization Plan

Date: 2026-05-06

This document turns the current implementation review of `LECT`, `link-interval-envelope`, and `sbf-standalone` into an executable experiment-improvement plan.  The goal is not to add more main-paper experiments blindly, but to make the existing compressed TRO 2026 experiment suite strong enough to justify implementation optimizations with measurable, auditable evidence.

## 1. Optimization principles

1. Measure before changing algorithms.  Every optimization must have a pre-change JSON/CSV baseline and a post-change JSON/CSV comparison.
2. Keep candidate success and strict-audit success separate in all planner-level artifacts.
3. Do not count provisional evidence as safe unless the final path passes strict audit or audited repair.
4. Prefer stage-attribution tables over aggregate-only runtime tables.
5. Keep main-text tables compressed; move per-stage, per-seed, and per-query diagnostics to appendix artifacts.
6. Every script should write enough metadata to reproduce the run: command line, git-relevant configuration, seed list, thread count, artifact inputs, and schema version.

## 2. Package-specific experiment gaps

### 2.1 LECT

Implementation opportunities to validate:

- incremental `node_intervals()` reconstruction;
- evidence/grid hash indexing;
- reduced LRU payload copies;
- lazy or streaming binary load/save;
- worker-local session reuse;
- Windows file-lock backend.

Required experiment artifacts:

| ID | Artifact | Purpose | Main metrics |
|---|---|---|---|
| LECT-01 | `tro2026_lect_interval_cache.json` | compare root-rebuild intervals vs incremental descent | interval calls, interval time, FFB time, build time |
| LECT-02 | `tro2026_lect_evidence_index.json` | compare linear store lookup vs indexed lookup | evidence lookup time, grid lookup time, cache hit rate |
| LECT-03 | `tro2026_lect_storage_lazy.json` | compare eager vs lazy/mmap load | load time, peak memory, first-query latency |
| LECT-04 | `tro2026_lect_worker_session.json` | quantify per-task vs per-worker session reuse | session count, merge time, grow time, determinism |

Validation gates:

- serialized tree round-trip must match node/evidence counts;
- query audit success must not decrease;
- indexed lookup must return exactly the same evidence payload as the linear implementation;
- lazy loading must preserve delta replay semantics.

### 2.2 link-interval-envelope

Implementation opportunities to validate:

- unified thread budget for CritSample and batch API;
- CritSample candidate cache and dirty-joint tracking;
- KDOP/SupportHull predicate optimization;
- lightweight Python binding output modes;
- GCPC interval lookup indexing.

Required experiment artifacts:

| ID | Artifact | Purpose | Main metrics |
|---|---|---|---|
| LIE-01 | `tro2026_lie_thread_budget.json` | detect nested-parallel oversubscription | total time, CPU efficiency, worker count, variance |
| LIE-02 | `tro2026_lie_critsample_cache.json` | compare raw candidate recompute vs cached candidates | endpoint time, candidate count, dirty joints |
| LIE-03 | `tro2026_lie_envelope_predicates.json` | compare LinkIAABB, KDOP, and SupportHull predicate cost | envelope time, predicate rejects, intersection time |
| LIE-04 | `tro2026_lie_python_payload.json` | quantify Python list/dict output overhead | return time, payload size, optional-field cost |

Validation gates:

- IFK/LinkIAABB conservative rows must remain numerically identical within tolerance;
- CritSample changes must be treated as provisional and discharged by SBF strict audit;
- tighter envelope predicates must not shrink conservative occupied sets incorrectly;
- Python schema changes must be opt-in or backwards compatible.

### 2.3 sbf-standalone

Implementation opportunities to validate:

- adjacency, merger, and connector candidate indexing;
- persistent thread pool and per-thread diagnostics;
- RRT nearest-neighbor indexing;
- batch/pooled collision audit;
- query point-location and graph-search caching;
- dynamic obstacle update compaction and evidence broadphase;
- Python binding GIL release.

Required experiment artifacts:

| ID | Artifact | Purpose | Main metrics |
|---|---|---|---|
| SBF-01 | `tro2026_sbf_graph_index.json` | compare pairwise graph construction vs indexed candidates | adjacency time, candidate pairs, edge count |
| SBF-02 | `tro2026_sbf_runtime_pool.json` | compare short-lived threads vs persistent executor | stage time, speedup, efficiency, variance |
| SBF-03 | `tro2026_sbf_query_index.json` | quantify point-location and graph-cache benefits | query time, locate time, Dijkstra time |
| SBF-04 | `tro2026_sbf_dynamic_broadphase.json` | compare full check vs evidence-gated obstacle update | removed boxes, collision-check time, rebuild time |
| SBF-05 | `tro2026_sbf_python_gil.json` | quantify multi-process/multi-thread Python throughput | wall time, GIL-released calls, failures |

Validation gates:

- indexed adjacency must be graph-equivalent to the existing all-pairs builder;
- persistent executor must preserve exception propagation and deterministic mode;
- query cache must be invalidated after dynamic update or forest mutation;
- dynamic update must not leave segment edges pointing to deleted boxes.

## 3. Compressed paper-facing experiment mapping

The implementation experiments above should feed the existing five compressed main groups rather than adding many new main-text tables.

1. Evidence Profiles and Validation Boundary: use LIE-01/LIE-02/LIE-03 only as appendix evidence behind `tab:tro_main_evidence_validation`.
2. Reuse and Multi-Query Amortization: use LECT-01/LECT-02/LECT-03 and SBF-03 to strengthen `tab:tro_lect_reuse` and `fig:tro_query_amortization`.
3. Shelf+IIWA Benchmark: use SBF-01/SBF-02/SBF-03 to justify faster build/query rows in `tab:tro_main_shelf_benchmark`.
4. Safety Accounting and GCS Composition: preserve strict audit semantics; do not let any optimization bypass audit.
5. Generalization and Systems Limits: use SBF-04/SBF-05 and parallel-stage output behind `tab:tro_main_systems_summary`.

## 4. Immediate script changes

### P0 scripts

1. Add an implementation-optimization matrix generator:
   - script: `experiments/tro2026_main_03_implementation_optimization.py`;
   - outputs: `tro2026_exp17_implementation_optimization_plan.json` and `.csv`;
   - purpose: centralize the optimization experiment registry and track artifact readiness.
2. Extend `experiments/tro2026_generate_tables.py`:
   - add an appendix table `tab_tro_implementation_optimization_plan.tex`;
   - include it in `--mode appendix` and `--mode all`.
3. Keep `tro2026_main_01_evidence_validation.py` as the source of strict/provisional planner-level evidence rows.
4. Keep `tro2026_main_02_query_amortization.py` as the source of multi-query amortization rows, but allow it to consume optimized SBF artifacts once they exist.

### P1 scripts

1. Extend `paper_07_parallel_scaling.py` with stage-fraction output: grow, connector, adjacency, query, audit.
2. Extend `paper_06_obstacle_rebuild.py` with post-update query/audit rows and full-rebuild comparator.
3. Add graph-index and query-index microbench scripts after the corresponding C++ counters are exposed.

## 5. Table and figure design

### Main text

No new main-text table is required immediately.  Implementation optimization evidence should be summarized through the existing compressed tables.

### Appendix / supplement

Add one roadmap table generated from `tro2026_exp17_implementation_optimization_plan.json`:

| Package | ID | Optimization target | Artifact | Primary metric | Gate | Priority |
|---|---|---|---|---|---|---|

When actual artifacts are produced, this table becomes the checklist for which optimizations are experimentally validated.

### Figures

Recommended follow-up figures:

1. stage-time stacked bar for SBF parallel scaling;
2. query-latency breakdown before/after query-index caching;
3. LECT cold/warm/lazy-load timeline;
4. LinkIAABB/KDOP/SupportHull predicate microbenchmark bar chart.

## 6. Execution order

1. Generate the optimization matrix artifact and appendix table.
2. Run current baseline scripts with unchanged code and archive JSON outputs.
3. Add missing counters before changing algorithms.
4. Implement one optimization at a time.
5. Re-run only the affected artifact group.
6. Regenerate tables with `experiments/tro2026_generate_tables.py --mode all`.
7. Compile the paper and verify no legacy table labels re-enter the source.

## 7. Minimum acceptable first implementation package

The first implementation pass is complete when:

- `experiments/tro2026_main_03_implementation_optimization.py` exists;
- it writes JSON and CSV registry artifacts;
- `tro2026_generate_tables.py --mode appendix` emits `tab_tro_implementation_optimization_plan.tex`;
- existing experiment scripts still pass Python syntax checks;
- placeholder/main table generation still works.