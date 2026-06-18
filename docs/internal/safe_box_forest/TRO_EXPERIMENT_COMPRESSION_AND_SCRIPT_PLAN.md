# TRO Experiment Compression and Script-Revision Plan

Date: 2026-05-06

## 1. Motivation

The current experiment infrastructure can support more than sixteen experiment artifacts, but a TRO manuscript cannot present them as sixteen independent main-text experiments. The paper should be organized around reviewer questions, not around script names.

The compressed design keeps five main experiment groups and moves detailed ablations to appendix/supplement.

## 2. Final main-text experiment groups

### Group 1: Evidence Profiles and Validation Boundary

Question: which evidence sources are strict certificates and which are provisional evidence discharged by final audit?

Main-text content:

- One compressed evidence/profile table.
- Rows: IFK+LinkIAABB strict, Crit+LinkIAABB provisional, Crit+KDOP26 provisional, SBF-SH keep-KDOP provisional, SupportHull no-KDOP provisional.
- Columns: evidence status, build time, boxes, candidate SR, audit SR, repair queries, certified/provisional/segment length fractions.

Source scripts:

- `experiments/paper_01_epiaabb_pipeline.py`
- `experiments/paper_02_link_envelope_pipeline.py`
- `experiments/tro2026_main_01_evidence_validation.py`

Move to appendix:

- endpoint-source microbench details;
- link-envelope payload/tightness details;
- width-bin curves;
- HullGrid variants.

Required script changes:

- Improve `tro2026_main_01_evidence_validation.py` with candidate/audit failure taxonomy, Wilson intervals, repair-count semantics, and per-query summaries.
- Generate one compressed main table from `tro2026_exp14_validation_profiles.json`.

### Group 2: Reuse and Multi-Query Amortization

Question: does the reusable forest and LECT cache pay off as query count increases?

Main-text content:

- One LECT reuse table.
- One query-amortization figure/table.

Source scripts:

- `experiments/paper_03_marcucci_envelope_build.py`
- `experiments/paper_04_marcucci_combined.py`
- `experiments/tro2026_main_02_query_amortization.py`

Move to appendix:

- FFB depth sweep;
- cache-stage details;
- full cold/warm/cross-scene/restart rows;
- per-query amortization rows.

Required script changes:

- Improve `tro2026_main_02_query_amortization.py` to auto-discover existing baseline artifacts and distinguish candidate vs audited success fields.
- Keep `fig_tro_query_amortization.pdf` as the preferred main-text figure when available.

### Group 3: Shelf+IIWA Planning Benchmark

Question: how does SBF compare against conventional baselines on the canonical Shelf+IIWA benchmark?

Main-text content:

- One main benchmark table with build, query, audit, success, repair/fallback, and path length.

Source scripts:

- `experiments/paper_04_marcucci_combined.py`
- `experiments/paper_04_baselines_marcucci.py`
- `experiments/paper_04_rrt_connect_baseline.py`
- `experiments/paper_04_grower_tradeoff.py`

Move to appendix:

- per-query canonical rows;
- envelope variants;
- grower trade-off;
- bridge/segment-edge detailed accounting.

Required script changes:

- Keep waypoint persistence in `paper_04_marcucci_combined.py`.
- Table generator should present a compressed main benchmark and move grower/variant tables to appendix output.

### Group 4: Safety Accounting and GCS Composition

Question: do GCS and merged/provisional corridors preserve the audited safety contract?

Main-text content:

- One safety/GCS table.
- Rows: direct GCS over provisional boxes, overlap-expanded corridor, path-tube GCS, protected final interface, paper-wide audited outputs.
- Columns: solver candidates, strict audit pass, solved-but-unsafe, fallback/repair, final audited success.

Source scripts:

- `experiments/paper_04_audited_corridor_gcs.py`
- `experiments/paper_07_merger_protected_study.py`
- `experiments/paper_11_soundness_audit_suite.py`
- future `experiments/paper_16_audit_sensitivity.py`

Move to appendix:

- full per-query GCS taxonomy;
- audit-resolution sweep;
- merger mode details;
- path-source stacked bars.

Required script changes:

- Extend `gcs_table()` in the generator to show solver-only vs strict-audited success separately.
- Add audit sensitivity later, but main-text can reserve only one summary row.

### Group 5: Generalization and Systems Limits

Question: does SBF generalize beyond one scene, and what are the dynamic/parallel limitations?

Main-text content:

- One cross-robot random-scene quality/efficiency table with comparable SBF and RRT-Connect metrics only. Dynamic update and parallel scaling are appendix diagnostics.

Source scripts:

- `experiments/paper_05_random_robot_scenes.py`
- `experiments/paper_12_random_scene_rrt_baseline.py`
- `experiments/paper_06_obstacle_rebuild.py`
- `experiments/paper_07_parallel_scaling.py`
- `experiments/paper_13_mechanism_diagnostics.py`

Move to appendix:

- full random-scene matrix;
- full RRT baseline rows;
- dynamic per-obstacle rows;
- parallel full thread sweep;
- mechanism diagnostics table.

Required script changes:

- Add a compact `main_generalization_table()` that cross-loads random SBF and same-oracle RRT-Connect artifacts without structural empty cells.
- Keep dynamic, parallel, and mechanism diagnostics as appendix outputs.

## 3. Main-text table/figure budget

Target main-text outputs:

1. `tab_tro_main_evidence_validation.tex`
2. `tab_tro_lect_reuse.tex`
3. `fig_tro_query_amortization.pdf`
4. `tab_tro_main_shelf_benchmark.tex`
5. `tab_tro_main_generalization.tex`
6. `text_tro_safety_fallback.tex`

Retired from main text: the duplicate query-amortization table, the compressed safety/GCS table, and the mixed systems-summary table.

Appendix outputs retain detailed diagnostics:

- endpoint source table;
- link envelope table;
- FFB depth table;
- envelope variants;
- grower trade-off;
- random-scene full table;
- dynamic full table;
- parallel full table;
- mechanism diagnostics;
- soundness audit suite.

## 4. Immediate implementation tasks

### P0. Improve `tro2026_main_01_evidence_validation.py`

Add fields:

- `candidate_failed_count`
- `candidate_failed_audit_count`
- `no_candidate_count`
- `missing_audit_count`
- `repair_query_count`
- `repair_count_sum`
- `query_sr_ci95`
- `audit_sr_ci95`
- `per_query_summary`

### P0. Improve `tro2026_main_02_query_amortization.py`

Add:

- automatic baseline discovery;
- `candidate_success_rate` and `audit_success_rate` fields;
- clearer `success_semantics` per method;
- robust baseline artifact parsing for current `marcucci_*.json` files.

### P0. Modify `tro2026_generate_tables.py`

Add:

- compressed main table functions;
- `--mode main|appendix|all`;
- `--strict-missing` and missing-artifact manifest;
- separate `main_writers` and `appendix_writers`.

## 5. Decisions

- Do not present sixteen experiments in the main text.
- Keep five main groups aligned with reviewer questions.
- Keep strict/provisional and candidate/audited distinction visible in every main-text planning table.
- Treat dynamic update and parallel scaling as systems-limit evidence unless stronger results are added.
- Preserve detailed artifacts and tables in appendix for reproducibility.

## 6. 2026-05-18 Major-Revision Update

The current manuscript should be treated as a planning-systems paper with an
endpoint-to-link geometry primitive, not as a geometry/theory paper whose main
experiments already prove primitive-level superiority. The main text should stay
compressed around the final-audited time/path-quality design point and move
cache/storage/symmetry/persistence details out of the claim path.

Updated main-text priorities:

1. Validation semantics first.
	 - Every planning table or caption that reports success should state whether
		 the row is candidate, corridor-certified, or final-audited.
	 - Add an audit-resolution sweep before submission if possible; otherwise the
		 0.01 segment step remains an empirical audit setting, not a continuous
		 guarantee.

2. Fairness before breadth.
	 - A seed-protocol audit is higher priority than adding more similar
		 random-scene curves: all reported planners should be checked against the
		 same Shelf+IIWA anchor seeds and fixed random scene/query seeds.
	 - Add seed-count or seed-set sensitivity rows if artifacts are available, and
		 keep the claim as full-pipeline unless a separate primitive-isolation study
		 is added.

3. Attribution before mechanism volume.
	 - Add planner-level ablations for unexplored-volume sampling, frontier
		 cooling, component connectors, post-corridor smoothing/repair, and LECT
		 on/off.
	 - Compress LECT details in the main paper; report warm prewarm as a modest
		 optimization and same-scene replay as an upper reference.

4. Generalization only if it changes the claim.
	 - A realistic repeated-query manipulation workload is more valuable than
		 another small synthetic matrix.
	 - If no new workload is added, keep generalization language limited to the
		 tested Shelf+IIWA and fixed-seed random-scene settings.

Script changes to prioritize:

- Add `experiments/paper_16_audit_sensitivity.py` or equivalent audit-sweep mode.
- Add seed-count or seed-set sweep support to the IRIS-NP+GCS baseline scripts.
- Add a compact ablation writer to `tro2026_generate_tables.py` only after the
	corresponding artifacts exist; do not generate placeholder rows into the main
	paper.
