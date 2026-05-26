# TRO Experiment, Table, and Figure Optimization Plan

Date: 2026-05-06

## 1. Goal

The experiment section should support the updated SafeBoxForest (SBF) claim boundary:

- IFK endpoint envelopes provide strict conservative box certificates.
- CritSample and other sampling-based endpoint sources provide coverage-oriented provisional evidence.
- Reported planning success is counted only after strict collision audit or audited local repair.
- LECT reuse accelerates kinematic evidence materialization, not scene-level safety labels.
- Segment edges and GCS adapters are path witnesses / optimization interfaces, not implicit convex-free certificates.

The revised table and visualization design should make these distinctions visible rather than hidden inside aggregate success rates.

## 2. Main-paper table budget

Keep the main paper focused. Recommended main-paper set:

1. **Experiment matrix**: one compact table listing artifact, protocol, primary claim, and output table/figure.
2. **Envelope evidence table**: endpoint source + link envelope trade-off, split into strict/provisional status.
3. **Validation profile table**: planner-level IFK strict vs CritSample/KDOP/SupportHull provisional profiles.
4. **LECT reuse table**: cold, warm, restart, cross-scene with cache-stage attribution.
5. **Shelf+IIWA benchmark table + amortization figure**: build/query split plus amortized time as query count grows.
6. **Safety accounting table**: audit sensitivity, repair use, solved-but-unsafe negative controls.
7. **Systems table**: dynamic update and parallel scaling root-cause summary.

Move full per-query, per-seed, and per-stage breakdowns to appendix.

## 3. Table redesign principles

### 3.1 Always separate build, query, and audit

Every planner-level table should use columns like:

| Method | Build (s) | Query med. (ms) | Audit med. (ms) | Path med. | Query SR | Audit SR | Repair/query | Notes |

Rules:
- `Query SR` means solver/query returned a candidate path.
- `Audit SR` means candidate path passed strict audit after any allowed repair/fallback.
- `Repair/query` should be reported as an event count or fraction, not only a median.
- Path statistics are success-only and should be labeled as such.

### 3.2 Label strict vs provisional explicitly

Add a `Validation` or `Evidence` column:

| Evidence | Meaning |
|---|---|
| `IFK strict` | interval-conservative endpoint/source and strict box certificate |
| `Crit provisional` | sampling endpoint evidence; counted safe only after audit |
| `Segment witness` | collision-checked path edge, not box overlap |
| `GCS optimized` | optimization output; counted only after audit |

### 3.3 Use confidence intervals for rates

For success rates, report Wilson 95% CI when sample count is more than a smoke test:

`Audit SR = 0.96 [0.86, 0.99]`

For small fixed canonical workloads (5 queries), report exact numerator/denominator:

`5/5`, `3/5`, `0/5`.

### 3.4 Prefer median + IQR for timing and path length

For random scenes and multi-seed runs:

- `Build med. [IQR]`
- `Query med. [IQR]`
- `Path med. [IQR]`

Avoid mean-only timing rows unless the table is a microbenchmark.

### 3.5 Use stage-attribution rows for cascades

For AABB--KDOP26--SupportHull, add a cascade table:

| Stage | Candidate pairs | Rejected | Rejection % | Time % |
|---|---:|---:|---:|---:|
| AABB gate | ... | ... | ... | ... |
| KDOP26 slabs | ... | ... | ... | ... |
| SupportHull/GJK | ... | ... | ... | ... |

This prevents reviewers from interpreting SupportHull as an always-on expensive narrow phase.

## 4. Figure redesign principles

### 4.1 Main multi-query figure: amortization curve

Plot:

- x-axis: number of queries (`1, 5, 10, 20, 50`).
- y-axis: amortized audited wall time per query.
- Lines: SBF-SH, RRT-Connect, PRM, IRIS/GCS if data available.
- Use log y-axis if needed.

Purpose: directly demonstrates why a reusable SBF forest matters.

### 4.2 Safety accounting figure: stacked path-source bars

For each query or method, show path length decomposition:

- strict box corridor length;
- provisional-but-audited length;
- segment-edge length;
- repaired length if available.

Purpose: makes `Safe` mean audited output accounting, not blind trust in provisional boxes.

### 4.3 GCS boundary figure

Use a three-panel figure:

1. Direct GCS over provisional/merged boxes: solved but audit fails.
2. Overlap-expanded corridor: fixes feasibility but still fails audit in some cases.
3. Protected final interface: GCS-strict-audited where possible, audited SBF fallback otherwise.

Annotate final source counts: e.g. `3/5 GCS strict-audited, 2/5 audited SBF fallback`.

### 4.4 Dynamic update figure

Use a before/after sequence:

1. Original forest and query path.
2. Inserted obstacle and invalidated boxes.
3. Surviving graph after adjacency rebuild.
4. Optional local regrowth around exposed frontiers.
5. Post-update audited query result.

The current evaluated result should be titled `localized invalidation + graph repair`; use `regrowth` only for the optional/future-enabled row.

### 4.5 Parallel scaling figure

Use stacked bars instead of only speedup:

- grow;
- FFB/oracle;
- connector;
- merge;
- adjacency;
- commit/remap;
- audit.

Purpose: explains why small Marcucci workloads do not show strong scaling.

## 5. Required new experiments

### P0. Validation-profile ablation

Script: `experiments/tro2026_main_01_evidence_validation.py`

Rows:

- `IFK+LinkIAABB strict`
- `Crit+LinkIAABB provisional`
- `Crit+KDOP26 provisional`
- `SBF-SH keep_kdop=true provisional`
- `SBF-SH keep_kdop=false provisional`

Metrics:

- build time;
- boxes;
- certified/provisional box count;
- query SR;
- strict audit SR;
- repair events;
- strict/provisional/segment path-length decomposition;
- median query time;
- median path length.

### P0. Audit-sensitivity sweep

Script: `experiments/paper_16_audit_sensitivity.py`

Sweep audit resolution: `16, 32, 64, 128`.

Metrics:

- candidate paths;
- audited pass count;
- audit SR;
- newly failed count relative to default;
- audit time;
- repair events;
- final path length.

### P0. GCS taxonomy table

Update `paper_04_audited_corridor_gcs.py` and table generator.

Columns:

- query;
- direct solved / direct audit;
- expanded solved / expanded audit;
- path-tube solved / path-tube audit;
- final audit;
- final source;
- fallback used;
- GCS time;
- adapter time;
- final length.

Wording: protected interface final audited success, not GCS-only success.

### P0. Dynamic post-update query

Extend `paper_06_obstacle_rebuild.py`.

Rows:

- no update negative control;
- delete only + adjacency rebuild;
- delete + local regrow if available;
- full rebuild.

Metrics:

- update time;
- removed boxes;
- regrown boxes;
- post-update query SR;
- post-update audit SR;
- path length change;
- full rebuild comparator.

### P1. Query amortization curve

Script: `experiments/tro2026_main_02_query_amortization.py`

Input existing SBF/baseline artifacts and output:

- `tro2026_exp15_query_amortization.json`
- `fig_tro_query_amortization.pdf` if matplotlib is installed.

The numeric table is not a main-text artifact in the revised framework; keep JSON/CSV for reproducibility and use the figure in the paper.

### P1. Cascade attribution

Extend oracle/build profile counters if needed. If C++ counters are not yet exposed, start with available diagnostics and mark missing stage counters in JSON as null rather than fabricating values.

### P1. Random-scene CI

Expand random scene seeds and add Wilson CI + IQR summaries. Keep failure rows; do not filter hard cases out of the denominator.

### P1. Parallel root-cause breakdown

Extend `paper_07_parallel_scaling.py` to output stage timings already present in the build profile, and generate a stacked-bar figure.

## 6. Generated artifact naming

Use stable names:

- `tro2026_exp14_validation_profiles.json`
- `tro2026_exp15_query_amortization.json`
- `tro2026_exp16_audit_sensitivity.json`
- `tab_tro_validation_profiles.tex`
- `tab_tro_audit_sensitivity.tex`
- `fig_tro_query_amortization.pdf`
- `fig_tro_parallel_stage_breakdown.pdf`

## 7. Implementation order

1. Implement validation profile ablation runner.
2. Implement query amortization postprocessor/plotter.
3. Add `waypoints` to Marcucci SBF query artifacts so future audit-sensitivity scripts can re-audit saved paths.
4. Implement audit-sensitivity runner.
5. Add table generator functions for the three new tables.
6. Extend dynamic update experiment with post-update query/full rebuild rows.
7. Extend GCS table taxonomy.
8. Extend parallel scaling stage breakdown figure.

## 8. Minimum acceptable improvement package

If time is limited, ship these first:

1. validation-profile ablation table;
2. query amortization figure;
3. GCS taxonomy wording/table fix;
4. dynamic update wording or post-update query comparator;
5. audit sensitivity table.

Together these close the biggest reviewer-facing gaps in safety, multi-query reuse, and experiment interpretability.
