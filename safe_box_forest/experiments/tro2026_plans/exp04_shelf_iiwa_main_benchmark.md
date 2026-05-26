# Exp.4 Shelf+IIWA Main Planner Benchmark

## Claim

On the canonical Marcucci shelf+IIWA workload, SBF is competitive because it builds reusable audited box structure and amortizes that build across repeated queries.

## Hypotheses

- SBF is strongest in multi-query accounting, not necessarily in single-query time.
- Audited success and solved success may differ for optimizer baselines; the distinction must be visible.
- Path quality should be compared to external planners, not only v6/internal variants.

## Existing Runner

- `experiments/paper_04_marcucci_combined.py`

## External Baseline References

- `cpp/v6/experiments/paper/04_e2e_baselines_combined.py`
- `cpp/v6/scripts/run_online_query_comparison.py`

## Protocol

1. Build SBF once per seed on shelf+IIWA.
2. Query the five canonical Marcucci pairs.
3. Add random-query sets for N=1, 5, 10, 20, 50 when the runner is extended.
4. Run external baselines on identical query sets and declared budgets.
5. Re-audit all returned paths when feasible.

## Comparison Groups

- SBF variants: `crit_link_coverage`, `coverage_hybrid`, `ifk_strict`.
- Historical reference: v6 artifact only as reference, not a substitute for external baselines.
- External baselines: OMPL RRTConnect, OMPL PRM, OMPL BIT*, IRIS-NP+GCS subset, IRIS-ZO+GCS if stable.

## Metrics

- Build time and box count.
- Total wall time for N queries.
- Amortized time/query.
- Query p50/p95.
- Audit SR, repair rate, fallback rate.
- Path length and ratio to baseline/reference.

## Visualizations

- Time-vs-path Pareto.
- Query-count amortization curve.
- Query-time ECDF.
- Per-query result table.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_04_marcucci_combined.py \
  --preset crit_link_coverage \
  --seeds 1 \
  --threads 2 \
  --max-boxes 500 \
  --timeout-ms 20000 \
  --quality-min-connected-boxes 64 \
  --post-connect-time-budget-ms 100 \
  --out-json outputs/paper/tro2026_exp04_marcucci_smoke.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_04_marcucci_combined.py \
  --preset crit_link_coverage \
  --seeds 10 \
  --threads 8 \
  --max-boxes 5000 \
  --timeout-ms 60000 \
  --quality-min-connected-boxes 128 \
  --post-connect-time-budget-ms 450 \
  --corridor-refine \
  --bridge-repaired-queries \
  --out-json outputs/paper/tro2026_exp04_marcucci_full.json
```

## Acceptance Criteria

- Main table separates raw query success, audit success, repair, and fallback.
- Baseline budgets are documented in the paper.
- If SBF loses on single-query time, the paper emphasizes multi-query amortization and audited reuse.
