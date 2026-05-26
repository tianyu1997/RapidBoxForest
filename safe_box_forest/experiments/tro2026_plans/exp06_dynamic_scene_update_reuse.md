# Exp.6 Dynamic Scene-Update Reuse

## Claim

SBF can reuse or locally repair geometric structure under scene edits better than rebuilding everything for repeated-query settings.

## Hypotheses

- Local invalidation/regrowth retains a meaningful fraction of boxes.
- Warm-cache rebuild is faster than cold rebuild but may still be slower than local updates.
- Post-update paths remain safe only if final strict audit passes.

## Existing Runner

- `experiments/paper_06_obstacle_rebuild.py`

## Protocol

1. Build a baseline Marcucci forest.
2. Add or modify an obstacle such as `front_bin`, `shelf_mouth`, or `table_edge`.
3. Measure invalidation, rebuild/regrow, and post-update queries.
4. Compare local update/regrow with full rebuild where runner support exists.
5. Extend later to deletion/movement/edit sequences if needed.

## Comparison Groups

- Added obstacle type.
- Preset: `crit_link_coverage`, `coverage_hybrid`, optional `ifk_strict`.
- Threads: 1 vs 8 if useful.
- Local update/regrow vs full rebuild when available.

## Metrics

- Initial build time and boxes.
- Invalidated/retained boxes.
- Update/regrow time.
- Adjacency rebuild time if exposed.
- Post-update SR/audit SR.
- Path degradation and repair/fallback count.

## Visualizations

- Before/after box retention bar.
- Update-time time series.
- Representative before/after corridor figure.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_06_obstacle_rebuild.py \
  --preset crit_link_coverage \
  --seeds 1 \
  --threads 1 \
  --max-boxes 300 \
  --added-obstacle front_bin \
  --out-json outputs/paper/tro2026_exp06_dynamic_smoke.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_06_obstacle_rebuild.py \
  --preset crit_link_coverage \
  --seeds 10 \
  --threads 8 \
  --task-batch-size 8 \
  --max-boxes 1200 \
  --added-obstacle front_bin \
  --out-json outputs/paper/tro2026_exp06_dynamic_full.json
```

## Acceptance Criteria

- Dynamic results report how much structure is reused.
- Post-update success is final audit success, not only query success.
- If only insertion is supported, the paper must avoid claiming general dynamic environments.
