# Exp.2 LECT Reuse And Cache Attribution

## Claim

LECT reuses kinematics-keyed evidence across repeated builds and scene changes, and the observed speedup can be attributed to cache/evidence reuse rather than incidental warm-start effects.

## Hypotheses

- Warm cache is faster than cold cache for expensive envelope payloads.
- Cross-scene same-robot reuse preserves downstream audit success.
- Cache footprint and I/O cost are small enough to justify reuse.

## Existing Runner

- `experiments/paper_03_marcucci_envelope_build.py`

## Protocol

1. Run cold, warm, and cross-scene protocols on Marcucci shelf+IIWA.
2. Use a stable cache root and unique `cache_run_id` for smoke/full.
3. Record cache file count/bytes, build time, query audit SR, and LECT diagnostics.
4. If implementation hooks exist later, add warm-with-cache-but-recompute as attribution control.
5. If process restart can be automated, run a second process against the saved cache root.

## Comparison Groups

- Protocol: cold, warm, cross_scene.
- Variant: `crit_link_coverage`, `coverage_hybrid`.
- Storage profile: fast_query by default; compact/balanced optional diagnostics.

## Metrics

- Build mean/median and CI.
- Prewarm build time when recorded.
- Cache bytes and file count.
- Lazy grid materializations and grid read time.
- Box count, certified/provisional counts, segment-edge count.
- Audited query SR and repair count.

## Visualizations

- Cache-attribution waterfall.
- Cold/warm/cross-scene build-time bars with CI.
- Storage footprint vs speedup plot.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_03_marcucci_envelope_build.py \
  --variants crit_link_coverage \
  --protocols cold,warm \
  --seeds 1 \
  --threads 2 \
  --max-boxes 300 \
  --cache-run-id tro2026_exp02_smoke \
  --record-prewarm-trials \
  --checkpoint-json outputs/paper/tro2026_exp02_lect_smoke.checkpoint.json \
  --out-json outputs/paper/tro2026_exp02_lect_smoke.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_03_marcucci_envelope_build.py \
  --variants crit_link_coverage,coverage_hybrid \
  --protocols cold,warm,cross_scene \
  --seeds 10 \
  --threads 8 \
  --task-batch-size 8 \
  --max-boxes 5000 \
  --timeout-ms 60000 \
  --cache-run-id tro2026_exp02_full_optimized \
  --record-prewarm-trials \
  --checkpoint-json outputs/paper/tro2026_exp02_lect_full.checkpoint.json \
  --out-json outputs/paper/tro2026_exp02_lect_full.json
```

The full command depends on the shared optimized grow-stop defaults in `experiments/common_sbf_config.py`: quality floor 128, 450 ms post-connect budget, best-tighten split, component-connect adaptive FFB, and the Marcucci connector/repair settings. A 10-seed rerun with these defaults completed without timeout; `crit_link_coverage` build means were 0.224 s cold, 0.294 s warm, and 0.323 s cross-scene.

## Acceptance Criteria

- Warm and cross-scene protocols have explicit cache metrics.
- Audit SR is not reduced by reuse.
- Any unimplemented attribution control is reported as future work, not claimed.
