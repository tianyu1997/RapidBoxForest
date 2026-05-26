# Exp.8 Parallel And Resource Scaling

## Claim

The implementation either demonstrates practical scaling or clearly bounds where it is sequential or contended. Negative scaling must be explained rather than hidden.

## Hypotheses

- Small workloads may not scale monotonically because fixed overhead dominates.
- Larger workloads or better batch sizes may improve efficiency.
- Stage breakdown identifies whether grower, connector, consolidation, or audit dominates.

## Existing Runner

- `experiments/paper_07_parallel_scaling.py`

## Protocol

1. Run shelf+IIWA with 1/2/4/8/16 threads.
2. Repeat on a larger workload if available.
3. Sweep task batch size if runner support is added.
4. If non-monotone, add perf/lock profiling as diagnostic evidence.

## Comparison Groups

- Thread count: 1, 2, 4, 8, 16.
- Workload size: default and larger max-box/quality floor.
- Batch size: default and tuned variants if supported.

## Metrics

- Wall time and CPU time.
- Speedup relative to 1 thread.
- Parallel efficiency.
- Build/query/audit stage shares.
- Lock wait/cache misses if measured.
- Peak memory.

## Visualizations

- Speedup and efficiency curves.
- Stacked stage timing bars.
- Resource table.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_07_parallel_scaling.py \
  --threads-grid 1,2 \
  --seeds 1 \
  --quality-min-connected-boxes 64 \
  --post-connect-time-budget-ms 100 \
  --out-json outputs/paper/tro2026_exp08_parallel_smoke.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_07_parallel_scaling.py \
  --threads-grid 1,2,4,8,16 \
  --seeds 5 \
  --quality-min-connected-boxes 128 \
  --post-connect-time-budget-ms 450 \
  --corridor-refine \
  --bridge-repaired-queries \
  --out-json outputs/paper/tro2026_exp08_parallel_full.json
```

## Acceptance Criteria

- If scaling is positive, report speedup and efficiency.
- If scaling is flat/negative, include root-cause stage breakdown and soften scalability claims.
- Do not cite scaling as a contribution unless the data supports it.
