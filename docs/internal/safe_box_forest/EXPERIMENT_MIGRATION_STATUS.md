# Standalone SBF Experiment Migration Status

Date: 2026-05-04

## TRO Completeness Map

- Exp.1 endpoint iAABB now has a standalone endpoint-only entry point: `cpp/SBF/experiments/paper_01_epiaabb_pipeline.py`. It calls the independent `link_interval_envelope` endpoint iAABB binding directly, samples widths randomly inside each width bin, and does not run SBF grower/build logic.
- Exp.2 link envelope now has a standalone SBF entry point: `cpp/SBF/experiments/paper_02_link_envelope_pipeline.py`. It reports standalone oracle/cache counters, not v6 disk-cache replay counters.
- Exp.3 Marcucci build/cache now has a standalone SBF entry point: `cpp/SBF/experiments/paper_03_marcucci_envelope_build.py`. Keep v6 `marcucci_envelope_build.json` as the paper source until the standalone matrix is calibrated.
- Exp.4 Marcucci combined SBF row now has a standalone entry point: `cpp/SBF/experiments/paper_04_marcucci_combined.py`; it now supports strict path audit and local BiRRT repair.
- Exp.5 random scenes now has a standalone SBF-only entry point: `cpp/SBF/experiments/paper_05_random_robot_scenes.py`. OMPL/Drake baselines are intentionally excluded from this migration pass.
- Exp.6 rebuild now has a standalone Marcucci-style entry point: `cpp/SBF/experiments/paper_06_obstacle_rebuild.py`. It validates the rebuild API but is not a full UR5/Panda replacement yet.

## Implemented Entry Points

### Current Implementation Checkpoint

- Connector `rrt_connect` now uses an explicit BiRRT-Connect implementation internally while preserving the public API name/signature.
- Query supports opt-in strict path audit with per-segment collision checking and query-local BiRRT repair.
- `PathAuditStatus`, audit timing, repair count, path decomposition, and remaining unsafe assumption counters are exposed to Python.
- `DatabaseBoxOracle` now caches validation results per scene/config/node and reports `oracle.validation_cache_hits` / `oracle.validation_cache_misses`.
- Smoke outputs written during implementation:
  - `cpp/SBF/outputs/paper/smoke_exp1.json`
  - `cpp/SBF/outputs/paper/smoke_exp2.json`
  - `cpp/SBF/outputs/paper/smoke_exp3_standalone.json`
  - `cpp/SBF/outputs/paper/smoke_exp4_audit_cache.json`
  - `cpp/SBF/outputs/paper/smoke_exp5.json`
  - `cpp/SBF/outputs/paper/smoke_exp1_endpoint_only.json`
  - `cpp/SBF/outputs/paper/epiaabb_pipeline_standalone_n400_random_widths_endpoint_only.json`
  - `cpp/SBF/outputs/paper/epiaabb_pipeline_standalone_sub2_n200_random_widths_endpoint_only.json`

### Exp.1 Endpoint IAABB Width Profile

Current standalone command semantics:

```bash
/home/tian/miniconda3/envs/sbf/bin/python experiments/paper_01_epiaabb_pipeline.py \
  --n-boxes 400 \
  --ref-samples 50000 \
  --out-json outputs/paper/epiaabb_pipeline_standalone_n400_random_widths_endpoint_only.json
```

Output: `cpp/SBF/outputs/paper/epiaabb_pipeline_standalone_n400_random_widths_endpoint_only.json`.

Protocol:

- Sources: IFK, CritSample, Analytical, MC.
- Default bins match v6 labels: 0.001-0.05, 0.05-0.10, 0.10-0.20, 0.20-0.50.
- Unlike the current v6 paper JSON, each trial samples actual joint widths uniformly inside the bin rather than using only the bin upper bound.
- `--subdivide-width-bins N` splits each configured bin into finer equal sub-bins; a 2-way run is stored at `cpp/SBF/outputs/paper/epiaabb_pipeline_standalone_sub2_n200_random_widths_endpoint_only.json`.
- The script uses `link_interval_envelope`'s endpoint-only pybind function, so Exp.1 no longer pays link-envelope construction cost.

Observed n=400 random-width comparison against `cpp/v6/experiments/results_paper/epiaabb_pipeline.json`:

- IFK timing is 0.68x-0.96x v6 across bins; endpoint volumes are 0.15x-0.43x v6 because random in-bin widths are smaller on average than v6's width-hi representative boxes.
- CritSample timing is 0.62x-0.81x v6; max negative gaps remain source-dependent and grow with width.
- Analytical timing is 0.53x-1.10x v6; the widest bin is slightly slower than v6, while narrower bins are faster under random widths.
- MC timing is 0.52x-0.89x v6; the 2-way sub-bin run shows the expected monotone growth from about 1.27 ms at 0.001-0.0255 to about 52.0 ms at 0.35-0.5.

Optimization implications for `cpp/link_interval_envelope`:

- MC is the dominant high-width cost and is sample-count driven; the standalone source now uses deterministic per-sample RNG and thread-local hull reductions for thread-level parallel sampling.
- Analytical spends most time in repeated scalar FK calls through phases 1-3; phases now share cached lo/mid/hi fit matrices and can parallelize phase work over endpoint-local reductions.
- CritSample is already microsecond-level for these iiwa14 boxes. Its sample-count parameter has been removed; it now defaults to automatic internal threading and exposes stateful incremental endpoint-only reuse in the Python API.
- IFK is already near the timing floor for isolated boxes; meaningful speedups are more likely from batch/incremental tree workloads than from one-box endpoint calls.

### Exp.4 Marcucci Combined

Primary command used for the current standalone comparison:

```bash
PYTHONPATH="$PWD/build_py310/python:$PWD/python" /home/tian/miniconda3/envs/sbf/bin/python \
  experiments/paper_04_marcucci_combined.py \
  --out-json outputs/paper/marcucci_combined_crit_link_3seed_rrt2s.json \
  --preset crit_link_coverage \
  --seeds 3 \
  --max-boxes 1200 \
  --timeout-ms 60000 \
  --ffb-depth 120 \
  --hard-frontier-failure-threshold 2 \
  --connector-rrt-timeout-ms 2000 \
  --connector-pair-timeout-ms 2000 \
  --connector-rrt-iters 30000 \
  --connector-max-pairs-per-gap 12
```

Output: `cpp/SBF/outputs/paper/marcucci_combined_crit_link_3seed_rrt2s.json`.

Observed result:

- Build mean: 14.92 s versus v6 paper mean 1.52 s.
- Box count: 1200 provisional boxes versus v6 paper mean unique count 4745.7.
- Segment edges: mean 2.0.
- Success rate: 5/5 queries at 3 seeds.
- Median path lengths: AS->TS 4.71, TS->CS 4.94, CS->LB 5.35, LB->RB 5.34, RB->AS 2.01.
- v6 median path lengths: AS->TS 2.05, TS->CS 2.91, CS->LB 3.57, LB->RB 3.89, RB->AS 1.90.

Interpretation:

- Segment edges and collision shortcut make the independent SBF query complete on this 3-seed run.
- Build is still about 9.8x slower than the v6 paper artifact under this standalone configuration.
- Query timing is much lower than v6 cached query timing because the standalone query currently reports graph lookup plus shortcut, not the same cached-query protocol used by v6.
- Paths are valid under exact segment shortcut checks but still longer for four pairs, indicating graph topology and connector edge placement need optimization.

### Exp.4 Coverage Hybrid Diagnostic

Command variant:

```bash
experiments/paper_04_marcucci_combined.py --preset coverage_hybrid --seeds 1 --max-boxes 1200 --ffb-depth 120
```

Observed result:

- Build mean before obstacle-grid cache: 23.8 s.
- Build mean after obstacle-grid cache: 22.0 s.
- Success rate: 3/5 for one seed.
- `oracle.grid_refinements` was about 116k in the diagnostic run.
- `oracle.obstacle_grid_builds` is now 1 and `oracle.obstacle_grid_cache_hits` is about 116.8k.

Interpretation:

- CritSample + HullGrid + no extra pad is the right safety-direction for coverage-first experiments, but obstacle-grid refinement is currently too expensive and still leaves islands.
- Use `crit_link_coverage` as the current standalone SBF paper-row migration while HullGrid is optimized.
- Obstacle-grid construction was not the main cost after caching; the remaining bottleneck is repeated per-node grid overlap refinement inside LECT query.

### Exp.6 Obstacle Rebuild

Command used:

```bash
PYTHONPATH="$PWD/build_py310/python:$PWD/python" /home/tian/miniconda3/envs/sbf/bin/python \
  experiments/paper_06_obstacle_rebuild.py \
  --out-json outputs/paper/obstacle_rebuild_standalone_1seed.json \
  --preset crit_link_coverage \
  --seeds 1 \
  --max-boxes 1200 \
  --timeout-ms 60000 \
  --ffb-depth 120 \
  --connector-timeout-ms 2000 \
  --connector-rrt-iters 30000
```

Output: `cpp/SBF/outputs/paper/obstacle_rebuild_standalone_1seed.json`.

Observed result:

- Build median: 15.05 s.
- Rebuild median: 5.75 ms.
- Collision check: 3.40 ms.
- Adjacency rebuild: 2.22 ms.
- Boxes removed: 468 / 1200, removal ratio 0.39.
- v6 random-scene reference median of group median rebuild time: 9.21 ms.

Interpretation:

- Standalone rebuild mechanics work and are fast on the Marcucci combined scene.
- This is not a full replacement for v6 Exp.6 because v6 aggregates UR5/Panda random scenes.

## Improvement Plan

1. Done: cache obstacle grids inside `DatabaseBoxOracle` for identical scene/grid-quality requests.
2. Done: replace connector RRT internals with explicit BiRRT-Connect and add BiRRT diagnostics.
3. Done: shortcut connector/query paths with exact collision-checked segment shortcut.
4. Done: try closest-box-point connector segments before rejecting candidates whose centers collide.
5. Done: add strict path audit and query-local BiRRT repair.
6. Done: cache per-node oracle validation results and expose cache hit/miss counters.
7. Done: add standalone Exp.1/2/3/5 SBF entry points without v6 runtime dependency.
8. Done: recalibrate standalone Exp.1 as an endpoint-only iAABB profile rather than an SBF grower/build profile.
9. Next: calibrate standalone Exp.2 metrics against the original microbench semantics before replacing paper JSON.
10. Next: expose a public Python wrapper for `compute_endpoint_iaabb_info` in `link_interval_envelope` instead of using the private `_cpp` module from the Exp.1 runner.
11. Next: improve HullGrid beyond validation caching by reducing repeated `query_aabb_gated_grid` refinement work.
12. Next: add connector candidate scoring that prefers shortest validated segment-edge length, not just island gap/center distance.
13. Next: migrate or independently define paper-parity UR5/Panda specs if exact Exp.5 parity becomes required.