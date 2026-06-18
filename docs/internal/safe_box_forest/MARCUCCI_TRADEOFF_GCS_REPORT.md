# Marcucci Path Visualization, Grower Trade-off, and Merger/GCS Report

Date: 2026-05-05

## 1. Current Path Visualization

Current Exp.4 SBF paths were exported with the same quality-aware defaults used by the latest Marcucci benchmark:

- Output JSON: `outputs/paper/marcucci_current_paths.json`
- Path-only JSON: `outputs/paper/marcucci_current_paths_only.json`
- Meshcat static HTML: `outputs/paper/marcucci_current_paths.html`

Single-seed visualization run:

| Metric | Value |
| --- | ---: |
| Build wall time | 0.380545 s |
| Boxes after corridor refinement | 146 |
| Segment edges | 5 |
| Successful paths | 5 / 5 |

| Query | Length | Repair count |
| --- | ---: | ---: |
| `AS->TS` | 2.817861 | 0 |
| `TS->CS` | 2.338472 | 1 |
| `CS->LB` | 3.140420 | 0 |
| `LB->RB` | 4.428957 | 0 |
| `RB->AS` | 2.551483 | 0 |

## 2. Grower Build-Time / Path-Quality Trade-off

Experiment script:

- `experiments/paper_04_grower_tradeoff.py`

Artifacts:

- JSON: `outputs/paper/marcucci_grower_tradeoff.json`
- CSV: `outputs/paper/marcucci_grower_tradeoff.csv`
- Markdown table: `outputs/paper/marcucci_grower_tradeoff.md`
- Curve plot: `outputs/paper/marcucci_grower_tradeoff.png`

Protocol:

- Sweep `quality_min_connected_boxes = 0, 16, 32, 64, 96, 128, 160, 224, 320, 512`.
- Use 5 seeds per setting.
- Disable the post-connect time cap with `post_connect_time_budget_ms = 0`.
- Disable corridor refinement and repaired-query bridge retry to isolate grower behavior.
- Quality objective: total median audited path length plus a small repair-count penalty.

Summary:

| Quality floor | Build mean (s) | Mean boxes | Path ratio | Repair median sum | Interpretation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 0 | 0.081262 | 14.6 | 1.1305 | 4.0 | Fast but too few boxes, methodologically close to repair fallback. |
| 16 | 0.101422 | 26.6 | 1.1305 | 4.0 | More boxes, no path-quality gain. |
| 32 | 0.121601 | 43.2 | 1.1258 | 4.0 | Small gain, still sparse. |
| 64 | 0.147444 | 73.8 | 1.1093 | 4.0 | Unconstrained knee/balance point. |
| 96 | 0.180706 | 102.2 | 1.1310 | 4.0 | First point inside the 80-200 box operating band. |
| 128 | 0.219044 | 142.4 | 1.1310 | 4.0 | Conservative high-box setting; useful once corridor refinement is enabled. |
| 160 | 0.240170 | 168.6 | 1.1310 | 4.0 | More boxes without pure-grower path gain. |
| 224 | 0.336990 | 233.4 | 1.1108 | 4.0 | Quality improves again, but outside the target 80-200 band. |
| 320 | 0.504469 | 332.0 | 1.1046 | 4.0 | Best pure-grower length in this sweep, but build cost jumps. |
| 512 | 0.910115 | 521.2 | 1.1108 | 4.0 | Extra grow cost with no further benefit. |

Relationship curve:

- Build time grows roughly with box count, from 0.08 s at 14.6 boxes to 0.91 s at 521.2 boxes.
- Pure-grower path quality is not monotonic. More boxes only help when the added boxes change endpoint selection or the graph corridor; otherwise query repair still dominates.
- The first meaningful quality drop occurs around 64 boxes, but this is slightly below the paper-facing 80-200 box band.
- Inside 80-200 boxes, 96 is the lowest-cost stable point, while the later paper sweep selected 64 as the global default once SBF-SH path post-processing and the post-connect time budget were enabled.
- Above 224 boxes the curve shows diminishing returns: 320 improves length only modestly relative to 224 while increasing build to about 0.50 s, and 512 regresses.

Practical balance points:

- Unconstrained knee: `quality_min_connected_boxes = 64`, build mean 0.147 s, mean boxes 73.8, path ratio 1.1093.
- Method-constrained lower bound: `quality_min_connected_boxes = 96`, build mean 0.181 s, mean boxes 102.2.
- Current recommended paper default: `quality_min_connected_boxes = 64` in the SBF-SH profile, with a 450 ms post-connect budget and collision-checked path post-processing.

## 3. Merger + GCS Attempt

Experiment script:

- `experiments/paper_04_merger_gcs.py`

Artifacts:

- JSON: `outputs/paper/marcucci_merger_gcs.json`
- Path JSON: `outputs/paper/marcucci_merger_gcs_paths.json`

Protocol:

- Enable SBF merger during build.
- Keep the current quality-aware grower and targeted corridor refinement.
- Convert generated SBF boxes to Drake `HPolyhedron.MakeBox` regions.
- Build a `LinearGCS` graph from geometric box intersections.
- Solve five Marcucci queries with GCS and post-audit the returned waypoint path using SBF collision checks.

Build result:

| Metric | Value |
| --- | ---: |
| Build wall time | 0.400455 s |
| Grow time | 0.213563 s |
| Merge time | 0.024432 s |
| Raw boxes before merger | 129 |
| Final boxes after merger | 89 |
| Boxes after corridor refinement | 106 |
| Segment edges | 5 |
| Geometric GCS edges | 1070 |

GCS result:

| Query | GCS solved | GCS length | Strict collision audit |
| --- | --- | ---: | --- |
| `AS->TS` | yes | 1.065163 | failed |
| `TS->CS` | yes | 0.738157 | failed |
| `CS->LB` | yes | 2.292639 | failed |
| `LB->RB` | yes | 3.010389 | failed |
| `RB->AS` | yes | 2.407658 | failed |

Interpretation:

- Merger itself works and is inexpensive in this setting: it reduced 129 raw boxes to 89 in about 24 ms.
- Drake/MOSEK GCS successfully solves all five shortest-path problems over the generated boxes.
- However, strict collision audit fails for all five GCS paths. The GCS paths are very short because the convex regions are coverage-heuristic provisional boxes, not fully certified collision-free regions.
- Therefore, direct GCS over current provisional SBF boxes is not yet a valid planner. It needs either certified/edge-validated regions, collision-aware GCS constraints, or a post-GCS repair/audit loop before it can replace the current SBF query path.

Next technical direction:

- Preserve merger, but protect query corridor boxes and segment-edge endpoints from unsafe over-merge.
- For GCS, build corridors from audited SBF query corridors rather than all provisional boxes.
- Add edge validation after GCS; invalid GCS segments should trigger targeted corridor paving or local repair, then rerun GCS on the repaired corridor.
- Longer term: introduce a certified-box mode or adaptive subdivision around GCS-selected regions so GCS does not optimize through provisional interiors.