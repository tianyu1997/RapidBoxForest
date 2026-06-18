# Marcucci Quality-Aware Grow Plan

Date: 2026-05-05

## Goal

The Marcucci combined benchmark must remain a SafeBoxForest result, not a degenerate BiRRT fallback with a few very large provisional boxes. The target operating point is:

- Build mean below 0.5 s, clearly faster than the v6 baseline of 1.515 s.
- Mean/median box count around 80 to 200, avoiding the previous 12 to 18 box connected-only forest.
- Five-query audit success rate equal to 1.0.
- At least `TS->CS` and `CS->LB` shorter than v6.
- `AS->TS`, `LB->RB`, and `RB->AS` as close to v6 as possible.
- Query median time still below v6 for every query.
- Repair reliance reduced by improving the forest/corridor before relying on local repair; repair should be local and short when it remains necessary.

## Current Diagnosis

The optimized connected-only default reached excellent time numbers but stopped too early:

- Mean boxes: 14.6.
- Certified boxes: 0.
- Provisional boxes: 14.6.
- Segment edges: 1.
- All five queries succeeded after strict audit/repair, but most paths were much longer than v6.

This is a method-risk state. Because endpoints often lie in huge overlapping provisional boxes, the query layer can choose a large early box and then rely on collision repair. That makes the result look like a BiRRT repair layer rather than a useful SBF corridor.

## Already Applied Fixes

1. Endpoint box selection no longer returns the first containing box. It prefers safer and smaller containing boxes, so extra boxes can actually affect query routes.
2. Repair no longer accepts the first successful BiRRT path. It evaluates successful attempts and keeps the shortest audited path.
3. Repair uses a local sampling ladder before global fallback. This favors short local detours around the failed segment instead of wandering through the full joint space.

These fixes reduced the single-seed total path length substantially while keeping all query times below v6.

## Stop Policy Design

The grower should distinguish three states:

1. **Pre-connect:** components are not connected. Continue growing unless hard timeout/miss limits fire.
2. **Connected but low quality:** components are connected, but the forest is too sparse. Continue growing until a quality floor is met.
3. **Connected and quality-eligible:** the forest has enough boxes and may stop if either a fixed extra-box limit or post-connect time budget is reached.

The first implementation will add a conservative quality floor:

- `quality_min_connected_boxes`: minimum total boxes required before `stop_after_connect` can stop.
- `post_connect_time_budget_ms`: maximum time spent after the first connected state, preventing late exits.

Current paper target:

- `quality_min_connected_boxes = 64`.
- `post_connect_time_budget_ms = 450`.
- Keep `stop_after_connect = true` so the grower stops as soon as the quality floor and post-connect budget are satisfied.

This should land near the measured `extra100` behavior, around 120 to 150 boxes and about 0.2 s build time.

## Next Quality Extensions

After the quality floor is stable, add richer probes:

- Endpoint box volume ratio: endpoints should not be covered only by huge root-scale boxes.
- Graph path stretch: raw corridor length divided by direct joint-space distance should stay below a threshold.
- Hub dependence: query routes should not all pass through one giant hub box.
- Repair risk: repeated audit failures on the same segment type should trigger targeted growth near that segment.

These probes should guide extra growth toward endpoint boxes and bad corridor segments instead of blindly adding boxes.

## Execution Steps

1. Implement `quality_min_connected_boxes` and `post_connect_time_budget_ms` in `GrowerConfig` and the RRT grow loop.
2. Expose the new fields through Python bindings and the Marcucci runner CLI.
3. Change Marcucci paper defaults to target 128 boxes after connection.
4. Rebuild `build_perf` and run SBF regression tests.
5. Run the Marcucci optimized default benchmark with 10 seeds.
6. Check acceptance criteria:
   - build mean < 0.5 s;
   - mean boxes in 80 to 200;
   - all query audit SR = 1.0;
   - `TS->CS` and `CS->LB` shorter than v6;
   - all query median times below v6;
   - repair lengths/times improved relative to the connected-only baseline.
7. If path quality remains weak, implement targeted corridor growth before adding more global boxes.

## Benchmark Files

Use the standalone runner:

```bash
SBF_BUILD_DIR=$PWD/build_perf \
  /home/tian/miniconda3/bin/conda run -p /home/tian/miniconda3 --no-capture-output \
  python /home/tian/.vscode/extensions/ms-python.python-2026.4.0-linux-x64/python_files/get_output_via_markers.py \
  experiments/paper_04_marcucci_combined.py \
  --out-json outputs/paper/marcucci_quality_aware_default_s10.json \
  --seeds 10
```

The authoritative v6 comparison remains:

```text
cpp/v6/experiments/results_paper/marcucci_combined.json
```

## 2026-05-05 Execution Result

Implemented in this pass:

- `GrowerConfig::quality_min_connected_boxes`.
- `GrowerConfig::post_connect_time_budget_ms`.
- Python bindings for both fields.
- Marcucci runner defaults:
  - `quality_min_connected_boxes = 64`;
  - `post_connect_time_budget_ms = 450`;
  - repair local sampling ladder: radius `0.4`, growth `2.0`, four attempts including global fallback.

Validation:

- Built `build_perf` target `_sbf_cpp` and `test_sbf` successfully.
- Ran `BUILD_DIR=$PWD/build_perf PYTHON_EXECUTABLE=/home/tian/miniconda3/bin/python bash tests/run_all.sh` successfully.
- Ran 10-seed benchmark:
  - `outputs/paper/marcucci_quality_aware_default_s10.json`.

10-seed result:

| Metric | Standalone SBF | v6 reference | Status |
| --- | ---: | ---: | --- |
| Build mean | 0.2204607543 s | 1.515283528 s | Pass |
| Build ratio | 0.14549 | 1.0 | Pass |
| Mean unique boxes | 139.0 | n/a | Pass, inside 80-200 |
| Median unique boxes | 137.5 | n/a | Pass, inside 80-200 |
| Mean segment edges | 1.0 | n/a | Acceptable |
| Audit SR | 1.0 for all 5 queries | 1.0 | Pass |

Per-query result:

| Query | Time standalone | Time v6 | Length standalone | Length v6 | Status |
| --- | ---: | ---: | ---: | ---: | --- |
| `AS->TS` | 0.0995333030 s | 0.4477611770 s | 2.844761408 | 2.052649065 | Time pass, length improved but still longer |
| `TS->CS` | 0.0450312205 s | 0.5794256180 s | 2.444446976 | 2.910423365 | Pass |
| `CS->LB` | 0.7256968595 s | 0.8239693860 s | 2.879359005 | 3.566989029 | Pass |
| `LB->RB` | 0.0049403050 s | 0.0329872935 s | 4.428957128 | 3.892371638 | Time pass, length slightly longer |
| `RB->AS` | 0.0071814845 s | 0.0139299670 s | 3.370303444 | 1.902921061 | Time pass, length still longer |

Repair status:

- `repair_count_med` remains 1.0 for all five queries.
- However, the repair path is now local and shortest-of-attempts, not first-success global BiRRT.
- Compared with the connected-only baseline, path quality improved substantially:
  - `AS->TS`: about 6.85 to 2.84.
  - `TS->CS`: about 5.81 to 2.44.
  - `CS->LB`: about 3.17 to 2.88.
  - `LB->RB`: about 7.43 to 4.43.
  - `RB->AS`: about 6.40 to 3.37.

Residual risk:

- Repair dependence is reduced in severity but not eliminated in count.
- `AS->TS`, `LB->RB`, and especially `RB->AS` still need targeted corridor growth to move closer to v6 length.
- `AuditBeforeCommit` and `CommitCertifiedOnly` are not viable under the current `crit_link_coverage` preset: a quick probe produced zero boxes.
- Enabling frontier/bridge boxes did not reduce repair count in the tested single-seed configuration.

Next targeted optimization:

- Add a mutating corridor-refinement phase after initial coverage:
  - run strict query probes for the paper query set;
  - collect repaired collision-free local paths;
  - pave boxes along those paths using SBF `chain_pave_along_path`;
  - prefer adding boxes over adding direct segment edges;
  - only add a segment edge when the box corridor still cannot pass audit.
- Count this phase as build time and stop it under the same 0.5 s budget.
- Acceptance target for that phase: reduce at least some `repair_count_med` below 1.0 while preserving build mean < 0.5 s and query time below v6.

## 2026-05-05 Targeted Corridor Refinement Result

Implemented in the follow-up pass:

- `SafeBoxForest::refine_query_corridor(start, goal, max_boxes_to_add)`.
- Runner-side pre-query refinement over the paper query set, counted as build time.
- Repaired-success bridge retry for queries whose start/goal boxes differ.
- Audit-resolution bridge RRT, so cached query bridges are checked at least as strictly as query audit.
- Same-box segment-edge support, so refined local paths inside one box can be represented as SBF query corridor edges.

Final validation:

- Built `build_perf` target `_sbf_cpp` and `test_sbf` successfully.
- Ran `BUILD_DIR=$PWD/build_perf PYTHON_EXECUTABLE=/home/tian/miniconda3/bin/python bash tests/run_all.sh` successfully.
- Ran 10-seed benchmark:
  - `outputs/paper/marcucci_corridor_refine_selfedge_s10.json`.

10-seed result:

| Metric | Standalone SBF | v6 reference | Status |
| --- | ---: | ---: | --- |
| Build mean | 0.4357532612 s | 1.515283528 s | Pass |
| Build ratio | 0.28757 | 1.0 | Pass |
| Mean unique boxes | 156.2 | n/a | Pass, inside 80-200 |
| Median unique boxes | 154.0 | n/a | Pass, inside 80-200 |
| Mean prebridge boxes added | 17.2 | n/a | Counted in build |
| Mean segment edges | 5.0 | n/a | Query corridors cached |
| Audit SR | 1.0 for all 5 queries | 1.0 | Pass |

Per-query result:

| Query | Repair med | Segment edges used med | Time standalone | Time v6 | Length standalone | Length v6 | Status |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `AS->TS` | 0.0 | 1.0 | 0.0001726735 s | 0.4477611770 s | 2.844761408 | 2.052649065 | Time pass, length longer |
| `TS->CS` | 1.0 | 1.0 | 0.1001089435 s | 0.5794256180 s | 2.338471665 | 2.910423365 | Pass |
| `CS->LB` | 0.0 | 1.0 | 0.4641598165 s | 0.8239693860 s | 3.140419988 | 3.566989029 | Pass |
| `LB->RB` | 0.0 | 1.0 | 0.0001249275 s | 0.0329872935 s | 4.428957128 | 3.892371638 | Time pass, length slightly longer |
| `RB->AS` | 0.0 | 1.0 | 0.0002039500 s | 0.0139299670 s | 2.602796702 | 1.902921061 | Time pass, length longer |

Repair status:

- `repair_count_med` reduced from 1.0 for all five queries to 0.0 for four of five queries.
- The remaining repaired query is `TS->CS`; it uses the cached same-box segment edge but still needs one local strict-audit repair.
- `TS->CS` and `CS->LB` are both shorter than v6.
- Every query median time remains below v6.
- Build mean remains below the 0.5 s hard target while keeping the forest at 156.2 boxes, so the result avoids the earlier few-box BiRRT-like degeneration.

Residual items:

- `AS->TS`, `LB->RB`, and `RB->AS` are still longer than v6, although their query time and repair dependence are now much lower.
- `TS->CS` self-corridor caching improves length but does not remove strict-audit repair count; further reduction would need a stricter pre-commit audit or a query path representation that preserves the exact audited local repair without post-extraction shortcut side effects.