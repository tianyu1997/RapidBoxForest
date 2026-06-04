# Experiment 4-6 Completion Audit, 2026-06-03

## Scope

This audit checks the current TRO manuscript scope in `paper/sbf_tro_2026.tex`
against the implemented LeafSweepRefine baseline and the current Experiment
4-6 artifacts. The goal is not to preserve an earlier stronger narrative; it is
to keep only claims that are supported by strict-audited current results.

## Implementation Evidence

- Leaf sweep refinement entry point:
  - `RBFPlanningForest::build_leaf_sweep_refined(...)` is implemented in
    `safe_box_forest/src/safe_box_forest.cpp`.
  - `LeafSweepRefineConfig` and `LeafSweepRefineResult` are exposed in
    `safe_box_forest/include/SBF/safe_box_forest.h`.
  - Python bindings expose `LeafSweepRefineConfig` and
    `SafeBoxForest.build_leaf_sweep_refined(...)` in
    `safe_box_forest/python/bindings.cpp`.
- Experiment runners:
  - Exp.4 leaf-refine ablation:
    `experiments/exp04_shelf_ablation/run_leaf_refine_ablation.py`.
  - Exp.4 trade-off runner:
    `experiments/exp04_shelf_ablation/run_leaf_refine_tradeoff.py`.
  - Exp.5 cross-algorithm dispatcher:
    `experiments/exp05_shelf_cross_algorithm/run_shelf_cross_algorithm.py`.
  - Exp.6 random dispatcher:
    `experiments/exp06_random_robot/run_random_robot.py`.
- API/test support:
  - `QueryResult.path_as_lists()` is bound for Python strict-audit consumers.
  - `safe_box_forest/tests/test_sbf.cpp` includes C++ leaf-refine empty-scene
    and domain-invariant tests.
  - `safe_box_forest/tests/test_python_api.py` includes a Python
    `build_leaf_sweep_refined` smoke test. The smoke toy explicitly disables
    canonical mode because its short joint-0 limit is not a valid full-sector
    rotational-symmetry test case.

## Exp.4 Shelf+IIWA Leaf-Refine Baseline

Authoritative artifact:

- `outputs/new_experiments/exp04_leaf_refine_ablation_opt_box200_full_20260603/`

Key rows from `leaf_refine_ablation_summary.csv/json`:

| row | success | planning median | path median | segment median | target3 max | boxes median |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline d23 SH box200/d28 | 8/8 | 175.7 ms | 12.760 | 0.021 | 0.023 | 1764 |
| baseline d23 SH box400/d28 | 8/8 | 259.5 ms | 12.760 | 0.021 | 0.023 | 1964 |
| baseline d23 SH box500/d24 | 8/8 | 380.7 ms | 12.158 | 0.000 | 0.000 | 2064 |
| no-cache SH box200/d28 | 8/8 | 257.1 ms | 13.363 | 0.119 | 0.043 | 1809 |
| no-cache AABB box200/d28 | 8/8 | 258.0 ms | 13.140 | 0.121 | 0.045 | 1810 |
| no-anchor roots box200/d28 | 0/8 | 247.2 ms | NA | NA | NA | 1809 |

Conclusion supported by Exp.4:

- The adopted fast baseline is `d23 + SupportHull + leaf8->14 + box200/d28`.
- The 12.760 median route passes the strict artifact protocol for all 8 seeds.
- The fast baseline is not box-only: it uses 2.1% median segment length.
- The box500/d24 point is the zero-segment quality point, at higher planning
  cost.
- The AABB comparison does not support universal SupportHull dominance. At the
  no-cache box200 budget, AABB is similar in runtime and slightly shorter in
  median path length, while SupportHull has slightly lower segment fraction.
- Priority anchor roots are required for this shelf passage; disabling them
  fails all 8 seeds.

Paper-facing generated table:

- `paper/sbf_old/generated/tab_tro_leaf_refine_ablation.tex`

Analysis note:

- `experiments/exp04_shelf_ablation/analysis_leaf_refine_20260603.md`

The obsolete `experiments/exp04_shelf_ablation/analysis_20260602.md` has been
deleted.

## Exp.5 Shelf Cross-Algorithm Check

Authoritative artifacts:

- `outputs/new_experiments/exp05_shelf_cross_algorithm_split_s3_20260603/`
- `outputs/new_experiments/exp05_iris_reduced_s1_r8_20260603/`

Key rows:

| method | success | charged time median | path median | note |
| --- | ---: | ---: | ---: | --- |
| SBF leaf-refine d23 box200 | 15/15 | 0.181 s | 12.760 | planning time excludes audit |
| PRM shared roadmap, 5 s grid | 13/15 | 16.236 s | 3.556 | cumulative incumbent accounting |
| BIT*, 5 s checkpoint | 12/15 | 34.495 s | 2.903 | cumulative incumbent accounting |
| RRTConnect, 10 s timeout | 15/15 | 0.169 s | 4.298 | query-only raw total |
| IRIS-NP+GCS r8 diagnostic | 0/5 | 121.207 s | NA | one-seed reduced diagnostic |

Conclusion supported by Exp.5:

- The updated SBF shelf baseline is much faster than the PRM/BIT* incumbent
  traces under the current artifact accounting and succeeds on every audited
  shelf query.
- RRTConnect is slightly faster as a one-shot query planner and produces much
  shorter paths. The manuscript must not claim SBF one-shot runtime dominance
  over RRTConnect on this table.
- The reduced Shelf IRIS-NP+GCS diagnostic is not competitive under the current
  configuration.

Paper-facing generated table:

- `paper/sbf_old/generated/tab_tro_shelf_cross_algorithm.tex`

Analysis note:

- `experiments/exp05_shelf_cross_algorithm/analysis_20260603.md`

## Exp.6 Balanced Random-Scene Pipeline Check

Authoritative artifacts:

- `outputs/new_experiments/exp06_random_robot_sbf_rrt_s1_20260603/`
- `outputs/new_experiments/exp06_random_robot_baselines_s1_20260603/`
- `outputs/new_experiments/exp06_iris_reduced_s1_r5_20260603/`

Key rows recomputed from panel records:

| method | success | time median | time max | path median |
| --- | ---: | ---: | ---: | ---: |
| SBF staged seed | 9/9 | 6.818 s | 7.074 s | 6.725 |
| SBF staged high | 9/9 | 34.034 s | 35.963 s | 6.411 |
| IRIS-NP+GCS r5 prefix | 9/9 | 4.159 s | 10.817 s | 6.248 |
| PRM, 1 s build grid | 9/9 | 3.109 s | 3.125 s | 6.404 |
| BIT*, 2 s checkpoint | 9/9 | 2.011 s | 2.014 s | 5.984 |
| RRTConnect | 9/9 | 0.0103 s | 0.0135 s | 6.868 |

Conclusion supported by Exp.6:

- The random-scene pipeline runs across IIWA, UR5, and Panda on the reduced
  balanced easy/medium/hard slice, and every reported row passes strict audit.
- This reduced run does not support staged-SBF runtime dominance on random
  one-shot queries. PRM, BIT*, RRTConnect, and the reduced IRIS prefix all have
  lower charged time than staged SBF on this slice.
- The manuscript correctly frames this as a reduced staged anytime
  generalization check, not as the Shelf+IIWA leaf-refine baseline and not as a
  full random-scene matrix.

Paper-facing generated table:

- `paper/sbf_old/generated/tab_tro_random_reduced.tex`

Analysis note:

- `experiments/exp06_random_robot/analysis_20260603.md`

## Manuscript Consistency

Checked claims in `paper/sbf_tro_2026.tex`:

- Exp.4 now reports the d23 SupportHull box200 fast baseline and the box500/d24
  zero-segment quality point.
- Exp.4 no longer claims universal SupportHull dominance over AABB.
- Exp.4 no longer describes the box200 row as box-only.
- Exp.5 explicitly avoids claiming one-shot dominance over RRTConnect.
- Exp.6 explicitly states the random run is reduced and does not support staged
  SBF dominance over single-query sampling planners.
- The conclusion/future-work scope now says broader random-scene seed counts,
  stronger cache transfer, and dependency-complete IRIS/GCS comparisons remain
  future work.

## Validation Commands

Executed successfully after the Python smoke canonical-mode fix:

```bash
ctest --test-dir build-leaf-sweep --output-on-failure
BUILD_DIR=build-leaf-sweep PYTHON_EXECUTABLE=/home/tian/miniconda3/envs/sbf/bin/python bash safe_box_forest/tests/run_all.sh
```

Observed result:

- 8/8 CTest targets passed.
- `safe_box_forest/tests/run_all.sh` completed with `All SBF tests passed.`

The first attempt to run the wrapper failed during `FetchContent` because
`nlohmann/json` was not yet available and network DNS was restricted. The same
command was rerun with approved network access, downloaded the dependency, and
then passed.

## Completion Status

For the current manuscript scope, Experiments 4-6 are complete and support the
revised paper narrative. The stronger rejected narratives are explicitly
unsupported and have been removed:

- SBF does not dominate RRTConnect as a one-shot planner on the shelf table.
- Staged SBF does not dominate single-query random-scene baselines in the
  reduced random slice.
- SupportHull is not universally better than AABB on Exp.4 path length.
- The fast box200 Shelf+IIWA baseline is not box-only.

