# Exp.5 Cross-Robot Randomized-Scene Benchmark

## Claim

SBF behavior generalizes beyond shelf+IIWA, but the paper reports clear success and failure boundaries by robot, difficulty, and baseline.

## Hypotheses

- SBF should be competitive on repeated-query randomized scenes, especially when strict audit and reuse are valued.
- OMPL RRTConnect may dominate easy single-query cases; this should be reported rather than hidden.
- Failures should cluster by robot/difficulty/method, which informs limitations.

## Existing Runners

- `experiments/paper_05_random_robot_scenes.py`
- `experiments/paper_12_random_scene_rrt_baseline.py`
- `experiments/common_scene_sampling.py`

## Protocol

1. Generate deterministic random scenes for UR5 and Panda across easy/medium/hard where supported.
2. Use identical start/goal pairs across planners.
3. Run SBF variants and OMPL RRTConnect with the same SBF collision checker.
4. Add OMPL PRM/BIT* only if the same scene/query interface is available.
5. Record per-trial details, not only aggregate rows.

## Comparison Groups

- Robot: UR5, Panda, optional IIWA.
- Difficulty: easy, medium, hard if supported.
- Planner/method: SBF variants, OMPL RRTConnect, optional PRM/BIT*.

## Metrics

- Scene/query count.
- SR and audit SR with Wilson CI.
- Build/query/audit time.
- Path length.
- Repair count and fallback count.
- Failure category by robot/difficulty.

## Visualizations

- Success heatmap.
- Time/path scatter by robot.
- Failure taxonomy stacked bars.
- Representative success/failure panels.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_05_random_robot_scenes.py \
  --robots ur5 \
  --difficulties easy \
  --methods crit_link_coverage \
  --scene-seeds 1 \
  --out-json outputs/paper/tro2026_exp05_random_sbf_smoke.json

$PYTHON_SBF experiments/paper_12_random_scene_rrt_baseline.py \
  --robots ur5 \
  --difficulties easy \
  --scene-seeds 1 \
  --trials 1 \
  --timeout-ms 1000 \
  --out-json outputs/paper/tro2026_exp05_random_rrt_smoke.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_05_random_robot_scenes.py \
  --robots ur5,panda \
  --difficulties easy,medium \
  --methods ifk_strict,crit_link_coverage,coverage_hybrid \
  --scene-seeds 10 \
  --threads 8 \
  --task-batch-size 8 \
  --max-boxes 1500 \
  --timeout-ms 3000 \
  --max-consecutive-miss 1000 \
  --out-json outputs/paper/tro2026_exp05_random_sbf_full.json

$PYTHON_SBF experiments/paper_12_random_scene_rrt_baseline.py \
  --robots ur5,panda \
  --difficulties easy,medium \
  --scene-seeds 10 \
  --trials 5 \
  --timeout-ms 3000 \
  --out-json outputs/paper/tro2026_exp05_random_rrt_full.json
```

Random-scene generation retries deterministic obstacle layouts for each seed before declaring a scene invalid. The SBF budget is intentionally bounded per case so disconnected or adversarial random seeds are counted as failures rather than consuming the global 60 s construction timeout.

## Acceptance Criteria

- RRT and SBF share scene/query seeds.
- Audit SR is reported separately from raw SR.
- Weak diagnostic baselines are labeled diagnostic.
