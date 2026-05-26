# Exp.9 Random-Scene Dynamic Rebuild

## Claim

SBF can update an already built random-scene forest under monotone obstacle-set edits faster than an explicit warm rebuild on the same random-scene SBF stage, while preserving post-update strict-audit accounting.

## Protocol

Use the balanced random-scene generator in prefix mode. For a fixed robot and scene seed, generate the hard scene once; the easy, medium, and hard scenes are prefixes of the same obstacle list. This gives matched start/goal pairs and nested obstacle sets:

- easy: 4 obstacles
- medium: 8 obstacles
- hard: 12 obstacles

Run only two directed paths:

- Few-to-many: easy -> medium -> hard. Build easy, then insert obstacles one by one to medium and hard using `SafeBoxForest::add_obstacle_and_rebuild`.
- Many-to-few: hard -> medium -> easy. Build hard, then remove suffix obstacle batches down to medium and easy using `SafeBoxForest::remove_obstacle_suffix_and_regrow`.

For each transition, run a separate warm rebuild baseline on the same path: start from the same source stage in the same `SafeBoxForest`, reuse its LECT/cache state, then call `build_coverage` on the target obstacle prefix. Both incremental and warm rebuild use the random-scene base timing defaults and the selected random-anytime SBF stage from `--sbf-stages`/`--sbf-stage` (default: `balanced`, i.e. the same stage definition used by `paper_15_random_anytime_tradeoff.py`). The primary reported speedup is the displayed warm median divided by the displayed incremental median. A cold from-scratch target-stage baseline may still be emitted as auxiliary context.

## Metrics

- Initial build time and initial box count for each direction.
- Dynamic update wall time and profile time.
- Target warm-rebuild time and incremental speedup over warm rebuild.
- Optional target cold-build time and dynamic speedup over cold build.
- Boxes removed by insertion, boxes added by removal regrow, and final box count.
- Dirty-region anchors, dirty seeds, regrow attempts, and warm-rebuild fallback rate for removal.
- Post-update query success, strict-audit success, query time, repair count, and path length.

The default removal policy does not trigger a warm rebuild merely because the dirty region is large. Deleting obstacles cannot invalidate already-certified boxes, so a broad dirty region is treated as an optional coverage-recovery opportunity: the forest is retained, local regrowth is budgeted, and full target-stage rebuild time is reported separately as the cold baseline. Warm rebuild remains available for empty forests or explicit threshold/fraction settings.

## Acceptance Criteria

- Few-to-many and many-to-few use the same random-scene seed, robot, start/goal, and obstacle ordering.
- Obstacle removal never treats deleted blocker IDs as free-space proof; it only triggers local regrow candidates that still pass oracle validation.
- Every reported success is strict-audit success when `--strict-path-audit` is enabled.
- If local removal regrow falls back to warm rebuild, the fallback is reported explicitly.

## Smoke Command

```bash
PYTHONPATH=build_py310/python:python:experiments \
  /home/tian/miniconda3/envs/sbf/bin/python experiments/paper_09_random_dynamic_rebuild.py \
  --robots iiwa \
  --scene-seeds 1 \
  --scene-profile balanced \
  --sbf-stage fast \
  --no-enable-connector \
  --out-json outputs/paper/tro2026_exp09_random_dynamic_smoke.json
```

## Full Command

```bash
PYTHONPATH=build_py310/python:python:experiments \
  /home/tian/miniconda3/envs/sbf/bin/python experiments/paper_09_random_dynamic_rebuild.py \
  --robots iiwa,ur5,panda \
  --scene-seeds 5 \
  --threads 8 \
  --task-batch-size 8 \
  --sbf-stage balanced \
  --out-json outputs/paper/tro2026_exp09_random_dynamic_rebuild.json
```