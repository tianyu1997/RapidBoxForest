# Exp.7 Obstacle-Count Dynamic Update Notes

## Scope

`experiments/exp07_dynamic_update/run_dynamic_update.py` now measures adaptive
leaf-sweep maintenance cost as obstacle count changes. It does not reuse the
Exp.6 random-query catalog and does not run online planning.

The experiment answers a narrow question: given an existing RBF adaptive
partition for a scene, how expensive is a batched obstacle-count update from
the minimum count to the maximum count, and how does that compare with fresh
warm builds at both counts?

## Catalog

The runner writes a dedicated saved catalog:

```text
schema = tro2026_exp07_ordered_obstacle_update_v1
```

Each record contains:

- robot name;
- random seed;
- minimum and maximum obstacle count;
- ordered AABB obstacle list.

The list is generated once and reused. The paper-facing transition directly
adds all obstacles from the minimum-count prefix to the maximum-count prefix
and then removes that suffix as one update.

## Measured Pipeline

For each record:

1. Build an adaptive leaf-sweep partition for the minimum-count prefix.
2. Insert the saved obstacle suffix via `add_obstacles_and_rebuild`.
3. Separately run a fresh adaptive leaf-sweep build on the maximum-count scene.
4. Remove the suffix via `remove_obstacle_suffix_and_regrow`.

Only adaptive leaf sweep and partition update are measured. Query bridge,
connector repair, residual segments, OMPL simplification, and final path audit
are intentionally excluded.

## Update-and-Replan Diagnostic

The archived update-and-replan diagnostic is a separate reviewer-risk check,
not the source of the main dynamic-update table. It reuses the saved ordered
obstacle catalog, samples queries that are free in the target scene, then
compares:

1. the source fresh build plus query,
2. insert update plus post-update query,
3. target fresh rebuild plus query,
4. remove update plus post-update query.

With the paper table's matched maintenance profile
(`deep_max_boxes=1600`, `leaf_max_depth=12`, `adaptive_target_depth=16`,
`deep_ffb_depth=62`, no virtual topology), the 8-seed/1-query diagnostic at
`outputs/new_experiments/tro2026/exp07_update_replan_diagnostic_matched/`
returned 0/8 audited paths for both updated and freshly rebuilt
maintenance-only forests. This confirms that Exp.7 should be reported only as
scene-cache maintenance; it is not evidence for end-to-end dynamic replanning.

## Outputs

- `ordered_obstacle_catalog.json`: saved deterministic scenes.
- `dynamic_update_events.csv`: one row for the batched insert and one row for
  the batched remove event per scene.
- `dynamic_update_builds.csv`: initial and maximum-count full-build
  diagnostics per scene.
- `dynamic_update_summary.csv`: paper-facing summary with \([Q_1,Q_3]\)
  statistics for Warm@min, batched insert, batched remove, Warm@max, and
  warm-build/update speedup.
- `dynamic_update_by_count_summary.csv`: compatibility diagnostics for older
  obstacle-count tooling; it is not the main paper table source.
- `tab_tro_dynamic_update.tex`: paper table generated from the summary.

The CSV files report times in seconds, and the LaTeX table also displays
seconds as \([Q_1,Q_3]\) to keep the single-column table compact.
