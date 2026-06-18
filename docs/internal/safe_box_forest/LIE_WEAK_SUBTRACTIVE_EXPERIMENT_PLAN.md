# Weak LIE Subtractive Experiment Plan

Date: 2026-05-15

This document defines the evaluation protocol for the current weak-dependency subtractive route implemented in `experiments/lie_subtractive_weak.py`. The goal is to measure what the route already exposes today, keep the metrics aligned with the TRO build/query/audit conventions, and explicitly separate immediately runnable build-update diagnostics from downstream path-quality experiments that need a thin query layer on top of the exported cells.

## 1. Scope and current code surface

Route under test:

- `experiments/lie_subtractive_weak.py`: standalone grouped-obstacle subtractive prototype using only `link_interval_envelope`.
- `experiments/paper_17_subtractive_grouped_shelf.py`: full-SBF grouped subtractive reference when OMPL/SBF bindings are available.

The weak runner already emits:

- `timing_s.initial_build`
- `timing_s.subtractive`
- `timing_s.envelope_total`
- `envelope_calls`
- `initial.summary.{count,volume_*,unique_collision_count,unique_collision_volume,group_collision_count,group_collision_volume}`
- `subtractive_groups[*].{removed_count,removed_volume,regrown_count,regrown_volume,unresolved_count,unresolved_volume,active_count_after,active_volume_after}`
- `final_validation.{validation_pruned_count,validation_pruned_volume,final_count,final_volume}`
- optional `initial.cells` and `final_cells` when `--emit-cells` is enabled.

The weak runner does not yet emit:

- per-group wall-clock timing
- an adjacency graph over final cells
- endpoint localization/query results
- audited path length.

Therefore the evaluation is split into:

1. immediately runnable build/update experiments using the current JSON payload;
2. a query-overlay experiment that consumes exported cells and adds graph/path metrics without introducing SBF/OMPL as a hard dependency;
3. an optional exact-reference comparison once the full SBF or another exact checker is buildable on the machine.

## 2. Evaluation questions

The current route should answer four questions.

1. Does the count-first initial growth strategy pay a moderate upfront build cost to make later obstacle insertion collide with fewer boxes, with count prioritized over volume?
2. When collisions remain, is local subtractive regrow cheaper than rebuilding the same target scene from scratch?
3. After the grouped obstacle sequence is applied, does the surviving weak cell set still support later queries with acceptable success and path length?
4. How sensitive are these answers to initial strategy, carving mode, endpoint source, and envelope type?

## 3. Workloads

### 3.1 Scene family

Use the grouped Shelf/Bin/Table construction already embedded in the weak runner:

- `shelf`
- `bin_negative_y`
- `bin_positive_y`
- `table`

Run both carving modes:

- `exact`: per-obstacle AABB carving; this is the default scientific view.
- `aggregate`: one grouped AABB per obstacle group; this is the pessimistic control.

### 3.2 Edit schedules

Use four fixed grouped-obstacle schedules.

- `O1 canonical_insert`: `shelf -> bin_negative_y -> bin_positive_y -> table`
- `O2 reverse_delete`: reverse of `O1`; use it only for rebuild/update comparison after a fully built target scene exists.
- `O3 table_first`: `table -> shelf -> bin_negative_y -> bin_positive_y`
- `O4 interleaved`: `bin_negative_y -> shelf -> table -> bin_positive_y`

`O1` is the primary report. `O2` and `O3/O4` are order-sensitivity controls.

### 3.3 Query sets

Use two query sets.

- `Q_fixed`: the five canonical Shelf+IIWA queries once a query overlay exists.
- `Q_random`: 50 free-free sampled query pairs drawn from surviving final cells for each seed; this is the fallback when named queries are not yet wired into the weak route.

### 3.4 Seeds and budgets

Recommended seed counts:

- deterministic `split` initial strategy: 1 seed is enough unless tie-breaking randomness is later introduced.
- stochastic `sample` initial strategy: 5 seeds for smoke, 20 seeds for full runs.

Recommended budgets:

- smoke: `max_initial_cells in {16, 32}`, `local_regrow_depth in {2, 3}`
- full: `max_initial_cells in {64, 128}`, `local_regrow_depth in {4, 6}`

Recommended `sample` sweep:

- `sample_box_initial_radius_ratio in {1e-3, 1e-2}`
- `sample_box_grow_steps in {12, 24}`
- `sample_box_grow_factor = 2.0`

## 4. Experiment matrix

| ID | Purpose | Compare arms | Primary metrics | Output artifact |
|---|---|---|---|---|
| WSUB-01 | Initial collision concentration | `split` vs `sample`; `exact` vs `aggregate`; cell-budget sweep | `initial_build_s`, `envelope_total_s`, `envelope_calls`, `initial_cell_count`, `unique_collision_count`, `group_collision_count`, `unique_collision_volume`, `group_collision_volume` | `outputs/paper/tro2026_exp18_weak_subtractive_build.json` |
| WSUB-02 | Update locality and rebuild cost | `sample`, `sample_keep_colliding`, `split`; grouped schedules `O1/O3/O4`; matched-scene weak rebuild from scratch | `subtractive_s`, `removed_count`, `removed_volume`, `regrown_count`, `unresolved_count`, `final_volume`, rebuild/update speedup, unresolved-volume ratio | `outputs/paper/tro2026_exp19_weak_subtractive_update_vs_rebuild.json` |
| WSUB-03 | Downstream query cost and path quality | updated weak route vs matched weak full rebuild; optional full-SBF reference when available | `query_sr`, `weak_audit_sr`, `query_time_ms_p50`, `path_length_p50`, `path_length_gap_to_rebuild`, `hop_count_p50`, amortized time | `outputs/paper/tro2026_exp20_weak_subtractive_query_quality.json` |
| WSUB-04 | Representation and source ablation | endpoint source `ifk/crit/analytical`; envelope type `link_iaabb/kdop/support_hull`; primary schedule `O1` | build/update metrics from WSUB-01/02 plus query/path metrics from WSUB-03 when available | `outputs/paper/tro2026_exp21_weak_subtractive_ablation.json` |

## 5. Metric definitions

Keep the metric naming parallel to the existing TRO conventions.

### 5.1 Build metrics

- `initial_build_s = timing_s.initial_build`
- `subtractive_s = timing_s.subtractive`
- `envelope_total_s = timing_s.envelope_total`
- `envelope_share = envelope_total_s / max(initial_build_s + subtractive_s, 1e-9)`
- `cells_final = final_validation.final_count`
- `volume_final = final_validation.final_volume`

### 5.2 Count-first collision metrics

These are the primary objective metrics for the current route.

- `collision_box_count = initial.summary.unique_collision_count`
- `collision_group_hits = initial.summary.group_collision_count`
- `collision_box_volume = initial.summary.unique_collision_volume`
- `collision_group_hit_volume = initial.summary.group_collision_volume`
- `collision_box_rate = collision_box_count / max(initial.summary.count, 1)`
- `collision_group_rate = collision_group_hits / max(initial.summary.count * n_groups, 1)`

Ranking rule for analysis:

1. minimize `collision_box_count`
2. then minimize `collision_group_hits`
3. then minimize `collision_box_volume`
4. then minimize `initial_build_s`

This matches the current implementation objective more closely than sorting only by total volume.

### 5.3 Update-locality metrics

- `removed_ratio = removed_count / max(active_count_before, 1)`
- `removed_volume_ratio = removed_volume / max(active_volume_before, 1e-12)`
- `regrow_recovery_ratio = regrown_volume / max(removed_volume, 1e-12)`
- `unresolved_ratio = unresolved_volume / max(removed_volume, 1e-12)`
- `rebuild_speedup = matched_full_rebuild_s / max(subtractive_s, 1e-9)`

If per-group timers are added later, also record:

- `group_collision_check_s`
- `group_regrow_s`
- `group_total_s`

### 5.4 Query and path-quality metrics

Path-quality reporting must stay separate from build/update metrics.

- `query_sr`: graph search found a route through accepted cells.
- `weak_audit_sr`: the returned route passed the weak conservative audit described below.
- `exact_audit_sr`: exact checker success if and only if a full SBF or another exact checker is available.
- `query_time_ms_p50`: success-only median.
- `path_length_p50`: success-only median joint-space length.
- `path_length_gap_to_rebuild = (L_update - L_rebuild) / max(L_rebuild, 1e-9)`.
- `amortized_s_per_query(N) = (build_s + N * query_s) / N`.

Until an exact checker is available, label the path rows as `weak conservative audit`, not `strict final audit`.

## 6. Query-overlay protocol

WSUB-03 needs a small postprocessor on top of the existing `--emit-cells` JSON.

Required overlay steps:

1. read `final_cells` from the weak artifact;
2. construct a cell graph by interval overlap/touch in joint space;
3. localize start and goal to containing cells;
4. run Dijkstra or A* with edge weight equal to straight-line joint distance between box representatives or gateway points;
5. reconstruct the joint-space polyline;
6. weak-audit each segment by discretizing the segment and checking tiny interval boxes against the exact obstacle AABBs through the same LIE envelope route.

This overlay keeps the dependency set weak: it still uses `link_interval_envelope`, obstacle AABBs, and exported cell intervals only.

When the exact SBF route is buildable, the same query set should be rerun with:

- full-SBF subtractive grouped shelf as the exact reusable reference;
- PRM / RRTConnect / BIT* only for path-quality context, not for update-locality claims.

## 7. Minimal code additions required for full coverage

The current weak runner is sufficient for WSUB-01 smoke runs, but full coverage needs four small additions.

1. Add grouped-order control such as `--group-order shelf,bin_negative_y,...`.
2. Add per-group timing fields so WSUB-02 can attribute collision-check and regrow time instead of using one aggregate `subtractive_s`.
3. Keep `--emit-cells` on by default for experiment runs or provide a dedicated `--emit-final-cells-only` mode.
4. Add one lightweight query-overlay script, for example `experiments/lie_subtractive_weak_query_overlay.py`.

None of these require OMPL or the full SBF bindings.

## 8. Recommended run sets

### 8.1 Smoke

`split` baseline:

```bash
python experiments/lie_subtractive_weak.py \
  --initial-only \
  --initial-strategy split \
  --carving-mode exact \
  --max-initial-cells 32 \
  --max-initial-depth 5 \
  --out-json outputs/paper/tro2026_exp18_split_smoke.json
```

primary deployment route:

```bash
python experiments/lie_subtractive_weak.py \
  --initial-strategy sample \
  --carving-mode exact \
  --max-initial-cells 64 \
  --initial-samples 512 \
  --sample-box-initial-radius-ratio 0.01 \
  --sample-box-grow-factor 2.0 \
  --sample-box-grow-steps 24 \
  --local-regrow-depth 4 \
  --emit-cells \
  --out-json outputs/paper/tro2026_exp18_sample_smoke.json
```

regrow control:

```bash
python experiments/lie_subtractive_weak.py \
  --initial-strategy sample \
  --sample-keep-colliding \
  --carving-mode exact \
  --max-initial-cells 64 \
  --initial-samples 128 \
  --sample-box-initial-radius-ratio 0.01 \
  --sample-box-grow-factor 2.0 \
  --sample-box-grow-steps 16 \
  --local-regrow-depth 4 \
  --emit-cells \
  --out-json outputs/paper/tro2026_exp19_keep_colliding_smoke.json
```

### 8.2 Full

Recommended primary full run:

- `initial_strategy = sample`
- `carving_mode = exact`
- `max_initial_cells = 128`
- `initial_samples = 2048`
- `sample_box_initial_radius_ratio = 0.01`
- `sample_box_grow_steps = 24`
- `local_regrow_depth = 6`
- `seeds = 20`

Run the same parameter set under `aggregate` as the pessimistic control and under `sample_keep_colliding` as the regrow stress case.

## 9. Reporting rules

Follow the same reporting rules already used in the TRO scripts.

- Keep build, query, and audit in separate columns.
- Report path statistics only on successful audited queries.
- Report Wilson 95% confidence intervals for success rates once the total trial count exceeds a smoke test.
- Keep weak-only rows explicitly labeled `weak conservative audit` and do not merge them with the paper's strict audited rows.
- Do not claim update superiority over full SBF or OMPL baselines unless the compared row uses the same success semantics.

## 10. Exit criteria

The current route is ready for a paper-facing appendix row only when all of the following hold.

1. `sample` beats `split` on collision-box count at matched or lower final unresolved volume.
2. weak subtractive update is faster than matched weak full rebuild on `O1` and `O3`.
3. query-overlay success stays high enough to support path reporting, with clear weak-audit semantics.
4. at least one exact-reference row is available, or the appendix explicitly states that the route is reported only as a weak-dependency systems diagnostic.