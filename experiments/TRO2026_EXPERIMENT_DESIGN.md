# TRO 2026 Experiment Design

This document is the implementation contract for the current `paper/sbf_tro_2026.tex`
experiment section. The goal is not to preserve the old experiment numbering, but
to produce a reproducible evidence chain that supports the manuscript claims.

## Principles

1. All paper-facing scripts live under `experiments/` and must not invoke
   bundled historical SBF experiment trees. Legacy baseline context, when used,
   must be supplied as an external artifact or explicit external script path.
2. Raw artifacts are authoritative. Tables, figures, and prose snippets are
   generated from JSON/JSONL/CSV artifacts, not copied by hand.
3. Final planning success means fixed-resolution final audit success. Audit time
   is reported separately and is not included in charged planning time.
4. Random scenes are generated once into a catalog, then reused by every method.
   Full runs must use catalog `reuse` or `verify` mode. The active catalog
   schema is `tro2026_random_scene_catalog_v5`; it includes an independent
   RRT-Connect feasibility gate so success rates are not contaminated by
   disconnected start/goal pairs.
5. Shelf and random planning rows use the current leaf-sweep + partition-native SBF
   backend. Segment bridges are fallback witnesses and must be reported
   separately from box-overlap graph edges.
6. Depth notation is LECT-tree depth. A label such as `d23` always refers to
   the depth in the cache's canonical tree and split schedule, not to any
   native/all-sector planning tree. Leaf sweep depths, free-box-finder depths,
   connector/paving depths, and `rbf_max_depth` are also LECT node depths in
   the active tree that generates or certifies boxes. Native query endpoints,
   paths, and final audit remain in native joint space; canonical mapping is an
   internal LECT evidence lookup operation only. Planner depths are reported
   separately and must not be folded into the cache depth label.

## Registered RBF Default

Unless an experiment row explicitly says otherwise, the RBF/SBF method uses the
shared algorithm profile `exp04_partition_leaf13_d23_fixed800_online25ms` from
`experiments/common/rbf_defaults.py`:

1. backend `build_leaf_sweep_refined`, offline grower `adaptive_deep_leaf`,
   online planner `partition_native`;
2. leaf sweep `d8 -> d13`, virtual topology enabled, parallel virtual
   validation;
3. deep offline refinement with `deep_max_boxes=400`, `deep_ffb_depth=62`,
   `refine_timeout_ms=800`, and query-agnostic build;
4. connector/query fallback is explicit: segment bridges remain fallback witnesses;
5. no query shortcut boxes, no collision shortcut, strict final path audit,
   audit time excluded from planning time.

Shelf+IIWA baseline rows additionally use the same profile with fixed d23 external
evidence cache and the canonical Shelf+IIWA cache root. The `d23` cache depth is
measured in that canonical LECT tree; all Shelf query endpoints and final-audit
paths are native joint-space values. Random multi-robot rows use the same
algorithm defaults but must use robot-local roots and must not reuse Shelf anchors
or the Shelf d23 cache. The 400-box default follows the current baseline sweep
practice with stable full-success over seeds `0..7`. Larger box budgets remain in
the trade-off curve and appendix sweeps; they are selected only when path-quality
gain outweighs build-time growth. Connector segment validation is matched to the
same `0.01` joint-space sample spacing as the final strict audit; earlier pilot
runs with looser connector validation are archived diagnostics and are not used for
paper-facing rows.

### 可调参数清单（本文有效）

以下参数为 Exp.4–Exp.6 的主流程可调参数；其余字段仅为诊断输出或已废弃项，不应作为实验扫描变量：

- leaf 阶段：
  - `leaf_start_depth`, `leaf_max_depth`
  - `use_virtual_topology`, `parallel_virtual_validation`, `leaf_threads`,
    `validation_batch_size`
  - `obstacle_cluster_gap`, `collision_overlap_prune_min_depth`,
    `collision_overlap_prune_threshold`,
    `collision_overlap_prune_ratio_threshold`
  - `adaptive_target_depth`
- deep/refine 阶段：
  - `deep_max_boxes`, `deep_ffb_depth`, `refine_timeout_ms`,
    `ffb_start_depth`, `ffb_search_mode`
  - `domain_seed_cap`, `domain_success_cap`, `domain_attempt_cap`
  - `adaptive_overlap_depth_threshold`,
    `adaptive_overlap_depth_min_threshold`,
    `adaptive_overlap_depth_decay_per_depth`,
    `adaptive_overlap_ratio_threshold`
  - `adaptive_seed_probe_count`, `adaptive_seed_anchor_probe_cap`
  - `adaptive_max_merge_ms`, `adaptive_max_merge_rounds`,
    `adaptive_max_merge_input_boxes`
  - `adaptive_max_merge_ms` 作为离线合并截止项，`adaptive_max_merge_rounds`
    限制合并轮次
- partition backend：
  - `adaptive_planning_backend`, `adaptive_grid_target_depth`,
    `adaptive_grid_planning_max_expansions`
  - `adaptive_grid_face_index_enabled` 在主实验中固定为 `true`，只作为
    debug/compatibility 开关，不作为 Exp.4--Exp.6 扫描变量
- connector/query（fallback）阶段：
  - `connector_pair_timeout_ms`, `connector_max_pairs_per_gap`
  - `connector_rrt_iters`, `connector_rrt_timeout_ms`,
    `connector_rrt_step_size`, `connector_rrt_goal_bias`
  - `connector_segment_resolution`
  - `connector_pave_depth`, `connector_pave_max_chain`, `connector_pave_steps`
  - `connector_pave_fill_gaps`, `connector_pave_require_connected_chain`
  - `query_bridge_pave_depth`, `query_bridge_direct_sample_step`,
    `query_bridge_adaptive_step_repair`,
    `query_bridge_adaptive_fine_step`,
    `query_bridge_adaptive_max_repair_subdivisions`,
    `query_bridge_adaptive_max_repair_calls`,
    `endpoint_main_target_k`, `endpoint_main_coarse_step`,
    `endpoint_main_fine_step`, `endpoint_main_max_ffb_calls`,
    `endpoint_main_max_boxes`, `endpoint_main_adaptive_ffb_depths`
  - `query_bridge_all`, `query_endpoint_anchor_before_bridge`,
    `query_bridge_labels`
- query 后处理：
  - `query_shortcut_boxes`, `final_collision_shortcut`,
    `final_rrt_simplify`, `final_rrt_simplify_timeout_ms`,
    `final_rrt_simplify_max_iters`, `final_rrt_simplify_attempts`
  - `audit_resolution`, `audit_segment_step`, `audit_collision_tolerance`

下列参数已移除、仅作旧 API 兼容，或已在当前主流程中固定；请勿继续在
Exp.4--Exp.6 的配置扫描、表格说明或 profile 名称中使用：

- `run_rrt_grower`, `rrt_grower_extra_boxes`, `rrt_grower_timeout_ms`
- `gap_fill_min_arc_gain`, `connector_adaptive_min_segment_fraction`
- `pre_split_to_max_depth`

论文主流程的铺箱语义固定为 boundary/front propagation；旧的全局 gap
candidate arc-gain 排序和 connector adaptive-depth segment-fraction 门控已经删除。
partition 邻接使用当前默认实现；已删除的实验性环境分支不再作为文档化参数、
扫描变量、profile 名称或 appendix 对照组的一部分。

## Main Experiments

### Exp.1 Endpoint Envelope

Purpose: justify endpoint interval sources used by downstream link envelopes.

Methods: IFK-AA, HIFK\_3, HIFK\_5, CritSample, Analytical, and MC from the
current `link_interval_envelope` binding. IFK/HIFK rows are certified endpoint
sources; CritSample, Analytical, and MC are retained as reference/negative
control rows because Table V uses them to expose the cost/volume trade-off.

Metrics: endpoint AABB volume, evaluation time, certified-source flag, and
relative volume against IFK-AA at the same width.

Paper output: compact mechanism table plus optional appendix detail.

### Exp.2 Link Envelope

Purpose: justify link-envelope collision filtering choices.

Fixed protocol: split count `S=1`.

Methods: Link AABB and SupportHull. The retired AABB->SupportHull row is not
part of the current paper-facing table.

Metrics: envelope construction time, obstacle/collision-check time, payload bytes,
candidate survival ratio, and false-positive/refinement ratio when available.

### Exp.3 LECT Performance

Purpose: isolate cache operations from planner behavior.

Operations: query, materialize, split, save, load, reopen, snapshot, and exact
evidence lookup.

Metrics: wall time distribution, node count, payload bytes, file size, VmRSS,
VmHWM, hit/miss counts, and replay materialization count.

### Exp.4 Shelf Leaf-Sweep + Query-Bridge

Purpose: main controlled planning study on the canonical Shelf+IIWA scene.

Baseline: d23 warm external evidence, AAFK, SupportHull, 8 threads,
AAFKVolumeMin split policy, `build_leaf_sweep_refined` + adaptive deep leaf
refine, fixed five Marcucci query pairs, seeds `0..7`. The registered default
budget is `400` deep boxes; the main trade-off curve sweeps `100/200/400/800`.

Ablations: no external LECT replay, envelope option, endpoint source, thread
count, split policy, anchor policy, leaf/deep depth, and box budget.

Metrics: build/query/audit time, final audited length, box count, depth
histogram, route box depth histogram, leaf/refine/connector timing, adjacency
islands, segment-edge count, raw no-post segment fraction, and target-gap
segment fraction.

### Exp.5 Shelf Cross-Algorithm

Purpose: contextualize Shelf+IIWA against common planners.

Methods: SBF, IRIS-NP+GCS, PRM, RRTConnect, BIT*.

Protocol: same scene, query set, seeds, final simplify rule, and `0.01`
joint-space final audit step. IRIS must receive enough region budget to be a
credible comparison, not a reduced diagnostic.

Current RBF is always regenerated from the current `build_leaf_sweep_refined`
implementation. Non-RBF baseline values may be reused from the old TRO artifact
only if the self-contained Exp.5 importer verifies the Shelf+IIWA scene, the
five canonical query names, the `0.01` final audit step, the OMPL planner
segment step, and source hashes. The importer excludes old SBF-SH rows.

Metrics: success rate, reusable build time, online query time, audited length,
and anytime curve checkpoints.

For the Shelf+IIWA RBF budget curve, `deep_max_boxes` is interpreted as the
adaptive partition free-box cap so that larger budgets correspond to larger
offline coverage. Random offline anchors are disabled in the registered Shelf
profile because the deterministic leaf sweep already covers the reusable shelf
basin; anchors are reserved for random-scene profiles where query-independent
coverage needs additional spatial dispersion.

### Exp.6 Random Multi-Robot Scenes

Purpose: test whether the SBF pipeline and baselines run coherently across
robots and scene difficulty.

Protocol: IIWA, UR5, and Panda; easy, medium, and hard; fixed scene catalog;
the manuscript run uses seeds 0--7 and budgets 100/200/400/800. Shelf anchors
must not be used. Random SBF anchors should be generated inside each robot root
with shallow LCA preference. UR5 and Panda use stateless d20 robot LECTDB
external evidence caches; IIWA uses the registered d23 cache.

Metrics: per robot/difficulty success rate, planning time, build/query split,
audited path length, segment fraction, and failure reason distribution. The
registered catalog is v7 `timed_probe_independent`: each query must have a
strictly valid RRTConnect difficulty probe accepted either by the
difficulty-specific first-solution time window or by the direct-obstruction
clearance window. The figure shows the complete budget curves; the table reports one
representative point per robot/difficulty, chosen as the fastest full-success
point within 8% of the shortest audited median path. Segment fraction remains
zero at the selected trade-off points. To match the old paper visual form, the
current figure/table includes PRM, RRTConnect, and BIT* rows rerun on the same
saved catalog. IRIS-NP+GCS remains imported from the old balanced
random-scene common-rule artifact until the external Drake/GCS pipeline is made
self-contained in the current runner; it is reported as protocol context rather
than a current-catalog row.

### Exp.7 Dynamic Update

Purpose: isolate the cost of maintaining an adaptive leaf-sweep partition as
the number of workspace obstacles changes. This experiment no longer reuses the
Exp.6 query catalog, because the independent variable is obstacle count rather
than random-scene planning difficulty.

Protocol: for each seed, generate and save an ordered obstacle list with schema
`tro2026_exp07_ordered_obstacle_update_v1`. Starting from the two-obstacle
prefix, build an adaptive leaf-sweep partition without query information, then
insert the next saved obstacle to reach three obstacles. From that
three-obstacle scene, remove the same obstacle back to two obstacles. The
experiment also records fresh warm adaptive leaf-sweep builds at two and three
obstacles. The sweep uses virtual topology, `deep_max_boxes=200`, and adaptive
depth checkpoints from d10 to d14; it descends past a checkpoint only if fewer
than 200 free partition cells have been retained. No query bridge, connector,
OMPL simplification, or post-hoc path audit is run in Exp.7.

Metrics: Warm@2 time, batched two-to-three insertion time, insertion speedup
relative to Warm@3, batched three-to-two removal time, removal speedup relative to
Warm@3, and Warm@3 time. The main paper table drops the robot column because
Exp.7 is a focused IIWA scene-maintenance study and reports \([Q_1,Q_3]\)
statistics over saved ordered random scenes. The manifest keeps all event
timings and build diagnostics for reproducibility.

## Appendix Sweeps

Appendix artifacts may include box budget, leaf start/max depth, deep FFB depth,
FFB start depth, sampling probability allocation, worker count, audit
resolution, segment fallback policy, anchor policy, and shortcut policy. These
studies must not replace the main registered rows.

## Table And Figure Rules

1. Figures show full trade-off curves whenever possible.
2. Tables report readable slices selected by a pre-registered rule.
3. A row is eligible only if it passes the common final audit.
4. For anytime curves, choose the highest success-rate frontier, then select the
   fastest point within 8% of that method/scenario's shortest audited median
   path unless the experiment document states a stricter rule.
5. Path length is not merged across robot types.

## Full-Run Acceptance

The experiment section is considered complete only when:

1. `paper/sbf_tro_2026.tex` no longer depends on bundled old-paper generated
   assets for experiment tables or figures.
2. `experiments/` paper-facing runners no longer call bundled historical SBF
   experiment scripts.
3. Exp.6 and Exp.7 consume saved scene catalogs in full runs.
4. `paper/generated/tro_table_generation_manifest.json` records the source
   artifact for every table and figure.
5. Smoke and py-compile checks pass for all new runners.
6. Main paper claims match measured artifacts, including cases where SBF is a
   latency/amortization trade-off rather than a shortest-path winner.
