# TRO 2026 Experiment Design

This document is the implementation contract for the current `paper/sbf_tro_2026.tex`
experiment section. The goal is not to preserve the old experiment numbering, but
to produce a reproducible evidence chain that supports the manuscript claims.

## Principles

1. All paper-facing scripts live under `experiments/` and must not invoke scripts
   from `safe_box_forest/experiments/sbf_old/`.
2. Raw artifacts are authoritative. Tables, figures, and prose snippets are
   generated from JSON/JSONL/CSV artifacts, not copied by hand.
3. Final planning success means fixed-resolution final audit success. Audit time
   is reported separately and is not included in charged planning time.
4. Random scenes are generated once into a catalog, then reused by every method.
   Full runs must use catalog `reuse` or `verify` mode. The active catalog
   schema is `tro2026_random_scene_catalog_v5`; it includes an independent
   RRT-Connect feasibility gate so success rates are not contaminated by
   disconnected start/goal pairs.
5. Shelf and random planning rows use the current leaf-sweep + RRT grower SBF
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
shared algorithm profile `exp04_leaf_sweep_rrt_b100_d34` from
`experiments/common/rbf_defaults.py`:

1. backend `build_leaf_sweep_refined`, grower mode `RRT`;
2. leaf sweep `d8 -> d14`, virtual topology enabled, serial virtual validation;
3. restricted deep refinement with `deep_max_boxes=100`, `deep_ffb_depth=34`,
   `refine_timeout_ms=800`, and endpoint/corridor priorities canonicalized into
   the active robot root;
4. bounded RRT grower enabled after refinement with `rrt_grower_extra_boxes=25`
   and `rrt_grower_timeout_ms=100`;
5. build-stage RRT segment edges enabled, `segment_edges_fallback_only=false`,
   BiRRT connector enabled with at most one 50 ms pair attempt per gap;
6. no query shortcut boxes, no collision shortcut, strict final path audit,
   audit time excluded from planning time.

Shelf+IIWA baseline rows additionally use the profile
`exp04_leaf_sweep_rrt_d23_b100_d34`, which enables the fixed d23 read-only
external evidence cache and the restricted Shelf+IIWA canonical root. The `d23`
cache depth is measured in that canonical LECT tree; all Shelf query endpoints
and final-audit paths are native joint-space values. Random
multi-robot rows use the same algorithm defaults but must use robot-local roots
and must not reuse Shelf anchors or the Shelf d23 cache. The 100-box default is
registered because the Exp.4 full run over seeds `0..7` passed all five
canonical Shelf+IIWA queries in every run, with median planning time `0.610 s`,
mean audited path length `2.517 rad`, and median raw segment fraction `0.022`.
Larger box budgets remain in the trade-off curve and appendix sweeps; they are
not the default because they trade lower path length for higher build time and
larger segment-witness fraction. Connector segment validation is matched to the
same `0.01` joint-space sample spacing as the final strict audit; earlier pilot
runs with looser connector validation are archived diagnostics and are not used
for paper-facing rows.

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

### Exp.4 Shelf Leaf-Sweep + RRT Grower

Purpose: main controlled planning study on the canonical Shelf+IIWA scene.

Baseline: d23 warm external evidence, AAFK, SupportHull, 8 threads,
AAFKVolumeMin split policy, `build_leaf_sweep_refined`, RRT grower/connector
mode, fixed five Marcucci query pairs, seeds `0..7`. The registered default
budget is `100` deep boxes; the main trade-off curve sweeps `100/200/400/800`.

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
current saved v5 `balanced_independent` run has no failures in 288 RBF budget
rows. The figure shows the complete budget curves; the table reports one
representative point per robot/difficulty, chosen as the fastest full-success
point within 8% of the shortest audited median path. Segment fraction remains
zero at the selected trade-off points. To match the old paper visual form, the
current figure/table includes PRM, RRTConnect, and BIT* rows rerun on the same
saved v5 catalog. IRIS-NP+GCS remains imported from the old balanced
random-scene common-rule artifact until the external Drake/GCS pipeline is made
self-contained in the current runner; it is reported as protocol context rather
than a current-catalog row.

### Exp.7 Dynamic Update

Purpose: evaluate the current dynamic-update implementation, not the retired
old Exp.7.

Protocol: saved random scene catalog, deterministic obstacle insertion/deletion
transitions, fresh warm rebuild baseline, and the same final audit policy used
elsewhere.

Metrics: invalidated boxes, promoted boxes, dirty domains, update time, warm
rebuild time, query success, segment fallback ratio, and audited path length.
The current pilot uses schema `tro2026_random_scene_catalog_v5`, seeds `0..7`,
and the registered RBF default. It passes all 32 target strict audits; two
insertion transitions require endpoint segment recovery after the first
incremental query audit fails, so Table VIII reports speedup while the manifest
keeps the fallback and segment-fraction diagnostics.

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

1. `paper/sbf_tro_2026.tex` no longer depends on `paper/sbf_old/generated` for
   experiment tables or figures.
2. `experiments/` paper-facing runners no longer call `sbf_old` scripts.
3. Exp.6 and Exp.7 consume saved scene catalogs in full runs.
4. `paper/generated/tro_table_generation_manifest.json` records the source
   artifact for every table and figure.
5. Smoke and py-compile checks pass for all new runners.
6. Main paper claims match measured artifacts, including cases where SBF is a
   latency/amortization trade-off rather than a shortest-path winner.
