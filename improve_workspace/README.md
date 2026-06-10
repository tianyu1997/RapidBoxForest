# Improve Workspace

This workspace contains sidecar implementations and validation artifacts for
RBF improvement plans.

Primary C-LECT source plan:

- Path: `docs/improve.md`
- Size: `36292` bytes
- Lines: `1439`
- SHA-256: `a8b1a4947ef5b4dc669ea5048d459f19b71fbc3994d1b76462a7dc63403feae0`

## Implemented In This Workspace

- `clect_sidecar/adaptive_sweep.py`: early-stop adaptive sweep, terminal cells,
  priority/relevance, local split scoring, targeted children, and no-good
  deferral.
- `clect_sidecar/reports.py`: richer validation report and blocker signature
  model.
- `clect_sidecar/occupied.py`: optional signed-distance occupied-certificate
  predicate.
- `clect_sidecar/connectivity.py`: connectivity-dominance budget controller for
  covered cells, component-connecting cells, single-component interior cells,
  and isolated cells.
- `clect_sidecar/dyadic.py`: per-dimension dyadic address, mixed-depth
  intervals, LCA, and jump-cell lookup.
- `clect_sidecar/sparse_tree.py`: runtime sparse node map that can materialize
  deep cells without materializing ancestors.
- `clect_sidecar/portal.py` and `portal_search.py`: compressed conservative
  portal corridors, lazy expansion, portal detection, and a greedy internal
  corridor search prototype.
- `tools/synthetic_clect_benchmark.py`: fixed virtual leaf vs adaptive sweep
  synthetic benchmark.
- `tools/run_clect_ablation_benchmark.py`: per-mechanism ablation benchmark for
  early-stop, occupied certificates, no-good deferral, sparse jump
  materialization, and portal compression.
- `tools/run_clect_experiment_suite.py`: Section-7-style sidecar experiment
  suite with fixed leaf sweep, early-stop, no-good, sparse dyadic, portal-edge,
  and full C-LECT variants.
- `tools/run_clect_scaling_experiment.py`: multi-depth scaling experiment for
  fixed, early-stop, occupied-certificate, no-good, sparse, portal, and full
  C-LECT sidecar effects.
- `tools/plot_clect_experiments.py`: figure generator for scaling curves and
  the two Section-7 depth histograms.
- `tools/run_production_experiment_bridge.py`: dry-run smoke bridge to the
  production Exp.4 shelf and Exp.6 random-scene runners. It verifies manifest
  compatibility from `improve_workspace` without claiming production C-LECT
  performance.
- `tools/audit_improve_requirements.py`: requirement-level audit that maps
  `docs/improve.md` plan items to sidecar artifacts, validation outputs, and
  production-pending integration boundaries.
- `tools/audit_production_integration_readiness.py`: source-anchor audit for
  the production-pending items, mapping them to concrete RBF/LECT files and
  symbols without modifying production code.
- `tools/audit_production_completion_status.py`: stricter completion-level
  audit that separates sidecar evidence from production integration and
  verifies the full production completion status.
- `tools/audit_improve_plan_sections.py`: heading-level audit that maps every
  `docs/improve.md` section title to its requirement evidence and production
  completion level.
- `patch_proposals/occupied_certificate_integration.md`: concrete production
  patch plan for optional signed-distance occupied certificates.
- `patch_proposals/interval_key_replay_integration.md`: concrete production
  patch plan for interval-key evidence replay compatibility.

## Hierarchical Partition Connectivity / HiPaC

`docs/分级partition连通.md` is implemented as an independent sidecar in
`clect_sidecar/hipac.py`.  It models the complete strategy in that document:

- `HiPaCCellState`: `FREE`, `CERT_OCCUPIED`, `MIXED`, `DEFERRED`, and
  `REFINED` cell states.
- `HierarchicalPartitionConnectivity.build()`: coarse partition
  classification, certified graph `G+`, optimistic graph `G~`,
  disconnected-component pair selection, optimistic path search, mixed-cell
  selection, and connectivity-driven refinement.
- `refine_mixed_cell()`: bounded domain-local refinement between entry/exit
  portals, anisotropic blocker-aware splitting, local certified free child
  graph search, and compressed conservative `PortalCorridor` insertion.
- `HiPaCCellSummary`: parent-cell boundary ports, certified portal pairs,
  unresolved portal pairs, blocker signatures, and sparse child provenance.
- `query()`: lazy hierarchical online query with endpoint attach/refine,
  certified path extraction, optimistic unresolved-cell repair, lifelong
  write-back accounting, and lazy portal expansion.
- `HiPaCMetrics`: connectivity-oriented metrics from the plan:
  certified component count, resolved/unresolved portal pairs, connected
  anchor pairs, `P_attach`, `P_samecomp`, online mixed-cell refinement count,
  and online repair time.

Validation entry point:

```bash
python3 improve_workspace/tools/run_hipac_validation.py
```

This writes:

- `improve_workspace/hipac_validation.json`
- `improve_workspace/hipac_validation.md`
- `improve_workspace/hipac_experiment_suite.json`
- `improve_workspace/hipac_experiment_suite.csv`
- `improve_workspace/hipac_experiment_suite.md`

## Validation

Run the complete sidecar validation:

```bash
python3 improve_workspace/tools/run_sidecar_validation.py
```

This writes:

- `improve_workspace/sidecar_validation.json`
- `improve_workspace/synthetic_clect_benchmark.md`
- `improve_workspace/clect_ablation_benchmark.json`
- `improve_workspace/clect_ablation_benchmark.csv`
- `improve_workspace/clect_ablation_benchmark.md`
- `improve_workspace/clect_experiment_suite.json`
- `improve_workspace/clect_experiment_suite.csv`
- `improve_workspace/clect_experiment_suite.md`
- `improve_workspace/clect_scaling_experiment.json`
- `improve_workspace/clect_scaling_experiment.csv`
- `improve_workspace/clect_scaling_experiment.md`
- `improve_workspace/clect_figures_manifest.json`
- `improve_workspace/clect_figures.md`
- `improve_workspace/hipac_validation.json`
- `improve_workspace/hipac_validation.md`
- `improve_workspace/hipac_experiment_suite.json`
- `improve_workspace/hipac_experiment_suite.csv`
- `improve_workspace/hipac_experiment_suite.md`
- `improve_workspace/production_experiment_bridge.json`
- `improve_workspace/production_experiment_bridge.md`
- `improve_workspace/production_experiment_bridge_executed.json`
- `improve_workspace/production_experiment_bridge_executed.md`
- `improve_workspace/production_clect_ablation.json`
- `improve_workspace/production_clect_ablation.md`
- `improve_workspace/performance_change_summary.json`
- `improve_workspace/performance_change_summary.md`
- `improve_workspace/production_bridge/catalogs/exp06_iiwa_easy_smoke_catalog.json`
- `improve_workspace/production_bridge/exp04_shelf_smoke_dry_run/shelf_leaf_rrt_manifest.json`
- `improve_workspace/production_bridge/exp06_random_smoke_dry_run/random_robot_manifest.json`
- `improve_workspace/production_bridge/exp04_shelf_smoke_executed/shelf_leaf_rrt_manifest.json`
- `improve_workspace/production_bridge/exp06_random_smoke_executed/random_robot_manifest.json`
- `improve_workspace/figures/clect_scaling_materialized_cells.png`
- `improve_workspace/figures/clect_scaling_reduction_factors.png`
- `improve_workspace/figures/clect_scaling_graph_vertices.png`
- `improve_workspace/figures/clect_accepted_free_depth_histogram.png`
- `improve_workspace/figures/clect_materialized_cell_depth_histogram.png`
- `improve_workspace/improve_requirements_audit.json`
- `improve_workspace/improve_requirements_audit.md`
- `improve_workspace/production_integration_readiness.json`
- `improve_workspace/production_integration_readiness.md`
- `improve_workspace/production_completion_status.json`
- `improve_workspace/production_completion_status.md`
- `improve_workspace/improve_plan_section_audit.json`
- `improve_workspace/improve_plan_section_audit.md`

Latest sidecar suite highlights at `start_depth=4`, `max_depth=12`:

- requirement evidence coverage: `24/24`
- sidecar validation commands: `14`
- plan-section audit: `46/46` headings mapped; all mapped headings have
  sidecar evidence.
- production-pending integration items: `13`
- production-pending items with source anchors found: `13/13`
- production source gaps exposed by readiness audit: `0`
- full production-complete C-LECT items: `13/13`
- production completion levels: `11` sidecar-complete non-pending items and
  `13` complete production items.
- production leaf/refine diagnostics now record sweep depth and seed-guided
  FFB depth separately, with C++ regressions proving the knobs are independent.
- production adaptive refinement now uses connectivity dominance: connector
  candidates and main-frontier cells are prioritized, while isolated and
  single non-main cells are deferred after `defer_min_depth` unless seed
  evidence protects them.
- occupied certificate now has a default-disabled production path backed by
  AABB signed-distance material-point witnesses. It samples fixed points on
  revolute active-link centerlines and certifies `Occupied` only when
  signed-distance plus revolute motion bound plus numerical margin is strictly
  negative.
- production validation detail now exposes blocker reports
  (`link/obstacle/stage/margin/overlap/affected_joints`) through C++ and Python,
  including full-overlap blocker lists and stable top-k blocker signatures.
- production interval-key replay now has an explicit compatibility gate:
  direct external evidence requires compatible robot/root/split/canonical/
  endpoint/envelope/payload identity, a valid canonical frame, and exact
  interval lookup through `endpoint_for_box_exact`; key-only replay is blocked
  and reported through C++/Python/experiment counters.
- production adaptive partition now carries a sparse virtual-cell runtime
  overlay keyed by root/grid intervals. It indexes only materialized terminal
  cells, supports exact interval lookup without materializing ancestor chains,
  and reports sparse-cell/index diagnostics through C++/Python/experiment
  manifests.
- production sparse staging records now expose interval-key replay metadata
  for materialized partition cells: root/grid coordinates, split counts,
  interval fingerprint, split-policy hash, address depth, and exact lookup
  eligibility. C++ regressions verify those intervals hit the shared LECT
  endpoint evidence cache exactly and reject perturbed intervals.
- production portal membership now has an explicit low-risk policy:
  `PortalMembershipPolicy::GlobalForestOnly` is the default, endpoint lookup
  uses the global forest/adaptive partition only, and a requested
  `PortalInteriorIndex` records unavailable/fallback diagnostics instead of
  silently treating hidden portal internals as endpoint owners.
- production portal corridor edges are now first-class typed graph edges:
  `SegmentEdgeType::PortalCorridor` carries a hidden conservative
  `CertifiedFree` box-chain certificate, is validated by exact overlap/touch
  adjacency, is excluded from segment-fraction accounting, and is lazily
  expanded during path extraction.
- fixed leaf sweep materialized cells: `4096`
- full C-LECT sidecar materialized cells: `272`
- full C-LECT materialization reduction vs fixed: `15.059x`
- sparse node reduction: `41.000x`
- occupied certificate materialization reduction vs fail-only control: `2.128x`
- full C-LECT synthetic query success: `true`
- production Exp.4/Exp.6 dry-run bridge: `ok=true`
- production Exp.4/Exp.6 dry-run manifests both declare
  `offline_query_agnostic_build=true`
- production Exp.4/Exp.6 executed-smoke bridge: `ok=true`
- executed-smoke Exp.4: `5/5` queries, planning median `1.315s`,
  online/query median `0.0342s`, path mean `2.730`, segment median `0.226`
- executed-smoke Exp.6: `3/3` queries on saved iiwa/easy catalog, planning
  median `1.635s`, online/query median `0.0170s`, path mean `3.697`,
  segment median `0.074`
- production C-LECT ablation/scale suite: `ok=true`, production-integrated
  `true`, sidecar-only `false`, with Exp.4 baseline budget scale (`6` rows,
  `3` summary rows) and Exp.6 saved-catalog RBF budget scale (`2` rows).

Latest scaling sweep highlights for `max_depth=8,10,12,14,16`:

- best full C-LECT materialization reduction vs fixed: `15.938x`
- best occupied-certificate reduction vs fail-only: `2.142x`
- best no-good reduction vs disabled repeated-blocker control: `7280.889x`
- full C-LECT materialized cells at depth 16: `4112` vs fixed `65536`

## Integration Boundary

The current completion audit reports full production integration for the
production-relevant `docs/improve.md` items: `13/13` are production-complete,
all `46/46` plan headings are mapped with sidecar evidence, and the production
C-LECT ablation/scale suite has been executed.
