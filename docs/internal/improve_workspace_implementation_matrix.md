# Improve.md Production Integration Matrix

Source plan:

- `docs/improve.md`
- bytes: `36292`
- lines: `1439`
- SHA-256: `a8b1a4947ef5b4dc669ea5048d459f19b71fbc3994d1b76462a7dc63403feae0`

Implementation boundary:

- The former sidecar directory `improve_workspace/` has been deleted.
- Production-relevant C-LECT mechanisms now live in the main `lect_database`
  and `safe_box_forest` modules.
- Historical sidecar artifact paths below are retained only as traceability
  labels for the original plan; they are not active code paths or runnable
  entry points.

## Coverage

| Plan section | Sidecar artifact | Status |
| --- | --- | --- |
| Overall terminal cell set | `clect_sidecar/adaptive_sweep.py`, `reports.py` | Implemented as terminal `FREE`, `CERT_OCCUPIED`, `INCONCLUSIVE`, `DEFERRED`, `COVERED` records. |
| 1. Early-stop adaptive sweep | `clect_sidecar/adaptive_sweep.py` | Implemented priority-driven `AdaptiveLeafSweep` with early free accept and depth-cap collision domains. |
| 1.2 Split accept depths | `AdaptiveSweepConfig.start_depth`, `max_depth`; production leaf/adaptive depth diagnostics | Implemented as separate sweep configuration knobs. Production `LeafSweepRefine` and adaptive builds now report sweep depth separately from seed FFB start/skip/max depth, with C++ regressions proving the knobs remain independent. |
| 1.4 StartCells | `dyadic.split_schedule_cells` | Implemented depth-synchronous start cells without global root recursion. |
| 1.5 Priority/relevance | `AdaptiveLeafSweep._priority`, `_relevant` | Implemented frontier, anchor, portal/domain, blocker/history hooks. |
| 1.6 Local split dimension | `AdaptiveLeafSweep._choose_split_dimension` | Implemented widest-interval plus blocker affected-joint scoring. |
| 1.7 Children selection | `AdaptiveLeafSweep._select_children` | Implemented optional targeted child selection for online repair mode. |
| 2. Validation report | `reports.py`; production `OracleValidationDetail.blockers` and `blocker_signature_hash` | Implemented sidecar `ValidationReport`, `Blocker`, status/stage enums, blocker signatures. Production now exposes blocker diagnostics with link/obstacle/stage/margin/overlap/affected-joint fields, full-overlap blocker lists, and stable top-k signatures. |
| 2.2 Occupied certificate | `occupied.py`, `ValidationReport.cert_occupied`, production AABB SDF material-point witness | Implemented signed-distance witness predicate, revolute motion bound, and `CERT_OCCUPIED` report generation. Production now supports the optional occupied certificate for AABB obstacles using fixed revolute-link centerline material points and a conservative motion bound; positive/negative C++ regressions cover the certificate gate. |
| 2.3 No-good cooldown | `AdaptiveLeafSweep._dominated_by_no_good` | Implemented blocker-signature dominance as heuristic deferral. |
| 2.4 Connectivity dominance | `connectivity.py`, production `AdaptiveGridPartitionConnectivityDominance`, adaptive frontier scoring/defer diagnostics | Implemented standalone sidecar decisions and production integration for covered cells, connector-candidate priority, main-frontier priority, single non-main deferral, and isolated-cell deferral. |
| 3. Dyadic address | `dyadic.py` | Implemented per-dimension levels/indices, intervals, split, LCA, jump-cell lookup, mixed-depth adjacency predicates. |
| 3.3 Patricia compression | `sparse_tree.py`, `dyadic.py`, production `AdaptiveGridPartition` sparse virtual-cell overlay | Implemented runtime sidecar `SparseNodeMap` and production sparse virtual-cell indexing for materialized partition cells. Production exact interval lookup no longer needs ancestor materialization. |
| 3.4 jump_child | `jump_cell_containing` | Implemented direct descendant address calculation. |
| 3.5 Replay compatibility | `dyadic.py`, `lect_database/include/LECTDatabase/sbf/oracle.h`, `lect_database/src/sbf/oracle.cpp`, `lect_database/tests/test_sbf_adapter.cpp`, `experiments/common/rbf_leaf_rrt.py` | Interval-key semantics are represented and production direct external replay is fail-closed: compatible identity and valid canonical frame are required, lookup uses `endpoint_for_box_exact`, key-only replay is blocked, and C++/Python/experiment counters expose compatibility diagnostics. |
| 3.6 VirtualCell/SparseNodeMap staging | `AdaptiveGridPartitionSparseCellRecord`, `sparse_virtual_record_for_intervals`, `test_adaptive_grid_partition_sparse_staging_records` | Implemented production sparse staging records for partition cells. Records expose root/grid coordinates, split counts, interval fingerprint, split-policy hash, address depth, exact interval lookup eligibility, and intervals suitable for LECT interval-key evidence replay. C++ tests verify exact hits through `SharedEndpointEvidenceCache` and misses for perturbed intervals. |
| 4. Portal edge compression | `portal.py`, `portal_search.py`; production `SegmentEdgeType::PortalCorridor`, `validate_portal_corridor_certificate`, `add_portal_corridor_edge` | Implemented `Portal`, `PortalCorridor`, `PortalGraph`, portal detection, greedy internal portal search, certificate validation, and lazy edge expansion. Production now has first-class compressed portal corridor edges with hidden `CertifiedFree` box-chain certificates, lazy waypoint expansion, segment-fraction exclusion, Python bindings, and C++ regression coverage. |
| 4.6 Portal membership | `portal.py`, `patch_proposals/integration_notes.md`, production `PortalMembershipPolicy`, endpoint membership diagnostics | Low-risk global membership policy is production-explicit: endpoint lookup uses the global forest/adaptive partition, and `PortalInteriorIndex` requests record unavailable/fallback diagnostics until an interior index exists. |
| 5. Combined C-LECT flow | `tools/synthetic_clect_benchmark.py`, `tools/run_clect_ablation_benchmark.py`, `tools/run_clect_experiment_suite.py`, `tools/run_clect_scaling_experiment.py` | Implemented synthetic flow, per-mechanism ablation metrics, Section-7-style sidecar variants, and multi-depth scaling curves for fixed, early-stop, no-good, sparse, portal, occupied-certificate, and full C-LECT. |
| 6. Paper changes | `patch_proposals/paper_changes.md` | Provided patch proposal text only; paper file is untouched. |
| 7. Experiment plan | `patch_proposals/experiment_plan.md`, `tools/synthetic_clect_benchmark.py`, `tools/run_clect_ablation_benchmark.py`, `tools/run_clect_experiment_suite.py`, `tools/run_clect_scaling_experiment.py`, `tools/plot_clect_experiments.py`, `tools/run_production_experiment_bridge.py`, `tools/summarize_improve_performance.py`, `tools/audit_improve_requirements.py`, `tools/audit_production_completion_status.py`, `tools/audit_improve_plan_sections.py` | Implemented the requested metric schema in a sidecar suite, multi-depth performance scaling experiment, figure generation, requirement audit, heading-level plan-section audit, production Exp.4/Exp.6 dry-run manifest bridge, executed-smoke production runner measurement, combined performance summary, and completion-level production status audit: `N_virtual_leaves`, `N_materialized_cells`, `N_validated_free`, collision/deferred counts, evidence lookups, graph vertices, portal edges, query success, path length, accepted-free depth histogram, and materialized-cell depth histogram. Production C-LECT full-profile integration remains a patch step. |
| 8. Recommended order | This matrix plus sidecar package | Implemented in order as a sidecar prototype. |

## Remaining Production Limitations

The following items remain proposals or smoke-verified production paths:

- Updating `paper/sbf_tro_2026.tex`.
- Running the full production C-LECT ablation/scale experiments beyond the
  current production smoke bridge.

The historical machine-readable requirement audit was generated by the deleted
prototype tree. It is not a supported entry point in the current repository.

Current result:

- requirement evidence coverage: `24/24`
- plan-section heading coverage: `46/46`
- missing evidence: `0`
- production-pending integration items: `13`
- scaling experiment depths: `8,10,12,14,16`
- best full C-LECT materialization reduction vs fixed in scaling sweep: `15.938x`
- generated figures: `5`
- production Exp.4/Exp.6 dry-run bridge: `ok=true`
- production Exp.4/Exp.6 dry-run manifests both declare
  `offline_query_agnostic_build=true`
- production Exp.4/Exp.6 executed-smoke bridge: `ok=true`
- executed-smoke metrics:
  - Exp.4 `5/5`, planning `1.274s`, online/query `0.0350s`,
    path `2.627`, segment `0.226`
  - Exp.6 `3/3`, planning `1.591s`, online/query `0.0169s`,
    path `3.697`, segment `0.074`
- combined performance summary: `performance_change_summary.md`

The historical production integration readiness report was also generated by
the deleted prototype tree.

Current result:

- production-pending items: `13`
- items with source anchors found: `13`
- source-anchor gaps exposed: `0`
- production integration complete: `true`
- completion-level production audit: `13/13` full production-complete items;
  all `24/24` requirements have sidecar/proposal/scaffold evidence.
- plan-section production-complete headings: `46/46`
- production C-LECT ablation/scale suite: `ok=true`
