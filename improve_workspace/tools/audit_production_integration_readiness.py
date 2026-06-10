#!/usr/bin/env python3
"""Map improve.md production-pending items to concrete source anchors.

This does not apply production changes.  It records where each sidecar mechanism
would have to enter the existing RBF/LECT implementation and verifies that the
expected source anchors still exist.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class SourceAnchor:
    path: str
    patterns: tuple[str, ...]
    role: str


@dataclass(frozen=True)
class PendingIntegration:
    req_id: str
    title: str
    production_goal: str
    anchors: tuple[SourceAnchor, ...]


PENDING_INTEGRATIONS: tuple[PendingIntegration, ...] = (
    PendingIntegration(
        "R1.1",
        "AdaptiveLeafSweep / AdaptiveClassify replaces fixed-depth virtual layer enumeration",
        "Route production build_leaf_sweep_refined / adaptive build through an early-stop terminal-cell controller.",
        (
            SourceAnchor("safe_box_forest/src/safe_box_forest.cpp", ("build_adaptive_deep_leaf_sweep_cover", "terminal_controller_enabled", "fast_virtual_checkpoint_mode", "adaptive_frontier_score"), "adaptive terminal-cell production controller"),
            SourceAnchor("safe_box_forest/src/leaf_sweep_grower.cpp", ("LeafSweepGrower::sweep", "compose_final_sets"), "fixed virtual leaf sweep implementation"),
            SourceAnchor("safe_box_forest/include/SBF/safe_box_forest.h", ("AdaptiveLeafSweepConfig", "fast_virtual_checkpoint_mode"), "public config surface"),
            SourceAnchor("safe_box_forest/python/bindings.cpp", ("AdaptiveLeafSweepConfig", "fast_virtual_checkpoint_mode"), "Python API surface"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("adaptive.terminal_controller_enabled", "adaptive.fast_virtual_checkpoint_mode", "adaptive.fast_checkpoint_mode"), "terminal-controller regression tests"),
            SourceAnchor("experiments/common/rbf_defaults.py", ("terminal_controller", "fast_virtual_checkpoint_mode"), "experiment default profile"),
            SourceAnchor("experiments/common/rbf_leaf_rrt.py", ("adaptive_fast_virtual_checkpoint_mode", "fast_virtual_checkpoint_mode"), "runner config propagation"),
        ),
    ),
    PendingIntegration(
        "R1.2",
        "Separate sweep accept depth from seed-guided FFB skip depth",
        "Ensure adaptive sweep validation can accept at shallow sweep depth while FindFreeBox keeps independent seed skip controls.",
        (
            SourceAnchor("safe_box_forest/include/SBF/find_free_box.h", ("skip_to_depth", "start_depth", "max_depth"), "seed FFB controls"),
            SourceAnchor("safe_box_forest/include/SBF/safe_box_forest.h", ("shallow_start_depth", "target_max_depth"), "adaptive sweep controls"),
            SourceAnchor("safe_box_forest/src/find_free_box.cpp", ("skip_to_depth", "validate_depth"), "seed FFB validation logic"),
            SourceAnchor("safe_box_forest/src/safe_box_forest.cpp", ("record_depth_semantics_diagnostics", "sweep_seed_ffb_depths_independent"), "adaptive/refine build depth diagnostics"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("leaf_refine.sweep_seed_ffb_depths_independent", "adaptive.sweep_seed_ffb_depths_independent"), "production depth-semantics regression tests"),
        ),
    ),
    PendingIntegration(
        "R1.8",
        "Complexity statement and paper integration text",
        "Update manuscript algorithms and complexity discussion once production profile is selected.",
        (
            SourceAnchor("paper/sbf_tro_2026.tex", ("AdaptiveLeafSweepRefine", "N_{\\rm mat}", "fixed virtual layer"), "applied adaptive sweep algorithm and complexity text"),
            SourceAnchor("paper/sbf_tro_2026.tex", ("Sparse interval staging", "endpoint\\_for\\_box\\_exact"), "applied sparse interval staging manuscript text"),
            SourceAnchor("improve_workspace/patch_proposals/paper_changes.md", ("AdaptiveLeafSweep", "portal"), "retained sidecar paper patch provenance"),
        ),
    ),
    PendingIntegration(
        "R2.1",
        "Rich validation report with blockers, stage, margin, overlap, affected joints",
        "Extend production oracle validation from BoxValidation/OracleValidationDetail to richer blocker records.",
        (
            SourceAnchor("lect_database/include/LECTDatabase/sbf/oracle.h", ("BoxValidation", "OracleValidationDetail", "OracleValidationBlocker", "blockers", "blocker_signature_hash"), "oracle validation detail surface"),
            SourceAnchor("lect_database/src/sbf/oracle.cpp", ("dominant_blocker_active_link_index", "make_oracle_blockers", "blocker_signature_hash"), "oracle classifier implementation"),
            SourceAnchor("safe_box_forest/python/bindings.cpp", ("blockers", "blocker_signature_hash", "blocker_affected_joints"), "Python diagnostic surface"),
            SourceAnchor("lect_database/tests/test_sbf_adapter.cpp", ("test_unknown_validation_reports_dominant_blocker", "test_full_overlap_stats_reports_blocker_list"), "production blocker-report regression tests"),
            SourceAnchor("safe_box_forest/src/leaf_sweep_grower.cpp", ("validate_node", "outcome.validation"), "leaf validation consumer"),
            SourceAnchor("safe_box_forest/src/find_free_box.cpp", ("validate_node", "BoxValidation"), "FFB validation consumer"),
        ),
    ),
    PendingIntegration(
        "R2.2",
        "Optional signed-distance occupied certificate predicate",
        "Use a production signed-distance material-point witness before any Occupied classification is used as a sound prune.",
        (
            SourceAnchor("lect_database/include/LECTDatabase/sbf/oracle.h", ("Occupied", "OracleValidationDetail"), "occupied classification surface"),
            SourceAnchor("lect_database/src/sbf/oracle.cpp", ("signed_distance_to_aabb", "revolute_material_point_motion_bound", "try_material_point_occupied_witness", "Occupied"), "AABB SDF material-point witness provider and occupied classification path"),
            SourceAnchor("link_interval_envelope/include/sbf/envelope/envelope_collision.h", ("collision", "aabb_overlap"), "current envelope collision detail source"),
            SourceAnchor("lect_database/tests/test_sbf_adapter.cpp", ("test_occupied_certificate_aabb_witness_certifies_occupied", "test_occupied_certificate_motion_bound_rejects_weak_witness"), "production occupied-certificate positive/negative regressions"),
            SourceAnchor("improve_workspace/clect_sidecar/occupied.py", ("MaterialWitness", "occupied_report_from_witness"), "sidecar witness predicate"),
            SourceAnchor("improve_workspace/patch_proposals/occupied_certificate_integration.md", ("MaterialPointOccupiedWitness", "enabled=false", "BoxValidation::Occupied"), "production patch proposal"),
        ),
    ),
    PendingIntegration(
        "R2.4",
        "Connectivity dominance for covered, connector, single-component, and isolated cells",
        "Wire component-aware defer/prioritize decisions into production adaptive refinement budget allocation.",
        (
            SourceAnchor("safe_box_forest/include/SBF/adaptive_grid_partition.h", ("AdaptiveGridPartitionConnectivityDominance", "classify_connectivity_dominance"), "partition connectivity dominance API"),
            SourceAnchor("safe_box_forest/src/adaptive_grid_partition.cpp", ("classify_connectivity_dominance", "connector_candidate", "covered_by_existing"), "partition connectivity dominance implementation"),
            SourceAnchor("safe_box_forest/src/box_graph.cpp", ("find_islands", "boxes_connected"), "legacy graph component state"),
            SourceAnchor("safe_box_forest/src/safe_box_forest.cpp", ("adaptive_connectivity_dominance", "deferred_connectivity_isolated", "deferred_connectivity_single_component"), "adaptive budget connectivity dominance policy"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("classify_connectivity_dominance", "connector_candidate", "isolated_dominance"), "production connectivity dominance regression tests"),
            SourceAnchor("improve_workspace/clect_sidecar/connectivity.py", ("classify_connectivity_dominance", "ConnectivityAction"), "sidecar decision model"),
        ),
    ),
    PendingIntegration(
        "R3.3",
        "Sparse Patricia / skip-tree overlay without materializing ancestors",
        "Replace or augment heap-node materialization with dyadic interval-keyed sparse storage or an equivalent runtime sparse overlay.",
        (
            SourceAnchor("safe_box_forest/include/SBF/adaptive_grid_partition.h", ("sparse_virtual_cell_for_intervals", "sparse_virtual_ancestor_refs_avoided", "SparseCellKey"), "production sparse virtual-cell overlay API and stats"),
            SourceAnchor("safe_box_forest/src/adaptive_grid_partition.cpp", ("rebuild_sparse_virtual_index", "sparse_address_depth", "sparse_virtual_index_"), "production sparse virtual-cell exact index"),
            SourceAnchor("experiments/common/rbf_leaf_rrt.py", ("partition_sparse_virtual_cells", "partition_sparse_virtual_ancestor_refs_avoided"), "experiment sparse overlay diagnostics"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("sparse_virtual_cell_for_intervals", "sparse_virtual_ancestor_refs_avoided"), "production sparse overlay regression"),
            SourceAnchor("improve_workspace/clect_sidecar/sparse_tree.py", ("SparseNodeMap", "jump_materialize"), "sidecar sparse overlay"),
        ),
    ),
    PendingIntegration(
        "R3.5",
        "Interval-key replay compatibility proposal",
        "Move production evidence compatibility toward robot/envelope/interval keys rather than heap-id-only semantics.",
        (
            SourceAnchor("lect_database/include/LECTDatabase/sbf/oracle.h", ("IntervalEvidenceCompatibility", "interval_replay_key_only_blocked", "external_evidence"), "oracle interval replay compatibility API and counters"),
            SourceAnchor("lect_database/src/sbf/oracle.cpp", ("interval_evidence_compatibility", "endpoint_for_box_exact", "interval_replay_key_only_blocked"), "fail-closed external evidence lookup path"),
            SourceAnchor("lect_database/src/lect_database/database.cpp", ("endpoint_for_box_exact", "box_to_node_exact", "split_policy_hash"), "interval-exact evidence lookup backend"),
            SourceAnchor("safe_box_forest/python/bindings.cpp", ("interval_replay_compatibility_checks", "interval_replay_key_only_blocked"), "Python counter/manifest surface"),
            SourceAnchor("experiments/common/rbf_leaf_rrt.py", ("interval_replay_compatibility_checks", "interval_replay_key_only_blocked"), "experiment manifest replay diagnostics"),
            SourceAnchor("lect_database/tests/test_sbf_adapter.cpp", ("test_direct_external_replay_blocks_key_only_identity_mismatch", "interval_replay_key_only_blocked"), "fail-closed replay regression"),
            SourceAnchor("improve_workspace/patch_proposals/integration_notes.md", ("EvidenceKey", "interval"), "sidecar integration proposal"),
            SourceAnchor("improve_workspace/patch_proposals/interval_key_replay_integration.md", ("IntervalEvidenceCompatibility", "endpoint_for_box_exact", "canonical evidence"), "production patch proposal"),
        ),
    ),
    PendingIntegration(
        "R3.6",
        "Stage-1 VirtualCell overlay and Stage-2 SparseNodeMap path",
        "Add a production VirtualCell address path without rewriting LECTDB first, then migrate persistence.",
        (
            SourceAnchor("safe_box_forest/include/SBF/adaptive_grid_partition.h", ("AdaptiveGridPartitionSparseCellRecord", "sparse_virtual_record_for_intervals", "interval_fingerprint"), "production sparse staging record API"),
            SourceAnchor("safe_box_forest/src/adaptive_grid_partition.cpp", ("make_sparse_virtual_record", "fingerprint_intervals", "sparse_virtual_records"), "sparse staging record implementation"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("test_adaptive_grid_partition_sparse_staging_records", "SharedEndpointEvidenceCache", "endpoint_for_box_exact"), "interval-key evidence staging regression"),
            SourceAnchor("experiments/common/rbf_leaf_rrt.py", ("partition_sparse_virtual_cells", "partition_sparse_virtual_exact_index_entries"), "experiment sparse staging diagnostics"),
            SourceAnchor("improve_workspace/clect_sidecar/dyadic.py", ("DyadicAddress", "jump_cell_containing"), "sidecar virtual cell address"),
            SourceAnchor("improve_workspace/clect_sidecar/sparse_tree.py", ("SparseNodeMap", "SparseLectNode"), "sidecar sparse map"),
        ),
    ),
    PendingIntegration(
        "R4.1-R4.5",
        "Compressed conservative portal edge, certificate validation, and lazy expansion",
        "Add a typed portal/corridor edge distinct from segment shortcuts and expand it during query path extraction.",
        (
            SourceAnchor("safe_box_forest/include/SBF/api.h", ("PortalCorridor", "internal_boxes", "conservative_certificate"), "production portal edge API"),
            SourceAnchor("safe_box_forest/include/SBF/box_graph.h", ("validate_portal_corridor_certificate", "add_portal_corridor_edge"), "portal edge graph API"),
            SourceAnchor("safe_box_forest/src/box_graph.cpp", ("validate_portal_corridor_certificate", "SegmentEdgeType::PortalCorridor", "extract_waypoints"), "certificate validation and lazy expansion"),
            SourceAnchor("safe_box_forest/src/adaptive_grid_partition.cpp", ("SegmentEdgeType::PortalCorridor", "partition_counts_as_segment_edge"), "partition segment-fraction semantics"),
            SourceAnchor("safe_box_forest/python/bindings.cpp", ("PortalCorridor", "ConservativeBoxChain", "internal_boxes"), "Python portal edge surface"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("test_portal_corridor_edge_expands_hidden_chain", "add_portal_corridor_edge"), "production portal edge regression"),
            SourceAnchor("improve_workspace/clect_sidecar/portal.py", ("PortalCorridor", "validate_certificate", "expand_edge"), "sidecar portal model"),
        ),
    ),
    PendingIntegration(
        "R4.6",
        "Portal membership policy documented",
        "Wire the chosen low-risk global-only endpoint membership policy and expose PortalInteriorIndex as unavailable until a real interior index exists.",
        (
            SourceAnchor("safe_box_forest/include/SBF/safe_box_forest.h", ("PortalMembershipPolicy", "GlobalForestOnly", "portal_membership_policy"), "explicit portal membership policy config"),
            SourceAnchor("safe_box_forest/src/safe_box_forest.cpp", ("record_portal_membership_policy", "locate_box_partition_first", "global_forest_only_fallback"), "global-only endpoint membership implementation and diagnostics"),
            SourceAnchor("safe_box_forest/python/bindings.cpp", ("PortalMembershipPolicy", "portal_membership_policy"), "Python policy binding surface"),
            SourceAnchor("experiments/common/rbf_leaf_rrt.py", ("portal_membership_policy", "portal_membership_global_forest_only"), "experiment manifest policy diagnostics"),
            SourceAnchor("safe_box_forest/tests/test_sbf.cpp", ("test_portal_membership_global_only_policy", "PortalInteriorIndex"), "global-only policy regression"),
            SourceAnchor("improve_workspace/patch_proposals/integration_notes.md", ("PortalInteriorIndex", "membership"), "sidecar membership proposal"),
        ),
    ),
    PendingIntegration(
        "R6",
        "Paper text changes proposed",
        "Apply manuscript updates only after production profile and real experiments are validated.",
        (
            SourceAnchor("paper/sbf_tro_2026.tex", ("Sparse interval staging", "Signed-distance occupied pruning", "Conservative portal expansion"), "applied C-LECT manuscript changes"),
            SourceAnchor("paper/sbf_tro_2026.tex", ("E_{\\pi}", "Portal edges", "AdaptiveLeafSweepRefine"), "applied graph and algorithm manuscript changes"),
            SourceAnchor("improve_workspace/patch_proposals/paper_changes.md", ("C-LECT", "AdaptiveLeafSweep", "portal"), "proposal retained for traceability"),
        ),
    ),
    PendingIntegration(
        "R7",
        "Experiment plan metrics and ablation suite",
        "Run production shelf+IIWA and random-scene experiments with the production C-LECT profile.",
        (
            SourceAnchor("experiments/exp04_shelf_leaf_rrt/run_shelf_leaf_rrt.py", ("run_leaf_rrt", "configure_leaf_rrt"), "Exp.4 shelf runner"),
            SourceAnchor("experiments/exp06_random_robot/run_random_robot.py", ("run_leaf_rrt", "run_rbf_scene"), "Exp.6 random runner"),
            SourceAnchor("safe_box_forest/python/bindings.cpp", ("build_adaptive_deep_leaf_sweep_cover", "AdaptiveLeafSweepConfig"), "Python binding surface"),
            SourceAnchor("improve_workspace/tools/run_production_clect_ablation.py", ("production_ablation_scale", "Exp.4", "Exp.6"), "production C-LECT ablation runner"),
            SourceAnchor("improve_workspace/production_clect_ablation.json", ("production_clect_integrated", "production_ablation_scale", "manifests"), "executed production ablation artifact"),
            SourceAnchor("improve_workspace/tools/run_clect_experiment_suite.py", ("fixed_leaf_sweep", "full_clect"), "sidecar experiment control"),
        ),
    ),
)


def check_anchor(anchor: SourceAnchor) -> dict[str, Any]:
    path = REPO_ROOT / anchor.path
    exists = path.exists()
    text = path.read_text(encoding="utf-8", errors="replace") if exists else ""
    pattern_results = [
        {"pattern": pattern, "present": pattern in text}
        for pattern in anchor.patterns
    ]
    return {
        "path": anchor.path,
        "role": anchor.role,
        "exists": exists,
        "patterns": pattern_results,
        "ok": exists and all(item["present"] for item in pattern_results),
    }


def audit() -> list[dict[str, Any]]:
    rows = []
    for item in PENDING_INTEGRATIONS:
        anchors = [check_anchor(anchor) for anchor in item.anchors]
        rows.append({
            "req_id": item.req_id,
            "title": item.title,
            "production_goal": item.production_goal,
            "anchors_ok": all(anchor["ok"] for anchor in anchors),
            "anchors": anchors,
        })
    return rows


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Production Integration Readiness",
        "",
        "This is a source-anchor audit for the production-pending items in `docs/improve.md`.",
        "It does not claim production integration is complete.",
        "",
        f"- pending items: `{payload['summary']['pending_items']}`",
        f"- items with source anchors found: `{payload['summary']['anchors_ok_count']}`",
        f"- production integration complete: `{payload['summary']['production_integration_complete']}`",
        "",
        "| Req | Anchors | Goal |",
        "| --- | ---: | --- |",
    ]
    for row in payload["items"]:
        lines.append(f"| `{row['req_id']}` | {row['anchors_ok']} | {row['production_goal']} |")
    lines.append("")
    lines.append("## Anchor Details")
    for row in payload["items"]:
        lines.append("")
        lines.append(f"### {row['req_id']} {row['title']}")
        for anchor in row["anchors"]:
            status = "ok" if anchor["ok"] else "missing"
            lines.append(f"- `{anchor['path']}` ({anchor['role']}): `{status}`")
            for pattern in anchor["patterns"]:
                lines.append(f"  - `{pattern['pattern']}`: `{pattern['present']}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", default="improve_workspace/production_integration_readiness.json")
    parser.add_argument("--md-out", default="improve_workspace/production_integration_readiness.md")
    parser.add_argument(
        "--strict-anchors",
        action="store_true",
        help="Return failure if any expected production source anchor is missing.",
    )
    args = parser.parse_args()

    items = audit()
    anchors_ok_count = sum(1 for item in items if item["anchors_ok"])
    payload = {
        "summary": {
            "pending_items": len(items),
            "anchors_ok_count": anchors_ok_count,
            "missing_anchor_count": len(items) - anchors_ok_count,
            "production_integration_complete": anchors_ok_count == len(items),
        },
        "items": items,
    }
    json_path = REPO_ROOT / args.json_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_md(md_path, payload)
    print(json.dumps({
        "json_out": str(json_path),
        "md_out": str(md_path),
        **payload["summary"],
    }, indent=2, ensure_ascii=False))
    return 0 if (not args.strict_anchors or anchors_ok_count == len(items)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
