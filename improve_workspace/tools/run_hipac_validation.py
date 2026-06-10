#!/usr/bin/env python3
"""Validate the HiPaC sidecar implementation for docs/分级partition连通.md."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from improve_workspace.clect_sidecar import (  # noqa: E402
    Blocker,
    DyadicAddress,
    FailStage,
    HiPaCConfig,
    HiPaCOfflineMode,
    HierarchicalPartitionConnectivity,
    ValidationReport,
)
from improve_workspace.clect_sidecar.synthetic import unit_root  # noqa: E402


def corridor_validator(address: DyadicAddress, box):
    lo = box[0].lo
    hi = box[0].hi
    if hi <= 0.25 + 1e-12:
        return ValidationReport.free("left")
    if lo >= 0.5 - 1e-12:
        return ValidationReport.free("right")
    if lo >= 0.25 - 1e-12 and hi <= 0.5 + 1e-12 and address.depth >= 3:
        return ValidationReport.free("hidden_mid")
    blocker = Blocker(0, 0, FailStage.GJK, margin=-0.1, overlap_score=0.5, affected_joints=(0,))
    return ValidationReport.fail([blocker], overlap_score=0.5)


def run_tests(repo: Path) -> dict[str, object]:
    proc = subprocess.run(
        [sys.executable, "-m", "unittest", "improve_workspace.tests.test_hipac_sidecar", "-v"],
        cwd=repo,
        text=True,
        capture_output=True,
        check=False,
    )
    return {
        "cmd": proc.args,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


def run_experiment_suite(repo: Path) -> dict[str, object]:
    proc = subprocess.run(
        [sys.executable, "improve_workspace/tools/run_hipac_experiment_suite.py"],
        cwd=repo,
        text=True,
        capture_output=True,
        check=False,
    )
    return {
        "cmd": proc.args,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


def demo_payload() -> dict[str, object]:
    planner = HierarchicalPartitionConnectivity(
        unit_root(1),
        corridor_validator,
        HiPaCConfig(
            coarse_depth=2,
            max_depth=4,
            max_refinement_iterations=4,
            local_refine_budget=8,
            offline_mode=HiPaCOfflineMode.QUERY_AGNOSTIC_SCENE_SKELETON,
        ),
        workload_anchors=((0.1,), (0.8,)),
        workload_anchor_pairs=((0, 1),),
    )
    build = planner.build()
    query = planner.query((0.1,), (0.8,), online_budget=2)
    metrics = planner.metrics(
        probe_points=((0.1,), (0.3,), (0.8,)),
        query_pairs=(((0.1,), (0.8,)),),
    )
    return {
        "build": {
            "certified_graph_nodes": build.certified_graph_nodes,
            "optimistic_graph_nodes": build.optimistic_graph_nodes,
            "certified_components": build.certified_components,
            "resolved_portal_pairs": build.resolved_portal_pairs,
            "unresolved_portal_pairs": build.unresolved_portal_pairs,
            "refined_mixed_cells": build.refined_mixed_cells,
            "deferred_mixed_cells": build.deferred_mixed_cells,
            "validation_counts": dict(build.validation_counts),
            "cell_state_counts": dict(build.cell_state_counts),
        },
        "query": {
            "success": query.success,
            "status": query.status,
            "certified_path": query.certified_path,
            "expanded_path": query.expanded_path,
            "refined_mixed_cells": query.refined_mixed_cells,
            "unresolved_cells_seen": query.unresolved_cells_seen,
            "used_portal_edges": query.used_portal_edges,
            "online_repair_s": query.online_repair_s,
        },
        "metrics": {
            "certified_components": metrics.certified_components,
            "resolved_portal_pairs": metrics.resolved_portal_pairs,
            "unresolved_portal_pairs": metrics.unresolved_portal_pairs,
            "connected_anchor_pairs": metrics.connected_anchor_pairs,
            "anchor_pairs": metrics.anchor_pairs,
            "p_attach": metrics.p_attach,
            "p_samecomp": metrics.p_samecomp,
            "online_mixed_cells_refined": metrics.online_mixed_cells_refined,
            "online_repair_s": metrics.online_repair_s,
        },
        "corridors": sorted(planner.portal_corridors),
        "summaries_with_portals": {
            key: {
                "state": summary.state.value,
                "certified_portal_pairs": {
                    f"{source}->{target}": corridor
                    for (source, target), corridor in summary.certified_portal_pairs.items()
                },
                "unresolved_portal_pairs": [
                    f"{source}->{target}"
                    for source, target in sorted(summary.unresolved_portal_pairs)
                ],
                "children": list(summary.children),
            }
            for key, summary in planner.summaries.items()
            if summary.certified_portal_pairs or summary.unresolved_portal_pairs
        },
    }


def requirement_matrix() -> list[dict[str, object]]:
    return [
        {
            "section": "1",
            "title": "coarse partition -> abstract connectivity -> refine gaps",
            "implemented_by": ["HierarchicalPartitionConnectivity.build"],
            "evidence": "build.resolved_portal_pairs > 0 and refined_mixed_cells > 0",
        },
        {
            "section": "2",
            "title": "cell states FREE/CERT_OCCUPIED/MIXED/DEFERRED/REFINED",
            "implemented_by": ["HiPaCCellState", "HiPaCCellSummary"],
            "evidence": "unit tests cover MIXED, CERT_OCCUPIED, REFINED portal summaries",
        },
        {
            "section": "3",
            "title": "lazy multi-resolution dyadic partition",
            "implemented_by": ["DyadicAddress", "split_schedule_cells", "refine_mixed_cell"],
            "evidence": "coarse cells are classified first; only selected mixed domains create children",
        },
        {
            "section": "4",
            "title": "certified graph and optimistic graph",
            "implemented_by": ["cert_adj", "opt_adj"],
            "evidence": "test_mixed_cells_only_enter_optimistic_graph",
        },
        {
            "section": "5",
            "title": "connectivity-driven coarse-to-fine refinement",
            "implemented_by": ["_select_disconnected_component_pair", "_shortest_path", "_select_mixed_cell_on_path"],
            "evidence": "build resolves the hidden mixed corridor without full-depth sweep",
        },
        {
            "section": "6",
            "title": "mixed-cell internal certified child-chain search",
            "implemented_by": ["refine_mixed_cell", "_local_refine_for_portals", "_local_free_chain"],
            "evidence": "portal corridor carries certified free internal cells",
        },
        {
            "section": "7",
            "title": "parent cell connectivity summary",
            "implemented_by": ["HiPaCCellSummary"],
            "evidence": "summaries_with_portals records certified portal pairs and children",
        },
        {
            "section": "8",
            "title": "offline budget improves component/portal connectivity",
            "implemented_by": ["HiPaCBuildResult", "HiPaCMetrics"],
            "evidence": "metrics report certified components and resolved portal pairs",
        },
        {
            "section": "9",
            "title": "lazy hierarchical online query",
            "implemented_by": ["query", "attach_or_refine"],
            "evidence": "test_query_refines_first_unresolved_mixed_cell_and_expands_portal",
        },
        {
            "section": "10",
            "title": "mixed-cell bottleneck scoring",
            "implemented_by": ["_mixed_score"],
            "evidence": "score combines component, portal, query, blocker, volume, and depth terms",
        },
        {
            "section": "11",
            "title": "anisotropic split dimensions",
            "implemented_by": ["_choose_split_dimension"],
            "evidence": "test_blocker_affected_joint_drives_anisotropic_split",
        },
        {
            "section": "12",
            "title": "three offline modes",
            "implemented_by": ["HiPaCOfflineMode"],
            "evidence": "query-agnostic, workload-aware, and lifelong modes are represented",
        },
        {
            "section": "13",
            "title": "complete algorithm API",
            "implemented_by": ["build"],
            "evidence": "build returns G+, G~, H-equivalent summaries and diagnostics",
        },
        {
            "section": "14",
            "title": "RefineForConnectivity API",
            "implemented_by": ["refine_mixed_cell"],
            "evidence": "entry/exit portals seed local refinement and certified chain extraction",
        },
        {
            "section": "15",
            "title": "portal edge compression",
            "implemented_by": ["PortalCorridor", "portal_corridors", "_expand_certified_edge_path"],
            "evidence": "expanded query path includes hidden corridor cell ids",
        },
        {
            "section": "16",
            "title": "conservative invariant",
            "implemented_by": ["PortalCorridor.validate_certificate", "cert_adj edge certified flag"],
            "evidence": "portal insertion rejects invalid certificates",
        },
        {
            "section": "17",
            "title": "online query uses coarse explanation",
            "implemented_by": ["query", "opt_adj", "_portal_boundary_candidates"],
            "evidence": "query refines unresolved optimistic path cells instead of global search",
        },
        {
            "section": "18",
            "title": "connectivity-oriented experiment metrics",
            "implemented_by": ["metrics", "HiPaCMetrics"],
            "evidence": "run_hipac_experiment_suite compares fixed, early-stop, HiPaC, and HiPaC+portal; metrics emit N components, portal pairs, P_attach, P_samecomp, online repair",
        },
        {
            "section": "19",
            "title": "minimal landing version",
            "implemented_by": ["HierarchicalPartitionConnectivity"],
            "evidence": "retains validation oracle, classifies coarse partition, uses G+/G~, refines mixed cells, adds portal edges",
        },
        {
            "section": "20",
            "title": "final HiPaC summary",
            "implemented_by": ["README and this validation artifact"],
            "evidence": "sidecar implements coarse structure first, gap refinement second, certified portal compression third",
        },
    ]


def write_markdown(path: Path, payload: dict[str, object]) -> None:
    lines = [
        "# HiPaC Sidecar Validation",
        "",
        "Source plan: `docs/分级partition连通.md`.",
        "",
        f"Overall status: `{payload['ok']}`",
        "",
        "## Demo Metrics",
        "",
    ]
    demo = payload["demo"]  # type: ignore[index]
    assert isinstance(demo, dict)
    for section in ("build", "query", "metrics"):
        lines.append(f"### {section}")
        lines.append("")
        values = demo[section]  # type: ignore[index]
        assert isinstance(values, dict)
        for key, value in values.items():
            lines.append(f"- `{key}`: `{value}`")
        lines.append("")
    lines.extend(["## Requirement Matrix", "", "| Section | Requirement | Evidence |", "|---|---|---|"])
    for row in payload["requirements"]:  # type: ignore[index]
        assert isinstance(row, dict)
        lines.append(f"| {row['section']} | {row['title']} | {row['evidence']} |")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", default="improve_workspace/hipac_validation.json")
    parser.add_argument("--md-out", default="improve_workspace/hipac_validation.md")
    args = parser.parse_args()

    tests = run_tests(REPO_ROOT)
    experiment_suite = run_experiment_suite(REPO_ROOT)
    demo = demo_payload()
    reqs = requirement_matrix()
    ok = (
        tests["returncode"] == 0
        and experiment_suite["returncode"] == 0
        and demo["build"]["resolved_portal_pairs"] >= 1  # type: ignore[index]
        and demo["query"]["success"]  # type: ignore[index]
        and len(reqs) == 20
    )
    payload: dict[str, object] = {
        "ok": ok,
        "source_plan": "docs/分级partition连通.md",
        "tests": tests,
        "experiment_suite": experiment_suite,
        "demo": demo,
        "requirements": reqs,
    }
    json_path = REPO_ROOT / args.json_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_markdown(md_path, payload)
    print(json.dumps({"ok": ok, "json_out": str(json_path), "md_out": str(md_path)}, indent=2, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
