#!/usr/bin/env python3
"""Requirement-level audit for docs/improve.md sidecar implementation."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@dataclass(frozen=True)
class EvidenceCheck:
    kind: str
    value: str


@dataclass(frozen=True)
class Requirement:
    req_id: str
    section: str
    title: str
    expected_state: str
    checks: tuple[EvidenceCheck, ...] = ()
    note: str = ""
    production_pending: bool = False


def file_check(path: str) -> EvidenceCheck:
    return EvidenceCheck("file", path)


def symbol_check(module: str, symbol: str) -> EvidenceCheck:
    return EvidenceCheck("symbol", f"{module}:{symbol}")


def json_key_check(path: str, dotted_key: str) -> EvidenceCheck:
    return EvidenceCheck("json_key", f"{path}:{dotted_key}")


REQUIREMENTS: tuple[Requirement, ...] = (
    Requirement(
        "R0",
        "Overall",
        "Represent terminal cells as FREE/CERT_OCCUPIED/INCONCLUSIVE/DEFERRED/COVERED",
        "sidecar_verified",
        (
            file_check("improve_workspace/clect_sidecar/adaptive_sweep.py"),
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "TerminalCell"),
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "AdaptiveSweepResult"),
            symbol_check("improve_workspace.clect_sidecar.reports", "ValidationStatus"),
        ),
    ),
    Requirement(
        "R1.1",
        "1.1-1.3",
        "AdaptiveLeafSweep / AdaptiveClassify replaces fixed-depth virtual layer enumeration",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "AdaptiveLeafSweep"),
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "AdaptiveSweepConfig"),
            json_key_check("improve_workspace/clect_experiment_suite.json", "summary.full_materialization_reduction_vs_fixed"),
        ),
        production_pending=True,
        note="Production LeafSweepRefine replacement is documented but not applied.",
    ),
    Requirement(
        "R1.2",
        "1.2",
        "Separate sweep accept depth from seed-guided FFB skip depth",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "AdaptiveSweepConfig"),
            file_check("improve_workspace/patch_proposals/integration_notes.md"),
        ),
        production_pending=True,
        note="Sidecar has sweep start/max depth knobs; production seed FFB split remains an integration task.",
    ),
    Requirement(
        "R1.4",
        "1.4",
        "StartCells from shallow depth cells rather than root-global recursion",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.dyadic", "split_schedule_cells"),
            json_key_check("improve_workspace/clect_experiment_suite.json", "rows"),
        ),
    ),
    Requirement(
        "R1.5",
        "1.5",
        "Priority and relevance scoring for frontier/component/anchor/portal/blocker/history",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "AdaptiveSweepConfig"),
            file_check("improve_workspace/clect_sidecar/adaptive_sweep.py"),
        ),
    ),
    Requirement(
        "R1.6",
        "1.6",
        "Local split-dimension scoring with blocker-aware tie-breaks",
        "sidecar_verified",
        (
            file_check("improve_workspace/clect_sidecar/adaptive_sweep.py"),
            symbol_check("improve_workspace.clect_sidecar.reports", "Blocker"),
        ),
    ),
    Requirement(
        "R1.7",
        "1.7",
        "Selected child expansion for restricted/query repair contexts",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.adaptive_sweep", "SweepContext"),
            file_check("improve_workspace/clect_sidecar/adaptive_sweep.py"),
        ),
    ),
    Requirement(
        "R1.8",
        "1.8",
        "Complexity statement and paper integration text",
        "proposal_verified",
        (file_check("improve_workspace/patch_proposals/paper_changes.md"),),
        production_pending=True,
    ),
    Requirement(
        "R2.1",
        "2.1",
        "Rich validation report with blockers, stage, margin, overlap, affected joints",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.reports", "ValidationReport"),
            symbol_check("improve_workspace.clect_sidecar.reports", "Blocker"),
            symbol_check("improve_workspace.clect_sidecar.reports", "FailStage"),
        ),
        production_pending=True,
        note="Production validators still need to emit this report.",
    ),
    Requirement(
        "R2.2",
        "2.2",
        "Optional signed-distance occupied certificate predicate",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.occupied", "MaterialWitness"),
            symbol_check("improve_workspace.clect_sidecar.occupied", "revolute_motion_bound"),
            symbol_check("improve_workspace.clect_sidecar.occupied", "occupied_report_from_witness"),
            file_check("improve_workspace/patch_proposals/occupied_certificate_integration.md"),
        ),
        production_pending=True,
        note="Production has default-disabled AABB signed-distance material-point occupied witnesses with conservative revolute motion bounds.",
    ),
    Requirement(
        "R2.3",
        "2.3",
        "Blocker-signature no-good cooldown",
        "sidecar_verified",
        (
            file_check("improve_workspace/clect_sidecar/adaptive_sweep.py"),
            json_key_check("improve_workspace/clect_ablation_benchmark.json", "summary.no_good_eval_reduction_vs_disabled"),
        ),
    ),
    Requirement(
        "R2.4",
        "2.4",
        "Connectivity dominance for covered, connector, single-component, and isolated cells",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.connectivity", "ConnectivityAction"),
            symbol_check("improve_workspace.clect_sidecar.connectivity", "classify_connectivity_dominance"),
        ),
        production_pending=True,
        note="Production component DSU integration remains pending.",
    ),
    Requirement(
        "R3.2",
        "3.2",
        "Per-dimension dyadic address and mixed-depth interval representation",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.dyadic", "DyadicAddress"),
            symbol_check("improve_workspace.clect_sidecar.dyadic", "Interval"),
        ),
    ),
    Requirement(
        "R3.3",
        "3.3",
        "Sparse Patricia / skip-tree overlay without materializing ancestors",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.sparse_tree", "SparseNodeMap"),
            json_key_check("improve_workspace/clect_ablation_benchmark.json", "summary.sparse_node_reduction"),
        ),
        production_pending=True,
        note="Persistent LECTDB Patricia storage is not applied.",
    ),
    Requirement(
        "R3.4",
        "3.4",
        "jump_child / direct descendant lookup",
        "sidecar_verified",
        (symbol_check("improve_workspace.clect_sidecar.dyadic", "jump_cell_containing"),),
    ),
    Requirement(
        "R3.5",
        "3.5",
        "Interval-key replay compatibility proposal",
        "proposal_verified",
        (
            file_check("improve_workspace/patch_proposals/integration_notes.md"),
            file_check("improve_workspace/patch_proposals/interval_key_replay_integration.md"),
        ),
        production_pending=True,
    ),
    Requirement(
        "R3.6",
        "3.6",
        "Stage-1 VirtualCell overlay and Stage-2 SparseNodeMap path",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.dyadic", "DyadicAddress"),
            symbol_check("improve_workspace.clect_sidecar.sparse_tree", "SparseNodeMap"),
            file_check("improve_workspace/patch_proposals/integration_notes.md"),
        ),
        production_pending=True,
    ),
    Requirement(
        "R4.1-R4.5",
        "4.1-4.5",
        "Compressed conservative portal edge, certificate validation, and lazy expansion",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.portal", "Portal"),
            symbol_check("improve_workspace.clect_sidecar.portal", "PortalCorridor"),
            symbol_check("improve_workspace.clect_sidecar.portal", "PortalGraph"),
            json_key_check("improve_workspace/clect_ablation_benchmark.json", "summary.portal_vertex_compression"),
        ),
        production_pending=True,
        note="Production typed PortalCorridor graph edges are wired with conservative box-chain certificates and lazy expansion; domain-internal portal search remains a sidecar prototype.",
    ),
    Requirement(
        "R4.3",
        "4.3",
        "Domain-internal portal search prototype",
        "sidecar_verified",
        (
            symbol_check("improve_workspace.clect_sidecar.portal_search", "detect_portals"),
            symbol_check("improve_workspace.clect_sidecar.portal_search", "greedy_portal_search"),
        ),
    ),
    Requirement(
        "R4.6",
        "4.6",
        "Portal membership policy documented",
        "proposal_verified",
        (file_check("improve_workspace/patch_proposals/integration_notes.md"),),
        production_pending=True,
    ),
    Requirement(
        "R5",
        "5",
        "Combined C-LECT flow and invariant",
        "sidecar_verified",
        (
            file_check("improve_workspace/tools/synthetic_clect_benchmark.py"),
            file_check("improve_workspace/tools/run_clect_experiment_suite.py"),
            json_key_check("improve_workspace/clect_experiment_suite.json", "summary.full_materialization_reduction_vs_fixed"),
        ),
    ),
    Requirement(
        "R6",
        "6",
        "Paper text changes proposed",
        "proposal_verified",
        (file_check("improve_workspace/patch_proposals/paper_changes.md"),),
        production_pending=True,
    ),
    Requirement(
        "R7",
        "7",
        "Experiment plan metrics and ablation suite",
        "sidecar_verified",
        (
            file_check("improve_workspace/tools/run_clect_experiment_suite.py"),
            file_check("improve_workspace/tools/run_clect_scaling_experiment.py"),
            file_check("improve_workspace/clect_experiment_suite.csv"),
            file_check("improve_workspace/clect_experiment_suite.md"),
            file_check("improve_workspace/clect_scaling_experiment.csv"),
            file_check("improve_workspace/clect_scaling_experiment.md"),
            file_check("improve_workspace/tools/plot_clect_experiments.py"),
            file_check("improve_workspace/tools/run_production_experiment_bridge.py"),
            file_check("improve_workspace/clect_figures_manifest.json"),
            file_check("improve_workspace/clect_figures.md"),
            file_check("improve_workspace/production_experiment_bridge.json"),
            file_check("improve_workspace/production_experiment_bridge.md"),
            file_check("improve_workspace/production_experiment_bridge_executed.json"),
            file_check("improve_workspace/production_experiment_bridge_executed.md"),
            file_check("improve_workspace/production_bridge/catalogs/exp06_iiwa_easy_smoke_catalog.json"),
            file_check("improve_workspace/tools/summarize_improve_performance.py"),
            file_check("improve_workspace/performance_change_summary.json"),
            file_check("improve_workspace/performance_change_summary.md"),
            file_check("improve_workspace/tools/audit_production_integration_readiness.py"),
            file_check("improve_workspace/production_integration_readiness.json"),
            file_check("improve_workspace/production_integration_readiness.md"),
            file_check("improve_workspace/tools/audit_production_completion_status.py"),
            file_check("improve_workspace/production_completion_status.json"),
            file_check("improve_workspace/production_completion_status.md"),
            file_check("improve_workspace/tools/audit_improve_plan_sections.py"),
            file_check("improve_workspace/improve_plan_section_audit.json"),
            file_check("improve_workspace/improve_plan_section_audit.md"),
            file_check("improve_workspace/figures/clect_scaling_materialized_cells.png"),
            file_check("improve_workspace/figures/clect_scaling_reduction_factors.png"),
            file_check("improve_workspace/figures/clect_scaling_graph_vertices.png"),
            file_check("improve_workspace/figures/clect_accepted_free_depth_histogram.png"),
            file_check("improve_workspace/figures/clect_materialized_cell_depth_histogram.png"),
            json_key_check("improve_workspace/clect_experiment_suite.json", "rows"),
            json_key_check("improve_workspace/clect_scaling_experiment.json", "depth_summaries"),
            json_key_check("improve_workspace/clect_figures_manifest.json", "count"),
            json_key_check("improve_workspace/production_experiment_bridge.json", "manifests"),
            json_key_check("improve_workspace/production_experiment_bridge_executed.json", "manifests"),
            json_key_check("improve_workspace/performance_change_summary.json", "production_executed_smoke"),
            json_key_check("improve_workspace/production_completion_status.json", "summary.production_completion_complete"),
            json_key_check("improve_workspace/improve_plan_section_audit.json", "summary.all_headings_mapped"),
        ),
        production_pending=True,
        note=(
            "Production Exp.4/6 runner dry-run manifests are bridged; "
            "production C-LECT shelf/random performance remains pending."
        ),
    ),
    Requirement(
        "R8",
        "8",
        "Recommended implementation order represented",
        "sidecar_verified",
        (file_check("improve_workspace/implementation_matrix.md"),),
    ),
)


def plan_metadata(path: Path) -> dict[str, Any]:
    data = path.read_bytes()
    text = data.decode("utf-8", errors="replace")
    return {
        "path": str(path),
        "bytes": len(data),
        "lines": len(text.splitlines()),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def get_json_key(path: Path, dotted_key: str) -> Any:
    data = json.loads(path.read_text(encoding="utf-8"))
    value: Any = data
    for part in dotted_key.split("."):
        if isinstance(value, list):
            value = value[int(part)]
        else:
            value = value[part]
    return value


def evaluate_check(check: EvidenceCheck) -> dict[str, Any]:
    if check.kind == "file":
        path = REPO_ROOT / check.value
        return {
            "kind": check.kind,
            "value": check.value,
            "ok": path.exists() and path.stat().st_size > 0,
            "detail": "exists" if path.exists() else "missing",
        }
    if check.kind == "symbol":
        module_name, symbol = check.value.split(":", 1)
        try:
            module = importlib.import_module(module_name)
            ok = hasattr(module, symbol)
            return {
                "kind": check.kind,
                "value": check.value,
                "ok": ok,
                "detail": "present" if ok else "missing symbol",
            }
        except Exception as exc:  # pragma: no cover - diagnostic path
            return {
                "kind": check.kind,
                "value": check.value,
                "ok": False,
                "detail": f"import failed: {exc}",
            }
    if check.kind == "json_key":
        path_text, dotted_key = check.value.split(":", 1)
        path = REPO_ROOT / path_text
        try:
            value = get_json_key(path, dotted_key)
            ok = value is not None
            return {
                "kind": check.kind,
                "value": check.value,
                "ok": ok,
                "detail": "present",
            }
        except Exception as exc:
            return {
                "kind": check.kind,
                "value": check.value,
                "ok": False,
                "detail": f"json lookup failed: {exc}",
            }
    return {
        "kind": check.kind,
        "value": check.value,
        "ok": False,
        "detail": "unknown check kind",
    }


def audit_requirements() -> list[dict[str, Any]]:
    rows = []
    for requirement in REQUIREMENTS:
        checks = [evaluate_check(check) for check in requirement.checks]
        evidence_ok = all(check["ok"] for check in checks)
        state = requirement.expected_state if evidence_ok else "missing_evidence"
        rows.append({
            "req_id": requirement.req_id,
            "section": requirement.section,
            "title": requirement.title,
            "expected_state": requirement.expected_state,
            "state": state,
            "evidence_ok": evidence_ok,
            "production_pending": requirement.production_pending,
            "note": requirement.note,
            "checks": checks,
        })
    return rows


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Improve.md Requirement Audit",
        "",
        f"- Plan SHA-256: `{payload['plan']['sha256']}`",
        f"- Workspace evidence ok: `{payload['summary']['workspace_evidence_ok']}`",
        f"- Production integration complete: `{payload['summary']['production_integration_complete']}`",
        "",
        "| Req | Section | State | Production relevant | Evidence | Title |",
        "| --- | --- | --- | ---: | ---: | --- |",
    ]
    for row in payload["requirements"]:
        lines.append(
            f"| `{row['req_id']}` | `{row['section']}` | `{row['state']}` | "
            f"{row['production_pending']} | {row['evidence_ok']} | {row['title']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", default="improve_workspace/improve_requirements_audit.json")
    parser.add_argument("--md-out", default="improve_workspace/improve_requirements_audit.md")
    args = parser.parse_args()

    requirements = audit_requirements()
    missing = [row for row in requirements if not row["evidence_ok"]]
    production_relevant = [row for row in requirements if row["production_pending"]]
    completion_status_path = REPO_ROOT / "improve_workspace/production_completion_status.json"
    production_completion_complete = False
    if completion_status_path.exists():
        try:
            completion_payload = json.loads(completion_status_path.read_text(encoding="utf-8"))
            production_completion_complete = bool(
                completion_payload.get("production_completion_complete", False)
                or completion_payload.get("summary", {}).get("production_completion_complete", False)
            )
        except Exception:
            production_completion_complete = False
    summary = {
        "total_requirements": len(requirements),
        "evidence_ok_count": len(requirements) - len(missing),
        "missing_evidence_count": len(missing),
        "production_pending_count": len(production_relevant),
        "production_relevant_count": len(production_relevant),
        "production_remaining_count": 0 if production_completion_complete else len(production_relevant),
        "workspace_evidence_ok": len(missing) == 0,
        "production_integration_complete": production_completion_complete,
    }
    payload = {
        "plan": plan_metadata(REPO_ROOT / "docs/improve.md"),
        "summary": summary,
        "requirements": requirements,
    }
    json_path = REPO_ROOT / args.json_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_md(md_path, payload)
    print(json.dumps({
        "json_out": str(json_path),
        "md_out": str(md_path),
        **summary,
    }, indent=2, ensure_ascii=False))
    return 0 if summary["workspace_evidence_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
