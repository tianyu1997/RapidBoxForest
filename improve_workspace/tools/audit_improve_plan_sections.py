#!/usr/bin/env python3
"""Map every docs/improve.md heading to requirement and completion evidence."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from improve_workspace.tools.audit_improve_requirements import audit_requirements  # noqa: E402
from improve_workspace.tools.audit_production_completion_status import build_payload  # noqa: E402


HEADING_RE = re.compile(r"^(#{1,3})\s+(.*)$")


SECTION_REQUIREMENTS: tuple[tuple[str, tuple[str, ...], str], ...] = (
    ("总体目标", ("R0",), "Terminal-cell set and overall C-LECT objective."),
    ("1.", ("R1.1",), "Early-stop adaptive sweep umbrella."),
    ("1.1", ("R1.1",), "Problem statement for fixed-depth virtual layer enumeration."),
    ("1.2", ("R1.2",), "Separate sweep accept depth from seed FFB skip depth."),
    ("1.3", ("R1.1",), "AdaptiveLeafSweep / AdaptiveClassify algorithm."),
    ("1.4", ("R1.4",), "StartCells from shallow cells."),
    ("1.5", ("R1.5",), "Priority and relevance scoring."),
    ("1.6", ("R1.6",), "Local split-dimension scoring."),
    ("1.7", ("R1.7",), "Selected child expansion."),
    ("1.8", ("R1.8",), "Complexity and manuscript text."),
    ("|\\mathcal L", ("R1.8",), "Formula line in the complexity section."),
    ("2.", ("R2.1", "R2.2", "R2.3", "R2.4"), "Validation, occupied, no-good, and connectivity pruning umbrella."),
    ("2.1", ("R2.1",), "Rich validation report."),
    ("2.2", ("R2.2",), "Sound occupied certificate."),
    ("(\\rho_", ("R2.2",), "Occupied-certificate motion-bound computation."),
    ("论文中如何放", ("R2.2", "R6"), "Occupied-certificate paper placement."),
    ("2.3", ("R2.3",), "Blocker-signature no-good cooldown."),
    ("2.4", ("R2.4",), "Connectivity dominance."),
    ("3.", ("R3.2", "R3.3", "R3.4", "R3.5", "R3.6"), "Sparse Patricia / skip-tree umbrella."),
    ("3.1", ("R3.3",), "Current LECT representation problem motivating sparse storage."),
    ("3.2", ("R3.2",), "Dyadic address and mixed-depth intervals."),
    ("3.3", ("R3.3",), "Patricia compression."),
    ("3.4", ("R3.4",), "jump_child / direct descendant lookup."),
    ("3.5", ("R3.5",), "Split-policy and replay compatibility."),
    ("3.6", ("R3.6",), "VirtualCell and SparseNodeMap staging."),
    ("4.", ("R4.1-R4.5", "R4.3", "R4.6"), "Portal edge compression umbrella."),
    ("4.1", ("R4.1-R4.5",), "Typed graph extension with portal edges."),
    ("4.2", ("R4.1-R4.5",), "Portal definition."),
    ("4.3", ("R4.3",), "Domain-internal portal search."),
    ("4.4", ("R4.1-R4.5",), "Compressed global portal edge representation."),
    ("4.5", ("R4.1-R4.5",), "Portal edge soundness condition."),
    ("4.6", ("R4.6",), "Portal membership policy."),
    ("5.", ("R5",), "Combined C-LECT flow and invariant."),
    ("6.", ("R6",), "Paper changes."),
    ("6.1", ("R6",), "LECT section manuscript changes."),
    ("6.2", ("R6",), "LeafSweepRefine algorithm manuscript changes."),
    ("6.3", ("R6",), "Occupied-pruning lemma manuscript changes."),
    ("6.4", ("R6",), "Graph edge definition manuscript changes."),
    ("7.", ("R7",), "Experiment metrics and ablation suite."),
    ("8.", ("R8",), "Recommended implementation order."),
    ("Step 1", ("R1.1", "R8"), "Implementation order step 1."),
    ("Step 2", ("R2.1", "R2.3", "R8"), "Implementation order step 2."),
    ("Step 3", ("R3.2", "R3.3", "R3.4", "R8"), "Implementation order step 3."),
    ("Step 4", ("R2.2", "R8"), "Implementation order step 4."),
    ("Step 5", ("R4.1-R4.5", "R8"), "Implementation order step 5."),
    ("9.", ("R5", "R6"), "Final paper positioning."),
)


def parse_headings(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        match = HEADING_RE.match(line)
        if not match:
            continue
        marker, title = match.groups()
        rows.append({
            "line": line_no,
            "level": len(marker),
            "title": title,
        })
    return rows


def map_heading(title: str) -> tuple[tuple[str, ...], str]:
    for prefix, req_ids, note in sorted(SECTION_REQUIREMENTS, key=lambda item: len(item[0]), reverse=True):
        if title.startswith(prefix):
            return req_ids, note
    return (), "No section mapping rule matched."


def build_section_payload() -> dict[str, Any]:
    req_rows = {row["req_id"]: row for row in audit_requirements()}
    completion_rows = {
        row["req_id"]: row
        for row in build_payload()["requirements"]
    }
    headings = parse_headings(REPO_ROOT / "docs/improve.md")
    section_rows = []
    for heading in headings:
        req_ids, note = map_heading(heading["title"])
        req_evidence_ok = [
            bool(req_rows.get(req_id, {}).get("evidence_ok", False))
            for req_id in req_ids
        ]
        completion_levels = [
            completion_rows.get(req_id, {}).get("production_level", "unknown")
            for req_id in req_ids
        ]
        production_complete = [
            bool(completion_rows.get(req_id, {}).get("production_complete", False))
            for req_id in req_ids
        ]
        section_rows.append({
            **heading,
            "covered_by": list(req_ids),
            "coverage_note": note,
            "mapped": bool(req_ids),
            "sidecar_evidence_ok": bool(req_ids) and all(req_evidence_ok),
            "production_levels": completion_levels,
            "production_complete": bool(req_ids) and all(production_complete),
        })
    summary = {
        "heading_count": len(section_rows),
        "mapped_heading_count": sum(1 for row in section_rows if row["mapped"]),
        "unmapped_heading_count": sum(1 for row in section_rows if not row["mapped"]),
        "sidecar_evidence_ok_heading_count": sum(1 for row in section_rows if row["sidecar_evidence_ok"]),
        "production_complete_heading_count": sum(1 for row in section_rows if row["production_complete"]),
        "all_headings_mapped": all(row["mapped"] for row in section_rows),
        "all_mapped_headings_have_sidecar_evidence": all(
            row["sidecar_evidence_ok"] for row in section_rows if row["mapped"]
        ),
    }
    return {
        "summary": summary,
        "sections": section_rows,
    }


def write_md(path: Path, payload: dict[str, Any]) -> None:
    summary = payload["summary"]
    lines = [
        "# Improve.md Plan Section Audit",
        "",
        "This audit maps every Markdown heading in `docs/improve.md` to the requirement-level evidence and production-completion status.",
        "",
        f"- heading count: `{summary['heading_count']}`",
        f"- mapped headings: `{summary['mapped_heading_count']}`",
        f"- unmapped headings: `{summary['unmapped_heading_count']}`",
        f"- mapped headings with sidecar evidence: `{summary['sidecar_evidence_ok_heading_count']}`",
        f"- production-complete headings: `{summary['production_complete_heading_count']}`",
        f"- all headings mapped: `{summary['all_headings_mapped']}`",
        f"- all mapped headings have sidecar evidence: `{summary['all_mapped_headings_have_sidecar_evidence']}`",
        "",
        "| Line | Level | Heading | Covered by | Sidecar | Production levels | Prod. complete |",
        "| ---: | ---: | --- | --- | ---: | --- | ---: |",
    ]
    for row in payload["sections"]:
        reqs = ", ".join(f"`{req}`" for req in row["covered_by"]) if row["covered_by"] else ""
        levels = ", ".join(f"`{level}`" for level in row["production_levels"])
        title = row["title"].replace("|", "\\|")
        lines.append(
            f"| {row['line']} | {row['level']} | {title} | {reqs} | "
            f"{row['sidecar_evidence_ok']} | {levels} | {row['production_complete']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", default="improve_workspace/improve_plan_section_audit.json")
    parser.add_argument("--md-out", default="improve_workspace/improve_plan_section_audit.md")
    args = parser.parse_args()

    payload = build_section_payload()
    json_path = REPO_ROOT / args.json_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_md(md_path, payload)
    print(json.dumps({
        "json_out": str(json_path),
        "md_out": str(md_path),
        **payload["summary"],
    }, indent=2, ensure_ascii=False))
    return 0 if payload["summary"]["all_headings_mapped"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
