#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUTPUTS = ROOT / "outputs" / "paper"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize direct vs protected merger/GCS safety boundaries.")
    parser.add_argument("--outputs", type=Path, default=PAPER_OUTPUTS)
    parser.add_argument("--out-json", type=Path, default=PAPER_OUTPUTS / "marcucci_protected_merger_study.json")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def load_first(outputs: Path, names: list[str]) -> dict[str, Any] | None:
    for name in names:
        payload = load_json(outputs / name)
        if payload is not None:
            return payload
    return None


def audit_pass(row: dict[str, Any]) -> bool:
    audit = row.get("audit", {})
    return bool(audit.get("passed")) if isinstance(audit, dict) else False


def attempt_rows(audited: dict[str, Any] | None, attempt_name: str) -> list[dict[str, Any]]:
    if not audited:
        return []
    rows: list[dict[str, Any]] = []
    for query in audited.get("queries", []):
        for attempt in query.get("gcs_attempts", []):
            if attempt.get("attempt") == attempt_name:
                rows.append(attempt)
                break
    return rows


def count_attempt(rows: list[dict[str, Any]]) -> dict[str, Any]:
    solved = sum(1 for row in rows if row.get("ok"))
    passed = sum(1 for row in rows if bool(row.get("strict_audit_passed")))
    unsafe = sum(1 for row in rows if row.get("ok") and not bool(row.get("strict_audit_passed")))
    max_gap = None
    total_expansion = None
    gap_values = []
    expansion_values = []
    for row in rows:
        metadata = row.get("metadata", {})
        if not isinstance(metadata, dict):
            continue
        if metadata.get("max_gap_before_expansion") is not None:
            gap_values.append(float(metadata["max_gap_before_expansion"]))
        if metadata.get("total_l1_bound_expansion") is not None:
            expansion_values.append(float(metadata["total_l1_bound_expansion"]))
    if gap_values:
        max_gap = max(gap_values)
    if expansion_values:
        total_expansion = sum(expansion_values)
    return {
        "query_count": len(rows),
        "solve_count": solved,
        "audit_pass_count": passed,
        "solved_unsafe_count": unsafe,
        "max_gap_before_expansion": max_gap,
        "total_l1_bound_expansion": total_expansion,
    }


def main() -> int:
    args = parse_args()
    direct = load_first(args.outputs, ["tro2026_exp07_merger_gcs_full.json", "marcucci_merger_gcs.json"])
    audited = load_first(args.outputs, ["tro2026_exp07_gcs_full.json", "marcucci_audited_corridor_gcs.json"])
    rows: list[dict[str, Any]] = []

    if direct:
        gcs_rows = direct.get("gcs_queries", [])
        rows.append({
            "mode": "direct_merger_gcs",
            "region_policy": "merged/provisional SBF boxes",
            "edge_policy": "geometric overlaps only",
            "query_count": len(gcs_rows),
            "solve_count": sum(1 for row in gcs_rows if row.get("ok")),
            "audit_pass_count": sum(1 for row in gcs_rows if audit_pass(row)),
            "solved_unsafe_count": sum(1 for row in gcs_rows if row.get("ok") and not audit_pass(row)),
            "safety_condition": "safe only if each region is separately certified convex-free; violated by provisional merged boxes",
        })

    attempt_specs = [
        ("sbf_box_corridor", "raw SBF query boxes", "requires existing overlaps; segment-edges may be infeasible for LinearGCS"),
        ("overlap_expanded_sbf_corridor", "pairwise overlap-expanded query boxes", "feasibility adapter; never counted without strict audit"),
        ("audited_path_tube", "narrow audited path-tube boxes", "protected corridor; final strict audit still required"),
    ]
    for attempt_name, region_policy, safety_condition in attempt_specs:
        rows_for_attempt = attempt_rows(audited, attempt_name)
        if not rows_for_attempt:
            continue
        summary = count_attempt(rows_for_attempt)
        rows.append({
            "mode": attempt_name,
            "region_policy": region_policy,
            "edge_policy": "declared sequential overlaps",
            "safety_condition": safety_condition,
            **summary,
        })

    if audited:
        summary = audited.get("summary", {})
        rows.append({
            "mode": "protected_final_policy",
            "region_policy": "first strict-audited GCS attempt, else audited SBF fallback",
            "edge_policy": "hidden behind strict final audit",
            "query_count": summary.get("query_count"),
            "solve_count": summary.get("gcs_solver_ok_count"),
            "audit_pass_count": summary.get("final_success_count"),
            "solved_unsafe_count": 0,
            "fallback_count": summary.get("fallback_count"),
            "safety_condition": "planner success means final path passed strict collision audit",
        })

    payload = {
        "experiment": "paper_07_merger_protected_study",
        "source_artifacts": {
            "direct": "tro2026_exp07_merger_gcs_full.json or marcucci_merger_gcs.json",
            "audited": "tro2026_exp07_gcs_full.json or marcucci_audited_corridor_gcs.json",
        },
        "interpretation": [
            "Merging or expanding boxes may improve GCS graph feasibility but does not create a free-space certificate.",
            "Direct GCS over provisional or merged boxes is a negative control unless strict audit passes.",
            "Protected merger/GCS uses merged boxes only as planning evidence and counts success only after strict path audit.",
        ],
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "rows": rows}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())