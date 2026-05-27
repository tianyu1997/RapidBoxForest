#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUTPUTS = ROOT / "outputs" / "paper"


DEFAULT_ARTIFACTS = [
    "tro2026_shelf_anytime_tradeoff_full.json",
    "tro2026_random_anytime_tradeoff_full.json",
    "tro2026_exp04_marcucci_full.json",
    "tro2026_exp05_random_sbf_full.json",
    "tro2026_exp04_rrt_connect_full.json",
    "tro2026_exp05_random_rrt_full.json",
    "tro2026_exp07_gcs_full.json",
    "tro2026_safety_accounting_full.json",
    "paper_soundness_audit_suite.json",
    "marcucci_iris_np_gcs.json",
    "marcucci_ompl_prm.json",
    "marcucci_ompl_bitstar_budget.json",
]

AUDIT_PASS_KEYS = (
    "audit_passed",
    "final_strict_audit_passed",
    "collision_free",
    "strict_audit_passed",
)
SOLVER_SUCCESS_KEYS = ("ok", "success", "solver_success")
CORRIDOR_KEYS = (
    "corridor_certified",
    "corridor_certified_success",
    "inside_validated_box_union",
    "inside_conservative_corridor",
    "strict_corridor_success",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize TRO follow-up validation closure from paper artifacts.")
    parser.add_argument("--outputs", type=Path, default=PAPER_OUTPUTS)
    parser.add_argument("--artifacts", nargs="*", default=None, help="Optional artifact filenames. Defaults to known paper artifacts that exist.")
    parser.add_argument("--scan-all", action="store_true", help="Scan every JSON file under --outputs instead of known paper artifacts.")
    parser.add_argument("--out-json", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_validation_closure.json")
    parser.add_argument("--out-csv", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_validation_closure.csv")
    parser.add_argument("--out-md", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_validation_closure.md")
    return parser.parse_args()


def load_json(path: Path) -> Any | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def bool_or_none(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "yes", "pass", "passed", "success", "ok", "1"}:
            return True
        if lowered in {"false", "no", "fail", "failed", "failure", "0"}:
            return False
    return None


def first_bool(row: dict[str, Any], keys: Iterable[str]) -> bool | None:
    for key in keys:
        if key in row:
            value = bool_or_none(row.get(key))
            if value is not None:
                return value
    return None


def number(row: dict[str, Any], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return float(default)


def classify_failure(row: dict[str, Any], audit_passed: bool | None, solver_success: bool | None, corridor_known: bool) -> str:
    if audit_passed is True:
        return "audited_success"
    if solver_success is False:
        return "no_candidate_or_timeout"

    reason_text = " ".join(
        str(row.get(key, ""))
        for key in ("reason", "audit_status", "status", "note", "failure_reason")
    ).lower()
    if "timeout" in reason_text:
        return "no_candidate_or_timeout"
    if "repair" in reason_text or number(row, "repair_count") > 0 or number(row, "repair_time_ms") > 0:
        return "repair_or_smoothing_related"
    if "bridge" in reason_text or number(row, "bridge_progress") > 0 or number(row, "segment_edges_used") > 0:
        return "bridge_or_external_composition"
    if audit_passed is False and corridor_known:
        return "corridor_status_known_audit_failed"
    if audit_passed is False:
        return "audit_failed_unknown_source"
    return "unknown_or_not_reported"


def row_label(path: tuple[str, ...]) -> str:
    trimmed = [part for part in path if not part.startswith("[")]
    return "/".join(trimmed[-4:]) or "artifact"


def looks_like_query_row(row: dict[str, Any]) -> bool:
    signal_keys = set(AUDIT_PASS_KEYS) | set(SOLVER_SUCCESS_KEYS) | set(CORRIDOR_KEYS)
    if signal_keys.intersection(row):
        return True
    if {"sr", "success_count", "trial_count"}.issubset(row):
        return True
    if {"audit_sr", "path_count"}.issubset(row):
        return True
    return False


def expand_aggregate_row(row: dict[str, Any]) -> tuple[int, int, int, int] | None:
    if "path_count" in row and "audit_pass_count" in row:
        path_count = int(round(number(row, "path_count")))
        audit_pass = int(round(number(row, "audit_pass_count")))
        solved_unsafe = int(round(number(row, "solved_unsafe_count")))
        solved = min(path_count, audit_pass + solved_unsafe)
        return path_count, audit_pass, solved_unsafe, solved
    if "trial_count" in row and "success_count" in row:
        path_count = int(round(number(row, "trial_count")))
        solved = int(round(number(row, "success_count")))
        if "audit_sr" in row:
            audit_pass = int(round(float(row.get("audit_sr") or 0.0) * path_count))
        else:
            audit_pass = solved
        solved_unsafe = max(0, solved - audit_pass)
        return path_count, audit_pass, solved_unsafe, solved
    return None


def direct_children_rows(payload: dict[str, Any], key: str) -> list[tuple[tuple[str, ...], dict[str, Any]]]:
    value = payload.get(key)
    if not isinstance(value, list):
        return []
    rows: list[tuple[tuple[str, ...], dict[str, Any]]] = []
    for index, item in enumerate(value):
        if isinstance(item, dict) and looks_like_query_row(item):
            rows.append(((key, f"[{index}]"), item))
    return rows


def collect_rows(node: Any, path: tuple[str, ...] = ()) -> list[tuple[tuple[str, ...], dict[str, Any]]]:
    rows: list[tuple[tuple[str, ...], dict[str, Any]]] = []
    if isinstance(node, dict):
        if looks_like_query_row(node):
            rows.append((path, node))
        for key, value in node.items():
            rows.extend(collect_rows(value, path + (str(key),)))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            rows.extend(collect_rows(value, path + (f"[{index}]",)))
    return rows


def selected_rows(payload: Any) -> tuple[str, list[tuple[tuple[str, ...], dict[str, Any]]]]:
    if not isinstance(payload, dict):
        return "recursive", collect_rows(payload)

    # Prefer top-level final summaries when available. Many baseline artifacts
    # also store retry attempts under seed_trials; counting both would inflate
    # the paper-facing path denominator.
    for key in ("rows", "queries", "gcs_queries"):
        rows = direct_children_rows(payload, key)
        if rows:
            return f"top_level_{key}", rows

    for key in ("trials", "records", "seed_trials", "settings"):
        value = payload.get(key)
        if isinstance(value, list) and value:
            rows = collect_rows(value, (key,))
            if rows:
                return f"nested_{key}", rows
        if isinstance(value, dict) and value:
            rows = collect_rows(value, (key,))
            if rows:
                return f"nested_{key}", rows

    return "recursive", collect_rows(payload)


def artifact_paths(outputs: Path, names: list[str] | None, scan_all: bool) -> list[Path]:
    if scan_all:
        return sorted(outputs.glob("*.json"))
    selected = names if names is not None else DEFAULT_ARTIFACTS
    paths = [outputs / name for name in selected]
    return [path for path in paths if path.exists()]


def summarize_artifact(path: Path) -> dict[str, Any]:
    payload = load_json(path)
    if payload is None:
        return {
            "artifact": path.name,
            "artifact_exists": False,
            "path_count": 0,
            "audit_pass_count": 0,
            "solved_unsafe_count": 0,
            "corridor_known_count": 0,
            "corridor_certified_count": 0,
            "failure_taxonomy": {},
            "unknown_failure_examples": [],
        }

    path_count = 0
    audit_pass_count = 0
    solved_unsafe_count = 0
    corridor_known_count = 0
    corridor_certified_count = 0
    failure_taxonomy: dict[str, int] = {}
    unknown_examples: list[dict[str, Any]] = []

    count_basis, rows_to_count = selected_rows(payload)

    for row_path, row in rows_to_count:
        aggregate = expand_aggregate_row(row)
        if aggregate is not None:
            aggregate_count, aggregate_audit_pass, aggregate_solved_unsafe, aggregate_solved = aggregate
            path_count += aggregate_count
            audit_pass_count += aggregate_audit_pass
            solved_unsafe_count += aggregate_solved_unsafe
            if aggregate_audit_pass:
                failure_taxonomy["audited_success"] = failure_taxonomy.get("audited_success", 0) + aggregate_audit_pass
            if aggregate_solved_unsafe:
                failure_taxonomy["aggregate_solved_unsafe"] = failure_taxonomy.get("aggregate_solved_unsafe", 0) + aggregate_solved_unsafe
            unsolved = max(0, aggregate_count - aggregate_solved)
            if unsolved:
                failure_taxonomy["no_candidate_or_timeout"] = failure_taxonomy.get("no_candidate_or_timeout", 0) + unsolved
            continue

        audit_passed = first_bool(row, AUDIT_PASS_KEYS)
        solver_success = first_bool(row, SOLVER_SUCCESS_KEYS)
        corridor_value = first_bool(row, CORRIDOR_KEYS)
        corridor_known = corridor_value is not None
        counted = audit_passed is not None or solver_success is not None or corridor_known
        if not counted:
            continue

        path_count += 1
        if audit_passed is True:
            audit_pass_count += 1
        if solver_success is True and audit_passed is False:
            solved_unsafe_count += 1
        if corridor_known:
            corridor_known_count += 1
            if corridor_value:
                corridor_certified_count += 1

        category = classify_failure(row, audit_passed, solver_success, corridor_known)
        failure_taxonomy[category] = failure_taxonomy.get(category, 0) + 1
        if category in {"audit_failed_unknown_source", "unknown_or_not_reported"} and len(unknown_examples) < 12:
            unknown_examples.append({
                "locator": row_label(row_path),
                "name": row.get("name") or row.get("query") or row.get("scope"),
                "audit_status": row.get("audit_status"),
                "reason": row.get("reason") or row.get("failure_reason"),
            })

    return {
        "artifact": path.name,
        "artifact_exists": True,
        "path_count": int(path_count),
        "audit_pass_count": int(audit_pass_count),
        "audit_sr": audit_pass_count / path_count if path_count else None,
        "solved_unsafe_count": int(solved_unsafe_count),
        "corridor_known_count": int(corridor_known_count),
        "corridor_certified_count": int(corridor_certified_count),
        "corridor_certified_rate_known_only": corridor_certified_count / corridor_known_count if corridor_known_count else None,
        "corridor_field_missing_count": max(0, path_count - corridor_known_count),
        "failure_taxonomy": dict(sorted(failure_taxonomy.items())),
        "count_basis": count_basis,
        "unknown_failure_examples": unknown_examples,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "artifact",
        "artifact_exists",
        "path_count",
        "audit_pass_count",
        "audit_sr",
        "solved_unsafe_count",
        "corridor_known_count",
        "corridor_certified_count",
        "corridor_certified_rate_known_only",
        "corridor_field_missing_count",
        "count_basis",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def write_markdown(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# TRO 2026 Follow-Up Validation Closure",
        "",
        "| Artifact | Paths | Audit Pass | Audit SR | Corridor Known | Corridor Certified | Missing Corridor Field | Solved Unsafe |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in payload["rows"]:
        audit_sr = row.get("audit_sr")
        lines.append(
            f"| {row['artifact']} | {row['path_count']} | {row['audit_pass_count']} | "
            f"{audit_sr:.3f} | " if isinstance(audit_sr, float) else f"| {row['artifact']} | {row['path_count']} | {row['audit_pass_count']} | -- | "
        )
        prefix = lines.pop()
        lines.append(
            prefix
            + f"{row['corridor_known_count']} | {row['corridor_certified_count']} | "
            + f"{row['corridor_field_missing_count']} | {row['solved_unsafe_count']} |"
        )
    summary = payload["summary"]
    lines.extend([
        "",
        "## Summary",
        "",
        f"- Paths counted: {summary['path_count']}",
        f"- Audit pass count: {summary['audit_pass_count']}",
        f"- Solved unsafe count: {summary['solved_unsafe_count']}",
        f"- Corridor-known paths: {summary['corridor_known_count']}",
        f"- Corridor-certified paths: {summary['corridor_certified_count']}",
        "",
        "## Failure Taxonomy",
        "",
    ])
    for category, count in summary["failure_taxonomy"].items():
        lines.append(f"- {category}: {count}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    rows = [summarize_artifact(path) for path in artifact_paths(args.outputs, args.artifacts, bool(args.scan_all))]
    summary_taxonomy: dict[str, int] = {}
    for row in rows:
        for category, count in row.get("failure_taxonomy", {}).items():
            summary_taxonomy[category] = summary_taxonomy.get(category, 0) + int(count)
    summary = {
        "artifact_count": len(rows),
        "path_count": sum(int(row["path_count"]) for row in rows),
        "audit_pass_count": sum(int(row["audit_pass_count"]) for row in rows),
        "solved_unsafe_count": sum(int(row["solved_unsafe_count"]) for row in rows),
        "corridor_known_count": sum(int(row["corridor_known_count"]) for row in rows),
        "corridor_certified_count": sum(int(row["corridor_certified_count"]) for row in rows),
        "failure_taxonomy": dict(sorted(summary_taxonomy.items())),
    }
    summary["audit_sr"] = summary["audit_pass_count"] / summary["path_count"] if summary["path_count"] else None
    payload = {
        "experiment": "tro2026_followup_01_validation_closure",
        "outputs": str(args.outputs),
        "rows": rows,
        "summary": summary,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(args.out_csv, rows)
    write_markdown(args.out_md, payload)
    print(json.dumps({"out_json": str(args.out_json), "summary": summary}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())