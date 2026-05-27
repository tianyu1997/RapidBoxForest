#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUTPUTS = ROOT / "outputs" / "paper"
DEFAULT_TOLERANCES = [0.0, 0.05, 0.08, 0.10, 0.15]
DEFAULT_TOLERANCE = 0.08

sys.path.insert(0, str(ROOT / "experiments"))
import RapidBoxForest.safe_box_forest.experiments.sbf_old.tro2026_generate_tables as tablegen  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize tabulated-row sensitivity and path-gap robustness from current anytime artifacts.")
    parser.add_argument("--outputs", type=Path, default=PAPER_OUTPUTS)
    parser.add_argument("--out-json", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_reporting_sensitivity.json")
    parser.add_argument("--out-md", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_reporting_sensitivity.md")
    parser.add_argument("--tolerances", nargs="*", type=float, default=DEFAULT_TOLERANCES)
    return parser.parse_args()


def finite_float(value: Any, default: float = math.inf) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result):
        return default
    return result


def full_success(point: dict[str, Any]) -> bool:
    return bool(tablegen.point_has_full_success(point))


def pareto_points(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    usable = [
        point
        for point in points
        if point.get("total_s") is not None
        and point.get("path_length") is not None
        and full_success(point)
    ]
    frontier: list[dict[str, Any]] = []
    for point in usable:
        time_s = finite_float(point.get("total_s"))
        path = finite_float(point.get("path_length"))
        dominated = False
        for other in usable:
            other_time = finite_float(other.get("total_s"))
            other_path = finite_float(other.get("path_length"))
            if (
                other_time <= time_s + 1e-12
                and other_path <= path + float(tablegen.PATH_DOMINATION_EPS)
                and (other_time < time_s - 1e-12 or other_path < path - float(tablegen.PATH_DOMINATION_EPS))
            ):
                dominated = True
                break
        if not dominated:
            frontier.append(point)
    return sorted(frontier, key=lambda point: finite_float(point.get("total_s")))


def choose_row(points: list[dict[str, Any]], tolerance: float) -> dict[str, Any] | None:
    frontier = pareto_points(points)
    if not frontier:
        success_candidates = [point for point in points if point.get("total_s") is not None]
        if not success_candidates:
            return None
        best = min(
            success_candidates,
            key=lambda point: (
                -finite_float(point.get("audit_sr"), 0.0),
                finite_float(point.get("total_s")),
                finite_float(point.get("path_length")),
            ),
        )
        row = dict(best)
        row["rule_status"] = "fallback_success_or_fastest"
        return row
    best_path = min(finite_float(point.get("path_length")) for point in frontier)
    path_limit = best_path * (1.0 + float(tolerance)) if best_path > 0.0 else best_path + float(tolerance)
    candidates = [point for point in frontier if finite_float(point.get("path_length")) <= path_limit]
    if not candidates:
        candidates = frontier

    def score(point: dict[str, Any]) -> float:
        time_s = max(0.0, finite_float(point.get("total_s"), 0.0))
        path = finite_float(point.get("path_length"))
        return path + float(tablegen.PATH_LOG_TIME_PENALTY_RAD) * math.log(float(tablegen.PATH_LOG_TIME_OFFSET_S) + time_s)

    best = min(
        candidates,
        key=lambda point: (
            score(point),
            finite_float(point.get("path_length")),
            finite_float(point.get("total_s")),
        ),
    )
    row = dict(best)
    row["rule_status"] = "pareto_path_log_time"
    row["tolerance"] = float(tolerance)
    row["path_limit"] = float(path_limit)
    row["utility"] = float(score(row))
    return row


def row_key(row: dict[str, Any] | None) -> tuple[Any, ...] | None:
    if row is None:
        return None
    return (
        row.get("scenario_key"),
        row.get("normalized_method"),
        row.get("stage_id"),
        row.get("stage_index"),
    )


def group_points(rows: list[dict[str, Any]]) -> dict[tuple[str, str], list[dict[str, Any]]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        method = str(row.get("normalized_method"))
        if method not in tablegen.METHOD_ORDER:
            continue
        scenario_key = str(row.get("scenario_key"))
        grouped.setdefault((scenario_key, method), []).append(row)
    return grouped


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(q)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_path_gaps(default_rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_scenario: dict[str, dict[str, dict[str, Any]]] = {}
    for row in default_rows:
        by_scenario.setdefault(str(row.get("scenario_key")), {})[str(row.get("normalized_method"))] = row
    gaps: list[float] = []
    rel_gaps: list[float] = []
    outliers: list[dict[str, Any]] = []
    for scenario_key, methods in sorted(by_scenario.items()):
        sbf_row = methods.get("sbf")
        if sbf_row is None:
            continue
        comparable = [row for row in methods.values() if row.get("path_length") is not None]
        if not comparable:
            continue
        best_row = min(comparable, key=lambda row: finite_float(row.get("path_length")))
        sbf_path = finite_float(sbf_row.get("path_length"))
        best_path = finite_float(best_row.get("path_length"))
        gap = sbf_path - best_path
        rel_gap = gap / best_path if best_path > 1e-12 else 0.0
        gaps.append(gap)
        rel_gaps.append(rel_gap)
        outliers.append({
            "scenario_key": scenario_key,
            "sbf_path": sbf_path,
            "best_method": str(best_row.get("normalized_method")),
            "best_path": best_path,
            "gap_rad": gap,
            "relative_gap": rel_gap,
            "sbf_total_s": finite_float(sbf_row.get("total_s")),
            "best_total_s": finite_float(best_row.get("total_s")),
        })
    outliers.sort(key=lambda row: float(row["gap_rad"]), reverse=True)
    return {
        "scenario_count": len(gaps),
        "gap_rad_median": statistics.median(gaps) if gaps else None,
        "gap_rad_p90": percentile(gaps, 0.90),
        "gap_rad_max": max(gaps) if gaps else None,
        "relative_gap_median": statistics.median(rel_gaps) if rel_gaps else None,
        "relative_gap_p90": percentile(rel_gaps, 0.90),
        "relative_gap_max": max(rel_gaps) if rel_gaps else None,
        "sbf_best_or_tied_count": sum(1 for gap in gaps if gap <= 1e-9),
        "largest_gap_examples": outliers[:10],
    }


def compact_row(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "scenario_key": row.get("scenario_key"),
        "method": row.get("normalized_method"),
        "stage_id": row.get("stage_id"),
        "stage_index": row.get("stage_index"),
        "total_s": finite_float(row.get("total_s")),
        "build_s": finite_float(row.get("build_s"), 0.0),
        "query_s": finite_float(row.get("query_s"), 0.0),
        "path_length": finite_float(row.get("path_length")),
        "audit_sr": finite_float(row.get("audit_sr"), 0.0),
        "rule_status": row.get("rule_status"),
    }


def write_markdown(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Mandatory Reporting Sensitivity",
        "",
        "This report postprocesses the current anytime artifacts. It does not rerun planners.",
        "",
        "## Summary",
        "",
        f"- Stage points scanned: {payload['stage_point_count']}",
        f"- Scenario/method groups: {payload['group_count']}",
        f"- Default tolerance: {payload['default_tolerance']:.2f}",
        "",
        "## Row Movement Against Default Tolerance",
        "",
        "| Tolerance | Comparable groups | Changed rows | Change rate | Missing rows |",
        "|---:|---:|---:|---:|---:|",
    ]
    for row in payload["tolerance_summaries"]:
        lines.append(
            f"| {row['tolerance']:.2f} | {row['comparable_count']} | {row['changed_count']} | {row['changed_rate']:.3f} | {row['missing_count']} |"
        )
    path_gap = payload["path_gap_summary"]
    lines.extend([
        "",
        "## SBF/RBF Path Gap Against Best Tabulated Method",
        "",
        f"- Scenario count: {path_gap['scenario_count']}",
        f"- Median gap: {path_gap['gap_rad_median']:.4f} rad" if path_gap["gap_rad_median"] is not None else "- Median gap: n/a",
        f"- P90 gap: {path_gap['gap_rad_p90']:.4f} rad" if path_gap["gap_rad_p90"] is not None else "- P90 gap: n/a",
        f"- Max gap: {path_gap['gap_rad_max']:.4f} rad" if path_gap["gap_rad_max"] is not None else "- Max gap: n/a",
        f"- Best-or-tied count: {path_gap['sbf_best_or_tied_count']}",
        "",
        "### Largest Gap Examples",
        "",
        "| Scenario | Best method | Best path | SBF/RBF path | Gap | Relative gap |",
        "|---|---|---:|---:|---:|---:|",
    ])
    for row in path_gap["largest_gap_examples"]:
        lines.append(
            f"| {row['scenario_key']} | {row['best_method']} | {row['best_path']:.3f} | {row['sbf_path']:.3f} | {row['gap_rad']:.3f} | {100.0 * row['relative_gap']:.1f}% |"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    rows = tablegen.collect_anytime_stage_points(args.outputs)
    grouped = group_points(rows)
    tolerance_rows: dict[float, dict[tuple[str, str], dict[str, Any]]] = {}
    for tolerance in args.tolerances:
        chosen: dict[tuple[str, str], dict[str, Any]] = {}
        for key, points in grouped.items():
            row = choose_row(points, float(tolerance))
            if row is None:
                continue
            row["scenario_key"] = key[0]
            row["normalized_method"] = key[1]
            chosen[key] = row
        tolerance_rows[float(tolerance)] = chosen

    default_rows_by_key = tolerance_rows.get(float(DEFAULT_TOLERANCE)) or tolerance_rows.get(float(args.tolerances[0]), {})
    default_keys = {key: row_key(row) for key, row in default_rows_by_key.items()}
    summaries = []
    for tolerance, chosen in sorted(tolerance_rows.items()):
        changed = 0
        comparable = 0
        missing = 0
        examples = []
        for key, default_key in default_keys.items():
            current = chosen.get(key)
            if current is None:
                missing += 1
                continue
            comparable += 1
            current_key = row_key(current)
            if current_key != default_key:
                changed += 1
                if len(examples) < 20:
                    examples.append({
                        "scenario_key": key[0],
                        "method": key[1],
                        "default": default_key,
                        "current": current_key,
                    })
        summaries.append({
            "tolerance": float(tolerance),
            "comparable_count": comparable,
            "changed_count": changed,
            "changed_rate": float(changed) / float(comparable) if comparable else 0.0,
            "missing_count": missing,
            "change_examples": examples,
        })

    default_rows = [dict(row) for row in default_rows_by_key.values()]
    payload = {
        "experiment": "tro2026_followup_03_reporting_sensitivity",
        "outputs": str(args.outputs),
        "stage_point_count": len(rows),
        "group_count": len(grouped),
        "default_tolerance": float(DEFAULT_TOLERANCE),
        "tolerances": [float(value) for value in args.tolerances],
        "tolerance_summaries": summaries,
        "path_gap_summary": summarize_path_gaps(default_rows),
        "default_rows": [compact_row(row) for row in sorted(default_rows, key=lambda item: (str(item.get("scenario_key")), str(item.get("normalized_method"))))],
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_markdown(args.out_md, payload)
    print(json.dumps({
        "out_json": str(args.out_json),
        "out_md": str(args.out_md),
        "stage_point_count": payload["stage_point_count"],
        "group_count": payload["group_count"],
        "path_gap_summary": payload["path_gap_summary"],
        "tolerance_summaries": payload["tolerance_summaries"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())