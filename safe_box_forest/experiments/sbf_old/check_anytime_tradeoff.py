#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check anytime-incumbent trade-off artifacts for timing/path monotonicity.")
    parser.add_argument("--shelf", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_shelf_anytime_tradeoff_full.json")
    parser.add_argument("--random", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_random_anytime_tradeoff_full.json")
    parser.add_argument("--only-existing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--epsilon-path", type=float, default=1e-6)
    parser.add_argument("--epsilon-time", type=float, default=1e-7)
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def as_float(value: Any, default: float | None = None) -> float | None:
    if value is None:
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result) or math.isinf(result):
        return default
    return result


def add_issue(items: list[dict[str, Any]], severity: str, scope: str, message: str, **extra: Any) -> None:
    items.append({"severity": severity, "scope": scope, "message": message, **extra})


def group_points(points: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for point in points:
        grouped.setdefault(str(point.get("method")), []).append(point)
    for method in grouped:
        grouped[method].sort(key=lambda row: (int(row.get("stage_index", 0)), str(row.get("stage_id", ""))))
    return grouped


def check_summary(name: str, summary: dict[str, Any], *, epsilon_path: float, epsilon_time: float) -> dict[str, Any]:
    errors: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    points = [dict(point) for point in summary.get("points", [])]
    promoted = [dict(point) for point in summary.get("promoted_points", [])]
    if not points:
        add_issue(errors, "error", name, "summary has no points")
    for point in points:
        method = str(point.get("method"))
        stage = str(point.get("stage_id"))
        build_s = as_float(point.get("build_s"), 0.0) or 0.0
        query_s = as_float(point.get("query_s"), 0.0) or 0.0
        total_s = as_float(point.get("total_s"), 0.0) or 0.0
        audit_sr = as_float(point.get("audit_sr"), -1.0) or -1.0
        if build_s < -epsilon_time or query_s < -epsilon_time or total_s < -epsilon_time:
            add_issue(errors, "error", name, "negative timing component", method=method, stage=stage, build_s=build_s, query_s=query_s, total_s=total_s)
        if abs((build_s + query_s) - total_s) > max(1e-5, 1e-4 * max(1.0, total_s)):
            add_issue(errors, "error", name, "build/query/total are inconsistent", method=method, stage=stage, build_s=build_s, query_s=query_s, total_s=total_s)
        if audit_sr < -1e-9 or audit_sr > 1.0 + 1e-9:
            add_issue(errors, "error", name, "audit success rate is outside [0,1]", method=method, stage=stage, audit_sr=audit_sr)
        if point.get("no_improvement_reason"):
            add_issue(warnings, "warning", name, "tier was not promoted", method=method, stage=stage, reason=point.get("no_improvement_reason"))

    for method, rows in group_points(points).items():
        previous_time: float | None = None
        previous_path: float | None = None
        for row in rows:
            current_time = as_float(row.get("total_s"))
            current_path = as_float(row.get("path_length"))
            if previous_time is not None and current_time is not None and current_time <= previous_time + epsilon_time:
                add_issue(errors, "error", name, "cumulative charged time is not strictly increasing", method=method, previous_time=previous_time, current_time=current_time)
            if previous_path is not None and current_path is not None and current_path > previous_path + epsilon_path:
                add_issue(errors, "error", name, "incumbent path length increased", method=method, previous_path=previous_path, current_path=current_path)
            if current_time is not None:
                previous_time = current_time
            if current_path is not None:
                previous_path = current_path

    promoted_by_method = group_points(promoted)
    point_methods = set(group_points(points))
    for method in sorted(point_methods):
        if method not in promoted_by_method:
            add_issue(errors, "error", name, "method has no promoted audited point", method=method)
        elif len(promoted_by_method[method]) < 2 and len(group_points(points).get(method, [])) > 1:
            add_issue(warnings, "warning", name, "method has only one promoted point; strict decreasing curve has a single visible tier", method=method)

    for method, rows in promoted_by_method.items():
        previous_time = as_float(rows[0].get("total_s"))
        previous_path = as_float(rows[0].get("path_length"))
        for row in rows[1:]:
            current_time = as_float(row.get("total_s"))
            current_path = as_float(row.get("path_length"))
            if previous_time is not None and current_time is not None and current_time <= previous_time + epsilon_time:
                add_issue(errors, "error", name, "promoted charged time is not strictly increasing", method=method, previous_time=previous_time, current_time=current_time)
            if previous_path is not None and current_path is not None and current_path >= previous_path - epsilon_path:
                add_issue(errors, "error", name, "promoted path length is not strictly decreasing", method=method, previous_path=previous_path, current_path=current_path)
            previous_time = current_time if current_time is not None else previous_time
            previous_path = current_path if current_path is not None else previous_path

    return {
        "name": name,
        "point_count": len(points),
        "promoted_count": len(promoted),
        "methods": sorted(point_methods),
        "errors": errors,
        "warnings": warnings,
    }


def check_artifact(path: Path, *, epsilon_path: float, epsilon_time: float) -> dict[str, Any]:
    payload = load_json(path)
    if payload is None:
        return {"path": str(path), "exists": False, "errors": [], "warnings": []}
    summaries: list[dict[str, Any]] = []
    if isinstance(payload.get("summary"), dict):
        summaries.append(check_summary(path.stem, payload["summary"], epsilon_path=epsilon_path, epsilon_time=epsilon_time))
    for key, panel in sorted((payload.get("panels", {}) or {}).items()):
        if isinstance(panel, dict) and isinstance(panel.get("summary"), dict):
            summaries.append(check_summary(f"{path.stem}:{key}", panel["summary"], epsilon_path=epsilon_path, epsilon_time=epsilon_time))
    errors = [issue for summary in summaries for issue in summary["errors"]]
    warnings = [issue for summary in summaries for issue in summary["warnings"]]
    if not summaries:
        add_issue(errors, "error", path.stem, "artifact contains no summary or panel summaries")
    return {
        "path": str(path),
        "exists": True,
        "experiment": payload.get("experiment"),
        "summaries": summaries,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    args = parse_args()
    checks: list[dict[str, Any]] = []
    for path in [args.shelf, args.random]:
        if args.only_existing and not path.exists():
            continue
        checks.append(check_artifact(path, epsilon_path=float(args.epsilon_path), epsilon_time=float(args.epsilon_time)))
    result = {
        "ok": not any(check.get("errors") for check in checks),
        "checks": checks,
        "error_count": sum(len(check.get("errors", [])) for check in checks),
        "warning_count": sum(len(check.get("warnings", [])) for check in checks),
    }
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())