#!/usr/bin/env python3
"""Summarize measured performance effects for docs/improve.md artifacts."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def fmt(value: Any, digits: int = 3) -> str:
    number = finite(value)
    if number is None:
        return "n/a"
    return f"{number:.{digits}g}"


def production_rows(bridge: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item in bridge.get("manifests", []):
        row_metrics = item.get("row_metrics") or {}
        summary_metrics = item.get("summary_metrics") or {}
        metrics = {**row_metrics, **summary_metrics}
        rows.append({
            "experiment": item.get("experiment"),
            "status": item.get("status"),
            "success_count": metrics.get("success_count", metrics.get("success_queries")),
            "query_count": metrics.get("query_count", metrics.get("total_queries")),
            "planning_s": metrics.get("planning_s_median", metrics.get("planning_s")),
            "offline_build_s": metrics.get("offline_build_s_median", metrics.get("offline_build_s")),
            "online_per_query_s": metrics.get("online_per_query_s_median", metrics.get("online_per_query_s")),
            "audit_s": metrics.get("audit_s_median", metrics.get("audit_s")),
            "path_length_mean": metrics.get("path_length_mean"),
            "raw_segment_fraction": metrics.get("raw_segment_fraction_median", metrics.get("raw_segment_fraction")),
        })
    return rows


def production_ablation_rows(ablation: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item in ablation.get("manifests", []):
        for metric in item.get("metrics", []):
            rows.append({
                "experiment": item.get("experiment"),
                "case": metric.get("case", metric.get("method")),
                "robot": metric.get("robot"),
                "difficulty": metric.get("difficulty"),
                "box_budget": metric.get("box_budget"),
                "planning_s": metric.get("planning_s_median", metric.get("planning_s")),
                "offline_build_s": metric.get("offline_build_s_median", metric.get("offline_build_s")),
                "online_per_query_s": metric.get("online_per_query_s_median", metric.get("online_per_query_s")),
                "path_length_mean": metric.get("path_length_mean"),
                "raw_segment_fraction": metric.get("raw_segment_fraction_median", metric.get("raw_segment_fraction")),
                "final_boxes": metric.get("final_boxes_median", metric.get("final_boxes")),
            })
    return rows


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Improve Performance Change Summary",
        "",
        "This summary combines sidecar C-LECT mechanism measurements with minimal",
        "production runner smoke measurements. Production smoke rows are not full",
        "paper experiments.",
        "",
        "## Sidecar Mechanism Effects",
        "",
        "| Metric | Value |",
        "| --- | ---: |",
    ]
    for key, value in payload["sidecar_mechanism_effects"].items():
        lines.append(f"| `{key}` | {fmt(value, 4)} |")
    lines.extend([
        "",
        "## Scaling Effects",
        "",
        "| Metric | Value |",
        "| --- | ---: |",
    ])
    for key, value in payload["scaling_effects"].items():
        lines.append(f"| `{key}` | {fmt(value, 4)} |")
    lines.extend([
        "",
        "## Production Executed Smoke",
        "",
        "| Experiment | SR | Plan (s) | Build (s) | Online/q (s) | Audit (s) | Path | Segment |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in payload["production_executed_smoke"]:
        success = row.get("success_count")
        total = row.get("query_count")
        sr = f"{int(success)}/{int(total)}" if success is not None and total is not None else "n/a"
        lines.append(
            f"| `{row.get('experiment')}` | {sr} | {fmt(row.get('planning_s'))} | "
            f"{fmt(row.get('offline_build_s'))} | {fmt(row.get('online_per_query_s'))} | "
            f"{fmt(row.get('audit_s'))} | {fmt(row.get('path_length_mean'))} | "
            f"{fmt(row.get('raw_segment_fraction'))} |"
        )
    lines.extend([
        "",
        "## Production C-LECT Ablation/Scale",
        "",
        "| Experiment | Case | Robot | Difficulty | Plan (s) | Build (s) | Online/q (s) | Path | Segment | Boxes |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in payload.get("production_clect_ablation", []):
        lines.append(
            f"| `{row.get('experiment')}` | `{row.get('case')}` | "
            f"`{row.get('robot') or ''}` | `{row.get('difficulty') or ''}` | "
            f"{fmt(row.get('planning_s'))} | {fmt(row.get('offline_build_s'))} | "
            f"{fmt(row.get('online_per_query_s'))} | {fmt(row.get('path_length_mean'))} | "
            f"{fmt(row.get('raw_segment_fraction'))} | {fmt(row.get('final_boxes'))} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", default="improve_workspace/performance_change_summary.json")
    parser.add_argument("--md-out", default="improve_workspace/performance_change_summary.md")
    args = parser.parse_args()

    ablation = load_json(REPO_ROOT / "improve_workspace/clect_ablation_benchmark.json")
    suite = load_json(REPO_ROOT / "improve_workspace/clect_experiment_suite.json")
    scaling = load_json(REPO_ROOT / "improve_workspace/clect_scaling_experiment.json")
    bridge = load_json(REPO_ROOT / "improve_workspace/production_experiment_bridge_executed.json")
    production_ablation = load_json(REPO_ROOT / "improve_workspace/production_clect_ablation.json")

    payload = {
        "sidecar_mechanism_effects": {
            **ablation.get("summary", {}),
            **suite.get("summary", {}),
        },
        "scaling_effects": scaling.get("summary", {}),
        "production_executed_smoke": production_rows(bridge),
        "production_clect_ablation": production_ablation_rows(production_ablation),
        "source_files": {
            "ablation": "improve_workspace/clect_ablation_benchmark.json",
            "suite": "improve_workspace/clect_experiment_suite.json",
            "scaling": "improve_workspace/clect_scaling_experiment.json",
            "production_executed": "improve_workspace/production_experiment_bridge_executed.json",
            "production_clect_ablation": "improve_workspace/production_clect_ablation.json",
        },
    }
    json_path = REPO_ROOT / args.json_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_md(md_path, payload)
    print(json.dumps({
        "json_out": str(json_path),
        "md_out": str(md_path),
        "production_rows": len(payload["production_executed_smoke"]),
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
