#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, write_json
from experiments.common.metrics import mean, median


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge Exp.6 IRIS+GCS attempts into a fixed-order adaptive profile.")
    parser.add_argument(
        "--inputs",
        nargs="+",
        type=Path,
        default=[
            DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06" / "current_iris_gcs" / "random_robot_iris_gcs_manifest.json",
            DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06" / "current_iris_gcs_r16" / "random_robot_iris_gcs_manifest.json",
            DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06" / "current_iris_gcs_r24_it4" / "random_robot_iris_gcs_manifest.json",
        ],
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06" / "current_iris_gcs_adaptive")
    parser.add_argument("--stage-id", default="adaptive_r8_r16_r24")
    return parser.parse_args()


def as_float(value: Any, default: float = math.nan) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def load_attempts(paths: list[Path]) -> tuple[list[dict[str, Any]], list[str]]:
    rows: list[dict[str, Any]] = []
    sources: list[str] = []
    for order, path in enumerate(paths):
        if not path.exists():
            continue
        payload = json.loads(path.read_text(encoding="utf-8"))
        sources.append(str(path))
        for row in payload.get("rows", []):
            item = dict(row)
            item["_attempt_order"] = order
            item["_attempt_source"] = str(path)
            rows.append(item)
    return rows, sources


def scene_key(row: dict[str, Any]) -> tuple[str, str, int]:
    return (
        str(row.get("robot", "")).lower(),
        str(row.get("difficulty", "")).lower(),
        int(float(row.get("scene_seed", 0) or 0)),
    )


def merge_scene(attempts: list[dict[str, Any]], stage_id: str) -> dict[str, Any]:
    attempts = sorted(attempts, key=lambda row: int(row.get("_attempt_order", 0)))
    first = attempts[0]
    chosen: dict[str, Any] | None = None
    used: list[dict[str, Any]] = []
    for attempt in attempts:
        used.append(attempt)
        if int(float(attempt.get("success_count", 0) or 0)) >= 1:
            chosen = attempt
            break
    if chosen is None:
        chosen = attempts[-1]
    planning_s = sum(as_float(row.get("planning_s"), 0.0) for row in used)
    build_s = sum(as_float(row.get("build_s"), 0.0) for row in used)
    query_s = sum(as_float(row.get("query_s"), 0.0) for row in used)
    audit_s = sum(as_float(row.get("audit_s"), 0.0) for row in used)
    success = int(float(chosen.get("success_count", 0) or 0)) >= 1
    attempt_notes = []
    for row in used:
        gcs_result = (row.get("diagnostics") or {}).get("gcs_result") or {}
        attempt_notes.append({
            "stage_id": row.get("stage_id"),
            "source": row.get("_attempt_source"),
            "success": int(float(row.get("success_count", 0) or 0)) >= 1,
            "planning_s": as_float(row.get("planning_s"), 0.0),
            "status": row.get("status"),
            "note": gcs_result.get("note"),
        })
    return {
        "method": "iris_np_gcs",
        "robot": first.get("robot"),
        "difficulty": first.get("difficulty"),
        "scene_seed": int(float(first.get("scene_seed", 0) or 0)),
        "stage_id": stage_id,
        "budget_s": sum(as_float(row.get("budget_s"), 0.0) for row in used),
        "deep_max_boxes": 0,
        "obstacle_count": chosen.get("obstacle_count", first.get("obstacle_count")),
        "queries": chosen.get("queries", 1),
        "query_count": chosen.get("query_count", 1),
        "success_count": 1 if success else 0,
        "status": "ok" if success else "failed_planning",
        "planning_s": planning_s,
        "build_s": build_s,
        "query_s": query_s,
        "audit_s": audit_s,
        "path_length_mean": chosen.get("path_length_mean") if success else math.nan,
        "raw_segment_fraction": chosen.get("raw_segment_fraction", 0.0) if success else math.nan,
        "final_boxes": math.nan,
        "diagnostics": {
            "adaptive_attempts": attempt_notes,
            "chosen_stage_id": chosen.get("stage_id"),
            "chosen_source": chosen.get("_attempt_source"),
        },
    }


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    keys = sorted({(row["robot"], row["difficulty"], row["stage_id"]) for row in rows})
    for robot, difficulty, stage_id in keys:
        items = [row for row in rows if row["robot"] == robot and row["difficulty"] == difficulty and row["stage_id"] == stage_id]
        successes = [row for row in items if int(float(row.get("success_count", 0) or 0)) >= 1]
        out.append({
            "method": "iris_np_gcs",
            "robot": robot,
            "difficulty": difficulty,
            "stage_id": stage_id,
            "budget_s": median(row.get("budget_s", math.nan) for row in items),
            "deep_max_boxes": 0,
            "scenes": len(items),
            "success_scenes": len(successes),
            "obstacles_median": median(row.get("obstacle_count", math.nan) for row in items),
            "planning_s_median": median(row.get("planning_s", math.nan) for row in items),
            "audit_s_median": median(row.get("audit_s", math.nan) for row in items),
            "path_length_mean": mean(row.get("path_length_mean", math.nan) for row in successes),
            "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in successes),
            "final_boxes_median": math.nan,
            "status": "executed",
        })
    return out


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "method", "robot", "difficulty", "stage_id", "budget_s", "deep_max_boxes", "scenes",
        "success_scenes", "obstacles_median", "planning_s_median", "audit_s_median",
        "path_length_mean", "raw_segment_fraction_median", "final_boxes_median", "status",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> int:
    args = parse_args()
    attempts, sources = load_attempts(list(args.inputs))
    by_scene: dict[tuple[str, str, int], list[dict[str, Any]]] = {}
    for row in attempts:
        by_scene.setdefault(scene_key(row), []).append(row)
    rows = [merge_scene(items, str(args.stage_id)) for _key, items in sorted(by_scene.items())]
    summary = summarize(rows)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out_dir / "random_robot_iris_gcs_manifest.json"
    summary_path = args.out_dir / "random_robot_iris_gcs_summary.csv"
    write_json(manifest_path, {
        "experiment": "exp06_random_robot_iris_gcs_adaptive",
        "source_script": str(Path(__file__).resolve()),
        "note": "Fixed-order adaptive IRIS+GCS profile. Attempts are tried in listed order; planning time is the measured sum of all attempts up to the first audited success.",
        "stage_id": str(args.stage_id),
        "sources": sources,
        "rows": rows,
    })
    write_summary(summary_path, summary)
    print(f"wrote {manifest_path}")
    print(f"wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
