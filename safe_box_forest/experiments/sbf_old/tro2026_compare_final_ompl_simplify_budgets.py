#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import final_ompl_simplify_path  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, sbf, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import make_random_scene, make_robot  # noqa: E402
from sbf.marcucci import load_iiwa14_robot, make_combined_obstacles  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.tro2026_generate_tables import RANDOM_SBF_MAIN_ARTIFACT, best_tradeoff_points, safe_float  # noqa: E402


DEFAULT_BUDGETS = "0,1e-4,1e-3,1e-2,5e-2,1e-1"


def parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline budget sweep for final OMPL simplify on the current Table III/IV operating-point paths.")
    parser.add_argument("--outputs", type=Path, default=ROOT / "outputs" / "paper")
    parser.add_argument("--budgets-s", default=DEFAULT_BUDGETS)
    parser.add_argument("--segment-step-shelf", type=float, default=0.04)
    parser.add_argument("--segment-step-random", type=float, default=0.06)
    parser.add_argument("--epsilon-path", type=float, default=1e-6)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_final_ompl_simplify_budget_compare.json")
    return parser.parse_args()


def parse_budgets(raw: str) -> list[float]:
    budgets = sorted({float(item) for item in parse_csv(raw)})
    if not budgets:
        raise ValueError("at least one budget is required")
    return budgets


def extract_path(task: dict[str, Any]) -> list[list[float]] | None:
    for container in (task.get("raw"), task):
        if not isinstance(container, dict):
            continue
        for key in ("waypoints", "path"):
            value = container.get(key)
            if not isinstance(value, list) or len(value) < 2:
                continue
            if not all(isinstance(point, (list, tuple)) for point in value):
                continue
            return [[float(coord) for coord in point] for point in value]
    return None


def mean(values: list[float]) -> float | None:
    return (sum(values) / len(values)) if values else None


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def summarize_rows(rows: list[dict[str, float]]) -> dict[str, Any]:
    before_query = [float(row["before_query_s"]) for row in rows]
    after_query = [float(row["after_query_s"]) for row in rows]
    before_path = [float(row["before_path_length"]) for row in rows]
    after_path = [float(row["after_path_length"]) for row in rows]
    path_delta = [float(row["after_path_length"]) - float(row["before_path_length"]) for row in rows]
    path_delta_pct = [100.0 * delta / float(row["before_path_length"]) for row, delta in zip(rows, path_delta) if abs(float(row["before_path_length"])) > 1e-12]
    added_query = [float(row["added_query_s"]) for row in rows]
    return {
        "count": len(rows),
        "improved_count": sum(1 for row in rows if bool(row["improved"])),
        "mean_before_query_s": mean(before_query),
        "mean_after_query_s": mean(after_query),
        "mean_added_query_s": mean(added_query),
        "median_added_query_s": median(added_query),
        "mean_before_path_length": mean(before_path),
        "mean_after_path_length": mean(after_path),
        "mean_path_delta": mean(path_delta),
        "median_path_delta": median(path_delta),
        "mean_path_delta_pct": mean(path_delta_pct),
        "median_path_delta_pct": median(path_delta_pct),
    }


def source_payload_name(point: dict[str, Any]) -> str:
    scenario_key = str(point.get("scenario_key"))
    method = str(point.get("normalized_method"))
    if scenario_key == "Shelf+IIWA":
        return "tro2026_shelf_iris_np_gcs_anytime.json" if method == "drake_iris_np_gcs" else "tro2026_shelf_anytime_tradeoff_full.json"
    if method == "sbf":
        return RANDOM_SBF_MAIN_ARTIFACT
    if method == "drake_iris_np_gcs":
        return "tro2026_random_iris_np_gcs_anytime.json"
    return "tro2026_random_anytime_tradeoff_full.json"


def selected_tasks(point: dict[str, Any], payload: dict[str, Any]) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    records = payload.get("records", []) if isinstance(payload, dict) else []
    selected: list[tuple[dict[str, Any], dict[str, Any]]] = []
    for raw_index in point.get("raw_record_indices") or []:
        index = int(raw_index)
        if index < 0 or index >= len(records):
            continue
        record = records[index]
        tasks = record.get("incumbent_tasks") or record.get("raw_tasks") or []
        for task in tasks:
            if not isinstance(task, dict):
                continue
            if not bool(task.get("audit_passed")):
                continue
            path = extract_path(task)
            if path is None:
                continue
            selected.append((record, task))
    return selected


def random_context(record: dict[str, Any], payload: dict[str, Any], cache: dict[tuple[str, str, int, int, str], tuple[Any, list[Any]]]) -> tuple[Any, list[Any]]:
    params = record.get("params", {}) if isinstance(record.get("params"), dict) else {}
    payload_params = payload.get("params", {}) if isinstance(payload.get("params"), dict) else {}
    robot_name = str(params.get("robot"))
    difficulty = str(params.get("difficulty"))
    scene_seed = int(params.get("scene_seed", 0))
    seed_base = int(payload_params.get("seed_base", 20260504))
    scene_profile = str(payload_params.get("scene_profile", "balanced"))
    cache_key = (robot_name, difficulty, scene_seed, seed_base, scene_profile)
    if cache_key not in cache:
        robot = make_robot(robot_name)
        scene = make_random_scene(robot_name, difficulty, seed_base + 1009 * scene_seed, scene_profile=scene_profile)
        cache[cache_key] = (robot, scene.obstacles)
    return cache[cache_key]


def budget_result_rows(
    point: dict[str, Any],
    payload: dict[str, Any],
    budgets: list[float],
    *,
    shelf_robot: Any,
    shelf_obstacles: list[Any],
    random_cache: dict[tuple[str, str, int, int, str], tuple[Any, list[Any]]],
    epsilon_path: float,
    segment_step_shelf: float,
    segment_step_random: float,
) -> tuple[list[dict[str, Any]], str | None]:
    selected = selected_tasks(point, payload)
    if not selected:
        return [], "no_audited_path_payload"
    rows: list[dict[str, Any]] = []
    scenario_key = str(point.get("scenario_key"))
    for budget_s in budgets:
        budget_rows: list[dict[str, float]] = []
        for record, task in selected:
            path = extract_path(task)
            if path is None:
                continue
            if scenario_key == "Shelf+IIWA":
                robot, obstacles = shelf_robot, shelf_obstacles
                segment_step = float(segment_step_shelf)
            else:
                robot, obstacles = random_context(record, payload, random_cache)
                segment_step = float(segment_step_random)
            result = final_ompl_simplify_path(
                sbf,
                robot,
                obstacles,
                path,
                segment_step=segment_step,
                simplify_time_s=float(budget_s),
                epsilon_path=float(epsilon_path),
            )
            before_query_s = safe_float(task.get("query_s"), 0.0)
            before_path_length = safe_float(task.get("path_length"), 0.0)
            after_query_s = before_query_s + safe_float(result.get("query_s"), 0.0)
            after_path_length = before_path_length
            if bool(result.get("applied")):
                after_path_length = safe_float(result.get("path_length"), before_path_length)
            budget_rows.append({
                "before_query_s": before_query_s,
                "after_query_s": after_query_s,
                "added_query_s": safe_float(result.get("query_s"), 0.0),
                "before_path_length": before_path_length,
                "after_path_length": after_path_length,
                "improved": 1.0 if bool(result.get("applied")) else 0.0,
            })
        if not budget_rows:
            continue
        rows.append({
            "budget_s": float(budget_s),
            **summarize_rows(budget_rows),
        })
    return rows, None


def main() -> int:
    args = parse_args()
    budgets = parse_budgets(args.budgets_s)
    outputs = args.outputs
    payload_cache = {
        name: json.loads((outputs / name).read_text(encoding="utf-8"))
        for name in {
            "tro2026_shelf_anytime_tradeoff_full.json",
            "tro2026_shelf_iris_np_gcs_anytime.json",
            "tro2026_random_anytime_tradeoff_full.json",
            "tro2026_random_iris_np_gcs_anytime.json",
            RANDOM_SBF_MAIN_ARTIFACT,
        }
        if (outputs / name).exists()
    }

    shelf_robot = load_iiwa14_robot()
    shelf_obstacles = make_combined_obstacles()
    random_cache: dict[tuple[str, str, int, int, str], tuple[Any, list[Any]]] = {}

    per_point: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    overall_by_budget: dict[float, list[dict[str, float]]] = {float(budget): [] for budget in budgets}
    by_method_budget: dict[str, dict[float, list[dict[str, float]]]] = {}

    for point in best_tradeoff_points(outputs):
        method = str(point.get("normalized_method"))
        payload_name = source_payload_name(point)
        payload = payload_cache.get(payload_name)
        if payload is None:
            skipped.append({
                "scenario_key": str(point.get("scenario_key")),
                "method": method,
                "reason": f"missing_payload:{payload_name}",
            })
            continue
        rows, reason = budget_result_rows(
            point,
            payload,
            budgets,
            shelf_robot=shelf_robot,
            shelf_obstacles=shelf_obstacles,
            random_cache=random_cache,
            epsilon_path=float(args.epsilon_path),
            segment_step_shelf=float(args.segment_step_shelf),
            segment_step_random=float(args.segment_step_random),
        )
        if reason is not None:
            skipped.append({
                "scenario_key": str(point.get("scenario_key")),
                "method": method,
                "reason": reason,
            })
            continue
        per_point.append({
            "scenario_key": str(point.get("scenario_key")),
            "scenario": str(point.get("scenario")),
            "method": method,
            "payload": payload_name,
            "build_s": safe_float(point.get("build_s"), 0.0),
            "baseline_query_s": safe_float(point.get("query_s"), 0.0),
            "baseline_path_length": safe_float(point.get("path_length"), 0.0),
            "seed_count": int(safe_float(point.get("seed_count"), 1.0)),
            "task_count": int(safe_float(point.get("task_count"), 1.0)),
            "budgets": rows,
        })
        method_bucket = by_method_budget.setdefault(method, {float(budget): [] for budget in budgets})
        for row in rows:
            overall_by_budget[float(row["budget_s"])] .append({
                "before_query_s": float(row["mean_before_query_s"] or 0.0),
                "after_query_s": float(row["mean_after_query_s"] or 0.0),
                "added_query_s": float(row["mean_added_query_s"] or 0.0),
                "before_path_length": float(row["mean_before_path_length"] or 0.0),
                "after_path_length": float(row["mean_after_path_length"] or 0.0),
                "improved": 1.0 if int(row["improved_count"]) > 0 else 0.0,
            })
            method_bucket[float(row["budget_s"])] .append({
                "before_query_s": float(row["mean_before_query_s"] or 0.0),
                "after_query_s": float(row["mean_after_query_s"] or 0.0),
                "added_query_s": float(row["mean_added_query_s"] or 0.0),
                "before_path_length": float(row["mean_before_path_length"] or 0.0),
                "after_path_length": float(row["mean_after_path_length"] or 0.0),
                "improved": 1.0 if int(row["improved_count"]) > 0 else 0.0,
            })

    summary = {
        "overall": [
            {"budget_s": float(budget), **summarize_rows(rows)}
            for budget, rows in sorted(overall_by_budget.items())
            if rows
        ],
        "by_method": {
            method: [
                {"budget_s": float(budget), **summarize_rows(rows)}
                for budget, rows in sorted(budget_rows.items())
                if rows
            ]
            for method, budget_rows in sorted(by_method_budget.items())
        },
    }
    payload = {
        "experiment": "tro2026_final_ompl_simplify_budget_compare",
        "source_script": str(Path(__file__).resolve()),
        "note": "Offline final-OMPL-simplify budget sweep on the current Table III/IV operating-point paths. Rows without stored waypoint paths in the artifacts are reported under skipped; this currently excludes the pre-regeneration IRIS anytime artifacts.",
        "budgets_s": budgets,
        "summary": summary,
        "rows": per_point,
        "skipped": skipped,
    }
    write_json(args.out_json, payload)
    print(json.dumps({
        "out_json": str(args.out_json),
        "rows": len(per_point),
        "skipped": len(skipped),
        "budgets": budgets,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())