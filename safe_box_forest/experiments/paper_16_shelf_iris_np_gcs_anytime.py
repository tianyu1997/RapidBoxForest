#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common_anytime_tradeoff import aggregate_stage_records, final_ompl_simplify_path, incumbent_stage_record, task_result, update_incumbents  # noqa: E402
from common_rrt_connect import path_length as list_path_length  # noqa: E402
from common_sbf_config import ROOT, sbf, write_json  # noqa: E402
from paper_04_iris_np_gcs_baseline import (  # noqa: E402
    DEFAULT_IRIS_NP,
    GCS_REPO,
    build_regions_for_seed,
    configure_threads,
    region_seed_configs,
    solve_regions_gcs,
    workload,
)
from sbf.marcucci import load_iiwa14_robot, make_combined_obstacles  # noqa: E402


def parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def parse_int_grid(raw: str) -> list[int]:
    return [int(float(item)) for item in parse_csv(raw)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prefix-budget IRIS-NP+GCS anytime curve for shelf+IIWA.")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260507)
    parser.add_argument("--logical-threads", type=int, default=8)
    parser.add_argument("--budget-s", type=float, default=600.0)
    parser.add_argument("--stage-region-counts", default="2,4,6,8")
    parser.add_argument("--iteration-limit", type=int, default=3)
    parser.add_argument("--relative-termination-threshold", type=float, default=2e-2)
    parser.add_argument("--query-time-limit-s", type=float, default=30.0)
    parser.add_argument("--allow-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rounding-max-paths", type=int, default=10)
    parser.add_argument("--rounding-max-trials", type=int, default=100)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-rounding", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gcs-repo", type=Path, default=GCS_REPO)
    parser.add_argument("--segment-step", type=float, default=0.04)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.01)
    parser.add_argument("--epsilon-path", type=float, default=1e-6)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_shelf_iris_np_gcs_anytime.json")
    return parser.parse_args()


def prefix_build_s(timings: list[float], target_regions: int) -> float:
    if not timings:
        return 0.0
    return sum(float(value) for value in timings[: max(1, min(int(target_regions), len(timings)))])


def run_seed(args: argparse.Namespace, seed_index: int, stages: list[int], items: list[dict[str, Any]], robot: Any, obstacles: list[Any]) -> list[dict[str, Any]]:
    seed = int(args.seed_base) + int(seed_index)
    print(f"[shelf-iris-anytime] seed={seed_index}", flush=True)
    regions, timings, failures, checker = build_regions_for_seed(args, seed, region_seed_configs(items), float(args.budget_s))
    incumbents: dict[str, dict[str, Any]] = {}
    records: list[dict[str, Any]] = []
    for stage_index, target in enumerate(stages):
        prefix = regions[: min(int(target), len(regions))]
        raw_tasks: list[dict[str, Any]] = []
        stage_query_s = 0.0
        for query_index, item in enumerate(items):
            result = solve_regions_gcs(
                np.asarray(item["q_start"], dtype=float),
                np.asarray(item["q_goal"], dtype=float),
                prefix,
                seed=seed + 101 * stage_index + query_index,
                checker=checker,
                query_time_limit_s=float(args.query_time_limit_s),
                allow_repair=bool(args.allow_repair),
                rounding_max_paths=int(args.rounding_max_paths),
                rounding_max_trials=int(args.rounding_max_trials),
                gcs_preprocessing=bool(args.gcs_preprocessing),
                use_rounding=bool(args.use_rounding),
            )
            query_s = float(result.get("time_s", 0.0) or 0.0)
            path = [list(point) for point in result.get("path", [])]
            if bool(result.get("success")) and len(path) >= 2:
                final_simplify = final_ompl_simplify_path(
                    sbf,
                    robot,
                    obstacles,
                    path,
                    segment_step=float(args.segment_step),
                    simplify_time_s=float(args.final_ompl_simplify_time_s),
                    epsilon_path=float(args.epsilon_path),
                )
                path = [list(point) for point in final_simplify["path"]]
                query_s += float(final_simplify["query_s"])
            stage_query_s += query_s
            raw_tasks.append(task_result(
                name=str(item["label"]),
                ok=bool(result.get("success")),
                audit_passed=bool(result.get("success")),
                path_length=list_path_length(path) if result.get("success") and len(path) >= 2 else None,
                query_s=query_s,
                reason=None if result.get("success") else str(result.get("note", "gcs_failed")),
                extra={
                    "raw": {**result, "path": path},
                    "n_regions": len(prefix),
                    "waypoints": path,
                    "waypoint_count": len(path),
                    "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if result.get("success") and len(path) >= 2 else 0.0,
                    "ompl_final_simplify_applied": bool(final_simplify["applied"]) if result.get("success") and len(path) >= 2 else False,
                    "ompl_final_simplify_reason": str(final_simplify["reason"]) if result.get("success") and len(path) >= 2 else "disabled",
                },
            ))
        incumbents, improved = update_incumbents(incumbents, raw_tasks, epsilon_path=float(args.epsilon_path))
        build_s = prefix_build_s([float(value) for value in timings], int(target))
        records.append(incumbent_stage_record(
            method="drake_iris_np_gcs",
            stage_id=f"r{target}",
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=len(items),
            cumulative_build_s=build_s,
            cumulative_query_s=stage_query_s,
            stage_build_s=build_s,
            stage_query_s=stage_query_s,
            raw_tasks=raw_tasks,
            incumbents=incumbents,
            improved_tasks=improved,
            params={"target_regions": int(target), "available_regions": len(regions), "failed_region_seeds": failures},
            protocol="independent_prefix_exit_condition",
        ))
    return records


def main() -> int:
    args = parse_args()
    configure_threads(int(args.logical_threads))
    stages = parse_int_grid(args.stage_region_counts)
    items = workload()
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    records: list[dict[str, Any]] = []
    for seed_index in range(max(1, int(args.seeds))):
        records.extend(run_seed(args, seed_index, stages, items, robot, obstacles))
    summary = aggregate_stage_records(records, epsilon_path=float(args.epsilon_path))
    payload = {
        "experiment": "tro2026_shelf_iris_np_gcs_anytime",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_shelf_iris_np_gcs_prefix_exit_tradeoff",
        "note": "IRIS-NP regions are grown once from the shelf seed schedule. Prefix region counts emulate earlier/later IRIS exit conditions; GCS is solved independently at each prefix.",
        "params": {
            **{key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
            "edge_step_size": DEFAULT_IRIS_NP["edge_step_size"],
            "env_padding": DEFAULT_IRIS_NP["env_padding"],
            "self_padding": DEFAULT_IRIS_NP["self_padding"],
        },
        "summary": summary,
        "records": records,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "records": len(records), "points": len(summary.get("points", []))}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())