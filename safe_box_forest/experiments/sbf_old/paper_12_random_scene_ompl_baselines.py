#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import path_length, segment_free  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, mean, median, sbf, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import DEFAULT_RANDOM_DIFFICULTIES, DEFAULT_RANDOM_ROBOTS, DEFAULT_RANDOM_SCENE_SEEDS, make_random_scene, make_robot  # noqa: E402


METHODS = {"prm": "ompl_prm", "bitstar": "ompl_bitstar"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Current SBF OMPL PRM/BIT* baselines for random robot scenes.")
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--methods", default="prm,bitstar")
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--segment-step", type=float, default=0.01)
    parser.add_argument("--prm-build-budget-s", type=float, default=10.0)
    parser.add_argument("--prm-query-budget-s", type=float, default=1.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=64)
    parser.add_argument("--prm-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--bitstar-budget-s", type=float, default=120.0)
    parser.add_argument("--bitstar-restarts", type=int, default=1, help="Deprecated compatibility flag; BIT* now uses one fixed-timeout invocation per trial.")
    parser.add_argument("--bitstar-simplify-time-s", type=float, default=0.2)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=-1)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_exp05_random_ompl_full.json")
    return parser.parse_args()


def parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def parse_methods(raw: str) -> list[str]:
    methods = parse_csv(raw)
    unknown = [method for method in methods if method not in METHODS]
    if unknown:
        raise ValueError(f"unknown methods: {unknown}; choices={sorted(METHODS)}")
    return methods


def audit_path(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    return all(segment_free(robot, obstacles, path[index], path[index + 1], step) for index in range(len(path) - 1))


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def post_audit_query_path(robot: Any, obstacles: list[Any], path: list[list[float]], step: float, *, start: list[float], goal: list[float]) -> bool:
    if len(path) < 2:
        return False
    if point_distance(path[0], list(start)) > 1e-6 or point_distance(path[-1], list(goal)) > 1e-6:
        return False
    return audit_path(robot, obstacles, path, step)


def failure_row(robot_name: str, difficulty: str, method: str, scene_seed: int, trial: int, reason: str) -> dict[str, Any]:
    return {
        "robot": robot_name,
        "difficulty": difficulty,
        "method": method,
        "scene_seed": int(scene_seed),
        "trial": int(trial),
        "scene_valid": False,
        "ok": False,
        "audit_passed": False,
        "reason": reason,
        "build_s": 0.0,
        "query_s": 0.0,
        "path_length": 0.0,
        "nodes": 0,
        "path_waypoint_count": 0,
    }


def run_prm_case(args: argparse.Namespace, robot: Any, obstacles: list[Any], start: list[float], goal: list[float], seed: int) -> dict[str, Any]:
    raw = sbf.ompl_prm_multiquery(
        robot,
        obstacles,
        [list(start)],
        [list(goal)],
        float(args.prm_build_budget_s),
        float(args.prm_query_budget_s),
        float(args.segment_step),
        float(args.prm_simplify_time_s),
        int(seed),
        int(args.prm_max_nearest_neighbors),
    )
    query = dict((raw.get("queries") or [{}])[0])
    path = [list(point) for point in query.get("path", [])]
    exact = str(query.get("status")) == "Exact solution"
    ok = bool(query.get("ok")) and exact and len(path) >= 2
    return {
        "ok": ok,
        "audit_passed": post_audit_query_path(robot, obstacles, path, float(args.segment_step), start=list(start), goal=list(goal)) if ok else False,
        "reason": query.get("reason"),
        "status": query.get("status"),
        "build_s": float(raw.get("build_s", 0.0)),
        "query_s": float(query.get("t_s", 0.0)) if ok else 0.0,
        "path_length": path_length(path) if ok else 0.0,
        "nodes": int(raw.get("nodes", 0) or 0),
        "path_waypoint_count": len(path),
        "planner": "OMPL_PRM_shared_roadmap",
    }


def run_bitstar_case(args: argparse.Namespace, robot: Any, obstacles: list[Any], start: list[float], goal: list[float], seed: int) -> dict[str, Any]:
    raw = sbf.ompl_bitstar_path(
        robot,
        obstacles,
        list(start),
        list(goal),
        float(args.bitstar_budget_s) * 1000.0,
        float(args.segment_step),
        float(args.bitstar_simplify_time_s),
        int(seed),
        int(args.bitstar_samples_per_batch),
        float(args.bitstar_rewire_factor),
        bool(args.bitstar_stop_on_solution_improvement),
    )
    path = [list(point) for point in raw.get("path", [])]
    exact = bool(raw.get("exact_solution")) or str(raw.get("status")) == "Exact solution"
    ok = bool(raw.get("ok")) and exact and len(path) >= 2
    audit_passed = post_audit_query_path(robot, obstacles, path, float(args.segment_step), start=list(start), goal=list(goal)) if ok else False
    return {
        "ok": ok,
        "audit_passed": bool(audit_passed),
        "reason": raw.get("reason"),
        "status": raw.get("status"),
        "build_s": 0.0,
        "query_s": float(raw.get("t_s", 0.0) or 0.0) if ok else 0.0,
        "path_length": path_length(path) if ok else 0.0,
        "nodes": int(raw.get("nodes", 0) or 0),
        "iterations": int(raw.get("iterations", 0) or 0),
        "batches": int(raw.get("batches", 0) or 0),
        "path_waypoint_count": len(path),
        "planner": "OMPL_BITstar",
    }


def summarize_metric(values: list[float]) -> dict[str, float | None]:
    return {"mean": mean(values), "median": median(values)}


def aggregate(rows: list[dict[str, Any]], robots: list[str], difficulties: list[str], methods: list[str]) -> dict[str, Any]:
    groups: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            method_summaries: dict[str, Any] = {}
            for method_arg in methods:
                method = f"ompl_{method_arg}"
                subset = [row for row in rows if row["robot"] == robot_name and row["difficulty"] == difficulty and row["method"] == method]
                successes = [row for row in subset if row.get("ok")]
                method_summaries[method] = {
                    "build_time_s": summarize_metric([float(row.get("build_s", 0.0)) for row in subset]),
                    "query_time_s": summarize_metric([float(row.get("query_s", 0.0)) for row in successes]),
                    "path_length": summarize_metric([float(row.get("path_length", 0.0)) for row in successes]),
                    "success_rate": mean(1.0 if row.get("ok") else 0.0 for row in subset),
                    "audit_success_rate": mean(1.0 if row.get("audit_passed") else 0.0 for row in subset),
                    "n_runs": len(subset),
                    "n_success": len(successes),
                }
            groups.append({"robot": robot_name, "difficulty": difficulty, "methods": method_summaries})
    return {"groups": groups}


def main() -> int:
    args = parse_args()
    robots = parse_csv(args.robots)
    difficulties = parse_csv(args.difficulties)
    methods = parse_methods(args.methods)
    rows: list[dict[str, Any]] = []
    for robot_name in robots:
        robot = make_robot(robot_name)
        for difficulty in difficulties:
            for scene_seed in range(max(1, int(args.scene_seeds))):
                try:
                    scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
                except RuntimeError as exc:
                    reason = f"scene_generation_failed:{exc}"
                    for method_arg in methods:
                        for trial in range(max(1, int(args.trials))):
                            rows.append(failure_row(robot_name, difficulty, METHODS[method_arg], scene_seed, trial, reason))
                    continue
                for method_arg in methods:
                    method = METHODS[method_arg]
                    for trial in range(max(1, int(args.trials))):
                        planner_seed = int(args.seed_base) + 73856093 * scene_seed + 19349663 * trial + 83492791 * (0 if method_arg == "prm" else 1)
                        print(f"[exp5-ompl] start robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial} method={method}", flush=True)
                        result = run_prm_case(args, robot, scene.obstacles, scene.start, scene.goal, planner_seed) if method_arg == "prm" else run_bitstar_case(args, robot, scene.obstacles, scene.start, scene.goal, planner_seed)
                        print(
                            f"[exp5-ompl] done robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial} "
                            f"method={method} ok={bool(result.get('ok'))} audit={bool(result.get('audit_passed'))} query_s={float(result.get('query_s', 0.0)):.3f}",
                            flush=True,
                        )
                        rows.append({
                            "robot": robot_name,
                            "difficulty": difficulty,
                            "method": method,
                            "scene_seed": int(scene_seed),
                            "trial": int(trial),
                            "scene_valid": True,
                            "obstacle_count": len(scene.obstacles),
                            "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
                            "direct_segment_blocked": bool(scene.direct_segment_blocked),
                            "segment_resolution": int(scene.segment_resolution),
                            "rng_seed": int(planner_seed),
                            **result,
                        })

    preserved_methods: list[str] = []
    if args.out_json.exists():
        existing = json.loads(args.out_json.read_text(encoding="utf-8"))
        requested = {METHODS[method_arg] for method_arg in methods}
        preserved_rows = [row for row in existing.get("rows", []) if row.get("method") not in requested]
        if preserved_rows:
            rows = preserved_rows + rows
            preserved_method_keys = {row.get("method") for row in preserved_rows}
            preserved_methods = [method_arg for method_arg, method in METHODS.items() if method in preserved_method_keys]
    aggregate_methods = list(dict.fromkeys([*preserved_methods, *methods]))

    payload = {
        "experiment": "paper_12_random_scene_ompl_baselines",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_sbf_random_scene_ompl_prm_bitstar_fixed_timeout",
        "note": "Current-version random-scene OMPL baselines using the SBF collision checker; PRM uses a fresh shared roadmap per scene/trial and BIT* uses one fixed-timeout invocation per trial.",
        "random_scene_checks": {
            "endpoint_clearance_margin_m": 0.12,
            "direct_start_goal_segment_blocked": True,
            "segment_resolution": 96,
            "scene_profile": args.scene_profile,
        },
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "preserved_methods_from_existing_artifact": preserved_methods,
        "aggregation": aggregate(rows, robots, difficulties, aggregate_methods),
        "rows": rows,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())