#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path
from typing import Any, Iterable

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common_rrt_connect import path_length, segment_free  # noqa: E402
from common_sbf_config import ROOT, mean, median, sbf, write_json  # noqa: E402
from sbf.marcucci import make_combined_obstacles, make_combined_queries, load_iiwa14_robot  # noqa: E402


METHOD_OUTPUTS = {
    "prm": "marcucci_ompl_prm.json",
    "bitstar_budget": "marcucci_ompl_bitstar_budget.json",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Current SBF OMPL PRM/BIT* baselines for the shelf+IIWA scene.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true")
    mode.add_argument("--full", action="store_true")
    parser.add_argument("--methods", default="prm,bitstar_budget")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs" / "paper")
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--logical-threads", type=int, default=8)
    parser.add_argument("--seed-base", type=int, default=20260507)
    parser.add_argument("--segment-step", type=float, default=0.01)
    parser.add_argument("--prm-simplify-time-s", type=float, default=0.10)
    parser.add_argument("--bitstar-simplify-time-s", type=float, default=0.2)
    parser.add_argument("--prm-build-budget-s", type=float, default=40.0)
    parser.add_argument("--prm-query-budget-s", type=float, default=2.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-roadmap-retries", type=int, default=3)
    parser.add_argument("--bitstar-budget-s", type=float, default=120.0)
    parser.add_argument("--bitstar-restarts", type=int, default=1, help="Deprecated compatibility flag; BIT* now uses one planner invocation per reported seed.")
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=-1)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def normalize_methods(raw: str) -> list[str]:
    methods = [item.strip() for item in raw.split(",") if item.strip()]
    unknown = [method for method in methods if method not in METHOD_OUTPUTS]
    if unknown:
        raise ValueError(f"unknown OMPL baseline methods: {unknown}; choices={sorted(METHOD_OUTPUTS)}")
    return methods


def mode_counts(args: argparse.Namespace) -> tuple[bool, int, int]:
    quick = bool(args.quick or not args.full)
    seeds = int(args.seeds if args.seeds is not None else (1 if quick else 5))
    timeout = int(args.timeout if args.timeout is not None else (30 if quick else 120))
    return quick, max(1, seeds), max(1, timeout)


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


def summarize_seed_trials(seed_trials: list[dict[str, Any]], labels: list[str]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for label in labels:
        subset = [query for trial in seed_trials for query in trial.get("queries", []) if query.get("query") == label]
        successes = [row for row in subset if row.get("success")]
        rows.append({
            "name": label,
            "sr": mean(1.0 if row.get("success") else 0.0 for row in subset),
            "audit_sr": mean(1.0 if row.get("audit_passed") else 0.0 for row in subset),
            "t_med_s": median(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
            "t_mean_s": mean(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
            "len_med": median(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
            "len_mean": mean(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
            "trial_count": len(subset),
            "success_count": len(successes),
        })
    return rows


def payload_summary(seed_trials: list[dict[str, Any]]) -> dict[str, Any]:
    build_samples = [float(trial["build_s"]) for trial in seed_trials if trial.get("build_s") is not None]
    query_rows = [query for trial in seed_trials for query in trial.get("queries", [])]
    successes = [row for row in query_rows if row.get("success")]
    return {
        "build_s_mean": mean(build_samples),
        "build_s_median": median(build_samples),
        "query_time_s_mean": mean(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
        "query_time_s_median": median(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
        "query_path_rad_mean": mean(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
        "query_path_rad_median": median(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
        "sr": 100.0 * sum(1 for row in query_rows if row.get("success")) / max(1, len(query_rows)),
        "audit_sr": 100.0 * sum(1 for row in query_rows if row.get("audit_passed")) / max(1, len(query_rows)),
        "n_queries": len(query_rows),
        "n_success": sum(1 for row in query_rows if row.get("success")),
    }


def run_prm(args: argparse.Namespace, seeds: int) -> dict[str, Any]:
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    starts = [list(query.start) for query in queries]
    goals = [list(query.goal) for query in queries]
    seed_trials: list[dict[str, Any]] = []
    for seed_index in range(seeds):
        attempts: list[dict[str, Any]] = []
        selected_rows: list[dict[str, Any]] = []
        selected_nodes = 0
        total_build_s = 0.0
        for attempt in range(max(1, int(args.prm_roadmap_retries))):
            rng_seed = int(args.seed_base) + 15485863 * seed_index + 104729 * attempt
            print(f"[exp4-ompl-prm] start seed={seed_index} attempt={attempt} rng_seed={rng_seed}", flush=True)
            result = sbf.ompl_prm_multiquery(
                robot,
                obstacles,
                starts,
                goals,
                float(args.prm_build_budget_s),
                float(args.prm_query_budget_s),
                float(args.segment_step),
                float(args.prm_simplify_time_s),
                int(rng_seed),
                int(args.prm_max_nearest_neighbors),
            )
            total_build_s += float(result.get("build_s", 0.0))
            query_rows: list[dict[str, Any]] = []
            for query, raw in zip(queries, result.get("queries", [])):
                path = [list(point) for point in raw.get("path", [])]
                exact = str(raw.get("status")) == "Exact solution"
                ok = bool(raw.get("ok")) and exact and len(path) >= 2
                audit_passed = post_audit_query_path(robot, obstacles, path, float(args.segment_step), start=list(query.start), goal=list(query.goal)) if ok else False
                row = {
                    "query": query.label,
                    "name": query.label,
                    "seed": int(seed_index),
                    "attempt": int(attempt),
                    "rng_seed": int(rng_seed),
                    "success": bool(ok),
                    "audit_passed": bool(audit_passed),
                    "time_s": float(raw.get("t_s", 0.0)) if ok else None,
                    "path_length": path_length(path) if ok else None,
                    "planner": "OMPL_PRM_shared_roadmap_multistart",
                    "status": raw.get("status"),
                    "reason": raw.get("reason"),
                    "nodes": int(result.get("nodes", 0)),
                    "path_waypoint_count": len(path),
                    "waypoints": path,
                }
                print(
                    f"[exp4-ompl-prm] seed={seed_index} attempt={attempt} query={query.label} ok={row['success']} "
                    f"audit={row['audit_passed']} t_s={row['time_s']}",
                    flush=True,
                )
                query_rows.append(row)
            attempt_record = {
                "attempt": int(attempt),
                "build_s": float(result.get("build_s", 0.0)),
                "nodes": int(result.get("nodes", 0)),
                "success_count": sum(1 for row in query_rows if row.get("success")),
                "audit_success_count": sum(1 for row in query_rows if row.get("audit_passed")),
                "queries": query_rows,
            }
            attempts.append(attempt_record)
            selected_count = sum(1 for row in selected_rows if row.get("audit_passed"))
            if not selected_rows or attempt_record["audit_success_count"] > selected_count:
                selected_rows = query_rows
                selected_nodes = int(result.get("nodes", 0))
            if attempt_record["audit_success_count"] == len(queries):
                break
        seed_trials.append({
            "seed": int(seed_index),
            "build_s": float(total_build_s),
            "selected_attempt": int(selected_rows[0].get("attempt", 0)) if selected_rows else None,
            "attempts": attempts,
            "nodes": selected_nodes,
            "queries": selected_rows,
        })
    return {
        "method": "ompl_prm",
        "scene": "iiwa14_marcucci_combined",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_sbf_ompl_prm_shared_roadmap_multistart",
        "note": "Current SBF PRM baseline: each seed builds shared OMPL PRM roadmaps until all five shelf queries pass audit or the retry cap is reached; all roadmap build time is charged and no query timing samples are discarded.",
        "params": {
            "prm_build_budget_s": float(args.prm_build_budget_s),
            "prm_query_budget_s": float(args.prm_query_budget_s),
            "prm_max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
            "prm_roadmap_retries": int(args.prm_roadmap_retries),
            "segment_step": float(args.segment_step),
            "simplify_time_s": float(args.prm_simplify_time_s),
            "logical_threads": int(args.logical_threads),
        },
        "queries": summarize_seed_trials(seed_trials, [query.label for query in queries]),
        "seed_trials": seed_trials,
        "summary": payload_summary(seed_trials),
    }


def run_bitstar(args: argparse.Namespace, seeds: int) -> dict[str, Any]:
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    seed_trials: list[dict[str, Any]] = []
    for seed_index in range(seeds):
        query_rows: list[dict[str, Any]] = []
        for query_index, query in enumerate(queries):
            rng_seed = (int(args.seed_base) + 32452843 * seed_index + 49979687 * query_index) % 2147483647
            print(f"[exp4-ompl-bitstar] start seed={seed_index} query={query.label} rng_seed={rng_seed}", flush=True)
            raw = sbf.ompl_bitstar_path(
                robot,
                obstacles,
                list(query.start),
                list(query.goal),
                float(args.bitstar_budget_s) * 1000.0,
                float(args.segment_step),
                float(args.bitstar_simplify_time_s),
                int(rng_seed),
                int(args.bitstar_samples_per_batch),
                float(args.bitstar_rewire_factor),
                bool(args.bitstar_stop_on_solution_improvement),
            )
            path = [list(point) for point in raw.get("path", [])]
            exact = bool(raw.get("exact_solution")) or str(raw.get("status")) == "Exact solution"
            ok = bool(raw.get("ok")) and exact and len(path) >= 2
            audit_passed = post_audit_query_path(robot, obstacles, path, float(args.segment_step), start=list(query.start), goal=list(query.goal)) if ok else False
            query_time_s = float(raw.get("t_s", 0.0) or 0.0)
            row = {
                "query": query.label,
                "name": query.label,
                "seed": int(seed_index),
                "rng_seed": int(rng_seed),
                "success": bool(ok),
                "audit_passed": bool(audit_passed),
                "time_s": query_time_s if ok else None,
                "path_length": path_length(path) if ok else None,
                "planner": "OMPL_BITstar",
                "status": raw.get("status"),
                "reason": raw.get("reason"),
                "nodes": int(raw.get("nodes", 0)),
                "iterations": int(raw.get("iterations", 0)),
                "batches": int(raw.get("batches", 0)),
                "path_waypoint_count": len(path),
                "waypoints": path,
            }
            print(
                f"[exp4-ompl-bitstar] done seed={seed_index} query={query.label} ok={row['success']} "
                f"audit={row['audit_passed']} t_s={row['time_s']}",
                flush=True,
            )
            query_rows.append(row)
        seed_trials.append({"seed": int(seed_index), "build_s": 0.0, "queries": query_rows})
    return {
        "method": "ompl_bitstar_budget",
        "scene": "iiwa14_marcucci_combined",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_sbf_ompl_bitstar_fixed_budget_multiseed",
        "note": "Current SBF BIT* baseline using the same SBF collision checker. Each seed/query is one planner invocation with a fixed wall-clock budget; summaries report success rate plus success-only time and path statistics.",
        "params": {
            "bitstar_budget_s": float(args.bitstar_budget_s),
            "bitstar_restarts_deprecated": int(args.bitstar_restarts),
            "bitstar_samples_per_batch": int(args.bitstar_samples_per_batch),
            "bitstar_rewire_factor": float(args.bitstar_rewire_factor),
            "bitstar_stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
            "segment_step": float(args.segment_step),
            "simplify_time_s": float(args.bitstar_simplify_time_s),
            "logical_threads": int(args.logical_threads),
        },
        "queries": summarize_seed_trials(seed_trials, [query.label for query in queries]),
        "seed_trials": seed_trials,
        "summary": payload_summary(seed_trials),
    }


def main() -> int:
    args = parse_args()
    _quick, seeds, _timeout = mode_counts(args)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    outputs: list[str] = []
    for method in normalize_methods(args.methods):
        payload = run_prm(args, seeds) if method == "prm" else run_bitstar(args, seeds)
        out_path = args.out_dir / METHOD_OUTPUTS[method]
        write_json(out_path, payload)
        outputs.append(str(out_path))
        print(f"[paper_04_ompl_baselines] wrote {out_path}", flush=True)
    print(json.dumps({"outputs": outputs, "seeds": seeds}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())