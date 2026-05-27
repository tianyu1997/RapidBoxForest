#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import rrt_connect  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, mean, median, write_json  # noqa: E402
from sbf.marcucci import make_combined_obstacles, make_combined_queries, load_iiwa14_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OMPL RRTConnect baseline for the shelf+IIWA Marcucci scene.")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--timeout-ms", type=float, default=10000.0)
    parser.add_argument("--max-iters", type=int, default=30000)
    parser.add_argument("--step-size", type=float, default=0.35)
    parser.add_argument("--goal-bias", type=float, default=0.10)
    parser.add_argument("--segment-step", type=float, default=0.04)
    parser.add_argument("--shortcut-passes", type=int, default=120)
    parser.add_argument("--simplify-time-s", type=float, default=0.1)
    parser.add_argument("--queries", default="all")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_exp04_rrt_connect_full.json")
    return parser.parse_args()


def select_queries(raw: str, queries: list[Any]) -> list[Any]:
    if raw.strip().lower() == "all":
        return queries
    wanted = {item.strip() for item in raw.split(",") if item.strip()}
    selected = [query for query in queries if query.label in wanted]
    missing = sorted(wanted - {query.label for query in selected})
    if missing:
        raise ValueError(f"unknown query labels: {missing}")
    return selected


def summarize_queries(seed_trials: list[dict[str, Any]], query_labels: list[str]) -> list[dict[str, Any]]:
    summary: list[dict[str, Any]] = []
    for label in query_labels:
        rows = [query for trial in seed_trials for query in trial.get("queries", []) if query.get("name") == label]
        successes = [row for row in rows if row.get("success")]
        summary.append({
            "name": label,
            "sr": mean(1.0 if row.get("success") else 0.0 for row in rows),
            "audit_sr": mean(1.0 if row.get("audit_passed") else 0.0 for row in rows),
            "t_med_s": median(float(row.get("time_s", 0.0)) for row in successes),
            "t_mean_s": mean(float(row.get("time_s", 0.0)) for row in successes),
            "len_med": median(float(row.get("path_length", 0.0)) for row in successes),
            "len_mean": mean(float(row.get("path_length", 0.0)) for row in successes),
            "trial_count": len(rows),
            "success_count": sum(1 for row in rows if row.get("success")),
        })
    return summary


def main() -> int:
    args = parse_args()
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = select_queries(str(args.queries), make_combined_queries())
    bounds = [(float(interval.lo), float(interval.hi)) for interval in robot.joint_limits().limits]
    seed_trials: list[dict[str, Any]] = []
    flat_rows: list[dict[str, Any]] = []

    for trial in range(max(1, int(args.trials))):
        query_rows: list[dict[str, Any]] = []
        for query_index, query in enumerate(queries):
            rng_seed = int(args.seed_base) + 7919 * trial + 104729 * query_index
            rng = random.Random(rng_seed)
            print(f"[exp4-rrt] start trial={trial} query={query.label} rng_seed={rng_seed}", flush=True)
            result = rrt_connect(robot, obstacles, list(query.start), list(query.goal), bounds, rng, args, seed=rng_seed)
            row = {
                "name": query.label,
                "query": query.label,
                "trial": int(trial),
                "rng_seed": int(rng_seed),
                "success": bool(result.get("ok")),
                "ok": bool(result.get("ok")),
                "audit_passed": bool(result.get("audit_passed", result.get("ok", False))),
                "reason": result.get("reason"),
                "time_s": float(result.get("t_s", 0.0)),
                "path_length": float(result.get("length", 0.0)) if result.get("ok") else 0.0,
                "iterations": int(result.get("iterations", 0)),
                "nodes": int(result.get("nodes", 0)),
                "path_waypoint_count": len(result.get("path", [])),
                "planner": result.get("planner", "OMPL_RRTConnect"),
                "ompl_status": result.get("ompl_status"),
            }
            print(
                f"[exp4-rrt] done trial={trial} query={query.label} ok={row['ok']} "
                f"reason={row['reason']} t_s={row['time_s']:.3f}",
                flush=True,
            )
            query_rows.append(row)
            flat_rows.append(dict(row))
        seed_trials.append({"seed": int(trial), "build_s": 0.0, "queries": query_rows})

    query_summary = summarize_queries(seed_trials, [query.label for query in queries])
    payload = {
        "experiment": "paper_04_rrt_connect_baseline",
        "robot": "iiwa14",
        "scene": "marcucci_combined",
        "source_script": str(Path(__file__).resolve()),
        "note": "OMPL RRTConnect baseline using the same SBF collision checker on the shelf+IIWA combined scene. It has no reusable build phase.",
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "summary": {
            "build_s_mean": 0.0,
            "build_s_median": 0.0,
            "trial_count": max(1, int(args.trials)),
            "query_count": len(queries),
            "path_count": len(flat_rows),
            "audit_sr": mean(1.0 if row.get("audit_passed") else 0.0 for row in flat_rows),
            "sr": mean(1.0 if row.get("success") else 0.0 for row in flat_rows),
        },
        "queries": query_summary,
        "seed_trials": seed_trials,
        "rows": flat_rows,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "summary": payload["summary"], "queries": query_summary}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())