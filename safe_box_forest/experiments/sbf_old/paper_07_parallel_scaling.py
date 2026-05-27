#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_04_marcucci_combined import (  # noqa: E402
    ROOT,
    configure,
    parse_args as parse_exp4_args,
    query_payload,
    refine_corridors,
    sbf,
)
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Parallel scaling sweep for the Marcucci SBF build/query pipeline.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "parallel_scaling_standalone.json")
    parser.add_argument("--threads-grid", default="1,2,4,8")
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--quality-min-connected-boxes", type=int, default=64)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=450.0)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def stage_fraction(stage_s: float | None, total_s: float | None) -> float | None:
    if stage_s is None or total_s is None or total_s <= 0.0:
        return None
    return float(stage_s) / float(total_s)


def make_exp4_args(args: argparse.Namespace, threads: int) -> argparse.Namespace:
    exp4 = parse_exp4_args([])
    exp4.seed_base = int(args.seed_base)
    exp4.threads = int(threads)
    exp4.task_batch_size = max(1, min(8, int(threads)))
    exp4.quality_min_connected_boxes = int(args.quality_min_connected_boxes)
    exp4.post_connect_time_budget_ms = float(args.post_connect_time_budget_ms)
    exp4.corridor_refine = bool(args.corridor_refine)
    exp4.bridge_repaired_queries = bool(args.bridge_repaired_queries)
    return exp4


def run_query_with_bridge(forest: Any, query: Any, exp4_args: argparse.Namespace) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    should_bridge = (not result.success and exp4_args.bridge_failed_queries) or (
        bool(exp4_args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        bridge_progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
        row = query_payload(query, result, query_s + bridge_s + retry_s)
        row["bridge_progress"] = bridge_progress
        row["bridge_time_s"] = float(bridge_s)
    else:
        row = query_payload(query, result, query_s)
        row["bridge_progress"] = 0
        row["bridge_time_s"] = 0.0
    row["name"] = query.label
    return row


def main() -> int:
    args = parse_args()
    threads_grid = [int(item) for item in str(args.threads_grid).split(",") if item.strip()]
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    trials: list[dict[str, Any]] = []

    for threads in threads_grid:
        exp4_args = make_exp4_args(args, threads)
        for seed_index in range(max(1, int(args.seeds))):
            cfg = configure(exp4_args, seed_index)
            forest = sbf.SafeBoxForest(robot, cfg)
            build_t0 = time.perf_counter()
            profile = forest.build_coverage(obstacles, seeds)
            prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, exp4_args)
            build_s = time.perf_counter() - build_t0
            query_rows = [run_query_with_bridge(forest, query, exp4_args) for query in queries]
            trials.append({
                "threads": int(threads),
                "seed_index": int(seed_index),
                "build_s": float(build_s),
                "prebridge_time_s": float(prebridge_time_s),
                "prebridge_added_boxes": int(prebridge_added_boxes),
                "prebridge_attempts": int(prebridge_attempts),
                "box_count": len(forest.boxes()),
                "segment_edges": len(forest.segment_edges()),
                "grow_ms": float(profile.grow_ms),
                "connector_ms": float(profile.connector_ms),
                "adjacency_ms": float(profile.adjacency_ms),
                "audit_sr": mean([1.0 if row.get("audit_passed") else 0.0 for row in query_rows]),
                "query_sr": mean([1.0 if row.get("ok") else 0.0 for row in query_rows]),
                "query_rows": query_rows,
            })

    baseline = None
    summary: list[dict[str, Any]] = []
    for threads in threads_grid:
        rows = [row for row in trials if int(row["threads"]) == int(threads)]
        build_mean = mean([float(row["build_s"]) for row in rows])
        grow_mean_s = mean([float(row["grow_ms"]) / 1000.0 for row in rows])
        connector_mean_s = mean([float(row["connector_ms"]) / 1000.0 for row in rows])
        adjacency_mean_s = mean([float(row["adjacency_ms"]) / 1000.0 for row in rows])
        prebridge_mean_s = mean([float(row["prebridge_time_s"]) for row in rows])
        query_mean_s = mean([
            float(query_row.get("t_s", 0.0))
            for row in rows
            for query_row in row.get("query_rows", [])
        ])
        audit_mean_s = mean([
            float(query_row.get("audit_time_ms", 0.0)) / 1000.0
            for row in rows
            for query_row in row.get("query_rows", [])
        ])
        if threads == threads_grid[0]:
            baseline = build_mean
        speedup = float(baseline) / float(build_mean) if baseline and build_mean else None
        summary.append({
            "threads": int(threads),
            "trials": len(rows),
            "build_mean_s": build_mean,
            "build_median_s": median([float(row["build_s"]) for row in rows]),
            "speedup": speedup,
            "efficiency": speedup / float(threads) if speedup else None,
            "box_count_mean": mean([float(row["box_count"]) for row in rows]),
            "segment_edge_count_mean": mean([float(row["segment_edges"]) for row in rows]),
            "audit_sr": mean([float(row["audit_sr"]) for row in rows if row.get("audit_sr") is not None]),
            "query_sr": mean([float(row["query_sr"]) for row in rows if row.get("query_sr") is not None]),
            "stage_mean_s": {
                "grow": grow_mean_s,
                "connector": connector_mean_s,
                "adjacency": adjacency_mean_s,
                "prebridge": prebridge_mean_s,
                "query": query_mean_s,
                "audit": audit_mean_s,
            },
            "stage_fraction_of_build": {
                "grow": stage_fraction(grow_mean_s, build_mean),
                "connector": stage_fraction(connector_mean_s, build_mean),
                "adjacency": stage_fraction(adjacency_mean_s, build_mean),
                "prebridge": stage_fraction(prebridge_mean_s, build_mean),
            },
        })

    payload = {
        "experiment": "paper_07_parallel_scaling",
        "params": {
            "threads_grid": threads_grid,
            "seeds": int(args.seeds),
            "quality_min_connected_boxes": int(args.quality_min_connected_boxes),
            "post_connect_time_budget_ms": float(args.post_connect_time_budget_ms),
            "corridor_refine": bool(args.corridor_refine),
            "bridge_repaired_queries": bool(args.bridge_repaired_queries),
        },
        "summary": summary,
        "trials": trials,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "summary": summary}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())