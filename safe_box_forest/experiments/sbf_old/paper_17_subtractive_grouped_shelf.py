#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any


def _bootstrap_imports() -> Path:
    root = Path(__file__).resolve().parents[1]
    build_dir = os.environ.get("SBF_BUILD_DIR")
    candidates = []
    if build_dir:
        candidates.append(Path(build_dir) / "python")
    candidates.extend((root / "build_py310" / "python", root / "build" / "python", root / "python"))
    for candidate in reversed(candidates):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root


ROOT = _bootstrap_imports()

import sbf  # noqa: E402
import RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_04_marcucci_combined as exp4  # noqa: E402
from sbf.marcucci import (  # noqa: E402
    make_bins_obstacles,
    make_combined_queries,
    make_coverage_seeds,
    make_shelves_obstacles,
    make_table_obstacles,
    load_iiwa14_robot,
)


def mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def obstacle_bounds(obstacle: Any) -> list[float]:
    return [float(value) for value in obstacle.bounds]


def aggregate_aabb(obstacles: list[Any], padding: float = 0.0) -> Any:
    if not obstacles:
        raise ValueError("Cannot aggregate an empty obstacle group")
    bounds = [obstacle_bounds(obstacle) for obstacle in obstacles]
    return sbf.Obstacle(
        min(row[0] for row in bounds) - padding,
        min(row[1] for row in bounds) - padding,
        min(row[2] for row in bounds) - padding,
        max(row[3] for row in bounds) + padding,
        max(row[4] for row in bounds) + padding,
        max(row[5] for row in bounds) + padding,
    )


def make_group(name: str, validation: list[Any], carving_mode: str, padding: float) -> Any:
    group = sbf.SubtractiveObstacleGroup()
    group.name = name
    group.validation_obstacles = validation
    group.carving_obstacles = [aggregate_aabb(validation, padding)] if carving_mode == "aggregate" else validation
    return group


def make_grouped_obstacles(carving_mode: str, padding: float) -> list[Any]:
    bins = make_bins_obstacles()
    left_bin = bins[:5]
    right_bin = bins[5:]
    return [
        make_group("shelf", make_shelves_obstacles(), carving_mode, padding),
        make_group("bin_negative_y", left_bin, carving_mode, padding),
        make_group("bin_positive_y", right_bin, carving_mode, padding),
        make_group("table", make_table_obstacles(), carving_mode, padding),
    ]


def default_config_args(args: argparse.Namespace) -> argparse.Namespace:
    cfg_args = exp4.parse_args([])
    cfg_args.preset = args.preset
    cfg_args.threads = args.threads
    cfg_args.task_batch_size = args.task_batch_size
    cfg_args.enable_merger = False
    cfg_args.enable_connector = bool(args.run_connector)
    cfg_args.seed_base = args.seed_base
    cfg_args.max_boxes = args.max_boxes
    cfg_args.timeout_ms = args.timeout_ms
    cfg_args.ffb_depth = args.ffb_depth
    cfg_args.max_consecutive_miss = args.max_consecutive_miss
    cfg_args.connector_pair_timeout_ms = args.connector_timeout_ms
    cfg_args.connector_rrt_timeout_ms = args.connector_timeout_ms
    cfg_args.connector_rrt_iters = args.connector_rrt_iters
    cfg_args.connector_rrt_step_size = args.connector_step_size
    cfg_args.connector_rrt_goal_bias = 0.3
    cfg_args.connector_segment_resolution = 16
    return cfg_args


def configure(args: argparse.Namespace, seed: int) -> Any:
    cfg = exp4.configure(default_config_args(args), seed)
    cfg.dynamic_update.dirty_seed_limit = int(args.dirty_seed_limit)
    cfg.dynamic_update.local_regrow_timeout_ms = float(args.local_regrow_timeout_ms)
    cfg.grower.find_free_box.max_depth = int(args.ffb_depth)
    cfg.grower.find_free_box.reject_seed_collision = True
    return cfg


def build_options(args: argparse.Namespace) -> Any:
    options = sbf.SubtractiveBuildOptions()
    options.run_connector = bool(args.run_connector)
    options.use_validation_obstacles_for_final_scene = True
    return options


def run_queries(forest: Any, queries: list[Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query in queries:
        query_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        row = exp4.query_payload(query, result, time.perf_counter() - query_t0)
        row["name"] = query.label
        rows.append(row)
    return rows


def profile_payload(profile: Any) -> dict[str, Any]:
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    return {
        "total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "connector_ms": float(profile.connector_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "segment_edges": int(profile.segment_edges),
        "adjacency_islands": int(profile.adjacency_islands),
        "bridge_boxes_added": int(profile.bridge_boxes_added),
        "segment_edges_added": int(profile.segment_edges_added),
        "diagnostics": diagnostics,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Subtractive grouped shelf/bin/table SBF runner.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "subtractive_grouped_shelf.json")
    parser.add_argument(
        "--preset",
        choices=["crit_link_coverage", "kdop26_coverage", "support_hull_coverage", "coverage_hybrid", "ifk_strict"],
        default="support_hull_coverage",
    )
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260517)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--task-batch-size", type=int, default=1)
    parser.add_argument("--max-boxes", type=int, default=2400)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--max-consecutive-miss", type=int, default=4000)
    parser.add_argument("--dirty-seed-limit", type=int, default=256)
    parser.add_argument("--local-regrow-timeout-ms", type=float, default=6000.0)
    parser.add_argument("--carving-mode", choices=["aggregate", "exact"], default="aggregate")
    parser.add_argument("--carving-padding", type=float, default=0.0)
    parser.add_argument("--run-connector", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--connector-timeout-ms", type=float, default=2000.0)
    parser.add_argument("--connector-rrt-iters", type=int, default=30000)
    parser.add_argument("--connector-step-size", type=float, default=0.18)
    parser.add_argument("--include-extra-anchors", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot = load_iiwa14_robot()
    groups = make_grouped_obstacles(args.carving_mode, args.carving_padding)
    queries = make_combined_queries()
    coverage_seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=args.include_extra_anchors)]
    runs: list[dict[str, Any]] = []

    for seed in range(max(1, int(args.seeds))):
        cfg = configure(args, seed)
        forest = sbf.SafeBoxForest(robot, cfg)
        t0 = time.perf_counter()
        profile = forest.build_subtractive(groups, coverage_seeds, build_options(args))
        wall_s = time.perf_counter() - t0
        query_rows = run_queries(forest, queries)
        runs.append({
            "seed": seed,
            "wall_s": float(wall_s),
            "profile": profile_payload(profile),
            "queries": query_rows,
            "query_sr": mean([1.0 if row.get("ok") else 0.0 for row in query_rows]),
            "audit_sr": mean([1.0 if row.get("audit_passed") else 0.0 for row in query_rows]),
            "query_time_s_median": median([float(row.get("t_s", 0.0)) for row in query_rows]),
        })

    payload = {
        "schema_version": 1,
        "experiment": "subtractive_grouped_shelf_bin_table",
        "scene": "marcucci_grouped_shelf_bin_table",
        "robot": "iiwa14",
        "preset": args.preset,
        "carving_mode": args.carving_mode,
        "carving_padding": float(args.carving_padding),
        "group_order": [str(group.name) for group in groups],
        "runs": runs,
        "summary": {
            "wall_s_median": median([float(run["wall_s"]) for run in runs]),
            "total_ms_median": median([float(run["profile"]["total_ms"]) for run in runs]),
            "final_boxes_median": median([float(run["profile"]["final_boxes"]) for run in runs]),
            "boxes_removed_median": median([float(run["profile"]["diagnostics"].get("subtractive.carve_boxes_removed", 0.0)) for run in runs]),
            "boxes_added_median": median([float(run["profile"]["diagnostics"].get("subtractive.carve_boxes_added", 0.0)) for run in runs]),
            "local_adjacency_ms_median": median([float(run["profile"]["diagnostics"].get("subtractive.carve_local_adjacency_ms", 0.0)) for run in runs]),
            "global_adjacency_ms_median": median([float(run["profile"]["diagnostics"].get("subtractive.carve_global_adjacency_ms", 0.0)) for run in runs]),
            "query_sr_median": median([float(run["query_sr"]) for run in runs if run.get("query_sr") is not None]),
            "audit_sr_median": median([float(run["audit_sr"]) for run in runs if run.get("audit_sr") is not None]),
        },
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "summary": payload["summary"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())