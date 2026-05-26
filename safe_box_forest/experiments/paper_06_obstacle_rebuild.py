#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any


def _bootstrap_imports() -> Path:
    root = Path(__file__).resolve().parents[1]
    build_dir = os.environ.get("SBF_BUILD_DIR")
    candidates = []
    if build_dir:
        candidates.append(Path(build_dir) / "python")
    candidates.extend((root / "build_py310" / "python", root / "build" / "python", root / "python", root / "experiments"))
    for candidate in reversed(candidates):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root


ROOT = _bootstrap_imports()
REPO_ROOT = ROOT.parents[1]

import sbf
import paper_04_marcucci_combined as exp4
from sbf.marcucci import make_aabb, make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot


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


def default_config_args(args: argparse.Namespace) -> argparse.Namespace:
    cfg_args = exp4.parse_args([])
    cfg_args.preset = args.preset
    cfg_args.threads = args.threads
    cfg_args.task_batch_size = args.task_batch_size
    cfg_args.enable_merger = False
    cfg_args.enable_connector = True
    cfg_args.seed_base = args.seed_base
    cfg_args.max_boxes = args.max_boxes
    cfg_args.timeout_ms = args.timeout_ms
    cfg_args.ffb_depth = args.ffb_depth
    cfg_args.max_consecutive_miss = 6000
    cfg_args.component_connect_ffb_max_depth = 120
    cfg_args.hard_frontier_failure_threshold = 2
    cfg_args.connector_pair_timeout_ms = args.connector_timeout_ms
    cfg_args.connector_max_pairs_per_gap = 12
    cfg_args.connector_rrt_iters = args.connector_rrt_iters
    cfg_args.connector_rrt_timeout_ms = args.connector_timeout_ms
    cfg_args.connector_rrt_step_size = 0.18
    cfg_args.connector_rrt_goal_bias = 0.25
    cfg_args.connector_segment_resolution = 16
    cfg_args.corridor_refine = False
    cfg_args.bridge_repaired_queries = False
    return cfg_args


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone SBF Exp.6-style obstacle rebuild runner on Marcucci combined scene.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "obstacle_rebuild_standalone.json")
    parser.add_argument("--v6-json", type=Path, default=REPO_ROOT / "cpp" / "v6" / "experiments" / "results_paper" / "exp6_sbf_obstacle_rebuild.json")
    parser.add_argument(
        "--preset",
        choices=["crit_link_coverage", "kdop26_coverage", "support_hull_coverage", "coverage_hybrid", "ifk_strict"],
        default="support_hull_coverage",
    )
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--task-batch-size", type=int, default=1)
    parser.add_argument("--max-boxes", type=int, default=1200)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--connector-timeout-ms", type=float, default=2000.0)
    parser.add_argument("--connector-rrt-iters", type=int, default=30000)
    parser.add_argument("--added-obstacle", choices=["front_bin", "shelf_mouth", "table_edge"], default="front_bin")
    parser.add_argument("--post-update-query", action=argparse.BooleanOptionalAction, default=True, help="Run the canonical queries after localized invalidation and adjacency rebuild.")
    return parser.parse_args()


def added_obstacle(name: str) -> sbf.Obstacle:
    if name == "shelf_mouth":
        return make_aabb(0.78, 0.0, 0.48, 0.08, 0.18, 0.10)
    if name == "table_edge":
        return make_aabb(0.15, -0.45, 0.10, 0.10, 0.10, 0.16)
    return make_aabb(0.25, -0.25, 0.45, 0.10, 0.12, 0.14)


def rebuild_payload(profile: sbf.RebuildProfile) -> dict[str, Any]:
    return {
        "boxes_before": int(profile.boxes_before),
        "boxes_after": int(profile.boxes_after),
        "boxes_removed": int(profile.boxes_removed),
        "raw_boxes_before": int(profile.raw_boxes_before),
        "raw_boxes_after": int(profile.raw_boxes_after),
        "raw_boxes_removed": int(profile.raw_boxes_removed),
        "removal_ratio": float(profile.boxes_removed) / float(profile.boxes_before) if profile.boxes_before else 0.0,
        "islands_after": int(profile.adjacency_islands),
        "collision_check_s": float(profile.collision_check_ms) / 1000.0,
        "adjacency_s": float(profile.adjacency_ms) / 1000.0,
        "rebuild_time_s": float(profile.total_ms) / 1000.0,
    }


def v6_reference(v6_path: Path | None) -> dict[str, Any] | None:
    if v6_path is None or not v6_path.exists():
        return None
    data = json.loads(v6_path.read_text(encoding="utf-8"))
    groups = data.get("aggregation", data.get("aggregate", {})).get("groups", [])
    rebuild_medians = [group.get("rebuild_time_s", {}).get("median") for group in groups]
    rebuild_medians = [float(value) for value in rebuild_medians if value is not None]
    return {
        "v6_path": str(v6_path),
        "scope": data.get("rebuild_scope"),
        "n_groups": len(groups),
        "median_of_group_median_rebuild_s": median(rebuild_medians),
    }


def run_post_update_queries(forest: sbf.SafeBoxForest, queries: list[Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query in queries:
        query_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        wall_s = time.perf_counter() - query_t0
        row = exp4.query_payload(query, result, wall_s)
        row["name"] = query.label
        rows.append(row)
    return rows


def main() -> int:
    args = parse_args()
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    added = added_obstacle(args.added_obstacle)
    runs: list[dict[str, Any]] = []

    for seed in range(max(1, int(args.seeds))):
        cfg = exp4.configure(default_config_args(args), seed)
        forest = sbf.SafeBoxForest(robot, cfg)
        build_t0 = time.perf_counter()
        build = forest.build_coverage(obstacles, seeds)
        build_s = time.perf_counter() - build_t0
        rebuild = forest.add_obstacle_and_rebuild(added)
        row = rebuild_payload(rebuild)
        post_update_queries = run_post_update_queries(forest, queries) if args.post_update_query else []
        row.update({
            "seed": seed,
            "build_time_s": float(build_s),
            "build_profile_total_s": float(build.total_ms) / 1000.0,
            "initial_boxes": int(build.final_boxes),
            "initial_segment_edges": int(build.segment_edges),
            "post_update_queries": post_update_queries,
            "post_update_query_sr": mean([1.0 if query.get("ok") else 0.0 for query in post_update_queries]),
            "post_update_audit_sr": mean([1.0 if query.get("audit_passed") else 0.0 for query in post_update_queries]),
            "post_update_query_time_s_median": median([float(query.get("t_s", 0.0)) for query in post_update_queries]),
        })
        runs.append(row)

    payload = {
        "schema_version": 1,
        "experiment": "exp6_obstacle_rebuild_standalone",
        "description": "Standalone SBF Marcucci combined-scene build followed by one added obstacle, invalidated-box deletion, and adjacency rebuild.",
        "scene": "marcucci_combined",
        "robot": "iiwa14",
        "preset": args.preset,
        "seeds": max(1, int(args.seeds)),
        "added_obstacle": args.added_obstacle,
        "runs": runs,
        "summary": {
            "build_time_s_median": median([run["build_time_s"] for run in runs]),
            "rebuild_time_s_median": median([run["rebuild_time_s"] for run in runs]),
            "collision_check_s_median": median([run["collision_check_s"] for run in runs]),
            "adjacency_s_median": median([run["adjacency_s"] for run in runs]),
            "removal_ratio_median": median([run["removal_ratio"] for run in runs]),
            "boxes_before_median": median([float(run["boxes_before"]) for run in runs]),
            "boxes_removed_median": median([float(run["boxes_removed"]) for run in runs]),
            "post_update_query_sr_median": median([float(run["post_update_query_sr"]) for run in runs if run.get("post_update_query_sr") is not None]),
            "post_update_audit_sr_median": median([float(run["post_update_audit_sr"]) for run in runs if run.get("post_update_audit_sr") is not None]),
            "post_update_query_time_s_median": median([float(run["post_update_query_time_s_median"]) for run in runs if run.get("post_update_query_time_s_median") is not None]),
        },
        "v6_reference": v6_reference(args.v6_json),
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "out_json": str(args.out_json),
        "summary": payload["summary"],
        "v6_reference": payload["v6_reference"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())