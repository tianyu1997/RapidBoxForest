#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
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
    parser = argparse.ArgumentParser(description="Export current Exp.4 Marcucci SBF planned paths and Meshcat HTML.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_current_paths.json")
    parser.add_argument("--out-paths-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_current_paths_only.json")
    parser.add_argument("--out-html", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_current_paths.html")
    parser.add_argument("--query", default="all", help="query label to export, or 'all'")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--task-batch-size", type=int, default=8)
    parser.add_argument("--quality-min-connected-boxes", type=int, default=64)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=450.0)
    parser.add_argument("--max-boxes", type=int, default=5000)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=2)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-merger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--gcs-repo", type=Path, default=None)
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--no-html", action="store_true")
    parser.add_argument("--allow-no-success", action="store_true")
    return parser.parse_args()


def make_exp4_args(args: argparse.Namespace) -> argparse.Namespace:
    exp4 = parse_exp4_args([])
    exp4.seed_base = int(args.seed_base)
    exp4.threads = int(args.threads)
    exp4.task_batch_size = int(args.task_batch_size)
    exp4.quality_min_connected_boxes = int(args.quality_min_connected_boxes)
    exp4.post_connect_time_budget_ms = float(args.post_connect_time_budget_ms)
    exp4.max_boxes = int(args.max_boxes)
    exp4.ffb_depth = int(args.ffb_depth)
    exp4.corridor_refine = bool(args.corridor_refine)
    exp4.corridor_refine_budget_ms = float(args.corridor_refine_budget_ms)
    exp4.corridor_refine_max_boxes = int(args.corridor_refine_max_boxes)
    exp4.corridor_refine_boxes_per_query = int(args.corridor_refine_boxes_per_query)
    exp4.corridor_refine_passes = int(args.corridor_refine_passes)
    exp4.bridge_repaired_queries = bool(args.bridge_repaired_queries)
    exp4.enable_merger = bool(args.enable_merger)
    return exp4


def to_float_list(values: Any) -> list[float]:
    return [float(value) for value in values]


def boxes_payload(boxes: list[Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for box in boxes:
        rows.append({
            "id": int(box.id),
            "tree_id": int(box.tree_id),
            "parent_box_id": int(box.parent_box_id),
            "root_id": int(box.root_id),
            "volume": float(box.volume),
            "safety_status": str(box.safety_status).split(".")[-1],
            "strict_audit_required": bool(box.strict_audit_required),
            "center": to_float_list(box.center()),
            "intervals": [[float(interval.lo), float(interval.hi)] for interval in box.joint_intervals],
        })
    return rows


def segment_edges_payload(edges: list[Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for edge in edges:
        rows.append({
            "id": int(edge.id),
            "source_box_id": int(edge.source_box_id),
            "target_box_id": int(edge.target_box_id),
            "type": str(edge.type).split(".")[-1],
            "validation": str(edge.validation).split(".")[-1],
            "segment_resolution": int(edge.segment_resolution),
            "length": float(edge.length),
            "strict_audit_required": bool(edge.strict_audit_required),
            "waypoints": [to_float_list(waypoint) for waypoint in edge.waypoints],
        })
    return rows


def profile_payload(profile: Any) -> dict[str, Any]:
    return {
        "total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "bridge_boxes_added": int(profile.bridge_boxes_added),
        "segment_edges": int(profile.segment_edges),
        "segment_edges_added": int(profile.segment_edges_added),
        "rrt_segment_edges_added": int(profile.rrt_segment_edges_added),
        "point_gap_segment_edges_added": int(profile.point_gap_segment_edges_added),
        "connector_attempted_pairs": int(profile.connector_attempted_pairs),
        "connector_connected": bool(profile.connector_connected),
        "adjacency_islands": int(profile.adjacency_islands),
        "diagnostics": {str(key): float(value) for key, value in dict(profile.diagnostics).items()},
    }


def run_query_with_bridge(forest: Any, query: Any, exp4_args: argparse.Namespace) -> tuple[Any, dict[str, Any]]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    initial_query_s = time.perf_counter() - query_t0
    bridge_time_s = 0.0
    retry_time_s = 0.0
    bridge_progress = 0
    should_bridge = (not result.success and exp4_args.bridge_failed_queries) or (
        bool(exp4_args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        bridge_progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_time_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        retry_time_s = time.perf_counter() - retry_t0
    return result, {
        "initial_query_s": float(initial_query_s),
        "bridge_time_s": float(bridge_time_s),
        "retry_query_s": float(retry_time_s),
        "total_query_s": float(initial_query_s + bridge_time_s + retry_time_s),
        "bridge_progress": int(bridge_progress),
    }


def main() -> int:
    args = parse_args()
    exp4_args = make_exp4_args(args)
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    if args.query != "all":
        queries = [query for query in queries if query.label == args.query]
        if not queries:
            raise ValueError(f"unknown query label {args.query!r}")
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]

    cfg = configure(exp4_args, int(args.seed))
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, exp4_args)
    build_s = time.perf_counter() - build_t0

    path_rows: list[dict[str, Any]] = []
    for index, query in enumerate(queries):
        result, timings = run_query_with_bridge(forest, query, exp4_args)
        row = query_payload(query, result, timings["total_query_s"])
        row.update(timings)
        row.update({
            "name": query.label,
            "pair_idx": int(index),
            "start": to_float_list(query.start),
            "goal": to_float_list(query.goal),
            "start_box_id": int(result.start_box_id),
            "goal_box_id": int(result.goal_box_id),
            "box_sequence": [int(value) for value in result.box_sequence],
            "segment_edge_sequence": [int(value) for value in result.segment_edge_sequence],
            "waypoints": [to_float_list(waypoint) for waypoint in result.path],
            "waypoint_count": len(result.path),
        })
        path_rows.append(row)

    successful = [row for row in path_rows if bool(row["ok"]) and len(row["waypoints"]) >= 2]
    payload: dict[str, Any] = {
        "experiment": "paper_04_marcucci_current_paths",
        "seed": int(args.seed),
        "params": {
            "quality_min_connected_boxes": int(exp4_args.quality_min_connected_boxes),
            "post_connect_time_budget_ms": float(exp4_args.post_connect_time_budget_ms),
            "corridor_refine": bool(exp4_args.corridor_refine),
            "corridor_refine_budget_ms": float(exp4_args.corridor_refine_budget_ms),
            "bridge_repaired_queries": bool(exp4_args.bridge_repaired_queries),
            "enable_merger": bool(exp4_args.enable_merger),
        },
        "build": {
            "wall_s": float(build_s),
            "prebridge_time_s": float(prebridge_time_s),
            "prebridge_added_boxes": int(prebridge_added_boxes),
            "prebridge_attempts": int(prebridge_attempts),
            "n_boxes": len(forest.boxes()),
            "segment_edge_count": len(forest.segment_edges()),
            "profile": profile_payload(profile),
        },
        "queries": path_rows,
        "successful_path_count": len(successful),
        "boxes": boxes_payload(forest.boxes()),
        "segment_edges": segment_edges_payload(forest.segment_edges()),
    }

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    args.out_paths_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_paths_json.write_text(json.dumps({"paths": path_rows}, indent=2, sort_keys=True), encoding="utf-8")

    if successful and not args.no_html:
        from sbf.drake_visualization import visualize_paths

        visualization = visualize_paths(
            [row["waypoints"] for row in successful],
            [row["name"].replace("->", "_to_") for row in successful],
            gcs_repo=args.gcs_repo,
            save_html=args.out_html,
            static=not args.animate,
            speed=float(args.speed),
            no_show=not args.show,
        )
        payload["visualization"] = visualization
        args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    print(json.dumps({
        "out_json": str(args.out_json),
        "out_paths_json": str(args.out_paths_json),
        "out_html": str(args.out_html),
        "successful_path_count": len(successful),
        "build_s": build_s,
        "n_boxes": len(forest.boxes()),
        "segment_edges": len(forest.segment_edges()),
        "queries": [{"name": row["name"], "ok": row["ok"], "length": row["length"], "repair_count": row["repair_count"]} for row in path_rows],
    }, indent=2, sort_keys=True))
    if not successful and not args.allow_no_success:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())