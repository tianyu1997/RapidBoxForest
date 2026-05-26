#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any


def _bootstrap_imports() -> Path:
    root = Path(__file__).resolve().parents[3]
    for candidate in (root / "python", root / "build" / "python", root / "build_py310" / "python"):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root


ROOT = _bootstrap_imports()

import sbf
from sbf.marcucci import (
    make_bins_obstacles,
    make_combined_obstacles,
    make_combined_queries,
    make_coverage_seeds,
    make_shelves_obstacles,
    make_table_obstacles,
    load_iiwa14_robot,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run standalone SBF on the Marcucci shelf+IIWA scene and export Drake visualization artifacts.")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs" / "marcucci_shelf_demo")
    parser.add_argument("--query", default="AS->TS", help="query label to run, or 'all'")
    parser.add_argument("--scene", choices=["combined", "shelves", "bins", "table"], default="combined")
    parser.add_argument("--endpoint-source", choices=["critsample", "ifk"], default="critsample")
    parser.add_argument("--envelope", choices=["link", "hull"], default="link")
    parser.add_argument("--seed-set", choices=["query", "canonical", "extended"], default="query")
    parser.add_argument("--max-boxes", type=int, default=900)
    parser.add_argument("--timeout-ms", type=float, default=90000.0)
    parser.add_argument("--ffb-depth", type=int, default=28)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--task-batch-size", type=int, default=8)
    parser.add_argument("--connector-bridge-boxes", type=int, default=320)
    parser.add_argument("--connector-pairs", type=int, default=10)
    parser.add_argument("--gap-connect-tol", type=float, default=0.0)
    parser.add_argument("--global-connector", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--prebridge-query", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prebridge-rounds", type=int, default=1)
    parser.add_argument("--rrt-iters", type=int, default=2200)
    parser.add_argument("--reject-seed-collision", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--save-html", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--animate", action="store_true", help="record a Meshcat animation instead of static end-effector traces")
    parser.add_argument("--show", action="store_true", help="keep Meshcat open after exporting")
    parser.add_argument("--gcs-repo", type=Path, default=None)
    parser.add_argument("--allow-failures", action="store_true")
    return parser.parse_args()


def configure_sbf(args: argparse.Namespace) -> sbf.SBFConfig:
    config = sbf.SBFConfig()
    config.endpoint_source.source = sbf.EndpointSource.CritSample if args.endpoint_source == "critsample" else sbf.EndpointSource.IFK
    envelope_map = {
        "link": sbf.EnvelopeType.LinkIAABB,
        "hull": sbf.EnvelopeType.Hull_Grid,
    }
    config.envelope_type.type = envelope_map[args.envelope]
    config.envelope_type.n_subdivisions = 4

    config.runtime.mode = sbf.ExecutionMode.Parallel if args.threads > 1 else sbf.ExecutionMode.Inline
    config.runtime.n_threads = max(1, int(args.threads))
    config.runtime.batch_size = max(1, int(args.task_batch_size))
    config.runtime.parallel_threshold = 1

    config.grower.mode = sbf.GrowerMode.RRT
    config.grower.max_boxes = int(args.max_boxes)
    config.grower.timeout_ms = float(args.timeout_ms)
    config.grower.max_consecutive_miss = 3500
    config.grower.rng_seed = 20260503
    config.grower.n_threads = max(1, int(args.threads))
    config.grower.task_batch_size = max(1, int(args.task_batch_size))
    config.grower.parallel_threshold = 1
    config.grower.worker_local_ffb = True
    config.grower.rrt_goal_bias = 0.20
    config.grower.rrt_step_ratio = 0.08
    config.grower.unexplored_sample_prob = 0.70
    config.grower.stop_after_connect = False
    config.grower.post_connect_extra_boxes = 120
    config.grower.find_free_box.max_depth = int(args.ffb_depth)
    config.grower.find_free_box.deadline_ms = 0.0
    config.grower.find_free_box.split_reserved_leaf = True
    config.grower.find_free_box.split_unknown_leaf = True
    config.grower.find_free_box.reject_seed_collision = bool(args.reject_seed_collision)

    config.enable_merger = True
    config.merger.max_rounds = 4
    config.merger.target_boxes = 0
    config.merger.n_threads = max(1, int(args.threads))
    config.merger.parallel_threshold = 16
    config.merger.candidate_batch_size = 64
    config.merger.score_threshold = 32.0

    config.enable_connector = bool(args.global_connector)
    config.connector.n_threads = max(1, int(args.threads))
    config.connector.parallel_threshold = 1
    config.connector.pair_batch_size = max(1, int(args.task_batch_size))
    config.connector.max_pairs_per_gap = int(args.connector_pairs)
    config.connector.max_total_bridge_boxes = int(args.connector_bridge_boxes)
    config.connector.per_pair_timeout_ms = 800.0
    config.connector.point_validated_gap_tolerance = float(args.gap_connect_tol)
    config.connector.point_validated_gap_resolution = 24
    config.connector.rrt.max_iters = int(args.rrt_iters)
    config.connector.rrt.timeout_ms = 800.0
    config.connector.rrt.step_size = 0.22
    config.connector.rrt.goal_bias = 0.25
    config.connector.rrt.segment_resolution = 12
    config.connector.pave.max_chain = int(args.connector_bridge_boxes)
    config.connector.pave.max_steps_per_waypoint = 64
    config.connector.pave.find_free_box.max_depth = int(args.ffb_depth)
    config.connector.pave.find_free_box.deadline_ms = 0.0
    config.connector.pave.find_free_box.split_reserved_leaf = True
    config.connector.pave.find_free_box.split_unknown_leaf = True
    config.connector.pave.find_free_box.reject_seed_collision = bool(args.reject_seed_collision)

    config.query.nearest_if_outside = True
    config.query.shortcut_boxes = True
    return config


def make_obstacles(scene: str):
    if scene == "combined":
        return make_combined_obstacles()
    if scene == "shelves":
        return make_shelves_obstacles()
    if scene == "bins":
        return make_bins_obstacles()
    if scene == "table":
        return make_table_obstacles()
    raise ValueError(f"unknown scene: {scene}")


def profile_to_dict(profile: sbf.BuildProfile) -> dict[str, Any]:
    return {
        "total_ms": profile.total_ms,
        "grow_ms": profile.grow_ms,
        "merge_ms": profile.merge_ms,
        "connector_ms": profile.connector_ms,
        "adjacency_ms": profile.adjacency_ms,
        "raw_boxes": profile.raw_boxes,
        "final_boxes": profile.final_boxes,
        "bridge_boxes_added": profile.bridge_boxes_added,
        "connector_attempted_pairs": profile.connector_attempted_pairs,
        "connector_connected": profile.connector_connected,
        "adjacency_islands": profile.adjacency_islands,
        "diagnostics": dict(profile.diagnostics),
    }


def query_to_dict(index: int, label: str, result: sbf.QueryResult) -> dict[str, Any]:
    return {
        "seed": 0,
        "pair_idx": index,
        "label": label,
        "success": bool(result.success),
        "start_box_id": int(result.start_box_id),
        "goal_box_id": int(result.goal_box_id),
        "box_sequence": [int(v) for v in result.box_sequence],
        "waypoints": [[float(x) for x in waypoint] for waypoint in result.path],
        "path_length": float(result.path_length),
        "query_time_ms": float(result.query_time_ms),
    }


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    robot = load_iiwa14_robot()
    obstacles = make_obstacles(args.scene)
    queries = make_combined_queries()
    if args.query != "all":
        queries = [query for query in queries if query.label == args.query]
        if not queries:
            raise ValueError(f"unknown query label: {args.query}")

    config = configure_sbf(args)
    forest = sbf.SafeBoxForest(robot, config)
    if args.seed_set == "query":
        seeds = []
        seen: set[tuple[float, ...]] = set()
        for query in queries:
            for seed in (query.start, query.goal):
                if seed not in seen:
                    seen.add(seed)
                    seeds.append(list(seed))
    else:
        seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=args.seed_set == "extended")]

    wall_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    build_wall_ms = (time.perf_counter() - wall_t0) * 1000.0

    query_entries: list[dict[str, Any]] = []
    prebridge_entries: list[dict[str, Any]] = []
    for index, query in enumerate(queries):
        result = forest.query(list(query.start), list(query.goal))
        if args.prebridge_query and not result.success:
            for round_idx in range(max(1, int(args.prebridge_rounds))):
                bridge_t0 = time.perf_counter()
                added = int(forest.bridge_query(list(query.start), list(query.goal)))
                bridge_ms = (time.perf_counter() - bridge_t0) * 1000.0
                result = forest.query(list(query.start), list(query.goal))
                prebridge_entries.append({
                    "label": query.label,
                    "round": round_idx,
                    "added_boxes": added,
                    "time_ms": bridge_ms,
                    "success_after": bool(result.success),
                })
                if result.success or added <= 0:
                    break
        query_entries.append(query_to_dict(index, query.label, result))

    successful = [entry for entry in query_entries if entry["success"]]
    timing = {
        "build_profile": profile_to_dict(profile),
        "build_wall_ms": build_wall_ms,
        "n_obstacles": len(obstacles),
        "scene": args.scene,
        "endpoint_source": args.endpoint_source,
        "envelope": args.envelope,
        "n_queries": len(query_entries),
        "n_success": len(successful),
        "threads": int(args.threads),
        "max_boxes": int(args.max_boxes),
        "ffb_depth": int(args.ffb_depth),
        "gap_connect_tol": float(args.gap_connect_tol),
        "global_connector": bool(args.global_connector),
        "prebridge_query": bool(args.prebridge_query),
        "prebridge_rounds": int(args.prebridge_rounds),
        "prebridge": prebridge_entries,
        "seed_set": args.seed_set,
        "n_seeds": len(seeds),
    }
    payload = {"timing": timing, "paths": query_entries}
    write_json(args.out_dir / "marcucci_shelf_sbf_run.json", payload)
    write_json(args.out_dir / "paths.json", {"paths": query_entries})

    visualization: dict[str, Any] = {}
    if successful and args.save_html:
        from sbf.drake_visualization import visualize_paths

        html_path = args.out_dir / "marcucci_shelf_sbf_path.html"
        visualization = visualize_paths(
            [entry["waypoints"] for entry in successful],
            [entry["label"].replace("->", "_to_") for entry in successful],
            gcs_repo=args.gcs_repo,
            save_html=html_path,
            static=not args.animate,
            no_show=not args.show,
        )
        timing["visualization"] = visualization
        write_json(args.out_dir / "marcucci_shelf_sbf_run.json", payload)

    print(json.dumps({"timing": timing, "queries": query_entries}, indent=2, sort_keys=True))
    if len(successful) != len(query_entries) and not args.allow_failures:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
