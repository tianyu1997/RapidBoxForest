#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common_sbf_config import (  # noqa: E402
    ROOT,
    add_common_sbf_args,
    box_volume_sum,
    configure_standalone_sbf,
    count_status,
    query_result_payload,
    sbf,
    write_json,
)
from collision_refine import RefineConfig, refine_path_rows  # noqa: E402
from sbf.marcucci import (  # noqa: E402
    make_bins_obstacles,
    make_combined_obstacles,
    make_combined_queries,
    make_coverage_seeds,
    make_shelves_obstacles,
    make_table_obstacles,
    load_iiwa14_robot,
)


SCENE_BUILDERS = {
    "shelves": make_shelves_obstacles,
    "bins": make_bins_obstacles,
    "table": make_table_obstacles,
    "combined": make_combined_obstacles,
    "marcucci": make_combined_obstacles,
    "marcucci_combined": make_combined_obstacles,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export standalone SBF Marcucci planned paths and render them in Drake/Meshcat."
    )
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset="crit_link_coverage",
        threads=4,
        task_batch_size=8,
        max_boxes=800,
        ffb_depth=90,
        max_consecutive_miss=8000,
        audit_resolution=48,
        repair_max_attempts=4,
        repair_rrt_max_iters=10000,
        repair_timeout_ms=1200.0,
    )
    parser.add_argument("--target-scene", choices=sorted(SCENE_BUILDERS), default="marcucci")
    parser.add_argument("--query", default="all", help="query label to export, or 'all'")
    parser.add_argument("--grid-pad-policy", choices=["strict_half_diagonal", "no_extra_pad"], default="strict_half_diagonal")
    parser.add_argument("--storage-profile", choices=["compact", "balanced", "fast_query"], default="fast_query")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_planned_paths.json")
    parser.add_argument("--out-paths-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_planned_paths_only.json")
    parser.add_argument("--out-html", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_planned_paths.html")
    parser.add_argument("--gcs-repo", type=Path, default=None)
    parser.add_argument("--animate", action="store_true", help="record a robot animation instead of static end-effector traces")
    parser.add_argument("--show", action="store_true", help="keep the Meshcat server open after exporting")
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--no-html", action="store_true", help="export JSON only")
    parser.add_argument("--allow-no-success", action="store_true")
    parser.add_argument("--rrt-goal-bias", type=float, default=0.2)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.25)
    parser.add_argument("--unexplored-prob", type=float, default=0.45)
    parser.add_argument("--step-ratio", type=float, default=0.08)
    parser.add_argument("--component-connect-prob", type=float, default=0.45)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=4)
    parser.add_argument("--component-connect-stage-normalized-linf", type=float, default=0.35)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=40)
    parser.add_argument("--component-connect-ffb-max-depth", type=int, default=140)
    parser.add_argument("--coverage-first-stop-loss", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hard-frontier-failure-threshold", type=int, default=1)
    parser.add_argument("--hard-frontier-box-horizon", type=int, default=300)
    parser.add_argument("--stop-after-connect", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--post-connect-extra-boxes", type=int, default=0)
    parser.add_argument("--frontier-bridge", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-bridge-boxes", type=int, default=0)
    parser.add_argument("--connector-pair-batch-size", type=int, default=8)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=800.0)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=12)
    parser.add_argument("--connector-pave-max-chain", type=int, default=0)
    parser.add_argument("--connector-pave-steps", type=int, default=18)
    parser.add_argument("--connector-pave-depth", type=int, default=140)
    parser.add_argument("--connector-point-gap-tolerance", type=float, default=0.0)
    parser.add_argument("--connector-point-gap-resolution", type=int, default=24)
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prebridge-rounds", type=int, default=5)
    parser.add_argument("--postprocess-paths", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--postprocess-rrt-candidates", type=int, default=20)
    parser.add_argument("--postprocess-workers", type=int, default=8)
    parser.add_argument("--postprocess-seed-base", type=int, default=1000)
    parser.add_argument("--postprocess-rrt-iters", type=int, default=26000)
    parser.add_argument("--postprocess-rrt-timeout-ms", type=float, default=250.0)
    parser.add_argument("--postprocess-rrt-step-size", type=float, default=0.11)
    parser.add_argument("--postprocess-rrt-goal-bias", type=float, default=0.25)
    parser.add_argument("--postprocess-segment-resolution", type=int, default=16)
    parser.add_argument("--postprocess-smooth-time-ms", type=float, default=45.0)
    parser.add_argument("--postprocess-segment-step", type=float, default=0.04)
    parser.add_argument("--postprocess-random-samples", type=int, default=10)
    return parser.parse_args()


def refine_config_from_args(args: argparse.Namespace) -> RefineConfig:
    return RefineConfig(
        enabled=bool(args.postprocess_paths),
        candidates=int(args.postprocess_rrt_candidates),
        workers=int(args.postprocess_workers),
        seed_base=int(args.postprocess_seed_base),
        rrt_max_iters=int(args.postprocess_rrt_iters),
        rrt_timeout_ms=float(args.postprocess_rrt_timeout_ms),
        rrt_step_size=float(args.postprocess_rrt_step_size),
        rrt_goal_bias=float(args.postprocess_rrt_goal_bias),
        rrt_segment_resolution=int(args.postprocess_segment_resolution),
        smooth_time_ms=float(args.postprocess_smooth_time_ms),
        segment_step=float(args.postprocess_segment_step),
        random_samples=int(args.postprocess_random_samples),
    )


def apply_database_profile(cfg: Any, name: str) -> None:
    key = name.strip().lower().replace("-", "_")
    if key == "compact":
        cfg.database.page_size_bytes = 4096
        cfg.database.max_resident_pages = 2048
    elif key == "balanced":
        cfg.database.page_size_bytes = 16384
        cfg.database.max_resident_pages = 8192
    elif key in {"fast", "fast_query", "fastquery"}:
        cfg.database.page_size_bytes = 65536
        cfg.database.max_resident_pages = 32768
    else:
        raise ValueError(f"unknown database profile {name!r}")


def apply_visualization_protocol(cfg: Any, args: argparse.Namespace) -> None:
    apply_database_profile(cfg, args.storage_profile)
    cfg.grower.root_seed_max_lca_depth = -1
    cfg.grower.rrt_goal_bias = float(args.rrt_goal_bias)
    cfg.grower.intertree_goal_bias = float(args.intertree_goal_bias)
    cfg.grower.sustained_goal_bias_cap = min(0.25, float(args.intertree_goal_bias))
    cfg.grower.rrt_step_ratio = float(args.step_ratio)
    cfg.grower.unexplored_sample_prob = float(args.unexplored_prob)
    cfg.grower.component_connect_prob = float(args.component_connect_prob)
    cfg.grower.component_connect_candidate_limit = int(args.component_connect_candidate_limit)
    cfg.grower.component_connect_stage_normalized_linf = float(args.component_connect_stage_normalized_linf)
    cfg.grower.component_connect_adaptive_ffb = True
    cfg.grower.component_connect_ffb_depth_increment = int(args.component_connect_ffb_depth_increment)
    cfg.grower.component_connect_ffb_max_depth = int(args.component_connect_ffb_max_depth)
    cfg.grower.coverage_first_stop_loss = bool(args.coverage_first_stop_loss)
    cfg.grower.hard_frontier_failure_threshold = int(args.hard_frontier_failure_threshold)
    cfg.grower.hard_frontier_box_horizon = int(args.hard_frontier_box_horizon)
    cfg.grower.stop_after_connect = bool(args.stop_after_connect)
    cfg.grower.post_connect_extra_boxes = int(args.post_connect_extra_boxes)

    cfg.connector.frontier_bridge = bool(args.frontier_bridge)
    cfg.connector.max_total_bridge_boxes = int(args.connector_bridge_boxes)
    cfg.connector.segment_edges_enabled = True
    cfg.connector.rrt_segment_edges = True
    cfg.connector.point_gap_segment_edges = True
    cfg.connector.pair_batch_size = max(1, int(args.connector_pair_batch_size))
    cfg.connector.per_pair_timeout_ms = float(args.connector_pair_timeout_ms)
    cfg.connector.max_pairs_per_gap = int(args.connector_max_pairs_per_gap)
    cfg.connector.point_validated_gap_tolerance = float(args.connector_point_gap_tolerance)
    cfg.connector.point_validated_gap_resolution = int(args.connector_point_gap_resolution)
    cfg.connector.pave.max_chain = int(args.connector_pave_max_chain)
    cfg.connector.pave.max_steps_per_waypoint = int(args.connector_pave_steps)
    cfg.connector.pave.find_free_box.max_depth = int(args.connector_pave_depth)
    cfg.connector.pave.find_free_box.split_reserved_leaf = True
    cfg.connector.pave.find_free_box.split_unknown_leaf = True
    cfg.connector.pave.find_free_box.reject_seed_collision = False
def make_obstacles(scene: str) -> list[Any]:
    try:
        return SCENE_BUILDERS[scene]()
    except KeyError as exc:
        raise ValueError(f"unknown scene {scene!r}; choices={sorted(SCENE_BUILDERS)}") from exc


def profile_payload(profile: Any) -> dict[str, Any]:
    return {
        "total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "bridge_boxes_added": int(getattr(profile, "bridge_boxes_added", 0)),
        "segment_edges": int(getattr(profile, "segment_edges", 0)),
        "segment_edges_added": int(getattr(profile, "segment_edges_added", 0)),
        "rrt_segment_edges_added": int(getattr(profile, "rrt_segment_edges_added", 0)),
        "point_gap_segment_edges_added": int(getattr(profile, "point_gap_segment_edges_added", 0)),
        "connector_attempted_pairs": int(getattr(profile, "connector_attempted_pairs", 0)),
        "connector_connected": bool(getattr(profile, "connector_connected", False)),
        "adjacency_islands": int(profile.adjacency_islands),
        "diagnostics": {str(key): float(value) for key, value in dict(profile.diagnostics).items()},
    }


def path_entry(index: int, query: Any, result: Any, wall_s: float) -> dict[str, Any]:
    entry = query_result_payload(query.label, result, wall_s)
    entry.update(
        {
            "pair_idx": int(index),
            "start": [float(value) for value in query.start],
            "goal": [float(value) for value in query.goal],
            "start_box_id": int(result.start_box_id),
            "goal_box_id": int(result.goal_box_id),
            "box_sequence": [int(value) for value in result.box_sequence],
            "waypoints": [[float(value) for value in waypoint] for waypoint in result.path],
            "raw_path_length": float(result.path_length),
            "query_time_ms_internal": float(result.query_time_ms),
            "failed_segment_index": int(result.failed_segment_index),
            "certified_box_length": float(result.certified_box_length),
            "provisional_audited_length": float(result.provisional_audited_length),
            "segment_edge_length": float(result.segment_edge_length),
            "box_sequence_count": len(result.box_sequence),
            "waypoint_count": len(result.path),
        }
    )
    return entry


def query_with_optional_bridges(forest: Any, query: Any, args: argparse.Namespace) -> tuple[Any, dict[str, Any]]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    initial_query_s = time.perf_counter() - query_t0
    bridge_rounds: list[dict[str, Any]] = []
    bridge_total_s = 0.0
    retry_total_s = 0.0
    bridge_progress_total = 0

    if args.bridge_failed_queries and not result.success:
        for round_index in range(max(0, int(args.prebridge_rounds))):
            bridge_t0 = time.perf_counter()
            progress = int(forest.bridge_query(list(query.start), list(query.goal)))
            bridge_s = time.perf_counter() - bridge_t0
            bridge_total_s += bridge_s
            bridge_progress_total += progress

            retry_t0 = time.perf_counter()
            retry = forest.query(list(query.start), list(query.goal))
            retry_s = time.perf_counter() - retry_t0
            retry_total_s += retry_s
            result = retry
            bridge_rounds.append(
                {
                    "round": int(round_index + 1),
                    "progress": int(progress),
                    "bridge_time_s": float(bridge_s),
                    "retry_query_s": float(retry_s),
                    "ok_after_retry": bool(retry.success),
                    "audit_status_after_retry": str(retry.audit_status).split(".")[-1],
                    "length_after_retry": float(retry.path_length) if retry.success else 0.0,
                    "boxes_after_retry": len(forest.boxes()),
                    "segment_edges_after_retry": len(forest.segment_edges()),
                }
            )
            if retry.success:
                break
            if progress <= 0:
                break

    timings = {
        "initial_query_s": float(initial_query_s),
        "bridge_round_count": len(bridge_rounds),
        "bridge_progress_total": int(bridge_progress_total),
        "bridge_time_s": float(bridge_total_s),
        "retry_query_s": float(retry_total_s),
        "total_query_s": float(initial_query_s + bridge_total_s + retry_total_s),
        "bridge_rounds": bridge_rounds,
        "boxes_after_query": len(forest.boxes()),
        "segment_edges_after_query": len(forest.segment_edges()),
    }
    return result, timings


def main() -> int:
    args = parse_args()
    robot = load_iiwa14_robot()
    obstacles = make_obstacles(args.target_scene)
    queries = make_combined_queries()
    if args.query != "all":
        queries = [query for query in queries if query.label == args.query]
        if not queries:
            raise ValueError(f"unknown query label {args.query!r}")

    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    cfg = configure_standalone_sbf(args, seed=0, preset=args.preset)
    apply_visualization_protocol(cfg, args)
    forest = sbf.SafeBoxForest(robot, cfg)

    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    build_s = time.perf_counter() - build_t0
    boxes = forest.boxes()

    path_rows: list[dict[str, Any]] = []
    query_loop_t0 = time.perf_counter()
    for index, query in enumerate(queries):
        result, timings = query_with_optional_bridges(forest, query, args)
        row = path_entry(index, query, result, timings["total_query_s"])
        row.update(timings)
        path_rows.append(row)
    query_loop_s = time.perf_counter() - query_loop_t0

    postprocess = refine_path_rows(path_rows, args.target_scene, refine_config_from_args(args))

    successful = [row for row in path_rows if bool(row["ok"]) and len(row["waypoints"]) >= 2]
    payload: dict[str, Any] = {
        "experiment": "exp3_marcucci_planned_paths_drake",
        "params": {
            "preset": args.preset,
            "target_scene": args.target_scene,
            "query": args.query,
            "threads": int(args.threads),
            "task_batch_size": int(args.task_batch_size),
            "max_boxes": int(args.max_boxes),
            "timeout_ms": float(args.timeout_ms),
            "ffb_depth": int(args.ffb_depth),
            "grid_pad_policy": args.grid_pad_policy,
            "strict_path_audit": bool(args.strict_path_audit),
            "audit_resolution": int(args.audit_resolution),
            "repair_on_audit_failure": bool(args.repair_on_audit_failure),
            "repair_max_attempts": int(args.repair_max_attempts),
            "repair_rrt_max_iters": int(args.repair_rrt_max_iters),
            "repair_timeout_ms": float(args.repair_timeout_ms),
            "rrt_goal_bias": float(args.rrt_goal_bias),
            "intertree_goal_bias": float(args.intertree_goal_bias),
            "unexplored_prob": float(args.unexplored_prob),
            "step_ratio": float(args.step_ratio),
            "component_connect_prob": float(args.component_connect_prob),
            "component_connect_candidate_limit": int(args.component_connect_candidate_limit),
            "component_connect_stage_normalized_linf": float(args.component_connect_stage_normalized_linf),
            "component_connect_ffb_depth_increment": int(args.component_connect_ffb_depth_increment),
            "component_connect_ffb_max_depth": int(args.component_connect_ffb_max_depth),
            "coverage_first_stop_loss": bool(args.coverage_first_stop_loss),
            "hard_frontier_failure_threshold": int(args.hard_frontier_failure_threshold),
            "hard_frontier_box_horizon": int(args.hard_frontier_box_horizon),
            "stop_after_connect": bool(args.stop_after_connect),
            "post_connect_extra_boxes": int(args.post_connect_extra_boxes),
            "frontier_bridge": bool(args.frontier_bridge),
            "connector_bridge_boxes": int(args.connector_bridge_boxes),
            "connector_pair_batch_size": int(args.connector_pair_batch_size),
            "connector_pair_timeout_ms": float(args.connector_pair_timeout_ms),
            "connector_max_pairs_per_gap": int(args.connector_max_pairs_per_gap),
            "connector_rrt_iters": int(args.connector_rrt_iters),
            "connector_rrt_timeout_ms": float(args.connector_rrt_timeout_ms),
            "connector_rrt_step_size": float(args.connector_rrt_step_size),
            "connector_rrt_goal_bias": float(args.connector_rrt_goal_bias),
            "connector_segment_resolution": int(args.connector_segment_resolution),
            "connector_pave_max_chain": int(args.connector_pave_max_chain),
            "connector_pave_steps": int(args.connector_pave_steps),
            "connector_pave_depth": int(args.connector_pave_depth),
            "connector_point_gap_tolerance": float(args.connector_point_gap_tolerance),
            "connector_point_gap_resolution": int(args.connector_point_gap_resolution),
            "bridge_failed_queries": bool(args.bridge_failed_queries),
            "prebridge_rounds": int(args.prebridge_rounds),
            "postprocess_paths": bool(args.postprocess_paths),
            "postprocess_rrt_candidates": int(args.postprocess_rrt_candidates),
            "postprocess_workers": int(args.postprocess_workers),
            "postprocess_seed_base": int(args.postprocess_seed_base),
            "postprocess_rrt_iters": int(args.postprocess_rrt_iters),
            "postprocess_rrt_timeout_ms": float(args.postprocess_rrt_timeout_ms),
            "postprocess_rrt_step_size": float(args.postprocess_rrt_step_size),
            "postprocess_rrt_goal_bias": float(args.postprocess_rrt_goal_bias),
            "postprocess_segment_resolution": int(args.postprocess_segment_resolution),
            "postprocess_smooth_time_ms": float(args.postprocess_smooth_time_ms),
            "postprocess_segment_step": float(args.postprocess_segment_step),
            "postprocess_random_samples": int(args.postprocess_random_samples),
        },
        "build": {
            "wall_s": float(build_s),
            "n_obstacles": len(obstacles),
            "n_seeds": len(seeds),
            "n_boxes": len(boxes),
            "certified_box_count": count_status(boxes, sbf.BoxSafetyStatus.CertifiedFree),
            "provisional_box_count": count_status(boxes, sbf.BoxSafetyStatus.ProvisionalFree),
            "box_volume_sum": box_volume_sum(boxes),
            "segment_edge_count": len(forest.segment_edges()),
            "profile": profile_payload(profile),
        },
        "timing": {
            "build_wall_s": float(build_s),
            "query_loop_wall_s": float(query_loop_s),
            "postprocess_wall_s": float(postprocess.get("wall_s", 0.0)),
            "total_planning_wall_s": float(build_s + query_loop_s + float(postprocess.get("wall_s", 0.0))),
        },
        "postprocess": postprocess,
        "queries": path_rows,
        "successful_path_count": len(successful),
    }
    write_json(args.out_json, payload)
    write_json(args.out_paths_json, {"paths": path_rows})

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
        write_json(args.out_json, payload)

    print(json.dumps({"out_json": str(args.out_json), "out_html": str(args.out_html), "successful_path_count": len(successful)}, indent=2))
    if not successful and not args.allow_no_success:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())