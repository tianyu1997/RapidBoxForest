#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable


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
    ANCHORS,
    make_bins_obstacles,
    make_combined_obstacles,
    make_combined_queries,
    make_shelves_obstacles,
    make_table_obstacles,
    load_iiwa14_robot,
)


SceneFactory = Callable[[], list[sbf.Obstacle]]
DEFAULT_DEPTH_STAGE_BOX_LIMITS = [400, 1400, 0]


def _shelf_subset(*indices: int) -> list[sbf.Obstacle]:
    shelves = make_shelves_obstacles()
    return [shelves[index] for index in indices]


SCENES: dict[str, SceneFactory] = {
    "empty": lambda: [],
    "table": make_table_obstacles,
    "shelf_top_board": lambda: _shelf_subset(2),
    "shelf_bottom_board": lambda: _shelf_subset(3),
    "shelf_mid_board": lambda: _shelf_subset(4),
    "shelf_top_mid": lambda: _shelf_subset(2, 4),
    "shelf_bottom_mid": lambda: _shelf_subset(3, 4),
    "shelf_horizontal": lambda: _shelf_subset(2, 3, 4),
    "shelf_sides": lambda: _shelf_subset(0, 1),
    "shelf_open_left": lambda: _shelf_subset(1, 2, 3, 4),
    "shelf_open_right": lambda: _shelf_subset(0, 2, 3, 4),
    "shelves": make_shelves_obstacles,
    "bins": make_bins_obstacles,
    "combined": make_combined_obstacles,
}


def parse_depth_schedule(text: str) -> list[sbf.GrowerDepthStage]:
    stages: list[sbf.GrowerDepthStage] = []
    if not text:
        return stages
    raw_items = [item.strip() for item in text.split(",") if item.strip()]
    if len(raw_items) == 1:
        parts = raw_items[0].split(":")
        if len(parts) not in (2, 4):
            raw_items = parts
    if all(":" not in item for item in raw_items):
        for index, item in enumerate(raw_items):
            stage = sbf.GrowerDepthStage()
            stage.ffb_depth = int(item)
            stage.box_limit = DEFAULT_DEPTH_STAGE_BOX_LIMITS[index] if index < len(DEFAULT_DEPTH_STAGE_BOX_LIMITS) else 0
            stages.append(stage)
        return stages
    for item in raw_items:
        parts = item.split(":")
        if len(parts) not in (2, 4):
            raise ValueError("depth schedule entries must be depth:box_limit or depth:box_limit:component_increment:component_max")
        stage = sbf.GrowerDepthStage()
        stage.ffb_depth = int(parts[0])
        stage.box_limit = int(parts[1])
        if len(parts) == 4:
            stage.component_connect_ffb_depth_increment = int(parts[2])
            stage.component_connect_ffb_max_depth = int(parts[3])
        stages.append(stage)
    return stages


def depth_schedule_to_payload(stages: list[sbf.GrowerDepthStage]) -> list[dict[str, int]]:
    return [
        {
            "ffb_depth": int(stage.ffb_depth),
            "box_limit": int(stage.box_limit),
            "component_connect_ffb_depth_increment": int(stage.component_connect_ffb_depth_increment),
            "component_connect_ffb_max_depth": int(stage.component_connect_ffb_max_depth),
        }
        for stage in stages
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Grower-only SBF staircase diagnostics on simple-to-shelf scenes.")
    parser.add_argument("--scenes", nargs="+", default=["empty", "table", "shelf_mid_board", "shelf_horizontal", "shelves", "combined"])
    parser.add_argument("--query", default="AS->TS")
    parser.add_argument("--endpoint-source", choices=["ifk", "critsample"], default="critsample")
    parser.add_argument("--grower", choices=["rrt", "frontwave"], default="rrt")
    parser.add_argument("--seed-set", choices=["query", "canonical"], default="query")
    parser.add_argument("--max-boxes", type=int, default=1500)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--ffb-depth", type=int, default=80)
    parser.add_argument("--depth-schedule", default="50:80:120", help="RRT depth stages. Use depth:box_limit entries, or shorthand depths like 50:80:120.")
    parser.add_argument("--split-policy", choices=["best-tighten", "widest", "widest-first"], default="best-tighten")
    parser.add_argument("--best-tighten-depth-synchronous", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-prefer-sector-boundary", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-use-minimax", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-width-penalty", type=float, default=0.0)
    parser.add_argument("--best-tighten-shape-balancing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-max-child-aspect", type=float, default=64.0)
    parser.add_argument("--best-tighten-min-split-width-fraction", type=float, default=0.05)
    parser.add_argument("--best-tighten-shape-weight", type=float, default=0.25)
    parser.add_argument("--best-tighten-balance-weight", type=float, default=0.05)
    parser.add_argument("--best-tighten-relative-gain-weight", type=float, default=0.10)
    parser.add_argument("--best-tighten-widest-tiebreak-weight", type=float, default=0.02)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--task-batch-size", type=int, default=1)
    parser.add_argument("--rrt-goal-bias", type=float, default=0.2)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.2)
    parser.add_argument("--sustained-goal-bias-cap", type=float, default=0.25)
    parser.add_argument("--high-goal-bias-pulse-period", type=int, default=8)
    parser.add_argument("--expand-all-roots-per-sample", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-connector", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--frontier-bridge", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--frontier-bridge-adaptive-ffb", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--frontier-bridge-gap-stall-iterations", type=int, default=3)
    parser.add_argument("--frontier-bridge-ffb-depth-increment", type=int, default=12)
    parser.add_argument("--frontier-bridge-ffb-max-depth", type=int, default=120)
    parser.add_argument("--frontier-bridge-candidate-limit", type=int, default=96)
    parser.add_argument("--connector-bridge-boxes", type=int, default=300)
    parser.add_argument("--extra-random-roots", type=int, default=0)
    parser.add_argument("--root-seed-candidate-count", type=int, default=128)
    parser.add_argument("--root-seed-min-normalized-linf", type=float, default=0.25)
    parser.add_argument("--root-seed-max-lca-depth", type=int, default=2)
    parser.add_argument("--component-connect-prob", type=float, default=0.55)
    parser.add_argument("--component-connect-max-parent-failures", type=int, default=8)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=4)
    parser.add_argument("--component-connect-adaptive-ffb", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=40)
    parser.add_argument("--component-connect-ffb-max-depth", type=int, default=120)
    parser.add_argument("--failure-cooling", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--failure-cooling-threshold", type=int, default=3)
    parser.add_argument("--failure-cooling-box-horizon", type=int, default=200)
    parser.add_argument("--failure-cooling-min-depth", type=int, default=0)
    parser.add_argument("--failure-cooling-unknown-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--failure-cooling-retry-on-depth-raise", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--unexplored-prob", type=float, default=0.45)
    parser.add_argument("--step-ratio", type=float, default=0.08)
    parser.add_argument("--stop-after-connect", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "grower_staircase" / "grower_staircase.json")
    return parser.parse_args()


def configure(args: argparse.Namespace) -> sbf.SBFConfig:
    config = sbf.SBFConfig()
    config.enable_merger = False
    config.enable_connector = bool(args.enable_connector)
    config.endpoint_source.source = sbf.EndpointSource.IFK if args.endpoint_source == "ifk" else sbf.EndpointSource.CritSample
    config.envelope_type.type = sbf.EnvelopeType.LinkIAABB
    config.envelope_type.n_subdivisions = 4

    config.runtime.mode = sbf.ExecutionMode.Parallel if args.threads > 1 else sbf.ExecutionMode.Inline
    config.runtime.n_threads = max(1, int(args.threads))
    config.runtime.batch_size = max(1, int(args.task_batch_size))
    config.runtime.parallel_threshold = 1

    config.grower.mode = sbf.GrowerMode.Frontwave if args.grower == "frontwave" else sbf.GrowerMode.RRT
    config.grower.max_boxes = int(args.max_boxes)
    config.grower.timeout_ms = float(args.timeout_ms)
    config.grower.max_consecutive_miss = 4000
    config.grower.rng_seed = 20260503
    config.grower.n_threads = max(1, int(args.threads))
    config.grower.task_batch_size = max(1, int(args.task_batch_size))
    config.grower.parallel_threshold = 1
    config.grower.worker_local_ffb = True
    config.grower.rrt_goal_bias = float(args.rrt_goal_bias)
    config.grower.intertree_goal_bias = float(args.intertree_goal_bias)
    config.grower.sustained_goal_bias_cap = float(args.sustained_goal_bias_cap)
    config.grower.high_goal_bias_pulse_period = int(args.high_goal_bias_pulse_period)
    config.grower.expand_all_roots_per_sample = bool(args.expand_all_roots_per_sample)
    config.grower.extra_random_roots = int(args.extra_random_roots)
    config.grower.root_seed_candidate_count = int(args.root_seed_candidate_count)
    config.grower.root_seed_min_normalized_linf = float(args.root_seed_min_normalized_linf)
    config.grower.root_seed_max_lca_depth = int(args.root_seed_max_lca_depth)
    config.grower.root_seed_include_user_seeds = True
    config.grower.component_connect_prob = float(args.component_connect_prob)
    config.grower.component_connect_max_parent_failures = int(args.component_connect_max_parent_failures)
    config.grower.component_connect_candidate_limit = int(args.component_connect_candidate_limit)
    config.grower.component_connect_adaptive_ffb = bool(args.component_connect_adaptive_ffb)
    config.grower.component_connect_ffb_depth_increment = int(args.component_connect_ffb_depth_increment)
    config.grower.component_connect_ffb_max_depth = int(args.component_connect_ffb_max_depth)
    config.grower.rrt_step_ratio = float(args.step_ratio)
    config.grower.unexplored_sample_prob = float(args.unexplored_prob)
    config.grower.connect_mode = True
    config.grower.stop_after_connect = bool(args.stop_after_connect)
    config.grower.post_connect_extra_boxes = 0
    config.grower.find_free_box.max_depth = int(args.ffb_depth)
    config.grower.find_free_box.deadline_ms = 0.0
    config.grower.find_free_box.split_reserved_leaf = True
    config.grower.find_free_box.split_unknown_leaf = True
    config.grower.find_free_box.reject_seed_collision = False
    config.grower.find_free_box.split.use_best_tighten = args.split_policy == "best-tighten"
    config.grower.find_free_box.split.best_tighten.depth_synchronous = bool(args.best_tighten_depth_synchronous)
    config.grower.find_free_box.split.best_tighten.prefer_sector_boundary = bool(args.best_tighten_prefer_sector_boundary)
    config.grower.find_free_box.split.best_tighten.use_minimax = bool(args.best_tighten_use_minimax)
    config.grower.find_free_box.split.best_tighten.width_penalty = float(args.best_tighten_width_penalty)
    config.grower.find_free_box.split.best_tighten.shape_balancing = bool(args.best_tighten_shape_balancing)
    config.grower.find_free_box.split.best_tighten.max_child_aspect = float(args.best_tighten_max_child_aspect)
    config.grower.find_free_box.split.best_tighten.min_split_width_fraction = float(args.best_tighten_min_split_width_fraction)
    config.grower.find_free_box.split.best_tighten.shape_weight = float(args.best_tighten_shape_weight)
    config.grower.find_free_box.split.best_tighten.balance_weight = float(args.best_tighten_balance_weight)
    config.grower.find_free_box.split.best_tighten.relative_gain_weight = float(args.best_tighten_relative_gain_weight)
    config.grower.find_free_box.split.best_tighten.widest_tiebreak_weight = float(args.best_tighten_widest_tiebreak_weight)
    config.grower.depth_stages = parse_depth_schedule(args.depth_schedule)
    config.grower.failure_cooling_enabled = bool(args.failure_cooling)
    config.grower.failure_cooling_threshold = int(args.failure_cooling_threshold)
    config.grower.failure_cooling_box_horizon = int(args.failure_cooling_box_horizon)
    config.grower.failure_cooling_min_depth = int(args.failure_cooling_min_depth)
    config.grower.failure_cooling_unknown_only = bool(args.failure_cooling_unknown_only)
    config.grower.failure_cooling_retry_on_depth_raise = bool(args.failure_cooling_retry_on_depth_raise)
    config.grower.n_boundary_samples = 8
    config.grower.goal_face_bias = 0.75

    config.connector.n_threads = max(1, int(args.threads))
    config.connector.pair_batch_size = max(1, int(args.task_batch_size))
    config.connector.parallel_threshold = 1
    config.connector.max_total_bridge_boxes = int(args.connector_bridge_boxes)
    config.connector.frontier_bridge = bool(args.frontier_bridge)
    config.connector.frontier_bridge_adaptive_ffb = bool(args.frontier_bridge_adaptive_ffb)
    config.connector.frontier_bridge_gap_stall_iterations = int(args.frontier_bridge_gap_stall_iterations)
    config.connector.frontier_bridge_ffb_depth_increment = int(args.frontier_bridge_ffb_depth_increment)
    config.connector.frontier_bridge_ffb_max_depth = int(args.frontier_bridge_ffb_max_depth)
    config.connector.frontier_bridge_candidate_limit = int(args.frontier_bridge_candidate_limit)
    config.connector.pave.max_chain = int(args.connector_bridge_boxes)
    config.connector.pave.find_free_box.max_depth = int(args.ffb_depth)
    config.connector.pave.find_free_box.deadline_ms = 0.0
    config.connector.pave.find_free_box.split_reserved_leaf = True
    config.connector.pave.find_free_box.split_unknown_leaf = True
    config.connector.pave.find_free_box.reject_seed_collision = False
    config.connector.pave.find_free_box.split.use_best_tighten = args.split_policy == "best-tighten"
    config.connector.pave.find_free_box.split.best_tighten.depth_synchronous = bool(args.best_tighten_depth_synchronous)
    config.connector.pave.find_free_box.split.best_tighten.prefer_sector_boundary = bool(args.best_tighten_prefer_sector_boundary)
    config.connector.pave.find_free_box.split.best_tighten.use_minimax = bool(args.best_tighten_use_minimax)
    config.connector.pave.find_free_box.split.best_tighten.width_penalty = float(args.best_tighten_width_penalty)
    config.connector.pave.find_free_box.split.best_tighten.shape_balancing = bool(args.best_tighten_shape_balancing)
    config.connector.pave.find_free_box.split.best_tighten.max_child_aspect = float(args.best_tighten_max_child_aspect)
    config.connector.pave.find_free_box.split.best_tighten.min_split_width_fraction = float(args.best_tighten_min_split_width_fraction)
    config.connector.pave.find_free_box.split.best_tighten.shape_weight = float(args.best_tighten_shape_weight)
    config.connector.pave.find_free_box.split.best_tighten.balance_weight = float(args.best_tighten_balance_weight)
    config.connector.pave.find_free_box.split.best_tighten.relative_gain_weight = float(args.best_tighten_relative_gain_weight)
    config.connector.pave.find_free_box.split.best_tighten.widest_tiebreak_weight = float(args.best_tighten_widest_tiebreak_weight)

    config.query.nearest_if_outside = False
    config.query.shortcut_boxes = True
    return config


def query_pair(label: str):
    for pair in make_combined_queries():
        if pair.label == label:
            return pair
    raise ValueError(f"unknown query label: {label}")


def seeds_for(args: argparse.Namespace, pair) -> list[list[float]]:
    if args.seed_set == "query":
        return [list(pair.start), list(pair.goal)]
    names = ["AS", "TS", "CS", "LB", "RB"]
    return [list(ANCHORS[name]) for name in names]


def component_sizes(adjacency: dict[int, list[int]]) -> list[int]:
    unseen = set(int(node) for node in adjacency)
    sizes: list[int] = []
    while unseen:
        root = unseen.pop()
        queue: deque[int] = deque([root])
        size = 0
        while queue:
            current = queue.popleft()
            size += 1
            for neighbor in adjacency.get(current, []):
                neighbor = int(neighbor)
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        sizes.append(size)
    return sorted(sizes, reverse=True)


def box_gap_squared(lhs: sbf.BoxNode, rhs: sbf.BoxNode) -> float:
    gap_sq = 0.0
    for lhs_interval, rhs_interval in zip(lhs.joint_intervals, rhs.joint_intervals):
        gap = 0.0
        if lhs_interval.hi < rhs_interval.lo:
            gap = rhs_interval.lo - lhs_interval.hi
        elif rhs_interval.hi < lhs_interval.lo:
            gap = lhs_interval.lo - rhs_interval.hi
        gap_sq += gap * gap
    return gap_sq


def cross_root_gap(boxes: list[sbf.BoxNode]) -> dict[str, Any]:
    best_gap_sq = float("inf")
    best_pair = [-1, -1]
    for i, lhs in enumerate(boxes):
        lhs_root = int(lhs.root_id)
        for rhs in boxes[i + 1:]:
            if lhs_root == int(rhs.root_id):
                continue
            gap_sq = box_gap_squared(lhs, rhs)
            if gap_sq < best_gap_sq:
                best_gap_sq = gap_sq
                best_pair = [int(lhs.id), int(rhs.id)]
    detail: dict[str, Any] = {
        "gap": best_gap_sq ** 0.5 if best_gap_sq != float("inf") else float("inf"),
        "box_ids": best_pair,
    }
    by_id = {int(box.id): box for box in boxes}
    if best_pair[0] in by_id and best_pair[1] in by_id:
        lhs = by_id[best_pair[0]]
        rhs = by_id[best_pair[1]]
        gap_by_dim: list[float] = []
        for lhs_interval, rhs_interval in zip(lhs.joint_intervals, rhs.joint_intervals):
            if lhs_interval.hi < rhs_interval.lo:
                gap_by_dim.append(float(rhs_interval.lo - lhs_interval.hi))
            elif rhs_interval.hi < lhs_interval.lo:
                gap_by_dim.append(float(lhs_interval.lo - rhs_interval.hi))
            else:
                gap_by_dim.append(0.0)
        detail["roots"] = [int(lhs.root_id), int(rhs.root_id)]
        detail["active_dims"] = [index for index, gap in enumerate(gap_by_dim) if gap > 0.0]
        detail["gap_by_dim"] = gap_by_dim
    return detail


def profile_to_dict(profile: sbf.BuildProfile) -> dict[str, Any]:
    return {
        "total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "bridge_boxes_added": int(profile.bridge_boxes_added),
        "connector_attempted_pairs": int(profile.connector_attempted_pairs),
        "connector_connected": bool(profile.connector_connected),
        "adjacency_islands": int(profile.adjacency_islands),
        "diagnostics": {str(key): float(value) for key, value in dict(profile.diagnostics).items()},
    }


def grower_diagnostic_summary(profile: dict[str, Any]) -> dict[str, float]:
    diagnostics = profile.get("diagnostics", {})
    keys = [
        "grower.ffb_result_seed_miss",
        "grower.connected_invariant_violation",
        "grower.rejected_disconnected",
        "grower.rejected_disconnected_gap_max",
        "grower.child_contained_in_parent",
        "grower.rejected_contained_child",
        "grower.seed_already_covered",
        "grower.component_connect_attempts",
        "grower.component_connect_target_tasks",
        "grower.component_connect_target_no_candidate",
        "grower.component_connect_candidate_rank_max",
        "grower.component_connect_adaptive_ffb_tasks",
        "grower.component_connect_adaptive_ffb_depth_max",
        "grower.component_connect_successes",
        "grower.component_connect_failures",
        "grower.component_connect_parent_failure_max",
        "grower.component_connect_parent_skipped",
        "grower.component_connect_retried_parent",
        "grower.component_connect_no_candidate",
        "grower.component_connect_no_frontier_seed",
        "grower.frontier_no_uncovered_seed",
        "grower.root_frontier_no_uncovered_seed",
        "grower.frontier_scanned_boxes",
        "grower.frontier_scanned_faces",
        "grower.growth_target_samples",
        "grower.growth_tasks_planned",
        "grower.executor_threads",
        "grower.config_threads",
        "grower.all_root_sample_batches",
        "grower.all_root_sample_root_attempts",
        "grower.intertree_goal_bias_tasks",
        "grower.goal_bias_high_pulse_tasks",
        "grower.goal_bias_high_capped_tasks",
        "connector.frontier_bridge_attempts",
        "connector.frontier_bridge_successes",
        "connector.frontier_bridge_no_expandable_face",
        "connector.frontier_bridge_face_candidates",
        "connector.frontier_bridge_face_rotation_tasks",
        "connector.frontier_bridge_gap_stall_count_max",
        "connector.frontier_bridge_adaptive_ffb_tasks",
        "connector.frontier_bridge_adaptive_ffb_depth_max",
        "connector.frontier_bridge_gap_max",
        "connector.frontier_bridge_gap_latest",
        "connector.frontier_bridge_ffb_failures",
        "connector.frontier_bridge_ffb_unknown_depth_cap",
        "connector.frontier_bridge_ffb_reserved_depth_cap",
        "connector.rrt_successes",
        "connector.rrt_failures",
        "connector.chain_pave_attempts",
        "connector.chain_pave_successes",
        "connector.chain_pave_zero_added",
        "grower.task_truncated_to_remaining",
        "grower.task_reserved_domain",
        "grower.task_duplicate_domain",
        "grower.task_skipped_no_worker_domain",
        "grower.worker_ffb_batches",
        "grower.worker_ffb_tasks",
        "grower.worker_ffb_commits",
        "grower.worker_ffb_disabled",
        "grower.worker_ffb_empty_tasks",
        "grower.worker_ffb_inline_executor",
        "grower.worker_ffb_non_lect_oracle",
        "grower.worker_ffb_missing_domain",
        "grower.root_seed_attempts",
        "grower.root_seeds_target_count",
        "grower.root_seeds_final_count",
        "grower.root_seeds_min_normalized_linf",
        "grower.root_seed_min_distance_rejected",
        "grower.root_seed_lca_rejected",
        "grower.root_seed_candidate_collision",
        "grower.root_seed_user_collision",
        "grower.goal_bias_high_warning",
        "grower.ffb_failures",
        "grower.ffb_reserved_depth_cap",
        "grower.ffb_unknown_depth_cap",
        "grower.depth_stage_index",
        "grower.depth_stage_depth",
        "grower.depth_stage_depth_max",
        "grower.depth_stage_root_depth",
        "grower.depth_stage_switches",
        "grower.depth_stage_box_count_at_switch_max",
        "grower.failure_cooling_recorded_failures",
        "grower.failure_cooling_activated",
        "grower.failure_cooling_hits",
        "grower.failure_cooling_skips",
        "grower.failure_cooling_expired",
        "grower.failure_cooling_retries_after_stage_raise",
        "grower.failure_cooling_node_count_max",
        "grower.failure_cooling_fail_count_max",
        "grower.failure_cooling_cool_until_box_count_max",
        "grower.failure_cooling_remaining_horizon_max",
        "grower.failure_cooling_success_clears",
        "grower.task_skipped_failure_cooling",
    ]
    return {key: float(diagnostics.get(key, 0.0)) for key in keys}


def run_scene(args: argparse.Namespace, scene_name: str, pair) -> dict[str, Any]:
    if scene_name not in SCENES:
        raise ValueError(f"unknown scene {scene_name!r}; choices: {sorted(SCENES)}")
    robot = load_iiwa14_robot()
    config = configure(args)
    forest = sbf.SafeBoxForest(robot, config)
    obstacles = SCENES[scene_name]()
    seeds = seeds_for(args, pair)

    start = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    wall_ms = (time.perf_counter() - start) * 1000.0
    result = forest.query(list(pair.start), list(pair.goal))
    adjacency = {int(node): [int(value) for value in neighbors] for node, neighbors in dict(forest.adjacency()).items()}
    boxes = forest.boxes()
    root_hist: dict[int, int] = {}
    for box in boxes:
        root_hist[int(box.root_id)] = root_hist.get(int(box.root_id), 0) + 1

    profile_dict = profile_to_dict(profile)
    return {
        "scene": scene_name,
        "n_obstacles": len(obstacles),
        "query": pair.label,
        "build_wall_ms": wall_ms,
        "profile": profile_dict,
        "grower_diagnostics": grower_diagnostic_summary(profile_dict),
        "query_result": {
            "success": bool(result.success),
            "start_box_id": int(result.start_box_id),
            "goal_box_id": int(result.goal_box_id),
            "box_sequence_len": len(result.box_sequence),
            "path_length": float(result.path_length),
            "query_time_ms": float(result.query_time_ms),
        },
        "component_sizes": component_sizes(adjacency),
        "cross_root_gap": cross_root_gap(boxes),
        "root_hist": root_hist,
    }


def main() -> int:
    args = parse_args()
    pair = query_pair(args.query)
    depth_stages = parse_depth_schedule(args.depth_schedule)
    payload = {
        "config": {
            "endpoint_source": args.endpoint_source,
            "grower": args.grower,
            "seed_set": args.seed_set,
            "max_boxes": args.max_boxes,
            "ffb_depth": args.ffb_depth,
            "depth_schedule": depth_schedule_to_payload(depth_stages),
            "split_policy": args.split_policy,
            "best_tighten_depth_synchronous": args.best_tighten_depth_synchronous,
            "best_tighten_prefer_sector_boundary": args.best_tighten_prefer_sector_boundary,
            "best_tighten_use_minimax": args.best_tighten_use_minimax,
            "best_tighten_width_penalty": args.best_tighten_width_penalty,
            "best_tighten_shape_balancing": args.best_tighten_shape_balancing,
            "best_tighten_max_child_aspect": args.best_tighten_max_child_aspect,
            "best_tighten_min_split_width_fraction": args.best_tighten_min_split_width_fraction,
            "best_tighten_shape_weight": args.best_tighten_shape_weight,
            "best_tighten_balance_weight": args.best_tighten_balance_weight,
            "best_tighten_relative_gain_weight": args.best_tighten_relative_gain_weight,
            "best_tighten_widest_tiebreak_weight": args.best_tighten_widest_tiebreak_weight,
            "threads": args.threads,
            "task_batch_size": args.task_batch_size,
            "rrt_goal_bias": args.rrt_goal_bias,
            "intertree_goal_bias": args.intertree_goal_bias,
            "sustained_goal_bias_cap": args.sustained_goal_bias_cap,
            "high_goal_bias_pulse_period": args.high_goal_bias_pulse_period,
            "expand_all_roots_per_sample": args.expand_all_roots_per_sample,
            "enable_connector": args.enable_connector,
            "frontier_bridge": args.frontier_bridge,
            "frontier_bridge_adaptive_ffb": args.frontier_bridge_adaptive_ffb,
            "frontier_bridge_gap_stall_iterations": args.frontier_bridge_gap_stall_iterations,
            "frontier_bridge_ffb_depth_increment": args.frontier_bridge_ffb_depth_increment,
            "frontier_bridge_ffb_max_depth": args.frontier_bridge_ffb_max_depth,
            "frontier_bridge_candidate_limit": args.frontier_bridge_candidate_limit,
            "connector_bridge_boxes": args.connector_bridge_boxes,
            "extra_random_roots": args.extra_random_roots,
            "root_seed_candidate_count": args.root_seed_candidate_count,
            "root_seed_min_normalized_linf": args.root_seed_min_normalized_linf,
            "root_seed_max_lca_depth": args.root_seed_max_lca_depth,
            "component_connect_prob": args.component_connect_prob,
            "component_connect_max_parent_failures": args.component_connect_max_parent_failures,
            "component_connect_candidate_limit": args.component_connect_candidate_limit,
            "component_connect_adaptive_ffb": args.component_connect_adaptive_ffb,
            "component_connect_ffb_depth_increment": args.component_connect_ffb_depth_increment,
            "component_connect_ffb_max_depth": args.component_connect_ffb_max_depth,
            "failure_cooling": args.failure_cooling,
            "failure_cooling_threshold": args.failure_cooling_threshold,
            "failure_cooling_box_horizon": args.failure_cooling_box_horizon,
            "failure_cooling_min_depth": args.failure_cooling_min_depth,
            "failure_cooling_unknown_only": args.failure_cooling_unknown_only,
            "failure_cooling_retry_on_depth_raise": args.failure_cooling_retry_on_depth_raise,
            "unexplored_prob": args.unexplored_prob,
            "step_ratio": args.step_ratio,
            "stop_after_connect": args.stop_after_connect,
        },
        "results": [run_scene(args, scene_name, pair) for scene_name in args.scenes],
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())