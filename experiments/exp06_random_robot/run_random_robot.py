#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, csv_list, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import DEFAULT_QUERIES_PER_SCENE, generate_catalog, make_robot, queries_for_key, scene_for_key
from experiments.common.rbf_defaults import (
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_DEEP_MAX_BOXES,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_MAX_DEPTH,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
    ROBOT_LECTDB_CACHE_ROOT,
    default_rbf_profile,
    rbf_budget_grid,
    robot_lectdb_profile,
    robot_joint_limit_tuples,
    robot_symmetry_aligned_root_tuples,
)
from experiments.common.rbf_leaf_rrt import QuerySpec, RBFLeafRRTOptions, run_leaf_rrt
from experiments.common.robot_lectdb_cache import ensure_robot_lectdb_cache, robot_external_evidence_path
from experiments.common.sbf_import import import_sbf


METHODS = ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
sbf = import_sbf()


def configure_thread_environment(threads: int) -> None:
    value = str(max(1, int(threads)))
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
    ):
        os.environ[key] = value


def effective_rbf_profile(args: argparse.Namespace, box_budgets: list[int] | None = None) -> dict[str, Any]:
    profile = copy.deepcopy(default_rbf_profile())
    inherited_profile = str(profile.get("profile", "registered_exp4_profile"))
    profile["profile"] = (
        f"exp06_leaf{int(args.leaf_max_depth)}"
        f"_ffb{int(args.deep_ffb_depth)}"
        f"_bridge{int(args.query_bridge_pave_depth)}"
    )
    profile["inherits_from"] = inherited_profile
    profile["override_reason"] = "Exp.6 controlled depth trade-off scan on saved random-scene catalog."
    profile["leaf_sweep"]["leaf_start_depth"] = int(args.leaf_start_depth)
    profile["leaf_sweep"]["leaf_max_depth"] = int(args.leaf_max_depth)
    profile["leaf_sweep"]["leaf_threads"] = int(args.threads)
    profile["deep_refine"]["deep_max_boxes"] = int(args.deep_max_boxes)
    profile["deep_refine"]["deep_ffb_depth"] = int(args.deep_ffb_depth)
    profile["deep_refine"]["ffb_start_depth"] = int(args.ffb_start_depth)
    profile["deep_refine"]["ffb_search_mode"] = str(args.ffb_search_mode)
    profile["connector"]["pave_depth"] = int(args.connector_pave_depth)
    profile["connector"]["max_pairs_per_gap"] = int(DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP)
    profile["connector"]["per_pair_timeout_ms"] = int(DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS)
    profile["query_bridge"]["pave_depth"] = int(args.query_bridge_pave_depth)
    profile["query_bridge"]["all_queries"] = bool(args.query_bridge_all)
    profile["query_bridge"]["adaptive_all"] = bool(args.query_bridge_adaptive_all)
    profile["query_bridge"]["adaptive_max_path_length"] = float(args.query_bridge_adaptive_max_path_length)
    profile["query_bridge"]["direct_sample_step"] = float(args.query_bridge_direct_sample_step)
    profile["query_bridge"]["repair_subdivisions"] = int(args.query_bridge_repair_subdivisions)
    profile["query_bridge"]["direct_max_length"] = float(args.query_bridge_direct_max_length)
    profile["query"]["final_rrt_simplify_timeout_ms"] = 1000.0 * float(args.ompl_simplify_time_s)
    profile["query"]["final_rrt_simplify_time_s"] = float(args.ompl_simplify_time_s)
    if box_budgets is not None:
        profile["box_budget_grid"] = [int(value) for value in box_budgets]
    return profile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.6 saved-catalog random multi-robot study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=50)
    parser.add_argument("--scene-profile", choices=["balanced", "balanced_independent", "balanced_probe", "legacy"], default="balanced_independent")
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--queries-per-scene", type=int, default=DEFAULT_QUERIES_PER_SCENE)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--methods", default="sbf_leaf_rrt")
    parser.add_argument("--deep-max-boxes", type=int, default=DEFAULT_RBF_DEEP_MAX_BOXES)
    parser.add_argument("--box-budgets", default="")
    parser.add_argument("--rbf-max-depth", type=int, default=DEFAULT_RBF_MAX_DEPTH)
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depth", type=int, default=DEFAULT_RBF_LEAF_MAX_DEPTH)
    parser.add_argument("--deep-ffb-depth", type=int, default=DEFAULT_RBF_DEEP_FFB_DEPTH)
    parser.add_argument("--connector-pave-depth", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_DEPTH)
    parser.add_argument("--query-bridge-pave-depth", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH)
    parser.add_argument("--ffb-start-depth", type=int, default=DEFAULT_RBF_FFB_START_DEPTH)
    parser.add_argument("--ffb-search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE)
    parser.add_argument("--connector-segment-resolution", type=int, default=None)
    parser.add_argument("--query-bridge-all", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-adaptive-all", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-adaptive-max-path-length", type=float, default=4.5)
    parser.add_argument("--query-bridge-accept-segment-fraction", type=float, default=0.25)
    parser.add_argument("--query-bridge-accept-path-ratio", type=float, default=1.50)
    parser.add_argument("--query-bridge-accept-path-additive", type=float, default=0.75)
    parser.add_argument("--query-bridge-direct-sample-step", type=float, default=0.04)
    parser.add_argument("--query-bridge-repair-subdivisions", type=int, default=1)
    parser.add_argument("--query-bridge-direct-max-length", type=float, default=6.5)
    parser.add_argument("--query-bridge-force-selected", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-to-main-island", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--query-bridge-to-main-direct-segment-max-length", type=float, default=0.0)
    parser.add_argument("--endpoint-main-target-k", type=int, default=8)
    parser.add_argument("--endpoint-main-coarse-step", type=float, default=0.08)
    parser.add_argument("--endpoint-main-fine-step", type=float, default=0.02)
    parser.add_argument("--endpoint-main-max-ffb-calls", type=int, default=48)
    parser.add_argument("--endpoint-main-max-boxes", type=int, default=64)
    parser.add_argument("--endpoint-main-adaptive-ffb-depths", default="50,58,62")
    parser.add_argument("--endpoint-main-residual-segment-max-length", type=float, default=0.25)
    parser.add_argument("--endpoint-main-lateral-offset", type=float, default=0.03)
    parser.add_argument("--endpoint-main-lateral-rounds", type=int, default=2)
    parser.add_argument("--endpoint-main-face-epsilon", type=float, default=1e-6)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--offline-random-anchors", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--offline-anchor-count", type=int, default=16)
    parser.add_argument("--offline-anchor-candidate-count", type=int, default=512)
    parser.add_argument("--offline-anchor-lca-lambda", type=float, default=0.35)
    parser.add_argument("--offline-anchor-distance-mu", type=float, default=0.10)
    parser.add_argument("--offline-shortcut-edges", type=int, default=0)
    parser.add_argument("--offline-shortcut-candidate-limit", type=int, default=48)
    parser.add_argument("--offline-shortcut-min-gain-ratio", type=float, default=1.6)
    parser.add_argument("--offline-shortcut-max-segment-length", type=float, default=3.0)
    parser.add_argument("--lect-cache-root", type=Path, default=ROBOT_LECTDB_CACHE_ROOT)
    parser.add_argument("--skip-lect-cache-ensure", action="store_true")
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--rrt-timeout-s", type=float, default=1.0)
    parser.add_argument("--rrt-range", type=float, default=0.35)
    parser.add_argument("--ompl-simplify-time-s", type=float, default=0.01)
    parser.add_argument("--prm-build-s", type=float, default=2.0)
    parser.add_argument("--prm-build-grid-s", default="2")
    parser.add_argument("--prm-query-s", type=float, default=4.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-planner-kind", default="prm")
    parser.add_argument("--prm-preload-query-endpoints", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bitstar-timeout-s", type=float, default=0.5)
    parser.add_argument("--bitstar-timeout-grid-s", default="")
    parser.add_argument("--bitstar-checkpoint-grid-s", default="")
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=0.005)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def bitstar_checkpoint_grid_from_args(args: argparse.Namespace, timeout_s: float) -> list[float]:
    raw_grid = str(getattr(args, "bitstar_checkpoint_grid_s", "")).strip()
    if not raw_grid and str(getattr(args, "bitstar_timeout_grid_s", "")).strip():
        raw_grid = str(args.bitstar_timeout_grid_s)
    if raw_grid:
        values = sorted({float(value) for value in csv_list(raw_grid) if float(value) > 0.0})
        values = [min(float(timeout_s), value) for value in values if value <= float(timeout_s) + 1e-9]
        if not values or abs(values[-1] - float(timeout_s)) > 1e-9:
            values.append(float(timeout_s))
        return values
    interval_s = max(float(args.bitstar_checkpoint_interval_s), 1e-9)
    values: list[float] = []
    target_s = interval_s
    while target_s < float(timeout_s) - 1e-9:
        values.append(float(target_s))
        target_s += interval_s
    values.append(float(timeout_s))
    return values


def bitstar_trace_interval_for_grid(args: argparse.Namespace, checkpoint_grid_s: list[float], timeout_s: float) -> float:
    deltas = [
        float(b) - float(a)
        for a, b in zip([0.0] + checkpoint_grid_s[:-1], checkpoint_grid_s)
        if float(b) - float(a) > 1e-9
    ]
    if deltas:
        return max(1e-9, min(deltas))
    return max(1e-9, min(float(args.bitstar_checkpoint_interval_s), float(timeout_s)))


def checkpoint_at_or_after(checkpoints: list[dict[str, Any]], target_s: float) -> dict[str, Any]:
    if not checkpoints:
        return {}
    for checkpoint in checkpoints:
        if float(checkpoint.get("checkpoint_s", 0.0) or 0.0) >= float(target_s) - 1e-9:
            return checkpoint
    return checkpoints[-1]


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return math.nan
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
    return total


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def audit_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    *,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    endpoint_tol: float = 1e-6,
    collision_tolerance: float = DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    if len(path) < 2:
        return False, time.perf_counter() - t0, "empty_path"
    if start is not None and point_distance(path[0], list(start)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "start_mismatch"
    if goal is not None and point_distance(path[-1], list(goal)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "goal_mismatch"
    step = max(1e-9, float(segment_step))
    for a, b in zip(path, path[1:]):
        distance = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(distance / step)))
        for index in range(steps + 1):
            if sbf.check_config_collision(
                robot,
                obstacles,
                interpolate(a, b, index / steps),
                float(collision_tolerance),
            ):
                return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"


def simplify_path_if_requested(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    simplify_time_s: float,
) -> tuple[list[list[float]], float, str]:
    if len(path) < 2 or simplify_time_s <= 0.0:
        return path, 0.0, "skipped"
    result = sbf.ompl_simplify_path(
        robot,
        obstacles,
        path,
        float(segment_step),
        float(simplify_time_s),
    )
    if bool(result.get("ok")):
        return [[float(value) for value in point] for point in result.get("path", [])], float(result.get("t_s", 0.0)), str(result.get("reason", "simplified"))
    return path, float(result.get("t_s", 0.0) or 0.0), str(result.get("reason", "simplify_failed"))


def run_rbf_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    queries = [
        QuerySpec(
            label=f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            start=[float(value) for value in query["start"]],
            goal=[float(value) for value in query["goal"]],
            actual_start=[float(value) for value in query["start"]],
            actual_goal=[float(value) for value in query["goal"]],
        )
        for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed))
    ]
    valid_root = robot_joint_limit_tuples(robot)
    root_override = robot_symmetry_aligned_root_tuples(robot) if str(robot_name) == "iiwa" else valid_root
    stage_id = (
        f"l{int(args.leaf_max_depth)}"
        f"_ffb{int(args.deep_ffb_depth)}"
        f"_b{int(args.deep_max_boxes)}"
        f"_a{int(args.offline_anchor_count)}"
        f"_c{int(args.offline_anchor_candidate_count)}"
        f"_os{int(args.offline_shortcut_edges)}"
        f"_tm{int(bool(args.query_bridge_to_main_island))}"
    )
    active_cache_name = (
        f"rbf_{robot_name}_{difficulty}_{int(scene_seed)}"
        f"_b{int(args.deep_max_boxes)}"
        f"_l{int(args.leaf_max_depth)}"
        f"_ffb{int(args.deep_ffb_depth)}"
        f"_a{int(args.offline_anchor_count)}"
        f"_c{int(args.offline_anchor_candidate_count)}"
        f"_os{int(args.offline_shortcut_edges)}"
        f"_tm{int(bool(args.query_bridge_to_main_island))}"
    )
    row = run_leaf_rrt(
        robot=robot,
        obstacles=list(scene.obstacles),
        queries=queries,
        database_path=args.out_dir / "active_cache" / active_cache_name,
        options=RBFLeafRRTOptions(
            seed=int(scene_seed),
            deep_max_boxes=int(args.deep_max_boxes),
            rbf_max_depth=int(args.rbf_max_depth),
            threads=int(args.threads),
            leaf_start_depth=int(args.leaf_start_depth),
            leaf_max_depth=int(args.leaf_max_depth),
            deep_ffb_depth=int(args.deep_ffb_depth),
            connector_pave_depth=int(args.connector_pave_depth),
            query_bridge_pave_depth=int(args.query_bridge_pave_depth),
            ffb_start_depth=int(args.ffb_start_depth),
            ffb_search_mode=str(args.ffb_search_mode),
            use_external_evidence=True,
            external_evidence_path=robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root)),
            external_evidence_verify_identity=False,
            root_override_tuples=root_override,
            coverage_override_tuples=valid_root,
            symmetry_aligned_native_root=str(robot_name) == "iiwa",
            symmetry_aligned_cache_schedule=str(robot_name) == "iiwa",
            database_canonical_mode=True,
            case_label=f"rbf_{robot_name}_{difficulty}",
            parallel_virtual_validation=True,
            leaf_threads=int(args.threads),
            canonicalize_queries=False,
            audit_collision_tolerance=float(args.audit_collision_tolerance),
            offline_query_agnostic_build=True,
            offline_random_anchors=bool(args.offline_random_anchors),
            offline_anchor_count=int(args.offline_anchor_count),
            offline_anchor_candidate_count=int(args.offline_anchor_candidate_count),
            offline_anchor_lca_lambda=float(args.offline_anchor_lca_lambda),
            offline_anchor_distance_mu=float(args.offline_anchor_distance_mu),
            offline_shortcut_edges=int(args.offline_shortcut_edges),
            offline_shortcut_candidate_limit=int(args.offline_shortcut_candidate_limit),
            offline_shortcut_min_gain_ratio=float(args.offline_shortcut_min_gain_ratio),
            offline_shortcut_max_segment_length=float(args.offline_shortcut_max_segment_length),
            connector_pair_timeout_ms=DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
            connector_max_pairs_per_gap=DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
            query_bridge_all=bool(args.query_bridge_all),
            query_bridge_adaptive_all=bool(args.query_bridge_adaptive_all),
            query_bridge_adaptive_max_path_length=float(args.query_bridge_adaptive_max_path_length),
            query_bridge_accept_segment_fraction=float(args.query_bridge_accept_segment_fraction),
            query_bridge_accept_path_ratio=float(args.query_bridge_accept_path_ratio),
            query_bridge_accept_path_additive=float(args.query_bridge_accept_path_additive),
            query_bridge_direct_sample_step=float(args.query_bridge_direct_sample_step),
            query_bridge_repair_subdivisions=int(args.query_bridge_repair_subdivisions),
            query_bridge_direct_max_length=float(args.query_bridge_direct_max_length),
            query_bridge_force_selected=bool(args.query_bridge_force_selected),
            query_bridge_to_main_island=bool(args.query_bridge_to_main_island),
            query_bridge_to_main_direct_segment_max_length=float(args.query_bridge_to_main_direct_segment_max_length),
            endpoint_main_target_k=int(args.endpoint_main_target_k),
            endpoint_main_coarse_step=float(args.endpoint_main_coarse_step),
            endpoint_main_fine_step=float(args.endpoint_main_fine_step),
            endpoint_main_max_ffb_calls=int(args.endpoint_main_max_ffb_calls),
            endpoint_main_max_boxes=int(args.endpoint_main_max_boxes),
            endpoint_main_adaptive_ffb_depths=str(args.endpoint_main_adaptive_ffb_depths),
            endpoint_main_residual_segment_max_length=float(args.endpoint_main_residual_segment_max_length),
            endpoint_main_lateral_offset=float(args.endpoint_main_lateral_offset),
            endpoint_main_lateral_rounds=int(args.endpoint_main_lateral_rounds),
            endpoint_main_face_epsilon=float(args.endpoint_main_face_epsilon),
            connector_segment_resolution=(
                int(args.connector_segment_resolution)
                if args.connector_segment_resolution is not None
                else RBFLeafRRTOptions().connector_segment_resolution
            ),
            final_rrt_simplify_timeout_ms=1000.0 * float(args.ompl_simplify_time_s),
            final_rrt_simplify_attempts=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
        ),
    )
    row.update(
        {
            "method": "sbf_leaf_rrt",
            "robot": robot_name,
            "difficulty": difficulty,
            "scene_seed": int(scene_seed),
            "stage_id": stage_id,
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "deep_ffb_depth": int(args.deep_ffb_depth),
            "connector_pave_depth": int(args.connector_pave_depth),
            "query_bridge_pave_depth": int(args.query_bridge_pave_depth),
            "offline_anchor_count": int(args.offline_anchor_count),
            "offline_anchor_candidate_count": int(args.offline_anchor_candidate_count),
            "offline_shortcut_edges": int(args.offline_shortcut_edges),
            "offline_shortcut_candidate_limit": int(args.offline_shortcut_candidate_limit),
            "offline_shortcut_min_gain_ratio": float(args.offline_shortcut_min_gain_ratio),
            "offline_shortcut_max_segment_length": float(args.offline_shortcut_max_segment_length),
            "query_bridge_to_main_island": bool(args.query_bridge_to_main_island),
            "query_bridge_to_main_direct_segment_max_length": float(args.query_bridge_to_main_direct_segment_max_length),
            "ffb_start_depth": int(args.ffb_start_depth),
            "rbf_max_depth": int(args.rbf_max_depth),
            "deep_max_boxes": int(args.deep_max_boxes),
            "obstacle_count": len(scene.obstacles),
            "queries_per_scene": len(queries),
            "scene_catalog": str(args.scene_catalog or (args.out_dir / "random_scene_catalog_v6.json")),
            "lectdb": robot_lectdb_profile(robot_name),
            "active_planning_root": "full_robot_joint_limits",
            "coverage_root": "full_robot_joint_limits",
            "canonical_mapping_scope": "LECT_internal_only",
            "external_evidence_path": str(robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root))),
        }
    )
    return row


def summarize_single_query_method(
    method: str,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    scene: Any,
    planning_s: float,
    audit_s: float,
    ok: bool,
    audit_passed: bool,
    audit_status: str,
    length: float,
    path: list[list[float]],
    diagnostics: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    success = bool(ok) and bool(audit_passed)
    return {
        "method": method,
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "deep_max_boxes": 0,
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "obstacle_count": len(scene.obstacles),
        "status": "ok" if success else "failed_audit" if ok else "failed_planning",
        "success_count": 1 if success else 0,
        "query_count": 1,
        "planning_s": float(planning_s),
        "audit_s": float(audit_s),
        "path_length_mean": float(length) if success else math.nan,
        "raw_segment_fraction": 0.0 if success else math.nan,
        "final_boxes": math.nan,
        "queries": [
            {
                "label": f"{robot_name}_{difficulty}_{scene_seed}",
                "success": success,
                "audit_passed": bool(audit_passed),
                "audit_status": audit_status,
                "query_ms": float(planning_s) * 1000.0,
                "solve_ms": float(planning_s) * 1000.0,
                "simplify_ms": 0.0,
                "audit_ms": float(audit_s) * 1000.0,
                "path_length": float(length) if success else math.nan,
                "segment_fraction": 0.0 if success else math.nan,
                "waypoint_count": len(path),
            }
        ],
        "diagnostics": dict(diagnostics or {}),
    }


def summarize_query_batch_method(
    method: str,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    scene: Any,
    qrows: list[dict[str, Any]],
    *,
    offline_build_s: float = 0.0,
    online_batch_s: float | None = None,
    audit_s: float = 0.0,
    diagnostics: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    successes = [row for row in qrows if bool(row.get("audit_passed"))]
    query_count = max(1, len(qrows))
    online_s = (
        sum(float(row.get("query_ms", 0.0)) for row in qrows) / 1000.0
        if online_batch_s is None
        else float(online_batch_s)
    )
    online_solve_s = 0.0
    online_simplify_s = 0.0
    split_available = False
    for row in qrows:
        try:
            total_ms = float(row.get("query_ms", math.nan))
            solve_ms = float(row.get("solve_ms", math.nan))
            simplify_ms = float(row.get("simplify_ms", math.nan))
        except (TypeError, ValueError):
            continue
        if math.isfinite(solve_ms) or math.isfinite(simplify_ms):
            split_available = True
            if math.isfinite(solve_ms):
                online_solve_s += solve_ms / 1000.0
            elif math.isfinite(total_ms) and math.isfinite(simplify_ms):
                online_solve_s += max(0.0, total_ms - simplify_ms) / 1000.0
            if math.isfinite(simplify_ms):
                online_simplify_s += simplify_ms / 1000.0
    if not split_available:
        online_solve_s = online_s
        online_simplify_s = 0.0
    else:
        residual_s = online_s - online_solve_s - online_simplify_s
        if residual_s > 1e-9:
            online_solve_s += residual_s
    online_total_s = online_s
    online_s = online_solve_s
    online_per_query_s = online_solve_s / query_count
    online_total_per_query_s = online_total_s / query_count
    online_solve_per_query_s = online_solve_s / query_count
    online_simplify_per_query_s = online_simplify_s / query_count
    amortized = {
        f"amortized_s_k{k}": float(offline_build_s) / float(k) + online_per_query_s
        for k in (1, 5, 10, 20, 50)
    }
    return {
        "method": method,
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "deep_max_boxes": 0,
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "obstacle_count": len(scene.obstacles),
        "queries_per_scene": len(qrows),
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "planning_s": float(offline_build_s) + online_s,
        "planning_total_s": float(offline_build_s) + online_total_s,
        "build_s": float(offline_build_s),
        "offline_build_s": float(offline_build_s),
        "online_batch_s": online_s,
        "online_total_s": online_total_s,
        "online_total_batch_s": online_total_s,
        "online_solve_s": online_solve_s,
        "online_simplify_s": online_simplify_s,
        "online_per_query_s": online_per_query_s,
        "online_total_per_query_s": online_total_per_query_s,
        "online_solve_per_query_s": online_solve_per_query_s,
        "online_simplify_per_query_s": online_simplify_per_query_s,
        **amortized,
        "audit_s": float(audit_s),
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": 0.0 if successes else math.nan,
        "final_boxes": math.nan,
        "queries": qrows,
        "diagnostics": dict(diagnostics or {}),
    }


def run_rrtconnect_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    qrows: list[dict[str, Any]] = []
    audit_total_s = 0.0
    for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed)):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        result = sbf.ompl_rrt_connect_path(
            robot,
            list(scene.obstacles),
            start,
            goal,
            float(args.rrt_timeout_s) * 1000.0,
            float(args.rrt_range),
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(args.seed_base) + int(scene_seed) * 1009 + index,
        )
        total_s = float(result.get("t_s", 0.0))
        solve_s = float(result.get("solve_s", max(0.0, total_s - float(result.get("simplify_s", 0.0)))))
        simplify_s = float(result.get("simplify_s", max(0.0, total_s - solve_s)))
        path = [[float(value) for value in point] for point in result.get("path", [])]
        audit_passed, audit_s, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            float(args.audit_segment_step),
            start=start,
            goal=goal,
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_total_s += audit_s
        success = bool(result.get("ok")) and bool(audit_passed)
        qrows.append({
            "label": f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            "success": success,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_s * 1000.0,
            "path_length": path_length(path) if success else math.nan,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        })
    return summarize_query_batch_method(
        "rrtconnect",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        qrows,
        audit_s=audit_total_s,
        diagnostics={
            "planner": "OMPL_RRTConnect",
            "timeout_s": float(args.rrt_timeout_s),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
    )


def run_prm_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int, build_budget_s: float) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    query_records = queries_for_key(catalog, robot_name, difficulty, scene_seed)
    starts = [[float(value) for value in query["start"]] for query in query_records]
    goals = [[float(value) for value in query["goal"]] for query in query_records]
    result = sbf.ompl_prm_multiquery(
        robot,
        list(scene.obstacles),
        starts,
        goals,
        float(build_budget_s),
        float(args.prm_query_s),
        float(args.audit_segment_step),
        float(args.ompl_simplify_time_s),
        int(args.seed_base) + 10007 * int(scene_seed),
        int(args.prm_max_nearest_neighbors),
        str(args.prm_planner_kind),
        bool(args.prm_preload_query_endpoints),
    )
    qrows: list[dict[str, Any]] = []
    audit_total_s = 0.0
    for index, (query, qresult) in enumerate(zip(query_records, list(result.get("queries", [])), strict=False)):
        path = [[float(value) for value in point] for point in qresult.get("path", [])]
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        audit_passed, audit_s, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            float(args.audit_segment_step),
            start=start,
            goal=goal,
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_total_s += audit_s
        success = bool(qresult.get("ok")) and bool(audit_passed)
        total_s = float(qresult.get("t_s", 0.0))
        solve_s = float(qresult.get("solve_s", max(0.0, total_s - float(qresult.get("simplify_s", 0.0)))))
        simplify_s = float(qresult.get("simplify_s", max(0.0, total_s - solve_s)))
        qrows.append({
            "label": f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            "success": success,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_s * 1000.0,
            "path_length": path_length(path) if success else math.nan,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        })
    return summarize_query_batch_method(
        "prm",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        qrows,
        offline_build_s=float(result.get("build_s", 0.0)),
        audit_s=audit_total_s,
        diagnostics={
            "planner": "OMPL_PRM",
            "build_s": float(result.get("build_s", 0.0)),
            "query_s": sum(float(row.get("query_ms", 0.0)) for row in qrows) / 1000.0,
            "nodes": int(result.get("nodes", 0) or 0),
            "query_budget_s": float(args.prm_query_s),
            "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
            "planner_kind": str(args.prm_planner_kind),
            "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
        stage_id=(
            f"{str(args.prm_planner_kind)}_build{float(build_budget_s):g}s"
            f"_k{int(args.prm_max_nearest_neighbors)}"
            f"_q{float(args.prm_query_s):g}s"
            f"_preload{int(bool(args.prm_preload_query_endpoints))}"
        ),
        budget_s=float(build_budget_s),
    )


def run_bitstar_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int, timeout_s: float) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    qrows: list[dict[str, Any]] = []
    audit_total_s = 0.0
    for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed)):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        result = sbf.ompl_bitstar_path(
            robot,
            list(scene.obstacles),
            start,
            goal,
            float(timeout_s) * 1000.0,
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(args.seed_base) + 20011 * int(scene_seed) + index,
            int(args.bitstar_samples_per_batch),
            float(args.bitstar_rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
        )
        path = [[float(value) for value in point] for point in result.get("path", [])]
        audit_passed, audit_s, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            float(args.audit_segment_step),
            start=start,
            goal=goal,
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_total_s += audit_s
        success = bool(result.get("ok")) and bool(audit_passed)
        total_s = float(result.get("t_s", 0.0))
        solve_s = float(result.get("solve_s", max(0.0, total_s - float(result.get("simplify_s", 0.0)))))
        simplify_s = float(result.get("simplify_s", max(0.0, total_s - solve_s)))
        qrows.append({
            "label": f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            "success": success,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_s * 1000.0,
            "path_length": path_length(path) if success else math.nan,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        })
    return summarize_query_batch_method(
        "bitstar",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        qrows,
        audit_s=audit_total_s,
        diagnostics={
            "planner": "OMPL_BITstar",
            "timeout_s": float(timeout_s),
            "samples_per_batch": int(args.bitstar_samples_per_batch),
            "rewire_factor": float(args.bitstar_rewire_factor),
            "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
        stage_id=(
            f"batch{int(args.bitstar_samples_per_batch)}"
            f"_rw{float(args.bitstar_rewire_factor):g}"
            f"_t{float(timeout_s):g}s"
        ),
        budget_s=float(timeout_s),
    )


def run_bitstar_scene_trace(args: argparse.Namespace,
                            catalog: dict[str, Any],
                            robot_name: str,
                            difficulty: str,
                            scene_seed: int,
                            timeout_s: float,
                            checkpoint_interval_s: float,
                            checkpoint_grid_s: list[float]) -> list[dict[str, Any]]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    query_traces: list[tuple[dict[str, Any], list[dict[str, Any]]]] = []
    for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed)):
        result = sbf.ompl_bitstar_trace(
            robot,
            list(scene.obstacles),
            [float(value) for value in query["start"]],
            [float(value) for value in query["goal"]],
            float(timeout_s) * 1000.0,
            float(checkpoint_interval_s) * 1000.0,
            float(args.audit_segment_step),
            int(args.seed_base) + 20011 * int(scene_seed) + index,
            int(args.bitstar_samples_per_batch),
            float(args.bitstar_rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
        )
        query_traces.append((dict(query), [dict(item) for item in result.get("checkpoints", [])]))
    rows: list[dict[str, Any]] = []
    best_by_query: dict[str, dict[str, Any]] = {}
    for target_checkpoint_s in checkpoint_grid_s:
        qrows: list[dict[str, Any]] = []
        audit_total_s = 0.0
        checkpoint_s = 0.0
        for index, (query, checkpoints) in enumerate(query_traces):
            checkpoint = checkpoint_at_or_after(checkpoints, target_checkpoint_s)
            checkpoint_s = max(checkpoint_s, float(checkpoint.get("checkpoint_s", target_checkpoint_s) or 0.0))
            elapsed_s = float(checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0)) or 0.0)
            solve_s = float(checkpoint.get("solve_s", checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0))) or 0.0)
            path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
            path, simplify_s, simplify_status = simplify_path_if_requested(
                robot,
                list(scene.obstacles),
                path,
                float(args.audit_segment_step),
                float(args.ompl_simplify_time_s),
            )
            start = [float(value) for value in query["start"]]
            goal = [float(value) for value in query["goal"]]
            audit_passed, audit_s, audit_status = audit_path(
                robot,
                list(scene.obstacles),
                path,
                float(args.audit_segment_step),
                start=start,
                goal=goal,
                collision_tolerance=float(args.audit_collision_tolerance),
            )
            audit_total_s += audit_s
            ok = bool(checkpoint.get("ok")) and audit_passed
            label = f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}"
            length = path_length(path) if ok else math.nan
            current_row = {
                "label": label,
                "success": ok,
                "audit_passed": audit_passed,
                "audit_status": audit_status,
                "query_ms": (elapsed_s + simplify_s) * 1000.0,
                "solve_ms": solve_s * 1000.0,
                "simplify_ms": simplify_s * 1000.0,
                "audit_ms": audit_s * 1000.0,
                "path_length": length,
                "segment_fraction": 0.0 if ok else math.nan,
                "waypoint_count": len(path),
                "simplify_status": simplify_status,
                "incumbent_checkpoint_s": float(target_checkpoint_s) if ok else math.nan,
            }
            best = best_by_query.get(label)
            if ok and math.isfinite(length) and (best is None or length < float(best.get("path_length", math.inf))):
                best_by_query[label] = dict(current_row)
                best = best_by_query[label]
            if best is None:
                qrows.append(current_row)
            else:
                row = dict(best)
                row["query_ms"] = (elapsed_s + float(row.get("simplify_ms", 0.0)) / 1000.0) * 1000.0
                row["solve_ms"] = solve_s * 1000.0
                qrows.append(row)
        rows.append(summarize_query_batch_method(
            "bitstar",
            robot_name,
            difficulty,
            scene_seed,
            scene,
            qrows,
            audit_s=audit_total_s,
            diagnostics={
                "planner": "OMPL_BITstar_trace",
                "timeout_s": float(timeout_s),
                "checkpoint_interval_s": float(checkpoint_interval_s),
                "checkpoint_grid_s": checkpoint_grid_s,
                "checkpoint_s": checkpoint_s,
                "target_checkpoint_s": float(target_checkpoint_s),
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
                "simplify_time_s": float(args.ompl_simplify_time_s),
            },
            stage_id=(
                f"batch{int(args.bitstar_samples_per_batch)}"
                f"_rw{float(args.bitstar_rewire_factor):g}"
                f"_trace{float(timeout_s):g}s_t{target_checkpoint_s:g}s"
            ),
            budget_s=float(target_checkpoint_s),
        ))
    return rows


def run_baseline_scene(args: argparse.Namespace, catalog: dict[str, Any], method: str, robot: str, difficulty: str, seed: int, budget_s: float | None = None) -> dict[str, Any]:
    if method == "rrtconnect":
        row = run_rrtconnect_scene(args, catalog, robot, difficulty, seed)
        row["stage_id"] = f"timeout{float(args.rrt_timeout_s):g}s"
        row["budget_s"] = float(args.rrt_timeout_s)
        return row
    if method == "prm":
        return run_prm_scene(args, catalog, robot, difficulty, seed, float(budget_s if budget_s is not None else args.prm_build_s))
    if method == "bitstar":
        return run_bitstar_scene(args, catalog, robot, difficulty, seed, float(budget_s if budget_s is not None else args.bitstar_timeout_s))
    return {
        "method": method,
        "robot": robot,
        "difficulty": difficulty,
        "scene_seed": int(seed),
        "deep_max_boxes": 0,
        "stage_id": method,
        "budget_s": math.nan,
        "status": "external_pending",
        "success_count": 0,
        "query_count": 1,
        "planning_s": math.nan,
        "audit_s": math.nan,
        "path_length_mean": math.nan,
        "raw_segment_fraction": math.nan,
        "final_boxes": math.nan,
        "diagnostics": {"reason": "IRIS/GCS backend is not executed by this self-contained runner."},
    }


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    def timeout_cap(row: dict[str, Any]) -> float:
        diagnostics = row.get("diagnostics", {})
        if isinstance(diagnostics, dict):
            try:
                value = float(diagnostics.get("timeout_s", math.nan))
            except (TypeError, ValueError):
                value = math.nan
            if math.isfinite(value):
                return value
        return math.nan

    out: list[dict[str, Any]] = []
    keys = sorted({
        (
            row["method"],
            row["robot"],
            row["difficulty"],
            str(row.get("stage_id", "")),
            int(row.get("deep_max_boxes", 0) or 0),
        )
        for row in rows
    })
    for method, robot, difficulty, stage_id, budget in keys:
        items = [
            row for row in rows
            if row["method"] == method
            and row["robot"] == robot
            and row["difficulty"] == difficulty
            and str(row.get("stage_id", "")) == stage_id
            and int(row.get("deep_max_boxes", 0) or 0) == budget
        ]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 1))]
        success_queries = sum(int(row.get("success_count", 0)) for row in items)
        total_queries = sum(int(row.get("query_count", 0)) for row in items)
        out.append(
            {
                "method": method,
                "robot": robot,
                "difficulty": difficulty,
                "stage_id": stage_id,
                "leaf_start_depth": median(row.get("leaf_start_depth", math.nan) for row in items),
                "leaf_max_depth": median(row.get("leaf_max_depth", math.nan) for row in items),
                "deep_ffb_depth": median(row.get("deep_ffb_depth", math.nan) for row in items),
                "connector_pave_depth": median(row.get("connector_pave_depth", math.nan) for row in items),
                "query_bridge_pave_depth": median(row.get("query_bridge_pave_depth", math.nan) for row in items),
                "ffb_start_depth": median(row.get("ffb_start_depth", math.nan) for row in items),
                "rbf_max_depth": median(row.get("rbf_max_depth", math.nan) for row in items),
                "budget_s": median(row.get("budget_s", math.nan) for row in items),
                "timeout_cap_s": median(timeout_cap(row) for row in items) if method == "bitstar" else math.nan,
                "deep_max_boxes": budget,
                "scenes": len(items),
                "success_scenes": len(success_items),
                "queries_per_scene": median(row.get("query_count", 0) for row in items),
                "success_queries": success_queries,
                "total_queries": total_queries,
                "obstacles_median": median(row.get("obstacle_count", math.nan) for row in items),
                "measured_time_s_median": median(row.get("planning_s", math.nan) for row in items),
                "planning_s_median": median(row.get("planning_s", math.nan) for row in items),
                "offline_build_s_median": median(row.get("offline_build_s", row.get("build_s", 0.0)) for row in items),
                "online_batch_s_median": median(row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0))) for row in items),
                "online_total_s_median": median(row.get("online_total_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) for row in items),
                "online_solve_s_median": median(row.get("online_solve_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) for row in items),
                "online_simplify_s_median": median(row.get("online_simplify_s", 0.0) for row in items),
                "online_solve_per_query_s_median": median(
                    row.get(
                        "online_solve_per_query_s",
                        row.get("online_solve_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "online_simplify_per_query_s_median": median(
                    row.get(
                        "online_simplify_per_query_s",
                        row.get("online_simplify_s", 0.0) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "online_per_query_s_median": median(
                    row.get(
                        "online_per_query_s",
                        row.get("online_solve_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0))) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "online_total_per_query_s_median": median(
                    row.get(
                        "online_total_per_query_s",
                        row.get("online_total_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "amortized_s_k1": median(row.get("amortized_s_k1", row.get("planning_s", math.nan)) for row in items),
                "amortized_s_k5": median(row.get("amortized_s_k5", row.get("planning_s", math.nan) / 5.0) for row in items),
                "amortized_s_k10": median(row.get("amortized_s_k10", row.get("planning_s", math.nan) / 10.0) for row in items),
                "amortized_s_k20": median(row.get("amortized_s_k20", row.get("planning_s", math.nan) / 20.0) for row in items),
                "amortized_s_k50": median(row.get("amortized_s_k50", row.get("planning_s", math.nan) / 50.0) for row in items),
                "audit_s_median": median(row.get("audit_s", math.nan) for row in items),
                "path_length_mean": mean(row.get("path_length_mean", math.nan) for row in success_items),
                "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in success_items),
                "final_boxes_median": median(row.get("final_boxes", math.nan) for row in items),
                "status": "executed",
            }
        )
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "method",
        "robot",
        "difficulty",
        "stage_id",
        "leaf_start_depth",
        "leaf_max_depth",
        "deep_ffb_depth",
        "connector_pave_depth",
        "query_bridge_pave_depth",
        "ffb_start_depth",
        "rbf_max_depth",
        "budget_s",
        "timeout_cap_s",
        "deep_max_boxes",
        "scenes",
        "success_scenes",
        "queries_per_scene",
        "success_queries",
        "total_queries",
        "obstacles_median",
        "measured_time_s_median",
        "planning_s_median",
        "offline_build_s_median",
        "online_batch_s_median",
        "online_total_s_median",
        "online_solve_s_median",
        "online_simplify_s_median",
        "online_solve_per_query_s_median",
        "online_simplify_per_query_s_median",
        "online_per_query_s_median",
        "online_total_per_query_s_median",
        "amortized_s_k1",
        "amortized_s_k5",
        "amortized_s_k10",
        "amortized_s_k20",
        "amortized_s_k50",
        "audit_s_median",
        "path_length_mean",
        "raw_segment_fraction_median",
        "final_boxes_median",
        "status",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def select_best_tradeoff_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Select one readable row per robot/difficulty; full budget curves stay in the figure/CSV."""
    def path_stat(row: dict[str, Any]) -> float:
        for key in ("path_length_mean", "path_length_median"):
            try:
                value = float(row.get(key, math.nan))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                return value
        return math.nan

    out: list[dict[str, Any]] = []
    keys = sorted({(str(row.get("robot", "")), str(row.get("difficulty", ""))) for row in rows})
    for robot, difficulty in keys:
        items = [row for row in rows if str(row.get("robot", "")) == robot and str(row.get("difficulty", "")) == difficulty]
        full = [
            row for row in items
            if int(float(row.get("success_scenes", 0) or 0)) == int(float(row.get("scenes", 0) or 0))
        ]
        candidates = full or items
        if not candidates:
            continue
        finite_path = [
            path_stat(row)
            for row in candidates
            if math.isfinite(path_stat(row))
        ]
        if finite_path:
            best_path = min(finite_path)
            candidates = [
                row for row in candidates
                if math.isfinite(path_stat(row))
                and path_stat(row) <= 1.08 * best_path
            ] or candidates
        out.append(sorted(
            candidates,
            key=lambda row: (
                float(row.get("online_per_query_s_median", math.nan)) if math.isfinite(float(row.get("online_per_query_s_median", math.nan))) else 1e9,
                (
                    float(row.get("offline_build_s_median", row.get("build_s", 0.0))) / 10.0
                    + float(row.get("online_per_query_s_median", math.nan))
                    if math.isfinite(float(row.get("online_per_query_s_median", math.nan)))
                    else 1e9
                ),
                int(float(row.get("deep_max_boxes", 0) or 0)),
            ),
        )[0])
    return out


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    rows = select_best_tradeoff_rows(rows)
    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Saved-catalog random-scene reusable-planner best trade-off points. Each obstacle scene stores fixed multi-query batches; RBF builds once per scene and online timings reuse the offline forest. Online/q excludes final simplification; Simplify/q reports the measured cost under the globally fixed 0.01~s OMPL post-processing budget. Times exclude final audit; full box-budget and amortization curves are shown in Fig.~\ref{fig:tro_random_tradeoff}.}",
        r"\label{tab:tro-random-summary}",
        r"\footnotesize",
        r"\begin{tabular}{llrrrrrrrrr}",
        r"\toprule",
        r"Robot & Difficulty & Boxes & SR & Build & Online/q & Simplify/q & Amort@10 & Path & Seg. \\",
        r"\midrule",
    ]
    for row in rows:
        sr = f"{int(row.get('success_queries', 0))}/{int(row.get('total_queries', 0))}"
        lines.append(
            f"{row.get('robot')} & {row.get('difficulty')} & {int(row.get('deep_max_boxes', 0) or 0)} & {sr} & "
            f"{tex_num(row.get('offline_build_s_median'))} & {tex_num(row.get('online_per_query_s_median'))} & "
            f"{tex_num(row.get('online_simplify_per_query_s_median'))} & "
            f"{tex_num(float(row.get('offline_build_s_median', row.get('build_s', 0.0))) / 10.0 + float(row.get('online_per_query_s_median', 0.0)))} & "
            f"{tex_num(row.get('path_length_mean', row.get('path_length_median')))} & "
            f"{tex_num(row.get('raw_segment_fraction_median'))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    configure_thread_environment(int(args.threads))
    scene_seeds = 1 if args.phase == "smoke" else (8 if args.phase == "paper" else int(args.scene_seeds))
    robots = csv_list(args.robots)
    difficulties = csv_list(args.difficulties)
    methods = csv_list(args.methods)
    box_budgets = [int(item) for item in csv_list(args.box_budgets)] if str(args.box_budgets).strip() else rbf_budget_grid(args.phase)
    prm_build_grid_s = [float(item) for item in csv_list(args.prm_build_grid_s)] if str(args.prm_build_grid_s).strip() else [float(args.prm_build_s)]
    bitstar_explicit_grid = [float(item) for item in csv_list(args.bitstar_timeout_grid_s)] if str(args.bitstar_timeout_grid_s).strip() else []
    bitstar_checkpoint_grid = [float(item) for item in csv_list(args.bitstar_checkpoint_grid_s)] if str(args.bitstar_checkpoint_grid_s).strip() else bitstar_explicit_grid
    bitstar_trace_timeout_s = max(bitstar_checkpoint_grid or bitstar_explicit_grid) if (bitstar_checkpoint_grid or bitstar_explicit_grid) else float(args.bitstar_timeout_s)
    bitstar_stage_s = bitstar_checkpoint_grid_from_args(args, bitstar_trace_timeout_s)
    args.bitstar_checkpoint_interval_s = bitstar_trace_interval_for_grid(args, bitstar_stage_s, bitstar_trace_timeout_s)
    if args.phase == "smoke":
        robots = robots[:1]
        difficulties = difficulties[:1]
        box_budgets = [int(args.deep_max_boxes)]
        prm_build_grid_s = [min(float(args.prm_build_s), 0.25)]
        bitstar_trace_timeout_s = min(float(args.bitstar_timeout_s), 0.25)
        bitstar_stage_s = [bitstar_trace_timeout_s]
        args.queries_per_scene = min(int(args.queries_per_scene), 3)
    rbf_profile = effective_rbf_profile(args, box_budgets)
    catalog_path = args.scene_catalog or (args.out_dir / "random_scene_catalog_v6.json")
    catalog_summary: dict[str, Any] = {
        "path": str(catalog_path),
        "mode": args.scene_catalog_mode,
        "records": None,
        "queries_per_scene": int(args.queries_per_scene),
    }
    catalog: dict[str, Any] | None = None
    cache_rows: list[dict[str, Any]] = []
    if "sbf_leaf_rrt" in methods:
        for robot in progress(robots, desc="exp06 lect cache", total=len(robots), disable=bool(args.dry_run or args.skip_lect_cache_ensure)):
            cache_rows.append(
                ensure_robot_lectdb_cache(
                    robot,
                    cache_root=Path(args.lect_cache_root),
                    threads=int(args.threads),
                    dry_run=bool(args.dry_run or args.skip_lect_cache_ensure),
                )
            )
    if not args.dry_run:
        catalog = generate_catalog(
            path=catalog_path,
            robots=robots,
            difficulties=difficulties,
            scene_seeds=scene_seeds,
            scene_profile=args.scene_profile,
            seed_base=int(args.seed_base),
            queries_per_scene=int(args.queries_per_scene),
            max_scene_tries=int(args.max_scene_tries),
            mode=args.scene_catalog_mode,
        )
        catalog_summary.update({
            "schema": catalog.get("schema"),
            "records": len(catalog.get("records", [])),
            "queries_per_scene": int(catalog.get("queries_per_scene", args.queries_per_scene)),
        })
    rows = []
    for method in METHODS:
        if method not in methods:
            continue
        if method == "sbf_leaf_rrt":
            budgets: list[Any] = box_budgets
        elif method == "prm":
            budgets = prm_build_grid_s
        elif method == "bitstar":
            budgets = [bitstar_trace_timeout_s]
        elif method == "rrtconnect":
            budgets = [float(args.rrt_timeout_s)]
        else:
            budgets = [None]
        for robot in robots:
            for difficulty in difficulties:
                for seed in range(scene_seeds):
                    for budget in budgets:
                        stage_id = (
                            f"b{int(budget)}" if method == "sbf_leaf_rrt"
                            else (
                                f"{str(args.prm_planner_kind)}_build{float(budget):g}s"
                                f"_k{int(args.prm_max_nearest_neighbors)}"
                                f"_q{float(args.prm_query_s):g}s"
                                f"_preload{int(bool(args.prm_preload_query_endpoints))}"
                            ) if method == "prm"
                            else (
                                f"batch{int(args.bitstar_samples_per_batch)}"
                                f"_rw{float(args.bitstar_rewire_factor):g}"
                                f"_trace{float(budget):g}s_dt{float(args.bitstar_checkpoint_interval_s):g}s"
                            ) if method == "bitstar"
                            else f"timeout{float(args.rrt_timeout_s):g}s" if method == "rrtconnect"
                            else method
                        )
                        rows.append({
                            "method": method,
                            "robot": robot,
                            "difficulty": difficulty,
                            "scene_seed": seed,
                            "stage_id": stage_id,
                            "budget_s": float(budget) if method != "sbf_leaf_rrt" and budget is not None else None,
                            "deep_max_boxes": budget if method == "sbf_leaf_rrt" else 0,
                            "scene_catalog": str(catalog_path),
                            "queries_per_scene": int(args.queries_per_scene),
                            "audit_segment_step": float(args.audit_segment_step),
                            "audit_collision_tolerance": float(args.audit_collision_tolerance),
                            "active_planning_root": "full_robot_joint_limits",
                            "coverage_root": "full_robot_joint_limits",
                            "canonical_mapping_scope": "LECT_internal_only",
                            "status": "planned" if args.dry_run else "planned_for_execution",
                            "rbf_default_profile": copy.deepcopy(rbf_profile) if method == "sbf_leaf_rrt" else None,
                            "rbf_robot_lectdb": robot_lectdb_profile(robot) if method == "sbf_leaf_rrt" else None,
                            "rbf_box_budgets": box_budgets if method == "sbf_leaf_rrt" else None,
                            "ompl_registered_profile": "exp05_registered" if method in {"prm", "bitstar"} else None,
                            "ompl_simplify_time_s": float(args.ompl_simplify_time_s) if method in {"prm", "bitstar", "rrtconnect"} else None,
                            "prm_config": {
                                "build_s": float(budget) if method == "prm" else None,
                                "query_s": float(args.prm_query_s),
                                "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
                                "planner_kind": str(args.prm_planner_kind),
                                "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
                            } if method == "prm" else None,
                            "bitstar_config": {
                                "timeout_s": float(budget) if method == "bitstar" else None,
                                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                                "checkpoint_grid_s": bitstar_stage_s,
                                "checkpoint_stage_s": bitstar_stage_s,
                                "samples_per_batch": int(args.bitstar_samples_per_batch),
                                "rewire_factor": float(args.bitstar_rewire_factor),
                                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
                            } if method == "bitstar" else None,
                            "metrics": ["success_rate", "planning_s", "audit_s", "path_length", "raw_segment_fraction"],
                        })
    run_rows: list[dict[str, Any]] = []
    if not args.dry_run and catalog is not None:
        for row in progress(rows, desc="exp06 runs", total=len(rows)):
            if row["method"] == "sbf_leaf_rrt":
                print(f"[exp06] method=sbf_leaf_rrt robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']} budget={row['deep_max_boxes']}", flush=True)
                args.deep_max_boxes = int(row["deep_max_boxes"])
                run_rows.append(run_rbf_scene(args, catalog, str(row["robot"]), str(row["difficulty"]), int(row["scene_seed"])))
            elif row["method"] == "bitstar":
                print(f"[exp06] method=bitstar stage={row.get('stage_id')} robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']}", flush=True)
                run_rows.extend(run_bitstar_scene_trace(
                    args,
                    catalog,
                    str(row["robot"]),
                    str(row["difficulty"]),
                    int(row["scene_seed"]),
                    float(row["budget_s"]) if row.get("budget_s") is not None else float(args.bitstar_timeout_s),
                    float(args.bitstar_checkpoint_interval_s),
                    bitstar_stage_s,
                ))
            else:
                print(f"[exp06] method={row['method']} stage={row.get('stage_id')} robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']}", flush=True)
                run_rows.append(run_baseline_scene(
                    args,
                    catalog,
                    str(row["method"]),
                    str(row["robot"]),
                    str(row["difficulty"]),
                    int(row["scene_seed"]),
                    float(row["budget_s"]) if row.get("budget_s") is not None else None,
                ))
    summary_rows = aggregate(run_rows) if run_rows else []
    payload: dict[str, Any] = {
        "experiment": "exp06_random_robot",
        "run_id": run_id("exp06"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "thread_policy": {
            "threads": int(args.threads),
            "scope": "RBF, LECT cache prewarm, and process math libraries use --threads=8 by default; OMPL RRTConnect, PRM, and BIT* are algorithmically single-thread in this runner unless OMPL exposes planner-internal parallelism.",
        },
        "rbf_default_profile": rbf_profile,
        "audit": {
            "segment_step": float(args.audit_segment_step),
            "collision_tolerance": float(args.audit_collision_tolerance),
            "tolerance_policy": "strict_zero_tolerance",
        },
        "ompl_baseline_profile": {
            "inherits_from": "Exp.5 registered Shelf+IIWA OMPL baseline profile",
            "simplify_time_s": float(args.ompl_simplify_time_s),
            "prm": {
                "build_grid_s": prm_build_grid_s,
                "query_s": float(args.prm_query_s),
                "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
                "planner_kind": str(args.prm_planner_kind),
                "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
            },
            "bitstar": {
                "stage_s": bitstar_stage_s,
                "timeout_s": float(bitstar_trace_timeout_s),
                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                "checkpoint_grid_s": bitstar_stage_s,
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
            },
        },
        "lectdb_caches": cache_rows,
        "scene_catalog": catalog_summary,
        "planned_rows": rows,
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "random_robot_manifest.json", payload)
    if summary_rows:
        write_csv(args.out_dir / "random_robot_summary.csv", summary_rows)
        if str(args.phase) == "paper" and any(str(row.get("method")) == "sbf_leaf_rrt" for row in summary_rows):
            write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_random_summary.tex", summary_rows)
    print(f"wrote {args.out_dir / 'random_robot_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
