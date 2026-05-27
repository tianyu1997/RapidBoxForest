#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common_sbf_config import (  # noqa: E402
    ROOT,
    add_common_sbf_args,
    configure_standalone_sbf,
    mean,
    median,
    query_result_payload,
    sbf,
    set_if_available,
    write_json,
)
from common_scene_sampling import (  # noqa: E402
    DEFAULT_RANDOM_ROBOTS,
    DEFAULT_RANDOM_SCENE_SEEDS,
    RANDOM_DIFFICULTY_ORDER,
    make_random_scene,
    make_robot,
    random_obstacle_count,
)


DEFAULT_SBF_STAGES = "fast:16:0:0:2500:80,balanced:64:256:450:5000:120,quality:128:1024:1500:8000:160,high:512:2000:5000:20000:200"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Random-scene dynamic SBF rebuild runner with few-to-many and many-to-few obstacle sequences."
    )
    add_common_sbf_args(parser)
    parser.set_defaults(
        max_boxes=5000,
        timeout_ms=5000.0,
        ffb_depth=160,
        component_connect_ffb_max_depth=200,
        post_connect_extra_boxes=2000,
        quality_min_connected_boxes=512,
        post_connect_time_budget_ms=5000.0,
        repair_timeout_ms=1500.0,
    )
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--methods", default="support_hull_coverage")
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--stage-order", default="easy,medium,hard")
    parser.add_argument("--sbf-stages", default=DEFAULT_SBF_STAGES)
    parser.add_argument("--sbf-stage", default="balanced", help="SBF stage id from --sbf-stages used for both incremental and warm rebuild timings.")
    parser.add_argument("--cache-root", type=Path, default=ROOT / "outputs" / "paper" / "lect_cache_random_dynamic")
    parser.add_argument("--cache-run-id", default="tro2026_random_dynamic")
    parser.add_argument("--clear-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--directions", default="few_to_many,many_to_few")
    parser.add_argument("--cold-baseline", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--warm-baseline", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--post-update-query", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dirty-region-padding", type=float, default=0.05)
    parser.add_argument("--dirty-anchor-limit", type=int, default=256)
    parser.add_argument("--dirty-seed-limit", type=int, default=512)
    parser.add_argument("--local-regrow-box-limit", type=int, default=64)
    parser.add_argument("--local-regrow-timeout-ms", type=float, default=250.0)
    parser.add_argument("--warm-rebuild-dirty-box-threshold", type=int, default=0)
    parser.add_argument("--warm-rebuild-dirty-box-fraction", type=float, default=-1.0)
    parser.add_argument("--warm-rebuild-min-local-boxes-added", type=int, default=-1)
    parser.add_argument("--warm-rebuild-on-empty-dirty-region", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_exp09_random_dynamic_rebuild.json")
    return parser.parse_args()


def split_csv(text: str) -> list[str]:
    return [item.strip() for item in str(text).split(",") if item.strip()]


def parse_sbf_stages(raw: str) -> list[dict[str, Any]]:
    stages: list[dict[str, Any]] = []
    for index, item in enumerate(split_csv(raw)):
        parts = item.split(":")
        if len(parts) != 6:
            raise ValueError("SBF stage format is label:quality:extra_boxes:post_budget_ms:max_boxes:ffb_depth")
        label, quality, extra_boxes, post_budget_ms, max_boxes, ffb_depth = parts
        stages.append({
            "stage_index": index,
            "stage_id": label,
            "quality_min_connected_boxes": int(quality),
            "post_connect_extra_boxes": int(extra_boxes),
            "post_connect_time_budget_ms": float(post_budget_ms),
            "max_boxes": int(max_boxes),
            "ffb_depth": int(ffb_depth),
        })
    return stages


def select_sbf_stage(args: argparse.Namespace) -> dict[str, Any]:
    stages = parse_sbf_stages(args.sbf_stages)
    requested = str(args.sbf_stage)
    for stage in stages:
        if str(stage["stage_id"]) == requested:
            return stage
    valid = ",".join(str(stage["stage_id"]) for stage in stages)
    raise ValueError(f"unknown --sbf-stage {requested!r}; valid stages: {valid}")


def apply_sbf_stage(args: argparse.Namespace, stage: dict[str, Any]) -> argparse.Namespace:
    args.max_boxes = int(stage["max_boxes"])
    args.ffb_depth = int(stage["ffb_depth"])
    args.quality_min_connected_boxes = int(stage["quality_min_connected_boxes"])
    args.post_connect_extra_boxes = int(stage["post_connect_extra_boxes"])
    args.post_connect_time_budget_ms = float(stage["post_connect_time_budget_ms"])
    return args


def stage_names(args: argparse.Namespace) -> list[str]:
    names = split_csv(args.stage_order)
    if len(names) < 2:
        raise ValueError("--stage-order must contain at least two random-scene difficulties")
    valid = set(RANDOM_DIFFICULTY_ORDER)
    unknown = [name for name in names if name not in valid]
    if unknown:
        raise ValueError(f"unknown random-scene difficulties: {unknown}")
    counts = [random_obstacle_count(name) for name in names]
    if counts != sorted(counts):
        raise ValueError("--stage-order must be monotone from fewer to more obstacles")
    return names


def configure_case(args: argparse.Namespace, method: str, seed: int, robot: Any | None = None) -> sbf.SBFConfig:
    cfg = configure_standalone_sbf(args, seed, preset=method, robot=robot)
    dynamic = getattr(cfg, "dynamic_update", None)
    if dynamic is not None:
        set_if_available(dynamic, "enable_spatial_dirty_region", True)
        set_if_available(dynamic, "dirty_region_padding", float(args.dirty_region_padding))
        set_if_available(dynamic, "dirty_anchor_limit", int(args.dirty_anchor_limit))
        set_if_available(dynamic, "dirty_seed_limit", int(args.dirty_seed_limit))
        set_if_available(dynamic, "local_regrow_box_limit", int(args.local_regrow_box_limit))
        set_if_available(dynamic, "local_regrow_timeout_ms", float(args.local_regrow_timeout_ms))
        set_if_available(dynamic, "enable_warm_rebuild_fallback", True)
        set_if_available(dynamic, "warm_rebuild_on_empty_forest", True)
        set_if_available(dynamic, "warm_rebuild_on_empty_dirty_region", bool(args.warm_rebuild_on_empty_dirty_region))
        set_if_available(dynamic, "warm_rebuild_dirty_box_threshold", int(args.warm_rebuild_dirty_box_threshold))
        set_if_available(dynamic, "warm_rebuild_dirty_box_fraction", float(args.warm_rebuild_dirty_box_fraction))
        set_if_available(dynamic, "warm_rebuild_min_local_boxes_added", int(args.warm_rebuild_min_local_boxes_added))
    return cfg


def configure_warm_case(args: argparse.Namespace, method: str, seed: int, namespace: str, robot: Any | None = None) -> sbf.SBFConfig:
    cfg = configure_case(args, method, seed, robot=robot)
    cfg.database.path = str(args.cache_root / namespace)
    cfg.database.create_if_missing = True
    cfg.database.verify_identity = True
    return cfg


def warm_cache_namespace(args: argparse.Namespace,
                         robot_name: str,
                         method: str,
                         direction: str,
                         from_stage: str,
                         to_stage: str,
                         scene_seed: int) -> str:
    return (
        f"{args.cache_run_id}_{robot_name}_{method}_{args.sbf_stage}_"
        f"{direction}_{from_stage}_to_{to_stage}_seed{int(scene_seed)}"
    )


def cache_metrics(cache_root: Path, namespace: str) -> dict[str, Any]:
    directory = cache_root / namespace
    files = sorted(path for path in directory.rglob("*") if path.is_file()) if directory.exists() else []
    return {
        "warm_cache_namespace": namespace,
        "warm_cache_file_count": len(files),
        "warm_cache_file_bytes": sum(path.stat().st_size for path in files),
    }


def obstacle_bounds(obstacle: sbf.Obstacle) -> list[float]:
    return [float(value) for value in obstacle.bounds]


def prefix_obstacles(obstacles: list[sbf.Obstacle], stage: str) -> list[sbf.Obstacle]:
    return list(obstacles[:random_obstacle_count(stage)])


def profile_value(profile: Any, name: str, default: Any = 0) -> Any:
    return getattr(profile, name, default)


def rebuild_profile_payload(profile: sbf.RebuildProfile) -> dict[str, Any]:
    return {
        "boxes_before": int(profile_value(profile, "boxes_before")),
        "boxes_after": int(profile_value(profile, "boxes_after")),
        "boxes_removed": int(profile_value(profile, "boxes_removed")),
        "raw_boxes_before": int(profile_value(profile, "raw_boxes_before")),
        "raw_boxes_after": int(profile_value(profile, "raw_boxes_after")),
        "raw_boxes_removed": int(profile_value(profile, "raw_boxes_removed")),
        "obstacles_before": int(profile_value(profile, "obstacles_before")),
        "obstacles_after": int(profile_value(profile, "obstacles_after")),
        "dirty_boxes": int(profile_value(profile, "dirty_boxes")),
        "dirty_boxes_used": int(profile_value(profile, "dirty_boxes_used")),
        "dirty_seed_count": int(profile_value(profile, "dirty_seed_count")),
        "regrow_attempts": int(profile_value(profile, "regrow_attempts")),
        "boxes_added": int(profile_value(profile, "boxes_added")),
        "raw_boxes_added": int(profile_value(profile, "raw_boxes_added")),
        "used_spatial_dirty_region": bool(profile_value(profile, "used_spatial_dirty_region", False)),
        "used_warm_rebuild": bool(profile_value(profile, "used_warm_rebuild", False)),
        "fallback_reason": str(profile_value(profile, "fallback_reason", "")),
        "dirty_region_s": float(profile_value(profile, "dirty_region_ms", 0.0)) / 1000.0,
        "regrow_s": float(profile_value(profile, "regrow_ms", 0.0)) / 1000.0,
        "warm_rebuild_s": float(profile_value(profile, "warm_rebuild_ms", 0.0)) / 1000.0,
        "collision_check_s": float(profile_value(profile, "collision_check_ms", 0.0)) / 1000.0,
        "adjacency_s": float(profile_value(profile, "adjacency_ms", 0.0)) / 1000.0,
        "update_s": float(profile_value(profile, "total_ms", 0.0)) / 1000.0,
    }


def aggregate_profiles(profiles: list[sbf.RebuildProfile], wall_s: float) -> dict[str, Any]:
    payloads = [rebuild_profile_payload(profile) for profile in profiles]
    if not payloads:
        return {
            "profiles": [],
            "update_wall_s": float(wall_s),
            "update_profile_s": 0.0,
            "boxes_removed": 0,
            "boxes_added": 0,
            "dirty_boxes": 0,
            "regrow_attempts": 0,
            "used_warm_rebuild": False,
            "warm_rebuild_count": 0,
            "fallback_reasons": [],
        }
    return {
        "profiles": payloads,
        "update_wall_s": float(wall_s),
        "update_profile_s": sum(float(item["update_s"]) for item in payloads),
        "boxes_removed": sum(int(item["boxes_removed"]) for item in payloads),
        "boxes_added": sum(int(item["boxes_added"]) for item in payloads),
        "raw_boxes_removed": sum(int(item["raw_boxes_removed"]) for item in payloads),
        "raw_boxes_added": sum(int(item["raw_boxes_added"]) for item in payloads),
        "dirty_boxes": sum(int(item["dirty_boxes"]) for item in payloads),
        "dirty_boxes_used": sum(int(item["dirty_boxes_used"]) for item in payloads),
        "dirty_seed_count": sum(int(item["dirty_seed_count"]) for item in payloads),
        "regrow_attempts": sum(int(item["regrow_attempts"]) for item in payloads),
        "used_warm_rebuild": any(bool(item["used_warm_rebuild"]) for item in payloads),
        "warm_rebuild_count": sum(1 for item in payloads if item["used_warm_rebuild"]),
        "fallback_reasons": [item["fallback_reason"] for item in payloads if item["fallback_reason"]],
    }


def run_query(args: argparse.Namespace, forest: sbf.SafeBoxForest, label: str, start: list[float], goal: list[float]) -> dict[str, Any] | None:
    if not args.post_update_query:
        return None
    query_t0 = time.perf_counter()
    result = forest.query(start, goal)
    return query_result_payload(label, result, time.perf_counter() - query_t0)


def cold_baseline_rows(args: argparse.Namespace,
                       robot: sbf.Robot,
                       method: str,
                       scene_seed: int,
                       scene: Any,
                       stages: list[str]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not args.cold_baseline:
        return rows
    for stage_index, stage in enumerate(stages):
        cfg = configure_case(args, method, seed=100000 * scene_seed + 997 * stage_index, robot=robot)
        forest = sbf.SafeBoxForest(robot, cfg)
        obstacles = prefix_obstacles(scene.obstacles, stage)
        build_t0 = time.perf_counter()
        profile = forest.build_coverage(obstacles, [scene.start, scene.goal])
        build_s = time.perf_counter() - build_t0
        query = run_query(args, forest, f"cold:{stage}:{scene_seed}", scene.start, scene.goal)
        rows.append({
            "stage": stage,
            "obstacle_count": len(obstacles),
            "build_s": float(build_s),
            "build_profile_s": float(profile.total_ms) / 1000.0,
            "box_count": len(forest.boxes()),
            "segment_edges": len(forest.segment_edges()),
            "query": query,
        })
    return rows


def cold_lookup(rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {str(row["stage"]): row for row in rows}


def transition_key(direction: str, from_stage: str, to_stage: str) -> str:
    return f"{direction}:{from_stage}->{to_stage}"


def warm_lookup(rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {transition_key(str(row["direction"]), str(row["from_stage"]), str(row["to_stage"])): row for row in rows}


def run_warm_rebuild_path(args: argparse.Namespace,
                          robot_name: str,
                          robot: sbf.Robot,
                          method: str,
                          scene_seed: int,
                          scene: Any,
                          stages: list[str],
                          direction: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    order = list(stages) if direction == "few_to_many" else list(reversed(stages))
    initial_stage = order[0]
    initial_obstacles = prefix_obstacles(scene.obstacles, initial_stage)
    initial_build_s = None
    initial_profile = None
    rows: list[dict[str, Any]] = []
    for transition_index, (from_stage, to_stage) in enumerate(zip(order, order[1:])):
        namespace = warm_cache_namespace(args, robot_name, method, direction, from_stage, to_stage, scene_seed)
        seed_base = 200000 * scene_seed + (11 if direction == "few_to_many" else 29) + 1009 * transition_index
        source_cfg = configure_warm_case(args, method, seed_base, namespace, robot=robot)
        source_forest = sbf.SafeBoxForest(robot, source_cfg)
        source_obstacles = prefix_obstacles(scene.obstacles, from_stage)
        source_t0 = time.perf_counter()
        source_profile = source_forest.build_coverage(source_obstacles, [scene.start, scene.goal])
        source_build_s = time.perf_counter() - source_t0
        if transition_index == 0:
            initial_build_s = float(source_build_s)
            initial_profile = source_profile

        target_cfg = configure_warm_case(args, method, seed_base, namespace, robot=robot)
        target_forest = sbf.SafeBoxForest(robot, target_cfg)
        target_obstacles = prefix_obstacles(scene.obstacles, to_stage)
        warm_t0 = time.perf_counter()
        warm_profile = target_forest.build_coverage(target_obstacles, [scene.start, scene.goal])
        warm_s = time.perf_counter() - warm_t0
        query = run_query(args, target_forest, f"warm:{direction}:{from_stage}->{to_stage}:{scene_seed}", scene.start, scene.goal)
        rows.append({
            "robot": robot_name,
            "method": method,
            "scene_seed": scene_seed,
            "direction": direction,
            "from_stage": from_stage,
            "to_stage": to_stage,
            "from_obstacle_count": random_obstacle_count(from_stage),
            "to_obstacle_count": random_obstacle_count(to_stage),
            "warm_source_build_s": float(source_build_s),
            "warm_source_build_profile_s": float(source_profile.total_ms) / 1000.0,
            "warm_source_boxes_after": len(source_forest.boxes()),
            "warm_build_s": float(warm_s),
            "warm_build_profile_s": float(warm_profile.total_ms) / 1000.0,
            "warm_boxes_after": len(target_forest.boxes()),
            "warm_raw_boxes_after": len(target_forest.raw_boxes()),
            "warm_segment_edges_after": len(target_forest.segment_edges()),
            **cache_metrics(args.cache_root, namespace),
            "warm_query": query,
        })
    if initial_build_s is None:
        cfg = configure_warm_case(args, method, 200000 * scene_seed + (11 if direction == "few_to_many" else 29),
                      warm_cache_namespace(args, robot_name, method, direction, initial_stage, initial_stage, scene_seed),
                      robot=robot)
        initial_forest = sbf.SafeBoxForest(robot, cfg)
        build_t0 = time.perf_counter()
        initial_profile = initial_forest.build_coverage(initial_obstacles, [scene.start, scene.goal])
        initial_build_s = time.perf_counter() - build_t0
    initial = {
        "direction": direction,
        "initial_stage": initial_stage,
        "initial_obstacle_count": len(initial_obstacles),
        "initial_build_s": float(initial_build_s),
        "initial_build_profile_s": float(initial_profile.total_ms) / 1000.0,
        "initial_boxes": int(initial_profile.final_boxes),
        "initial_segment_edges": int(initial_profile.segment_edges),
    }
    return initial, rows


def transition_row(args: argparse.Namespace,
                   robot_name: str,
                   method: str,
                   scene_seed: int,
                   direction: str,
                   from_stage: str,
                   to_stage: str,
                   forest: sbf.SafeBoxForest,
                   start: list[float],
                   goal: list[float],
                   profiles: list[sbf.RebuildProfile],
                   wall_s: float,
                   cold: dict[str, dict[str, Any]],
                   warm: dict[str, dict[str, Any]]) -> dict[str, Any]:
    from_count = random_obstacle_count(from_stage)
    to_count = random_obstacle_count(to_stage)
    query = run_query(args, forest, f"{direction}:{from_stage}->{to_stage}:{scene_seed}", start, goal)
    aggregate = aggregate_profiles(profiles, wall_s)
    cold_target = cold.get(to_stage)
    cold_build_s = float(cold_target["build_s"]) if cold_target else None
    warm_target = warm.get(transition_key(direction, from_stage, to_stage))
    warm_build_s = float(warm_target["warm_build_s"]) if warm_target else None
    update_s = float(aggregate["update_wall_s"])
    speedup_vs_cold = cold_build_s / update_s if cold_build_s is not None and update_s > 0.0 else None
    speedup_vs_warm = warm_build_s / update_s if warm_build_s is not None and update_s > 0.0 else None
    return {
        "robot": robot_name,
        "method": method,
        "scene_seed": scene_seed,
        "direction": direction,
        "from_stage": from_stage,
        "to_stage": to_stage,
        "from_obstacle_count": from_count,
        "to_obstacle_count": to_count,
        "obstacles_added": max(0, to_count - from_count),
        "obstacles_removed": max(0, from_count - to_count),
        "boxes_after": len(forest.boxes()),
        "raw_boxes_after": len(forest.raw_boxes()),
        "segment_edges_after": len(forest.segment_edges()),
        "query": query,
        "cold_build_s": cold_build_s,
        "speedup_vs_cold": speedup_vs_cold,
        "warm_build_s": warm_build_s,
        "warm_build_profile_s": warm_target.get("warm_build_profile_s") if warm_target else None,
        "warm_source_build_s": warm_target.get("warm_source_build_s") if warm_target else None,
        "warm_source_build_profile_s": warm_target.get("warm_source_build_profile_s") if warm_target else None,
        "warm_cache_namespace": warm_target.get("warm_cache_namespace") if warm_target else None,
        "warm_cache_file_count": warm_target.get("warm_cache_file_count") if warm_target else None,
        "warm_cache_file_bytes": warm_target.get("warm_cache_file_bytes") if warm_target else None,
        "warm_boxes_after": warm_target.get("warm_boxes_after") if warm_target else None,
        "warm_segment_edges_after": warm_target.get("warm_segment_edges_after") if warm_target else None,
        "warm_query": warm_target.get("warm_query") if warm_target else None,
        "speedup_vs_warm": speedup_vs_warm,
        **aggregate,
    }


def run_few_to_many(args: argparse.Namespace,
                    robot_name: str,
                    robot: sbf.Robot,
                    method: str,
                    scene_seed: int,
                    scene: Any,
                    stages: list[str],
                    cold: dict[str, dict[str, Any]],
                    warm: dict[str, dict[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    cfg = configure_case(args, method, seed=200000 * scene_seed + 11, robot=robot)
    forest = sbf.SafeBoxForest(robot, cfg)
    initial_stage = stages[0]
    initial_obstacles = prefix_obstacles(scene.obstacles, initial_stage)
    build_t0 = time.perf_counter()
    build = forest.build_coverage(initial_obstacles, [scene.start, scene.goal])
    initial_build_s = time.perf_counter() - build_t0
    rows: list[dict[str, Any]] = []
    current_count = len(initial_obstacles)
    for from_stage, to_stage in zip(stages, stages[1:]):
        target_count = random_obstacle_count(to_stage)
        profiles: list[sbf.RebuildProfile] = []
        update_t0 = time.perf_counter()
        for obstacle in scene.obstacles[current_count:target_count]:
            profiles.append(forest.add_obstacle_and_rebuild(obstacle))
        wall_s = time.perf_counter() - update_t0
        current_count = target_count
        row = transition_row(args, robot_name, method, scene_seed, "few_to_many", from_stage, to_stage,
                             forest, scene.start, scene.goal, profiles, wall_s, cold, warm)
        row["initial_stage"] = initial_stage
        row["initial_build_s"] = float(initial_build_s)
        rows.append(row)
    initial = {
        "direction": "few_to_many",
        "initial_stage": initial_stage,
        "initial_obstacle_count": len(initial_obstacles),
        "initial_build_s": float(initial_build_s),
        "initial_build_profile_s": float(build.total_ms) / 1000.0,
        "initial_boxes": int(build.final_boxes),
        "initial_segment_edges": int(build.segment_edges),
    }
    return initial, rows


def run_many_to_few(args: argparse.Namespace,
                    robot_name: str,
                    robot: sbf.Robot,
                    method: str,
                    scene_seed: int,
                    scene: Any,
                    stages: list[str],
                    cold: dict[str, dict[str, Any]],
                    warm: dict[str, dict[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    cfg = configure_case(args, method, seed=200000 * scene_seed + 29, robot=robot)
    forest = sbf.SafeBoxForest(robot, cfg)
    ordered = list(stages)
    initial_stage = ordered[-1]
    initial_obstacles = prefix_obstacles(scene.obstacles, initial_stage)
    build_t0 = time.perf_counter()
    build = forest.build_coverage(initial_obstacles, [scene.start, scene.goal])
    initial_build_s = time.perf_counter() - build_t0
    rows: list[dict[str, Any]] = []
    current_count = len(initial_obstacles)
    reversed_stages = list(reversed(ordered))
    for from_stage, to_stage in zip(reversed_stages, reversed_stages[1:]):
        target_count = random_obstacle_count(to_stage)
        profiles: list[sbf.RebuildProfile] = []
        update_t0 = time.perf_counter()
        if hasattr(forest, "remove_obstacle_suffix_and_regrow"):
            profiles.append(forest.remove_obstacle_suffix_and_regrow(target_count))
            current_count = target_count
        else:
            while current_count > target_count:
                profiles.append(forest.remove_obstacle_and_regrow(current_count - 1))
                current_count -= 1
        wall_s = time.perf_counter() - update_t0
        row = transition_row(args, robot_name, method, scene_seed, "many_to_few", from_stage, to_stage,
                             forest, scene.start, scene.goal, profiles, wall_s, cold, warm)
        row["initial_stage"] = initial_stage
        row["initial_build_s"] = float(initial_build_s)
        rows.append(row)
    initial = {
        "direction": "many_to_few",
        "initial_stage": initial_stage,
        "initial_obstacle_count": len(initial_obstacles),
        "initial_build_s": float(initial_build_s),
        "initial_build_profile_s": float(build.total_ms) / 1000.0,
        "initial_boxes": int(build.final_boxes),
        "initial_segment_edges": int(build.segment_edges),
    }
    return initial, rows


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    keys = sorted({
        (str(row["robot"]), str(row["method"]), str(row["direction"]), str(row["from_stage"]), str(row["to_stage"]))
        for row in rows
    })
    summary: list[dict[str, Any]] = []
    for robot_name, method, direction, from_stage, to_stage in keys:
        subset = [
            row for row in rows
            if row["robot"] == robot_name and row["method"] == method and row["direction"] == direction
            and row["from_stage"] == from_stage and row["to_stage"] == to_stage
        ]
        queries = [row.get("query") for row in subset if row.get("query") is not None]
        update_median_s = median(float(row["update_wall_s"]) for row in subset)
        cold_build_median_s = median(row.get("cold_build_s") for row in subset if row.get("cold_build_s") is not None)
        warm_build_median_s = median(row.get("warm_build_s") for row in subset if row.get("warm_build_s") is not None)
        warm_source_build_median_s = median(row.get("warm_source_build_s") for row in subset if row.get("warm_source_build_s") is not None)
        summary.append({
            "robot": robot_name,
            "method": method,
            "direction": direction,
            "from_stage": from_stage,
            "to_stage": to_stage,
            "from_obstacle_count": random_obstacle_count(from_stage),
            "to_obstacle_count": random_obstacle_count(to_stage),
            "n": len(subset),
            "update_mean_s": mean(float(row["update_wall_s"]) for row in subset),
            "update_median_s": update_median_s,
            "cold_build_median_s": cold_build_median_s,
            "speedup_vs_cold_median": (cold_build_median_s / update_median_s) if cold_build_median_s is not None and update_median_s else None,
            "warm_build_median_s": warm_build_median_s,
            "warm_source_build_median_s": warm_source_build_median_s,
            "speedup_vs_warm_median": (warm_build_median_s / update_median_s) if warm_build_median_s is not None and update_median_s else None,
            "speedup_vs_cold_pair_median": median(row.get("speedup_vs_cold") for row in subset if row.get("speedup_vs_cold") is not None),
            "speedup_vs_warm_pair_median": median(row.get("speedup_vs_warm") for row in subset if row.get("speedup_vs_warm") is not None),
            "warm_boxes_after_mean": mean(float(row["warm_boxes_after"]) for row in subset if row.get("warm_boxes_after") is not None),
            "boxes_after_mean": mean(float(row["boxes_after"]) for row in subset),
            "boxes_removed_mean": mean(float(row["boxes_removed"]) for row in subset),
            "boxes_added_mean": mean(float(row["boxes_added"]) for row in subset),
            "dirty_boxes_mean": mean(float(row["dirty_boxes"]) for row in subset),
            "regrow_attempts_mean": mean(float(row["regrow_attempts"]) for row in subset),
            "warm_rebuild_rate": mean(1.0 if row.get("used_warm_rebuild") else 0.0 for row in subset),
            "query_sr": mean(1.0 if query.get("ok") else 0.0 for query in queries),
            "audit_sr": mean(1.0 if query.get("audit_passed") else 0.0 for query in queries),
            "query_time_median_s": median(float(query.get("t_s", 0.0)) for query in queries),
            "warm_query_sr": mean(
                1.0 if row.get("warm_query", {}).get("ok") else 0.0
                for row in subset
                if row.get("warm_query") is not None
            ),
            "warm_audit_sr": mean(
                1.0 if row.get("warm_query", {}).get("audit_passed") else 0.0
                for row in subset
                if row.get("warm_query") is not None
            ),
        })
    return summary


def run_case(args: argparse.Namespace,
             robot_name: str,
             method: str,
             scene_seed: int,
             stages: list[str],
             directions: list[str]) -> dict[str, Any]:
    hard_stage = stages[-1]
    scene_seed_value = int(args.seed_base) + 1009 * scene_seed
    print(f"[exp9-dynamic] scene robot={robot_name} method={method} seed={scene_seed} profile={args.scene_profile}", flush=True)
    scene = make_random_scene(robot_name, hard_stage, scene_seed_value, scene_profile=args.scene_profile)
    robot = make_robot(robot_name)
    cold_rows = cold_baseline_rows(args, robot, method, scene_seed, scene, stages)
    cold = cold_lookup(cold_rows)
    warm_initials: list[dict[str, Any]] = []
    warm_rows: list[dict[str, Any]] = []
    if args.warm_baseline:
        for direction in directions:
            initial, direction_rows = run_warm_rebuild_path(args, robot_name, robot, method, scene_seed, scene, stages, direction)
            warm_initials.append(initial)
            warm_rows.extend(direction_rows)
    warm = warm_lookup(warm_rows)
    initials: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    if "few_to_many" in directions:
        initial, direction_rows = run_few_to_many(args, robot_name, robot, method, scene_seed, scene, stages, cold, warm)
        initials.append(initial)
        rows.extend(direction_rows)
    if "many_to_few" in directions:
        initial, direction_rows = run_many_to_few(args, robot_name, robot, method, scene_seed, scene, stages, cold, warm)
        initials.append(initial)
        rows.extend(direction_rows)
    return {
        "robot": robot_name,
        "method": method,
        "scene_seed": scene_seed,
        "scene_seed_value": scene_seed_value,
        "start": list(scene.start),
        "goal": list(scene.goal),
        "full_obstacle_count": len(scene.obstacles),
        "full_obstacles": [obstacle_bounds(obstacle) for obstacle in scene.obstacles],
        "cold_baselines": cold_rows,
        "warm_baselines": warm_rows,
        "warm_initial_builds": warm_initials,
        "initial_builds": initials,
        "transitions": rows,
    }


def main() -> int:
    args = parse_args()
    sbf_stage = select_sbf_stage(args)
    args = apply_sbf_stage(args, sbf_stage)
    if args.clear_cache and args.cache_root.exists():
        shutil.rmtree(args.cache_root)
    stages = stage_names(args)
    directions = split_csv(args.directions)
    allowed_directions = {"few_to_many", "many_to_few"}
    unknown_directions = [direction for direction in directions if direction not in allowed_directions]
    if unknown_directions:
        raise ValueError(f"unknown dynamic directions: {unknown_directions}")
    robots = split_csv(args.robots)
    methods = split_csv(args.methods)
    cases: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    for robot_name in robots:
        for method in methods:
            for scene_seed in range(max(1, int(args.scene_seeds))):
                case = run_case(args, robot_name, method, scene_seed, stages, directions)
                cases.append(case)
                rows.extend(case["transitions"])
                for row in case["transitions"]:
                    print(
                        f"[exp9-dynamic] done robot={row['robot']} dir={row['direction']} "
                        f"{row['from_stage']}->{row['to_stage']} update={row['update_wall_s']:.4f}s "
                        f"boxes={row['boxes_after']} audit={row.get('query', {}).get('audit_passed') if row.get('query') else 'skip'}",
                        flush=True,
                    )
    params = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    payload = {
        "schema_version": 4,
        "experiment": "exp09_random_dynamic_rebuild",
        "source_protocol": "balanced_random_scene_two_path_dynamic_vs_warm_rebuild",
        "source_script": str(Path(__file__).resolve()),
        "note": "Uses the balanced random-scene hard instance as the parent scene; easy/medium/hard are 4/8/12-obstacle prefixes. The only dynamic paths are easy->medium->hard and hard->medium->easy. Incremental and warm-rebuild timings use the selected random-anytime SBF stage configuration. Each warm rebuild transition uses an isolated random-scene-style fingerprint-cache namespace: build the source prefix to seed the cache, then time a fresh target-prefix forest from that cache. Summary speedup is warm target median divided by incremental median.",
        "stage_order": stages,
        "stage_obstacle_counts": {stage: random_obstacle_count(stage) for stage in stages},
        "sbf_stage": sbf_stage,
        "sbf_config": {
            "max_boxes": int(args.max_boxes),
            "timeout_ms": float(args.timeout_ms),
            "ffb_depth": int(args.ffb_depth),
            "component_connect_ffb_max_depth": int(args.component_connect_ffb_max_depth),
            "quality_min_connected_boxes": int(args.quality_min_connected_boxes),
            "post_connect_extra_boxes": int(args.post_connect_extra_boxes),
            "post_connect_time_budget_ms": float(args.post_connect_time_budget_ms),
            "repair_timeout_ms": float(args.repair_timeout_ms),
        },
        "directions": directions,
        "params": params,
        "cases": cases,
        "rows": rows,
        "summary": summarize(rows),
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())