#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (
    REPO_ROOT / "build-leaf-sweep" / "python",
    REPO_ROOT / "build" / "python",
):
    if candidate.exists() and str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))
if str(REPO_ROOT.parent) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT.parent))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, write_json  # noqa: E402
from experiments.common import random_scene_catalog as scene_sampling  # noqa: E402
from experiments.exp04_shelf_ablation import run_leaf_refine_ablation as exp04  # noqa: E402
from experiments.exp04_shelf_ablation.run_leaf_refine_ablation import AblationRow  # noqa: E402


sbf = exp04.sbf
DEFAULT_OUT = REPO_ROOT / "outputs" / "new_experiments" / "exp07_dynamic_update"
DEFAULT_TABLE = REPO_ROOT / "paper" / "generated" / "tab_tro_dynamic_update.tex"
SCENE_CATALOG_SCHEMA = "tro2026_dynamic_update_d23_scene_catalog_v1"


@dataclass(frozen=True)
class Transition:
    name: str
    kind: str
    source_stage: str
    target_stage: str
    edit_group: str
    edit_obstacles: int


RANDOM_OBSTACLE_COUNTS = dict(scene_sampling.RANDOM_OBSTACLE_COUNTS)
RANDOM_OBSTACLE_SCALES = dict(scene_sampling.RANDOM_OBSTACLE_SCALES)
RANDOM_DIFFICULTIES = tuple(scene_sampling.RANDOM_DIFFICULTY_ORDER)

TRANSITIONS = [
    Transition("easy->medium", "insert", "easy", "medium", "medium_suffix", 4),
    Transition("medium->hard", "insert", "medium", "hard", "hard_suffix", 4),
    Transition("hard->medium", "delete", "hard", "medium", "hard_suffix", 4),
    Transition("medium->easy", "delete", "medium", "easy", "medium_suffix", 4),
]


def finite(values: list[float]) -> list[float]:
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def median(values: list[float]) -> float | None:
    vals = finite(values)
    return float(statistics.median(vals)) if vals else None


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "NA"
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return "NA"
    return f"{x:.{digits}f}"


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def d23_root_intervals() -> list[Any]:
    intervals = []
    for item in str(exp04.profile.D23_ROOT_INTERVALS).split(";"):
        lo, hi = item.split(":")
        intervals.append(sbf.Interval(float(lo), float(hi)))
    return intervals


def sample_d23_q(robot: Any, rng: random.Random) -> list[float]:
    q = [rng.uniform(float(interval.lo), float(interval.hi)) for interval in d23_root_intervals()]
    return list(sbf.canonicalize_configuration_for_robot(robot, q, True, "joint_symmetry_native_v1"))


def sample_d23_pair(robot: Any,
                    rng: random.Random,
                    min_l2: float,
                    max_tries: int) -> tuple[list[float], list[float]]:
    start: list[float] | None = None
    for _ in range(max_tries):
        q = sample_d23_q(robot, rng)
        if not sbf.check_config_collision(robot, [], q):
            start = q
            break
    if start is None:
        raise RuntimeError("could not sample d23-root start")
    for _ in range(max_tries):
        goal = sample_d23_q(robot, rng)
        if sbf.check_config_collision(robot, [], goal):
            continue
        dist = math.sqrt(sum((a - b) * (a - b) for a, b in zip(start, goal)))
        if dist >= min_l2:
            return start, goal
    raise RuntimeError("could not sample d23-root goal")


def append_constrained_obstacle(robot: Any,
                                obstacles: list[Any],
                                start: list[float],
                                goal: list[float],
                                rng: random.Random,
                                base: float,
                                require_direct_blocker: bool,
                                max_attempts: int) -> bool:
    for _ in range(max_attempts):
        candidate = scene_sampling.random_workspace_obstacle(rng, base)
        if not scene_sampling.obstacle_clears_fixed_robot("iiwa", candidate):
            continue
        proposed = [*obstacles, candidate]
        if not scene_sampling.endpoint_pair_has_clearance(robot, proposed, start, goal):
            continue
        if require_direct_blocker and scene_sampling.segment_is_collision_free(robot, [candidate], start, goal):
            continue
        obstacles.append(candidate)
        return True
    return False


def constrained_prefixes_are_valid(args: argparse.Namespace,
                                   robot: Any,
                                   obstacles: list[Any],
                                   start: list[float],
                                   goal: list[float],
                                   seed: int,
                                   scene_try: int) -> bool:
    for difficulty in RANDOM_DIFFICULTIES:
        prefix = stage_obstacles(difficulty, obstacles)
        if not scene_sampling.obstacles_clear_fixed_robot("iiwa", prefix):
            return False
        if not scene_sampling.endpoint_pair_has_clearance(robot, prefix, start, goal):
            return False
        if scene_sampling.segment_is_collision_free(robot, prefix, start, goal):
            return False
        if str(args.scene_profile) == "balanced":
            difficulty_offset = RANDOM_DIFFICULTIES.index(difficulty) * 104729
            if not scene_sampling.scene_passes_balanced_probe(
                robot, prefix, start, goal, seed + 8191 * scene_try + difficulty_offset):
                return False
    return True


def random_d23_scene(args: argparse.Namespace, seed: int, robot: Any) -> tuple[list[Any], list[float], list[float]]:
    last_error: Exception | None = None
    for scene_try in range(max(1, int(args.max_scene_tries))):
        rng = random.Random(int(args.random_obstacle_seed_offset) + int(seed) + 1000003 * scene_try)
        obstacles: list[Any] = []
        try:
            start, goal = sample_d23_pair(robot, rng, float(args.min_query_l2), int(args.max_query_sample_attempts))
            if not append_constrained_obstacle(
                robot,
                obstacles,
                start,
                goal,
                rng,
                RANDOM_OBSTACLE_SCALES["easy"] * float(args.random_obstacle_scale_multiplier),
                True,
                int(args.max_obstacle_sample_attempts),
            ):
                raise RuntimeError("could not add direct-blocking easy obstacle")
            for difficulty in RANDOM_DIFFICULTIES:
                target = RANDOM_OBSTACLE_COUNTS[difficulty]
                base = RANDOM_OBSTACLE_SCALES[difficulty] * float(args.random_obstacle_scale_multiplier)
                while len(obstacles) < target:
                    if not append_constrained_obstacle(
                        robot,
                        obstacles,
                        start,
                        goal,
                        rng,
                        base,
                        False,
                        int(args.max_obstacle_sample_attempts),
                    ):
                        raise RuntimeError(f"could not fill {difficulty} prefix")
            if not constrained_prefixes_are_valid(args, robot, obstacles, start, goal, int(seed), scene_try):
                continue
            return obstacles, start, goal
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample constrained d23 random scene for seed={seed}: {last_error}")


def stage_obstacles(stage: str, hard_obstacles: list[Any]) -> list[Any]:
    if stage not in RANDOM_OBSTACLE_COUNTS:
        raise ValueError(f"unknown random obstacle stage {stage!r}")
    return list(hard_obstacles[:RANDOM_OBSTACLE_COUNTS[stage]])


def edit_obstacles(name: str, hard_obstacles: list[Any]) -> list[Any]:
    if name == "medium_suffix":
        return list(hard_obstacles[RANDOM_OBSTACLE_COUNTS["easy"]:RANDOM_OBSTACLE_COUNTS["medium"]])
    if name == "hard_suffix":
        return list(hard_obstacles[RANDOM_OBSTACLE_COUNTS["medium"]:RANDOM_OBSTACLE_COUNTS["hard"]])
    raise ValueError(f"unknown edit group {name!r}")


def obstacle_bounds(obstacle: Any) -> list[float]:
    return [float(value) for value in list(obstacle.bounds)]


def obstacle_from_bounds(bounds: list[float]) -> Any:
    return sbf.Obstacle(*[float(value) for value in bounds])


def scene_payload(args: argparse.Namespace,
                  seed: int,
                  obstacles: list[Any],
                  start: list[float],
                  goal: list[float]) -> dict[str, Any]:
    return {
        "schema": SCENE_CATALOG_SCHEMA,
        "seed": int(seed),
        "robot": "iiwa",
        "scene_profile": str(args.scene_profile),
        "start": [float(value) for value in start],
        "goal": [float(value) for value in goal],
        "obstacles": [obstacle_bounds(obstacle) for obstacle in obstacles],
        "difficulty_order": list(RANDOM_DIFFICULTIES),
        "obstacle_counts": RANDOM_OBSTACLE_COUNTS,
        "obstacle_scales": RANDOM_OBSTACLE_SCALES,
        "d23_root_intervals": [
            [float(interval.lo), float(interval.hi)]
            for interval in d23_root_intervals()
        ],
        "constraints": {
            "fixed_robot_clearance_margin_m": float(scene_sampling.FIXED_ROBOT_CLEARANCE_MARGIN_M),
            "endpoint_clearance_margin_m": float(scene_sampling.ENDPOINT_CLEARANCE_MARGIN_M),
            "direct_obstruction_min_obstacles": int(scene_sampling.DIRECT_OBSTRUCTION_MIN_OBSTACLES),
            "direct_obstruction_min_hits_per_obstacle": int(scene_sampling.DIRECT_OBSTRUCTION_MIN_HITS_PER_OBSTACLE),
            "direct_obstruction_min_total_hits": int(scene_sampling.DIRECT_OBSTRUCTION_MIN_TOTAL_HITS),
            "balanced_probe_timeout_ms": float(scene_sampling.BALANCED_PROBE_TIMEOUT_MS),
            "balanced_probe_range": float(scene_sampling.BALANCED_PROBE_RANGE),
            "balanced_probe_segment_step": float(scene_sampling.BALANCED_PROBE_SEGMENT_STEP),
            "sample_domain": "d23_canonical_inverse_root",
        },
    }


def write_scene_artifact(args: argparse.Namespace, scenes: dict[int, dict[str, Any]]) -> None:
    payload = {
        "schema": SCENE_CATALOG_SCHEMA,
        "artifact": "exp07_d23_random_scenes",
        "description": "Constrained random AABB scenes sampled inside the Exp04 d23 canonical inverse root.",
        "scene_profile": str(args.scene_profile),
        "random_obstacle_seed_offset": int(args.random_obstacle_seed_offset),
        "random_obstacle_scale_multiplier": float(args.random_obstacle_scale_multiplier),
        "seeds": sorted(int(seed) for seed in scenes),
        "scenes": [scenes[seed] for seed in sorted(scenes)],
    }
    write_json(args.scenes_out, payload)


def read_scene_artifact(path: Path) -> dict[int, dict[str, Any]]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if payload.get("schema", SCENE_CATALOG_SCHEMA) != SCENE_CATALOG_SCHEMA:
        raise ValueError(f"unsupported Exp07 scene catalog schema in {path}: {payload.get('schema')!r}")
    scenes: dict[int, dict[str, Any]] = {}
    for scene in payload.get("scenes", []):
        scenes[int(scene["seed"])] = scene
    return scenes


def materialize_scene(scene: dict[str, Any]) -> tuple[list[Any], list[float], list[float]]:
    obstacles = [obstacle_from_bounds(list(bounds)) for bounds in scene["obstacles"]]
    start = [float(value) for value in scene["start"]]
    goal = [float(value) for value in scene["goal"]]
    return obstacles, start, goal


def saved_scene_satisfies_update_constraints(args: argparse.Namespace,
                                             robot: Any,
                                             scene: dict[str, Any]) -> bool:
    try:
        obstacles, start, goal = materialize_scene(scene)
    except Exception:
        return False
    if len(obstacles) < RANDOM_OBSTACLE_COUNTS["hard"]:
        return False
    for obstacle in obstacles:
        if not scene_sampling.obstacle_clears_fixed_robot("iiwa", obstacle):
            return False
    for difficulty in RANDOM_DIFFICULTIES:
        prefix = stage_obstacles(difficulty, obstacles)
        if not scene_sampling.obstacles_clear_fixed_robot("iiwa", prefix):
            return False
        if not scene_sampling.endpoint_pair_has_clearance(robot, prefix, start, goal):
            return False
        if scene_sampling.segment_is_collision_free(robot, prefix, start, goal):
            return False
    if str(args.scene_profile) != str(scene.get("scene_profile", args.scene_profile)):
        return False
    return True


def ensure_scenes(args: argparse.Namespace, seeds: list[int], robot: Any) -> dict[int, dict[str, Any]]:
    mode = "generate" if bool(args.regenerate_scenes) else str(args.scene_catalog_mode)
    scenes = {} if mode == "generate" else read_scene_artifact(args.scenes_out)
    missing = [
        seed for seed in seeds
        if seed not in scenes or not saved_scene_satisfies_update_constraints(args, robot, scenes[int(seed)])
    ]
    if missing and mode in {"reuse", "verify"}:
        raise RuntimeError(f"Exp07 scene catalog {args.scenes_out} is missing/invalid for {len(missing)} seeds; first={missing[0]}")
    if missing:
        args.scenes_out.parent.mkdir(parents=True, exist_ok=True)
        for seed in missing:
            obstacles, start, goal = random_d23_scene(args, seed, robot)
            scenes[int(seed)] = scene_payload(args, seed, obstacles, start, goal)
        write_scene_artifact(args, scenes)
    return {int(seed): scenes[int(seed)] for seed in seeds}


def baseline_row(args: argparse.Namespace) -> AblationRow:
    return AblationRow(
        name="exp07_d23_sh_8t_leaf8_14_box200_d28",
        factor="dynamic",
        description="Exp07 dynamic update row using Exp04 fast d23 SupportHull leaf-refine configuration.",
        deep_max_boxes=int(args.deep_max_boxes),
        deep_ffb_depth=int(args.deep_ffb_depth),
        refine_timeout_ms=float(args.refine_timeout_ms),
        leaf_start_depth=int(args.leaf_start_depth),
        leaf_max_depth=int(args.leaf_max_depth),
        envelope="support_hull",
        threads=int(args.threads),
        leaf_threads=int(args.leaf_threads),
        use_external_evidence=True,
        endpoint_evidence_cache=False,
        parallel_virtual_validation=bool(args.parallel_virtual_validation),
        allow_anchor_roots=True,
    )


def configure_case(args: argparse.Namespace,
                   seed: int,
                   case_name: str) -> tuple[Any, Any]:
    row = baseline_row(args)
    robot, _obstacles, _queries, cfg = exp04.configure_forest(args, row, seed, case_name)
    return robot, cfg


def priority_points(robot: Any, start: list[float], goal: list[float]) -> list[list[float]]:
    points: list[list[float]] = []
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        q = [(1.0 - fraction) * a + fraction * b for a, b in zip(start, goal)]
        points.append(list(sbf.canonicalize_configuration_for_robot(robot, q, True, "joint_symmetry_native_v1")))
    return points


def make_refine_config(args: argparse.Namespace) -> Any:
    refine_cfg = sbf.LeafSweepRefineConfig()
    refine_cfg.leaf_start_depth = int(args.leaf_start_depth)
    refine_cfg.leaf_max_depth = int(args.leaf_max_depth)
    refine_cfg.obstacle_cluster_gap = 1000.0
    refine_cfg.use_virtual_topology = True
    refine_cfg.parallel_virtual_validation = bool(args.parallel_virtual_validation)
    refine_cfg.store_group_results = False
    refine_cfg.validation_batch_size = 512
    refine_cfg.leaf_threads = int(args.leaf_threads)
    refine_cfg.deep_max_boxes = int(args.deep_max_boxes)
    refine_cfg.deep_ffb_depth = int(args.deep_ffb_depth)
    refine_cfg.domain_seed_cap = int(args.domain_seed_cap)
    refine_cfg.domain_success_cap = int(args.domain_success_cap)
    refine_cfg.domain_attempt_cap = int(args.domain_attempt_cap)
    refine_cfg.allow_anchor_roots = True
    refine_cfg.refine_timeout_ms = float(args.refine_timeout_ms)
    return refine_cfg


def configure_dynamic_update(cfg: Any, args: argparse.Namespace) -> None:
    dynamic = getattr(cfg, "dynamic_update", None)
    if dynamic is None:
        return
    dynamic.enable_spatial_dirty_region = True
    dynamic.dirty_region_padding = float(args.dirty_region_padding)
    dynamic.dirty_anchor_limit = int(args.dirty_anchor_limit)
    dynamic.dirty_seed_limit = int(args.dirty_seed_limit)
    dynamic.local_regrow_box_limit = int(args.local_regrow_box_limit)
    dynamic.local_regrow_timeout_ms = float(args.local_regrow_timeout_ms)
    dynamic.insertion_leaf_sweep_max_depth = int(args.insertion_leaf_sweep_max_depth)
    dynamic.insertion_leaf_sweep_relative_depth = int(args.insertion_leaf_sweep_relative_depth)
    dynamic.enable_warm_rebuild_fallback = bool(args.enable_warm_rebuild_fallback)
    dynamic.warm_rebuild_on_empty_forest = True
    dynamic.warm_rebuild_on_empty_dirty_region = False
    dynamic.warm_rebuild_dirty_box_threshold = 0
    dynamic.warm_rebuild_dirty_box_fraction = -1.0
    dynamic.warm_rebuild_min_local_boxes_added = -1


def build_forest(args: argparse.Namespace,
                 seed: int,
                 case_name: str,
                 obstacles: list[Any],
                 start: list[float],
                 goal: list[float],
                 cfg: Any,
                 robot: Any) -> tuple[Any, Any, float]:
    forest = sbf.SafeBoxForest(robot, cfg)
    configure_dynamic_update(cfg, args)
    priority = priority_points(robot, start, goal)
    refine_cfg = make_refine_config(args)
    t0 = time.perf_counter()
    build = forest.build_leaf_sweep_refined(obstacles, refine_cfg, priority)
    wall_ms = 1000.0 * (time.perf_counter() - t0)
    return forest, build, wall_ms


def profile_payload(profile: Any) -> dict[str, Any]:
    names = [
        "boxes_before", "boxes_after", "boxes_removed", "boxes_added",
        "raw_boxes_before", "raw_boxes_after", "raw_boxes_removed", "raw_boxes_added",
        "dirty_boxes", "dirty_boxes_used", "dirty_seed_count", "regrow_attempts",
        "bridge_boxes_added", "segment_edges_added", "rrt_segment_edges_added",
        "point_gap_segment_edges_added",
        "adjacency_islands", "collision_cache_boxes_before", "collision_cache_boxes_after",
        "collision_cache_candidates", "collision_cache_promoted",
        "collision_cache_rejected_collision", "collision_cache_rejected_contained",
        "collision_cache_rejected_disconnected",
    ]
    out = {name: int(getattr(profile, name, 0)) for name in names}
    out.update({
        "used_warm_rebuild": bool(getattr(profile, "used_warm_rebuild", False)),
        "fallback_reason": str(getattr(profile, "fallback_reason", "")),
        "dirty_region_ms": float(getattr(profile, "dirty_region_ms", 0.0)),
        "collision_check_ms": float(getattr(profile, "collision_check_ms", 0.0)),
        "regrow_ms": float(getattr(profile, "regrow_ms", 0.0)),
        "warm_rebuild_ms": float(getattr(profile, "warm_rebuild_ms", 0.0)),
        "adjacency_ms": float(getattr(profile, "adjacency_ms", 0.0)),
        "total_ms": float(getattr(profile, "total_ms", 0.0)),
        "diagnostics": {str(key): float(value) for key, value in dict(getattr(profile, "diagnostics", {})).items()},
    })
    return out


def run_query(args: argparse.Namespace,
              robot: Any,
              forest: Any,
              start: list[float],
              goal: list[float]) -> dict[str, Any]:
    if args.skip_query:
        return {"ok": None, "route_length": None, "segment_fraction": None, "queries": []}
    q0 = list(sbf.canonicalize_configuration_for_robot(robot, start, True, "joint_symmetry_native_v1"))
    q1 = list(sbf.canonicalize_configuration_for_robot(robot, goal, True, "joint_symmetry_native_v1"))
    result = forest.query(q0, q1)
    length = float(result.path_length)
    segment = float(result.segment_edge_length)
    fraction = segment / length if length > 1e-12 else float("nan")
    return {
        "ok": bool(result.success) and bool(result.audit_passed),
        "route_length": length if bool(result.success) else float("nan"),
        "segment_fraction": fraction if bool(result.success) else float("nan"),
        "queries": [{
            "name": "random_query",
            "success": bool(result.success),
            "audit_passed": bool(result.audit_passed),
            "path_length": length,
            "segment_edge_length": segment,
            "segment_fraction": fraction,
            "segment_edges_used": int(result.segment_edges_used),
        }],
    }


def run_transition(args: argparse.Namespace,
                   transition: Transition,
                   seed: int,
                   scene: dict[str, Any]) -> dict[str, Any]:
    source_case = f"{transition.name.replace('->', '_to_')}_source_seed{seed}"
    hard_obstacles, start, goal = materialize_scene(scene)
    robot, source_cfg = configure_case(args, seed, source_case)
    configure_dynamic_update(source_cfg, args)
    source_obstacles = stage_obstacles(transition.source_stage, hard_obstacles)
    target_obstacles = stage_obstacles(transition.target_stage, hard_obstacles)
    forest, source_build, source_wall_ms = build_forest(
        args, seed, source_case, source_obstacles, start, goal, source_cfg, robot)

    t0 = time.perf_counter()
    if transition.kind == "insert":
        if hasattr(forest, "add_obstacles_and_rebuild"):
            profiles = [forest.add_obstacles_and_rebuild(edit_obstacles(transition.edit_group, hard_obstacles))]
        else:
            profiles = [forest.add_obstacle_and_rebuild(obstacle) for obstacle in edit_obstacles(transition.edit_group, hard_obstacles)]
    else:
        profiles = [forest.remove_obstacle_suffix_and_regrow(len(target_obstacles))]
    update_wall_ms = 1000.0 * (time.perf_counter() - t0)
    update_profiles = [profile_payload(profile) for profile in profiles]
    update_ms = sum(item["total_ms"] for item in update_profiles)
    update_query = run_query(args, robot, forest, start, goal)
    segment_fallback_used = False
    segment_fallback_ms = 0.0
    if (
        transition.kind == "insert"
        and bool(args.segment_fallback_after_failed_insert)
        and update_query["ok"] is False
        and (hasattr(forest, "connect_update_endpoint_segment_fallback") or hasattr(forest, "connect_update_segment_fallback"))
    ):
        q0 = list(sbf.canonicalize_configuration_for_robot(robot, start, True, "joint_symmetry_native_v1"))
        q1 = list(sbf.canonicalize_configuration_for_robot(robot, goal, True, "joint_symmetry_native_v1"))
        fallback_t0 = time.perf_counter()
        if hasattr(forest, "connect_update_endpoint_segment_fallback"):
            fallback_profile = forest.connect_update_endpoint_segment_fallback(q0, q1)
        else:
            fallback_profile = forest.connect_update_segment_fallback()
        segment_fallback_wall_ms = 1000.0 * (time.perf_counter() - fallback_t0)
        fallback_payload = profile_payload(fallback_profile)
        fallback_payload["diagnostics"]["segment_fallback.wall_ms"] = float(segment_fallback_wall_ms)
        profiles.append(fallback_profile)
        update_profiles.append(fallback_payload)
        segment_fallback_used = True
        segment_fallback_ms = float(fallback_payload["total_ms"])
        update_ms = sum(item["total_ms"] for item in update_profiles)
        update_wall_ms += segment_fallback_wall_ms
        update_query = run_query(args, robot, forest, start, goal)
        if update_query["ok"] is False and hasattr(forest, "bridge_query_known_needed"):
            query_bridge_t0 = time.perf_counter()
            query_bridge_added = int(forest.bridge_query_known_needed(q0, q1))
            query_bridge_wall_ms = 1000.0 * (time.perf_counter() - query_bridge_t0)
            update_wall_ms += query_bridge_wall_ms
            segment_fallback_ms += query_bridge_wall_ms
            update_ms += query_bridge_wall_ms
            fallback_payload["diagnostics"]["segment_fallback.query_bridge_added"] = float(query_bridge_added)
            fallback_payload["diagnostics"]["segment_fallback.query_bridge_wall_ms"] = float(query_bridge_wall_ms)
            fallback_payload["total_ms"] += query_bridge_wall_ms
            update_query = run_query(args, robot, forest, start, goal)

    warm_case = f"{transition.name.replace('->', '_to_')}_warm_seed{seed}"
    warm_robot, warm_cfg = configure_case(args, seed, warm_case)
    configure_dynamic_update(warm_cfg, args)
    warm_forest, warm_build, warm_wall_ms = build_forest(
        args, seed, warm_case, target_obstacles, start, goal, warm_cfg, warm_robot)
    warm_query = run_query(args, warm_robot, warm_forest, start, goal)

    del forest
    del warm_forest
    return {
        "transition": transition.name,
        "kind": transition.kind,
        "source_stage": transition.source_stage,
        "target_stage": transition.target_stage,
        "edit_group": transition.edit_group,
        "edit_obstacles": int(transition.edit_obstacles),
        "seed": int(seed),
        "scene_profile": str(scene.get("scene_profile", "")),
        "start": [float(value) for value in start],
        "goal": [float(value) for value in goal],
        "source_build_ms": float(source_build.total_ms),
        "source_build_wall_ms": float(source_wall_ms),
        "source_final_boxes": int(source_build.profile.final_boxes),
        "source_collision_cache_boxes": int(source_build.diagnostics.get("leaf_refine.collision_cache_boxes", 0.0)),
        "update_ms": float(update_ms),
        "update_wall_ms": float(update_wall_ms),
        "warm_ms": float(warm_build.total_ms),
        "warm_wall_ms": float(warm_wall_ms),
        "warm_final_boxes": int(warm_build.profile.final_boxes),
        "warm_collision_cache_boxes": int(warm_build.diagnostics.get("leaf_refine.collision_cache_boxes", 0.0)),
        "profiles": update_profiles,
        "segment_fallback_used": bool(segment_fallback_used),
        "segment_fallback_ms": float(segment_fallback_ms),
        "boxes_removed": sum(item["boxes_removed"] for item in update_profiles),
        "boxes_added": sum(item["boxes_added"] for item in update_profiles),
        "dirty_boxes": sum(item["dirty_boxes"] for item in update_profiles),
        "dirty_boxes_used": sum(item["dirty_boxes_used"] for item in update_profiles),
        "regrow_attempts": sum(item["regrow_attempts"] for item in update_profiles),
        "bridge_boxes_added": sum(item["bridge_boxes_added"] for item in update_profiles),
        "segment_edges_added": sum(item["segment_edges_added"] for item in update_profiles),
        "cache_before": update_profiles[0]["collision_cache_boxes_before"] if update_profiles else 0,
        "cache_after": update_profiles[-1]["collision_cache_boxes_after"] if update_profiles else 0,
        "cache_candidates": sum(item["collision_cache_candidates"] for item in update_profiles),
        "cache_promoted": sum(item["collision_cache_promoted"] for item in update_profiles),
        "cache_rejected_collision": sum(item["collision_cache_rejected_collision"] for item in update_profiles),
        "cache_rejected_contained": sum(item["collision_cache_rejected_contained"] for item in update_profiles),
        "cache_rejected_disconnected": sum(item["collision_cache_rejected_disconnected"] for item in update_profiles),
        "adjacency_islands": update_profiles[-1]["adjacency_islands"] if update_profiles else 0,
        "update_query_ok": update_query["ok"],
        "update_route_length": update_query["route_length"],
        "update_segment_fraction": update_query["segment_fraction"],
        "update_queries": update_query["queries"],
        "warm_query_ok": warm_query["ok"],
        "warm_route_length": warm_query["route_length"],
        "warm_segment_fraction": warm_query["segment_fraction"],
        "warm_queries": warm_query["queries"],
        "speedup": float(warm_build.total_ms / update_ms) if update_ms > 1e-12 else float("inf"),
    }


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "n": len(rows),
        "ok_updates": sum(1 for row in rows if row.get("update_query_ok") is True),
        "ok_warm": sum(1 for row in rows if row.get("warm_query_ok") is True),
        "update_ms_median": median([row["update_ms"] for row in rows]),
        "warm_ms_median": median([row["warm_ms"] for row in rows]),
        "speedup_median": median([row["speedup"] for row in rows]),
        "boxes_removed_median": median([row["boxes_removed"] for row in rows]),
        "boxes_added_median": median([row["boxes_added"] for row in rows]),
        "cache_candidates_median": median([row["cache_candidates"] for row in rows]),
        "cache_promoted_median": median([row["cache_promoted"] for row in rows]),
        "cache_reject_collision_median": median([row["cache_rejected_collision"] for row in rows]),
        "cache_reject_contained_median": median([row["cache_rejected_contained"] for row in rows]),
        "cache_reject_disconnected_median": median([row["cache_rejected_disconnected"] for row in rows]),
        "regrow_attempts_median": median([row["regrow_attempts"] for row in rows]),
        "bridge_boxes_added_median": median([row["bridge_boxes_added"] for row in rows]),
        "segment_edges_added_median": median([row["segment_edges_added"] for row in rows]),
        "segment_fallback_used": sum(1 for row in rows if row.get("segment_fallback_used")),
        "segment_fallback_ms_median": median([row["segment_fallback_ms"] for row in rows if row.get("segment_fallback_used")]),
        "dirty_boxes_used_median": median([row["dirty_boxes_used"] for row in rows]),
        "adjacency_islands_median": median([row["adjacency_islands"] for row in rows]),
        "update_route_length_median": median([row["update_route_length"] for row in rows if row["update_route_length"] is not None]),
        "update_segment_fraction_median": median([row["update_segment_fraction"] for row in rows if row["update_segment_fraction"] is not None]),
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "transition", "kind", "seed", "update_ms", "warm_ms", "speedup",
        "boxes_removed", "boxes_added", "cache_candidates", "cache_promoted",
        "cache_rejected_collision", "cache_rejected_contained", "cache_rejected_disconnected",
        "regrow_attempts", "bridge_boxes_added", "segment_edges_added",
        "segment_fallback_used", "segment_fallback_ms",
        "dirty_boxes_used", "adjacency_islands",
        "update_query_ok", "warm_query_ok", "update_route_length", "update_segment_fraction",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def tex_num(value: Any, digits: int = 1) -> str:
    if value is None:
        return "--"
    x = float(value)
    if not math.isfinite(x):
        return "--"
    return f"{x:.{digits}f}"


def write_table(path: Path, cases: list[dict[str, Any]]) -> None:
    lines = [
        "\\begin{table*}[t]",
        "\\centering",
        "\\caption{Leaf-sweep--RRT dynamic updates on constrained random AABB scenes sampled inside the Exp.~4 d23 canonical inverse-root domain. Update time excludes final query audit; warm is a fresh target-scene leaf-refine rebuild from the same read-only d23 evidence cache.}",
        "\\label{tab:tro_dynamic_rebuild}",
        "\\footnotesize",
        "\\setlength{\\tabcolsep}{3.0pt}",
        "\\renewcommand{\\arraystretch}{0.98}",
        "\\resizebox{\\textwidth}{!}{%",
        "\\begin{tabular}{@{}llrrrrrrrrrrrr@{}}",
        "\\toprule",
        "Transition & Edit & Update (ms) & Warm (ms) & Speedup & Invalid & Promoted & Cache cand. & Cache rej. & Repair boxes & Fallback & Islands & Seg. & Ok \\\\",
        "\\midrule",
    ]
    for case in cases:
        agg = case["aggregate"]
        invalid = agg["boxes_removed_median"] if case["kind"] == "insert" else 0.0
        repair = agg["boxes_added_median"]
        cache_rej = sum(float(agg.get(key) or 0.0) for key in (
            "cache_reject_collision_median",
            "cache_reject_contained_median",
            "cache_reject_disconnected_median",
        ))
        ok = f"{int(agg.get('ok_updates', 0))}/{int(agg.get('n', 0))}"
        lines.append(
            f"{case['transition']} & {case['kind']} & "
            f"{tex_num(agg['update_ms_median'], 1)} & {tex_num(agg['warm_ms_median'], 1)} & "
            f"{tex_num(agg['speedup_median'], 2)} & {tex_num(invalid, 0)} & "
            f"{tex_num(agg['cache_promoted_median'], 0)} & {tex_num(agg['cache_candidates_median'], 0)} & "
            f"{tex_num(cache_rej, 0)} & {tex_num(repair + float(agg.get('bridge_boxes_added_median') or 0.0), 0)} & "
            f"{int(agg.get('segment_fallback_used', 0))}/{int(agg.get('n', 0))} & "
            f"{tex_num(agg['adjacency_islands_median'], 0)} & "
            f"{tex_num(agg['update_segment_fraction_median'], 2)} & {ok} \\\\"
        )
    lines += [
        "\\bottomrule",
        "\\end{tabular}",
        "}",
        "\\end{table*}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_markdown(path: Path, cases: list[dict[str, Any]]) -> None:
    lines = [
        "# Exp07 Dynamic Update",
        "",
        "| transition | edit | ok | update ms | warm ms | speedup | invalid | promoted | cache cand | cache rejects | repair boxes | fallback | route | seg frac |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for case in cases:
        agg = case["aggregate"]
        cache_rej = sum(float(agg.get(key) or 0.0) for key in (
            "cache_reject_collision_median",
            "cache_reject_contained_median",
            "cache_reject_disconnected_median",
        ))
        invalid = agg["boxes_removed_median"] if case["kind"] == "insert" else 0.0
        lines.append(
            f"| {case['transition']} | {case['kind']} | {agg['ok_updates']}/{agg['n']} | "
            f"{fmt(agg['update_ms_median'], 1)} | {fmt(agg['warm_ms_median'], 1)} | {fmt(agg['speedup_median'], 2)} | "
            f"{fmt(invalid, 0)} | {fmt(agg['cache_promoted_median'], 0)} | {fmt(agg['cache_candidates_median'], 0)} | "
            f"{fmt(cache_rej, 0)} | {fmt(agg['boxes_added_median'], 0)} | "
            f"{int(agg.get('segment_fallback_used', 0))}/{int(agg.get('n', 0))} | "
            f"{fmt(agg['update_route_length_median'], 3)} | {fmt(agg['update_segment_fraction_median'], 3)} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Exp07 constrained random-obstacle dynamic update study with Exp04 SBF configuration.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--table-out", type=Path, default=DEFAULT_TABLE)
    parser.add_argument("--scenes-out", type=Path, default=DEFAULT_OUT / "scenes" / "d23_random_scenes.json")
    parser.add_argument("--scene-catalog", type=Path, default=None,
                        help="Alias for --scenes-out, used by unified dispatchers.")
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--regenerate-scenes", action="store_true")
    parser.add_argument("--seeds-list", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--only", default="all", help="Comma-separated transition names or all.")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--leaf-threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--leaf-start-depth", type=int, default=8)
    parser.add_argument("--leaf-max-depth", type=int, default=14)
    parser.add_argument("--deep-max-boxes", type=int, default=200)
    parser.add_argument("--deep-ffb-depth", type=int, default=28)
    parser.add_argument("--refine-timeout-ms", type=float, default=800.0)
    parser.add_argument("--connector-pave-depth", type=int, default=28)
    parser.add_argument("--domain-seed-cap", type=int, default=24)
    parser.add_argument("--domain-success-cap", type=int, default=8)
    parser.add_argument("--domain-attempt-cap", type=int, default=24)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--rbf-ffb-start-depth", type=int, default=15)
    parser.add_argument("--rbf-cache-root", type=Path, default=exp04.profile.D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=exp04.profile.D23_CACHE_LABEL)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dirty-region-padding", type=float, default=0.05)
    parser.add_argument("--dirty-anchor-limit", type=int, default=256)
    parser.add_argument("--dirty-seed-limit", type=int, default=512)
    parser.add_argument("--local-regrow-box-limit", type=int, default=64)
    parser.add_argument("--local-regrow-timeout-ms", type=float, default=250.0)
    parser.add_argument("--insertion-leaf-sweep-max-depth", type=int, default=28,
                        help="Global LECT depth used when leaf-sweeping invalidated insertion boxes.")
    parser.add_argument("--insertion-leaf-sweep-relative-depth", type=int, default=-1,
                        help="If >=0, sweep each invalidated insertion box for at most this many relative LECT levels, capped by --insertion-leaf-sweep-max-depth.")
    parser.add_argument("--enable-warm-rebuild-fallback", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--segment-fallback-after-failed-insert", action=argparse.BooleanOptionalAction, default=True,
                        help="After an insertion update fails the query, run the existing segment connector once as a recovery fallback.")
    parser.add_argument("--random-obstacle-seed-offset", type=int, default=710000)
    parser.add_argument("--random-obstacle-scale-multiplier", type=float, default=1.0)
    parser.add_argument("--scene-profile", choices=["constrained", "balanced"], default="balanced")
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--max-query-sample-attempts", type=int, default=2000)
    parser.add_argument("--min-query-l2", type=float, default=0.8)
    parser.add_argument("--max-obstacle-sample-attempts", type=int, default=5000)
    parser.add_argument("--skip-query", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.scene_catalog is not None:
        args.scenes_out = args.scene_catalog
    seeds = parse_int_list(args.seeds_list)
    wanted = {item.strip() for item in str(args.only).split(",") if item.strip()}
    transitions = [t for t in TRANSITIONS if not wanted or "all" in wanted or t.name in wanted or t.kind in wanted]
    if args.dry_run:
        print(json.dumps({
            "seeds": seeds,
            "transitions": [t.__dict__ for t in transitions],
            "out_dir": str(args.out_dir),
            "table_out": str(args.table_out),
            "scenes_out": str(args.scenes_out),
        }, indent=2))
        return 0

    args.out_dir.mkdir(parents=True, exist_ok=True)
    scene_robot, _scene_cfg = configure_case(args, 0, "scene_generation_probe")
    scenes = ensure_scenes(args, seeds, scene_robot)
    rows: list[dict[str, Any]] = []
    cases: list[dict[str, Any]] = []
    for transition in transitions:
        print(f"[exp07] transition={transition.name}", flush=True)
        transition_rows: list[dict[str, Any]] = []
        for seed in seeds:
            print(f"  seed={seed}", flush=True)
            row = run_transition(args, transition, seed, scenes[int(seed)])
            transition_rows.append(row)
            rows.append(row)
        cases.append({
            "transition": transition.name,
            "kind": transition.kind,
            "source_stage": transition.source_stage,
            "target_stage": transition.target_stage,
            "edit_group": transition.edit_group,
            "edit_obstacles": transition.edit_obstacles,
            "rows": transition_rows,
            "aggregate": aggregate(transition_rows),
        })

    payload = {
        "experiment": "exp07_dynamic_update",
        "description": "Constrained random-obstacle dynamic update study using Exp04 d23 SupportHull leaf-refine SBF configuration.",
        "environment": environment_metadata(),
        "config": {
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "deep_max_boxes": int(args.deep_max_boxes),
            "deep_ffb_depth": int(args.deep_ffb_depth),
            "refine_timeout_ms": float(args.refine_timeout_ms),
            "threads": int(args.threads),
            "leaf_threads": int(args.leaf_threads),
            "d23_cache_root": str(args.rbf_cache_root),
            "warm_cache_label": str(args.warm_cache_label),
            "scenes_out": str(args.scenes_out),
            "local_regrow_box_limit": int(args.local_regrow_box_limit),
            "local_regrow_timeout_ms": float(args.local_regrow_timeout_ms),
            "insertion_leaf_sweep_max_depth": int(args.insertion_leaf_sweep_max_depth),
            "insertion_leaf_sweep_relative_depth": int(args.insertion_leaf_sweep_relative_depth),
        },
        "obstacle_protocol": {
            "difficulty_order": list(RANDOM_DIFFICULTIES),
            "obstacle_counts": RANDOM_OBSTACLE_COUNTS,
            "obstacle_scales": RANDOM_OBSTACLE_SCALES,
            "random_obstacle_seed_offset": int(args.random_obstacle_seed_offset),
            "random_obstacle_scale_multiplier": float(args.random_obstacle_scale_multiplier),
            "scene_profile": str(args.scene_profile),
            "constraints": {
                "sample_domain": "d23_canonical_inverse_root",
                "fixed_robot_clearance_margin_m": float(scene_sampling.FIXED_ROBOT_CLEARANCE_MARGIN_M),
                "endpoint_clearance_margin_m": float(scene_sampling.ENDPOINT_CLEARANCE_MARGIN_M),
                "direct_obstruction_min_obstacles": int(scene_sampling.DIRECT_OBSTRUCTION_MIN_OBSTACLES),
                "direct_obstruction_min_hits_per_obstacle": int(scene_sampling.DIRECT_OBSTRUCTION_MIN_HITS_PER_OBSTACLE),
                "direct_obstruction_min_total_hits": int(scene_sampling.DIRECT_OBSTRUCTION_MIN_TOTAL_HITS),
                "balanced_probe_timeout_ms": float(scene_sampling.BALANCED_PROBE_TIMEOUT_MS),
            },
            "transitions": [t.__dict__ for t in transitions],
        },
        "cases": cases,
    }
    write_json(args.out_dir / "dynamic_update_summary.json", payload)
    write_csv(args.out_dir / "dynamic_update_rows.csv", rows)
    write_markdown(args.out_dir / "dynamic_update_summary.md", cases)
    write_table(args.table_out, cases)
    print(f"[exp07] wrote {args.out_dir}", flush=True)
    print(f"[exp07] wrote table {args.table_out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
