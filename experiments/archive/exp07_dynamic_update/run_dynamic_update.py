#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import math
import random
import sys
import time
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (
    DEFAULT_OUTPUT_ROOT,
    configure_thread_environment,
    environment_metadata,
    namespace_dict,
    run_id,
    write_csv as write_csv_rows,
    write_json,
)
from experiments.common.metrics import percentile, tex_num
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import (
    aabb_overlaps,
    make_robot,
    narrow_obstacle_count,
    normalize_obstacles,
    obstacle_bounds,
    obstacle_clears_fixed_robot,
    obstacle_from_bounds,
    random_narrow_workspace_obstacle,
    random_obstacle_count,
    random_obstacle_scale,
    random_workspace_obstacle,
)
from experiments.common.rbf_defaults import (
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_FFB_BINARY_PROBE_DEPTH,
    DEFAULT_RBF_FFB_IMPLEMENTATION,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_MAX_DEPTH,
    ROBOT_LECTDB_CACHE_ROOT,
    default_rbf_profile,
    robot_lectdb_profile,
)
from experiments.common.rbf_leaf_rrt import (
    RBFLeafRRTOptions,
    configure_leaf_rrt,
    make_adaptive_leaf_sweep_config,
)
from experiments.common.robot_lectdb_cache import ensure_robot_lectdb_cache, robot_external_evidence_path
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()

CATALOG_SCHEMA = "tro2026_exp07_ordered_obstacle_update_v1"
DEFAULT_LEAF_MAX_DEPTH = 14
DEFAULT_ADAPTIVE_TARGET_DEPTH = 14
DEFAULT_EXP07_BOX_BUDGET = 200
DEFAULT_EXP07_LOCAL_REGROW_BOX_LIMIT = 200
DEFAULT_MIN_OBSTACLES = 2
DEFAULT_MAX_OBSTACLES = 3


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def finite(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def q1(values: Iterable[Any]) -> float | None:
    return percentile(values, 0.25)


def q3(values: Iterable[Any]) -> float | None:
    return percentile(values, 0.75)


def qrange_ms(values_s: Iterable[Any]) -> str:
    lo = q1(1000.0 * finite(value) for value in values_s)
    hi = q3(1000.0 * finite(value) for value in values_s)
    if lo is None or hi is None:
        return "--"
    return f"\\([{lo:.2f},{hi:.2f}]\\)"


def tex_qrange_ms(lo_s: Any, hi_s: Any) -> str:
    lo = finite(lo_s)
    hi = finite(hi_s)
    if not math.isfinite(lo) or not math.isfinite(hi):
        return "--"
    return f"\\([{1000.0 * lo:.2f},{1000.0 * hi:.2f}]\\)"


def tex_qrange_s(lo_s: Any, hi_s: Any) -> str:
    lo = finite(lo_s)
    hi = finite(hi_s)
    if not math.isfinite(lo) or not math.isfinite(hi):
        return "--"
    return f"\\([{lo:.3f},{hi:.3f}]\\)"


def tex_speedup_range(lo_value: Any, hi_value: Any) -> str:
    lo = finite(lo_value)
    hi = finite(hi_value)
    if not math.isfinite(lo) or not math.isfinite(hi):
        return "--"
    return f"{lo:.1f}--{hi:.1f}$\\times$"


def profile_row(profile: Any) -> dict[str, Any]:
    diagnostics: dict[str, float] = {}
    raw = getattr(profile, "diagnostics", {}) or {}
    try:
        items = dict(raw).items()
    except TypeError:
        items = []
    for key, value in items:
        try:
            diagnostics[str(key)] = float(value)
        except (TypeError, ValueError):
            continue
    return {
        "boxes_before": int(getattr(profile, "boxes_before", 0)),
        "boxes_after": int(getattr(profile, "boxes_after", 0)),
        "boxes_added": int(getattr(profile, "boxes_added", 0)),
        "boxes_removed": int(getattr(profile, "boxes_removed", 0)),
        "dirty_boxes": int(getattr(profile, "dirty_boxes", 0)),
        "dirty_seed_count": int(getattr(profile, "dirty_seed_count", 0)),
        "regrow_attempts": int(getattr(profile, "regrow_attempts", 0)),
        "segment_edges_added": int(getattr(profile, "segment_edges_added", 0)),
        "collision_cache_boxes_before": int(getattr(profile, "collision_cache_boxes_before", 0)),
        "collision_cache_boxes_after": int(getattr(profile, "collision_cache_boxes_after", 0)),
        "collision_cache_candidates": int(getattr(profile, "collision_cache_candidates", 0)),
        "collision_cache_promoted": int(getattr(profile, "collision_cache_promoted", 0)),
        "used_warm_rebuild": bool(getattr(profile, "used_warm_rebuild", False)),
        "fallback_reason": str(getattr(profile, "fallback_reason", "")),
        "dirty_region_ms": float(getattr(profile, "dirty_region_ms", 0.0)),
        "regrow_ms": float(getattr(profile, "regrow_ms", 0.0)),
        "warm_rebuild_ms": float(getattr(profile, "warm_rebuild_ms", 0.0)),
        "total_ms": float(getattr(profile, "total_ms", 0.0)),
        "diagnostics": diagnostics,
    }


def build_result_row(result: Any, wall_s: float) -> dict[str, Any]:
    diagnostics: dict[str, float] = {}
    raw = getattr(result, "diagnostics", {}) or {}
    try:
        items = dict(raw).items()
    except TypeError:
        items = []
    for key, value in items:
        try:
            diagnostics[str(key)] = float(value)
        except (TypeError, ValueError):
            continue
    return {
        "wall_s": float(wall_s),
        "reported_s": float(getattr(result, "total_ms", 0.0)) / 1000.0,
        "leaf_sweep_s": float(getattr(result, "leaf_sweep_ms", 0.0)) / 1000.0,
        "adaptive_s": float(getattr(result, "adaptive_ms", 0.0)) / 1000.0,
        "coverage_probe_s": float(getattr(result, "coverage_probe_ms", 0.0)) / 1000.0,
        "shallow_free_count": int(getattr(result, "shallow_free_count", 0)),
        "shallow_collision_count": int(getattr(result, "shallow_collision_count", 0)),
        "adaptive_free_added": int(getattr(result, "adaptive_free_added", 0)),
        "adaptive_validated": int(getattr(result, "adaptive_validated", 0)),
        "adaptive_splits": int(getattr(result, "adaptive_splits", 0)),
        "adaptive_deferred": int(getattr(result, "adaptive_deferred", 0)),
        "adaptive_promoted": int(getattr(result, "adaptive_promoted", 0)),
        "unresolved_domains": int(getattr(result, "unresolved_domains", 0)),
        "p_box_covered": float(getattr(result, "p_box_covered", math.nan)),
        "p_main_accessible": float(getattr(result, "p_main_accessible", math.nan)),
        "selected_leaf_depth": int(getattr(result, "selected_leaf_depth", -1)),
        "partition_cell_count": int(getattr(result, "partition_cell_count", 0)),
        "partition_grid_cell_count": int(getattr(result, "partition_grid_cell_count", 0)),
        "partition_non_grid_cell_count": int(getattr(result, "partition_non_grid_cell_count", 0)),
        "partition_face_index_entries": int(getattr(result, "partition_face_index_entries", 0)),
        "partition_islands": int(getattr(result, "partition_islands", 0)),
        "partition_largest_island": int(getattr(result, "partition_largest_island", 0)),
        "diagnostics": diagnostics,
    }


def obstacle_to_json(obstacle: Any) -> list[float]:
    return [float(value) for value in obstacle_bounds(obstacle)]


def obstacles_sha256(records: list[dict[str, Any]]) -> str:
    text = repr([
        (record["robot"], record["seed"], record["obstacles"])
        for record in records
    ]).encode("utf-8")
    return hashlib.sha256(text).hexdigest()


def candidate_obstacle(robot_name: str, rng: random.Random, profile: str, base: float) -> Any:
    if profile == "narrow":
        return random_narrow_workspace_obstacle(rng, base)
    if profile == "mixed":
        if rng.random() < 0.5:
            return random_narrow_workspace_obstacle(rng, max(base, 0.18))
        return random_workspace_obstacle(rng, base)
    return random_workspace_obstacle(rng, base)


def generate_ordered_obstacles(
    robot_name: str,
    seed: int,
    *,
    seed_base: int,
    max_obstacles: int,
    obstacle_profile: str,
    obstacle_scale: float,
    min_separation_margin: float,
    max_tries: int,
) -> list[Any]:
    robot_offsets = {"iiwa": 101, "ur5": 503, "panda": 907}
    rng = random.Random(int(seed_base) + 1009 * int(seed) + robot_offsets.get(str(robot_name), 1709))
    obstacles: list[Any] = []
    tries = 0
    while len(obstacles) < int(max_obstacles) and tries < int(max_tries):
        tries += 1
        obstacle = candidate_obstacle(robot_name, rng, obstacle_profile, float(obstacle_scale))
        if not obstacle_clears_fixed_robot(robot_name, obstacle):
            continue
        if any(aabb_overlaps(obstacle, existing, margin=float(min_separation_margin)) for existing in obstacles):
            continue
        obstacles.append(obstacle)
    if len(obstacles) < int(max_obstacles):
        raise RuntimeError(
            f"could only generate {len(obstacles)}/{max_obstacles} obstacles "
            f"for {robot_name} seed {seed}"
        )
    return obstacles


def generate_catalog(args: argparse.Namespace, path: Path) -> dict[str, Any]:
    robots = [item.strip() for item in str(args.robots).split(",") if item.strip()]
    seeds = parse_int_list(str(args.seeds))
    records: list[dict[str, Any]] = []
    for robot_name in robots:
        for seed in seeds:
            obstacles = generate_ordered_obstacles(
                robot_name,
                seed,
                seed_base=int(args.seed_base),
                max_obstacles=int(args.max_obstacles),
                obstacle_profile=str(args.obstacle_profile),
                obstacle_scale=float(args.obstacle_scale),
                min_separation_margin=float(args.min_obstacle_separation),
                max_tries=int(args.max_obstacle_tries),
            )
            records.append(
                {
                    "robot": robot_name,
                    "seed": int(seed),
                    "min_obstacles": int(args.min_obstacles),
                    "max_obstacles": int(args.max_obstacles),
                    "obstacle_profile": str(args.obstacle_profile),
                    "obstacle_scale": float(args.obstacle_scale),
                    "obstacles": [obstacle_to_json(obstacle) for obstacle in obstacles],
                }
            )
    payload = {
        "schema": CATALOG_SCHEMA,
        "seed_base": int(args.seed_base),
        "robots": robots,
        "seeds": seeds,
        "min_obstacles": int(args.min_obstacles),
        "max_obstacles": int(args.max_obstacles),
        "obstacle_profile": str(args.obstacle_profile),
        "obstacle_scale": float(args.obstacle_scale),
        "records": records,
    }
    payload["obstacle_sha256"] = obstacles_sha256(records)
    write_json(path, payload)
    return payload


def load_or_create_catalog(args: argparse.Namespace) -> dict[str, Any]:
    path = Path(args.scene_catalog or (args.out_dir / "ordered_obstacle_catalog.json"))
    mode = str(args.scene_catalog_mode)
    if mode == "generate" or (mode == "auto" and not path.exists()):
        return generate_catalog(args, path)
    if not path.exists():
        raise FileNotFoundError(path)
    payload = __import__("json").loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != CATALOG_SCHEMA:
        raise RuntimeError(f"catalog {path} schema mismatch: {payload.get('schema')} != {CATALOG_SCHEMA}")
    if mode == "verify":
        expected = generate_catalog(args, path.with_suffix(path.suffix + ".verify_tmp"))
        if expected.get("obstacle_sha256") != payload.get("obstacle_sha256"):
            raise RuntimeError("catalog verify failed: obstacle SHA differs")
        path.with_suffix(path.suffix + ".verify_tmp").unlink(missing_ok=True)
    return payload


def make_options(args: argparse.Namespace, robot_name: str, seed: int, label: str) -> RBFLeafRRTOptions:
    adaptive_target_depth = max(int(args.leaf_max_depth), int(args.adaptive_target_depth))
    return RBFLeafRRTOptions(
        seed=int(seed),
        deep_max_boxes=int(args.deep_max_boxes),
        rbf_max_depth=int(args.rbf_max_depth),
        threads=int(args.threads),
        leaf_start_depth=int(args.leaf_start_depth),
        leaf_max_depth=int(args.leaf_max_depth),
        adaptive_target_depth=adaptive_target_depth,
        adaptive_grid_target_depth=adaptive_target_depth,
        adaptive_planning_backend="partition_native",
        adaptive_max_free_boxes=int(args.deep_max_boxes),
        adaptive_time_budget_ms=float(args.adaptive_time_budget_ms),
        adaptive_node_budget=int(args.adaptive_node_budget),
        adaptive_defer_min_depth=int(args.adaptive_defer_min_depth),
        adaptive_overlap_depth_threshold=float(args.adaptive_overlap_depth_threshold),
        adaptive_overlap_depth_min_threshold=float(args.adaptive_overlap_depth_min_threshold),
        adaptive_overlap_depth_decay_per_depth=float(args.adaptive_overlap_depth_decay_per_depth),
        adaptive_overlap_ratio_threshold=0.0,
        adaptive_seed_probe_count=int(args.adaptive_seed_probe_count),
        adaptive_seed_anchor_probe_cap=int(args.adaptive_seed_anchor_probe_cap),
        adaptive_depth_enabled=bool(args.adaptive_depth_enabled),
        adaptive_depth_min=min(int(args.adaptive_depth_min), adaptive_target_depth),
        adaptive_depth_max=adaptive_target_depth,
        adaptive_depth_probe_count=int(args.adaptive_depth_probe_count),
        adaptive_depth_anchor_probe_cap=int(args.adaptive_depth_anchor_probe_cap),
        adaptive_depth_probe_seed=int(args.adaptive_depth_probe_seed),
        adaptive_depth_min_free_probes=int(args.adaptive_depth_min_free_probes),
        adaptive_depth_min_covered_probes=int(args.adaptive_depth_min_covered_probes),
        adaptive_depth_min_main_probes=int(args.adaptive_depth_min_main_probes),
        adaptive_depth_min_main_ratio=float(args.adaptive_depth_min_main_ratio),
        adaptive_depth_min_cells=int(args.adaptive_depth_min_cells),
        adaptive_depth_min_main_cells=int(args.adaptive_depth_min_main_cells),
        adaptive_depth_max_online_cells=int(args.adaptive_depth_max_online_cells),
        adaptive_depth_max_probe_ms=float(args.adaptive_depth_max_probe_ms),
        adaptive_fast_virtual_checkpoint_mode=bool(args.adaptive_fast_virtual_checkpoint_mode),
        adaptive_max_merge_ms=float(args.adaptive_max_merge_ms),
        adaptive_max_merge_rounds=int(args.adaptive_max_merge_rounds),
        adaptive_max_merge_input_boxes=int(args.adaptive_max_merge_input_boxes),
        validation_batch_size=int(args.validation_batch_size),
        deep_ffb_depth=int(args.deep_ffb_depth),
        ffb_start_depth=int(args.ffb_start_depth),
        ffb_binary_probe_depth=int(args.ffb_binary_probe_depth),
        ffb_search_mode=str(args.ffb_search_mode),
        use_external_evidence=True,
        external_evidence_path=robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root)),
        external_evidence_verify_identity=False,
        symmetry_aligned_native_root=False,
        symmetry_aligned_cache_schedule=False,
        database_canonical_mode=True,
        case_label=label,
        use_virtual_topology=bool(args.use_virtual_topology),
        parallel_virtual_validation=bool(args.parallel_virtual_validation),
        leaf_threads=int(args.threads),
        canonicalize_queries=False,
    )


def configure_dynamic_update(cfg: Any, args: argparse.Namespace) -> None:
    cfg.enable_merger = bool(args.enable_merger)
    cfg.dynamic_update.dirty_region_padding = float(args.dirty_region_padding)
    cfg.dynamic_update.local_regrow_box_limit = int(args.local_regrow_box_limit)
    cfg.dynamic_update.local_regrow_timeout_ms = float(args.local_regrow_timeout_ms)
    cfg.dynamic_update.enable_warm_rebuild_fallback = False
    cfg.dynamic_update.warm_rebuild_dirty_box_fraction = 0.0
    cfg.dynamic_update.warm_rebuild_min_local_boxes_added = int(args.local_regrow_box_limit) + 1


def make_forest(args: argparse.Namespace, robot: Any, robot_name: str, seed: int, label: str) -> Any:
    opt = make_options(args, robot_name, seed, label)
    cfg = configure_leaf_rrt(robot, args.out_dir / "active_cache" / label, opt)
    configure_dynamic_update(cfg, args)
    return sbf.SafeBoxForest(robot, cfg), opt


def build_adaptive(args: argparse.Namespace, forest: Any, obstacles: list[Any], opt: RBFLeafRRTOptions) -> dict[str, Any]:
    start = time.perf_counter()
    result = forest.build_adaptive_deep_leaf_sweep_cover(obstacles, make_adaptive_leaf_sweep_config(opt))
    wall_s = time.perf_counter() - start
    row = build_result_row(result, wall_s)
    row["forest_boxes"] = len(list(forest.boxes()))
    row["forest_segment_edges"] = len(list(forest.segment_edges()))
    return row


def require_nonempty_build(row: dict[str, Any], *, label: str) -> None:
    boxes = int(row.get("forest_boxes", 0) or 0)
    cells = int(row.get("partition_cell_count", 0) or 0)
    if boxes <= 0 and cells <= 0:
        raise RuntimeError(
            f"{label} produced an empty adaptive leaf-sweep forest. "
            "This makes dynamic updates vacuous; increase leaf/adaptive depth "
            "or reduce obstacle density before reporting Exp.7."
        )


def run_record(args: argparse.Namespace, record: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    robot_name = str(record["robot"])
    seed = int(record["seed"])
    robot = make_robot(robot_name)
    obstacles = normalize_obstacles(obstacle_from_bounds(bounds) for bounds in record["obstacles"])
    min_count = int(record.get("min_obstacles", args.min_obstacles))
    max_count = int(record.get("max_obstacles", args.max_obstacles))
    prefix = obstacles[:min_count]

    forest, opt = make_forest(args, robot, robot_name, seed, f"exp07_{robot_name}_seed{seed}_incremental")
    initial_build = build_adaptive(args, forest, prefix, opt)
    require_nonempty_build(initial_build, label=f"initial build {robot_name} seed {seed}")

    events: list[dict[str, Any]] = []
    insert_profile = profile_row(forest.add_obstacles_and_rebuild(obstacles[min_count:max_count]))
    events.append(
        {
            "robot": robot_name,
            "seed": seed,
            "operation": "insert",
            "obstacle_index": f"{min_count}-{max_count - 1}",
            "count_before": min_count,
            "count_after": max_count,
            "time_s": insert_profile["total_ms"] / 1000.0,
            **insert_profile,
        }
    )

    max_forest, max_opt = make_forest(args, robot, robot_name, seed, f"exp07_{robot_name}_seed{seed}_max_build")
    max_build = build_adaptive(args, max_forest, obstacles[:max_count], max_opt)
    require_nonempty_build(max_build, label=f"max build {robot_name} seed {seed}")

    remove_profile = profile_row(forest.remove_obstacle_suffix_and_regrow(min_count))
    events.append(
        {
            "robot": robot_name,
            "seed": seed,
            "operation": "remove",
            "obstacle_index": f"{min_count}-{max_count - 1}",
            "count_before": max_count,
            "count_after": min_count,
            "time_s": remove_profile["total_ms"] / 1000.0,
            **remove_profile,
        }
    )

    build_row = {
        "robot": robot_name,
        "seed": seed,
        "min_obstacles": min_count,
        "max_obstacles": max_count,
        "initial_build": initial_build,
        "max_build": max_build,
        "initial_build_s": initial_build["wall_s"],
        "max_build_s": max_build["wall_s"],
    }
    return events, build_row


def summarize_by_count(events: list[dict[str, Any]], builds: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    keys = sorted({(row["robot"], int(row["count_after"] if row["operation"] == "insert" else row["count_before"])) for row in events})
    for robot_name, obstacle_count in keys:
        insert_items = [
            row for row in events
            if row["robot"] == robot_name and row["operation"] == "insert" and int(row["count_after"]) == obstacle_count
        ]
        remove_items = [
            row for row in events
            if row["robot"] == robot_name and row["operation"] == "remove" and int(row["count_before"]) == obstacle_count
        ]
        build_items = [
            row for row in builds
            if row["robot"] == robot_name and int(row["max_obstacles"]) == obstacle_count
        ]
        rows.append(
            {
                "robot": robot_name,
                "obstacle_count": obstacle_count,
                "insert_runs": len(insert_items),
                "insert_s_q1": q1(row["time_s"] for row in insert_items),
                "insert_s_q3": q3(row["time_s"] for row in insert_items),
                "insert_dirty_boxes_q1": q1(row["dirty_boxes"] for row in insert_items),
                "insert_dirty_boxes_q3": q3(row["dirty_boxes"] for row in insert_items),
                "insert_boxes_added_q1": q1(row["boxes_added"] for row in insert_items),
                "insert_boxes_added_q3": q3(row["boxes_added"] for row in insert_items),
                "remove_runs": len(remove_items),
                "remove_s_q1": q1(row["time_s"] for row in remove_items),
                "remove_s_q3": q3(row["time_s"] for row in remove_items),
                "remove_dirty_boxes_q1": q1(row["dirty_boxes"] for row in remove_items),
                "remove_dirty_boxes_q3": q3(row["dirty_boxes"] for row in remove_items),
                "remove_boxes_added_q1": q1(row["boxes_added"] for row in remove_items),
                "remove_boxes_added_q3": q3(row["boxes_added"] for row in remove_items),
                "max_build_runs": len(build_items),
                "max_build_s_q1": q1(row["max_build_s"] for row in build_items),
                "max_build_s_q3": q3(row["max_build_s"] for row in build_items),
                "max_build_boxes_q1": q1(row["max_build"]["forest_boxes"] for row in build_items),
                "max_build_boxes_q3": q3(row["max_build"]["forest_boxes"] for row in build_items),
                "max_build_cells_q1": q1(row["max_build"]["partition_cell_count"] for row in build_items),
                "max_build_cells_q3": q3(row["max_build"]["partition_cell_count"] for row in build_items),
            }
        )
    return rows


def summarize(events: list[dict[str, Any]], builds: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for robot_name in sorted({str(row["robot"]) for row in events}):
        insert_items = [
            row for row in events
            if row["robot"] == robot_name and row["operation"] == "insert"
        ]
        remove_items = [
            row for row in events
            if row["robot"] == robot_name and row["operation"] == "remove"
        ]
        build_items = [row for row in builds if row["robot"] == robot_name]
        max_build_by_seed = {
            int(row["seed"]): float(row["max_build_s"])
            for row in build_items
            if finite(row.get("max_build_s")) > 0.0
        }
        insert_speedups = [
            max_build_by_seed[int(row["seed"])] / float(row["time_s"])
            for row in insert_items
            if int(row["seed"]) in max_build_by_seed and finite(row.get("time_s")) > 0.0
        ]
        remove_speedups = [
            max_build_by_seed[int(row["seed"])] / float(row["time_s"])
            for row in remove_items
            if int(row["seed"]) in max_build_by_seed and finite(row.get("time_s")) > 0.0
        ]
        rows.append(
            {
                "robot": robot_name,
                "source_obstacles": int(build_items[0]["min_obstacles"]) if build_items else None,
                "target_obstacles": int(build_items[0]["max_obstacles"]) if build_items else None,
                "insert_runs": len(insert_items),
                "insert_s_q1": q1(row["time_s"] for row in insert_items),
                "insert_s_q3": q3(row["time_s"] for row in insert_items),
                "insert_speedup_q1": q1(insert_speedups),
                "insert_speedup_q3": q3(insert_speedups),
                "insert_dirty_boxes_q1": q1(row["dirty_boxes"] for row in insert_items),
                "insert_dirty_boxes_q3": q3(row["dirty_boxes"] for row in insert_items),
                "insert_boxes_added_q1": q1(row["boxes_added"] for row in insert_items),
                "insert_boxes_added_q3": q3(row["boxes_added"] for row in insert_items),
                "remove_runs": len(remove_items),
                "remove_s_q1": q1(row["time_s"] for row in remove_items),
                "remove_s_q3": q3(row["time_s"] for row in remove_items),
                "remove_speedup_q1": q1(remove_speedups),
                "remove_speedup_q3": q3(remove_speedups),
                "remove_dirty_boxes_q1": q1(row["dirty_boxes"] for row in remove_items),
                "remove_dirty_boxes_q3": q3(row["dirty_boxes"] for row in remove_items),
                "remove_boxes_added_q1": q1(row["boxes_added"] for row in remove_items),
                "remove_boxes_added_q3": q3(row["boxes_added"] for row in remove_items),
                "max_build_runs": len(build_items),
                "source_warm_s_q1": q1(row["initial_build_s"] for row in build_items),
                "source_warm_s_q3": q3(row["initial_build_s"] for row in build_items),
                "target_warm_s_q1": q1(row["max_build_s"] for row in build_items),
                "target_warm_s_q3": q3(row["max_build_s"] for row in build_items),
                "max_build_s_q1": q1(row["max_build_s"] for row in build_items),
                "max_build_s_q3": q3(row["max_build_s"] for row in build_items),
                "max_build_boxes_q1": q1(row["max_build"]["forest_boxes"] for row in build_items),
                "max_build_boxes_q3": q3(row["max_build"]["forest_boxes"] for row in build_items),
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames and key != "diagnostics":
                fieldnames.append(key)
    write_csv_rows(path, rows, fieldnames)


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    source_n = int(rows[0].get("source_obstacles") or DEFAULT_MIN_OBSTACLES) if rows else DEFAULT_MIN_OBSTACLES
    target_n = int(rows[0].get("target_obstacles") or DEFAULT_MAX_OBSTACLES) if rows else DEFAULT_MAX_OBSTACLES
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        rf"\caption{{Adaptive leaf-sweep maintenance only, not end-to-end replanning. Times are milliseconds shown as \([Q_1,Q_3]\) over saved ordered random scenes. Warm@{source_n} and Warm@{target_n} are fresh adaptive leaf-sweep builds; Insert and Remove are batched updates between the two obstacle counts. Speedup is Warm@{target_n}/Update.}}",
        r"\label{tab:tro-dynamic-update}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.5pt}",
        r"\resizebox{\columnwidth}{!}{%",
        r"\begin{tabular}{rrrrrr}",
        r"\toprule",
        rf"Warm@{source_n} & Insert & Ins. sp. & Remove & Rem. sp. & Warm@{target_n} \\",
        r"\midrule",
    ]
    for row in rows:
        lines.append(
            f"{tex_qrange_ms(row.get('source_warm_s_q1'), row.get('source_warm_s_q3'))} & "
            f"{tex_qrange_ms(row.get('insert_s_q1'), row.get('insert_s_q3'))} & "
            f"{tex_speedup_range(row.get('insert_speedup_q1'), row.get('insert_speedup_q3'))} & "
            f"{tex_qrange_ms(row.get('remove_s_q1'), row.get('remove_s_q3'))} & "
            f"{tex_speedup_range(row.get('remove_speedup_q1'), row.get('remove_speedup_q3'))} & "
            f"{tex_qrange_ms(row.get('target_warm_s_q1'), row.get('target_warm_s_q3'))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}%", r"}", r"\end{table}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.7 obstacle-count dynamic update for adaptive leaf sweep.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp07")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--robots", default="iiwa")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--seed-base", type=int, default=9207)
    parser.add_argument("--min-obstacles", type=int, default=DEFAULT_MIN_OBSTACLES)
    parser.add_argument("--max-obstacles", type=int, default=DEFAULT_MAX_OBSTACLES)
    parser.add_argument("--obstacle-profile", choices=["random", "narrow", "mixed"], default="random")
    parser.add_argument("--obstacle-scale", type=float, default=0.12)
    parser.add_argument("--min-obstacle-separation", type=float, default=0.01)
    parser.add_argument("--max-obstacle-tries", type=int, default=10000)
    parser.add_argument("--deep-max-boxes", type=int, default=DEFAULT_EXP07_BOX_BUDGET)
    parser.add_argument("--rbf-max-depth", type=int, default=DEFAULT_RBF_MAX_DEPTH)
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depth", type=int, default=DEFAULT_LEAF_MAX_DEPTH)
    parser.add_argument("--adaptive-target-depth", type=int, default=DEFAULT_ADAPTIVE_TARGET_DEPTH)
    parser.add_argument("--deep-ffb-depth", type=int, default=DEFAULT_RBF_DEEP_FFB_DEPTH)
    parser.add_argument("--ffb-start-depth", type=int, default=16)
    parser.add_argument("--ffb-binary-probe-depth", type=int, default=DEFAULT_RBF_FFB_BINARY_PROBE_DEPTH)
    parser.add_argument("--ffb-search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE, choices=["linear", "binary", "binary-depth", "BinaryDepth", "Linear"])
    parser.add_argument("--adaptive-time-budget-ms", type=float, default=60000.0)
    parser.add_argument("--adaptive-node-budget", type=int, default=0)
    parser.add_argument("--adaptive-defer-min-depth", type=int, default=16)
    parser.add_argument("--adaptive-overlap-depth-threshold", type=float, default=0.05)
    parser.add_argument("--adaptive-overlap-depth-min-threshold", type=float, default=0.01)
    parser.add_argument("--adaptive-overlap-depth-decay-per-depth", type=float, default=0.002)
    parser.add_argument("--adaptive-seed-probe-count", type=int, default=1024)
    parser.add_argument("--adaptive-seed-anchor-probe-cap", type=int, default=64)
    parser.add_argument("--adaptive-depth-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--adaptive-depth-min", type=int, default=10)
    parser.add_argument("--adaptive-depth-probe-count", type=int, default=1024)
    parser.add_argument("--adaptive-depth-anchor-probe-cap", type=int, default=64)
    parser.add_argument("--adaptive-depth-probe-seed", type=int, default=20260607)
    parser.add_argument("--adaptive-depth-min-free-probes", type=int, default=64)
    parser.add_argument("--adaptive-depth-min-covered-probes", type=int, default=0)
    parser.add_argument("--adaptive-depth-min-main-probes", type=int, default=0)
    parser.add_argument("--adaptive-depth-min-main-ratio", type=float, default=0.0)
    parser.add_argument("--adaptive-depth-min-cells", type=int, default=200)
    parser.add_argument("--adaptive-depth-min-main-cells", type=int, default=1)
    parser.add_argument("--adaptive-depth-max-online-cells", type=int, default=320)
    parser.add_argument("--adaptive-depth-max-probe-ms", type=float, default=10.0)
    parser.add_argument("--adaptive-fast-virtual-checkpoint-mode", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--adaptive-max-merge-ms", type=float, default=1500.0)
    parser.add_argument("--adaptive-max-merge-rounds", type=int, default=2)
    parser.add_argument("--adaptive-max-merge-input-boxes", type=int, default=20000)
    parser.add_argument("--validation-batch-size", type=int, default=512)
    parser.add_argument("--enable-merger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--use-virtual-topology", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dirty-region-padding", type=float, default=0.0)
    parser.add_argument("--local-regrow-box-limit", type=int, default=DEFAULT_EXP07_LOCAL_REGROW_BOX_LIMIT)
    parser.add_argument("--local-regrow-timeout-ms", type=float, default=1000.0)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--lect-cache-root", type=Path, default=ROBOT_LECTDB_CACHE_ROOT)
    parser.add_argument("--skip-lect-cache-ensure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.phase == "smoke":
        args.seeds = ",".join(parse_int_list(args.seeds)[:1] and [str(parse_int_list(args.seeds)[0])] or ["0"])
        args.min_obstacles = 1
        args.max_obstacles = 2
    elif args.phase == "pilot":
        args.seeds = ",".join(str(seed) for seed in parse_int_list(args.seeds)[:3])
        args.min_obstacles = DEFAULT_MIN_OBSTACLES
        args.max_obstacles = DEFAULT_MAX_OBSTACLES
    configure_thread_environment(int(args.threads))

    catalog_path = Path(args.scene_catalog or (args.out_dir / "ordered_obstacle_catalog.json"))
    if args.dry_run:
        catalog_payload: dict[str, Any] | None = None
    else:
        catalog_payload = load_or_create_catalog(args)

    cache_rows: list[dict[str, Any]] = []
    for robot_name in progress(
        [item.strip() for item in str(args.robots).split(",") if item.strip()],
        desc="exp07 lect cache",
        total=len([item for item in str(args.robots).split(",") if item.strip()]),
        disable=bool(args.dry_run or args.skip_lect_cache_ensure),
    ):
        cache_rows.append(
            ensure_robot_lectdb_cache(
                robot_name,
                cache_root=Path(args.lect_cache_root),
                threads=int(args.threads),
                dry_run=bool(args.dry_run or args.skip_lect_cache_ensure),
            )
        )

    planned_records = [] if catalog_payload is None else list(catalog_payload.get("records", []))
    events: list[dict[str, Any]] = []
    builds: list[dict[str, Any]] = []
    if not args.dry_run and catalog_payload is not None:
        for record in progress(planned_records, desc="exp07 ordered scenes", total=len(planned_records)):
            print(f"[exp07] robot={record['robot']} seed={record['seed']} obstacles={record['max_obstacles']}", flush=True)
            record_events, build_row = run_record(args, record)
            events.extend(record_events)
            builds.append(build_row)

    by_count_rows = summarize_by_count(events, builds) if events else []
    summary_rows = summarize(events, builds) if events else []
    payload = {
        "experiment": "exp07_obstacle_count_dynamic_update",
        "run_id": run_id("exp07"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "args": namespace_dict(args),
        "environment": environment_metadata(),
        "rbf_default_profile": default_rbf_profile(),
        "rbf_exp07_profile": {
            "backend": "adaptive_leaf_sweep_only_partition_native",
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "adaptive_target_depth": max(int(args.leaf_max_depth), int(args.adaptive_target_depth)),
            "adaptive_time_budget_ms": float(args.adaptive_time_budget_ms),
            "deep_max_boxes": int(args.deep_max_boxes),
            "ffb_binary_probe_depth": int(args.ffb_binary_probe_depth),
            "ffb_search_mode": str(args.ffb_search_mode),
            "ffb_implementation": DEFAULT_RBF_FFB_IMPLEMENTATION,
            "use_virtual_topology": bool(args.use_virtual_topology),
            "parallel_virtual_validation": bool(args.parallel_virtual_validation),
            "canonical_mapping_scope": "LECT_internal_only",
            "planner_stage": "none",
            "query_stage": "none",
            "statistic": "Q1_Q3_over_ordered_random_scenes",
        },
        "lectdb_caches": cache_rows,
        "scene_catalog": {
            "path": str(catalog_path),
            "mode": str(args.scene_catalog_mode),
            "schema": None if catalog_payload is None else catalog_payload.get("schema"),
            "records": None if catalog_payload is None else len(catalog_payload.get("records", [])),
            "obstacle_sha256": None if catalog_payload is None else catalog_payload.get("obstacle_sha256"),
        },
        "build_rows": builds,
        "event_rows": events,
        "by_count_summary": by_count_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "dynamic_update_manifest.json", payload)
    if events:
        write_csv(args.out_dir / "dynamic_update_events.csv", events)
    if builds:
        flat_builds = []
        for row in builds:
            flat = {
                "robot": row["robot"],
                "seed": row["seed"],
                "min_obstacles": row["min_obstacles"],
                "max_obstacles": row["max_obstacles"],
                "initial_build_s": row["initial_build_s"],
                "max_build_s": row["max_build_s"],
            }
            for prefix in ("initial_build", "max_build"):
                for key, value in row[prefix].items():
                    if key != "diagnostics":
                        flat[f"{prefix}_{key}"] = value
            flat_builds.append(flat)
        write_csv(args.out_dir / "dynamic_update_builds.csv", flat_builds)
    if summary_rows:
        write_csv(args.out_dir / "dynamic_update_by_count_summary.csv", by_count_rows)
        write_csv(args.out_dir / "dynamic_update_summary.csv", summary_rows)
        write_tex(args.out_dir / "tab_tro_dynamic_update.tex", summary_rows)
        if str(args.phase) == "paper":
            write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_dynamic_update.tex", summary_rows)
    print(f"wrote {args.out_dir / 'dynamic_update_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
