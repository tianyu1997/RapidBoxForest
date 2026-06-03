#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import (  # noqa: E402
    EPS_PATH_DEFAULT,
    aggregate_stage_records,
    assert_promoted_monotone,
    euclidean_path_length,
    final_ompl_simplify_path,
    incumbent_stage_record,
    path_passes_post_audit,
    task_result,
    update_incumbents,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import segment_free  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, RBF_LIFELONG_PRESET, add_common_sbf_args, configure_external_evidence_reuse, configure_standalone_sbf, query_result_payload, sbf, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import (  # noqa: E402
    DEFAULT_RANDOM_DIFFICULTIES,
    DEFAULT_RANDOM_ROBOTS,
    DEFAULT_RANDOM_SCENE_SEEDS,
    ENDPOINT_CLEARANCE_MARGIN_M,
    FIXED_ROBOT_CLEARANCE_MARGIN_M,
    SEGMENT_RESOLUTION,
    SceneSpec,
    make_random_scene,
    make_robot,
    scene_profile_requires_balanced_probe,
)  # noqa: E402


DEFAULT_SBF_STAGES = "seed:2:0:0:2:48,fast:16:0:0:2500:80,balanced:64:256:450:5000:120,quality:128:1024:1500:8000:160,high:512:2000:5000:20000:200"
DEFAULT_PRM_BUILD_GRID = "2,5,10,20,40,80"
DEFAULT_BITSTAR_TIMEOUT_S = 120.0
DEFAULT_BITSTAR_CHECKPOINT_INTERVAL_S = 2.0
DEFAULT_RRT_TIMEOUT_MS = 120000.0
LEGACY_SBF_STAGE_SEED_OFFSETS = {
    "seed": 0,
    "fast": 0,
    "balanced": 9176,
    "quality": 2 * 9176,
    "high": 3 * 9176,
}


def parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def _obstacle_bounds_from_obj(obstacle: Any) -> list[float]:
    if hasattr(obstacle, "bounds"):
        return [float(v) for v in list(getattr(obstacle, "bounds"))]
    if hasattr(obstacle, "__iter__"):
        values = list(obstacle)
        if len(values) >= 6:
            return [float(v) for v in values[:6]]
    raise TypeError("unsupported obstacle representation")


def _obstacle_from_bounds(item: Any) -> Any:
    if not isinstance(item, (list, tuple)) or len(item) < 6:
        raise ValueError(f"invalid obstacle record: {item!r}")
    return sbf.Obstacle(*[float(v) for v in item[:6]])


def _scene_cache_key(robot_name: str, difficulty: str, scene_seed: int) -> str:
    return f"{robot_name}:{difficulty}:{int(scene_seed)}"


def scene_to_record(
    *,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    scene: Any,
    scene_profile: str,
    seed_base: int,
) -> dict[str, Any]:
    return {
        "schema": 1,
        "robot_name": str(robot_name),
        "difficulty": str(difficulty),
        "scene_seed": int(scene_seed),
        "scene_profile": str(scene_profile),
        "seed_base": int(seed_base),
        "scene_valid": True,
        "start": [float(value) for value in list(scene.start)],
        "goal": [float(value) for value in list(scene.goal)],
        "obstacles": [_obstacle_bounds_from_obj(obstacle) for obstacle in list(scene.obstacles)],
        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
        "fixed_robot_clearance_margin_m": float(scene.fixed_robot_clearance_margin_m),
        "direct_segment_blocked": bool(scene.direct_segment_blocked),
        "segment_resolution": int(scene.segment_resolution),
        "robot_joint_limits_hash": int(hash(str(scene.robot_name))),
    }


def scene_from_record(record: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> Any:
    if not bool(record.get("scene_valid", False)):
        raise RuntimeError(f"scene cache entry invalid for {robot_name}:{difficulty}:{scene_seed}")
    return SceneSpec(
        robot_name=str(record.get("robot_name", robot_name)),
        difficulty=str(record.get("difficulty", difficulty)),
        obstacles=[_obstacle_from_bounds(bounds) for bounds in list(record.get("obstacles", []))],
        start=[float(value) for value in list(record.get("start", []))],
        goal=[float(value) for value in list(record.get("goal", []))],
        endpoint_clearance_margin_m=float(record.get("endpoint_clearance_margin_m", ENDPOINT_CLEARANCE_MARGIN_M)),
        fixed_robot_clearance_margin_m=float(record.get("fixed_robot_clearance_margin_m", FIXED_ROBOT_CLEARANCE_MARGIN_M)),
        direct_segment_blocked=bool(record.get("direct_segment_blocked", True)),
        segment_resolution=int(record.get("segment_resolution", SEGMENT_RESOLUTION)),
    )


def load_or_build_scene_catalog(
    *,
    scene_catalog_path: Path | None,
    robots: list[str],
    difficulties: list[str],
    scene_seeds: int,
    scene_profile: str,
    seed_base: int,
    cache_mode: str = "auto",
) -> tuple[dict[str, Any], dict[str, Any]]:
    requested = int(scene_seeds)
    keys = [
        _scene_cache_key(robot, difficulty, int(seed))
        for robot in robots
        for difficulty in difficulties
        for seed in range(max(1, requested))
    ]
    if scene_catalog_path is None:
        cache: dict[str, Any] = {}
        for key in keys:
            robot_name, difficulty, scene_seed = key.split(":")
            scene = make_random_scene(
                robot_name,
                difficulty,
                int(seed_base) + 1009 * int(scene_seed),
                scene_profile=scene_profile,
            )
            cache[key] = scene_to_record(
                robot_name=robot_name,
                difficulty=difficulty,
                scene_seed=int(scene_seed),
                scene=scene,
                scene_profile=scene_profile,
                seed_base=int(seed_base),
            )
        return cache, {
            "schema": 1,
            "mode": "generate_no_cache",
            "scene_profile": str(scene_profile),
            "seed_base": int(seed_base),
            "request": {"robots": robots, "difficulties": difficulties, "scene_seeds": int(requested)},
            "records": list(cache.values()),
            "record_count": len(cache),
        }

    existing: dict[str, dict[str, Any]] = {}
    catalog_path = Path(scene_catalog_path)
    for _ in range(2):
        if cache_mode == "generate":
            break
        if catalog_path.exists():
            payload = json.loads(catalog_path.read_text(encoding="utf-8", errors="replace"))
            if (
                str(payload.get("scene_profile", "")) != str(scene_profile)
                or int(payload.get("seed_base", -1)) != int(seed_base)
            ):
                break
            if int(payload.get("schema", -1)) != 1:
                break
            for item in payload.get("records", []):
                if not isinstance(item, dict):
                    continue
                record_key = _scene_cache_key(
                    str(item.get("robot_name")),
                    str(item.get("difficulty")),
                    int(item.get("scene_seed", -1)),
                )
                existing[record_key] = item
            if len(existing) == len(list(keys)):
                return existing, payload
            if cache_mode == "reuse":
                raise RuntimeError(
                    f"scene catalog reuse requested but incomplete/invalid: missing {len(keys) - len(existing)} entries from {catalog_path}"
                )
            break
    # regenerate (auto mode or missing incomplete cache)
    generated: dict[str, Any] = {}
    for key in keys:
        robot_name, difficulty, scene_seed = key.split(":")
        if cache_mode != "generate" and key in existing:
            generated[key] = existing[key]
            continue
        if cache_mode == "reuse":
            raise RuntimeError(f"scene catalog reuse missing entry {key} in {catalog_path}")
        scene = make_random_scene(
            robot_name,
            difficulty,
            int(seed_base) + 1009 * int(scene_seed),
            scene_profile=scene_profile,
        )
        generated[key] = scene_to_record(
            robot_name=robot_name,
            difficulty=difficulty,
            scene_seed=int(scene_seed),
            scene=scene,
            scene_profile=scene_profile,
            seed_base=int(seed_base),
        )
    payload = {
        "schema": 1,
        "mode": "reused" if cache_mode == "reuse" else "auto_regen",
        "scene_profile": str(scene_profile),
        "seed_base": int(seed_base),
        "request": {"robots": robots, "difficulties": difficulties, "scene_seeds": int(requested)},
        "records": list(generated.values()),
        "record_count": len(generated),
    }
    catalog_path.parent.mkdir(parents=True, exist_ok=True)
    catalog_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return generated, payload


def parse_sbf_stages(raw: str) -> list[dict[str, Any]]:
    stages: list[dict[str, Any]] = []
    for index, item in enumerate(parse_csv(raw)):
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


def parse_float_grid(raw: str) -> list[float]:
    return [float(item) for item in parse_csv(raw)]


def parse_int_grid(raw: str) -> list[int]:
    return [int(float(item)) for item in parse_csv(raw)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Warm-cache anytime SBF trade-off runner for random robot scenes.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        threads=8,
        task_batch_size=8,
        max_boxes=20000,
        timeout_ms=5000.0,
        ffb_depth=160,
        component_connect_ffb_max_depth=200,
        post_connect_extra_boxes=2000,
        quality_min_connected_boxes=512,
        post_connect_time_budget_ms=5000.0,
        repair_timeout_ms=1500.0,
    )
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--methods", default="support_hull_coverage")
    parser.add_argument("--baseline-methods", default="rrt,prm,bitstar")
    parser.add_argument("--baseline-trials", type=int, default=1)
    parser.add_argument("--sbf-stages", default=DEFAULT_SBF_STAGES)
    parser.add_argument("--cache-root", type=Path, default=ROOT / "outputs" / "paper" / "lect_cache_random_anytime")
    parser.add_argument("--cache-run-id", default="tro2026_random_anytime")
    parser.add_argument("--sbf-cache-scope", choices=["scene_stage", "disjoint_warm"], default="scene_stage")
    parser.add_argument("--random-prewarm-depth", type=int, default=20)
    parser.add_argument("--sbf-use-external-prewarm", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clear-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prewarm-scene-seeds", type=int, default=0)
    parser.add_argument("--prewarm-seed-base", type=int, default=20270504)
    parser.add_argument("--segment-step", type=float, default=0.01)
    parser.add_argument("--prm-build-grid-s", default=DEFAULT_PRM_BUILD_GRID)
    parser.add_argument("--prm-query-budget-s", type=float, default=1.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=64)
    parser.add_argument("--prm-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--bitstar-timeout-s", type=float, default=DEFAULT_BITSTAR_TIMEOUT_S)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=DEFAULT_BITSTAR_CHECKPOINT_INTERVAL_S)
    parser.add_argument("--bitstar-restart-grid", default="", help="Deprecated compatibility flag; ignored because BIT* now runs one fixed-timeout trace per trial.")
    parser.add_argument("--bitstar-budget-s", type=float, default=None, help="Deprecated alias for --bitstar-timeout-s when provided.")
    parser.add_argument("--bitstar-simplify-time-s", type=float, default=0.2)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=-1)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rrt-timeout-ms", type=float, default=DEFAULT_RRT_TIMEOUT_MS)
    parser.add_argument("--rrt-step-size", type=float, default=0.35)
    parser.add_argument("--rrt-simplify-time-s", type=float, default=0.0)
    parser.add_argument("--rrt-restarts", type=int, default=0, help="Deprecated compatibility flag; ignored because RRTConnect now runs one max-timeout attempt per trial.")
    parser.add_argument("--rrt-restart-grid", default="", help="Deprecated compatibility flag; ignored because RRTConnect now runs one max-timeout attempt per trial.")
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.01)
    parser.add_argument("--sbf-query-shortcut-passes", type=int, default=3)
    parser.add_argument("--sbf-query-shortcut-samples", type=int, default=32)
    parser.add_argument("--sbf-stage-restarts", type=int, default=1, help="Independent SBF build/query restarts per stage; all time is charged and the shortest audited stage path is retained.")
    parser.add_argument("--sbf-build-backend", choices=["coverage", "leaf_refine"], default="leaf_refine")
    parser.add_argument("--leaf-start-depth", type=int, default=8)
    parser.add_argument("--leaf-max-depth", type=int, default=14)
    parser.add_argument("--leaf-obstacle-cluster-gap", type=float, default=1000.0)
    parser.add_argument("--leaf-virtual-topology", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--leaf-parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--leaf-store-group-results", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--leaf-validation-batch-size", type=int, default=512)
    parser.add_argument("--leaf-threads", type=int, default=0)
    parser.add_argument("--leaf-timeout-ms", type=float, default=0.0)
    parser.add_argument("--leaf-domain-seed-cap", type=int, default=8)
    parser.add_argument("--leaf-domain-success-cap", type=int, default=2)
    parser.add_argument("--leaf-domain-attempt-cap", type=int, default=8)
    parser.add_argument("--leaf-allow-anchor-roots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sbf-bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sbf-bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--sbf-post-audit-local-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sbf-post-audit-repair-timeout-ms", type=float, default=50.0)
    parser.add_argument("--sbf-post-audit-repair-trials", type=int, default=4)
    parser.add_argument("--sbf-post-audit-repair-simplify-time-s", type=float, default=0.02)
    parser.add_argument("--sbf-corridor-refine", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument(
        "--scene-catalog-mode",
        choices=["auto", "generate", "reuse"],
        default="auto",
        help="auto=load matching catalog if possible, otherwise regenerate; reuse=only load existing matching catalog; generate=always regenerate.",
    )
    parser.add_argument("--sbf-corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--sbf-corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--sbf-corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--sbf-corridor-refine-passes", type=int, default=2)
    parser.add_argument("--sbf-corridor-refine-start-margin-ms", type=float, default=120.0)
    parser.add_argument("--epsilon-path", type=float, default=EPS_PATH_DEFAULT)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_random_anytime_tradeoff_full.json")
    return parser.parse_args()


def audit_path(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    return all(segment_free(robot, obstacles, path[index], path[index + 1], step) for index in range(len(path) - 1))


def post_audit_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    step: float,
    *,
    start: list[float],
    goal: list[float],
) -> bool:
    return path_passes_post_audit(
        sbf,
        robot,
        obstacles,
        path,
        segment_step=float(step),
        start=list(start),
        goal=list(goal),
    )


def audit_segment_step(args: argparse.Namespace) -> float:
    value = float(getattr(args, "audit_segment_step", 0.0) or 0.0)
    return value if value > 0.0 else float(args.segment_step)


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def cumulative_lengths(points: list[list[float]]) -> list[float]:
    lengths = [0.0]
    for index in range(len(points) - 1):
        lengths.append(lengths[-1] + point_distance(points[index], points[index + 1]))
    return lengths


def planner_internal_simplify_time(unified_simplify_s: float, planner_simplify_s: float) -> float:
    return 0.0 if float(unified_simplify_s) > 0.0 else float(planner_simplify_s)


def unified_final_simplify_enabled(args: argparse.Namespace) -> bool:
    return float(getattr(args, "final_ompl_simplify_time_s", 0.0)) > 0.0


def interpolate_at(points: list[list[float]], lengths: list[float], target: float) -> tuple[list[float], int, bool]:
    if target <= 0.0:
        return list(points[0]), 0, True
    if target >= lengths[-1]:
        return list(points[-1]), len(points) - 2, True
    for index in range(len(lengths) - 1):
        lo = lengths[index]
        hi = lengths[index + 1]
        if target <= hi + 1e-12:
            if abs(target - lo) <= 1e-12:
                return list(points[index]), index, True
            if abs(target - hi) <= 1e-12:
                return list(points[index + 1]), index, True
            denom = max(1e-12, hi - lo)
            alpha = (target - lo) / denom
            return [float(a) + alpha * (float(b) - float(a)) for a, b in zip(points[index], points[index + 1])], index, False
    return list(points[-1]), len(points) - 2, True


def replace_path_section(points: list[list[float]], start_s: float, end_s: float, epsilon: float) -> list[list[float]]:
    lengths = cumulative_lengths(points)
    start_point, start_segment, start_is_vertex = interpolate_at(points, lengths, start_s)
    end_point, end_segment, end_is_vertex = interpolate_at(points, lengths, end_s)
    out: list[list[float]] = []
    out.extend(list(point) for point in points[: start_segment + 1])
    if not out or point_distance(out[-1], start_point) > epsilon:
        out.append(start_point)
    if point_distance(out[-1], end_point) > epsilon:
        out.append(end_point)
    tail_start = end_segment + (2 if end_is_vertex and abs(end_s - lengths[end_segment + 1]) <= 1e-10 else 1)
    out.extend(list(point) for point in points[tail_start:])
    deduped: list[list[float]] = []
    for point in out:
        if not deduped or point_distance(deduped[-1], point) > epsilon:
            deduped.append(point)
    return deduped


def shortcut_path(robot: Any, obstacles: list[Any], path: list[list[float]], step: float, passes: int, samples: int, epsilon: float) -> list[list[float]]:
    points = [list(point) for point in path]
    min_gain = max(1e-6, 10.0 * float(epsilon))
    sample_count = max(6, int(samples))
    for _ in range(max(0, int(passes))):
        if len(points) <= 2:
            break
        lengths = cumulative_lengths(points)
        total = lengths[-1]
        if total <= min_gain:
            break
        sample_positions = sorted(set(lengths + [total * index / float(sample_count - 1) for index in range(sample_count)]))
        changed = False
        for start_index, start_s in enumerate(sample_positions[:-2]):
            for end_s in reversed(sample_positions[start_index + 2:]):
                along = end_s - start_s
                if along <= min_gain:
                    continue
                start_point, _, _ = interpolate_at(points, lengths, start_s)
                end_point, _, _ = interpolate_at(points, lengths, end_s)
                direct = point_distance(start_point, end_point)
                if along - direct <= min_gain:
                    continue
                if not segment_free(robot, obstacles, start_point, end_point, step):
                    continue
                candidate = replace_path_section(points, start_s, end_s, float(epsilon))
                if len(candidate) >= 2 and euclidean_path_length(candidate) < euclidean_path_length(points) - min_gain and audit_path(robot, obstacles, candidate, step):
                    points = candidate
                    changed = True
                    break
            if changed:
                break
        if not changed:
            break
    return points


def post_audit_local_repair(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    step: float,
    *,
    start: list[float],
    goal: list[float],
    timeout_ms: float,
    trials: int,
    simplify_time_s: float,
    seed: int,
    epsilon: float,
) -> dict[str, Any]:
    points = [list(point) for point in path]
    if len(points) < 2:
        return {"ok": False, "path": points, "query_s": 0.0, "attempts": 0, "repaired_segments": 0}
    repaired = [list(points[0])]
    total_query_s = 0.0
    attempts = 0
    repaired_segments = 0
    for index in range(len(points) - 1):
        a = list(points[index])
        b = list(points[index + 1])
        if segment_free(robot, obstacles, a, b, float(step)):
            if point_distance(repaired[-1], b) > float(epsilon):
                repaired.append(b)
            continue
        segment_path: list[list[float]] | None = None
        for trial in range(max(1, int(trials))):
            attempts += 1
            raw = dict(sbf.ompl_rrt_connect_path(
                robot,
                obstacles,
                a,
                b,
                float(timeout_ms),
                0.35,
                float(step),
                float(simplify_time_s),
                int(seed) + 7919 * index + 104729 * trial,
            ))
            total_query_s += float(raw.get("t_s", 0.0) or 0.0)
            candidate = [list(point) for point in raw.get("path", [])]
            if bool(raw.get("ok")) and len(candidate) >= 2 and audit_path(robot, obstacles, candidate, float(step)):
                segment_path = candidate
                break
        if segment_path is None:
            return {
                "ok": False,
                "path": points,
                "query_s": float(total_query_s),
                "attempts": int(attempts),
                "repaired_segments": int(repaired_segments),
            }
        repaired.extend(list(point) for point in segment_path[1:])
        repaired_segments += 1
    if post_audit_path(robot, obstacles, repaired, float(step), start=list(start), goal=list(goal)):
        return {
            "ok": True,
            "path": repaired,
            "query_s": float(total_query_s),
            "attempts": int(attempts),
            "repaired_segments": int(repaired_segments),
            "path_length": float(euclidean_path_length(repaired)),
        }
    return {
        "ok": False,
        "path": points,
        "query_s": float(total_query_s),
        "attempts": int(attempts),
        "repaired_segments": int(repaired_segments),
    }


def apply_stage(args: argparse.Namespace, stage: dict[str, Any]) -> argparse.Namespace:
    args.max_boxes = int(stage["max_boxes"])
    args.ffb_depth = int(stage["ffb_depth"])
    args.quality_min_connected_boxes = int(stage["quality_min_connected_boxes"])
    args.post_connect_extra_boxes = int(stage["post_connect_extra_boxes"])
    args.post_connect_time_budget_ms = float(stage["post_connect_time_budget_ms"])
    if str(stage.get("stage_id", "")) == "seed":
        # The random-scene seed row is a minimal endpoint-certificate design
        # point. Random roots/anchors are useful for later coverage stages, but
        # generating them here adds hundreds of collision checks before the
        # grower can stop after the start/goal boxes.
        args.extra_random_roots = 0
        args.random_anchor_targets = 0
        args.anchor_target_prob = 0.0
        args.anchor_wave_targets_per_batch = 0
        args.root_seed_candidate_count = 0
        args.anchor_target_candidate_count = 0
    return args


def stage_seed_offset(stage: dict[str, Any]) -> int:
    stage_id = str(stage.get("stage_id", ""))
    if stage_id in LEGACY_SBF_STAGE_SEED_OFFSETS:
        return int(LEGACY_SBF_STAGE_SEED_OFFSETS[stage_id])
    return 9176 * int(stage.get("stage_index", 0))


def configure_warm(args: argparse.Namespace, seed: int, preset: str, namespace: str, robot: Any | None = None) -> Any:
    config_preset = RBF_LIFELONG_PRESET if str(preset) == "support_hull_coverage" else preset
    cfg = configure_standalone_sbf(args, seed, preset=config_preset, robot=robot)
    cfg.database.path = str(args.cache_root / namespace)
    cfg.database.create_if_missing = True
    cfg.database.verify_identity = True
    if unified_final_simplify_enabled(args):
        cfg.query.collision_shortcut = False
    return cfg


def random_prewarm_cache_label(args: argparse.Namespace, robot_name: str) -> str:
    return (
        f"exp06_{robot_name}_p{int(args.random_prewarm_depth)}_"
        f"{str(args.rbf_envelope)}_d{int(args.rbf_max_depth)}_canonical_native"
    )


def maybe_bind_random_external_prewarm(args: argparse.Namespace,
                                       cfg: Any,
                                       *,
                                       robot_name: str) -> dict[str, Any]:
    if not bool(getattr(args, "sbf_use_external_prewarm", True)):
        return {"external_prewarm_enabled": False}
    source = Path(args.cache_root) / random_prewarm_cache_label(args, robot_name)
    info = configure_external_evidence_reuse(
        cfg,
        source,
        args,
        materialization=True,
        scoring=True,
        backfill_active=False,
    )
    return {
        "external_prewarm_enabled": True,
        "external_prewarm_label": source.name,
        "external_prewarm_path": str(source),
        "external_prewarm_mode": info.get("mode"),
        "external_prewarm_snapshot_path": info.get("snapshot_path"),
    }


def canonical_point(robot: Any, point: list[float]) -> list[float]:
    if hasattr(sbf, "canonicalize_configuration_for_robot"):
        return [float(value) for value in sbf.canonicalize_configuration_for_robot(robot, list(point))]
    return [float(value) for value in point]


def random_leaf_refine_priority_points(robot: Any, start: list[float], goal: list[float]) -> list[list[float]]:
    points: list[list[float]] = []
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        point = [
            (1.0 - fraction) * float(a) + fraction * float(b)
            for a, b in zip(start, goal)
        ]
        points.append(canonical_point(robot, point))
    return points


def make_leaf_refine_config(args: argparse.Namespace) -> Any:
    refine_cfg = sbf.LeafSweepRefineConfig()
    refine_cfg.leaf_start_depth = int(args.leaf_start_depth)
    refine_cfg.leaf_max_depth = int(args.leaf_max_depth)
    refine_cfg.obstacle_cluster_gap = float(args.leaf_obstacle_cluster_gap)
    refine_cfg.use_virtual_topology = bool(args.leaf_virtual_topology)
    refine_cfg.parallel_virtual_validation = bool(args.leaf_parallel_virtual_validation)
    refine_cfg.store_group_results = bool(args.leaf_store_group_results)
    refine_cfg.validation_batch_size = max(1, int(args.leaf_validation_batch_size))
    refine_cfg.leaf_threads = max(1, int(args.leaf_threads) if int(args.leaf_threads) > 0 else int(args.threads))
    refine_cfg.leaf_timeout_ms = max(0.0, float(args.leaf_timeout_ms))
    refine_cfg.deep_max_boxes = max(0, int(args.max_boxes))
    refine_cfg.deep_ffb_depth = max(1, int(args.ffb_depth))
    refine_cfg.domain_seed_cap = max(0, int(args.leaf_domain_seed_cap))
    refine_cfg.domain_success_cap = max(0, int(args.leaf_domain_success_cap))
    refine_cfg.domain_attempt_cap = max(0, int(args.leaf_domain_attempt_cap))
    refine_cfg.allow_anchor_roots = bool(args.leaf_allow_anchor_roots)
    refine_cfg.refine_timeout_ms = max(0.0, float(args.post_connect_time_budget_ms))
    return refine_cfg


def run_sbf_build_backend(
    args: argparse.Namespace,
    forest: Any,
    robot: Any,
    scene: Any,
) -> tuple[Any, dict[str, Any]]:
    if str(args.sbf_build_backend) == "coverage":
        profile = forest.build_coverage(scene.obstacles, [scene.start, scene.goal])
        diagnostics = dict(getattr(profile, "diagnostics", {}) or {})
        return profile, {
            "sbf_build_backend": "coverage",
            "oracle_materialization_reused_external_evidence": float(diagnostics.get("oracle.materialization_reused_external_evidence", 0.0)),
            "oracle_node_validations": float(diagnostics.get("oracle.node_validations", 0.0)),
        }
    refine_cfg = make_leaf_refine_config(args)
    priority = random_leaf_refine_priority_points(robot, list(scene.start), list(scene.goal))
    result = forest.build_leaf_sweep_refined(scene.obstacles, refine_cfg, priority)
    diagnostics = dict(getattr(result.profile, "diagnostics", {}) or {})
    return result.profile, {
        "sbf_build_backend": "leaf_refine",
        "leaf_start_depth": int(refine_cfg.leaf_start_depth),
        "leaf_max_depth": int(refine_cfg.leaf_max_depth),
        "leaf_threads": int(refine_cfg.leaf_threads),
        "leaf_sweep_ms": float(result.leaf_sweep_ms),
        "leaf_free_count": int(result.leaf_free_count),
        "leaf_collision_count": int(result.leaf_collision_count),
        "deep_refine_ms": float(result.deep_refine_ms),
        "deep_boxes_added": int(result.deep_boxes_added),
        "deep_domain_attempts": int(result.deep_domain_attempts),
        "deep_ffb_success": int(result.deep_ffb_success),
        "deep_ffb_fail": int(result.deep_ffb_fail),
        "deep_commit_rejects": int(result.deep_commit_rejects),
        "deep_domain_rejects": int(result.deep_domain_rejects),
        "deep_contained_rejects": int(result.deep_contained_rejects),
        "deep_adjacency_rejects": int(result.deep_adjacency_rejects),
        "deep_anchor_roots_added": int(result.deep_anchor_roots_added),
        "connector_ms": float(result.connector_ms),
        "leaf_refine_total_ms": float(result.total_ms),
        "oracle_materialization_reused_external_evidence": float(diagnostics.get("oracle.materialization_reused_external_evidence", 0.0)),
        "oracle_node_validations": float(diagnostics.get("oracle.node_validations", 0.0)),
        "leaf_sweep_external_hits": float(diagnostics.get("leaf_sweep.worker_oracle.materialization_reused_external_evidence", 0.0)),
        "leaf_sweep_node_validations": float(diagnostics.get("leaf_sweep.worker_oracle.node_validations", 0.0)),
    }


def refine_random_corridor(forest: Any, start: list[float], goal: list[float], args: argparse.Namespace) -> tuple[float, int, int]:
    if not bool(args.sbf_corridor_refine):
        return 0.0, 0, 0
    budget_s = max(0.0, float(args.sbf_corridor_refine_budget_ms)) / 1000.0
    max_total = max(0, int(args.sbf_corridor_refine_max_boxes))
    per_query = max(1, int(args.sbf_corridor_refine_boxes_per_query))
    if budget_s <= 0.0 or max_total <= 0:
        return 0.0, 0, 0
    t0 = time.perf_counter()
    added_total = 0
    attempted = 0
    start_margin_s = max(0.0, float(args.sbf_corridor_refine_start_margin_ms)) / 1000.0
    for _ in range(max(1, int(args.sbf_corridor_refine_passes))):
        elapsed_s = time.perf_counter() - t0
        if added_total >= max_total or elapsed_s >= budget_s:
            break
        if attempted > 0 and budget_s - elapsed_s < start_margin_s:
            break
        quota = min(per_query, max_total - added_total)
        added = int(forest.refine_query_corridor(list(start), list(goal), quota))
        attempted += 1
        added_total += added
        if added <= 0:
            break
    return time.perf_counter() - t0, added_total, attempted


def attach_sbf_path(payload: dict[str, Any], result: Any) -> dict[str, Any]:
    if hasattr(result, "path_as_lists"):
        raw_waypoints = result.path_as_lists()
    else:
        raw_waypoints = getattr(result, "path", [])
    waypoints = [[float(value) for value in waypoint] for waypoint in raw_waypoints]
    payload["waypoints"] = waypoints
    payload["waypoint_count"] = len(waypoints)
    return payload


def run_sbf_bridge_retry(
    forest: Any,
    task_name: str,
    start: list[float],
    goal: list[float],
    *,
    prior_query_s: float,
    pre_bridge_ok: bool,
) -> dict[str, Any]:
    bridge_t0 = time.perf_counter()
    if hasattr(forest, "bridge_query_known_needed"):
        bridge_progress = int(forest.bridge_query_known_needed(list(start), list(goal)))
    else:
        bridge_progress = int(forest.bridge_query(list(start), list(goal)))
    bridge_s = time.perf_counter() - bridge_t0
    retry_t0 = time.perf_counter()
    retry = forest.query(list(start), list(goal))
    retry_s = time.perf_counter() - retry_t0
    payload = attach_sbf_path(query_result_payload(task_name, retry, float(prior_query_s) + bridge_s + retry_s), retry)
    payload["bridge_progress"] = int(bridge_progress)
    payload["bridge_time_s"] = float(bridge_s)
    payload["pre_bridge_ok"] = bool(pre_bridge_ok)
    return payload


def run_sbf_query_with_bridge(
    args: argparse.Namespace,
    forest: Any,
    task_name: str,
    start: list[float],
    goal: list[float],
) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(start), list(goal))
    query_s = time.perf_counter() - query_t0
    should_bridge = (not result.success and bool(args.sbf_bridge_failed_queries)) or (
        bool(args.sbf_bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if not should_bridge:
        payload = attach_sbf_path(query_result_payload(task_name, result, query_s), result)
        payload["bridge_progress"] = 0
        payload["bridge_time_s"] = 0.0
        payload["pre_bridge_ok"] = bool(result.success)
        return payload
    return run_sbf_bridge_retry(
        forest,
        task_name,
        start,
        goal,
        prior_query_s=query_s,
        pre_bridge_ok=bool(result.success),
    )


def sbf_cache_namespace(
    args: argparse.Namespace,
    *,
    robot_name: str,
    method: str,
    stage_id: str,
    difficulty: str | None = None,
    scene_seed: int | None = None,
    prewarm: bool = False,
) -> str:
    base = f"{args.cache_run_id}_{robot_name}_{method}_{stage_id}"
    if getattr(args, "sbf_cache_scope", "scene_stage") == "scene_stage":
        if prewarm:
            return f"{base}_prewarm"
        if difficulty is None or scene_seed is None:
            raise ValueError("scene_stage cache scope requires difficulty and scene_seed")
        return f"{base}_{difficulty}_seed{int(scene_seed)}"
    if difficulty is None:
        raise ValueError("disjoint_warm cache scope requires difficulty")
    if prewarm:
        return f"{base}_{difficulty}_prewarm"
    if scene_seed is None:
        raise ValueError("disjoint_warm cache scope requires scene_seed")
    return f"{base}_{difficulty}_eval_seed{int(scene_seed)}"


def cache_metrics(cache_root: Path, namespace: str) -> dict[str, Any]:
    directory = cache_root / namespace
    files = sorted(path for path in directory.rglob("*") if path.is_file()) if directory.exists() else []
    return {
        "cache_namespace": namespace,
        "cache_file_count": len(files),
        "cache_file_bytes": sum(path.stat().st_size for path in files),
    }


def seed_eval_cache_from_prewarm(args: argparse.Namespace, prewarm_namespace: str, eval_namespace: str) -> None:
    if getattr(args, "sbf_cache_scope", "scene_stage") != "disjoint_warm":
        return
    source = args.cache_root / prewarm_namespace
    target = args.cache_root / eval_namespace
    if target.exists():
        shutil.rmtree(target)
    if source.exists():
        shutil.copytree(source, target)
    else:
        target.mkdir(parents=True, exist_ok=True)


def prefixed_cache_metrics(cache_root: Path, namespace: str, prefix: str) -> dict[str, Any]:
    metrics = cache_metrics(cache_root, namespace)
    return {f"{prefix}_{key}": value for key, value in metrics.items()}


def prewarm_cache(args: argparse.Namespace, robot_name: str, method: str, stages: list[dict[str, Any]], difficulties: list[str]) -> list[dict[str, Any]]:
    robot = make_robot(robot_name)
    rows: list[dict[str, Any]] = []
    for difficulty in difficulties:
        for stage in stages:
            stage_args = argparse.Namespace(**vars(args))
            stage_args = apply_stage(stage_args, stage)
            namespace = sbf_cache_namespace(
                args,
                robot_name=robot_name,
                method=method,
                stage_id=str(stage["stage_id"]),
                difficulty=difficulty,
                prewarm=True,
            )
            total_build_s = 0.0
            success_count = 0
            for seed in range(max(0, int(args.prewarm_scene_seeds))):
                print(f"[random-anytime] prewarm robot={robot_name} difficulty={difficulty} method={method} stage={stage['stage_id']} scene_seed={seed}", flush=True)
                scene = make_random_scene(robot_name, difficulty, int(args.prewarm_seed_base) + 1009 * seed, scene_profile=args.scene_profile)
                cfg = configure_warm(stage_args, seed, method, namespace, robot=robot)
                maybe_bind_random_external_prewarm(stage_args, cfg, robot_name=robot_name)
                forest = sbf.SafeBoxForest(robot, cfg)
                t0 = time.perf_counter()
                run_sbf_build_backend(stage_args, forest, robot, scene)
                total_build_s += time.perf_counter() - t0
                success_count += 1
            metrics = cache_metrics(args.cache_root, namespace)
            rows.append({
                "robot": robot_name,
                "difficulty": difficulty,
                "method": method,
                "stage_id": stage["stage_id"],
                "namespace": namespace,
                "prewarm_scene_count": int(args.prewarm_scene_seeds),
                "prewarm_seed_base": int(args.prewarm_seed_base),
                "eval_seed_base": int(args.seed_base),
                "disjoint_from_eval": True,
                "prewarm_build_s": float(total_build_s),
                "prewarm_success_count": int(success_count),
                **metrics,
            })
    return rows


def run_scene_trace(
    args: argparse.Namespace,
    *,
    robot_name: str,
    difficulty: str,
    method: str,
    scene_seed: int,
    stages: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    robot, scene = scene_and_robot(args, robot_name, difficulty, scene_seed)
    incumbents: dict[str, dict[str, Any]] = {}
    cumulative_build_s = 0.0
    cumulative_query_s = 0.0
    records: list[dict[str, Any]] = []
    task_name = f"{robot_name}:{difficulty}:{scene_seed}"

    def finalize_sbf_query_payload(query_payload: dict[str, Any]) -> dict[str, Any]:
        strict_step = audit_segment_step(args)
        shortcut_info: dict[str, Any] = {}
        raw_path = [list(point) for point in query_payload.get("waypoints", [])]
        if (not unified_final_simplify_enabled(args)) and bool(query_payload.get("ok")) and len(raw_path) >= 2:
            shortened = shortcut_path(
                robot,
                scene.obstacles,
                raw_path,
                strict_step,
                int(args.sbf_query_shortcut_passes),
                int(args.sbf_query_shortcut_samples),
                float(args.epsilon_path),
            )
            if len(shortened) >= 2 and audit_path(robot, scene.obstacles, shortened, strict_step):
                shortened_length = euclidean_path_length(shortened)
                raw_length = float(query_payload.get("length", shortened_length))
                if shortened_length < raw_length - float(args.epsilon_path):
                    shortcut_info = {
                        "python_shortcut_applied": True,
                        "python_shortcut_length": float(shortened_length),
                        "python_shortcut_waypoints": len(shortened),
                        "raw_sbf_length": float(raw_length),
                    }
        if shortcut_info:
            query_payload.update(shortcut_info)
            query_payload["waypoints"] = [list(point) for point in shortened]
            query_payload["waypoint_count"] = len(query_payload["waypoints"])
            query_payload["length"] = float(shortcut_info["python_shortcut_length"])
            query_payload["audit_passed"] = True
            query_payload["audit_status"] = "PythonShortcutPassed"
        if bool(query_payload.get("ok")) and bool(query_payload.get("audit_passed")):
            final_simplify = final_ompl_simplify_path(
                sbf,
                robot,
                scene.obstacles,
                [list(point) for point in query_payload.get("waypoints", [])],
                segment_step=strict_step,
                audit_segment_step=strict_step,
                simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            query_payload["t_s"] = float(query_payload.get("t_s", 0.0) or 0.0) + float(final_simplify["query_s"])
            query_payload["waypoints"] = [list(point) for point in final_simplify["path"]]
            query_payload["waypoint_count"] = len(query_payload["waypoints"])
            if final_simplify["path_length"] is not None:
                query_payload["length"] = float(final_simplify["path_length"])
            query_payload["ompl_final_simplify_time_s"] = float(final_simplify["query_s"])
            query_payload["ompl_final_simplify_applied"] = bool(final_simplify["applied"])
            query_payload["ompl_final_simplify_reason"] = str(final_simplify["reason"])
        if bool(query_payload.get("ok")):
            query_payload["audit_passed"] = post_audit_path(
                robot,
                scene.obstacles,
                [list(point) for point in query_payload.get("waypoints", [])],
                strict_step,
                start=list(scene.start),
                goal=list(scene.goal),
            )
            if not bool(query_payload.get("audit_passed")):
                if bool(args.sbf_post_audit_local_repair):
                    repair = post_audit_local_repair(
                        robot,
                        scene.obstacles,
                        [list(point) for point in query_payload.get("waypoints", [])],
                        strict_step,
                        start=list(scene.start),
                        goal=list(scene.goal),
                        timeout_ms=float(args.sbf_post_audit_repair_timeout_ms),
                        trials=int(args.sbf_post_audit_repair_trials),
                        simplify_time_s=float(args.sbf_post_audit_repair_simplify_time_s),
                        seed=int(scene_seed) + 10000019 * int(query_payload.get("stage_restart", 0) or 0),
                        epsilon=float(args.epsilon_path),
                    )
                    query_payload["t_s"] = float(query_payload.get("t_s", 0.0) or 0.0) + float(repair["query_s"])
                    query_payload["post_audit_local_repair_query_s"] = float(repair["query_s"])
                    query_payload["post_audit_local_repair_attempts"] = int(repair["attempts"])
                    query_payload["post_audit_local_repair_segments"] = int(repair["repaired_segments"])
                    if bool(repair.get("ok")):
                        repaired_path = [list(point) for point in repair["path"]]
                        repair_final_simplify = final_ompl_simplify_path(
                            sbf,
                            robot,
                            scene.obstacles,
                            repaired_path,
                            segment_step=strict_step,
                            audit_segment_step=strict_step,
                            simplify_time_s=float(args.final_ompl_simplify_time_s),
                            epsilon_path=float(args.epsilon_path),
                        )
                        query_payload["t_s"] = float(query_payload.get("t_s", 0.0) or 0.0) + float(repair_final_simplify["query_s"])
                        query_payload["post_audit_local_repair_ompl_final_simplify_time_s"] = float(repair_final_simplify["query_s"])
                        query_payload["post_audit_local_repair_ompl_final_simplify_applied"] = bool(repair_final_simplify["applied"])
                        query_payload["post_audit_local_repair_ompl_final_simplify_reason"] = str(repair_final_simplify["reason"])
                        repaired_path = [list(point) for point in repair_final_simplify["path"]]
                        shortened = shortcut_path(
                            robot,
                            scene.obstacles,
                            repaired_path,
                            strict_step,
                            int(args.sbf_query_shortcut_passes),
                            int(args.sbf_query_shortcut_samples),
                            float(args.epsilon_path),
                        )
                        if (
                            len(shortened) >= 2
                            and euclidean_path_length(shortened) < euclidean_path_length(repaired_path) - float(args.epsilon_path)
                            and post_audit_path(
                                robot,
                                scene.obstacles,
                                shortened,
                                strict_step,
                                start=list(scene.start),
                                goal=list(scene.goal),
                            )
                        ):
                            query_payload["post_audit_local_repair_shortcut_applied"] = True
                            repaired_path = shortened
                        query_payload["waypoints"] = repaired_path
                        query_payload["waypoint_count"] = len(query_payload["waypoints"])
                        query_payload["length"] = float(euclidean_path_length(repaired_path))
                        query_payload["audit_passed"] = True
                        query_payload["audit_status"] = "PostAuditLocalRepairPassed"
                    else:
                        query_payload["audit_status"] = "PostAuditFailed"
                        query_payload["ok"] = False
                else:
                    query_payload["audit_status"] = "PostAuditFailed"
                    query_payload["ok"] = False
        return query_payload

    for stage_index, stage in enumerate(stages):
        stage_args = argparse.Namespace(**vars(args))
        stage_args = apply_stage(stage_args, stage)
        base_namespace = sbf_cache_namespace(
            args,
            robot_name=robot_name,
            method=method,
            stage_id=str(stage["stage_id"]),
            difficulty=difficulty,
            scene_seed=scene_seed,
        )
        prewarm_namespace = None
        if args.sbf_cache_scope == "disjoint_warm":
            prewarm_namespace = sbf_cache_namespace(
                args,
                robot_name=robot_name,
                method=method,
                stage_id=str(stage["stage_id"]),
                difficulty=difficulty,
                prewarm=True,
            )
        stage_build_s = 0.0
        stage_query_s = 0.0
        stage_tasks: list[dict[str, Any]] = []
        refine_s_total = 0.0
        refine_added_total = 0
        refine_attempts_total = 0
        box_counts: list[int] = []
        segment_edge_counts: list[int] = []
        profile_total_ms = 0.0
        build_metric_totals: dict[str, float] = {}
        restart_count = max(1, int(getattr(args, "sbf_stage_restarts", 1)))
        for restart in range(restart_count):
            namespace = base_namespace if restart_count == 1 else f"{base_namespace}_r{restart}"
            if prewarm_namespace is not None:
                seed_eval_cache_from_prewarm(args, prewarm_namespace, namespace)
            seed_offset = stage_seed_offset(stage)
            restart_seed = int(scene_seed) + 1000003 * int(restart) + int(seed_offset)
            cfg = configure_warm(stage_args, restart_seed, method, namespace, robot=robot)
            external_prewarm_info = maybe_bind_random_external_prewarm(stage_args, cfg, robot_name=robot_name)
            forest = sbf.SafeBoxForest(robot, cfg)
            build_t0 = time.perf_counter()
            profile, build_metrics = run_sbf_build_backend(stage_args, forest, robot, scene)
            refine_s, refine_added, refine_attempts = refine_random_corridor(forest, scene.start, scene.goal, stage_args)
            restart_build_s = time.perf_counter() - build_t0
            for metric_key, metric_value in build_metrics.items():
                if isinstance(metric_value, (int, float)):
                    build_metric_totals[metric_key] = build_metric_totals.get(metric_key, 0.0) + float(metric_value)
            query_payload = run_sbf_query_with_bridge(stage_args, forest, task_name, scene.start, scene.goal)
            query_payload["stage_restart"] = int(restart)
            query_payload["restart_seed"] = int(restart_seed)
            query_payload["cache_namespace"] = namespace
            query_payload = finalize_sbf_query_payload(query_payload)
            if (
                not bool(query_payload.get("ok"))
                and bool(query_payload.get("pre_bridge_ok"))
                and int(query_payload.get("bridge_progress", 0) or 0) == 0
                and bool(stage_args.sbf_bridge_failed_queries)
            ):
                retry_payload = run_sbf_bridge_retry(
                    forest,
                    task_name,
                    scene.start,
                    scene.goal,
                    prior_query_s=float(query_payload.get("t_s", 0.0) or 0.0),
                    pre_bridge_ok=True,
                )
                retry_payload["bridge_audit_fallback"] = True
                retry_payload = finalize_sbf_query_payload(retry_payload)
                if bool(retry_payload.get("ok")):
                    query_payload = retry_payload
                else:
                    query_payload["bridge_audit_fallback"] = True
                    query_payload["bridge_audit_fallback_ok"] = False
                    query_payload["bridge_audit_fallback_progress"] = int(retry_payload.get("bridge_progress", 0) or 0)
            restart_query_s = float(query_payload.get("t_s", 0.0))
            query_payload["stage_restart"] = int(restart)
            query_payload["restart_seed"] = int(restart_seed)
            query_payload["cache_namespace"] = namespace
            stage_tasks.append(task_result(
                name=task_name,
                ok=bool(query_payload.get("ok")),
                audit_passed=bool(query_payload.get("audit_passed")),
                path_length=float(query_payload.get("length", 0.0)) if query_payload.get("ok") else None,
                query_s=restart_query_s,
                reason=str(query_payload.get("audit_status", "")),
                extra={"raw": query_payload},
            ))
            stage_build_s += restart_build_s
            stage_query_s += restart_query_s
            refine_s_total += float(refine_s)
            refine_added_total += int(refine_added)
            refine_attempts_total += int(refine_attempts)
            box_counts.append(len(forest.boxes()))
            segment_edge_counts.append(len(forest.segment_edges()))
            profile_total_ms += float(profile.total_ms)
        incumbents, improved = update_incumbents(incumbents, stage_tasks, epsilon_path=float(args.epsilon_path))
        cumulative_build_s += stage_build_s
        cumulative_query_s += stage_query_s
        method_key = f"sbf_scene_stage_{method}" if args.sbf_cache_scope == "scene_stage" else f"sbf_warm_{method}"
        records.append(incumbent_stage_record(
            method=method_key,
            stage_id=str(stage["stage_id"]),
            stage_index=stage_index,
            seed_index=scene_seed,
            task_count=1,
            cumulative_build_s=cumulative_build_s,
            cumulative_query_s=cumulative_query_s,
            stage_build_s=stage_build_s,
            stage_query_s=stage_query_s,
            raw_tasks=stage_tasks,
            incumbents=incumbents,
            improved_tasks=improved,
            params={
                **stage,
                "robot": robot_name,
                "difficulty": difficulty,
                "scene_seed": int(scene_seed),
                "namespace": base_namespace,
                "cache_scope": str(args.sbf_cache_scope),
                "prewarm_namespace": prewarm_namespace,
                "disjoint_prewarm": bool(args.sbf_cache_scope == "disjoint_warm"),
                "prewarm_scene_seeds": int(args.prewarm_scene_seeds),
                "prewarm_seed_base": int(args.prewarm_seed_base),
                "eval_seed_base": int(args.seed_base),
                "stage_restarts": int(restart_count),
                "stage_seed_offset": int(stage_seed_offset(stage)),
                "box_count": max(box_counts) if box_counts else 0,
                "box_count_mean": float(sum(box_counts) / len(box_counts)) if box_counts else 0.0,
                "segment_edge_count": max(segment_edge_counts) if segment_edge_counts else 0,
                "build_profile_total_ms": float(profile_total_ms),
                "corridor_refine_time_s": float(refine_s_total),
                "corridor_refine_added_boxes": int(refine_added_total),
                "corridor_refine_attempts": int(refine_attempts_total),
                "bridge_failed_queries": bool(args.sbf_bridge_failed_queries),
                "bridge_repaired_queries": bool(args.sbf_bridge_repaired_queries),
                "sbf_build_backend": str(args.sbf_build_backend),
                **build_metric_totals,
                **external_prewarm_info,
                **prefixed_cache_metrics(args.cache_root, base_namespace, "eval_cache"),
            },
            protocol=f"{args.sbf_cache_scope}_cumulative_attempts",
        ))
    return records


def scene_and_robot(args: argparse.Namespace, robot_name: str, difficulty: str, scene_seed: int) -> tuple[Any, Any]:
    scene_cache = getattr(args, "scene_cache", None)
    if isinstance(scene_cache, dict):
        key = _scene_cache_key(robot_name, difficulty, int(scene_seed))
        if key in scene_cache:
            scene_record = scene_cache[key]
            if isinstance(scene_record, dict):
                return make_robot(robot_name), scene_from_record(
                    scene_record,
                    robot_name=robot_name,
                    difficulty=difficulty,
                    scene_seed=int(scene_seed),
                )
    robot = make_robot(robot_name)
    scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
    return robot, scene


def random_task_name(robot_name: str, difficulty: str, scene_seed: int, trial: int) -> str:
    return f"{robot_name}:{difficulty}:{scene_seed}:trial{trial}"


def run_random_prm_trace(
    args: argparse.Namespace,
    *,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    trial: int,
) -> list[dict[str, Any]]:
    robot, scene = scene_and_robot(args, robot_name, difficulty, scene_seed)
    task_name = random_task_name(robot_name, difficulty, scene_seed, trial)
    seed_index = scene_seed * max(1, int(args.baseline_trials)) + trial
    incumbents: dict[str, dict[str, Any]] = {}
    cumulative_build_s = 0.0
    cumulative_query_s = 0.0
    records: list[dict[str, Any]] = []
    strict_step = audit_segment_step(args)
    for stage_index, build_budget_s in enumerate(parse_float_grid(args.prm_build_grid_s)):
        rng_seed = (int(args.seed_base) + 73856093 * scene_seed + 19349663 * trial + 104729 * stage_index) % 2147483647
        raw = sbf.ompl_prm_multiquery(
            robot,
            scene.obstacles,
            [list(scene.start)],
            [list(scene.goal)],
            float(build_budget_s),
            float(args.prm_query_budget_s),
            strict_step,
            planner_internal_simplify_time(float(args.final_ompl_simplify_time_s), float(args.prm_simplify_time_s)),
            int(rng_seed),
            int(args.prm_max_nearest_neighbors),
        )
        query = dict((raw.get("queries") or [{}])[0])
        path = [list(point) for point in query.get("path", [])]
        exact = str(query.get("status")) == "Exact solution"
        ok = bool(query.get("ok")) and exact and len(path) >= 2
        query_s = float(query.get("t_s", 0.0)) if ok else 0.0
        if ok:
            final_simplify = final_ompl_simplify_path(
                sbf,
                robot,
                scene.obstacles,
                path,
                segment_step=strict_step,
                audit_segment_step=strict_step,
                simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            path = [list(point) for point in final_simplify["path"]]
            query_s += float(final_simplify["query_s"])
        audit_passed = post_audit_path(
            robot,
            scene.obstacles,
            path,
            strict_step,
            start=list(scene.start),
            goal=list(scene.goal),
        ) if ok else False
        task = task_result(
            name=task_name,
            ok=bool(audit_passed),
            audit_passed=bool(audit_passed),
            path_length=euclidean_path_length(path) if audit_passed else None,
            query_s=query_s,
            reason=query.get("reason") if audit_passed else "strict_post_audit_failed",
            extra={
                "status": query.get("status"),
                "waypoint_count": len(path),
                "waypoints": path,
                "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if ok else 0.0,
                "ompl_final_simplify_applied": bool(final_simplify["applied"]) if ok else False,
                "ompl_final_simplify_reason": str(final_simplify["reason"]) if ok else "disabled",
            },
        )
        incumbents, improved = update_incumbents(incumbents, [task], epsilon_path=float(args.epsilon_path))
        stage_build_s = float(raw.get("build_s", 0.0))
        stage_query_s = float(task.get("query_s", 0.0))
        cumulative_build_s += stage_build_s
        cumulative_query_s += stage_query_s
        records.append(incumbent_stage_record(
            method="ompl_prm",
            stage_id=f"build{build_budget_s:g}s",
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=1,
            cumulative_build_s=cumulative_build_s,
            cumulative_query_s=cumulative_query_s,
            stage_build_s=stage_build_s,
            stage_query_s=stage_query_s,
            raw_tasks=[task],
            incumbents=incumbents,
            improved_tasks=improved,
            params={
                "robot": robot_name,
                "difficulty": difficulty,
                "scene_seed": int(scene_seed),
                "trial": int(trial),
                "build_budget_s": float(build_budget_s),
                "rng_seed": int(rng_seed),
                "nodes": int(raw.get("nodes", 0) or 0),
            },
            protocol="cumulative_independent_shared_roadmaps",
        ))
    return records


def run_random_bitstar_trace(
    args: argparse.Namespace,
    *,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    trial: int,
) -> list[dict[str, Any]]:
    robot, scene = scene_and_robot(args, robot_name, difficulty, scene_seed)
    task_name = random_task_name(robot_name, difficulty, scene_seed, trial)
    seed_index = scene_seed * max(1, int(args.baseline_trials)) + trial
    incumbents: dict[str, dict[str, Any]] = {}
    records: list[dict[str, Any]] = []
    strict_step = audit_segment_step(args)
    timeout_s = float(args.bitstar_timeout_s if args.bitstar_budget_s is None else args.bitstar_budget_s)
    interval_s = float(args.bitstar_checkpoint_interval_s)
    rng_seed = (int(args.seed_base) + 73856093 * scene_seed + 19349663 * trial + 83492791) % 2147483647
    raw = sbf.ompl_bitstar_trace(
        robot,
        scene.obstacles,
        list(scene.start),
        list(scene.goal),
        timeout_s * 1000.0,
        interval_s * 1000.0,
        strict_step,
        int(rng_seed),
        int(args.bitstar_samples_per_batch),
        float(args.bitstar_rewire_factor),
        bool(args.bitstar_stop_on_solution_improvement),
    )
    previous_elapsed_s = 0.0
    for stage_index, checkpoint in enumerate([dict(item) for item in raw.get("checkpoints", [])]):
        path = [list(point) for point in checkpoint.get("path", [])]
        ok = bool(checkpoint.get("ok")) and len(path) >= 2
        query_s = float(checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0)) or 0.0)
        if ok:
            final_simplify = final_ompl_simplify_path(
                sbf,
                robot,
                scene.obstacles,
                path,
                segment_step=strict_step,
                audit_segment_step=strict_step,
                simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            path = [list(point) for point in final_simplify["path"]]
            query_s += float(final_simplify["query_s"])
        audit_passed = post_audit_path(
            robot,
            scene.obstacles,
            path,
            strict_step,
            start=list(scene.start),
            goal=list(scene.goal),
        ) if ok else False
        task = task_result(
            name=task_name,
            ok=bool(audit_passed),
            audit_passed=bool(audit_passed),
            path_length=euclidean_path_length(path) if audit_passed else None,
            query_s=query_s,
            reason=checkpoint.get("reason") if audit_passed else "strict_post_audit_failed",
            extra={
                "checkpoint_s": float(checkpoint.get("checkpoint_s", (stage_index + 1) * interval_s) or 0.0),
                "rng_seed": int(rng_seed),
                "waypoint_count": len(path),
                "waypoints": path,
                "iterations": int(checkpoint.get("iterations", 0) or 0),
                "batches": int(checkpoint.get("batches", 0) or 0),
                "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if ok else 0.0,
                "ompl_final_simplify_applied": bool(final_simplify["applied"]) if ok else False,
                "ompl_final_simplify_reason": str(final_simplify["reason"]) if ok else "disabled",
            },
        )
        tasks = [task]
        incumbents, improved = update_incumbents(incumbents, tasks, epsilon_path=float(args.epsilon_path))
        stage_query_s = max(0.0, query_s - previous_elapsed_s)
        previous_elapsed_s = query_s
        records.append(incumbent_stage_record(
            method="ompl_bitstar",
            stage_id=f"t{float(checkpoint.get('checkpoint_s', (stage_index + 1) * interval_s) or 0.0):g}s",
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=1,
            cumulative_build_s=0.0,
            cumulative_query_s=query_s,
            stage_build_s=0.0,
            stage_query_s=stage_query_s,
            raw_tasks=tasks,
            incumbents=incumbents,
            improved_tasks=improved,
            params={
                "robot": robot_name,
                "difficulty": difficulty,
                "scene_seed": int(scene_seed),
                "trial": int(trial),
                "timeout_s": float(timeout_s),
                "checkpoint_interval_s": float(interval_s),
                "checkpoint_s": float(checkpoint.get("checkpoint_s", (stage_index + 1) * interval_s) or 0.0),
            },
            protocol="fixed_timeout_checkpoint_trace",
        ))
    return records


def run_random_rrt_trace(
    args: argparse.Namespace,
    *,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    trial: int,
) -> list[dict[str, Any]]:
    robot, scene = scene_and_robot(args, robot_name, difficulty, scene_seed)
    strict_step = audit_segment_step(args)
    task_name = random_task_name(robot_name, difficulty, scene_seed, trial)
    seed_index = scene_seed * max(1, int(args.baseline_trials)) + trial
    incumbents: dict[str, dict[str, Any]] = {}
    rng_seed = (int(args.seed_base) + 7919 * trial + 104729 * scene_seed + 17 * scene_seed) % 2147483647
    raw = sbf.ompl_rrt_connect_path(
        robot,
        scene.obstacles,
        list(scene.start),
        list(scene.goal),
        float(args.rrt_timeout_ms),
        float(args.rrt_step_size),
        strict_step,
        planner_internal_simplify_time(float(args.final_ompl_simplify_time_s), float(args.rrt_simplify_time_s)),
        int(rng_seed),
    )
    path = [list(point) for point in raw.get("path", [])]
    exact = bool(raw.get("exact_solution")) or str(raw.get("status")) == "Exact solution"
    ok = bool(raw.get("ok")) and exact and len(path) >= 2
    query_s = float(raw.get("t_s", 0.0) or 0.0)
    if ok:
        final_simplify = final_ompl_simplify_path(
            sbf,
            robot,
            scene.obstacles,
            path,
            segment_step=strict_step,
            audit_segment_step=strict_step,
            simplify_time_s=float(args.final_ompl_simplify_time_s),
            epsilon_path=float(args.epsilon_path),
        )
        path = [list(point) for point in final_simplify["path"]]
        query_s += float(final_simplify["query_s"])
    audit_passed = post_audit_path(
        robot,
        scene.obstacles,
        path,
        strict_step,
        start=list(scene.start),
        goal=list(scene.goal),
    ) if ok else False
    task = task_result(
        name=task_name,
        ok=bool(audit_passed),
        audit_passed=bool(audit_passed),
        path_length=euclidean_path_length(path) if audit_passed else None,
        query_s=query_s,
        reason=raw.get("reason") if audit_passed else "strict_post_audit_failed",
        extra={
            "rng_seed": int(rng_seed),
            "timeout_ms": float(args.rrt_timeout_ms),
            "waypoint_count": len(path),
            "waypoints": path,
            "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if ok else 0.0,
            "ompl_final_simplify_applied": bool(final_simplify["applied"]) if ok else False,
            "ompl_final_simplify_reason": str(final_simplify["reason"]) if ok else "disabled",
        },
    )
    incumbents, improved = update_incumbents(incumbents, [task], epsilon_path=float(args.epsilon_path))
    return [incumbent_stage_record(
        method="ompl_rrtconnect",
        stage_id=f"timeout{float(args.rrt_timeout_ms) / 1000.0:g}s",
        stage_index=0,
        seed_index=seed_index,
        task_count=1,
        cumulative_build_s=0.0,
        cumulative_query_s=query_s,
        stage_build_s=0.0,
        stage_query_s=query_s,
        raw_tasks=[task],
        incumbents=incumbents,
        improved_tasks=improved,
        params={"robot": robot_name, "difficulty": difficulty, "scene_seed": int(scene_seed), "trial": int(trial), "timeout_ms": float(args.rrt_timeout_ms)},
        protocol="single_run_max_timeout",
    )]


def aggregate_panels(records: list[dict[str, Any]], epsilon_path: float) -> dict[str, Any]:
    panels: dict[str, dict[str, Any]] = {}
    for record in records:
        params = record.get("params", {})
        panel_key = f"{params.get('robot')}:{params.get('difficulty')}"
        panels.setdefault(panel_key, {"robot": params.get("robot"), "difficulty": params.get("difficulty"), "records": []})["records"].append(record)
    for panel in panels.values():
        summary = aggregate_stage_records(panel["records"], epsilon_path=epsilon_path)
        assert_promoted_monotone(summary, epsilon_path=epsilon_path)
        panel["summary"] = summary
    return panels


def main() -> int:
    args = parse_args()
    if args.scene_catalog is None:
        args.scene_catalog = args.out_json.parent / "random_scene_catalog.json"
    args.scene_cache, args.scene_catalog_metadata = load_or_build_scene_catalog(
        scene_catalog_path=Path(args.scene_catalog),
        robots=parse_csv(args.robots),
        difficulties=parse_csv(args.difficulties),
        scene_seeds=max(1, int(args.scene_seeds)),
        scene_profile=str(args.scene_profile),
        seed_base=int(args.seed_base),
        cache_mode=str(args.scene_catalog_mode),
    )
    stages = parse_sbf_stages(args.sbf_stages)
    robots = parse_csv(args.robots)
    difficulties = parse_csv(args.difficulties)
    methods = parse_csv(args.methods)
    baseline_methods = set(parse_csv(args.baseline_methods))
    unknown_baselines = sorted(method for method in baseline_methods if method not in {"rrt", "prm", "bitstar"})
    if unknown_baselines:
        raise ValueError(f"unknown baseline methods: {unknown_baselines}; choices=['rrt', 'prm', 'bitstar']")
    if args.clear_cache and args.cache_root.exists():
        shutil.rmtree(args.cache_root)
    prewarm_rows: list[dict[str, Any]] = []
    if args.sbf_cache_scope == "disjoint_warm" and int(args.prewarm_scene_seeds) > 0:
        for robot_name in robots:
            for method in methods:
                prewarm_rows.extend(prewarm_cache(args, robot_name, method, stages, difficulties))
    records: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            for method in methods:
                for scene_seed in range(max(1, int(args.scene_seeds))):
                    print(f"[random-anytime] robot={robot_name} difficulty={difficulty} method={method} scene_seed={scene_seed}", flush=True)
                    records.extend(run_scene_trace(
                        args,
                        robot_name=robot_name,
                        difficulty=difficulty,
                        method=method,
                        scene_seed=scene_seed,
                        stages=stages,
                    ))
            for scene_seed in range(max(1, int(args.scene_seeds))):
                for trial in range(max(1, int(args.baseline_trials))):
                    if "rrt" in baseline_methods:
                        print(f"[random-anytime] robot={robot_name} difficulty={difficulty} baseline=rrt scene_seed={scene_seed} trial={trial}", flush=True)
                        records.extend(run_random_rrt_trace(args, robot_name=robot_name, difficulty=difficulty, scene_seed=scene_seed, trial=trial))
                    if "prm" in baseline_methods:
                        print(f"[random-anytime] robot={robot_name} difficulty={difficulty} baseline=prm scene_seed={scene_seed} trial={trial}", flush=True)
                        records.extend(run_random_prm_trace(args, robot_name=robot_name, difficulty=difficulty, scene_seed=scene_seed, trial=trial))
                    if "bitstar" in baseline_methods:
                        print(f"[random-anytime] robot={robot_name} difficulty={difficulty} baseline=bitstar scene_seed={scene_seed} trial={trial}", flush=True)
                        records.extend(run_random_bitstar_trace(args, robot_name=robot_name, difficulty=difficulty, scene_seed=scene_seed, trial=trial))
    panels = aggregate_panels(records, float(args.epsilon_path))
    payload = {
        "experiment": "tro2026_random_anytime_tradeoff",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "scene_stage_cache_isolated_anytime_incumbent" if args.sbf_cache_scope == "scene_stage" else "warm_disjoint_cache_anytime_incumbent",
        "note": "SBF evaluation uses per-scene, per-stage cache namespaces to prevent disjoint random scenes from inflating charged build time." if args.sbf_cache_scope == "scene_stage" else "SBF cache is prewarmed on disjoint calibration random scenes, then evaluation scenes run warm staged builds while retaining audited incumbents. Prewarm cost is reported separately and is not included in per-scene warm charged time.",
        "params": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
            if key not in {"scene_cache"}
        },
        "scene_catalog": str(args.scene_catalog),
        "scene_catalog_mode": str(args.scene_catalog_mode),
        "scene_catalog_metadata": args.scene_catalog_metadata,
        "scene_filter_protocol": {
            "fixed_robot_exclusion_margin_m": float(FIXED_ROBOT_CLEARANCE_MARGIN_M),
            "endpoint_clearance_margin_m": float(ENDPOINT_CLEARANCE_MARGIN_M),
            "direct_segment_required_blocked": True,
            "direct_segment_resolution": int(SEGMENT_RESOLUTION),
            "balanced_probe_required": scene_profile_requires_balanced_probe(str(args.scene_profile)),
            "path_audit_segment_step": float(audit_segment_step(args)),
        },
        "prewarm": prewarm_rows,
        "panels": panels,
        "records": records,
        "method_build_query_semantics": {
            "sbf_scene_stage_support_hull_coverage": "per-scene, per-stage cache-isolated SBF staged builds with strict post-hoc path audit; charged time is cumulative by tier",
            "sbf_warm_support_hull_coverage": "warm-cache SBF staged builds with strict post-hoc path audit; cache is prewarmed on disjoint calibration scenes and evaluation charged time is cumulative by tier",
            "ompl_prm": "cumulative independent shared-roadmap attempts; all roadmap build and query time is charged; paths count only after strict post-hoc audit",
            "ompl_bitstar": "one fixed-timeout BIT* invocation per scene/trial with incumbent checkpoints at a fixed interval; each checkpoint counts strict-audited paths available by that elapsed time",
            "ompl_rrtconnect": "one query-only RRTConnect/BiRRT invocation per scene/trial with a maximum timeout; the planner returns on connection and timeout is failure",
        },
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "records": len(records), "panels": sorted(panels)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
