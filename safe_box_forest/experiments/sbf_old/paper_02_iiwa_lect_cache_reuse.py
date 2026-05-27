#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
from datetime import datetime
import json
import math
import os
import random
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Iterable

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, add_common_sbf_args, configure_standalone_sbf, mean, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_03_marcucci_envelope_build import (  # noqa: E402
    apply_cache_protocol,
    cache_metrics,
    endpoints_clear,
    first_link_clear,
    make_aabb,
    obstacle_bounds,
    parse_csv,
    random_obstacle,
    run_build_trial,
    safe_namespace,
    summarize_variant,
    unique_query_endpoints,
)
from sbf.marcucci import QueryPair, make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


SCENARIO_KEY = "shelf_iiwa"
RANDOM_SCENARIO_KEY = "random_obstacle_iiwa"
SCENE_NAME = "shelf_iiwa_marcucci_combined"
SCENE_LABEL = "Shelf+IIWA"
RANDOM_SCENE_NAME = "random_obstacle_iiwa"
RANDOM_SCENE_LABEL = "Random+IIWA"
STRICT_WARM_MODE = "strict"
GUIDED_EXTERNAL_WARM_MODE = "guided_external"
GUIDED_ACTIVE_WARM_MODE = "guided_active"
WARM_TARGET_MODES = (STRICT_WARM_MODE, GUIDED_EXTERNAL_WARM_MODE, GUIDED_ACTIVE_WARM_MODE)


def scene_name_for(scenario: str) -> str:
    return RANDOM_SCENE_NAME if scenario == RANDOM_SCENARIO_KEY else SCENE_NAME


def scene_label_for(scenario: str) -> str:
    return RANDOM_SCENE_LABEL if scenario == RANDOM_SCENARIO_KEY else SCENE_LABEL


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Shelf+IIWA LECT cold, blind-Warm prewarm, and same-scene replay protocol for the TRO main LECT table.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        threads=1,
        task_batch_size=1,
        max_boxes=2500,
        timeout_ms=5000.0,
        ffb_depth=80,
        component_connect_ffb_max_depth=200,
        quality_min_connected_boxes=256,
        post_connect_extra_boxes=0,
        post_connect_time_budget_ms=0.0,
        repair_timeout_ms=1500.0,
    )
    parser.add_argument("--variants", default="support_hull_coverage")
    parser.add_argument("--scenario", choices=[SCENARIO_KEY, RANDOM_SCENARIO_KEY], default=SCENARIO_KEY)
    parser.add_argument("--difficulties", default="", help="Deprecated compatibility option; Table II now always uses the Shelf+IIWA scenario.")
    parser.add_argument("--scene-seeds", type=int, default=5)
    parser.add_argument("--scene-seed-indices", default="1,3,5,8,10", help="Comma-separated build/RNG seed indices for repeated trials on the same Shelf+IIWA scene. Empty string falls back to range(--scene-seeds).")
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced", help="Deprecated compatibility option; ignored by the Shelf+IIWA Table II protocol.")
    parser.add_argument("--prewarm-budgets-ms", default="1000,2000,5000,10000,30000", help="Comma-separated blind prewarm budgets in milliseconds. For random_obstacle_ffb this is overridden by --random-ffb-scene-counts when that list is nonempty.")
    parser.add_argument("--include-matched-cross", action=argparse.BooleanOptionalAction, default=True, help="Compatibility name for the main Warm budget rows: for each time budget, build blind cache coverage, then time a fresh Shelf+IIWA target tree against that cache.")
    parser.add_argument("--warm-prewarm-mode", choices=["random_empty", "random_obstacle_ffb"], default="random_obstacle_ffb", help="Robot-space blind prewarm policy for Warm rows. random_empty builds empty-scene coverage from random joint-space seeds. random_obstacle_ffb runs FFB-only seeds in random obstacle scenes without target obstacles, queries, or route traces.")
    parser.add_argument("--warm-target-mode", choices=WARM_TARGET_MODES, default=STRICT_WARM_MODE, help="Warm target protocol. strict preserves the route-locked same-route comparison and keeps external scoring disabled by default. guided_external keeps a fresh active target tree but lets blind prewarm evidence accelerate materialization and split scoring. guided_active loads the blind prewarm LECT as the active target tree.")
    parser.add_argument("--matched-trajectory-reuse", action=argparse.BooleanOptionalAction, default=True, help="For the main Warm rows, use a fresh target LECT tree so the target build follows the Cold scene/search route unless route audit says otherwise.")
    parser.add_argument("--cross-external-evidence-materialization", action=argparse.BooleanOptionalAction, default=True, help="Warm target switch. Keep true for the paper-facing Warm row so blind prewarm evidence can serve target materialization without becoming the active target tree.")
    parser.add_argument("--cross-external-evidence-backfill-active", action=argparse.BooleanOptionalAction, default=False, help="Warm target switch. Keep false for the paper-facing Warm row so external hits do not write endpoint evidence into the active target LECT and perturb split scoring.")
    parser.add_argument("--replay-external-evidence-materialization", action=argparse.BooleanOptionalAction, default=True, help="Deprecated diagnostic side-cache switch; ignored by the main active-LECT Table II path.")
    parser.add_argument("--cross-external-evidence-scoring", action=argparse.BooleanOptionalAction, default=False, help="Diagnostic switch. Keep false for the paper-facing Warm row; enabling it lets prewarmed evidence affect split scoring and can change the Cold route.")
    parser.add_argument("--stateless-materialization-context", action=argparse.BooleanOptionalAction, default=False, help="Reset the LECT materializer's incremental endpoint context around each node materialization. Route locking no longer requires this slower diagnostic mode.")
    parser.add_argument("--replay-external-evidence-scoring", action=argparse.BooleanOptionalAction, default=True, help="Deprecated diagnostic side-cache switch; ignored by the main active-LECT Table II path.")
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True, help="For Shelf+IIWA Table II, bridge failed query corridors and retry, matching the Shelf paper-facing query path.")
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True, help="For Shelf+IIWA Table II, bridge repaired disconnected query corridors and retry, matching the Shelf paper-facing query path.")
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True, help="Run the Shelf+IIWA corridor-refinement pass after coverage construction and before timed queries.")
    parser.add_argument("--corridor-refine-deterministic", action=argparse.BooleanOptionalAction, default=False, help="Diagnostic option: use a fixed query/pass/box schedule for the Shelf+IIWA corridor-refinement pass instead of the audited paper-facing time-budgeted pass.")
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=2)
    parser.add_argument("--corridor-refine-start-margin-ms", type=float, default=120.0)
    parser.add_argument("--corridor-refine-defer-labels", default="CS->LB")
    parser.add_argument("--include-cross-diagnostic", action=argparse.BooleanOptionalAction, default=False, help="Run cross-scene cache-transfer diagnostics. These rows are not used by the main LECT replay table because endpoint-only cross transfer is not a stable build-time acceleration regime.")
    parser.add_argument("--cross-prewarm-mode", choices=["route_empty"], default="route_empty", help="Cross-scene offline prewarm protocol. route_empty builds the target query in an empty scene before timing the target obstacles.")
    parser.add_argument("--random-prewarm-seeds", type=int, default=1, help="Robot-space random coverage seeds per second for --warm-prewarm-mode random_empty when --random-prewarm-scale-with-budget is enabled; otherwise the fixed seed count for every budget.")
    parser.add_argument("--random-prewarm-scale-with-budget", action=argparse.BooleanOptionalAction, default=True, help="Scale random_empty seed count linearly with the prewarm budget.")
    parser.add_argument("--random-prewarm-sequence", choices=["random", "halton"], default="halton", help="Blind robot-space sample sequence for random_empty prewarm. halton is deterministic prefix-stable low-discrepancy coverage; random is pseudo-random uniform sampling.")
    parser.add_argument("--random-prewarm-query-pairs", type=int, default=0, help="Blind robot-space random query pairs per second for random_empty prewarm. Queries run in an empty scene and do not use target obstacles or target routes.")
    parser.add_argument("--random-prewarm-max-boxes", type=int, default=20000, help="Max boxes for random_empty prewarm; set <=0 to inherit --max-boxes.")
    parser.add_argument("--random-prewarm-seed-base", type=int, default=20280513, help="Seed base for robot-space random prewarm sampling.")
    parser.add_argument("--random-ffb-seeds", type=int, default=384, help="Robot-space random FFB seeds per scene-second for --warm-prewarm-mode random_obstacle_ffb.")
    parser.add_argument("--random-ffb-obstacles", type=int, default=12, help="Number of random workspace obstacles for random_obstacle_ffb prewarm.")
    parser.add_argument("--random-ffb-difficulty", choices=["easy", "medium", "hard"], default="medium", help="Obstacle size distribution for random_obstacle_ffb prewarm.")
    parser.add_argument("--random-ffb-obstacle-profile", choices=["global", "shelf_like"], default="shelf_like", help="Blind random obstacle distribution for random_obstacle_ffb. shelf_like samples randomized panels, dividers, bins, and slabs in the robot workspace without using target routes or queries.")
    parser.add_argument("--random-ffb-seed-base", type=int, default=2028051301, help="Seed base for random_obstacle_ffb obstacle and seed generation.")
    parser.add_argument("--target-random-obstacles", type=int, default=0, help="Number of random target obstacles for --scenario random_obstacle_iiwa. 0 reuses --random-ffb-obstacles.")
    parser.add_argument("--target-random-seed-base", type=int, default=2029051301, help="Independent seed base for target random obstacle scenes.")
    parser.add_argument("--target-random-max-tries", type=int, default=20000, help="Maximum obstacle placement attempts for endpoint-clear random target scenes.")
    parser.add_argument("--random-ffb-scene-counts", default="1,2,5,10,30", help="Comma-separated random-obstacle scene counts used as the 5 budget levels for random_obstacle_ffb. Each scene receives --random-ffb-ms-per-scene milliseconds.")
    parser.add_argument("--random-ffb-ms-per-scene", type=float, default=1000.0, help="Per-random-scene prewarm budget in milliseconds for --random-ffb-scene-counts.")
    parser.add_argument("--random-ffb-scenes-per-10s", type=float, default=0.0, help="Number of independent random obstacle FFB prewarm scenes per 10 seconds of budget. Values <=0 keep a single blind random scene for all budgets.")
    parser.add_argument("--external-scoring-cover-ratio", type=float, default=1000.0, help="Max volume ratio for scoring-only covering external evidence.")
    parser.add_argument("--external-scoring-min-iou", type=float, default=0.0015, help="Minimum interval IoU for scoring-only overlapping external evidence; set <=0 to disable overlap fallback.")
    parser.add_argument("--external-scoring-overlap-max-nodes", type=int, default=5000, help="Max external LECT nodes visited per scoring overlap lookup; set <=0 to disable overlap fallback.")
    parser.add_argument("--external-materialization-cover-ratio", type=float, default=0.0, help="Max volume ratio for conservative covering external evidence during materialization; set <=0 to require exact materialization evidence.")
    parser.add_argument("--cross-random-obstacles", type=int, default=10)
    parser.add_argument("--cross-random-seed-base", type=int, default=None, help="Deprecated compatibility option; by default evaluation scenes use --seed-base to match paper_15_random_anytime_tradeoff.py.")
    parser.add_argument("--cross-random-blocked-seeds", default="20262523")
    parser.add_argument("--cross-random-max-tries", type=int, default=20000)
    parser.add_argument("--cross-clearance", type=float, default=0.12)
    parser.add_argument("--cross-first-link-clearance", type=float, default=0.02)
    parser.add_argument("--grid-pad-policy", choices=["strict_half_diagonal", "no_extra_pad"], default="strict_half_diagonal")
    parser.add_argument("--storage-profile", choices=["compact", "balanced", "fast_query"], default="fast_query")
    parser.add_argument("--cache-root", type=Path, default=ROOT / "outputs" / "paper" / "lect_cache_iiwa_incremental")
    parser.add_argument("--cache-run-id", default=None)
    parser.add_argument("--clear-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--cached-replay-worker-local-ffb", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--cached-replay-best-tighten", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--cross-detach-cache-tree", action=argparse.BooleanOptionalAction, default=False, help="Diagnostic option: detach the copied prewarm namespace from the active target LECT tree.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_iiwa_lect_incremental_reuse.json")
    return parser.parse_args()


def parse_float_csv(text: str) -> list[float]:
    values = []
    for item in parse_csv(text):
        values.append(float(item))
    return values


def parse_int_csv(text: str) -> list[int]:
    values = []
    for item in parse_csv(text):
        values.append(int(item))
    return values


def budget_label(budget_ms: float) -> str:
    if abs(budget_ms - round(budget_ms)) < 1e-9:
        return f"b{int(round(budget_ms)):04d}ms"
    return "b" + (f"{budget_ms:.3f}".rstrip("0").rstrip(".")).replace(".", "p") + "ms"


def mean_present(values: Iterable[float | None]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return mean(rows) if rows else None


def diagnostic_value(trial: dict[str, Any], key: str) -> float:
    return float((trial.get("diagnostics") or {}).get(key, 0.0))


def validation_hit_rate(trial: dict[str, Any]) -> float | None:
    hits = diagnostic_value(trial, "oracle.validation_cache_hits")
    misses = diagnostic_value(trial, "oracle.validation_cache_misses")
    total = hits + misses
    if total <= 0.0:
        return None
    return hits / total


def lect_materialization_hit_rate(trial: dict[str, Any]) -> float | None:
    materializations = diagnostic_value(trial, "oracle.materializations")
    if materializations <= 0.0:
        return None
    hits = diagnostic_value(trial, "oracle.materialization_reused_external_evidence")
    return hits / materializations


def lect_endpoint_hit_rate(trial: dict[str, Any]) -> float | None:
    materializations = diagnostic_value(trial, "oracle.materializations")
    if materializations <= 0.0:
        return None
    hits = diagnostic_value(trial, "oracle.materialization_reused_endpoint_cache")
    return hits / materializations


def scoring_external_hit_rate(trial: dict[str, Any]) -> float | None:
    evaluations = diagnostic_value(trial, "oracle.scoring_evaluations")
    if evaluations <= 0.0:
        return None
    hits = diagnostic_value(trial, "oracle.scoring_reused_external_evidence")
    return hits / evaluations


def copy_cache_namespace(cache_root: Path, source_namespace: str, target_namespace: str) -> None:
    source = cache_root / source_namespace
    target = cache_root / target_namespace
    if target.exists():
        shutil.rmtree(target)
    if source.exists():
        shutil.copytree(source, target)
    else:
        target.mkdir(parents=True, exist_ok=True)


def progress(message: str) -> None:
    print(f"[exp2] {message}", flush=True)


def random_robot_seeds(robot: Any, count: int, seed: int) -> list[list[float]]:
    rng = random.Random(int(seed))
    limits = list(robot.joint_limits().limits)
    rows: list[list[float]] = []
    for _ in range(max(1, int(count))):
        rows.append([rng.uniform(float(limit.lo), float(limit.hi)) for limit in limits])
    return rows


def halton_unit(index: int, base: int) -> float:
    value = 0.0
    fraction = 1.0 / float(base)
    current = int(index)
    while current > 0:
        digit = current % base
        value += digit * fraction
        current //= base
        fraction /= float(base)
    return value


def halton_robot_seeds(robot: Any, count: int, skip: int) -> list[list[float]]:
    primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43]
    limits = list(robot.joint_limits().limits)
    if len(limits) > len(primes):
        raise ValueError(f"Halton prewarm supports up to {len(primes)} dimensions, got {len(limits)}")
    rows: list[list[float]] = []
    start_index = max(1, int(skip) + 1)
    for row_index in range(max(1, int(count))):
        halton_index = start_index + row_index
        row: list[float] = []
        for dim, limit in enumerate(limits):
            unit = halton_unit(halton_index, primes[dim])
            row.append(float(limit.lo) + unit * (float(limit.hi) - float(limit.lo)))
        rows.append(row)
    return rows


def robot_space_samples(args: argparse.Namespace, robot: Any, count: int) -> list[list[float]]:
    if str(args.random_prewarm_sequence) == "halton":
        return halton_robot_seeds(robot, count, int(args.random_prewarm_seed_base))
    return random_robot_seeds(robot, count, int(args.random_prewarm_seed_base))


def blind_robot_query_pairs(samples: list[list[float]], count: int) -> list[QueryPair]:
    queries: list[QueryPair] = []
    for query_index in range(max(0, int(count))):
        start = tuple(float(value) for value in samples[2 * query_index])
        goal = tuple(float(value) for value in samples[2 * query_index + 1])
        queries.append(QueryPair(
            label=f"blind-random-{query_index:03d}",
            start_name=f"R{2 * query_index:03d}",
            goal_name=f"R{2 * query_index + 1:03d}",
            start=start,
            goal=goal,
        ))
    return queries


def run_random_empty_prewarm_trial(
    args: argparse.Namespace,
    *,
    robot: Any,
    variant: str,
    difficulty: str,
    budget_ms: float,
    cache_namespace: str,
) -> dict[str, Any]:
    progress(f"start variant={variant} protocol=random_empty_prewarm scenario={difficulty} budget_ms={budget_ms:g} cache={cache_namespace}")
    cache_dir = args.cache_root / cache_namespace
    if cache_dir.exists():
        shutil.rmtree(cache_dir)
    prewarm_args = copy.copy(args)
    prewarm_args.timeout_ms = max(1.0, float(budget_ms))
    if int(args.random_prewarm_max_boxes) > 0:
        prewarm_args.max_boxes = int(args.random_prewarm_max_boxes)
    seed = int(args.random_prewarm_seed_base)
    if bool(args.random_prewarm_scale_with_budget):
        seed_count = max(1, int(round(float(args.random_prewarm_seeds) * max(1.0, float(budget_ms)) / 1000.0)))
        query_pair_count = max(0, int(round(float(args.random_prewarm_query_pairs) * max(1.0, float(budget_ms)) / 1000.0)))
    else:
        seed_count = max(1, int(args.random_prewarm_seeds))
        query_pair_count = max(0, int(args.random_prewarm_query_pairs))
    samples = robot_space_samples(args, robot, seed_count + 2 * query_pair_count)
    queries = blind_robot_query_pairs(samples[:2 * query_pair_count], query_pair_count)
    seeds = samples[2 * query_pair_count:]
    metadata = {
        "scene_kind": "robot_space_random_empty_prewarm",
        "robot": "iiwa",
        "scenario": difficulty,
        "scene_label": scene_label_for(difficulty),
        "prewarm_starts_empty": True,
        "random_seed": seed,
        "random_seed_count": len(seeds),
        "random_query_pair_count": len(queries),
        "random_sequence": str(args.random_prewarm_sequence),
        "charged_to_reuse_build_time": False,
    }
    trial = run_build_trial(
        prewarm_args,
        robot=robot,
        variant=variant,
        seed=0,
        protocol="random_empty_prewarm",
        scene_name="empty",
        obstacles=[],
        scene_metadata=metadata,
        seeds=seeds,
        queries=queries,
        cache_namespace=cache_namespace,
        prewarm={
            "protocol": "robot_space_random_empty_prewarm",
            "difficulty": difficulty,
            "budget_ms": float(budget_ms),
            "budget_label": budget_label(budget_ms),
            "random_seed": seed,
            "random_seed_count": len(seeds),
            "random_query_pair_count": len(queries),
            "random_seeds_per_second": int(args.random_prewarm_seeds),
            "random_query_pairs_per_second": int(args.random_prewarm_query_pairs),
            "random_seed_count_scales_with_budget": bool(args.random_prewarm_scale_with_budget),
            "random_sequence": str(args.random_prewarm_sequence),
            "charged_to_reuse_build_time": False,
        },
    )
    trial["protocol"] = "random_empty_prewarm"
    trial["phase"] = "random_empty_prewarm"
    trial["difficulty"] = difficulty
    trial["scenario"] = difficulty
    trial["budget_ms"] = float(budget_ms)
    trial["budget_label"] = budget_label(budget_ms)
    trial["prewarm_is_blind"] = True
    return trial


def random_shelf_like_obstacle(rng: random.Random, difficulty: str) -> Any:
    scale = {"easy": 0.8, "medium": 1.0, "hard": 1.25}.get(difficulty, 1.0)
    kind = rng.random()
    if kind < 0.28:
        cx = rng.uniform(0.68, 1.02)
        cy = rng.choice([-1.0, 1.0]) * rng.uniform(0.24, 0.34)
        cz = rng.uniform(0.20, 0.55)
        return make_aabb(cx, cy, cz, rng.uniform(0.08, 0.18) * scale, rng.uniform(0.008, 0.025) * scale, rng.uniform(0.24, 0.42) * scale)
    if kind < 0.56:
        cx = rng.uniform(0.70, 0.98)
        cy = rng.uniform(-0.25, 0.25)
        cz = rng.choice([0.27, 0.53, 0.80]) + rng.uniform(-0.035, 0.035)
        return make_aabb(cx, cy, cz, rng.uniform(0.12, 0.23) * scale, rng.uniform(0.24, 0.35) * scale, rng.uniform(0.006, 0.022) * scale)
    if kind < 0.84:
        cx = rng.uniform(-0.30, 0.30)
        cy = rng.choice([-1.0, 1.0]) * rng.uniform(0.36, 0.84)
        cz = rng.uniform(0.06, 0.14)
        return make_aabb(cx, cy, cz, rng.uniform(0.20, 0.34) * scale, rng.uniform(0.012, 0.035) * scale, rng.uniform(0.06, 0.14) * scale)
    cx = rng.uniform(-0.20, 0.70)
    cy = rng.uniform(-0.90, 0.90)
    cz = rng.uniform(-0.28, -0.08)
    return make_aabb(cx, cy, cz, rng.uniform(0.35, 1.10) * scale, rng.uniform(0.35, 1.10) * scale, rng.uniform(0.04, 0.12) * scale)


def make_blind_random_obstacles(args: argparse.Namespace, count: int, scene_index: int = 0) -> list[Any]:
    rng = random.Random(int(args.random_ffb_seed_base) + 1009 * int(scene_index))
    if str(args.random_ffb_obstacle_profile) == "shelf_like":
        return [random_shelf_like_obstacle(rng, str(args.random_ffb_difficulty)) for _ in range(max(0, int(count)))]
    return [random_obstacle(rng, str(args.random_ffb_difficulty)) for _ in range(max(0, int(count)))]


def sample_profile_obstacle(rng: random.Random, args: argparse.Namespace) -> Any:
    if str(args.random_ffb_obstacle_profile) == "shelf_like":
        return random_shelf_like_obstacle(rng, str(args.random_ffb_difficulty))
    return random_obstacle(rng, str(args.random_ffb_difficulty))


def make_random_target_obstacles(
    args: argparse.Namespace,
    robot: Any,
    queries: list[Any],
    seed_index: int,
) -> tuple[list[Any], dict[str, Any]]:
    target_count = int(args.target_random_obstacles) if int(args.target_random_obstacles) > 0 else int(args.random_ffb_obstacles)
    actual_seed = int(args.target_random_seed_base) + 1009 * int(seed_index)
    rng = random.Random(actual_seed)
    endpoints = unique_query_endpoints(queries)
    obstacles: list[Any] = []
    attempts = 0
    first_link_rejections = 0
    while len(obstacles) < target_count and attempts < int(args.target_random_max_tries):
        attempts += 1
        candidate = sample_profile_obstacle(rng, args)
        proposed = [*obstacles, candidate]
        if not endpoints_clear(robot, proposed, endpoints, float(args.cross_clearance)):
            continue
        if not first_link_clear(proposed, endpoints, float(args.cross_first_link_clearance)):
            first_link_rejections += 1
            continue
        obstacles.append(candidate)
    if len(obstacles) < target_count:
        raise RuntimeError(
            f"could place only {len(obstacles)}/{target_count} random target obstacles while preserving "
            f"endpoint clearance {args.cross_clearance}"
        )
    metadata = {
        "scene_kind": RANDOM_SCENE_NAME,
        "scene": RANDOM_SCENE_NAME,
        "scene_label": RANDOM_SCENE_LABEL,
        "robot": "iiwa",
        "seed_index": int(seed_index),
        "seed": actual_seed,
        "random_obstacle_profile": str(args.random_ffb_obstacle_profile),
        "random_obstacle_difficulty": str(args.random_ffb_difficulty),
        "requested_obstacles": target_count,
        "obstacle_count": len(obstacles),
        "placement_attempts": attempts,
        "endpoint_count": len(endpoints),
        "clearance": float(args.cross_clearance),
        "first_link_clearance": float(args.cross_first_link_clearance),
        "endpoint_clearance_ok": endpoints_clear(robot, obstacles, endpoints, float(args.cross_clearance)),
        "first_link_clearance_ok": first_link_clear(obstacles, endpoints, float(args.cross_first_link_clearance)),
        "first_link_rejections": first_link_rejections,
        "obstacle_bounds": [obstacle_bounds(obstacle) for obstacle in obstacles],
    }
    return obstacles, metadata


def merge_trial_diagnostics(trials: list[dict[str, Any]]) -> dict[str, float]:
    merged: dict[str, float] = {}
    for trial in trials:
        for key, value in trial.get("diagnostics", {}).items():
            try:
                merged[str(key)] = merged.get(str(key), 0.0) + float(value)
            except (TypeError, ValueError):
                continue
    return merged


def run_random_obstacle_ffb_prewarm_trial(
    args: argparse.Namespace,
    *,
    robot: Any,
    variant: str,
    difficulty: str,
    budget_ms: float,
    cache_namespace: str,
    scene_count: int | None = None,
) -> dict[str, Any]:
    progress(f"start variant={variant} protocol=random_obstacle_ffb_prewarm scenario={difficulty} budget_ms={budget_ms:g} cache={cache_namespace}")
    cache_dir = args.cache_root / cache_namespace
    if cache_dir.exists():
        shutil.rmtree(cache_dir)
    if scene_count is not None:
        scene_count = max(1, int(scene_count))
        per_scene_budget_ms = max(1.0, float(args.random_ffb_ms_per_scene))
        per_scene_seed_count = max(1, int(round(float(args.random_ffb_seeds) * per_scene_budget_ms / 1000.0)))
        total_seed_count = per_scene_seed_count * scene_count
    else:
        scene_rate = float(args.random_ffb_scenes_per_10s)
        scene_count = 1 if scene_rate <= 0.0 else max(1, int(round(scene_rate * max(1.0, float(budget_ms)) / 10000.0)))
        total_seed_count = max(1, int(round(float(args.random_ffb_seeds) * max(1.0, float(budget_ms)) / 1000.0)))
        per_scene_seed_count = max(1, int(math.ceil(total_seed_count / float(scene_count))))
        per_scene_budget_ms = max(1.0, float(budget_ms) / float(scene_count))
    scene_trials: list[dict[str, Any]] = []
    last_trial: dict[str, Any] | None = None
    build_s_total = 0.0
    prebridge_time_s_total = 0.0
    prebridge_added_total = 0
    prebridge_attempts_total = 0
    n_boxes_total = 0
    raw_boxes_total = 0
    final_boxes_total = 0
    grow_ms_total = 0.0
    total_ms_total = 0.0
    cache_prebuild_files: dict[str, Any] | None = None
    for scene_index in range(scene_count):
        scene_args = copy.copy(args)
        scene_args.random_prewarm_seed_base = int(args.random_prewarm_seed_base) + 1000003 * scene_index
        seeds = robot_space_samples(scene_args, robot, per_scene_seed_count)
        obstacles = make_blind_random_obstacles(args, int(args.random_ffb_obstacles), scene_index)
        prewarm_args = copy.copy(args)
        prewarm_args.timeout_ms = per_scene_budget_ms
        prewarm_args.max_boxes = per_scene_seed_count
        prewarm_args.max_consecutive_miss = 0
        prewarm_args.enable_connector = False
        prewarm_args.enable_merger = False
        prewarm_args.bridge_failed_queries = False
        prewarm_args.bridge_repaired_queries = False
        prewarm_args.post_connect_extra_boxes = 0
        metadata = {
            "scene_kind": "blind_random_obstacle_ffb_prewarm",
            "robot": "iiwa",
            "scenario": difficulty,
            "scene_label": scene_label_for(difficulty),
            "prewarm_starts_empty": scene_index == 0,
            "random_seed": int(args.random_ffb_seed_base) + 1009 * scene_index,
            "random_seed_count": len(seeds),
            "random_sequence": str(args.random_prewarm_sequence),
            "random_obstacle_count": len(obstacles),
            "random_obstacle_difficulty": str(args.random_ffb_difficulty),
            "random_obstacle_profile": str(args.random_ffb_obstacle_profile),
            "random_scene_index": scene_index,
            "random_scene_count": scene_count,
            "random_ms_per_scene": float(per_scene_budget_ms),
            "ffb_only": True,
            "charged_to_reuse_build_time": False,
        }
        trial_part = run_build_trial(
            prewarm_args,
            robot=robot,
            variant=variant,
            seed=scene_index,
            protocol="random_obstacle_ffb_prewarm",
            scene_name=f"blind_random_obstacle_ffb_{scene_index}",
            obstacles=obstacles,
            scene_metadata=metadata,
            seeds=seeds,
            queries=[],
            cache_namespace=cache_namespace,
            prewarm={
                "protocol": "blind_random_obstacle_ffb_prewarm",
                "difficulty": difficulty,
                "budget_ms": float(per_scene_budget_ms),
                "budget_label": budget_label(per_scene_budget_ms),
                "random_seed": int(args.random_ffb_seed_base) + 1009 * scene_index,
                "random_seed_count": len(seeds),
                "random_seeds_per_second": int(args.random_ffb_seeds),
                "random_sequence": str(args.random_prewarm_sequence),
                "random_obstacle_count": len(obstacles),
                "random_obstacle_difficulty": str(args.random_ffb_difficulty),
                "random_obstacle_profile": str(args.random_ffb_obstacle_profile),
                "random_scene_index": scene_index,
                "random_scene_count": scene_count,
                "random_ms_per_scene": float(per_scene_budget_ms),
                "ffb_only": True,
                "charged_to_reuse_build_time": False,
            },
        )
        if cache_prebuild_files is None:
            cache_prebuild_files = {
                "prebuild_cache_file_count": int(trial_part.get("prebuild_cache_file_count", 0)),
                "prebuild_cache_file_bytes": int(trial_part.get("prebuild_cache_file_bytes", 0)),
                "prebuild_cache_files": list(trial_part.get("prebuild_cache_files", [])),
            }
        build_s_total += float(trial_part.get("build_s", 0.0))
        prebridge_time_s_total += float(trial_part.get("prebridge_time_s", 0.0))
        prebridge_added_total += int(trial_part.get("prebridge_added_boxes", 0))
        prebridge_attempts_total += int(trial_part.get("prebridge_attempts", 0))
        n_boxes_total += int(trial_part.get("n_boxes", 0))
        profile = trial_part.get("profile", {})
        raw_boxes_total += int(profile.get("raw_boxes", 0))
        final_boxes_total += int(profile.get("final_boxes", 0))
        grow_ms_total += float(profile.get("grow_ms", 0.0))
        total_ms_total += float(profile.get("total_ms", 0.0))
        scene_trials.append(trial_part)
        last_trial = trial_part
    assert last_trial is not None
    merged_diagnostics = merge_trial_diagnostics(scene_trials)
    trial = dict(last_trial)
    trial["protocol"] = "random_obstacle_ffb_prewarm"
    trial["phase"] = "random_obstacle_ffb_prewarm"
    trial["scene"] = "blind_random_obstacle_ffb"
    trial["scene_metadata"] = {
        "scene_kind": "blind_random_obstacle_ffb_prewarm",
        "robot": "iiwa",
        "scenario": difficulty,
        "scene_label": scene_label_for(difficulty),
        "prewarm_starts_empty": True,
        "random_seed": int(args.random_ffb_seed_base),
        "random_seed_count": int(total_seed_count),
        "random_sequence": str(args.random_prewarm_sequence),
        "random_obstacle_count": int(args.random_ffb_obstacles),
        "random_obstacle_difficulty": str(args.random_ffb_difficulty),
        "random_obstacle_profile": str(args.random_ffb_obstacle_profile),
        "random_scene_count": scene_count,
        "random_ms_per_scene": float(per_scene_budget_ms),
        "ffb_only": True,
        "charged_to_reuse_build_time": False,
    }
    trial.update(cache_prebuild_files or {})
    trial["build_s"] = float(build_s_total)
    trial["prebridge_time_s"] = float(prebridge_time_s_total)
    trial["prebridge_added_boxes"] = int(prebridge_added_total)
    trial["prebridge_attempts"] = int(prebridge_attempts_total)
    trial["n_boxes"] = int(n_boxes_total)
    trial["profile"] = dict(last_trial.get("profile", {}))
    trial["profile"]["total_ms"] = float(total_ms_total)
    trial["profile"]["grow_ms"] = float(grow_ms_total)
    trial["profile"]["raw_boxes"] = int(raw_boxes_total)
    trial["profile"]["final_boxes"] = int(final_boxes_total)
    trial["diagnostics"] = merged_diagnostics
    trial["queries"] = []
    trial["prewarm"] = {
        "protocol": "blind_random_obstacle_ffb_prewarm",
        "difficulty": difficulty,
        "budget_ms": float(budget_ms),
        "budget_label": budget_label(budget_ms),
        "random_seed": int(args.random_ffb_seed_base),
        "random_seed_count": int(total_seed_count),
        "random_seeds_per_second": int(args.random_ffb_seeds),
        "random_sequence": str(args.random_prewarm_sequence),
        "random_obstacle_count": int(args.random_ffb_obstacles),
        "random_obstacle_difficulty": str(args.random_ffb_difficulty),
        "random_obstacle_profile": str(args.random_ffb_obstacle_profile),
        "random_scene_count": scene_count,
        "random_ms_per_scene": float(per_scene_budget_ms),
        "random_scene_seed_stride": 1009,
        "ffb_only": True,
        "charged_to_reuse_build_time": False,
    }
    trial["prewarm_parts"] = [
        {
            "scene_index": int(part.get("scene_metadata", {}).get("random_scene_index", index)),
            "build_s": float(part.get("build_s", 0.0)),
            "n_boxes": int(part.get("n_boxes", 0)),
            "raw_boxes": int(part.get("profile", {}).get("raw_boxes", 0)),
            "cache_file_bytes": int(part.get("cache_file_bytes", 0)),
        }
        for index, part in enumerate(scene_trials)
    ]
    trial["difficulty"] = difficulty
    trial["scenario"] = difficulty
    trial["budget_ms"] = float(budget_ms)
    trial["budget_label"] = budget_label(budget_ms)
    trial["prewarm_is_blind"] = True
    return trial


def run_route_empty_prewarm_trial(
    args: argparse.Namespace,
    *,
    robot: Any,
    variant: str,
    difficulty: str,
    seed_index: int,
    budget_ms: float,
    cache_namespace: str,
    seeds: list[list[float]],
    queries: list[Any],
) -> dict[str, Any]:
    prewarm_args = copy.copy(args)
    prewarm_args.timeout_ms = min(float(args.timeout_ms), max(1.0, float(budget_ms)))
    prewarm_args.cross_detach_cache_tree = False
    metadata = {
        "scene_kind": "empty_scene_query_prewarm",
        "robot": "iiwa",
        "difficulty": difficulty,
        "seed_index": int(seed_index),
        "obstacle_count": 0,
        "offline_budget_ms": float(budget_ms),
        "charged_to_reuse_build_time": False,
    }
    trial = run_build_trial(
        prewarm_args,
        robot=robot,
        variant=variant,
        seed=seed_index,
        protocol="route_empty_prewarm",
        scene_name="empty",
        obstacles=[],
        scene_metadata=metadata,
        seeds=seeds,
        queries=queries,
        cache_namespace=cache_namespace,
        prewarm={
            "protocol": "empty_scene_same_query_prewarm",
            "difficulty": difficulty,
            "budget_ms": float(budget_ms),
            "budget_label": budget_label(budget_ms),
            "charged_to_reuse_build_time": False,
        },
    )
    trial["protocol"] = "route_empty_prewarm"
    trial["phase"] = "route_empty_prewarm"
    trial["difficulty"] = difficulty
    trial["budget_ms"] = float(budget_ms)
    trial["budget_label"] = budget_label(budget_ms)
    return trial


def summarize_trials(trials: list[dict[str, Any]]) -> dict[str, Any]:
    summary = summarize_variant(trials)
    summary["lect_materialization_hit_rate_mean"] = mean_present(lect_materialization_hit_rate(trial) for trial in trials)
    summary["lect_prewarm_hit_rate_mean"] = summary["lect_materialization_hit_rate_mean"]
    summary["lect_endpoint_hit_rate_mean"] = mean_present(lect_endpoint_hit_rate(trial) for trial in trials)
    summary["scoring_external_hit_rate_mean"] = mean_present(scoring_external_hit_rate(trial) for trial in trials)
    summary["materializations_mean"] = mean_present(diagnostic_value(trial, "oracle.materializations") for trial in trials)
    summary["materialization_reused_endpoint_cache_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_reused_endpoint_cache") for trial in trials)
    summary["materialization_reused_external_evidence_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_reused_external_evidence") for trial in trials)
    summary["materialization_reused_cached_envelope_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_reused_cached_envelope") for trial in trials)
    summary["scoring_evaluations_mean"] = mean_present(diagnostic_value(trial, "oracle.scoring_evaluations") for trial in trials)
    summary["scoring_reused_external_evidence_mean"] = mean_present(diagnostic_value(trial, "oracle.scoring_reused_external_evidence") for trial in trials)
    summary["scoring_endpoint_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.scoring_endpoint_time_us") for trial in trials)
    summary["scoring_envelope_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.scoring_envelope_time_us") for trial in trials)
    summary["materialization_endpoint_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_endpoint_time_us") for trial in trials)
    summary["materialization_envelope_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_envelope_time_us") for trial in trials)
    summary["materialization_cache_lookup_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_cache_lookup_time_us") for trial in trials)
    summary["materialization_cache_read_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_cache_read_time_us") for trial in trials)
    summary["materialization_envelope_compute_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_envelope_compute_time_us") for trial in trials)
    summary["materialization_envelope_read_time_us_mean"] = mean_present(diagnostic_value(trial, "oracle.materialization_envelope_read_time_us") for trial in trials)
    summary["validation_cache_hit_rate_mean"] = mean_present(validation_hit_rate(trial) for trial in trials)
    summary["validation_cache_hits_mean"] = mean_present(diagnostic_value(trial, "oracle.validation_cache_hits") for trial in trials)
    summary["validation_cache_misses_mean"] = mean_present(diagnostic_value(trial, "oracle.validation_cache_misses") for trial in trials)
    summary["prebuild_cache_file_bytes_mean"] = mean_present(float(trial.get("prebuild_cache_file_bytes", 0.0)) for trial in trials)
    summary["prebuild_cache_file_count_mean"] = mean_present(float(trial.get("prebuild_cache_file_count", 0.0)) for trial in trials)
    return summary


def route_fingerprint(trial: dict[str, Any]) -> dict[str, Any]:
    diagnostics = trial.get("diagnostics") or {}
    profile = trial.get("profile") or {}
    queries = trial.get("queries") or []
    query_route = []
    for query in queries:
        query_route.append({
            "label": str(query.get("label") or query.get("name") or ""),
            "ok": bool(query.get("ok", False)),
            "audit_passed": bool(query.get("audit_passed", False)),
            "bridge_progress": int(query.get("bridge_progress", 0)),
            "repair_count": int(query.get("repair_count", 0)),
            "start_box_id": int(query.get("start_box_id", -1)),
            "goal_box_id": int(query.get("goal_box_id", -1)),
        })
    return {
        "lect_split_events": int(float(diagnostics.get("lect.locked_split_events_consumed", diagnostics.get("lect.split_events", 0.0)))),
        "lect_split_event_hash_hi": int(float(diagnostics.get("lect.locked_split_event_hash_hi", diagnostics.get("lect.split_event_hash_hi", 0.0)))),
        "lect_split_event_hash_lo": int(float(diagnostics.get("lect.locked_split_event_hash_lo", diagnostics.get("lect.split_event_hash_lo", 0.0)))),
        "oracle_materializations": int(float(diagnostics.get("oracle.materializations", 0.0))),
        "profile_raw_boxes": int(profile.get("raw_boxes", 0)),
        "profile_final_boxes": int(profile.get("final_boxes", 0)),
        "profile_segment_edges": int(profile.get("segment_edges", 0)),
        "profile_adjacency_islands": int(profile.get("adjacency_islands", 0)),
        "n_boxes": int(trial.get("n_boxes", 0)),
        "segment_edge_count": int(trial.get("segment_edge_count", 0)),
        "prebridge_added_boxes": int(trial.get("prebridge_added_boxes", 0)),
        "prebridge_attempts": int(trial.get("prebridge_attempts", 0)),
        "query_route": query_route,
    }


def route_differences(lhs: dict[str, Any], rhs: dict[str, Any]) -> list[str]:
    keys = sorted(set(lhs) | set(rhs))
    return [key for key in keys if lhs.get(key) != rhs.get(key)]


def annotate_warm_route_matches(trials: list[dict[str, Any]]) -> dict[str, Any]:
    cold_by_key: dict[tuple[str, str, int], dict[str, Any]] = {}
    mismatches: list[dict[str, Any]] = []
    compared = 0
    skipped = 0
    for trial in trials:
        if trial.get("phase") != "cold":
            continue
        fingerprint = route_fingerprint(trial)
        trial["route_fingerprint"] = fingerprint
        cold_by_key[(str(trial.get("variant")), str(trial.get("difficulty")), int(trial.get("seed", -1)))] = fingerprint
    for trial in trials:
        if trial.get("phase") != "warm_reuse":
            continue
        prewarm = trial.get("prewarm") or {}
        route_required = bool(prewarm.get("target_route_locked_to_cold", False))
        key = (str(trial.get("variant")), str(trial.get("difficulty")), int(trial.get("seed", -1)))
        fingerprint = route_fingerprint(trial)
        trial["route_fingerprint"] = fingerprint
        trial["route_match_required"] = route_required
        if not route_required:
            skipped += 1
            continue
        cold_fingerprint = cold_by_key.get(key)
        diffs = route_differences(cold_fingerprint or {}, fingerprint)
        matches = cold_fingerprint is not None and not diffs
        trial["cold_route_fingerprint"] = cold_fingerprint
        trial["route_matches_cold"] = bool(matches)
        trial["route_mismatch_fields"] = diffs
        compared += 1
        if not matches:
            mismatches.append({
                "variant": trial.get("variant"),
                "difficulty": trial.get("difficulty"),
                "seed": trial.get("seed"),
                "budget_label": trial.get("budget_label"),
                "fields": diffs,
            })
    return {
        "route_mismatches_count": len(mismatches),
        "route_mismatches": mismatches,
        "route_compared_count": compared,
        "route_skipped_count": skipped,
    }


def main() -> int:
    args = parse_args()
    if args.clear_cache and args.cache_root.exists():
        shutil.rmtree(args.cache_root)
    if args.cache_run_id is None:
        args.cache_run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    robot = load_iiwa14_robot()
    variants = parse_csv(args.variants)
    legacy_difficulties = parse_csv(args.difficulties)
    difficulties = [str(args.scenario)]
    random_ffb_scene_counts = [count for count in parse_int_csv(args.random_ffb_scene_counts) if count > 0]
    random_ffb_ms_per_scene = max(1.0, float(args.random_ffb_ms_per_scene))
    if str(args.warm_prewarm_mode) == "random_obstacle_ffb" and random_ffb_scene_counts:
        budgets_ms = [float(count) * random_ffb_ms_per_scene for count in random_ffb_scene_counts]
        random_ffb_scene_count_by_label = {
            budget_label(float(count) * random_ffb_ms_per_scene): int(count)
            for count in random_ffb_scene_counts
        }
    else:
        budgets_ms = parse_float_csv(args.prewarm_budgets_ms)
        random_ffb_scene_count_by_label: dict[str, int] = {}
    warm_target_mode = str(args.warm_target_mode)
    strict_route_warm = warm_target_mode == STRICT_WARM_MODE
    scene_seed_indices = [int(item) for item in parse_csv(args.scene_seed_indices)] if str(args.scene_seed_indices).strip() else list(range(max(1, int(args.scene_seeds))))
    eval_seed_base = int(args.seed_base)

    all_trials: list[dict[str, Any]] = []
    summaries: dict[str, dict[str, dict[str, dict[str, Any]]]] = {}

    def experiment_scene_for(difficulty: str, seed: int, seed_base: int) -> tuple[str, str, list[Any], dict[str, Any], list[list[float]], list[Any]]:
        if difficulty not in {SCENARIO_KEY, RANDOM_SCENARIO_KEY}:
            raise ValueError(f"unsupported Table II scenario {difficulty!r}; expected one of {[SCENARIO_KEY, RANDOM_SCENARIO_KEY]}")
        scene_seed = int(seed_base) + 1009 * int(seed)
        scene_seeds = [list(item) for item in make_coverage_seeds(include_extra_anchors=False)]
        scene_queries = list(make_combined_queries())
        target_scene_name = scene_name_for(difficulty)
        target_scene_label = scene_label_for(difficulty)
        if difficulty == RANDOM_SCENARIO_KEY:
            obstacles, random_metadata = make_random_target_obstacles(args, robot, scene_queries, seed)
        else:
            obstacles = make_combined_obstacles()
            random_metadata = {}
        metadata = {
            "scene_kind": target_scene_name,
            "scene": target_scene_name,
            "scene_label": target_scene_label,
            "robot": "iiwa",
            "difficulty": difficulty,
            "scenario": difficulty,
            "seed_index": int(seed),
            "seed": int(scene_seed),
            "scene_profile_deprecated": str(args.scene_profile),
            "obstacle_count": len(obstacles),
            "query_count": len(scene_queries),
            "query_labels": [query.label for query in scene_queries],
            "seed_protocol": "sbf.marcucci.make_coverage_seeds",
            "coverage_seed_count": len(scene_seeds),
            "coverage_seed_names": ["AS", "TS", "CS", "LB", "RB"],
            **random_metadata,
        }
        return target_scene_name, target_scene_label, obstacles, metadata, scene_seeds, scene_queries

    for variant in variants:
        summaries[variant] = {}
        for difficulty in difficulties:
            scenes = [
                (seed_index, *experiment_scene_for(difficulty, seed_index, eval_seed_base))
                for seed_index in scene_seed_indices
            ]
            cold_split_events_by_seed: dict[int, list[dict[str, Any]]] = {}
            cold_validation_events_by_seed: dict[int, list[dict[str, Any]]] = {}
            for seed_index, target_scene_name, target_scene_label, obstacles, metadata, scene_seeds, scene_queries in scenes:
                cold_locked_split_events = None
                cold_locked_validation_events = None
                cold_prewarm: dict[str, Any] | None = None
                if strict_route_warm:
                    route_trace_trial = run_build_trial(
                        args,
                        robot=robot,
                        variant=variant,
                        seed=seed_index,
                        protocol="route_trace",
                        scene_name=target_scene_name,
                        obstacles=obstacles,
                        scene_metadata=metadata,
                        seeds=scene_seeds,
                        queries=scene_queries,
                        cache_namespace=None,
                        prewarm=None,
                        return_split_events=True,
                        return_validation_events=True,
                    )
                    cold_split_events_by_seed[seed_index] = list(route_trace_trial.pop("split_events", []))
                    cold_validation_events_by_seed[seed_index] = list(route_trace_trial.pop("validation_events", []))
                    route_trace_trial["phase"] = "route_trace"
                    route_trace_trial["difficulty"] = difficulty
                    route_trace_trial["scenario"] = difficulty
                    route_trace_trial["used_for_route_lock"] = True
                    route_trace_trial["route_lock_split_event_count"] = len(cold_split_events_by_seed[seed_index])
                    route_trace_trial["route_lock_validation_event_count"] = len(cold_validation_events_by_seed[seed_index])
                    all_trials.append(route_trace_trial)
                    cold_locked_split_events = cold_split_events_by_seed[seed_index]
                    cold_locked_validation_events = cold_validation_events_by_seed[seed_index]
                    cold_prewarm = {
                        "protocol": "route_locked_cold_replay",
                        "route_trace_build_s": float(route_trace_trial.get("build_s", 0.0)),
                        "charged_to_cold_build_time": False,
                        "locked_split_event_count": len(cold_split_events_by_seed[seed_index]),
                        "locked_validation_event_count": len(cold_validation_events_by_seed[seed_index]),
                    }

                cold_trial = run_build_trial(
                    args,
                    robot=robot,
                    variant=variant,
                    seed=seed_index,
                    protocol="cold",
                    scene_name=target_scene_name,
                    obstacles=obstacles,
                    scene_metadata=metadata,
                    seeds=scene_seeds,
                    queries=scene_queries,
                    cache_namespace=None,
                    locked_split_events=cold_locked_split_events,
                    locked_validation_events=cold_locked_validation_events,
                    prewarm=cold_prewarm,
                )
                cold_trial["phase"] = "cold"
                cold_trial["difficulty"] = difficulty
                cold_trial["scenario"] = difficulty
                cold_trial["route_locked_to_trace"] = strict_route_warm
                all_trials.append(cold_trial)

            if args.include_matched_cross:
                for budget_ms in budgets_ms:
                    label = budget_label(budget_ms)
                    prewarm_namespace = safe_namespace("paper2", args.cache_run_id, variant, difficulty, label, f"blind_{args.warm_prewarm_mode}_prewarm")
                    if args.warm_prewarm_mode == "random_empty":
                        prewarm_trial = run_random_empty_prewarm_trial(
                            args,
                            robot=robot,
                            variant=variant,
                            difficulty=difficulty,
                            budget_ms=float(budget_ms),
                            cache_namespace=prewarm_namespace,
                        )
                        prewarm_protocol = "robot_space_random_empty_prewarm"
                    elif args.warm_prewarm_mode == "random_obstacle_ffb":
                        prewarm_trial = run_random_obstacle_ffb_prewarm_trial(
                            args,
                            robot=robot,
                            variant=variant,
                            difficulty=difficulty,
                            budget_ms=float(budget_ms),
                            cache_namespace=prewarm_namespace,
                            scene_count=random_ffb_scene_count_by_label.get(label),
                        )
                        prewarm_protocol = "blind_random_obstacle_ffb_prewarm"
                    else:
                        raise ValueError(f"unsupported warm prewarm mode {args.warm_prewarm_mode!r}")
                    prewarm_trial["phase"] = "warm_prewarm"
                    prewarm_trial["budget_ms"] = float(budget_ms)
                    prewarm_trial["budget_label"] = label
                    all_trials.append(prewarm_trial)
                    prewarm_cache_metrics = cache_metrics(args.cache_root, prewarm_namespace)

                    for seed_index, target_scene_name, target_scene_label, obstacles, metadata, scene_seeds, scene_queries in scenes:
                        warm_namespace = safe_namespace("paper2", args.cache_run_id, variant, difficulty, label, f"eval_seed{seed_index}", f"blind_cache_{warm_target_mode}")
                        copy_cache_namespace(args.cache_root, prewarm_namespace, warm_namespace)
                        if warm_target_mode == STRICT_WARM_MODE:
                            warm_protocol = "warm_budget"
                            locked_split_events = cold_split_events_by_seed[seed_index]
                            locked_validation_events = cold_validation_events_by_seed[seed_index]
                            active_lect_cache_tree = False
                            fresh_target_lect_tree = bool(args.matched_trajectory_reuse)
                            target_route_locked_to_cold = True
                            target_route_matches_cold = "route_audited_after_run"
                        elif warm_target_mode == GUIDED_EXTERNAL_WARM_MODE:
                            warm_protocol = "warm_guided_external"
                            locked_split_events = None
                            locked_validation_events = None
                            active_lect_cache_tree = False
                            fresh_target_lect_tree = True
                            target_route_locked_to_cold = False
                            target_route_matches_cold = "not_required_cache_guided"
                        elif warm_target_mode == GUIDED_ACTIVE_WARM_MODE:
                            warm_protocol = "warm_guided_active"
                            locked_split_events = None
                            locked_validation_events = None
                            active_lect_cache_tree = True
                            fresh_target_lect_tree = False
                            target_route_locked_to_cold = False
                            target_route_matches_cold = "not_required_cache_guided"
                        else:
                            raise ValueError(f"unsupported warm target mode {warm_target_mode!r}")
                        warm_trial = run_build_trial(
                            args,
                            robot=robot,
                            variant=variant,
                            seed=seed_index,
                            protocol=warm_protocol,
                            scene_name=target_scene_name,
                            obstacles=obstacles,
                            scene_metadata=metadata,
                            seeds=scene_seeds,
                            queries=scene_queries,
                            cache_namespace=warm_namespace,
                            locked_split_events=locked_split_events,
                            locked_validation_events=locked_validation_events,
                            prewarm={
                                "protocol": f"{args.warm_prewarm_mode}_{warm_target_mode}",
                                "prewarm_protocol": prewarm_protocol,
                                "warm_prewarm_mode": str(args.warm_prewarm_mode),
                                "warm_target_mode": warm_target_mode,
                                "difficulty": difficulty,
                                "scenario": difficulty,
                                "scene": target_scene_name,
                                "scene_label": target_scene_label,
                                "budget_ms": float(budget_ms),
                                "budget_label": label,
                                "random_scene_count": prewarm_trial.get("prewarm", {}).get("random_scene_count"),
                                "random_ms_per_scene": prewarm_trial.get("prewarm", {}).get("random_ms_per_scene"),
                                "eval_seed_base": int(eval_seed_base),
                                "eval_seed_index": int(seed_index),
                                "build_s": float(prewarm_trial.get("build_s", 0.0)),
                                "charged_to_warm_build_time": False,
                                "scene_independent": True,
                                "target_scene": target_scene_name,
                                "active_lect_cache_tree": active_lect_cache_tree,
                                "fresh_target_lect_tree": fresh_target_lect_tree,
                                "target_cache_enabled": True,
                                "target_route_matches_cold": target_route_matches_cold,
                                "target_route_locked_to_cold": target_route_locked_to_cold,
                                "locked_split_event_count": len(cold_split_events_by_seed.get(seed_index, [])),
                                "locked_validation_event_count": len(cold_validation_events_by_seed.get(seed_index, [])),
                                "prewarm_consumed_by_target": True,
                                "prewarm_starts_empty": True,
                                "prewarm_is_blind": True,
                                "source_cache_namespace": prewarm_namespace,
                                "target_cache_namespace": warm_namespace,
                                **prewarm_cache_metrics,
                            },
                        )
                        warm_trial["phase"] = "warm_reuse"
                        warm_trial["difficulty"] = difficulty
                        warm_trial["scenario"] = difficulty
                        warm_trial["budget_ms"] = float(budget_ms)
                        warm_trial["budget_label"] = label
                        all_trials.append(warm_trial)

            if args.include_cross_diagnostic:
                for budget_ms in budgets_ms:
                    label = budget_label(budget_ms)
                    for seed_index, target_scene_name, target_scene_label, obstacles, metadata, scene_seeds, scene_queries in scenes:
                        reuse_namespace = safe_namespace("paper2", args.cache_run_id, variant, difficulty, label, f"eval_seed{seed_index}", "cross_scene_reuse")
                        prewarm_trial_for_seed = run_route_empty_prewarm_trial(
                            args,
                            robot=robot,
                            variant=variant,
                            difficulty=difficulty,
                            seed_index=seed_index,
                            budget_ms=float(budget_ms),
                            cache_namespace=reuse_namespace,
                            seeds=scene_seeds,
                            queries=scene_queries,
                        )
                        all_trials.append(prewarm_trial_for_seed)
                        prewarm_cache_metrics = cache_metrics(args.cache_root, reuse_namespace)
                        prewarm_protocol = "empty_scene_same_query_prewarm"
                        scene_independent = False
                        source_cache_namespace = reuse_namespace
                        prewarm_info = {
                            "protocol": prewarm_protocol,
                            "difficulty": difficulty,
                            "scenario": difficulty,
                            "scene": target_scene_name,
                            "scene_label": target_scene_label,
                            "budget_ms": float(budget_ms),
                            "budget_label": label,
                            "eval_seed_base": int(eval_seed_base),
                            "eval_seed_index": int(seed_index),
                            "build_s": float(prewarm_trial_for_seed.get("build_s", 0.0)),
                            "charged_to_reuse_build_time": False,
                            "scene_independent": scene_independent,
                            "source_cache_namespace": source_cache_namespace,
                            **prewarm_cache_metrics,
                        }

                        trial = run_build_trial(
                            args,
                            robot=robot,
                            variant=variant,
                            seed=seed_index,
                            protocol="cross_scene",
                                scene_name=target_scene_name,
                            obstacles=obstacles,
                            scene_metadata=metadata,
                            seeds=scene_seeds,
                            queries=scene_queries,
                            cache_namespace=reuse_namespace,
                            prewarm=prewarm_info,
                        )
                        trial["phase"] = "cross_diagnostic_reuse"
                        trial["difficulty"] = difficulty
                        trial["scenario"] = difficulty
                        trial["budget_ms"] = float(budget_ms)
                        trial["budget_label"] = label
                        all_trials.append(trial)

            for seed_index, target_scene_name, target_scene_label, obstacles, metadata, scene_seeds, scene_queries in scenes:
                replay_namespace = safe_namespace("paper2", args.cache_run_id, variant, difficulty, f"eval_seed{seed_index}", "same_scene_replay")
                populate_trial = run_build_trial(
                    args,
                    robot=robot,
                    variant=variant,
                    seed=seed_index,
                    protocol="same_scene_populate",
                    scene_name=target_scene_name,
                    obstacles=obstacles,
                    scene_metadata=metadata,
                    seeds=scene_seeds,
                    queries=scene_queries,
                    cache_namespace=replay_namespace,
                    prewarm={
                        "protocol": "same_scene_first_build_populates_lect_cache",
                        "difficulty": difficulty,
                        "scenario": difficulty,
                        "scene": target_scene_name,
                        "scene_label": target_scene_label,
                        "eval_seed_base": int(eval_seed_base),
                        "eval_seed_index": int(seed_index),
                        "charged_to_replay_build_time": False,
                    },
                )
                populate_trial["phase"] = "replay_populate"
                populate_trial["difficulty"] = difficulty
                populate_trial["scenario"] = difficulty
                all_trials.append(populate_trial)

                replay_trial = run_build_trial(
                    args,
                    robot=robot,
                    variant=variant,
                    seed=seed_index,
                    protocol="warm",
                    scene_name=target_scene_name,
                    obstacles=obstacles,
                    scene_metadata=metadata,
                    seeds=scene_seeds,
                    queries=scene_queries,
                    cache_namespace=replay_namespace,
                    prewarm={
                        "protocol": "same_scene_second_build_lect_replay",
                        "difficulty": difficulty,
                        "scenario": difficulty,
                        "scene": target_scene_name,
                        "scene_label": target_scene_label,
                        "eval_seed_base": int(eval_seed_base),
                        "eval_seed_index": int(seed_index),
                        "populate_build_s": float(populate_trial.get("build_s", 0.0)),
                        "charged_to_replay_build_time": False,
                        **cache_metrics(args.cache_root, replay_namespace),
                    },
                )
                replay_trial["phase"] = "replay"
                replay_trial["difficulty"] = difficulty
                replay_trial["scenario"] = difficulty
                all_trials.append(replay_trial)

        for difficulty in difficulties:
            summaries[variant][difficulty] = {}
            cold_rows = [
                trial for trial in all_trials
                if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "cold"
            ]
            cold_summary = summarize_trials(cold_rows)
            summaries[variant][difficulty]["cold"] = cold_summary
            warm_budget_summaries: dict[str, Any] = {}
            if args.include_matched_cross:
                for budget_ms in budgets_ms:
                    label = budget_label(budget_ms)
                    prewarm_rows = [
                        trial for trial in all_trials
                        if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "warm_prewarm" and trial.get("budget_label") == label
                    ]
                    reuse_rows = [
                        trial for trial in all_trials
                        if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "warm_reuse" and trial.get("budget_label") == label
                    ]
                    prewarm_summary = summarize_trials(prewarm_rows)
                    reuse_summary = summarize_trials(reuse_rows)
                    cold_median = cold_summary.get("build_median_s")
                    reuse_median = reuse_summary.get("build_median_s")
                    speedup = (float(cold_median) / float(reuse_median)) if cold_median and reuse_median else None
                    warm_budget_summaries[label] = {
                        "budget_ms": float(budget_ms),
                        "random_scene_count": random_ffb_scene_count_by_label.get(label),
                        "random_ms_per_scene": random_ffb_ms_per_scene if label in random_ffb_scene_count_by_label else None,
                        "prewarm": prewarm_summary,
                        "reuse": reuse_summary,
                        "warm_speedup_vs_cold": speedup,
                    }
            summaries[variant][difficulty]["warm_budgets"] = warm_budget_summaries
            best_warm_label = max(
                warm_budget_summaries,
                key=lambda label: warm_budget_summaries[label]["warm_speedup_vs_cold"] or -1.0,
            ) if warm_budget_summaries else None
            summaries[variant][difficulty]["best_warm_budget_label"] = best_warm_label
            if best_warm_label is not None:
                summaries[variant][difficulty]["warm_populate"] = warm_budget_summaries[best_warm_label]["prewarm"]
                summaries[variant][difficulty]["warm_budget"] = warm_budget_summaries[best_warm_label]["reuse"]
                summaries[variant][difficulty]["warm_speedup_vs_cold"] = warm_budget_summaries[best_warm_label]["warm_speedup_vs_cold"]
            else:
                summaries[variant][difficulty]["warm_populate"] = summarize_trials([])
                summaries[variant][difficulty]["warm_budget"] = summarize_trials([])
            replay_populate_rows = [
                trial for trial in all_trials
                if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "replay_populate"
            ]
            replay_rows = [
                trial for trial in all_trials
                if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "replay"
            ]
            replay_populate_summary = summarize_trials(replay_populate_rows)
            replay_summary = summarize_trials(replay_rows)
            summaries[variant][difficulty]["replay_populate"] = replay_populate_summary
            summaries[variant][difficulty]["replay"] = replay_summary
            if cold_summary.get("build_median_s") and replay_summary.get("build_median_s"):
                summaries[variant][difficulty]["replay_speedup_vs_cold"] = (
                    float(cold_summary["build_median_s"]) / float(replay_summary["build_median_s"])
                )
            budget_summaries: dict[str, Any] = {}
            if args.include_cross_diagnostic:
                for budget_ms in budgets_ms:
                    label = budget_label(budget_ms)
                    prewarm_rows = [
                        trial for trial in all_trials
                        if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "route_empty_prewarm" and trial.get("budget_label") == label
                    ]
                    reuse_rows = [
                        trial for trial in all_trials
                        if trial.get("variant") == variant and trial.get("difficulty") == difficulty and trial.get("phase") == "cross_diagnostic_reuse" and trial.get("budget_label") == label
                    ]
                    prewarm_summary = summarize_trials(prewarm_rows)
                    reuse_summary = summarize_trials(reuse_rows)
                    cold_median = cold_summary.get("build_median_s")
                    reuse_median = reuse_summary.get("build_median_s")
                    budget_summaries[label] = {
                        "budget_ms": float(budget_ms),
                        "prewarm": prewarm_summary,
                        "reuse": reuse_summary,
                        "cross": reuse_summary,
                        "cross_speedup_vs_cold": (float(cold_median) / float(reuse_median)) if cold_median and reuse_median else None,
                    }
            summaries[variant][difficulty]["budgets"] = budget_summaries
            best_label = max(
                budget_summaries,
                key=lambda label: budget_summaries[label]["cross_speedup_vs_cold"] or -1.0,
            ) if budget_summaries else None
            summaries[variant][difficulty]["best_budget_label"] = best_label
            if best_label is not None:
                summaries[variant][difficulty]["prewarm"] = budget_summaries[best_label]["prewarm"]
                summaries[variant][difficulty]["reuse"] = budget_summaries[best_label]["reuse"]
                summaries[variant][difficulty]["cross"] = budget_summaries[best_label]["cross"]
                if replay_summary.get("build_median_s") and budget_summaries[best_label]["reuse"].get("build_median_s"):
                    summaries[variant][difficulty]["replay_speedup_vs_cross"] = (
                        float(budget_summaries[best_label]["reuse"]["build_median_s"]) /
                        float(replay_summary["build_median_s"])
                    )

    route_audit = annotate_warm_route_matches(all_trials)
    if warm_target_mode == GUIDED_ACTIVE_WARM_MODE:
        main_cache_path = "active_blind_prewarm_tree"
    elif warm_target_mode == GUIDED_EXTERNAL_WARM_MODE:
        main_cache_path = "fresh_target_tree_external_blind_prewarm_guided"
    else:
        main_cache_path = "fresh_target_tree_external_blind_prewarm_route_locked" if bool(args.matched_trajectory_reuse) else "active_lect_tree"

    payload = {
        "experiment": "tro2026_iiwa_lect_incremental_reuse",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": f"{args.scenario}_blind_{args.warm_prewarm_mode}_{warm_target_mode}_warm_and_same_scene_replay",
        "params": {
            "variants": variants,
            "scenario": str(args.scenario),
            "scenarios": difficulties,
            "scene_name": scene_name_for(str(args.scenario)),
            "scene_label": scene_label_for(str(args.scenario)),
            "difficulties": difficulties,
            "legacy_difficulties_arg": legacy_difficulties,
            "scene_seeds": len(scene_seed_indices),
            "scene_seed_indices": scene_seed_indices,
            "prewarm_budgets_ms": budgets_ms,
            "random_ffb_scene_count_budgets": random_ffb_scene_counts if str(args.warm_prewarm_mode) == "random_obstacle_ffb" else [],
            "random_ffb_effective_scene_count_by_budget_label": random_ffb_scene_count_by_label,
            "include_warm_budget_rows": bool(args.include_matched_cross),
            "warm_target_mode": warm_target_mode,
            "warm_prewarm_mode": str(args.warm_prewarm_mode),
            "matched_trajectory_reuse": bool(args.matched_trajectory_reuse) if warm_target_mode == STRICT_WARM_MODE else False,
            "warm_external_evidence_materialization": True if warm_target_mode in {GUIDED_EXTERNAL_WARM_MODE, GUIDED_ACTIVE_WARM_MODE} else bool(args.cross_external_evidence_materialization),
            "warm_external_evidence_backfill_active": True if warm_target_mode in {GUIDED_EXTERNAL_WARM_MODE, GUIDED_ACTIVE_WARM_MODE} else bool(args.cross_external_evidence_backfill_active),
            "replay_external_evidence_materialization": bool(args.replay_external_evidence_materialization),
            "warm_external_evidence_scoring": True if warm_target_mode in {GUIDED_EXTERNAL_WARM_MODE, GUIDED_ACTIVE_WARM_MODE} else bool(args.cross_external_evidence_scoring),
            "replay_external_evidence_scoring": bool(args.replay_external_evidence_scoring),
            "stateless_materialization_context": bool(args.stateless_materialization_context),
            "include_cross_diagnostic": bool(args.include_cross_diagnostic),
            "cross_prewarm_mode": str(args.cross_prewarm_mode),
            "random_prewarm_seeds": int(args.random_prewarm_seeds),
            "random_prewarm_scale_with_budget": bool(args.random_prewarm_scale_with_budget),
            "random_prewarm_sequence": str(args.random_prewarm_sequence),
            "random_prewarm_query_pairs": int(args.random_prewarm_query_pairs),
            "random_prewarm_max_boxes": int(args.random_prewarm_max_boxes),
            "random_prewarm_seed_base": int(args.random_prewarm_seed_base),
            "random_ffb_seeds": int(args.random_ffb_seeds),
            "random_ffb_obstacles": int(args.random_ffb_obstacles),
            "random_ffb_difficulty": str(args.random_ffb_difficulty),
            "random_ffb_obstacle_profile": str(args.random_ffb_obstacle_profile),
            "random_ffb_seed_base": int(args.random_ffb_seed_base),
            "target_random_obstacles": int(args.target_random_obstacles),
            "target_random_seed_base": int(args.target_random_seed_base),
            "target_random_max_tries": int(args.target_random_max_tries),
            "random_ffb_scene_counts": str(args.random_ffb_scene_counts),
            "random_ffb_ms_per_scene": float(args.random_ffb_ms_per_scene),
            "random_ffb_scenes_per_10s": float(args.random_ffb_scenes_per_10s),
            "external_scoring_cover_ratio": float(args.external_scoring_cover_ratio),
            "external_scoring_min_iou": float(args.external_scoring_min_iou),
            "external_scoring_overlap_max_nodes": int(args.external_scoring_overlap_max_nodes),
            "external_materialization_cover_ratio": float(args.external_materialization_cover_ratio),
            "cross_detach_cache_tree": bool(args.cross_detach_cache_tree),
            "disjoint_prewarm": True,
            "prewarm_starts_empty_per_budget": True,
            "prewarm_is_blind_to_target_route": True,
            "main_cache_path": main_cache_path,
            "warm_target_route_audited_against_cold": warm_target_mode == STRICT_WARM_MODE,
            "warm_target_route_free_cache_guided": warm_target_mode != STRICT_WARM_MODE,
            "warm_prewarm_consumed_by_target": True,
            "route_mismatches_count": int(route_audit["route_mismatches_count"]),
            "route_compared_count": int(route_audit.get("route_compared_count", 0)),
            "route_skipped_count": int(route_audit.get("route_skipped_count", 0)),
            "seed_base": int(args.seed_base),
            "eval_seed_base": int(eval_seed_base),
            "scene_profile": str(args.scene_profile),
            "cache_root": str(args.cache_root),
            "cache_run_id": args.cache_run_id,
            "cross_random_obstacles": int(args.cross_random_obstacles),
            "cross_random_seed_base": args.cross_random_seed_base,
            "cross_clearance": float(args.cross_clearance),
            "storage_profile": args.storage_profile,
            "split_policy": args.split_policy,
            "threads": int(args.threads),
            "task_batch_size": int(args.task_batch_size),
            "max_boxes": int(args.max_boxes),
            "timeout_ms": float(args.timeout_ms),
            "ffb_depth": int(args.ffb_depth),
            "component_connect_ffb_max_depth": int(args.component_connect_ffb_max_depth),
            "quality_min_connected_boxes": int(args.quality_min_connected_boxes),
            "post_connect_extra_boxes": int(args.post_connect_extra_boxes),
            "post_connect_time_budget_ms": float(args.post_connect_time_budget_ms),
            "repair_timeout_ms": float(args.repair_timeout_ms),
            "validation_cache": bool(args.validation_cache),
            "bridge_failed_queries": bool(args.bridge_failed_queries),
            "bridge_repaired_queries": bool(args.bridge_repaired_queries),
            "corridor_refine": bool(args.corridor_refine),
            "corridor_refine_deterministic": bool(args.corridor_refine_deterministic),
            "corridor_refine_budget_ms": float(args.corridor_refine_budget_ms),
            "corridor_refine_max_boxes": int(args.corridor_refine_max_boxes),
            "corridor_refine_boxes_per_query": int(args.corridor_refine_boxes_per_query),
            "corridor_refine_passes": int(args.corridor_refine_passes),
            "corridor_refine_start_margin_ms": float(args.corridor_refine_start_margin_ms),
            "corridor_refine_defer_labels": str(args.corridor_refine_defer_labels),
        },
        "route_audit": route_audit,
        "summaries": summaries,
        "trials": all_trials,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "trials": len(all_trials), "variants": variants, "scenarios": difficulties}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())