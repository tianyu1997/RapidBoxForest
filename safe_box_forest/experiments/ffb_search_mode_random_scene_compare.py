#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import random
import shutil
import statistics
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SBF_OLD_DIR = Path(__file__).resolve().parent / "sbf_old"
if str(SBF_OLD_DIR) not in sys.path:
    sys.path.insert(0, str(SBF_OLD_DIR))

from common_scene_sampling import make_random_scene, make_robot  # noqa: E402

import sbf  # noqa: E402


OUTPUT_ROOT = ROOT / "safe_box_forest" / "outputs" / "logs"
CACHE_ROOT = OUTPUT_ROOT / "ffb_search_mode_compare_cache"


def parse_csv_ints(text: str) -> list[int]:
    values: list[int] = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(int(item))
    return values


def mode_name(mode: Any) -> str:
    if mode == sbf.FindFreeBoxSearchMode.BinaryDepth:
        return "BinaryDepth"
    return "Linear"


def diagnostic_value(diagnostics: dict[str, float], key: str) -> float:
    value = diagnostics.get(key)
    return float(value) if value is not None else 0.0


def make_config(cache_dir: Path,
                search_mode: Any,
                max_depth: int,
                start_depth: int,
                max_boxes: int,
                max_consecutive_miss: int,
                timeout_ms: float) -> Any:
    config = sbf.SBFConfig()
    config.enable_merger = False
    config.enable_connector = False
    config.runtime.mode = sbf.ExecutionMode.Inline
    config.grower.mode = sbf.GrowerMode.Frontwave
    config.grower.max_boxes = int(max_boxes)
    config.grower.max_consecutive_miss = int(max_consecutive_miss)
    config.grower.timeout_ms = float(timeout_ms)
    config.grower.find_free_box.max_depth = int(max_depth)
    config.grower.find_free_box.start_depth = int(start_depth)
    config.grower.find_free_box.search_mode = search_mode
    config.grower.find_free_box.split.use_best_tighten = False
    config.endpoint_source.source = sbf.EndpointSource.IFK
    config.envelope_type.type = sbf.EnvelopeType.LinkIAABB
    config.database.path = str(cache_dir)
    config.database.checkpoint_after_build = False
    return config


def sample_collision_free_seeds(robot: Any,
                                obstacles: list[Any],
                                sample_seed: int,
                                count: int,
                                fixed_seeds: list[list[float]]) -> list[list[float]]:
    seeds: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()

    def try_append(q: list[float]) -> None:
        key = tuple(round(float(value), 12) for value in q)
        if key in seen:
            return
        if sbf.check_config_collision(robot, obstacles, q):
            return
        seen.add(key)
        seeds.append([float(value) for value in q])

    for q in fixed_seeds:
        try_append(q)
        if len(seeds) >= count:
            return seeds

    rng = random.Random(int(sample_seed))
    limits = list(robot.joint_limits().limits)
    max_attempts = max(256, count * 300)
    attempts = 0
    while len(seeds) < count and attempts < max_attempts:
        attempts += 1
        q = [rng.uniform(interval.lo, interval.hi) for interval in limits]
        try_append(q)

    if len(seeds) < count:
        raise RuntimeError(
            f"only sampled {len(seeds)} collision-free seeds out of requested {count}"
        )
    return seeds


def run_mode(robot: Any,
             obstacles: list[Any],
             seeds: list[list[float]],
             mode: Any,
             args: argparse.Namespace) -> dict[str, Any]:
    cache_dir = Path(tempfile.mkdtemp(prefix=f"ffb-{mode_name(mode).lower()}-", dir=str(CACHE_ROOT)))
    try:
        config = make_config(cache_dir,
                             mode,
                             args.max_depth,
                             args.start_depth,
                             args.max_boxes,
                             args.max_consecutive_miss,
                             args.timeout_ms)
        forest = sbf.SafeBoxForest(robot, config)
        t0 = time.perf_counter()
        profile = forest.build_coverage(obstacles, seeds)
        wall_ms = (time.perf_counter() - t0) * 1000.0
        diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
        return {
            "mode": mode_name(mode),
            "wall_ms": float(wall_ms),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "adjacency_islands": int(profile.adjacency_islands),
            "oracle_node_validations": diagnostic_value(diagnostics, "oracle.node_validations"),
            "oracle_materializations": diagnostic_value(diagnostics, "oracle.materializations"),
            "oracle_collision_queries": diagnostic_value(diagnostics, "oracle.envelope_collision_queries"),
            "diagnostics": diagnostics,
        }
    finally:
        if not args.keep_cache_dirs:
            shutil.rmtree(cache_dir, ignore_errors=True)


def summarize_mode(rows: list[dict[str, Any]], key: str) -> dict[str, float]:
    values = [float(row[key]) for row in rows]
    return {
        "mean": float(statistics.fmean(values)),
        "median": float(statistics.median(values)),
        "min": float(min(values)),
        "max": float(max(values)),
    }


def build_summary(cases: list[dict[str, Any]]) -> dict[str, Any]:
    linear_rows = [case["modes"]["Linear"] for case in cases]
    binary_rows = [case["modes"]["BinaryDepth"] for case in cases]

    pairwise_ratios: dict[str, list[float]] = {
        "wall_ms": [],
        "grow_ms": [],
        "oracle_node_validations": [],
        "oracle_materializations": [],
    }
    for case in cases:
        linear = case["modes"]["Linear"]
        binary = case["modes"]["BinaryDepth"]
        for key in pairwise_ratios:
            linear_value = float(linear[key])
            binary_value = float(binary[key])
            if linear_value > 0.0:
                pairwise_ratios[key].append(binary_value / linear_value)

    return {
        "cases": len(cases),
        "Linear": {
            "wall_ms": summarize_mode(linear_rows, "wall_ms"),
            "grow_ms": summarize_mode(linear_rows, "grow_ms"),
            "oracle_node_validations": summarize_mode(linear_rows, "oracle_node_validations"),
            "oracle_materializations": summarize_mode(linear_rows, "oracle_materializations"),
        },
        "BinaryDepth": {
            "wall_ms": summarize_mode(binary_rows, "wall_ms"),
            "grow_ms": summarize_mode(binary_rows, "grow_ms"),
            "oracle_node_validations": summarize_mode(binary_rows, "oracle_node_validations"),
            "oracle_materializations": summarize_mode(binary_rows, "oracle_materializations"),
        },
        "binary_over_linear": {
            key: {
                "mean": float(statistics.fmean(values)),
                "median": float(statistics.median(values)),
                "min": float(min(values)),
                "max": float(max(values)),
            }
            for key, values in pairwise_ratios.items() if values
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare linear vs binary-depth FFB on deterministic random scenes."
    )
    parser.add_argument("--robot", default="iiwa")
    parser.add_argument("--difficulty", default="medium")
    parser.add_argument("--scene-seeds", default="0,1,2")
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--scene-seed-base", type=int, default=20260527)
    parser.add_argument("--ffb-seed-base", type=int, default=314159)
    parser.add_argument("--ffb-seed-count", type=int, default=24)
    parser.add_argument("--max-depth", type=int, default=24)
    parser.add_argument("--start-depth", type=int, default=8)
    parser.add_argument("--max-boxes", type=int, default=128)
    parser.add_argument("--max-consecutive-miss", type=int, default=512)
    parser.add_argument("--timeout-ms", type=float, default=20000.0)
    parser.add_argument(
        "--out-json",
        type=Path,
        default=OUTPUT_ROOT / "ffb_search_mode_random_scene_compare.json",
    )
    parser.add_argument("--keep-cache-dirs", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scene_seeds = parse_csv_ints(args.scene_seeds)
    if not scene_seeds:
        raise SystemExit("--scene-seeds must contain at least one integer")

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)

    cases: list[dict[str, Any]] = []
    for scene_seed in scene_seeds:
        scene_generation_seed = int(args.scene_seed_base) + 1009 * int(scene_seed)
        sample_seed = int(args.ffb_seed_base) + 65537 * int(scene_seed)
        scene = make_random_scene(args.robot,
                                  args.difficulty,
                                  scene_generation_seed,
                                  scene_profile=args.scene_profile)
        robot = make_robot(args.robot)
        seeds = sample_collision_free_seeds(robot,
                                            list(scene.obstacles),
                                            sample_seed,
                                            int(args.ffb_seed_count),
                                            [list(scene.start), list(scene.goal)])
        print(
            f"[ffb-compare] robot={args.robot} difficulty={args.difficulty} scene_seed={scene_seed} "
            f"obstacles={len(scene.obstacles)} seeds={len(seeds)}",
            flush=True,
        )

        linear = run_mode(robot,
                          list(scene.obstacles),
                          seeds,
                          sbf.FindFreeBoxSearchMode.Linear,
                          args)
        binary = run_mode(robot,
                          list(scene.obstacles),
                          seeds,
                          sbf.FindFreeBoxSearchMode.BinaryDepth,
                          args)
        cases.append({
            "robot": args.robot,
            "difficulty": args.difficulty,
            "scene_seed": int(scene_seed),
            "scene_generation_seed": int(scene_generation_seed),
            "ffb_seed_sampling_seed": int(sample_seed),
            "scene_profile": args.scene_profile,
            "obstacle_count": len(scene.obstacles),
            "seed_count": len(seeds),
            "modes": {
                "Linear": linear,
                "BinaryDepth": binary,
            },
        })

    payload = {
        "experiment": "ffb_search_mode_random_scene_compare",
        "config": {
            "robot": args.robot,
            "difficulty": args.difficulty,
            "scene_seeds": scene_seeds,
            "scene_profile": args.scene_profile,
            "scene_seed_base": int(args.scene_seed_base),
            "ffb_seed_base": int(args.ffb_seed_base),
            "ffb_seed_count": int(args.ffb_seed_count),
            "max_depth": int(args.max_depth),
            "start_depth": int(args.start_depth),
            "max_boxes": int(args.max_boxes),
            "max_consecutive_miss": int(args.max_consecutive_miss),
            "timeout_ms": float(args.timeout_ms),
        },
        "cases": cases,
        "summary": build_summary(cases),
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    summary = payload["summary"]
    print(json.dumps({
        "out_json": str(args.out_json),
        "cases": summary["cases"],
        "binary_over_linear": summary.get("binary_over_linear", {}),
    }, indent=2, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()