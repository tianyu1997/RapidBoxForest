#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, add_common_sbf_args, configure_standalone_sbf, mean, median, query_result_payload, sbf, write_json
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import DEFAULT_RANDOM_DIFFICULTIES, DEFAULT_RANDOM_ROBOTS, DEFAULT_RANDOM_SCENE_SEEDS, make_random_scene, make_robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone SBF-only Exp.5 random robot scene runner.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        max_boxes=20000,
        timeout_ms=120000.0,
        ffb_depth=160,
        component_connect_ffb_max_depth=200,
        post_connect_extra_boxes=2000,
        quality_min_connected_boxes=512,
        post_connect_time_budget_ms=5000.0,
        repair_timeout_ms=1500.0,
    )
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--methods", default="support_hull_coverage")
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "exp5_random_robot_scenes_standalone.json")
    return parser.parse_args()


def failure_query_payload(reason: str) -> dict[str, Any]:
    return {
        "ok": False,
        "success": False,
        "audit_passed": False,
        "repair_count": 0,
        "t_s": 0.0,
        "reason": reason,
    }


def run_case(args: argparse.Namespace, robot_name: str, difficulty: str, method: str, scene_seed: int) -> dict[str, Any]:
    print(f"[exp5-sbf] start robot={robot_name} difficulty={difficulty} method={method} seed={scene_seed}", flush=True)
    try:
        scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
        robot = make_robot(robot_name)
    except RuntimeError as exc:
        reason = f"scene_generation_failed:{exc}"
        print(
            f"[exp5-sbf] skipped robot={robot_name} difficulty={difficulty} method={method} seed={scene_seed} reason={reason}",
            flush=True,
        )
        return {
            "robot": robot_name,
            "robot_spec_source": "standalone_sbf_parametric",
            "difficulty": difficulty,
            "method": method,
            "scene_seed": scene_seed,
            "scene_valid": False,
            "failure_reason": reason,
            "start": [],
            "goal": [],
            "endpoint_clearance_margin_m": 0.0,
            "direct_segment_blocked": False,
            "segment_resolution": 0,
            "obstacle_count": 0,
            "build_s": 0.0,
            "query": failure_query_payload(reason),
            "box_count": 0,
            "certified": 0,
            "provisional": 0,
            "segment_edges": 0,
            "diagnostics": {},
        }
    cfg = configure_standalone_sbf(args, scene_seed, preset=method, robot=robot)
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(scene.obstacles, [scene.start, scene.goal])
    build_s = time.perf_counter() - build_t0
    query_t0 = time.perf_counter()
    query = forest.query(scene.start, scene.goal)
    query_payload = query_result_payload(f"{robot_name}:{difficulty}:{scene_seed}", query, time.perf_counter() - query_t0)
    boxes = forest.boxes()
    print(
        f"[exp5-sbf] done robot={robot_name} difficulty={difficulty} method={method} seed={scene_seed} "
        f"build_s={build_s:.3f} boxes={len(boxes)} audit={query_payload['audit_passed']}",
        flush=True,
    )
    return {
        "robot": robot_name,
        "robot_spec_source": "standalone_sbf_parametric",
        "difficulty": difficulty,
        "method": method,
        "scene_seed": scene_seed,
        "scene_valid": True,
        "start": scene.start,
        "goal": scene.goal,
        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
        "direct_segment_blocked": bool(scene.direct_segment_blocked),
        "segment_resolution": int(scene.segment_resolution),
        "obstacle_count": len(scene.obstacles),
        "build_s": float(build_s),
        "query": query_payload,
        "box_count": len(boxes),
        "certified": sum(1 for box in boxes if box.safety_status == sbf.BoxSafetyStatus.CertifiedFree),
        "provisional": sum(1 for box in boxes if box.safety_status == sbf.BoxSafetyStatus.ProvisionalFree),
        "segment_edges": len(forest.segment_edges()),
        "diagnostics": {str(k): float(v) for k, v in dict(profile.diagnostics).items()},
    }


def main() -> int:
    args = parse_args()
    robots = [item.strip() for item in args.robots.split(",") if item.strip()]
    difficulties = [item.strip() for item in args.difficulties.split(",") if item.strip()]
    methods = [item.strip() for item in args.methods.split(",") if item.strip()]
    rows: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            for method in methods:
                for scene_seed in range(max(1, int(args.scene_seeds))):
                    rows.append(run_case(args, robot_name, difficulty, method, scene_seed))

    summary: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            for method in methods:
                subset = [row for row in rows if row["robot"] == robot_name and row["difficulty"] == difficulty and row["method"] == method]
                summary.append({
                    "robot": robot_name,
                    "difficulty": difficulty,
                    "method": method,
                    "build_mean_s": mean(float(row["build_s"]) for row in subset),
                    "build_median_s": median(float(row["build_s"]) for row in subset),
                    "query_sr": mean(1.0 if row["query"]["ok"] else 0.0 for row in subset),
                    "audit_sr": mean(1.0 if row["query"]["audit_passed"] else 0.0 for row in subset),
                    "repair_count_mean": mean(float(row["query"]["repair_count"]) for row in subset),
                    "box_count_mean": mean(float(row["box_count"]) for row in subset),
                    "segment_edge_count_mean": mean(float(row["segment_edges"]) for row in subset),
                })

    params = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    payload = {
        "experiment": "exp5_random_robot_scenes",
        "source_protocol": "standalone_sbf_no_v6_runtime_dependency_sbf_only",
        "source_script": str(Path(__file__).resolve()),
        "note": "Balanced random-scene SBF run for the TRO Table IV comparison with an increased grower quality budget; OMPL RRTConnect is generated by paper_12_random_scene_rrt_baseline.py and current PRM/BIT* by paper_12_random_scene_ompl_baselines.py.",
        "random_scene_checks": {
            "endpoint_clearance_margin_m": 0.12,
            "direct_start_goal_segment_blocked": True,
            "segment_resolution": 96,
            "scene_profile": args.scene_profile,
        },
        "params": params,
        "summary": summary,
        "rows": rows,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())