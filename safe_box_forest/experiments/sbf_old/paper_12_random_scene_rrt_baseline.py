#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common_sbf_config import ROOT, mean, median, sbf, write_json  # noqa: E402
from common_rrt_connect import rrt_connect  # noqa: E402
from common_scene_sampling import DEFAULT_RANDOM_DIFFICULTIES, DEFAULT_RANDOM_ROBOTS, DEFAULT_RANDOM_SCENE_SEEDS, make_random_scene, make_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OMPL RRTConnect baseline for the standalone random-scene TRO experiment.")
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--timeout-ms", type=float, default=2000.0)
    parser.add_argument("--max-iters", type=int, default=6000)
    parser.add_argument("--step-size", type=float, default=0.35)
    parser.add_argument("--goal-bias", type=float, default=0.10)
    parser.add_argument("--segment-step", type=float, default=0.06)
    parser.add_argument("--shortcut-passes", type=int, default=80)
    parser.add_argument("--simplify-time-s", type=float, default=0.0)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "random_scene_rrt_connect_baseline.json")
    return parser.parse_args()


def summarize(rows: list[dict[str, Any]], robots: list[str], difficulties: list[str]) -> list[dict[str, Any]]:
    summary: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            subset = [row for row in rows if row["robot"] == robot_name and row["difficulty"] == difficulty]
            successes = [row for row in subset if row.get("ok")]
            summary.append({
                "robot": robot_name,
                "difficulty": difficulty,
                "scene_count": len({int(row["scene_seed"]) for row in subset}),
                "trial_count": len(subset),
                "sr": mean(1.0 if row.get("ok") else 0.0 for row in subset),
                "audit_sr": mean(1.0 if row.get("audit_passed") else 0.0 for row in subset),
                "query_mean_s": mean(float(row.get("t_s", 0.0)) for row in subset),
                "query_median_s": median(float(row.get("t_s", 0.0)) for row in subset),
                "success_time_mean_s": mean(float(row.get("t_s", 0.0)) for row in successes),
                "success_length_mean": mean(float(row.get("length", 0.0)) for row in successes),
                "success_length_median": median(float(row.get("length", 0.0)) for row in successes),
            })
    return summary


def main() -> int:
    args = parse_args()
    robots = [item.strip() for item in str(args.robots).split(",") if item.strip()]
    difficulties = [item.strip() for item in str(args.difficulties).split(",") if item.strip()]
    rows: list[dict[str, Any]] = []
    for robot_name in robots:
        robot = make_robot(robot_name)
        bounds = [(float(interval.lo), float(interval.hi)) for interval in robot.joint_limits().limits]
        for difficulty in difficulties:
            for scene_seed in range(max(1, int(args.scene_seeds))):
                try:
                    scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
                except RuntimeError as exc:
                    reason = f"scene_generation_failed:{exc}"
                    for trial in range(max(1, int(args.trials))):
                        print(
                            f"[exp5-rrt] skipped robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial} reason={reason}",
                            flush=True,
                        )
                        rows.append({
                            "robot": robot_name,
                            "difficulty": difficulty,
                            "scene_seed": int(scene_seed),
                            "trial": int(trial),
                            "obstacle_count": 0,
                            "endpoint_clearance_margin_m": 0.0,
                            "direct_segment_blocked": False,
                            "segment_resolution": 0,
                            "ok": False,
                            "audit_passed": False,
                            "reason": reason,
                            "t_s": 0.0,
                            "length": 0.0,
                            "iterations": 0,
                            "nodes": 0,
                            "path_waypoint_count": 0,
                            "scene_valid": False,
                        })
                    continue
                for trial in range(max(1, int(args.trials))):
                    print(
                        f"[exp5-rrt] start robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial}",
                        flush=True,
                    )
                    rng_seed = int(args.seed_base) + 7919 * trial + 104729 * scene_seed + 17 * len(rows)
                    rng = random.Random(rng_seed)
                    result = rrt_connect(robot, scene.obstacles, scene.start, scene.goal, bounds, rng, args, seed=rng_seed)
                    print(
                        f"[exp5-rrt] done robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial} "
                        f"ok={bool(result.get('ok'))} reason={result.get('reason')} t_s={float(result.get('t_s', 0.0)):.3f}",
                        flush=True,
                    )
                    rows.append({
                        "robot": robot_name,
                        "difficulty": difficulty,
                        "scene_seed": int(scene_seed),
                        "trial": int(trial),
                        "scene_valid": True,
                        "obstacle_count": len(scene.obstacles),
                        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
                        "direct_segment_blocked": bool(scene.direct_segment_blocked),
                        "segment_resolution": int(scene.segment_resolution),
                        "ok": bool(result.get("ok")),
                        "audit_passed": bool(result.get("audit_passed", result.get("ok", False))),
                        "reason": result.get("reason"),
                        "t_s": float(result.get("t_s", 0.0)),
                        "length": float(result.get("length", 0.0)) if result.get("ok") else 0.0,
                        "iterations": int(result.get("iterations", 0)),
                        "nodes": int(result.get("nodes", 0)),
                        "path_waypoint_count": len(result.get("path", [])),
                        "planner": result.get("planner", "OMPL_RRTConnect"),
                        "ompl_status": result.get("ompl_status"),
                    })

    payload = {
        "experiment": "paper_12_random_scene_rrt_connect_baseline",
        "source_script": str(Path(__file__).resolve()),
        "note": "Raw OMPL RRTConnect baseline using the same standalone SBF collision checker and deterministic random scenes as Exp.8; no OMPL shortcut simplification is applied by default.",
        "random_scene_checks": {
            "endpoint_clearance_margin_m": 0.12,
            "direct_start_goal_segment_blocked": True,
            "segment_resolution": 96,
            "scene_profile": args.scene_profile,
        },
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "summary": summarize(rows, robots, difficulties),
        "rows": rows,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows), "summary": payload["summary"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())